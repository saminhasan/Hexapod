# Inverse dynamics

All added code, documentation, and results live in the `inverseDynamics/`
folder. Run `startup` from the repository root to add it to MATLAB's path,
or open the runner script in this folder directly. Output paths below are
relative to this folder. Original CAD data and SLX models remain in the
repository's `Backup/` and `simulationFiles/` folders.

Run `inverse_dynamics_main` in MATLAB. It builds the same synthetic running
trajectory as `debug_main`, calculates six torques using **Lagrange** and
**Newton–Euler**, and runs `Hexapod_dev`, `Hexapod_PD`, and `SP` separately.
Set `idRunSimulations = false` at the top for analytical estimates only.
The settings also control trajectory mode, number of cycles, plots and saving.

The six-panel plots and tables compare each model with its own mass profile.
The PD controller changes the executed motion, so two predictions are retained:

- **Command:** torque required for the requested trajectory.
- **Achieved:** torque required for the platform motion recorded by Simscape.

The plotted Lagrange/Newton–Euler curves use achieved motion when simulations
are enabled. The gray curve shows the command prediction. The reported error
uses achieved motion, so controller lag is not counted as a dynamics error.
The comparison uses logged world-frame velocities and accelerations directly;
it does not differentiate interpolated simulation poses or use simulated
torques as inputs to either analytical calculation.
All three `platform_pose` loggers add `+g` to vertical acceleration. The script
removes this logging bias before passing kinematic acceleration into ID.

Results stay in `idResults`, `idSummary`, and `idMotorSummary` in the workspace.
`output/inverse_dynamics/` contains `results.mat`, `summary.csv`, `per_motor.csv`,
and one PNG per model. A new run replaces these generated result files.

## Torque convention

All comparison plots and CSVs use **crank/output-shaft N m**, directly matching
columns `4:4:24` of `motorData`. The CAD logger already corrects the alternating
motor signs. No extra sign flip or fitted torque scaling is applied.
The function also returns ideal motor torques, `crankTorque / gearRatio`.
Gearbox efficiency is not included in that conversion.

## Mass profiles

Edit `inverseDynamicsProfiles.m` when changing a model. The defaults describe
the saved models inspected for this implementation:

| Quantity | Hexapod_dev | Hexapod_PD | SP |
|---|---:|---:|---:|
| Platform assembly mass, kg | 2.7845 | 2.7845 | 2.0000 |
| Each rod mass, kg | 0.2985 | 0.2985 | 0.5500 |
| Each crank assembly mass excluding rotor, kg | 0.6204 | 0.6204 | 0.3300 |
| Torso mass, kg | 2.5000 | 2.5000 | 2.5000 |
| Torso COM above platform frame, m | 0.2205 | 0.2205 | 0.1920 |

The two CAD models currently have the same physical mass assembly but different
drive paths. Their profiles are separate so they can be changed independently.
CAD masses are summed from `Backup/Hexapod_DataFile.m`; the similarly named file
in `simulationFiles` only declares a signal. CAD assembly COM/inertias are
snapshots calculated from the saved solid data and rigid transforms. Torso
inertia was calculated from `torso.STL`. Those snapshots do not automatically
update when CAD geometry changes. SP uses the active polygon/brick inertias,
not the inactive `Iplat` value in `params.m`.

The profiles include reflected rotor inertia, the torso's COM offset and
inertia, and each crank's inertia about its COM. Limb masses/inertias can be
scalars or six-element vectors. Profile platform mass must exclude rods:
the solver adds the upper rod halves itself.

## Calculation and limits

Both methods use the paper's ballpark approximation: split each rod's mass
equally between its endpoints. Upper halves move with the platform; lower
halves move with the crank tips. The upper halves add rotational inertia as
well as mass. The CAD crank's small axial COM offset is omitted.

Newton–Euler computes the composite moving body's force and moment, solves for
six rod forces, then adds each crank/rotor's torque. Lagrange independently
forms the kinetic-energy mass matrix, its coordinate derivatives and the
potential-energy gradient, and solves the rod constraints for their reactions.
Both include inertia, gravity and rotational velocity terms. This is an
adaptation to the repo's XYZ rotation convention, not a literal transcription
of the paper's Euler-angle equations. No Symbolic Math Toolbox is required.

Agreement between the two calculations checks their implementation under the
shared approximation; it does not establish agreement with a distributed CAD
body model. Expect differences from rod mass distribution, simplified crank
geometry, and any later profile/model changes. Flexibility and gearbox losses
are omitted. Optional viscous friction defaults to zero.

For arbitrary trajectories, use:

```matlab
profiles = inverseDynamicsProfiles(sp);
estimate = inverseDynamics(platformTrajectory, sp, profiles(1));
% platformTrajectory: [t x y z Xangle Yangle Zangle], seconds/metres/radians
% pose is relative to home, exactly as expected by sp.move.
tauL = estimate.torqueLagrange;
tauNE = estimate.torqueNewtonEuler;
```

The default derivatives use nonperiodic finite differences, so a trajectory
need not wrap back to its starting state. Measured noisy trajectories should
be filtered first. Unreachable poses and near-singular geometry produce errors.

Run the physical checks without Simscape using:

```matlab
tests = runtests('testInverseDynamics.m');
assertSuccess(tests);
```

They check static weight support, zero load, independent method agreement,
known rotor acceleration, gravitational virtual work, time sampling and
supplied world-frame kinematics.

## Validation run

The default 30-cycle synthetic trajectory (11 seconds including ramps) was
run through all three saved models. Seven physical tests passed; MATLAB Code
Analyzer reported no issues in the four new MATLAB files. Lagrange and
Newton–Euler differed by less than 4e-11 N m on this run.

| Model | Simscape peak, N m | ID for achieved motion peak, N m | Mean per-motor normalized RMS error |
|---|---:|---:|---:|
| Hexapod_dev | 19.730 | 19.734 | 1.10% |
| Hexapod_PD | 10.885 | 10.975 | 1.24% |
| SP | 18.444 | 18.400 | 2.58% |

The error is RMS(ID minus Simscape) divided by RMS(Simscape), calculated per
motor and then averaged over six motors, using the full timeline. Peaks use
each record's full sampling. These are simulation checks for this trajectory
and these mass profiles, not an accuracy claim for other motions or hardware.
The requested-motion peaks were 21.641 N m for the CAD profiles and 20.616 N m
for SP; filtering explains why these differ from the achieved-motion values.
