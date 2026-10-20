# Inverse dynamics

Crank torques for a prescribed platform motion, worked out two ways (Newton–Euler and Lagrange) on the paper's
model. Each rod is split into two end masses: the upper halves ride with the platform, the lower halves with the
crank tips. The derivation, with every step checked numerically, is in `deriveInverseDynamics`.

```matlab
robot = hexapodRobot();
bodies = bodyModels(robot);                          % Hexapod_dev, Hexapod_PD, SP
ref = referenceTrajectory(robot, 'syn', 30);
id = inverseDynamics(robot, bodies(3), ref.t, ref.x);   % id.tauNewtonEuler, id.tauLagrange (crank side, N m)

dyn = inverseDynamicsModel(robot, bodies(3));        % numeric, goes into MATLAB Function blocks
[M, bias] = inverseDynamicsStep(dyn, x, q, qd);      % tau = M*qdd + bias at the current state
```

- Pose rates come from 5-point differences. With Simscape logs, pass the world-frame rates instead
  (the `kinematics` argument). Filter measured data first.
- `M` includes the reflected rotors. `bias` is velocity terms + gravity + viscous friction. Viscous friction
  is 0 in the body models; don't use `robot.drive` for it, that is the identified velocity loop.
- Motor torque is crank torque / N, with no gearbox efficiency.

## runInverseDynamics

Runs the three models on the reference trajectory. It compares each model's torque with the ID of the
requested motion and with the ID of the motion the model actually made (logged world-frame rates; the
`platform_pose` loggers add +g to z, which is taken off). Outputs go to `output/inverse_dynamics/`:
`results.mat`, `summary.csv`, `per_motor.csv` and one PNG per model.

Last run (syn, 30 cycles, 11 s):

| | Simscape peak, N m | achieved ID peak, N m | mean per-motor NRMSE |
|---|---:|---:|---:|
| Hexapod_dev | 19.698 | 19.514 | 1.11% |
| Hexapod_PD | 10.888 | 10.984 | 1.24% |
| SP | 18.444 | 18.400 | 2.58% |

Lagrange and NE agree to 4e-11 N m. The requested-motion peaks are 21.641 (CAD) and 20.616 (SP); the gap is
the loop filtering the motion. The CAD numbers move by about 1% from run to run because their variable-step solver
(RelTol 1e-3) reacts to rounding-level input changes.

## Body models

| | Hexapod_dev | Hexapod_PD | SP |
|---|---:|---:|---:|
| platform, kg | 2.7845 | 2.7845 | 2.0000 |
| each rod, kg | 0.2985 | 0.2985 | 0.5500 |
| each crank (no rotor), kg | 0.6204 | 0.6204 | 0.3300 |
| torso, kg | 2.5 | 2.5 | 2.5 |
| torso COM above platform, m | 0.2205 | 0.2205 | 0.192 |

The CAD COM and inertia values are snapshots from the data file and the saved transforms. Change them in
`robot/bodyModels.m` if the CAD changes.

At home, at rest, the CAD models need 1.5037549 N m per crank and SP needs 1.4366742 N m.
