# Inertia and bias at one controller sample

These files and their generated results are in `inverseDynamics/`.
Output paths below are relative to this folder. Run `startup` from the
repository root before using the command-window setup below, or run
`example_hexapodDynamicsStep.m` directly from this folder.

`hexapodDynamicsStep` calculates the actual numerical matrices from the same
approximate mass models used by `inverseDynamics.m`:

```matlab
tauCrank = Mtheta*thetaDDot + biasTheta;
```

Both are crank-side quantities. `Mtheta` is 6-by-6 in kg*m^2 and already
includes reflected rotor inertia. `biasTheta` is 6-by-1 in N*m and includes
gravity, velocity-dependent forces, and the profile's viscous friction.
The checked-in profiles currently specify zero viscous friction; set
`profile.viscousFriction` before setup if the new plant has physical damping.
Do not substitute the unrelated fitted control-plant coefficient `b` from
`params.m` as physical joint friction.

## Once before simulation

```matlab
startup;
params;
sp = StuartPlatform(r,n,rB,dB,rP,dP);
profiles = inverseDynamicsProfiles(sp);
model = hexapodDynamicsSetup(sp,profiles(2));
% profiles(1): Hexapod_dev, (2): Hexapod_PD, (3): SP stick model
```

`model` is a numeric struct. In a Simulink MATLAB Function block, declare it
as a Parameter in the Symbols pane, supplied from the model/base workspace.
The step routine has been compiled and run in a MATLAB Function block.

## Every sample

```matlab
[Mtheta,biasTheta] = hexapodDynamicsStep(pose,theta,thetaDot,model);
```

Supply actual state, not reference state:

- `pose`: 6-by-1 `[x;y;z;a;b;c]`. Translation is the measured world position
  of the moving platform origin **minus `sp.homez`**. Orientation is XYZ
  Euler angles in radians, with `R = Rx(a)*Ry(b)*Rz(c)` mapping platform
  coordinates to world. With a consistent quaternion sensor this is
  `quat2eul(quaternionRow,'XYZ').'`.
- `theta`: six measured crank angles [rad], in `StuartPlatform.move` order
  and sign. At home all are zero.
- `thetaDot`: corresponding crank velocities [rad/s].

The pose and crank measurements must describe the same assembly branch and
geometry. A third output reports rod closure residual and conditioning.
No acceleration measurements, previous samples, saved torque data, or
numerical time differentiation are used.

`biasTheta` means torque at **zero crank acceleration**, at the current
pose and velocity. The routine retains the platform acceleration caused by
the moving constraints; setting platform acceleration to zero instead
would not produce the correct joint-space bias.

For a complete controller that calculates these quantities internally:

```matlab
[tauCrank,tauMotor,Mtheta,biasTheta] = hexapodComputedTorqueStep( ...
    pose,theta,thetaDot,thetaRef,thetaDotRef,thetaDDotRef,model,wn,zeta);
```

Reference signals are 6-by-1 crank-side quantities. `wn` [rad/s] and `zeta`
are scalar tuning inputs. Torque outputs are unsaturated demands; `tauMotor`
uses the ideal gear ratio. Apply actual actuator limits and current-loop
dynamics in the plant. To implement position-only second-order command
shaping, supply zero reference velocity and acceleration. Hold the torque
command over the controller sample interval and let Simscape integrate the
mechanical motion. This is a controller implementation, not a completed
closed-loop performance validation.

## Concrete values and validation

Run `example_hexapodDynamicsStep.m` to display the selected model's home
matrix and bias and exercise the complete controller. At the home pose,
zero crank speeds and zero crank accelerations, the CAD profiles require
`1.5037549*ones(6,1)` N*m; the stick profile requires
`1.4366742*ones(6,1)` N*m. CAD dev/PD currently have the same mass profile.

`output/step_dynamics/values.mat` contains `idStepValues`, with fields
`Hexapod_dev`, `Hexapod_PD`, and `SP`. Each includes:

- `homeMtheta`, `homeBiasTheta`, and the prepared `model`.
- `time`, `pose`, `Mtheta(:,:,k)`, and `biasTheta(k,:)` for every sample of
  the **commanded trajectory** in the saved inverse-dynamics runs.
- Maximum differences from both saved inverse-dynamics formulations.

Home values are also provided as CSV files in the same directory.
Reconstructed torque agreed across all 33,003 saved command samples within
4.8e-14 N*m of Newton-Euler and 3.63e-11 N*m of Lagrange. These are checks
against the existing analytical model; they are not new Simscape runs.
`testHexapodDynamicsStep` additionally exercises asymmetric masses/COM,
nonzero viscous friction, kinetic energy, positive-definite inertia, and
the complete controller's acceleration command.
