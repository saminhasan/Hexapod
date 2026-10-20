# Hexapod MATLAB

Rotary Stewart platform (6 cranks, 3 slave Teensies) that moves a torso manikin through a running motion.
Run `startup` first: it puts everything except `unused/` on the path and keeps Simulink build caches in
`.simcache/` (safe to delete).

## The workflow: `main.m`

One path, every step every run. The only choices are `mode` (which motion), `cycles` (for designed motions) and
`model` (which sim).

```
main.m
│
├─ 1 robot        robot = hexapodRobot()                          robot/
│                 geometry, masses, motor + drive, joint limits, speed and torque limits
│
├─ 2 trajectory   ref = referenceTrajectory(robot, mode, cycles)   trajectory/
│     a  what moves      genTrajectory: a body point's motion      trajectory/generate/
│     b  to the platform placePlatform: body point -> platform centre, centred
│     c  ease in/out     warpTrajectoryClock + taperTrajectory, 1 s each end
│     d  crank angles    inverseKinematics (rows that can't close are flagged)
│     plots: plotReferenceStages, plotMotorAngles
│
├─ 3 check        checkTrajectory: leg reach, rod-end tilt, motor speed, motor torque.
│                 Any limit failed -> main stops here (no sim, no rig file)
│
├─ 4 rig file     exportActuatorAngles -> actuatorAngles/<name>.csv + .json
│
├─ 5 controllers  velocityControlSetup, torqueControlSetup, computedTorqueSetup
│
├─ 6 simulate     sim(model)
│
└─ 7 compare      plotSimResults: platform pose, error, rates; motor torque, speed, angles
```

| model | what it's for |
|---|---|
| `Hexapod_dev` | vets a new trajectory: CAD model, joints follow `ref.q` |
| `Hexapod_PD` | tries velocity-loop gains on the identified drive, instead of on the rig |
| `SP`, `SP_TC`, `SP_CTC` | theory checks: stick model, ideal continuous time; position, torque per motor, computed torque |

A 15 min recording takes a long time in any model; an 11 s file already takes minutes.

## Trajectory: from a motion to actuator angles

Every motion, designed or recorded, goes through the same four steps in `referenceTrajectory`.

**a. What moves** (`trajectory/generate/`). `genTrajectory(robot, mode, n)` is the only place that lists the
modes. Each one hands back the motion of one body point, `[t x y z Rx Ry Rz]` at 1 kHz, and where the platform
centre sits from that point:

- **Designed**, n cycles, a torso point 10 cm below the platform centre:
  - `sin` (`genPoseSine`)
  - `syn` (`genPoseSynthetic`, synthetic running)
  - `cam` (`genPoseCam`, camera capture in `trajectory/data/`)
  - `mix` (`genPoseMix`, cam with synthetic z)
  - `step` (`genPoseStep`, one axis at a time)
- **Recorded**, the sternum IMU recordings in `oct 6/ExemplarBreastTorsoMovements_IMUs`. On the manikin the sternum
  IMU sits 32 cm above the platform centre.
  - One small file per recording in `generate/imu/`: `imuRun34C`, `imuRun36D`, `imuThrow34D`, `imuThrow34DD`,
    `imuStairs34D`, `imuStairs34DD`.
  - Each file holds only what's particular to that recording: which csv, junk at the start, where to cut, which
    seconds are upright, whether it's ever still enough for the gyro bias.
  - All of them call `genPoseImu`, the one shared IMU-to-motion step. It integrates the gyro and lets gravity hold
    the tilt. It drops translation and heading below 0.5 Hz as drift, keeps 0 to 100 Hz, and keeps the lean.
  - `testGenPoseImu` checks it against a known motion.
- **Motor test**: `jointStep` (`genJointStep`), crank angles directly. It skips b and c, and its pose comes from
  `forwardKinematics`.

**b. To the platform** (`trajectory/shape/placePlatform`):
- Every motion: the body point is moved to the platform centre (`rigidTransform`) and the result is centred.
- Recordings also drop the drift, and their average position goes where the legs have the most room
  (`private/bestShift`), because the IMU can't know it.
- Nothing else is changed: lean and amplitude are played as recorded.

**c. Ease in and out** (`warpTrajectoryClock`, `taperTrajectory`, `bumpWindow`). Every file starts and ends at home
over 1 s.

**d. Crank angles** (`kinematics/inverse/inverseKinematics`). `ref.reachable` flags any row a leg can't close.

**Then the check** (`trajectory/checkTrajectory`). It prints one table with the worst value, when it happens and
the allowed value:
- leg reach (`robot.joint.reachLimit`)
- rod-end tilt on its ball screw (`robot.joint.sphericalLimitDeg`, via `workspaceUse`)
- motor speed (`robot.motor.maxSpeedRpm`)
- motor torque, peak and rms (`peakTorque`, `ratedTorque`, inverse dynamics with the CAD body)

The trajectory is never adjusted to pass. **Then the rig file** (`trajectory/exportActuatorAngles`):
- Writes `actuatorAngles/<mode>_<cycles>c.csv`, or `<mode>.csv` when there are no cycles: 6 crank angles in rad, one
  row per 1 ms, no header. A `.json` next to it records rate, rows, duration, offsets and sha256.
- A file already there with the same content is left alone. One with different content is kept and reported.
- Copy the csv to the firmware's `Trajectories/` and upload.

```
trajectory/
  referenceTrajectory.m   steps a-d, the one entry
  checkTrajectory.m       the check
  exportActuatorAngles.m  the rig file
  generate/               a: genTrajectory, genPoseSine/Synthetic/Step, runningZ, genPoseCam/Mix,
                          genPoseImu, genJointStep, testGenPoseImu; imu/ one file per recording
  shape/                  b, c: placePlatform, rigidTransform, warpTrajectoryClock, taperTrajectory, bumpWindow
  data/                   camera capture
```

## Layout

```
main.m                    the workflow above
robot/                    hexapodRobot (the rig as one struct), bodyModels (masses of the three sims), cad/
trajectory/               see above
kinematics/
  inverse/                inverseKinematics, inverseKinematicsStep, workspaceUse (leg reach, rod-end tilt),
                          deriveInverseKinematics (symbolic)
  forward/                forwardKinematics, forwardKinematicsStep, kinematicJacobian, deriveForwardKinematics, test
dynamics/
  inverse/                inverseDynamics, inverseDynamicsModel, inverseDynamicsStep, deriveInverseDynamics,
                          runInverseDynamics, tests
control/
  position/               Hexapod_dev (CAD), SP (stick): joints follow ref.q directly
  velocity/               Hexapod_PD (CAD) + velocityControlSetup: the rig's velocity-mode PD on the identified drive
  torque/                 SP_TC, SP_CTC, their setups and build scripts, runMismatch
plotting/  utils/
validation/               2026-10-05 rig runs vs sim and torso IMU; presentation/ web page
actuatorAngles/           rig files, what gets copied to the firmware's Trajectories/
papers/  unused/  oct 6/  (oct 6 is raw rig data, left as recorded)
```

## What the models read

Every model reads only these, nothing loose:

| name | from | read by |
|---|---|---|
| `robot` | `hexapodRobot()` | all |
| `ref` | `referenceTrajectory(robot, mode, n)`: `t`, `x`, `q`, `qd`, `qdd` | all (stop time is `ref.t(end)`) |
| `vc` | `velocityControlSetup()` | Hexapod_PD |
| `tc` | `torqueControlSetup(robot, ref, wn, zeta)` | SP_TC |
| `ctc` | `computedTorqueSetup(robot, ref, wn, zeta)` | SP_CTC |

The CAD models also read `smiData`, which they load into their own model workspace from `robot/cad/Hexapod_DataFile.m`.

## Conventions

- Symbols: platform pose `x = [x y z Rx Ry Rz]` relative to home (m, rad, XYZ Euler); crank angles `q`, `qd`, `qdd`;
  torque `tau` (crank side unless it says motor); time `t`. A trajectory matrix is `[t x]`.
- Every module has a batch function and a one-sample `...Step` (`%#codegen`, used in MATLAB Function blocks):
  `inverseKinematics`/`inverseKinematicsStep`, `forwardKinematics`/`forwardKinematicsStep`,
  `inverseDynamics`/`inverseDynamicsStep`, `computedTorqueStep`. Batch functions take `robot` first; step functions
  take the numeric struct they need (`robot.geom` or `inverseDynamicsModel`).
- Optional inputs are positional with `nargin` defaults. Scripts start with `run`, `build`, `derive` or `export`.
  One function per file; helpers used by one folder sit in that folder's `private/`.
- Each function opens with a short comment: what it returns, inputs, units.

## Side studies

These reuse the same robot, trajectory and controller steps:

- **Inverse dynamics.** `dynamics/inverse/runInverseDynamics` runs the three models and compares their torque
  against both ID methods. See `dynamics/inverse/README.md`.
- **Torque control.** `control/torque/runMismatch` runs SP_TC and SP_CTC with a deliberately wrong mass model.
  After editing `SP.slx`, rerun `control/torque/buildSP_TC` and `buildSP_CTC`.
- **Rig validation.** `validation/runValidation` reads the session logs and IMU captures in `oct 6/`, replays
  the measured angles through `Hexapod_dev`, lines the simulated torso IMU up with the measured one and folds every
  loop. The first run fills `validation/cache/` (about 30 min). `exportPresentation` then writes the page data.
- **Tests.** `runtests('kinematics', 'IncludeSubfolders', true)`, `runtests('dynamics', 'IncludeSubfolders', true)`,
  `runtests('trajectory', 'IncludeSubfolders', true)`.
