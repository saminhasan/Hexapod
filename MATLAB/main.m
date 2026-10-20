% main.m
% the one workflow, every step every run: robot -> trajectory -> check -> rig file -> controller constants -> sim
% -> compare. Pick the motion (mode, cycles) and the model; nothing else changes. The check stops here if the
% trajectory asks for more than the rig can do
clc; close all; clear all; %#ok<CLALL>
startup

mode = 'syn';           % designed: sin, syn, cam, mix, step
                        % recorded: imuRun34C, imuRun36D, imuThrow34D, imuThrow34DD, imuStairs34D, imuStairs34DD
                        % motor test: jointStep
cycles = 30;            % designed modes only
model = 'Hexapod_dev';  % Hexapod_dev: vets a new trajectory (CAD, joints follow ref.q)
                        % Hexapod_PD: tries velocity-loop gains on the identified drive
                        % SP, SP_TC, SP_CTC: theory checks (stick model, ideal continuous time)

%% 1 robot
robot = hexapodRobot();

%% 2 trajectory
ref = referenceTrajectory(robot, mode, cycles);
plotReferenceStages(ref);
plotMotorAngles(ref.t, ref.q);

%% 3 check against the rig's limits
check = checkTrajectory(robot, ref);
assert(check.pass, 'main:Check', '%s asks for more than the rig can do, see the table above', mode);

%% 4 rig file: actuatorAngles/<name>.csv, copy it to the firmware's Trajectories/
exportActuatorAngles(ref);

%% 5 controller constants
vc = velocityControlSetup();                            % Hexapod_PD: rig velocity loop
tc = torqueControlSetup(robot, ref, 2*pi*15, 1);        % SP_TC: feedforward + PD per motor
ctc = computedTorqueSetup(robot, ref, 2*pi*15, 1);      % SP_CTC: full computed torque, pose from forward kinematics

%% 6 simulate
simOut = sim(model);

%% 7 compare
plotSimResults(simOut, ref, robot);
