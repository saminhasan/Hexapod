clc; close all; clear all; %#ok<CLALL>
startup
Hexapod_DataFile;
params;
sp = StuartPlatform(r, n, rB, dB, rP, dP);


modes = {'sin', 'syn', 'cam', 'mix', 'step'};
mode = modes{2};
trjectory_torso_frame = genTrajectory(mode, 30);

rAB_body = [0.0; 0.0; 0.1];
[trjectory_platform_frame, centerOffset] = rigidTransform(trjectory_torso_frame,rAB_body);

[trjectory_platform_frame_time_warped, info] = warpTrajectoryClock(trjectory_platform_frame, riseTime, fallTime, 'spline');

trjectory_platform_frame_time_warped_windowed = taperTrajectory(trjectory_platform_frame_time_warped, riseTime, fallTime);


% warped stages go back on the source time each row replays, so they line up with the original exactly
source_clock = trjectory_platform_frame(1,1) + info.srcTime;


% torso frame: what was asked for vs what actually runs (platform path mapped back, offset restored)
trjectory_torso_frame_final = rigidTransform(trjectory_platform_frame_time_warped_windowed, -rAB_body, false);
trjectory_torso_frame_final(:,2:4) = trjectory_torso_frame_final(:,2:4) + centerOffset;

trjectory_torso_frame_final(:,1) = source_clock;

% plotTrajectory( ...
%     {trjectory_torso_frame, trjectory_torso_frame_final}, ...
%     {'Original', 'Final, as run'}, ...
%     'Torso Frame Trajectory');


% platform frame: warped trace should sit on the original, tapered shows the envelope
trjectory_platform_frame_time_warped_aligned = trjectory_platform_frame_time_warped;
trjectory_platform_frame_time_warped_aligned(:,1) = source_clock;

trjectory_platform_frame_time_warped_windowed_aligned = trjectory_platform_frame_time_warped_windowed;
trjectory_platform_frame_time_warped_windowed_aligned(:,1) = source_clock;

% plotTrajectory( ...
%     {trjectory_platform_frame, ...
%      trjectory_platform_frame_time_warped_aligned, ...
%      trjectory_platform_frame_time_warped_windowed_aligned}, ...
%     {'Platform frame', 'Time warped', 'Tapered'}, ...
%     'Platform Frame Trajectory');

jointAngles = sp.move(trjectory_platform_frame_time_warped_windowed); % jointAngles = debugStep();
% plotMotorAngles(jointAngles);


time = jointAngles(:,1);
tf = time(end);
u = jointAngles(:,2:7);
du = periodicDerivative5pt(u, dt);
ctrl = torqueControlSetup(sp, trjectory_platform_frame_time_warped_windowed, u, du, dt, 2*pi*15, 1); % SP_TC: feedforward + PD per motor
ctc = computedTorqueSetup(sp, trjectory_platform_frame_time_warped_windowed, du, dt, 2*pi*15, 1); % SP_CTC: full computed torque, pose from forward kinematics
% writematrix(u, "rigFiles/synth_3Hz_motorAngles.csv");
% simOut = sim("models/Hexapod_dev.slx");
% simOut = sim("models/Hexapod_PD.slx");
% simOut = sim("models/SP.slx");
% simOut = sim("models/SP_TC.slx");
simOut = sim("models/SP_CTC.slx");


plotSimResults(simOut, jointAngles, trjectory_platform_frame_time_warped_windowed, sp);
