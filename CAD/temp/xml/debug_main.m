clc; close all; clear all; %#ok<CLALL>
Hexapod_DataFile;
params;
sp = StuartPlatform(r, n, rB, dB, rP, dP);


modes = {'sin', 'syn', 'cam', 'mix', 'step'};
mode = modes{2};
trjectory_torso_frame = genTrajectory(mode, 30);

rAB_body = [0.0; 0.0; 0.1];
[trjectory_platform_frame, centerOffset] = rigid_transform(trjectory_torso_frame,rAB_body);

[trjectory_platform_frame_time_warped, info] = warpTrajectoryClock(trjectory_platform_frame, riseTime, fallTime, 'spline');

trjectory_platform_frame_time_warped_windowed = taperTrajectory(trjectory_platform_frame_time_warped, riseTime, fallTime);


% =========================================================================
% Undo the clock warp for plotting.
%
% warpTrajectoryClock records the source time every output sample replays,
% so putting the warped stages back on that clock aligns them with the
% un-warped trajectory EXACTLY, at every sample. A constant riseTime/2
% shift only holds across the plateau and drifts through the ramps, which
% is where the warp is actually doing something.
% =========================================================================
source_clock = trjectory_platform_frame(1,1) + info.srcTime;


% =========================================================================
% TORSO FRAME - what was asked for against what actually gets run
%
% Map the trajectory that feeds the IK back through the same rigid transform
% to recover the torso motion it represents, restoring the offset that the
% forward transform removed so both traces sit in the same absolute frame.
% On the source clock the timing is removed, so what is left on screen is
% purely what the taper did to the amplitude.
% =========================================================================
trjectory_torso_frame_final = rigid_transform( ...
    trjectory_platform_frame_time_warped_windowed, -rAB_body, false);

trjectory_torso_frame_final(:,2:4) = ...
    trjectory_torso_frame_final(:,2:4) + centerOffset;

trjectory_torso_frame_final(:,1) = source_clock;

plotTrajectory( ...
    {trjectory_torso_frame, trjectory_torso_frame_final}, ...
    {'Original', 'Final, as run'}, ...
    'Torso Frame Trajectory');


% =========================================================================
% PLATFORM FRAME - the warp and taper themselves
%
% Warped and tapered both go back on the source clock. The warped trace then
% lands on top of the un-warped one, which is the check that the warp only
% changed the pace and left the path alone; the tapered trace shows the
% amplitude envelope it rides under.
% =========================================================================
trjectory_platform_frame_time_warped_aligned = trjectory_platform_frame_time_warped;
trjectory_platform_frame_time_warped_aligned(:,1) = source_clock;

trjectory_platform_frame_time_warped_windowed_aligned = trjectory_platform_frame_time_warped_windowed;
trjectory_platform_frame_time_warped_windowed_aligned(:,1) = source_clock;

plotTrajectory( ...
    {trjectory_platform_frame, ...
     trjectory_platform_frame_time_warped_aligned, ...
     trjectory_platform_frame_time_warped_windowed_aligned}, ...
    {'Platform frame', 'Time warped', 'Tapered'}, ...
    'Platform Frame Trajectory');

jointAngles = sp.move(trjectory_platform_frame_time_warped_windowed); % jointAngles = debugStep();
plotMotorAngles(jointAngles);


time = jointAngles(:,1);
tf = time(end);
u = jointAngles(:,2:7);
% writematrix(u, "_.csv");
simOut = sim("Hexapod_dev.slx");
% simOut = sim("SP.slx");


plotSimResults(simOut, jointAngles, trjectory_platform_frame_time_warped_windowed, sp);
% RMS position error [X Y Z] mm: 0.2319  0.0609  0.7822
% RMS angular error [Rx Ry Rz] deg: 0.0376  0.0512  0.1002
% 
% Max Power: 58.996229 W
% Max Dynamic Torque: 2.049295 Nm 
% Max RPM (Motor Frame): 763.273498 RPM
% Max Current: 7.318910 A


% RMS position error [X Y Z] mm: 0.1875  0.0519  0.8069
% RMS angular error [Rx Ry Rz] deg: 0.0321  0.0450  0.0917
% 
% Max Power: 77.649745 W
% Max Dynamic Torque: 2.251894 Nm 
% Max RPM (Motor Frame): 884.567680 RPM
% Max Current: 8.042479 A