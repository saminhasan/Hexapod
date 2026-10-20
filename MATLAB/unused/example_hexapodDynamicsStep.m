% example_hexapodDynamicsStep.m
% M and bias at home for one profile, then one computed-torque sample; swap the zeros for measurements
run(fullfile(fileparts(fileparts(mfilename('fullpath'))), 'startup.m'));
params;
sp = StuartPlatform(r, n, rB, dB, rP, dP);
profiles = inverseDynamicsProfiles(sp);
profileIndex = 2; % 1 Hexapod_dev, 2 Hexapod_PD, 3 SP
dynamicsModel = hexapodDynamicsSetup(sp, profiles(profileIndex));

pose = zeros(6,1);      % [platform origin - sp.homez; XYZ Euler]
theta = zeros(6,1);
thetaDot = zeros(6,1);
[Mtheta, biasTheta] = hexapodDynamicsStep(pose, theta, thetaDot, dynamicsModel);
disp(profiles(profileIndex).name);
disp('Mtheta [kg*m^2], crank side:'); disp(Mtheta);
disp('biasTheta [N*m], crank side:'); disp(biasTheta);

thetaRef = zeros(6,1);
thetaDotRef = zeros(6,1);
thetaDDotRef = zeros(6,1);
wn = 10; zeta = 0.9;
[tauCrank, tauMotor] = hexapodComputedTorqueStep(pose, theta, thetaDot, thetaRef, thetaDotRef, thetaDDotRef, dynamicsModel, wn, zeta);
