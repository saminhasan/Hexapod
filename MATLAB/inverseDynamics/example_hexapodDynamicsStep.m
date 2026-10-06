% Run once to prepare the selected model and calculate concrete home values.
run(fullfile(fileparts(fileparts(mfilename('fullpath'))),'startup.m'));
params;
sp = StuartPlatform(r,n,rB,dB,rP,dP);
profiles = inverseDynamicsProfiles(sp);
profileIndex = 2; % 1=Hexapod_dev, 2=Hexapod_PD, 3=SP (stick)
dynamicsModel = hexapodDynamicsSetup(sp,profiles(profileIndex));

% Replace these THREE state inputs with measurements each controller sample.
% pose = [world platform origin - sp.homez; XYZ Euler angles in radians].
pose = zeros(6,1);
theta = zeros(6,1);
thetaDot = zeros(6,1);
[Mtheta,biasTheta] = hexapodDynamicsStep(pose,theta,thetaDot,dynamicsModel);
disp(profiles(profileIndex).name);
disp('Mtheta [kg*m^2], crank side:'); disp(Mtheta);
disp('biasTheta [N*m], crank side:'); disp(biasTheta);

% Complete controller at this sample; numerical tuning is illustrative.
thetaRef = zeros(6,1);
thetaDotRef = zeros(6,1);
thetaDDotRef = zeros(6,1);
wn = 10; zeta = 0.9;
[tauCrank,tauMotor] = hexapodComputedTorqueStep(pose,theta,thetaDot, ...
    thetaRef,thetaDotRef,thetaDDotRef,dynamicsModel,wn,zeta);
% Feed tauCrank into torque-driven crank joints, OR tauMotor into drives.
% No need to supply Mtheta/biasTheta: the complete controller calculates them.
