function tests = testInverseDynamics
%TESTINVERSEDYNAMICS Physical and cross-formulation checks (no simulations).
% Run with: results = runtests('testInverseDynamics'); assertSuccess(results)
tests = functiontests(localfunctions);
end

function setupOnce(~)
run(fullfile(fileparts(fileparts(mfilename('fullpath'))),'startup.m'));
end

function testStaticSymmetricWeight(testCase)
[sp,p] = fixture();
t = (0:0.01:0.1).';
out = inverseDynamics([t zeros(numel(t),6)],sp,p);
% At theta=0, each crank's vertical lever is r. Its load is one sixth
% of platform+payload weight, one whole rod weight, and half an arm weight.
expected = -p.gravity(3)*sp.r*((p.platformMass+p.payloadMass)/6 ...
    +p.rodMass+p.armMass/2);
verifyEqual(testCase,out.torqueNewtonEuler,expected*ones(numel(t),6),'AbsTol',1e-10);
verifyEqual(testCase,out.torqueLagrange,out.torqueNewtonEuler,'AbsTol',1e-10);
verifyLessThan(testCase,max(out.closureResidual),1e-12);
end

function testZeroMassInertia(testCase)
[sp,p] = fixture();
p.platformMass=0; p.payloadMass=0; p.rodMass=0; p.armMass=0;
p.platformInertia=zeros(3); p.payloadInertia=zeros(3);
p.armInertia=0; p.rotorInertia=0;
out = inverseDynamics(mixedTrajectory(),sp,p);
verifyEqual(testCase,out.torqueLagrange,zeros(size(out.torqueLagrange)),'AbsTol',0);
verifyEqual(testCase,out.torqueNewtonEuler,zeros(size(out.torqueNewtonEuler)),'AbsTol',0);
end

function testIndependentMethodsOnMixedMotion(testCase)
[sp,p] = fixture();
p.platformCOM=[0.005;-0.004;0.012];
p.payloadCOM=[0.025;-0.017;0.1];
p.rodMass=[0.41;0.45;0.48;0.51;0.55;0.6];
p.armMass=[0.2;0.22;0.24;0.26;0.28;0.3];
p.armInertia=p.armMass*sp.r^2/12;
p.viscousFriction=0.01;
out = inverseDynamics(mixedTrajectory(),sp,p);
scale = max(1,max(abs(out.torqueNewtonEuler),[],'all'));
verifyLessThan(testCase,out.maxMethodDifference,1e-6*scale);
verifyEqual(testCase,out.motorTorqueNewtonEuler, ...
    out.torqueNewtonEuler/p.gearRatio,'AbsTol',1e-12);
end

function testKnownRotorAcceleration(testCase)
[sp,p] = fixture();
p.platformMass=0; p.payloadMass=0; p.rodMass=0; p.armMass=0;
p.platformInertia=zeros(3); p.payloadInertia=zeros(3);
p.armInertia=0; p.rotorInertia=0.02; p.gravity=zeros(3,1);
t = (-0.05:0.001:0.05).';
az = 0.7;
trajectory = [t zeros(numel(t),6)];
trajectory(:,4) = 0.5*az*t.^2;
out = inverseDynamics(trajectory,sp,p);
[~,middle] = min(abs(t));
% At rest at home, z_ddot = r*theta_ddot exactly; only rotor inertia remains.
expected = p.rotorInertia*az/sp.r;
verifyEqual(testCase,out.torqueNewtonEuler(middle,:),expected*ones(1,6),'AbsTol',1e-10);
verifyEqual(testCase,out.torqueLagrange(middle,:),expected*ones(1,6),'AbsTol',1e-10);
% A quadratic heave must keep its known acceleration at BOTH boundaries.
verifyEqual(testCase,out.poseAcceleration(:,3),az*ones(numel(t),1),'AbsTol',1e-9);
end

function testGravityVirtualWorkAtTiltedPose(testCase)
[sp,p] = fixture();
p.payloadCOM=[0.025;-0.017;0.11];
p.platformCOM=[0.004;0.009;-0.01];
p.rodMass=[0.31;0.4;0.48;0.39;0.55;0.43];
pose = [0.004 -0.007 0.008 0.07 -0.06 0.05];
t = (0:0.01:0.1).';
out = inverseDynamics([t repmat(pose,numel(t),1)],sp,p);
direction = [0.02 -0.013 0.015 0.2 -0.17 0.12];
step = 1e-5;
plus = pose+step*direction; minus=pose-step*direction;
ap = sp.move([0 plus]); am = sp.move([0 minus]);
dTheta = (ap(2:7)-am(2:7))/(2*step);
dEnergy = (potentialEnergy(plus,sp,p)-potentialEnergy(minus,sp,p))/(2*step);
verifyEqual(testCase,out.torqueNewtonEuler(6,:)*dTheta.',dEnergy,'AbsTol',1e-7);
verifyEqual(testCase,out.torqueLagrange(6,:)*dTheta.',dEnergy,'AbsTol',1e-7);
end

function testNonuniformTimeAndInputValidation(testCase)
[sp,p] = fixture();
t = [0;0.009;0.021;0.029;0.041;0.055;0.061;0.073;0.08];
trajectory = [t zeros(numel(t),6)];
trajectory(:,4)=0.5*0.5*t.^2;
out = inverseDynamics(trajectory,sp,p);
verifyEqual(testCase,out.poseAcceleration(:,3),0.5*ones(numel(t),1),'AbsTol',1e-9);
verifyError(testCase,@() inverseDynamics([0 zeros(1,6)],sp,p),'inverseDynamics:Time');
trajectory(3,1)=trajectory(2,1);
verifyError(testCase,@() inverseDynamics(trajectory,sp,p),'inverseDynamics:Time');
end

function testSuppliedWorldKinematics(testCase)
[sp,p] = fixture();
t = (0:0.002:0.5).';
w = 2*pi; pitch = 0.08;
trajectory = [t,0.005*sin(w*t),zeros(size(t)),0.008*cos(w*t), ...
    zeros(size(t)),pitch*ones(size(t)),0.04*sin(w*t)];
kin.linearVelocity = [0.005*w*cos(w*t),zeros(size(t)),-0.008*w*sin(w*t)];
kin.linearAcceleration = [-0.005*w^2*sin(w*t),zeros(size(t)),-0.008*w^2*cos(w*t)];
% R=Ry(pitch)*Rz(yaw): the yaw axis in world is [sin(pitch),0,cos(pitch)].
kin.angularVelocity = 0.04*w*cos(w*t)*[sin(pitch) 0 cos(pitch)];
kin.angularAcceleration = -0.04*w^2*sin(w*t)*[sin(pitch) 0 cos(pitch)];
sensor = inverseDynamics(trajectory,sp,p,kin);
sampled = inverseDynamics(trajectory,sp,p);
verifyEqual(testCase,sensor.angularVelocity,kin.angularVelocity,'AbsTol',1e-12);
verifyEqual(testCase,sensor.angularAcceleration,kin.angularAcceleration,'AbsTol',1e-12);
verifyEqual(testCase,sensor.torqueNewtonEuler(3:end-2,:), ...
    sampled.torqueNewtonEuler(3:end-2,:),'AbsTol',1e-5);
verifyLessThan(testCase,sensor.maxMethodDifference,1e-7);
bad = kin; bad.linearVelocity = bad.linearVelocity(1:end-1,:);
verifyError(testCase,@() inverseDynamics(trajectory,sp,p,bad),'inverseDynamics:Kinematics');
end

function trajectory = mixedTrajectory()
t = (0:0.005:1).';
trajectory = [t, 0.006*sin(2*pi*0.8*t), 0.005*cos(2*pi*0.6*t), ...
    0.007*sin(2*pi*1.1*t), 0.05*sin(2*pi*0.7*t), ...
    0.04*cos(2*pi*0.9*t), 0.035*sin(2*pi*0.5*t)];
end

function value = potentialEnergy(pose,sp,p)
% Independently sum each physical mass's height; do not use composite-body
% properties, Jacobians, or internal inverseDynamics helper functions.
R = eul2rotm(pose(4:6),'XYZ');
origin = pose(1:3).'+sp.homez;
value = -p.platformMass*p.gravity.'*(origin+R*p.platformCOM) ...
    -p.payloadMass*p.gravity.'*(origin+R*p.payloadCOM);
angles = sp.move([0 pose]);
rods = repmat(p.rodMass(:),6/numel(p.rodMass),1);
arms = repmat(p.armMass(:),6/numel(p.armMass),1);
armCOM = repmat(p.armCOM(:),6/numel(p.armCOM),1);
for k = 1:6
    theta = angles(k+1);
    crank = sp.r*[cos(theta)*cos(sp.betaB(k));cos(theta)*sin(sp.betaB(k));sin(theta)];
    base = sp.B(k,:).';
    value = value-p.gravity.'*(rods(k)/2*(origin+R*sp.Pp(k,:).') ...
        +rods(k)/2*(base+crank)+arms(k)*(base+crank*armCOM(k)/sp.r));
end
end

function [sp,p] = fixture()
sp = StuartPlatform(0.1,3.5878,0.1160,0.106/2,0.086602254038,0.02);
p = struct('platformMass',2,'platformCOM',zeros(3,1), ...
    'platformInertia',diag([0.00535189 0.00535189 0.0106204]), ...
    'payloadMass',2.5,'payloadCOM',[0;0;0.1], ...
    'payloadInertia',diag([0.003 0.004 0.002]), ...
    'rodMass',0.55,'armMass',0.33,'armCOM',0.05, ...
    'armInertia',0.33*0.1^2/12,'rotorInertia',12.1013101225e-5*9^2, ...
    'gravity',[0;0;-9.80665],'gearRatio',9);
end
