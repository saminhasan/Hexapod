function tests = testHexapodDynamicsStep
%TESTHEXAPODDYNAMICSSTEP Check instantaneous model against both batch ID methods.
tests = functiontests(localfunctions);
end

function setupOnce(~)
run(fullfile(fileparts(fileparts(mfilename('fullpath'))),'startup.m'));
end

function testMovingTorqueAndEnergy(testCase)
params;
sp = StuartPlatform(r,n,rB,dB,rP,dP);
profiles = inverseDynamicsProfiles(sp);
t = (0:0.01:0.4).';
trajectory = [t,0.006*sin(5*t),0.005*cos(4*t),0.007*sin(6*t), ...
    0.05*sin(4*t),0.04*cos(3*t),0.035*sin(5*t)];
for j = 1:3
    profile = profiles(j);
    % Exercise asymmetric masses/COM and nonzero physical friction too.
    profile.rodMass = profile.rodMass*(0.8:0.08:1.2).';
    profile.payloadCOM = profile.payloadCOM+[0.012;-0.008;0];
    profile.viscousFriction = (0.01:0.01:0.06).';
    model = hexapodDynamicsSetup(sp,profile);
    expected = inverseDynamics(trajectory,sp,profile);
    for k = 1:numel(t)
        w = expected.jointVelocity(k,:).';
        [M,bias,diagnostics] = hexapodDynamicsStep(trajectory(k,2:7).', ...
            expected.jointAngles(k,:).',w,model);
        torque = M*expected.jointAcceleration(k,:).'+bias;
        verifyEqual(testCase,torque,expected.torqueNewtonEuler(k,:).','AbsTol',1e-10);
        verifyEqual(testCase,torque,expected.torqueLagrange(k,:).','AbsTol',1e-7);
        verifyEqual(testCase,0.5*w.'*M*w,expected.kineticEnergy(k),'AbsTol',1e-12);
        verifyEqual(testCase,M,M.','AbsTol',1e-14);
        verifyGreaterThan(testCase,min(eig(M)),0);
        verifyLessThan(testCase,diagnostics.closureResidual,1e-12);
    end
end
end

function testViscousBiasAndController(testCase)
params;
sp = StuartPlatform(r,n,rB,dB,rP,dP);
profiles = inverseDynamicsProfiles(sp);
model = hexapodDynamicsSetup(sp,profiles(3));
pose = [0.003;-0.004;0.005;0.04;-0.03;0.02];
ik = sp.move([0 pose.']); theta = ik(2:7).';
w = [0.1;-0.2;0.3;-0.4;0.5;-0.6];
[M,bias] = hexapodDynamicsStep(pose,theta,w,model);
model.viscousFriction = (0.01:0.01:0.06).';
[Mf,biasF] = hexapodDynamicsStep(pose,theta,w,model);
verifyEqual(testCase,Mf,M,'AbsTol',0);
verifyEqual(testCase,biasF-bias,model.viscousFriction.*w,'AbsTol',1e-14);
ref = theta+0.01; refDot = w+0.03; refDDot = ones(6,1)*0.4;
[tc,tm] = hexapodComputedTorqueStep(pose,theta,w,ref,refDot,refDDot,model,8,0.8);
accel = refDDot+2*0.8*8*(refDot-w)+8^2*(ref-theta);
verifyEqual(testCase,Mf\(tc-biasF),accel,'AbsTol',1e-12);
verifyEqual(testCase,tm,tc/model.gearRatio,'AbsTol',0);
end
