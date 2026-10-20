function tests = testInverseDynamics
    % physics checks on inverseDynamics, no sims: static load, zero mass, NE vs Lagrange, rotor, virtual work,
    % uneven timing, supplied kinematics
    tests = functiontests(localfunctions);
end

function setupOnce(~)
    run(fullfile(fileparts(fileparts(fileparts(mfilename('fullpath')))), 'startup.m'));
end

function testStaticSymmetricWeight(testCase)
    % at q = 0 each crank lever is r and carries 1/6 of platform + torso, one whole rod and half an arm
    [robot, p] = fixture();
    t = (0:0.01:0.1).';
    out = inverseDynamics(robot, p, t, zeros(numel(t), 6));
    expected = -p.gravity(3)*robot.geom.r*((p.platformMass + p.torsoMass)/6 + p.rodMass + p.armMass/2);
    verifyEqual(testCase, out.tauNewtonEuler, expected*ones(numel(t), 6), 'AbsTol', 1e-10);
    verifyEqual(testCase, out.tauLagrange, out.tauNewtonEuler, 'AbsTol', 1e-10);
    verifyLessThan(testCase, max(out.closureResidual), 1e-12);
end

function testZeroMassInertia(testCase)
    [robot, p] = fixture();
    p.platformMass = 0; p.torsoMass = 0; p.rodMass = 0; p.armMass = 0;
    p.platformInertia = zeros(3); p.torsoInertia = zeros(3);
    p.armInertia = 0; p.reflectedInertia = 0;
    [t, x] = mixedMotion();
    out = inverseDynamics(robot, p, t, x);
    verifyEqual(testCase, out.tauLagrange, zeros(size(out.tauLagrange)), 'AbsTol', 0);
    verifyEqual(testCase, out.tauNewtonEuler, zeros(size(out.tauNewtonEuler)), 'AbsTol', 0);
end

function testIndependentMethodsOnMixedMotion(testCase)
    [robot, p] = fixture();
    p.platformCOM = [0.005; -0.004; 0.012];
    p.torsoCOM = [0.025; -0.017; 0.1];
    p.rodMass = [0.41; 0.45; 0.48; 0.51; 0.55; 0.6];
    p.armMass = [0.2; 0.22; 0.24; 0.26; 0.28; 0.3];
    p.armInertia = p.armMass*robot.geom.r^2/12;
    p.viscousFriction = 0.01;
    [t, x] = mixedMotion();
    out = inverseDynamics(robot, p, t, x);
    scale = max(1, max(abs(out.tauNewtonEuler), [], 'all'));
    verifyLessThan(testCase, out.maxMethodDifference, 1e-6*scale);
    verifyEqual(testCase, out.motorTauNewtonEuler, out.tauNewtonEuler/p.gearRatio, 'AbsTol', 1e-12);
end

function testKnownRotorAcceleration(testCase)
    % at rest at home z'' = r*q'' exactly, so only the rotor inertia is left
    [robot, p] = fixture();
    p.platformMass = 0; p.torsoMass = 0; p.rodMass = 0; p.armMass = 0;
    p.platformInertia = zeros(3); p.torsoInertia = zeros(3);
    p.armInertia = 0; p.reflectedInertia = 0.02; p.gravity = zeros(3, 1);
    t = (-0.05:0.001:0.05).';
    az = 0.7;
    x = zeros(numel(t), 6);
    x(:,3) = 0.5*az*t.^2;
    out = inverseDynamics(robot, p, t, x);
    [~, middle] = min(abs(t));
    expected = p.reflectedInertia*az/robot.geom.r;
    verifyEqual(testCase, out.tauNewtonEuler(middle,:), expected*ones(1, 6), 'AbsTol', 1e-10);
    verifyEqual(testCase, out.tauLagrange(middle,:), expected*ones(1, 6), 'AbsTol', 1e-10);
    verifyEqual(testCase, out.xdd(:,3), az*ones(numel(t), 1), 'AbsTol', 1e-9); % ends too
end

function testGravityVirtualWorkAtTiltedPose(testCase)
    [robot, p] = fixture();
    p.torsoCOM = [0.025; -0.017; 0.11];
    p.platformCOM = [0.004; 0.009; -0.01];
    p.rodMass = [0.31; 0.4; 0.48; 0.39; 0.55; 0.43];
    pose = [0.004 -0.007 0.008 0.07 -0.06 0.05];
    t = (0:0.01:0.1).';
    out = inverseDynamics(robot, p, t, repmat(pose, numel(t), 1));
    direction = [0.02 -0.013 0.015 0.2 -0.17 0.12];
    step = 1e-5;
    plus = pose + step*direction; minus = pose - step*direction;
    dq = (inverseKinematics(robot, plus) - inverseKinematics(robot, minus))/(2*step);
    dV = (potentialEnergy(plus, robot, p) - potentialEnergy(minus, robot, p))/(2*step);
    verifyEqual(testCase, out.tauNewtonEuler(6,:)*dq.', dV, 'AbsTol', 1e-7);
    verifyEqual(testCase, out.tauLagrange(6,:)*dq.', dV, 'AbsTol', 1e-7);
end

function testNonuniformTimeAndInputValidation(testCase)
    [robot, p] = fixture();
    t = [0; 0.009; 0.021; 0.029; 0.041; 0.055; 0.061; 0.073; 0.08];
    x = zeros(numel(t), 6);
    x(:,3) = 0.5*0.5*t.^2;
    out = inverseDynamics(robot, p, t, x);
    verifyEqual(testCase, out.xdd(:,3), 0.5*ones(numel(t), 1), 'AbsTol', 1e-9);
    verifyError(testCase, @() inverseDynamics(robot, p, 0, zeros(1, 6)), 'inverseDynamics:Time');
    t(3) = t(2);
    verifyError(testCase, @() inverseDynamics(robot, p, t, x), 'inverseDynamics:Time');
end

function testSuppliedWorldKinematics(testCase)
    [robot, p] = fixture();
    t = (0:0.002:0.5).';
    w = 2*pi; pitch = 0.08;
    x = [0.005*sin(w*t), zeros(size(t)), 0.008*cos(w*t), zeros(size(t)), pitch*ones(size(t)), 0.04*sin(w*t)];
    kin.linearVelocity = [0.005*w*cos(w*t), zeros(size(t)), -0.008*w*sin(w*t)];
    kin.linearAcceleration = [-0.005*w^2*sin(w*t), zeros(size(t)), -0.008*w^2*cos(w*t)];
    kin.angularVelocity = 0.04*w*cos(w*t)*[sin(pitch) 0 cos(pitch)]; % R = Ry(pitch)*Rz(yaw): yaw axis in world
    kin.angularAcceleration = -0.04*w^2*sin(w*t)*[sin(pitch) 0 cos(pitch)];
    sensor = inverseDynamics(robot, p, t, x, kin);
    sampled = inverseDynamics(robot, p, t, x);
    verifyEqual(testCase, sensor.omega, kin.angularVelocity, 'AbsTol', 1e-12);
    verifyEqual(testCase, sensor.alpha, kin.angularAcceleration, 'AbsTol', 1e-12);
    verifyEqual(testCase, sensor.tauNewtonEuler(3:end-2,:), sampled.tauNewtonEuler(3:end-2,:), 'AbsTol', 1e-5);
    verifyLessThan(testCase, sensor.maxMethodDifference, 1e-7);
    bad = kin; bad.linearVelocity = bad.linearVelocity(1:end-1,:);
    verifyError(testCase, @() inverseDynamics(robot, p, t, x, bad), 'inverseDynamics:Kinematics');
end

function [t, x] = mixedMotion()
    t = (0:0.005:1).';
    x = [0.006*sin(2*pi*0.8*t), 0.005*cos(2*pi*0.6*t), 0.007*sin(2*pi*1.1*t), ...
        0.05*sin(2*pi*0.7*t), 0.04*cos(2*pi*0.9*t), 0.035*sin(2*pi*0.5*t)];
end

function V = potentialEnergy(pose, robot, p)
    % every mass's height summed on its own, no composite body or Jacobians from inverseDynamics
    geom = robot.geom;
    R = eul2rotm(pose(4:6), 'XYZ');
    origin = pose(1:3).' + geom.home;
    V = -p.platformMass*p.gravity.'*(origin + R*p.platformCOM) - p.torsoMass*p.gravity.'*(origin + R*p.torsoCOM);
    q = inverseKinematics(robot, pose);
    rods = repmat(p.rodMass(:), 6/numel(p.rodMass), 1);
    arms = repmat(p.armMass(:), 6/numel(p.armMass), 1);
    armCOM = repmat(p.armCOM(:), 6/numel(p.armCOM), 1);
    for k = 1:6
        crank = geom.r*[cos(q(k))*cos(geom.betaB(k)); cos(q(k))*sin(geom.betaB(k)); sin(q(k))];
        base = geom.base(k,:).';
        V = V - p.gravity.'*(rods(k)/2*(origin + R*geom.platformPoints(k,:).') + rods(k)/2*(base + crank) ...
            + arms(k)*(base + crank*armCOM(k)/geom.r));
    end
end

function [robot, p] = fixture()
    robot = hexapodRobot();
    p = struct('platformMass', 2, 'platformCOM', zeros(3, 1), 'platformInertia', diag([0.00535189 0.00535189 0.0106204]), ...
        'torsoMass', 2.5, 'torsoCOM', [0; 0; 0.1], 'torsoInertia', diag([0.003 0.004 0.002]), ...
        'rodMass', 0.55, 'armMass', 0.33, 'armCOM', 0.05, 'armInertia', 0.33*0.1^2/12, ...
        'reflectedInertia', 12.1013101225e-5*9^2, 'gravity', [0; 0; -9.80665], 'gearRatio', 9);
end
