function tests = testInverseDynamicsStep
    % inverseDynamicsStep against both batch methods, plus viscous bias and the computed-torque step
    tests = functiontests(localfunctions);
end

function setupOnce(~)
    run(fullfile(fileparts(fileparts(fileparts(mfilename('fullpath')))), 'startup.m'));
end

function testMovingTorqueAndEnergy(testCase)
    robot = hexapodRobot();
    bodies = bodyModels(robot);
    t = (0:0.01:0.4).';
    x = [0.006*sin(5*t), 0.005*cos(4*t), 0.007*sin(6*t), 0.05*sin(4*t), 0.04*cos(3*t), 0.035*sin(5*t)];
    for j = 1:3
        body = bodies(j); % uneven masses/COM and nonzero friction too
        body.rodMass = body.rodMass*(0.8:0.08:1.2).';
        body.torsoCOM = body.torsoCOM + [0.012; -0.008; 0];
        body.viscousFriction = (0.01:0.01:0.06).';
        dyn = inverseDynamicsModel(robot, body);
        expected = inverseDynamics(robot, body, t, x);
        for k = 1:numel(t)
            qd = expected.qd(k,:).';
            [M, bias, diagnostics] = inverseDynamicsStep(dyn, x(k,:).', expected.q(k,:).', qd);
            tau = M*expected.qdd(k,:).' + bias;
            verifyEqual(testCase, tau, expected.tauNewtonEuler(k,:).', 'AbsTol', 1e-10);
            verifyEqual(testCase, tau, expected.tauLagrange(k,:).', 'AbsTol', 1e-7);
            verifyEqual(testCase, 0.5*qd.'*M*qd, expected.kineticEnergy(k), 'AbsTol', 1e-12);
            verifyEqual(testCase, M, M.', 'AbsTol', 1e-14);
            verifyGreaterThan(testCase, min(eig(M)), 0);
            verifyLessThan(testCase, diagnostics.closureResidual, 1e-12);
        end
    end
end

function testViscousBiasAndController(testCase)
    robot = hexapodRobot();
    bodies = bodyModels(robot);
    dyn = inverseDynamicsModel(robot, bodies(3));
    x = [0.003; -0.004; 0.005; 0.04; -0.03; 0.02];
    q = inverseKinematics(robot, x.').';
    qd = [0.1; -0.2; 0.3; -0.4; 0.5; -0.6];
    [M, bias] = inverseDynamicsStep(dyn, x, q, qd);
    dyn.viscousFriction = (0.01:0.01:0.06).';
    [Mf, biasF] = inverseDynamicsStep(dyn, x, q, qd);
    verifyEqual(testCase, Mf, M, 'AbsTol', 0);
    verifyEqual(testCase, biasF - bias, dyn.viscousFriction.*qd, 'AbsTol', 1e-14);
    qRef = q + 0.01; qdRef = qd + 0.03; qddRef = ones(6, 1)*0.4;
    [tau, tauMotor] = computedTorqueStep(dyn, x, q, qd, qRef, qdRef, qddRef, 8, 0.8);
    accel = qddRef + 2*0.8*8*(qdRef - qd) + 8^2*(qRef - q);
    verifyEqual(testCase, Mf\(tau - biasF), accel, 'AbsTol', 1e-12);
    verifyEqual(testCase, tauMotor, tau/dyn.gearRatio, 'AbsTol', 0);
end
