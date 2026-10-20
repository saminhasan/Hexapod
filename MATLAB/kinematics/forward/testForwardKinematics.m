function tests = testForwardKinematics
    % IK -> FK round trips on the rig trajectory: whole file, cold starts, quantised angles, Jacobian, speed
    tests = functiontests(localfunctions);
end

function setupOnce(testCase)
    run(fullfile(fileparts(fileparts(fileparts(mfilename('fullpath')))), 'startup.m'));
    testCase.TestData.robot = hexapodRobot();
    testCase.TestData.ref = referenceTrajectory(testCase.TestData.robot, 'syn', 30);
end

function testRoundTripWholeTrajectory(testCase)
    % every row of the 11 s file, each started from the previous answer, the first from home
    robot = testCase.TestData.robot; ref = testCase.TestData.ref;
    [x, info] = forwardKinematics(robot, ref.q, zeros(1, 6));
    e = x - ref.x;
    fprintf('round trip: max pose error %.2e m, %.2e rad; iterations %d max, %.2f mean\n', ...
        max(abs(e(:,1:3)), [], 'all'), max(abs(e(:,4:6)), [], 'all'), max(info.iter), mean(info.iter));
    verifyLessThan(testCase, max(abs(e), [], 'all'), 1e-9);
    verifyLessThanOrEqual(testCase, max(info.iter), 6);
end

function testColdStartFromHome(testCase)
    % one-shot solves from home to the 20 most extreme poses
    robot = testCase.TestData.robot; ref = testCase.TestData.ref;
    [~, far] = maxk(vecnorm(ref.x./max(abs(ref.x)), 2, 2), 20);
    for k = far.'
        [x, info] = forwardKinematics(robot, ref.q(k,:), zeros(1, 6));
        verifyLessThan(testCase, max(abs(x - ref.x(k,:))), 1e-9);
        verifyLessThanOrEqual(testCase, info.iter, 10);
    end
end

function testQuantisedAngles(testCase)
    % the rig reports angles in 0.022 deg steps: pose error from that alone
    robot = testCase.TestData.robot; ref = testCase.TestData.ref;
    step = 25/65535;
    x = forwardKinematics(robot, round(ref.q/step)*step, zeros(1, 6));
    e = x - ref.x;
    fprintf('0.022 deg angle steps -> pose error rms [%s] mm, [%s] deg\n', ...
        sprintf('%.3f ', 1000*rms(e(:,1:3))), sprintf('%.4f ', rad2deg(rms(e(:,4:6)))));
    verifyLessThan(testCase, max(abs(e(:,1:3)), [], 'all'), 1e-3);
    verifyLessThan(testCase, max(abs(e(:,4:6)), [], 'all'), deg2rad(0.1));
end

function testJacobianMatchesDifferences(testCase)
    % closed-form dq/dx against central differences of the IK at a tilted pose
    robot = testCase.TestData.robot;
    x = [0.004; -0.007; 0.008; 0.07; -0.06; 0.05];
    h = 1e-7;
    Jn = zeros(6);
    for j = 1:6
        d = zeros(6, 1); d(j) = h;
        Jn(:,j) = (inverseKinematicsStep(robot.geom, x + d) - inverseKinematicsStep(robot.geom, x - d))/(2*h);
    end
    verifyLessThan(testCase, max(abs(kinematicJacobian(robot.geom, x) - Jn), [], 'all'), 1e-6);
end

function testStepVersionMatches(testCase)
    % the block version walks the trajectory to the same poses
    robot = testCase.TestData.robot; ref = testCase.TestData.ref;
    x = zeros(6, 1); e = 0;
    for k = 1:10:size(ref.q, 1)
        x = forwardKinematicsStep(robot.geom, ref.q(k,:).', x);
        e = max(e, max(abs(x.' - ref.x(k,:))));
    end
    fprintf('step version: max pose error %.2e over the trajectory (every 10th row)\n', e);
    verifyLessThan(testCase, e, 1e-9);
end

function testSpeed(testCase)
    robot = testCase.TestData.robot; ref = testCase.TestData.ref;
    tic; forwardKinematics(robot, ref.q(1:2000,:), zeros(1, 6)); dt = toc/2000;
    fprintf('%.1f us per sample (MATLAB)\n', 1e6*dt);
    verifyLessThan(testCase, dt, 1e-3); % keeps up with 1 kHz even in MATLAB
end
