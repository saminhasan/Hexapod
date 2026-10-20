function tests = testGenPoseImu
    % known answer: a platform motion -> what the manikin's sternum IMU would read -> genPoseImu + placePlatform ->
    % the platform motion again. Hard case on purpose: 15 deg lean, +-32 deg yaw, pitch ripple in step with the
    % bounce (what fooled the old body-frame lean and the summed-up gyro)
    tests = functiontests(localfunctions);
end

function setupOnce(testCase)
    run(fullfile(fileparts(fileparts(fileparts(mfilename('fullpath')))), 'startup.m'));
    robot = hexapodRobot();
    x = genPoseSynthetic(360);                               % 120 s of running at 3 steps/s
    x(:,5) = 2.5*x(:,5);
    x(:,6) = deg2rad(15) + 2*(x(:,6) - deg2rad(10));
    x(:,7) = 4*x(:,7);
    testCase.TestData.robot = robot;
    testCase.TestData.motion = taperTrajectory(x, 3, 3);    % eases in from home over 3 s
end

function testSessionWithStanding(testCase)
    % 60 s standing, then the run: angles relative to standing, so the lean comes back as it was
    robot = testCase.TestData.robot;
    m = testCase.TestData.motion;
    x = [(0:60e3-1).'/1e3, zeros(60e3, 6); m(:,1) + 60, m(:,2:7)];
    platform = recovered(robot, genPoseImu(sternumImu(robot, x), [5 50], [5 50]));
    [eRms, eMeanPitch] = compare(platform, x, x(:,1) > 70 & x(:,1) < x(end,1) - 10);
    fprintf('with standing: error rms x y z [%s] mm, Rx Ry Rz [%s] deg, mean pitch %+.2f deg\n', ...
        sprintf('%.2f ', 1000*eRms(1:3)), sprintf('%.2f ', rad2deg(eRms(4:6))), rad2deg(eMeanPitch));
    verifyLessThan(testCase, eRms(1:3), 2e-3);
    verifyLessThan(testCase, eRms(4:6), deg2rad(0.5));
    verifyLessThan(testCase, abs(eMeanPitch), deg2rad(0.3));
end

function testShortClip(testCase)
    % like the stairs and throws: 1 s upright, then 8 s of motion, no gyro bias known
    robot = testCase.TestData.robot;
    m = testCase.TestData.motion;
    clip = m(m(:,1) >= 50 & m(:,1) < 58,:);
    clip(:,1) = clip(:,1) - clip(1,1);
    clip = taperTrajectory(clip, 1, 1);
    x = [(0:999).'/1e3, zeros(1000, 6); clip(:,1) + 1, clip(:,2:7)];
    platform = recovered(robot, genPoseImu(sternumImu(robot, x), [0.1 0.9]));
    eRms = compare(platform, x, x(:,1) > 1.5 & x(:,1) < x(end,1) - 1.5);
    fprintf('9 s clip: error rms x y z [%s] mm, Rx Ry Rz [%s] deg\n', ...
        sprintf('%.2f ', 1000*eRms(1:3)), sprintf('%.2f ', rad2deg(eRms(4:6))));
    verifyLessThan(testCase, eRms(1:3), 4e-3);
    verifyLessThan(testCase, eRms(4:6), deg2rad(1));
end

function testRodEndsAtHome(testCase)
    % at home every rod end already sits 4.7 deg off square on its screw (CAD), legs far from stretched
    [~, reach, rodEnd] = workspaceUse(testCase.TestData.robot, zeros(1, 6));
    verifyEqual(testCase, rodEnd, 4.70, 'AbsTol', 0.01);
    verifyLessThan(testCase, reach, 0.2);
end

function platform = recovered(robot, sternum)
    % the recording path of referenceTrajectory: sternum motion -> platform origin, drift out, average moved
    platform = placePlatform(robot, struct('body', sternum, 'bodyToPlatform', -robot.sternumImu, 'recorded', true));
end

function imu = sternumImu(robot, x)
    % specific force and rate a rigid manikin's sternum IMU reads on platform motion x ([t x], 1 kHz), in the
    % IMU's axes: x up, y side, z out of the chest, tilted 26.6 deg back like on the CAD
    fs = 1000; g = 9.80665;
    R = eul2rotm(x(:,5:7), 'XYZ');
    p = x(:,2:4) + reshape(pagemtimes(R, robot.sternumImu), 3, []).';
    a = gradient(gradient(p.', 1/fs), 1/fs).';
    Rd = zeros(size(R));
    for i = 1:9
        [r, c] = ind2sub([3 3], i);
        Rd(r,c,:) = reshape(gradient(squeeze(R(r,c,:)), 1/fs), 1, 1, []);
    end
    W = pagemtimes(permute(R, [2 1 3]), Rd);                 % skew of the body rate
    w = [squeeze(W(3,2,:)), squeeze(W(1,3,:)), squeeze(W(2,1,:))];
    fb = reshape(pagemtimes(permute(R, [2 1 3]), permute(a + [0 0 g], [2 3 1])), 3, []).';
    phi = deg2rad(26.6);
    S = [[-sin(phi); 0; cos(phi)], [0; -1; 0], [cos(phi); 0; sin(phi)]]; % IMU axes in manikin axes
    imu = struct('t', x(:,1), 'f', fb*S, 'w', w*S);
end

function [eRms, eMeanPitch] = compare(platform, x, k)
    % rms error over rows k after dropping what can't be recovered: translation below 0.5 Hz and its average
    % (placePlatform picks that), mean heading
    [b, a] = butter(2, 0.5/500, 'high');
    truth = [filtfilt(b, a, x(:,2:4)), x(:,5:7)];
    got = platform(:,2:7);
    got(:,[1:3 6]) = got(:,[1:3 6]) - mean(got(k,[1:3 6])) + mean(truth(k,[1:3 6]));
    e = got(k,:) - truth(k,:);
    eRms = rms(e);
    eMeanPitch = mean(e(:,5));
end
