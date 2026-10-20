function sternum = imuThrow34DD()
    % sternum motion [t x y z Rx Ry Rz] of the 34DD lateral throws (Stealth-LAMN-34DD-C5-lateralthrow01): stands,
    % then ~10 twisting throws (torso turns about +-47 deg, one every ~2.3 s) from 4.8 s to 27.2 s. Cut there: after
    % the last throw they bend down ~90 deg and back up (27.3-29.5 s), which isn't part of the throws. Stands
    % 0.5..4.5 s (zero angles are that posture), never still enough for a gyro bias
    imu = readSternumImu('IMU_Lateral_Throw_34DD');
    keep = imu.t < 27.2;
    imu = struct('t', imu.t(keep), 'f', imu.f(keep,:), 'w', imu.w(keep,:));
    sternum = genPoseImu(imu, [0.5 4.5]);
end
