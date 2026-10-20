function sternum = imuThrow34D()
    % sternum motion [t x y z Rx Ry Rz] of the 34D lateral throws (Stealth-AYAT-34D-C5-lateral01): stands, then ~10
    % twisting throws (torso turns about +-35 deg, one every ~2.4 s) from 5.5 s to 29.8 s. Cut there: after the last
    % throw they bend down ~90 deg and back up (30-32 s), which isn't part of the throws. Stands 0.5..5 s (zero
    % angles are that posture), never still enough for a gyro bias
    imu = readSternumImu('IMU_Lateral_Throw_34D');
    keep = imu.t < 29.8;
    imu = struct('t', imu.t(keep), 'f', imu.f(keep,:), 'w', imu.w(keep,:));
    sternum = genPoseImu(imu, [0.5 5]);
end
