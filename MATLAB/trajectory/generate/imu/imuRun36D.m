function sternum = imuRun36D()
    % sternum motion [t x y z Rx Ry Rz] of the 36D incremental treadmill run (VO2-RLJM), all 15.9 min: standing to
    % 50 s, walking, then running up to ~3.2 steps/s. The first 1.5 s are sensor junk (up to 25 g), held at the
    % standing values. Stands still 5..50 s: zero angles are that posture and the gyro bias comes from there
    imu = readSternumImu('IMU_Running_36D_4ms_energy');
    junk = imu.t < 2;
    standing = imu.t >= 2 & imu.t < 5;
    imu.f(junk,:) = repmat(mean(imu.f(standing,:)), nnz(junk), 1);
    imu.w(junk,:) = repmat(mean(imu.w(standing,:)), nnz(junk), 1);
    sternum = genPoseImu(imu, [5 50], [5 50]);
end
