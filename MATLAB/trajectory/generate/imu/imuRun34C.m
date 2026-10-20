function sternum = imuRun34C()
    % sternum motion [t x y z Rx Ry Rz] of the 34C incremental treadmill run (VO2-BNBY-34C), all 15.4 min:
    % standing to 50 s, walking, then running up to ~3 steps/s. Stands still 5..50 s: zero angles are that posture
    % and the gyro bias comes from there
    imu = readSternumImu('IMU_Running_34C_4p5m_energy');
    sternum = genPoseImu(imu, [5 50], [5 50]);
end
