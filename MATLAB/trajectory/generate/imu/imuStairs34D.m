function sternum = imuStairs34D()
    % sternum motion [t x y z Rx Ry Rz] of 34D walking down stairs (Stealth-AYAT-34D-C5-stairs01), 8.8 s: ~5 steps
    % from 3 s on. The descent itself is slow and goes with the drift; the step-by-step bounce stays. Stands at the
    % top 0.2..2.8 s (zero angles are that posture), never still enough for a gyro bias
    imu = readSternumImu('IMU_Stairs_Down_34D');
    sternum = genPoseImu(imu, [0.2 2.8]);
end
