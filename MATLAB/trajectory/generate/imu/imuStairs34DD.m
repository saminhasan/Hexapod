function sternum = imuStairs34DD()
    % sternum motion [t x y z Rx Ry Rz] of 34DD walking down stairs (Stealth-LAMN-34DD-C5-stairs01), 5.5 s: ~6 steps
    % from 1 s on. The descent itself is slow and goes with the drift; the step-by-step bounce stays. Only 0.1..1 s
    % before the first step (zero angles are that posture), never still enough for a gyro bias
    imu = readSternumImu('IMU_Stairs_Down_34DD');
    sternum = genPoseImu(imu, [0.1 1]);
end
