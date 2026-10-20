function m = readImu(captureDir, run, sensor)
    % aligned imu file of one run: [t ax ay az gx gy gz], m/s^2 and deg/s, imu axes (x up, z out of the body)
    f = dir(fullfile(captureDir, '**', run + "_TS-" + sensor + "_*_aligned.csv"));
    m = readmatrix(fullfile(f(1).folder, f(1).name));
    m = m(:, 1:7); % low-g accel + gyro, low-g never saturates in these runs
end
