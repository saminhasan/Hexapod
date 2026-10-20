function L = lineUpImu(tS, fS, wS, imu, fc)
    % low pass sim and measured imu to fc, find the capture offset, put both on the sim clock at 1 kHz
    % L.offset: t_sim = t_imu + offset
    fs = 1000;
    [b, a] = butter(4, fc / (fs/2));
    ts = (ceil(tS(1)*fs) : floor(tS(end)*fs))' / fs;
    S = filtfilt(b, a, interp1(tS, [fS wS], ts));

    [b, a] = butter(4, fc / (1600/2)); % imu filtered at its own 1600 Hz before going to 1 kHz
    tm = (imu(1,1) : 1/fs : imu(end,1))';
    M = interp1(imu(:,1), filtfilt(b, a, imu(:,2:7)), tm);

    % coarse lag off the motion envelope (start, stop, the ramps every loop), then fine lag off the
    % signals within +-0.3 s - the 3 Hz step repeats every 0.33 s so the fine search alone could alias
    env = @(x) movmean(abs(x - mean(x)), fs);
    lag = bestLag(env(S), env(M), []);
    lag = bestLag(S, M, lag + (-300:300));
    L.offset = ts(1) - tm(1) + lag/fs;

    k = ts >= tm(1) + L.offset & ts <= tm(end) + L.offset;
    L.t = ts(k);
    L.sim = S(k,:);
    L.meas = interp1(tm + L.offset, M, L.t);
    L.r = diag(corr(L.sim, L.meas)).';
    L.rmse = rms(L.sim - L.meas);
    L.rmsMeas = rms(L.meas - mean(L.meas));
end
