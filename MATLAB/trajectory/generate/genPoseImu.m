function sternum = genPoseImu(imu, level, bias)
    % sternum pose [t x y z Rx Ry Rz] (m, rad) at 1 kHz from a sternum IMU recording, level frame (x forward, y left,
    % z up). imu: t (s), f specific force (N x 3, m/s^2), w rate (N x 3, rad/s), sensor axes (x up, y side, z out of
    % the chest). level: [t1 t2] standing upright, zero angles are this posture (default: the whole recording's
    % average posture). bias: [t1 t2] truly still, for the gyro bias (default: none). Kept: 0..100 Hz; translation
    % and heading below 0.5 Hz are drift and dropped, tilt is kept
    if nargin < 2, level = []; end
    if nargin < 3, bias = []; end
    fs = 1000;
    g = 9.80665;
    lowPassHz = 100;
    driftHz = 0.5;

    % level: up from gravity, forward from the sensor's z axis flattened
    fsIn = 1/median(diff(imu.t));
    if isempty(level), k = true(size(imu.t)); else, k = imu.t > level(1) & imu.t < level(2); end
    up = mean(imu.f(k,:)).';
    up = up/norm(up);
    fw = [0; 0; 1] - up(3)*up;
    fw = fw/norm(fw);
    R0 = [fw.'; cross(up, fw).'; up.'];
    x0 = [imu.f*R0.', imu.w*R0.'];
    [b, a] = butter(4, lowPassHz/(fsIn/2));
    x0 = filtfilt(b, a, x0);
    [P, Q] = rat(fs/fsIn);
    x = resample(x0, P, Q);
    t = (0:size(x, 1)-1).'/fs;
    f = x(:,1:3);
    w = x(:,4:6);
    if ~isempty(bias), w = w - mean(w(t > bias(1) - imu.t(1) & t < bias(2) - imu.t(1),:)); end

    % orientation: the gyro integrated step by step carries all of it, gravity only stops the tilt drifting.
    % Gravity is averaged in the level frame, where the bounce averages out, not in the body frame, where the
    % torso pitching in step with the bounce would read as lean
    R = integrateGyro(w, fs);
    [b, a] = butter(2, 0.3/(fs/2));
    for it = 1:4
        G = filtfilt(b, a, squeeze(pagemtimes(R, permute(f, [2 3 1]))).');
        G = G./vecnorm(G, 2, 2);
        ax = cross(G, repmat([0 0 1], size(G, 1), 1), 2);
        ang = asin(min(vecnorm(ax, 2, 2), 1));
        R = pagemtimes(axang2rotm([ax./max(vecnorm(ax, 2, 2), eps), ang]), R);
    end
    [b, a] = butter(2, driftHz/(fs/2));
    psi = filtfilt(b, a, unwrap(squeeze(atan2(R(2,1,:), R(1,1,:)))));
    R = pagemtimes(axang2rotm([zeros(numel(psi), 2), ones(numel(psi), 1), -psi]), R); % slow heading out

    % position: specific force to the level frame, gravity out, integrated twice
    p = fftIntegrate(squeeze(pagemtimes(R, permute(f, [2 3 1]))).' - [0 0 g], fs, driftHz, 2);
    sternum = [t + imu.t(1), p, rotm2eul(R, 'XYZ')];
end
