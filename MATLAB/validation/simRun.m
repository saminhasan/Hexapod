function [tS, fS, wS] = simRun(robot, M, pad, fcMotor, simFile)
    % torso imu sim of one run, measured crank angles held for pad s either side of the PLAY, cached.
    % Sim time tS <-> trajectory row (tS - pad)*1000. fcMotor: zero-phase low pass on the measured angles;
    % feedback positions are quantised to 0.022 deg (16 bit over +-12.5 rad) and differentiated twice that
    % turns into acceleration rising as f^4, so above ~30 Hz the replay would be quantisation, not motion
    if isfile(simFile)
        load(simFile, 'tS', 'fS', 'wS');
        return
    end
    [b, a] = butter(4, fcMotor/500);
    q = filtfilt(b, a, M.q);
    n = pad*1000;
    q = [repmat(q(1,:), n, 1); q; repmat(q(end,:), n, 1)]; % even 1 kHz: the lookups are cubic splines, a gap would ring
    [tS, fS, wS] = simTorsoImu(robot, (0:size(q, 1)-1)'/1000, q);
    save(simFile, 'tS', 'fS', 'wS');
end
