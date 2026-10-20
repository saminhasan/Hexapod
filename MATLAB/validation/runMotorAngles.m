function M = runMotorAngles(S, playN, qRef)
    % commanded (qCmd) and measured (q) crank angles of one PLAY, rad, IK column order
    % on the firmware row clock: the slave advances one row per 1 kHz tick, so row/1000 is time,
    % free of the USB batching that the PC arrival stamps carry
    % M.refErr: max |qCmd - qRef| deg at the raw samples, ~1e-6 (float32) when the stored trajectory is qRef
    nRows = size(qRef, 1);
    play = S.tx.t(S.tx.msg == "PLAY");
    stop = S.tx.t(S.tx.msg == "STOP");
    t0 = play(playN);
    t1 = stop(find(stop > t0, 1));
    fb = S.fb(S.fb(:,1) > t0 & S.fb(:,1) < t1, :);

    R = cell(1,3); V = cell(1,3); M.refErr = 0;
    for s = 1:3 % slave s drives columns 2s-1 (L) and 2s (R)
        k = fb(:,2) == s;
        r = fb(k,9) - 1; % the command was loaded from the row before the reported playhead
        r = r + nRows * cumsum([0; diff(r) < -nRows/2]); % unwrap end-of-trajectory wraps
        v = deg2rad(fb(k, [3 4 6 7])); % [uL yL uR yR]
        [r, ia] = unique(r);
        v = v(ia,:);
        R{s} = r(r >= 0);
        V{s} = v(r >= 0, :);
        e = V{s}(:, [1 3]) - qRef(mod(R{s}, nRows) + 1, 2*s-1:2*s);
        M.refErr = max(M.refErr, rad2deg(max(abs(e), [], 'all')));
    end

    M.row = (0 : min(cellfun(@max, R)))';
    for s = 1:3
        x = interp1(R{s}, V{s}, M.row, 'spline'); % C2, so the sim gets clean accelerations
        M.qCmd(:, 2*s-1:2*s) = x(:, [1 3]);
        M.q(:, 2*s-1:2*s) = x(:, [2 4]);
    end
    M.t = M.row / 1000;
end
