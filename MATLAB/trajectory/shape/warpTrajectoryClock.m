function [warped, info] = warpTrajectoryClock(trajectory, riseTime, fallTime, method)
    % replays an [t x] trajectory on a clock whose rate goes 0 -> 1 -> 0 with bumpWindow edges, so the
    % path eases in and out with its shape untouched; same dt, (riseTime + fallTime)/2 longer.
    % info.srcTime: source time replayed at each output row; info.plateauDelay = riseTime/2, the lag once
    % the clock is up to speed
    if nargin < 4 || isempty(method), method = 'spline'; end
    assert(size(trajectory, 2) == 7, 'warpTrajectoryClock:Size', 'trajectory must be N x 7 [t x y z Rx Ry Rz]')
    t = trajectory(:,1);
    x = trajectory(:,2:end);
    dt = t(2) - t(1);
    assert(all(abs(diff(t) - dt) <= 1e-9*dt), 'warpTrajectoryClock:Sampling', 'trajectory must be evenly sampled')

    Tin = t(end) - t(1);
    Tout = Tin + 0.5*(riseTime + fallTime); % each ramp loses half its length of source time
    M = round(Tout/dt) + 1;
    tau = (0:M-1).'*dt;
    b = tau(end);
    assert(riseTime + fallTime <= b, 'warpTrajectoryClock:Ramps', ...
        'ramps do not fit, riseTime + fallTime must be <= %.6g s', 2*Tin)

    rate = bumpWindow(tau, 0, riseTime, fallTime, b);
    rate = rate(:);
    s = cumtrapz(tau, rate);
    rateScale = Tin/s(end);     % ~1 + 1e-6, so the last row lands exactly on the end
    s = min(max(s*rateScale, 0), Tin);

    angles = x(:,4:6);
    wasWrapped = all(abs(angles(:)) <= pi + 1e-12);
    x(:,4:6) = unwrap(angles, [], 1);
    xq = interp1(t, x, t(1) + s, method);
    if wasWrapped
        xq(:,4:6) = mod(xq(:,4:6) + pi, 2*pi) - pi;
    end

    warped = [t(1) + tau, xq];
    info = struct('tau', tau, 'rate', rate*rateScale, 'srcTime', s, 'delay', tau - s, ...
        'plateauDelay', 0.5*riseTime, 'dt', dt, 'Tin', Tin, 'Tout', b, 'rateScale', rateScale, ...
        'Nin', size(trajectory, 1), 'Nout', M);
end
