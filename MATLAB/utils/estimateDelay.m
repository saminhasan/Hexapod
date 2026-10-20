function [d, dAxis, yd] = estimateDelay(t, y, tr, yr, dMax)
    % one lag d (s) for all axes so that setpoint yr(t - d) best matches measured y(t)
    % dAxis: same fit per axis, diagnostic, should all come out close to d
    % yd: setpoint shifted by d, sampled on t
    pp = spline(tr, yr.');
    f = @(d) ppval(pp, min(max(t.' - d, tr(1)), tr(end))).'; % shifted setpoint, clamped at the ends
    J = @(d, w) sum(mean((y - f(d)).^2) .* w);
    opt = optimset('TolX', 1e-7);

    w = 1 ./ var(yr); % normalize so m and rad can be added
    w(var(yr) < 1e-12) = 0; % skip axes that dont move
    d = fminbnd(@(d) J(d, w), -dMax, dMax, opt);
    dAxis = arrayfun(@(i) fminbnd(@(d) J(d, (1:size(y,2)) == i), -dMax, dMax, opt), 1:size(y,2));
    dAxis(w == 0) = NaN; % no motion, no delay to see

    if abs(d) > 0.99 * dMax
        warning('delay hit dMax = %.4f s, increase dMax', dMax);
    end
    yd = f(d);
end
