function [c, u] = bestShift(robot, x, c)
    % constant shift c (1 x 3, m, started from the given c) that keeps the worst row of x (N x 6 platform poses)
    % furthest from the limits; u = worst workspaceUse after the shift. Optimises on the rows near the worst, then
    % rechecks every row, until no other row takes over (every row counts: short peaks fall between subsamples)
    opt = optimset('Display', 'off', 'TolX', 1e-5, 'TolFun', 1e-5, 'MaxFunEvals', 600);
    for it = 1:6
        use = workspaceUse(robot, x + [c, 0 0 0]);
        xs = x(use > max(use) - 0.3,:);
        f = @(c) max(workspaceUse(robot, xs + [c, 0 0 0]));
        cNew = fminsearch(f, fminsearch(f, c, opt), opt);  % restarted: the worst row switches, the max isn't smooth
        if max(workspaceUse(robot, x + [cNew, 0 0 0])) >= max(use) - 1e-4, break, end
        c = cNew;
    end
    u = max(workspaceUse(robot, x + [c, 0 0 0]));
end
