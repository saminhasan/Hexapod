function tapered = taperTrajectory(trajectory, riseTime, fallTime)
    % scales every pose column of an [t x] trajectory by a bumpWindow, so it starts and ends at 0
    t = trajectory(:,1);
    assert(riseTime + fallTime <= t(end) - t(1), 'taperTrajectory:Ramps', ...
        'ramps do not fit, riseTime + fallTime must be <= %.6g s', t(end) - t(1))
    w = bumpWindow(t, t(1), riseTime, fallTime, t(end));
    tapered = [t, trajectory(:,2:end).*w(:)];
end
