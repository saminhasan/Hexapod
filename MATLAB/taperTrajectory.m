function wtrajectory = taperTrajectory(trajectory, riseTime, fallTime)
    %TAPERTRAJECTORY  Taper a trajectory in and out with a C-infinity bump window.
    %
    %   wtrajectory = taperTrajectory(trajectory, riseTime, fallTime)
    %
    %   riseTime  - 0-to-100% taper length at the start (s)
    %   fallTime  - 100-to-0% taper length at the end   (s)
    %
    %   Unlike a Tukey taper, every derivative of this window vanishes at the
    %   four corners, so the taper itself excites nothing.
    
    t = trajectory(:,1);
    T = t(end) - t(1);
    
    if riseTime + fallTime > T
        error('Ramps do not fit: riseTime + fallTime must be <= %.6g s.', T);
    end
    
    win = bumpwin(t, t(1), riseTime, fallTime, t(end));
    win = win(:);
    
    wtrajectory          = zeros(size(trajectory));
    wtrajectory(:,1)     = t;
    wtrajectory(:,2:end) = trajectory(:,2:end) .* win;
end
