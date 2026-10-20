function w = bumpWindow(t, a, riseTime, fallTime, b)
    % window that is 0 before a, rises to 1 over riseTime, falls back to 0 by b over fallTime;
    % edges are exp(-1/s) blends so every derivative is zero at the corners
    assert(riseTime >= 0 && fallTime >= 0 && b > a && a + riseTime <= b - fallTime, ...
        'bumpWindow:Ramps', 'rise and fall do not fit in [a, b]')
    w = zeros(size(t));
    w(t >= a + riseTime & t <= b - fallTime) = 1;

    if riseTime == 0
        w(t >= a & t <= b - fallTime) = 1;
    else
        idx = t > a & t < a + riseTime;
        s = (t(idx) - a) ./ riseTime;
        f1 = exp(-1 ./ s);
        f2 = exp(-1 ./ (1 - s));
        w(idx) = f1 ./ (f1 + f2);
    end

    if fallTime == 0
        w(t >= a + riseTime & t < b) = 1;
    else
        idx = t > b - fallTime & t < b;
        s = (t(idx) - (b - fallTime)) ./ fallTime;
        f1 = exp(-1 ./ s);
        f2 = exp(-1 ./ (1 - s));
        w(idx) = f2 ./ (f1 + f2);
    end

    w(t <= a | t >= b) = 0;
end
