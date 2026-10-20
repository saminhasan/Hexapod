function [t, q] = bumpSequence(amplitudes, n, dt)
    % one column at a time: 0 -> +A -> hold -> 0, pause, 0 -> -A -> hold -> 0, pause, then the next;
    % every rise, hold, fall and pause is 1 s, with 1 s of zero before each set, whole set n times
    ramp = 1; flat = 1; gap = 1; lead = 1;
    bump = ramp + flat + ramp;
    setTime = lead + numel(amplitudes) * (bump + gap + bump + gap);
    t = (0:dt:n*setTime)';
    q = zeros(numel(t), numel(amplitudes));
    for rep = 1:n
        t0 = (rep - 1)*setTime + lead;
        for k = 1:numel(amplitudes)
            for sgn = [1 -1]
                q(:,k) = q(:,k) + sgn*amplitudes(k) * bumpWindow(t, t0, ramp, ramp, t0 + bump);
                t0 = t0 + bump + gap;
            end
        end
    end
end
