function trajectory = genPoseStep(n)
    % +/- steps on one axis at a time, x y z (m) then Rx Ry Rz (rad), smooth edges, n sets: [t x y z Rx Ry Rz]
    amplitudes = [0.01, 0.01, 0.04, deg2rad(10.0), deg2rad(10.0), deg2rad(15.0)];
    [t, x] = bumpSequence(amplitudes, n, 1e-3);
    trajectory = [t, x];
end
