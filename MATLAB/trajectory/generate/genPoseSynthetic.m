function trajectory = genPoseSynthetic(n)
    % synthetic running torso motion, n steps of p = 1/3 s: [t x y z Rx Ry Rz] (m, rad)
    dt = 1e-3;
    p = 1/3;                % step period, 0.33 to 0.44 (stride = 2p); p = 0.40 gives z -6..+2 cm
    t = (0:dt:p*n)';
    tShift = 0.192*p;       % lines up with the cam trajectory, 0.064 s at p = 1/3
    [z, tE] = runningZ(t, p, tShift);

    f = 1/p;                % step frequency, 3 Hz
    ws = 2*pi*f;            % step
    wst = pi*f;             % stride
    tC = tE - tShift;       % peak vertical load
    tOn = tC - 0.10;        % contact onset, ~200 ms of stance

    x = 0.010 - 0.012*cos(ws*(t - tC));                     % fore-aft, in phase with z (bouncing gait)
    y = 0.010*cos(wst*(t - tC));                            % sway toward the stance side
    Rx = deg2rad(3.0)*cos(wst*(t - tC));                    % roll
    Ry = deg2rad(10) + deg2rad(2.0)*cos(ws*(t - tC + 0.03)); % 10 deg forward lean + ripple
    Rz = deg2rad(8.0)*cos(wst*(t - tOn));                   % yaw, turned left when the right foot is forward
    trajectory = [t, x, y, z, Rx, Ry, Rz];
end
