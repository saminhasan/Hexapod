function trajectory = genPoseSine(n)
    % plain sine torso motion, 1.5 Hz with z and pitch at twice that, n cycles: [t x y z Rx Ry Rz]
    f = 1.5;
    dt = 1e-3;
    t = (0:dt:n/f)';
    x = 0.05*sin(2*pi*f*t)*0.0;                 % fore-aft, switched off
    y = 0.02*sin(2*pi*f*t);                     % sway
    z = 0.04*sin(2*pi*2*f*t);                   % vertical
    Rx = deg2rad(3.0)*sin(2*pi*f*t);            % roll
    Ry = deg2rad(2.5)*sin(2*pi*2*f*t);          % pitch
    Rz = deg2rad(15.0)*sin(2*pi*f*t);           % yaw
    trajectory = [t, x, y, z, Rx, Ry, Rz];
end
