function [R, d, c] = fitImuMount(L)
    % real imu mount relative to the sim (CAD) imu, from one lined-up run (L from lineUpImu)
    % gyro does not care where on a rigid body it sits, only how it is turned:
    %   w_cad = R * w_meas                                   -> R, real imu axes in CAD imu axes (Kabsch)
    % accel then differs by the lever arm d (m, CAD imu axes) plus a constant bias c (m/s^2):
    %   R * f_meas - f_cad = alpha x d + w x (w x d) + c      -> linear in [d; c]
    fs = 1000;
    wc = deg2rad(L.sim(:,4:6));
    wm = deg2rad(L.meas(:,4:6));
    [U, ~, V] = svd((wm - mean(wm)).' * (wc - mean(wc)));
    R = V * diag([1 1 det(V * U.')]) * U.';

    a = gradient(wc.', 1/fs).'; % angular acceleration from the sim rate, smooth enough to differentiate
    w = wc;
    e = (R * L.meas(:,1:3).').' - L.sim(:,1:3);
    o = ones(size(w,1), 1); z = 0 * o;
    A = [-(w(:,2).^2 + w(:,3).^2),  -a(:,3) + w(:,1).*w(:,2),   a(:,2) + w(:,1).*w(:,3),  o, z, z;
          a(:,3) + w(:,1).*w(:,2),  -(w(:,1).^2 + w(:,3).^2),  -a(:,1) + w(:,2).*w(:,3),  z, o, z;
         -a(:,2) + w(:,1).*w(:,3),   a(:,1) + w(:,2).*w(:,3),  -(w(:,1).^2 + w(:,2).^2),  z, z, o];
    x = A \ e(:);
    d = x(1:3);
    c = x(4:6);
end
