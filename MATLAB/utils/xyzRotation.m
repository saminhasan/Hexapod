function [R, E, Ea, Eb] = xyzRotation(a)
    % R = Rx(a1)*Ry(a2)*Rz(a3), same as eul2rotm(a,'XYZ'); E maps Euler rates to angular velocity,
    % Ea and Eb are its derivatives w.r.t. a1 and a2
    %#codegen
    ca = cos(a(1)); sa = sin(a(1)); cb = cos(a(2)); sb = sin(a(2)); cc = cos(a(3)); sc = sin(a(3));
    R = [1 0 0; 0 ca -sa; 0 sa ca] * [cb 0 sb; 0 1 0; -sb 0 cb] * [cc -sc 0; sc cc 0; 0 0 1];
    E = [1 0 sb; 0 ca -sa*cb; 0 sa ca*cb];
    Ea = [0 0 0; 0 -sa -ca*cb; 0 ca -sa*cb];
    Eb = [0 0 cb; 0 0 sa*sb; 0 0 -ca*sb];
end
