function [q, ok] = inverseKinematicsStep(geom, x)
    % crank angles q (6x1, rad) for one platform pose x = [x y z Rx Ry Rz] (6x1, relative to home, XYZ Euler).
    % Closed form of the rod closure, see deriveInverseKinematics; ok is false when a leg can't close
    % (that angle is clipped instead of going complex). geom: robot.geom
    %#codegen
    R = xyzRotation(x(4:6));
    l = (x(1:3) + geom.home).' + (R*geom.platformPoints.').' - geom.base; % crank pivot -> platform joint, 6x3
    E = 2*geom.r*l(:,3);
    F = 2*geom.r*(geom.crankDirections(:,1).*l(:,1) + geom.crankDirections(:,2).*l(:,2));
    G = sum(l.^2, 2) - (geom.rodLength^2 - geom.r^2);
    s = G./sqrt(E.^2 + F.^2);
    ok = all(abs(s) <= 1);
    q = asin(max(min(s, 1), -1)) - atan2(F, E);
end
