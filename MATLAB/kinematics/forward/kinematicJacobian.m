function J = kinematicJacobian(geom, x)
    % dq/dx at pose x (6x6), closed form from differentiating the rod closure (see deriveForwardKinematics):
    % row i = [u_i', (rho_i x u_i)'*E] / (u_i . dh_i/dq_i), u_i the rod vector, rho_i the platform joint
    % offset in world axes, E the Euler-rate to angular-velocity map. geom: robot.geom
    %#codegen
    q = inverseKinematicsStep(geom, x);
    [R, E] = xyzRotation(x(4:6));
    rho = (R*geom.platformPoints.').';
    ez = repmat([0 0 1], 6, 1);
    h = geom.r*(cos(q).*geom.crankDirections + sin(q).*ez);
    hq = geom.r*(-sin(q).*geom.crankDirections + cos(q).*ez);
    u = (x(1:3) + geom.home).' + rho - geom.base - h;
    J = [u, cross(rho, u, 2)*E]./sum(u.*hq, 2);
end
