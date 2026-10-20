function [M, bias, diagnostics] = inverseDynamicsStep(dyn, x, q, qd)
    % crank-space dynamics at the current state, tau = M*qdd + bias (crank side). x: platform pose (6x1,
    % relative to home); q, qd: crank angles and rates (6x1), same mechanism as x. M (6x6) includes the
    % reflected rotors; bias is the torque at zero crank acceleration (velocity terms, gravity, viscous
    % friction). dyn: inverseDynamicsModel
    %#codegen
    x = x(:); q = q(:); qd = qd(:);
    assert(numel(x) == 6 && numel(q) == 6 && numel(qd) == 6);
    assert(all(isfinite(x)) && all(isfinite(q)) && all(isfinite(qd)));
    geom = dyn.geom;
    R = xyzRotation(x(4:6));
    origin = x(1:3) + geom.home;
    rho = (R*geom.platformPoints.').';
    ez = repmat([0 0 1], 6, 1);
    h = geom.r*(cos(q).*geom.crankDirections + sin(q).*ez);
    hq = geom.r*(-sin(q).*geom.crankDirections + cos(q).*ez);
    rod = origin.' + rho - geom.base - h;
    lengths = sqrt(sum(rod.^2, 2));
    assert(all(lengths > 0));
    u = rod./lengths;
    A = [u, cross(rho, u, 2)];
    D = sum(u.*hq, 2);
    diagnostics.closureResidual = max(abs(lengths - geom.rodLength));
    diagnostics.reciprocalCondition = rcond(A);
    assert(diagnostics.reciprocalCondition > 1e-10 && all(abs(D) > 1e-9*geom.r));

    % crank rates -> platform [origin velocity; angular velocity]
    W = A\diag(D);
    twist = W*qd;
    omega = twist(4:6);
    rc = R*dyn.upperCOM;
    Iworld = R*dyn.upperInertia*R.';
    Jc = [eye(3), -skew3(rc)];
    Jw = [zeros(3), eye(3)];
    Mspatial = dyn.upperMass*(Jc.'*Jc) + Jw.'*Iworld*Jw;
    M = diag(dyn.jointJ) + W.'*Mspatial*W;
    M = (M + M.')/2;

    % at zero crank acceleration the platform still accelerates (moving constraints), keep that part
    omegaRows = repmat(omega.', 6, 1);
    pointVelocity = twist(1:3).' + cross(omegaRows, rho, 2);
    relativeVelocity = pointVelocity - hq.*qd;
    centripetal = cross(omegaRows, cross(omegaRows, rho, 2), 2);
    gamma = sum(u.*centripetal, 2) + sum(u.*h, 2).*qd.^2 + sum(relativeVelocity.^2, 2)./lengths;
    twistAccBias = -(A\gamma);
    alphaBias = twistAccBias(4:6);
    comAccBias = twistAccBias(1:3) + cross(alphaBias, rc) + cross(omega, cross(omega, rc));
    force = dyn.upperMass*(comAccBias - dyn.gravity);
    moment = Iworld*alphaBias + cross(omega, Iworld*omega) + cross(rc, force);
    rodForces = A.'\[force; moment];
    bias = D.*rodForces - dyn.gravityMassRadius.*(hq*dyn.gravity) + dyn.viscousFriction.*qd;
end
