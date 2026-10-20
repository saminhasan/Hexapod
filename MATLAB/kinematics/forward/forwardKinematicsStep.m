function [x, iter, res, ok] = forwardKinematicsStep(geom, q, x, tol, maxIter)
    % platform pose from the 6 crank angles q: Newton on inverseKinematicsStep, started from a nearby pose x
    % (6x1, relative to home), one Jacobian per call unless progress stalls. res: final max angle error
    % (rad), ok: false if a leg left the workspace on the way. geom: robot.geom
    %#codegen
    if nargin < 4, tol = 1e-12; end
    if nargin < 5, maxIter = 8; end
    x = x(:);
    J = kinematicJacobian(geom, x);
    prev = inf;
    ok = true;
    res = inf;
    for iter = 1:maxIter
        [qx, okNow] = inverseKinematicsStep(geom, x);
        ok = ok && okNow;
        e = q(:) - qx;
        res = max(abs(e));
        if res < tol, break, end
        if res > 0.1*prev, J = kinematicJacobian(geom, x); end
        prev = res;
        x = x + J\e;
    end
end
