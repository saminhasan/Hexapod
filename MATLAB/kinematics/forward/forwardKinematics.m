function [x, info] = forwardKinematics(robot, q, x0, tol, maxIter)
    % platform poses (N x 6) for N rows of crank angles, solved in order, each from the previous answer, so
    % a sequence never jumps to another FK branch; x0 is the known starting pose (zeros(1, 6) = home).
    % info: per row iterations, final residual (rad), reciprocal condition of the Jacobian
    if nargin < 4 || isempty(tol), tol = 1e-12; end
    if nargin < 5 || isempty(maxIter), maxIter = 20; end
    N = size(q, 1);
    x = zeros(N, 6);
    info = struct('iter', zeros(N, 1), 'residual', zeros(N, 1), 'rcond', zeros(N, 1));
    xk = x0(:);
    for k = 1:N
        [xk, info.iter(k), info.residual(k), ok] = forwardKinematicsStep(robot.geom, q(k,:), xk, tol, maxIter);
        if ~ok, error('forwardKinematics:Unreachable', 'pose left the workspace at row %d', k); end
        x(k,:) = xk.';
        if nargout > 1
            info.rcond(k) = rcond(kinematicJacobian(robot.geom, xk));
        end
    end
end
