function [q, ok] = inverseKinematics(robot, x)
    % crank angles (N x 6, rad) for N platform poses (N x 6, relative to home, XYZ Euler), row by row with
    % inverseKinematicsStep. ok (N x 1) flags the rows every leg can close; without it asked for, any row that
    % can't is counted and an error is raised
    q = zeros(size(x, 1), 6);
    ok = true(size(x, 1), 1);
    for k = 1:size(x, 1)
        [qk, ok(k)] = inverseKinematicsStep(robot.geom, x(k,:).');
        q(k,:) = qk.';
    end
    if nargout < 2 && ~all(ok)
        error('inverseKinematics:Unreachable', ...
            'pose outside the workspace on %d of %d rows (first row %d), reduce the amplitude', nnz(~ok), numel(ok), find(~ok, 1));
    end
end
