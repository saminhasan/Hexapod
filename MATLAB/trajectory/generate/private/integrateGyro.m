function R = integrateGyro(w, fs)
    % body rates w (N x 3, rad/s, at fs Hz) -> rotations body -> world (3 x 3 x N), starting at identity;
    % quaternion stepped with the mid-sample rate
    n = size(w, 1);
    Q = zeros(n, 4);
    q = [1 0 0 0];
    Q(1,:) = q;
    for k = 2:n
        v = (w(k-1,:) + w(k,:))/(2*fs);
        th = norm(v);
        if th > 0
            dq = [cos(th/2), sin(th/2)*v/th];
        else
            dq = [1 0 0 0];
        end
        q = [q(1)*dq(1) - q(2:4)*dq(2:4).', q(1)*dq(2:4) + dq(1)*q(2:4) + cross(q(2:4), dq(2:4))];
        q = q/norm(q);
        Q(k,:) = q;
    end
    R = quat2rotm(Q);
end
