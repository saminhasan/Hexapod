function tc = torqueControlSetup(robot, ref, wn, zeta, body)
    % constants for SP_TC, each motor on its own: tau = tauFF + Kp*(qRef - q) + Kd*(qdRef - qd), no pose needed.
    % tauFF: inverse dynamics of the reference, what would be uploaded with each row. Kp, Kd: each motor's
    % own inertia at home times wn^2 and 2*zeta*wn. body: mass model the controller assumes (default SP's own)
    if nargin < 5
        bodies = bodyModels(robot);
        body = bodies(3);
    end
    J = diag(inverseDynamicsStep(inverseDynamicsModel(robot, body), zeros(6, 1), zeros(6, 1), zeros(6, 1))).';
    id = inverseDynamics(robot, body, ref.t, ref.x);
    tc.tauFF = id.tauNewtonEuler;
    tc.Kp = J*wn^2;
    tc.Kd = J*2*zeta*wn;
end
