function ctc = computedTorqueSetup(robot, ref, wn, zeta, body)
    % constants for SP_CTC, full computed torque with the pose from forward kinematics:
    %   tau = M(q, x)*(qddRef + 2*zeta*wn*(qdRef - qd) + wn^2*(qRef - q)) + bias(q, qd, x)
    % xRef: commanded pose per row, where forward kinematics starts (always within mm of the real one, so
    % the block needs no memory). body: mass model the controller assumes (default SP's own)
    if nargin < 5
        bodies = bodyModels(robot);
        body = bodies(3);
    end
    ctc.dyn = inverseDynamicsModel(robot, body);
    ctc.xRef = ref.x;
    ctc.dt = ref.t(2) - ref.t(1);
    ctc.wn = wn;
    ctc.zeta = zeta;
end
