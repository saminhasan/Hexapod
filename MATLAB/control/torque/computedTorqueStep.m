function [tau, tauMotor, M, bias] = computedTorqueStep(dyn, x, q, qd, qRef, qdRef, qddRef, wn, zeta)
    % computed torque at one sample, tau = M*(qddRef + 2*zeta*wn*(qdRef - qd) + wn^2*(qRef - q)) + bias,
    % all 6x1 crank side, not saturated; tauMotor = tau/gearRatio. dyn: inverseDynamicsModel
    %#codegen
    [M, bias] = inverseDynamicsStep(dyn, x, q, qd);
    assert(isscalar(wn) && isscalar(zeta) && wn > 0 && zeta > 0);
    qddCommand = qddRef(:) + 2*zeta*wn*(qdRef(:) - qd(:)) + wn^2*(qRef(:) - q(:));
    tau = M*qddCommand + bias;
    tauMotor = tau/dyn.gearRatio;
end
