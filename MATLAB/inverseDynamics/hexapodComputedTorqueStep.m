function [tauCrank, tauMotor, Mtheta, biasTheta] = hexapodComputedTorqueStep( ...
    pose, theta, thetaDot, thetaRef, thetaDotRef, thetaDDotRef, model, wn, zeta)
%HEXAPODCOMPUTEDTORQUESTEP Complete model calculation and torque at one sample.
%#codegen
% Inputs: actual pose/angles/speeds and desired crank trajectory, all 6x1.
% model comes from hexapodDynamicsSetup; wn [rad/s] and zeta are scalars.
% Outputs are UNSATURATED torque demands; tauMotor assumes ideal gearing.
% Current-loop dynamics and actual actuator limits belong in the plant.
% For the position-only command response, pass zero reference derivatives.
[Mtheta,biasTheta] = hexapodDynamicsStep(pose,theta,thetaDot,model);
assert(isscalar(wn) && isscalar(zeta) && wn>0 && zeta>0);
accelCommand = thetaDDotRef(:) + 2*zeta*wn*(thetaDotRef(:)-thetaDot(:)) + wn^2*(thetaRef(:)-theta(:));
tauCrank = Mtheta*accelCommand+biasTheta;
tauMotor = tauCrank/model.gearRatio;
end
