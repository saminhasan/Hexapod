function [Mtheta, biasTheta, diagnostics] = hexapodDynamicsStep( ...
    pose, theta, thetaDot, model)
    %HEXAPODDYNAMICSSTEP Full crank inertia and bias torque at ONE current state.
    %#codegen
    % [Mtheta,biasTheta] = hexapodDynamicsStep(pose,theta,thetaDot,model)
    %   tauCrank = Mtheta*thetaDDot + biasTheta
    %
    % pose: 6x1 [x;y;z;a;b;c], translation RELATIVE TO HOME [m], XYZ Euler
    %       angles [rad], R=Rx(a)*Ry(b)*Rz(c), platform-to-world rotation.
    % theta, thetaDot: 6x1 actual crank angles [rad] and speeds [rad/s], with
    %       the signs/order used by StuartPlatform.move. They must describe
    %       the same closed mechanism as pose. No acceleration input is needed.
    % model: numeric constants returned by hexapodDynamicsSetup, once at setup.
    % Mtheta: 6x6 [kg*m^2], FULL inertia, already including reflected rotors.
    % biasTheta: 6x1 [N*m], torque at ZERO CRANK ACCELERATION but current speed;
    %       includes velocity effects, gravity, and specified viscous friction.
    % diagnostics: closure residual [m] and reciprocal condition number.
    %
    % Evaluate at measured state at each controller sample. World/platform pose
    % is required (sensor or forward kinematics); no ambiguous FK is hidden here.
    % No finite differences, stored history, simulation outputs, or torque
    % measurements are used. The mechanics match inverseDynamics.m.

    pose = pose(:); theta = theta(:); thetaDot = thetaDot(:);
    assert(numel(pose)==6 && numel(theta)==6 && numel(thetaDot)==6);
    assert(all(isfinite(pose)) && all(isfinite(theta)) && all(isfinite(thetaDot)));
    ca=cos(pose(4)); sa=sin(pose(4));
    cb=cos(pose(5)); sb=sin(pose(5));
    cc=cos(pose(6)); sc=sin(pose(6));
    R = [1 0 0;0 ca -sa;0 sa ca]*[cb 0 sb;0 1 0;-sb 0 cb] ...
        *[cc -sc 0;sc cc 0;0 0 1];
    origin = pose(1:3)+model.home;
    rho = (R*model.platformPoints.').';
    ez = repmat([0 0 1],6,1);
    h = model.r*(cos(theta).*model.crankDirections + sin(theta).*ez);
    ht = model.r*(-sin(theta).*model.crankDirections + cos(theta).*ez);
    rod = origin.'+rho-model.base-h;
    lengths = sqrt(sum(rod.^2,2));
    assert(all(lengths>0));
    u = rod./lengths;
    A = [u,cross(rho,u,2)];
    D = sum(u.*ht,2);
    diagnostics.closureResidual = max(abs(lengths-model.rodLength));
    diagnostics.reciprocalCondition = rcond(A);
    assert(diagnostics.reciprocalCondition>1e-10 && all(abs(D)>1e-9*model.r));

    % Map crank speeds to world [platform-origin velocity; angular velocity].
    W = A\diag(D);
    twist = W*thetaDot;
    omega = twist(4:6);
    rc = R*model.upperCOM;
    Iworld = R*model.upperInertia*R.';
    Jc = [eye(3),-skew3(rc)];
    Jw = [zeros(3),eye(3)];
    Mspatial = model.upperMass*(Jc.'*Jc)+Jw.'*Iworld*Jw;
    Mtheta = diag(model.jointJ)+W.'*Mspatial*W;
    Mtheta = (Mtheta+Mtheta.')/2;

    % With thetaDDot=0, the moving constraints still require platform
    % acceleration. Retain it: merely setting platform acceleration to zero
    % would give the wrong velocity-dependent bias torque.
    omegaRows = repmat(omega.',6,1);
    pointVelocity = twist(1:3).'+cross(omegaRows,rho,2);
    relativeVelocity = pointVelocity-ht.*thetaDot;
    centripetal = cross(omegaRows,cross(omegaRows,rho,2),2);
    gamma = sum(u.*centripetal,2)+sum(u.*h,2).*thetaDot.^2 ...
        +sum(relativeVelocity.^2,2)./lengths;
    twistAccBias = -(A\gamma);
    alphaBias = twistAccBias(4:6);
    comAccBias = twistAccBias(1:3)+cross(alphaBias,rc) ...
        +cross(omega,cross(omega,rc));
    force = model.upperMass*(comAccBias-model.gravity);
    moment = Iworld*alphaBias+cross(omega,Iworld*omega)+cross(rc,force);
    rodForces = A.'\[force;moment];
    biasTheta = D.*rodForces ...
        -model.gravityMassRadius.*(ht*model.gravity) ...
        +model.viscousFriction.*thetaDot;
end

function S = skew3(v)
    S = [0 -v(3) v(2);v(3) 0 -v(1);-v(2) v(1) 0];
end
