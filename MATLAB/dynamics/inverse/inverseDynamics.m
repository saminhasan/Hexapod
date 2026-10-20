function result = inverseDynamics(robot, body, t, x, kinematics)
    % crank torques for platform poses x (N x 6, relative to home, XYZ Euler) at times t, worked out twice,
    % Newton-Euler and Lagrange, on one model: each rod split into point masses at its ends (as in the paper),
    % upper halves ride with the platform, lower halves with the crank tips. body: one of bodyModels.
    % Pose rates are 5-point differences unless kinematics is given (world frame, N x 3: linearVelocity,
    % linearAcceleration, angularVelocity, angularAcceleration). Torques crank side, motor = crank/gearRatio
    t = t(:);
    validateattributes(x, {'numeric'}, {'real', 'finite', '2d', 'ncols', 6}, mfilename, 'x');
    if numel(t) < 5 || numel(t) ~= size(x, 1) || any(diff(t) <= 0)
        error('inverseDynamics:Time', 'need at least five rows with strictly increasing times');
    end
    geom = robot.geom;
    p = fillBody(body);
    [upperMass, upperCOM, upperInertia] = upperBody(geom, p);

    x(:,4:6) = unwrap(x(:,4:6), [], 1);
    count = numel(t);
    useKinematics = nargin >= 5 && ~isempty(kinematics);
    if useKinematics
        names = {'linearVelocity', 'linearAcceleration', 'angularVelocity', 'angularAcceleration'};
        valid = @(v) isnumeric(v) && isreal(v) && isequal(size(v), [count 3]) && all(isfinite(v), 'all');
        if ~isstruct(kinematics) || ~all(isfield(kinematics, names)) || ~all(cellfun(@(f) valid(kinematics.(f)), names))
            error('inverseDynamics:Kinematics', 'kinematics needs finite real N x 3 %s, %s, %s and %s', names{:});
        end
        xd = zeros(size(x)); xdd = xd;
    else
        [xd, xdd] = sampledDerivatives(t, x);
    end
    q = inverseKinematics(robot, x);

    tauNE = zeros(count, 6); tauL = tauNE;
    qdLog = tauNE; qddLog = tauNE;
    forcesNE = tauNE; forcesL = tauNE;
    omegaLog = zeros(count, 3); alphaLog = omegaLog;
    reciprocalCondition = zeros(count, 1);
    closureResidual = zeros(count, 1);
    energyK = zeros(count, 1); energyV = energyK;

    e = geom.crankDirections;
    ez = repmat([0 0 1], 6, 1);
    armJ = p.armInertia + p.armMass.*p.armCOM.^2;
    jointJ = p.reflectedInertia + armJ + (p.rodMass/2)*geom.r^2;
    gravityMassRadius = p.armMass.*p.armCOM/geom.r + p.rodMass/2;
    angleStep = 1e-5;

    for k = 1:count
        angles = x(k,4:6).';
        [R, E, Ea, Eb] = xyzRotation(angles);
        if abs(cos(angles(2))) < 1e-7
            error('inverseDynamics:EulerSingular', 'XYZ Euler singularity at row %d', k);
        end
        if useKinematics
            av = E\kinematics.angularVelocity(k,:).';
            xd(k,:) = [kinematics.linearVelocity(k,:), av.'];
            angularDD = E\(kinematics.angularAcceleration(k,:).' - (Ea*av(1) + Eb*av(2))*av);
            xdd(k,:) = [kinematics.linearAcceleration(k,:), angularDD.'];
        end
        av = xd(k,4:6).';
        omega = E*av;
        alpha = E*xdd(k,4:6).' + (Ea*av(1) + Eb*av(2))*av;
        omegaLog(k,:) = omega.';
        alphaLog(k,:) = alpha.';
        origin = x(k,1:3).' + geom.home(:);
        rho = (R*geom.platformPoints.').';
        h = geom.r*(cos(q(k,:).').*e + sin(q(k,:).').*ez);
        hq = geom.r*(-sin(q(k,:).').*e + cos(q(k,:).').*ez);
        rod = origin.' + rho - geom.base - h;
        lengths = vecnorm(rod, 2, 2);
        u = rod./lengths;
        A = [u, cross(rho, u, 2)];
        D = sum(u.*hq, 2);
        reciprocalCondition(k) = rcond(A);
        closureResidual(k) = max(abs(lengths - geom.rodLength));
        if reciprocalCondition(k) < 1e-10 || any(abs(D) < 1e-9*geom.r)
            error('inverseDynamics:Singular', 'near-singular platform/crank geometry at row %d (t = %g)', k, t(k));
        end

        % crank rates from the rod closure itself, not from differencing the IK
        vp = xd(k,1:3) + cross(repmat(omega.', 6, 1), rho, 2);
        ap = xdd(k,1:3) + cross(repmat(alpha.', 6, 1), rho, 2) ...
            + cross(repmat(omega.', 6, 1), cross(repmat(omega.', 6, 1), rho, 2), 2);
        qd = sum(u.*vp, 2)./D;
        relativeVelocity = vp - hq.*qd;
        qdd = (sum(u.*ap, 2) + sum(u.*h, 2).*qd.^2 + sum(relativeVelocity.^2, 2)./lengths)./D;
        qdLog(k,:) = qd.';
        qddLog(k,:) = qdd.';

        % Newton-Euler, force and moment about the platform origin
        rc = R*upperCOM;
        Iworld = R*upperInertia*R.';
        acom = xdd(k,1:3).' + cross(alpha, rc) + cross(omega, cross(omega, rc));
        force = upperMass*(acom - p.gravity);
        moment = Iworld*alpha + cross(omega, Iworld*omega) + cross(rc, force);
        fNE = A.'\[force; moment];

        % Lagrange on the upper body kinetic energy; M only depends on the angles, differenced numerically
        Mx = energyMassMatrix(angles, upperMass, upperCOM, upperInertia);
        Mdot = zeros(6); christoffel = zeros(6, 1);
        gradV = [-upperMass*p.gravity; zeros(3, 1)];
        for j = 1:3
            plus = angles; minus = angles;
            plus(j) = plus(j) + angleStep;
            minus(j) = minus(j) - angleStep;
            [Mp, Rp] = energyMassMatrix(plus, upperMass, upperCOM, upperInertia);
            [Mm, Rm] = energyMassMatrix(minus, upperMass, upperCOM, upperInertia);
            dM = (Mp - Mm)/(2*angleStep);
            Mdot = Mdot + dM*av(j);
            christoffel(j+3) = xd(k,:)*dM*xd(k,:).';
            gradV(j+3) = -upperMass*p.gravity.'*(Rp - Rm)*upperCOM/(2*angleStep); % orientation part only, keeps precision
        end
        Q = Mx*xdd(k,:).' + Mdot*xd(k,:).' - christoffel/2 + gradV;
        K = A*blkdiag(eye(3), E);
        fL = K.'\Q;

        % crank side: Lagrange (fixed axis, constant inertia) ...
        gravityTorque = -gravityMassRadius.*(hq*p.gravity);
        crankEL = jointJ.*qdd + gravityTorque + p.viscousFriction.*qd;
        % ... and Newton-Euler, arm COM and lower rod mass forces about the motor axis
        crankAxis = cross(e, ez, 2);
        armPosition = h.*(p.armCOM/geom.r);
        armAcceleration = (hq.*qdd - h.*qd.^2).*(p.armCOM/geom.r);
        tipAcceleration = hq.*qdd - h.*qd.^2;
        armForce = p.armMass.*(armAcceleration - p.gravity.');
        lowerRodForce = (p.rodMass/2).*(tipAcceleration - p.gravity.');
        crankNE = sum(crankAxis.*(cross(armPosition, armForce, 2) + cross(h, lowerRodForce, 2)), 2) ...
            + (p.armInertia + p.reflectedInertia).*qdd + p.viscousFriction.*qd;

        tauNE(k,:) = (D.*fNE + crankNE).';
        tauL(k,:) = (D.*fL + crankEL).';
        forcesNE(k,:) = fNE.'; forcesL(k,:) = fL.';
        energyK(k) = 0.5*xd(k,:)*Mx*xd(k,:).' + 0.5*sum(jointJ.*qd.^2);
        energyV(k) = -upperMass*p.gravity.'*(origin + rc) ...
            - sum((p.armMass.*p.armCOM/geom.r + p.rodMass/2).*(h*p.gravity));
    end

    result.t = t;
    result.tauNewtonEuler = tauNE;
    result.tauLagrange = tauL;
    result.motorTauNewtonEuler = tauNE/p.gearRatio;
    result.motorTauLagrange = tauL/p.gearRatio;
    result.q = q;
    result.qd = qdLog;
    result.qdd = qddLog;
    result.xd = xd;
    result.xdd = xdd;
    result.omega = omegaLog;
    result.alpha = alphaLog;
    result.rodForceNewtonEuler = forcesNE;
    result.rodForceLagrange = forcesL;
    result.kineticEnergy = energyK;
    result.potentialEnergy = energyV;
    result.reciprocalCondition = reciprocalCondition;
    result.closureResidual = closureResidual;
    result.upperBody = struct('mass', upperMass, 'COM', upperCOM, 'inertia', upperInertia);
    result.body = p;
    if useKinematics
        result.derivativeSource = 'supplied world-frame kinematics';
    else
        result.derivativeSource = 'nonperiodic numerical differences';
    end
    result.maxMethodDifference = max(abs(tauL - tauNE), [], 'all');
end
