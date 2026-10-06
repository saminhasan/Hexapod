function result = inverseDynamics(trajectory, sp, profile, kinematics)
%INVERSEDYNAMICS Ballpark rotary Stewart-platform inverse dynamics.
% result = inverseDynamics(trajectory, sp, profile)
% result = inverseDynamics(trajectory, sp, profile, kinematics)
% trajectory is N-by-7 [time x y z Xangle Yangle Zangle], metres/radians,
% in the same centred platform frame used by StuartPlatform.move. Rotations
% follow eul2rotm(angles,'XYZ'). Times must be strictly increasing (N >= 5).
%
% Two independent calculations use the same physical approximation:
%   Newton-Euler: rigid-body force/moment balance and rod force mapping.
%   Lagrange: kinetic-energy mass matrix, its numerical pose derivatives,
%             potential energy, and six rod-length constraints.
% Each rod is split equally into point masses at its two ends, as in the
% paper. The upper halves contribute BOTH mass and rotational inertia to
% the platform. The lower halves move with their crank tips. Distributed
% rod dynamics, joint friction other than specified viscosity, flexibility,
% gearbox losses and controller transients are outside this approximation.
%
% Required profile fields (SI units):
%   platformMass, platformCOM(3), platformInertia(3x3, ABOUT ITS OWN COM)
%   payloadMass, payloadCOM(3), payloadInertia(3x3, ABOUT ITS OWN COM)
%   rodMass, armMass, armCOM, armInertia, rotorInertia, gravity(3)
% COM vectors and inertia axes are expressed in the platform body frame.
% armCOM is distance along the crank from its pivot; armInertia is about
% the crank's own COM, parallel to its rotation axis. rotorInertia is
% ALREADY REFLECTED TO THE CRANK (e.g. Jm*N^2); do not reflect it twice.
% Limb quantities may be scalar or six-vectors. Optional viscousFriction
% defaults to zero; optional gearRatio defaults to 9. Motor torques are
% ideal crankTorque/gearRatio, with no efficiency correction.
%
% Numerical derivatives are nonperiodic: five-point interior stencils and
% at least second-order boundary stencils. Angular coordinates are unwrapped
% before differentiation. Sampled motion must resolve the commanded motion;
% raw measured/noisy trajectories need filtering before calling this function.
% Optional kinematics replaces numerical differentiation. Its N-by-3 fields
% linearVelocity, linearAcceleration, angularVelocity, angularAcceleration
% are WORLD/base-frame vectors in SI units at the supplied trajectory times.
% The linear fields describe the same platform origin as trajectory(:,2:4).

validateattributes(trajectory, {'numeric'}, ...
    {'real','finite','2d','ncols',7}, mfilename, 'trajectory');
if size(trajectory,1) < 5 || any(diff(trajectory(:,1)) <= 0)
    error('inverseDynamics:Time', ...
        'Provide at least five samples with strictly increasing times.');
end
validateGeometry(sp);
p = validateProfile(profile);
[upperMass, upperCOM, upperInertia] = upperBody(sp, p);

t = trajectory(:,1);
q = trajectory(:,2:7);
q(:,4:6) = unwrap(q(:,4:6), [], 1);
useKinematics = nargin >= 4 && ~isempty(kinematics);
if useKinematics
    validateKinematics(kinematics,numel(t));
    qd = zeros(size(q)); qdd = qd;
else
    [qd, qdd] = sampledDerivatives(t, q);
end
jointTrajectory = sp.move([t q]);
theta = jointTrajectory(:,2:7);
count = numel(t);

tauNE = zeros(count,6); tauL = tauNE;
thetaD = tauNE; thetaDD = tauNE;
forcesNE = tauNE; forcesL = tauNE;
omegaLog = zeros(count,3); alphaLog = omegaLog;
reciprocalCondition = zeros(count,1);
closureResidual = zeros(count,1);
energyK = zeros(count,1); energyV = energyK;

e = [cos(sp.betaB(:)), sin(sp.betaB(:)), zeros(6,1)];
ez = repmat([0 0 1],6,1);
armJ = p.armInertia + p.armMass .* p.armCOM.^2;
jointJ = p.rotorInertia + armJ + (p.rodMass/2) * sp.r^2;
gravityMassRadius = p.armMass .* p.armCOM/sp.r + p.rodMass/2;
angleStep = 1e-5;

for k = 1:count
    angles = q(k,4:6).';
    [R, E, Ea, Eb] = xyzRotation(angles);
    if abs(cos(angles(2))) < 1e-7
        error('inverseDynamics:EulerSingular', ...
            'XYZ Euler coordinate singularity at sample %d.',k);
    end
    if useKinematics
        av = E \ kinematics.angularVelocity(k,:).';
        qd(k,:) = [kinematics.linearVelocity(k,:), av.'];
        angularDD = E \ (kinematics.angularAcceleration(k,:).' ...
            -(Ea*av(1)+Eb*av(2))*av);
        qdd(k,:) = [kinematics.linearAcceleration(k,:), angularDD.'];
    end
    av = qd(k,4:6).';
    omega = E*av;
    alpha = E*qdd(k,4:6).' + (Ea*av(1) + Eb*av(2))*av;
    omegaLog(k,:) = omega.';
    alphaLog(k,:) = alpha.';
    origin = q(k,1:3).' + sp.homez(:);
    rho = (R*sp.Pp.').';
    h = sp.r*(cos(theta(k,:).').*e + sin(theta(k,:).').*ez);
    ht = sp.r*(-sin(theta(k,:).').*e + cos(theta(k,:).').*ez);
    rod = origin.' + rho - sp.B - h;
    lengths = vecnorm(rod,2,2);
    u = rod ./ lengths;
    A = [u, cross(rho,u,2)];
    D = sum(u.*ht,2);
    reciprocalCondition(k) = rcond(A);
    closureResidual(k) = max(abs(lengths-sp.d));
    if reciprocalCondition(k) < 1e-10 || any(abs(D) < 1e-9*sp.r)
        error('inverseDynamics:Singular', ...
            'Near-singular platform/crank geometry at sample %d (t=%g).', k,t(k));
    end

    % Differentiate the exact rod-length constraint, not the IK samples.
    vp = qd(k,1:3) + cross(repmat(omega.',6,1),rho,2);
    ap = qdd(k,1:3) + cross(repmat(alpha.',6,1),rho,2) ...
        + cross(repmat(omega.',6,1), ...
                cross(repmat(omega.',6,1),rho,2),2);
    td = sum(u.*vp,2)./D;
    relativeVelocity = vp-ht.*td;
    tdd = (sum(u.*ap,2) + sum(u.*h,2).*td.^2 ...
        + sum(relativeVelocity.^2,2)./lengths)./D;
    thetaD(k,:) = td.';
    thetaDD(k,:) = tdd.';

    % NEWTON-EULER: force and moment about the platform reference origin.
    rc = R*upperCOM;
    Iworld = R*upperInertia*R.';
    acom = qdd(k,1:3).' + cross(alpha,rc) + cross(omega,cross(omega,rc));
    force = upperMass*(acom-p.gravity);
    moment = Iworld*alpha + cross(omega,Iworld*omega) + cross(rc,force);
    fNE = A.' \ [force;moment];

    % LAGRANGE: Euler-Lagrange equations of the upper-body kinetic energy.
    % The translational derivatives of M are zero. Finite-differencing the
    % three Euler coordinates avoids a Symbolic Math Toolbox dependency.
    Mq = energyMassMatrix(angles,upperMass,upperCOM,upperInertia);
    Mdot = zeros(6); christoffel = zeros(6,1);
    gradV = [-upperMass*p.gravity; zeros(3,1)];
    for j = 1:3
        plus = angles; minus = angles;
        plus(j) = plus(j)+angleStep;
        minus(j) = minus(j)-angleStep;
        [Mp,Rp] = energyMassMatrix(plus,upperMass,upperCOM,upperInertia);
        [Mm,Rm] = energyMassMatrix(minus,upperMass,upperCOM,upperInertia);
        dM = (Mp-Mm)/(2*angleStep);
        Mdot = Mdot + dM*av(j);
        christoffel(j+3) = qd(k,:)*dM*qd(k,:).';
        % Subtract only orientation-dependent potential, avoiding loss of
        % precision from a large constant absolute platform height.
        gradV(j+3) = -upperMass*p.gravity.'*(Rp-Rm)*upperCOM/(2*angleStep);
    end
    Q = Mq*qdd(k,:).' + Mdot*qd(k,:).' - christoffel/2 + gradV;
    K = A*blkdiag(eye(3),E);
    fL = K.' \ Q;

    % Crank Lagrange equation (fixed rotation axis, constant inertia).
    gravityTorque = -gravityMassRadius .* (ht*p.gravity);
    crankEL = jointJ.*tdd + gravityTorque + p.viscousFriction.*td;

    % Independently form the Newton-Euler crank moment: COM and lower rod
    % endpoint inertia/gravity forces projected on the motor's fixed axis.
    axis = cross(e,ez,2);
    armPosition = h.*(p.armCOM/sp.r);
    armAcceleration = (ht.*tdd-h.*td.^2).*(p.armCOM/sp.r);
    tipAcceleration = ht.*tdd-h.*td.^2;
    armForce = p.armMass.*(armAcceleration-p.gravity.');
    lowerRodForce = (p.rodMass/2).*(tipAcceleration-p.gravity.');
    crankNE = sum(axis.*(cross(armPosition,armForce,2) ...
        + cross(h,lowerRodForce,2)),2) ...
        + (p.armInertia+p.rotorInertia).*tdd + p.viscousFriction.*td;

    tauNE(k,:) = (D.*fNE+crankNE).';
    tauL(k,:) = (D.*fL+crankEL).';
    forcesNE(k,:) = fNE.'; forcesL(k,:) = fL.';
    energyK(k) = 0.5*qd(k,:)*Mq*qd(k,:).' + 0.5*sum(jointJ.*td.^2);
    energyV(k) = -upperMass*p.gravity.'*(origin+rc) ...
        -sum((p.armMass.*p.armCOM/sp.r+p.rodMass/2).*(h*p.gravity));
end

result.time = t;
result.torqueLagrange = tauL;
result.torqueNewtonEuler = tauNE;
result.motorTorqueLagrange = tauL/p.gearRatio;
result.motorTorqueNewtonEuler = tauNE/p.gearRatio;
result.jointAngles = theta;
result.jointVelocity = thetaD;
result.jointAcceleration = thetaDD;
result.poseVelocity = qd;
result.poseAcceleration = qdd;
result.angularVelocity = omegaLog;
result.angularAcceleration = alphaLog;
result.rodForceLagrange = forcesL;
result.rodForceNewtonEuler = forcesNE;
result.kineticEnergy = energyK;
result.potentialEnergy = energyV;
result.reciprocalCondition = reciprocalCondition;
result.closureResidual = closureResidual;
result.upperBody = struct('mass',upperMass,'COM',upperCOM,'inertia',upperInertia);
result.profile = p;
if useKinematics
    result.derivativeSource = 'Supplied world-frame kinematics';
else
    result.derivativeSource = 'Nonperiodic numerical trajectory derivatives';
end
result.maxMethodDifference = max(abs(tauL-tauNE),[],'all');
result.torqueFrame = 'Crank/output shaft, N m; positive increases StuartPlatform angle';
end

function validateKinematics(kinematics,count)
if ~isstruct(kinematics) || ~isscalar(kinematics)
    error('inverseDynamics:Kinematics','kinematics must be a scalar struct.');
end
names = {'linearVelocity','linearAcceleration','angularVelocity','angularAcceleration'};
for k = 1:numel(names)
    field = names{k};
    if ~isfield(kinematics,field)
        error('inverseDynamics:Kinematics','Missing kinematics.%s.',field);
    end
    value = kinematics.(field);
    if ~isnumeric(value) || ~isreal(value) || any(~isfinite(value),'all') ...
            || ~isequal(size(value),[count 3])
        error('inverseDynamics:Kinematics', ...
            'kinematics.%s must be a finite real N-by-3 array.',field);
    end
end
end

function [M,R] = energyMassMatrix(angles,m,c,I)
[R,E] = xyzRotation(angles);
rho = R*c;
Jc = [eye(3),-skew(rho)*E];
Jw = [zeros(3),E];
M = m*(Jc.'*Jc) + Jw.'*(R*I*R.')*Jw;
end

function [R,E,Ea,Eb] = xyzRotation(a)
ca=cos(a(1)); sa=sin(a(1)); cb=cos(a(2)); sb=sin(a(2));
cc=cos(a(3)); sc=sin(a(3));
R = [1 0 0;0 ca -sa;0 sa ca] ...
    * [cb 0 sb;0 1 0;-sb 0 cb] * [cc -sc 0;sc cc 0;0 0 1];
E = [1 0 sb;0 ca -sa*cb;0 sa ca*cb];
if nargout > 2
    Ea = [0 0 0;0 -sa -ca*cb;0 ca -sa*cb];
    Eb = [0 0 cb;0 0 sa*sb;0 0 -ca*sb];
end
end

function S = skew(v)
S = [0 -v(3) v(2);v(3) 0 -v(1);-v(2) v(1) 0];
end

function [m,c,I] = upperBody(sp,p)
masses = [p.platformMass;p.payloadMass;p.rodMass/2];
centres = [p.platformCOM,p.payloadCOM,sp.Pp.'];
m = sum(masses);
if m > 0
    c = centres*masses/m;
else
    c = zeros(3,1);
end
I = p.platformInertia+p.payloadInertia;
for i = 1:8
    d = centres(:,i)-c;
    I = I+masses(i)*(dot(d,d)*eye(3)-d*d.');
end
end

function [dy,ddy] = sampledDerivatives(t,y)
n = numel(t); dy = zeros(size(y)); ddy = dy;
h = mean(diff(t));
if max(abs(diff(t)/h-1)) < 1e-7
    dy(3:n-2,:) = (y(1:n-4,:)-8*y(2:n-3,:)+8*y(4:n-1,:)-y(5:n,:))/(12*h);
    ddy(3:n-2,:) = (-y(1:n-4,:)+16*y(2:n-3,:)-30*y(3:n-2,:) ...
        +16*y(4:n-1,:)-y(5:n,:))/(12*h^2);
    dy(1,:) = (-3*y(1,:)+4*y(2,:)-y(3,:))/(2*h);
    dy(2,:) = (y(3,:)-y(1,:))/(2*h);
    dy(n-1,:) = (y(n,:)-y(n-2,:))/(2*h);
    dy(n,:) = (3*y(n,:)-4*y(n-1,:)+y(n-2,:))/(2*h);
    ddy(1,:) = (2*y(1,:)-5*y(2,:)+4*y(3,:)-y(4,:))/h^2;
    ddy(2,:) = (y(1,:)-2*y(2,:)+y(3,:))/h^2;
    ddy(n-1,:) = (y(n-2,:)-2*y(n-1,:)+y(n,:))/h^2;
    ddy(n,:) = (2*y(n,:)-5*y(n-1,:)+4*y(n-2,:)-y(n-3,:))/h^2;
else
    % Polynomial stencils also support nonuniform sample times. Scaling the
    % local time offsets avoids poorly conditioned powers of small dt.
    for k = 1:n
        if k <= 2
            idx = 1:4;
        elseif k >= n-1
            idx = n-3:n;
        else
            idx = k-2:k+2;
        end
        offsets = t(idx)-t(k); scale = max(abs(offsets));
        z = offsets/scale;
        V = (z(:).').^((0:numel(idx)-1).');
        rhs = zeros(numel(idx),2); rhs(2,1)=1; rhs(3,2)=2;
        weights = V\rhs;
        dy(k,:) = weights(:,1).'*y(idx,:)/scale;
        ddy(k,:) = weights(:,2).'*y(idx,:)/scale^2;
    end
end
end

function p = validateProfile(p)
if ~isstruct(p) || ~isscalar(p)
    error('inverseDynamics:Profile','profile must be a scalar struct.');
end
names = {'platformMass','platformCOM','platformInertia','payloadMass', ...
    'payloadCOM','payloadInertia','rodMass','armMass','armCOM','armInertia', ...
    'rotorInertia','gravity'};
for k = 1:numel(names)
    if ~isfield(p,names{k})
        error('inverseDynamics:Profile','Missing profile.%s.',names{k});
    end
end
validateattributes(p.platformMass,{'numeric'},{'real','finite','scalar','nonnegative'});
validateattributes(p.payloadMass,{'numeric'},{'real','finite','scalar','nonnegative'});
for name = {'platformCOM','payloadCOM','gravity'}
    field = name{1};
    validateattributes(p.(field),{'numeric'},{'real','finite','vector','numel',3});
    p.(field) = p.(field)(:);
end
for name = {'platformInertia','payloadInertia'}
    field=name{1}; value=p.(field);
    validateattributes(value,{'numeric'},{'real','finite','size',[3 3]});
    if norm(value-value.','fro') > 1e-10*max(1,norm(value,'fro')) ...
            || min(eig((value+value.')/2)) < -1e-12
        error('inverseDynamics:Inertia','profile.%s must be symmetric positive semidefinite.',field);
    end
end
if ~isfield(p,'viscousFriction'), p.viscousFriction=0; end
if ~isfield(p,'gearRatio'), p.gearRatio=9; end
for name = {'rodMass','armMass','armCOM','armInertia','rotorInertia','viscousFriction'}
    field = name{1}; value = p.(field);
    validateattributes(value,{'numeric'},{'real','finite','vector','nonnegative'});
    if isscalar(value)
        p.(field) = repmat(value,6,1);
    elseif numel(value) == 6
        p.(field) = value(:);
    else
        error('inverseDynamics:Profile','profile.%s must be scalar or six elements.',field);
    end
end
validateattributes(p.gearRatio,{'numeric'},{'real','finite','scalar','positive'});
end

function validateGeometry(sp)
names = {'r','d','B','Pp','betaB','homez','move'};
for k = 1:numel(names)
    if ~isfield(sp,names{k})
        error('inverseDynamics:Geometry','Missing sp.%s; use StuartPlatform.',names{k});
    end
end
validateattributes(sp.r,{'numeric'},{'real','finite','scalar','positive'});
validateattributes(sp.d,{'numeric'},{'real','finite','scalar','positive'});
validateattributes(sp.B,{'numeric'},{'real','finite','size',[6 3]});
validateattributes(sp.Pp,{'numeric'},{'real','finite','size',[6 3]});
validateattributes(sp.betaB,{'numeric'},{'real','finite','vector','numel',6});
validateattributes(sp.homez,{'numeric'},{'real','finite','vector','numel',3});
if ~isa(sp.move,'function_handle')
    error('inverseDynamics:Geometry','sp.move must be a function handle.');
end
end
