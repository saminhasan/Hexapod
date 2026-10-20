%[text] # Forward kinematics
%[text] Pose from the six crank angles. There is no closed form, so `forwardKinematicsStep` runs Newton on the inverse kinematics, $x\_{k+1}=x\_k+J^{-1}(q-q\_{IK}(x\_k))$, started from a pose known to be close (home at power-up, the last answer after that). Starting close is also what keeps it on the rig's assembly branch. The one thing to derive is $J=\\partial q/\\partial x$. It is checked here against `kinematicJacobian` and against differences of the IK.
here = fileparts(mfilename('fullpath'));
run(fullfile(fileparts(fileparts(here)), 'startup.m'));
robot = hexapodRobot();
geom = robot.geom;
%%
%[text] ## Differentiate the closure
%[text] Leg $i$ closes when $f\_i=\\|u\_i\\|^2-d^2=0$, with rod vector $u\_i=p+p\_0+\\rho\_i-B\_i-h\_i(q\_i)$ and $\\rho\_i=RP\_i$. Along any motion $\\dot f\_i=0$:
%[text]{"align":"center"} $u\_i^T(\\dot p+\\omega\\times\\rho\_i-h\_{q,i}\\dot q\_i)=0,\\qquad h\_{q,i}=\\partial h\_i/\\partial q\_i=r(-\\sin q\_i\\,e\_i+\\cos q\_i\\,e\_z)$
%[text] With $u^T(\\omega\\times\\rho)=(\\rho\\times u)^T\\omega$ and $\\omega=\\mathcal{E}(\\eta)\\dot\\eta$, row $i$ of the Jacobian is
%[text]{"align":"center"} $J\_{i,:}=\\frac{\[u\_i^T,\\ (\\rho\_i\\times u\_i)^T\\mathcal{E}\]}{u\_i^Th\_{q,i}},\\qquad \\mathcal{E}=\\left\[\\matrix{1&0&\\sin b\\cr 0&\\cos a&-\\sin a\\cos b\\cr 0&\\sin a&\\cos a\\cos b}\\right\]$
%[text] First the step that needs care, $\\partial\\rho/\\partial\\eta=-\[\\rho\]\_\\times\\mathcal{E}$, i.e. the Euler-angle rates really do turn the platform at $\\omega=\\mathcal{E}\\dot\\eta$:
syms a b c q beta r d p0 real
syms P [3 1] real
syms B [3 1] real
syms p [3 1] real
R = [1 0 0; 0 cos(a) -sin(a); 0 sin(a) cos(a)]*[cos(b) 0 sin(b); 0 1 0; -sin(b) 0 cos(b)]*[cos(c) -sin(c) 0; sin(c) cos(c) 0; 0 0 1];
Emap = [1 0 sin(b); 0 cos(a) -sin(a)*cos(b); 0 sin(a) cos(a)*cos(b)];
rho = R*P;
rotationRateIsOmega = all(isAlways(simplify(jacobian(rho, [a b c]) + skew3(rho)*Emap) == 0), 'all')
%[text] Then the whole row straight from the implicit function theorem, $\\partial q\_i/\\partial x=-(\\partial f\_i/\\partial x)/(\\partial f\_i/\\partial q\_i)$:
e = [cos(beta); sin(beta); 0];
h = r*(cos(q)*e + sin(q)*[0; 0; 1]);
u = p + [0; 0; p0] + rho - B - h;
f = u.'*u - d^2;
row = -jacobian(f, [p; a; b; c])/diff(f, q);
%%
%[text] ## Check against the code
%[text] At a tilted pose, the symbolic row for every leg against `kinematicJacobian` (closed form) and against central differences of `inverseKinematicsStep`:
x = [0.004; -0.007; 0.008; 0.07; -0.06; 0.05];
qx = inverseKinematicsStep(geom, x);
Jsym = zeros(6);
for i = 1:6
    Jsym(i,:) = double(subs(row, [p; a; b; c; q; beta; r; d; p0; P; B], ...
        [x; qx(i); geom.betaB(i); geom.r; geom.rodLength; geom.home(3); geom.platformPoints(i,:).'; geom.base(i,:).']));
end
hStep = 1e-7;
Jdiff = zeros(6);
for j = 1:6
    dx = zeros(6, 1); dx(j) = hStep;
    Jdiff(:,j) = (inverseKinematicsStep(geom, x + dx) - inverseKinematicsStep(geom, x - dx))/(2*hStep);
end
jacobianCheck = table(max(abs(Jsym - kinematicJacobian(geom, x)), [], 'all'), max(abs(Jsym - Jdiff), [], 'all'), ...
    'VariableNames', {'symbolicMinusCode', 'symbolicMinusDifferences'})
%%
%[text] ## Newton from home
%[text] Cold start from home to the same tilted pose. The error roughly squares each iteration; on the rig trajectory, warm-started from the previous row, it takes 4 to 6.
xk = zeros(6, 1);
qTarget = qx;
iter = (0:6).';
poseError = zeros(7, 1);
for k = 1:7
    poseError(k) = max(abs(xk - x));
    xk = xk + kinematicJacobian(geom, xk)\(qTarget - inverseKinematicsStep(geom, xk));
end
newton = table(iter, poseError)
%%
%[text] ## Where it breaks
%[text] $J$ goes singular when a rod lines up with its crank's tangent ($u\_i^Th\_{q,i}\\to0$) or when the rods can't resist some platform twist. On the reference trajectory it stays well conditioned:
ref = referenceTrajectory(robot, 'syn', 30);
rc = arrayfun(@(k) rcond(kinematicJacobian(geom, ref.x(k,:).')), 1:50:size(ref.x, 1));
conditioning = table(min(rc), max(rc), 'VariableNames', {'minRcond', 'maxRcond'})

%[appendix]{"version":"1.0"}
%---
%[metadata:view]
%   data: {"layout":"inline"}
%---
