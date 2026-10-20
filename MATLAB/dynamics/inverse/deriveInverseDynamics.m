%[text] # Inverse dynamics
%[text] Crank torques for a prescribed platform motion, worked out twice, Newton–Euler and Lagrange, on the model from the paper: each rod split into point masses at its two ends. Geometry, the closed-form IK and the Jacobian come from `deriveInverseKinematics` and `deriveForwardKinematics`; notation is the same, pose $x=\[p;\\ \\eta\]$, crank angles $q$, rod vector $u\_i$, $\\rho\_i=RP\_i$. Torques are at the crank shaft, motor torque is that over $N$.
here = fileparts(mfilename('fullpath'));
run(fullfile(fileparts(fileparts(here)), 'startup.m'));
robot = hexapodRobot();
bodies = bodyModels(robot);
%%
%[text] ## Crank rates from the closure
%[text] With $\\hat u\_i=u\_i/d$, $h\_{q,i}=\\partial h\_i/\\partial q\_i$ and $D\_i=\\hat u\_i^Th\_{q,i}$ (the crank's leverage along its rod), the platform joint moves at $v\_{P,i}=\\dot p+\\omega\\times\\rho\_i$ and $a\_{P,i}=\\ddot p+\\alpha\\times\\rho\_i+\\omega\\times(\\omega\\times\\rho\_i)$. Differentiating $u\_i^Tu\_i=d^2$ once and twice:
%[text]{"align":"center"} $\\dot q\_i=\\frac{\\hat u\_i^Tv\_{P,i}}{D\_i},\\qquad \\ddot q\_i=\\frac{\\hat u\_i^Ta\_{P,i}+(\\hat u\_i^Th\_i)\\dot q\_i^2+\\|v\_{P,i}-h\_{q,i}\\dot q\_i\\|^2/d}{D\_i}$
%[text] $D\_i\\to0$ means a small platform motion needs a huge crank speed; the code stops there rather than carry on.
%%
%[text] ## Euler rates are not angular velocity
%[text]{"align":"center"} $\\omega=\\mathcal{E}(\\eta)\\dot\\eta,\\qquad \\alpha=\\mathcal{E}\\ddot\\eta+\\dot{\\mathcal{E}}\\dot\\eta$
%[text] The second term matters whenever two angles change together. With a sampled trajectory the pose rates come from 5-point differences; with Simscape logs the world-frame $\\omega$ and $\\alpha$ go in directly.
%%
%[text] ## Rods carry motion one way and force the other
%[text] Stacking the six closures, $A\_{i,:}=\[\\hat u\_i^T,\\ (\\rho\_i\\times\\hat u\_i)^T\]$ and $D=\\mathrm{diag}(D\_i)$:
%[text]{"align":"center"} $A\\left\[\\matrix{\\dot p\\cr \\omega}\\right\]=D\\dot q,\\qquad W=\\left\[\\matrix{F\\cr M}\\right\]=A^Tf,\\qquad \\tau\_{rod}=Df$
%[text] with $f\_i$ positive when the rod pushes the platform. Power is the same on both sides, $W^T\[\\dot p;\\ \\omega\]=(Df)^T\\dot q$, so there is no hidden sign. A near-singular $A$ means the rods can't resist some twist, a separate problem from a small $D\_i$.
%%
%[text] ## Which mass moves with what
%[text] Upper rod halves ride with the platform and the torso, lower halves with the crank tips. The upper body (platform, torso, six upper halves) becomes one rigid body:
%[text]{"align":"center"} $m=\\sum\_jm\_j,\\qquad c=\\frac{1}{m}\\sum\_jm\_jc\_j,\\qquad I\_c=\\sum\_j\\left\[I\_j+m\_j\\left(\\|c\_j-c\\|^2I\_3-(c\_j-c)(c\_j-c)^T\\right)\\right\]$
%[text] The upper halves add inertia as well as mass. The three body models differ only in numbers; the two CAD ones share the same assembly.
bodyTable = table(string({bodies.name}).', [bodies.platformMass].', [bodies.rodMass].', [bodies.armMass].', ...
    [bodies.torsoMass].', arrayfun(@(s) s.torsoCOM(3), bodies).', ...
    'VariableNames', {'model', 'platform_kg', 'eachRod_kg', 'eachArm_kg', 'torso_kg', 'torsoHeight_m'})
%%
%[text] ## Newton–Euler
%[text] Upper body COM in world axes $r\_c=Rc$, inertia $I\_w=RI\_cR^T$, gravity $g=\[0\\ 0\\ -9.80665\]^T$:
%[text]{"align":"center"} $a\_c=\\ddot p+\\alpha\\times r\_c+\\omega\\times(\\omega\\times r\_c),\\qquad F=m(a\_c-g),\\qquad M=I\_w\\alpha+\\omega\\times(I\_w\\omega)+r\_c\\times F$
%[text] Moments are about the platform origin, hence the last term. Solve $A^Tf=W$, then $\\tau\_{rod}=Df$.
%%
%[text] ## Lagrange
%[text] In pose coordinates $x$ the upper body's COM velocity and angular velocity are linear in $\\dot x$:
%[text]{"align":"center"} $J\_c=\\left\[\\matrix{I\_3&-\[r\_c\]\_\\times\\mathcal{E}}\\right\],\\qquad J\_\\omega=\\left\[\\matrix{0&\\mathcal{E}}\\right\],\\qquad M\_x=mJ\_c^TJ\_c+J\_\\omega^TI\_wJ\_\\omega,\\qquad V=-mg^T(p+r\_c)$
%[text]{"align":"center"} $Q=M\_x\\ddot x+\\dot M\_x\\dot x-\\frac{1}{2}z+\\nabla\_xV,\\qquad z\_j=\\dot x^T\\frac{\\partial M\_x}{\\partial x\_j}\\dot x$
%[text] $M\_x$ only depends on the angles; the code differences it at $10^{-5}$ rad. $Q$ is a generalized force, so the rod forces come from virtual work with $K=A\\,\\mathrm{blkdiag}(I\_3,\\mathcal{E})$: $K^Tf\_L=Q$. Nothing is borrowed from the Newton–Euler side.
%%
%[text] ## Cranks, lower rod halves, rotors
%[text] Arm mass $m\_a$ at radius $c\_a$ with own inertia $I\_a$, lower rod half at $r$, rotor already reflected to the crank ($N^2J\_m$, don't reflect it twice):
%[text]{"align":"center"} $J\_i=J\_{rot}+I\_a+m\_ac\_a^2+\\frac{m\_r}{2}r^2,\\qquad \\tau\_i=D\_if\_i+J\_i\\ddot q\_i-\\left(\\frac{m\_ac\_a}{r}+\\frac{m\_r}{2}\\right)g^Th\_{q,i}+b\_i\\dot q\_i$
%[text] Newton–Euler gets the same crank term from the arm and lower rod forces taken about the motor axis $e\_i\\times e\_z$.
%%
%[text] ## Static check
%[text] At home, symmetric, each crank carries a sixth of platform plus torso, a whole rod (half through the platform, half at its tip) and half an arm:
%[text]{"align":"center"} $\\tau\_{home}=gr\\left(\\frac{m\_p+m\_t}{6}+m\_r+\\frac{m\_a}{2}\\right)$
stick = bodies(3);
t = (0:0.01:0.10).';
atHome = inverseDynamics(robot, stick, t, zeros(numel(t), 6));
expected = robot.g*robot.geom.r*((stick.platformMass + stick.torsoMass)/6 + stick.rodMass + stick.armMass/2);
staticCheck = table(expected, atHome.tauNewtonEuler(1,1), atHome.tauLagrange(1,1), ...
    'VariableNames', {'weightBalance_Nm', 'newtonEuler_Nm', 'lagrange_Nm'})
%%
%[text] ## On the reference trajectory
%[text] Six cycles of the synthetic run through `referenceTrajectory`, stick model. The two methods sit on top of each other: that checks the code and the conventions, not the endpoint-mass model itself.
ref = referenceTrajectory(robot, 'syn', 6);
estimate = inverseDynamics(robot, stick, ref.t, ref.x);
tiles = tiledlayout(3, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
set(gcf, 'Position', [100 100 780 800]);
for leg = 1:6
    nexttile;
    plot(estimate.t, estimate.tauLagrange(:,leg), '-', 'Color', [0.10 0.35 0.65], 'LineWidth', 1.3);
    hold on;
    plot(estimate.t, estimate.tauNewtonEuler(:,leg), '--', 'Color', [0.85 0.30 0.10], 'LineWidth', 1);
    title("Motor " + leg); xlabel('Time (s)'); ylabel('Crank torque (N m)'); grid on;
end
title(tiles, 'Requested-motion torque: Lagrange and Newton-Euler');
lg = legend('Lagrange', 'Newton-Euler', 'Orientation', 'horizontal');
lg.Layout.Tile = 'north';
calculationCheck = table(max(estimate.closureResidual), estimate.maxMethodDifference, ...
    max(abs(estimate.tauNewtonEuler), [], 'all'), min(estimate.reciprocalCondition), ...
    'VariableNames', {'maxClosureError_m', 'maxMethodDifference_Nm', 'peakCrankTorque_Nm', 'minRcondA'})
%%
%[text] ## Against Simscape
%[text] `runInverseDynamics` runs the three models and also feeds each one's achieved motion (logged world-frame rates, the logger's +g taken off) back through the ID, so tracking lag isn't counted as model error. Error below is RMS(ID − Simscape)/RMS(Simscape) per motor, averaged.
summaryFile = fullfile(here, 'output', 'inverse_dynamics', 'summary.csv');
if isfile(summaryFile)
    simCheck = readtable(summaryFile, 'TextType', 'string');
    simCheck = simCheck(:, {'Model', 'SimPeakNm', 'CommandIDPeakNm', 'AchievedIDPeakNm', 'MeanMotorNRMSEPercent'})
else
    disp('run runInverseDynamics first');
end
resultsFile = fullfile(here, 'output', 'inverse_dynamics', 'results.mat');
if isfile(resultsFile) && ismember('results', who('-file', resultsFile))
    saved = load(resultsFile, 'results');
    pd = saved.results.Hexapod_PD;
    figure;
    plot(pd.command.t, pd.command.tauNewtonEuler(:,3), '-', 'Color', [0.65 0.65 0.65], 'DisplayName', 'ID, requested motion');
    hold on;
    plot(pd.simTime, pd.simTau(:,3), 'k-', 'LineWidth', 1.3, 'DisplayName', 'Simscape');
    plot(pd.achieved.t, pd.achieved.tauNewtonEuler(:,3), '--', 'Color', [0.85 0.30 0.10], 'LineWidth', 1.2, 'DisplayName', 'ID, achieved motion');
    xlim([2 3]); grid on;
    xlabel('Time (s)'); ylabel('Crank torque (N m)');
    title('Hexapod\_PD, motor 3: the velocity loop changes the torque it needs');
    legend('Location', 'northoutside');
end
%%
%[text] ## Reference
%[text] Z. Mahmoud, M. R. Arvan, V. Nekoukar, M. Rezaei, *A comparative study of two inverse dynamic models of 6 degree-of-freedom rotary Stewart–Gough parallel robot*, Int. J. Dynamics and Control 10, 377–390 (2022), [doi:10.1007/s40435-021-00820-5](https://doi.org/10.1007/s40435-021-00820-5). Copy in `papers/`.

%[appendix]{"version":"1.0"}
%---
%[metadata:view]
%   data: {"layout":"inline"}
%---
