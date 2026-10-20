%[text] # Inverse kinematics
%[text] Crank angles from a platform pose, one leg at a time, closed form. Notation follows the code: pose $x=\[p;\\ \\eta\]$ relative to home $p\_0$, XYZ Euler angles $\\eta=\[a\\ b\\ c\]$, crank angle $q\_i$, crank radius $r$, rod length $d$. Symbolic steps first, then a numeric check against `inverseKinematicsStep`.
here = fileparts(mfilename('fullpath'));
run(fullfile(fileparts(fileparts(here)), 'startup.m'));
robot = hexapodRobot();
geom = robot.geom;
%%
%[text] ## One leg
%[text] Crank pivot $B\_i$ (base frame), platform joint $P\_i$ (platform frame), crank direction $e\_i=\[\\cos\\beta\_i\\ \\sin\\beta\_i\\ 0\]^T$. The crank turns in the vertical plane through $e\_i$, and the rod closes when $\\|\\ell\_i-h\_i\\|=d$:
%[text]{"align":"center"} $R=R\_x(a)R\_y(b)R\_z(c),\\qquad \\ell\_i=p+p\_0+RP\_i-B\_i,\\qquad h\_i=r(\\cos q\_i\\,e\_i+\\sin q\_i\\,e\_z)$
syms a b c q beta r d real
syms l [3 1] real
Rx = [1 0 0; 0 cos(a) -sin(a); 0 sin(a) cos(a)];
Ry = [cos(b) 0 sin(b); 0 1 0; -sin(b) 0 cos(b)];
Rz = [cos(c) -sin(c) 0; sin(c) cos(c) 0; 0 0 1];
R = Rx*Ry*Rz;
eta = deg2rad([3 -4 5]);
rotationMatchesCode = max(abs(double(subs(R, [a b c], eta)) - xyzRotation(eta)), [], 'all') < 1e-15
e = [cos(beta); sin(beta); 0];
h = r*(cos(q)*e + sin(q)*[0; 0; 1]);
closure = expand((l - h).'*(l - h) - d^2);
%%
%[text] ## Closure as one trig equation
%[text] With $\\|e\_i\\|=1$ and $\\|h\_i\\|=r$ everything that depends on $q\_i$ collects into
%[text]{"align":"center"} $E\_i\\sin q\_i+F\_i\\cos q\_i=G\_i,\\qquad E\_i=2r\\ell\_{iz},\\qquad F\_i=2r\\,e\_i^T\\ell\_i,\\qquad G\_i=\\|\\ell\_i\\|^2+r^2-d^2$
E = 2*r*l(3);
F = 2*r*(e.'*l);
G = l.'*l + r^2 - d^2;
closureIsTrigEquation = isAlways(simplify(closure - (G - E*sin(q) - F*cos(q))) == 0)
%%
%[text] ## Solve for q
%[text] Writing $E=k\\cos\\varphi$, $F=k\\sin\\varphi$ with $k=\\sqrt{E^2+F^2}$, $\\varphi=\\mathrm{atan2}(F,E)$ turns the left side into $k\\sin(q+\\varphi)$, so
%[text]{"align":"center"} $q\_i=\\sin^{-1}\\!\\left(G\_i/\\sqrt{E\_i^2+F\_i^2}\\right)-\\mathrm{atan2}(F\_i,E\_i)$
%[text] The other root, $\\pi-\\sin^{-1}(\\cdot)-\\mathrm{atan2}(F\_i,E\_i)$, is the crank folded the other way. The rig is assembled on the first one and never switches. $|G\_i|>\\sqrt{E\_i^2+F\_i^2}$ means the leg can't close, which is the `ok` flag in `inverseKinematicsStep`.
syms k phi real
harmonicForm = isAlways(simplify(k*cos(phi)*sin(q) + k*sin(phi)*cos(q) - k*sin(q + phi)) == 0)
%%
%[text] ## Check against the code
%[text] Same pose through the symbolic $E$, $F$, $G$ and through `inverseKinematicsStep`; the last column is the rod length error with the angle put back in.
x = [0.004; -0.006; 0.008; deg2rad([3; -4; 5])];
Rn = xyzRotation(x(4:6));
L = (x(1:3) + geom.home).' + (Rn*geom.platformPoints.').' - geom.base;
qClosed = zeros(6, 1);
for i = 1:6
    v = double(subs([E F G], [l; r; d; beta], [L(i,:).'; geom.r; geom.rodLength; geom.betaB(i)]));
    qClosed(i) = asin(v(3)/hypot(v(1), v(2))) - atan2(v(2), v(1));
end
hn = geom.r*(cos(qClosed).*geom.crankDirections + sin(qClosed).*[0 0 1]);
ikCheck = table((1:6).', rad2deg(qClosed), qClosed - inverseKinematicsStep(geom, x), vecnorm(L - hn, 2, 2) - geom.rodLength, ...
    'VariableNames', {'leg', 'q_deg', 'minusCode_rad', 'closure_m'})

%[appendix]{"version":"1.0"}
%---
%[metadata:view]
%   data: {"layout":"inline"}
%---
