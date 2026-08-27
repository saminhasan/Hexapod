riseTime = 1.0;
fallTime = 1.0;
% rB = (((609.6 ) * sqrt(3) / 6) - 59.85) /1000
% [r, n, rB, dB, rP, dP] = deal(0.1, 3.5878,0.1160, 0.106/2, 0.0716025403784439,  0.02);
[r, n, rB, dB, rP, dP] = deal(0.1, 3.5878,0.1160, 0.106/2, 0.086602254038,  0.02);
fs = 1000;
dt = 1/fs;

efficiency = 0.9;
g = 9.80665;
torso_mass = 2.5; % kg
platform_mass= 2.0; % kg
N = 9; % gear ratio
Jm = 12.1013101225e-5; % motor inertia in motor frame ( kg . m^2)
Jmr = Jm*N^2; % motor inertia in robot frame ( kg . m^2)
Jmr = Jmr;
cr = 1e-2;
cl = 1e-2;
density_cyl = (2 * (Jm * N^2)) / (pi * cr^4 * cl);
rod_mass = 0.55; % kg
arm_mass  = 0.33; % kg
m = platform_mass + 3*rod_mass;
Iplat = diag([0.00535189, 0.00535189, 0.0106204]);
J = Jmr + (arm_mass * (r/2)^2) + ((rod_mass/2) * (r)^2);
peak_torque = 45*efficiency/N; % mit motor peak torque in motor frame
rated_torque = 13*efficiency/N; % mit motor rated torque in motor frame
Jr = (platform_mass / 6) * r^2; % robot equivalent inertia in robot frame
Je = (Jmr + Jr);


hmjm_max_deg = 27.5;

smoothStep = false;
cycle = 0;
order = 20;
tauK = riseTime/ (gammaincinv(0.9, order) - gammaincinv(0.1, order));
if (smoothStep)
    riseDealy = calcDealy(order,tauK); %#ok<UNRCH>
else
    riseDealy = 0;
end
%fliplr(poly(ones(1, order) * -1) .* tau.^(0:order));  % (taus + 1)^25
function delay =  calcDealy(order, tauK) %#ok<DEFNU>
    syms t tau k
    % symbolic y(t)
    y_sym = t- order*tau + tau*exp(-t/tau)* symsum( (order - k)*(t/tau)^k / factorial(k), k, 0, order-1 );
    % pretty(y_sym)
    % 4) Solve symbolically y(t)=1 for general n,tau
    % sol_general = solve(y_sym == 1, t);
    % → returns a RootOf(...) object, since no elementary closed‑form exists.
    %solve numerically
    y_num = subs(y_sym, tau, tauK);
    initial_guess = order*0.01 + 1;    % ≈ n*tau + 1
    t_sol = vpasolve(y_num == 1, t, initial_guess);
    % fprintf('Numeric solution : ', double(t_sol));
    delay = double(t_sol-1);
end