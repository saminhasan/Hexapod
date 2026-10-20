function robot = hexapodRobot()
    % the rig as one numeric struct (m, kg, kg*m^2, rad, s): geometry, masses, motor, drive, joint limits.
    % Every model, script and MATLAB Function block reads it, so keep it numeric (no handles, no strings)
    robot.g = 9.80665;
    robot.fs = 1000;                    % trajectory row rate = firmware tick, Hz
    robot.dt = 1/robot.fs;
    robot.torsoOffset = [0; 0; 0.1];    % platform origin from the torso point, body axes: the designed motions
                                        % (sin, syn, cam, mix, step) are of a point 10 cm below the platform
    robot.sternumImu = [0.0808; -0.0010; 0.3205]; % manikin's sternum IMU from the platform origin (Hexapod_dev torso_imu)

    % rotary Stewart layout: crank radius r, rod n*r, base joints (rB, dB), platform joints (rP, dP)
    % rB = (609.6*sqrt(3)/6 - 59.85)/1000; earlier platform rP = 0.0716025403784439
    r = 0.1; n = 3.5878; rB = 0.1160; dB = 0.106/2; rP = 0.086602254038; dP = 0.02;
    gamma = deg2rad(repelem([0, 120, 240], 2)) + deg2rad(180);
    beta = gamma + repmat([-pi/2, pi/2], 1, 3);
    geom.r = r;
    geom.n = n;
    geom.rodLength = n*r;
    geom.rB = rB; geom.dB = dB; geom.rP = rP; geom.dP = dP;
    geom.gammaB = gamma; geom.betaB = beta;
    geom.gammaP = gamma; geom.betaP = beta;
    geom.base = [rB*cos(gamma) + dB*cos(beta); rB*sin(gamma) + dB*sin(beta); zeros(1, 6)]';     % crank pivots
    geom.crankTip = geom.base + [r*cos(beta); r*sin(beta); zeros(1, 6)]';                       % at zero angle
    geom.platformPoints = [rP*cos(gamma) + dP*cos(beta); rP*sin(gamma) + dP*sin(beta); zeros(1, 6)]'; % platform frame
    P = geom.platformPoints;
    P(:,3) = sqrt(geom.rodLength^2 - sum((geom.crankTip - P).^2, 2));
    geom.platformJoints = P;            % at home, base frame
    geom.home = [0; 0; mean(P(:,3))'];  % platform origin at home; poses are relative to it
    geom.crankDirections = [cos(beta(:)), sin(beta(:)), zeros(6, 1)];
    geom.rodQuat = zeros(6, 4);         % rod orientation at home, initial state of the Simscape joints
    for i = 1:6
        u = rotz(rad2deg(beta(i))) * rotx(90) * [1 0 0; 0 1 0; 0 0 1];
        geom.rodQuat(i,:) = quatBetween([1, 0 ,0]', u\(P(i,:) - geom.crankTip(i,:))');
    end
    robot.geom = geom;

    % stick model (SP) masses; the CAD models carry their own, see bodyModels
    robot.mass.platform = 2.0;
    robot.mass.torso = 2.5;
    robot.mass.rod = 0.55;
    robot.mass.arm = 0.33;

    % motor + 9:1 gearbox; torques motor side
    N = 9;
    Jm = 12.1013101225e-5;
    robot.motor.N = N;
    robot.motor.rotorInertia = Jm;              % motor side
    robot.motor.reflectedInertia = Jm*N^2;      % crank side
    robot.motor.efficiency = 0.9;
    robot.motor.peakTorque = 45*0.9/N;
    robot.motor.ratedTorque = 13*0.9/N;
    robot.motor.maxSpeedRpm = 981;              % bench, 36 V
    robot.motor.Kt = 0.28;                      % Nm/A, datasheet, on the safe side
    cr = 1e-2; cl = 1e-2;                       % SP models the reflected rotor as this solid cylinder
    robot.motor.cylinder = struct('radius', cr, 'length', cl, 'density', (2*(Jm*N^2))/(pi*cr^4*cl));

    % SteadyWin driver in velocity mode, identified: speed/command = a/(s + b)
    robot.drive.a = 41.81;
    robot.drive.b = 31.72;

    robot.joint.sphericalLimitDeg = 27.5;       % rod end cone limit
    robot.joint.reachLimit = 0.98;              % legs kept under this |G|/sqrt(E^2 + F^2) (1 = crank and rod in line)
    % ball screws the rod ends turn on (CAD): along the crank axis at the crank, radial at the platform (platform frame)
    robot.joint.crankScrews = cross(geom.crankDirections, repmat([0 0 1], 6, 1), 2);
    robot.joint.platformScrews = [cos(gamma(:)), sin(gamma(:)), zeros(6, 1)];
end
