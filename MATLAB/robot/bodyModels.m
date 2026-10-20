function bodies = bodyModels(robot)
    % lumped mass models of the three sims, [Hexapod_dev, Hexapod_PD, SP]: kg, m, kg*m^2 about each body's
    % own COM, COMs in the platform frame. CAD numbers are summed from cad/Hexapod_DataFile.m and frozen here,
    % edit them if a solid changes. Arms are reduced to their radial COM; rods get split by inverseDynamics.
    % reflectedInertia is the rotor seen at the crank
    run(fullfile(fileparts(mfilename('fullpath')), 'cad', 'Hexapod_DataFile.m'));
    cadMass = [smiData.Solid.mass];

    cad.name = 'Hexapod_dev';
    cad.platformMass = cadMass(22) + 6*cadMass(12) + 3*cadMass(25) + 6*cadMass(24) + 6*cadMass(23) ...
        + 12*cadMass(26) + 12*cadMass(17);
    cad.platformCOM = [0; 0; 0.0201231801];
    cad.platformInertia = diag([0.0116545883, 0.0116545883, 0.0218994532]);
    cad.torsoMass = 2.5;                % the torso.STL blocks carry a literal 2.5 kg
    cad.torsoCOM = [0; 0; 0.2205];
    cad.torsoInertia = [0.0378430359, -0.0000742720, 0.00221245392;     % volume integral of torso.STL
        -0.0000742720, 0.0287118993, 0.000104256524;
        0.00221245392, 0.000104256524, 0.0234715779];
    cad.rodMass = sum(cadMass(19:21));
    cad.armMass = cadMass(18) + cadMass(12) + cadMass(13) + cadMass(15) + 3*cadMass(10) + cadMass(17) + cadMass(14);
    cad.armCOM = 0.0570759746;
    cad.armInertia = 0.00104953726;
    cad.reflectedInertia = robot.motor.reflectedInertia + smiData.Solid(16).MoI(3)*1e-6; % inertia block + CAD rotor solid
    cad.gravity = [0; 0; -robot.g];
    cad.viscousFriction = 0;
    cad.gearRatio = robot.motor.N;

    pd = cad;                           % same CAD assembly, only the drive differs
    pd.name = 'Hexapod_PD';

    stick = cad;
    stick.name = 'SP';
    stick.platformMass = robot.mass.platform;
    stick.platformCOM = zeros(3, 1);
    stick.platformInertia = polygonInertia(robot.geom.platformPoints(:,1:2), robot.mass.platform, 0.02); % SP's extrusion
    stick.torsoCOM = [0; 0; 0.192];
    stick.rodMass = robot.mass.rod;
    stick.armMass = robot.mass.arm;
    stick.armCOM = robot.geom.r/2;
    stick.armInertia = robot.mass.arm*(robot.geom.r^2 + (robot.geom.r/10)^2)/12;
    stick.reflectedInertia = robot.motor.reflectedInertia;
    bodies = [cad, pd, stick];
end
