function profiles = inverseDynamicsProfiles(sp)
%INVERSEDYNAMICSPROFILES Editable, approximate mass models for the three SLXs.
% All lengths in m, masses in kg, inertias in kg*m^2 ABOUT THE BODY'S COM.
% COM vectors use the same moving-platform frame as sp.Pp (not torso frame).
% These profiles describe the checked-in models; they do not modify them.
% If you change a solid, payload, or frame in Simscape, update its profile.
%
% CAD masses are summed from Backup/Hexapod_DataFile.m. CAD aggregate COM and
% inertia below were calculated from its solids and the saved SLX transforms.
% They are snapshots, NOT a live CAD inertia extractor. The arm is simplified
% to its radial COM; its small axial offset is omitted. Rods are split into
% two endpoint masses by inverseDynamics, as in the paper.

    params;
    if nargin == 0
        sp = StuartPlatform(r, n, rB, dB, rP, dP);
    end
    repoRoot = fileparts(fileparts(mfilename('fullpath')));
    run(fullfile(repoRoot, 'Backup', 'Hexapod_DataFile.m'));
    cadMass = [smiData.Solid.mass];

    cad.name = 'Hexapod_dev';
    cad.platformMass = cadMass(22) + 6*cadMass(12) + 3*cadMass(25) + ...
        6*cadMass(24) + 6*cadMass(23) + 12*cadMass(26) + 12*cadMass(17);
    cad.platformCOM = [0; 0; 0.0201231801];
    cad.platformInertia = diag([0.0116545883, 0.0116545883, 0.0218994532]);
    cad.payloadMass = 2.5; % literal mass in all three active torso.STL blocks
    cad.payloadCOM = [0; 0; 0.2205];
    % From volume integration of torso.STL, in its exposed COM-frame axes.
    cad.payloadInertia = [0.0378430359, -0.0000742720, 0.00221245392; ...
        -0.0000742720, 0.0287118993, 0.000104256524; ...
        0.00221245392, 0.000104256524, 0.0234715779];
    cad.rodMass = sum(cadMass(19:21));
    cad.armMass = cadMass(18) + cadMass(12) + cadMass(13) + cadMass(15) + ...
        3*cadMass(10) + cadMass(17) + cadMass(14);
    cad.armCOM = 0.0570759746;
    cad.armInertia = 0.00104953726;
    % Explicit reflected inertia block PLUS the CAD rotor's own solid inertia.
    cad.rotorInertia = Jmr + smiData.Solid(16).MoI(3)*1e-6;
    cad.gravity = [0; 0; -g];
    cad.viscousFriction = 0;
    cad.gearRatio = N;
    cad.notes = 'CAD assembly; endpoint-lumped rods; aggregate inertia snapshot.';

    pd = cad;
    pd.name = 'Hexapod_PD';
    pd.notes = 'Same saved CAD mass assembly as dev; PD/velocity plant changes achieved motion.';
    % Override pd.rodMass, pd.platformMass, etc. here if you later change PD.

    stick = cad;
    stick.name = 'SP';
    stick.platformMass = platform_mass;
    stick.platformCOM = zeros(3,1);
    % SP uses CalculateFromGeometry, so params.Iplat is NOT its active inertia.
    stick.platformInertia = polygonInertia(sp.Pp(:,1:2), platform_mass, 0.02);
    stick.payloadCOM = [0; 0; 0.192];
    stick.rodMass = rod_mass;
    stick.armMass = arm_mass;
    stick.armCOM = sp.r/2;
    stick.armInertia = arm_mass*(sp.r^2 + (sp.r/10)^2)/12;
    stick.rotorInertia = Jmr;
    stick.notes = 'Stick geometry; params masses; exact brick/polygon inertia; endpoint-lumped rods.';
    profiles = [cad, pd, stick];
end

function inertia = polygonInertia(vertices, mass, thickness)
% Uniform extrusion of the centered polygon used by the SP Solid block.
    x = vertices(:,1); y = vertices(:,2);
    xn = circshift(x,-1); yn = circshift(y,-1);
    c = x.*yn-xn.*y;
    area = sum(c)/2;
    if abs(area) < eps
        error('inverseDynamicsProfiles:Polygon', 'Platform polygon has zero area.');
    end
    xx = mass*sum((y.^2 + y.*yn + yn.^2).*c)/(12*area);
    yy = mass*sum((x.^2 + x.*xn + xn.^2).*c)/(12*area);
    xy = -mass*sum((2*x.*y + x.*yn + xn.*y + 2*xn.*yn).*c)/(24*area);
    inertia = [xx+mass*thickness^2/12, xy, 0; ...
               xy, yy+mass*thickness^2/12, 0; 0, 0, xx+yy];
end
