function dyn = inverseDynamicsModel(robot, body)
    % numeric struct for inverseDynamicsStep: robot.geom plus the lumped masses of one body model, same
    % model as inverseDynamics. Build once, then pass it as a MATLAB Function block parameter
    p = fillBody(body);
    dyn.geom = robot.geom;
    dyn.jointJ = p.reflectedInertia + p.armInertia + p.armMass.*p.armCOM.^2 + p.rodMass*robot.geom.r^2/2;
    dyn.gravityMassRadius = p.armMass.*p.armCOM/robot.geom.r + p.rodMass/2;
    dyn.gravity = p.gravity;
    dyn.viscousFriction = p.viscousFriction;
    dyn.gearRatio = p.gearRatio;
    [dyn.upperMass, dyn.upperCOM, dyn.upperInertia] = upperBody(robot.geom, p);
end
