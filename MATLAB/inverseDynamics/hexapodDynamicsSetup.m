function model = hexapodDynamicsSetup(sp, profile)
%HEXAPODDYNAMICSSETUP Prepare numeric constants for hexapodDynamicsStep.
% Run once before simulation. Uses the same endpoint-rod approximation and
% mass profiles as inverseDynamics; no simulation or trajectory is needed.
% The returned numeric struct can be a parameter of a MATLAB Function block.

model.r = sp.r;
model.rodLength = sp.d;
model.home = sp.homez(:);
model.base = sp.B;
model.platformPoints = sp.Pp;
model.crankDirections = [cos(sp.betaB(:)), sin(sp.betaB(:)), zeros(6,1)];

rodMass = six(profile.rodMass);
armMass = six(profile.armMass);
armCOM = six(profile.armCOM);
model.jointJ = six(profile.rotorInertia) + six(profile.armInertia) ...
    + armMass.*armCOM.^2 + rodMass*sp.r^2/2;
model.gravityMassRadius = armMass.*armCOM/sp.r + rodMass/2;
model.gravity = profile.gravity(:);
model.viscousFriction = zeros(6,1);
if isfield(profile,'viscousFriction')
    model.viscousFriction = six(profile.viscousFriction);
end
model.gearRatio = profile.gearRatio;

masses = [profile.platformMass; profile.payloadMass; rodMass/2];
centres = [profile.platformCOM(:), profile.payloadCOM(:), sp.Pp.'];
model.upperMass = sum(masses);
model.upperCOM = zeros(3,1);
if model.upperMass > 0
    model.upperCOM = centres*masses/model.upperMass;
end
model.upperInertia = profile.platformInertia + profile.payloadInertia;
for i = 1:8
    offset = centres(:,i)-model.upperCOM;
    model.upperInertia = model.upperInertia ...
        + masses(i)*(dot(offset,offset)*eye(3)-offset*offset.');
end
end

function value = six(value)
validateattributes(value,{'numeric'},{'real','finite','vector','nonnegative'});
if isscalar(value)
    value = repmat(value,6,1);
elseif numel(value) == 6
    value = value(:);
else
    error('hexapodDynamicsSetup:Size','Limb parameters must be scalar or six-vectors.');
end
end
