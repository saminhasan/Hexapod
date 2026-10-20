function [m, c, I] = upperBody(geom, p)
    % platform + torso + upper rod halves as one body: mass, COM and inertia about that COM (platform frame)
    masses = [p.platformMass; p.torsoMass; p.rodMass/2];
    centres = [p.platformCOM, p.torsoCOM, geom.platformPoints.'];
    m = sum(masses);
    if m > 0
        c = centres*masses/m;
    else
        c = zeros(3, 1);
    end
    I = p.platformInertia + p.torsoInertia;
    for i = 1:8
        d = centres(:,i) - c;
        I = I + masses(i)*(dot(d, d)*eye(3) - d*d.');
    end
end
