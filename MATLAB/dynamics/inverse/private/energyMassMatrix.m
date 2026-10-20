function [M, R] = energyMassMatrix(angles, m, c, I)
    % 6x6 mass matrix of the upper body in pose coordinates x = [x y z Rx Ry Rz], kinetic energy = xd'*M*xd/2
    [R, E] = xyzRotation(angles);
    Jc = [eye(3), -skew3(R*c)*E];
    Jw = [zeros(3), E];
    M = m*(Jc.'*Jc) + Jw.'*(R*I*R.')*Jw;
end
