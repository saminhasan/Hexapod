function obj = StuartPlatform(r, n, rB, dB, rP, dP)
    % rotary Stewart platform geometry: crank radius r, rod length n*r, base/platform joints from
    % (rB, dB) and (rP, dP); obj.move(trajectory) gives [t q1..q6] for an [t x y z Rx Ry Rz] path
    obj.r = r;
    obj.n = n;
    obj.d = n * r;
    gamma = deg2rad(repelem([0,120,240],2)) + deg2rad(180);
    beta = gamma + repmat([-pi/2, pi/2], 1, 3);
    obj.gammaB = gamma;
    obj.betaB = beta;
    obj.gammaP = gamma;
    obj.betaP = beta;

    obj.B = [rB*cos(gamma) + dB*cos(beta); rB*sin(gamma) + dB*sin(beta); zeros(1,6)]'; % crank pivots
    obj.H = obj.B + [r*cos(beta); r*sin(beta); zeros(1,6)]'; % crank tips at zero angle
    obj.P = [rP*cos(gamma) + dP*cos(beta); rP*sin(gamma) + dP*sin(beta); zeros(1,6)]'; % platform joints, base frame
    obj.Pp = obj.P; % platform joints, platform frame
    obj.P(:,3) = sqrt(obj.d^2 - sum((obj.H - obj.P).^2, 2));
    obj.homez = [0; 0; mean(obj.P(:,3))'];
    obj.HP = obj.P - obj.H;
    obj.BH = obj.H - obj.B;

    obj.q_rots = zeros(6, 4); % rod orientations at home, initial state for the Simscape joints
    for i = 1:6
        u = rotz(rad2deg(beta(i))) * rotx(90) * [1 0 0; 0 1 0; 0 0 1];
        obj.q_rots(i, :) = quatBetween([1, 0 ,0]', u\obj.HP(i,:)');
    end
    obj.move = @(pose) stuartMove(obj, pose);
end
