function obj = StuartPlatform(r, n, rB, dB, rP, dP)
    % StuartPlatform class constructor
    % r: radius of the crank arm
    % n: crank arm to connecting rod length ratio
    obj.r = r;
    obj.n = n;
    obj.d = n * r;
    gamma = deg2rad(repelem([0,120,240],2)) + deg2rad(180);
    beta  = gamma + repmat([-pi/2, pi/2], 1, 3); %

    obj.gammaB = gamma;
    obj.betaB  = beta;
    obj.gammaP = gamma;
    obj.betaP  = beta;

    obj.B = [rB*cos(gamma) + dB*cos(beta); rB*sin(gamma) + dB*sin(beta); zeros(1,6)]'; % Base points / Servo Arm base
    obj.H = obj.B + [r*cos(beta); r*sin(beta); zeros(1,6)]'; % Servo arm tips
    obj.P = [rP*cos(gamma) + dP*cos(beta); rP*sin(gamma) + dP*sin(beta); zeros(1,6)]'; % platform co-ordinate in B frame
    obj.Pp = obj.P; % in P frame, platform points
    obj.P(:,3) = sqrt(obj.d^2 - sum((obj.H - obj.P).^2, 2));
    obj.homez = [0; 0; mean(obj.P(:,3))'];
    obj.HP = obj.P - obj.H;
    obj.BH = obj.H - obj.B;
    %--------calculate quaternion for correct simscape initialization------------------------------------------
    obj.q_rots = zeros(6, 4);
    for i = 1:6
        u = rotz(rad2deg(beta(i))) * rotx(90) * [1 0 0; 0 1 0; 0 0 1];
        v = u\obj.HP(i,:)';% from o frame to u frame
        obj.q_rots(i, :) = calcQuat([1, 0 ,0]', v);
    end
    %--------calculate quaternion------------------------------------------
    obj.move = @(pose) moveFunc(obj, pose);
end

function motorAngles = moveFunc(obj, trajectory)
    nRows = size(trajectory, 1);
    motorAngles = zeros(nRows, 7);

    % Track workspace violations so the whole trajectory is reported at once,
    % rather than dying on the first bad sample.
    nBad      = 0;
    worstArg  = 0;
    tFirstBad = NaN;

    for row = 1:nRows
        pose = trajectory(row,2:7);
        R = eul2rotm(pose(4:6), 'XYZ');
        t = pose(1:3)' + obj.homez;
        l = repmat(t, 1, 6)' + (R * obj.Pp')' - obj.B; % leg length
        ek = 2 * obj.r * l(:,3);
        fk = 2 * obj.r * (cos(obj.betaB') .* l(:,1) + sin(obj.betaB') .* l(:,2));

        % |arg| > 1 means the leg cannot close: asin would return a complex
        % angle and silently poison everything downstream.
        arg = ((vecnorm(l, 2, 2).^2) - ((obj.n^2 - 1)*obj.r^2)) ./ sqrt(ek.^2 + fk.^2);

        if any(abs(arg) > 1)
            nBad = nBad + 1;
            if isnan(tFirstBad)
                tFirstBad = trajectory(row,1);
            end
            worstArg = max(worstArg, max(abs(arg)));
        end

        motorAngles(row,2:7) = (asin(arg) - atan2(fk, ek)).';
    end

    motorAngles(:,1) = trajectory(:,1);

    if nBad > 0
        error('StuartPlatform:Unreachable', ...
            ['Pose outside workspace on %d of %d samples ' ...
             '(first at t = %.4f s, worst |arg| = %.4f > 1). ' ...
             'Reduce trajectory amplitude.'], ...
            nBad, nRows, tFirstBad, worstArg);
    end
end
function quaternion = calcQuat(U, bp)
    cross_product = cross(U, bp);
    angle_radians = acos(dot(U, bp) / (norm(U) * norm(bp)));
    axis_of_rotation = cross_product / norm(cross_product);
    qw = cos(angle_radians / 2);
    qx = axis_of_rotation(1) * sin(angle_radians / 2);
    qy = axis_of_rotation(2) * sin(angle_radians / 2);
    qz = axis_of_rotation(3) * sin(angle_radians / 2);
    quaternion = [qw, qx, qy, qz];
    quaternion = quaternion / norm(quaternion);
end