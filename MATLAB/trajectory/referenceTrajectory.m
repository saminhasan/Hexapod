function ref = referenceTrajectory(robot, mode, n)
    % what the rig plays, every stage kept for the plots and the check:
    %   a  genTrajectory(robot, mode, n)  what moves: ref.body, a body point's [t x], ref.bodyToPlatform
    %   b  placePlatform                  that point -> platform origin, centred: ref.platform
    %   c  warpTrajectoryClock, taperTrajectory   eases in and out over 1 s: ref.warped, ref.sourceTime
    %   d  inverseKinematics              crank angles; ref.reachable flags the rows every leg can close
    % ref.t, ref.x (platform pose, N x 6), ref.q, ref.qd, ref.qdd (cranks) are what the models and controllers read.
    % The motor test (jointStep) has no body motion: it starts at d with its crank angles, ref.x from
    % forwardKinematics. n: cycles of a designed mode, not used otherwise
    if nargin < 3, n = []; end
    rampTime = 1; % every uploaded file eases in and out over 1 s
    src = genTrajectory(robot, mode, n);
    ref.mode = mode;
    ref.cycles = n;
    ref.rampTime = rampTime;
    ref.recorded = src.recorded;
    ref.body = src.body;
    ref.bodyToPlatform = src.bodyToPlatform;
    if isempty(src.joints)
        [ref.platform, ref.centerOffset] = placePlatform(robot, src);
        [ref.warped, warp] = warpTrajectoryClock(ref.platform, rampTime, rampTime, 'spline');
        tapered = taperTrajectory(ref.warped, rampTime, rampTime);
        ref.sourceTime = ref.platform(1,1) + warp.srcTime;
        ref.window = bumpWindow(tapered(:,1), tapered(1,1), rampTime, rampTime, tapered(end,1));
        ref.t = tapered(:,1);
        ref.x = tapered(:,2:7);
        [ref.q, ref.reachable] = inverseKinematics(robot, ref.x);
    else
        ref.t = src.joints(:,1);
        ref.q = src.joints(:,2:7);
        ref.x = forwardKinematics(robot, ref.q, zeros(1, 6));
        ref.reachable = true(size(ref.t));
        ref.platform = [ref.t, ref.x];
        ref.centerOffset = zeros(1, 3);
        ref.warped = ref.platform;
        ref.sourceTime = ref.t;
        ref.window = ones(size(ref.t));
    end
    if src.recorded || ~isempty(src.joints), ref.cycles = []; end
    ref.qd = periodicDerivative5pt(ref.q, robot.dt);
    ref.qdd = periodicDerivative5pt(ref.qd, robot.dt);
end
