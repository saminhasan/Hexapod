function check = checkTrajectory(robot, ref)
    % does ref stay inside the rig's limits? Prints one row per limit: the worst value, when it happens (s into the
    % source motion), the limit, pass or fail. Leg reach and rod-end tilt from workspaceUse, motor speed from ref.qd,
    % motor torque from inverseDynamics with the CAD body (Hexapod_dev). Speed and torque need every pose in reach
    % (IK clips the others, so their angles jump). check.limits (table), check.pass
    N = robot.motor.N;
    [~, reach, rodEnd] = workspaceUse(robot, ref.x);
    name = ["leg reach (1 = straight)"; "rod-end tilt (deg)"; "motor speed (rpm)"; "motor torque peak (Nm)"; ...
        "motor torque rms (Nm)"];
    allowed = [robot.joint.reachLimit; robot.joint.sphericalLimitDeg; robot.motor.maxSpeedRpm; ...
        robot.motor.peakTorque; robot.motor.ratedTorque];
    [worst, k] = deal(nan(5, 1));
    [worst(1), k(1)] = max(reach);
    [worst(2), k(2)] = max(rodEnd);
    if all(ref.reachable)
        [worst(3), k(3)] = max(max(abs(ref.qd), [], 2)*N*60/(2*pi));
        bodies = bodyModels(robot);
        tau = inverseDynamics(robot, bodies(1), ref.t, ref.x).motorTauNewtonEuler;
        [worst(4), k(4)] = max(max(abs(tau), [], 2));
        worst(5) = max(rms(tau));
    end
    at = nan(5, 1);
    at(~isnan(k)) = ref.sourceTime(k(~isnan(k)));
    pass = worst <= allowed;
    check.limits = table(name, worst, at, allowed, pass);
    check.pass = all(pass);

    fprintf('\n%s against the rig''s limits\n', ref.mode);
    fprintf('  %-26s %10s %10s %10s\n', '', 'worst', 'at (s)', 'allowed');
    for i = 1:5
        verdict = "ok";
        if isnan(worst(i)), verdict = "not worked out: a pose is out of reach";
        elseif ~pass(i), verdict = "FAIL"; end
        when = "";
        if ~isnan(at(i)), when = sprintf('%.2f', at(i)); end
        fprintf('  %-26s %10.3g %10s %10.3g   %s\n', name(i), worst(i), when, allowed(i), verdict);
    end
    if ~all(ref.reachable)
        fprintf('  %d rows out of reach, the first at %.2f s\n', nnz(~ref.reachable), ...
            ref.sourceTime(find(~ref.reachable, 1)));
    end
end
