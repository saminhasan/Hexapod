function row = trackingError(simOut, ref, robot)
    % one table row: platform position (mm) and angle (deg) rms error against ref.x, motor angle rms error
    % against ref.q (deg), peak crank torque; the first and last 0.5 s are left out
    [t, d] = getSimSignal(simOut, 'platform_pose', 19);
    pRef = interp1(ref.t, ref.x(:,1:3) + robot.geom.home.', t);
    aRef = interp1(ref.t, ref.x(:,4:6), t);
    k = t > 0.5 & t < t(end) - 0.5;
    ep = 1000*(d(k,11:13) - pRef(k,:));
    ea = rad2deg(wrapToPi(quat2eul(d(k,1:4), 'XYZ') - aRef(k,:)));
    [tm, md] = getSimSignal(simOut, 'motorData', 24);
    km = tm > 0.5 & tm < tm(end) - 0.5;
    em = rad2deg(md(km,1:4:24) - interp1(ref.t, ref.q, tm(km)));
    row = table(rms(ep), rms(ea), rms(em(:)), max(abs(md(:,4:4:24)), [], 'all'), ...
        'VariableNames', {'pos_mm', 'ang_deg', 'motor_deg', 'peak_Nm'});
end
