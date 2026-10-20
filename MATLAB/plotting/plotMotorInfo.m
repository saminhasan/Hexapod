function plotMotorInfo(simOut, ref, robot)
    % motor side torque/speed/power peaks, torque vs speed, time and angle, and commanded vs simulated angle
    N = robot.motor.N;
    colors = lines(6);
    labels = arrayfun(@(i) sprintf('Motor %d', i), 1:6, 'UniformOutput', false);

    [simTime, data] = getSimSignal(simOut, 'motorData', 24); % per motor: q qd qdd tau
    q = data(:,1:4:21);
    qd = data(:,2:4:22);
    tau = data(:,4:4:24);           % crank side

    omegaMotor = qd*N;              % rad/s, motor side
    tauMotor = tau/N;
    powerMotor = tauMotor.*omegaMotor;
    rpmMotor = qd*(60/(2*pi))*N;

    maxTau = max(abs(tauMotor), [], 'all');
    fprintf('motors in the sim: peak torque %.2f Nm (%.1f A), peak speed %.0f rpm, peak power %.0f W\n', ...
        maxTau, maxTau/robot.motor.Kt, max(abs(rpmMotor), [], 'all'), max(abs(powerMotor), [], 'all'));

    limits = [robot.motor.peakTorque, robot.motor.ratedTorque];
    plotTorqueVs(rpmMotor, tauMotor, colors, labels, 'Angular Velocity (RPM)', ...
        'Torque (Nm) vs Angular Velocity (RPM)', limits);
    plotTorqueVs(repmat(simTime, 1, 6), tauMotor, colors, labels, 'Time (s)', 'Torque (Nm) vs Time (s)', limits);
    plotTorqueVs(rad2deg(q), tauMotor, colors, labels, 'Theta (degrees)', 'Torque (Nm) vs Theta (degrees)', limits);

    figure('Name', 'Joint Angles vs Motor Thetas', 'NumberTitle', 'off');
    tiledlayout(3, 2, 'Padding', 'compact', 'TileSpacing', 'compact');
    ax = gobjects(1, 6);
    for i = 1:6
        ax(i) = nexttile;
        hold(ax(i), 'on');
        hCmd = plot(ax(i), ref.t, rad2deg(ref.q(:,i)), 'k--', 'LineWidth', 1.2);
        hSim = plot(ax(i), simTime, rad2deg(q(:,i)), 'b', 'LineWidth', 1.2);
        decorateAxes(ax(i), sprintf('Joint %d', i), 'Angle (deg)');
    end
    lg = legend([hCmd hSim], {'Commanded', 'Simscape'});
    lg.Layout.Tile = 'north';
    lg.Orientation = 'horizontal';
    linkaxes(ax, 'x');
end
