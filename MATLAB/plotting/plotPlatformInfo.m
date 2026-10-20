function plotPlatformInfo(simOut, ref, robot)
    % simulated platform pose vs ref.x, error after taking out the delay (one lag for all six axes,
    % estimateDelay), velocity and acceleration; m, deg, error in mm and deg, z acceleration also in g
    C = [1 0 0; 0 .6 0; 0 0 1];
    xyz = {'X', 'Y', 'Z'};
    rxyz = {'R_x', 'R_y', 'R_z'};

    [t, data] = getSimSignal(simOut, 'platform_pose', 19);
    Q = data(:,1:4);    % quaternion
    w = data(:,5:7);    % angular velocity
    b = data(:,8:10);   % angular acceleration
    p = data(:,11:13);  % position
    v = data(:,14:16);  % velocity
    a = data(:,17:19);  % acceleration
    r = unwrap(quat2eul(Q, 'XYZ'), [], 1);

    tr = ref.t;
    pr0 = [ref.x(:,1:2), ref.x(:,3) + robot.geom.home(3)];
    rr0 = unwrap(ref.x(:,4:6), [], 1);
    pr = spline(tr, pr0.', t.').';
    rr = spline(tr, rr0.', t.').';

    [d, dAxis, yd] = estimateDelay(t, [p r], tr, [pr0 rr0], 0.01); % setpoint shifted by the common lag
    ep = 1000*(p - yd(:,1:3));                          % mm
    er = rad2deg(mod(r - yd(:,4:6) + pi, 2*pi) - pi);   % deg
    rmsP = sqrt(mean(ep.^2, 1));
    rmsR = sqrt(mean(er.^2, 1));

    % pose
    figure('Name', 'Platform Pose', 'NumberTitle', 'off');
    tiledlayout(3, 2, 'Padding', 'compact', 'TileSpacing', 'compact');
    ax = gobjects(1, 6);
    for k = 1:3
        ax(2*k-1) = nexttile(2*k-1);
        hold(ax(2*k-1), 'on');
        plot(ax(2*k-1), t, pr(:,k), '--', 'Color', C(k,:), 'LineWidth', 1.2);
        plot(ax(2*k-1), t, p(:,k), '-', 'Color', C(k,:), 'LineWidth', 1.2);
        decorateAxes(ax(2*k-1), [xyz{k} ' Position'], 'm');
        ax(2*k) = nexttile(2*k);
        hold(ax(2*k), 'on');
        plot(ax(2*k), t, rad2deg(rr(:,k)), '--', 'Color', C(k,:), 'LineWidth', 1.2);
        plot(ax(2*k), t, rad2deg(r(:,k)), '-', 'Color', C(k,:), 'LineWidth', 1.2);
        decorateAxes(ax(2*k), rxyz{k}, 'deg');
    end
    hSet = plot(ax(1), NaN, NaN, '--', 'Color', [0 0 0], 'LineWidth', 1.2); % black keys, colour already means the axis
    hSim = plot(ax(1), NaN, NaN, '-', 'Color', [0 0 0], 'LineWidth', 1.2);
    lg = legend([hSet hSim], {'Setpoint', 'Simscape'});
    lg.Layout.Tile = 'north';
    lg.Orientation = 'horizontal';
    linkaxes(ax, 'x');

    % pose error
    figure('Name', 'Platform Pose Error', 'NumberTitle', 'off');
    tl = tiledlayout(3, 2, 'Padding', 'compact', 'TileSpacing', 'compact');
    title(tl, sprintf('Platform Pose Error | %.3f ms lead added', 1000*d));
    ax = gobjects(1, 6);
    for k = 1:3
        ax(2*k-1) = nexttile(2*k-1);
        plot(ax(2*k-1), t, ep(:,k), 'Color', C(k,:), 'LineWidth', 1.2);
        decorateAxes(ax(2*k-1), sprintf('%s Error | RMS %.4f mm', xyz{k}, rmsP(k)), 'mm');
        ax(2*k) = nexttile(2*k);
        plot(ax(2*k), t, er(:,k), 'Color', C(k,:), 'LineWidth', 1.2);
        decorateAxes(ax(2*k), sprintf('%s Error | RMS %.4f deg', rxyz{k}, rmsR(k)), 'deg');
    end
    linkaxes(ax, 'x');

    % velocity
    figure('Name', 'Platform Velocity', 'NumberTitle', 'off');
    tiledlayout(3, 2, 'Padding', 'compact', 'TileSpacing', 'compact');
    ax = gobjects(1, 6);
    for k = 1:3
        ax(2*k-1) = nexttile(2*k-1);
        plot(ax(2*k-1), t, v(:,k), 'Color', C(k,:), 'LineWidth', 1.2);
        decorateAxes(ax(2*k-1), [xyz{k} ' Velocity'], 'm/s');
        ax(2*k) = nexttile(2*k);
        plot(ax(2*k), t, rad2deg(w(:,k)), 'Color', C(k,:), 'LineWidth', 1.2);
        decorateAxes(ax(2*k), [rxyz{k} ' Angular Velocity'], 'deg/s');
    end
    linkaxes(ax, 'x');

    % acceleration
    figure('Name', 'Platform Acceleration', 'NumberTitle', 'off');
    tiledlayout(3, 2, 'Padding', 'compact', 'TileSpacing', 'compact');
    ax = gobjects(1, 6);
    for k = 1:3
        ax(2*k-1) = nexttile(2*k-1);
        plot(ax(2*k-1), t, a(:,k), 'Color', C(k,:), 'LineWidth', 1.2);
        decorateAxes(ax(2*k-1), [xyz{k} ' Acceleration'], 'm/s^2');
        if k == 3
            addGAxis(ax(2*k-1), t, a(:,k), robot.g);
        end
        ax(2*k) = nexttile(2*k);
        plot(ax(2*k), t, rad2deg(b(:,k)), 'Color', C(k,:), 'LineWidth', 1.2);
        decorateAxes(ax(2*k), [rxyz{k} ' Angular Acceleration'], 'deg/s^2');
    end
    linkaxes(ax, 'x');

    fprintf('\nRMS position error [X Y Z] mm: %.4f  %.4f  %.4f\n', rmsP);
    fprintf('RMS angular error [Rx Ry Rz] deg: %.4f  %.4f  %.4f\n', rmsR);
    fprintf('Delay [X Y Z Rx Ry Rz] ms: %.3f  %.3f  %.3f  %.3f  %.3f  %.3f | lead added %.3f\n\n', 1000*dAxis, 1000*d);
end
