function plotPlatformInfo(simOut, trajectory, sp)
%PLOTPLATFORMINFO  Compare simulated platform motion against the setpoint.
%
%   Units:
%       position       m        angles                deg
%       velocity       m/s      angular velocity      deg/s
%       acceleration   m/s^2    angular acceleration  deg/s^2
%
%   Position error stays in mm and angular error in deg - those are the
%   scales the numbers are actually read at.
%
%   Z acceleration carries a second axis in g, since the vertical load is
%   the quantity that gets quoted that way.

    G = 9.80665;

    C    = [1 0 0; 0 .6 0; 0 0 1];
    xyz  = {'X','Y','Z'};
    rxyz = {'R_x','R_y','R_z'};

    % Retrieve logged platform data
    [t, data] = getSimSignal(simOut, 'platform_pose', 19);

    % ---------------------------------------------------------------------
    % Logged platform_pose layout: 19 signals
    % ---------------------------------------------------------------------
    Q = data(:,1:4);        % quaternion
    w = data(:,5:7);        % angular velocity
    b = data(:,8:10);       % angular acceleration

    p = data(:,11:13);      % position
    v = data(:,14:16);      % linear velocity
    a = data(:,17:19);      % linear acceleration

    % Convert simulated quaternion to Euler angles
    r = unwrap(quat2eul(Q, 'XYZ'), [], 1);

    % ---------------------------------------------------------------------
    % Trajectory setpoints, interpolated onto the simulation timestamps
    % ---------------------------------------------------------------------
    tr = trajectory(:,1);

    pr0 = [trajectory(:,2:3), trajectory(:,4) + sp.homez(3)];
    rr0 = unwrap(trajectory(:,5:7), [], 1);

    pr = spline(tr, pr0.', t.').';
    rr = spline(tr, rr0.', t.').';

    % ---------------------------------------------------------------------
    % Errors
    % ---------------------------------------------------------------------
    ep = 1000 * (p - pr);                       % mm
    er = rad2deg(mod(r - rr + pi, 2*pi) - pi);  % deg

    rmsP = sqrt(mean(ep.^2, 1));
    rmsR = sqrt(mean(er.^2, 1));

    % =====================================================================
    % PLATFORM POSE - setpoint against simulation
    % =====================================================================
    figure('Name','Platform Pose','NumberTitle','off');
    tiledlayout(3,2,'Padding','compact','TileSpacing','compact');

    ax = gobjects(1,6);

    for k = 1:3

        ax(2*k-1) = nexttile(2*k-1);
        hold(ax(2*k-1), 'on');
        plot(ax(2*k-1), t, pr(:,k), '--', 'Color', C(k,:), 'LineWidth', 1.2);
        plot(ax(2*k-1), t,  p(:,k), '-',  'Color', C(k,:), 'LineWidth', 1.2);
        decorate(ax(2*k-1), [xyz{k} ' Position'], 'm');

        ax(2*k) = nexttile(2*k);
        hold(ax(2*k), 'on');
        plot(ax(2*k), t, rad2deg(rr(:,k)), '--', 'Color', C(k,:), 'LineWidth', 1.2);
        plot(ax(2*k), t, rad2deg( r(:,k)), '-',  'Color', C(k,:), 'LineWidth', 1.2);
        decorate(ax(2*k), rxyz{k}, 'deg');
    end

    % Setpoint/Simscape is the same pair on every tile, so label it once.
    % Colour already means the axis (X red, Y green, Z blue) and line style
    % means setpoint vs simulation, so the legend gets neutral style keys
    % rather than borrowing one axis's colour and reading as Z-specific.
    hKeySet = plot(ax(1), NaN, NaN, '--', 'Color', [0 0 0], 'LineWidth', 1.2);
    hKeySim = plot(ax(1), NaN, NaN, '-',  'Color', [0 0 0], 'LineWidth', 1.2);

    lg = legend([hKeySet hKeySim], {'Setpoint','Simscape'});
    lg.Layout.Tile = 'north';
    lg.Orientation = 'horizontal';

    linkaxes(ax, 'x');

    % =====================================================================
    % PLATFORM POSE ERROR
    % =====================================================================
    figure('Name','Platform Pose Error','NumberTitle','off');
    tiledlayout(3,2,'Padding','compact','TileSpacing','compact');

    ax = gobjects(1,6);

    for k = 1:3

        ax(2*k-1) = nexttile(2*k-1);
        plot(ax(2*k-1), t, ep(:,k), 'Color', C(k,:), 'LineWidth', 1.2);
        decorate(ax(2*k-1), ...
            sprintf('%s Error | RMS %.4f mm', xyz{k}, rmsP(k)), 'mm');

        ax(2*k) = nexttile(2*k);
        plot(ax(2*k), t, er(:,k), 'Color', C(k,:), 'LineWidth', 1.2);
        decorate(ax(2*k), ...
            sprintf('%s Error | RMS %.4f deg', rxyz{k}, rmsR(k)), 'deg');
    end

    linkaxes(ax, 'x');

    % =====================================================================
    % PLATFORM VELOCITY
    % =====================================================================
    figure('Name','Platform Velocity','NumberTitle','off');
    tiledlayout(3,2,'Padding','compact','TileSpacing','compact');

    ax = gobjects(1,6);

    for k = 1:3

        ax(2*k-1) = nexttile(2*k-1);
        plot(ax(2*k-1), t, v(:,k), 'Color', C(k,:), 'LineWidth', 1.2);
        decorate(ax(2*k-1), [xyz{k} ' Velocity'], 'm/s');

        ax(2*k) = nexttile(2*k);
        plot(ax(2*k), t, rad2deg(w(:,k)), 'Color', C(k,:), 'LineWidth', 1.2);
        decorate(ax(2*k), [rxyz{k} ' Angular Velocity'], 'deg/s');
    end

    linkaxes(ax, 'x');

    % =====================================================================
    % PLATFORM ACCELERATION
    % =====================================================================
    figure('Name','Platform Acceleration','NumberTitle','off');
    tiledlayout(3,2,'Padding','compact','TileSpacing','compact');

    ax = gobjects(1,6);

    for k = 1:3

        ax(2*k-1) = nexttile(2*k-1);
        plot(ax(2*k-1), t, a(:,k), 'Color', C(k,:), 'LineWidth', 1.2);
        decorate(ax(2*k-1), [xyz{k} ' Acceleration'], 'm/s^2');

        % Z carries the vertical load, which is read in g.
        if k == 3
            addGAxis(ax(2*k-1), t, a(:,k), G);
        end

        ax(2*k) = nexttile(2*k);
        plot(ax(2*k), t, rad2deg(b(:,k)), 'Color', C(k,:), 'LineWidth', 1.2);
        decorate(ax(2*k), [rxyz{k} ' Angular Acceleration'], 'deg/s^2');
    end

    linkaxes(ax, 'x');

    % =====================================================================
    % PRINT RMS RESULTS
    % =====================================================================
    fprintf('\nRMS position error [X Y Z] mm: %.4f  %.4f  %.4f\n', rmsP);
    fprintf('RMS angular error [Rx Ry Rz] deg: %.4f  %.4f  %.4f\n\n', rmsR);

end


function decorate(ax, ttl, ylab)
%DECORATE  Common tile furniture: title, labels, grid, box.
    title(ax,  ttl);
    ylabel(ax, ylab);
    xlabel(ax, 'Time (s)');
    grid(ax, 'on');
    box(ax, 'on');
end


function addGAxis(ax, t, y, G)
%ADDGAXIS  Give an m/s^2 tile a matching g scale on the right.
%
%   Replots the same trace scaled by 1/g with the line hidden, so the right
%   axis autoscales in lockstep with the left one instead of needing its
%   limits kept in sync by hand.
    yyaxis(ax, 'right');
    plot(ax, t, y / G, 'LineStyle', 'none', 'HandleVisibility', 'off');
    ylabel(ax, 'g');
    ax.YAxis(2).Color = [0 0 0];
    yyaxis(ax, 'left');
end
