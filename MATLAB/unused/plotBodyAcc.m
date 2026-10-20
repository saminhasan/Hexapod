function plotBodyAcc(simOut)
%PLOTBODYACC  Linear acceleration of the sternum, belt and shoulder bodies.
%
%   Reads sternum_a, belt_a and shoulder_a from the simulation output.

    bodies = {'sternum', 'belt', 'shoulder'};
    names  = {'Sternum', 'Belt', 'Shoulder'};

    G = 9.80665;

    cleanupObj = useLatexInterpreters(); %#ok<NASGU>

    % Fetch once and reuse - the two figures below plot the same signals.
    nBodies = numel(bodies);
    t = cell(1, nBodies);
    a = cell(1, nBodies);

    peakAbs  = zeros(nBodies, 3);
    peakMagG = zeros(nBodies, 1);

    for i = 1:nBodies
        [t{i}, a{i}] = getSig(simOut, [bodies{i} '_a']);
        a{i} = asNx(a{i}, t{i}, 3);

        peakAbs(i,:) = max(abs(a{i}), [], 1);
        peakMagG(i)  = max(vecnorm(a{i}, 2, 2)) / G;
    end

    % =====================================================================
    % PER-AXIS ACCELERATION
    % =====================================================================
    figure('Name', 'Body Linear Accelerations', 'NumberTitle', 'off');
    tiledlayout(nBodies, 1, 'Padding', 'compact', 'TileSpacing', 'compact');

    ax = gobjects(1, nBodies);

    for i = 1:nBodies

        ax(i) = nexttile;
        hold(ax(i), 'on');

        hx = plot(ax(i), t{i}, a{i}(:,1));
        hy = plot(ax(i), t{i}, a{i}(:,2));
        hz = plot(ax(i), t{i}, a{i}(:,3));

        title(ax(i), [names{i} ' acceleration']);
        xlabel(ax(i), 'Time (s)');
        ylabel(ax(i), '$a$ (m/s$^2$)');

        grid(ax(i), 'on');
        box(ax(i), 'on');
    end

    % Same three components on every tile - one legend covers the figure.
    lg = legend([hx hy hz], {'$a_x$', '$a_y$', '$a_z$'});
    lg.Layout.Tile = 'north';
    lg.Orientation = 'horizontal';

    linkaxes(ax, 'x');

    % =====================================================================
    % ACCELERATION MAGNITUDE
    % =====================================================================
    figure('Name', 'Body Acceleration Magnitude', 'NumberTitle', 'off');

    axMag = axes;
    hold(axMag, 'on');

    for i = 1:nBodies
        plot(axMag, t{i}, vecnorm(a{i}, 2, 2) / G, 'DisplayName', names{i});
    end

    title(axMag, 'Acceleration magnitude');
    xlabel(axMag, 'Time (s)');
    ylabel(axMag, '$|a|$ (g)');

    % Three bodies overlaid with nothing else to tell them apart.
    legend(axMag, 'Location', 'best');

    grid(axMag, 'on');
    box(axMag, 'on');

    % =====================================================================
    % PRINT PEAKS
    % =====================================================================
    fprintf('\nPeak abs acceleration [ax ay az] m/s^2:\n');
    for i = 1:nBodies
        fprintf('%8s: %.6g %.6g %.6g, peak |a| = %.6g g\n', ...
            bodies{i}, peakAbs(i,1), peakAbs(i,2), peakAbs(i,3), peakMagG(i));
    end
    fprintf('\n');

end


function [t, x] = getSig(simOut, name)
%GETSIG  Read a body signal, whichever container Simulink returned it in.
%
%   Separate from getSimSignal: these come back as timeseries/timetable via
%   simOut.get, not as the structure-with-time that getSimSignal unpacks.

    s = simOut.get(name);

    if isa(s, 'timeseries')
        t = s.Time;
        x = s.Data;

    elseif istimetable(s)
        t = seconds(s.Properties.RowTimes - s.Properties.RowTimes(1));
        x = s{:,:};

    elseif isstruct(s)
        if isfield(s, 'Time'), t = s.Time; else, t = s.time; end
        if isfield(s, 'Data'), x = s.Data; else, x = s.data; end

    else
        error('plotBodyAcc:UnsupportedSignal', ...
            'Unsupported simOut signal type for "%s"', name);
    end

    if isduration(t), t = seconds(t); end

    t = double(t(:));
    x = squeeze(x);

    if size(x,1) ~= numel(t) && size(x,2) == numel(t)
        x = x.';
    end
end


function x = asNx(x, t, n)
%ASNX  Orient a signal to N x n and keep the first n columns.

    x = squeeze(x);

    if isvector(x)
        x = x(:);
    end

    if size(x,1) ~= numel(t) && size(x,2) == numel(t)
        x = x.';
    end

    x = reshape(x, numel(t), []);

    if size(x,2) < n
        error('plotBodyAcc:TooFewColumns', ...
            'Signal has %d columns, expected at least %d', size(x,2), n);
    end

    x = x(:,1:n);
end
