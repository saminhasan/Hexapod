function plotTrajectory(trajectories, labels, figName)
%PLOTTRAJECTORY  Plot one or more 6-DoF pose trajectories on shared axes.
%
%   plotTrajectory(traj)
%   plotTrajectory({t1, t2, ...}, {'label1', 'label2', ...})
%   plotTrajectory({t1, t2, ...}, {'label1', ...}, 'Figure name')
%
%   traj - N x 7 array [t x y z roll pitch yaw], angles in rad
%
%   Overlaying the stages of a pipeline (torso frame, platform frame, time
%   warped, tapered) on one figure makes the effect of each stage directly
%   readable, instead of forcing a flick between four windows to compare.
%   The stages may have different durations - that is the point of the plot.
%
%   A legend is drawn only when more than one trajectory is supplied. With a
%   single line the tile title already says what it is.

    % ------------------------------------------------------------- inputs
    if ~iscell(trajectories)
        trajectories = {trajectories};
    end

    nTraj = numel(trajectories);

    if nargin < 2 || isempty(labels)
        labels = {};
    elseif ~iscell(labels)
        labels = {labels};
    end

    if ~isempty(labels) && numel(labels) ~= nTraj
        error('plotTrajectory:LabelCount', ...
            'Got %d trajectories but %d labels.', nTraj, numel(labels));
    end

    if nargin < 3 || isempty(figName)
        figName = 'Trajectory';
    end

    for k = 1:nTraj
        if size(trajectories{k}, 2) ~= 7
            error('plotTrajectory:BadSize', ...
                'Trajectory %d must be N x 7 [t x y z roll pitch yaw], got N x %d.', ...
                k, size(trajectories{k}, 2));
        end
    end

    cleanupObj = useLatexInterpreters(); %#ok<NASGU>

    % --------------------------------------------------------- tile layout
    % Column 1 holds the translations, column 2 the rotations.
    %
    %   tile | column | title | ylabel | convert rad -> deg
    panels = { ...
        1, 2, 'Position $x$',   'Position $[\mathrm{m}]$', false; ...
        3, 3, 'Position $y$',   'Position $[\mathrm{m}]$', false; ...
        5, 4, 'Position $z$',   'Position $[\mathrm{m}]$', false; ...
        2, 5, 'Roll $\phi$',    'Angle $[^\circ]$',        true;  ...
        4, 6, 'Pitch $\theta$', 'Angle $[^\circ]$',        true;  ...
        6, 7, 'Yaw $\psi$',     'Angle $[^\circ]$',        true};

    f = figure('Name', figName, 'NumberTitle', 'off');
    f.Theme = 'light';

    tl = tiledlayout(3, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
    title(tl, 'Pose vs Time');

    ax    = gobjects(1, size(panels,1));
    first = gobjects(1, nTraj);   % one handle per trajectory, for the legend

    for pIdx = 1:size(panels,1)

        [tile, col, ttl, ylab, isAngle] = panels{pIdx,:};

        ax(pIdx) = nexttile(tile);
        hold(ax(pIdx), 'on');

        for k = 1:nTraj

            traj = trajectories{k};
            y    = traj(:, col);

            if isAngle
                y = rad2deg(y);
            end

            h = plot(ax(pIdx), traj(:,1), y, 'LineWidth', 1.2);

            if pIdx == 1
                first(k) = h;
            end
        end

        title(ax(pIdx),  ttl);
        xlabel(ax(pIdx), 'Time (s)');
        ylabel(ax(pIdx), ylab);

        grid(ax(pIdx), 'on');
        box(ax(pIdx), 'on');
    end

    % One legend for the whole figure, and only when it adds something the
    % tile titles do not already say.
    if nTraj > 1 && ~isempty(labels)
        lg = legend(first, labels);
        lg.Layout.Tile = 'north';
        lg.Orientation = 'horizontal';
    end

    linkaxes(ax, 'x');

end
