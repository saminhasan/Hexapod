function plotTrajectory(trajectories, labels, figName)
    % one or more [t x y z Rx Ry Rz] trajectories overlaid (cell array), translations left, rotations right;
    % handy for comparing pipeline stages, legend only when there is more than one
    if ~iscell(trajectories)
        trajectories = {trajectories};
    end
    if nargin < 2 || isempty(labels)
        labels = {};
    elseif ~iscell(labels)
        labels = {labels};
    end
    if nargin < 3 || isempty(figName)
        figName = 'Trajectory';
    end
    nTraj = numel(trajectories);
    assert(isempty(labels) || numel(labels) == nTraj, 'got %d trajectories but %d labels', nTraj, numel(labels))
    assert(all(cellfun(@(x) size(x, 2) == 7, trajectories)), 'trajectories must be N x 7 [t x y z Rx Ry Rz]')

    cleanupObj = useLatexInterpreters(); %#ok<NASGU>
    % tile, column, title, ylabel, rad -> deg
    panels = {
        1, 2, 'Position $x$', 'Position $[\mathrm{m}]$', false
        3, 3, 'Position $y$', 'Position $[\mathrm{m}]$', false
        5, 4, 'Position $z$', 'Position $[\mathrm{m}]$', false
        2, 5, 'Roll $\phi$', 'Angle $[^\circ]$', true
        4, 6, 'Pitch $\theta$', 'Angle $[^\circ]$', true
        6, 7, 'Yaw $\psi$', 'Angle $[^\circ]$', true};

    f = figure('Name', figName, 'NumberTitle', 'off');
    f.Theme = 'light';
    tl = tiledlayout(3, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
    title(tl, 'Pose vs Time');
    ax = gobjects(1, size(panels, 1));
    first = gobjects(1, nTraj);
    for i = 1:size(panels, 1)
        [tile, col, ttl, ylab, isAngle] = panels{i,:};
        ax(i) = nexttile(tile);
        hold(ax(i), 'on');
        for k = 1:nTraj
            y = trajectories{k}(:,col);
            if isAngle
                y = rad2deg(y);
            end
            h = plot(ax(i), trajectories{k}(:,1), y, 'LineWidth', 1.2);
            if i == 1
                first(k) = h;
            end
        end
        decorateAxes(ax(i), ttl, ylab);
    end
    if nTraj > 1 && ~isempty(labels)
        lg = legend(first, labels);
        lg.Layout.Tile = 'north';
        lg.Orientation = 'horizontal';
    end
    linkaxes(ax, 'x');
end
