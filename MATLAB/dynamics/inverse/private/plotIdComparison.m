function fig = plotIdComparison(name, result)
    % six crank torques (N m): Lagrange vs Newton-Euler, plus Simscape and the requested-motion ID when simulated
    fig = figure('Name', [name ' inverse dynamics'], 'NumberTitle', 'off', 'Position', [100 100 1200 760], ...
        'Color', 'w');
    tiles = tiledlayout(fig, 3, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
    simulated = isfield(result, 'achieved');
    if simulated
        estimate = result.achieved;
        subtitle(tiles, 'ID uses achieved motion; gray trace predicts the requested motion');
    else
        estimate = result.command;
        subtitle(tiles, 'Requested motion; analytical estimates only');
    end
    title(tiles, [name ' | crank/output-shaft torque (N m)'], 'Interpreter', 'none');
    ax = gobjects(6, 1);
    for k = 1:6
        ax(k) = nexttile(tiles);
        hold(ax(k), 'on');
        if simulated
            plot(ax(k), result.command.t, result.command.tauNewtonEuler(:,k), 'Color', [.72 .72 .72], ...
                'DisplayName', 'ID: requested motion');
            plot(ax(k), result.simTime, result.simTau(:,k), 'k-', 'LineWidth', 1, 'DisplayName', 'Simscape');
        end
        plot(ax(k), estimate.t, estimate.tauLagrange(:,k), 'Color', [0 .35 .75], 'LineWidth', 1.2, ...
            'DisplayName', 'Lagrange');
        plot(ax(k), estimate.t, estimate.tauNewtonEuler(:,k), '--', 'Color', [.85 .25 .08], 'LineWidth', 1, ...
            'DisplayName', 'Newton-Euler');
        title(ax(k), sprintf('Motor %d', k));
        xlabel(ax(k), 'Time (s)'); ylabel(ax(k), 'Crank torque (N m)'); grid(ax(k), 'on');
    end
    linkaxes(ax, 'x');
    lg = legend(ax(1), 'Location', 'northoutside', 'Orientation', 'horizontal');
    lg.Layout.Tile = 'north';
end
