function fig = plotFold(tp, F, grp, overlay, ttl, ch, un)
    % every loop of every run stacked on the loop clock, coloured by condition
    % F(phase, channel, loop); overlay: {data, style, label} rows drawn on top (means, ref, sim)
    fig = figure('Name', ttl, 'NumberTitle', 'off', 'Position', [50 50 1400 800]);
    tl = tiledlayout(3, 2, 'Padding', 'compact', 'TileSpacing', 'compact');
    title(tl, ttl);
    C = containers.Map({'bra', 'nobra'}, {[.3 .6 .9], [.95 .6 .3]});
    ax = gobjects(1,6);
    for k = 1:6
        ax(k) = nexttile(k);
        hold(ax(k), 'on');
        for j = 1:size(F,3)
            plot(ax(k), tp, F(:,k,j), 'Color', [C(grp(j)) .35], 'HandleVisibility', 'off');
        end
        for j = 1:size(overlay,1)
            plot(ax(k), tp, overlay{j,1}(:,k), overlay{j,2}, 'LineWidth', 1.2, 'DisplayName', overlay{j,3});
        end
        if iscell(un), u = un{k}; else, u = un; end
        title(ax(k), ch{k}); ylabel(ax(k), u); xlabel(ax(k), 'time in loop (s)'); grid(ax(k), 'on'); box(ax(k), 'on');
    end
    plot(ax(1), NaN, NaN, 'Color', C('bra'), 'DisplayName', 'loops, bra');
    plot(ax(1), NaN, NaN, 'Color', C('nobra'), 'DisplayName', 'loops, no bra');
    lg = legend(ax(1)); lg.Layout.Tile = 'north'; lg.Orientation = 'horizontal';
    linkaxes(ax, 'x');
end
