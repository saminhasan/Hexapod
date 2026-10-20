function addGAxis(ax, t, y, G)
    % right-hand axis in g for an m/s^2 tile: same trace / G with no line, so both sides autoscale together
    yyaxis(ax, 'right');
    plot(ax, t, y / G, 'LineStyle', 'none', 'HandleVisibility', 'off');
    ylabel(ax, 'g');
    ax.YAxis(2).Color = [0 0 0];
    yyaxis(ax, 'left');
end
