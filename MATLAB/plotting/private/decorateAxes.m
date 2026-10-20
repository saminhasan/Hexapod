function decorateAxes(ax, ttl, ylab)
    % title, y label, time on x, grid and box
    title(ax, ttl);
    ylabel(ax, ylab);
    xlabel(ax, 'Time (s)');
    grid(ax, 'on');
    box(ax, 'on');
end
