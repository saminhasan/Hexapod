function plotTorqueVs(x, tau, colors, labels, xlab, figName, limits)
    % six motor torques against x (one column per motor) with the datasheet [peak rated] torque lines
    figure('Name', figName, 'NumberTitle', 'off');
    ax = axes;
    hold(ax, 'on');
    h = gobjects(1, size(tau, 2));
    for i = 1:size(tau, 2)
        h(i) = plot(ax, x(:,i), tau(:,i), 'Color', colors(i,:), 'LineWidth', 1.0);
    end
    yline(ax, limits(1), '-r', 'T-motor-peak', 'HandleVisibility', 'off', 'LabelVerticalAlignment', 'top');
    yline(ax, limits(2), '-g', 'T-motor-rated', 'HandleVisibility', 'off', 'LabelVerticalAlignment', 'top');
    yline(ax, -limits(2), '-g', 'T-motor-rated', 'HandleVisibility', 'off', 'LabelVerticalAlignment', 'bottom');
    yline(ax, -limits(1), '-r', 'T-motor-peak', 'HandleVisibility', 'off', 'LabelVerticalAlignment', 'bottom');
    legend(ax, h, labels, 'Location', 'northwest');
    xlabel(ax, xlab);
    ylabel(ax, 'Torque (Nm)');
    title(ax, figName);
    grid(ax, 'on');
    box(ax, 'on');
end
