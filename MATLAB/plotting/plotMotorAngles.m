function plotMotorAngles(t, q)
    % crank angles q (N x 6, rad) against t, one tile per motor
    colors = lines(6);
    f = figure('Name', 'Motor Angles (degrees) vs Time (s)', 'NumberTitle', 'off');
    f.Theme = 'light';
    tiledlayout(3, 2, 'Padding', 'compact', 'TileSpacing', 'compact');
    ax = gobjects(1, 6);
    for i = 1:6
        ax(i) = nexttile;
        plot(ax(i), t, rad2deg(q(:,i)), 'Color', colors(i,:), 'LineWidth', 1.2);
        decorateAxes(ax(i), sprintf('Motor %d', i), 'Angle (deg)');
    end
    linkaxes(ax, 'x');
end
