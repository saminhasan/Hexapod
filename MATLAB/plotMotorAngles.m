function plotMotorAngles(motorAngles)
%PLOTMOTORANGLES  Commanded motor angle per joint.
%
%   motorAngles - N x 7 array [t q1 q2 q3 q4 q5 q6], angles in rad
%
%   One trace per tile and the tile title names it, so there is no legend.

    nMotors = 6;

    time   = motorAngles(:,1);
    colors = lines(nMotors);   % same palette as plotMotorInfo

    f = figure('Name', 'Motor Angles (degrees) vs Time (s)', ...
        'NumberTitle', 'off');
    f.Theme = 'light';

    tiledlayout(3, 2, 'Padding', 'compact', 'TileSpacing', 'compact');

    ax = gobjects(1, nMotors);

    for i = 1:nMotors

        ax(i) = nexttile;

        plot(ax(i), time, rad2deg(motorAngles(:, i+1)), ...
            'Color', colors(i,:), 'LineWidth', 1.2);

        title(ax(i), sprintf('Motor %d', i));
        xlabel(ax(i), 'Time (s)');
        ylabel(ax(i), 'Angle (deg)');

        grid(ax(i), 'on');
        box(ax(i), 'on');
    end

    linkaxes(ax, 'x');

end
