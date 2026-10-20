function plotImuRun(L, pad, name, outDir)
    % measured vs sim torso imu of one run, time since PLAY, saved as png
    ch = {'a_x (up)', 'a_y', 'a_z (out)', '\omega_x (up)', '\omega_y', '\omega_z (out)'};
    un = {'m/s^2', 'm/s^2', 'm/s^2', 'deg/s', 'deg/s', 'deg/s'};
    fig = figure('Name', "IMU " + name, 'NumberTitle', 'off', 'Visible', 'off', 'Position', [50 50 1400 800]);
    tl = tiledlayout(3, 2, 'Padding', 'compact', 'TileSpacing', 'compact');
    title(tl, sprintf('Torso IMU %s | measured vs sim | capture offset %.3f s', name, L.offset));
    ax = gobjects(1,6);
    for k = 1:6
        ax(k) = nexttile(2*mod(k-1,3) + 1 + (k > 3));
        hold(ax(k), 'on');
        plot(ax(k), L.t - pad, L.meas(:,k), 'Color', [0 .45 .74]);
        plot(ax(k), L.t - pad, L.sim(:,k), '--', 'Color', [.85 .33 .1]);
        title(ax(k), sprintf('%s | r %.3f | rmse %.2f', ch{k}, L.r(k), L.rmse(k)));
        ylabel(ax(k), un{k}); xlabel(ax(k), 'time since PLAY (s)'); grid(ax(k), 'on'); box(ax(k), 'on');
    end
    legend(ax(1), {'measured', 'sim'});
    linkaxes(ax, 'x');
    exportgraphics(fig, fullfile(outDir, "imu_" + name + ".png"));
    close(fig);
end
