function plotMotorInfo(out, jointAngles)
%PLOTMOTORINFO  Motor torque, speed and tracking from a simulation run.
%
%   Torque and speed are reported in the MOTOR frame (after the gear ratio
%   N), which is the frame the datasheet limits are quoted in.

params

nMotors = 6;

% lines() keeps the six traces distinguishable; the old 'y' was invisible
% against a white axes.
colors = lines(nMotors);
labels = arrayfun(@(i) sprintf('Motor %d', i), 1:nMotors, 'UniformOutput', false);

% retrieve simulation data
[sim_time, data] = getSimSignal(out, 'motorData', 24);

% motorData contains 24 values:
% motor 1: q w b t
% motor 2: q w b t
% motor 3: q w b t
% motor 4: q w b t
% motor 5: q w b t
% motor 6: q w b t
thetas    = data(:, (2:4:22)-1);
omegas    = data(:, 2:4:22);
alphas    = data(:, (2:4:22)+1); %#ok<NASGU>
taus_load = data(:, (2:4:22) + 2);

omegas_rad_motor = omegas * N;   % Angular velocity in rad/s in motor frame
tau_motor        = taus_load / N; % torque in motor frame
power_motor      = tau_motor .* omegas_rad_motor;

% RMS torque for each motor
tau_motor_rms = sqrt(mean(tau_motor.^2, 1)); %#ok<NASGU>

omegas_rpm_motor = omegas * (60 / (2 * pi)) * N;

% Compute max values
max_power = max(max(abs(power_motor)));
max_tau   = max(max(abs(tau_motor)));
max_rad   = max(max(abs(omegas_rad_motor)));

% Bench test - at 36 V, max RPM(motor frame) =  981 rpm -> 103 rad/s
% Kt = 36/103  = 36/103 Nm/A = 36/103 V/ (rad/s)
Kt = 0.28; % Nm/A from datasheet to be on the safe side.
max_current = max_tau / Kt;

% Print results
fprintf('Max Power: %.6f W\n', max_power);
fprintf('Max Dynamic Torque: %.6f Nm \n', max_tau);
fprintf('Max RPM (Motor Frame): %.6f RPM\n', max_rad * 9.549297);
fprintf('Max Current: %.6f A\n', max_current);

% =====================================================================
% TORQUE against speed, time and angle
%
% Same six traces and the same peak/rated limit lines each time - only the
% x axis changes, so they go through one routine.
% =====================================================================
plotTorqueVs(omegas_rpm_motor, tau_motor, colors, labels, ...
    'Angular Velocity (RPM)', 'Torque (Nm) vs Angular Velocity (RPM)', ...
    peak_torque, rated_torque);

plotTorqueVs(repmat(sim_time, 1, nMotors), tau_motor, colors, labels, ...
    'Time (s)', 'Torque (Nm) vs Time (s)', ...
    peak_torque, rated_torque);

plotTorqueVs(rad2deg(thetas), tau_motor, colors, labels, ...
    'Theta (degrees)', 'Torque (Nm) vs Theta (degrees)', ...
    peak_torque, rated_torque);

% =====================================================================
% COMMANDED JOINT ANGLE against simulated joint angle
% =====================================================================
figure('Name', 'Joint Angles vs Motor Thetas', 'NumberTitle', 'off');
tiledlayout(3, 2, 'Padding', 'compact', 'TileSpacing', 'compact');

ax = gobjects(1, nMotors);

for i = 1:nMotors

    ax(i) = nexttile;
    hold(ax(i), 'on');

    % jointAngles(:,1) is time, jointAngles(:,2:7) are joints 1 through 6
    hCmd = plot(ax(i), jointAngles(:,1), rad2deg(jointAngles(:, i+1)), ...
        'k--', 'LineWidth', 1.2);

    % Simscape revolute joint theta
    hSim = plot(ax(i), sim_time, rad2deg(thetas(:, i)), ...
        'b', 'LineWidth', 1.2);

    title(ax(i), sprintf('Joint %d', i));
    xlabel(ax(i), 'Time (s)');
    ylabel(ax(i), 'Angle (deg)');

    grid(ax(i), 'on');
    box(ax(i), 'on');
end

% The same two traces appear on all six tiles, so label them once.
lg = legend([hCmd hSim], {'Commanded', 'Simscape'});
lg.Layout.Tile = 'north';
lg.Orientation = 'horizontal';

linkaxes(ax, 'x');

end


function plotTorqueVs(x, tau, colors, labels, xlab, figName, peak, rated)
%PLOTTORQUEVS  Six motor torque traces against a shared x quantity.
%
%   x - N x 6, one column per motor (repmat a shared vector if needed)

    figure('Name', figName, 'NumberTitle', 'off');

    ax = axes;
    hold(ax, 'on');

    h = gobjects(1, size(tau, 2));

    for i = 1:size(tau, 2)
        h(i) = plot(ax, x(:,i), tau(:,i), 'Color', colors(i,:), 'LineWidth', 1.0);
    end

    torqueLimits(ax, peak, rated);

    % Six traces with no other way to tell them apart - the legend earns its place.
    legend(ax, h, labels, 'Location', 'northwest');

    xlabel(ax, xlab);
    ylabel(ax, 'Torque (Nm)');
    title(ax, figName);

    grid(ax, 'on');
    box(ax, 'on');
end


function torqueLimits(ax, peak, rated)
%TORQUELIMITS  Datasheet peak and rated torque bands.
    yline(ax,  peak,  '-r', 'T-motor-peak', ...
        'HandleVisibility', 'off', 'LabelVerticalAlignment', 'top');

    yline(ax,  rated, '-g', 'T-motor-rated', ...
        'HandleVisibility', 'off', 'LabelVerticalAlignment', 'top');

    yline(ax, -rated, '-g', 'T-motor-rated', ...
        'HandleVisibility', 'off', 'LabelVerticalAlignment', 'bottom');

    yline(ax, -peak,  '-r', 'T-motor-peak', ...
        'HandleVisibility', 'off', 'LabelVerticalAlignment', 'bottom');
end
