% INVERSE_DYNAMICS_MAIN Estimate and compare all six crank torques.
% Run this script from MATLAB. Edit these settings or the separate profiles
% in inverseDynamicsProfiles.m. The existing models and params.m are not edited.
% Results: idResults, idSummary, idMotorSummary, and output/inverse_dynamics/.
% Torque units are N*m at the CRANK/output shaft; motor torque = crank/N.

idMode = 'syn';
idCycles = 30;
idRunSimulations = true;     % false: analytical estimates only (much faster)
idMakePlots = true;
idSaveResults = true;

idCodeDir = fileparts(mfilename('fullpath'));
idRepoRoot = fileparts(idCodeDir);
run(fullfile(idRepoRoot,'startup.m'));
params;
sp = StuartPlatform(r, n, rB, dB, rP, dP);
idProfiles = inverseDynamicsProfiles(sp);

% Explicit data-file path: startup also adds Backup to the MATLAB path, and
% simulationFiles/Hexapod_DataFile.m only defines Sig, not the CAD assembly.
run(fullfile(idRepoRoot, 'Backup', 'Hexapod_DataFile.m'));
run(fullfile(idRepoRoot, 'simulationFiles', 'Hexapod_DataFile.m'));

idTorso = generateIDTrajectory(idMode, idCycles, idRepoRoot);
idTrajectory = rigid_transform(idTorso, [0; 0; 0.1]);
idTrajectory = warpTrajectoryClock(idTrajectory, riseTime, fallTime, 'spline');
idTrajectory = taperTrajectory(idTrajectory, riseTime, fallTime);
jointAngles = sp.move(idTrajectory);
time = jointAngles(:,1);
tf = time(end);
u = jointAngles(:,2:7);
du = periodicDerivative5pt(u, dt);

idResults = struct();
idSummary = table();
idMotorSummary = table();
for idIndex = 1:numel(idProfiles)
    idProfile = idProfiles(idIndex);
    idName = idProfile.name;
    fprintf('\n%s: platform %.3f kg; each rod %.3f kg; each arm %.3f kg; payload %.3f kg\n', ...
        idName, idProfile.platformMass, mean(idProfile.rodMass), ...
        mean(idProfile.armMass), idProfile.payloadMass);

    idCommand = inverseDynamics(idTrajectory, sp, idProfile);
    idResult = struct('profile', idProfile, 'command', idCommand);
    idMethodDifference = max(abs(idCommand.torqueLagrange - ...
        idCommand.torqueNewtonEuler), [], 'all');
    fprintf('Command peak crank torque %.3f N*m; Lagrange/NE max difference %.3g N*m\n', ...
        max(abs(idCommand.torqueNewtonEuler), [], 'all'), idMethodDifference);

    if idRunSimulations
        fprintf('Running %s ...\n', idName);
        idTimer = tic;
        % Keep each result: unlike debug_main, no simulation overwrites the
        % result of another model. Run-only options do not change saved SLX.
        idSim = sim(fullfile(idRepoRoot, 'simulationFiles', [idName '.slx']), ...
            'StopTime', num2str(tf, 17), 'ReturnWorkspaceOutputs', 'on');
        idRuntime = toc(idTimer);
        [idSimTime, idMotorData] = getSimSignal(idSim, 'motorData', 24);
        idSimTorque = idMotorData(:,4:4:24);
        idResult.simulationTime = idSimTime;
        idResult.simulationTorque = idSimTorque;
        idResult.simulationMotorData = idMotorData;
        idResult.simulationRuntime = idRuntime;

        % PD filtering changes the executed motion. Evaluate the achieved
        % platform motion as well, so controller lag is not mistaken for an
        % inverse-dynamics error. The command prediction above remains the
        % answer to "what torque does my requested trajectory need?".
        [idPoseTime, idPoseData] = getSimSignal(idSim, 'platform_pose', 19);
        [idPoseTime, idUnique] = unique(idPoseTime, 'stable');
        idPoseData = idPoseData(idUnique,:);
        idEuler = unwrap(quat2eul(idPoseData(:,1:4), 'XYZ'), [], 1);
        idActual = [time, interp1(idPoseTime, ...
            [idPoseData(:,11:13) - sp.homez.', idEuler], time, 'pchip')];
        % All three Transform Sensors use MeasurementFrame=World. Their
        % recorded derivatives avoid spikes from differentiating resampled
        % variable-step pose data. No simulated torque is an input to ID.
        idRates = interp1(idPoseTime, idPoseData(:,[14:19 5:10]), time, 'linear');
        % Each saved platform_pose logger's "g" subsystem adds +g to Z.
        % Undo that logging bias: the dynamics engine applies gravity itself.
        idRates(:,6) = idRates(:,6) - g;
        idKinematics = struct('linearVelocity',idRates(:,1:3), ...
            'linearAcceleration',idRates(:,4:6), 'angularVelocity',idRates(:,7:9), ...
            'angularAcceleration',idRates(:,10:12));
        idAchieved = inverseDynamics(idActual, sp, idProfile, idKinematics);
        idResult.achieved = idAchieved;
        [idSimTimeUnique, idUnique] = unique(idSimTime, 'stable');
        idReference = interp1(idSimTimeUnique, idSimTorque(idUnique,:), time, 'linear');

        % Use the entire achieved-motion record, including ramp transients.
        idInterior = true(size(time));
        idError = idAchieved.torqueNewtonEuler(idInterior,:) - idReference(idInterior,:);
        idReferenceRMS = sqrt(mean(idReference(idInterior,:).^2, 1));
        idErrorRMS = sqrt(mean(idError.^2, 1));
        idRelativeError = 100 * idErrorRMS ./ max(idReferenceRMS, eps);
        idSimPeak = max(abs(idSimTorque), [], 1);
        idCommandPeak = max(abs(idCommand.torqueNewtonEuler), [], 1);
        idAchievedPeak = max(abs(idAchieved.torqueNewtonEuler), [], 1);
        idMethodDifference = max(idMethodDifference, max(abs( ...
            idAchieved.torqueLagrange - idAchieved.torqueNewtonEuler), [], 'all'));

        idRow = table(string(idName), max(idSimPeak), max(idCommandPeak), ...
            max(idAchievedPeak), mean(idRelativeError), idMethodDifference, idRuntime, ...
            'VariableNames', {'Model','SimPeakNm','CommandIDPeakNm','AchievedIDPeakNm', ...
            'MeanMotorNRMSEPercent','MethodDifferenceNm','RuntimeSeconds'});
        idMotorRows = table(repmat(string(idName),6,1), (1:6).', ...
            idSimPeak.', idCommandPeak.', idAchievedPeak.', idErrorRMS.', idRelativeError.', ...
            'VariableNames', {'Model','Motor','SimPeakNm','CommandIDPeakNm', ...
            'AchievedIDPeakNm','RMSE_Nm','NRMSEPercent'});
        idSummary = [idSummary; idRow]; %#ok<AGROW>
        idMotorSummary = [idMotorSummary; idMotorRows]; %#ok<AGROW>
        fprintf('Sim peak %.3f; achieved-motion ID peak %.3f N*m; mean motor NRMSE %.1f%%\n', ...
            max(idSimPeak), max(idAchievedPeak), mean(idRelativeError));
    end

    idResults.(idName) = idResult;
    if idMakePlots
        idFigure = plotIDComparison(idName, idResult);
        if idSaveResults
            idOutputDir = fullfile(fileparts(mfilename('fullpath')), 'output', 'inverse_dynamics');
            if ~isfolder(idOutputDir), mkdir(idOutputDir); end
            exportgraphics(idFigure, fullfile(idOutputDir, [idName '.png']), 'Resolution', 140);
        end
    end
end

fprintf('\nAll torques below are crank/output-shaft N*m. Divide by N=%g for ideal motor torque.\n', N);
disp(idSummary);
if idSaveResults
    idOutputDir = fullfile(fileparts(mfilename('fullpath')), 'output', 'inverse_dynamics');
    if ~isfolder(idOutputDir), mkdir(idOutputDir); end
    save(fullfile(idOutputDir, 'results.mat'), 'idResults', 'idSummary', ...
        'idMotorSummary', 'idProfiles', 'idTrajectory', 'idMode', 'idCycles');
    if ~isempty(idSummary)
        writetable(idSummary, fullfile(idOutputDir, 'summary.csv'));
        writetable(idMotorSummary, fullfile(idOutputDir, 'per_motor.csv'));
    end
    fprintf('Saved results to %s\n', idOutputDir);
end

function trajectory = generateIDTrajectory(mode, cycles, repoRoot)
    % The shared camera generator reads CSV paths relative to the repo root.
    previousFolder = pwd;
    restoreFolder = onCleanup(@() cd(previousFolder));
    cd(repoRoot);
    trajectory = genTrajectory(mode,cycles);
end

function fig = plotIDComparison(name, result)
    fig = figure('Name', [name ' inverse dynamics'], 'NumberTitle', 'off', ...
        'Position', [100 100 1200 760], 'Color', 'w');
    tiles = tiledlayout(fig, 3, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
    hasSimulation = isfield(result, 'achieved');
    if hasSimulation
        estimate = result.achieved;
        subtitle(tiles, 'ID uses achieved motion; gray trace predicts the requested motion');
    else
        estimate = result.command;
        subtitle(tiles, 'Requested motion; analytical estimates only');
    end
    title(tiles, [name ' | crank/output-shaft torque (N m)'], 'Interpreter', 'none');
    ax = gobjects(6,1);
    for k = 1:6
        ax(k) = nexttile(tiles);
        hold(ax(k), 'on');
        if hasSimulation
            plot(ax(k), result.command.time, result.command.torqueNewtonEuler(:,k), ...
                'Color', [.72 .72 .72], 'DisplayName', 'ID: requested motion');
            plot(ax(k), result.simulationTime, result.simulationTorque(:,k), ...
                'k-', 'LineWidth', 1, 'DisplayName', 'Simscape');
        end
        plot(ax(k), estimate.time, estimate.torqueLagrange(:,k), ...
            'Color', [0 .35 .75], 'LineWidth', 1.2, 'DisplayName', 'Lagrange');
        plot(ax(k), estimate.time, estimate.torqueNewtonEuler(:,k), ...
            '--', 'Color', [.85 .25 .08], 'LineWidth', 1, 'DisplayName', 'Newton-Euler');
        title(ax(k), sprintf('Motor %d', k));
        xlabel(ax(k), 'Time (s)'); ylabel(ax(k), 'Crank torque (N m)'); grid(ax(k), 'on');
    end
    linkaxes(ax, 'x');
    lg = legend(ax(1), 'Location', 'northoutside', 'Orientation', 'horizontal');
    lg.Layout.Tile = 'north';
end
