% runInverseDynamics.m
% crank torques for the reference trajectory from inverseDynamics (Lagrange and Newton-Euler), checked
% against Hexapod_dev, Hexapod_PD and SP, each with its own body model (bodyModels). Torques are crank side
% N m, motor = crank/N. Leaves results, summary, motorSummary in the workspace, files go to output/inverse_dynamics/
mode = 'syn';
cycles = 30;
runSims = true;     % false: analytical estimates only, much faster
makePlots = true;
saveResults = true;

here = fileparts(mfilename('fullpath'));
run(fullfile(fileparts(fileparts(here)), 'startup.m'));
robot = hexapodRobot();
bodies = bodyModels(robot);
ref = referenceTrajectory(robot, mode, cycles);
vc = velocityControlSetup();
outDir = fullfile(here, 'output', 'inverse_dynamics');

results = struct();
summary = table();
motorSummary = table();
for i = 1:numel(bodies)
    body = bodies(i);
    fprintf('\n%s: platform %.3f kg; each rod %.3f kg; each arm %.3f kg; torso %.3f kg\n', ...
        body.name, body.platformMass, mean(body.rodMass), mean(body.armMass), body.torsoMass);

    command = inverseDynamics(robot, body, ref.t, ref.x);
    res = struct('body', body, 'command', command);
    methodDiff = max(abs(command.tauLagrange - command.tauNewtonEuler), [], 'all');
    fprintf('Command peak crank torque %.3f N*m; Lagrange/NE max difference %.3g N*m\n', ...
        max(abs(command.tauNewtonEuler), [], 'all'), methodDiff);

    if runSims
        fprintf('Running %s ...\n', body.name);
        in = Simulink.SimulationInput(body.name).setVariable('robot', robot).setVariable('ref', ref) ...
            .setVariable('vc', vc);
        tic; out = sim(in); runtime = toc;
        [simTime, motorData] = getSimSignal(out, 'motorData', 24);
        simTau = motorData(:,4:4:24);
        res.simTime = simTime;
        res.simTau = simTau;
        res.simMotorData = motorData;
        res.simRuntime = runtime;

        % the loops change the motion, so also run ID on what the platform actually did, from the logged
        % world-frame rates rather than differentiating variable-step poses
        [poseTime, poseData] = getSimSignal(out, 'platform_pose', 19);
        [poseTime, keep] = unique(poseTime, 'stable');
        poseData = poseData(keep,:);
        euler = unwrap(quat2eul(poseData(:,1:4), 'XYZ'), [], 1);
        actual = interp1(poseTime, [poseData(:,11:13) - robot.geom.home.', euler], ref.t, 'pchip');
        rates = interp1(poseTime, poseData(:,[14:19 5:10]), ref.t, 'linear');
        rates(:,6) = rates(:,6) - robot.g; % the platform_pose loggers add +g to z
        kin = struct('linearVelocity', rates(:,1:3), 'linearAcceleration', rates(:,4:6), ...
            'angularVelocity', rates(:,7:9), 'angularAcceleration', rates(:,10:12));
        achieved = inverseDynamics(robot, body, ref.t, actual, kin);
        res.achieved = achieved;
        [simTimeU, keep] = unique(simTime, 'stable');
        simOnRef = interp1(simTimeU, simTau(keep,:), ref.t, 'linear');

        err = achieved.tauNewtonEuler - simOnRef; % whole record, ramps included
        errRMS = sqrt(mean(err.^2, 1));
        relErr = 100*errRMS./max(sqrt(mean(simOnRef.^2, 1)), eps);
        simPeak = max(abs(simTau), [], 1);
        commandPeak = max(abs(command.tauNewtonEuler), [], 1);
        achievedPeak = max(abs(achieved.tauNewtonEuler), [], 1);
        methodDiff = max(methodDiff, max(abs(achieved.tauLagrange - achieved.tauNewtonEuler), [], 'all'));

        summary = [summary; table(string(body.name), max(simPeak), max(commandPeak), max(achievedPeak), ...
            mean(relErr), methodDiff, runtime, 'VariableNames', {'Model', 'SimPeakNm', 'CommandIDPeakNm', ...
            'AchievedIDPeakNm', 'MeanMotorNRMSEPercent', 'MethodDifferenceNm', 'RuntimeSeconds'})]; %#ok<AGROW>
        motorSummary = [motorSummary; table(repmat(string(body.name), 6, 1), (1:6).', simPeak.', commandPeak.', ...
            achievedPeak.', errRMS.', relErr.', 'VariableNames', {'Model', 'Motor', 'SimPeakNm', 'CommandIDPeakNm', ...
            'AchievedIDPeakNm', 'RMSE_Nm', 'NRMSEPercent'})]; %#ok<AGROW>
        fprintf('Sim peak %.3f; achieved-motion ID peak %.3f N*m; mean motor NRMSE %.1f%%\n', ...
            max(simPeak), max(achievedPeak), mean(relErr));
    end

    results.(body.name) = res;
    if makePlots
        fig = plotIdComparison(body.name, res);
        if saveResults
            if ~isfolder(outDir), mkdir(outDir); end
            exportgraphics(fig, fullfile(outDir, [body.name '.png']), 'Resolution', 140);
        end
    end
end

fprintf('\nAll torques below are crank/output-shaft N*m. Divide by N = %g for ideal motor torque.\n', robot.motor.N);
disp(summary);
if saveResults
    if ~isfolder(outDir), mkdir(outDir); end
    save(fullfile(outDir, 'results.mat'), 'results', 'summary', 'motorSummary', 'bodies', 'ref');
    if ~isempty(summary)
        writetable(summary, fullfile(outDir, 'summary.csv'));
        writetable(motorSummary, fullfile(outDir, 'per_motor.csv'));
    end
    fprintf('Saved results to %s\n', outDir);
end
