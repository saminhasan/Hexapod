% runMismatch.m
% SP_TC (feedforward + PD per joint) vs SP_CTC (full computed torque) when the controller's mass model is
% wrong. The plant (SP) never changes; both controllers are built from the same wrong body model, gains
% included. Set casesToRun first for a subset (each sim takes minutes); results collect in output/mismatch.mat
if ~exist('casesToRun', 'var'), casesToRun = 1:5; end
startup
robot = hexapodRobot();
ref = referenceTrajectory(robot, 'syn', 30);

bodies = bodyModels(robot);
b0 = bodies(3); % what the plant really is
scaleTorso = @(b, s) setfield(setfield(b, 'torsoMass', s*b.torsoMass), 'torsoInertia', s*b.torsoInertia);
cases = {
    'nominal',              b0
    'torso +30%',           scaleTorso(b0, 1.3)
    'torso -30%',           scaleTorso(b0, 0.7)
    'no rotor inertia',     setfield(b0, 'reflectedInertia', 0*b0.reflectedInertia) %#ok<SFLD>
    'torso +30%, no rotor', setfield(scaleTorso(b0, 1.3), 'reflectedInertia', 0*b0.reflectedInertia) %#ok<SFLD>
    };

outDir = fullfile(fileparts(mfilename('fullpath')), 'output');
if ~isfolder(outDir), mkdir(outDir); end
resultFile = fullfile(outDir, 'mismatch.mat');
if isfile(resultFile), load(resultFile, 'R'); else, R = table(); end

for c = casesToRun
    tc = torqueControlSetup(robot, ref, 2*pi*15, 1, cases{c,2});
    ctc = computedTorqueSetup(robot, ref, 2*pi*15, 1, cases{c,2});
    for mdl = ["SP_TC", "SP_CTC"]
        in = Simulink.SimulationInput(mdl).setVariable('robot', robot).setVariable('ref', ref) ...
            .setVariable('tc', tc).setVariable('ctc', ctc);
        tic; simOut = sim(in); elapsed = toc;
        row = trackingError(simOut, ref, robot);
        row = [table(string(cases{c,1}), mdl, 'VariableNames', {'case', 'model'}), row, ...
            table(elapsed, 'VariableNames', {'sim_s'})]; %#ok<AGROW>
        if ~isempty(R), R(R.case == row.case & R.model == row.model, :) = []; end % a rerun replaces its row
        R = [R; row]; %#ok<AGROW>
        fprintf('%-22s %-7s pos rms [%s] mm  ang rms [%s] deg  motor rms %.3f deg  peak %.1f N*m  (%.0f s)\n', ...
            cases{c,1}, mdl, sprintf('%.3f ', row.pos_mm), sprintf('%.4f ', row.ang_deg), row.motor_deg, ...
            row.peak_Nm, elapsed);
        save(resultFile, 'R');
    end
end
disp(R);
