% runValidation.m
% the 2026-10-05 rig runs (X-1 left out):
% 1. motor feedback: reference reproduced exactly, every loop on top of each other, how repeatable the machine was
% 2. measured motor angles replayed through Hexapod_dev -> sim torso imu, lined up with the measured torso imu
clc; close all; clear all; %#ok<CLALL>
here = fileparts(mfilename('fullpath'));
root = fileparts(here);
run(fullfile(root, 'startup.m'));
robot = hexapodRobot();

raw      = fullfile(root, 'oct 6', 'raw');
logDir   = fullfile(raw, 'TeensyFirmwire', 'logs');
capDir   = fullfile(raw, 'CaptureU');
cacheDir = fullfile(here, 'cache');
outDir   = fullfile(here, 'output');
if ~isfolder(outDir), mkdir(outDir); end

qRef  = readmatrix(fullfile(raw, 'TeensyFirmwire', 'Trajectories', 'synth_3Hz.csv')); % uploaded, = actuatorAngles/syn_30c
nRows = size(qRef, 1);
tp    = (0:nRows-1)'/1000;   % time within one loop
fc    = 100;                 % low pass on both imu, Hz
fcMot = 30;                  % low pass on the replayed motor angles, Hz; see simRun, quantisation x w^2 above this
pad   = 30;                  % s of held pose either side of the PLAY, covers the imu capture window
torso = "04695";

runs = runList();
nR = height(runs);
Fy = cell(nR, 1); Fm = cell(nR, 1); Fs = cell(nR, 1);
summary = table();

for i = 1:nR
    S = readSessionLog(fullfile(logDir, runs.session(i) + ".jsonl"), cacheDir);
    M = runMotorAngles(S, runs.play(i), qRef);
    simFile = fullfile(cacheDir, sprintf('sim_%s_%dHz.mat', runs.name(i), fcMot));
    [tS, fS, wS] = simRun(robot, M, pad, fcMot, simFile);
    L = lineUpImu(tS, fS, wS, readImu(capDir, runs.imu(i), torso), fc);
    plotImuRun(L, pad, runs.name(i), outDir);

    rowL = (L.t - pad)*1000;             % sim clock -> trajectory row
    k = rowL >= 0 & rowL <= M.row(end);  % PLAY only, not the held pose either side
    Fy{i} = foldLoops(M.row, M.q, nRows);
    Fm{i} = foldLoops(rowL(k), L.meas(k,:), nRows);
    Fs{i} = foldLoops(rowL(k), L.sim(k,:), nRows);

    summary = [summary; table(runs.name(i), runs.cond(i), size(Fy{i}, 3), size(Fm{i}, 3), M.refErr, ...
        mean(rad2deg(rms(M.q - M.qCmd))), L.offset - pad, L.r, mean(L.rmse(1:3)), mean(L.rmse(4:6)), ...
        'VariableNames', {'run', 'cond', 'loops', 'imuLoops', 'refErr_deg', 'track_deg', 'imuStart_s', 'r', ...
        'rmse_acc', 'rmse_gyro'})]; %#ok<AGROW>
end
summary = splitvars(summary, 'r', 'NewVariableNames', "r_" + ["ax" "ay" "az" "wx" "wy" "wz"]); % imuStart_s: from PLAY

%% motors: every loop of every run on the loop clock
Y = cat(3, Fy{:});
grp = repelem(runs.cond, cellfun(@(F) size(F, 3), Fy));
Ybar = mean(Y, 3);
[d, dAxis] = estimateDelay(tp, Ybar, tp, qRef, 0.05); % one lag of the mean actual against the reference
for c = ["bra", "nobra"]
    [dc, dcAxis] = estimateDelay(tp, mean(Y(:,:,grp == c), 3), tp, qRef, 0.05);
    fprintf('motor lag %-5s %.2f ms, per motor [%s] ms\n', c, 1000*dc, sprintf('%.2f ', 1000*dcAxis));
end
fprintf('motor lag all   %.2f ms, per motor [%s] ms\n', 1000*d, sprintf('%.2f ', 1000*dAxis));

summary.dev_deg = cellfun(@(F) rad2deg(rms(F - Ybar, 'all')), Fy); % how far each run sat from the all-run mean
Yc = containers.Map(cellstr(["bra" "nobra"]), {mean(Y(:,:,grp == "bra"), 3), mean(Y(:,:,grp == "nobra"), 3)});
% same, from its own condition mean (the two had different gains)
summary.devCond_deg = arrayfun(@(i) rad2deg(rms(Fy{i} - Yc(char(runs.cond(i))), 'all')), (1:nR)');

devC = zeros(size(Y)); % every loop against the mean of its own condition
for j = 1:size(Y, 3), devC(:,:,j) = rad2deg(Y(:,:,j) - Yc(char(grp(j)))); end

mch = "motor " + (1:6);
f1 = plotFold(tp, rad2deg(Y), grp, {rad2deg(qRef), 'k--', 'reference'; rad2deg(Ybar), 'k-', 'mean actual'}, ...
    sprintf('Motor angles, all loops | mean actual lags reference by %.1f ms', 1000*d), mch, 'deg');
f2 = plotFold(tp, rad2deg(Y - qRef), grp, {}, ...
    sprintf('Motor tracking error (actual - reference), every loop | rms %.2f deg, mostly the %.1f ms lag', ...
    rms(rad2deg(Y - qRef), 'all'), 1000*d), mch, 'deg');
f5 = plotFold(tp, devC, grp, {}, ...
    sprintf('Motor angle deviation from its condition mean, every loop | rms bra %.3f deg, no bra %.3f deg', ...
    rms(devC(:,:,grp == "bra"), 'all'), rms(devC(:,:,grp == "nobra"), 'all')), mch, 'deg');

%% torso imu: every loop folded, measured against sim
o = [1 4 2 5 3 6]; % tiles fill row-wise: accel left column, rate right column
Am = cat(3, Fm{:}); Am = Am(:,o,:);
As = cat(3, Fs{:}); As = As(:,o,:);
gi = repelem(runs.cond, cellfun(@(F) size(F, 3), Fm));
ich = {'a_x (up)', 'a_y', 'a_z (out)', '\omega_x (up)', '\omega_y', '\omega_z (out)'};
iun = {'m/s^2', 'm/s^2', 'm/s^2', 'deg/s', 'deg/s', 'deg/s'};
f3 = plotFold(tp, Am, gi, {mean(Am, 3), 'k-', 'measured mean'; mean(As, 3), 'r--', 'sim mean'}, ...
    sprintf('Torso IMU, all loops of all runs | %d Hz low pass, motor replay %d Hz', fc, fcMot), ich(o), iun(o));

exportgraphics(f1, fullfile(outDir, 'motor_fold.png'));
exportgraphics(f2, fullfile(outDir, 'motor_tracking.png'));
exportgraphics(f5, fullfile(outDir, 'motor_deviation.png'));
exportgraphics(f3, fullfile(outDir, 'imu_fold.png'));
writetable(summary, fullfile(outDir, 'summary.csv'));
disp(summary);
