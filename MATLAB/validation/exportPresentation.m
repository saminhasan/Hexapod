% exportPresentation.m
% data for the repeatability page (validation/presentation/index.html), written to presentation/data.js:
% motor reference vs actual and real torso imu (platform axes: x forward, y left, z up, gravity kept), every
% 11.001 s loop, plus 0.667 s strides cut from the full-amplitude part of each loop
clc; close all; clear all; %#ok<CLALL>
here = fileparts(mfilename('fullpath'));
root = fileparts(here);
run(fullfile(root, 'startup.m'));
robot = hexapodRobot();
g = robot.g;

raw      = fullfile(root, 'oct 6', 'raw');
logDir   = fullfile(raw, 'TeensyFirmwire', 'logs');
capDir   = fullfile(raw, 'CaptureU');
cacheDir = fullfile(here, 'cache');
outDir   = fullfile(here, 'presentation');
if ~isfolder(outDir), mkdir(outDir); end

qRef  = readmatrix(fullfile(raw, 'TeensyFirmwire', 'Trajectories', 'synth_3Hz.csv'));
nRows = size(qRef, 1);
fc = 100; fcMot = 30; pad = 30;
qImu = [-0.0164 0.5338 -0.0016 0.8454]; % real torso imu axes in platform frame, fitImuMount mean over the 9 runs
Rimu = quat2rotm(qImu / norm(qImu));

%% every complete loop of every run
runs = runList();
Y = []; A = []; loopRun = []; loopIdx = []; BR = {{}, {}}; BRR = {{}, {}};
for i = 1:height(runs)
    S = readSessionLog(fullfile(logDir, runs.session(i) + ".jsonl"), cacheDir);
    M = runMotorAngles(S, runs.play(i), qRef);
    simFile = fullfile(cacheDir, sprintf('sim_%s_%dHz.mat', runs.name(i), fcMot));
    [tS, fS, wS] = simRun(robot, M, pad, fcMot, simFile); % only lines the imu up
    L = lineUpImu(tS, fS, wS, readImu(capDir, runs.imu(i), "04695"), fc);
    rowL = (L.t - pad) * 1000;
    k = rowL >= 0 & rowL <= M.row(end);
    m = [(Rimu * L.meas(k,1:3).').', (Rimu * L.meas(k,4:6).').']; % imu axes -> platform axes
    Fy = foldLoops(M.row, M.q, nRows);
    Fm = foldLoops(rowL(k), m, nRows);
    nL = min(size(Fy,3), size(Fm,3));
    Y = cat(3, Y, Fy(:,:,1:nL));
    A = cat(3, A, Fm(:,:,1:nL));

    % breast imus (left 04694, right 04696 - bra runs only), same capture clock as the torso. Soft tissue,
    % mount unknown: tilted so gravity at rest (before PLAY, platform level) is +z, x = sensor's outward
    % z made horizontal (forward), y left. Heading about vertical is assumed. Run mean (gravity + bias) out, in g.
    for b = 1:2
        id = ["04694" "04696"];
        if isempty(dir(fullfile(capDir, '**', runs.imu(i) + "_TS-" + id(b) + "_*_aligned.csv")))
            BR{b}(end+1:end+nL) = {[]};
            BRR{b}(end+1:end+nL) = {[]};
            continue
        end
        mb = readImu(capDir, runs.imu(i), id(b));
        rest = L.t - pad < -1;
        up = mean(interp1(mb(:,1), mb(:,2:4), L.t(rest) - L.offset)).'; up = up / norm(up);
        fw = [0; 0; 1] - up(3)*up; fw = fw / norm(fw);
        Rb = [fw.'; cross(up, fw).'; up.']; % sensor -> x forward, y left, z up
        fprintf('%-9s %s up in sensor axes [%s], %.1f deg off sensor x\n', runs.name(i), id(b), ...
            sprintf('%.3f ', up), acosd(up(1)));
        [bb, ab] = butter(4, fc / 800);
        xb = interp1(mb(:,1), filtfilt(bb, ab, mb(:,2:4)) * Rb.', L.t(k) - L.offset);
        Fb = foldLoops(rowL(k), xb, nRows);
        Fb = (Fb(:,:,1:nL) - mean(Fb(:,:,1:nL), [1 3])) / g;
        BR{b}(end+1:end+nL) = num2cell(Fb, [1 2]);
        % relative to the sternum: raw specific forces in the same levelled axes, so gravity cancels;
        % run mean out (bias difference). Gyro can't give the breast->sternum rotation (tissue turns on
        % its own: only yaw rate tracks), so the rest-gravity tilt is the only offset taken out.
        Fr = foldLoops(rowL(k), xb - m(:,1:3), nRows);
        Fr = (Fr(:,:,1:nL) - mean(Fr(:,:,1:nL), [1 3])) / g;
        BRR{b}(end+1:end+nL) = num2cell(Fr, [1 2]);
    end
    loopRun = [loopRun; repmat(i, nL, 1)]; %#ok<AGROW>
    loopIdx = [loopIdx; (1:nL)']; %#ok<AGROW>
end
nLoops = size(Y, 3);
cond = runs.cond(loopRun);

%% gait timing off the generator: rebuild the uploaded file to get every row's source time
rebuilt = referenceTrajectory(robot, 'syn', 30);
fprintf('rebuilt trajectory vs uploaded file: max diff %.2e rad\n', max(abs(rebuilt.q - qRef), [], 'all'));
tau = (0:nRows-1)' / 1000;

% gravity out of the platform-axes accel: g*R(3,:) with R the platform orientation, taken from the
% commanded pose 14 ms earlier (the measured lag). Then one constant per run so every loop averages
% zero linear acceleration - it starts and ends at rest - which also takes out the accelerometer bias
Ybar0 = mean(Y, 3);
lagRows = round(1000 * estimateDelay(tau, Ybar0, tau, qRef, 0.05));
Rp = eul2rotm(rebuilt.x([ones(1, lagRows), 1:end-lagRows], 4:6), 'XYZ');
A(:,1:3,:) = A(:,1:3,:) - g * squeeze(Rp(3,:,:)).';
for i = 1:height(runs)
    q = loopRun == i;
    off = mean(A(:,1:3,q), [1 3]);
    A(:,1:3,q) = A(:,1:3,q) - off;
    fprintf('%-9s accel offset removed [%s] m/s^2\n', runs.name(i), sprintf('%6.3f ', off));
end
A(:,1:3,:) = A(:,1:3,:) / g; % in g from here on
win = rebuilt.window;
[su, iu] = unique(rebuilt.sourceTime(:));
tauOf = @(s) interp1(su, tau(iu), s);

p = 1/3; % step period, as in genPoseSynthetic
[~, tE] = runningZ(0, p, 0);
tC = tE - 0.192*p; % peak vertical load
tOn = tC - 0.10;   % contact onset
% yaw = 8 deg cos(stride phase from tOn): turned left (+yaw) at step 0 contact -> right foot forward -> right step
kk = 0 : floor((su(end) - tOn) / p);
sK = tOn + kk*p;
steps.t = tauOf(sK);
steps.peak = tauOf(sK + (tC - tOn));
steps.side = mod(kk, 2); % 0 right, 1 left

% strides start at right contact, kept only where the loop runs at full amplitude
sg = (0 : 0.005 : 2*p - 1e-9)';
STR = []; strLoop = []; BRS = {{}, {}}; BRRS = {{}, {}};
for j = find(mod(kk, 2) == 0 & sK + 2*p <= su(end))
    tq = tauOf(sK(j) + sg);
    if any(interp1(tau, win, tq) < 1 - 1e-9), continue, end
    for l = 1:nLoops
        STR = cat(3, STR, interp1(tau, A(:,:,l), tq));
        strLoop = [strLoop; l]; %#ok<AGROW>
        for b = 1:2 % breasts, same cut
            if isempty(BR{b}{l}), BRS{b}{end+1} = []; else, BRS{b}{end+1} = interp1(tau, BR{b}{l}, tq); end
            if isempty(BRR{b}{l}), BRRS{b}{end+1} = []; else, BRRS{b}{end+1} = interp1(tau, BRR{b}{l}, tq); end
        end
    end
end

%% per-loop numbers
Ybar = mean(Y, 3);
Abar = mean(A, 3);
[lag, lagAxis] = estimateDelay(tau, Ybar, tau, qRef, 0.05);
T.track  = squeeze(rms(rad2deg(Y - qRef), [1 2]));
T.devAll = squeeze(rms(rad2deg(Y - Ybar), [1 2]));
T.devCond = zeros(nLoops, 1);
for c = unique(cond)'
    q = cond == c;
    T.devCond(q) = squeeze(rms(rad2deg(Y(:,:,q) - mean(Y(:,:,q), 3)), [1 2]));
end
T.accDev = squeeze(rms(A(:,1:3,:) - Abar(:,1:3), 1)).';
T.gyrDev = squeeze(rms(A(:,4:6,:) - Abar(:,4:6), 1)).';
T.accStd = squeeze(std(A(:,1:3,:), 0, 1)).';
T.accPeakZ = squeeze(max(A(:,3,:), [], 1));
T.r = arrayfun(@(l) corr(reshape(A(:,1:3,l), [], 1), reshape(Abar(:,1:3), [], 1)), (1:nLoops)');

%% write
D.runs = runs.name; D.runCond = runs.cond;
D.loopRun = loopRun; D.loopIdx = loopIdx;
D.loopT = nRows / 1000; D.lagMs = 1000*lag; D.lagAxisMs = 1000*lagAxis;
D.motor = struct('fs', 100, 'scale', 100, 'ref', {base64Columns(rad2deg(qRef(1:10:end,:)), 100)}, 'act', {{}});
D.imu = struct('fs', 200, 'scale', [1000 1000 1000 10 10 10], 'loops', {{}});
for l = 1:nLoops
    D.motor.act{l} = base64Columns(rad2deg(Y(1:10:end,:,l)), 100);
    D.imu.loops{l} = base64Columns(A(1:5:end,:,l), [1000 1000 1000 10 10 10]);
end
D.breast = struct('fs', 200, 'scale', 1000, 'left', {cell(1, nLoops)}, 'right', {cell(1, nLoops)}, ...
    'strideLeft', {cell(1, numel(strLoop))}, 'strideRight', {cell(1, numel(strLoop))});
for l = 1:nLoops
    if ~isempty(BR{1}{l}), D.breast.left{l} = base64Columns(BR{1}{l}(1:5:end,:), 1000); end
    if ~isempty(BR{2}{l}), D.breast.right{l} = base64Columns(BR{2}{l}(1:5:end,:), 1000); end
end
for j = 1:numel(strLoop)
    if ~isempty(BRS{1}{j}), D.breast.strideLeft{j} = base64Columns(BRS{1}{j}, 1000); end
    if ~isempty(BRS{2}{j}), D.breast.strideRight{j} = base64Columns(BRS{2}{j}, 1000); end
end
% breast minus sternum, per loop and per stride
D.rel = struct('left', {cell(1, nLoops)}, 'right', {cell(1, nLoops)}, ...
    'strideLeft', {cell(1, numel(strLoop))}, 'strideRight', {cell(1, numel(strLoop))});
for l = 1:nLoops
    if ~isempty(BRR{1}{l}), D.rel.left{l} = base64Columns(BRR{1}{l}(1:5:end,:), 1000); end
    if ~isempty(BRR{2}{l}), D.rel.right{l} = base64Columns(BRR{2}{l}(1:5:end,:), 1000); end
end
for j = 1:numel(strLoop)
    if ~isempty(BRRS{1}{j}), D.rel.strideLeft{j} = base64Columns(BRRS{1}{j}, 1000); end
    if ~isempty(BRRS{2}{j}), D.rel.strideRight{j} = base64Columns(BRRS{2}{j}, 1000); end
end
D.steps = steps;
D.stride = struct('fs', 200, 'T', 2*p, 'loop', strLoop, 'marks', [0, tC - tOn, p, p + tC - tOn, 2*p], 'data', {{}});
for j = 1:size(STR, 3)
    D.stride.data{j} = base64Columns(STR(:,:,j), [1000 1000 1000 10 10 10]);
end
D.table = T;
fid = fopen(fullfile(outDir, 'data.js'), 'w');
fprintf(fid, 'window.DATA = %s;\n', jsonencode(D));
fclose(fid);
fprintf('%d loops, %d strides (%d per loop), lag %.1f ms -> %s\n', nLoops, size(STR, 3), size(STR, 3)/nLoops, ...
    1000*lag, fullfile(outDir, 'data.js'));
