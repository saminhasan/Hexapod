% buildSP_CTC.m
% builds SP_CTC.slx from ../position/SP.slx: torque driven joints under full computed torque
% (ctc, computedTorqueSetup)
%   x   = forwardKinematicsStep of the six angles, started from the commanded pose
%   tau = M(q, x)*(qddRef + 2*zeta*wn*(qdRef - qd) + wn^2*(qRef - q)) + bias(q, qd, x)
% M couples all six joints, so the law sits in one central block; references come from cubic-spline
% lookups inside each servo like SP's. Ideal, continuous, no delay, no limit. SP.slx is not touched
here = fileparts(mfilename('fullpath'));
bdclose('SP'); bdclose('SP_CTC');
copyfile(fullfile(fileparts(here), 'position', 'SP.slx'), fullfile(here, 'SP_CTC.slx'), 'f');
mdl = 'SP_CTC';
load_system(fullfile(here, 'SP_CTC.slx'));

for k = 1:6
    s = sprintf('%s/Servo%d', mdl, k);
    [cv, qName, qdName, p] = torqueServo(s, k, 'qddRef', sprintf('ref.qdd(:,%d)', k));

    % torque comes in from the central controller; state and references go out to it
    add_block('simulink/Sources/In1', [s '/tau'], 'Position', p + [120 -60 90 -60]); % inport 2
    add_line(s, 'tau/1', [cv '/1'], 'autorouting', 'on');
    add_block('simulink/Signal Routing/Mux', [s '/ctc Mux'], 'Inputs', '5', 'Position', [560 130 565 230]);
    add_block('simulink/Sinks/Out1', [s '/ctc'], 'Position', [600 170 630 190]); % outport 2: [q qd qRef qdRef qddRef]
    add_line(s, [qName '/1'], 'ctc Mux/1', 'autorouting', 'on');
    add_line(s, [qdName '/1'], 'ctc Mux/2', 'autorouting', 'on');
    add_line(s, 'qRef/1', 'ctc Mux/3', 'autorouting', 'on');
    add_line(s, 'qdRef/1', 'ctc Mux/4', 'autorouting', 'on');
    add_line(s, 'qddRef/1', 'ctc Mux/5', 'autorouting', 'on');
    add_line(s, 'ctc Mux/1', 'ctc/1', 'autorouting', 'on');
end

add_block('simulink/User-Defined Functions/MATLAB Function', [mdl '/CTC'], 'Position', [140 520 260 580]);
chart = find(sfroot, '-isa', 'Stateflow.EMChart', 'Path', [mdl '/CTC']);
chart.Script = strjoin({
    'function tau = CTC(t, s, ctc)'
    '% pose from forward kinematics started at the commanded pose for t (always close, so no memory'
    '% and the block stays continuous), then computed torque'
    'S = reshape(s, 5, 6); % rows q qd qRef qdRef qddRef, one column per motor'
    'i = min(max(floor(t/ctc.dt) + 1, 1), size(ctc.xRef, 1));'
    'x = forwardKinematicsStep(ctc.dyn.geom, S(1,:).'', ctc.xRef(i,:).'');'
    'tau = computedTorqueStep(ctc.dyn, x, S(1,:).'', S(2,:).'', S(3,:).'', S(4,:).'', S(5,:).'', ctc.wn, ctc.zeta);'
    }, newline);
chart.find('-isa', 'Stateflow.Data', 'Name', 'ctc').Scope = 'Parameter';
add_block('simulink/Signal Routing/Mux', [mdl '/ctc all'], 'Inputs', '6', 'Position', [60 500 65 600]);
add_block('simulink/Signal Routing/Demux', [mdl '/Demux'], 'Outputs', '6', 'Position', [320 500 325 600]);
add_line(mdl, 'Clock/1', 'CTC/1', 'autorouting', 'on');
add_line(mdl, 'ctc all/1', 'CTC/2', 'autorouting', 'on');
add_line(mdl, 'CTC/1', 'Demux/1', 'autorouting', 'on');
for k = 1:6
    add_line(mdl, sprintf('Servo%d/2', k), sprintf('ctc all/%d', k), 'autorouting', 'on');
    add_line(mdl, sprintf('Demux/%d', k), sprintf('Servo%d/2', k), 'autorouting', 'on');
end
% Servo3-6 "unconnected line" warning: stray line in SP.slx, see buildSP_TC.m
save_system(mdl);
fprintf('built %s\n', fullfile(here, 'SP_CTC.slx'));
