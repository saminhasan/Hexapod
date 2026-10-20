% buildSP_TC.m
% builds SP_TC.slx from ../position/SP.slx: torque driven joints, each servo on its own
%   tau = tauFF + Kp*(qRef - q) + Kd*(qdRef - qd)   (tc, torqueControlSetup)
% Ideal like the rest of the SP series: continuous, true angle and rate, no delay, no limit. References come
% from cubic-spline lookups like SP's; linear ones kink every 1 ms row and ripple at 1 kHz.
% SP.slx is not touched, rerun after editing it
here = fileparts(mfilename('fullpath'));
bdclose('SP'); bdclose('SP_TC');
copyfile(fullfile(fileparts(here), 'position', 'SP.slx'), fullfile(here, 'SP_TC.slx'), 'f');
mdl = 'SP_TC';
load_system(fullfile(here, 'SP_TC.slx'));

for k = 1:6
    s = sprintf('%s/Servo%d', mdl, k);
    [cv, qName, qdName, p] = torqueServo(s, k, 'tauFF', sprintf('tc.tauFF(:,%d)', k));
    add_block('simulink/Math Operations/Sum', [s '/eq'], 'Inputs', '+-', 'Position', p + [120 0 90 0]);
    add_block('simulink/Math Operations/Sum', [s '/eqd'], 'Inputs', '+-', 'Position', p + [120 60 90 60]);
    add_block('simulink/Math Operations/Gain', [s '/Kp'], 'Gain', sprintf('tc.Kp(%d)', k), 'Position', p + [180 0 160 0]);
    add_block('simulink/Math Operations/Gain', [s '/Kd'], 'Gain', sprintf('tc.Kd(%d)', k), 'Position', p + [180 60 160 60]);
    add_block('simulink/Math Operations/Sum', [s '/tau'], 'Inputs', '+++', 'Position', p + [260 20 230 100]);
    add_line(s, 'qRef/1', 'eq/1', 'autorouting', 'on');
    add_line(s, [qName '/1'], 'eq/2', 'autorouting', 'on');
    add_line(s, 'qdRef/1', 'eqd/1', 'autorouting', 'on');
    add_line(s, [qdName '/1'], 'eqd/2', 'autorouting', 'on');
    add_line(s, 'eq/1', 'Kp/1', 'autorouting', 'on');
    add_line(s, 'eqd/1', 'Kd/1', 'autorouting', 'on');
    add_line(s, 'Kp/1', 'tau/1', 'autorouting', 'on');
    add_line(s, 'Kd/1', 'tau/2', 'autorouting', 'on');
    add_line(s, 'tauFF/1', 'tau/3', 'autorouting', 'on');
    add_line(s, 'tau/1', [cv '/1'], 'autorouting', 'on');
end
% the "unconnected line" warning in Servo3-6 is a stray line already in SP.slx, delete it there by hand
% once; cutting physical lines from a script takes real ones with it
save_system(mdl);
fprintf('built %s\n', fullfile(here, 'SP_TC.slx'));
