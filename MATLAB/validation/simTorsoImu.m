function [t, f, w] = simTorsoImu(robot, tq, q)
    % replays crank angles q (N x 6, rad, at times tq) through Hexapod_dev (motion driven joints, no
    % controller): f torso imu specific force (m/s^2), w rate (deg/s), both in imu axes (x up, z out of the chest)
    in = Simulink.SimulationInput('Hexapod_dev');
    in = in.setModelParameter('StopTime', num2str(tq(end)), 'SimMechanicsOpenEditorOnUpdate', 'off'); % no explorer, faster
    in = in.setVariable('robot', robot).setVariable('ref', struct('t', tq, 'q', q));
    simOut = sim(in);

    [t, d] = getSimSignal(simOut, 'torso_imu', 19);
    Rt = permute(quat2rotm(d(:,1:4)), [2 1 3]); % world -> imu
    f = squeeze(pagemtimes(Rt, permute(d(:,17:19), [2 3 1]))).'; % logger already adds +g on world z
    w = rad2deg(squeeze(pagemtimes(Rt, permute(d(:,5:7), [2 3 1]))).');
end
