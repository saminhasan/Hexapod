function jointTrajectory = genJointStep()
    % joint-space test signal [t q1..q6] (rad), skips IK: each motor in turn steps to +1 and -1 rad
    [t, q] = bumpSequence(ones(1, 6), 1, 1e-3);
    jointTrajectory = [t, q];
end
