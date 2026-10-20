function trajectory = genPoseMix(n)
    % camera-recorded motion with z swapped for the running model (step period 3/8 s): [t x y z Rx Ry Rz]
    trajectory = genPoseCam(n);
    trajectory(:,4) = runningZ(trajectory(:,1), 3/8, 0.0640);
end
