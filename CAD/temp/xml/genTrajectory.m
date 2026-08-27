function trajectory = genTrajectory(type, num_cycle)
    switch lower(type)
        case 'sin'
            trajectory = genPoseSine(num_cycle);
        case 'syn'
            trajectory = genPoseSynthetic(num_cycle);
            
        case 'cam'
            trajectory = genPoseCam(num_cycle);
            
        case 'imu'
            error('genTrajectory:NotImplemented', ...
                'Mode "imu" is not implemented (genPoseImu does not exist).');
        case 'mix'
            trajectory = genPoseMix(num_cycle);
        case 'step'
            trajectory = genPoseStep(num_cycle);
        otherwise
            error('genTrajectory:UnknownType', 'Unknown mode "%s". Valid modes are sin, syn, cam, mix, step.', type);
    end
end