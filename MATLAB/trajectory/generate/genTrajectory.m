function src = genTrajectory(robot, mode, n)
    % what moves, for referenceTrajectory. Every mode gives the same struct:
    %   body            [t x y z Rx Ry Rz] (m, rad, 1 kHz) of one body point
    %   bodyToPlatform  the platform origin from that point, body axes (m)
    %   recorded        true for a recording: its average position and slow drift aren't known
    %   joints          [t q1..q6] (rad) for the motor test, which has no body motion (body empty)
    % designed, n cycles, torso point 10 cm below the platform: sin, syn (synthetic running), cam (camera capture),
    % mix (cam with synthetic z), step (one axis at a time). Recorded sternum IMU, n not used, sternum 32 cm above
    % the platform (manikin): imuRun34C, imuRun36D, imuThrow34D, imuThrow34DD, imuStairs34D, imuStairs34DD.
    % Motor test, n not used: jointStep
    src = struct('body', [], 'bodyToPlatform', robot.torsoOffset, 'recorded', false, 'joints', []);
    switch mode
        case 'sin'
            src.body = genPoseSine(n);
        case 'syn'
            src.body = genPoseSynthetic(n);
        case 'cam'
            src.body = genPoseCam(n);
        case 'mix'
            src.body = genPoseMix(n);
        case 'step'
            src.body = genPoseStep(n);
        case {'imuRun34C', 'imuRun36D', 'imuThrow34D', 'imuThrow34DD', 'imuStairs34D', 'imuStairs34DD'}
            src.body = feval(mode);
            src.bodyToPlatform = -robot.sternumImu;
            src.recorded = true;
        case 'jointStep'
            src.joints = genJointStep();
        otherwise
            error('genTrajectory:UnknownMode', ['unknown mode "%s": sin, syn, cam, mix, step, imuRun34C, imuRun36D, ' ...
                'imuThrow34D, imuThrow34DD, imuStairs34D, imuStairs34DD or jointStep'], mode);
    end
end
