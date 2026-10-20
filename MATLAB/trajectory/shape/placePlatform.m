function [platform, centerOffset] = placePlatform(robot, src)
    % platform pose [t x] for a body motion from genTrajectory: the body point moved to the platform origin
    % (rigidTransform) and centred. A recording's slow drift and average position aren't known, so for one the
    % translation below 0.5 Hz is dropped and the average goes where the legs and rod ends have the most room
    % (bestShift); the motion itself is never changed. centerOffset: what came off the platform position (m)
    [platform, centerOffset] = rigidTransform(src.body, src.bodyToPlatform);
    if src.recorded
        [b, a] = butter(2, 0.5/(robot.fs/2), 'high');
        platform(:,2:4) = filtfilt(b, a, platform(:,2:4));
        shift = bestShift(robot, platform(:,2:7), [0 0 0]);
        platform(:,2:4) = platform(:,2:4) + shift;
        centerOffset = -shift;
    end
end
