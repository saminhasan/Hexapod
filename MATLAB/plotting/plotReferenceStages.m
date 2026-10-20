function plotReferenceStages(ref)
    % what referenceTrajectory did: the body point's motion requested vs as run (platform path mapped back), then
    % the platform path, time warped and tapered. Warped stages go on the source time each row replays, so they
    % line up with the original exactly and only the taper is left to see. The motor test has no body motion,
    % so only its platform path
    if ~isempty(ref.body)
        asRun = rigidTransform([ref.t, ref.x], -ref.bodyToPlatform, false);
        asRun(:,2:4) = asRun(:,2:4) + ref.centerOffset;
        asRun(:,1) = ref.sourceTime;
        plotTrajectory({ref.body, asRun}, {'requested', 'as run'}, 'Body Point Trajectory');
    end
    plotTrajectory({ref.platform, [ref.sourceTime, ref.warped(:,2:7)], [ref.sourceTime, ref.x]}, ...
        {'platform frame', 'time warped', 'tapered'}, 'Platform Frame Trajectory');
end
