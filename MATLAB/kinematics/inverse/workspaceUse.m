function [use, reach, rodEnd] = workspaceUse(robot, x)
    % how close each platform pose (N x 6) is to the rig's limits, worst leg per row (N x 1 each):
    % reach |G|/sqrt(E^2 + F^2) of the IK closure (1 = crank and rod in line, can't close), rodEnd the tilt of the
    % rod end on its ball screw (deg), use the worse of reach/robot.joint.reachLimit and
    % rodEnd/robot.joint.sphericalLimitDeg (1 = at a limit). The rod spins freely, so it's taken where the worse
    % of its two ends is least; the two rod ends on a rod keep their home twist relative to each other
    geom = robot.geom;
    n = size(x, 1);
    R = eul2rotm(x(:,4:6), 'XYZ');
    P = pagemtimes(R, geom.platformPoints.') + permute(x(:,1:3).' + geom.home, [1 3 2]); % platform joints, 3 x 6 x N
    l = P - geom.base.';
    d = geom.crankDirections.';
    E = 2*geom.r*reshape(l(3,:,:), 6, n);
    F = 2*geom.r*reshape(sum(d.*l, 1), 6, n);
    G = reshape(sum(l.^2, 1), 6, n) - (geom.rodLength^2 - geom.r^2);
    s = G./sqrt(E.^2 + F.^2);
    q = asin(max(min(s, 1), -1)) - atan2(F, E);
    tip = geom.base.' + geom.r*(permute(cos(q), [3 1 2]).*d + permute(sin(q), [3 1 2]).*[0; 0; 1]);
    rod = (P - tip)/geom.rodLength;

    % each end: how square its screw is to the rod (A, B) and the twist between the two screws about the rod (phi)
    [A, B, phi] = screwsAcrossRod(rod, repmat(robot.joint.crankScrews.', 1, 1, n), pagemtimes(R, robot.joint.platformScrews.'));
    [~, ~, phi0] = screwsAcrossRod((geom.platformJoints - geom.crankTip).'/geom.rodLength, robot.joint.crankScrews.', robot.joint.platformScrews.');
    phi = phi - phi0;
    % best spin: the larger of the two cosines is maximal at one end's own best spin or where both are equal
    sp = cat(3, zeros(size(phi)), phi, atan((A - B.*cos(phi))./(B.*sin(phi))), atan(-(A + B.*cos(phi))./(B.*sin(phi))));
    sp(isnan(sp)) = 0;
    best = max(min(A.*abs(cos(sp)), B.*abs(cos(sp - phi))), [], 3);

    reach = max(abs(s), [], 1).';
    rodEnd = max(acosd(min(best, 1)), [], 1).';
    use = max(reach/robot.joint.reachLimit, rodEnd/robot.joint.sphericalLimitDeg);
end
