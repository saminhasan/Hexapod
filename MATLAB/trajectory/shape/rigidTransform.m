function [moved, centerOffset] = rigidTransform(trajectory, rAB, doCenter)
    % moves an [t x] trajectory from body point A to point B, rAB in A's body frame; angles pass through,
    % so rigidTransform(moved, -rAB, false) + centerOffset gets the original back. Centred by default
    if nargin < 3 || isempty(doCenter), doCenter = true; end
    moved = trajectory;
    R = eul2rotm(trajectory(:,5:7), 'XYZ');
    moved(:,2:4) = trajectory(:,2:4) + reshape(pagemtimes(R, rAB(:)), 3, []).';
    centerOffset = zeros(1, 3);
    if doCenter
        centerOffset = mean(moved(:,2:4), 1);
        moved(:,2:4) = moved(:,2:4) - centerOffset;
    end
end
