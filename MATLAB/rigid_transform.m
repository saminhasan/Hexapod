function [poseB, centerOffset] = rigid_transform(poseA, rAB_body, doCenter)
% poseA: Nx7 = [t x y z roll pitch yaw] % Euler angles use XYZ convention
% rAB_body: 3x1 vector from A to B expressed in A's body/local coordinate frame
% doCenter: subtract the mean XYZ from the result (default true)
%
% poseB: Nx7 = [t x y z roll pitch yaw]
% centerOffset: 1x3 mean that was removed, or [0 0 0] when doCenter is false
%
% Only the positions move - the orientation columns pass through untouched.
% That makes the transform exactly invertible at each sample:
%
%     poseB = rigid_transform(poseA, r)                  % centred
%     poseA = rigid_transform(poseB, -r, false) + centerOffset
%
% because the same rotation is applied on the way back. Returning
% centerOffset is what makes the round trip land on the original absolute
% position rather than a re-centred copy of it.

if nargin < 3 || isempty(doCenter)
    doCenter = true;
end

n = size(poseA,1);
poseB = poseA;
rAB_body = rAB_body(:);   % ensure 3x1 column vector

for k = 1:n
    R = eul2rotm(poseA(k,5:7), 'XYZ');% Body -> world rotation
    poseB(k,2:4) = poseA(k,2:4) + (R*rAB_body).'; % Position of B in world frame
end

if doCenter
    centerOffset = mean(poseB(:,2:4),1);
    poseB(:,2:4) = poseB(:,2:4) - centerOffset;   % Center XYZ trajectory
else
    centerOffset = zeros(1,3);
end

end
