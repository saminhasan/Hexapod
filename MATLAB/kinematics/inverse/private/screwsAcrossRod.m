function [A, B, phi] = screwsAcrossRod(rod, crankScrew, platformScrew)
    % the two ball screws seen along the rod (all 3 x 6 x N, unit): A, B how square each is to the rod (1 = square,
    % then the rod end sits straight on it), phi the twist from the crank screw to the platform screw about the
    % rod (rad). A, B, phi are 6 x N
    b1 = crankScrew - sum(crankScrew.*rod, 1).*rod;
    A = vecnorm(b1, 2, 1);
    b1 = b1./A;
    b2 = cross(rod, b1, 1);
    a = platformScrew - sum(platformScrew.*rod, 1).*rod;
    B = vecnorm(a, 2, 1);
    phi = atan2(sum(a.*b2, 1), sum(a.*b1, 1));
    A = reshape(A, 6, []);
    B = reshape(B, 6, []);
    phi = reshape(phi, 6, []);
end
