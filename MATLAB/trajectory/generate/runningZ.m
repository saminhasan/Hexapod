function [z, tE] = runningZ(t, p, tShift)
    % vertical torso motion of running (m, zero mean), step period p (s), step phase moved by tShift.
    % Each step: gravity, a cosine "muscle" term and a sharp push that ends tE = 0.5387*p into the step,
    % which is where the vertical load peaks
    g = 9.80665;
    gm = 1.5*g/2;           % muscle created g, leave at this value
    amax = 6.0*g;           % peak acceleration, little effect on displacement or velocity
    o = (amax - 4*g + 4*gm + sqrt(amax^2 + 8*amax*g - 8*amax*gm))/(4*(g - gm));
    j1 = p*(g*o - gm*o + g - gm)/(p*(o + 2)/(2*(o + 1)))^(o + 1);
    tE = p*(o + 2)/(2*(o + 1));
    phi0 = 2*pi/p*(p/3);
    ts = mod(t + tShift, p); % time within the step
    s = (-g + gm)/2*ts.^2 - gm*(p/2/pi)^2*cos(2*pi/p*ts + phi0) + j1/(o + 1)/(o + 2)*min(ts, tE).^(o + 2) ...
        + j1/(o + 1)*((ts > tE).*tE).^(o + 1).*(ts - tE);
    z = s - mean(s);
end
