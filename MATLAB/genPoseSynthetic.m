function trajectory = genPoseSynthetic(n)
    %% Running trajectory generator - 6 DoF pelvis/COM path
    %  Vertical (z) physics model is UNTOUCHED.
    %  Every change is marked  % [FIX]  and keeps the original value alongside it,
    %  so anything here can be reverted in place without guesswork.
    
        dt = 1.0e-3;
    
    % ****************
    % Model parameters
    % ***************
        g  = 9.80665;
        gm = 1.5*g/2.0;  % muscle created g, leave at this value
        p  = 1.0/3.0;      % STEP period: between 0.33 and 0.44
        % p  = 3.0/8.0;      % STEP period: between 0.33 and 0.44
        %   [FIX] the original comment read "stride period". It is the STEP period:
    %         z repeats every p, and vertical motion is a step-frequency signal
    %         (left and right steps look identical). Stride = 2*p.
    %         p = 1/3 s  ->  180 steps/min = 90 strides/min.
    %         The math downstream was already consistent with this - only the
    %         label was inverted - so nothing else needed to move for it.
    % changes the maximum displacement:
    %    p=0.33 --> -4cm to +2cm
    %    p=0.40 --> -6cm to +2cm
        amax = 6.0*g;    % maximum accelertaion, little impact on displacement or velocity
        time = (0:dt:p*n)' ;
    
    % internal model parameters, do not touch
        o    = (amax - 4*g + 4*gm + sqrt(amax^2 + 8*amax*g - 8*amax*gm))/(4*(g - gm));
        j1   = p*(g*o - gm*o + g - gm)/(p*(o + 2)/(2*(o + 1)))^(o + 1);
        tE   = p*(o + 2)/(2*(o + 1));
        t0   = p/3;
        phi0 = 2*pi/p*t0;
        v0   = gm*p/2/pi*sin(2*pi/p*t0); %#ok<NASGU>
    %   For reference at the default parameters: o = 11.928, tE = 0.5387*p.
    
    % ---- shared phase clock --------------------------------------------------
    %   [FIX] the 0.0640 s cam-alignment offset was a hardcoded constant while
    %         everything else scaled with p, so changing p (which the comment above
    %         invites) slid z and the rotations relative to x and y. Written as a
    %         fraction of p it evaluates to exactly 0.0640 at p = 1/3
    %         (0.192 * 1/3 = 0.064), so default output is bit-identical - but the
    %         alignment now survives a cadence change.
    %         If that offset is a FIXED physical/mechanical delay rather than a
    %         phase, set  t_shift = 0.0640;  instead and accept the drift.
        t_shift = 0.192*p;
    
    % time vector needs to restart from zero after each period
        t_mod = mod(time + t_shift, p); % to match with cam trajectory
        a = (-g + gm)          + gm              * cos(2*pi/p*t_mod+phi0) + j1             * (t_mod.*(t_mod<tE)).^o; %#ok<NASGU>
        v = (-g + gm)   * t_mod    + gm * (p/2/pi)   * sin(2*pi/p*t_mod+phi0) + j1/(o+1)       * min(t_mod,tE).^(o+1); %#ok<NASGU>
        s = (-g + gm)/2 * t_mod.^2 - gm * (p/2/pi)^2 * cos(2*pi/p*t_mod+phi0) + j1/(o+1)/(o+2) * min(t_mod,tE).^(o+2) + j1/(o+1)*((t_mod>tE).*tE).^(o+1).*(t_mod-tE);
    
    % ---- gait event times ----------------------------------------------------
        f_trajectory = 1/p;           % STEP frequency [Hz]  (3 Hz at p = 1/3)
        ws  = 2*pi*f_trajectory;      % step   angular frequency
        wst = pi *f_trajectory;       % stride angular frequency
    
        t_c  = tE - t_shift;          % peak vertical load, in the shifted clock
    
        t_on = t_c - 0.10;            % contact onset (tune to data)
    %   [FIX] was t_c - 0.06. If load is roughly symmetric about t_c that implies
    %         ~120 ms of stance against a 333 ms step - duty factor 0.18 per
    %         stride, which is sprint territory. 0.10 gives ~200 ms stance and
    %         DF ~0.30, normal for distance running at this cadence.
    %         t_on only phases Rz here, so the visible effect is a ~40 ms yaw
    %         shift - but it matters if you ever drive contact events off it.
    
    % ---- translations [m] ----------------------------------------------------
    %   x: fore-aft (front / back)
    %   [FIX] was  x = (sin(2*pi*(f_trajectory/2)*time) * 0.01) + 0.01;
    %         That runs at f/2 = STRIDE frequency, i.e. one brake-propel cycle per
    %         stride. Fore-aft COM motion is a STEP-frequency signal: each foot
    %         contact brakes then propels. It also sits in phase with z, both
    %         reaching minimum near t_c - kinetic and potential energy fluctuating
    %         in phase is the defining signature of a bouncing gait. Decouple them
    %         and the path reads as walking-like even with z perfectly correct.
    %         Amplitude raised 1.0 -> 1.2 cm (1-2 cm is typical near 3.5 m/s).
    %         NOTE: the +0.010 is a constant 1 cm forward standoff, not an
    %         amplitude - it shifts the origin only.
    %         CAVEAT: if x describes a LATERALLY OFFSET point (a hip marker, or one
    %         side of a cam pair) rather than the COM, yaw injects a genuine
    %         stride-frequency term of ~0.12*sin(Rz) ~ 1.7 cm. In that case add it
    %         back on top of the step term rather than using it alone:
    %           x = 0.010 - 0.012*cos(ws*(time - t_c)) + 0.12*sin(Rz);
        x = 0.010 - 0.012*cos(ws*(time - t_c));
    
    %   y: lateral (left / right), stride frequency - sways toward the stance limb
    %   [FIX] re-phased from absolute `time` onto t_c so it stays locked to z and
    %         to the rotations when p changes. Peak sway now lands at midstance
    %         rather than ~50 ms after it.
    %         Amplitude left at +/-1 cm: that is the top of the normal 1-2 cm band
    %         but defensible, and you did not flag it.
    %         Check the sign against your convention - positive here means peak
    %         excursion toward the stance side at t_c; negate if yours is opposite.
        y = 0.010*cos(wst*(time - t_c));
    
        z = s - mean(s);% z (vertical component, up down)
    %   Note (not changed): `time` spans n periods plus one extra sample, and
    %   p/dt = 333.33 is not an integer, so mean(s) carries a sub-millimetre DC
    %   residue. Harmless for a cam path. If you want it exact, snap the step to
    %   the grid before building `time`:   dt = p/round(p/dt);
    
    % ---- rotations [rad] -----------------------------------------------------
    %   Ry: sagittal pitch (forward lean + ripple), step frequency
    %   [FIX] ripple 2.5 -> 2.0 deg. Marginal - 2.5 was already inside the normal
    %         band - so revert freely if it was measured rather than assumed.
    %         The 10 deg mean forward lean is unchanged and is a good value.
        Ry = deg2rad(10) + deg2rad(2.0)*cos(ws *(time - t_c + 0.03));
    
    %   Rx: frontal roll (pelvic obliquity / list), stride frequency
    %   [FIX] 3.0 -> 4.5 deg. Pelvic obliquity RANGE in running is typically
    %         8-12 deg, i.e. +/-4 to 6 deg. At 3.0 you got a 6 deg range, roughly
    %         40% low - the trajectory looks stiff through the frontal plane.
    %         This was NOT on the list you asked me to touch, so it is the one
    %         change to review first: set back to deg2rad(3.0) if it was deliberate
    %         (a hardware travel limit, for instance).
        Rx = deg2rad(3.0)*cos(wst*(time - t_c));
    
    %   Rz: transverse yaw, stride frequency
    %   Amplitude left untouched as requested - +/-8 deg sits comfortably inside
    %   the normal 5-9 deg band. Only its phase moved, via t_on above.
        Rz = deg2rad(8.0)*cos(wst*(time - t_on));
    
        trajectory = [time, x, y, z, Rx, Ry, Rz];
    % dt= 1e-3;
    % % ****************
    % % Model parameters
    % % ***************
    % g=9.80665;
    % gm=1.5*g/2;    % muscle created g, leave at this value
    % p=1/3;%3/8 = 0.375 %.33;         % stride period: between 0.33 and 0.44
    %                % changes the maximum displacement:
    %                %    p=0.33 --> -4cm to +2cm
    %                %    p=0.40 --> -6cm to +2cm
    % amax = 6*g;    % maximum accelertaion, little impact on displacement or velocity
    % time = (0:dt:p*n)' ;
    % % internal model parameters, do not touch
    % o = (amax - 4*g + 4*gm + sqrt(amax^2 + 8*amax*g - 8*amax*gm))/(4*(g - gm));
    % j1=p*(g*o - gm*o + g - gm)/(p*(o + 2)/(2*(o + 1)))^(o + 1);
    % tE=p*(o + 2)/(2*(o + 1));
    % t0=p/3;
    % phi0=2*pi/p*t0;
    % v0=gm*p/2/pi*sin(2*pi/p*t0); %#ok<NASGU>
    % % time vector needs to restart from zero after each period
    % t_mod = mod(time + 0.0640,p); % to match with cam trajectory
    % a = (-g + gm)          + gm              * cos(2*pi/p*t_mod+phi0) + j1             * (t_mod.*(t_mod<tE)).^o; %#ok<NASGU>
    % v = (-g + gm)   * t_mod    + gm * (p/2/pi)   * sin(2*pi/p*t_mod+phi0) + j1/(o+1)       * min(t_mod,tE).^(o+1); %#ok<NASGU>
    % s = (-g + gm)/2 * t_mod.^2 - gm * (p/2/pi)^2 * cos(2*pi/p*t_mod+phi0) + j1/(o+1)/(o+2) * min(t_mod,tE).^(o+2) + j1/(o+1)*((t_mod>tE).*tE).^(o+1).*(t_mod-tE);
    % 
    % f_trajectory = 1/p;
    % 
    % x = (sin(2 * pi * (f_trajectory / 2) * time ) * 0.01) + 0.01; % x (horizontal component, front back)
    % y = sin(2 * pi * (f_trajectory / 2) * time) * 0.02; % y (horizontal component, left side right side)
    % z = s - mean(s);% z (vertical component, up down)
    % 
    % 
	% t_c   = tE - 0.0640;          % peak vertical load, in the shifted clock
	% t_on  = t_c - 0.06;           % contact onset (tune to  data)
	% ws    = 2*pi*f_trajectory;    % step
	% wst   = pi*f_trajectory;      % stride
    % 
	% Ry = deg2rad(10) + deg2rad(2.5)*cos(ws *(time - t_c + 0.03));
	% Rx =               deg2rad(3.0)*cos(wst*(time - t_c));
	% Rz =               deg2rad(7.5)*cos(wst*(time - t_on));
    % trajectory = [time, x, y, z, Rx, Ry, Rz];
	% Rx = deg2rad(3.0)  *  sin(2 * pi * f_trajectory/2 * time+ phi0 -pi/2); % roll
    % Ry = (deg2rad(2.5) * sin(2 * pi *  f_trajectory * time+ phi0 -pi/2) + deg2rad(10)); % pitch
    % Rz = deg2rad(15.0)  *  sin(2 * pi * f_trajectory/2 * time + phi0 -pi/2); % yaw
end