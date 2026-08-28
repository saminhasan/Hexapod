function [trajOut, info] = warpTrajectoryClock(traj, riseTime, fallTime, method)
    %WARPTRAJECTORYCLOCK  Replay a trajectory through a C-infinity ramping clock.
    %
    %   [trajOut, info] = warpTrajectoryClock(traj, riseTime, fallTime, method)
    %
    % Inputs:
    %   traj      - N x 7 array [t x y z roll pitch yaw], uniform dt
    %   riseTime  - time (s) for the clock rate to go 0 -> 1 at the start
    %   fallTime  - time (s) for the clock rate to go 1 -> 0 at the end
    %   method    - interpolation method (default 'spline'), see interpTrajStates
    %
    % Outputs:
    %   trajOut   - M x 7 array, SAME dt as the input, M > N
    %   info      - struct with the clock signals for inspection/plotting:
    %               .tau          output timeline (relative)
    %               .rate         ds/dtau, the clock rate (bump window)
    %               .srcTime      s(tau), warped source time
    %               .delay        tau - s(tau), the lag at every sample
    %               .plateauDelay riseTime/2, the constant lag once the clock
    %                             has ramped up - shift the un-warped
    %                             trajectory by this to compare like with like
    %               .Tin/.Tout    input/output durations
    %               .rateScale    tiny correction applied to hit the end exactly
    %
    % Idea:
    %   Normally the trajectory clock runs at ds/dtau = 1. Here it starts at 0,
    %   ramps smoothly to 1, holds, then decays smoothly back to 0. Every
    %   derivative of the rate vanishes at the four corners, so the motion eases
    %   in and out with no jerk/snap transient - nothing extra gets excited.
    %   The spatial path is untouched; only the pace along it changes.
    %
    % Duration bookkeeping:
    %   bumpStep is symmetric (y(s) + y(1-s) = 1), so the integral of the rate
    %   over a transition is exactly half its duration. The source time "lost"
    %   to each ramp is therefore half the ramp length, and the output must be
    %   longer by riseTime/2 + fallTime/2 to cover the same source span.
    %
    % Example:
    %   [tw, info] = warpTrajectoryClock(traj, 0.5, 0.75);
    %   plot(info.tau, info.rate); grid on; ylabel('ds/d\tau')
    
    % ---------------------------------------------------------------- checks
    if size(traj,2) ~= 7
        error('traj must be N x 7: [t x y z roll pitch yaw].');
    end
    if nargin < 4 || isempty(method), method = 'spline'; end
    if riseTime < 0 || fallTime < 0
        error('riseTime and fallTime must be non-negative.');
    end
    
    t = traj(:,1);
    X = traj(:,2:end);          % [x y z roll pitch yaw]
    
    dt = t(2) - t(1);
    if any(abs(diff(t) - dt) > 1e-9*dt)
        error('Input time vector must be uniformly sampled.');
    end
    
    Tin = t(end) - t(1);
    
    % ------------------------------------------------------- output timeline
    Tout = Tin + 0.5*(riseTime + fallTime);
    M    = round(Tout/dt) + 1;
    tau  = (0:M-1).' * dt;      % same dt as input
    b    = tau(end);            % fall completes exactly on the last sample
    
    if riseTime + fallTime > b
        error(['Ramps do not fit: riseTime + fallTime must be <= %.6g s ' ...
               '(i.e. <= 2*Tin).'], 2*Tin);
    end
    
    % ---------------------------------------------------------- clock rate
    rate = bumpwin(tau, 0, riseTime, fallTime, b);
    rate = rate(:);
    
    % --------------------------------------------- warped source time s(tau)
    s = cumtrapz(tau, rate);
    rateScale = Tin / s(end);   % ~1 + O(1e-6); kills discretisation drift
    s = s * rateScale;
    s = min(max(s, 0), Tin);    % guard against interp1 extrapolation

    tq = t(1) + s;

    % ------------------------------------------------------------ clock lag
    % The output at time tau replays source time s(tau), so the trajectory
    % comes out lagging by tau - s(tau). bumpwin is symmetric, so the
    % integral of the rate over the rise is exactly half its duration and the
    % lag settles to precisely riseTime/2 across the plateau (verified
    % constant to ~1e-16). That is the number to shift the un-warped
    % trajectory by when overlaying the two for comparison.
    delay        = tau - s;
    plateauDelay = 0.5 * riseTime;
    
    % ------------------------------------------------- angle-safe resampling
    ang        = X(:,4:6);
    wasWrapped = all(abs(ang(:)) <= pi + 1e-12);
    X(:,4:6)   = unwrap(ang, [], 1);            % avoid interpolating jumps
    
    Xq = interpTrajStates(t, X, tq, method);
    
    if wasWrapped
        Xq(:,4:6) = mod(Xq(:,4:6) + pi, 2*pi) - pi;
    end
    
    % ------------------------------------------------------------- assemble
    trajOut = [t(1) + tau, Xq];
    
    info = struct( ...
        'tau',          tau, ...
        'rate',         rate * rateScale, ...
        'srcTime',      s, ...
        'delay',        delay, ...
        'plateauDelay', plateauDelay, ...
        'dt',           dt, ...
        'Tin',          Tin, ...
        'Tout',         b, ...
        'rateScale',    rateScale, ...
        'Nin',          size(traj,1), ...
        'Nout',         M);
    
    end
    
    
    % =======================================================================
    % Interpolation kernel - kept separate so it can be swapped/tuned later.
    % =======================================================================
    function Xq = interpTrajStates(tSrc, X, tq, method)
    %INTERPTRAJSTATES  Resample trajectory states at arbitrary query times.
    %
    %   Xq = interpTrajStates(tSrc, X, tq, method)
    %
    %   tSrc   - N x 1 source time vector (monotonic)
    %   X      - N x 6 states [x y z roll pitch yaw], angles already unwrapped
    %   tq     - M x 1 query times, inside [tSrc(1), tSrc(end)]
    %   method - 'spline' (C2, smoothest), 'makima' (less overshoot),
    %            'pchip' (monotone, no overshoot), 'linear' (C0 - avoid)
    %
    %   Swap this out for a quaternion SLERP/SQUAD path, a B-spline fit, or a
    %   band-limited resampler if the Euler-angle route ever bites.
    
    Xq = interp1(tSrc, X, tq, method);

end