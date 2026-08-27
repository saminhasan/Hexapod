function w = bumpwin(t, a, riseTime, fallTime, b)
%BUMPWIN Infinitely smooth compact-support window.
%
%   w = bumpwin(t, a, riseTime, fallTime, b)
%
% Inputs:
%   t         - Time array
%   a         - Time where the window begins rising from 0
%   riseTime  - 0-to-100% rise time
%   fallTime  - 100-to-0% fall time
%   b         - Time where the fall is completed
%
% Output:
%   w         - Window with same size as t
%
% Window shape:
%
%          1       ____________
%                 /            \
%                /              \
%          0 ___/                \___
%              a  a+Tr       b-Tf   b
%
% The rise and fall are C-infinity smooth. All derivatives vanish
% at the four transition boundaries.
%
% Example:
%   t = linspace(0, 10, 2000);
%   w = bumpwin(t, 2, 1, 1.5, 8);
%
%   plot(t, w, 'LineWidth', 2)
%   grid on
%   xlabel('Time')
%   ylabel('Window')

    % -------------------------------------------------------------
    % Input validation
    % -------------------------------------------------------------
    if riseTime < 0
        error('riseTime must be non-negative.');
    end

    if fallTime < 0
        error('fallTime must be non-negative.');
    end

    if b <= a
        error('b must be greater than a.');
    end

    % Ensure there is no overlap between rise and fall
    if a + riseTime > b - fallTime
        error(['Rise and fall transitions overlap. Require: ' ...
               'a + riseTime <= b - fallTime.']);
    end

    % Start with zero everywhere
    w = zeros(size(t));

    % -------------------------------------------------------------
    % Flat portion
    % -------------------------------------------------------------
    plateau = (t >= a + riseTime) & (t <= b - fallTime);
    w(plateau) = 1;

    % -------------------------------------------------------------
    % Rising edge
    % -------------------------------------------------------------
    if riseTime == 0

        w(t >= a & t <= b - fallTime) = 1;

    else

        idx = (t > a) & (t < a + riseTime);

        s = (t(idx) - a) ./ riseTime;

        f1 = exp(-1 ./ s);
        f2 = exp(-1 ./ (1 - s));

        w(idx) = f1 ./ (f1 + f2);
    end

    % -------------------------------------------------------------
    % Falling edge
    % -------------------------------------------------------------
    if fallTime == 0

        w(t >= a + riseTime & t < b) = 1;

    else

        idx = (t > b - fallTime) & (t < b);

        % Normalized coordinate:
        % s = 0 at start of fall
        % s = 1 at b
        s = (t(idx) - (b - fallTime)) ./ fallTime;

        f1 = exp(-1 ./ s);
        f2 = exp(-1 ./ (1 - s));

        % Reverse the smooth step
        w(idx) = f2 ./ (f1 + f2);
    end

    % Explicit endpoint behavior
    w(t <= a) = 0;
    w(t >= b) = 0;
end