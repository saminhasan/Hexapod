function y = bumpStep(t, a, riseTime)
%SMOOTHHEAVISIDE Infinitely smooth Heaviside step.
%
%   y = smoothHeaviside(t, a, riseTime)
%
% Inputs:
%   t        - Time array
%   a        - Time at which the rise begins
%   riseTime - Time required to go from exactly 0 to exactly 1
%
% Output:
%   y        - Smooth step, same size as t
%
% The transition is C-infinity: all derivatives are continuous,
% including at the beginning and end of the transition.
%
% Example:
  % t = linspace(0,10,1000);
  % y = bumpStep(t, 3, 2);
  % plot(t,y), grid on

if riseTime < 0
    error('riseTime must be non-negative.');
end

% Ordinary Heaviside step for zero rise time
if riseTime == 0
    y = double(t >= a);
    return;
end

% Normalized transition coordinate:
% s = 0 at t = a
% s = 1 at t = a + riseTime
s = (t - a) ./ riseTime;

y = zeros(size(t));

% After transition
y(s >= 1) = 1;

% Inside smooth transition
idx = (s > 0) & (s < 1);
si = s(idx);

f1 = exp(-1 ./ si);
f2 = exp(-1 ./ (1 - si));

y(idx) = f1 ./ (f1 + f2);
end