function [t, x] = getSimSignal(simOut, name, n)
%GETSIMSIGNAL  Retrieve a structure-with-time signal from a simulation output.
%
%   [t, x] = getSimSignal(simOut, name, n)
%
%   simOut - Simulink simulation output object
%   name   - logged signal name, e.g. 'motorData' or 'platform_pose'
%   n      - expected number of signal columns
%
%   t      - N x 1 time vector
%   x      - N x n signal values
%
%   Simulink hands these back either as N x n or as n x 1 x N depending on
%   how the signal was logged, so squeeze and orient before returning.
%
%   Example:
%       [t, data] = getSimSignal(out, 'motorData', 24);

    s = simOut.(name);

    t = double(s.time(:));

    x = double(s.signals.values);

    % Drop any singleton dimension Simulink added (n x 1 x N -> n x N)
    x = squeeze(x);

    % Orient to N x n if it came back transposed
    if size(x, 1) ~= numel(t) && size(x, 2) == numel(t)
        x = x.';
    end

    if size(x, 1) ~= numel(t) || size(x, 2) ~= n
        error('getSimSignal:UnexpectedSize', ...
            'Unexpected size for %s: got %dx%d, expected %dx%d.', ...
            name, size(x,1), size(x,2), numel(t), n);
    end

end
