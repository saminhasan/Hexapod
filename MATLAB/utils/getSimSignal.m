function [t, x] = getSimSignal(simOut, name, n)
    % logged structure-with-time signal as N x n (Simulink hands it back as N x n or n x 1 x N)
    s = simOut.(name);
    t = double(s.time(:));
    x = squeeze(double(s.signals.values));
    if size(x, 1) ~= numel(t) && size(x, 2) == numel(t)
        x = x.';
    end
    if size(x, 1) ~= numel(t) || size(x, 2) ~= n
        error('getSimSignal:UnexpectedSize', 'Unexpected size for %s: got %dx%d, expected %dx%d.', ...
            name, size(x,1), size(x,2), numel(t), n);
    end
end
