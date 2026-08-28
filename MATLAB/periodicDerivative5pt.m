function omega = periodicDerivative5pt(theta, samplePeriod)
    %PERIODICDERIVATIVE5PT Compute velocity using a circular 5-point derivative.
    %
    %   omega = periodicDerivative5pt(theta, samplePeriod)
    %
    %   Inputs:
    %       theta        - N x 6 trajectory array
    %       samplePeriod - time between samples in seconds
    %
    %   Output:
    %       omega        - N x 6 velocity array
    %
    %   The trajectory is assumed to be periodic/circular.
    
    if nargin < 2
        samplePeriod = 0.001;   % Default: 1 kHz
    end
    
    if samplePeriod <= 0
        error('samplePeriod must be positive.');
    end
    
    if size(theta, 2) ~= 6
        error('theta must be an N x 6 array.');
    end
    
    omega = ( ...
        circshift(theta,  2, 1) ...
        - 8 * circshift(theta,  1, 1) ...
        + 8 * circshift(theta, -1, 1) ...
        -     circshift(theta, -2, 1) ...
        ) / (12 * samplePeriod);

end