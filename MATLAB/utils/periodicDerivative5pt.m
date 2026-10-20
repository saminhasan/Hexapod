function xd = periodicDerivative5pt(x, dt)
    % 5-point central derivative of each column, the signal treated as periodic (the ends wrap)
    if nargin < 2, dt = 0.001; end
    xd = (circshift(x, 2, 1) - 8*circshift(x, 1, 1) + 8*circshift(x, -1, 1) - circshift(x, -2, 1))/(12*dt);
end
