function lag = bestLag(x, y, window)
    % lag (samples) where the summed channel xcorr peaks, x(n+lag) ~ y(n)
    n = max(size(x,1), size(y,1));
    c = 0;
    for k = 1:size(x,2)
        c = c + xcorr(normalize(x(:,k)), normalize(y(:,k)), n);
    end
    lags = (-n:n)';
    if ~isempty(window), c(~ismember(lags, window)) = -inf; end
    [~, i] = max(c);
    lag = lags(i);
end
