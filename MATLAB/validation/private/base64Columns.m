function s = base64Columns(X, scale)
    % one base64 string of int16 per column, value = int / scale (scale per column or one for all)
    if isscalar(scale), scale = repmat(scale, 1, size(X,2)); end
    encode = @(k) string(matlab.net.base64encode(typecast(int16(round(X(:,k)*scale(k))), 'uint8')));
    s = arrayfun(encode, 1:size(X, 2), 'UniformOutput', false);
end
