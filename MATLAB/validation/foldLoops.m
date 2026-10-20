function F = foldLoops(row, x, nRows)
    % stacks every complete trajectory loop inside row: F(phase row, channel, loop)
    k0 = ceil(row(1) / nRows);
    k1 = floor((row(end) + 1) / nRows) - 1;
    F = zeros(nRows, size(x,2), max(k1 - k0 + 1, 0));
    for k = k0:k1
        F(:,:,k-k0+1) = interp1(row, x, k*nRows + (0:nRows-1)');
    end
end
