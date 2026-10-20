function y = fftIntegrate(x, fs, fc, order)
    % order-th time integral of the columns of x (sampled at fs) in the frequency domain: below fc/2 removed,
    % fc/2..fc tapered in. Zero padded 30 s so the end of a recording doesn't wrap onto its start
    pad = 30*fs;
    n0 = size(x, 1);
    x = [zeros(pad, size(x, 2)); x - mean(x); zeros(pad, size(x, 2))];
    N = size(x, 1);
    f = (0:N-1).'*fs/N;
    f(f > fs/2) = f(f > fs/2) - fs;
    af = abs(f);
    H = zeros(N, 1);
    k = af >= fc/2;
    H(k) = 1./(2i*pi*f(k)).^order;
    taper = af >= fc/2 & af < fc;
    H(taper) = H(taper).*(1 - cos(pi*(af(taper) - fc/2)/(fc/2)))/2;
    y = real(ifft(fft(x).*H));
    y = y(pad+1:pad+n0,:);
    y = y - mean(y);
end
