function S = skew3(v)
    % cross-product matrix: skew3(a)*b == cross(a,b)
    %#codegen
    S = [0 -v(3) v(2); v(3) 0 -v(1); -v(2) v(1) 0];
end
