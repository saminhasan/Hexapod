function [dy, ddy] = sampledDerivatives(t, y)
    % first and second time derivatives of the columns of y: 5-point stencils inside, 2nd order at the ends;
    % uneven t gets a local polynomial fit per sample instead
    n = numel(t); dy = zeros(size(y)); ddy = dy;
    h = mean(diff(t));
    if max(abs(diff(t)/h - 1)) < 1e-7
        dy(3:n-2,:) = (y(1:n-4,:) - 8*y(2:n-3,:) + 8*y(4:n-1,:) - y(5:n,:))/(12*h);
        ddy(3:n-2,:) = (-y(1:n-4,:) + 16*y(2:n-3,:) - 30*y(3:n-2,:) + 16*y(4:n-1,:) - y(5:n,:))/(12*h^2);
        dy(1,:) = (-3*y(1,:) + 4*y(2,:) - y(3,:))/(2*h);
        dy(2,:) = (y(3,:) - y(1,:))/(2*h);
        dy(n-1,:) = (y(n,:) - y(n-2,:))/(2*h);
        dy(n,:) = (3*y(n,:) - 4*y(n-1,:) + y(n-2,:))/(2*h);
        ddy(1,:) = (2*y(1,:) - 5*y(2,:) + 4*y(3,:) - y(4,:))/h^2;
        ddy(2,:) = (y(1,:) - 2*y(2,:) + y(3,:))/h^2;
        ddy(n-1,:) = (y(n-2,:) - 2*y(n-1,:) + y(n,:))/h^2;
        ddy(n,:) = (2*y(n,:) - 5*y(n-1,:) + 4*y(n-2,:) - y(n-3,:))/h^2;
    else
        for k = 1:n
            if k <= 2
                idx = 1:4;
            elseif k >= n-1
                idx = n-3:n;
            else
                idx = k-2:k+2;
            end
            offsets = t(idx) - t(k); scale = max(abs(offsets)); % scaled so the Vandermonde stays well conditioned
            z = offsets/scale;
            V = (z(:).').^((0:numel(idx)-1).');
            rhs = zeros(numel(idx), 2); rhs(2,1) = 1; rhs(3,2) = 2;
            weights = V\rhs;
            dy(k,:) = weights(:,1).'*y(idx,:)/scale;
            ddy(k,:) = weights(:,2).'*y(idx,:)/scale^2;
        end
    end
end
