function I = polygonInertia(vertices, mass, thickness)
    % inertia about the COM of a uniform plate: the centred polygon (N x 2) extruded by thickness
    x = vertices(:,1); y = vertices(:,2);
    xn = circshift(x, -1); yn = circshift(y, -1);
    c = x.*yn - xn.*y;
    area = sum(c)/2;
    assert(abs(area) >= eps, 'bodyModels:Polygon', 'platform polygon has zero area')
    xx = mass*sum((y.^2 + y.*yn + yn.^2).*c)/(12*area);
    yy = mass*sum((x.^2 + x.*xn + xn.^2).*c)/(12*area);
    xy = -mass*sum((2*x.*y + x.*yn + xn.*y + 2*xn.*yn).*c)/(24*area);
    I = [xx + mass*thickness^2/12, xy, 0; xy, yy + mass*thickness^2/12, 0; 0, 0, xx + yy];
end
