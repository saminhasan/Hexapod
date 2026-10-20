function q = quatBetween(u, v)
    % unit quaternion [w x y z] of the shortest rotation taking direction u onto v
    a = acos(dot(u, v)/(norm(u)*norm(v)));
    k = cross(u, v)/norm(cross(u, v));
    q = [cos(a/2), k(:).'*sin(a/2)];
    q = q/norm(q);
end
