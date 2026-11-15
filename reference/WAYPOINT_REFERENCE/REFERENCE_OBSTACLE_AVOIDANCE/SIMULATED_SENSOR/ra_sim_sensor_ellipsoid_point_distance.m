function d = ra_sim_sensor_ellipsoid_point_distance(point, center, R, axesLength)
%RA_SIM_SENSOR_ELLIPSOID_POINT_DISTANCE Euclidean distance point -> solid ellipse.
% Exact convex-distance formula, evaluated by a scalar monotone root.
% Solid E={center+R*diag(a)*u : norm(u)<=1}. No bounding-sphere
% false positive is accepted by this function.
point=double(point(:));center=double(center(:));a=double(axesLength(:));
if numel(point)~=3 || numel(center)~=3 || numel(a)~=3 || ...
        any(~isfinite([point;center;a])) || any(a<=0) || ...
        ~isequal(size(R),[3,3]) || any(~isfinite(R(:)))
    error('RaSensor:InvalidEllipsoid','Invalid point, center, or semiaxes.');
end
if norm(R'*R-eye(3),'fro')>1e-6 || det(R)<0.999999
    error('RaSensor:InvalidRotation','Ellipsoid rotation must be SO(3).');
end
y=R'*(point-center);
if sum((y./a).^2)<=1
    d=0; return
end
b=a.^2;
% Closest point x_i = a_i^2*y_i / (a_i^2+lambda).
% For an outside point, lambda>0 and
% f(lambda)=sum((a_i*y_i/(a_i^2+lambda))^2)-1=0.
lo=0;
hi=max(b);
while sum((a.*y./(b+hi)).^2)>1
    hi=hi*2;
    if ~isfinite(hi)
        error('RaSensor:DistanceOverflow','Ellipsoid distance overflow.');
    end
end
% 65 iterations bring double-precision root error below useful tolerance.
for k=1:65
    mid=lo+(hi-lo)/2;
    if sum((a.*y./(b+mid)).^2)>1
        lo=mid;
    else
        hi=mid;
    end
end
lambda=lo+(hi-lo)/2;
x=b.*y./(b+lambda);
d=norm(y-x);
end
