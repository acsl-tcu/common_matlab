function d=wp_point_ellipsoid_distance(p,c,R,axes)
% Exact unsigned Euclidean distance point-to-solid-ellipsoid, in metres.
% Root of scalar Lagrange multiplier for an exterior point.
a=axes(:);u=R'*(p(:)-c(:));
if sum((u./a).^2)<=1
    d=0;return;
end
lo=0;hi=max(a.^2);
for j=1:75
    test=sum((a.*u./(a.^2+hi)).^2);
    if test<=1,break;end
    hi=2*hi;
end
for j=1:42
    lam=(lo+hi)/2;
    q=sum((a.*u./(a.^2+lam)).^2);
    if q>1,lo=lam;else,hi=lam;end
end
x=a.^2.*u./(a.^2+hi);
d=norm(u-x);
end
