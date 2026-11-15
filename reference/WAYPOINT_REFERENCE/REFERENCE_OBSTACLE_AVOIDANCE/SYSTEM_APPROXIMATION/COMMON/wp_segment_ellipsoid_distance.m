function d=wp_segment_ellipsoid_distance(p0,p1,c,R,a)
% Exact minimum Euclidean distance between a finite segment and an ellipsoid,
% up to scalar root-search tolerance. Zero if segment intersects ellipsoid.
p0=p0(:);p1=p1(:);c=c(:);a=a(:);v=p1-p0;
u0=R'*(p0-c);du=R'*v;
A=sum((du./a).^2);
if A<1e-18
    d=wp_point_ellipsoid_distance(p0,c,R,a);return;
end
sstar=max(0,min(1,-sum(u0.*du./a.^2)/A));
if sum(((u0+sstar*du)./a).^2)<=1+1e-13
    d=0;return;
end
% Convex one-dimensional minimization. Golden-section interval reduction.
lo=0;hi=1;phi=(sqrt(5)-1)/2;
s1=hi-phi*(hi-lo);s2=lo+phi*(hi-lo);
f1=wp_point_ellipsoid_distance(p0+s1*v,c,R,a);
f2=wp_point_ellipsoid_distance(p0+s2*v,c,R,a);
for k=1:35
    if f1<=f2
        hi=s2;s2=s1;f2=f1;
        s1=hi-phi*(hi-lo);f1=wp_point_ellipsoid_distance(p0+s1*v,c,R,a);
    else
        lo=s1;s1=s2;f1=f2;
        s2=lo+phi*(hi-lo);f2=wp_point_ellipsoid_distance(p0+s2*v,c,R,a);
    end
end
d=min([f1,f2,wp_point_ellipsoid_distance(p0,c,R,a), ...
    wp_point_ellipsoid_distance(p1,c,R,a)]);
end
