function [d,s]=wp_segment_point_distance(p0,p1,p)
p0=p0(:);p1=p1(:);p=p(:);v=p1-p0;
vv=dot(v,v);if vv<eps,s=0;else,s=max(0,min(1,dot(p-p0,v)/vv));end
d=norm((p0+s*v)-p);
end
