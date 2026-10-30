function D=AF7R58R177_ELLIPSOID_SEGMENT(c,R,a,pQ,pL,rQ,rC,rL)
% Numerical closest distance from original oriented ellipsoid to:
% (a) UAV ball (b) finite-width STRAIGHT cable segment (c) payload ball.
% All three radii are independent. NO L-radius sphere, NO time sampling.
% Exact geometric problem, numerical fminbnd/1D root execution; this is NOT
% an outward-rounded interval certificate or an execution safety theorem.
D=struct('status','UNKNOWN','uav_m',NaN,'cable_m',NaN, ...
    'payload_m',NaN,'minimum_m',NaN,'closest_cable_fraction',NaN, ...
    'instantaneous_estimate_only',true,'production_certified',false);
c=double(c(:));R=double(R);a=double(a(:));
pQ=double(pQ(:));pL=double(pL(:));
r=double([rQ;rC;rL]);
if numel(c)~=3||numel(a)~=3||numel(pQ)~=3||numel(pL)~=3|| ...
 ~isequal(size(R),[3,3])||numel(r)~=3|| ...
 any(~isfinite([c;R(:);a;pQ;pL;r]))||any(a<=0)||any(r<0)|| ...
 norm(R'*R-eye(3),'fro')>1e-8||abs(det(R)-1)>1e-8
 D.status='UNKNOWN_INVALID_ORIGINAL_SHAPE_OR_RADII';return;
end
D.uav_m=r177_point_distance(pQ,c,R,a)-r(1);
D.payload_m=r177_point_distance(pL,c,R,a)-r(3);
v=pQ-pL;
if norm(v)==0
 f=0;d=r177_point_distance(pL,c,R,a);
else
 opt=optimset('TolX',1e-10,'Display','off');
 [f,d]=fminbnd(@(s)r177_point_distance(pL+s*v,c,R,a),0,1,opt);
 d=min([d,r177_point_distance(pL,c,R,a),r177_point_distance(pQ,c,R,a)]);
end
D.cable_m=d-r(2);
D.minimum_m=min([D.uav_m,D.cable_m,D.payload_m]);
D.closest_cable_fraction=f;
D.status='NUMERICAL_ORIENTED_ELLIPSOID_BALL_AND_CAPSULE_DISTANCE__NOT_INTERVAL_PROOF';
end

function d=r177_point_distance(x,c,R,a)
% Euclidean distance from point to solid axis-aligned ellipsoid after SO3 transform.
y=R'*(x-c);
if sum((y./a).^2)<=1
 d=0;return;
end
% Outside closest point z_i=a_i^2*y_i/(a_i^2+lambda), lambda>=0.
a2=a.^2;
f=@(l) sum((a.*y./(a2+l)).^2)-1;
lo=0;hi=max(a2);
for j=1:256
 if f(hi)<=0,break;end
 hi=hi*2;
 if ~isfinite(hi),d=NaN;return;end
end
for j=1:80
 m=lo+(hi-lo)/2;
 if f(m)>0,lo=m;else,hi=m;end
end
z=a2.*y./(a2+(lo+hi)/2);
d=norm(y-z);
end
