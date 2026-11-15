function [separated,normalCount]=wp_six_trigger_separation(body,known,j)
%WP_SIX_TRIGGER_SEPARATION Sufficient disjointness via a support plane.
% Body = convexHull(vertices) + ball(radius).
% Obstacle = observed outer sphere or rotated outer ellipsoid.
% Find a unit n s.t. min(n*body) > max(n*obstacle).
% Failure to find n means UNKNOWN/POSSIBLE, NOT proven intersection.
V=body.vertices;
c=known.center(:,j);
R=known.R(:,:,j);
a=known.axes(:,j);
normalCount=0;
separated=false;
center=mean(V,2);

% Cheap, typically useful normal first: obstacle -> body.
q=center-c;
nq=norm(q);
if nq>0
    [separated,normalCount]=tryNormal(q/nq,V,body.radius,c,R,a,normalCount);
    if separated,return;end
end

% Extra separating directions only when the center-to-center test fails.
% Fixed candidate directions are sufficient tests, not sampling proofs.
N=[eye(3),R,V-c];
for m=1:size(N,2)
    n=N(:,m); normN=norm(n);
    if normN<=1e-14,continue;end
    n=n/normN;
    [separated,normalCount]=tryNormal(n,V,body.radius,c,R,a,normalCount);
    if separated,return;end
    [separated,normalCount]=tryNormal(-n,V,body.radius,c,R,a,normalCount);
    if separated,return;end
end
end
function [yes,cnt]=tryNormal(n,V,r,c,R,a,cnt)
cnt=cnt+1;
low=min(n'*V)-r;
high=n'*c+norm(a.*(R'*n));
% A numerical guard for near-tangency; not an outward rounding proof.
proj=n'*V;
scale=max(1,max(abs([low;high;proj(:)])));
yes=(low-high)>64*eps(scale);
end
