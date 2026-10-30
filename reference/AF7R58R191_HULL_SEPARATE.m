function [separated,gap]=AF7R58R191_HULL_SEPARATE(V,c,rot,axes,rad)
% Sufficient interval halfspace separation: original rotated ellipsoid
% and convex hull of the linearly swept full cable, including ball radii.
% This is a conditional LINEAR EKF PROXY result; not an HLC proof.
assert(isequal(size(V),[3 4])&&isequal(size(rot),[3 3]) && ...
 isequal(size(c),[3 1])&&isequal(size(axes),[3 1]) && ...
 all(isfinite([V(:);c;rot(:);axes;rad])) && ...
 all(axes>0)&&rad>=0,'R191:InvalidGeometry',...
 'Finite oriented-ellipsoid geometry and swept hull required.');
mu=mean(V,2);hv=max(sqrt(sum((V-mu).^2,1)));
gap=norm(c-mu)-hv-max(axes)-rad;
if gap>0,separated=true;return;end
N=[mu-c,V-c,rot,-rot];separated=false;
for j=1:size(N,2)
 n=N(:,j);d=norm(n);
 if d==0||~isfinite(d),continue;end
 n=n/d;support=norm(axes.*(rot'*n));
 gm=max(min(n'*V)-(n'*c+support)-rad, ...
        (n'*c-support)-max(n'*V)-rad);
 gap=max(gap,gm);
 if gm>0,separated=true;return;end
end
end
