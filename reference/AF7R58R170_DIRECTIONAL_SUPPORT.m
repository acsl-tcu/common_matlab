function [hi,lo,ok,alpha]=AF7R58R170_DIRECTIONAL_SUPPORT(H,c,rho)
% Exact support of normalized F/||F|| over BALL(c,rho), in real arithmetic.
% q is confined to the circular cone of half-angle asin(rho/||c||), provided
% rho<||c||. Returns facewise maxima/minima H*q, not a ball of radius L.
% The ball is a proven Bernstein enclosure of F=a_L+g*e3, not an arbitrary
% axis-direction or user-prescribed cable swing bound.
hi=[];lo=[];ok=false;alpha=NaN;
if ~isnumeric(H)||size(H,2)~=3||~isnumeric(c)||numel(c)~=3||...
 ~isscalar(rho)||~isfinite(rho)||rho<0||...
 any(~isfinite([H(:);c(:)])),return;end
c=double(c(:));H=double(H);rho=double(rho);s=norm(c);
if ~isfinite(s)||s<=rho||s<=0,return;end
alpha=asin(min(1,rho/s));v=c/s;
hi=local_bound(H,v,alpha);
lo=-local_bound(-H,v,alpha);
ok=all(isfinite([hi;lo]));
end
function top=local_bound(H,v,alpha)
hnorm=sqrt(sum(H.^2,2));top=zeros(size(hnorm));
for j=1:numel(hnorm)
 if hnorm(j)==0,continue;end
 z=max(-1,min(1,(H(j,:)*v)/hnorm(j)));
 theta=acos(z);
 top(j)=hnorm(j)*cos(max(0,theta-alpha));
end
end
