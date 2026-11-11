function [qmax,qmin,ok]=TRAJECTORY_C9_DIRECTION_CONE_SUPPORT(c,rho,n)
% TRAJECTORY_C9_DIRECTION_CONE_SUPPORT
% Sharp REAL-ARITHMETIC support of q=w/||w|| with ||w-c||<=rho<||c||.
% For any unit direction n, returns bounds n'*q in [qmin,qmax].
% Unlike the chord-padding bound, this is a directional exact cap bound.
% This function is NOT a directed-rounding machine certificate by itself.
% Scalar c and n are physical WORLD directions, no fixed bypass direction.
% Missing/degenerate data -> ok=false, never an optimistic bound.
qmax=NaN;qmin=NaN;ok=false;
if ~(isnumeric(c)&&isnumeric(n)&&isnumeric(rho)&&isreal(c)&& ...
        isreal(n)&&isreal(rho)&&isequal(size(c),[3 1])&& ...
        isequal(size(n),[3 1])&&isscalar(rho)&& ...
        all(isfinite(c))&&all(isfinite(n))&&isfinite(rho)&&rho>=0)
    return
end
nc=norm(c);nn=norm(n);
if ~(isfinite(nc)&&nc>rho&&isfinite(nn)&&nn>0),return;end
a=(c.'*n)/(nc*nn);
a=max(-1,min(1,a));
sinAlpha=rho/nc;
cosAlpha=sqrt(max(0,1-sinAlpha^2));
sinTheta=sqrt(max(0,1-a^2));
% All unit vectors in the ball cone have angular distance <=asin(rho/nc).
% Maximum projection: 1 if n is in the cap; otherwise cos(theta-alpha).
if a>=cosAlpha
    qmax=1;
else
    qmax=a*cosAlpha+sinTheta*sinAlpha;
end
% Minimum projection: -1 if -n is in the cap; otherwise cos(theta+alpha).
if a<=-cosAlpha
    qmin=-1;
else
    qmin=a*cosAlpha-sinTheta*sinAlpha;
end
qmax=max(-1,min(1,qmax));
qmin=max(-1,min(1,qmin));
ok=isfinite(qmax)&&isfinite(qmin)&&qmax>=qmin;
if ~ok,qmax=NaN;qmin=NaN;end
end
