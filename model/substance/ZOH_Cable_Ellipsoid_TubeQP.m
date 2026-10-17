function [A,b,info] = ZOH_Cable_Ellipsoid_TubeQP(x,P,dt,obstacles,ropeRadius,marginExtra)
% ZOH_Cable_Ellipsoid_TubeQP
% Certified sufficient, affine-in-u1 conditions for avoiding static, oriented
% ellipsoid obstacles throughout ONE ZOH period, in the ideal 19-state model.
%
% x = [q(4); bodyOmega(3); pL(3); vL(3); pT(3); omegaL(3)]
% P = [mass,jx,jy,jz,gravity,loadmass,cableL,dstx,dsty]
% q3=[u1;u2;u3] (u4 fixed elsewhere), 0<=u1<=20.
% Outputs A*q3<=b (rows only depend on u1 to second order).
% Every inequality is a *hard* sufficient swept-volume safety certificate.
%
% THEOREM (conditional): if x is exact, q and pT remain unit, dynamics are
% FLxyDst+GLxyDst*u with fixed P and |u1|<=20 during each hold, and ALL
% inequalities A*u<=b hold, then the entire rope capsule of radius
% ropeRadius+obstacle.d_margin+marginExtra does not intersect any obstacle
% for any t in [tk,tk+dt]. Repeat feasibility at every step -> noncollision.
% This DOES NOT prove recursive feasibility or robustness to model uncertainty.
%
% Proof core: pL(t)=pL+vL*t+0.5*aL(0)*t^2+ep, |ep|<=AMAX*t^2.
% pT(t)=pT+(omegaL x pT)*t+eT, |eT|<=0.5*BTMAX*t^2.
% For a unit n, an entire line segment stays outside an ellipsoid if both
% endpoints satisfy n'p>=n'c+sqrt(n'Sobs*n)+ropeRadius+margin.
% Subtract AMAX*t^2 + 0.5*L*BTMAX*t^2 from both endpoint predictions.
% The resulting quadratic in t has nonnegative values over [0,dt] when
% all three degree-2 Bernstein coefficients are nonnegative.
%
% Stronger optional condition B2>= (1-alpha)*B0 is NOT imposed here:
% alpha=1 is the ordinary discrete-time CBF (next-step safe-set) condition.
% The swept tube additionally certifies all intersample instants.

assert(numel(x)==19 && numel(P)==9 && dt>0 && isfinite(dt));
x=x(:); P=P(:).';
q=x(1:4); pL=x(8:10);vL=x(11:13);pT=x(14:16);ol=x(17:19);
if abs(norm(q)-1)>1e-3 || abs(norm(pT)-1)>1e-3
    error('ZOH tube certificate requires unit quaternion and unit cable direction.');
end
m=P(1);mL=P(6);L=P(7);g=P(5);Umax=20;
assert(all(isfinite(P)) && m>0 && mL>=0 && L>0 && g>0 && ...
    isfinite(ropeRadius) && isfinite(marginExtra) && ...
    ropeRadius>=0 && marginExtra>=0, ...
    'Invalid physical parameters.');
P22=zeros(1,22);P22([1 6 7 8 9 19 20 21 22])=P;
f=FLxyDst(x,P22);G=GLxyDst(x,P22);
aDrift=f(11:13);aThrust=G(11:13,1);
pTdot=f(14:16);
% Conservative bounds valid throughout the whole interval (any 0<=u1<=20).
OMax=norm(ol)+Umax*dt/(m*L);
AMax=norm([P(8);P(9);-g])+L*m/(m+mL)*OMax^2+Umax/(m+mL);
BTMax=OMax^2+Umax/(m*L);
% |aL(t)-aL(0)| <= 2*AMax -> |pL Taylor remainder|<=AMax*t^2
% |pT''| <= BTMax -> |pT Taylor remainder|<=0.5*BTMax*t^2
E2=AMax+0.5*L*BTMax;

nObs=numel(obstacles);
A=zeros(0,3);b=zeros(0,1);
info=struct('startGap',zeros(nObs,1),'plane',zeros(3,nObs), ...
    'maxErrorBound_m',E2*dt^2,'AMax',AMax,'BTMax',BTMax, ...
    'minNominalSlack',NaN,'activeConstraints',0);
if nObs==0,return;end
for j=1:nObs
    obs=obstacles(j);
    c=obs.p_obs(:);R=obs.R_obs;r=obs.ellipsoid_radii(:);
    assert(all(r>0) && norm(R.'*R-eye(3),'fro')<1e-8, ...
        'Obstacle ellipsoid geometry invalid.');
    rho=ropeRadius+obs.d_margin+marginExtra+1e-6; % numeric guard > QP tolerance
    Sobs=R*diag(r.^2)*R.';
    pQuad=pL-L*pT;
    line=pQuad-pL;
    z=dot(c-pL,line)/(dot(line,line)+realmin);
    pNear=pL+max(0,min(1,z))*line;
    % Candidate separating normals only influence feasibility, not proof:
    % every accepted plane is independently checked exactly below.
    points=[pNear,pL,pQuad];cand=zeros(3,0);
    for ii=1:size(points,2)
        d=points(:,ii)-c;
        cand=[cand,d,Sobs\d]; %#ok<AGROW>
    end
    cand=[cand,R,-R];
    bestGap=-Inf;n=[0;0;1];
    for ii=1:size(cand,2)
        ni=cand(:,ii);
        if norm(ni)<1e-12,continue;end
        ni=ni/norm(ni);
        gap=min(ni.'*pL,ni.'*pQuad)-ni.'*c-sqrt(ni.'*Sobs*ni)-rho;
        if gap>bestGap,bestGap=gap;n=ni;end
    end
    info.startGap(j)=bestGap;
    info.plane(:,j)=n;
    % A nonnegative gap from SOME separating plane is a sufficient condition.
    % A negative gap does NOT mean collision (conservative plane selection).
    % Require both cable endpoints at all times, using quadratic Bernstein.
    limit=n.'*c+sqrt(n.'*Sobs*n)+rho;
    for endpoint=0:1
        p0=pL-endpoint*L*pT;
        v0=vL-endpoint*L*pTdot;
        % f(t;u1) = c0+c1*t+c2*t^2
        c0=n.'*p0-limit;
        c1=n.'*v0;
        c2const=0.5*n.'*aDrift-E2;
        c2u1=0.5*n.'*aThrust;
        B0=c0;
        B1=c0+0.5*dt*c1; % Bernstein middle control point
        B2const=c0+dt*c1+dt^2*c2const;
        B2u1=dt^2*c2u1;
        % IMPORTANT: Bernstein coefficient 1 also contains quadratic/2!
        % For polynomial c0+c1*t+c2*t^2 on [0,dt]:
        % b0=c0, b1=c0+c1*dt/2, b2=c0+c1*dt+c2*dt^2.
        % Only B2 depends on u1, b0/b1 independent of u1.
        A=[A; 0 0 0; 0 0 0; -B2u1 0 0]; %#ok<AGROW>
        b=[b; B0; B1; B2const]; %#ok<AGROW>
    end
end
assert(all(isfinite([A(:);b(:)])), 'Nonfinite swept-tube constraints.');
info.activeConstraints=size(A,1);
end
