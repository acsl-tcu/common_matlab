function tab = CBF_V22_ZhengDirectionAudit()
% Exact geometric/virtual-acceleration authority identity from Zheng et al.
% Paper's point: p_lambda=x_drone+lambda*L*n, 0<=lambda<=1.
% Eq. (5),(6),(16) imply G(lambda)=(1-lambda)*I+lambda*n*n'.
% Distinguishes directions with full-rank authority vs endpoint rank loss.
% Also checks paper's transformed H=h*(atan(L_f h)+pi/2), whose input
% coefficient is identically zero at h=0. This is an analytic counterexample
% to an unrestricted forward-invariance claim, not an experimental refutation.
rows={}; n=[0;0;1]; dPerp=[1;0;0];dAxial=[0;0;1];
for lambda=[0,.25,.75,1]
 G=(1-lambda)*eye(3)+lambda*(n*n.');
 E=eig(G);
 rows(end+1,:)={lambda,min(E),max(E),rank(G,1e-10), ...
     norm(dPerp.'*G,2), norm(dAxial.'*G,2), ...
     'ZHENG_EQ5_6_DIRECTIONAL_GAIN'}; %#ok<AGROW>
end
tab=cell2table(rows,'VariableNames',{'lambda','minEigenvalue','maxEigenvalue', ...
 'rankMatrix','lateralGain','axialGain','scope'});
% Paper boundary demonstration: choose sphere distance r=0.5, approaching.
r=.5; radialVelocity=-.2;
h=.5*(r^2-r^2);hdot=r*radialVelocity;
fac=atan(hdot)+pi/2;
H=fac*h;LgH=h/(1+hdot^2)*r;
Hdot=fac*hdot; % h=0, so all remaining terms vanish
assert(h==0 && H==0 && LgH==0 && Hdot<0, ...
 'Theorem-1 boundary counterexample failed.');
end
