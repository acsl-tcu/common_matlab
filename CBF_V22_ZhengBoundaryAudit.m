function T=CBF_V22_ZhengBoundaryAudit()
% Reproduces paper Eq.(21),(23),(26) boundary issue for approaching sphere.
% h=0, L_f h<0 => H=0 and L_g H=0, Hdot<0 regardless of finite control.
r=.5;v=-.2;h=.5*(r^2-r^2);hdot=r*v;
gain=atan(hdot)+pi/2;
H=gain*h;
LgH=h/(1+hdot^2)*r;  % unit-acceleration virtual control in radial x
Hdot=gain*hdot; % exact at h=0: the h*d/dt(atan(hdot)) term vanishes
pass=(h==0 && H==0 && LgH==0 && Hdot<0);
assert(pass,'Zheng boundary algebra counterexample must hold.');
T=table(r,v,h,hdot,H,LgH,Hdot,pass, ...
 'VariableNames',{'separationRadius','approachVelocity','h','Lfh','H','LgH','Hdot','boundaryCounterexample'});
end
