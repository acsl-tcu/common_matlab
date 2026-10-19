function O = af7r61_observer_contract_kernel(C0,C1,F,xp,xq,xpL,xpT,xvL,xwL,xw)
% AF7R61 exact numeric two-event contraction kernel.
% Algebra/order copied from production REPLANNING_SYSTEM.observer_contract_two_events.

y0=double(C0.y(:)); y1=double(C1.y(:));
n0=double(C0.nu_abs_ub(:)); n1=double(C1.nu_abs_ub(:));
h=double(C1.event_time-C0.event_time);
assert(h>0&&isfinite(h),'[REPLANNER OBSERVER ERROR] contraction h invalid.');
iq=4:6; ipL=7:9; ipT=10:12;
MEAS=measured_error_box_local(xp,xq,xpL,xpT,C1);

VLF=double(F.vL_norm_ub_end);
ALmax=double(F.vL_accel_norm_ub);
dPL=(y1(ipL)-y0(ipL))/h;
rVL=0.5*ALmax*h+(norm(n0(ipL))+norm(n1(ipL)))/h;
cVL=dPL;
VLtrue=min(VLF,norm(cVL)+rVL);
eVL=min(norm(double(xvL(:)))+VLF, ...
    norm(double(xvL(:))-cVL)+rVL);

WLF=double(F.wL_norm_ub_end); Pbar=double(F.pT_norm_ub);
aWL=double(F.wL_accel_norm_ub);
pTddot=Pbar*(aWL+WLF^2);
dPT=(y1(ipT)-y0(ipT))/h;
rPTdot=0.5*pTddot*h+(norm(n0(ipT))+norm(n1(ipT)))/h;
cWL=cross(y1(ipT),dPT);
pTdotUb=Pbar*WLF;
rWL=norm(n1(ipT))*pTdotUb+norm(y1(ipT))*rPTdot;
WLtrue=min(WLF,norm(cWL)+rWL);
eWL=min(norm(double(xwL(:)))+WLF, ...
    norm(double(xwL(:))-cWL)+rWL);

Wf=double(F.w_norm_ub_end); Jd=double(F.Jdiag(:));
Jmin=min(Jd); Jmax=max(Jd); tauNorm=norm(double(F.held_input(2:4)));
Theta=double(F.q_abs_ub_interval(2));
assert(isfinite(Theta)&&Theta<pi/2, ...
    '[REPLANNER OBSERVER ERROR] Euler chart is not closed.');
tanT=tan(Theta); secT=1/cos(Theta); Kphi=1+tanT;
Aomega=tauNorm/Jmin+(Jmax/Jmin)*Wf^2;
adot=Aomega+Kphi*Wf^2;
qddotUb=norm([Aomega+secT^2*Wf^2+tanT*adot; ...
    Aomega+Kphi*Wf^2; secT*(tanT*Wf^2+adot)]);
dq=(y1(iq)-y0(iq))/h;
rQdot=0.5*qddotUb*h+(norm(n0(iq))+norm(n1(iq)))/h;
lim=[pi;pi/2;pi];
assert(all(abs(y0(iq))+n0(iq)<lim)&&all(abs(y1(iq))+n1(iq)<lim), ...
    '[REPLANNER OBSERVER ERROR] measured Euler box reaches chart boundary.');
Ey=euler_rate_inverse_ZYX_local(y1(iq));
cW=Ey*dq;
dphi=n1(4); dtheta=n1(5);
LE=sqrt(dtheta^2+2*dphi^2+2*(dphi+dtheta)^2);
Kq=sqrt(Kphi^2+1+secT^2); qdotUb=Kq*Wf;
EyNorm2=nextfloat_up_local( ...
    sqrt(1+abs(sin(double(y1(iq(2)))))));
rW=LE*qdotUb+EyNorm2*rQdot;
Wtrue=min(Wf,norm(cW)+rW);
eW=min(norm(double(xw(:)))+Wf,norm(double(xw(:))-cW)+rW);

O=struct(); O.closed=true; O.h=h; O.measured_error=MEAS;
O.vL=struct('center',cVL,'radius',rVL,'true_norm_ub',VLtrue, ...
    'observer_error_norm_ub',eVL);
O.wL=struct('center',cWL,'radius',rWL,'true_norm_ub',WLtrue, ...
    'observer_error_norm_ub',eWL);
O.w=struct('center',cW,'radius',rW,'true_norm_ub',Wtrue, ...
    'observer_error_norm_ub',eW);
end

function M = measured_error_box_local(xp,xq,xpL,xpT,C)
y=double(C.y(:)); nu=double(C.nu_abs_ub(:)); M=struct();
M.p=struct('center',double(xp(:))-y(1:3),'radius',nu(1:3),'abs_ub',abs(double(xp(:))-y(1:3))+nu(1:3));
M.q=struct('center',double(xq(:))-y(4:6),'radius',nu(4:6),'abs_ub',abs(double(xq(:))-y(4:6))+nu(4:6));
M.pL=struct('center',double(xpL(:))-y(7:9),'radius',nu(7:9),'abs_ub',abs(double(xpL(:))-y(7:9))+nu(7:9));
M.pT=struct('center',double(xpT(:))-y(10:12),'radius',nu(10:12),'abs_ub',abs(double(xpT(:))-y(10:12))+nu(10:12));
end

function E = euler_rate_inverse_ZYX_local(q)
phi=q(1); theta=q(2);
E=[1,0,-sin(theta);0,cos(phi),sin(phi)*cos(theta); ...
    0,-sin(phi),cos(phi)*cos(theta)];
end

function y = nextfloat_up_local(x)
x=double(x);
if isnan(x)||x==Inf,y=x;return;end
assert(x>=0,'[REPLANNER OBSERVER ERROR] nextfloat input must be nonnegative.');
if x==0,y=realmin*eps(1);return;end
u=typecast(x,'uint64'); u=u+uint64(1); y=typecast(u,'double');
end
