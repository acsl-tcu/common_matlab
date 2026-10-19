function S = af7r62_observer_one_step_kernel(h,u,H,q0,Pnorm,mLint,Jd,m,g,L)
% AF7R62 exact numeric observer one-step kernel.
% Algebra/order copied from production REPLANNING_SYSTEM.observer_one_step_support.

Jd=double(Jd(:));
u=double(u(:));
q0=double(q0(:));
mLint=double(mLint(:));

Jmin=min(Jd); Jmax=max(Jd);
tauNorm=norm(u(2:4)); u1abs=abs(u(1));
W0=double(H.w_norm_ub); WL0=double(H.wL_norm_ub);
VL0=double(H.vL_norm_ub);
Pnorm=double(Pnorm); mLlb=mLint(1);

V0=0.5*Jmax*W0^2;
sqrtVend=sqrt(V0)+tauNorm*h/sqrt(2*Jmin);
VwEnd=sqrtVend^2;
Wabs0=sqrt(2*V0/Jmin); WabsEnd=sqrt(2*VwEnd/Jmin);
Iw=Wabs0*h+tauNorm*h^2/(2*Jmin);

Theta=q0(2)+Iw;
pitchClosed=Theta<pi/2;
qAbs=[Inf;Theta;Inf]; fullClosed=false;
if pitchClosed
    qAbs(1)=q0(1)+(1+tan(Theta))*Iw;
    qAbs(3)=q0(3)+(1/cos(Theta))*Iw;
    fullClosed=qAbs(1)<pi&&qAbs(3)<pi;
end

aWL=Pnorm*u1abs/(m*L);
WLEnd=WL0+aWL*h;
IWL=WL0*h+0.5*aWL*h^2;
pTSpan=Pnorm*IWL; %#ok<NASGU>
den=m+mLlb;
assert(den>0,'[REPLANNER OBSERVER ERROR] total mass lower bound invalid.');
Aconst=g+(Pnorm^2*u1abs)/den;
Aquad=(Pnorm^3)*m*L/den;
intWL2=WL0^2*h+WL0*aWL*h^2+(aWL^2*h^3)/3;
VLEnd=VL0+Aconst*h+Aquad*intWL2;
ALmax=Aconst+Aquad*(WLEnd^2);

S=struct();
S.closed=pitchClosed&&fullClosed; S.h=h; S.held_input=u;
S.Jdiag=Jd; S.w_norm_ub_end=WabsEnd;
S.q_abs_ub_interval=qAbs;
S.wL_accel_norm_ub=aWL; S.wL_norm_ub_end=WLEnd;
S.vL_accel_norm_ub=ALmax; S.vL_norm_ub_end=VLEnd; S.pT_norm_ub=Pnorm;
S.sampled_acceptance=false; S.guessed_bound=false;
end
