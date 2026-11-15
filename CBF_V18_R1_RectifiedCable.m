function out=CBF_V18_R1_RectifiedCable(x19,s,obs,opts)
% R1: physical [T;tau] one-row ReCBF with geometric relative degree 2 to T.
% Torques do NOT enter the 2nd-order row; this is NOT a universal lateral
% avoidance solution. No slack or deletion of uncontrollable constraints.
if nargin<4,opts=struct();end
if ~isfield(opts,'k'),opts.k=1.2;end
if ~isfield(opts,'kR'),opts.kR=opts.k;end
if ~isfield(opts,'c'),opts.c=0.5;end
assert(opts.k>0&&opts.kR>0&&opts.c>0);
nom=[(s.physical.massDrone+s.physical.massLoad)*s.physical.gravity;0;0;0];
H=CBF_V18_CableOnly('R1',x19,nom,s,obs,2);
step=0.1;e=[step;0;0;0];
HP=CBF_V18_CableOnly('R1',x19,nom+e,s,obs,2);
HM=CBF_V18_CableOnly('R1',x19,nom-e,s,obs,2);
aT=(HP(3)-HM(3))/step; % 2nd derivative w.r.t. T
nonAffine=abs(2*(HP(3)+HM(3)-2*H(3)));
% Check torques have NO order-2 effect even when perturbed jointly.
maxTorqueEffect=0;
for jj=2:4
    e=zeros(4,1);e(jj)=0.1;
    HH=CBF_V18_CableOnly('R1',x19,nom+e,s,obs,2);
    maxTorqueEffect=max(maxTorqueEffect,max(abs((HH-H).*factorial(0:2))));
end
h=H(1);hd=H(2);hdd0=2*H(3);
w=hd+opts.k*h;hR=h-opts.c*min(w,0)^2;
fac=-2*opts.c*min(w,0);
A=fac*aT;
base=hd+fac*(hdd0+opts.k*hd)+opts.kR*hR;
% base + A*(T-Tnom) >= 0
rhs=-base+A*nom(1);
lb=s.input.thrustMin;ub=s.input.thrustMax;
status='';T=NaN;
if hR < -1e-9
    status='INVALID_INITIAL_RECTIFIED_SET';
elseif nonAffine>1e-7 || maxTorqueEffect>1e-8
    status='NONAFFINE_OR_EARLY_TORQUE_EFFECT';
elseif abs(A)<1e-10
    if rhs>1e-9,status='ZERO_INPUT_AUTHORITY_POSITIVE_RHS';
    else,status='INPUT_INDEPENDENT_RECTIFIED_ROW';T=min(max(nom(1),lb),ub);end
else
    if A>0,lb=max(lb,rhs/A);else,ub=min(ub,rhs/A);end
    if lb>ub+1e-9,status='HARD_ROW_INFEASIBLE';
    else,status='POINTWISE_R1_RECTIFIED_ROW_FEASIBLE';T=min(max(nom(1),lb),ub);end
end
u=[T;0;0;0];
if isfinite(T),res=A*T-rhs;else,res=NaN;end
out=struct('status',status,'input',u,'h',h,'hdot',hd,'hR',hR,'w',w,...
    'thrustCoefficientH2',aT,'rectifiedCoefficient',A,'rectifiedRHS',rhs,...
    'qpResidual',res,'nonAffineH2',nonAffine,'earlyTorqueEffect',maxTorqueEffect,...
    'scope','POINTWISE_RECBF_R1_CABLE_ONLY_NOT_CLOSED_LOOP_CERTIFIED');
end
