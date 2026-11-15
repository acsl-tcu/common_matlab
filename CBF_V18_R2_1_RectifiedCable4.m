function out=CBF_V18_R2_1_RectifiedCable4(x23,s,obs,opts)
% R2-1 pointwise high-order Rectified CBF candidate, using FULL 3D h-jet.
% On regular region where torque first enters at order 4, build 3 nested
% rectifications, then one CBF inequality linear in physical torque.
% Snap j does not act at order 4. DOES NOT certify mixed-order degenerate
% states, theorem hypotheses, global safety, or repeated feasibility.
if nargin<4,opts=struct();end
if ~isfield(opts,'k'),opts.k=1.2;end
if ~isfield(opts,'kR'),opts.kR=opts.k;end
if ~isfield(opts,'c'),opts.c=1e-4*ones(3,1);end
if ~isfield(opts,'dt'),opts.dt=.025;end
assert(numel(x23)==23&&all(opts.c(:)>0)&&numel(opts.c)==3&&opts.k>0);
u0=zeros(4,1);[H3,d0,stage,earlyPsi]=rectified(u0);
A=zeros(1,4);curvature=0;earlyAction=0;
for j=1:4
 e=zeros(4,1);e(j)=.001;
 [Hplus,dp]=rectified(u0+e);[Hminus,dm]=rectified(u0-e);
 A(j)=(dp-dm)/(.002);
 curvature=max(curvature,abs(dp+dm-2*d0));
 earlyAction=max(earlyAction,max(abs([Hplus-H3,Hminus-H3])));
end
rhs=-(d0+opts.kR*H3);
status='';torque=nan(3,1);j=NaN;res=NaN;
if H3< -1e-9
 status='INVALID_INITIAL_RECTIFIED_SET';
elseif abs(A(1))>1e-7 || curvature>1e-6 || earlyAction>1e-8
 status='SNAP_EARLY_OR_NONAFFINE_OR_RECTIFIED_INITIAL_INPUT_EFFECT';
elseif norm(A(2:4),2)<1e-10
 if rhs>1e-8,status='ZERO_TORQUE_AUTHORITY_POSITIVE_RHS';
 else,status='INPUT_INDEPENDENT_POINTWISE_ROW';torque=zeros(3,1);j=0;end
else
 try
    op=optimoptions('quadprog','Display','off');
    [sol,~,ef]=quadprog(2*eye(3),zeros(3,1),-A(2:4),-rhs,[],[],...
        s.input.torqueMin(:),s.input.torqueMax(:),[],op);
    if ef>0&&~isempty(sol)&&all(isfinite(sol))
      torque=sol;j=0;status='POINTWISE_R2_1_RECTIFIED_TORQUE_FEASIBLE_UNCERTIFIED';
    else,status='R2_1_TORQUE_QP_INFEASIBLE';end
 catch ME
    status=['R2_1_SOLVER_ERROR_' ME.identifier];
 end
end
% The candidate j=0 is checked against the FULL intersample physical
% thrust polynomial, not certified under all future trajectories.
if isfinite(j)
 t=[0;opts.dt];c=[j/6,x23(23)/2,x23(22),x23(21)];
 if any(abs(c)>1e-14)
     rr=roots(c);mask=abs(imag(rr))<1e-9 & real(rr)>=0 & real(rr)<=opts.dt;
     t=[t;real(rr(mask))];
 end
 th=x23(20)+x23(21)*t+x23(22)/2*t.^2+x23(23)/6*t.^3+j/24*t.^4;
 if min(th)<s.input.thrustMin-1e-8 || max(th)>s.input.thrustMax+1e-8
    status='R2_1_SNAP_CANDIDATE_PHYSICAL_THRUST_OUT_OF_BOUNDS';j=NaN;
 end
end
if isfinite(j),res=A*[j;torque]-rhs;end
out=struct('status',status,'hRectified3',H3,'stagePsi',stage,...
 'nearZeroPsi',earlyPsi,'inputRow',A,'rhs',rhs,'input',[j;torque],...
 'residual',res,'nonaffineAudit',curvature,'earlyInitialEffect',earlyAction,...
 'scope','ORIGINAL_3D_POINTWISE_R2_1_TORQUE_ORDER4_ONLY_NO_GLOBAL_CBF_PROOF');
    function [Hout,hdot,stage0,near0]=rectified(u)
       Hj=CBF_V18_CableOnly('R2',x23,u,s,obs,4);
       stage0=zeros(3,1);near0=0;
       for ii=1:3
           len=numel(Hj);
           wjet=(1:len-1).*Hj(2:end)+opts.k*Hj(1:end-1);
           stage0(ii)=wjet(1);
           if abs(wjet(1))<1e-9,near0=near0+1;end
           if wjet(1)<0
              pp=1;
              for pw=1:5
                pp=conv(pp,-wjet);
                pp=pp(1:min(numel(pp),len-1));
              end
              penalty=opts.c(ii)*pp(1:len-1);
           else,penalty=zeros(1,len-1);end
           Hj=Hj(1:end-1)-penalty;
       end
       Hout=Hj(1);hdot=Hj(2);
    end
end
