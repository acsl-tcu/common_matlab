function D = AF7R58R164_FLAT_DYNAMICS(route,live,work)
% Offline, necessary sample checks + sufficient double Bernstein bounds.
% Actual taut suspended-load flat model: pQ=pL+L*(pL''+g*e3)/norm(...).
% Uses EXACT user-provided live physical parameters, not fabricated defaults.
% No HLC tracking, motor torques, robust tube, IEEE outward rounding, or
% safe-flight/commit claim. Works in all XYZ directions without altitude rules.
if nargin<3 || isempty(work),work=struct('max_leaves',128,'samples_per_segment',41);end
assert(isscalar(work.max_leaves)&&work.max_leaves>=1&&fix(work.max_leaves)==work.max_leaves);
assert(isscalar(work.samples_per_segment)&&work.samples_per_segment>=3&&fix(work.samples_per_segment)==work.samples_per_segment);
D=struct('status','UNKNOWN_LIVE_PHYSICAL_AUTHORITY_REQUIRED', ...
    'sample_rejected',false,'conditional_sufficient_reference_bounds',false, ...
    'source_is_saved_event',true,'actual_HLC_torque_certified',false, ...
    'continuous_execution_certified',false,'production_commit_authorized',false, ...
    'sample_peak_uav_speed_mps',NaN,'sample_peak_thrust_N',NaN, ...
    'sample_min_thrust_N',NaN,'sample_min_tension_N',NaN,'sample_max_tension_N',NaN, ...
    'interval_tension_certified',false, ...
    'interval_min_tension_lower_N',NaN,'interval_max_tension_upper_N',NaN, ...
    'interval_max_uav_speed_upper_mps',NaN, ...
    'interval_max_thrust_upper_N',NaN,'interval_min_thrust_lower_N',NaN, ...
    'interval_min_specific_force_lower_mps2',NaN, ...
    'evaluated_segments',0,'split_nodes',0,'violation',{{}}, ...
    'reference_samples',zeros(0,10));
required={'vehicle_mass_kg','latest_EKF_mL_kg','cable_length_m', ...
    'gravity_m_s2','thrust_bounds_N','finite_sensing_vQ_limit_m_s'};
if isempty(live)||~isstruct(live)||~isscalar(live)||~all(isfield(live,required))
    D.violation={'Missing one or more authoritative event parameters; no pass inferred.'};return;
end
mQ=double(live.vehicle_mass_kg);mL=double(live.latest_EKF_mL_kg);
L=double(live.cable_length_m);g=double(live.gravity_m_s2);
fB=double(live.thrust_bounds_N(:));vLim=double(live.finite_sensing_vQ_limit_m_s);
assert(all(isfinite([mQ;mL;L;g;fB;vLim]))&&mQ>0&&mL>0&&L>0&&g>0&& ...
 numel(fB)==2&&fB(1)>=0&&fB(2)>fB(1)&&vLim>0, ...
 'R164:LiveParameterAuthority','Invalid physical data.');
hasTension=isfield(live,'tension_bounds_N');
if hasTension
 tB=double(live.tension_bounds_N(:));
 assert(numel(tB)==2&&all(isfinite(tB))&&tB(1)>=0&&tB(2)>tB(1), ...
  'R164:InvalidTensionAuthority');
else
 tB=[0;Inf];
end
C=route.C7;
if ~isstruct(C)||~isfield(C,'feasible')||~C.feasible||~isfield(C,'control_points')|| ...
   ~isfield(C,'times_s')||numel(C.control_points)~=numel(C.times_s)-1
 D.status='UNKNOWN_ROUTE_WITHOUT_C7_CONTROL_POINTS';return;
end
t=double(C.times_s(:)');
assert(all(isfinite(t))&&all(diff(t)>0),'R164:SegmentDuration');
sp=0;fp=0;fm=Inf;tn=Inf;tx=0;sf=Inf;samples=[];
for s=1:numel(C.control_points)
 P=r164_cp(C.control_points{s});T=t(s+1)-t(s);
 for u=linspace(0,1,work.samples_per_segment)
  v=r164_eval(P,1,u,T);a=r164_eval(P,2,u,T);
  j=r164_eval(P,3,u,T);snap=r164_eval(P,4,u,T);
  F=a+[0;0;g];sigma=norm(F);
  if ~(isfinite(sigma)&&sigma>0)
   D.sample_rejected=true;D.violation{end+1}='Sampled taut model singular';
   continue;
  end
  q=F/sigma;sdot=q'*j;
  qdot=(j-q*sdot)/sigma;
  sddot=qdot'*j+q'*snap;
  qddot=(snap-q*sddot-2*qdot*sdot)/sigma;
  vQ=v+L*qdot;fvec=(mQ+mL)*F+mQ*L*qddot;
  tension=mL*sigma;fmag=norm(fvec);vmag=norm(vQ);
  sp=max(sp,vmag);fp=max(fp,fmag);fm=min(fm,fmag);tn=min(tn,tension);tx=max(tx,tension);sf=min(sf,sigma);
  samples(end+1,:)=[t(s)+u*T,vmag,fmag,tension, ...
      q(:)',norm(qdot),norm(qddot),sigma]; %#ok<AGROW>
  if hasTension&&(tension<tB(1)-128*eps(max(1,tB(1))) || ...
                  tension>tB(2)+128*eps(max(1,tB(2))))
      D.sample_rejected=true;end
  if vmag>vLim+128*eps(max(1,vLim))
      D.sample_rejected=true;end
  if fmag<fB(1)-128*eps(max(1,fB(1)))||fmag>fB(2)+128*eps(max(1,fB(2)))
      D.sample_rejected=true;end
 end
end
D.reference_samples=samples;
D.sample_peak_uav_speed_mps=sp;
D.sample_peak_thrust_N=fp;
D.sample_min_thrust_N=fm;
D.sample_min_tension_N=tn;D.sample_max_tension_N=tx;
D.interval_min_specific_force_lower_mps2=sf;
if D.sample_rejected
 D.status='REJECTED_SAMPLED_TAUT_REFERENCE_DYNAMICS';
 D.violation{end+1}='Sampled necessary speed/thrust or taut-force condition failed.';
 % A sampled violation is sufficient to reject, not to certify good routes.
 return;
end
hiF=0;loF=Inf;hiV=0;loSig=Inf;loT=Inf;hiT=0;
for s=1:numel(C.control_points)
 P=r164_cp(C.control_points{s}); T=t(s+1)-t(s);
 q=struct('P',P,'T',T); tested=0;
 while ~isempty(q)
  x=q(end);q(end)=[];tested=tested+1;
  [lb,ub,vub,siglb]=r164_hull(x.P,x.T,mQ,mL,L,g);
  guard=1024*eps(max([1;abs(lb);abs(ub);abs(vub);abs(fB);abs(vLim)]));
  tensionPass=~hasTension || (siglb>0 && mL*siglb>=tB(1)+guard && ...
    mL*r164_Fhi(x.P,x.T,g)<=tB(2)-guard);
  if siglb>0 && lb>=fB(1)+guard && ub<=fB(2)-guard && vub<=vLim-guard && tensionPass
   loF=min(loF,lb);hiF=max(hiF,ub);hiV=max(hiV,vub);
   loSig=min(loSig,siglb);
   loT=min(loT,mL*siglb);hiT=max(hiT,mL*r164_Fhi(x.P,x.T,g));
   continue;
  end
  if tested+numel(q)+2>work.max_leaves
   D.status='UNKNOWN_INTERVAL_TAUT_THRUST_SPEED_ENCLOSURE_OR_BUDGET';
   D.split_nodes=D.split_nodes+tested;return;
  end
  [a,b]=r164_split(x.P);
  q(end+1)=struct('P',b,'T',x.T/2); %#ok<AGROW>
  q(end+1)=struct('P',a,'T',x.T/2); %#ok<AGROW>
 end
 D.split_nodes=D.split_nodes+tested;D.evaluated_segments=s;
end
D.interval_max_thrust_upper_N=hiF;
D.interval_min_thrust_lower_N=loF;
D.interval_max_uav_speed_upper_mps=hiV;
D.interval_min_specific_force_lower_mps2=loSig;
if hasTension
 D.interval_tension_certified=true;
 D.interval_min_tension_lower_N=loT;D.interval_max_tension_upper_N=hiT;
end
D.conditional_sufficient_reference_bounds=true;
D.status='CONDITIONAL_BERNSTEIN_TAUT_REFERENCE_FORCE_SPEED__NO_TORQUE_OR_FLIGHT';
end
function P=r164_cp(raw)
P=double(raw);
if isequal(size(P),[16 3]),P=P';end
assert(isequal(size(P),[3 16])&&all(isfinite(P(:))),'R164:SavedCPShape');
end
function y=r164_eval(P,k,u,T)
N=size(P,2)-1;
for j=1:k,P=(N-j+1)/T*diff(P,1,2);end
while size(P,2)>1,P=(1-u)*P(:,1:end-1)+u*P(:,2:end);end
y=P(:,1);
end
function [lo,hi,vu,siglb]=r164_hull(P,T,mQ,mL,L,g)
A=(15*14/T^2)*diff(P,2,2)+[0;0;g];
J=(15*14*13/T^3)*diff(P,3,2);
S=(15*14*13*12/T^4)*diff(P,4,2);
V=(15/T)*diff(P,1,2);
ref=mean(A,2);rho=max(vecnorm(A-ref,2,1));
siglb=norm(ref)-rho;
lo=-Inf;hi=Inf;vu=Inf;
if ~(isfinite(siglb)&&siglb>0),return;end
Fhi=max(vecnorm(A,2,1));Jhi=max(vecnorm(J,2,1));Shi=max(vecnorm(S,2,1));
qdotHi=Jhi/siglb;
qddotHi=Shi/siglb+3*qdotHi^2;
vu=max(vecnorm(V,2,1))+L*qdotHi;
hi=(mQ+mL)*Fhi+mQ*L*qddotHi;
lo=max(0,(mQ+mL)*siglb-mQ*L*qddotHi);
end
function hi=r164_Fhi(P,T,g)
A=(15*14/T^2)*diff(P,2,2)+[0;0;g];
hi=max(vecnorm(A,2,1));
end
function [A,B]=r164_split(P)
n=size(P,2);X=P;A=zeros(size(P));B=zeros(size(P));
A(:,1)=X(:,1);B(:,end)=X(:,end);
for k=2:n
 X=(X(:,1:end-1)+X(:,2:end))/2;
 A(:,k)=X(:,1);B(:,n-k+1)=X(:,end);
end
end
