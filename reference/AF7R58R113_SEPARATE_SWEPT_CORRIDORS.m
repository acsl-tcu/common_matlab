function C=AF7R58R113_SEPARATE_SWEPT_CORRIDORS(route,obs,live,opts)
% R113 -- continuous-time, separate UAV / taut cable / payload corridors.
% Real reference pL is a degree-15 Bezier; existing flat model reconstructs
% pQ = pL + L*(pL''+g*e3)/norm(pL''+g*e3). NO whole-system bounding ball.
% The cable is the complete segment [(1-lambda)*pL+lambda*pQ], lambda in [0,1].
% Each leaf gives THREE independent oriented-ellipsoid separating normals.
% Velocity-derived swept enclosures cover the WHOLE leaf, not sampled times.
% R99-style separate sizes; R80-style independent full-time certificates.
% Scope: static KNOWN obstacles and taut REFERENCE geometry; not real HLC,
% uncertain execution, IEEE outward rounding, or production authorization.
assert(isstruct(route)&&isscalar(route)&&isfield(route,'segments')&& ...
  ~isempty(route.segments)&&isstruct(route.segments), ...
  'R113:Route','Need nonempty C7 reference segments.');
assert(isstruct(obs)&&isstruct(live)&&isscalar(live)&& ...
  isstruct(opts)&&isscalar(opts),'R113 failure - Inputs');
need={'gravity_m_s2','cable_length_m','rQ_m','rL_m','rC_m', ...
  'd_margin_m','robust_tracking_margin_enabled'};
for k=1:numel(need)
 assert(isfield(live,need{k}),'R113:MissingRuntimeAuthority', ...
   'Missing authoritative %s',need{k});
end
pnames={'gravity_m_s2','cable_length_m','rQ_m','rL_m','rC_m','d_margin_m'};
for k=1:numel(pnames)
 z=live.(pnames{k});
 assert(isnumeric(z)&&isscalar(z)&&isfinite(z)&&z>=0, ...
   'R113:InvalidRuntimeAuthority','Invalid %s',pnames{k});
end
assert(live.gravity_m_s2>0&&live.cable_length_m>0,'R113 failure - Physics');
robust=double(live.robust_tracking_margin_enabled);
assert(isscalar(robust)&&any(robust==[0 1]),'R113 failure - RobustSwitch');
tube=0;
if robust==1
 assert(isfield(live,'tracking_tube_m')&& ...
   isnumeric(live.tracking_tube_m)&&isscalar(live.tracking_tube_m)&& ...
   isfinite(live.tracking_tube_m)&&live.tracking_tube_m>=0, ...
   'R113:MissingCertifiedTube','Robust ON requires source-authorized tube.');
 tube=live.tracking_tube_m;
end
assert(isfield(opts,'max_nodes_per_check')&& ...
 isnumeric(opts.max_nodes_per_check)&&isscalar(opts.max_nodes_per_check)&& ...
 isfinite(opts.max_nodes_per_check)&&opts.max_nodes_per_check>=1&& ...
 fix(opts.max_nodes_per_check)==opts.max_nodes_per_check, ...
 'R113:WorkBudget','Provide a positive finite work budget.');
maxNodes=opts.max_nodes_per_check;
for k=1:numel(obs)
 o=obs(k);
 assert(all(isfield(o,{'center','R','axes'}))&& ...
  isequal(size(o.center),[3 1])&&isequal(size(o.R),[3 3])&& ...
  isequal(size(o.axes),[3 1])&& ...
  all(isfinite([o.center;o.R(:);o.axes]))&&all(o.axes>0)&& ...
  norm(o.R'*o.R-eye(3),'fro')<1e-9&& ...
  abs(det(o.R)-1)<1e-9, ...
  'R113:Obstacle','Known ellipsoids must have valid SO(3) geometry.');
end
M=numel(obs);
C=struct('status','INCONCLUSIVE_NOT_EVALUATED', ...
 'reference_geometry_certified',false,'separate_body_planes',true, ...
 'whole_body_ball_used',false,'continuous_time',false, ...
 'nodes',0,'leaves',0,'minimum_margin_m',NaN, ...
 'max_c7_seam_residual',NaN,'numeric_c7_seams_verified',false, ...
 'first_unresolved_segment',NaN,'known_obstacle_count',M, ...
 'cells',struct([]),'robust_inflation_applied_m',tube, ...
 'unknown_space_proved',false,'real_hlc_closed_loop_proved',false, ...
 'ieee_directed_rounding_proved',false,'production_commit_authorized',false);
if M==0
 C.status='NO_KNOWN_OBSTACLE_COVERAGE__NOT_UNKNOWN_FREE';return;
end
cellTemplate=struct('segment',0,'t0',0,'t1',0, ...
 'normal_L',zeros(3,M),'normal_Q',zeros(3,M), ...
 'normal_C',zeros(3,M),'margin_L_m',zeros(1,M), ...
 'margin_Q_m',zeros(1,M),'margin_C_m',zeros(1,M), ...
 'speed_L_ub_m_s',NaN,'speed_Q_ub_m_s',NaN, ...
 'specific_force_lb_m_s2',NaN);
result=repmat(cellTemplate,1,0);
lastRight=[];lastEnd=NaN;seamWorst=0;
for s=1:numel(route.segments)
 seg=route.segments(s);
 assert(all(isfield(seg,{'t0','t1','payload_cp','trajectory_frame'}))&& ...
  strcmp(char(seg.trajectory_frame),'HLC_PAYLOAD_WORLD')&& ...
  isfinite(seg.t0)&&isfinite(seg.t1)&&seg.t1>seg.t0&& ...
  isequal(size(seg.payload_cp),[3 16])&& ...
  all(isfinite(seg.payload_cp(:))), ...
  'R113:Segment','Only finite HLC payload degree-15 C7 segments.');
 [leftJet,rightJet]=local_jets(double(seg.payload_cp),seg.t1-seg.t0);
 if s>1
  timeTol=128*eps(max([1,abs(seg.t0),abs(lastEnd)]));
  if abs(seg.t0-lastEnd)>timeTol
   C.status='INCONCLUSIVE_SEGMENT_TIME_GAP';
   C.first_unresolved_segment=s;return;
  end
  seam=max(abs(leftJet(:)-lastRight(:)));
  seamWorst=max(seamWorst,seam);
  mag=max([1;abs(leftJet(:));abs(lastRight(:))]);
  tscale=max(15/(seg.t1-seg.t0),15/(lastEnd-lastStart));
  tol=8192*eps(mag)*max(1,tscale^7);
  if ~(isfinite(seam)&&seam<=tol)
   C.status='INCONCLUSIVE_C7_SEAM_MISMATCH';
   C.first_unresolved_segment=s;
   C.max_c7_seam_residual=seam;return;
  end
 end
 lastRight=rightJet;lastStart=seg.t0;lastEnd=seg.t1;
 todo={struct('cp',double(seg.payload_cp),'a',double(seg.t0), ...
   'b',double(seg.t1))};
 while ~isempty(todo)
  if C.nodes>=maxNodes
   C.status='INCONCLUSIVE_SWEEP_RESOURCE_BUDGET';
   C.first_unresolved_segment=s;C.cells=result;return;
  end
  X=todo{end};todo(end)=[];C.nodes=C.nodes+1;
  [pass,leaf]=local_leaf(X.cp,X.a,X.b,obs,live,tube,cellTemplate);
  if pass
   leaf.segment=s;result(end+1)=leaf; %#ok<AGROW>
   C.leaves=C.leaves+1;
  else
   mid=X.a+0.5*(X.b-X.a);
   if ~(mid>X.a && mid<X.b)
    C.status='INCONCLUSIVE_SWEEP_FLOAT_RESOLUTION';
    C.first_unresolved_segment=s;C.cells=result;return;
   end
   [A,B]=local_split(X.cp);
   todo{end+1}=struct('cp',B,'a',mid,'b',X.b); %#ok<AGROW>
   todo{end+1}=struct('cp',A,'a',X.a,'b',mid); %#ok<AGROW>
  end
 end
end
C.cells=result;C.max_c7_seam_residual=seamWorst;
C.numeric_c7_seams_verified=true;
C.minimum_margin_m=min(cellfun(@min,{result.margin_L_m}));
C.minimum_margin_m=min(C.minimum_margin_m, min(cellfun(@min,{result.margin_Q_m})));
C.minimum_margin_m=min(C.minimum_margin_m, min(cellfun(@min,{result.margin_C_m})));
C.reference_geometry_certified=true;C.continuous_time=true;
C.status='NUMERIC_CONTINUOUS_SEPARATE_Q_C_L_REFERENCE_PASS__NO_EXECUTION_PROOF';
end

function [pass,leaf]=local_leaf(P,a,b,obs,live,tube,leaf)
pass=false;T=b-a;L=double(live.cable_length_m);g=double(live.gravity_m_s2);
Vcp=(15/T)*diff(P,1,2);
Acp=(14/T)*diff(Vcp,1,2);
Jcp=(13/T)*diff(Acp,1,2);
vL=max(vecnorm(Vcp,2,1));j=max(vecnorm(Jcp,2,1));
F=Acp+repmat([0;0;g],1,size(Acp,2));
nF=mean(F,2);fn=norm(nF);
if ~(isfinite(fn)&&fn>0),return;end
nF=nF/fn;sigmaLB=min(nF'*F);
if ~(isfinite(sigmaLB)&&sigmaLB>256*eps(max([1;abs(F(:))]))),return;end
pLm=local_eval(P,0.5);fmid=local_eval(F,0.5);
fm=norm(fmid);if ~(isfinite(fm)&&fm>=sigmaLB),return;end
pQm=pLm+L*fmid/fm;
vQ=vL+L*j/sigmaLB;
if ~all(isfinite([pLm;pQm;vL;vQ])),return;end
travelL=vL*T/2;travelQ=vQ*T/2;
extra=double(live.d_margin_m)+tube;
rL=live.rL_m+extra;rQ=live.rQ_m+extra;rC=live.rC_m+extra;
leaf.t0=a;leaf.t1=b;leaf.speed_L_ub_m_s=vL;
leaf.speed_Q_ub_m_s=vQ;leaf.specific_force_lb_m_s2=sigmaLB;
for jObs=1:numel(obs)
 o=obs(jObs);c=double(o.center);R=double(o.R);axes=double(o.axes);
 d=pQm-pLm;
 if d'*d>0
  tau=max(0,min(1,((c-pLm)'*d)/(d'*d)));
 else
  tau=0;
 end
 pc=pLm+tau*d;
 D=[pLm-c,pQm-c,pc-c,(pLm+pQm)/2-c,R,-R];
 for z=1:4
  dir=D(:,z);nr=norm(dir);
  if nr>0
   D(:,end+1)=R*((R'*dir)./(axes.^2)); %#ok<AGROW>
  end
 end
 [mL,nL]=best_margin(D,o,pLm,pQm,'L',rL,travelL,travelQ);
 [mQ,nQ]=best_margin(D,o,pLm,pQm,'Q',rQ,travelL,travelQ);
 [mC,nC]=best_margin(D,o,pLm,pQm,'C',rC,travelL,travelQ);
 if ~(mL>0&&mQ>0&&mC>0),return;end
 leaf.normal_L(:,jObs)=nL;leaf.normal_Q(:,jObs)=nQ;
 leaf.normal_C(:,jObs)=nC;
 leaf.margin_L_m(jObs)=mL;leaf.margin_Q_m(jObs)=mQ;
 leaf.margin_C_m(jObs)=mC;
end
pass=true;
end

function [best,win]=best_margin(D,o,pL,pQ,kind,r,deltaL,deltaQ)
best=-Inf;win=zeros(3,1);
for k=1:size(D,2)
 n=D(:,k);nn=norm(n);
 if ~isfinite(nn)||nn==0,continue;end
 n=n/nn;
 h=n'*o.center+norm(o.axes.*(o.R'*n));
 switch kind
  case 'L', low=n'*pL-deltaL;
  case 'Q', low=n'*pQ-deltaQ;
  otherwise, low=min(n'*pL-deltaL,n'*pQ-deltaQ);
 end
 margin=low-h-r;
 guard=512*eps(max([1,abs(low),abs(h),abs(r),abs(deltaL),abs(deltaQ)]));
 margin=margin-guard;
 if margin>best,best=margin;win=n;end
end
end

function q=local_eval(P,u)
while size(P,2)>1
 P=(1-u)*P(:,1:end-1)+u*P(:,2:end);
end
q=P(:,1);
end

function [A,B]=local_split(P)
n=size(P,2);A=zeros(size(P));B=zeros(size(P));
A(:,1)=P(:,1);B(:,end)=P(:,end);
for k=2:n
 P=0.5*(P(:,1:end-1)+P(:,2:end));
 A(:,k)=P(:,1);B(:,n-k+1)=P(:,end);
end
end

function [J0,J1]=local_jets(P,T)
J0=zeros(3,8);J1=zeros(3,8);D=P;
for k=0:7
 J0(:,k+1)=D(:,1);J1(:,k+1)=D(:,end);
 if k<7,D=(size(D,2)-1)/T*diff(D,1,2);end
end
end
