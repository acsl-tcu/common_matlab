function [Z,S] = AF7R58R171_INTERVAL_TRIGGER(pred,ledger,live,proof,S)
% R171 current-EKF, full-cable 4s PREVENTIVE trigger / conditional clearance.
% Balls: UAV rQ, payload rL; cable: swept straight finite-width capsule rC.
% Never use a sphere of radius cable length. All observed obstacles checked
% simultaneously. Without genuine continuous HLC/plant bounds: UNKNOWN.
% The separating-normal search is a sufficient certificate search only;
% failure to find a plane NEVER proves collision. Floating-point arithmetic
% is not outward rounded, so this routine NEVER certifies a Flight commit.
if nargin<5 || isempty(S)
    S=struct('ids',zeros(0,1),'versions',zeros(0,1), ...
       'reported',false(0,1),'accepted',false(0,1));
end
Z=struct('status','UNKNOWN_MISSING_CURRENT_EKF_PREDICTION', ...
 'request',false,'model_risk',false,'unknown',true, ...
 'backup_or_reachability_proof_required',false, ...
 'proof_usable_conditional',false,'conditional_all_clear',false, ...
 'new_event_ids',zeros(0,1),'pending_event_ids',zeros(0,1), ...
 'failed_component','','first_unresolved_obstacle_id',NaN, ...
 'first_risk_interval_index',NaN, ...
 'tested_obstacles',0,'tested_intervals',0,'source_time_s',NaN, ...
 'used_radii_order','[rQ;rC;rL]', ...
 'used_length_sphere',false,'flight_trigger_swapped',false, ...
 'production_commit_authorized',false);
reqP={'t_s','Q_m','L_m','initialized_from_current_EKF', ...
 'ekf_timestamp_s','mass_used_by_prediction_kg'};
reqL={'t_s','estimated_mL_kg','source_horizon_s','source_radii_m', ...
 'position_Q_m','position_L_m','live_physical_parameters_valid'};
if ~isstruct(pred)||~isscalar(pred)||~all(isfield(pred,reqP))|| ...
 ~isstruct(live)||~isscalar(live)||~all(isfield(live,reqL))|| ...
 ~isequal(live.live_physical_parameters_valid,true)|| ...
 ~isequal(pred.initialized_from_current_EKF,true)|| ...
 ~isequal(double(pred.ekf_timestamp_s),double(live.t_s))|| ...
 ~isequal(double(pred.mass_used_by_prediction_kg),double(live.estimated_mL_kg))
 return;
end
Z.source_time_s=double(live.t_s);
t=double(pred.t_s(:)');Q=double(pred.Q_m);L=double(pred.L_m);
if isequal(size(Q),[numel(t),3]),Q=Q';end
if isequal(size(L),[numel(t),3]),L=L';end
r=double(live.source_radii_m(:));horizon=double(live.source_horizon_s);
if numel(t)<2||~isequal(size(Q),[3,numel(t)])|| ...
 ~isequal(size(L),size(Q))||numel(r)~=3|| ...
 any(~isfinite([t(:);Q(:);L(:);r(:);horizon]))|| ...
 any(r<0)||any(diff(t)<=0)||~isscalar(horizon)||horizon<=0|| ...
 horizon~=4 || abs(t(1)-live.t_s)>64*eps(max(1,abs(live.t_s)))|| ...
 abs(t(end)-t(1)-horizon)>64*eps(max([1,abs(t(1)),horizon]))|| ...
 norm(Q(:,1)-double(live.position_Q_m(:)),inf)> ...
      128*eps(max([1;abs(Q(:,1))]))|| ...
 norm(L(:,1)-double(live.position_L_m(:)),inf)> ...
      128*eps(max([1;abs(L(:,1))]))
 Z.status='UNKNOWN_STALE_TIME_GRID_OR_EKF_ENDPOINTS';return;
end
if isempty(ledger)
 Z.status='NO_OBSERVED_OBSTACLES__FREE_SPACE_NOT_GLOBALLY_CERTIFIED';
 Z.unknown=false;return;
end
requiredObs={'id','sensor_version','sensor_known', ...
 'occluded_in_range','previously_observed','type','p_center','R_obs'};
if ~isstruct(ledger)||~all(isfield(ledger,requiredObs))|| ...
 ~isstruct(S)||~all(isfield(S,{'ids','versions','reported','accepted'}))
 Z.status='UNKNOWN_OBSERVATION_PROVENANCE';return;
end
mask=[ledger.sensor_known] & ...
 (~[ledger.occluded_in_range] | [ledger.previously_observed]);
obs=ledger(mask);
if isempty(obs)
 Z.status='NO_UNOCCLUDED_OR_RETAINED_KNOWN_OBJECTS';Z.unknown=false;return;
end
ids=double([obs.id]);versions=double([obs.sensor_version]);
if numel(unique(ids))~=numel(ids)||any(~isfinite(ids))|| ...
 any(~isfinite(versions))
 Z.status='UNKNOWN_NONUNIQUE_OBSTACLE_ID_OR_VERSION';return;
end
nI=numel(t)-1;
% A proof must explicitly bind continuous velocity and prediction error
% bounds to THIS EKF event/mass and the whole interval partition.
boundsOK=isstruct(proof)&&isscalar(proof)&& ...
 all(isfield(proof,{'source_time_s','mass_used_kg', ...
 'continuous_HLC_plant_coverage_proven','producer', ...
 'velocity_Q_upper_mps','velocity_L_upper_mps', ...
 'error_Q_upper_m','error_L_upper_m'}));
if boundsOK
 boundsOK=isequal(double(proof.source_time_s),double(live.t_s))&& ...
 isequal(double(proof.mass_used_kg),double(live.estimated_mL_kg))&& ...
 isequal(proof.continuous_HLC_plant_coverage_proven,true)&& ...
 (ischar(proof.producer)||isstring(proof.producer))&& ...
 ~isempty(char(proof.producer));
end
if boundsOK
 vQ=double(proof.velocity_Q_upper_mps(:));
 vL=double(proof.velocity_L_upper_mps(:));
 eQ=double(proof.error_Q_upper_m(:));
 eL=double(proof.error_L_upper_m(:));
 boundsOK=all([numel(vQ),numel(vL),numel(eQ),numel(eL)]==nI)&& ...
 all(isfinite([vQ;vL;eQ;eL]))&&all([vQ;vL;eQ;eL]>=0);
else
 vQ=[];vL=[];eQ=[];eL=[];
end
Z.proof_usable_conditional=boundsOK;
% A numeric envelope attestation is NOT a trusted proof by itself.
Z.tested_obstacles=numel(obs);allclear=boundsOK;anyRisk=false;
for k=1:numel(obs)
 o=obs(k);risk=false;unresolved='';
 for i=1:nI
  h=t(i+1)-t(i);
  % With |p'|<=V, every point is within V*h/2 of one endpoint;
  % hence it lies in conv(endpoint pair) + Ball(V*h/2+E).
  % An entire straight cable is in the convex hull of its moving endpoints.
  if boundsOK
   bQ=vQ(i)*h/2+eQ(i);bL=vL(i)*h/2+eL(i);
   bC=max(bQ,bL);
  else
   bQ=0;bL=0;bC=0;
  end
  [sQ,valid]=separated([Q(:,i),Q(:,i+1)],r(1)+bQ,o,1);
  if ~valid,Z.status='UNKNOWN_UNSUPPORTED_OR_INVALID_ORIGINAL_SHAPE';return;end
  [sL,valid]=separated([L(:,i),L(:,i+1)],r(3)+bL,o,3);
  if ~valid,Z.status='UNKNOWN_UNSUPPORTED_OR_INVALID_ORIGINAL_SHAPE';return;end
  [sC,valid]=separated([Q(:,i),L(:,i),Q(:,i+1),L(:,i+1)],r(2)+bC,o,2);
  if ~valid,Z.status='UNKNOWN_UNSUPPORTED_OR_INVALID_ORIGINAL_SHAPE';return;end
  % Nonseparation is preventive RISK/UNKNOWN, NOT proven actual collision.
  if ~(sQ&&sL&&sC)
   risk=true;allclear=false;
   if isnan(Z.first_risk_interval_index) || i<Z.first_risk_interval_index || ...
     (i==Z.first_risk_interval_index && double(o.id)<Z.first_unresolved_obstacle_id)
    Z.first_risk_interval_index=i;
    Z.first_unresolved_obstacle_id=double(o.id);
    if ~sQ,Z.failed_component='UAV';
    elseif ~sL,Z.failed_component='PAYLOAD';
    else,Z.failed_component='CABLE';end
   end
   if isempty(unresolved)
    if ~sQ,unresolved='UAV';elseif ~sL,unresolved='PAYLOAD';
    else,unresolved='CABLE';end
   end
  end
  Z.tested_intervals=Z.tested_intervals+1;
 end
 if ~boundsOK,allclear=false;end
 if risk
  anyRisk=true;
  if isnan(Z.first_unresolved_obstacle_id)
   Z.first_unresolved_obstacle_id=double(o.id);
   if isempty(unresolved),Z.failed_component='UNKNOWN_ENCLOSURE';
   else,Z.failed_component=unresolved;end
  end
  j=find(double(S.ids(:))==double(o.id),1);
  if isempty(j)
   j=numel(S.ids)+1;S.ids(j,1)=o.id;S.versions(j,1)=o.sensor_version;
   S.reported(j,1)=false;S.accepted(j,1)=false;
  elseif S.versions(j)~=o.sensor_version
   S.versions(j)=o.sensor_version;
   S.reported(j)=false;S.accepted(j)=false;
  end
  if ~S.reported(j)
   Z.new_event_ids(end+1,1)=o.id; %#ok<AGROW>
   S.reported(j)=true;
  else
   Z.pending_event_ids(end+1,1)=o.id; %#ok<AGROW>
  end
  % Still request while unsafe/UNKNOWN; only suppress duplicate EVENT log.
  % A failed planner cannot permanently consume the hazard by latching it.
 end
end
Z.model_risk=anyRisk;Z.request=anyRisk;Z.unknown=~boundsOK || ~allclear;
Z.backup_or_reachability_proof_required=Z.unknown;
Z.conditional_all_clear=boundsOK&&allclear;
if Z.request
 Z.status='PREVENTIVE_CURRENT_EKF_WHOLE_BODY_REPLAN_REQUEST__NOT_PROVEN_COLLISION';
elseif Z.conditional_all_clear
 Z.status='CONDITIONAL_4S_WHOLE_BODY_SEPARATION__NOT_IEEE_OR_EXECUTION_CERTIFIED';
else
 Z.status='UNKNOWN_CURRENT_EKF_4S_SAFETY';
end
end

function [sep,valid]=separated(P,radius,o,part)
sep=false;valid=false;
if ~isequal(size(P,1),3)||isempty(P)||~isscalar(radius)|| ...
 radius<0||~isfinite(radius)||any(~isfinite(P(:))),return;end
c=double(o.p_center(:));R=double(o.R_obs);
if numel(c)~=3||~isequal(size(R),[3 3])|| ...
 any(~isfinite([c;R(:)])),return;end
% Candidate normals are 3D and covariant with the obstacle orientation.
% Their incompleteness only causes UNKNOWN, never a false clearance.
V=[eye(3),-eye(3),R,-R,P-c,mean(P,2)-c];
valid=true;
for j=1:size(V,2)
 n=V(:,j);nn=norm(n);
 if nn==0||~isfinite(nn),continue;end
 n=n/nn;
 [h,ok]=r176_original_shape_support(o,n);
 if ~ok,sep=false;valid=false;return;end
 % Shared exact ball/capsule support with C7 generation (R179).
 % P(:,1:4) for the cable is [Q_i,L_i,Q_(i+1),L_(i+1)].
 switch part
  case 1
   U=AF7R58R179_BODY_SUPPORT(P,P,[radius;0;0],n);
  case 2
   if size(P,2)~=4,sep=false;valid=false;return;end
   U=AF7R58R179_BODY_SUPPORT(P(:,[1 3]),P(:,[2 4]),[0;radius;0],n);
  case 3
   U=AF7R58R179_BODY_SUPPORT(P,P,[0;0;radius],n);
  otherwise
   sep=false;valid=false;return;
 end
 if ~U.valid,sep=false;valid=false;return;end
 lower=U.lower(part);
 guard=1024*eps(max([1;abs(P(:));abs(h);abs(radius)]));
 if lower>h+guard,sep=true;return;end
end
end

% Local geometry is source-bound: no MATLAB path shadowing possible.
function [h,valid]=r176_original_shape_support(o,n)
valid=false;h=NaN;
if ~isfield(o,'type'),return;end
R=double(o.R_obs);c=double(o.p_center(:));
if ~isequal(size(R),[3 3])||numel(c)~=3|| ...
 any(~isfinite([R(:);c]))||norm(R'*R-eye(3),'fro')>1e-7||det(R)<0.9,return;end
u=R'*n;typ=lower(char(o.type));
switch typ
 case 'cylinder'
  if ~isfield(o,'raw_param'),return;end
  p=double(o.raw_param(:));if numel(p)~=2||any(p<=0),return;end
  excess=p(1)*norm(u(1:2))+(p(2)/2)*abs(u(3));
 case {'sphere','ball'}
  if ~isfield(o,'raw_param'),return;end
  p=double(o.raw_param(:));if numel(p)~=1||p<=0,return;end
  excess=p;
 case 'ellipsoid'
  if ~isfield(o,'ellipsoid_radii'),return;end
  p=double(o.ellipsoid_radii(:));if numel(p)~=3||any(p<=0),return;end
  excess=norm(p.*u);
 case {'box','cuboid'}
  if ~isfield(o,'raw_param'),return;end
  p=double(o.raw_param(:));if numel(p)~=3||any(p<=0),return;end
  excess=sum(abs(u).*p/2); % raw_param is complete edge lengths
 otherwise
  return;
end
h=n'*c+excess;valid=isfinite(h);
end
