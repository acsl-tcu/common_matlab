function R=AF7R58R181_LIVE_TRIGGER(obj,active,t,pQ,pL)
% Called INSIDE actual replanner.do while EKF, HLC and sensor memory match
% the SAME Flight tick. Executes unchanged R71 HLC+plant physical rollout.
% This is a simulation-only sampled warning: NOT 4-s reachability proof.
R=struct('status','UNKNOWN_NO_REAL_HLC_PREVIEW', ...
 'numeric_request',false,'first_obstacle_id',NaN,...
 'first_component_code',0,'first_ttc_s',NaN,...
 'numeric_min_margin_m',NaN,'execution_certified',false,...
 'commit_authorized',false,'error_message','');
assert(isa(obj,'REPLANNING_SYSTEM') && ...
 obj.r181_sim_preview_enabled && ~obj.r172_strict_replacement, ...
 'R181:SimulationModeNotEnabled');
assert(isequal(double(obj.horizon_s),4),'R181:FourSecondHorizon');
s=obj.self.estimator.result.state;
ml=double(s.mL);
assert(isscalar(ml)&&isfinite(ml)&&ml>0 && ...
 isequal(ml,double(obj.self.parameter.get("loadmass"))), ...
 'R181:UnsynchronizedControllerMass');
assert(isequal(double(s.p(:)),double(pQ(:))) && ...
 isequal(double(s.pL(:)),double(pL(:))), ...
 'R181:NotSameEKFEndpoints');
K=obj.obstacle_packet;mask=logical(active(:));N=numel(mask);
assert(isstruct(K)&&all(isfield(K,{'C','R','Araw','ids'})) && ...
 isequal(size(K.C),[3 N])&&isequal(size(K.R),[9 N]) && ...
 isequal(size(K.Araw),[3 N]) && numel(K.ids)==N, ...
 'R181:KnownObstaclePacketShape');
obs=struct([]);
for j=find(mask(:)).'
 o=struct('id',double(K.ids(j)), ...
 'p_center',double(K.C(:,j)), ...
 'R_obs',reshape(double(K.R(:,j)),3,3), ...
 'ellipsoid_radii',double(K.Araw(:,j)));
 if isempty(obs),obs=o;else,obs(end+1)=o;end %#ok<AGROW>
end
% Do not add the ORIGINAL MEX intersample sweep inflation to sampled
% HLC geometry. It is not a proof of preview error and double counting
% could create an artificial 4-s global expansion.
rad=double([obj.r_drone;obj.r_cable;obj.r_load])+double(obj.d_margin);
assert(all(isfinite(rad))&&all(rad>=0),'R181:InvalidLiveRadii');
origin=struct('t_s',double(t),'estimated_mL_kg',ml, ...
 'pQ',double(pQ(:)),'pL',double(pL(:)), ...
 'horizon_s',double(obj.horizon_s));
try
 H=obj.r71_predict_live_hlc_future(double(t));
catch ME
 R.status=['UNKNOWN_HLC_PREVIEW_' char(ME.identifier)];
 R.error_message=char(ME.message);return;
end
if ~isstruct(H)||~isscalar(H)||...
 ~all(isfield(H,{'t','pQ','pL','state','estimated_mass_kg'}))
 R.status='UNKNOWN_R71_PREVIEW_SCHEMA';return;
end
x=double(H.state);tt=double(H.t(:).');
if numel(tt)<2||~isequal(size(x),[24 numel(tt)])||...
 ~isequal(size(H.pQ),[3 numel(tt)])||...
 ~isequal(size(H.pL),[3 numel(tt)])||...
 ~isequal(double(H.estimated_mass_kg),ml)||...
 any(~isfinite([tt(:);x(:);H.pQ(:);H.pL(:)]))
 R.status='UNKNOWN_R71_PREVIEW_DATA';return;
end
% R71 must start from the actual current EKF 24-state, not a nominal
% state, previous Flight snapshot, or artificially tautened endpoints.
x24=double([s.p(:);s.q(:);s.v(:);s.w(:); ...
 s.pL(:);s.vL(:);s.pT(:);s.wL(:)]);
% Model-specific 24-state ordering is as in R178. If one of the EKF
% symbolic aliases differs, trust only directly compared endpoints and
% require the R71 source itself to provide the EKF initial-state proof.
if ~isequal(size(x24),[24 1]) || any(~isfinite(x24))
 R.status='UNKNOWN_EKF_24_SCHEMA';return;
end
if any(abs(x(:,1)-x24)>1024*eps(max([1;abs(x24)])))
 R.status='UNKNOWN_R71_EKF_INITIAL_STATE_MISMATCH';return;
end
P=struct('t_s',tt,'Q_m',double(H.pQ),'L_m',double(H.pL),...
 'source_time_s',double(t),'estimated_mass_kg',ml, ...
 'source_EKF_Q',double(pQ(:)),'source_EKF_L',double(pL(:)));
R=AF7R58R181_CORE(P,obs,rad,origin);
% R182: independent time-LOCAL interval enclosure around the actual R71
% numerical HLC/plant rollout.  This does not broaden the entire 4 s into
% a worst-thrust ball and never asserts closed-loop execution proof.
R.r182_status='UNKNOWN_INTERVAL_NOT_RUN';
R.r182_model_unseparated=false;
R.r182_first_interval=NaN;
R.r182_first_obstacle_id=NaN;
R.r182_first_component_code=0;
R.r182_conditional_clear=false;
R.r182_max_local_error_Q_m=NaN;
R.r182_max_local_error_L_m=NaN;
R.r182_real_execution_proven=false;
try
   CL=obj.command_limit_certificate;
   assert(isstruct(CL)&&isfield(CL,'certified')&&logical(CL.certified)&&...
       isfield(CL,'limits')&&isequal(size(CL.limits),[4 2]),...
       'R182:LiveHLCCommandBoundsUnverified');
   limits=double(CL.limits);
   assert(all(isfinite(limits(:)))&&limits(1,1)>=0&&limits(1,2)>0,...
       'R182:InvalidLiveHLCThrustBound');
   mQ=double(obj.self.parameter.get("mass"));
   g=double(obj.self.parameter.get("gravity"));
   Lc=double(obj.self.parameter.get("cableL"));
   assert(all(isfinite([mQ,g,Lc]))&&all([mQ,g,Lc]>0),...
       'R182:LivePhysicalParametersMissing');
   localPhysics=struct('mQ_kg',mQ,'mL_kg',ml,'L_m',Lc,...
       'g_mps2',g,'fmax_N',limits(1,2),'horizon_s',double(obj.horizon_s));
   intervalP=struct('t',tt,'pQ',double(H.pQ),...
       'pL',double(H.pL),'x24',x);
   J=AF7R58R182_INTERVAL_TRIGGER(intervalP,obs,rad,localPhysics);
   R.r182_status=J.status;
   R.r182_model_unseparated=logical(J.possible_risk_request);
   R.r182_first_interval=J.first_unseparated_interval;
   R.r182_first_obstacle_id=J.first_obstacle_id;
   R.r182_first_component_code=J.first_component_code;
   R.r182_conditional_clear=logical(J.model_conditional_clear);
   R.r182_max_local_error_Q_m=J.max_local_interpolation_Q_m;
   R.r182_max_local_error_L_m=J.max_local_interpolation_L_m;
catch ME
   R.r182_status=['UNKNOWN_INTERVAL_EXCEPTION_' char(ME.identifier)];
   R.r182_error=char(ME.message);
end
R.reference_version=double(obj.active_reference_version);
R.used_current_EKF=true;
R.used_actual_HLC_plant_numeric_preview=true;
R.continuous_time_collision_proved=false;
R.continuous_time_clearance_proved=false;
R.execution_certified=false;
R.commit_authorized=false;
end
