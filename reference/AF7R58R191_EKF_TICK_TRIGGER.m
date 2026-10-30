function R=AF7R58R191_EKF_TICK_TRIGGER(obj,active,t,pQ,pL)
% R191 | Real-time SIMULATION-ONLY current-EKF constant-velocity reach screen.
% Check ALL known original SO(3) ellipsoids, UAV, full cable, load on H=4s.
% Each local time slab contains the entire linearly extrapolated cable in
% the convex hull of its four endpoint vertices. A separating hyperplane
% using exact ellipsoid support certifies only this LINEAR PROXY geometry.
% A nonseparated slab is a conservative REQUEST, not an actual contact.
% It is NOT a nonlinear HLC/plant/observer reachability proof or a SAFE verdict.
R=struct('status','UNKNOWN_R191_INPUT','numeric_request',false, ...
 'first_obstacle_id',NaN,'first_component_code',0,'first_ttc_s',NaN,...
 'numeric_min_margin_m',NaN,'used_current_EKF',true,...
 'reference_version',double(obj.active_reference_version),...
 'r182_model_unseparated',false,'r182_conditional_clear',false,...
 'r182_first_obstacle_id',NaN,'r182_first_component_code',0,...
 'r182_first_unseparated_time_s',NaN,'r182_status','UNKNOWN',...
 'execution_certified',false,'commit_authorized',false,...
 'continuous_time_collision_proved',false,...
 'continuous_time_clearance_proved',false,'sampled_preview_only',false,...
 'source_kind','CURRENT_EKF_LINEAR_KINEMATIC_PROXY__NOT_HLC',...
 'error_message','');
try
  assert(isa(obj,'REPLANNING_SYSTEM') && obj.r191_fast_trigger_enabled,...
   'R191:NotArmed','R191 simulation-only mode not armed.');
  H=double(obj.horizon_s);dt=double(obj.prediction_dt);
  assert(isequal(H,4)&&isfinite(dt)&&dt>0,...
   'R191:InvalidHorizon','H=4s and positive control dt required.');
  S=obj.self.estimator.result.state;
  pQ=double(pQ(:));pL=double(pL(:));
  qV=double(S.v(:));lV=double(S.vL(:));
  assert(isequal(size([pQ,pL,qV,lV]),[3,4]) && ...
   all(isfinite([pQ;pL;qV;lV])) && ...
   isequal(pQ,double(S.p(:))) && isequal(pL,double(S.pL(:))),...
   'R191:EKFSourceMismatch','Use same-tick EKF endpoints and velocities.');
  mask=logical(active(:).');K=obj.obstacle_packet;
  assert(numel(mask)==numel(K.ids) && ...
   isequal(size(K.C),[3 numel(mask)]) && ...
   isequal(size(K.Araw),[3 numel(mask)]) && ...
   isequal(size(K.R),[9 numel(mask)]),...
   'R191:ObstaclePacketMismatch','Known obstacle packet source mismatch.');
  if ~any(mask)
    R.status='NO_KNOWN_OBSTACLE__NOT_GLOBAL_SAFETY';return;
  end
  % Representation split only: no arbitrary physical speed/accel assumptions.
  slabs=ceil(sqrt(H/dt));
  assert(slabs>=1 && slabs<100000, 'R191:InvalidGrid','Grid invalid.');
  times=H*(0:slabs)/slabs;
  r=max(double([obj.r_drone,obj.r_cable,obj.r_load]))+double(obj.d_margin);
  assert(isfinite(r)&&r>=0,'R191:InvalidBodyRadius','Body radius invalid.');
  idx=find(mask);bestGap=Inf;
  for k=1:slabs
    a=times(k);b=times(k+1);
    q0=pQ+a*qV;q1=pQ+b*qV;
    l0=pL+a*lV;l1=pL+b*lV;
    % Convex hull includes all points on the straight full cable for each
    % interpolated time in [a,b]; never just cable endpoints or an L-ball.
    V=[q0 q1 l0 l1];vmean=mean(V,2);
    for jj=idx
      c=double(K.C(:,jj));rot=reshape(double(K.R(:,jj)),3,3);
      axes=double(K.Araw(:,jj));
      assert(all(isfinite([c;rot(:);axes])) && all(axes>0) && ...
       norm(rot'*rot-eye(3),'fro')<1e-6 && det(rot)>0.9,...
       'R191:InvalidSO3','SO(3) ellipsoid invalid.');
      [separated,gap]=AF7R58R191_HULL_SEPARATE(V,c,rot,axes,r);
      bestGap=min(bestGap,gap);
      if ~separated
        R.r182_model_unseparated=true;
        R.r182_first_obstacle_id=double(K.ids(jj));
        % Endpoint/cable categorization is descriptive; not a contact proof.
        dq=min(norm(q0-c),norm(q1-c));
        dl=min(norm(l0-c),norm(l1-c));
        ds=norm(vmean-c);
        [~,which]=min([dl,dq,ds]);
        R.r182_first_component_code=which;
        R.r182_first_unseparated_time_s=double(t)+a;
        R.first_obstacle_id=R.r182_first_obstacle_id;
        R.first_component_code=which;
        R.first_ttc_s=a;
        R.numeric_min_margin_m=bestGap;
        R.status='R191_LINEAR_EKF_HULL_POSSIBLE_RISK__HLC_UNVERIFIED';
        R.r182_status=R.status;
        return;
      end
    end
  end
  R.r182_conditional_clear=true;
  R.numeric_min_margin_m=bestGap;
  R.status='R191_LINEAR_EKF_HULL_SEPARATED__HLC_UNVERIFIED';
  R.r182_status=R.status;
catch ME
  R.status=['UNKNOWN_R191_' char(ME.identifier)];
  R.r182_status=R.status;
  R.error_message=char(ME.message);
end
end
