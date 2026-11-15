function R=TRAJECTORY_C9_REPLAN_ACTIVE_AVOIDANCE(active_piece,request,ekf_state,entry_tolerances,trigger,previous_event)
%TRAJECTORY_C9_REPLAN_ACTIVE_AVOIDANCE Nominal-only mid-avoidance replan API.
% Reconstructs an EXACT degree-27 C9 suffix at the current Flight tick,
% not an old-clock return or a multi-stage generated trajectory. Reads only
% the request's sensor-known obstacle set and the CURRENT EKF snapshot.
% Replanning is permitted even during an ongoing avoidance. A new plan
% revision re-arms the same obstacle IDs after a previous replan; an
% unchanged plan/risk pair is duplicate-suppressed. Trigger prediction
% does NOT constitute physical clearance or a Flight execution certificate.
% No HLC/EKF/plant/runner is modified. Always safe_to_commit=false.
R=struct('status','UNKNOWN_REPLAN_NOT_STARTED','safe_to_commit',false, ...
  'nominal_model_complete',false,'HLC_B_proven',false, ...
  'event_consumption_authorized',false, ...
  'C7_actual_EKF_join_proven',false,'nominal_C9_suffix_exact',false, ...
  'event_signature',struct(),'entry_status','NOT_RUN', ...
  'planner_result',struct(),'candidate_piece',struct(), ...
  'candidate_available',false,'elapsed_ms',NaN);
clock=tic;
try
    if ~(isstruct(active_piece)&&isscalar(active_piece)&& ...
         all(isfield(active_piece,{'t0_s','duration_s','control_points_m'}))&& ...
         isnumeric(active_piece.control_points_m)&& ...
         isequal(size(active_piece.control_points_m),[3 28])&& ...
         all(isfinite(active_piece.control_points_m(:))) && ...
         isstruct(request)&&isscalar(request)&& ...
         all(isfield(request,{'t0_s','observed','authority','search_options', ...
                              'flight_interval_s'}))&& ...
         isstruct(trigger)&&isscalar(trigger)&& ...
         all(isfield(trigger,{'time_s','collision_predicted', ...
             'collision_obstacle_ids','collision_ttc_s','collision_decision'}))&& ...
         isstruct(ekf_state)&&isscalar(ekf_state)&& ...
         isstruct(entry_tolerances)&&isscalar(entry_tolerances))
        R.status='UNKNOWN_REPLAN_REQUIRED_SNAPSHOT_MISSING';R.elapsed_ms=1000*toc(clock);return
    end
    if ~isfield(request,'active_plan_revision') || ...
         ~isscalar(request.active_plan_revision)|| ...
         ~isnumeric(request.active_plan_revision)|| ...
         ~isfinite(double(request.active_plan_revision))|| ...
         double(request.active_plan_revision)<0|| ...
         mod(double(request.active_plan_revision),1)~=0
        R.status='UNKNOWN_REPLAN_ACTIVE_REVISION_MISSING';R.elapsed_ms=1000*toc(clock);return
    end
    t0=double(request.t0_s);
    p=double(active_piece.control_points_m);
    tStart=double(active_piece.t0_s);
    tEnd=tStart+double(active_piece.duration_s);
    if ~(isfinite(t0)&&isfinite(tStart)&&isfinite(tEnd)&& ...
         t0>=tStart&&t0<tEnd&& ...
         t0>=request.flight_interval_s(1)&& ...
         tEnd<=request.flight_interval_s(2))
        R.status='UNKNOWN_REPLAN_OUTSIDE_ACTIVE_FLIGHT_PIECE';R.elapsed_ms=1000*toc(clock);return
    end
    if ~(isfinite(trigger.time_s)&&isequal(double(trigger.time_s),t0))
        R.status='UNKNOWN_REPLAN_TRIGGER_EPOCH_MISMATCH';R.elapsed_ms=1000*toc(clock);return
    end
    if ~isequal(trigger.collision_predicted,true) || ...
         ~strcmp(char(trigger.collision_decision),'POSSIBLE_INTERSAMPLE_CONTACT')
        R.status='NO_NEW_PREDICTED_NOMINAL_COLLISION';R.elapsed_ms=1000*toc(clock);return
    end
    horizon=4; % User-designated trigger horizon, not maneuver duration.
    if isfield(request,'trigger_horizon_s'),horizon=request.trigger_horizon_s;end
    if ~(isscalar(horizon)&&isnumeric(horizon)&&isfinite(horizon)&&horizon>0 && ...
         isfinite(trigger.collision_ttc_s)&& ...
         trigger.collision_ttc_s>=0 && ...
         trigger.collision_ttc_s<=horizon)
        R.status='UNKNOWN_REPLAN_TRIGGER_NOT_WITHIN_HORIZON';R.elapsed_ms=1000*toc(clock);return
    end
    if ~(isstruct(request.observed)&& ...
         all(isfield(request.observed,{'id'})))
        R.status='UNKNOWN_REPLAN_OBSERVED_MEMORY_INVALID';R.elapsed_ms=1000*toc(clock);return
    end
    if isempty(request.observed)
        R.status='UNKNOWN_REPLAN_NO_OBSERVED_SHAPES';R.elapsed_ms=1000*toc(clock);return
    end
    observed_ids=double([request.observed.id]);
    risk_ids=double(trigger.collision_obstacle_ids(:).');
    if isempty(risk_ids)||any(~isfinite([observed_ids risk_ids]))|| ...
         numel(unique(observed_ids))~=numel(observed_ids)|| ...
         ~all(ismember(risk_ids,observed_ids))
        R.status='UNKNOWN_REPLAN_RISK_NOT_IN_OBSERVED_MEMORY';R.elapsed_ms=1000*toc(clock);return
    end
    risk_ids=unique(risk_ids,'sorted');
    revision=double(request.active_plan_revision);
    signature=struct('plan_revision',revision,'risk_ids',risk_ids, ...
                     'event_time_s',t0);
    % Diagnostic only: this signature MUST NOT be consumed as an active
    % Flight-plan acknowledgment until execution certification and actual
    % reference switching exist. A failed replan must remain retryable.
    R.event_signature=signature;
    if nargin>=6 && isstruct(previous_event)&&isscalar(previous_event) && ...
         all(isfield(previous_event,{'plan_revision','risk_ids'})) && ...
         isequal(double(previous_event.plan_revision),revision) && ...
         isequal(unique(double(previous_event.risk_ids(:).'),'sorted'),risk_ids)
        R.status='SKIPPED_REPLAN_DUPLICATE_CURRENT_PLAN_AND_RISKS';R.elapsed_ms=1000*toc(clock);return
    end
    % Exact de Casteljau suffix. All endpoint 0..9 jets are derived from
    % the EXISTING active piece, never invented from EKF pL/vL alone.
    u=(t0-tStart)/double(active_piece.duration_s);
    work=p;right=zeros(3,28);right(:,28)=p(:,28);
    for level=1:27
        work=(1-u)*work(:,1:end-1)+u*work(:,2:end);
        right(:,28-level)=work(:,end);
    end
    remaining=tEnd-t0;
    jets0=zeros(3,10);jets1=zeros(3,10);
    diffctrl=right;scale=1;
    for k=0:9
        if k>0
            diffctrl=diff(diffctrl,1,2);
            scale=scale*(28-k)/remaining;
        end
        jets0(:,k+1)=scale*diffctrl(:,1);
        jets1(:,k+1)=scale*diffctrl(:,end);
    end
    local=request;
    local.start_jets=jets0;
    local.end_jets=jets1;
    local.nominal_free27=right(:,11:18);
    local.duration_s=remaining;
    local.search_options.enable_degree27_25d_joint_search=1;
    local.search_options.enable_13d_variable_time_joint_search=0;
    % Check exact inherited segment representation before invoking search.
    [rebuilt,~]=TRAJECTORY_C9_27_SINGLE_PIECE_BASIS( ...
        jets0,jets1,right(:,11:18),remaining);
    rel=norm(rebuilt-right,'fro')/max(1,norm(right,'fro'));
    R.suffix_reconstruction_relative_error=rel;
    if rel>512*eps(max(1,norm(right,'fro')))
        R.status='UNKNOWN_REPLAN_C9_SUFFIX_RECONSTRUCTION';R.elapsed_ms=1000*toc(clock);return
    end
    R.nominal_C9_suffix_exact=true;
    local.current_ekf_state=ekf_state;
    local.entry_tolerances=entry_tolerances;
    E=TRAJECTORY_C9_ENTRY_STATE_GATE(local,ekf_state,entry_tolerances);
    R.entry_status=E.status;
    if ~E.necessary_conditions_met
        R.status='UNKNOWN_REPLAN_CURRENT_EKF_ENTRY_NOT_MATCHED';R.elapsed_ms=1000*toc(clock);return
    end
    V=TRAJECTORY_C9_INTEGRATED_NOMINAL_PIPELINE(local);
    R.planner_result=V;
    R.nominal_model_complete=V.nominal_model_complete;
    if V.nominal_model_complete
        R.candidate_available=true;
        R.candidate_piece=V.candidate_piece;
        R.status='CONDITIONAL_REPLAN_NOMINAL_FULLTIME__EXECUTION_TUBE_PENDING';
    else
        R.status=['UNKNOWN_REPLAN_' char(V.status)];
    end
catch ME
    R.status=['UNKNOWN_REPLAN_EXCEPTION_' ME.identifier];
    R.error_message=ME.message;
end
R.elapsed_ms=1000*toc(clock);
end
