function R=AF7R58R94_RUN_SAVED_ROUTE(agent,snapshotTime,config)
% Real R88 event source -> NEW multi-segment C7 route, reference-only.
% Caller provides physical-time knot vector and internal waypoint positions.
% No nominal-clock forcing: endpoint jets are evaluated at t_end-phaseDelay.
% This is a planning experiment, never an active controller/Flight call.
assert(isstruct(config)&&isscalar(config), 'R92:ExplicitRouteOptionsRequired');
need={'times_s','internal_waypoints','phase_delay_s','max_nodes_per_check'};
for k=1:numel(need)
    assert(isfield(config,need{k}),'R92:MissingOption:%s',need{k});
end
obj=find_replanner(agent,5);
assert(~isempty(obj),'R92:ReplannerNotFound');
stack=dbstack;
assert(~any(strcmp({stack.name},'REPLANNING_SYSTEM.do')), ...
    'R92:NoFullSolverIn25msFlightLoop');
assert(isstruct(obj.r88_snapshots),'R92:MissingFlightSnapshots');
ii=AF7R58R88_SELECT_TIME(obj.r88_snapshots,snapshotTime);
R=struct('status','R94_NO_EXACT_SNAPSHOT','snapshot_time_s',snapshotTime, ...
    'reference_version',NaN,'route',struct([]),'assessment',struct([]), ...
    'production_commit_authorized',false,'closed_loop_proved',false);
if isempty(ii),return;end
Z=obj.r88_snapshots(ii);
R.reference_version=Z.reference_version;
times=double(config.times_s);
assert(isrow(times)&&numel(times)>=2&&all(isfinite(times))&& ...
    all(diff(times)>0),'R92:OrderedPhysicalTime');
assert(abs(times(1)-double(Z.time_s))<= ...
    64*eps(max([1,abs(times(1)),abs(double(Z.time_s))])), ...
    'R92:StaleRouteStart');
isAuto=(ischar(config.internal_waypoints)||isstring(config.internal_waypoints));
if isAuto
    assert(strcmp(char(config.internal_waypoints),'AUTO_SUPPORT') && ...
        numel(times)==2 && isfield(config,'max_side_assignments') && ...
        isnumeric(config.max_side_assignments) && ...
        isscalar(config.max_side_assignments) && ...
        isfinite(config.max_side_assignments) && ...
        config.max_side_assignments>=1 && ...
        fix(config.max_side_assignments)==config.max_side_assignments, ...
        'R92:AutoSupportNeedsTotalTimeAndCPUWorkBudget');
else
    assert(isnumeric(config.internal_waypoints)&& ...
        isequal(size(config.internal_waypoints),[3 numel(times)-2])&& ...
        all(isfinite(config.internal_waypoints(:))), ...
        'R92:Interior3DWaypointsRequired');
end
assert(isnumeric(config.phase_delay_s)&&isscalar(config.phase_delay_s)&& ...
    isfinite(config.phase_delay_s)&&config.phase_delay_s>=0, ...
    'R92:ExplicitNonnegativeDelay');
assert(exist('af7r36_single_core_mex','file')==3, ...
    'R92:ProductionNativeMexRequired');
nominal=@(t)AF7R58R88_NOMINAL_JETS( ...
    Z.nominal_derivative_fns,Z.flight_reference_time_origin,t);
J0=nominal(Z.time_s);
if strcmp(char(Z.reference_mode),'NOMINAL')
    active=J0;
elseif strcmp(char(Z.reference_mode),'REPAIRED')
    M=Z.batch_flat_model;
    assert(isstruct(M)&&isfield(M,'certified')&&logical(M.certified), ...
        'R92:SavedActiveModelUncertified');
    assert(~(isfield(M,'phase_layers')&&~isempty(M.phase_layers)), ...
        'R92:CannotAssumeIdentityForActualPhaseReparameterization');
    Q=obj.af7r48_native_eval_packet_from_batch_model(M);
    assert(isfield(Q,'repair_phase_enabled')&& ...
        ~any(double(Q.repair_phase_enabled(:))~=0), ...
        'R92:NonidentityPhaseRequiresSeparateNativeJet7');
    nv=double(af7r36_single_core_mex(2,Q,Z.time_s));nv=nv(:);
    j7=double(af7r36_single_core_mex(25,Q,Z.time_s));j7=j7(:);
    assert(numel(nv)==21&&numel(j7)==3&& ...
        all(isfinite([nv;j7])),'R92:NativeSourceJetsInvalid');
    active=zeros(3,8);
    for d=0:6,active(:,d+1)=nv(3*d+(1:3));end
    active(:,8)=J0(:,8)+j7;
    xf=double(Z.flat_eval_fn(Z.time_s));xf=xf(:);
    assert(numel(xf)>=27&&all(isfinite(xf(1:27))), ...
        'R92:SourceActiveFlatEvaluatorInvalid');
    for d=0:6
        lhs=active(:,d+1);rhs=xf(4*d+(1:3));
        tol=512*max(eps(max(1,abs(lhs))),eps(max(1,abs(rhs))));
        assert(all(abs(lhs-rhs)<=tol),'R92:NativeLiveFlatMismatch');
    end
else
    error('R92:UnsupportedActiveSourceMode');
end
S=struct('active_raw_jets',active,'nominal_raw_jets_at',nominal, ...
    'known_mask',Z.known_mask,'obstacle_packet',Z.obstacle_packet, ...
    'reference_version',Z.reference_version, ...
    'reference_mode',Z.reference_mode, ...
    'gravity_m_s2',Z.gravity_m_s2,'cable_length_m',Z.cable_length_m, ...
    'rQ_m',Z.rQ_m,'rC_m',Z.rC_m,'rL_m',Z.rL_m, ...
    'd_margin_m',Z.d_margin_m, ...
    'sensor_range_m',Z.sensor_range_m,'horizon_s',Z.horizon_s, ...
    'prediction_dt_s',Z.prediction_dt_s, ...
    'latest_EKF_mL_kg',Z.estimated_mass_kg);
B=AF7R58R83_BIND_R81_INPUTS(S);
endPhase=times(end)-double(config.phase_delay_s);
endJets=B.nominal_payload_jets_at(endPhase);
% Runtime physical parameters not present in the frozen R88 snapshot
% remain unavailable until explicitly passed by the execution authority.
V=B.live;
V.source_reference_version=double(Z.reference_version);
if isfield(config,'input_authority')
    I=config.input_authority;
    assert(isstruct(I)&&isscalar(I),'R92:InputAuthorityStructRequired');
    req={'vehicle_mass_kg','thrust_bounds_N','torque_bounds_Nm'};
    for k=1:numel(req)
        assert(isfield(I,req{k}),'R92:ExternalInputAuthority:%s',req{k});
        V.(req{k})=I.(req{k});
    end
    if isfield(I,'tension_max_N'),V.tension_max_N=I.tension_max_N;end
end
corridors={};
if isfield(config,'corridors'),corridors=config.corridors;end
if isAuto
    inflation=max([V.rQ_m,V.rC_m,V.rL_m]); % seed ONLY, not full-body proof
    seeds=AF7R58R94_SUPPORT_SEED_ROUTES( ...
        B.old_payload_jets(:,1),endJets(:,1),B.obstacles, ...
        inflation,config.max_side_assignments);
    paths=seeds.routes;
    R.seed_information=rmfield(seeds,'routes');
else
    paths={double(config.internal_waypoints)};
end
R.attempted_routes=0;
R.rejected_routes=0;
% R93: preserve EVERY attempted support/homotopy seed. A final failed
% seed must not erase an earlier partially certified route.
R.seed_attempt_log=repmat(struct('index',NaN,'status','NOT_RUN', ...
    'reason','','num_segments',0,'polyline_length_m',NaN, ...
    'passed_segments',0,'kinematic_passed_segments',0, ...
    'heading_passed_segments',0,'geometry_passed_segments',0, ...
    'geometry_kinematics_heading_pass',false, ...
    'all_segment_reference_conditions_pass',false, ...
    'input_authorities_available',false,'error',''),1,0);
R.partial_reference_found=false;
R.best_partial_route=struct([]);
R.best_partial_assessment=struct([]);
R.best_partial_length_m=Inf;
R.full_reference_found=false;
for candidateIdx=1:numel(paths)
    Wp=[B.old_payload_jets(:,1),paths{candidateIdx},endJets(:,1)];
    thisTimes=times;
    E=struct('index',candidateIdx,'status','NOT_RUN', ...
        'reason','','num_segments',0,'polyline_length_m', ...
        sum(vecnorm(diff(Wp,1,2),2,1)), ...
        'passed_segments',0,'kinematic_passed_segments',0, ...
        'heading_passed_segments',0,'geometry_passed_segments',0, ...
        'geometry_kinematics_heading_pass',false, ...
        'all_segment_reference_conditions_pass',false, ...
        'input_authorities_available',false,'error','');
    try
        if isAuto
            ds=vecnorm(diff(Wp,1,2),2,1);
            if any(~isfinite(ds))||any(ds<=0)
                E.status='R92_DEGENERATE_AUTOMATIC_WAYPOINT';
                R.rejected_routes=R.rejected_routes+1;
                R.seed_attempt_log(end+1)=E; %#ok<AGROW>
                continue;
            end
            f=cumsum(ds)/sum(ds);
            thisTimes=[times(1),times(1)+(times(end)-times(1))*f];
            thisTimes(end)=times(end);
            if any(diff(thisTimes)<=0)
                E.status='R92_AUTOMATIC_TIME_RESOLUTION';
                R.rejected_routes=R.rejected_routes+1;
                R.seed_attempt_log(end+1)=E; %#ok<AGROW>
                continue;
            end
        end
        route=AF7R58R92_C7_MINIMUM8_ROUTE(thisTimes,Wp, ...
            B.old_payload_jets,endJets,config.phase_delay_s,B.live.cable_length_m);
        R.attempted_routes=R.attempted_routes+1;
        E.status=route.status;
        if isfield(route,'num_segments'), E.num_segments=route.num_segments;end
        R.route=route;
        R.known_ids=B.known_ids;
        if ~route.reference_candidate_found
            R.rejected_routes=R.rejected_routes+1;
            E.reason=route.status;
            R.seed_attempt_log(end+1)=E; %#ok<AGROW>
            continue;
        end
        if isAuto&&~isempty(corridors)
            error('R92:AutoCorridorFamilyMustBeGeneratedAlongsideWaypoints');
        end
        A=AF7R58R94_CERTIFY_ALL_SEGMENTS( ...
            route,B.obstacles,V,config,corridors);
        % An incompatible/missing certifier schema is a program fault,
        % never a clearance, speed or physical-feasibility failure.
        certFields={'status','segments','input_authorities_available', ...
            'geometry_kinematics_heading_pass', ...
            'all_segment_reference_conditions_pass', ...
            'production_commit_authorized'};
        if ~isstruct(A)||~isscalar(A)||~all(isfield(A,certFields))
            error('R94:CertificateSchemaMismatch', ...
              'Pinned R94 certifier returned invalid fields from %s.', ...
              which('AF7R58R94_CERTIFY_ALL_SEGMENTS'));
        end
        if A.production_commit_authorized
            error('R94:UnexpectedProductionAuthorization', ...
                'Reference-only certifier cannot authorize a Flight commit.');
        end
        R.assessment=A;
        E.status=A.status;
        E.input_authorities_available=A.input_authorities_available;
        E.geometry_kinematics_heading_pass=A.geometry_kinematics_heading_pass;
        E.all_segment_reference_conditions_pass=A.all_segment_reference_conditions_pass;
        for si=1:numel(A.segments)
            U=A.segments(si);
            if U.reference_pass, E.passed_segments=E.passed_segments+1;end
            if isstruct(U.kinematics)&&isfield(U.kinematics,'certified')&&U.kinematics.certified
                E.kinematic_passed_segments=E.kinematic_passed_segments+1;
            end
            if isstruct(U.heading)&&isfield(U.heading,'certified')&&U.heading.certified
                E.heading_passed_segments=E.heading_passed_segments+1;
            end
            if isstruct(U.geometry)&&isfield(U.geometry,'certified')&&U.geometry.certified
                E.geometry_passed_segments=E.geometry_passed_segments+1;
            end
            if isempty(E.reason)&&~U.reference_pass
                E.reason=sprintf('seg%d:%s',si,U.reason);
            end
        end
        if A.geometry_kinematics_heading_pass
            R.partial_reference_found=true;
            if E.polyline_length_m<R.best_partial_length_m
                R.best_partial_route=route;
                R.best_partial_assessment=A;
                R.best_partial_length_m=E.polyline_length_m;
            end
        end
        R.seed_attempt_log(end+1)=E; %#ok<AGROW>
        if A.all_segment_reference_conditions_pass
            R.full_reference_found=true;
            R.status='R92_MULTISEGMENT_PARTIAL_REFERENCE_CHECKS_PASS__NOT_COMMITTABLE';
            break;
        end
        R.rejected_routes=R.rejected_routes+1;
    catch ME
        % Do not hide a failed or malformed seed as a certified route.
        E.status='FAIL_CLOSED_SEED_EXCEPTION';
        E.error=[char(ME.identifier) ':' char(ME.message)];
        if ~isempty(ME.stack)
            E.error=[E.error sprintf(' @%s:%d', ...
                ME.stack(1).name,ME.stack(1).line)];
        end
        R.seed_attempt_log(end+1)=E; %#ok<AGROW>
        R.rejected_routes=R.rejected_routes+1;
    end
end
if ~R.full_reference_found
    if R.partial_reference_found
        R.status='R93_PARTIAL_GEO_KIN_HEADING_ONLY__NOT_COMMITTABLE';
        R.route=R.best_partial_route;
        R.assessment=R.best_partial_assessment;
    elseif isempty(R.seed_attempt_log)
        R.status='R93_NO_SEEDS_EVALUATED';
    else
        R.status='R93_ALL_SEEDS_UNCERTIFIED';
    end
end
% The saved packet is read-only and may not have been the trigger's first
% actual-state event. Never silently use it as a real-time collision trigger.
assert(isequaln(Z,obj.r88_snapshots(ii)),'R92:FrozenSourceMutated');
assert(~R.production_commit_authorized,'R92:NoFlightCommit');
if ~isempty(R.assessment)
    assert(~R.assessment.production_commit_authorized,'R92:NoFlightCommit');
end
end
function obj=find_replanner(x,depth)
obj=[];
if isa(x,'REPLANNING_SYSTEM')&&isscalar(x),obj=x;return;end
if depth<1||isempty(x)||numel(x)~=1,return;end
if isobject(x)
    try,names=properties(x);catch,return;end
elseif isstruct(x)
    names=fieldnames(x);
elseif iscell(x)
    for j=1:numel(x)
        obj=find_replanner(x{j},depth-1);
        if ~isempty(obj),return;end
    end
    return;
else,return;end
skip={'self','result','parameter','plant','sensor','estimator', ...
    'controller','logger','time','state','parent'};
for j=1:numel(names)
    f=names{j};if any(strcmpi(f,skip)),continue;end
    try
        obj=find_replanner(x.(f),depth-1);
    catch
        obj=[];
    end
    if ~isempty(obj),return;end
end
end
