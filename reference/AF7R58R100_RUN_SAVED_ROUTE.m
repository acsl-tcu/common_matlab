function R=AF7R58R100_RUN_SAVED_ROUTE(agent,snapshotTime,config)
% R100 generator-time velocity-shape and knot-time co-design, R99 safety gates; reference only.
% R98-style adaptive timing, unchanged R94 full-body safety gate.
% Real R88 source; all known obstacles remain in final R80. No Flight commit.
% Caller provides physical-time knot vector and internal waypoint positions.
% No nominal-clock forcing: endpoint jets are evaluated at t_end-phaseDelay.
% This is a planning experiment, never an active controller/Flight call.
assert(isstruct(config)&&isscalar(config), 'R92_ExplicitRouteOptionsRequired');
need={'times_s','internal_waypoints','phase_delay_s','max_nodes_per_check'};
for k=1:numel(need)
    assert(isfield(config,need{k}),'R92_MissingOption:%s',need{k});
end
obj=find_replanner(agent,5);
assert(~isempty(obj),'R92_ReplannerNotFound');
stack=dbstack;
assert(~any(strcmp({stack.name},'REPLANNING_SYSTEM.do')), ...
    'R92_NoFullSolverIn25msFlightLoop');
assert(isstruct(obj.r88_snapshots),'R92_MissingFlightSnapshots');
ii=AF7R58R88_SELECT_TIME(obj.r88_snapshots,snapshotTime);
R=struct('status','R94_NO_EXACT_SNAPSHOT','snapshot_time_s',snapshotTime, ...
    'reference_version',NaN,'route',struct([]),'assessment',struct([]), ...
    'production_commit_authorized',false,'closed_loop_proved',false);
if isempty(ii),return;end
Z=obj.r88_snapshots(ii);
R.reference_version=Z.reference_version;
times=double(config.times_s);
assert(isrow(times)&&numel(times)>=2&&all(isfinite(times))&& ...
    all(diff(times)>0),'R92_OrderedPhysicalTime');
assert(abs(times(1)-double(Z.time_s))<= ...
    64*eps(max([1,abs(times(1)),abs(double(Z.time_s))])), ...
    'R92_StaleRouteStart');
isAuto=(ischar(config.internal_waypoints)||isstring(config.internal_waypoints));
if isAuto
    assert(strcmp(char(config.internal_waypoints),'AUTO_SUPPORT') && ...
        numel(times)==2 && isfield(config,'max_side_assignments') && ...
        isnumeric(config.max_side_assignments) && ...
        isscalar(config.max_side_assignments) && ...
        isfinite(config.max_side_assignments) && ...
        config.max_side_assignments>=1 && ...
        fix(config.max_side_assignments)==config.max_side_assignments, ...
        'R92_AutoSupportNeedsTotalTimeAndCPUWorkBudget');
else
    assert(isnumeric(config.internal_waypoints)&& ...
        isequal(size(config.internal_waypoints),[3 numel(times)-2])&& ...
        all(isfinite(config.internal_waypoints(:))), ...
        'R92_Interior3DWaypointsRequired');
end
assert(isnumeric(config.phase_delay_s)&&isscalar(config.phase_delay_s)&& ...
    isfinite(config.phase_delay_s)&&config.phase_delay_s>=0, ...
    'R92_ExplicitNonnegativeDelay');
assert(exist('af7r36_single_core_mex','file')==3, ...
    'R92_ProductionNativeMexRequired');
nominal=@(t)AF7R58R88_NOMINAL_JETS( ...
    Z.nominal_derivative_fns,Z.flight_reference_time_origin,t);
J0=nominal(Z.time_s);
if strcmp(char(Z.reference_mode),'NOMINAL')
    active=J0;
elseif strcmp(char(Z.reference_mode),'REPAIRED')
    M=Z.batch_flat_model;
    assert(isstruct(M)&&isfield(M,'certified')&&logical(M.certified), ...
        'R92_SavedActiveModelUncertified');
    assert(~(isfield(M,'phase_layers')&&~isempty(M.phase_layers)), ...
        'R92_CannotAssumeIdentityForActualPhaseReparameterization');
    Q=obj.af7r48_native_eval_packet_from_batch_model(M);
    assert(isfield(Q,'repair_phase_enabled')&& ...
        ~any(double(Q.repair_phase_enabled(:))~=0), ...
        'R92_NonidentityPhaseRequiresSeparateNativeJet7');
    nv=double(af7r36_single_core_mex(2,Q,Z.time_s));nv=nv(:);
    j7=double(af7r36_single_core_mex(25,Q,Z.time_s));j7=j7(:);
    assert(numel(nv)==21&&numel(j7)==3&& ...
        all(isfinite([nv;j7])),'R92_NativeSourceJetsInvalid');
    active=zeros(3,8);
    for d=0:6,active(:,d+1)=nv(3*d+(1:3));end
    active(:,8)=J0(:,8)+j7;
    xf=double(Z.flat_eval_fn(Z.time_s));xf=xf(:);
    assert(numel(xf)>=27&&all(isfinite(xf(1:27))), ...
        'R92_SourceActiveFlatEvaluatorInvalid');
    for d=0:6
        lhs=active(:,d+1);rhs=xf(4*d+(1:3));
        tol=512*max(eps(max(1,abs(lhs))),eps(max(1,abs(rhs))));
        assert(all(abs(lhs-rhs)<=tol),'R92_NativeLiveFlatMismatch');
    end
else
    error('R92_UnsupportedActiveSourceMode');
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
    assert(isstruct(I)&&isscalar(I),'R92_InputAuthorityStructRequired');
    req={'vehicle_mass_kg','thrust_bounds_N','torque_bounds_Nm'};
    for k=1:numel(req)
        assert(isfield(I,req{k}),'R92_ExternalInputAuthority:%s',req{k});
        V.(req{k})=I.(req{k});
    end
    if isfield(I,'tension_max_N'),V.tension_max_N=I.tension_max_N;end
end
corridors={};
if isfield(config,'corridors'),corridors=config.corridors;end
R.attempted_routes=0;
R.rejected_routes=0;
% R95: preserve EVERY attempted support/homotopy seed. A final failed
% seed must not erase an earlier partially certified route.
R.seed_attempt_log=repmat(struct('index',NaN,'status','NOT_RUN', ...
    'reason','','num_segments',0,'polyline_length_m',NaN, ...
    'passed_segments',0,'kinematic_passed_segments',0, ...
    'heading_passed_segments',0,'geometry_passed_segments',0, ...
    'geometry_kinematics_heading_pass',false, ...
    'all_segment_reference_conditions_pass',false, ...
    'input_authorities_available',false,'error','', ...
    'corridor_precheck','','knot_screen','','knot_speed_max_m_s',NaN, ...
        'c7_numeric_endpoint_residual',NaN, ...
        'corridor_projection','','corridor_projection_sweeps',0, ...
        'adaptive_payload_corridor_status','','adaptive_payload_leaves',0, ...
        'overlap_count',0,'minimum_overlap_margin_m',NaN, ...
        'multi_size_knot_status','','multi_size_support_margin_m',NaN, ...
        'knot_time_allocation_status','','necessary_total_time_s',NaN),1,0);
R.partial_reference_found=false;
R.best_partial_route=struct([]);
R.best_partial_assessment=struct([]);
R.best_partial_length_m=Inf;
R.full_reference_found=false;
% Physically NECESSARY and seed-independent: the UAV's two exact endpoint
% positions must be reachable within the allotted trajectory time. Do this
% ONCE, before creating any support-point route seed or polynomial.
vEndLimit=V.finite_sensing_vQ_limit_m_s;
if isfield(config,'operational_uav_speed_limit_m_s')
    vEndLimit=min(vEndLimit,double(config.operational_uav_speed_limit_m_s));
end
f0=B.old_payload_jets(:,3)+[0;0;V.gravity_m_s2];
f1=endJets(:,3)+[0;0;V.gravity_m_s2];
if norm(f0)<=0||norm(f1)<=0||~isfinite(norm(f0))||~isfinite(norm(f1))
    R.status='NECESSARY_ENDPOINT_CABLE_DIRECTION_UNDEFINED__NO_CERT';
    return;
end
pQ0=B.old_payload_jets(:,1)+V.cable_length_m*f0/norm(f0);
pQ1=endJets(:,1)+V.cable_length_m*f1/norm(f1);
Tactual=times(end)-times(1);
R.endpoint_necessary_vQ_m_s=norm(pQ1-pQ0)/Tactual;
if R.endpoint_necessary_vQ_m_s>vEndLimit+ ...
        128*eps(max([1,vEndLimit,R.endpoint_necessary_vQ_m_s]))
    R.status='NECESSARY_ENDPOINT_UAV_TRAVEL_TIME_VIOLATION__NO_SEEDS';
    return;
end
if isAuto
    inflation=max([V.rQ_m,V.rC_m,V.rL_m]); % seed ONLY, not full-body proof
    seeds=AF7R58R97_SUPPORT_SEED_ROUTES( ...
        B.old_payload_jets(:,1),endJets(:,1),B.obstacles, ...
        inflation,config.max_side_assignments);
    paths=seeds.routes;
    R.seed_information=rmfield(seeds,'routes');
    assert(isempty(corridors), ...
        'R95:ExternallyProvidedCorridorsRequireExplicitSectionKnots', ...
        'AUTO_SUPPORT constructs route-specific convex corridors. Provide explicit knot times and waypoints when supplying corridors.');
else
    paths={double(config.internal_waypoints)};
end
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
        'input_authorities_available',false,'error','', ...
        'corridor_precheck','','knot_screen','','knot_speed_max_m_s',NaN, ...
        'c7_numeric_endpoint_residual',NaN, ...
        'corridor_projection','','corridor_projection_sweeps',0, ...
        'adaptive_payload_corridor_status','','adaptive_payload_leaves',0, ...
        'overlap_count',0,'minimum_overlap_margin_m',NaN, ...
        'multi_size_knot_status','','multi_size_support_margin_m',NaN, ...
        'knot_time_allocation_status','','necessary_total_time_s',NaN, ...
        'retime_trials',0,'timing_peak_before_m_s',NaN, ...
        'timing_peak_after_m_s',NaN,'interior_peak_sample_m_s',NaN, ...
        'interior_peak_baseline_sample_m_s',NaN, ...
        'speed_blend_alpha',NaN,'internal_speed_trials',0);
    try
        if isAuto
            ds=vecnorm(diff(Wp,1,2),2,1);
            if any(~isfinite(ds))||any(ds<=0)
                E.status='R92_DEGENERATE_AUTOMATIC_WAYPOINT';
                R.rejected_routes=R.rejected_routes+1;
                R.seed_attempt_log(end+1)=E; %#ok<AGROW>
                continue;
            end
            % R98: necessary UAV reachability FIRST; allocate nonuniform
            % knot times so each interval receives its lower-bound time.
            % No arbitrary maneuver-duration upper bound.
            vForTime=V.finite_sensing_vQ_limit_m_s;
            if isfield(config,'operational_uav_speed_limit_m_s')
                vForTime=min(vForTime,double(config.operational_uav_speed_limit_m_s));
            end
            timing=AF7R58R100_CO_DESIGN_TIMES(Wp,times(1),times(end), ...
                B.old_payload_jets,endJets,config.phase_delay_s, ...
                V.cable_length_m,V.gravity_m_s2,vForTime,config);
            E.knot_time_allocation_status=timing.status;
            E.retime_trials=timing.retime_trials;
            E.timing_peak_before_m_s=timing.estimated_peak_before_m_s;
            E.timing_peak_after_m_s=timing.estimated_peak_after_m_s;
            E.necessary_total_time_s=timing.necessary_total_time_s;
            if ~timing.feasible_necessary_time
                E.status='PREFILTER_ADAPTIVE_KNOT_TIME_UNRESOLVED';
                E.reason=timing.status;
                R.rejected_routes=R.rejected_routes+1;
                R.seed_attempt_log(end+1)=E; %#ok<AGROW>
                continue;
            end
            thisTimes=timing.times_s;
        end
        % CHEAP, RIGOROUS NECESSARY cable/UAV reachability bound:
        % ||pQ_b-pQ_a|| >= max(0,||pL_b-pL_a||-2L).
        % Unlike payload chord/vQ, this does NOT falsely assume q is fixed.
        vPre=V.finite_sensing_vQ_limit_m_s;
        if isfield(config,'operational_uav_speed_limit_m_s')
            vPre=min(vPre,double(config.operational_uav_speed_limit_m_s));
        end
        reach=AF7R58R96_KNOT_REACHABILITY_PREFLIGHT( ...
            Wp,thisTimes,V.cable_length_m,vPre);
        E.knot_screen=reach.status;
        if ~reach.passes_necessary_condition
            E.status='PREFILTER_PHYSICALLY_UNREACHABLE_KNOT_TIMES';
            E.reason=reach.status;
            R.rejected_routes=R.rejected_routes+1;
            R.seed_attempt_log(end+1)=E; %#ok<AGROW>
            continue;
        end
        % Corridor is a convex sufficient *candidate family*, not a claim
        % about all collision-free paths. Do not synthesize full degree-15
        % trajectories whose waypoint chain already lacks a support-plane
        % corridor with these particular waypoints.
        localCorridors=corridors;
        if isAuto
            payloadClearance=V.rL_m;
            if isfield(V,'d_margin_m')
                payloadClearance=payloadClearance+V.d_margin_m;
            end
            pre=AF7R58R97_TIGHT_SUPPORT_CORRIDOR( ...
                Wp,B.obstacles,payloadClearance);
            E.corridor_precheck=pre.status;
            if ~pre.feasible_corridor_family
                E.status='PREFILTER_NO_SUPPORT_CORRIDOR_FAMILY';
                E.reason=pre.status;
                R.rejected_routes=R.rejected_routes+1;
                R.seed_attempt_log(end+1)=E; %#ok<AGROW>
                continue;
            end
            localCorridors=pre.corridors;
        end
        vMax=V.finite_sensing_vQ_limit_m_s;
        if isfield(config,'operational_uav_speed_limit_m_s')
            vOperational=config.operational_uav_speed_limit_m_s;
            assert(isnumeric(vOperational)&&isscalar(vOperational)&& ...
                isfinite(vOperational)&&vOperational>0, ...
                'R95:OperationalSpeedAuthorityRequired');
            vMax=min(vMax,double(vOperational));
        end
        route=AF7R58R100_C7_INTERIOR_SPEED_AWARE(thisTimes,Wp, ...
            B.old_payload_jets,endJets,config.phase_delay_s, ...
            B.live.cable_length_m,B.live.gravity_m_s2,vMax,config);
        R.attempted_routes=R.attempted_routes+1;
        E.interior_peak_sample_m_s=route.interior_peak_sample_m_s;
        E.interior_peak_baseline_sample_m_s=route.interior_peak_baseline_sample_m_s;
        E.speed_blend_alpha=route.knot_jet_blend_alpha;
        E.internal_speed_trials=route.internal_speed_trials;
        if isfield(route,'knot_max_uav_speed_m_s')
            E.knot_speed_max_m_s=route.knot_max_uav_speed_m_s;
        end
        if isfield(route,'max_jet_residual')
            E.c7_numeric_endpoint_residual=route.max_jet_residual;
        end
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
        if ~route.knot_speed_necessary_pass
            E.status='PREFILTER_C7_KNOT_SPEED_UNCERTIFIED';
            E.reason=route.status;
            R.rejected_routes=R.rejected_routes+1;
            R.seed_attempt_log(end+1)=E; %#ok<AGROW>
            continue;
        end
        % Multi-size UAV/payload/cable KNOT corridor (Acetrans-inspired).
        % It is a seed-quality diagnostic, never a substitute for R80;
        % an unsuccessful finite normal search does not rule out a path.
        M99=AF7R58R99_MULTI_SIZE_KNOT_CORRIDOR(route,B.obstacles,V);
        E.multi_size_knot_status=M99.status;
        E.multi_size_support_margin_m=M99.minimum_support_margin_m;
        R.multi_size_knot_corridor=M99;
        % R98: exact Bezier de Casteljau subdivision of the SAME C7
        % reference. Local support corridors may be valid even if a single
        % coarse segment-wide corridor excludes fixed endpoint control points.
        % Each leaf must satisfy ALL known-ellipsoid support halfspaces.
        % This is payload-only numeric reference geometry, NOT a whole-body,
        % IEEE or closed-loop certificate. R80 remains mandatory later.
        localPayload=AF7R58R99_OVERLAP_LOCAL_CORRIDOR_CHAIN( ...
            route,B.obstacles,payload_clearance(V),config);
        E.adaptive_payload_corridor_status=localPayload.status;
        E.adaptive_payload_leaves=localPayload.leaves;
        E.overlap_count=localPayload.overlap_count;
        E.minimum_overlap_margin_m=localPayload.minimum_overlap_margin_m;
        if localPayload.payload_corridor_certified
            E.corridor_projection='ADAPTIVE_EXACT_SPLIT_NO_TRAJECTORY_CHANGE';
            E.corridor_projection_sweeps=0;
            % The old one-plane-per-whole-segment corridor is redundant
            % once every leaf has a verified local support corridor.
            % The R94 whole-body independent geometry remains mandatory.
            verifierCorridors={};
        else
            % Retain existing generation-time linear constraint projection
            % as a separate route to a CERTIFIED convex candidate.
            [route,project]=AF7R58R96_C7_CORRIDOR_PROJECT( ...
                route,localCorridors,config);
            E.corridor_projection=project.status;
            E.corridor_projection_sweeps=project.projection_sweeps;
            if ~project.control_point_contained
                E.status='GENERATOR_CORRIDOR_CONSTRAINTS_UNRESOLVED';
                E.reason=[localPayload.status ' | ' project.status];
                R.rejected_routes=R.rejected_routes+1;
                R.seed_attempt_log(end+1)=E; %#ok<AGROW>
                continue;
            end
            verifierCorridors=localCorridors;
        end
        % Projection can modify internal accelerations/jerks. Check the
        % exact UAV knot velocity again: no stale pre-projection pass.
        kk=AF7R58R96_KNOT_VQ_SCREEN(route,V.gravity_m_s2, ...
            V.cable_length_m,vMax);
        E.knot_speed_max_m_s=kk.max_speed_m_s;
        if ~kk.necessary_speed_pass
            E.status='GENERATOR_POSTPROJECTION_UAV_KNOT_SPEED_UNCERTIFIED';
            E.reason=kk.status;
            R.rejected_routes=R.rejected_routes+1;
            R.seed_attempt_log(end+1)=E; %#ok<AGROW>
            continue;
        end
        screen=AF7R58R96_KNOT_DISPLACEMENT_SCREEN( ...
            route,V.gravity_m_s2,V.cable_length_m,vMax);
        E.knot_screen=screen.status;
        if ~screen.necessary_speed_pass
            E.status='PREFILTER_UAV_KNOT_DISPLACEMENT_UNCERTIFIED';
            E.reason=screen.status;
            R.rejected_routes=R.rejected_routes+1;
            R.seed_attempt_log(end+1)=E; %#ok<AGROW>
            continue;
        end
        R.route=route;
        % Only independent R94 continuous-time per-body & kinematic
        % certification can evaluate the entire generated Bezier curve.
        A=AF7R58R94_CERTIFY_ALL_SEGMENTS( ...
            route,B.obstacles,V,config,verifierCorridors);
        A.r99_overlap_local_payload_corridor_certificate=localPayload;
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
assert(isequaln(Z,obj.r88_snapshots(ii)),'R92_FrozenSourceMutated');
assert(~R.production_commit_authorized,'R92_NoFlightCommit');
if ~isempty(R.assessment)
    assert(~R.assessment.production_commit_authorized,'R92_NoFlightCommit');
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

function c=payload_clearance(V)
c=double(V.rL_m);
if isfield(V,'d_margin_m'),c=c+double(V.d_margin_m);end
assert(isfinite(c)&&c>=0,'R98:LivePayloadRadiusInvalid', ...
    'Live payload radius + externally provided margin must be finite and nonnegative.');
end
