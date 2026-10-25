function G = AF7R58R105_CLOSE_PARTIAL(agent,A,opts)
% Reuse the ACTUAL R104 best partial without resampling or regenerating.
% Re-certifies ALL same-time sensor-known ellipsoids using unchanged R94.
% Numerical nominal thrust/taut cable bound is not HLC/IEEE/torque proof.
% A real external cable strength rating is required; never infer it.
G=struct('status','R104_PARTIAL_NOT_AVAILABLE', ...
    'reused_existing_C7_control_points',false, ...
    'source_version_matches',false,'source_obstacles_bound',false, ...
    'nominal_input_authority_status','NOT_CHECKED', ...
    'rating_source_supplied',false,'estimated_mL_kg',NaN, ...
    'segments',struct([]),'nominal_tension_upper_N',NaN, ...
    'nominal_thrust_upper_N',NaN,'minimum_verified_rating_needed_N',NaN, ...
    'numeric_c7_tail_seam_error',NaN,'all_c7_seams',struct([]), ...
    'nominal_phase_end_s',NaN, ...
    'finite_nominal_tail_until_s',NaN,'reference_certifier',struct([]), ...
    'reference_conditions_pass',false,'geometry_heading_kinematics_pass',false, ...
    'hlc_closed_loop_certified',false,'ieee_interval_certified',false, ...
    'physical_future_trigger_bound',false, ...
    'production_commit_authorized',false,'reference_packet',struct([]));
assert(isstruct(A)&&isscalar(A)&&isfield(A,'partial_reference_found')&& ...
    isfield(A,'best_partial')&&isfield(A,'reference_version')&& ...
    isfield(A,'snapshot_time_s'), ...
    'R105:ActualR104ResultRequired','Expected a completed R104 search result.');
assert(isstruct(opts)&&isscalar(opts)&&isfield(opts,'max_nodes_per_check'), ...
    'R105:ExplicitBudgetRequired','Use your existing work102/work104 options.');
if ~logical(A.partial_reference_found)||isempty(A.best_partial)
    return;
end
R=A.best_partial;
assert(isstruct(R)&&isscalar(R)&&isfield(R,'route')&& ...
    isfield(R,'assessment')&&isfield(R,'reference_version'), ...
    'R105:BestPartialFormat','Malformed saved partial route.');
route=R.route;
assert(isstruct(route)&&isscalar(route)&& ...
    isfield(route,'reference_candidate_found')&& ...
    route.reference_candidate_found&&isfield(route,'knot_jets'), ...
    'R105:BestPartialRoute','Only an existing generated C7 connector is supported.');
O=local_find(agent,5);
assert(~isempty(O)&&isprop(O,'r88_snapshots'), ...
    'R105:NativeSnapshotAuthority','Source-bound replanner snapshots required.');
idx=AF7R58R88_SELECT_TIME(O.r88_snapshots,double(A.snapshot_time_s));
assert(~isempty(idx),'R105:SnapshotNotFound','Exact Flight snapshot required.');
Z=O.r88_snapshots(idx);
if ~isequal(double(Z.reference_version),double(A.reference_version))|| ...
        ~isequal(double(R.reference_version),double(A.reference_version))|| ...
        ~isequal(double(Z.time_s),double(A.snapshot_time_s))
    G.status='STALE_REFERENCE_OR_SNAPSHOT_VERSION';return;
end
G.source_version_matches=true;
G.estimated_mL_kg=double(Z.estimated_mass_kg);
J0nom=AF7R58R88_NOMINAL_JETS(Z.nominal_derivative_fns, ...
    Z.flight_reference_time_origin,Z.time_s);
if strcmp(char(Z.reference_mode),'NOMINAL')
    J0raw=J0nom;
elseif strcmp(char(Z.reference_mode),'REPAIRED')
    % Preserve the same exact native source-binding contract as the pinned
    % R102 generator; never replace repaired jets with nominal samples.
    M=Z.batch_flat_model;
    assert(isstruct(M)&&isfield(M,'certified')&&logical(M.certified), ...
        'R105:ActiveModelNotCertified','Saved active repaired model is not source certified.');
    assert(~(isfield(M,'phase_layers')&&~isempty(M.phase_layers)), ...
        'R105:NontrivialPhaseRequiresNativeJet7', ...
        'No identity assumption for a repaired reference time law.');
    Q=O.af7r48_native_eval_packet_from_batch_model(M);
    assert(isfield(Q,'repair_phase_enabled')&& ...
        ~any(double(Q.repair_phase_enabled(:))~=0), ...
        'R105:RepairedPhaseNotIdentity','Exact source C7 jets unavailable.');
    assert(exist('af7r36_single_core_mex','file')==3, ...
        'R105:NativeJetEvaluatorMissing','Pinned R102 requires native MEX.');
    nv=double(af7r36_single_core_mex(2,Q,Z.time_s));nv=nv(:);
    j7=double(af7r36_single_core_mex(25,Q,Z.time_s));j7=j7(:);
    assert(numel(nv)==21&&numel(j7)==3&&all(isfinite([nv;j7])), ...
        'R105:NativeJetPacket','Invalid exact repaired source 0..7 jets.');
    J0raw=zeros(3,8);
    for d=0:6,J0raw(:,d+1)=nv(3*d+(1:3));end
    J0raw(:,8)=J0nom(:,8)+j7;
    xf=double(Z.flat_eval_fn(Z.time_s));xf=xf(:);
    assert(numel(xf)>=27&&all(isfinite(xf(1:27))), ...
        'R105:ActiveFlatEvaluator','Missing exact active 0..6 target evaluation.');
    for d=0:6
        lhs=J0raw(:,d+1);rhs=xf(4*d+(1:3));
        tol=512*max(eps(max(1,abs(lhs))),eps(max(1,abs(rhs))));
        assert(all(abs(lhs-rhs)<=tol), ...
            'R105:NativeLiveFlatMismatch','Actual repaired jets are inconsistent.');
    end
else
    G.status='UNSUPPORTED_SNAPSHOT_REFERENCE_MODE';return;
end
J0=J0raw;
J0(3,1)=J0(3,1)-double(Z.cable_length_m);
if ~isequaln(route.knot_jets(:,:,1),J0)
    G.status='CONNECTOR_NOT_EXACT_ACTIVE_PAYLOAD_JET07';return;
end
% The nominal derivatives in Z are RAW flight-reference z; R83 is the
% single authoritative converter to HLC payload frame, including obstacle
% sweep inflation. This preserves the R102/R104 exact obstacle semantics.
raw=J0;raw(3,1)=raw(3,1)+double(Z.cable_length_m);
nominal=@(t)AF7R58R88_NOMINAL_JETS( ...
    Z.nominal_derivative_fns,Z.flight_reference_time_origin,t);
S=struct('active_raw_jets',raw,'nominal_raw_jets_at',nominal, ...
    'known_mask',Z.known_mask,'obstacle_packet',Z.obstacle_packet, ...
    'reference_version',Z.reference_version,'reference_mode',Z.reference_mode, ...
    'gravity_m_s2',Z.gravity_m_s2,'cable_length_m',Z.cable_length_m, ...
    'rQ_m',Z.rQ_m,'rC_m',Z.rC_m,'rL_m',Z.rL_m, ...
    'd_margin_m',Z.d_margin_m,'sensor_range_m',Z.sensor_range_m, ...
    'horizon_s',Z.horizon_s,'prediction_dt_s',Z.prediction_dt_s, ...
    'latest_EKF_mL_kg',Z.estimated_mass_kg);
B=AF7R58R83_BIND_R81_INPUTS(S);
if ~isequaln(B.old_payload_jets,route.knot_jets(:,:,1))
    G.status='R83_ACTIVE_FRAME_MISMATCH';return;
end
G.source_obstacles_bound=true;
if numel(B.obstacles)~=nnz(logical(Z.known_mask))
    G.status='KNOWN_OBSTACLE_COUNT_MISMATCH';return;
end
rating=[];
if isfield(opts,'input_authority')&&isstruct(opts.input_authority)&& ...
        isfield(opts.input_authority,'tension_max_N')
    rating=opts.input_authority.tension_max_N;
end
[I,Istat]=AF7R58R102_LIVE_INPUT_AUTHORITY(agent,rating);
G.nominal_input_authority_status=Istat.status;
G.rating_source_supplied=Istat.cable_strength_available;
if ~Istat.input_authorities_available
    G.status='MISSING_VERIFIED_LIVE_HLC_COMMAND_LIMITS';return;
end
V=B.live;
V.source_reference_version=Z.reference_version;
V.vehicle_mass_kg=I.vehicle_mass_kg;
V.thrust_bounds_N=I.thrust_bounds_N;
V.torque_bounds_Nm=I.torque_bounds_Nm;
if Istat.cable_strength_available,V.tension_max_N=I.tension_max_N;end
% Identify the finite source tail; no fabricated infinite horizon.
assert(isprop(O,'nominal_table_tend')&&isprop(O,'nominal_table_valid')&& ...
    logical(O.nominal_table_valid), ...
    'R105:NominalTableDomain','Valid runtime nominal table required.');
source=struct('nominal_derivative_fns',{Z.nominal_derivative_fns}, ...
    'flight_reference_time_origin',double(Z.flight_reference_time_origin), ...
    'nominal_table_tend',double(O.nominal_table_tend), ...
    'cable_length_m',double(Z.cable_length_m));
[Jend,tail]=AF7R58R105_C7_TAIL_EVAL(route,source,route.times_s(end));
if ~tail.inside_finite_source_domain
    G.status='C7_SOURCE_TAIL_OUTSIDE_NOMINAL_DOMAIN';return;
end
Jtarget=B.nominal_payload_jets_at(route.times_s(end)-route.phase_delay_s);
G.all_c7_seams=AF7R58R105_C7_SEAM_AUDIT(route,source);
if ~G.all_c7_seams.finite_nominal_source_matches_last_stored_knot
    G.status='C7_TERMINAL_PHASE_JETS_DO_NOT_MATCH_LIVE_NOMINAL';return;
end
if ~tail.heading_nondegenerate_numeric
    G.status='C6_PATH_TANGENT_HEADING_DEGENERATE_AT_REJOIN';return;
end
G.nominal_phase_end_s=route.times_s(end)-route.phase_delay_s;
G.finite_nominal_tail_until_s=source.nominal_table_tend+route.phase_delay_s;
% Compare ALL seven derivatives of the actual polynomial, not just stored
% knot jets. Only numerical diagnostics; no IEEE or global C7 proof claim.
G.numeric_c7_tail_seam_error=max(abs(Jend-Jtarget),[],'all');
% No re-optimization; unchanged R94 independent continuous-time gates.
C=AF7R58R94_CERTIFY_ALL_SEGMENTS(route,B.obstacles,V,opts,{});
G.reference_certifier=C;
G.segments=C.segments;
G.reference_conditions_pass=C.all_segment_reference_conditions_pass;
G.geometry_heading_kinematics_pass=C.geometry_kinematics_heading_pass;
upperT=[];upperF=[];
for k=1:numel(C.segments)
    In=C.segments(k).input;
    if isstruct(In)&&isfield(In,'tension_upper_N')&& ...
            isfinite(In.tension_upper_N)&&In.tension_upper_N>0
        upperT(end+1)=In.tension_upper_N; %#ok<AGROW>
    end
    if isstruct(In)&&isfield(In,'nominal_thrust_upper_N')&& ...
            isfinite(In.nominal_thrust_upper_N)
        upperF(end+1)=In.nominal_thrust_upper_N; %#ok<AGROW>
    end
end
if ~isempty(upperT)
    G.nominal_tension_upper_N=max(upperT);
    G.minimum_verified_rating_needed_N=G.nominal_tension_upper_N;
end
if ~isempty(upperF),G.nominal_thrust_upper_N=max(upperF);end
G.reused_existing_C7_control_points=true;
G.reference_packet=struct('segments',route.segments, ...
    'knot_jets',route.knot_jets,'times_s',route.times_s, ...
    'phase_delay_s',route.phase_delay_s, ...
    'reference_version',double(Z.reference_version), ...
    'snapshot_time_s',double(Z.time_s), ...
    'known_obstacle_ids',B.known_ids, ...
    'estimated_mL_kg',double(Z.estimated_mass_kg), ...
    'finite_tail_until_s',G.finite_nominal_tail_until_s, ...
    'position_c7_connector_numeric_only',true, ...
    'heading_c6_reference_certified',C.heading_c6_certified, ...
    'full_execution_certified',false, ...
    'production_commit_authorized',false);
if ~G.geometry_heading_kinematics_pass
    G.status='REVALIDATION_GEOMETRY_HEADING_KINEMATICS_NOT_ALL_PASS';
elseif ~G.rating_source_supplied
    G.status='GEOMETRY_HEADING_KINEMATICS_PASS__CABLE_STRENGTH_AUTHORITY_REQUIRED';
elseif ~G.reference_conditions_pass
    G.status='GEOMETRY_PASS__NOMINAL_INPUT_OR_STRENGTH_UNCERTIFIED';
else
    G.status='NOMINAL_REFERENCE_SEGMENTS_PASS__TAIL_TORQUE_TUBE_OPEN';
end
end

function O=local_find(x,depth)
O=[];
if isa(x,'REPLANNING_SYSTEM')&&isscalar(x),O=x;return;end
if depth<1||isempty(x)||numel(x)~=1,return;end
if isobject(x)
    try,n=properties(x);catch,return;end
elseif isstruct(x)
    n=fieldnames(x);
else
    return;
end
skip={'self','result','parameter','plant','sensor','estimator', ...
    'controller','logger','time','state','parent'};
for j=1:numel(n)
    if any(strcmpi(n{j},skip)),continue;end
    try,O=local_find(x.(n{j}),depth-1);catch,O=[];end
    if ~isempty(O),return;end
end
end
