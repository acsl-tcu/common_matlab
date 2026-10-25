function A=AF7R58R92_CERTIFY_ALL_SEGMENTS(R,knownObs,live,opts,corridors)
% Validate every segment of R92 against source-bound reference constraints.
% Separate strict per-segment UAV, CABLE, PAYLOAD, heading, speed, nominal
% taut thrust and tension certificates. Safety NEVER follows from finding
% a waypoint route; only these independent checks may establish reference
% feasibility, and NOT closed-loop/execution safety or production commit.
% The nominal continuation/tail must have its own complete certificate.
% Corridor is an optional per-segment payload CP constraint A*p <= b.
assert(isstruct(R)&&isfield(R,'reference_candidate_found')&& ...
    logical(R.reference_candidate_found),'R92:ValidGeneratedRouteRequired');
assert(isstruct(knownObs),'R92:KnownObstacleAuthorityRequired');
assert(isstruct(live)&&isscalar(live)&&isstruct(opts)&&isscalar(opts), ...
    'R92:ExplicitAuthorityAndWorkRequired');
fields={'gravity_m_s2','cable_length_m','rQ_m','rC_m','rL_m', ...
    'finite_sensing_vQ_limit_m_s'};
for i=1:numel(fields)
    assert(isfield(live,fields{i})&&isnumeric(live.(fields{i}))&& ...
        isscalar(live.(fields{i}))&&isfinite(live.(fields{i})), ...
        'R92:MissingLiveAuthority:%s',fields{i});
end
assert(isfield(opts,'max_nodes_per_check')&& ...
    isnumeric(opts.max_nodes_per_check)&& ...
    isscalar(opts.max_nodes_per_check)&& ...
    isfinite(opts.max_nodes_per_check)&& ...
    opts.max_nodes_per_check>=1&& ...
    fix(opts.max_nodes_per_check)==opts.max_nodes_per_check, ...
    'R92:WorkBudgetRequired');
[live,robustDiag]=AF7R58R92_TRACKING_INFLATION(live,opts);
if nargin<5,corridors={};end
assert(iscell(corridors)&&(isempty(corridors)|| ...
    numel(corridors)==R.num_segments),'R92:OneCorridorPerSegment');
requiredInput={'vehicle_mass_kg','latest_EKF_mL_kg', ...
    'thrust_bounds_N','torque_bounds_Nm'};
inputProvided=all(isfield(live,requiredInput));
% Finite sensing is a necessary kinematic bound, not automatically the
% operational speed limit. Apply an additional live-supplied limit when it
% exists; never invent one based on a particular test flight.
vLimit=live.finite_sensing_vQ_limit_m_s;
operationalLimitKnown=isfield(opts,'operational_uav_speed_limit_m_s');
if operationalLimitKnown
    vOperational=opts.operational_uav_speed_limit_m_s;
    assert(isnumeric(vOperational)&&isscalar(vOperational)&& ...
        isfinite(vOperational)&&vOperational>0, ...
        'R92:OperationalSpeedLimitMustComeFromExternalAuthority');
    vLimit=min(vLimit,double(vOperational));
end
res=repmat(struct('heading',struct([]),'geometry',struct([]), ...
    'kinematics',struct([]),'input',struct([]), ...
    'corridor_pass',false,'reference_pass',false, ...
    'reason','NOT_EVALUATED'),1,R.num_segments);
A=struct('status','FAIL_CLOSED_REFERENCE_NOT_CERTIFIED', ...
    'segments',res,'all_segment_reference_conditions_pass',false, ...
    'input_authorities_available',inputProvided, ...
    'operational_speed_limit_source_supplied',operationalLimitKnown, ...
    'known_obstacles_count',numel(knownObs), ...
    'corridor_constraints_pass',false, ...
    'tail_entirely_certified',false, ...
    'heading_c6_certified',false,'position_c7_constructed',true, ...
    'robust_tracking_tube_certified',false, ...
    'diagnostic_switches',robustDiag, ...
    'actual_hlc_torque_certified',false, ...
    'inter_sample_ieee_proof',false, ...
    'production_commit_authorized',false);
if isempty(knownObs)
    A.status='NOT_EVALUATED_NO_KNOWN_OBSTACLE_COVERAGE';return;
end
allGood=true;allCorr=true;
for s=1:R.num_segments
    C=R.segments(s);
    Q=true;
    if ~isempty(corridors)
        corridor=corridors{s};
        assert(isstruct(corridor)&&isfield(corridor,'A')&& ...
            isfield(corridor,'b'),'R92:MalformedCorridor');
        Ahs=double(corridor.A);rhs=double(corridor.b(:));
        assert(size(Ahs,2)==3&&size(Ahs,1)==numel(rhs)&& ...
            all(isfinite(Ahs(:)))&&all(isfinite(rhs)), ...
            'R92:InvalidConvexCorridor');
        m=Ahs*C.payload_cp-rhs*ones(1,16);
        % All Bezier control points in the convex corridor ==> all
        % payload positions in it. Only an optional additional constraint,
        % it does NOT replace the 3-part R80 whole-body certificate.
        tol=128*eps(max([1,abs(Ahs(:).'),abs(rhs.') ]));
        Q=all(m(:)<-tol);
    end
    A.segments(s).corridor_pass=Q;
    allCorr=allCorr&&Q;
    if ~Q
        A.segments(s).reason='CORRIDOR_CP_UNCERTIFIED';
        allGood=false;
        continue;
    end
    K=AF7R58R91_STABLE_KINEMATIC_CERT(C,live.gravity_m_s2, ...
        live.cable_length_m,vLimit, ...
        opts.max_nodes_per_check);
    A.segments(s).kinematics=K;
    if ~K.certified
        A.segments(s).reason=K.status;allGood=false;continue;
    end
    H=AF7R58R80_HEADING_C6_CERT(C,opts.max_nodes_per_check);
    A.segments(s).heading=H;
    if ~H.certified
        A.segments(s).reason=H.status;allGood=false;continue;
    end
    G=AF7R58R80_CONTINUOUS_SEPARATE_BODY_CERT(C,knownObs, ...
        live.gravity_m_s2,live.cable_length_m,live.rQ_m,live.rC_m, ...
        live.rL_m,opts.max_nodes_per_check);
    A.segments(s).geometry=G;
    if ~G.certified
        A.segments(s).reason=G.status;allGood=false;continue;
    end
    if ~inputProvided
        A.segments(s).reason='MISSING_EXTERNAL_INPUT_LIMITS';
        allGood=false;continue;
    end
    P=struct('gravity_m_s2',live.gravity_m_s2, ...
        'cable_length_m',live.cable_length_m, ...
        'vehicle_mass_kg',live.vehicle_mass_kg, ...
        'latest_EKF_mL_kg',live.latest_EKF_mL_kg, ...
        'thrust_bounds_N',live.thrust_bounds_N, ...
        'torque_bounds_Nm',live.torque_bounds_Nm);
    if isfield(live,'tension_max_N')
        P.tension_max_N=live.tension_max_N;
    end
    In=AF7R58R90_INPUT_ENVELOPE(C,P,opts.max_nodes_per_check);
    A.segments(s).input=In;
    if ~(In.taut_tension_positive_certified && ...
            In.nominal_thrust_certified && In.cable_strength_certified)
        A.segments(s).reason=In.status;allGood=false;continue;
    end
    A.segments(s).reference_pass=true;
    A.segments(s).reason='NOMINAL_REFERENCE_PARTIAL_CERTIFICATES_PASS';
end
A.corridor_constraints_pass=allCorr;
A.segments=res_assign(A.segments);
A.all_segment_reference_conditions_pass=allGood;
A.heading_c6_certified=all(arrayfun(@(x) isstruct(x.heading)&& ...
    isfield(x.heading,'certified')&&x.heading.certified,A.segments));
if allGood
    A.status='ALL_SEGMENTS_NUMERIC_REFERENCE_ONLY__NO_TORQUE_TUBE_TAIL';
end
% A robust tube and documented full-tail closure are mandatory for a
% production safety proof, and neither can be inferred from this function.
end
function x=res_assign(x)
% Keep uniform struct shape; guard against accidental scalar collapsing.
assert(isstruct(x));
end
