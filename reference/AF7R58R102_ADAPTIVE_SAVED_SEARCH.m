function A=AF7R58R102_ADAPTIVE_SAVED_SEARCH(agent,snapshotTime,opts)
% R100: honest, full diagnostics for multi-segment reference-only C7 search.
% Does NOT change the Flight trigger/generator/commit. Computational work
% budgets are not maneuver-time limits. No physical fallback constants.
req={'max_total_attempts','max_side_assignments','max_nodes_per_check'};
assert(isstruct(opts)&&isscalar(opts),'R94:OptionsRequired','Provide a scalar struct containing positive computational work budgets.');
for i=1:numel(req)
    assert(isfield(opts,req{i}), ...
        'R94:MissingWorkBudget','Missing required work budget %s.',req{i});
    x=opts.(req{i});
    assert(isnumeric(x)&&isscalar(x)&&isfinite(x)&&x>=1&&fix(x)==x, ...
        'R94:BadWorkBudget','Work budget %s must be a positive integer.',req{i});
end
base=fileparts(mfilename('fullpath'));
resolved=which('AF7R58R102_RUN_SAVED_ROUTE');
assert(strcmp(fileparts(resolved),base),'R94:RunnerPath', 'MATLAB did not resolve the R96 runner in this reference directory.');
% No fragile cross-version SHA whitelist. Pinned R96 function names are
% unique and all code files must resolve from the same reference directory.
deps={'AF7R58R94_CERTIFY_ALL_SEGMENTS','AF7R58R97_SUPPORT_SEED_ROUTES', ...
    'AF7R58R96_C7_KNOT_SPEED_ROUTE','AF7R58R96_KNOT_DISPLACEMENT_SCREEN', ...
    'AF7R58R97_TIGHT_SUPPORT_CORRIDOR', ...
    'AF7R58R96_C7_CORRIDOR_PROJECT', ...
    'AF7R58R96_KNOT_VQ_SCREEN', ...
    'AF7R58R96_KNOT_REACHABILITY_PREFLIGHT', ...
    'AF7R58R96_SHA256_FILE', ...
    'AF7R58R98_ALLOCATE_KNOT_TIMES', ...
    'AF7R58R99_OVERLAP_LOCAL_CORRIDOR_CHAIN', ...
    'AF7R58R100_CO_DESIGN_TIMES', ...
    'AF7R58R100_C7_INTERIOR_SPEED_AWARE', ...
    'AF7R58R102_C7_CONTINUOUS_JET_OPT', ...
    'AF7R58R102_LIVE_INPUT_AUTHORITY'};
for i=1:numel(deps)
    actual=which(deps{i});
    if isempty(actual)||~strcmp(fileparts(actual),base)
       error('R94:DependencyPathShadow', ...
         'Expected %s in reference directory, got %s.',deps{i},actual);
    end
end
obj=locate(agent,5);
assert(~isempty(obj),'R94:ReplannerNotFound', 'Could not locate the REPLANNING_SYSTEM instance in agent.');
idx=AF7R58R88_SELECT_TIME(obj.r88_snapshots,snapshotTime);
assert(~isempty(idx),'R94:SnapshotMissing', 'There is no R88 snapshot at this exact time.');
S=obj.r88_snapshots(idx);
% Bind the source-verified ACTUAL HLC command clamps and the vehicle mass.
% No guessed physical values. Exact episode mL remains snapshot-owned.
externalCableRating=[];
if isfield(opts,'input_authority') && isstruct(opts.input_authority) && ...
        isfield(opts.input_authority,'tension_max_N')
    externalCableRating=opts.input_authority.tension_max_N;
end
[liveAuthority,authorityStatus]=AF7R58R102_LIVE_INPUT_AUTHORITY( ...
    agent,externalCableRating);
if authorityStatus.input_authorities_available
    if isfield(opts,'input_authority') && isstruct(opts.input_authority)
        given=opts.input_authority;
        bindNames={'vehicle_mass_kg','thrust_bounds_N','torque_bounds_Nm'};
        for z=1:numel(bindNames)
            key=bindNames{z};
            if isfield(given,key)
                expected=double(liveAuthority.(key));supplied=double(given.(key));
                assert(isequal(size(expected),size(supplied)) && ...
                    all(isfinite(supplied(:))) && ...
                    all(abs(expected(:)-supplied(:))<= ...
                    64*eps(max([1;abs(expected(:));abs(supplied(:))]))), ...
                    'R102:ConflictingLiveInputAuthority', ...
                    'Supplied %s conflicts with verified live HLC/vehicle authority.',key);
            end
        end
    end
    opts.input_authority=liveAuthority;
end
assert(isfinite(S.gravity_m_s2)&&S.gravity_m_s2>0&& ...
    isfinite(S.cable_length_m)&&S.cable_length_m>0, ...
    'R94:MissingLivePhysics','R88 snapshot must contain the live gravity and cable length.');
baseT=sqrt(double(S.cable_length_m)/double(S.gravity_m_s2));
inputAvailable=isfield(opts,'input_authority')&&isstruct(opts.input_authority)&& ...
    all(isfield(opts.input_authority, ...
    {'vehicle_mass_kg','thrust_bounds_N','torque_bounds_Nm'}));
A=struct('status','INCONCLUSIVE_NO_REFERENCE_CANDIDATE', ...
    'attempts',0,'partial_reference_found',false, ...
    'candidate_found',false,'best_partial',struct([]), ...
    'best_full_reference',struct([]), ...
    'best_partial_polyline_length_m',Inf, ...
    'best_partial_duration_s',NaN,'best_partial_phase_delay_s',NaN, ...
    'first_failed_reason','','last_failed_reason','','first_exception','', ...
    'attempt_log',repmat(struct('attempt_index',0,'duration_s',NaN, ...
        'phase_delay_s',NaN,'status','','seed_count',0, ...
        'partial_seed_count',0,'full_reference_seed_count',0, ...
        'first_failed_reason','','error',''),1,0), ...
    'seed_log',repmat(AF7R58R102_SEED_RECORD(),1,0), ...
    'status_counts',struct(),'external_input_authority_supplied',inputAvailable, ...
    'external_tension_rating_supplied', ...
        inputAvailable&&isfield(opts.input_authority,'tension_max_N'), ...
    'source_snapshot_time_s',double(S.time_s), ...
    'runner_source_path',resolved, ...
    'runner_source_sha256', ...
       AF7R58R96_SHA256_FILE(resolved), ...
    'pinned_certifier_path',which('AF7R58R94_CERTIFY_ALL_SEGMENTS'), ...
    'reference_version',double(S.reference_version), ...
    'estimated_mass_at_snapshot_kg',double(S.estimated_mass_kg), ...
    'known_obstacle_count',nnz(S.known_mask), ...
    'r102_live_input_authority_status',authorityStatus.status, ...
    'nominal_tail_certified',false,'actual_hlc_torque_certified',false, ...
    'closed_loop_tracking_tube_certified',false, ...
    'ieee_interval_proved',false,'production_commit_authorized',false);
level=0;
while A.attempts<opts.max_total_attempts
    T=baseT*2^level;
    if ~isfinite(T)||~isfinite(snapshotTime+T)||snapshotTime+T<=snapshotTime
        A.status='INCONCLUSIVE_MACHINE_TIME_RANGE';break;
    end
    delays=0;
    if level>=1,delays=[0 baseT*2.^(0:level-1)];end
    for j=1:numel(delays)
        if A.attempts>=opts.max_total_attempts,break;end
        delta=delays(j);
        cfg=opts;
        cfg.times_s=[snapshotTime snapshotTime+T];
        cfg.phase_delay_s=delta;
        cfg.internal_waypoints='AUTO_SUPPORT';
        A.attempts=A.attempts+1;
        X=struct('attempt_index',A.attempts,'duration_s',T, ...
            'phase_delay_s',delta,'status','NOT_EVALUATED','seed_count',0, ...
            'partial_seed_count',0,'full_reference_seed_count',0, ...
            'first_failed_reason','','error','');
        try
            B=AF7R58R102_RUN_SAVED_ROUTE(agent,snapshotTime,cfg);
            X.status=B.status;
            if isfield(B,'seed_attempt_log')
                X.seed_count=numel(B.seed_attempt_log);
                for k=1:numel(B.seed_attempt_log)
                    L=B.seed_attempt_log(k);
                    if L.geometry_kinematics_heading_pass
                        X.partial_seed_count=X.partial_seed_count+1;
                    end
                    if L.all_segment_reference_conditions_pass
                        X.full_reference_seed_count=X.full_reference_seed_count+1;
                    end
                    if isempty(X.first_failed_reason)
                        if ~isempty(L.reason)
                            X.first_failed_reason=L.reason;
                        elseif ~isempty(L.error)
                            X.first_failed_reason=L.error;
                        elseif ~L.all_segment_reference_conditions_pass
                            X.first_failed_reason=L.status;
                        end
                    end
                    C=L;C.attempt_index=A.attempts;
                    A.seed_log(end+1)=orderfields(C,AF7R58R102_SEED_RECORD()); %#ok<AGROW>
                end
            else
                X.status='R96_RUNNER_MISSING_SEED_LOG__SOURCE_MISMATCH';
            end
            if X.seed_count==0 && isempty(X.first_failed_reason)
                X.first_failed_reason=B.status;
            end
            if isfield(B,'partial_reference_found') && B.partial_reference_found
                pathLen=B.best_partial_length_m;
                if ~A.partial_reference_found||pathLen<A.best_partial_polyline_length_m
                    A.partial_reference_found=true;
                    A.best_partial=B;
                    A.best_partial_polyline_length_m=pathLen;
                    A.best_partial_duration_s=T;
                    A.best_partial_phase_delay_s=delta;
                end
            end
            if isfield(B,'full_reference_found') && B.full_reference_found
                A.candidate_found=true;
                A.best_full_reference=B;
                A.status='PARTIAL_NUMERIC_REFERENCE_ONLY__NOT_COMMITTABLE';
                A.attempt_log(end+1)=X; %#ok<AGROW>
                A.status_counts=countStatuses(A.seed_log);
                return;
            end
        catch ME
            X.status='FAIL_CLOSED_ATTEMPT_EXCEPTION';
            X.error=[char(ME.identifier) ':' char(ME.message)];
            if isempty(A.first_exception),A.first_exception=X.error;end
        end
        if ~isempty(X.first_failed_reason)
            A.last_failed_reason=X.first_failed_reason;
        end
        if isempty(A.first_failed_reason)&&~isempty(X.first_failed_reason)
            A.first_failed_reason=X.first_failed_reason;
        end
        A.attempt_log(end+1)=X; %#ok<AGROW>
    end
    level=level+1;
end
A.status_counts=countStatuses(A.seed_log);
if ~isempty(A.first_exception)
    A.status='INCONCLUSIVE_PROGRAM_EXCEPTION__NOT_COMMITTABLE';
elseif A.partial_reference_found
    A.status='PARTIAL_GEO_KINEMATIC_HEADING_FOUND__NOT_COMMITTABLE';
elseif ~inputAvailable
    A.status='NO_PARTIAL_CANDIDATE_AND_NO_INPUT_AUTHORITY__NOT_COMMITTABLE';
elseif ~A.external_tension_rating_supplied
    A.status='NO_PARTIAL_CANDIDATE_AND_NO_CABLE_RATING__NOT_COMMITTABLE';
end
% Missing input authority is always reported even when a geometry-only
% candidate is found. It must never be interpreted as an actuator proof.
assert(~A.production_commit_authorized,'R94:ReferenceOnly','Reference search may never authorize a production commit.');
end

function C=countStatuses(log)
C=struct();
for i=1:numel(log)
    key=matlab.lang.makeValidName(char(log(i).status));
    if isempty(key),key='UNKNOWN';end
    if isfield(C,key),C.(key)=C.(key)+1;else,C.(key)=1;end
end
end

function obj=locate(x,depth)
obj=[];
if isa(x,'REPLANNING_SYSTEM')&&isscalar(x),obj=x;return;end
if depth<1||isempty(x)||numel(x)~=1,return;end
if isobject(x)
    try,n=properties(x);catch,return;end
elseif isstruct(x),n=fieldnames(x);
elseif iscell(x)
    for j=1:numel(x)
        obj=locate(x{j},depth-1);
        if ~isempty(obj),return;end
    end
    return;
else,return;end
skip={'self','result','parameter','plant','sensor','estimator', ...
    'controller','logger','time','state','parent'};
for j=1:numel(n)
    if any(strcmpi(n{j},skip)),continue;end
    try,obj=locate(x.(n{j}),depth-1);catch,obj=[];end
    if ~isempty(obj),return;end
end
end
