function A=AF7R58R104_NATURAL_RETURN_SEARCH(agent,snapshotTime,opts)
% R104: actual R102 C7 generator with independently selected maneuver time
% and delayed NOMINAL PHASE. Offline saved-Flight replay only. Not an online
% solver, not a closed-loop predictor or authority to bypass native commit.
req={'max_total_attempts','max_side_assignments','max_nodes_per_check'};
assert(isstruct(opts)&&isscalar(opts)&&all(isfield(opts,req)), ...
    'R104:WorkOptions','Explicit solver budgets are required.');
for j=1:numel(req)
    x=opts.(req{j});
    assert(isnumeric(x)&&isscalar(x)&&isfinite(x)&&x>=1&&fix(x)==x, ...
        'R104:WorkOptionValue','Budget %s must be a positive integer.',req{j});
end
O=local_find(agent,5);
assert(~isempty(O)&&isprop(O,'r88_snapshots'), ...
    'R104:LiveSourceMissing','REPLANNING_SYSTEM with saved Flight snapshot required.');
idx=AF7R58R88_SELECT_TIME(O.r88_snapshots,snapshotTime);
assert(~isempty(idx),'R104:SnapshotTimeMissing', ...
    'No exact Flight snapshot at the requested event.');
Z=O.r88_snapshots(idx);
assert(isscalar(Z)&&isfinite(Z.gravity_m_s2)&&Z.gravity_m_s2>0 && ...
    isfinite(Z.cable_length_m)&&Z.cable_length_m>0, ...
    'R104:PhysicalAuthorityMissing','Live gravity/cable parameters required.');
if isfield(opts,'input_authority')&&isfield(opts.input_authority,'tension_max_N')
    cableRating=opts.input_authority.tension_max_N;
else
    cableRating=[];
end
[authority,authorityStatus]=AF7R58R102_LIVE_INPUT_AUTHORITY(agent,cableRating);
if authorityStatus.input_authorities_available
    if isfield(opts,'input_authority')&&isstruct(opts.input_authority)
        fields={'vehicle_mass_kg','thrust_bounds_N','torque_bounds_Nm'};
        for j=1:numel(fields)
            k=fields{j};
            if isfield(opts.input_authority,k)
                a=double(authority.(k));b=double(opts.input_authority.(k));
                assert(isequal(size(a),size(b))&&all(isfinite(b(:)))&& ...
                    all(abs(a(:)-b(:))<=64*eps(max([1;abs(a(:));abs(b(:))]))), ...
                    'R104:AuthorityConflict','%s disagrees with the verified live controller.',k);
            end
        end
    end
    opts.input_authority=authority;
end
baseT=sqrt(double(Z.cable_length_m)/double(Z.gravity_m_s2));
ratios=[];
if isfield(opts,'phase_progress_ratios'),ratios=opts.phase_progress_ratios;end
schedule=AF7R58R104_PHASE_SCHEDULE(snapshotTime,baseT,opts.max_total_attempts,ratios);
seedTemplate=AF7R58R102_SEED_RECORD();
X0=struct('attempt_index',0,'duration_s',NaN, ...
    'phase_advance_s',NaN,'phase_delay_s',NaN,'status','', ...
    'seed_count',0,'partial_count',0,'full_count',0, ...
    'first_reason','','exception','');
A=struct('status','NO_CERTIFIED_C7_CANDIDATE', ...
    'attempts',0,'reference_version',double(Z.reference_version), ...
    'snapshot_time_s',double(Z.time_s),'known_obstacle_count',nnz(Z.known_mask), ...
    'estimated_mass_kg',double(Z.estimated_mass_kg), ...
    'r102_live_input_authority_status',authorityStatus.status, ...
    'partial_reference_found',false,'candidate_found',false, ...
    'best_partial',struct([]),'best_full_reference',struct([]), ...
    'best_partial_length_m',Inf,'first_failed_reason','', ...
    'first_exception','','attempt_log',repmat(X0,1,0), ...
    'seed_log',repmat(seedTemplate,1,0), ...
    'status_counts',struct(), ...
    'physical_predictor_source_bound',false, ...
    'complete_hlc_closed_loop_certificate',false, ...
    'production_commit_authorized',false, ...
    'native_generator_switched',false);
for i=1:numel(schedule)
    Q=schedule(i);A.attempts=A.attempts+1;
    E=X0;E.attempt_index=i;E.duration_s=Q.duration_s;
    E.phase_advance_s=Q.phase_advance_s;
    E.phase_delay_s=Q.phase_delay_s;
    cfg=opts;cfg.times_s=[snapshotTime,Q.terminal_time_s];
    cfg.phase_delay_s=Q.phase_delay_s;
    cfg.internal_waypoints='AUTO_SUPPORT';
    % The source-bounded NOMINAL PHASE is not the physical maneuver
    % duration.  A long maneuver may remain legal even if t_end exceeds
    % the nominal table's supported domain, provided tau_end does not.
    % Never extrapolate a known finite reference table in a prefilter.
    [phaseInDomain,phaseReason]=local_source_phase_domain(O,Z,Q);
    if ~phaseInDomain
        E.status='PREFILTER_REFERENCE_PHASE_OUTSIDE_LIVE_DOMAIN';
        E.first_reason=phaseReason;
        if isempty(A.first_failed_reason),A.first_failed_reason=phaseReason;end
        A.attempt_log(end+1)=E; %#ok<AGROW>
        continue;
    end
    % Nominal-only CHEAP necessary lower bound before generating any seed.
    % Repaired references require native C7 jets and are deliberately left
    % to the existing source-bound R102 runner (no nominal substitution).
    % A failure to evaluate the exact nominal derivative is INCONCLUSIVE,
    % not proof that a collision-free path is physically impossible.
    try
        [prePass,preReason]=local_endpoint_screen(Z,Q,opts);
    catch ME
        E.status='INCONCLUSIVE_NOMINAL_ENDPOINT_EVALUATION';
        E.first_reason='SOURCE_NOMINAL_JETS_UNAVAILABLE';
        E.exception=[char(ME.identifier) ':' char(ME.message)];
        if ~isempty(ME.stack)
            E.exception=[E.exception sprintf(' @%s:%d',ME.stack(1).name,ME.stack(1).line)];
        end
        if isempty(A.first_exception),A.first_exception=E.exception;end
        if isempty(A.first_failed_reason),A.first_failed_reason=E.first_reason;end
        A.attempt_log(end+1)=E; %#ok<AGROW>
        continue;
    end
    if ~prePass
        E.status='PREFILTER_ENDPOINT_UAV_NECESSARY_TIME';
        E.first_reason=preReason;
        if isempty(A.first_failed_reason),A.first_failed_reason=preReason;end
        A.attempt_log(end+1)=E; %#ok<AGROW>
        continue;
    end
    try
        R=AF7R58R102_RUN_SAVED_ROUTE(agent,snapshotTime,cfg);
        E.status=char(R.status);
        if isfield(R,'seed_attempt_log')
            E.seed_count=numel(R.seed_attempt_log);
            for k=1:E.seed_count
                L=orderfields(R.seed_attempt_log(k),seedTemplate);
                L.attempt_index=i;
                A.seed_log(end+1)=L; %#ok<AGROW>
                if logical(L.geometry_kinematics_heading_pass)
                    E.partial_count=E.partial_count+1;
                end
                if logical(L.all_segment_reference_conditions_pass)
                    E.full_count=E.full_count+1;
                end
                if isempty(E.first_reason)&&~isempty(L.reason)
                    E.first_reason=char(L.reason);
                end
            end
        end
        if E.seed_count==0,E.first_reason=E.status;end
        if isfield(R,'partial_reference_found')&&logical(R.partial_reference_found)
            d=double(R.best_partial_length_m);
            if ~A.partial_reference_found||d<A.best_partial_length_m
                A.partial_reference_found=true;A.best_partial=R;
                A.best_partial_length_m=d;
            end
        end
        if isfield(R,'full_reference_found')&&logical(R.full_reference_found)
            A.candidate_found=true;A.best_full_reference=R;
            A.status='REFERENCE_CERTIFICATE_FOUND__EXECUTION_UNPROVEN';
            A.attempt_log(end+1)=E; %#ok<AGROW>
            break;
        end
    catch ME
        E.status='INCONCLUSIVE_GENERATOR_EXCEPTION';
        E.exception=[char(ME.identifier) ':' char(ME.message)];
        if ~isempty(ME.stack)
            E.exception=[E.exception sprintf(' @%s:%d',ME.stack(1).name,ME.stack(1).line)];
        end
        if isempty(A.first_exception),A.first_exception=E.exception;end
    end
    if isempty(A.first_failed_reason)&&~isempty(E.first_reason)
        A.first_failed_reason=E.first_reason;
    end
    A.attempt_log(end+1)=E; %#ok<AGROW>
end
A.status_counts=local_count(A.seed_log);
if ~isempty(A.first_exception)
    A.status='INCONCLUSIVE_GENERATOR_EXCEPTION__NO_COMMIT';
elseif ~A.candidate_found&&A.partial_reference_found
    A.status='PARTIAL_GEOMETRY_KINEMATICS__NO_COMMIT';
elseif ~A.candidate_found
    A.status='NO_CERTIFIED_C7_CANDIDATE__NO_COMMIT';
end
end

function C=local_count(log)
C=struct();
for i=1:numel(log)
    name=matlab.lang.makeValidName(char(log(i).status));
    if isempty(name),name='UNSPECIFIED';end
    if ~isfield(C,name),C.(name)=0;end
    C.(name)=C.(name)+1;
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

function [pass,reason]=local_source_phase_domain(O,Z,Q)
% The existing nominal table is the runtime source of truth for a finite
% tabulated reference domain. This bounds nominal PHASE only; it does NOT
% bound physical maneuver duration or replace the 4 s trigger horizon.
pass=true;reason='';
tau=double(Q.nominal_phase_time_s);
if ~isfinite(tau)||tau<double(Z.time_s)
    pass=false;reason='INVALID_NOMINAL_REJOIN_PHASE';return;
end
if isprop(O,'nominal_table_valid') && ...
        logical(O.nominal_table_valid) && ...
        isprop(O,'nominal_table_tend')
    bound=double(O.nominal_table_tend);
    if isscalar(bound)&&isfinite(bound)
        tol=64*eps(max([1,abs(tau),abs(bound)]));
        if tau>bound+tol
            pass=false;reason='NOMINAL_PHASE_EXCEEDS_LIVE_REFERENCE_TABLE';
        end
    end
end
end

function [pass,reason]=local_endpoint_screen(Z,Q,opts)
pass=true;reason='';
if ~strcmp(char(Z.reference_mode),'NOMINAL'),return;end
f=Z.nominal_derivative_fns;
J0=AF7R58R88_NOMINAL_JETS(f,Z.flight_reference_time_origin,Z.time_s);
J1=AF7R58R88_NOMINAL_JETS(f,Z.flight_reference_time_origin,Q.nominal_phase_time_s);
g=double(Z.gravity_m_s2);L=double(Z.cable_length_m);
f0=J0(:,3)+[0;0;g];f1=J1(:,3)+[0;0;g];
if norm(f0)<=0||norm(f1)<=0||~all(isfinite([f0;f1]))
    pass=false;reason='PRECHECK_UNDEFINED_CABLE_DIRECTION';return;
end
rho=max([double(Z.rQ_m),L+double(Z.rL_m),L+double(Z.rC_m)]);
vmax=(double(Z.sensor_range_m)-rho)/(double(Z.horizon_s)+double(Z.prediction_dt_s));
if isfield(opts,'operational_uav_speed_limit_m_s')
    vmax=min(vmax,double(opts.operational_uav_speed_limit_m_s));
end
assert(isfinite(vmax)&&vmax>0,'R104:FiniteSensingLimit', ...
    'Sensor range and geometric reach must produce a positive velocity budget.');
q0=f0/norm(f0);q1=f1/norm(f1);
required=norm(J1(:,1)+L*q1-J0(:,1)-L*q0);
if required>vmax*Q.duration_s+128*eps(max([1,required,vmax*Q.duration_s]))
    pass=false;reason='NECESSARY_ENDPOINT_UAV_TRAVEL_TIME_VIOLATION';
end
end
