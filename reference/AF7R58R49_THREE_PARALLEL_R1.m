% AF7R58R49_THREE_PARALLEL_R1.m
% Three parallel, source-bound NON-PRODUCTION experiments.
% A: C7 no-catch-up time law + exact monotone held-offset taper candidate.
% B: Replanner-only same-event timing decomposition from R48R2 evidence.
% C: Source trace of EKF-current / reference-predicted trigger divergence.
% No generated trajectory is committed. No production source or HLC changes.
% Neither numeric double output nor this program is a whole-body certificate.
%
% Place in common_matlab/reference and execute exactly:
%   run('AF7R58R49_THREE_PARALLEL_R1.m')

fprintf('\n[R49] THREE PARALLEL TRACKS | DIAGNOSTIC / SHADOW ONLY\n');
R49 = af7r49_main();
assignin('base','AF7R58R49_OUT',R49);
fprintf('[R49] RETURN = %s\n',R49.return.status);
fprintf('[R49] TIME   = %s\n',R49.timing.status);
fprintf('[R49] TRIG   = %s\n',R49.trigger.status);
fprintf('[R49] REPORT = %s\n',R49.report_file);
fprintf('[R49] MAT    = %s\n',R49.mat_file);

function OUT=af7r49_main()
root=fileparts(mfilename('fullpath'));
if isempty(root),root=pwd;end
OUT=struct();
OUT.schema='AF7R58R49_PARALLEL_THREE_SHADOW_V1';
OUT.production_modified=false;
OUT.commit_changed=false;
OUT.trigger_changed=false;
OUT.hlc_or_plant_changed=false;
OUT.controller_24_95_spike_in_scope=false;
OUT.continuous_whole_body_certified=false;
OUT.yaw_C6_certified=false;
OUT.executed_C7_rejoin_certified=false;
OUT.actual_state_trigger_certified=false;
OUT.replanner_hard_wcET_certified=false;
OUT.source_pairing_verified=false;
OUT.return=struct('status','NOT_RUN');
OUT.timing=struct('status','NOT_RUN');
OUT.trigger=struct('status','NOT_RUN');
OUT.sources=struct();

[r48,r48file]=af7r49_latest(root, ...
    'AF7R58R48R2_RESULTS_*','AF7R58R48R2_OUT.mat');
[r47,r47file]=af7r49_latest(root, ...
    'AF7R58R47_R3_RESULTS_*','AF7R58R47_OUT.mat');
if isempty(r48file) && isfile(fullfile(root,'AF7R58R48R2_OUT.mat'))
    r48file=fullfile(root,'AF7R58R48R2_OUT.mat');
    r48=af7r49_read_out(r48file);
end
if isempty(r47file) && isfile(fullfile(root,'AF7R58R47_OUT.mat'))
    r47file=fullfile(root,'AF7R58R47_OUT.mat');
    r47=af7r49_read_out(r47file);
end
OUT.sources.r48_file=r48file;
OUT.sources.r47_file=r47file;
OUT.sources.production_source=fullfile(root,'REPLANNING_SYSTEM.m');
if ~isfile(OUT.sources.production_source)
    path=which('REPLANNING_SYSTEM');
    if ~isempty(path),OUT.sources.production_source=path;end
end

try
    OUT.return=af7r49_return_track(r48,r47);
catch ME
    OUT.return=af7r49_failed('RETURN_EXPERIMENT_ERROR',ME);
end
try
    OUT.timing=af7r49_timing_track(r48,r47,OUT.sources.production_source);
catch ME
    OUT.timing=af7r49_failed('TIMING_EXPERIMENT_ERROR',ME);
end
try
    OUT.trigger=af7r49_trigger_track(r48,OUT.sources.production_source);
catch ME
    OUT.trigger=af7r49_failed('TRIGGER_EXPERIMENT_ERROR',ME);
end

% Record only the names and schema of workspace authority; do not infer
% active implementation from older snapshots or place large objects in OUT.
vars={'R44R2','R45R1','C44R2','L45','runtime_profile','avoidance_status'};
W=struct('name',{},'class',{},'size',{},'fields',{});
for k=1:numel(vars)
    try
        exists=evalin('base',sprintf('exist(''%s'',''var'')==1',vars{k}));
        if ~exists,continue;end
        X=evalin('base',vars{k});
        s=struct('name',vars{k},'class',class(X),'size',size(X),'fields',{{}});
        if isstruct(X),s.fields=fieldnames(X);end
        W(end+1)=s; %#ok<AGROW>
    catch ME
        s=struct('name',vars{k},'class',['READ_ERROR_' char(ME.identifier)], ...
            'size',[],'fields',{{}});
        W(end+1)=s; %#ok<AGROW>
    end
end
OUT.workspace_schema=W;

outdir=fullfile(root,['AF7R58R49_RESULTS_' datestr(now,'yyyymmdd_HHMMSS_FFF')]);
assert(~isfolder(outdir),'[R49] Destination already exists.');
mkdir(outdir);
OUT.result_directory=outdir;
OUT.mat_file=fullfile(outdir,'AF7R58R49_OUT.mat');
OUT.report_file=fullfile(outdir,'AF7R58R49_REPORT.txt');
save(OUT.mat_file,'OUT');
af7r49_write_report(OUT);
end

function [R,f]=af7r49_latest(root,pat,name)
R=[];f='';
L=dir(fullfile(root,pat,name));
if isempty(L),return;end
[~,ix]=sort([L.datenum],'descend');
for ii=ix
    candidate=fullfile(L(ii).folder,L(ii).name);
    R=af7r49_read_out(candidate);
    if ~isempty(R),f=candidate;return;end
end
end

function R=af7r49_read_out(f)
R=[];
try
    S=load(f,'OUT');
    if isfield(S,'OUT') && isstruct(S.OUT) && isscalar(S.OUT)
        R=S.OUT;
    end
catch
    % Input absent or mismatched is not an acceptance.
end
end

function E=af7r49_return_track(R48,R47)
E=struct();
E.status='NO_LIVE_TERMINAL_DATA__ALGEBRA_ONLY';
E.clock_catchup_forbidden=true;
E.delay_preserving_required=true;
E.free_return_duration_required=true;
E.C7_required=true;
E.heading_C6_required=true;
E.full_body_certificate_required=true;
E.produces_committable_candidate=false;
E.flight_rejoin_proved=false;
E.flown_candidate=false;
E.bezier_degree=15;
E.phase_polynomial_degree=16;
E.hold_starts_after_flight=NaN;
E.old_live_terminal_offset_m=nan(3,1);
E.source_live_rejoin_time_s=Inf;

% Exact *source-measured* 3-D held correction, not an invented offset.
if isstruct(R47) && isfield(R47,'return') && isstruct(R47.return)
    D=R47.return;
    if isfield(D,'live_terminal_left_limit_m') && ...
            isnumeric(D.live_terminal_left_limit_m) && numel(D.live_terminal_left_limit_m)==3
        E.old_live_terminal_offset_m=double(D.live_terminal_left_limit_m(:));
    end
    if isfield(D,'flight_end_s') && isfield(D,'live_terminal_hold_start_s')
        E.flight_end_s=double(D.flight_end_s);
        E.hold_start_s=double(D.live_terminal_hold_start_s);
        E.hold_starts_after_flight=E.hold_start_s>E.flight_end_s;
    end
    if isfield(D,'live_rejoin_time_s')
        E.source_live_rejoin_time_s=double(D.live_rejoin_time_s);
    end
end
if isstruct(R48) && isfield(R48,'return') && isstruct(R48.return)
    E.source_return_status=char(R48.return.status);
end

% Degree-15 Bernstein smoother: CP(0..7)=0, CP(8..15)=1.
% Its derivatives 1..7 vanish at both endpoints. The complement
% 1-S7(u) never changes sign or overshoots the held correction.
S=[zeros(1,8) ones(1,8)];
E.unit_smoother_control_points=S;
E.taper_control_points=[];
if all(isfinite(E.old_live_terminal_offset_m))
    E.taper_control_points=E.old_live_terminal_offset_m*(1-S);
    [J0,J1]=af7r49_endjets(E.taper_control_points,1,7);
    E.held_offset_taper_C7_start_error=max(abs(J0(:,2:end)),[],'all');
    E.held_offset_taper_C7_end_error=max(abs(J1),[],'all');
    E.held_offset_taper_zero_end=norm(J1(:,1));
    E.hold_constant_offset_norm=norm(E.old_live_terminal_offset_m);
    E.no_reverse_by_control_points=all(1-S>=0 & 1-S<=1);
    E.taper_is_only_valid_when_start_jet_1_to_7_zero=true;
    E.taper_has_free_positive_duration=true; % T not selected or hard-capped.
else
    E.held_offset_taper_C7_start_error=NaN;
    E.held_offset_taper_C7_end_error=NaN;
    E.held_offset_taper_zero_end=NaN;
    E.no_reverse_by_control_points=false;
end

% Exact dimensionless *algebraic* phase speed law for generic v0 in [0,1]:
% d(tau)/dt = v0 + (1-v0) S7(u), u=(t-a)/T, T>0.
% Integral degree-16 Bernstein CP, from derivative relation:
%   F_j = max(j-8,0)/16, j=0..16; F(1)=1/2.
% Phase duration T is genuinely free, NOT the 4-s trigger horizon.
F=max((0:16)-8,0)/16;
E.phase_integral_control_points=F;
E.phase_integral_terminal=F(end);
E.phase_delay_formula='Delta_final = Delta_initial + (1-v_initial)*T/2, T>0';
E.phase_speed_certificate_symbolic='v_initial <= tau_dot <= 1 if 0<=v_initial<=1';
E.phase_no_catchup_symbolic=true;
E.phase_jet_assumption='entry phase derivatives 2..7 = 0 (otherwise general Hermite reconstruction required)';
E.phase_demo_dimensionless_only=true;
E.phase_demo_v0=0.5; % dimensionless algebraic test, NOT runtime parameter.
E.phase_demo_T=1;    % normalized unit test, NOT flight time.
Pc=E.phase_demo_T*(E.phase_demo_v0*(0:16)/16 + ...
    (1-E.phase_demo_v0)*F);
[J0,J1]=af7r49_endjets(Pc,E.phase_demo_T,7);
E.phase_demo_start_jet=J0;
E.phase_demo_end_jet=J1;
E.phase_demo_start_rate_error=abs(J0(1,2)-E.phase_demo_v0);
E.phase_demo_end_rate_error=abs(J1(1,2)-1);
E.phase_demo_high_order_jet_max=max(abs([J0(1,3:end) J1(1,3:end)]));
Vcp=16*diff(Pc)/E.phase_demo_T;
E.phase_demo_speed_cp_min_max=[min(Vcp),max(Vcp)];
E.phase_demo_final_delay=(1-E.phase_demo_v0)*E.phase_demo_T/2;
E.phase_demo_is_valid= E.phase_demo_start_rate_error<1e-10 && ...
    E.phase_demo_end_rate_error<1e-10 && ...
    E.phase_demo_high_order_jet_max<1e-8 && ...
    all(Vcp>=-1e-12 & Vcp<=1+1e-12);
E.phase_proof_status='DIMENSIONLESS_PHASE_FORMULA_CHECKED__NOT_SOURCE_C7_OR_YAW_PROOF';

if all(isfinite(E.old_live_terminal_offset_m))
    E.status='CONSTRUCTED_MONOTONE_C7_HELD_TAIL__POST_FLIGHT_NOT_LIVE_REJOIN';
    if isequal(E.hold_starts_after_flight,false)
        E.status='CONSTRUCTED_MONOTONE_C7_HELD_TAIL__UNCOMMITTED_AND_UNCERTIFIED';
    end
end
E.remaining_obligations={ ...
    'Use actual at-FLIGHT splice jets 0..7 from active R44R2/R52 source, not held-only jets', ...
    'Select positive terminal time by joint whole-body/towing feasibility, no arbitrary cap', ...
    'Require nonincreasing signed detour with zero overshoot on all intervals', ...
    'C7 total path and true path-tangent yaw C6 including zero-speed treatment', ...
    'Restore delay-preserved phase, no same-clock nominal catch-up', ...
    'Whole continuous-time UAV/cable/payload ellipsoid and tracking-tube proof', ...
    'Flight-to-Landing authority and post-flight continuation must be consistent'};
end

function [L,R]=af7r49_endjets(C,T,kmax)
% Bézier physical-time endpoint derivative jets: exact finite differences.
assert(isnumeric(C) && ismatrix(C) && T>0 && isfinite(T));
C=double(C);
kmax=min(kmax,size(C,2)-1);
L=zeros(size(C,1),kmax+1);
R=L;
for k=0:kmax
    L(:,k+1)=C(:,1);
    R(:,k+1)=C(:,end);
    if k<kmax
        C=(size(C,2)-1)/T*diff(C,1,2);
    end
end
end

function E=af7r49_timing_track(R48,R47,source)
E=struct('status','NO_R48R2_TIMING_RECORD');
E.controller_24_95_spike_excluded=true;
E.replanner_10ms_certified=false;
E.full_25ms_certified=false;
E.stage1_internal_ledger_closed=false;
E.same_source_pairing_verified=false;
E.packet_vs_native_production_speedup_proved=false;
E.open_exclusive_wall_bucket_ms=NaN;
E.source_callsites=struct();
if isstruct(R48) && isfield(R48,'timing') && isstruct(R48.timing)
    D=R48.timing;
    if isfield(D,'replanner_ranked_top_rows')
        E.replanner_ranked_top_rows=D.replanner_ranked_top_rows;
        E.replanner_ranked_column_labels=D.replanner_ranked_column_labels;
    end
    if isfield(D,'reference_p50_p95_max_ms')
        E.reference_p50_p95_max_ms=double(D.reference_p50_p95_max_ms);
    end
    req={'stage1_times_s','stage1_native_ms', ...
        'stage1_reference_same_time_ms'};
    if all(isfield(D,req))
        E.stage1_times_s=double(D.stage1_times_s(:));
        E.stage1_native_ms=double(D.stage1_native_ms(:));
        E.stage1_reference_same_time_ms=double(D.stage1_reference_same_time_ms(:));
        E.unattributed_same_cycle_difference_ms= ...
            E.stage1_reference_same_time_ms-E.stage1_native_ms;
        E.open_exclusive_wall_bucket_ms=max(E.unattributed_same_cycle_difference_ms);
        E.status='MEASURED_REPLANNER_EXCESS__INTERNAL_EXCLUSIVE_TIMERS_REQUIRED';
    end
end

% Source inventory distinguishes one current packet and one future activation
% packet. They have different times and may NOT be blindly de-duplicated.
if isfile(source)
    txt=fileread(source);
    lines=regexp(txt,'\r\n|\r|\n','split');
    keys={ ...
        'current_future_packet','obj.get_active_reference_packet(time,cha,varargin,xd_nom)'; ...
        'stage1_future_activation_packet','obj.staged_reference_packet_at(ta,varargin_in)'; ...
        'stage1_activation_reference','obj.get_active_reference_packet('; ...
        'stage1_native_proposal','obj.af7r46_single_core_proposal_compact(CTX)'; ...
        'native_mex','af7r36_single_core_mex('; ...
        'stage2_bundle','stage2_bundle_ms'; ...
        'stage1_latency','obj.staged_replan_last_stage1_ms=stage_ms'; ...
        'stage2_latency','obj.staged_replan_last_stage2_ms='};
    for k=1:size(keys,1)
        E.source_callsites.(keys{k,1})=af7r49_line_find(lines,keys{k,2});
    end
    E.source_sourcefile=source;
    dsrc=dir(source);
    E.source_size_bytes=dsrc.bytes;
    E.future_packet_not_assumed_duplicate=true;
    E.source_has_current_future_packet=~isempty( ...
        E.source_callsites.current_future_packet);
    E.source_has_stage1_activation_packet=~isempty( ...
        E.source_callsites.stage1_future_activation_packet);
end
E.next_speedup_target={ ...
    'Same Flight-cycle QPC: reference total / packet current / trigger / Stage1 activation packet / native wrapper / MEX internal', ...
    'Stage2 certificate and bundle + Stage3 pre/postcommit separate wall timings', ...
    'Investigate repeated version-bound packet generation only if activation times and sources permit mathematically equivalent reuse', ...
    'Do not turn off any hard safety certificate or change current active reference semantics', ...
    'Exclude controller 24.95 s spike from REPLANNER speedup attribution'};
if isstruct(R47) && isfield(R47,'source_pairing_verified')
    E.same_source_pairing_verified=logical(R47.source_pairing_verified);
end
end

function E=af7r49_trigger_track(R48,source)
E=struct('status','SOURCE_NOT_FOUND');
E.reference_only_prediction_is_not_actual_state_prediction=true;
E.sensor_from_current_ekf=false;
E.trigger_from_reference=false;
E.stage1_start_from_reference=false;
E.actual_state_closed_loop_certificate=false;
E.production_trigger_unchanged=true;
E.source_matches_R48R2=false;
E.source_lines=struct();
if ~isfile(source),return;end
s=fileread(source);
lines=regexp(s,'\r\n|\r|\n','split');
checks={ ...
    'ekf_position','pQ_cur=est_state.p(:)'; ...
    'ekf_cable','q_cur=est_state.pT(:)'; ...
    'sensor_from_current_position','obj.raw_sensor_gate(pQ_cur,obj.static_obstacle_map)'; ...
    'future_reference_packet','obj.get_active_reference_packet(time,cha,varargin,xd_nom)'; ...
    'reference_collision_call','obj.trigger_core(active,pL_ref,q_ref,time.t)'; ...
    'stage1_activation_future','obj.staged_reference_packet_at(ta,varargin_in)'; ...
    'stage1_reference_position','pL_a=double(pL_future(:,1))'; ...
    'stage1_reference_cable','q_a=double(q_future(:,1))'; ...
    'obstacle_trigger_kernel','g26_future_trigger_kernel_mex('};
for i=1:size(checks,1)
    E.source_lines.(checks{i,1})=af7r49_line_find(lines,checks{i,2});
end
E.sensor_from_current_ekf=~isempty(E.source_lines.ekf_position) && ...
    ~isempty(E.source_lines.sensor_from_current_position);
E.trigger_from_reference=~isempty(E.source_lines.reference_collision_call);
E.stage1_start_from_reference=~isempty(E.source_lines.stage1_reference_position) && ...
    ~isempty(E.source_lines.stage1_reference_cable);
E.source_matches_R48R2=E.sensor_from_current_ekf && ...
    E.trigger_from_reference && E.stage1_start_from_reference;
E.status='SOURCE_CHANGED__INSPECT_BEFORE_UPDATE';
if E.source_matches_R48R2
    E.status='CONFIRMED__EKF_SENSES_BUT_H4_TRIGGER_USES_ACTIVE_REFERENCE';
end
E.current_state_variables_to_bind={ ...
    'EKF estimated pQ,pL,q,qdot and velocity', ...
    'Latest estimator mL at execution time', ...
    'Live HLC and actuator bounds with actual plant dynamics', ...
    'Versioned repaired reference supplied as command trajectory', ...
    'Full-state reachable tube for UAV/cable/payload over [t,t+H]', ...
    'Sensor-known ellipsoids only, retained until certified release'};
E.required_logic='Predict from EKF CURRENT state, not by assigning EKF pL into pL_ref(:,1).';
E.reference_error_tube_math= ...
    ['For continuous t, ||p_actual(t)-p_ref(t)|| <= E(t) must follow ' ...
     'from certified HLC/observer/plant dynamics; E(0)=initial position ' ...
     'mismatch alone is insufficient to bound E(t>0).'];
E.actual_state_shadow_run=false;
E.closed_loop_model_missing='No live EKF/HLC/plant object and certified tube passed to read-only R49 input.';
E.bridge_requirements={ ...
    'Reference packet unchanged for HLC commands; estimate anchors prediction initial state', ...
    'Use only detected/retained obstacles, H=4s, no premature obstacle-specific planning', ...
    'No sample-only positive safety conclusion', ...
    'Tracking tube toggle separate from failsafe toggle; default ON', ...
    'No production activation unless exact full continuous safety proof and C7 output are closed'};
end

function ix=af7r49_line_find(lines,needle)
ix=[];
want=regexprep(needle,'\s+','');
for k=1:numel(lines)
    a=regexprep(lines{k},'\s+','');
    if contains(a,want),ix(end+1)=k;end %#ok<AGROW>
end
end

function X=af7r49_failed(label,ME)
X=struct('status',label,'error_id',char(ME.identifier), ...
    'error_message',char(ME.message));
end

function af7r49_write_report(OUT)
fid=fopen(OUT.report_file,'wt');
assert(fid>0,'[R49] Could not open report file.');
c=onCleanup(@() fclose(fid)); %#ok<NASGU>
fprintf(fid,'AF7R58R49 THREE PARALLEL TRACKS | SOURCE-BOUND SHADOW ONLY\n');
fprintf(fid,'production_modified=0 | commit_changed=0 | trigger_changed=0\n');
fprintf(fid,'controller_24_95_spike_in_scope=0\n');
fprintf(fid,'continuous_safety_proved=0 | executed_rejoin_proved=0 | 25ms_WCET_proved=0\n');
fprintf(fid,'R47=%s\nR48=%s\nSOURCE=%s\n\n', ...
    OUT.sources.r47_file,OUT.sources.r48_file,OUT.sources.production_source);
A=OUT.return;
fprintf(fid,'[1] C7 RETURN / NATURAL PHASE\nstatus=%s\n',A.status);
if isfield(A,'old_live_terminal_offset_m')
    fprintf(fid,'original_live_held_offset_m=[%.12g %.12g %.12g]\n', ...
        A.old_live_terminal_offset_m);
end
if isfield(A,'hold_starts_after_flight')
    fprintf(fid,'hold_after_flight=%g\n',A.hold_starts_after_flight);
end
if isfield(A,'phase_demo_is_valid')
    fprintf(fid,'dimensionless_phase_demo_valid=%d\n',A.phase_demo_is_valid);
    fprintf(fid,'phase_rate_CP_range=[%.12g %.12g] (unit test only)\n', ...
        A.phase_demo_speed_cp_min_max);
    fprintf(fid,'no_catchup=1 | Delta_final=Delta_start+(1-v0)*T/2\n');
end
if isfield(A,'held_offset_taper_C7_start_error')
    fprintf(fid,'held_taper_start_C7_r1_7=%.12g | end_C7_r0_7=%.12g\n', ...
        A.held_offset_taper_C7_start_error,A.held_offset_taper_C7_end_error);
end
fprintf(fid,'CAVEAT: held-tail extension after 49 s is NOT an in-Flight rejoin (Flight ends at 47 s).\n');
fprintf(fid,'Next: from actual pre-landing C7 jets, optimize free T + delay and recertify dynamics and whole-body.\n\n');

B=OUT.timing;
fprintf(fid,'[2] REPLANNER-ONLY TIMING\nstatus=%s\n',B.status);
if isfield(B,'stage1_times_s')
    fprintf(fid,'time[s],stage1_native[ms],replanner_reference[ms],difference[ms] (not exclusive)\n');
    for k=1:numel(B.stage1_times_s)
        fprintf(fid,'%.9f %.6f %.6f %.6f\n',B.stage1_times_s(k), ...
            B.stage1_native_ms(k),B.stage1_reference_same_time_ms(k), ...
            B.unattributed_same_cycle_difference_ms(k));
    end
end
if isfield(B,'source_callsites')
    ns=fieldnames(B.source_callsites);
    for i=1:numel(ns)
        fprintf(fid,'source %s at lines: ',ns{i});
        fprintf(fid,'%d ',B.source_callsites.(ns{i}));
        fprintf(fid,'\n');
    end
end
fprintf(fid,'No production speedup claimed. Same-cycle exclusive QPC buckets still needed.\n\n');

C=OUT.trigger;
fprintf(fid,'[3] ACTUAL-STATE TRIGGER\nstatus=%s\n',C.status);
if isfield(C,'source_lines')
    ns=fieldnames(C.source_lines);
    for i=1:numel(ns)
        fprintf(fid,'source %s at lines: ',ns{i});
        fprintf(fid,'%d ',C.source_lines.(ns{i}));
        fprintf(fid,'\n');
    end
end
fprintf(fid,'Current EKF sensing != EKF-initialized HLC/plant prediction.\n');
fprintf(fid,'Do NOT overwrite only future packet first sample. Require full-state reachability tube.\n');
fprintf(fid,'No live plant closed-loop execution or continuous-time proof in R49.\n\n');
fprintf(fid,'[BASE WORKSPACE AUTHORITY INVENTORY]\n');
for i=1:numel(OUT.workspace_schema)
    w=OUT.workspace_schema(i);
    fprintf(fid,'%s | %s | size=%s | fields=',w.name,w.class,mat2str(w.size));
    if ~isempty(w.fields),fprintf(fid,'%s ',w.fields{:});end
    fprintf(fid,'\n');
end
fprintf(fid,'\nOUT=%s\n',OUT.mat_file);
end
