function output=EXPORT_TRAJECTORY_C9_MULTI_REPLAN_RESULT(R,requested_root)
% EXPORT_TRAJECTORY_C9_MULTI_REPLAN_RESULT
% Save the already computed 8-case C9 audit R without rerunning any proof.
% Always store MAT, TXT and ZIP in the actual common_matlab project root,
% independent of pwd and the physical location of the CHECK source file.
%
%   P=EXPORT_TRAJECTORY_C9_MULTI_REPLAN_RESULT(R);
%   disp(P.zip_path); disp(P.zip_verified)
%
% May also be called from CHECK with an explicitly validated project root.
% UNKNOWN cases are NEVER counted as certified. This is an audit export,
% not a flight input and not a safety/torque/HLC tracking certificate.
assert(isstruct(R)&&isscalar(R)&&isfield(R,'status'), ...
    'C9_AUDIT_EXPORT:Input','Pass the actual R struct from the MATLAB check.');
if nargin>=2
    root=char(requested_root);
    assert(known_project_root(root),'C9_AUDIT_EXPORT:Root', ...
        'Requested project root does not contain the required source tree.');
else
    root=find_project_root();
end
base='TRAJECTORY_C9_MULTI_REPLAN_C7_YAW_HLC_RESULT';
matfile=fullfile(root,[base '.mat']);
txtfile=fullfile(root,[base '.txt']);
zipfile=fullfile(root,[base '_FOR_CHATGPT.zip']);
output=struct('status','NOT_EXPORTED','project_root',root, ...
    'mat_path',matfile,'txt_path',txtfile,'zip_path',zipfile, ...
    'zip_verified',false,'zip_bytes',0, ...
    'case_count',0,'number_certified',0,'number_unknown',0, ...
    'declared_case_count',0,'cases_not_executed',0, ...
    'scenarios_executed',false,'audit_complete',false);
% No future file, README, installer or unrelated source is packaged.
save(matfile,'R');
fid=fopen(txtfile,'wt');
if fid<0
    error('C9_AUDIT_EXPORT:Text','Unable to create: %s',txtfile);
end
cleanup=onCleanup(@() fclose(fid));
fprintf(fid,'status=%s\n',char(R.status));
fprintf(fid,'project_root=%s\n',root);
fprintf(fid,'result_scope=GEOMETRY_TAUT_TENSION_THRUST_C7_YAW_AUDIT__HLC_TORQUE_UNCERTIFIED\n');
fprintf(fid,'UNKNOWN_is_not_certified=1\n');
if isfield(R,'hlc_force_limits_N')
    fprintf(fid,'hlc_source_thrust_limits_N=%s\n',mat2str(R.hlc_force_limits_N));
end
if isfield(R,'hlc_force_source')
    fprintf(fid,'hlc_source_file=%s\n',aschar(R.hlc_force_source));
end
if isfield(R,'hlc_input_lower')&&isfield(R,'hlc_input_upper')
    fprintf(fid,'real_HLC_input_lower=%s\n',mat2str(R.hlc_input_lower'));
    fprintf(fid,'real_HLC_input_upper=%s\n',mat2str(R.hlc_input_upper'));
end
if isfield(R,'eight_case_mass_source')
    fprintf(fid,'eight_case_mass_source=%s\n', ...
        aschar(R.eight_case_mass_source));
end
if isfield(R,'production_live_authority_connected_to_runner')
    fprintf(fid,'live_authority_installed_in_Flight_runner=%d\n', ...
        R.production_live_authority_connected_to_runner);
end
if isfield(R,'live_force_authority_audit')&& ...
        isstruct(R.live_force_authority_audit)&& ...
        isfield(R.live_force_authority_audit,'status')
    fprintf(fid,'live_force_authority_audit=%s\n', ...
        aschar(R.live_force_authority_audit.status));
end
if isfield(R,'error_identifier'),fprintf(fid,'error_identifier=%s\n',aschar(R.error_identifier));end
if isfield(R,'error_message'),fprintf(fid,'error_message=%s\n',aschar(R.error_message));end
% A run that failed in a preflight math check has evaluated ZERO cases.
% In particular, 0 evaluated / 0 UNKNOWN must NEVER be presented as a
% measured zero-failure result. A partially evaluated run is also distinct
% from the full eight-fixture benchmark; unexecuted cells are not UNKNOWN.
if isfield(R,'case_status')&&iscell(R.case_status)
    allocated=numel(R.case_status);
else
    allocated=0;
end
if isfield(R,'case_count')&&isnumeric(R.case_count)&& ...
        isscalar(R.case_count)&&isfinite(R.case_count)&& ...
        R.case_count>=0&&R.case_count==fix(R.case_count)
    declared=min(allocated,double(R.case_count));
else
    declared=allocated;
end
completed=[];
for k=1:declared
    if ischar(R.case_status{k})&&~isempty(R.case_status{k})
        completed(end+1)=k; %#ok<AGROW>
    end
end
count=numel(completed);
output.case_count=count;
output.declared_case_count=declared;
output.cases_not_executed=declared-count;
output.scenarios_executed=count>0;
output.audit_complete=(declared>0&&count==declared&& ...
    strncmp(char(R.status),'PASS',4));
fprintf(fid,'declared_case_count=%d\n',declared);
fprintf(fid,'evaluated_case_count=%d\n',count);
fprintf(fid,'unexecuted_case_count=%d\n',output.cases_not_executed);
if count==0
    fprintf(fid,'evaluation_state=NOT_EVALUATED_PRECHECK_STOP\n');
elseif ~output.audit_complete
    fprintf(fid,'evaluation_state=PARTIAL_OR_FAILED_AUDIT\n');
else
    fprintf(fid,'evaluation_state=ALL_DECLARED_CASES_EVALUATED\n');
end
for i=1:count
    k=completed(i);
    success=getnum(R,'case_certified',k,0);
    output.number_certified=output.number_certified+double(success~=0);
    fprintf(fid,'\ncase=%d\n',k);
    fprintf(fid,'  name=%s\n',getcell(R,'case_labels',k,''));
    fprintf(fid,'  status=%s\n',getcell(R,'case_status',k,''));
    fprintf(fid,'  full_time_certified=%d\n',success~=0);
    fprintf(fid,'  obstacle_count=%.0f\n',getnum(R,'case_obstacle_counts',k,NaN));
    fprintf(fid,'  flight_ground_included=1\n');
    fprintf(fid,'  duration_and_obstacles_from_actual_CHECK=1\n');
    fprintf(fid,'  joint_qp_status=%s\n',getcell(R,'case_joint_qp_status',k,'NOT_RECORDED_OLD_CHECK'));
    fprintf(fid,'  continuous_proof_status=%s\n',getcell(R,'case_proof_status',k,'NOT_RECORDED_OLD_CHECK'));
    fprintf(fid,'  failure_class=%s\n',getcell(R,'case_failure_class',k,'NOT_RECORDED_OLD_CHECK'));
    fprintf(fid,'  failure_reason=%s\n',getcell(R,'case_failure_reason',k,'NOT_RECORDED_OLD_CHECK'));
    fprintf(fid,'  heading_certified=%d\n',getnum(R,'case_heading_certified',k,0)~=0);
    fprintf(fid,'  heading_status=%s\n',getcell(R,'case_heading_status',k,''));
    fprintf(fid,'  robust_tube=%d\n',getnum(R,'case_robust_tube',k,0)~=0);
    fprintf(fid,'  fixture_payload_mass_kg=%.9g\n', ...
        getnum(R,'case_payload_mass_kg',k,NaN));
    fprintf(fid,'  certified_thrust_headroom_N=%.9g\n', ...
        getnum(R,'case_certified_thrust_headroom_N',k,NaN));
    fprintf(fid,'  elapsed_ms=%.9g\n',getnum(R,'case_elapsed_ms',k,NaN));
    fprintf(fid,'  proof_ms=%.9g\n',getnum(R,'case_proof_ms',k,NaN));
    fprintf(fid,'  planes_ms=%.9g\n',getnum(R,'case_plane_ms',k,NaN));
    fprintf(fid,'  joint_qp_ms=%.9g\n',getnum(R,'case_qp_ms',k,NaN));
    fprintf(fid,'  proof_calls=%.0f\n',getnum(R,'case_proof_calls',k,NaN));
    fprintf(fid,'  qp_pivots=%.0f\n',getnum(R,'case_qp_pivots',k,NaN));
    fprintf(fid,'  relaxed_probe_attempts=%.0f\n', ...
        getnum(R,'case_relaxed_probe_attempts',k,NaN));
    fprintf(fid,'  relaxed_probe_accepted_directions=%.0f\n', ...
        getnum(R,'case_relaxed_probe_accepted_directions',k,NaN));
    fprintf(fid,'  relaxed_probe_iterations=%.0f\n', ...
        getnum(R,'case_relaxed_probe_iterations',k,NaN));
    fprintf(fid,'  proof_witness_probes=%.0f\n', ...
        getnum(R,'case_proof_witness_probes',k,NaN));
    fprintf(fid,'  screened_qp_working_planes=%.0f\n', ...
        getnum(R,'case_screened_qp_working_planes',k,NaN));
    fprintf(fid,'  screened_qp_generation_passes=%.0f\n', ...
        getnum(R,'case_screened_qp_generation_passes',k,NaN));
    fprintf(fid,'  force_witness_calls=%.0f\n', ...
        getnum(R,'case_force_witness_calls',k,NaN));
    fprintf(fid,'  force_witness_planes=%.0f\n', ...
        getnum(R,'case_force_witness_planes',k,NaN));
    fprintf(fid,'  force_witness_status=%s\n', ...
        getcell(R,'case_force_witness_status',k,'NOT_RECORDED'));
    fprintf(fid,'  force_witness_max_deficit_N=%.9g\n', ...
        getnum(R,'case_force_witness_max_deficit_N',k,NaN));
    fprintf(fid,'  joint_search_iterations=%.0f\n',getnum(R,'case_search_iterations',k,NaN));
    fprintf(fid,'  qp_planes_total=%.0f\n',getnum(R,'case_search_planes',k,NaN));
    fprintf(fid,'  ground_planes_total=%.0f\n',getnum(R,'case_ground_planes',k,NaN));
    fprintf(fid,'  immovable_ground_violations=%.0f\n',getnum(R,'case_immovable_ground_violations',k,NaN));
    fprintf(fid,'  qp_primal_residual_m=%.9g\n',getnum(R,'case_qp_max_primal_violation_m',k,NaN));
    fprintf(fid,'  proof_nodes=%.0f\n',getnum(R,'case_final_proof_nodes',k,NaN));
    fprintf(fid,'  shape_support_calls=%.0f\n',getnum(R,'case_final_support_calls',k,NaN));
    if isfield(R,'case_unresolved_obstacle_ids') && ...
            iscell(R.case_unresolved_obstacle_ids) && ...
            numel(R.case_unresolved_obstacle_ids)>=k
        fprintf(fid,'  unresolved_obstacle_ids=%s\n', ...
            mat2str(R.case_unresolved_obstacle_ids{k}));
    end
    if isfield(R,'case_unresolved_time_interval_s') && ...
            size(R.case_unresolved_time_interval_s,2)>=k && ...
            size(R.case_unresolved_time_interval_s,1)>=2
        fprintf(fid,'  unresolved_time_interval_s=%s\n', ...
            mat2str(R.case_unresolved_time_interval_s(:,k).',12));
    end
end
output.number_unknown=count-output.number_certified;
if isfield(R,'relaxed_qp_math_audit')&&isstruct(R.relaxed_qp_math_audit)&& ...
        isfield(R.relaxed_qp_math_audit,'status')
    fprintf(fid,'relaxed_qp_math_audit_status=%s\n', ...
        char(R.relaxed_qp_math_audit.status));
end
if isfield(R,'screened_joint_qp_math_audit')&& ...
        isfield(R.screened_joint_qp_math_audit,'status')
    fprintf(fid,'screened_qp_math_audit_status=%s\n', ...
        char(R.screened_joint_qp_math_audit.status));
end
if isfield(R,'force_witness_math_audit')&& ...
        isfield(R.force_witness_math_audit,'status')
    fprintf(fid,'force_witness_math_audit_status=%s\n', ...
        char(R.force_witness_math_audit.status));
    fprintf(fid,'force_witness_math_audit_upper_grad_error=%.12g\n', ...
        R.force_witness_math_audit.maximum_whitened_gradient_error_N_per_m);
end
fprintf(fid,'\ncase_count_evaluated=%d\n',count);
if count==0
    fprintf(fid,'number_full_time_certified=NOT_EVALUATED\n');
    fprintf(fid,'number_unknown=NOT_EVALUATED\n');
else
    fprintf(fid,'number_full_time_certified=%d\n',output.number_certified);
    fprintf(fid,'number_unknown=%d\n',output.number_unknown);
end
for field={'number_unknown_qp','number_unknown_proof','number_unknown_other', ...
        'endpoint_necessary_gate_preserves_safe_start', ...
        'endpoint_necessary_gate_rejects_fixed_force_deficit', ...
        'overall_median_ms','overall_p95_ms','overall_max_ms', ...
        'offclock_replan_status','offclock_replan_initial_join_C7_C6', ...
        'offclock_replan_candidate_C7_C6','fixture_tick_log_correct_pass', ...
        'fixture_tick_log_corruption_rejected','real_HLC_tracking_certified', ...
        'real_torque_certified'}
    f=field{1};
    if ~isfield(R,f),continue;end
    v=R.(f);
    if ischar(v),fprintf(fid,'%s=%s\n',f,v);
    elseif (isnumeric(v)||islogical(v))&&isscalar(v)
        fprintf(fid,'%s=%.12g\n',f,double(v));
    end
end
clear cleanup; % Force flush and close BEFORE MATLAB zip reads TXT.
if isfile(zipfile)
    delete(zipfile); % A stale archive must never be mistaken for a new one.
end
try
    extras={};
    f1='TRAJECTORY_C9_LIVE_FORCE_AUTHORITY_RESULT.mat';
    f2='TRAJECTORY_C9_LIVE_FORCE_AUTHORITY_RESULT.txt';
    if isfield(R,'live_force_authority_audit')&& ...
            isstruct(R.live_force_authority_audit)&& ...
            isfield(R.live_force_authority_audit,'status')&& ...
            ~isempty(R.live_force_authority_audit.status) && ...
            isfile(fullfile(root,f1))&&isfile(fullfile(root,f2))
        extras={f1,f2};
    end
    zip(zipfile,[{[base '.mat'],[base '.txt']},extras],root);
catch ME
    error('C9_AUDIT_EXPORT:Zip', ...
        'Could not create %s: %s',zipfile,ME.message);
end
if ~isfile(zipfile)
    error('C9_AUDIT_EXPORT:ZipMissing','No ZIP created at: %s',zipfile);
end
meta=dir(zipfile);
if numel(meta)~=1||meta.bytes<100
    error('C9_AUDIT_EXPORT:ZipEmpty','Result ZIP is empty: %s',zipfile);
end
output.zip_bytes=meta.bytes;
output.zip_verified=true;
output.status='EXPORTED_AND_ZIP_VERIFIED';
if count==0
    fprintf('[C9 AUDIT EXPORT] %s | %d bytes | NO SCENARIOS EXECUTED (math precheck stopped)\n', ...
        zipfile,output.zip_bytes);
else
    fprintf('[C9 AUDIT EXPORT] %s | %d bytes | %d certified / %d UNKNOWN among %d evaluated; %d unexecuted\n', ...
        zipfile,output.zip_bytes,output.number_certified, ...
        output.number_unknown,count,output.cases_not_executed);
end
end

function t=getcell(R,field,k,fallback)
t=fallback;
if isfield(R,field)&&iscell(R.(field))&&numel(R.(field))>=k
    v=R.(field){k};
    if ischar(v)||isstring(v),t=char(v);end
end
end

function n=getnum(R,field,k,fallback)
n=fallback;
if isfield(R,field)&&isnumeric(R.(field))&&numel(R.(field))>=k
    values=R.(field);
    v=values(k);
    if isscalar(v),n=double(v);end
elseif isfield(R,field)&&islogical(R.(field))&&numel(R.(field))>=k
    values=R.(field);
    n=double(values(k));
end
end

function t=aschar(x)
if ischar(x)||isstring(x),t=char(x);
else,t='';end
end

function root=find_project_root()
% Check location first; then current pwd and any validated R.project_root
% supplied by CHECK. Do not assume any particular checked-out directory.
seeds={fileparts(mfilename('fullpath')),pwd};
for s=1:numel(seeds)
    p=seeds{s};
    for i=0:16
        if known_project_root(p),root=p;return;end
        parent=fileparts(p);
        if strcmp(parent,p),break;end
        p=parent;
    end
end
error('C9_AUDIT_EXPORT:Root', ...
    'No common_matlab project root found above exporter or pwd.');
end

function yes=known_project_root(p)
m=fullfile(p,'reference','REPLANNING_SYSTEM');
yes=isfolder(p)&& ...
    isfile(fullfile(m,'TRAJECTORY_GENERATION', ...
        'TRAJECTORY_C9_SIMULTANEOUS_GUIDE_SEARCH.m')) && ...
    isfile(fullfile(m,'OBSTACLE_INFORMATION','OBSTACLE_INFORMATION.m'));
end
