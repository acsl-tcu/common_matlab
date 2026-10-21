function R41 = AF7R41_NATIVE_RING_SPEED_SPLIT_AUDIT()
% AF7R41_NATIVE_RING_SPEED_SPLIT_AUDIT
% DIAGNOSTIC ONLY.
%
% Reads the diagnostic rings that already exist inside the CURRENT
% af7r36_single_core_mex. No production source is edited and no MEX is rebuilt.
%
% Goal:
%   Separate two independent blockers:
%     A) legacy floating exact-C7 rejection before Stage-A
%     B) genuine/inconclusive finite-sensing speed rejection of candidates
%        that are already constructively exact C7
%
% Existing native diagnostic opcodes used:
%   6  : TOWMAP matrix
%   7  : clear TOWMAP
%   8  : C7 frontier matrix
%   9  : clear C7 frontier
%   10 : R7 shadow matrix
%   11 : clear R7 shadow
%   12 : primary-heading/R7 map matrix
%   13 : clear primary-heading/R7 map
%
% Safety:
%   production modified                  = 0
%   MEX rebuilt                          = 0
%   acceptance semantics modified        = 0
%   C7 / yaw-C6 requirement modified     = 0
%   finite-sensing theorem modified      = 0
%   sampled acceptance introduced        = 0
%
% Outputs are saved beside THIS file:
%   AF7R41_NATIVE_RING_SPEED_SPLIT_REPORT.txt
%   AF7R41_NATIVE_RING_SPEED_SPLIT_CAPTURE.mat
%
% Run:
%   R41 = AF7R41_NATIVE_RING_SPEED_SPLIT_AUDIT;

fprintf('\n');
fprintf('====================================================================================================\n');
fprintf(' AF7R41 NATIVE-RING SPEED SPLIT AUDIT (DIAGNOSTIC ONLY)\n');
fprintf(' production modified = 0 | MEX rebuilt = 0 | safety semantics modified = 0\n');
fprintf('====================================================================================================\n');

R41 = struct();
R41.audit_name = 'AF7R41_NATIVE_RING_SPEED_SPLIT_AUDIT';
R41.timestamp = char(datetime('now','Format','yyyy-MM-dd HH:mm:ss.SSS'));
R41.production_modified = false;
R41.mex_rebuilt = false;
R41.acceptance_modified = false;
R41.certificate_semantics_modified = false;

auditDir = fileparts(mfilename('fullpath'));
if isempty(auditDir), auditDir = pwd; end
reportFile  = fullfile(auditDir,'AF7R41_NATIVE_RING_SPEED_SPLIT_REPORT.txt');
captureFile = fullfile(auditDir,'AF7R41_NATIVE_RING_SPEED_SPLIT_CAPTURE.mat');
R41.audit_directory = auditDir;
R41.report_file = reportFile;
R41.capture_file = captureFile;

%% A. Source / binary binding
mexPath = which('af7r36_single_core_mex');
replanPath = which('REPLANNING_SYSTEM');
R41.mex_path = mexPath;
R41.replanning_system_path = replanPath;
R41.mex_exists = exist('af7r36_single_core_mex','file') == 3;
R41.replanning_exists = ~isempty(replanPath) && isfile(replanPath);

if ~R41.mex_exists
    error('AF7R41:MissingMex','Current af7r36_single_core_mex is not resolved as MEX.');
end

if ~isempty(mexPath) && isfile(mexPath)
    R41.mex_sha256 = local_sha256(mexPath);
else
    R41.mex_sha256 = '';
end
if R41.replanning_exists
    R41.replanning_sha256 = local_sha256(replanPath);
else
    R41.replanning_sha256 = '';
end

%% B. Verify diagnostic opcodes without modifying planner semantics.
% Clear only diagnostic rings.
ops = [7 9 11 13];
R41.clear_ops_ok = false(size(ops));
for k=1:numel(ops)
    try
        af7r36_single_core_mex(ops(k));
        R41.clear_ops_ok(k)=true;
    catch ME
        R41.clear_ops_error{k,1}=getReport(ME,'extended','hyperlinks','off'); %#ok<AGROW>
    end
end

if ~all(R41.clear_ops_ok)
    R41.classification='CURRENT_MEX_DIAGNOSTIC_OPS_UNAVAILABLE';
    local_write_report(reportFile,R41);
    save(captureFile,'R41','-v7.3');
    fprintf('[R41] Required existing diagnostic opcodes are unavailable. No production changes made.\n');
    return;
end

%% C. Run the same R38 diagnostic once.
try
    D38 = AF7R38_SPEED_ROOTCAUSE_AUDIT;
    R41.r38_error='';
catch ME
    D38=struct();
    R41.r38_error=getReport(ME,'extended','hyperlinks','off');
end
R41.D38=D38;

%% D. Read existing native rings after R38.
[R41.towmap,R41.towmap_error] = local_read_op(6);
[R41.c7front,R41.c7front_error] = local_read_op(8);
[R41.r7shadow,R41.r7shadow_error] = local_read_op(10);
[R41.phr9,R41.phr9_error] = local_read_op(12);

R41.columns = struct();
R41.columns.towmap = { ...
    'seq','t0','tm','t1','limit','success','reason','family', ...
    'envelope','VQ','VL','A','J','sigma','qdot','total_ms', ...
    'primary_candidates','primary_speed','primary_rejoin','primary_geom', ...
    'towing_candidates','towing_speed','towing_rejoin','towing_geom', ...
    'fallback_candidates','fallback_speed','fallback_rejoin','fallback_geom', ...
    'primary_witness_count','primary_witness_min_vq','primary_witness_min_t', ...
    'primary_witness_min_seg','primary_witness_min_class', ...
    'primary_witness_max_vq','primary_witness_max_t','primary_witness_max_seg','primary_witness_max_class', ...
    'towing_witness_count','towing_witness_min_vq','towing_witness_min_t', ...
    'towing_witness_min_seg','towing_witness_min_class', ...
    'towing_witness_max_vq','towing_witness_max_t','towing_witness_max_seg','towing_witness_max_class'};

R41.columns.c7front = { ...
    'seq','t0','tm','t1','alpha','sign_code','generated','speed','rejoin', ...
    'geometry','heading','c7','envelope','VL','A','J','sigma','qdot','VQ', ...
    'heading_lb','heading_splits'};

R41.columns.r7shadow = { ...
    'seq','t0','tm','t1','limit','success','reason','candidate_count', ...
    'speed_pass','rejoin_pass','geom_pass','heading_pass','envelope','VQ', ...
    'VL','A','J','sigma','qdot','total_ms','witness_count','witness_min_vq', ...
    'witness_min_t','witness_min_seg','witness_min_class','witness_max_vq', ...
    'witness_max_t','witness_max_seg','witness_max_class'};

R41.columns.phr9 = { ...
    'seq','t0','tm','t1','limit','prod_ok','prod_family','prod_heading', ...
    'prod_env','prod_vq','prod_qdot','r7_ok','r7_reason','r7_candidate_count', ...
    'r7_speed','r7_rejoin','r7_geom','r7_heading','r7_env','r7_vq','r7_qdot', ...
    'witness_count','witness_min_vq','witness_min_t','witness_min_seg', ...
    'witness_min_class','witness_max_vq','witness_max_t','witness_max_seg','witness_max_class'};

%% E. Pull exact scalar/array facts from D38.
A = struct();
A.available = isstruct(D38) && isfield(D38,'pipeline');

if A.available
    P=D38.pipeline;
    A.r37_side_assignments = local_f(P,'r37_side_assignments',NaN);
    A.r37_exact_c7_candidates = local_f(P,'r37_exact_c7_candidates',NaN);
    A.r37_heading_pass_count = local_f(P,'r37_heading_pass_count',NaN);
    A.r37_best_total_velocity_ub = local_f(P,'r37_best_total_velocity_ub',NaN);
    A.r37_best_accel_ub = local_f(P,'r37_best_accel_ub',NaN);
    A.r37_best_jerk_ub = local_f(P,'r37_best_jerk_ub',NaN);

    A.r38_constructive_c7_count = local_f(P,'r38_constructive_c7_count',NaN);
    A.r38_legacy_float_checker_pass_count = local_f(P,'r38_legacy_float_checker_pass_count',NaN);
    A.r38_legacy_float_false_reject_count = local_f(P,'r38_legacy_float_false_reject_count',NaN);

    A.family_seen = local_row(P,'r24e_family_seen');
    A.family_exact = local_row(P,'r24e_family_exact');
    A.family_exact_reject = local_row(P,'r24e_family_exact_reject');
    A.family_speed_pass = local_row(P,'r24e_family_speed_pass');
    A.family_speed_fail = local_row(P,'r24e_family_speed_fail');
    A.split_repr_reject = local_f(P,'r24e_split_representability_reject',NaN);
else
    A.r37_side_assignments=NaN;
    A.r37_exact_c7_candidates=NaN;
    A.r37_heading_pass_count=NaN;
    A.r37_best_total_velocity_ub=NaN;
    A.r37_best_accel_ub=NaN;
    A.r37_best_jerk_ub=NaN;
    A.r38_constructive_c7_count=NaN;
    A.r38_legacy_float_checker_pass_count=NaN;
    A.r38_legacy_float_false_reject_count=NaN;
    A.family_seen=[];
    A.family_exact=[];
    A.family_exact_reject=[];
    A.family_speed_pass=[];
    A.family_speed_fail=[];
    A.split_repr_reject=NaN;
end

if isstruct(D38) && isfield(D38,'generator')
    G=D38.generator;
    A.limit = local_f(G,'finite_sensing_limit_m_s',NaN);
    A.primary_candidate_count = local_f(G,'primary_candidate_count',NaN);
    A.primary_speed_pass_count = local_f(G,'primary_speed_pass_count',NaN);
    A.primary_stageA_certified_count = local_f(G,'primary_stageA_certified_count',NaN);
    A.primary_stageB_certified_count = local_f(G,'primary_stageB_certified_count',NaN);
    A.primary_stageB_finite_witness_count = local_f(G,'primary_stageB_finite_witness_count',NaN);
    A.primary_stageB_repr_fail_count = local_f(G,'primary_stageB_repr_fail_count',NaN);
    A.primary_min_stageA_vq = local_f(G,'primary_min_stageA_uav_speed_UB_m_s',NaN);
    A.primary_min_stageA_sigma = local_f(G,'primary_min_stageA_specific_force_LB_m_s2',NaN);
    A.primary_U0 = local_f(G,'primary_min_stageA_correction_speed_UB_m_s',NaN);
    A.primary_U1 = local_f(G,'primary_min_stageA_correction_accel_UB_m_s2',NaN);
    A.primary_U2 = local_f(G,'primary_min_stageA_correction_jerk_UB_m_s3',NaN);
else
    A.limit=NaN;
    A.primary_candidate_count=NaN;
    A.primary_speed_pass_count=NaN;
    A.primary_stageA_certified_count=NaN;
    A.primary_stageB_certified_count=NaN;
    A.primary_stageB_finite_witness_count=NaN;
    A.primary_stageB_repr_fail_count=NaN;
    A.primary_min_stageA_vq=NaN;
    A.primary_min_stageA_sigma=NaN;
    A.primary_U0=NaN; A.primary_U1=NaN; A.primary_U2=NaN;
end
R41.facts=A;

%% F. Native-ring summaries.
S=struct();

% R7 rows: exact constructive shadow candidate path.
S.r7_rows=size(R41.r7shadow,1);
if S.r7_rows>0 && size(R41.r7shadow,2)>=29
    M=R41.r7shadow;
    S.r7_candidate_positive = sum(M(:,8)>0);
    S.r7_speed_zero = sum(M(:,8)>0 & M(:,9)==0);
    S.r7_reason24 = sum(M(:,7)==24);
    S.r7_finite_witness_rows = sum(M(:,21)>0);
    S.r7_max_candidate_count = max(M(:,8));
    S.r7_max_speed_pass = max(M(:,9));
else
    S.r7_candidate_positive=0; S.r7_speed_zero=0; S.r7_reason24=0;
    S.r7_finite_witness_rows=0; S.r7_max_candidate_count=NaN; S.r7_max_speed_pass=NaN;
end

% C7 frontier rows.
S.c7front_rows=size(R41.c7front,1);
if S.c7front_rows>0 && size(R41.c7front,2)>=21
    M=R41.c7front;
    S.c7_generated=sum(M(:,7)>0);
    S.c7_speed_pass=sum(M(:,8)>0);
    S.c7_rejoin_pass=sum(M(:,9)>0);
    S.c7_geometry_pass=sum(M(:,10)>0);
    S.c7_heading_pass=sum(M(:,11)>0);
    S.c7_finite_vq=sum(isfinite(M(:,19)));
    S.c7_vq_over_limit=sum(isfinite(M(:,19)) & isfinite(M(:,1)*0+A.limit) & M(:,19)>=A.limit);
else
    S.c7_generated=0; S.c7_speed_pass=0; S.c7_rejoin_pass=0;
    S.c7_geometry_pass=0; S.c7_heading_pass=0; S.c7_finite_vq=0; S.c7_vq_over_limit=0;
end

% Production/towing map witnesses.
S.towmap_rows=size(R41.towmap,1);
if S.towmap_rows>0 && size(R41.towmap,2)>=46
    M=R41.towmap;
    S.primary_witness_total=sum(M(:,29),'omitnan');
    S.towing_witness_total=sum(M(:,38),'omitnan');
    S.any_prod_success=sum(M(:,6)>0);
else
    S.primary_witness_total=NaN; S.towing_witness_total=NaN; S.any_prod_success=0;
end

% PHR9 rows.
S.phr9_rows=size(R41.phr9,1);
if S.phr9_rows>0 && size(R41.phr9,2)>=30
    M=R41.phr9;
    S.phr9_r7_candidate_positive=sum(M(:,14)>0);
    S.phr9_r7_speed_zero=sum(M(:,14)>0 & M(:,15)==0);
    S.phr9_r7_reason24=sum(M(:,13)==24);
else
    S.phr9_r7_candidate_positive=0; S.phr9_r7_speed_zero=0; S.phr9_r7_reason24=0;
end

R41.summary=S;

%% G. Conservative classification.
legacyGate=false;
if isfinite(A.r38_constructive_c7_count) && A.r38_constructive_c7_count>0 && ...
        isfinite(A.r38_legacy_float_false_reject_count) && ...
        A.r38_legacy_float_false_reject_count==A.r38_constructive_c7_count && ...
        A.r38_legacy_float_checker_pass_count==0
    legacyGate=true;
end

% family vector element k+1 corresponds native family index k.
family1PreSpeedReject=false;
if numel(A.family_seen)>=2 && numel(A.family_exact_reject)>=2
    family1PreSpeedReject = A.family_seen(2)>0 && ...
        A.family_exact_reject(2)==A.family_seen(2) && ...
        A.family_speed_fail(2)==0;
end

exactFamiliesReachSpeed=false;
if ~isempty(A.family_exact) && ~isempty(A.family_speed_fail)
    exactFamiliesReachSpeed = any(A.family_exact>0 & A.family_speed_fail>0);
end

r7HardSpeedBlock = (S.r7_candidate_positive>0 && S.r7_speed_zero>0) || ...
                   (S.phr9_r7_candidate_positive>0 && S.phr9_r7_speed_zero>0);

finiteWitness = false;
if isfinite(A.primary_stageB_finite_witness_count) && A.primary_stageB_finite_witness_count>0
    finiteWitness=true;
end
if isfinite(S.primary_witness_total) && S.primary_witness_total>0
    finiteWitness=true;
end
if isfinite(S.towing_witness_total) && S.towing_witness_total>0
    finiteWitness=true;
end
if S.r7_finite_witness_rows>0
    finiteWitness=true;
end

R41.flags = struct( ...
    'legacy_constructive_exact_but_float_rejected',legacyGate, ...
    'family1_rejected_before_speed',family1PreSpeedReject, ...
    'other_exact_families_reach_and_fail_speed',exactFamiliesReachSpeed, ...
    'r7_constructive_path_blocked_at_speed',r7HardSpeedBlock, ...
    'finite_speed_violation_witness_observed',finiteWitness);

if legacyGate && family1PreSpeedReject && exactFamiliesReachSpeed && r7HardSpeedBlock
    R41.classification='TWO_DISTINCT_BLOCKERS: LEGACY_EXACT_C7_GATE + HARD_SPEED_FAILURE';
elseif legacyGate && family1PreSpeedReject
    R41.classification='LEGACY_EXACT_C7_GATE_PREEMPTS_PRIMARY_STAGE_A';
elseif exactFamiliesReachSpeed && r7HardSpeedBlock
    if finiteWitness
        R41.classification='CONSTRUCTIVE_EXACT_C7_REACHES_SPEED_AND_HAS_FINITE_VIOLATION_WITNESS';
    else
        R41.classification='CONSTRUCTIVE_EXACT_C7_REACHES_SPEED_BUT_SPEED_CERT_DOES_NOT_CLOSE';
    end
else
    R41.classification='NATIVE_RING_EVIDENCE_REQUIRES_FURTHER_TRACE';
end

%% H. Save/report.
local_write_report(reportFile,R41);
save(captureFile,'R41','-v7.3');

fprintf('\n[A. ROOT-CAUSE SPLIT]\n');
fprintf('classification                                = %s\n',R41.classification);
fprintf('constructive exact C7 but float rejected      = %d\n',legacyGate);
fprintf('family-1 rejected before speed                = %d\n',family1PreSpeedReject);
fprintf('other exact families reach+fail speed         = %d\n',exactFamiliesReachSpeed);
fprintf('R7 constructive path blocked at speed         = %d\n',r7HardSpeedBlock);
fprintf('finite speed witness observed                 = %d\n',finiteWitness);

fprintf('\n[B. R7 NATIVE SHADOW]\n');
fprintf('rows                                          = %d\n',S.r7_rows);
fprintf('candidate-positive rows                       = %d\n',S.r7_candidate_positive);
fprintf('candidate-positive but speed=0 rows           = %d\n',S.r7_speed_zero);
fprintf('reason 24 rows                                = %d\n',S.r7_reason24);
fprintf('finite witness rows                           = %d\n',S.r7_finite_witness_rows);

fprintf('\n[C. C7 FRONTIER]\n');
fprintf('rows / generated / speed pass                 = %d / %d / %d\n', ...
    S.c7front_rows,S.c7_generated,S.c7_speed_pass);

fprintf('\n[D. OUTPUT]\n');
fprintf('report                                        = %s\n',reportFile);
fprintf('capture                                       = %s\n',captureFile);
fprintf('====================================================================================================\n');

assignin('base','R41',R41);

end

% =================================================================================================
function [M,err]=local_read_op(op)
M=[];
err='';
try
    M=af7r36_single_core_mex(op);
    if ~isnumeric(M)
        err=sprintf('op %d returned nonnumeric class %s',op,class(M));
        M=[];
    else
        M=double(M);
    end
catch ME
    err=getReport(ME,'extended','hyperlinks','off');
end
end

function v=local_f(S,f,d)
if isstruct(S) && isfield(S,f)
    v=double(S.(f));
    if ~isscalar(v), v=d; end
else
    v=d;
end
end

function v=local_row(S,f)
if isstruct(S) && isfield(S,f)
    v=double(S.(f)(:).');
else
    v=[];
end
end

function h=local_sha256(file)
h='';
try
    md=java.security.MessageDigest.getInstance('SHA-256');
    fid=fopen(file,'rb');
    c=onCleanup(@() fclose(fid));
    while true
        b=fread(fid,1024*1024,'*uint8');
        if isempty(b), break; end
        md.update(b);
    end
    d=typecast(md.digest(),'uint8');
    h=lower(reshape(dec2hex(d,2).',1,[]));
catch
end
end

function local_write_report(file,R)
fid=fopen(file,'w');
if fid<0, return; end
c=onCleanup(@() fclose(fid));

fprintf(fid,'AF7R41 NATIVE-RING SPEED SPLIT AUDIT\n');
fprintf(fid,'timestamp=%s\n',R.timestamp);
fprintf(fid,'production_modified=0\n');
fprintf(fid,'mex_rebuilt=0\n');
fprintf(fid,'acceptance_modified=0\n');
fprintf(fid,'certificate_semantics_modified=0\n');
fprintf(fid,'mex_path=%s\n',R.mex_path);
fprintf(fid,'mex_sha256=%s\n',R.mex_sha256);
fprintf(fid,'REPLANNING_SYSTEM=%s\n',R.replanning_system_path);
fprintf(fid,'REPLANNING_SYSTEM_sha256=%s\n',R.replanning_sha256);
fprintf(fid,'classification=%s\n\n',R.classification);

if isfield(R,'flags')
    F=R.flags;
    fprintf(fid,'FLAGS\n');
    fprintf(fid,'legacy_constructive_exact_but_float_rejected=%d\n',F.legacy_constructive_exact_but_float_rejected);
    fprintf(fid,'family1_rejected_before_speed=%d\n',F.family1_rejected_before_speed);
    fprintf(fid,'other_exact_families_reach_and_fail_speed=%d\n',F.other_exact_families_reach_and_fail_speed);
    fprintf(fid,'r7_constructive_path_blocked_at_speed=%d\n',F.r7_constructive_path_blocked_at_speed);
    fprintf(fid,'finite_speed_violation_witness_observed=%d\n\n',F.finite_speed_violation_witness_observed);
end

if isfield(R,'facts')
    A=R.facts;
    fprintf(fid,'D38 FACTS\n');
    fprintf(fid,'r37_side_assignments=%s\n',local_num(A.r37_side_assignments));
    fprintf(fid,'r37_exact_c7_candidates=%s\n',local_num(A.r37_exact_c7_candidates));
    fprintf(fid,'r37_heading_pass_count=%s\n',local_num(A.r37_heading_pass_count));
    fprintf(fid,'r37_best_total_velocity_ub=%s\n',local_num(A.r37_best_total_velocity_ub));
    fprintf(fid,'r37_best_accel_ub=%s\n',local_num(A.r37_best_accel_ub));
    fprintf(fid,'r37_best_jerk_ub=%s\n',local_num(A.r37_best_jerk_ub));
    fprintf(fid,'r38_constructive_c7_count=%s\n',local_num(A.r38_constructive_c7_count));
    fprintf(fid,'r38_legacy_float_checker_pass_count=%s\n',local_num(A.r38_legacy_float_checker_pass_count));
    fprintf(fid,'r38_legacy_float_false_reject_count=%s\n',local_num(A.r38_legacy_float_false_reject_count));
    fprintf(fid,'finite_sensing_limit=%s\n',local_num(A.limit));
    fprintf(fid,'primary_candidate_count=%s\n',local_num(A.primary_candidate_count));
    fprintf(fid,'primary_speed_pass_count=%s\n',local_num(A.primary_speed_pass_count));
    fprintf(fid,'primary_stageA_certified_count=%s\n',local_num(A.primary_stageA_certified_count));
    fprintf(fid,'primary_stageB_certified_count=%s\n',local_num(A.primary_stageB_certified_count));
    fprintf(fid,'primary_stageB_finite_witness_count=%s\n',local_num(A.primary_stageB_finite_witness_count));
    fprintf(fid,'primary_stageB_repr_fail_count=%s\n',local_num(A.primary_stageB_repr_fail_count));
    fprintf(fid,'primary_min_stageA_vq=%s\n',local_num(A.primary_min_stageA_vq));
    fprintf(fid,'primary_min_stageA_sigma=%s\n',local_num(A.primary_min_stageA_sigma));
    fprintf(fid,'primary_U0/U1/U2=%s / %s / %s\n',local_num(A.primary_U0),local_num(A.primary_U1),local_num(A.primary_U2));
    fprintf(fid,'family_seen=%s\n',mat2str(A.family_seen,17));
    fprintf(fid,'family_exact=%s\n',mat2str(A.family_exact,17));
    fprintf(fid,'family_exact_reject=%s\n',mat2str(A.family_exact_reject,17));
    fprintf(fid,'family_speed_pass=%s\n',mat2str(A.family_speed_pass,17));
    fprintf(fid,'family_speed_fail=%s\n\n',mat2str(A.family_speed_fail,17));
end

if isfield(R,'summary')
    S=R.summary;
    fprintf(fid,'NATIVE RING SUMMARY\n');
    fn=fieldnames(S);
    for k=1:numel(fn)
        fprintf(fid,'%s=%s\n',fn{k},local_num(S.(fn{k})));
    end
end

local_matrix(fid,'TOWMAP',R.towmap,R.columns.towmap);
local_matrix(fid,'C7FRONT',R.c7front,R.columns.c7front);
local_matrix(fid,'R7SHADOW',R.r7shadow,R.columns.r7shadow);
local_matrix(fid,'PHR9',R.phr9,R.columns.phr9);

if ~isempty(R.towmap_error), fprintf(fid,'\nTOWMAP_ERROR\n%s\n',R.towmap_error); end
if ~isempty(R.c7front_error), fprintf(fid,'\nC7FRONT_ERROR\n%s\n',R.c7front_error); end
if ~isempty(R.r7shadow_error), fprintf(fid,'\nR7SHADOW_ERROR\n%s\n',R.r7shadow_error); end
if ~isempty(R.phr9_error), fprintf(fid,'\nPHR9_ERROR\n%s\n',R.phr9_error); end
if isfield(R,'r38_error') && ~isempty(R.r38_error), fprintf(fid,'\nR38_ERROR\n%s\n',R.r38_error); end
end

function local_matrix(fid,title,M,cols)
fprintf(fid,'\n====================================================================================================\n');
fprintf(fid,'%s\n',title);
fprintf(fid,'size=%s\n',mat2str(size(M)));
if isempty(M), return; end
fprintf(fid,'columns:\n');
for j=1:min(numel(cols),size(M,2))
    fprintf(fid,' %2d = %s\n',j,cols{j});
end
fprintf(fid,'data:\n');
for i=1:size(M,1)
    fprintf(fid,'row %d: %s\n',i,mat2str(M(i,:),17));
end
end

function s=local_num(x)
if isempty(x), s='[]'; return; end
if isscalar(x)
    s=sprintf('%.17g',double(x));
else
    s=mat2str(x,17);
end
end
