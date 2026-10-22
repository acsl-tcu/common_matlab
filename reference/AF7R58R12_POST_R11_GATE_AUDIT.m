function R = AF7R58R12_POST_R11_GATE_AUDIT(R58A, D38)
% AF7R58R12_POST_R11_GATE_AUDIT
% Read-only, after-run triage for the R58R11 shadow and R38 diagnostic.
% Does NOT re-run the planner/MEX; does NOT modify production, constraints,
% feasible sets, certificates, trajectory, control, or acceptance.
%
% Run in the SAME MATLAB session immediately after:
%   D38  = AF7R38_SPEED_ROOTCAUSE_AUDIT;
%   R58A = AF7R58R11_AUGMENTED_KKT_COMMON_STEP_AUDIT;
%   R12  = AF7R58R12_POST_R11_GATE_AUDIT(R58A, D38);
% Or simply R12 = AF7R58R12_POST_R11_GATE_AUDIT, if both are in base.
% No constants or thresholds are substituted for runtime limits.

if nargin < 1
    assert(evalin('base','exist(''R58A'',''var'')==1'), ...
        'R58A not in base workspace; run R58R11 first.');
    R58A = evalin('base','R58A');
end
if nargin < 2
    if evalin('base','exist(''D38'',''var'')==1')
        D38 = evalin('base','D38');
    else
        D38 = struct(); % A missing prior result remains UNKNOWN, not PASS.
    end
end
assert(isstruct(R58A) && isscalar(R58A), 'R58A must be a scalar result struct.');
assert(isstruct(D38) && isscalar(D38), 'D38 must be a scalar result struct.');

R = struct();
R.name = 'AF7R58R12_POST_R11_GATE_AUDIT';
R.read_only = true;
R.production_modified = false;
R.acceptance_modified = false;
R.certificate_modified = false;
R.R58A = R58A;
R.D38 = D38;
R.source.REPLANNING_SYSTEM = which('REPLANNING_SYSTEM');
R.source.R58R11_audit = which('AF7R58R11_AUGMENTED_KKT_COMMON_STEP_AUDIT');
R.source.R38_audit = which('AF7R38_SPEED_ROOTCAUSE_AUDIT');

% Stage and acceptance are independent: QP closure does not mean safe flight.
qKeys = {'r58_shadow_ran','r58_reference_only', ...
    'r58_bnb_full_assignments','r58_bnb_qp_infeasible', ...
    'r58_bnb_speed_fail','r58_bnb_heading_fail', ...
    'r58_bnb_geometry_fail','r58_bnb_feasible_full', ...
    'r58r7_full_optimality_closed','r58r11_augmented_kkt_solve_fail', ...
    'r58_free_c7_constructive','r58_speed_pass','r58_heading_pass', ...
    'r58_geometry_pass','r58_needs_time_law','r58_commit_attempted', ...
    'r58_rejoin_selected'};
for k = 1:numel(qKeys)
    key = qKeys{k};
    R.counters.(key) = local_scalar(R58A,key);
end

R.qp_both_closed = local_eq(R58A,'r58r7_full_optimality_closed',2) && ...
    local_eq(R58A,'r58_bnb_full_assignments',2) && ...
    local_eq(R58A,'r58_bnb_qp_infeasible',0) && ...
    local_eq(R58A,'r58r11_augmented_kkt_solve_fail',0);
R.both_branches_rejected_at_speed = R.qp_both_closed && ...
    local_eq(R58A,'r58_bnb_speed_fail',2) && ...
    local_eq(R58A,'r58_bnb_feasible_full',0);
R.no_commit = local_eq(R58A,'r58_commit_attempted',0);
R.no_free_C7_candidate = local_eq(R58A,'r58_free_c7_constructive',0);
R.time_law_indicated_not_constructed = ...
    local_eq(R58A,'r58_needs_time_law',1) && R.no_free_C7_candidate;

% These are historical diagnostic measurements. They are not automatically
% proven causes of the CURRENT speed-gate failure or physical infeasibility.
measureKeys = {'r58_quantized_jet_max_abs_error', ...
    'r58r6_prequant_scaled_jet_max_abs', ...
    'r58r6_quantized_scaled_jet_rel_error', ...
    'r58r6_quantized_physical_jet_max_abs_error', ...
    'r58r6_failed_corr_v_ub','r58r6_failed_corr_a_ub', ...
    'r58r6_failed_corr_j_ub','r58r6_failed_speed_witness', ...
    'r58_speed_VL','r58_speed_A','r58_speed_J', ...
    'r58_speed_sigma','r58_speed_qdot','r58_speed_VQ', ...
    'r58_heading_gamma','r58_geometry_min_margin', ...
    'r58r7_final_max_violation','r58r7_final_duality_gap', ...
    'r58r8_ipm_primal_residual','r58r8_ipm_dual_residual', ...
    'r58r11_newton_max_rel_residual', ...
    'r58r11_common_step_min','r58r11_common_step_last'};
for k = 1:numel(measureKeys)
    key = measureKeys{k};
    R.measurements.(key) = local_scalar(R58A,key);
end

% Timing is recorded as observed; no time cap is inferred from this code.
timingKeys = {'r58_build_ms','r58_homotopy_ms', ...
    'r58_certificate_ms','r58_shadow_ms', ...
    'r58r7_solver_ms','r58r8_ipm_ms', ...
    'r58r9_full_scan_ms','r58r9_subset_solver_ms', ...
    'r58r9_plus_cg_ms','r58r9_minus_cg_ms', ...
    'r58r11_solver_ms','native_total_ms','last_stage1_ms'};
for k = 1:numel(timingKeys)
    key = timingKeys{k};
    R.timings.(key) = local_scalar(R58A,key);
end
R.timings.two_branch_solver_sum_ms = ...
    R.timings.r58r9_plus_cg_ms + R.timings.r58r9_minus_cg_ms;
R.timings.r58r11_solver_minus_sum_ms = ...
    R.timings.r58r11_solver_ms - R.timings.two_branch_solver_sum_ms;

% Generic recursive discovery of existing (not invented) branch/knot/leaf
% arrays or structs. This never implies the data are sufficiently exposed.
R.exposed_arrays = local_exposed_arrays(R58A,'R58A',0);
R.current_speed_witness_is_finite = ...
    isfinite(R.measurements.r58_speed_VQ) || ...
    isfinite(R.measurements.r58r6_failed_speed_witness);
R.internal_gate_provenance_present = local_has_gate_provenance(R58A);

if R.both_branches_rejected_at_speed
    R.classification = 'TWO_QPS_CLOSED_BOTH_REJECTED_BY_SPEED_GATE';
elseif R.no_free_C7_candidate
    R.classification = 'NO_CONSTRUCTED_FREE_C7_CANDIDATE';
else
    R.classification = 'OTHER_OR_INCOMPLETE_TELEMETRY';
end
if R.both_branches_rejected_at_speed && ...
        ~R.internal_gate_provenance_present
    R.root_cause_identified = false;
    R.next_required = 'INSTRUMENT_R58_SHADOW_SOURCE_AT_EACH_SPEED_REJECT';
else
    R.root_cause_identified = false; % Independent physical proof not supplied.
    R.next_required = 'VERIFY_EXPOSED_BRANCH_GATE_RECORDS_AND_PHYSICAL_CERTIFICATE';
end

fprintf('\n====================================================================================================\n');
fprintf(' AF7R58R12 | POST-R11 ROOT-CAUSE TRIAGE (READ-ONLY)\n');
fprintf(' production/acceptance/certificate modified: 0 / 0 / 0\n');
fprintf('====================================================================================================\n');
fprintf('R58R11 audit source              = %s\n',R.source.R58R11_audit);
fprintf('REPLANNING_SYSTEM source        = %s\n',R.source.REPLANNING_SYSTEM);
fprintf('QP branches both closed         = %d\n',R.qp_both_closed);
fprintf('Branches rejected at speed      = %d\n',R.both_branches_rejected_at_speed);
fprintf('Constructed free-C7 is zero    = %d\n',R.no_free_C7_candidate);
fprintf('Time-law needed, not built      = %d\n',R.time_law_indicated_not_constructed);
fprintf('Current speed VQ witness finite = %d\n',R.current_speed_witness_is_finite);
fprintf('Per-gate provenance exposed     = %d\n',R.internal_gate_provenance_present);
fprintf('Class                            = %s\n',R.classification);
fprintf('Root cause independently proven = %d\n',R.root_cause_identified);

fprintf('\n[QP / FINITE-SENSING / DOWNSTREAM]\n');
for k = 1:numel(qKeys)
    key = qKeys{k};
    fprintf('  %-43s = %.16g\n',key,R.counters.(key));
end
fprintf('\n[NUMERICS: INDICATORS, NOT CAUSAL PROOF]\n');
for k = 1:numel(measureKeys)
    key = measureKeys{k};
    fprintf('  %-43s = %.16g\n',key,R.measurements.(key));
end
fprintf('\n[TIMING: RECORDED, NO RELAXATION]\n');
for k = 1:numel(timingKeys)
    key = timingKeys{k};
    fprintf('  %-43s = %.7g\n',key,R.timings.(key));
end
fprintf('  %-43s = %.7g\n','two_branch_solver_sum_ms', ...
    R.timings.two_branch_solver_sum_ms);
fprintf('  %-43s = %.7g\n','r58r11_solver_minus_sum_ms', ...
    R.timings.r58r11_solver_minus_sum_ms);

fprintf('\n[EXPOSED NONSCALAR BRANCH / KNOT / WITNESS OBJECTS]\n');
if isempty(R.exposed_arrays)
    fprintf('  NONE FOUND in R58A (not evidence of feasibility).\n');
else
    for k = 1:numel(R.exposed_arrays)
        fprintf('  %s\n',R.exposed_arrays{k});
    end
end
fprintf('\n[DECISIVE SHADOW-SOURCE INSTRUMENTATION STILL REQUIRED]\n');
fprintf('  1) one record per full +/- branch: exact speed-reject reason and stage\n');
fprintf('  2) pre-/post-quantization physical 0..7 node jets, knot times, durations\n');
fprintf('  3) total (nominal + correction) Bernstein V/A/J bounds per segment\n');
fprintf('  4) bound rejection vs certified finite violating witness: KEEP DISTINCT\n');
fprintf('  5) Stage-B worst leaf: segment, [ta,tb], VL, sigma, qdot, VQ\n');
fprintf('  6) time-law existence/lead/safety checks; no arbitrary stretching\n');
fprintf('  7) later: C7/yaw-C6 and UAV/cable/payload continuous certificates\n');
fprintf('A sampled safe point NEVER grants continuous-time acceptance.\n');
fprintf('Speed upper bound > limit does NOT prove physical infeasibility.\n');
fprintf('NaN is UNKNOWN, NEVER PASS.\n');
fprintf('====================================================================================================\n');

saveDir = fileparts(mfilename('fullpath'));
if isempty(saveDir), saveDir = pwd; end
R.report_path = fullfile(saveDir,'AF7R58R12_POST_R11_GATE_REPORT.txt');
R.mat_path = fullfile(saveDir,'AF7R58R12_POST_R11_GATE_CAPTURE.mat');
% Preserve exact prior results for source-bound repeatability.
save(R.mat_path,'R','R58A','D38','-v7.3');
fid = fopen(R.report_path,'w');
if fid >= 0
    cc = onCleanup(@() fclose(fid));
    fprintf(fid,'AF7R58R12 | POST-R11 READ-ONLY TRIAGE\n');
    fprintf(fid,'classification = %s\n',R.classification);
    fprintf(fid,'root_cause_identified = %d\n',R.root_cause_identified);
    fprintf(fid,'next_required = %s\n',R.next_required);
    fprintf(fid,'R58R11 audit source = %s\n',R.source.R58R11_audit);
    fprintf(fid,'REPLANNING_SYSTEM source = %s\n',R.source.REPLANNING_SYSTEM);
    fprintf(fid,'QP both closed = %d, speed both rejected = %d\n', ...
        R.qp_both_closed,R.both_branches_rejected_at_speed);
    fprintf(fid,'free-C7 not built = %d, needs time-law = %d\n', ...
        R.no_free_C7_candidate,R.time_law_indicated_not_constructed);
    fprintf(fid,'current finite VQ witness = %d\n',R.current_speed_witness_is_finite);
    fprintf(fid,'per-gate provenance exposed = %d\n',R.internal_gate_provenance_present);
    fprintf(fid,'\nCOUNTERS\n');
    for k = 1:numel(qKeys)
        key=qKeys{k}; fprintf(fid,'%s = %.17g\n',key,R.counters.(key));
    end
    fprintf(fid,'\nMEASUREMENTS\n');
    for k = 1:numel(measureKeys)
        key=measureKeys{k}; fprintf(fid,'%s = %.17g\n',key,R.measurements.(key));
    end
    fprintf(fid,'\nTIMINGS\n');
    for k = 1:numel(timingKeys)
        key=timingKeys{k}; fprintf(fid,'%s = %.17g\n',key,R.timings.(key));
    end
    fprintf(fid,'\nEXPOSED ARRAYS\n');
    for k = 1:numel(R.exposed_arrays)
        fprintf(fid,'%s\n',R.exposed_arrays{k});
    end
    clear cc
end
fprintf('Saved report = %s\n',R.report_path);
fprintf('Saved MAT    = %s\n',R.mat_path);
end

function v = local_scalar(S,name)
v = NaN;
if isfield(S,name)
    x=S.(name);
    if (isnumeric(x) || islogical(x)) && isscalar(x) && isreal(x)
        v=double(x);
    end
end
end

function tf = local_eq(S,name,value)
v=local_scalar(S,name);
tf=isfinite(v) && v==value;
end

function names=local_exposed_arrays(S,prefix,depth)
names={};
if depth>4 || ~isstruct(S), return; end
if numel(S)~=1
    names={sprintf('%s: struct array [%s]',prefix,num2str(size(S)))};
    return
end
f=fieldnames(S);
for i=1:numel(f)
    key=f{i}; x=S.(key); path=[prefix '.' key];
    if isstruct(x)
        names=[names local_exposed_arrays(x,path,depth+1)]; %#ok<AGROW>
    elseif (isnumeric(x) || islogical(x)) && ~isscalar(x) && ...
            any(contains(lower(key), ...
            {'branch','candidate','side','knot','jet','segment', ...
            'bernstein','leaf','witness','time_law','duration','control'}))
        names{end+1}=sprintf('%s: [%s] %s', ...
            path,num2str(size(x)),class(x)); %#ok<AGROW>
    end
end
end

function tf=local_has_gate_provenance(S)
% A numeric speed_fail COUNT by itself is not provenance.
tf=false;
f=fieldnames(S);
for i=1:numel(f)
    key=lower(f{i}); x=S.(f{i});
    if (contains(key,'speed_reject_reason') || ...
        contains(key,'speed_gate_trace') || ...
        contains(key,'speed_reject_stage') || ...
        contains(key,'branch_speed_detail')) && ~isempty(x)
        if isnumeric(x), tf=any(isfinite(x(:))); else, tf=true; end
        if tf, return; end
    end
end
end
