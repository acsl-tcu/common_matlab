function CHECKPOINT = test_C6_BSPLINE_PHASE9CR8F101_CHECKPOINT_CURRENT_BEST()
% =========================================================================
% F101 CHECKPOINT -- CURRENT BEST RESEARCH SNAPSHOT
%
% PURPOSE
%   Freeze the current best validated MATLAB architecture as an intermediate
%   research checkpoint BEFORE integrating the experimental MEX acceleration.
%
% AUTHORITY
%   Calls:
%     test_C6_BSPLINE_PHASE9CR8F101R_FOUR_STAGE_ARCHITECTURE_REDUCTION_EXPLORER_FIXED
%
%   Current best extracted here:
%     result.f101r.records  (R1 four-stage reduced architecture)
%
% IMPORTANT
%   - This file does NOT change any safety tolerance or trajectory equation.
%   - This file does NOT integrate the W1 tension MEX into production.
%   - Numerical C6 checking remains validation-only and is excluded from R1
%     production timing, exactly as in F101-R.
%   - Runtime is empirical MATLAB runtime, NOT WCET.
%
% OUTPUT FILES
%   F101_CHECKPOINT_CURRENT_BEST.mat
%   F101_CHECKPOINT_CURRENT_BEST_CASES.csv
%   F101_CHECKPOINT_CURRENT_BEST_STATUS.csv
%   F101_CHECKPOINT_CURRENT_BEST_REPORT.txt
%
% RUN
%   CHECKPOINT = test_C6_BSPLINE_PHASE9CR8F101_CHECKPOINT_CURRENT_BEST;
% =========================================================================

clc;
fprintf('================================================================================\n');
fprintf(' F101 CHECKPOINT -- CURRENT BEST INTERMEDIATE RESEARCH SNAPSHOT\n');
fprintf('================================================================================\n');
fprintf(' Production authority : F101-R / R1 four-stage reduced architecture\n');
fprintf(' MEX status           : W1 tension MEX is EXPERIMENTAL, NOT integrated here\n');
fprintf(' Safety semantics     : unchanged\n');
fprintf(' Numerical C6 timing  : validation-only, excluded from production latency\n');
fprintf(' Runtime meaning      : empirical MATLAB timing, NOT WCET\n');
fprintf('================================================================================\n\n');

authority_fun='test_C6_BSPLINE_PHASE9CR8F101R_FOUR_STAGE_ARCHITECTURE_REDUCTION_EXPLORER_FIXED';

if exist(authority_fun,'file')~=2
    error('F101Checkpoint:AuthorityMissing', ...
        ['Required authority file is not on the MATLAB path:\n  %s.m\n' ...
         'Place this checkpoint file in the same folder as the current F101-R file.'], ...
         authority_fun);
end

%% ------------------------------------------------------------------------
% 1. Execute the unchanged authority
% -------------------------------------------------------------------------
fprintf('[1/6] Running unchanged F101-R authority...\n');
wall=tic;
AUTH=feval(authority_fun);
wall_s=toc(wall);

if ~isstruct(AUTH) || ~isfield(AUTH,'f101r') || ...
        ~isstruct(AUTH.f101r) || ~isfield(AUTH.f101r,'records')
    error('F101Checkpoint:BadAuthorityOutput', ...
        'Authority output does not contain result.f101r.records.');
end

R=AUTH.f101r.records;
if isempty(R)
    error('F101Checkpoint:NoRecords','F101-R returned no checkpoint records.');
end

required={'case_id','r0_ms','r1_ms','safe','exact','Perr','gerr','Merr','c6', ...
    'c6_validation_ms','nobs','nlocal','ndir','cache_ms','dyn_ms','corr_ms','gate_ms'};
for k=1:numel(required)
    if ~isfield(R,required{k})
        error('F101Checkpoint:SchemaMismatch', ...
            'F101-R record field "%s" is missing.',required{k});
    end
end

fprintf('Authority completed in %.3f s wall time.\n',wall_s);

%% ------------------------------------------------------------------------
% 2. Build immutable case table from the actual authority output
% -------------------------------------------------------------------------
fprintf('\n[2/6] Building detailed case table...\n');

case_id=[R.case_id]';
r0_ms=[R.r0_ms]';
r1_ms=[R.r1_ms]';
safe=logical([R.safe]');
exact=logical([R.exact]');
Perr=[R.Perr]';
gerr=[R.gerr]';
Merr=[R.Merr]';
c6=[R.c6]';
c6_validation_ms=[R.c6_validation_ms]';
nobs=[R.nobs]';
nlocal=[R.nlocal]';
ndir=[R.ndir]';
cache_ms=[R.cache_ms]';
dyn_ms=[R.dyn_ms]';
corr_ms=[R.corr_ms]';
gate_ms=[R.gate_ms]';

stage_sum_ms=cache_ms+dyn_ms+corr_ms+gate_ms;
other_ms=r1_ms-stage_sum_ms;
speedup_r0_over_r1=r0_ms./r1_ms;
under_2ms=safe & r1_ms<2;
under_3ms=safe & r1_ms<3;

CASES=table(case_id,nobs,nlocal,ndir,safe,exact, ...
    r0_ms,r1_ms,speedup_r0_over_r1, ...
    cache_ms,dyn_ms,corr_ms,gate_ms,stage_sum_ms,other_ms, ...
    Perr,gerr,Merr,c6,c6_validation_ms,under_2ms,under_3ms);

%% ------------------------------------------------------------------------
% 3. Statistical summary
% -------------------------------------------------------------------------
fprintf('[3/6] Computing timing/safety/exactness statistics...\n');

S=struct();
S.ncase=height(CASES);
S.safe_count=nnz(safe);
S.exact_count=nnz(exact);
S.safe_fraction=S.safe_count/S.ncase;
S.exact_fraction=S.exact_count/S.ncase;

S.r1=stats_full(r1_ms);
S.r0=stats_full(r0_ms);
S.cache=stats_full(cache_ms);
S.dynamics=stats_full(dyn_ms);
S.corridor=stats_full(corr_ms);
S.gate=stats_full(gate_ms);
S.c6_validation=stats_full(c6_validation_ms);

S.under_2ms_count=nnz(under_2ms);
S.under_3ms_count=nnz(under_3ms);
S.max_Perr=max(Perr);
S.max_gerr=max(gerr);
S.max_Merr=max(Merr);
S.max_C6=max(c6);

[~,iworst]=max(r1_ms);
S.worst_case_id=case_id(iworst);
S.worst_case_ms=r1_ms(iworst);
S.worst_case=CASES(iworst,:);

[~,icorr]=max(corr_ms);
S.worst_corridor_case_id=case_id(icorr);
S.worst_corridor_ms=corr_ms(icorr);

[~,igate]=max(gate_ms);
S.worst_gate_case_id=case_id(igate);
S.worst_gate_ms=gate_ms(igate);

%% ------------------------------------------------------------------------
% 4. Research-status ledger
% -------------------------------------------------------------------------
fprintf('[4/6] Building achieved / partial / not-yet-achieved ledger...\n');

item={ ...
'Unknown obstacles: first knowledge at sensor detection'; ...
'Position C6 construction across replanning joins'; ...
'6D online decision variable P=Pbase+R6*y'; ...
'O8 tangent geometry candidate direction generation'; ...
'Rotated anisotropic ellipsoid obstacle geometry'; ...
'Suspended UAV+cable+payload directional geometry'; ...
'Hard v/a/j constraints and box authority'; ...
'Multiple simultaneous obstacles in benchmark'; ...
'Final ALL-span x ALL-detected-obstacle continuous-time certificate'; ...
'Candidate physics separated from final-certificate physics'; ...
'Numerical C6 endpoint regression diagnostic'; ...
'Event-driven heavy replan + stored trajectory 25-ms evaluator architecture'; ...
'Automatic robust avoidance-duration selection from current state/geometry'; ...
'Replan again when a newly detected obstacle threatens current avoidance'; ...
'Obstacle memory retention after leaving sensing range'; ...
'Yaw policy integrated and join-consistency verified'; ...
'Every controller desired value logged/checked at every 25-ms control step'; ...
'Controller-aware trackability guarantee tied to literature controller model'; ...
'Moving-obstacle generality demonstrated'; ...
'2-3 ms certified production latency in MATLAB'; ...
'MEX tension kernel exactness experiment'; ...
'MEX tension kernel integrated into F101-R production gate'; ...
'Corridor/gate/solver compiled MEX production path'; ...
'Onboard Python/ROS 2 Jazzy implementation'; ...
'Hard real-time / WCET proof'};

status={ ...
'ESTABLISHED_ASSUMPTION'; ...
'ESTABLISHED'; ...
'ESTABLISHED'; ...
'ESTABLISHED_CANDIDATE_METHOD'; ...
'ESTABLISHED'; ...
'ESTABLISHED'; ...
'ESTABLISHED'; ...
'VALIDATED_IN_BENCHMARK'; ...
'ESTABLISHED_EXECUTION_AUTHORITY'; ...
'ESTABLISHED'; ...
'VALIDATION_ONLY'; ...
'ARCHITECTURE_DEFINED_NOT_END_TO_END_DEPLOYED'; ...
'NOT_YET_REALIZED'; ...
'NOT_YET_END_TO_END_VALIDATED'; ...
'NOT_YET_REALIZED'; ...
'PARTIAL_POLICY_ONLY'; ...
'NOT_YET_REALIZED'; ...
'NOT_YET_ESTABLISHED'; ...
'NOT_DEMONSTRATED_BY_CURRENT_CHECKPOINT'; ...
'NOT_YET_ACHIEVED'; ...
'VALIDATED_SEPARATE_EXPERIMENT'; ...
'NOT_YET_INTEGRATED'; ...
'NOT_YET_REALIZED'; ...
'NOT_YET_REALIZED'; ...
'NOT_ESTABLISHED'};

evidence={ ...
'F101 benchmark semantics reveal obstacles at t_detect; candidate generation then replans.'; ...
'Structural C6 construction retained; numerical endpoint check is regression validation.'; ...
'F101-R uses R6 with y in R^6.'; ...
'Current best candidate path uses geometry-derived TANGENT directions.'; ...
'Final certificate uses ellipsoid support geometry.'; ...
'Directional payload/UAV/cable support is retained in certificate.'; ...
'No safety/dynamic tolerance relaxation in R1.'; ...
'Benchmark includes simultaneous obstacle cases; checkpoint records nobs.'; ...
'Final gate evaluates all spans against all detected obstacles.'; ...
'Candidate and final certificate are numerically separated after solve.'; ...
'c6 and c6_validation_ms recorded separately; excluded from production latency.'; ...
'Design principle fixed, but this MATLAB benchmark is a replanning-kernel experiment.'; ...
'Current duration T is inherited from benchmark/planning setup; final robust automatic selector remains open.'; ...
'Architecture permits replanning, but full sequential sensor-arrival experiment is not this checkpoint.'; ...
'No completed retention/discard policy in current production benchmark.'; ...
'Nominal/held yaw is acceptable; yaw C6 across switch is not claimed here.'; ...
'Current benchmark validates generated trajectory, not the complete 25-ms controller-output logger.'; ...
'v/a/j bounds exist; closed-loop controller-aware tracking theorem is still open.'; ...
'Current certificate/benchmark evidence here is not a demonstrated moving-obstacle theorem.'; ...
'Pass only when certified runtime is <3 ms; current result determines count automatically.'; ...
'Separate W1: original-vs-MEX max error 3.499e-13; single 3.407x; 22-span 2.761x; 29-span 2.697x.'; ...
'This checkpoint intentionally runs unchanged MATLAB F101-R.'; ...
'Next optimization target after checkpoint; especially corridor and full gate.'; ...
'Migration target only; this checkpoint is MATLAB/Windows.'; ...
'Measured runtime is empirical, explicitly not WCET.'};

STATUS=table(item,status,evidence);

%% ------------------------------------------------------------------------
% 5. Print detailed report
% -------------------------------------------------------------------------
fprintf('\n[5/6] CURRENT BEST RESULTS\n');
fprintf('================================================================================\n');
fprintf(' Authority                    : F101-R R1\n');
fprintf(' Cases                        : %d\n',S.ncase);
fprintf(' SAFE                         : %d/%d\n',S.safe_count,S.ncase);
fprintf(' Exact vs R0                  : %d/%d\n',S.exact_count,S.ncase);
fprintf(' Max P/g/M error              : %.3e / %.3e / %.3e\n',S.max_Perr,S.max_gerr,S.max_Merr);
fprintf(' Max numerical C6 error       : %.3e\n',S.max_C6);
fprintf('\n R1 production latency [ms]\n');
print_stats('R1 TOTAL',S.r1);
print_stats('cache',S.cache);
print_stats('dynamics',S.dynamics);
print_stats('corridor',S.corridor);
print_stats('gate',S.gate);
fprintf('\n Certified <2 ms              : %d/%d\n',S.under_2ms_count,S.ncase);
fprintf(' Certified <3 ms              : %d/%d\n',S.under_3ms_count,S.ncase);
fprintf(' Worst total case             : Case %d = %.3f ms\n',S.worst_case_id,S.worst_case_ms);
fprintf(' Worst corridor case          : Case %d = %.3f ms\n',S.worst_corridor_case_id,S.worst_corridor_ms);
fprintf(' Worst gate case              : Case %d = %.3f ms\n',S.worst_gate_case_id,S.worst_gate_ms);
fprintf('\n CASE TABLE\n');
disp(CASES);

fprintf('\n STATUS LEDGER\n');
disp(STATUS(:,1:2));

%% ------------------------------------------------------------------------
% 6. Persist checkpoint
% -------------------------------------------------------------------------
fprintf('[6/6] Saving checkpoint artifacts...\n');

stamp=char(datetime('now','Format','yyyy-MM-dd HH:mm:ss'));
CHECKPOINT=struct();
CHECKPOINT.timestamp=stamp;
CHECKPOINT.authority_function=authority_fun;
CHECKPOINT.authority_wall_s=wall_s;
CHECKPOINT.current_best='F101-R R1 four-stage reduced MATLAB architecture';
CHECKPOINT.production_timing_excludes_numerical_C6=true;
CHECKPOINT.runtime_is_WCET=false;
CHECKPOINT.cases=CASES;
CHECKPOINT.summary=S;
CHECKPOINT.status=STATUS;
CHECKPOINT.mex_tension_experiment=struct( ...
    'integrated_in_current_best',false, ...
    'validation_max_abs_error',3.4994e-13, ...
    'single_speedup',3.4068, ...
    'span22_speedup',2.761, ...
    'span29_speedup',2.697, ...
    'note','Separate W1 K6 EIGFIX experiment supplied by user; not production R1 timing.');

save('F101_CHECKPOINT_CURRENT_BEST.mat','CHECKPOINT','AUTH','-v7.3');
writetable(CASES,'F101_CHECKPOINT_CURRENT_BEST_CASES.csv');
writetable(STATUS,'F101_CHECKPOINT_CURRENT_BEST_STATUS.csv');

write_text_report('F101_CHECKPOINT_CURRENT_BEST_REPORT.txt',CHECKPOINT);

fprintf('\n================================================================================\n');
fprintf(' CHECKPOINT SAVED\n');
fprintf('  F101_CHECKPOINT_CURRENT_BEST.mat\n');
fprintf('  F101_CHECKPOINT_CURRENT_BEST_CASES.csv\n');
fprintf('  F101_CHECKPOINT_CURRENT_BEST_STATUS.csv\n');
fprintf('  F101_CHECKPOINT_CURRENT_BEST_REPORT.txt\n');
fprintf('================================================================================\n');
fprintf('\nINTERPRETATION RULE:\n');
fprintf(' This checkpoint freezes what is achieved NOW. Experimental MEX speedups are\n');
fprintf(' recorded as separate evidence and are NOT silently counted as R1 production.\n');
fprintf('================================================================================\n');
end

function s=stats_full(x)
x=x(:);
x=x(isfinite(x));
if isempty(x)
    s=struct('min',NaN,'median',NaN,'p90',NaN,'p95',NaN,'p99',NaN,'max',NaN);
    return;
end
x=sort(x);
s=struct('min',x(1),'median',pct_sorted(x,50),'p90',pct_sorted(x,90), ...
    'p95',pct_sorted(x,95),'p99',pct_sorted(x,99),'max',x(end));
end

function y=pct_sorted(x,p)
n=numel(x);
if n==1, y=x(1); return; end
q=1+(n-1)*p/100;
i=floor(q); j=ceil(q);
if i==j
    y=x(i);
else
    y=x(i)+(q-i)*(x(j)-x(i));
end
end

function print_stats(name,s)
fprintf(' %-12s min %7.3f | med %7.3f | p90 %7.3f | p95 %7.3f | p99 %7.3f | max %7.3f\n', ...
    name,s.min,s.median,s.p90,s.p95,s.p99,s.max);
end

function write_text_report(fname,C)
fid=fopen(fname,'w');
if fid<0, error('F101Checkpoint:ReportWrite','Cannot write %s.',fname); end
cc=onCleanup(@() fclose(fid));

S=C.summary;
fprintf(fid,'F101 CURRENT BEST INTERMEDIATE RESEARCH CHECKPOINT\n');
fprintf(fid,'Generated: %s\n',C.timestamp);
fprintf(fid,'Authority: %s\n',C.authority_function);
fprintf(fid,'Current best: %s\n',C.current_best);
fprintf(fid,'Runtime is WCET: %d\n',C.runtime_is_WCET);
fprintf(fid,'Numerical C6 included in production timing: %d\n\n',~C.production_timing_excludes_numerical_C6);

fprintf(fid,'=== VALIDATION ===\n');
fprintf(fid,'SAFE: %d/%d\n',S.safe_count,S.ncase);
fprintf(fid,'Exact vs R0: %d/%d\n',S.exact_count,S.ncase);
fprintf(fid,'Max P error: %.16e\n',S.max_Perr);
fprintf(fid,'Max g error: %.16e\n',S.max_gerr);
fprintf(fid,'Max M error: %.16e\n',S.max_Merr);
fprintf(fid,'Max C6 error: %.16e\n\n',S.max_C6);

fprintf(fid,'=== R1 PRODUCTION LATENCY [ms] ===\n');
write_stats(fid,'TOTAL',S.r1);
write_stats(fid,'CACHE',S.cache);
write_stats(fid,'DYNAMICS',S.dynamics);
write_stats(fid,'CORRIDOR',S.corridor);
write_stats(fid,'GATE',S.gate);
fprintf(fid,'Certified <2 ms: %d/%d\n',S.under_2ms_count,S.ncase);
fprintf(fid,'Certified <3 ms: %d/%d\n',S.under_3ms_count,S.ncase);
fprintf(fid,'Worst total: Case %d %.6f ms\n',S.worst_case_id,S.worst_case_ms);
fprintf(fid,'Worst corridor: Case %d %.6f ms\n',S.worst_corridor_case_id,S.worst_corridor_ms);
fprintf(fid,'Worst gate: Case %d %.6f ms\n\n',S.worst_gate_case_id,S.worst_gate_ms);

fprintf(fid,'=== SEPARATE MEX EVIDENCE -- NOT INTEGRATED ===\n');
M=C.mex_tension_experiment;
fprintf(fid,'Original-vs-MEX max error: %.16e\n',M.validation_max_abs_error);
fprintf(fid,'Single-call speedup: %.4fx\n',M.single_speedup);
fprintf(fid,'22-span speedup: %.4fx\n',M.span22_speedup);
fprintf(fid,'29-span speedup: %.4fx\n',M.span29_speedup);
fprintf(fid,'%s\n\n',M.note);

fprintf(fid,'=== RESEARCH STATUS ===\n');
for i=1:height(C.status)
    fprintf(fid,'[%s] %s\n',C.status.status{i},C.status.item{i});
    fprintf(fid,'  %s\n',C.status.evidence{i});
end

fprintf(fid,'\n=== IMPORTANT OPEN ITEMS ===\n');
fprintf(fid,'1. Certified 2-3 ms production latency is not yet achieved.\n');
fprintf(fid,'2. W1 tension MEX is not yet integrated into the production R1 gate.\n');
fprintf(fid,'3. Corridor/gate/solver remain the main compilation/architecture targets.\n');
fprintf(fid,'4. Robust automatic avoidance-duration selection remains open.\n');
fprintf(fid,'5. Sequential new-obstacle-during-avoidance end-to-end validation remains open.\n');
fprintf(fid,'6. Obstacle-memory retention/discard policy remains open.\n');
fprintf(fid,'7. 25-ms controller desired-output logging/checking remains open.\n');
fprintf(fid,'8. Controller-aware trackability guarantee remains open.\n');
fprintf(fid,'9. Moving-obstacle generality is not demonstrated by this checkpoint.\n');
fprintf(fid,'10. Onboard Python/ROS 2 Jazzy migration remains open.\n');
fprintf(fid,'11. Hard real-time/WCET proof remains open.\n');
end

function write_stats(fid,name,s)
fprintf(fid,'%-10s min %.6f median %.6f p90 %.6f p95 %.6f p99 %.6f max %.6f\n', ...
    name,s.min,s.median,s.p90,s.p95,s.p99,s.max);
end
