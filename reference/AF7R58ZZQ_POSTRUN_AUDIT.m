function OUT = AF7R58ZZQ_POSTRUN_AUDIT()
% AF7R58ZZQ_POSTRUN_AUDIT
% Read-only success/timing audit after the fixed-H=4 s / one-control-period
% synchronous validation architecture is installed and the SAME flight case
% is rerun.

EXPECTED_SHA='bc97df3a0a7341ac5605d52f4dd94923bfd831a11be535450666ec6d10ff41ec';

OUT=struct('schema','AF7R58ZZQ_POSTRUN_AUDIT_V1', ...
    'production_modified',false,'status','OPEN','error',struct());

fprintf('\n====================================================================================================\n');
fprintf(' AF7R58ZZQ | POST-RUN FIXED-4S SUCCESS/TIMING AUDIT\n');
fprintf('====================================================================================================\n');
fprintf('[AUTHORITY] readOnly=1 | fixedHorizon4s=1 | noObstacleInjection=1 | noCommit=1\n');

try
    pR=which('REPLANNING_SYSTEM');
    assert(~isempty(pR)&&isfile(pR),'[AF7R58ZZQ-AUDIT] REPLANNING_SYSTEM unresolved.');
    OUT.source_sha=file_hash(pR);
    OUT.source_exact=strcmp(OUT.source_sha,EXPECTED_SHA);
    assert(OUT.source_exact,'[AF7R58ZZQ-AUDIT] source differs from installed ZZQ authority.');

    R=fileread(pR);
    OUT.contract=struct();
    OUT.contract.fixed_horizon_4s=contains(R,'horizon_s     = 4.0;');
    OUT.contract.no_extended_preview= ...
        ~contains(R,'prepare_early_intervention_reference_cache') && ...
        ~contains(R,'AF7R58ZZP theorem-derived EARLY intervention scheduler');
    OUT.contract.one_cycle_budget= ...
        count_exact(R,'attempt_budget_s=double(obj.prediction_dt);')==2;
    OUT.contract.legacy4ms_still_stop_only= ...
        contains(R,'hard_modify_deadline_s      = 4.0e-3;') && ...
        contains(R,'historical_4ms_used_as_normal_planner_deadline=false');
    OUT.contract.finite_sensing= ...
        contains(R,'The generator receives ONLY sensor-known obstacles:') && ...
        contains(R,'''detected_only_obstacle_contract'',true') && ...
        contains(R,'Initial known set is empty.');
    OUT.contract.closed=all(structfun(@logical,OUT.contract));

    assert(evalin('base','exist(''AF7R58ZZB_RUNTIME_PROFILE'',''var'')')==1, ...
        '[AF7R58ZZQ-AUDIT] AF7R58ZZB_RUNTIME_PROFILE absent. Run the same flight case first.');
    P=evalin('base','AF7R58ZZB_RUNTIME_PROFILE');

    if istable(P)
        vn=string(P.Properties.VariableNames);
        getv=@(nm) P.(char(nm));
    elseif isstruct(P)&&isscalar(P)
        vn=string(fieldnames(P));
        getv=@(nm) P.(char(nm));
    else
        error('[AF7R58ZZQ-AUDIT] runtime profile must be table or scalar struct.');
    end

    reqNames=["phase","t","gen_request_delta","gen_deadline_delta", ...
        "gen_commit_delta","gen_last_cycle_critical_ms","gen_last_callback_ms", ...
        "reference_s","full_iteration_s"];
    missing=reqNames(~arrayfun(@(x)any(vn==x),reqNames));
    assert(isempty(missing), ...
        '[AF7R58ZZQ-AUDIT] profile missing: %s',strjoin(cellstr(missing),', '));

    phase=getv("phase");
    reqDelta=double(getv("gen_request_delta"));
    reqMask=(phase=='f') & reqDelta>0;
    idx=find(reqMask);

    OUT.request_count=numel(idx);
    OUT.request_index=idx(:);

    t=double(getv("t"));
    OUT.request_t_s=t(idx);
    d=double(getv("gen_deadline_delta"));
    OUT.deadline_fail=d(idx);
    c=double(getv("gen_commit_delta"));
    OUT.commit=c(idx);
    h=double(getv("gen_last_cycle_critical_ms"));
    OUT.attempt_ms=h(idx);
    cb=double(getv("gen_last_callback_ms"));
    OUT.callback_ms=cb(idx);
    ref=double(getv("reference_s"));
    OUT.reference_ms=1e3*ref(idx);
    fi=double(getv("full_iteration_s"));
    OUT.full_iteration_ms=1e3*fi(idx);

    OUT.first_commit_request=first_positive(OUT.commit);
    OUT.first_deadline_fail_request=first_positive(OUT.deadline_fail);
    OUT.deadline_fail_count=sum(OUT.deadline_fail);
    OUT.commit_count=sum(OUT.commit);

    if OUT.request_count>0
        OUT.first_request_time_s=OUT.request_t_s(1);
    else
        OUT.first_request_time_s=NaN;
    end
    if isfinite(OUT.first_commit_request)
        OUT.first_commit_time_s=OUT.request_t_s(OUT.first_commit_request);
        OUT.trigger_to_commit_s=OUT.first_commit_time_s-OUT.first_request_time_s;
    else
        OUT.first_commit_time_s=NaN;
        OUT.trigger_to_commit_s=NaN;
    end

    OUT.max_attempt_ms=max_or_nan(OUT.attempt_ms);
    OUT.median_attempt_ms=median_or_nan(OUT.attempt_ms);
    OUT.max_full_iteration_ms=max_or_nan(OUT.full_iteration_ms);
    OUT.request_rows_over_25ms=nnz(OUT.full_iteration_ms>25);

    OUT.object_found=false;
    if evalin('base','exist(''replanner_instance'',''var'')')==1
        obj=evalin('base','replanner_instance');
        if isa(obj,'REPLANNING_SYSTEM')
            OUT.object_found=true;
            OUT.horizon_s=double(obj.horizon_s);
            OUT.prediction_dt=double(obj.prediction_dt);
            OUT.legacy_stop_fallback_4ms_s=double(obj.hard_modify_deadline_s);
            B=obj.get_reference_only_sync_budget_certificate();
            OUT.sync_budget_certificate=B;
            OUT.generator_request_count=double(obj.generator_request_count);
            OUT.generator_deadline_fail_count=double(obj.generator_deadline_fail_count);
            OUT.replan_commit_count=double(obj.replan_commit_count);
            OUT.reference_only_commit_count=double(obj.reference_only_commit_count);
        end
    end

    if ~OUT.contract.closed
        OUT.status='CONTRACT_OPEN';
    elseif OUT.request_count==0
        OUT.status='NO_REAL_REPLAN_REQUEST';
    elseif ~isfinite(OUT.first_commit_request)
        OUT.status='NO_CERTIFIED_COMMIT__DO_NOT_ADVANCE';
    elseif OUT.first_commit_request==1 && OUT.deadline_fail_count==0
        if OUT.request_rows_over_25ms==0
            OUT.status='FIRST_REQUEST_COMMIT__NO_ATTEMPT_FAIL__CONTROL_PERIOD_CLOSED';
        else
            OUT.status='FIRST_REQUEST_COMMIT__PLANNER_SUCCESS__OUTER_LOOP_OVERRUN_REMAINS';
        end
    else
        OUT.status='COMMIT_SUCCEEDED_BUT_NOT_FIRST_REQUEST__ONE_MORE_ROOTCAUSE_PASS_REQUIRED';
    end

    fprintf(' source exact                         = %d\n',OUT.source_exact);
    fprintf(' fixed H=4 s                          = %d\n',OUT.contract.fixed_horizon_4s);
    fprintf(' extended preview absent              = %d\n',OUT.contract.no_extended_preview);
    fprintf(' one-control-period planner budget    = %d\n',OUT.contract.one_cycle_budget);
    fprintf(' request count                        = %d\n',OUT.request_count);
    fprintf(' deadline-fail count                  = %.0f\n',OUT.deadline_fail_count);
    fprintf(' commit count                         = %.0f\n',OUT.commit_count);
    fprintf(' first commit request                 = %s\n',numtxt(OUT.first_commit_request));
    fprintf(' trigger->commit [s]                  = %s\n',numtxt(OUT.trigger_to_commit_s));
    fprintf(' attempt median/max [ms]              = %.6f / %.6f\n', ...
        OUT.median_attempt_ms,OUT.max_attempt_ms);
    fprintf(' full-iteration max [ms]              = %.6f\n',OUT.max_full_iteration_ms);
    fprintf(' request rows >25 ms                  = %d\n',OUT.request_rows_over_25ms);

    fprintf('\n request rows:\n');
    fprintf(' i   t[s]    attempt  callback  ref_ms  full_ms  fail commit\n');
    for k=1:OUT.request_count
        fprintf('%2d %7.3f %8.3f %8.3f %7.3f %8.3f %4.0f %6.0f\n', ...
            k,OUT.request_t_s(k),OUT.attempt_ms(k),OUT.callback_ms(k), ...
            OUT.reference_ms(k),OUT.full_iteration_ms(k), ...
            OUT.deadline_fail(k),OUT.commit(k));
    end

    fprintf('\nstatus = %s\n',OUT.status);

catch ME
    OUT.status='AUDIT_ERROR__NO_PRODUCTION_CHANGE';
    OUT.error=struct('identifier',ME.identifier,'message',ME.message,'stack',{ME.stack});
    fprintf('[AUDIT ERROR] %s | %s\n',ME.identifier,ME.message);
end

root=fileparts(which('REPLANNING_SYSTEM'));
if isempty(root), root=pwd; end
outdir=tempname(root); [ok,msg]=mkdir(outdir); assert(ok,'%s',msg);
report=fullfile(outdir,'AF7R58ZZQ_POSTRUN_REPORT.txt');
matp=fullfile(outdir,'AF7R58ZZQ_POSTRUN_OUT.mat');

fid=fopen(report,'wt','n','UTF-8'); assert(fid>=0);
fprintf(fid,'status=%s\n',OUT.status);
if isfield(OUT,'source_sha'), fprintf(fid,'sourceSHA=%s\n',OUT.source_sha); end
if isfield(OUT,'request_count'), fprintf(fid,'requests=%d\n',OUT.request_count); end
if isfield(OUT,'deadline_fail_count'), fprintf(fid,'deadlineFails=%.0f\n',OUT.deadline_fail_count); end
if isfield(OUT,'commit_count'), fprintf(fid,'commits=%.0f\n',OUT.commit_count); end
if isfield(OUT,'first_commit_request'), fprintf(fid,'firstCommitRequest=%s\n',numtxt(OUT.first_commit_request)); end
if isfield(OUT,'trigger_to_commit_s'), fprintf(fid,'triggerToCommitS=%s\n',numtxt(OUT.trigger_to_commit_s)); end
if isfield(OUT,'median_attempt_ms'), fprintf(fid,'attemptMedianMs=%.9g\n',OUT.median_attempt_ms); end
if isfield(OUT,'max_attempt_ms'), fprintf(fid,'attemptMaxMs=%.9g\n',OUT.max_attempt_ms); end
if isfield(OUT,'max_full_iteration_ms'), fprintf(fid,'fullIterationMaxMs=%.9g\n',OUT.max_full_iteration_ms); end
if isfield(OUT,'request_rows_over_25ms'), fprintf(fid,'requestRowsOver25ms=%d\n',OUT.request_rows_over_25ms); end
if isfield(OUT,'error')&&isstruct(OUT.error)&&isfield(OUT.error,'message')
    fprintf(fid,'errorIdentifier=%s\n',OUT.error.identifier);
    fprintf(fid,'errorMessage=%s\n',OUT.error.message);
end
fclose(fid);

OUT.report_path=report;
OUT.mat_path=matp;
save(matp,'OUT','-v7');
assignin('base','AF7R58ZZQ_POSTRUN_OUT',OUT);
fprintf('RETURN: %s\nRETURN: %s\n',report,matp);
end

function x=first_positive(v)
i=find(isfinite(v)&v>0,1,'first');
if isempty(i), x=NaN; else, x=i; end
end

function y=max_or_nan(x)
x=x(isfinite(x));
if isempty(x), y=NaN; else, y=max(x); end
end

function y=median_or_nan(x)
x=x(isfinite(x));
if isempty(x), y=NaN; else, y=median(x); end
end

function s=numtxt(x)
if isempty(x)||~isscalar(x)||~isfinite(x), s='NaN'; else, s=sprintf('%.17g',x); end
end

function n=count_exact(s,pat)
n=numel(strfind(s,pat));
end

function h=file_hash(path)
md=java.security.MessageDigest.getInstance('SHA-256');
fid=fopen(path,'rb'); assert(fid>=0); c=onCleanup(@()fclose(fid));
while true
    b=fread(fid,1024*1024,'*uint8');
    if isempty(b), break; end
    md.update(b);
end
u=typecast(md.digest(),'uint8');
h=lower(reshape(dec2hex(u,2).',1,[]));
end
