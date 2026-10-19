function OUT = AF7R58ZO_ONEFILE_POST_ZN_REFERENCE_TIMING_AUDIT()
% AF7R58ZO
% Read-only timing audit after AF7R58ZN.
% Controller is intentionally excluded from all pass/fail classification.

EXPECTED_SHA='eb757924038c33a9005e521bf080c1436786d186234f10e09ec146c7211acbf6';

fprintf('\n====================================================================================================\n');
fprintf(' AF7R58ZO | POST-ZN REFERENCE/REPLANNER TIMING AUDIT\n');
fprintf('====================================================================================================\n');
fprintf('[AUTHORITY] readOnly=1 | controllerExcluded=1 | empiricalTimingNotFormalWCET=1\n');

OUT=struct();
OUT.schema='AF7R58ZO_POST_ZN_REFERENCE_TIMING_V1';
OUT.production_modified=false;
OUT.status='OPEN';
OUT.error=struct();

try
    pR=which('REPLANNING_SYSTEM');
    assert(~isempty(pR)&&isfile(pR));
    OUT.source_sha=file_hash(pR);
    OUT.source_exact=strcmp(OUT.source_sha,EXPECTED_SHA);

    assert(evalin('base','exist(''replanner_instance'',''var'')')==1, ...
        '[AF7R58ZO] replanner_instance missing.');
    R=evalin('base','replanner_instance');

    OUT.requests=double(R.generator_request_count);
    OUT.deadline_failures=double(R.generator_deadline_fail_count);
    OUT.commits=double(R.replan_commit_count);
    OUT.reference_only_commits=double(R.reference_only_commit_count);
    OUT.last_status_code=double(R.generator_last_status_code);
    OUT.last_hard_cycle_ms=double(R.generator_last_cycle_critical_ms);
    OUT.last_callback_ms=double(R.generator_last_callback_ms);
    OUT.last_critical_ms=double(R.generator_last_critical_ms);
    OUT.cache_stores=double(R.reference_only_prepared_packet_cache_store_count);
    OUT.cache_gate=double(R.reference_only_prepared_packet_cache_gate_pass_count);
    OUT.cache_hits=double(R.reference_only_prepared_packet_cache_hit_count);
    OUT.cache_retargets=double(R.reference_only_prepared_packet_cache_retarget_count);
    OUT.cache_invalidates=double(R.reference_only_prepared_packet_cache_invalidate_count);

    flags=[logical(R.ENABLE_LIGHT_RUNTIME_AUDIT), ...
        logical(R.ENABLE_HOTPATH_DIAGNOSTIC_MIRROR), ...
        logical(R.ENABLE_AF7R58M_TIMING_RING), ...
        logical(R.ENABLE_AF7R58Q_MIN_TIMING), ...
        logical(R.ENABLE_AF7R58T_OTHER_TIMING), ...
        logical(R.ENABLE_AF7R58V_R4_PRETRY_TIMING)];
    OUT.diagnostic_flags=flags;

    fprintf('\n[A. LIVE]\n');
    fprintf(' source exact              = %d | %s\n',OUT.source_exact,OUT.source_sha);
    fprintf(' diagnostic flags          = %s\n',mat2str(double(flags)));
    fprintf(' requests / deadline fails = %.0f / %.0f\n',OUT.requests,OUT.deadline_failures);
    fprintf(' commits                   = %.0f\n',OUT.commits);
    fprintf(' cache store/gate/hit/retarget/invalidate = %.0f / %.0f / %.0f / %.0f / %.0f\n', ...
        OUT.cache_stores,OUT.cache_gate,OUT.cache_hits,OUT.cache_retargets,OUT.cache_invalidates);
    fprintf(' last hard/callback/critical [ms] = %.6f / %.6f / %.6f\n', ...
        OUT.last_hard_cycle_ms,OUT.last_callback_ms,OUT.last_critical_ms);

    assert(evalin('base','exist(''AF7R58ZH_R4_RUNTIME_PROFILE'',''var'')')==1, ...
        '[AF7R58ZO] run R4 diagnostic copy immediately before this audit.');
    P=evalin('base','AF7R58ZH_R4_RUNTIME_PROFILE');

    flight=(P.phase=='f') & isfinite(P.reference_s);
    request=flight & P.gen_request_delta>0;
    nonrequest=flight & ~request;

    OUT.flight_count=nnz(flight);
    OUT.reference_max_ms=1e3*max(P.reference_s(flight));
    OUT.reference_over_25ms=nnz(P.reference_s(flight)>0.025);
    OUT.nonrequest_reference_max_ms=1e3*max(P.reference_s(nonrequest));
    OUT.nonrequest_reference_over_25ms=nnz(P.reference_s(nonrequest)>0.025);

    ridx=find(request);
    n=numel(ridx);
    OUT.request=struct();
    OUT.request.t_s=zeros(1,n);
    OUT.request.reference_ms=zeros(1,n);
    OUT.request.hard_cycle_ms=zeros(1,n);
    OUT.request.request_delta=zeros(1,n);
    OUT.request.deadline_delta=zeros(1,n);
    OUT.request.commit_delta=zeros(1,n);
    OUT.request.cache_store_delta=zeros(1,n);
    OUT.request.cache_hit_delta=zeros(1,n);
    OUT.request.cache_invalidate_delta=zeros(1,n);
    OUT.request.cache_reason=zeros(1,n);

    for k=1:n
        j=ridx(k);
        OUT.request.t_s(k)=double(P.t(j));
        OUT.request.reference_ms(k)=1e3*double(P.reference_s(j));
        OUT.request.hard_cycle_ms(k)=double(P.gen_last_cycle_critical_ms(j));
        OUT.request.request_delta(k)=double(P.gen_request_delta(j));
        OUT.request.deadline_delta(k)=double(P.gen_deadline_delta(j));
        OUT.request.commit_delta(k)=double(P.gen_commit_delta(j));
        OUT.request.cache_store_delta(k)=double(P.cache_store_delta(j));
        OUT.request.cache_hit_delta(k)=double(P.cache_hit_delta(j));
        OUT.request.cache_invalidate_delta(k)=double(P.cache_invalidate_delta(j));
        OUT.request.cache_reason(k)=double(P.cache_reason(j));
    end

    if n>0
        OUT.request_reference_max_ms=max(OUT.request.reference_ms);
        OUT.request_hard_max_ms=max(OUT.request.hard_cycle_ms);
        OUT.request_hard_over_4ms=nnz(OUT.request.hard_cycle_ms>4);
        OUT.request_reference_over_25ms=nnz(OUT.request.reference_ms>25);
    else
        OUT.request_reference_max_ms=NaN;
        OUT.request_hard_max_ms=NaN;
        OUT.request_hard_over_4ms=0;
        OUT.request_reference_over_25ms=0;
    end

    fprintf('\n[B. REFERENCE STAGE -- CONTROLLER EXCLUDED]\n');
    fprintf(' Flight samples                    = %d\n',OUT.flight_count);
    fprintf(' max reference [ms]                = %.6f\n',OUT.reference_max_ms);
    fprintf(' reference >25 ms                  = %d\n',OUT.reference_over_25ms);
    fprintf(' max nonrequest reference [ms]     = %.6f\n',OUT.nonrequest_reference_max_ms);
    fprintf(' nonrequest reference >25 ms       = %d\n',OUT.nonrequest_reference_over_25ms);

    fprintf('\n[C. REQUEST-BY-REQUEST]\n');
    fprintf(' t[s]    ref[ms]  hard[ms] fail commit cacheStore cacheHit cacheInv reason\n');
    for k=1:n
        fprintf('%6.3f %8.3f %9.3f %4.0f %6.0f %10.0f %8.0f %8.0f %6.0f\n', ...
            OUT.request.t_s(k),OUT.request.reference_ms(k), ...
            OUT.request.hard_cycle_ms(k),OUT.request.deadline_delta(k), ...
            OUT.request.commit_delta(k),OUT.request.cache_store_delta(k), ...
            OUT.request.cache_hit_delta(k),OUT.request.cache_invalidate_delta(k), ...
            OUT.request.cache_reason(k));
    end

    fprintf('\n[D. REQUEST SUMMARY]\n');
    fprintf(' request count                     = %d\n',n);
    fprintf(' max request reference [ms]        = %.6f\n',OUT.request_reference_max_ms);
    fprintf(' max request hard-cycle [ms]       = %.6f\n',OUT.request_hard_max_ms);
    fprintf(' request hard-cycle >4 ms          = %d\n',OUT.request_hard_over_4ms);
    fprintf(' request reference >25 ms          = %d\n',OUT.request_reference_over_25ms);

    cleanFlags=~any(flags);
    if ~OUT.source_exact
        OUT.status='SOURCE_NOT_EXACT_ZN';
    elseif ~cleanFlags
        OUT.status='DIAGNOSTIC_FLAGS_NOT_CLEAN';
    elseif OUT.deadline_failures==0 && OUT.request_hard_over_4ms==0 && ...
            OUT.reference_over_25ms==0
        OUT.status='POST_ZN_EMPIRICAL_TIMING_CLOSED_THIS_RUN';
    elseif OUT.deadline_failures==0 && OUT.request_hard_over_4ms==0
        OUT.status='HARD_4MS_CLOSED__REFERENCE_CONTROL_PERIOD_SPIKE_REMAINS';
    elseif OUT.request_hard_over_4ms>0
        OUT.status='HARD_4MS_STILL_OPEN';
    else
        OUT.status='TIMING_OPEN';
    end

    fprintf('\n[E. CLASSIFICATION]\n');
    fprintf(' status = %s\n',OUT.status);
    fprintf(' empirical result is not a formal OS/WCET theorem.\n');

catch ME
    OUT.status='AUDIT_ERROR__NO_PRODUCTION_CHANGE';
    OUT.error=struct('identifier',ME.identifier,'message',ME.message,'stack',{ME.stack});
    fprintf('\n[AUDIT ERROR] %s | %s\n',ME.identifier,ME.message);
end

root=fileparts(which('REPLANNING_SYSTEM'));
outdir=tempname(root);
[ok,msg]=mkdir(outdir); assert(ok,'%s',msg);
report=fullfile(outdir,'AF7R58ZO_REPORT.txt');
matp=fullfile(outdir,'AF7R58ZO_OUT.mat');

fid=fopen(report,'wt','n','UTF-8'); assert(fid>=0);
fprintf(fid,'====================================================================================================\n');
fprintf(fid,' AF7R58ZO | POST-ZN REFERENCE/REPLANNER TIMING AUDIT\n');
fprintf(fid,'====================================================================================================\n');
fprintf(fid,'status = %s\n',OUT.status);
if isfield(OUT,'source_sha'), fprintf(fid,'source SHA = %s\n',OUT.source_sha); end
if isfield(OUT,'requests')
    fprintf(fid,'requests / deadline failures / commits = %.0f / %.0f / %.0f\n', ...
        OUT.requests,OUT.deadline_failures,OUT.commits);
end
if isfield(OUT,'request_hard_max_ms')
    fprintf(fid,'max request hard-cycle ms = %.9g\n',OUT.request_hard_max_ms);
    fprintf(fid,'request hard-cycle >4ms = %d\n',OUT.request_hard_over_4ms);
    fprintf(fid,'max request reference ms = %.9g\n',OUT.request_reference_max_ms);
    fprintf(fid,'max nonrequest reference ms = %.9g\n',OUT.nonrequest_reference_max_ms);
    fprintf(fid,'reference >25ms = %d\n',OUT.reference_over_25ms);
    fprintf(fid,'nonrequest reference >25ms = %d\n',OUT.nonrequest_reference_over_25ms);
    fprintf(fid,'\nrequest table:\n');
    fprintf(fid,'t_s,reference_ms,hard_cycle_ms,deadline_delta,commit_delta,cache_store,cache_hit,cache_invalidate,reason\n');
    for k=1:numel(OUT.request.t_s)
        fprintf(fid,'%.9g,%.9g,%.9g,%.0f,%.0f,%.0f,%.0f,%.0f,%.0f\n', ...
            OUT.request.t_s(k),OUT.request.reference_ms(k), ...
            OUT.request.hard_cycle_ms(k),OUT.request.deadline_delta(k), ...
            OUT.request.commit_delta(k),OUT.request.cache_store_delta(k), ...
            OUT.request.cache_hit_delta(k),OUT.request.cache_invalidate_delta(k), ...
            OUT.request.cache_reason(k));
    end
end
fprintf(fid,'empirical timing is not formal WCET.\n');
fclose(fid);

OUT.report_path=report;
OUT.mat_path=matp;
save(matp,'OUT','-v7');
assignin('base','AF7R58ZO_OUT',OUT);
fprintf('\nRETURN: %s\n',report);
fprintf('RETURN: %s\n',matp);
end


function h=file_hash(p)
fid=fopen(p,'rb'); assert(fid>=0);
c=onCleanup(@()fclose(fid)); %#ok<NASGU>
b=fread(fid,Inf,'*uint8');
m=java.security.MessageDigest.getInstance('SHA-256');
m.update(b);
d=typecast(m.digest(),'uint8');
h=lower(reshape(dec2hex(d,2).',1,[]));
end
