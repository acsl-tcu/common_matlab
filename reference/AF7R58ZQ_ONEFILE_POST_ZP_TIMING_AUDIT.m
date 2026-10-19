function OUT = AF7R58ZQ_ONEFILE_POST_ZP_TIMING_AUDIT()
% AF7R58ZQ
% Read-only audit after AF7R58ZP.
% Controller timing is excluded by design.

EXPECTED_SHA='afdab70a7a68f545dac3dede9814c95b703cb7823b9da5374ed05004735a1c11';

fprintf('\n====================================================================================================\n');
fprintf(' AF7R58ZQ | POST-ZP SENSOR-TO-TARGET + REFERENCE TIMING AUDIT\n');
fprintf('====================================================================================================\n');
fprintf('[AUTHORITY] readOnly=1 | controllerExcluded=1 | sensorToTargetHardClock=1 | empiricalNotFormalWCET=1\n');

OUT=struct();
OUT.schema='AF7R58ZQ_POST_ZP_TIMING_V1';
OUT.production_modified=false;
OUT.status='OPEN';
OUT.error=struct();

try
    pR=which('REPLANNING_SYSTEM');
    assert(~isempty(pR)&&isfile(pR));
    OUT.source_sha=file_hash(pR);
    OUT.source_exact=strcmp(OUT.source_sha,EXPECTED_SHA);

    assert(evalin('base','exist(''replanner_instance'',''var'')')==1, ...
        '[AF7R58ZQ] replanner_instance missing.');
    R=evalin('base','replanner_instance');

    OUT.requests=double(R.generator_request_count);
    OUT.deadline_failures=double(R.generator_deadline_fail_count);
    OUT.commits=double(R.replan_commit_count);
    OUT.last_hard_cycle_ms=double(R.generator_last_cycle_critical_ms);
    OUT.last_elapsed_before_ms=double(R.generator_last_elapsed_before_ms);
    OUT.last_context_ms=double(R.generator_last_context_ms);
    OUT.last_callback_ms=double(R.generator_last_callback_ms);
    OUT.last_critical_ms=double(R.generator_last_critical_ms);

    OUT.cache_stores=double(R.reference_only_prepared_packet_cache_store_count);
    OUT.cache_hits=double(R.reference_only_prepared_packet_cache_hit_count);
    OUT.cache_retargets=double(R.reference_only_prepared_packet_cache_retarget_count);
    OUT.cache_invalidates=double(R.reference_only_prepared_packet_cache_invalidate_count);

    O=R.observer_hard_support_internal;
    if isstruct(O) && isfield(O,'runtime_max_ms')
        OUT.observer_runtime_max_ms=double(O.runtime_max_ms);
    else
        OUT.observer_runtime_max_ms=NaN;
    end

    flags=[logical(R.ENABLE_LIGHT_RUNTIME_AUDIT), ...
        logical(R.ENABLE_HOTPATH_DIAGNOSTIC_MIRROR), ...
        logical(R.ENABLE_AF7R58M_TIMING_RING), ...
        logical(R.ENABLE_AF7R58Q_MIN_TIMING), ...
        logical(R.ENABLE_AF7R58T_OTHER_TIMING), ...
        logical(R.ENABLE_AF7R58V_R4_PRETRY_TIMING)];
    OUT.diagnostic_flags=flags;

    fprintf('\n[A. SOURCE / REQUEST COUNTERS]\n');
    fprintf(' source exact                       = %d | %s\n',OUT.source_exact,OUT.source_sha);
    fprintf(' diagnostic flags                   = %s\n',mat2str(double(flags)));
    fprintf(' requests / deadline failures       = %.0f / %.0f\n',OUT.requests,OUT.deadline_failures);
    fprintf(' commits                            = %.0f\n',OUT.commits);
    fprintf(' last sensor->target hard cycle [ms]= %.6f\n',OUT.last_hard_cycle_ms);
    fprintf(' last elapsed before generator [ms] = %.6f\n',OUT.last_elapsed_before_ms);
    fprintf(' last context / callback [ms]        = %.6f / %.6f\n',OUT.last_context_ms,OUT.last_callback_ms);
    fprintf(' cache store/hit/retarget/invalidate = %.0f / %.0f / %.0f / %.0f\n', ...
        OUT.cache_stores,OUT.cache_hits,OUT.cache_retargets,OUT.cache_invalidates);
    fprintf(' observer runtime max [ms]           = %.6f\n',OUT.observer_runtime_max_ms);

    assert(evalin('base','exist(''AF7R58ZH_R4_RUNTIME_PROFILE'',''var'')')==1, ...
        '[AF7R58ZQ] run the R4 diagnostic copy immediately before this audit.');
    P=evalin('base','AF7R58ZH_R4_RUNTIME_PROFILE');

    flight=(P.phase=='f') & isfinite(P.reference_s);
    request=flight & P.gen_request_delta>0;
    nonrequest=flight & ~request;

    OUT.reference_max_ms=1e3*max(P.reference_s(flight));
    OUT.reference_over_25ms=nnz(P.reference_s(flight)>0.025);
    OUT.nonrequest_reference_max_ms=1e3*max(P.reference_s(nonrequest));
    OUT.nonrequest_reference_over_25ms=nnz(P.reference_s(nonrequest)>0.025);

    ridx=find(request);
    n=numel(ridx);
    OUT.request_t=zeros(1,n);
    OUT.request_reference_ms=zeros(1,n);
    OUT.request_hard_ms=zeros(1,n);
    OUT.request_fail=zeros(1,n);
    OUT.request_commit=zeros(1,n);
    for k=1:n
        j=ridx(k);
        OUT.request_t(k)=double(P.t(j));
        OUT.request_reference_ms(k)=1e3*double(P.reference_s(j));
        OUT.request_hard_ms(k)=double(P.gen_last_cycle_critical_ms(j));
        OUT.request_fail(k)=double(P.gen_deadline_delta(j));
        OUT.request_commit(k)=double(P.gen_commit_delta(j));
    end

    if n>0
        OUT.request_hard_max_ms=max(OUT.request_hard_ms);
        OUT.request_hard_over_4ms=nnz(OUT.request_hard_ms>4);
        OUT.request_reference_max_ms=max(OUT.request_reference_ms);
        OUT.request_reference_over_25ms=nnz(OUT.request_reference_ms>25);
    else
        OUT.request_hard_max_ms=NaN;
        OUT.request_hard_over_4ms=0;
        OUT.request_reference_max_ms=NaN;
        OUT.request_reference_over_25ms=0;
    end

    fprintf('\n[B. REQUEST-BY-REQUEST -- CONTROLLER EXCLUDED]\n');
    fprintf(' t[s]   reference[ms] sensor->target[ms] deadlineFail commit\n');
    for k=1:n
        fprintf('%6.3f %13.3f %18.3f %12.0f %6.0f\n', ...
            OUT.request_t(k),OUT.request_reference_ms(k),OUT.request_hard_ms(k), ...
            OUT.request_fail(k),OUT.request_commit(k));
    end

    fprintf('\n[C. SUMMARY]\n');
    fprintf(' max sensor->target request [ms] = %.6f\n',OUT.request_hard_max_ms);
    fprintf(' request hard cycles >4 ms       = %d / %d\n',OUT.request_hard_over_4ms,n);
    fprintf(' max request reference [ms]      = %.6f\n',OUT.request_reference_max_ms);
    fprintf(' max nonrequest reference [ms]   = %.6f\n',OUT.nonrequest_reference_max_ms);
    fprintf(' all reference cycles >25 ms     = %d\n',OUT.reference_over_25ms);
    fprintf(' nonrequest reference >25 ms     = %d\n',OUT.nonrequest_reference_over_25ms);

    clean=OUT.source_exact && ~any(flags);
    if ~clean
        OUT.status='SOURCE_OR_DIAGNOSTIC_FLAGS_NOT_CLEAN';
    elseif OUT.deadline_failures==0 && OUT.request_hard_over_4ms==0 && ...
            OUT.reference_over_25ms==0
        OUT.status='SENSOR_TO_TARGET_4MS_AND_REFERENCE_25MS_EMPIRICALLY_CLOSED_THIS_RUN';
    elseif OUT.deadline_failures==0 && OUT.request_hard_over_4ms==0
        OUT.status='SENSOR_TO_TARGET_4MS_CLOSED__REFERENCE_25MS_SPIKE_REMAINS';
    elseif OUT.request_hard_over_4ms>0
        OUT.status='SENSOR_TO_TARGET_4MS_STILL_OPEN';
    else
        OUT.status='TIMING_STILL_OPEN';
    end

    fprintf('\n[D. CLASSIFICATION]\n');
    fprintf(' status = %s\n',OUT.status);
    fprintf(' empirical timing is not a formal OS/WCET theorem.\n');

catch ME
    OUT.status='AUDIT_ERROR__NO_PRODUCTION_CHANGE';
    OUT.error=struct('identifier',ME.identifier,'message',ME.message,'stack',{ME.stack});
    fprintf('\n[AUDIT ERROR] %s | %s\n',ME.identifier,ME.message);
end

root=fileparts(which('REPLANNING_SYSTEM'));
outdir=tempname(root);
[ok,msg]=mkdir(outdir); assert(ok,'%s',msg);
report=fullfile(outdir,'AF7R58ZQ_REPORT.txt');
matp=fullfile(outdir,'AF7R58ZQ_OUT.mat');

fid=fopen(report,'wt','n','UTF-8'); assert(fid>=0);
fprintf(fid,'====================================================================================================\n');
fprintf(fid,' AF7R58ZQ | POST-ZP SENSOR-TO-TARGET + REFERENCE TIMING AUDIT\n');
fprintf(fid,'====================================================================================================\n');
fprintf(fid,'status = %s\n',OUT.status);
if isfield(OUT,'source_sha'), fprintf(fid,'source SHA = %s\n',OUT.source_sha); end
if isfield(OUT,'requests')
    fprintf(fid,'requests / deadline failures / commits = %.0f / %.0f / %.0f\n', ...
        OUT.requests,OUT.deadline_failures,OUT.commits);
    fprintf(fid,'cache store/hit/retarget/invalidate = %.0f / %.0f / %.0f / %.0f\n', ...
        OUT.cache_stores,OUT.cache_hits,OUT.cache_retargets,OUT.cache_invalidates);
end
if isfield(OUT,'request_hard_max_ms')
    fprintf(fid,'max sensor->target request ms = %.9g\n',OUT.request_hard_max_ms);
    fprintf(fid,'request hard >4ms = %d\n',OUT.request_hard_over_4ms);
    fprintf(fid,'max request reference ms = %.9g\n',OUT.request_reference_max_ms);
    fprintf(fid,'max nonrequest reference ms = %.9g\n',OUT.nonrequest_reference_max_ms);
    fprintf(fid,'all reference >25ms = %d\n',OUT.reference_over_25ms);
    fprintf(fid,'nonrequest reference >25ms = %d\n',OUT.nonrequest_reference_over_25ms);
    fprintf(fid,'observer runtime max ms = %.9g\n',OUT.observer_runtime_max_ms);
    fprintf(fid,'\nrequest table:\n');
    fprintf(fid,'t_s,reference_ms,sensor_to_target_ms,deadline_fail,commit\n');
    for k=1:numel(OUT.request_t)
        fprintf(fid,'%.9g,%.9g,%.9g,%.0f,%.0f\n', ...
            OUT.request_t(k),OUT.request_reference_ms(k),OUT.request_hard_ms(k), ...
            OUT.request_fail(k),OUT.request_commit(k));
    end
end
fprintf(fid,'empirical timing is not formal WCET.\n');
fclose(fid);

OUT.report_path=report;
OUT.mat_path=matp;
save(matp,'OUT','-v7');
assignin('base','AF7R58ZQ_OUT',OUT);
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
