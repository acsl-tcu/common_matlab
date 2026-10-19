function OUT = AF7R58ZM_ONEFILE_REFERENCE_ONLY_TIMING_AUDIT()
% AF7R58ZM
% Read-only post-run audit for AF7R58ZL.
% Controller timing is intentionally excluded from classification.
%
% Best evidence path:
%   run_flight_headless_AF7R58ZH_R4_spike_audit
% then, WITHOUT CLEARING:
%   AF7R58ZM_ONEFILE_REFERENCE_ONLY_TIMING_AUDIT

EXPECTED_SHA='8ce3089c8a1baddd448d3896f7146d810527efc344f504d2da751d4a5bf53f73';

fprintf('\n====================================================================================================\n');
fprintf(' AF7R58ZM | REFERENCE/REPLANNER-ONLY POST-ZL TIMING AUDIT\n');
fprintf('====================================================================================================\n');
fprintf('[AUTHORITY] readOnly=1 | productionModified=0 | controllerExcluded=1 | empiricalTimingNotFormalWCET=1\n');

OUT=struct();
OUT.schema='AF7R58ZM_REFERENCE_ONLY_TIMING_AUDIT_V1';
OUT.production_modified=false;
OUT.status='OPEN';
OUT.error=struct();

try
    pR=which('REPLANNING_SYSTEM');
    assert(~isempty(pR)&&isfile(pR),'[AF7R58ZM] REPLANNING_SYSTEM unresolved.');
    OUT.source_path=pR;
    OUT.source_sha=file_hash(pR);
    OUT.source_exact=strcmp(OUT.source_sha,EXPECTED_SHA);

    assert(evalin('base','exist(''replanner_instance'',''var'')')==1, ...
        '[AF7R58ZM] replanner_instance missing. Run the scenario first.');
    R=evalin('base','replanner_instance');

    OUT.light_runtime_audit=logical(R.ENABLE_LIGHT_RUNTIME_AUDIT);
    OUT.hotpath_diagnostic_mirror=logical(R.ENABLE_HOTPATH_DIAGNOSTIC_MIRROR);
    OUT.legacy_timing_flags=[ ...
        logical(R.ENABLE_AF7R58M_TIMING_RING), ...
        logical(R.ENABLE_AF7R58Q_MIN_TIMING), ...
        logical(R.ENABLE_AF7R58T_OTHER_TIMING), ...
        logical(R.ENABLE_AF7R58V_R4_PRETRY_TIMING)];

    OUT.generator_requests=double(R.generator_request_count);
    OUT.generator_deadline_failures=double(R.generator_deadline_fail_count);
    OUT.replan_commits=double(R.replan_commit_count);
    OUT.reference_only_commits=double(R.reference_only_commit_count);
    OUT.last_status_code=double(R.generator_last_status_code);
    OUT.last_hard_cycle_ms=double(R.generator_last_cycle_critical_ms);
    OUT.last_callback_ms=double(R.generator_last_callback_ms);
    OUT.last_critical_ms=double(R.generator_last_critical_ms);

    OUT.cache_valid=logical(R.reference_only_prepared_packet_cache_valid);
    OUT.cache_stores=double(R.reference_only_prepared_packet_cache_store_count);
    OUT.cache_gate_passes=double(R.reference_only_prepared_packet_cache_gate_pass_count);
    OUT.cache_hits=double(R.reference_only_prepared_packet_cache_hit_count);
    OUT.cache_retargets=double(R.reference_only_prepared_packet_cache_retarget_count);
    OUT.cache_commits=double(R.reference_only_prepared_packet_cache_commit_count);
    OUT.cache_invalidates=double(R.reference_only_prepared_packet_cache_invalidate_count);
    OUT.cache_last_reason=double(R.reference_only_prepared_packet_cache_last_reason_code);
    OUT.cache_miss_reasons=double(R.reference_only_prepared_packet_cache_miss_reason_count);

    fprintf('\n[A. SOURCE / LIVE FLAGS]\n');
    fprintf(' source SHA exact                    = %d | %s\n',OUT.source_exact,OUT.source_sha);
    fprintf(' ENABLE_LIGHT_RUNTIME_AUDIT          = %d\n',OUT.light_runtime_audit);
    fprintf(' ENABLE_HOTPATH_DIAGNOSTIC_MIRROR    = %d\n',OUT.hotpath_diagnostic_mirror);
    fprintf(' legacy M/Q/T/V timing flags         = %s\n',mat2str(double(OUT.legacy_timing_flags)));

    fprintf('\n[B. GENERATOR HARD-DEADLINE COUNTERS]\n');
    fprintf(' generator requests                  = %.0f\n',OUT.generator_requests);
    fprintf(' deadline failures                   = %.0f\n',OUT.generator_deadline_failures);
    fprintf(' replan commits                      = %.0f\n',OUT.replan_commits);
    fprintf(' reference-only commits              = %.0f\n',OUT.reference_only_commits);
    fprintf(' last status code                    = %.0f\n',OUT.last_status_code);
    fprintf(' last hard-cycle critical [ms]       = %.6f\n',OUT.last_hard_cycle_ms);
    fprintf(' last callback [ms]                  = %.6f\n',OUT.last_callback_ms);
    fprintf(' last generator critical [ms]        = %.6f\n',OUT.last_critical_ms);

    fprintf('\n[C. PREPARED CACHE / CURRENT-SPEED RETARGET]\n');
    fprintf(' valid                               = %d\n',OUT.cache_valid);
    fprintf(' stores / gate-passes / hits         = %.0f / %.0f / %.0f\n', ...
        OUT.cache_stores,OUT.cache_gate_passes,OUT.cache_hits);
    fprintf(' current-speed exact retargets       = %.0f\n',OUT.cache_retargets);
    fprintf(' successful cache commits            = %.0f\n',OUT.cache_commits);
    fprintf(' invalidates                         = %.0f\n',OUT.cache_invalidates);
    fprintf(' last reason                         = %.0f\n',OUT.cache_last_reason);
    fprintf(' miss reason counts                  = %s\n',mat2str(OUT.cache_miss_reasons));

    hasR4=evalin('base','exist(''AF7R58ZH_R4_RUNTIME_PROFILE'',''var'')')==1;
    OUT.has_same_cycle_profile=hasR4;

    if hasR4
        P=evalin('base','AF7R58ZH_R4_RUNTIME_PROFILE');
        req={'t','phase','reference_s','gen_request_delta','gen_deadline_delta', ...
            'gen_commit_delta','gen_last_cycle_critical_ms','cache_store_delta', ...
            'cache_hit_delta','cache_invalidate_delta','cache_reason'};
        for k=1:numel(req)
            assert(isfield(P,req{k}),'[AF7R58ZM] R4 profile missing %s.',req{k});
        end

        flight=(P.phase=='f') & isfinite(P.reference_s);
        request=flight & (P.gen_request_delta>0);
        nonrequest=flight & ~(P.gen_request_delta>0);

        refms=1e3*P.reference_s(flight);
        OUT.reference_flight_samples=nnz(flight);
        OUT.reference_max_ms=max(refms);
        OUT.reference_over_4ms_count=nnz(refms>4);
        OUT.reference_over_25ms_count=nnz(refms>25);

        if any(request)
            OUT.request_count_profile=nnz(request);
            OUT.request_reference_max_ms=1e3*max(P.reference_s(request));
            OUT.request_reference_over_4ms_count=nnz(P.reference_s(request)>0.004);
            OUT.request_reference_over_25ms_count=nnz(P.reference_s(request)>0.025);
            hc=P.gen_last_cycle_critical_ms(request);
            hc=hc(isfinite(hc));
            if isempty(hc), OUT.request_hard_cycle_max_ms=NaN;
            else, OUT.request_hard_cycle_max_ms=max(hc); end
        else
            OUT.request_count_profile=0;
            OUT.request_reference_max_ms=NaN;
            OUT.request_reference_over_4ms_count=0;
            OUT.request_reference_over_25ms_count=0;
            OUT.request_hard_cycle_max_ms=NaN;
        end

        if any(nonrequest)
            OUT.nonrequest_reference_max_ms=1e3*max(P.reference_s(nonrequest));
            OUT.nonrequest_reference_over_25ms_count=nnz(P.reference_s(nonrequest)>0.025);
        else
            OUT.nonrequest_reference_max_ms=NaN;
            OUT.nonrequest_reference_over_25ms_count=0;
        end

        fprintf('\n[D. REFERENCE/REPLANNER STAGE ONLY -- CONTROLLER EXCLUDED]\n');
        fprintf(' Flight reference samples             = %d\n',OUT.reference_flight_samples);
        fprintf(' max reference stage [ms]             = %.6f\n',OUT.reference_max_ms);
        fprintf(' reference stage >4 ms                = %d\n',OUT.reference_over_4ms_count);
        fprintf(' reference stage >25 ms               = %d\n',OUT.reference_over_25ms_count);
        fprintf(' request cycles                        = %d\n',OUT.request_count_profile);
        fprintf(' max request reference [ms]            = %.6f\n',OUT.request_reference_max_ms);
        fprintf(' request reference >4 ms               = %d\n',OUT.request_reference_over_4ms_count);
        fprintf(' request reference >25 ms              = %d\n',OUT.request_reference_over_25ms_count);
        fprintf(' max request hard-cycle [ms]           = %.6f\n',OUT.request_hard_cycle_max_ms);
        fprintf(' max nonrequest reference [ms]         = %.6f\n',OUT.nonrequest_reference_max_ms);
        fprintf(' nonrequest reference >25 ms           = %d\n',OUT.nonrequest_reference_over_25ms_count);

        ridx=find(request);
        fprintf('\n[E. REAL REQUEST CYCLES]\n');
        fprintf(' t[s]   ref[ms] hard[ms] req fail commit cacheStore cacheHit cacheInv reason\n');
        for k=1:numel(ridx)
            j=ridx(k);
            fprintf('%6.3f %8.3f %8.3f %3.0f %4.0f %6.0f %10.0f %8.0f %8.0f %6.0f\n', ...
                P.t(j),1e3*P.reference_s(j),P.gen_last_cycle_critical_ms(j), ...
                P.gen_request_delta(j),P.gen_deadline_delta(j),P.gen_commit_delta(j), ...
                P.cache_store_delta(j),P.cache_hit_delta(j), ...
                P.cache_invalidate_delta(j),P.cache_reason(j));
        end
    else
        fprintf('\n[D. SAME-CYCLE PROFILE]\n');
        fprintf(' R4 same-cycle profile unavailable; hard-deadline counters only.\n');
    end

    flagsOff=~OUT.light_runtime_audit && ~OUT.hotpath_diagnostic_mirror && ...
        ~any(OUT.legacy_timing_flags);

    if ~OUT.source_exact
        OUT.status='SOURCE_NOT_EXACT_ZL__DO_NOT_INTERPRET_TIMING';
    elseif ~flagsOff
        OUT.status='DIAGNOSTIC_FLAGS_NOT_CLEAN__TIMING_NOT_FINAL';
    elseif OUT.generator_deadline_failures==0
        if hasR4 && OUT.request_reference_over_25ms_count==0 && ...
                OUT.nonrequest_reference_over_25ms_count==0
            OUT.status='HARD_4MS_EMPIRICALLY_CLOSED_THIS_RUN__REFERENCE_STAGE_WITHIN_25MS';
        else
            OUT.status='HARD_4MS_EMPIRICALLY_CLOSED_THIS_RUN__REFERENCE_STAGE_CONTROL_BUDGET_REVIEW';
        end
    elseif OUT.cache_retargets>0
        OUT.status='CURRENT_SPEED_RETARGET_ACTIVE__HARD_4MS_STILL_OPEN';
    elseif OUT.cache_stores>0 && OUT.cache_hits==0
        OUT.status='CACHE_STORED_BUT_RETARGET_NOT_REACHED__CHECK_MISS_REASON';
    else
        OUT.status='HARD_4MS_STILL_OPEN__NO_RETARGET_EVIDENCE';
    end

    fprintf('\n[F. CLASSIFICATION]\n');
    fprintf(' status = %s\n',OUT.status);
    fprintf(' NOTE: empirical timing is not a formal OS/WCET theorem.\n');

catch ME
    OUT.status='AUDIT_ERROR__NO_PRODUCTION_CHANGE';
    OUT.error=struct('identifier',ME.identifier,'message',ME.message,'stack',{ME.stack});
    fprintf('\n[AUDIT ERROR] %s | %s\n',ME.identifier,ME.message);
end

pR=which('REPLANNING_SYSTEM');
root=fileparts(pR);
outdir=tempname(root);
[ok,msg]=mkdir(outdir); assert(ok,'%s',msg);
report=fullfile(outdir,'AF7R58ZM_REPORT.txt');
matp=fullfile(outdir,'AF7R58ZM_OUT.mat');

fid=fopen(report,'wt','n','UTF-8'); assert(fid>=0);
fprintf(fid,'====================================================================================================\n');
fprintf(fid,' AF7R58ZM | REFERENCE/REPLANNER-ONLY POST-ZL TIMING AUDIT\n');
fprintf(fid,'====================================================================================================\n');
fprintf(fid,'status = %s\n',OUT.status);
if isfield(OUT,'source_sha'), fprintf(fid,'source SHA = %s\n',OUT.source_sha); end
if isfield(OUT,'generator_requests')
    fprintf(fid,'generator requests = %.0f\n',OUT.generator_requests);
    fprintf(fid,'deadline failures = %.0f\n',OUT.generator_deadline_failures);
    fprintf(fid,'replan commits = %.0f\n',OUT.replan_commits);
    fprintf(fid,'cache stores/gate/hits/retarget/invalidates = %.0f / %.0f / %.0f / %.0f / %.0f\n', ...
        OUT.cache_stores,OUT.cache_gate_passes,OUT.cache_hits,OUT.cache_retargets,OUT.cache_invalidates);
end
if isfield(OUT,'reference_max_ms')
    fprintf(fid,'max reference stage ms = %.9g\n',OUT.reference_max_ms);
    fprintf(fid,'max request reference ms = %.9g\n',OUT.request_reference_max_ms);
    fprintf(fid,'max request hard-cycle ms = %.9g\n',OUT.request_hard_cycle_max_ms);
    fprintf(fid,'max nonrequest reference ms = %.9g\n',OUT.nonrequest_reference_max_ms);
    fprintf(fid,'reference >25ms = %d\n',OUT.reference_over_25ms_count);
end
fprintf(fid,'empirical timing is not formal WCET.\n');
fclose(fid);

OUT.report_path=report;
OUT.mat_path=matp;
save(matp,'OUT','-v7');
assignin('base','AF7R58ZM_OUT',OUT);

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
