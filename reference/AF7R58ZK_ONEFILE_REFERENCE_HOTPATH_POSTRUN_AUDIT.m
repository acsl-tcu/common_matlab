function OUT = AF7R58ZK_ONEFILE_REFERENCE_HOTPATH_POSTRUN_AUDIT()
% AF7R58ZK
% Read-only post-run audit after AF7R58ZJ.
%
% Works after either:
%   run_flight_headless
% or
%   run_flight_headless_AF7R58ZH_R4_spike_audit
%
% If AF7R58ZH_R4_RUNTIME_PROFILE exists, stage-specific reference evidence
% is reported. Otherwise standard runtime_profile outer-cycle evidence is
% reported.

fprintf('\n====================================================================================================\n');
fprintf(' AF7R58ZK | REFERENCE HOT-PATH POST-RUN AUDIT\n');
fprintf('====================================================================================================\n');
fprintf('[AUTHORITY] readOnly=1 | productionModified=0 | controllerJudgementExcluded=1\n');

OUT=struct();
OUT.schema='AF7R58ZK_REFERENCE_HOTPATH_POSTRUN_V1';
OUT.status='OPEN';
OUT.error=struct();

try
    assert(evalin('base','exist(''replanner_instance'',''var'')')==1, ...
        '[AF7R58ZK] replanner_instance missing.');
    R=evalin('base','replanner_instance');

    OUT.light_runtime_audit=logical(R.ENABLE_LIGHT_RUNTIME_AUDIT);
    OUT.hotpath_diagnostic_mirror=logical(R.ENABLE_HOTPATH_DIAGNOSTIC_MIRROR);
    OUT.generator_requests=double(R.generator_request_count);
    OUT.generator_deadline_failures=double(R.generator_deadline_fail_count);
    OUT.replan_commits=double(R.replan_commit_count);

    fprintf('\n[A. LIVE PRODUCTION FLAGS]\n');
    fprintf(' ENABLE_LIGHT_RUNTIME_AUDIT       = %d\n',OUT.light_runtime_audit);
    fprintf(' ENABLE_HOTPATH_DIAGNOSTIC_MIRROR = %d\n',OUT.hotpath_diagnostic_mirror);
    fprintf(' generator requests               = %.0f\n',OUT.generator_requests);
    fprintf(' deadline failures                = %.0f\n',OUT.generator_deadline_failures);
    fprintf(' replan commits                   = %.0f\n',OUT.replan_commits);

    if evalin('base','exist(''AF7R58ZH_R4_RUNTIME_PROFILE'',''var'')')==1
        P=evalin('base','AF7R58ZH_R4_RUNTIME_PROFILE');
        mask=(P.phase=='f') & isfinite(P.reference_s);
        refms=1e3*P.reference_s(mask);
        OUT.reference_samples=numel(refms);
        OUT.reference_max_ms=max(refms);
        OUT.reference_over_4ms_count=nnz(refms>4);

        reqMask=mask & P.gen_request_delta>0;
        nonReqMask=mask & ~(P.gen_request_delta>0);

        if any(reqMask)
            OUT.request_reference_max_ms=1e3*max(P.reference_s(reqMask));
        else
            OUT.request_reference_max_ms=NaN;
        end
        if any(nonReqMask)
            OUT.nonrequest_reference_max_ms=1e3*max(P.reference_s(nonReqMask));
        else
            OUT.nonrequest_reference_max_ms=NaN;
        end

        fprintf('\n[B. REFERENCE / REPLANNER SAME-CYCLE STAGE]\n');
        fprintf(' samples                       = %d\n',OUT.reference_samples);
        fprintf(' max reference [ms]            = %.6f\n',OUT.reference_max_ms);
        fprintf(' reference cycles >4 ms        = %d\n',OUT.reference_over_4ms_count);
        fprintf(' max request-reference [ms]    = %.6f\n',OUT.request_reference_max_ms);
        fprintf(' max nonrequest-reference [ms] = %.6f\n',OUT.nonrequest_reference_max_ms);

        if OUT.reference_over_4ms_count==0
            OUT.status='REFERENCE_STAGE_ALL_SAMPLES_WITHIN_4MS__FINAL_CLEAN_TIMING_NEXT';
        elseif OUT.request_reference_max_ms>4 && OUT.nonrequest_reference_max_ms<=4
            OUT.status='BASELINE_FIXED__REQUEST_PATH_STILL_OVER_4MS';
        elseif OUT.nonrequest_reference_max_ms>4
            OUT.status='NONREQUEST_REFERENCE_PATH_STILL_OVER_4MS';
        else
            OUT.status='REFERENCE_TIMING_STILL_OPEN';
        end
    else
        assert(evalin('base','exist(''runtime_profile'',''var'')')==1, ...
            '[AF7R58ZK] neither R4 profile nor runtime_profile exists.');
        P=evalin('base','runtime_profile');
        mask=(P.phase=='f') & isfinite(P.online_before_plant_s);
        OUT.online_samples=nnz(mask);
        OUT.online_max_ms=1e3*max(P.online_before_plant_s(mask));
        OUT.online_over_25ms_count=nnz(P.online_before_plant_s(mask)>0.025);

        fprintf('\n[B. STANDARD OUTER-CYCLE SCREEN]\n');
        fprintf(' Flight samples           = %d\n',OUT.online_samples);
        fprintf(' max online [ms]          = %.6f\n',OUT.online_max_ms);
        fprintf(' online cycles >25 ms     = %d\n',OUT.online_over_25ms_count);
        OUT.status='STANDARD_RUN_ONLY__USE_R4_COPY_FOR_REFERENCE_STAGE_4MS';
    end

    fprintf('\n[C. CLASSIFICATION]\n');
    fprintf(' status = %s\n',OUT.status);

catch ME
    OUT.status='AUDIT_ERROR__NO_PRODUCTION_CHANGE';
    OUT.error=struct('identifier',ME.identifier,'message',ME.message,'stack',{ME.stack});
    fprintf('\n[AUDIT ERROR] %s | %s\n',ME.identifier,ME.message);
end

root=fileparts(which('run_flight_headless'));
outdir=tempname(root);
[ok,msg]=mkdir(outdir); assert(ok,'%s',msg);
report=fullfile(outdir,'AF7R58ZK_REPORT.txt');
matp=fullfile(outdir,'AF7R58ZK_OUT.mat');

fid=fopen(report,'wt','n','UTF-8'); assert(fid>=0);
fprintf(fid,'AF7R58ZK status = %s\n',OUT.status);
if isfield(OUT,'reference_max_ms')
    fprintf(fid,'reference max ms = %.9g\n',OUT.reference_max_ms);
    fprintf(fid,'reference >4 ms count = %d\n',OUT.reference_over_4ms_count);
    fprintf(fid,'request reference max ms = %.9g\n',OUT.request_reference_max_ms);
    fprintf(fid,'nonrequest reference max ms = %.9g\n',OUT.nonrequest_reference_max_ms);
end
if isfield(OUT,'online_max_ms')
    fprintf(fid,'outer online max ms = %.9g\n',OUT.online_max_ms);
    fprintf(fid,'outer online >25 ms count = %d\n',OUT.online_over_25ms_count);
end
fprintf(fid,'generator requests = %.0f\n',OUT.generator_requests);
fprintf(fid,'generator deadline failures = %.0f\n',OUT.generator_deadline_failures);
fprintf(fid,'replan commits = %.0f\n',OUT.replan_commits);
fclose(fid);

OUT.report_path=report;
OUT.mat_path=matp;
save(matp,'OUT','-v7');
assignin('base','AF7R58ZK_OUT',OUT);

fprintf('\nRETURN: %s\n',report);
fprintf('RETURN: %s\n',matp);
end
