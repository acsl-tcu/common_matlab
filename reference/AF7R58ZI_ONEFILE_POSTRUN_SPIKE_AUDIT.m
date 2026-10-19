function OUT = AF7R58ZI_ONEFILE_POSTRUN_SPIKE_AUDIT()
% AF7R58ZI
% Read-only post-run analysis of AF7R58ZH-R4 diagnostic copy.
%
% Requires the just-finished diagnostic run to leave:
%   AF7R58ZH_R4_RUNTIME_PROFILE
%   AF7R58ZH_R4_REPLANNER
%
% Creates REPORT + OUT.mat. Does not modify production.

fprintf('\n====================================================================================================\n');
fprintf(' AF7R58ZI | SAME-CYCLE SPIKE POST-RUN AUDIT\n');
fprintf('====================================================================================================\n');
fprintf('[AUTHORITY] readOnly=1 | productionModified=0 | sameCycleOuterWallClock=1\n');

assert(evalin('base','exist(''AF7R58ZH_R4_RUNTIME_PROFILE'',''var'')')==1, ...
    '[AF7R58ZI] AF7R58ZH_R4_RUNTIME_PROFILE missing. Run R4 diagnostic copy first.');
assert(evalin('base','exist(''AF7R58ZH_R4_REPLANNER'',''var'')')==1, ...
    '[AF7R58ZI] AF7R58ZH_R4_REPLANNER missing. Run R4 diagnostic copy first.');

P=evalin('base','AF7R58ZH_R4_RUNTIME_PROFILE');
R=evalin('base','AF7R58ZH_R4_REPLANNER');

reqFields={ ...
    't','phase','sensor_acquisition_s','sensor_dispatch_s','estimator_s', ...
    'reference_s','controller_s','input_transform_s','online_residual_s', ...
    'online_before_plant_s','simulation_plant_s','logger_s','full_iteration_s', ...
    'gen_request_delta','gen_deadline_delta','gen_commit_delta', ...
    'cache_store_delta','cache_hit_delta','cache_invalidate_delta','cache_reason', ...
    'gen_last_callback_ms','gen_last_critical_ms','gen_last_cycle_critical_ms'};
for k=1:numel(reqFields)
    assert(isfield(P,reqFields{k}),'[AF7R58ZI] missing runtime field: %s',reqFields{k});
end

mask=(P.phase=='f') & isfinite(P.online_before_plant_s);
idx=find(mask);
assert(~isempty(idx),'[AF7R58ZI] no Flight samples.');

[~,o]=sort(P.online_before_plant_s(idx),'descend');
ord=idx(o);
iw=ord(1);

stageNames={ ...
    'sensor_acquisition','sensor_dispatch','estimator', ...
    'reference_replanner','controller','input_transform','online_residual'};
stageFields={ ...
    'sensor_acquisition_s','sensor_dispatch_s','estimator_s', ...
    'reference_s','controller_s','input_transform_s','online_residual_s'};

stageMs=zeros(1,numel(stageFields));
for k=1:numel(stageFields)
    stageMs(k)=1e3*P.(stageFields{k})(iw);
end
[dominantMs,idom]=max(stageMs);

OUT=struct();
OUT.schema='AF7R58ZI_SAME_CYCLE_SPIKE_POSTRUN_V1';
OUT.production_modified=false;
OUT.worst_index=iw;
OUT.worst_time_s=P.t(iw);
OUT.worst_online_ms=1e3*P.online_before_plant_s(iw);
OUT.worst_full_ms=1e3*P.full_iteration_s(iw);
OUT.worst_plant_ms=1e3*P.simulation_plant_s(iw);
OUT.worst_logger_ms=1e3*P.logger_s(iw);
OUT.stage_names=stageNames;
OUT.stage_ms=stageMs;
OUT.dominant_stage=stageNames{idom};
OUT.dominant_stage_ms=dominantMs;
OUT.flight_count=nnz(mask);
OUT.online_over_4ms_count=nnz(mask & P.online_before_plant_s>0.004);
OUT.online_over_25ms_count=nnz(mask & P.online_before_plant_s>0.025);
OUT.generator_request_count=double(R.generator_request_count);
OUT.generator_deadline_fail_count=double(R.generator_deadline_fail_count);
OUT.replan_commit_count=double(R.replan_commit_count);

requestMask=mask & P.gen_request_delta>0;
OUT.request_cycle_count=nnz(requestMask);
if any(requestMask)
    OUT.max_online_request_cycle_ms=1e3*max(P.online_before_plant_s(requestMask));
    OUT.max_reference_request_cycle_ms=1e3*max(P.reference_s(requestMask));
else
    OUT.max_online_request_cycle_ms=NaN;
    OUT.max_reference_request_cycle_ms=NaN;
end
nonRequestMask=mask & ~(P.gen_request_delta>0);
if any(nonRequestMask)
    OUT.max_online_nonrequest_cycle_ms=1e3*max(P.online_before_plant_s(nonRequestMask));
else
    OUT.max_online_nonrequest_cycle_ms=NaN;
end

if strcmp(OUT.dominant_stage,'reference_replanner')
    OUT.status='WORST_SAME_CYCLE_DOMINATED_BY_REFERENCE_REPLANNER';
elseif strcmp(OUT.dominant_stage,'controller')
    OUT.status='WORST_SAME_CYCLE_DOMINATED_BY_CONTROLLER';
elseif strcmp(OUT.dominant_stage,'estimator')
    OUT.status='WORST_SAME_CYCLE_DOMINATED_BY_ESTIMATOR';
elseif strcmp(OUT.dominant_stage,'sensor_acquisition') || strcmp(OUT.dominant_stage,'sensor_dispatch')
    OUT.status='WORST_SAME_CYCLE_DOMINATED_BY_SENSOR_PATH';
else
    OUT.status=['WORST_SAME_CYCLE_DOMINATED_BY_' upper(OUT.dominant_stage)];
end

root=fileparts(which('run_flight_headless'));
outdir=tempname(root);
[ok,msg]=mkdir(outdir); assert(ok,'%s',msg);
report=fullfile(outdir,'AF7R58ZI_REPORT.txt');
matp=fullfile(outdir,'AF7R58ZI_OUT.mat');

fid=fopen(report,'wt','n','UTF-8');
assert(fid>=0,'[AF7R58ZI] cannot open report.');
c=onCleanup(@()fclose(fid)); %#ok<NASGU>

fprintf(fid,'====================================================================================================\n');
fprintf(fid,' AF7R58ZI | SAME-CYCLE SPIKE POST-RUN AUDIT\n');
fprintf(fid,'====================================================================================================\n');
fprintf(fid,'[AUTHORITY] readOnly=1 | sameCycleOuterWallClock=1 | productionModified=0\n');
fprintf(fid,' stage tic/toc values are diagnostic decomposition, not final WCET authority.\n');

fprintf(fid,'\n[A. WORST FLIGHT SAME CYCLE]\n');
fprintf(fid,' t [s]                         = %.9g\n',OUT.worst_time_s);
fprintf(fid,' online before plant [ms]      = %.6f\n',OUT.worst_online_ms);
fprintf(fid,' full iteration [ms]           = %.6f\n',OUT.worst_full_ms);
fprintf(fid,' numerical plant [ms]          = %.6f\n',OUT.worst_plant_ms);
fprintf(fid,' logger [ms]                   = %.6f\n',OUT.worst_logger_ms);
fprintf(fid,' Flight cycles >4 ms           = %d / %d\n',OUT.online_over_4ms_count,OUT.flight_count);
fprintf(fid,' Flight cycles >25 ms          = %d / %d\n',OUT.online_over_25ms_count,OUT.flight_count);

fprintf(fid,'\n[B. WORST-CYCLE ONLINE STAGES]\n');
for k=1:numel(stageNames)
    fprintf(fid,' %-24s = %10.6f ms\n',stageNames{k},stageMs(k));
end
fprintf(fid,' dominant stage                = %s | %.6f ms\n',OUT.dominant_stage,OUT.dominant_stage_ms);

fprintf(fid,'\n[C. WORST-CYCLE REPLANNER EVENTS]\n');
fprintf(fid,' generator request delta       = %.0f\n',P.gen_request_delta(iw));
fprintf(fid,' deadline-fail delta           = %.0f\n',P.gen_deadline_delta(iw));
fprintf(fid,' replan commit delta           = %.0f\n',P.gen_commit_delta(iw));
fprintf(fid,' cache store/hit/invalidate    = %.0f / %.0f / %.0f\n', ...
    P.cache_store_delta(iw),P.cache_hit_delta(iw),P.cache_invalidate_delta(iw));
fprintf(fid,' cache reason                  = %.0f\n',P.cache_reason(iw));
fprintf(fid,' generator last callback [ms]  = %.6f\n',P.gen_last_callback_ms(iw));
fprintf(fid,' generator last critical [ms]  = %.6f\n',P.gen_last_critical_ms(iw));
fprintf(fid,' generator cycle critical [ms] = %.6f\n',P.gen_last_cycle_critical_ms(iw));

fprintf(fid,'\n[D. REQUEST-CYCLE VS NONREQUEST-CYCLE]\n');
fprintf(fid,' request cycles                = %d\n',OUT.request_cycle_count);
fprintf(fid,' max online request cycle [ms] = %.6f\n',OUT.max_online_request_cycle_ms);
fprintf(fid,' max ref request cycle [ms]    = %.6f\n',OUT.max_reference_request_cycle_ms);
fprintf(fid,' max online nonrequest [ms]    = %.6f\n',OUT.max_online_nonrequest_cycle_ms);

fprintf(fid,'\n[E. TOP 20 FLIGHT ONLINE CYCLES]\n');
fprintf(fid,' rank   t[s]   online  sensorA sensorD estimator reference controller inputTx residual plant logger req fail commit cacheS cacheH cacheI reason\n');
nTop=min(20,numel(ord));
OUT.top_indices=ord(1:nTop);
for rr=1:nTop
    j=ord(rr);
    fprintf(fid,['%4d %7.3f %8.3f %8.3f %8.3f %9.3f %9.3f %10.3f %7.3f %8.3f ' ...
        '%6.3f %6.3f %3.0f %4.0f %6.0f %6.0f %6.0f %6.0f %6.0f\n'], ...
        rr,P.t(j),1e3*P.online_before_plant_s(j),1e3*P.sensor_acquisition_s(j), ...
        1e3*P.sensor_dispatch_s(j),1e3*P.estimator_s(j),1e3*P.reference_s(j), ...
        1e3*P.controller_s(j),1e3*P.input_transform_s(j),1e3*P.online_residual_s(j), ...
        1e3*P.simulation_plant_s(j),1e3*P.logger_s(j),P.gen_request_delta(j), ...
        P.gen_deadline_delta(j),P.gen_commit_delta(j),P.cache_store_delta(j), ...
        P.cache_hit_delta(j),P.cache_invalidate_delta(j),P.cache_reason(j));
end

fprintf(fid,'\n[F. REPLAN WINDOW 9.70--10.40 s]\n');
w=find(P.t>=9.70 & P.t<=10.40);
fprintf(fid,' t[s] online reference controller estimator inputTx sensorA sensorD residual plant logger req fail commit cacheS cacheH cacheI reason\n');
for q=1:numel(w)
    j=w(q);
    fprintf(fid,['%6.3f %7.3f %9.3f %10.3f %9.3f %7.3f %7.3f %7.3f %8.3f ' ...
        '%6.3f %6.3f %3.0f %4.0f %6.0f %6.0f %6.0f %6.0f %6.0f\n'], ...
        P.t(j),1e3*P.online_before_plant_s(j),1e3*P.reference_s(j), ...
        1e3*P.controller_s(j),1e3*P.estimator_s(j),1e3*P.input_transform_s(j), ...
        1e3*P.sensor_acquisition_s(j),1e3*P.sensor_dispatch_s(j), ...
        1e3*P.online_residual_s(j),1e3*P.simulation_plant_s(j),1e3*P.logger_s(j), ...
        P.gen_request_delta(j),P.gen_deadline_delta(j),P.gen_commit_delta(j), ...
        P.cache_store_delta(j),P.cache_hit_delta(j),P.cache_invalidate_delta(j),P.cache_reason(j));
end

fprintf(fid,'\n[G. FINAL COUNTERS]\n');
fprintf(fid,' generator requests              = %.0f\n',OUT.generator_request_count);
fprintf(fid,' generator deadline failures     = %.0f\n',OUT.generator_deadline_fail_count);
fprintf(fid,' replan commits                   = %.0f\n',OUT.replan_commit_count);

fprintf(fid,'\n[H. CLASSIFICATION]\n');
fprintf(fid,' status = %s\n',OUT.status);

OUT.report_path=report;
OUT.mat_path=matp;
save(matp,'OUT','P','-v7');
assignin('base','AF7R58ZI_OUT',OUT);

fprintf('\n[AF7R58ZI RESULT]\n');
fprintf(' worst t=%.3f s | online=%.3f ms | dominant=%s %.3f ms\n', ...
    OUT.worst_time_s,OUT.worst_online_ms,OUT.dominant_stage,OUT.dominant_stage_ms);
fprintf(' >4 ms=%d/%d | >25 ms=%d/%d\n', ...
    OUT.online_over_4ms_count,OUT.flight_count,OUT.online_over_25ms_count,OUT.flight_count);
fprintf(' report: %s\n',report);
fprintf(' mat:    %s\n',matp);
fprintf(' status: %s\n',OUT.status);
end
