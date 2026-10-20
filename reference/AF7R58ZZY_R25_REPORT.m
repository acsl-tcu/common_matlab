function OUT = AF7R58ZZY_R25_REPORT()
% AF7R58ZZY-R25 post-run performance report.
% Run after run_flight_headless. No rerun, no patching.

clc;
fprintf('\n====================================================================================================\n');
fprintf(' AF7R58ZZY-R25 | POST-RUN PERFORMANCE REPORT\n');
fprintf('====================================================================================================\n');

assert(evalin('base','exist(''replanner_instance'',''var'')==1'), ...
    '[R25 REPORT] replanner_instance missing.');
assert(evalin('base','exist(''runtime_profile'',''var'')==1'), ...
    '[R25 REPORT] runtime_profile missing.');
obj=evalin('base','replanner_instance');
P=evalin('base','runtime_profile');

OUT=struct();
OUT.schema='AF7R58ZZY_R25_REPORT_V1';
OUT.class=class(obj);
OUT.source=which(OUT.class);
OUT.production_source=which('REPLANNING_SYSTEM');
OUT.runner_source=which('run_flight_headless');
OUT.closed=false;

assert(strcmp(OUT.class,'REPLANNING_SYSTEM_R25_LEAN'), ...
    '[R25 REPORT] active class is not REPLANNING_SYSTEM_R25_LEAN.');

S25=obj.get_r25_performance_status();
S21=obj.get_r21_performance_status();
S19=obj.get_r19_performance_status();
SP=obj.get_staged_pipeline_status();
OUT.r25=S25; OUT.r21=S21; OUT.r19=S19; OUT.pipeline=SP;
try, OUT.merge=af7r77_merge_result_reference_fast_R25(); catch, OUT.merge=struct(); end

% Runtime profile
assert(isstruct(P)&&isfield(P,'t')&&isfield(P,'online_before_plant_s'));
t=double(P.t(:)); ms=1e3*double(P.online_before_plant_s(:));
if isfield(P,'phase')
    ph=P.phase;
    if ischar(ph), fl=(ph(:)=='f'); else, fl=strcmpi(string(ph(:)),'f'); end
else
    fl=true(size(t));
end
valid=fl&isfinite(t)&isfinite(ms);
OUT.flight_online_median_ms=median(ms(valid),'omitnan');
OUT.flight_online_max_ms=max(ms(valid),[],'omitnan');
OUT.flight_over25_count=nnz(valid&ms>25);
OUT.flight_over20_count=nnz(valid&ms>20);

commit=double(obj.last_replan_commit_time);
OUT.commit_time=commit;
replanMask=valid&t>=commit-0.075&t<=commit+0.125;
postMask=valid&t>=commit+0.025&t<=commit+0.200;
OUT.replan_window_max_ms=max(ms(replanMask),[],'omitnan');
OUT.postcommit_max_ms=max(ms(postMask),[],'omitnan');

% Exact scheduled stage samples when present.
dt=double(obj.prediction_dt);
OUT.stage_times=[commit-2*dt,commit-dt,commit];
OUT.stage_online_ms=nan(1,3);
for k=1:3
    [d,ix]=min(abs(t-OUT.stage_times(k)));
    if isfinite(d)&&d<=1e-9*max(1,abs(OUT.stage_times(k)))
        OUT.stage_online_ms(k)=ms(ix);
    end
end

% Top online cycles, excluding the known controller-only very large event only
% in a second diagnostic metric; never hide it from the full maximum above.
idx=find(valid);
[~,ord]=sort(ms(idx),'descend');
idxTop=idx(ord(1:min(20,numel(ord))));
OUT.top_times=t(idxTop); OUT.top_online_ms=ms(idxTop);
nonHuge=valid&ms<100;
OUT.flight_online_max_below100_ms=max(ms(nonHuge),[],'omitnan');

% R23 audited baseline constants from the immediately preceding run.
B=struct('stage1_ms',4.2186,'stage2_ms',9.6085,'stage3_ms',5.8230, ...
    'steady_median_ms',10.1812,'replan_window_max_ms',29.7394, ...
    'online_9850_ms',27.6576,'online_9875_ms',29.7394,'online_9900_ms',21.5163, ...
    'postcommit_max_ms',19.7844,'compile_ms',2.0770,'merge_median_ms',2.9302);
OUT.r23_baseline=B;

fprintf(' class                                  = %s\n',OUT.class);
fprintf(' commit time                            = %.9f s\n',commit);
fprintf(' commit/fail                            = %.0f / %.0f\n', ...
    double(SP.commit_count),double(SP.fail_count));
fprintf(' Stage1/2/3                             = %.6f / %.6f / %.6f ms\n', ...
    double(SP.last_stage1_ms),double(SP.last_stage2_ms),double(SP.last_stage3_ms));
fprintf(' R25 compile                            = %.6f ms\n',double(S25.compile_ms));
fprintf(' R25 core packet                        = %.6f ms\n',double(S25.last_core_packet_ms));
fprintf(' R25 fast obstacle packet               = %.6f ms | count=%.0f\n', ...
    double(S25.last_fast_packet_rebuild_ms),double(S25.fast_packet_rebuild_count));
fprintf(' nominal/repaired core cache hits       = %.0f / %.0f\n', ...
    double(S25.nominal_core_cache_hit_count),double(S25.repaired_core_cache_hit_count));
fprintf(' waiting freshness skips                = %.0f\n',double(S25.wait_freshness_skip_count));
fprintf(' Stage3 cache/guard/assign/internal     = %.6f / %.6f / %.6f / %.6f ms\n', ...
    double(S25.stage3_cache_ms),double(S25.stage3_target_guard_ms), ...
    double(S25.stage3_assignment_ms),double(S25.stage3_total_ms));
fprintf(' last repaired packet/current eval      = %.6f / %.6f ms\n', ...
    double(S25.last_repaired_packet_ms),double(S25.last_current_eval_ms));
if isstruct(OUT.merge)&&isfield(OUT.merge,'fast_count')
    fprintf(' fast merge / fallback count            = %.0f / %.0f\n', ...
        double(OUT.merge.fast_count),double(OUT.merge.fallback_count));
end
fprintf(' steady online median                   = %.6f ms  (R23 %.6f)\n', ...
    OUT.flight_online_median_ms,B.steady_median_ms);
fprintf(' replan-window online max               = %.6f ms  (R23 %.6f)\n', ...
    OUT.replan_window_max_ms,B.replan_window_max_ms);
fprintf(' online @ stage1/stage2/stage3          = %.6f / %.6f / %.6f ms\n',OUT.stage_online_ms);
fprintf(' postcommit online max                  = %.6f ms  (R23 %.6f)\n', ...
    OUT.postcommit_max_ms,B.postcommit_max_ms);
fprintf(' Flight >25 ms count                    = %d\n',OUT.flight_over25_count);
fprintf(' Flight full max                        = %.6f ms\n',OUT.flight_online_max_ms);
fprintf(' Flight max below 100 ms                = %.6f ms\n',OUT.flight_online_max_below100_ms);

fprintf('\n[R25 VS R23 DELTAS]\n');
fprintf(' Stage1 delta                           = %+.6f ms\n',double(SP.last_stage1_ms)-B.stage1_ms);
fprintf(' Stage2 delta                           = %+.6f ms\n',double(SP.last_stage2_ms)-B.stage2_ms);
fprintf(' Stage3 delta                           = %+.6f ms\n',double(SP.last_stage3_ms)-B.stage3_ms);
fprintf(' steady median delta                    = %+.6f ms\n',OUT.flight_online_median_ms-B.steady_median_ms);
fprintf(' replan max delta                       = %+.6f ms\n',OUT.replan_window_max_ms-B.replan_window_max_ms);
fprintf(' postcommit max delta                   = %+.6f ms\n',OUT.postcommit_max_ms-B.postcommit_max_ms);

fprintf('\n[TOP ONLINE CYCLES]\n');
for k=1:numel(OUT.top_times)
    fprintf(' t=%.6f | online=%.6f ms\n',OUT.top_times(k),OUT.top_online_ms(k));
end

OUT.closed=true;
assignin('base','AF7R58ZZY_R25_REPORT_OUT',OUT);
save('AF7R58ZZY_R25_OUT.mat','OUT','-v7.3');

fid=fopen('AF7R58ZZY_R25_REPORT.txt','wt','n','UTF-8');
if fid>=0
    fprintf(fid,'AF7R58ZZY-R25 RESULT\n');
    fprintf(fid,'class=%s\n',OUT.class);
    fprintf(fid,'stage1=%.9f\n',double(SP.last_stage1_ms));
    fprintf(fid,'stage2=%.9f\n',double(SP.last_stage2_ms));
    fprintf(fid,'stage3=%.9f\n',double(SP.last_stage3_ms));
    fprintf(fid,'compile=%.9f\n',double(S25.compile_ms));
    fprintf(fid,'core_packet=%.9f\n',double(S25.last_core_packet_ms));
    fprintf(fid,'fast_packet=%.9f\n',double(S25.last_fast_packet_rebuild_ms));
    fprintf(fid,'steady_median=%.9f\n',OUT.flight_online_median_ms);
    fprintf(fid,'replan_max=%.9f\n',OUT.replan_window_max_ms);
    fprintf(fid,'stage_online=%s\n',mat2str(OUT.stage_online_ms,10));
    fprintf(fid,'postcommit_max=%.9f\n',OUT.postcommit_max_ms);
    fprintf(fid,'over25=%d\n',OUT.flight_over25_count);
    if isstruct(OUT.merge)&&isfield(OUT.merge,'fast_count')
        fprintf(fid,'merge_fast=%.0f\n',double(OUT.merge.fast_count));
        fprintf(fid,'merge_fallback=%.0f\n',double(OUT.merge.fallback_count));
    end
    fclose(fid);
end

fprintf('====================================================================================================\n');
fprintf('Saved: AF7R58ZZY_R25_REPORT.txt / AF7R58ZZY_R25_OUT.mat\n');
fprintf('====================================================================================================\n');
end
