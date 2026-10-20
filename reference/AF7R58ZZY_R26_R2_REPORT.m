function OUT = AF7R58ZZY_R26_R2_REPORT()
% AF7R58ZZY-R26-R2 clean post-run report.
% Controller is NOT hidden: total online is always reported.
% For this research task, a second controller-excluded metric is also reported
% so replanner/reference progress can be judged independently.

clc;
fprintf('\n====================================================================================================\n');
fprintf(' AF7R58ZZY-R26-R2 | CLEAN FOUR-STAGE PERFORMANCE REPORT\n');
fprintf('====================================================================================================\n');

assert(evalin('base','exist(''replanner_instance'',''var'')==1'), ...
    '[R26-R2 REPORT] replanner_instance missing.');
assert(evalin('base','exist(''runtime_profile'',''var'')==1'), ...
    '[R26-R2 REPORT] runtime_profile missing.');
obj=evalin('base','replanner_instance');
P=evalin('base','runtime_profile');

OUT=struct();
OUT.schema='AF7R58ZZY_R26_R2_REPORT_V1';
OUT.class=class(obj);
OUT.source=which(OUT.class);
OUT.production_source=which('REPLANNING_SYSTEM');
OUT.runner_source=which('run_flight_headless');
OUT.closed=false;

assert(strcmp(OUT.class,'REPLANNING_SYSTEM_R26_BALANCED'), ...
    '[R26-R2 REPORT] active class is not REPLANNING_SYSTEM_R26_BALANCED.');

S26=obj.get_r26_performance_status();
S21=obj.get_r21_performance_status();
SP=obj.get_staged_pipeline_status();
OUT.r26=S26; OUT.r21=S21; OUT.pipeline=SP;
assert(isfield(S26,'build_id')&&strcmp(char(S26.build_id),'AF7R58ZZY_R26_BALANCED_R2'), ...
    '[R26-R2 REPORT] R26-R2 build id mismatch.');
try, OUT.merge=af7r77_merge_result_reference_fast_R25(); catch, OUT.merge=struct(); end

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
OUT.flight_over17_count=nnz(valid&ms>17);
OUT.flight_over15_count=nnz(valid&ms>15);

commit=double(obj.last_replan_commit_time);
OUT.commit_time=commit;
dt=double(obj.prediction_dt);
OUT.stage_times=[commit-3*dt,commit-2*dt,commit-dt,commit];
OUT.stage_online_ms=sample_fields(P,t,OUT.stage_times,'online_before_plant_s',1e3);

% Outer runner fields, if present.
OUT.stage_controller_ms=sample_fields(P,t,OUT.stage_times,'r22_controller_prop_s',1e3);
OUT.stage_reference_ms=sample_fields(P,t,OUT.stage_times,'r22_reference_prop_s',1e3);
OUT.stage_replanner_ms=sample_fields(P,t,OUT.stage_times,'r22_replanner_s',1e3);
OUT.stage_estimator_ms=sample_fields(P,t,OUT.stage_times,'r22_estimator_prop_s',1e3);
OUT.stage_merge_ms=sample_fields(P,t,OUT.stage_times,'r22_reference_merge_s',1e3);

controllerAll=nan(size(ms));
if isfield(P,'r22_controller_prop_s')
    q=double(P.r22_controller_prop_s(:));
    if numel(q)==numel(ms), controllerAll=1e3*q; end
end
OUT.controller_excluded_online_ms=ms-controllerAll;
OUT.stage_controller_excluded_ms=OUT.stage_online_ms-OUT.stage_controller_ms;

replanMask=valid&t>=commit-3*dt-0.025&t<=commit+0.125;
postMask=valid&t>=commit+0.025&t<=commit+0.200;
OUT.replan_window_max_ms=max(ms(replanMask),[],'omitnan');
OUT.postcommit_max_ms=max(ms(postMask),[],'omitnan');
if any(isfinite(OUT.controller_excluded_online_ms(replanMask)))
    OUT.replan_window_controller_excluded_max_ms= ...
        max(OUT.controller_excluded_online_ms(replanMask),[],'omitnan');
else
    OUT.replan_window_controller_excluded_max_ms=NaN;
end

idx=find(valid);
[~,ord]=sort(ms(idx),'descend');
idxTop=idx(ord(1:min(20,numel(ord))));
OUT.top_times=t(idxTop);
OUT.top_online_ms=ms(idxTop);
nonHuge=valid&ms<100;
OUT.flight_online_max_below100_ms=max(ms(nonHuge),[],'omitnan');

OUT.clean_profile_flags=struct();
try, OUT.clean_profile_flags.ENABLE_PROFILING=logical(obj.ENABLE_PROFILING); catch, end
try, OUT.clean_profile_flags.ENABLE_LIGHT_RUNTIME_AUDIT=logical(obj.ENABLE_LIGHT_RUNTIME_AUDIT); catch, end
try, OUT.clean_profile_flags.SPIKE_CAPTURE_THRESHOLD_MS=double(obj.SPIKE_CAPTURE_THRESHOLD_MS); catch, end

B=struct( ...
    'r25_steady_median_ms',6.8827, ...
    'r25_replan_max_ms',29.3305, ...
    'r25_postcommit_max_ms',16.3547, ...
    'r25_over25',2, ...
    'r25_compile_ms',2.5024, ...
    'r23_compile_ms',2.0770);
OUT.baseline=B;

fprintf(' class                                  = %s\n',OUT.class);
fprintf(' build                                  = %s\n',char(S26.build_id));
fprintf(' commit time                            = %.9f s\n',commit);
fprintf(' commit/fail                            = %.0f / %.0f\n', ...
    double(SP.commit_count),double(SP.fail_count));
fprintf(' pipeline intervals / delay             = %.0f / %.6f s\n', ...
    double(SP.pipeline_intervals),double(SP.pipeline_delay_s));
fprintf(' Stage1 generate                        = %.6f ms\n',double(SP.last_stage1_ms));
fprintf(' Stage2 reference certificate           = %.6f ms\n',double(S26.reference_cert_ms));
fprintf(' Stage3 core prepare                    = %.6f ms\n',double(S26.core_prepare_ms));
fprintf(' Stage4 compile / activate / total      = %.6f / %.6f / %.6f ms\n', ...
    double(S26.final_compile_ms),double(S26.final_activate_ms),double(S26.final_total_ms));
fprintf(' R21 core prepare total                 = %.6f ms\n',double(S21.prepare_total_ms));
fprintf(' fast obstacle packet                   = %.6f ms | count=%.0f\n', ...
    double(S26.last_fast_packet_rebuild_ms),double(S26.fast_packet_rebuild_count));
fprintf(' Stage4 cache/guard/assign internal     = %.6f / %.6f / %.6f / %.6f ms\n', ...
    double(S26.stage3_cache_ms),double(S26.stage3_target_guard_ms), ...
    double(S26.stage3_assignment_ms),double(S26.stage3_total_ms));
fprintf(' repaired packet/current eval           = %.6f / %.6f ms\n', ...
    double(S26.last_repaired_packet_ms),double(S26.last_current_eval_ms));
if isstruct(OUT.merge)&&isfield(OUT.merge,'fast_count')
    fprintf(' fast merge / fallback count            = %.0f / %.0f\n', ...
        double(OUT.merge.fast_count),double(OUT.merge.fallback_count));
end
fprintf(' detailed profiling / light audit       = %d / %d\n', ...
    getflag(OUT.clean_profile_flags,'ENABLE_PROFILING'), ...
    getflag(OUT.clean_profile_flags,'ENABLE_LIGHT_RUNTIME_AUDIT'));

fprintf('\n[END-TO-END ONLINE]\n');
fprintf(' steady online median                   = %.6f ms  (R25 %.6f)\n', ...
    OUT.flight_online_median_ms,B.r25_steady_median_ms);
fprintf(' replan-window online max               = %.6f ms  (R25 %.6f)\n', ...
    OUT.replan_window_max_ms,B.r25_replan_max_ms);
fprintf(' online @ gen/cert/core/final           = %.6f / %.6f / %.6f / %.6f ms\n', ...
    OUT.stage_online_ms);
fprintf(' controller @ gen/cert/core/final       = %.6f / %.6f / %.6f / %.6f ms\n', ...
    OUT.stage_controller_ms);
fprintf(' controller-excluded @ stages           = %.6f / %.6f / %.6f / %.6f ms\n', ...
    OUT.stage_controller_excluded_ms);
fprintf(' reference @ gen/cert/core/final        = %.6f / %.6f / %.6f / %.6f ms\n', ...
    OUT.stage_reference_ms);
fprintf(' replanner @ gen/cert/core/final        = %.6f / %.6f / %.6f / %.6f ms\n', ...
    OUT.stage_replanner_ms);
fprintf(' estimator @ gen/cert/core/final        = %.6f / %.6f / %.6f / %.6f ms\n', ...
    OUT.stage_estimator_ms);
fprintf(' merge @ gen/cert/core/final            = %.6f / %.6f / %.6f / %.6f ms\n', ...
    OUT.stage_merge_ms);
fprintf(' replan max excluding controller        = %.6f ms\n', ...
    OUT.replan_window_controller_excluded_max_ms);
fprintf(' postcommit online max                  = %.6f ms  (R25 %.6f)\n', ...
    OUT.postcommit_max_ms,B.r25_postcommit_max_ms);
fprintf(' Flight >25 / >20 / >17 / >15 ms        = %d / %d / %d / %d\n', ...
    OUT.flight_over25_count,OUT.flight_over20_count,OUT.flight_over17_count,OUT.flight_over15_count);
fprintf(' Flight full max                        = %.6f ms\n',OUT.flight_online_max_ms);
fprintf(' Flight max below 100 ms                = %.6f ms\n',OUT.flight_online_max_below100_ms);

fprintf('\n[R26-R2 VS R25]\n');
fprintf(' steady median delta                    = %+.6f ms\n', ...
    OUT.flight_online_median_ms-B.r25_steady_median_ms);
fprintf(' replan max delta                       = %+.6f ms\n', ...
    OUT.replan_window_max_ms-B.r25_replan_max_ms);
fprintf(' postcommit delta                       = %+.6f ms\n', ...
    OUT.postcommit_max_ms-B.r25_postcommit_max_ms);
fprintf(' final compile vs R25 compile           = %+.6f ms\n', ...
    double(S26.final_compile_ms)-B.r25_compile_ms);
fprintf(' final compile vs R23 compile           = %+.6f ms\n', ...
    double(S26.final_compile_ms)-B.r23_compile_ms);

fprintf('\n[TOP ONLINE CYCLES]\n');
for k=1:numel(OUT.top_times)
    fprintf(' t=%.6f | online=%.6f ms\n',OUT.top_times(k),OUT.top_online_ms(k));
end

OUT.closed=true;
assignin('base','AF7R58ZZY_R26_R2_REPORT_OUT',OUT);
save('AF7R58ZZY_R26_R2_OUT.mat','OUT','-v7.3');

fid=fopen('AF7R58ZZY_R26_R2_REPORT.txt','wt','n','UTF-8');
if fid>=0
    fprintf(fid,'AF7R58ZZY-R26-R2 RESULT\n');
    fprintf(fid,'class=%s\n',OUT.class);
    fprintf(fid,'build=%s\n',char(S26.build_id));
    fprintf(fid,'stage1_generate=%.9f\n',double(SP.last_stage1_ms));
    fprintf(fid,'stage2_refcert=%.9f\n',double(S26.reference_cert_ms));
    fprintf(fid,'stage3_core=%.9f\n',double(S26.core_prepare_ms));
    fprintf(fid,'stage4_compile=%.9f\n',double(S26.final_compile_ms));
    fprintf(fid,'stage4_activate=%.9f\n',double(S26.final_activate_ms));
    fprintf(fid,'stage4_total=%.9f\n',double(S26.final_total_ms));
    fprintf(fid,'steady_median=%.9f\n',OUT.flight_online_median_ms);
    fprintf(fid,'replan_max=%.9f\n',OUT.replan_window_max_ms);
    fprintf(fid,'stage_online=%s\n',mat2str(OUT.stage_online_ms,10));
    fprintf(fid,'stage_controller=%s\n',mat2str(OUT.stage_controller_ms,10));
    fprintf(fid,'stage_controller_excluded=%s\n',mat2str(OUT.stage_controller_excluded_ms,10));
    fprintf(fid,'stage_reference=%s\n',mat2str(OUT.stage_reference_ms,10));
    fprintf(fid,'stage_replanner=%s\n',mat2str(OUT.stage_replanner_ms,10));
    fprintf(fid,'stage_estimator=%s\n',mat2str(OUT.stage_estimator_ms,10));
    fprintf(fid,'stage_merge=%s\n',mat2str(OUT.stage_merge_ms,10));
    fprintf(fid,'replan_max_controller_excluded=%.9f\n',OUT.replan_window_controller_excluded_max_ms);
    fprintf(fid,'postcommit_max=%.9f\n',OUT.postcommit_max_ms);
    fprintf(fid,'over25=%d\n',OUT.flight_over25_count);
    fprintf(fid,'over20=%d\n',OUT.flight_over20_count);
    fprintf(fid,'over17=%d\n',OUT.flight_over17_count);
    fprintf(fid,'over15=%d\n',OUT.flight_over15_count);
    fprintf(fid,'pipeline_intervals=%.0f\n',double(SP.pipeline_intervals));
    if isstruct(OUT.merge)&&isfield(OUT.merge,'fast_count')
        fprintf(fid,'merge_fast=%.0f\n',double(OUT.merge.fast_count));
        fprintf(fid,'merge_fallback=%.0f\n',double(OUT.merge.fallback_count));
    end
    fclose(fid);
end

fprintf('====================================================================================================\n');
fprintf('Saved: AF7R58ZZY_R26_R2_REPORT.txt / AF7R58ZZY_R26_R2_OUT.mat\n');
fprintf('====================================================================================================\n');
end

function y=sample_fields(P,t,tq,name,scale)
y=nan(size(tq));
if ~isfield(P,name), return; end
v=double(P.(name)(:));
if numel(v)~=numel(t), return; end
for k=1:numel(tq)
    [d,ix]=min(abs(t-tq(k)));
    if isfinite(d)&&d<=1e-9*max(1,abs(tq(k)))
        y(k)=scale*v(ix);
    end
end
end

function y=getflag(S,nm)
y=NaN;
if isfield(S,nm), y=double(logical(S.(nm))); end
end
