function OUT = AF7R58ZZY_R27_REPORT()
% AF7R58ZZY-R27 post-run performance report.
% Research timing target is reference/replanner path; controller is reported
% separately and is not modified by R27.

clc;
fprintf('\n====================================================================================================\n');
fprintf(' AF7R58ZZY-R27 | EXACT WORK-ELIMINATION PERFORMANCE REPORT\n');
fprintf('====================================================================================================\n');
assert(evalin('base','exist(''replanner_instance'',''var'')==1'), ...
    '[R27 REPORT] replanner_instance missing.');
assert(evalin('base','exist(''runtime_profile'',''var'')==1'), ...
    '[R27 REPORT] runtime_profile missing.');
obj=evalin('base','replanner_instance');
P=evalin('base','runtime_profile');
assert(strcmp(class(obj),'REPLANNING_SYSTEM_R27_LEAN_EXACT'), ...
    '[R27 REPORT] active class mismatch: %s',class(obj));

R27=obj.get_r27_performance_status();
R21=obj.get_r21_performance_status();
SP=obj.get_staged_pipeline_status();
OUT=struct();
OUT.schema='AF7R58ZZY_R27_REPORT_V1';
OUT.class=class(obj); OUT.source=which(OUT.class);
OUT.r27=R27; OUT.r21=R21; OUT.pipeline=SP;
try, OUT.merge=af7r77_merge_result_reference_fast_R25(); catch, OUT.merge=struct(); end

assert(strcmp(char(R27.build_id),'AF7R58ZZY_R27_LEAN_EXACT_V1'), ...
    '[R27 REPORT] build id mismatch.');
assert(double(SP.pipeline_intervals)==3, ...
    '[R27 REPORT] R26 schedule was changed; comparison is not work-elimination-only.');

t=double(P.t(:)); online=1e3*double(P.online_before_plant_s(:));
if isfield(P,'phase')
    ph=P.phase;
    if ischar(ph), fl=(ph(:)=='f'); else, fl=strcmpi(string(ph(:)),'f'); end
else, fl=true(size(t)); end
valid=fl&isfinite(t)&isfinite(online);
commit=double(obj.last_replan_commit_time); dt=double(obj.prediction_dt);
st=[commit-3*dt commit-2*dt commit-dt commit];

OUT.commit_time=commit;
OUT.stage_times=st;
OUT.stage_online_ms=sample(P,t,st,'online_before_plant_s',1e3);
OUT.stage_reference_ms=sample(P,t,st,'r22_reference_prop_s',1e3);
OUT.stage_replanner_ms=sample(P,t,st,'r22_replanner_s',1e3);
OUT.stage_controller_ms=sample(P,t,st,'r22_controller_prop_s',1e3);
OUT.stage_estimator_ms=sample(P,t,st,'r22_estimator_prop_s',1e3);
OUT.stage_merge_ms=sample(P,t,st,'r22_reference_merge_s',1e3);
OUT.stage_controller_excluded_ms=OUT.stage_online_ms-OUT.stage_controller_ms;

replanMask=valid&t>=commit-3*dt-0.025&t<=commit+0.125;
postMask=valid&t>=commit+0.025&t<=commit+0.200;
OUT.steady_median_ms=median(online(valid),'omitnan');
OUT.replan_max_ms=max(online(replanMask),[],'omitnan');
OUT.postcommit_max_ms=max(online(postMask),[],'omitnan');
OUT.over25=nnz(valid&online>25); OUT.over20=nnz(valid&online>20);
OUT.over17=nnz(valid&online>17); OUT.over15=nnz(valid&online>15);
controller=nan(size(online));
if isfield(P,'r22_controller_prop_s')
    q=double(P.r22_controller_prop_s(:));
    if numel(q)==numel(online), controller=1e3*q; end
end
cx=online-controller;
OUT.replan_max_controller_excluded_ms=max(cx(replanMask),[],'omitnan');
OUT.all_stage_controller_excluded_le17=all(OUT.stage_controller_excluded_ms<=17 | isnan(OUT.stage_controller_excluded_ms));
OUT.research_target_closed=OUT.replan_max_controller_excluded_ms<=17;

B=struct();
B.r26_stage_online=[18.1319 14.3628 18.3426 24.3640];
B.r26_stage_controller_excluded=[15.8571 12.0924 16.1092 22.0512];
B.r26_stage_reference=[13.0045 9.3128 13.3274 19.2463];
B.r26_stage_replanner=[12.7387 9.0537 13.0733 18.9846];
B.r26_replan_max=24.3640;
B.r26_replan_max_controller_excluded=22.0512;
B.r26_steady_median=6.4995;
B.r26_postcommit_max=15.0818;
B.r26_final_compile=4.7852;
B.r26_stage3_cache=1.5044;
B.r26_stage3_target_guard=0.5835;
B.r26_core_prepare=8.9788;
OUT.r26_baseline=B;

fprintf(' class                                  = %s\n',OUT.class);
fprintf(' build                                  = %s\n',char(R27.build_id));
fprintf(' commit/fail                            = %.0f / %.0f\n',double(SP.commit_count),double(SP.fail_count));
fprintf(' pipeline intervals / delay             = %.0f / %.6f s (UNCHANGED from R26)\n', ...
    double(SP.pipeline_intervals),double(SP.pipeline_delay_s));
fprintf('\n[REAL WORK]\n');
fprintf(' Stage1 total                           = %.6f ms\n',double(SP.last_stage1_ms));
fprintf('   incremental compile                  = %.6f ms | count=%.0f\n', ...
    double(R27.incremental_compile_ms),double(R27.incremental_compile_count));
fprintf(' Stage2 continuous ref certificate      = %.6f ms\n',double(R27.reference_cert_ms));
fprintf(' Stage3 canonical core prepare          = %.6f ms\n',double(R27.core_prepare_ms));
fprintf('   batch / packet / handles / residual  = %.6f / %.6f / %.6f / %.6f ms\n', ...
    double(R21.batch_model_ms),double(R21.packet_ms),double(R21.handles_ms),double(R21.stage2_residual_ms));
fprintf(' Stage4 final compile                   = %.6f ms  (R26 %.6f)\n', ...
    double(R27.final_compile_ms),B.r26_final_compile);
fprintf(' Stage4 cache                           = %.6f ms  (R26 %.6f)\n', ...
    double(R27.stage3_cache_ms),B.r26_stage3_cache);
fprintf(' Stage4 target guard / assignment       = %.6f / %.6f ms\n', ...
    double(R27.stage3_target_guard_ms),double(R27.stage3_assignment_ms));
fprintf(' Stage4 activate / total                = %.6f / %.6f ms\n', ...
    double(R27.final_activate_ms),double(R27.final_total_ms));
fprintf(' full compile avoided count             = %.0f\n',double(R27.full_compile_avoided_count));
fprintf(' lazy postcache commit count            = %.0f\n',double(R27.lazy_postcache_commit_count));
fprintf(' first lazy repaired packet             = %.6f ms\n',double(R27.last_lazy_cache_rebuild_ms));

fprintf('\n[END-TO-END -- SAME FOUR CYCLES AS R26]\n');
fprintf(' online @ gen/cert/core/final           = %.6f / %.6f / %.6f / %.6f ms\n',OUT.stage_online_ms);
fprintf(' controller-excluded @ stages           = %.6f / %.6f / %.6f / %.6f ms\n',OUT.stage_controller_excluded_ms);
fprintf(' reference @ stages                     = %.6f / %.6f / %.6f / %.6f ms\n',OUT.stage_reference_ms);
fprintf(' replanner @ stages                     = %.6f / %.6f / %.6f / %.6f ms\n',OUT.stage_replanner_ms);
fprintf(' controller @ stages                    = %.6f / %.6f / %.6f / %.6f ms\n',OUT.stage_controller_ms);
fprintf(' steady median                          = %.6f ms (R26 %.6f)\n',OUT.steady_median_ms,B.r26_steady_median);
fprintf(' replan max                             = %.6f ms (R26 %.6f)\n',OUT.replan_max_ms,B.r26_replan_max);
fprintf(' replan max excluding controller        = %.6f ms (R26 %.6f)\n', ...
    OUT.replan_max_controller_excluded_ms,B.r26_replan_max_controller_excluded);
fprintf(' postcommit max                         = %.6f ms (R26 %.6f)\n',OUT.postcommit_max_ms,B.r26_postcommit_max);
fprintf(' Flight >25 / >20 / >17 / >15 ms        = %d / %d / %d / %d\n', ...
    OUT.over25,OUT.over20,OUT.over17,OUT.over15);
fprintf(' all four research-path stages <=17 ms  = %d\n',OUT.all_stage_controller_excluded_le17);
fprintf(' research replan-window max <=17 ms     = %d\n',OUT.research_target_closed);

fprintf('\n[R27 - R26 STAGE DELTAS]\n');
fprintf(' online delta                           = %s ms\n',mat2str(OUT.stage_online_ms-B.r26_stage_online,7));
fprintf(' controller-excluded delta              = %s ms\n',mat2str(OUT.stage_controller_excluded_ms-B.r26_stage_controller_excluded,7));
fprintf(' reference delta                        = %s ms\n',mat2str(OUT.stage_reference_ms-B.r26_stage_reference,7));
fprintf(' replanner delta                        = %s ms\n',mat2str(OUT.stage_replanner_ms-B.r26_stage_replanner,7));

OUT.closed=true;
assignin('base','AF7R58ZZY_R27_REPORT_OUT',OUT);
save('AF7R58ZZY_R27_OUT.mat','OUT','-v7.3');

fid=fopen('AF7R58ZZY_R27_REPORT.txt','wt','n','UTF-8');
if fid>=0
    fprintf(fid,'AF7R58ZZY-R27 RESULT\n');
    fprintf(fid,'class=%s\n',OUT.class);
    fprintf(fid,'build=%s\n',char(R27.build_id));
    fprintf(fid,'stage1=%.9f\n',double(SP.last_stage1_ms));
    fprintf(fid,'incremental_compile=%.9f\n',double(R27.incremental_compile_ms));
    fprintf(fid,'stage2_refcert=%.9f\n',double(R27.reference_cert_ms));
    fprintf(fid,'stage3_core=%.9f\n',double(R27.core_prepare_ms));
    fprintf(fid,'stage4_compile=%.9f\n',double(R27.final_compile_ms));
    fprintf(fid,'stage4_cache=%.9f\n',double(R27.stage3_cache_ms));
    fprintf(fid,'stage4_guard=%.9f\n',double(R27.stage3_target_guard_ms));
    fprintf(fid,'stage4_assign=%.9f\n',double(R27.stage3_assignment_ms));
    fprintf(fid,'stage4_activate=%.9f\n',double(R27.final_activate_ms));
    fprintf(fid,'stage4_total=%.9f\n',double(R27.final_total_ms));
    fprintf(fid,'stage_online=%s\n',mat2str(OUT.stage_online_ms,10));
    fprintf(fid,'stage_controller_excluded=%s\n',mat2str(OUT.stage_controller_excluded_ms,10));
    fprintf(fid,'stage_reference=%s\n',mat2str(OUT.stage_reference_ms,10));
    fprintf(fid,'stage_replanner=%s\n',mat2str(OUT.stage_replanner_ms,10));
    fprintf(fid,'replan_max=%.9f\n',OUT.replan_max_ms);
    fprintf(fid,'replan_max_controller_excluded=%.9f\n',OUT.replan_max_controller_excluded_ms);
    fprintf(fid,'steady_median=%.9f\n',OUT.steady_median_ms);
    fprintf(fid,'postcommit_max=%.9f\n',OUT.postcommit_max_ms);
    fprintf(fid,'lazy_cache_rebuild=%.9f\n',double(R27.last_lazy_cache_rebuild_ms));
    fprintf(fid,'full_compile_avoided=%.0f\n',double(R27.full_compile_avoided_count));
    fprintf(fid,'target_le17=%d\n',OUT.research_target_closed);
    fclose(fid);
end
fprintf('====================================================================================================\n');
fprintf('Saved: AF7R58ZZY_R27_REPORT.txt / AF7R58ZZY_R27_OUT.mat\n');
fprintf('====================================================================================================\n');
end

function y=sample(P,t,tq,name,scale)
y=nan(size(tq));
if ~isfield(P,name), return; end
v=double(P.(name)(:)); if numel(v)~=numel(t), return; end
for k=1:numel(tq)
    [d,ix]=min(abs(t-tq(k)));
    if isfinite(d)&&d<=1e-9*max(1,abs(tq(k))), y(k)=scale*v(ix); end
end
end
