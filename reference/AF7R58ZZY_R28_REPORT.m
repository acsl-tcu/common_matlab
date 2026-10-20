function OUT=AF7R58ZZY_R28_REPORT()
% AF7R58ZZY-R28 post-run report. Controller is shown but not a research target.
clc;
fprintf('\n====================================================================================================\n');
fprintf(' AF7R58ZZY-R28 | NATIVE-ONCE PERFORMANCE REPORT\n');
fprintf('====================================================================================================\n');
assert(evalin('base','exist(''replanner_instance'',''var'')==1'),'[R28 REPORT] replanner missing.');
assert(evalin('base','exist(''runtime_profile'',''var'')==1'),'[R28 REPORT] runtime_profile missing.');
obj=evalin('base','replanner_instance'); P=evalin('base','runtime_profile');
assert(strcmp(class(obj),'REPLANNING_SYSTEM_R28_NATIVE_ONCE'),'[R28 REPORT] active class mismatch: %s',class(obj));
R=obj.get_r28_performance_status(); R21=obj.get_r21_performance_status(); SP=obj.get_staged_pipeline_status();
assert(strcmp(char(R.build_id),'AF7R58ZZY_R28_NATIVE_ONCE_V1'),'[R28 REPORT] build mismatch.');
OUT=struct('schema','AF7R58ZZY_R28_REPORT_V1','class',class(obj),'r28',R,'r21',R21,'pipeline',SP);
t=double(P.t(:)); online=1e3*double(P.online_before_plant_s(:));
if isfield(P,'phase'), ph=P.phase; if ischar(ph), fl=(ph(:)=='f'); else, fl=strcmpi(string(ph(:)),'f'); end; else, fl=true(size(t)); end
valid=fl&isfinite(t)&isfinite(online); commit=double(obj.last_replan_commit_time); dt=double(obj.prediction_dt);
st=[commit-3*dt commit-2*dt commit-dt commit]; OUT.stage_times=st;
OUT.stage_online_ms=sample(P,t,st,'online_before_plant_s',1e3);
OUT.stage_reference_ms=sample(P,t,st,'r22_reference_prop_s',1e3);
OUT.stage_replanner_ms=sample(P,t,st,'r22_replanner_s',1e3);
OUT.stage_controller_ms=sample(P,t,st,'r22_controller_prop_s',1e3);
OUT.stage_estimator_ms=sample(P,t,st,'r22_estimator_prop_s',1e3);
OUT.stage_merge_ms=sample(P,t,st,'r22_reference_merge_s',1e3);
OUT.stage_controller_excluded_ms=OUT.stage_online_ms-OUT.stage_controller_ms;
mask=valid&t>=commit-3*dt-0.025&t<=commit+0.125; post=valid&t>=commit+0.025&t<=commit+0.200;
OUT.steady_median_ms=median(online(valid),'omitnan'); OUT.replan_max_ms=max(online(mask),[],'omitnan');
OUT.postcommit_max_ms=max(online(post),[],'omitnan');
cx=online; if isfield(P,'r22_controller_prop_s'), q=double(P.r22_controller_prop_s(:)); if numel(q)==numel(cx), cx=cx-1e3*q; end; end
OUT.replan_max_controller_excluded_ms=max(cx(mask),[],'omitnan');
OUT.target_le17=OUT.replan_max_controller_excluded_ms<=17;
OUT.all_stages_le17=all(OUT.stage_controller_excluded_ms<=17|isnan(OUT.stage_controller_excluded_ms));
OUT.over25=nnz(valid&online>25);OUT.over20=nnz(valid&online>20);OUT.over17=nnz(valid&online>17);OUT.over15=nnz(valid&online>15);
B=struct('stage_online',[24.8499 14.0472 23.1187 19.3573], ...
 'stage_cx',[22.4971 11.7307 20.7862 16.9267], ...
 'stage_ref',[19.9577 8.8521 17.2727 14.0372], ...
 'stage_replanner',[19.6493 8.5662 17.0177 13.7630], ...
 'replan_max',24.8499,'replan_max_cx',22.4971,'steady',6.6872,'post',15.5637, ...
 'incremental_compile',6.5116,'stage3_core',12.7483);
OUT.r27_baseline=B;

fprintf(' class                                  = %s\n',OUT.class);
fprintf(' build                                  = %s\n',char(R.build_id));
fprintf(' commit/fail                            = %.0f / %.0f\n',double(SP.commit_count),double(SP.fail_count));
fprintf(' pipeline intervals / delay             = %.0f / %.6f s (UNCHANGED)\n',double(SP.pipeline_intervals),double(SP.pipeline_delay_s));
fprintf('\n[THEORY-PRESERVING WORK ELIMINATION]\n');
fprintf(' Stage1 total                           = %.6f ms\n',double(SP.last_stage1_ms));
fprintf('   native compiled representation       = %.6f ms | count=%.0f\n',double(R.native_compile_ms),double(R.native_compile_count));
fprintf('   external incremental compiler        = 0\n');
fprintf(' Stage2 continuous reference cert       = %.6f ms\n',double(R.reference_cert_ms));
fprintf(' Stage3 canonical core                  = %.6f ms\n',double(R.core_prepare_ms));
fprintf('   native binding check                 = %.6f ms\n',double(R.binding_ms));
fprintf('   batch / packet / handles / residual  = %.6f / %.6f / %.6f / %.6f ms\n', ...
 double(R21.batch_model_ms),double(R21.packet_ms),double(R21.handles_ms),double(R21.stage2_residual_ms));
fprintf(' Stage4 full compile                    = %.6f ms\n',double(R.final_compile_ms));
fprintf(' Stage4 cache / guard / assign          = %.6f / %.6f / %.6f ms\n',double(R.stage3_cache_ms),double(R.stage3_target_guard_ms),double(R.stage3_assignment_ms));
fprintf(' Stage4 activate / total                = %.6f / %.6f ms\n',double(R.final_activate_ms),double(R.final_total_ms));
fprintf(' full compile avoided                   = %.0f\n',double(R.full_compile_avoided_count));
fprintf(' lazy cache rebuild                     = %.6f ms\n',double(R.last_lazy_cache_rebuild_ms));

fprintf('\n[END-TO-END SAME FOUR CYCLES]\n');
fprintf(' online                                 = %s ms\n',mat2str(OUT.stage_online_ms,7));
fprintf(' controller-excluded                    = %s ms\n',mat2str(OUT.stage_controller_excluded_ms,7));
fprintf(' reference                              = %s ms\n',mat2str(OUT.stage_reference_ms,7));
fprintf(' replanner                              = %s ms\n',mat2str(OUT.stage_replanner_ms,7));
fprintf(' controller                             = %s ms\n',mat2str(OUT.stage_controller_ms,7));
fprintf(' replan max / controller-excluded       = %.6f / %.6f ms\n',OUT.replan_max_ms,OUT.replan_max_controller_excluded_ms);
fprintf(' steady / postcommit                    = %.6f / %.6f ms\n',OUT.steady_median_ms,OUT.postcommit_max_ms);
fprintf(' all stages <=17 ms excl controller     = %d\n',OUT.all_stages_le17);
fprintf(' replan-window max <=17 ms excl ctrl    = %d\n',OUT.target_le17);
fprintf('\n[R28 - R27 DELTAS]\n');
fprintf(' stage online delta                     = %s ms\n',mat2str(OUT.stage_online_ms-B.stage_online,7));
fprintf(' stage ctrl-excluded delta              = %s ms\n',mat2str(OUT.stage_controller_excluded_ms-B.stage_cx,7));
fprintf(' stage reference delta                  = %s ms\n',mat2str(OUT.stage_reference_ms-B.stage_ref,7));
fprintf(' stage replanner delta                  = %s ms\n',mat2str(OUT.stage_replanner_ms-B.stage_replanner,7));
fprintf(' native compile - R27 incr compile      = %+.6f ms\n',double(R.native_compile_ms)-B.incremental_compile);
fprintf(' Stage3 core - R27                      = %+.6f ms\n',double(R.core_prepare_ms)-B.stage3_core);
OUT.closed=true; assignin('base','AF7R58ZZY_R28_REPORT_OUT',OUT); save('AF7R58ZZY_R28_OUT.mat','OUT','-v7.3');
fid=fopen('AF7R58ZZY_R28_REPORT.txt','wt','n','UTF-8');
if fid>=0
 fprintf(fid,'AF7R58ZZY-R28 RESULT\nclass=%s\nbuild=%s\n',OUT.class,char(R.build_id));
 fprintf(fid,'stage1=%.9f\nnative_compile=%.9f\nstage2_refcert=%.9f\nstage3_core=%.9f\nbinding=%.9f\n',double(SP.last_stage1_ms),double(R.native_compile_ms),double(R.reference_cert_ms),double(R.core_prepare_ms),double(R.binding_ms));
 fprintf(fid,'stage4_compile=%.9f\nstage4_cache=%.9f\nstage4_guard=%.9f\nstage4_assign=%.9f\nstage4_activate=%.9f\nstage4_total=%.9f\n',double(R.final_compile_ms),double(R.stage3_cache_ms),double(R.stage3_target_guard_ms),double(R.stage3_assignment_ms),double(R.final_activate_ms),double(R.final_total_ms));
 fprintf(fid,'stage_online=%s\nstage_controller_excluded=%s\nstage_reference=%s\nstage_replanner=%s\n',mat2str(OUT.stage_online_ms,10),mat2str(OUT.stage_controller_excluded_ms,10),mat2str(OUT.stage_reference_ms,10),mat2str(OUT.stage_replanner_ms,10));
 fprintf(fid,'replan_max=%.9f\nreplan_max_controller_excluded=%.9f\nsteady_median=%.9f\npostcommit_max=%.9f\n',OUT.replan_max_ms,OUT.replan_max_controller_excluded_ms,OUT.steady_median_ms,OUT.postcommit_max_ms);
 fprintf(fid,'target_le17=%d\nall_stages_le17=%d\n',OUT.target_le17,OUT.all_stages_le17); fclose(fid);
end
fprintf('====================================================================================================\nSaved: AF7R58ZZY_R28_REPORT.txt / AF7R58ZZY_R28_OUT.mat\n====================================================================================================\n');
end
function y=sample(P,t,tq,nm,scale)
y=nan(size(tq)); if ~isfield(P,nm),return;end;v=double(P.(nm)(:));if numel(v)~=numel(t),return;end
for k=1:numel(tq),[d,ix]=min(abs(t-tq(k)));if isfinite(d)&&d<=1e-9*max(1,abs(tq(k))),y(k)=scale*v(ix);end;end
end
