function OUT=AF7R58ZZY_R29_REPORT()
% AF7R58ZZY-R29 post-run report. Controller is reported but not modified.
clc;
fprintf('\n====================================================================================================\n');
fprintf(' AF7R58ZZY-R29 | STAGE3-LEAN EXACT PERFORMANCE REPORT\n');
fprintf('====================================================================================================\n');
assert(evalin('base','exist(''replanner_instance'',''var'')==1'),'[R29 REPORT] replanner missing.');
assert(evalin('base','exist(''runtime_profile'',''var'')==1'),'[R29 REPORT] runtime_profile missing.');
obj=evalin('base','replanner_instance'); P=evalin('base','runtime_profile');
assert(strcmp(class(obj),'REPLANNING_SYSTEM_R29_STAGE3_LEAN_EXACT'), ...
    '[R29 REPORT] active class mismatch: %s',class(obj));
R=obj.get_r29_performance_status(); SP=obj.get_staged_pipeline_status();
assert(strcmp(char(R.build_id),'AF7R58ZZY_R29_STAGE3_LEAN_EXACT_V1'),'[R29 REPORT] build mismatch.');

OUT=struct('schema','AF7R58ZZY_R29_REPORT_V1','class',class(obj),'r29',R,'pipeline',SP);
t=double(P.t(:)); online=1e3*double(P.online_before_plant_s(:));
if isfield(P,'phase')
    ph=P.phase;
    if ischar(ph), fl=(ph(:)=='f'); else, fl=strcmpi(string(ph(:)),'f'); end
else
    fl=true(size(t));
end
valid=fl&isfinite(t)&isfinite(online);
commit=double(obj.last_replan_commit_time); dt=double(obj.prediction_dt);
st=[commit-3*dt commit-2*dt commit-dt commit]; OUT.stage_times=st;
OUT.stage_online_ms=sample(P,t,st,'online_before_plant_s',1e3);
OUT.stage_reference_ms=sample(P,t,st,'r22_reference_prop_s',1e3);
OUT.stage_replanner_ms=sample(P,t,st,'r22_replanner_s',1e3);
OUT.stage_controller_ms=sample(P,t,st,'r22_controller_prop_s',1e3);
OUT.stage_estimator_ms=sample(P,t,st,'r22_estimator_prop_s',1e3);
OUT.stage_merge_ms=sample(P,t,st,'r22_reference_merge_s',1e3);
OUT.stage_controller_excluded_ms=OUT.stage_online_ms-OUT.stage_controller_ms;

mask=valid&t>=commit-3*dt-0.025&t<=commit+0.125;
post=valid&t>=commit+0.025&t<=commit+0.200;
OUT.steady_median_ms=median(online(valid),'omitnan');
OUT.replan_max_ms=max(online(mask),[],'omitnan');
OUT.postcommit_max_ms=max(online(post),[],'omitnan');
cx=online;
if isfield(P,'r22_controller_prop_s')
    q=double(P.r22_controller_prop_s(:));
    if numel(q)==numel(cx), cx=cx-1e3*q; end
end
OUT.replan_max_controller_excluded_ms=max(cx(mask),[],'omitnan');
OUT.target_le17=OUT.replan_max_controller_excluded_ms<=17;
OUT.all_stages_le17=all(OUT.stage_controller_excluded_ms<=17|isnan(OUT.stage_controller_excluded_ms));

B=struct( ...
 'stage_online',[17.8743 14.2673 21.6045 18.1167], ...
 'stage_cx',[15.6300 11.9880 18.9993 15.7336], ...
 'stage_ref',[12.8418 8.9737 16.1587 12.8171], ...
 'stage_replanner',[12.6003 8.6722 15.8626 12.5454], ...
 'replan_max',21.6045,'replan_max_cx',18.9993,'steady',6.6570,'post',15.8041, ...
 'stage3_core',12.8097,'binding',1.6156);
OUT.r28_baseline=B;

fprintf(' class                                  = %s\n',OUT.class);
fprintf(' build                                  = %s\n',char(R.build_id));
fprintf(' pipeline intervals / delay             = %.0f / %.6f s (UNCHANGED)\n', ...
    double(SP.pipeline_intervals),double(SP.pipeline_delay_s));
fprintf('\n[R29 STAGE3 WORK]\n');
fprintf(' Stage1 native compile                  = %.6f ms\n',double(R.native_compile_ms));
fprintf(' Stage2 continuous reference cert       = %.6f ms\n',double(R.reference_cert_ms));
fprintf(' Stage3 total                           = %.6f ms\n',double(R.core_prepare_ms));
fprintf('   fast canonical batch                 = %.6f ms\n',double(R.r29_batch_model_ms));
fprintf('   exact packet retarget                = %.6f ms\n',double(R.r29_packet_ms));
fprintf('   canonical diagnostic handles         = %.6f ms\n',double(R.r29_handles_ms));
fprintf('   bundle struct                        = %.6f ms\n',double(R.r29_bundle_core_ms));
fprintf('   native/canonical binding             = %.6f ms\n',double(R.r29_binding_ms));
fprintf(' packet-MEX count / fast-batch count    = %.0f / %.0f\n', ...
    double(R.r29_packet_mex_count),double(R.r29_fast_batch_count));
fprintf(' Stage4 total                           = %.6f ms\n',double(R.final_total_ms));

fprintf('\n[END-TO-END SAME FOUR CYCLES]\n');
fprintf(' online                                 = %s ms\n',mat2str(OUT.stage_online_ms,7));
fprintf(' controller-excluded                    = %s ms\n',mat2str(OUT.stage_controller_excluded_ms,7));
fprintf(' reference                              = %s ms\n',mat2str(OUT.stage_reference_ms,7));
fprintf(' replanner                              = %s ms\n',mat2str(OUT.stage_replanner_ms,7));
fprintf(' replan max / controller-excluded       = %.6f / %.6f ms\n', ...
    OUT.replan_max_ms,OUT.replan_max_controller_excluded_ms);
fprintf(' steady / postcommit                    = %.6f / %.6f ms\n', ...
    OUT.steady_median_ms,OUT.postcommit_max_ms);
fprintf(' all stages <=17 ms excl controller     = %d\n',OUT.all_stages_le17);
fprintf(' replan-window max <=17 ms excl ctrl    = %d\n',OUT.target_le17);

fprintf('\n[R29 - R28 DELTAS]\n');
fprintf(' stage online delta                     = %s ms\n',mat2str(OUT.stage_online_ms-B.stage_online,7));
fprintf(' stage ctrl-excluded delta              = %s ms\n',mat2str(OUT.stage_controller_excluded_ms-B.stage_cx,7));
fprintf(' stage reference delta                  = %s ms\n',mat2str(OUT.stage_reference_ms-B.stage_ref,7));
fprintf(' stage replanner delta                  = %s ms\n',mat2str(OUT.stage_replanner_ms-B.stage_replanner,7));
fprintf(' Stage3 core delta                      = %+.6f ms\n',double(R.core_prepare_ms)-B.stage3_core);
fprintf(' binding delta                          = %+.6f ms\n',double(R.r29_binding_ms)-B.binding);

OUT.closed=true;
assignin('base','AF7R58ZZY_R29_REPORT_OUT',OUT);
save('AF7R58ZZY_R29_OUT.mat','OUT','-v7.3');
fid=fopen('AF7R58ZZY_R29_REPORT.txt','wt','n','UTF-8');
if fid>=0
    fprintf(fid,'AF7R58ZZY-R29 RESULT\nclass=%s\nbuild=%s\n',OUT.class,char(R.build_id));
    fprintf(fid,'stage1=%.9f\nnative_compile=%.9f\nstage2_refcert=%.9f\nstage3_core=%.9f\n', ...
        double(SP.last_stage1_ms),double(R.native_compile_ms),double(R.reference_cert_ms),double(R.core_prepare_ms));
    fprintf(fid,'r29_batch=%.9f\nr29_packet=%.9f\nr29_handles=%.9f\nr29_bundle=%.9f\nr29_binding=%.9f\n', ...
        double(R.r29_batch_model_ms),double(R.r29_packet_ms),double(R.r29_handles_ms), ...
        double(R.r29_bundle_core_ms),double(R.r29_binding_ms));
    fprintf(fid,'stage4_total=%.9f\n',double(R.final_total_ms));
    fprintf(fid,'stage_online=%s\nstage_controller_excluded=%s\nstage_reference=%s\nstage_replanner=%s\n', ...
        mat2str(OUT.stage_online_ms,10),mat2str(OUT.stage_controller_excluded_ms,10), ...
        mat2str(OUT.stage_reference_ms,10),mat2str(OUT.stage_replanner_ms,10));
    fprintf(fid,'replan_max=%.9f\nreplan_max_controller_excluded=%.9f\nsteady_median=%.9f\npostcommit_max=%.9f\n', ...
        OUT.replan_max_ms,OUT.replan_max_controller_excluded_ms,OUT.steady_median_ms,OUT.postcommit_max_ms);
    fprintf(fid,'target_le17=%d\nall_stages_le17=%d\n',OUT.target_le17,OUT.all_stages_le17);
    fclose(fid);
end
fprintf('====================================================================================================\n');
fprintf('Saved: AF7R58ZZY_R29_REPORT.txt / AF7R58ZZY_R29_OUT.mat\n');
fprintf('====================================================================================================\n');
end

function y=sample(P,t,tq,nm,scale)
y=nan(size(tq)); if ~isfield(P,nm),return;end
v=double(P.(nm)(:)); if numel(v)~=numel(t),return;end
for k=1:numel(tq)
    [d,ix]=min(abs(t-tq(k)));
    if isfinite(d)&&d<=1e-9*max(1,abs(tq(k))), y(k)=scale*v(ix); end
end
end
