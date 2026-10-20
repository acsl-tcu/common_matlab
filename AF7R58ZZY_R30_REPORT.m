function OUT=AF7R58ZZY_R30_REPORT()
% Post-run report for R30. No controller modification.
clc;
fprintf('\n====================================================================================================\n');
fprintf(' AF7R58ZZY-R30 | SINGLE-BUILD EXACT PERFORMANCE REPORT\n');
fprintf('====================================================================================================\n');

assert(evalin('base','exist(''replanner_instance'',''var'')==1'),'[R30 REPORT] replanner missing.');
assert(evalin('base','exist(''runtime_profile'',''var'')==1'),'[R30 REPORT] runtime_profile missing.');
obj=evalin('base','replanner_instance');
P=evalin('base','runtime_profile');

assert(strcmp(class(obj),'REPLANNING_SYSTEM_R30_SINGLE_BUILD_EXACT'), ...
    '[R30 REPORT] active class mismatch: %s',class(obj));
R=obj.get_r30_performance_status();
SP=obj.get_staged_pipeline_status();
assert(strcmp(char(R.build_id),'AF7R58ZZY_R30_SINGLE_BUILD_EXACT_V1'), ...
    '[R30 REPORT] build mismatch.');

OUT=struct();
OUT.schema='AF7R58ZZY_R30_REPORT_V1';
OUT.class=class(obj);
OUT.r30=R;
OUT.pipeline=SP;

t=double(P.t(:));
online=1e3*double(P.online_before_plant_s(:));
if isfield(P,'phase')
    ph=P.phase;
    if ischar(ph), fl=(ph(:)=='f'); else, fl=strcmpi(string(ph(:)),'f'); end
else
    fl=true(size(t));
end
valid=fl&isfinite(t)&isfinite(online);

commit=double(obj.last_replan_commit_time);
dt=double(obj.prediction_dt);
st=[commit-3*dt commit-2*dt commit-dt commit];
OUT.stage_times=st;
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

R29=struct( ...
    'stage1',4.7921,'native_compile',0.0144,'stage2',0.4418,'stage3',12.7695, ...
    'batch',3.4031,'packet',1.8093,'handles',0.5389,'bundle',0.2238,'binding',0.1370, ...
    'stage4',4.8428,'stage_online',[18.1357 13.5205 21.0830 17.7572], ...
    'stage_cx',[15.6317 11.3616 18.8097 15.5390], ...
    'stage_ref',[12.8869 8.6841 16.1380 12.7777], ...
    'stage_replanner',[12.6565 8.4377 15.8918 12.5354], ...
    'replan_max',21.0830,'replan_max_cx',18.8097,'steady',6.5676,'post',15.4024);
OUT.r29_baseline=R29;

fprintf(' class                                  = %s\n',OUT.class);
fprintf(' build                                  = %s\n',char(R.build_id));
fprintf(' pipeline intervals / delay             = %.0f / %.6f s (UNCHANGED)\n', ...
    double(SP.pipeline_intervals),double(SP.pipeline_delay_s));

fprintf('\n[R30 STAGE3 INTERNAL]\n');
fprintf(' Stage1 native compile                  = %.6f ms\n',double(R.native_compile_ms));
fprintf(' Stage2 continuous reference cert       = %.6f ms\n',double(R.reference_cert_ms));
fprintf(' Stage3 total                           = %.6f ms\n',double(R.core_prepare_ms));
fprintf('   guard/certificate/spacing            = %.6f ms\n',double(R.r30_guard_ms));
fprintf('   exact AF7R58I batch                  = %.6f ms\n',double(R.r30_batch_model_ms));
fprintf('   exact runtime packet                 = %.6f ms\n',double(R.r30_packet_ms));
fprintf('   canonical handles                    = %.6f ms\n',double(R.r30_handles_ms));
fprintf('   native/canonical binding             = %.6f ms\n',double(R.r30_binding_ms));
fprintf('   final reference assembly             = %.6f ms\n',double(R.r30_reference_assembly_ms));
fprintf('   final bundle single build            = %.6f ms\n',double(R.r30_bundle_ms));
fprintf('   measured prepare total               = %.6f ms\n',double(R.r30_prepare_total_ms));
fprintf(' single-build count                     = %.0f\n',double(R.r30_single_build_count));
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

fprintf('\n[R30 - R29 DELTAS]\n');
fprintf(' Stage3 core delta                      = %+.6f ms\n',double(R.core_prepare_ms)-R29.stage3);
fprintf(' Stage3 online delta                    = %+.6f ms\n',OUT.stage_online_ms(3)-R29.stage_online(3));
fprintf(' Stage3 ctrl-excluded delta             = %+.6f ms\n',OUT.stage_controller_excluded_ms(3)-R29.stage_cx(3));
fprintf(' Stage3 reference delta                 = %+.6f ms\n',OUT.stage_reference_ms(3)-R29.stage_ref(3));
fprintf(' Stage3 replanner delta                 = %+.6f ms\n',OUT.stage_replanner_ms(3)-R29.stage_replanner(3));
fprintf(' replan-max ctrl-excluded delta         = %+.6f ms\n', ...
    OUT.replan_max_controller_excluded_ms-R29.replan_max_cx);

OUT.closed=true;
assignin('base','AF7R58ZZY_R30_REPORT_OUT',OUT);
save('AF7R58ZZY_R30_OUT.mat','OUT','-v7.3');

fid=fopen('AF7R58ZZY_R30_REPORT.txt','wt','n','UTF-8');
if fid>=0
    fprintf(fid,'AF7R58ZZY-R30 RESULT\nclass=%s\nbuild=%s\n',OUT.class,char(R.build_id));
    fprintf(fid,'stage1=%.9f\nnative_compile=%.9f\nstage2_refcert=%.9f\nstage3_core=%.9f\n', ...
        double(SP.last_stage1_ms),double(R.native_compile_ms),double(R.reference_cert_ms),double(R.core_prepare_ms));
    fprintf(fid,'r30_guard=%.9f\nr30_batch=%.9f\nr30_packet=%.9f\nr30_handles=%.9f\n', ...
        double(R.r30_guard_ms),double(R.r30_batch_model_ms),double(R.r30_packet_ms),double(R.r30_handles_ms));
    fprintf(fid,'r30_binding=%.9f\nr30_reference_assembly=%.9f\nr30_bundle=%.9f\nr30_prepare_total=%.9f\n', ...
        double(R.r30_binding_ms),double(R.r30_reference_assembly_ms),double(R.r30_bundle_ms),double(R.r30_prepare_total_ms));
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
fprintf('Saved: AF7R58ZZY_R30_REPORT.txt / AF7R58ZZY_R30_OUT.mat\n');
fprintf('====================================================================================================\n');
end

function y=sample(P,t,tq,nm,scale)
y=nan(size(tq));
if ~isfield(P,nm), return; end
v=double(P.(nm)(:));
if numel(v)~=numel(t), return; end
for k=1:numel(tq)
    [d,ix]=min(abs(t-tq(k)));
    if isfinite(d)&&d<=1e-9*max(1,abs(tq(k))), y(k)=scale*v(ix); end
end
end
