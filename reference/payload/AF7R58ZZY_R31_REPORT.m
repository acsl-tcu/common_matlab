function OUT=AF7R58ZZY_R31_REPORT()
clc;
fprintf('\n====================================================================================================\n');
fprintf(' AF7R58ZZY-R31 | DIRECT-STAGE-STORE PERFORMANCE REPORT\n');
fprintf('====================================================================================================\n');
assert(evalin('base','exist(''replanner_instance'',''var'')==1'),'[R31 REPORT] replanner missing.');
assert(evalin('base','exist(''runtime_profile'',''var'')==1'),'[R31 REPORT] runtime_profile missing.');
obj=evalin('base','replanner_instance');
P=evalin('base','runtime_profile');
assert(strcmp(class(obj),'REPLANNING_SYSTEM_R31_DIRECT_STAGE_STORE'), ...
    '[R31 REPORT] class mismatch: %s',class(obj));

R=obj.get_r31_performance_status();
SP=obj.get_staged_pipeline_status();
assert(strcmp(char(R.build_id),'AF7R58ZZY_R31_DIRECT_STAGE_STORE_V1'), ...
    '[R31 REPORT] build mismatch.');

OUT=struct();
OUT.schema='AF7R58ZZY_R31_REPORT_V1';
OUT.class=class(obj);
OUT.r31=R; OUT.pipeline=SP;

t=double(P.t(:));
online=1e3*double(P.online_before_plant_s(:));
if isfield(P,'phase')
    ph=P.phase;
    if ischar(ph), fl=(ph(:)=='f'); else, fl=strcmpi(string(ph(:)),'f'); end
else
    fl=true(size(t));
end
valid=fl&isfinite(t)&isfinite(online);
commit=double(obj.last_replan_commit_time); dt=double(obj.prediction_dt);
st=[commit-3*dt commit-2*dt commit-dt commit];
OUT.stage_times=st;
OUT.stage_online_ms=sample(P,t,st,'online_before_plant_s',1e3);
OUT.stage_reference_ms=sample(P,t,st,'r22_reference_prop_s',1e3);
OUT.stage_replanner_ms=sample(P,t,st,'r22_replanner_s',1e3);
OUT.stage_controller_ms=sample(P,t,st,'r22_controller_prop_s',1e3);
OUT.stage_estimator_ms=sample(P,t,st,'r22_estimator_prop_s',1e3);
OUT.stage_merge_ms=sample(P,t,st,'r22_reference_merge_s',1e3);
OUT.stage_diag_snapshot_ms=sample(P,t,st,'r22_diag_snapshot_s',1e3);
OUT.stage_controller_excluded_ms=OUT.stage_online_ms-OUT.stage_controller_ms;

% Non-reference outer overhead before plant (sensor/diag/input etc.) diagnostic.
OUT.stage_non_est_ref_ctrl_ms=OUT.stage_online_ms- ...
    OUT.stage_estimator_ms-OUT.stage_reference_ms-OUT.stage_controller_ms;

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

% Exact R30 observed baseline from the user's supplied result.
B=struct();
B.stage1=4.7626; B.stage2=0.4766; B.stage3_core=13.0778; B.stage4=5.5521;
B.stage_online=[17.8959 13.7748 21.9642 18.5428];
B.stage_cx=[15.4407 11.4892 19.5427 16.2193];
B.stage_ref=[12.8275 8.8022 16.4319 13.3703];
B.stage_replanner=[12.5832 8.5728 16.1894 13.1166];
B.replan_max=21.9642; B.replan_max_cx=19.5427; B.steady=6.5778; B.post=14.9798;
B.prepare_total=6.9180;
OUT.r30_baseline=B;

OUT.r30_outer_core_minus_inner_prepare_ms=B.stage3_core-B.prepare_total;
OUT.r30_stage3_replanner_minus_core_ms=B.stage_replanner(3)-B.stage3_core;

fprintf(' class                                  = %s\n',OUT.class);
fprintf(' build                                  = %s\n',char(R.build_id));
fprintf(' pipeline intervals / delay             = %.0f / %.6f s (UNCHANGED)\n', ...
    double(SP.pipeline_intervals),double(SP.pipeline_delay_s));
fprintf(' direct-store count                      = %.0f\n',double(R.direct_stage_store_count));
fprintf(' detailed internal timing                = %d\n',logical(R.detailed_internal_timing));

fprintf('\n[WHY R30 DID NOT IMPROVE]\n');
fprintf(' R30 inner prepare                       = %.4f ms\n',B.prepare_total);
fprintf(' R30 outer core prepare                  = %.4f ms\n',B.stage3_core);
fprintf(' unexplained method-boundary gap         = %.4f ms\n',OUT.r30_outer_core_minus_inner_prepare_ms);
fprintf(' R30 Stage3 replanner - outer core       = %.4f ms\n',OUT.r30_stage3_replanner_minus_core_ms);
fprintf(' NOTE: R31 removes B value-return + external B property assignment + onCleanup.\n');

fprintf('\n[R31 END-TO-END SAME FOUR CYCLES]\n');
fprintf(' Stage1 / Stage2 / Stage3-core / Stage4  = %.4f / %.4f / %.4f / %.4f ms\n', ...
    double(SP.last_stage1_ms),double(R.reference_cert_ms),double(R.core_prepare_ms),double(R.final_total_ms));
fprintf(' online                                 = %s ms\n',mat2str(OUT.stage_online_ms,7));
fprintf(' controller-excluded                    = %s ms\n',mat2str(OUT.stage_controller_excluded_ms,7));
fprintf(' reference                              = %s ms\n',mat2str(OUT.stage_reference_ms,7));
fprintf(' replanner                              = %s ms\n',mat2str(OUT.stage_replanner_ms,7));
fprintf(' diag snapshot                          = %s ms\n',mat2str(OUT.stage_diag_snapshot_ms,7));
fprintf(' non(est+ref+ctrl) online overhead       = %s ms\n',mat2str(OUT.stage_non_est_ref_ctrl_ms,7));
fprintf(' replan max / controller-excluded       = %.4f / %.4f ms\n', ...
    OUT.replan_max_ms,OUT.replan_max_controller_excluded_ms);
fprintf(' steady / postcommit                    = %.4f / %.4f ms\n', ...
    OUT.steady_median_ms,OUT.postcommit_max_ms);
fprintf(' all stages <=17 ms excl controller     = %d\n',OUT.all_stages_le17);
fprintf(' replan-window max <=17 ms excl ctrl    = %d\n',OUT.target_le17);

fprintf('\n[R31 - R30 DELTAS]\n');
fprintf(' Stage3 core delta                      = %+.4f ms\n',double(R.core_prepare_ms)-B.stage3_core);
fprintf(' Stage3 replanner delta                 = %+.4f ms\n',OUT.stage_replanner_ms(3)-B.stage_replanner(3));
fprintf(' Stage3 reference delta                 = %+.4f ms\n',OUT.stage_reference_ms(3)-B.stage_ref(3));
fprintf(' Stage3 controller-excluded delta       = %+.4f ms\n',OUT.stage_controller_excluded_ms(3)-B.stage_cx(3));
fprintf(' replan-max ctrl-excluded delta         = %+.4f ms\n', ...
    OUT.replan_max_controller_excluded_ms-B.replan_max_cx);

OUT.closed=true;
assignin('base','AF7R58ZZY_R31_REPORT_OUT',OUT);
save('AF7R58ZZY_R31_OUT.mat','OUT','-v7.3');

fid=fopen('AF7R58ZZY_R31_REPORT.txt','wt','n','UTF-8');
if fid>=0
    fprintf(fid,'AF7R58ZZY-R31 RESULT\nclass=%s\nbuild=%s\n',OUT.class,char(R.build_id));
    fprintf(fid,'stage1=%.9f\nstage2_refcert=%.9f\nstage3_core=%.9f\nstage4_total=%.9f\n', ...
        double(SP.last_stage1_ms),double(R.reference_cert_ms),double(R.core_prepare_ms),double(R.final_total_ms));
    fprintf(fid,'stage_online=%s\nstage_controller_excluded=%s\nstage_reference=%s\nstage_replanner=%s\n', ...
        mat2str(OUT.stage_online_ms,10),mat2str(OUT.stage_controller_excluded_ms,10), ...
        mat2str(OUT.stage_reference_ms,10),mat2str(OUT.stage_replanner_ms,10));
    fprintf(fid,'stage_diag_snapshot=%s\nstage_non_est_ref_ctrl=%s\n', ...
        mat2str(OUT.stage_diag_snapshot_ms,10),mat2str(OUT.stage_non_est_ref_ctrl_ms,10));
    fprintf(fid,'replan_max=%.9f\nreplan_max_controller_excluded=%.9f\nsteady_median=%.9f\npostcommit_max=%.9f\n', ...
        OUT.replan_max_ms,OUT.replan_max_controller_excluded_ms,OUT.steady_median_ms,OUT.postcommit_max_ms);
    fprintf(fid,'target_le17=%d\nall_stages_le17=%d\n',OUT.target_le17,OUT.all_stages_le17);
    fclose(fid);
end
fprintf('====================================================================================================\n');
fprintf('Saved: AF7R58ZZY_R31_REPORT.txt / AF7R58ZZY_R31_OUT.mat\n');
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
