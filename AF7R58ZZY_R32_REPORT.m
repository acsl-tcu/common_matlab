function OUT=AF7R58ZZY_R32_REPORT()
clc;
fprintf('\n====================================================================================================\n');
fprintf(' AF7R58ZZY-R32 | DIRECT-CANDIDATE-STORE PERFORMANCE REPORT\n');
fprintf('====================================================================================================\n');
assert(evalin('base','exist(''replanner_instance'',''var'')==1'),'[R32 REPORT] replanner missing.');
assert(evalin('base','exist(''runtime_profile'',''var'')==1'),'[R32 REPORT] runtime_profile missing.');
obj=evalin('base','replanner_instance');
P=evalin('base','runtime_profile');

assert(strcmp(class(obj),'REPLANNING_SYSTEM_R32_DIRECT_CANDIDATE_STORE'), ...
    '[R32 REPORT] class mismatch: %s',class(obj));
R=obj.get_r32_performance_status();
SP=obj.get_staged_pipeline_status();
assert(strcmp(char(R.build_id),'AF7R58ZZY_R32_DIRECT_CANDIDATE_STORE_V1'), ...
    '[R32 REPORT] build mismatch.');

OUT=struct();
OUT.schema='AF7R58ZZY_R32_REPORT_V1';
OUT.class=class(obj); OUT.r32=R; OUT.pipeline=SP;

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

% Exact R31 supplied result baseline.
B=struct();
B.stage1=4.5562; B.stage2=0.6059; B.stage3=7.9053; B.stage4=4.5303;
B.stage_online=[19.9617 16.5669 15.9964 18.8029];
B.stage_cx=[17.4489 14.1059 13.7878 16.4721];
B.stage_ref=[14.1957 11.3504 10.9999 13.6297];
B.stage_replanner=[13.9428 11.0532 10.7690 13.3706];
B.replan_max=19.9617; B.replan_max_cx=17.4489; B.steady=6.5692; B.post=16.7229;
OUT.r31_baseline=B;

fprintf(' class                                  = %s\n',OUT.class);
fprintf(' build                                  = %s\n',char(R.build_id));
fprintf(' pipeline intervals / delay             = %.0f / %.6f s (UNCHANGED)\n', ...
    double(SP.pipeline_intervals),double(SP.pipeline_delay_s));
fprintf(' direct candidate stores                = %.0f\n',double(R.direct_candidate_store_count));
fprintf(' native RAW retained                    = %d\n',logical(R.native_raw_retained));

fprintf('\n[R32 END-TO-END SAME FOUR CYCLES]\n');
fprintf(' Stage1 / Stage2 / Stage3 / Stage4      = %.4f / %.4f / %.4f / %.4f ms\n', ...
    double(SP.last_stage1_ms),double(R.reference_cert_ms), ...
    double(R.core_prepare_ms),double(R.final_total_ms));
fprintf(' online                                 = %s ms\n',mat2str(OUT.stage_online_ms,7));
fprintf(' controller-excluded                    = %s ms\n',mat2str(OUT.stage_controller_excluded_ms,7));
fprintf(' reference                              = %s ms\n',mat2str(OUT.stage_reference_ms,7));
fprintf(' replanner                              = %s ms\n',mat2str(OUT.stage_replanner_ms,7));
fprintf(' diag snapshot                          = %s ms\n',mat2str(OUT.stage_diag_snapshot_ms,7));
fprintf(' non(est+ref+ctrl) overhead             = %s ms\n',mat2str(OUT.stage_non_est_ref_ctrl_ms,7));
fprintf(' replan max / controller-excluded       = %.4f / %.4f ms\n', ...
    OUT.replan_max_ms,OUT.replan_max_controller_excluded_ms);
fprintf(' steady / postcommit                    = %.4f / %.4f ms\n', ...
    OUT.steady_median_ms,OUT.postcommit_max_ms);
fprintf(' all stages <=17 ms excl controller     = %d\n',OUT.all_stages_le17);
fprintf(' replan-window max <=17 ms excl ctrl    = %d\n',OUT.target_le17);

fprintf('\n[R32 - R31 DELTAS]\n');
fprintf(' Stage1 core delta                      = %+.4f ms\n',double(SP.last_stage1_ms)-B.stage1);
fprintf(' Stage1 replanner delta                 = %+.4f ms\n',OUT.stage_replanner_ms(1)-B.stage_replanner(1));
fprintf(' Stage1 reference delta                 = %+.4f ms\n',OUT.stage_reference_ms(1)-B.stage_ref(1));
fprintf(' Stage1 controller-excluded delta       = %+.4f ms\n',OUT.stage_controller_excluded_ms(1)-B.stage_cx(1));
fprintf(' Stage4 controller-excluded delta       = %+.4f ms\n',OUT.stage_controller_excluded_ms(4)-B.stage_cx(4));
fprintf(' replan-max ctrl-excluded delta         = %+.4f ms\n', ...
    OUT.replan_max_controller_excluded_ms-B.replan_max_cx);

OUT.closed=true;
assignin('base','AF7R58ZZY_R32_REPORT_OUT',OUT);
save('AF7R58ZZY_R32_OUT.mat','OUT','-v7.3');

fid=fopen('AF7R58ZZY_R32_REPORT.txt','wt','n','UTF-8');
if fid>=0
    fprintf(fid,'AF7R58ZZY-R32 RESULT\nclass=%s\nbuild=%s\n',OUT.class,char(R.build_id));
    fprintf(fid,'stage1=%.9f\nstage2_refcert=%.9f\nstage3_core=%.9f\nstage4_total=%.9f\n', ...
        double(SP.last_stage1_ms),double(R.reference_cert_ms), ...
        double(R.core_prepare_ms),double(R.final_total_ms));
    fprintf(fid,'stage_online=%s\nstage_controller_excluded=%s\nstage_reference=%s\nstage_replanner=%s\n', ...
        mat2str(OUT.stage_online_ms,10),mat2str(OUT.stage_controller_excluded_ms,10), ...
        mat2str(OUT.stage_reference_ms,10),mat2str(OUT.stage_replanner_ms,10));
    fprintf(fid,'stage_diag_snapshot=%s\nstage_non_est_ref_ctrl=%s\n', ...
        mat2str(OUT.stage_diag_snapshot_ms,10),mat2str(OUT.stage_non_est_ref_ctrl_ms,10));
    fprintf(fid,'replan_max=%.9f\nreplan_max_controller_excluded=%.9f\nsteady_median=%.9f\npostcommit_max=%.9f\n', ...
        OUT.replan_max_ms,OUT.replan_max_controller_excluded_ms, ...
        OUT.steady_median_ms,OUT.postcommit_max_ms);
    fprintf(fid,'target_le17=%d\nall_stages_le17=%d\n',OUT.target_le17,OUT.all_stages_le17);
    fclose(fid);
end
fprintf('====================================================================================================\n');
fprintf('Saved: AF7R58ZZY_R32_REPORT.txt / AF7R58ZZY_R32_OUT.mat\n');
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
