function OUT=AF7R58ZZY_R33_REPORT()
clc;
fprintf('\n====================================================================================================\n');
fprintf(' AF7R58ZZY-R33 | R31 + NO NATIVE_RAW A/B REPORT\n');
fprintf('====================================================================================================\n');
assert(evalin('base','exist(''replanner_instance'',''var'')==1'),'[R33 REPORT] replanner missing.');
assert(evalin('base','exist(''runtime_profile'',''var'')==1'),'[R33 REPORT] runtime_profile missing.');
obj=evalin('base','replanner_instance');
P=evalin('base','runtime_profile');

assert(strcmp(class(obj),'REPLANNING_SYSTEM_R33_R31_PLUS_NO_NATIVE_RAW'), ...
    '[R33 REPORT] class mismatch: %s',class(obj));
R=obj.get_r33_performance_status();
SP=obj.get_staged_pipeline_status();
assert(strcmp(char(R.build_id),'AF7R58ZZY_R33_R31_PLUS_NO_NATIVE_RAW_V1'), ...
    '[R33 REPORT] build mismatch.');

OUT=struct();
OUT.schema='AF7R58ZZY_R33_REPORT_V1';
OUT.class=class(obj); OUT.r33=R; OUT.pipeline=SP;

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

% Exact supplied baselines.
R31=struct();
R31.stage1=4.5562; R31.stage2=0.6059; R31.stage3=7.9053; R31.stage4=4.5303;
R31.stage_online=[19.9617 16.5669 15.9964 18.8029];
R31.stage_cx=[17.4489 14.1059 13.7878 16.4721];
R31.stage_ref=[14.1957 11.3504 10.9999 13.6297];
R31.stage_replanner=[13.9428 11.0532 10.7690 13.3706];
R31.replan_max=19.9617; R31.replan_max_cx=17.4489; R31.steady=6.5692; R31.post=16.7229;

R32=struct();
R32.stage1=7.1076; R32.stage2=0.4577; R32.stage3=6.5517; R32.stage4=4.6044;
R32.stage_online=[22.1992 14.0552 14.2809 17.0247];
R32.stage_cx=[19.7429 11.7109 12.1014 14.7060];
R32.stage_ref=[15.3662 8.9876 9.3424 12.0441];
R32.stage_replanner=[15.0955 8.7327 9.1478 11.8148];
R32.replan_max=22.1992; R32.replan_max_cx=19.7429; R32.steady=6.5339; R32.post=17.3118;

OUT.r31_baseline=R31;
OUT.r32_baseline=R32;

fprintf(' class                                  = %s\n',OUT.class);
fprintf(' build                                  = %s\n',char(R.build_id));
fprintf(' isolated changes                       = %.0f\n',double(R.isolated_change_count));
fprintf(' native RAW retained                    = %d\n',logical(R.native_raw_retained));
fprintf(' Stage1 semantics                       = %s\n',char(R.stage1_candidate_return_semantics));
fprintf(' pipeline intervals / delay             = %.0f / %.6f s (UNCHANGED)\n', ...
    double(SP.pipeline_intervals),double(SP.pipeline_delay_s));

fprintf('\n[R33 END-TO-END SAME FOUR CYCLES]\n');
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

fprintf('\n[R33 - R31 : ISOLATED native_raw EFFECT]\n');
fprintf(' Stage1 core delta                      = %+.4f ms\n',double(SP.last_stage1_ms)-R31.stage1);
fprintf(' Stage1 replanner delta                 = %+.4f ms\n',OUT.stage_replanner_ms(1)-R31.stage_replanner(1));
fprintf(' Stage1 controller-excluded delta       = %+.4f ms\n',OUT.stage_controller_excluded_ms(1)-R31.stage_cx(1));
fprintf(' Stage4 controller-excluded delta       = %+.4f ms\n',OUT.stage_controller_excluded_ms(4)-R31.stage_cx(4));
fprintf(' replan-max ctrl-excluded delta         = %+.4f ms\n', ...
    OUT.replan_max_controller_excluded_ms-R31.replan_max_cx);

fprintf('\n[R33 - R32 : DIRECT-CANDIDATE REVERSION CHECK]\n');
fprintf(' Stage1 core delta                      = %+.4f ms\n',double(SP.last_stage1_ms)-R32.stage1);
fprintf(' Stage1 replanner delta                 = %+.4f ms\n',OUT.stage_replanner_ms(1)-R32.stage_replanner(1));
fprintf(' Stage1 controller-excluded delta       = %+.4f ms\n',OUT.stage_controller_excluded_ms(1)-R32.stage_cx(1));

OUT.closed=true;
assignin('base','AF7R58ZZY_R33_REPORT_OUT',OUT);
save('AF7R58ZZY_R33_OUT.mat','OUT','-v7.3');

fid=fopen('AF7R58ZZY_R33_REPORT.txt','wt','n','UTF-8');
if fid>=0
    fprintf(fid,'AF7R58ZZY-R33 RESULT\nclass=%s\nbuild=%s\n',OUT.class,char(R.build_id));
    fprintf(fid,'stage1=%.9f\nstage2_refcert=%.9f\nstage3_core=%.9f\nstage4_total=%.9f\n', ...
        double(SP.last_stage1_ms),double(R.reference_cert_ms), ...
        double(R.core_prepare_ms),double(R.final_total_ms));
    fprintf(fid,'stage_online=%s\nstage_controller_excluded=%s\nstage_reference=%s\nstage_replanner=%s\n', ...
        mat2str(OUT.stage_online_ms,10),mat2str(OUT.stage_controller_excluded_ms,10), ...
        mat2str(OUT.stage_reference_ms,10),mat2str(OUT.stage_replanner_ms,10));
    fprintf(fid,'replan_max=%.9f\nreplan_max_controller_excluded=%.9f\nsteady_median=%.9f\npostcommit_max=%.9f\n', ...
        OUT.replan_max_ms,OUT.replan_max_controller_excluded_ms, ...
        OUT.steady_median_ms,OUT.postcommit_max_ms);
    fprintf(fid,'target_le17=%d\nall_stages_le17=%d\n',OUT.target_le17,OUT.all_stages_le17);
    fclose(fid);
end
fprintf('====================================================================================================\n');
fprintf('Saved: AF7R58ZZY_R33_REPORT.txt / AF7R58ZZY_R33_OUT.mat\n');
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
