function OUT=AF7R58ZZY_FINAL_REPORT()
clc;
fprintf('\n====================================================================================================\n');
fprintf(' AF7R58ZZY FINAL | INTEGRATED PRODUCTION PERFORMANCE REPORT\n');
fprintf('====================================================================================================\n');

assert(evalin('base','exist(''replanner_instance'',''var'')==1'),'[FINAL REPORT] replanner missing.');
assert(evalin('base','exist(''runtime_profile'',''var'')==1'),'[FINAL REPORT] runtime_profile missing.');
obj=evalin('base','replanner_instance');
P=evalin('base','runtime_profile');
assert(strcmp(class(obj),'REPLANNING_SYSTEM'),'[FINAL REPORT] active class is %s.',class(obj));

R=obj.get_final_integrated_performance_status();
SP=obj.get_staged_pipeline_status();
assert(strcmp(char(R.build_id),'AF7R58ZZY_FINAL_INTEGRATED_R33_V1'), ...
    '[FINAL REPORT] integrated build mismatch.');

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

OUT=struct();
OUT.schema='AF7R58ZZY_FINAL_REPORT_V1';
OUT.class=class(obj);
OUT.status=R;
OUT.pipeline=SP;
OUT.stage_times=st;
OUT.stage_online_ms=sample(P,t,st,'online_before_plant_s',1e3);
OUT.stage_reference_ms=sample(P,t,st,'r22_reference_prop_s',1e3);
OUT.stage_replanner_ms=sample(P,t,st,'r22_replanner_s',1e3);
OUT.stage_controller_ms=sample(P,t,st,'r22_controller_prop_s',1e3);
OUT.stage_estimator_ms=sample(P,t,st,'r22_estimator_prop_s',1e3);
OUT.stage_diag_snapshot_ms=sample(P,t,st,'r22_diag_snapshot_s',1e3);
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

% Flown R33 baseline.
B=struct();
B.stage1=4.6658; B.stage2=0.5693; B.stage3=6.4911; B.stage4=5.8671;
B.stage_online=[17.9768 14.2235 14.2626 18.8795];
B.stage_cx=[15.5486 11.9251 12.2355 16.3927];
B.stage_ref=[12.5384 9.1559 9.5498 13.1863];
B.stage_replanner=[12.2961 8.8733 9.3412 12.9664];
B.replan_max=18.8795; B.replan_max_cx=16.3927; B.steady=6.4133; B.post=15.3247;
OUT.r33_baseline=B;

fprintf(' class                                  = %s\n',OUT.class);
fprintf(' build                                  = %s\n',char(R.build_id));
fprintf(' historical subclass required           = %d\n',logical(R.historical_runtime_subclass_required));
fprintf(' pipeline intervals / delay             = %.0f / %.6f s\n', ...
    double(SP.pipeline_intervals),double(SP.pipeline_delay_s));
fprintf(' Stage1 / Stage2 / Stage3 / Stage4      = %.4f / %.4f / %.4f / %.4f ms\n', ...
    double(SP.last_stage1_ms),double(R.reference_cert_ms), ...
    double(R.core_prepare_ms),double(R.final_total_ms));
fprintf(' online                                 = %s ms\n',mat2str(OUT.stage_online_ms,7));
fprintf(' controller-excluded                    = %s ms\n',mat2str(OUT.stage_controller_excluded_ms,7));
fprintf(' reference                              = %s ms\n',mat2str(OUT.stage_reference_ms,7));
fprintf(' replanner                              = %s ms\n',mat2str(OUT.stage_replanner_ms,7));
fprintf(' replan max / controller-excluded       = %.4f / %.4f ms\n', ...
    OUT.replan_max_ms,OUT.replan_max_controller_excluded_ms);
fprintf(' steady / postcommit                    = %.4f / %.4f ms\n', ...
    OUT.steady_median_ms,OUT.postcommit_max_ms);
fprintf(' all stages <=17 ms excl controller     = %d\n',OUT.all_stages_le17);
fprintf(' replan-window max <=17 ms excl ctrl    = %d\n',OUT.target_le17);

fprintf('\n[FINAL - FLOWN R33 DELTAS]\n');
fprintf(' Stage1 core delta                      = %+.4f ms\n',double(SP.last_stage1_ms)-B.stage1);
fprintf(' Stage3 core delta                      = %+.4f ms\n',double(R.core_prepare_ms)-B.stage3);
fprintf(' Stage4 total delta                     = %+.4f ms\n',double(R.final_total_ms)-B.stage4);
fprintf(' replan max ctrl-excluded delta         = %+.4f ms\n', ...
    OUT.replan_max_controller_excluded_ms-B.replan_max_cx);

root=fileparts(mfilename('fullpath'));
if evalin('base','exist(''AF7R58ZZY_FINAL_OUTPUT_DIR'',''var'')==1')
    outdir=evalin('base','AF7R58ZZY_FINAL_OUTPUT_DIR');
else
    outdir=fullfile(root,'generated','AF7R58ZZY_FINAL');
end
if ~isfolder(outdir), mkdir(outdir); end

matPath=fullfile(outdir,'AF7R58ZZY_FINAL_OUT.mat');
txtPath=fullfile(outdir,'AF7R58ZZY_FINAL_REPORT.txt');
save(matPath,'OUT','-v7.3');

fid=fopen(txtPath,'wt','n','UTF-8');
assert(fid>=0,'[FINAL REPORT] cannot open report output.');
c=onCleanup(@()fclose(fid));
fprintf(fid,'AF7R58ZZY FINAL RESULT\nclass=%s\nbuild=%s\n',OUT.class,char(R.build_id));
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
clear c;

OUT.output_dir=outdir;
OUT.mat_path=matPath;
OUT.report_path=txtPath;
OUT.closed=true;
assignin('base','AF7R58ZZY_FINAL_REPORT_OUT',OUT);

fprintf(' output directory                       = %s\n',outdir);
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
