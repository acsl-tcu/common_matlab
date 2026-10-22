function R = AF7R58R29_POST_MORTEM()
% Examine current in-process live object, without substituting prior run results.
% The existing failure record is only accepted with the NEW R29 schema.
R=struct();
R.schema='AF7R58R29_POST_MORTEM_V1';
R.new_roll_failure=false;
R.simulation_complete_not_proven=true;
R.commit_attempts=NaN;
R.commit_successes=NaN;
R.stage_commits=NaN;
R.active_reference_mode='NO_LIVE_INSTANCE';
R.active_reference_version=NaN;
R.stage1_count=NaN;
R.stage2_count=NaN;
R.first_stage1_ms=NaN;
R.last_stage1_ms=NaN;
R.last_stage2_ms=NaN;
R.t_s=NaN;
R.roll_old_bound=NaN;
R.roll_after_endpoint_bound=NaN;
R.theta_old=NaN;
R.theta_used=NaN;
if evalin('base','exist(''replanner_instance'',''var'')==1')
    obj=evalin('base','replanner_instance');
    assert(isa(obj,'REPLANNING_SYSTEM'), '[R29] invalid live replanner_instance.');
    R.commit_attempts=double(obj.r26_sim_commit_attempted);
    R.commit_successes=double(obj.r26_sim_commit_succeeded);
    R.stage_commits=double(obj.staged_replan_commit_count);
    R.active_reference_mode=char(obj.active_reference_mode);
    R.active_reference_version=double(obj.active_reference_version);
    R.stage1_count=double(obj.staged_replan_stage1_count);
    R.stage2_count=double(obj.staged_replan_stage2_count);
    R.last_stage1_ms=double(obj.staged_replan_last_stage1_ms);
    R.last_stage2_ms=double(obj.staged_replan_last_stage2_ms);
    if obj.r25_event_capture_valid
        E=obj.r25_event_capture.events;
        idx=find(strcmp({E.name},'STAGE1_NATIVE_RETURNED'),1,'first');
        if ~isempty(idx),R.first_stage1_ms=double(E(idx).stage_ms);end
    end
end
key='AF7R58R27_ROLL_FAILURE';
if isappdata(0,key)
    F=getappdata(0,key);
    if isstruct(F) && isfield(F,'r29_schema') && ...
            strcmp(char(F.r29_schema),'AF7R58R29_TWO_ENDPOINT_CERTIFICATE_V1')
        R.new_roll_failure=true;
        R.t_s=double(F.t_s);
        R.roll_old_bound=double(F.r29_original_roll_dq);
        R.roll_after_endpoint_bound=double(F.r29_used_roll_dq);
        R.theta_old=double(F.r29_original_theta);
        R.theta_used=double(F.r29_used_theta);
        R.kmin=double(F.kmin);R.kmax=double(F.kmax);
        R.branch_count=double(F.branch_count);
        R.roll_measurement_difference=double(F.roll1-F.roll0);
        R.W_ub=double(F.omega_reach_ub);
    end
end
if R.new_roll_failure
    R.classification='NEW_R29_OBSERVER_FAILURE__OLD_SAFETY_GATE_RETAINED';
elseif ~isnan(R.commit_successes)
    R.classification='LIVE_INSTANCE__NO_R29_ROLL_FAILURE_RECORDED';
else
    R.classification='NO_NEW_ROLL_FAILURE_AND_NO_LIVE_INSTANCE';
end
out=fileparts(mfilename('fullpath'));
save(fullfile(out,'AF7R58R29_POST_MORTEM.mat'),'R');
fid=fopen(fullfile(out,'AF7R58R29_POST_MORTEM.txt'),'w');
assert(fid>=0,'[R29] output file open failed');
c=onCleanup(@()fclose(fid));
fprintf(fid,'R29 | CURRENT SESSION POST MORTEM | NON-PRODUCTION DIAGNOSTIC\n');
fprintf(fid,'classification=%s\n',R.classification);
fprintf(fid,'R29_roll_failure=%d t=%.12g\n',R.new_roll_failure,R.t_s);
fprintf(fid,'rollBound old/new = %.12g / %.12g\n',R.roll_old_bound,R.roll_after_endpoint_bound);
fprintf(fid,'theta old/new = %.12g / %.12g\n',R.theta_old,R.theta_used);
fprintf(fid,'commit_attempts/success=%g/%g, mode=%s, version=%g\n', ...
    R.commit_attempts,R.commit_successes,R.active_reference_mode,R.active_reference_version);
fprintf(fid,'Stage1 count=%g Stage2 count=%g first_stage1_ms=%g last_stage1_ms=%g last_stage2_ms=%g\n', ...
    R.stage1_count,R.stage2_count,R.first_stage1_ms,R.last_stage1_ms,R.last_stage2_ms);
fprintf(fid,'WARNING: not a certificate for real plant tracking, actuation, or time bound.\n');
fprintf('[R29] %s\n',R.classification);
disp(R);
end
