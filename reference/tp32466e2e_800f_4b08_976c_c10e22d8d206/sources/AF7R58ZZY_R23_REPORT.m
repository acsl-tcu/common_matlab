function AF7R58ZZY_R23_REPORT
assert(evalin('base','exist(''replanner_instance'',''var'')==1'),'[R23 report] replanner_instance missing.');
assert(evalin('base','exist(''runtime_profile'',''var'')==1'),'[R23 report] runtime_profile missing.');
obj=evalin('base','replanner_instance');
P=evalin('base','runtime_profile');
assert(isa(obj,'REPLANNING_SYSTEM_R23_COMPILED'),'[R23 report] active replanner is not R23.');
S=obj.get_staged_pipeline_status(); R=obj.get_r23_performance_status();
tc=double(obj.last_replan_commit_time); dt=double(obj.prediction_dt);
mb=P.phase=='f' & P.t>=tc-0.70 & P.t<=tc-0.20;
base=median(1e3*P.online_before_plant_s(mb));
win=P.phase=='f' & P.t>=tc-0.075 & P.t<=tc+0.150;
[pk,ii]=max(1e3*P.online_before_plant_s(win)); tw=P.t(win);

fprintf('\n============================================================\n');
fprintf(' AF7R58ZZY-R23 RESULT\n');
fprintf('============================================================\n');
fprintf('class                    = %s\n',class(obj));
fprintf('commit/fail              = %g / %g\n',S.commit_count,S.fail_count);
fprintf('Stage1                   = %.6f ms\n',S.last_stage1_ms);
fprintf('Stage2                   = %.6f ms\n',S.last_stage2_ms);
fprintf('Stage3                   = %.6f ms\n',S.last_stage3_ms);
fprintf('compiled packet build    = %.6f ms\n',R.compile_ms);
fprintf('Stage3 cache             = %.6f ms\n',R.stage3_cache_ms);
fprintf('Stage3 target guard      = %.6f ms\n',R.stage3_target_guard_ms);
fprintf('Stage3 assignment        = %.6f ms\n',R.stage3_assignment_ms);
fprintf('Stage3 total internal    = %.6f ms\n',R.stage3_total_ms);
fprintf('last repaired packet     = %.6f ms\n',R.last_repaired_packet_ms);
fprintf('last current eval        = %.6f ms\n',R.last_current_eval_ms);
fprintf('compiled current count   = %g\n',R.compiled_current_count);
fprintf('compiled packet count    = %g\n',R.compiled_packet_count);
fprintf('steady online median     = %.6f ms\n',base);
fprintf('replan-window online max = %.6f ms @ %.3f s\n',pk,tw(ii));

for j=1:3
    tt=tc-(3-j)*dt;
    [~,k]=min(abs(P.t-tt));
    fprintf('online @ %.3f           = %.6f ms\n',P.t(k),1e3*P.online_before_plant_s(k));
end
m=P.phase=='f' & P.t>=tc+dt & P.t<=tc+0.125;
if any(m)
    fprintf('postcommit online max    = %.6f ms\n',max(1e3*P.online_before_plant_s(m)));
end
fprintf('============================================================\n');
end
