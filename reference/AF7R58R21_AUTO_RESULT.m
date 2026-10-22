function T = AF7R58R21_AUTO_RESULT()
% R21: no Flight disk I/O; inherited R20 reference-only source-bound reporter.
% Can report active-avoidance replan only after an actual REPAIRED source event.
T=AF7R58R20_AUTO_RESULT();
fields={'r21_active_base_stage1_requests','r21_active_base_native_returns', ...
    'r21_active_base_stage2_prepared','r21_last_source_mode', ...
    'r21_last_source_version','r21_first_active_replan_time_s', ...
    'r21_last_known_obstacle_count'};
for j=1:numel(fields)
    assert(isfield(T,fields{j}), ...
        '[R21] Active-reference capture missing: %s',fields{j});
end
T.r21_active_replan_tested=T.r21_active_base_stage1_requests>0;
T.r21_active_replan_stage2_tested=T.r21_active_base_stage2_prepared>0;
T.r21_reference_activation_forbidden=true;
report=fullfile(fileparts(mfilename('fullpath')),'AF7R58R21_RESULT.txt');
fid=fopen(report,'w','n','UTF-8');
assert(fid>=0,'[R21] Report file cannot be written.');
c=onCleanup(@()fclose(fid));
fprintf(fid,'AF7R58R21 | LOWJET HESSIAN OPT + ACTIVE-BASE AUDIT | NO COMMIT\n');
fprintf(fid,'classification=%s\n',T.classification);
fprintf(fid,'stage1_ms=%.9g stage2_ms=%.9g\n',T.stage1_ms,T.stage2_ms);
fprintf(fid,'stage1_cycle_ms=%.9g stage2_cycle_ms=%.9g\n', ...
    T.stage1_cycle_ms,T.stage2_cycle_ms);
fprintf(fid,'stage1_deadline_would_reject=%d stage2_deadline_would_reject=%d\n', ...
    T.deadline_stage1_would_reject,T.deadline_stage2_would_reject);
F=T.r20_profile;
fprintf(fid,'native_packet_ms=%.9g native_mex_ms=%.9g native_internal_ms=%.9g\n', ...
    F.native_packet_ms,F.native_mex_ms,F.native_internal_ms);
fprintf(fid,'stage2_bundle_ms=%.9g\n',F.stage2_bundle_ms);
fprintf(fid,'native_connected=%.9g native_success=%.9g\n', ...
    r21_scalar(T,'native_connected'),r21_scalar(T,'native_success'));
fprintf(fid,'speed/heading/geometry=%.9g/%.9g/%.9g\n', ...
    r21_scalar(T,'native_speed_pass'),r21_scalar(T,'native_heading_pass'), ...
    r21_scalar(T,'native_geometry_pass'));
fprintf(fid,'active_source_stage1_requests=%d\n',T.r21_active_base_stage1_requests);
fprintf(fid,'active_source_native_returns=%d\n',T.r21_active_base_native_returns);
fprintf(fid,'active_source_stage2_prepared=%d\n',T.r21_active_base_stage2_prepared);
fprintf(fid,'last_source_mode=%s last_source_version=%.9g\n', ...
    T.r21_last_source_mode,T.r21_last_source_version);
fprintf(fid,'first_active_replan_time_s=%.12g known_obstacles=%g\n', ...
    T.r21_first_active_replan_time_s,T.r21_last_known_obstacle_count);
fprintf(fid,'ACTUAL_COMMIT=0; active-replan ability is not established unless REPAIRED source was actually tested.\n');
fprintf(fid,'\nEVENTS\n');
for i=1:numel(T.events)
    e=T.events(i);
    fprintf(fid,'%s | %.12g | %.9g | %.9g\n',e.name,e.time_s,e.stage_ms,e.cycle_ms);
end
save(fullfile(fileparts(mfilename('fullpath')),'AF7R58R21_RESULT.mat'),'T');
disp('========== R21 RESULT ==========');
disp(T.classification);
fprintf('Stage1/Stage2 [ms] = %.6f / %.6f\n',T.stage1_ms,T.stage2_ms);
fprintf('Native packet/MEX [ms] = %.6f / %.6f\n', ...
    F.native_packet_ms,F.native_mex_ms);
fprintf('Already-repaired-source requests = %d; Stage2 prepared = %d\n', ...
    T.r21_active_base_stage1_requests,T.r21_active_base_stage2_prepared);
fprintf('Report = %s\n',report);
end
function y=r21_scalar(s,k)
y=NaN;
if isfield(s,k) && isnumeric(s.(k)) && isscalar(s.(k))
    y=double(s.(k));
end
end
