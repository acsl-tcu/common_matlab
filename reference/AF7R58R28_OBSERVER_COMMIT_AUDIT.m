function R = AF7R58R28_OBSERVER_COMMIT_AUDIT()
% Source-bound post-failure numeric audit. No safety/observer/commit changes.
key='AF7R58R27_ROLL_FAILURE';
assert(isappdata(0,key), ...
    '[R28] No new R28 observer failure in this MATLAB session.');
R=getappdata(0,key);
assert(isstruct(R)&&isfield(R,'r28_schema') && ...
    strcmp(char(R.r28_schema),'AF7R58R28_ROLL_REACH_AND_COMMIT_V1'), ...
    '[R28] Stale R27 result: run simulation with R28 source first.');
assert(isfield(R,'r28_ring_data') && ...
    size(R.r28_ring_data,2)==16,'[R28] invalid observer trace.');
if R.kmin<R.kmax
    R.classification='NONUNIQUE_LIFT_PROVEN_BY_REACHABLE_SET';
elseif R.kmin>R.kmax
    R.classification='NO_LIFT_INTERSECTS_CERTIFIED_SUPPORT';
else
    R.classification='LIFT_FAILURE_REQUIRES_SOURCE_RECHECK';
end
R.roll_difference=R.roll1-R.roll0;
% Counterfactual two-endpoint pitch reach: DIAGNOSTIC ONLY.
% Variation |theta_dot|<=||omega|| and total integral<=Iw imply
% 2*|theta(t)| <= Iw + |theta(0)| + |theta(h)|.
% This can be significantly tighter than the one-ended pitch reach.
% Keep upward numerical padding; do not promote it to acceptance in R28.
a0=abs(double(R.measured_pitch0))+double(R.r28_pitch0_error_ub);
a1=abs(double(R.measured_pitch1))+double(R.r28_pitch1_error_ub);
Iw=double(R.r28_Iw);
sc=max([1,a0,a1,Iw]);
pad=64*eps(sc);
R.r28_two_end_pitch_ub=0.5*(Iw+a0+a1)+pad;
if isfinite(R.r28_two_end_pitch_ub) && R.r28_two_end_pitch_ub<pi/2
    val=Iw/cos(R.r28_two_end_pitch_ub);
    R.r28_two_end_roll_delta_ub=val+64*eps(max(1,abs(val)));
else
    R.r28_two_end_roll_delta_ub=Inf;
end
R.r28_two_end_branch_only_diagnostic=true;
R.alt_k_abs_min=min(abs(R.roll_difference+2*pi), ...
    abs(R.roll_difference-2*pi));
R.conditional_true_omega_norm=norm(R.r28_plant_true_omega);
R.conditional_estimated_omega_norm=norm(R.r28_estimated_omega);
R.r28_event_count=numel(R.r28_events);
if R.r28_event_count>0
    names={R.r28_events.name};
    R.r28_sim_success_events=sum(strcmp(names,'STAGE3_SIM_ONLY_ATOMIC_COMMIT_PASS'));
    isActive=strcmp({R.r28_events.source_mode},'REPAIRED');
    isStage1=strcmp(names,'STAGE1_ENTERED');
    R.r28_repaired_source_stage1_events=sum(isActive & isStage1);
else
    R.r28_sim_success_events=0;
    R.r28_repaired_source_stage1_events=0;
end
outputDir=fileparts(which(mfilename));
save(fullfile(outputDir,'AF7R58R28_OBSERVER_COMMIT_AUDIT.mat'),'R');
fid=fopen(fullfile(outputDir,'AF7R58R28_OBSERVER_COMMIT_AUDIT.txt'),'w');
assert(fid>=0,'[R28] output TXT unavailable.');
cc=onCleanup(@()fclose(fid));
fprintf(fid,'R28 | OBSERVER REACH/COMMIT | SIMULATION DIAGNOSTIC ONLY\n');
fprintf(fid,'classification=%s\n',R.classification);
fprintf(fid,'t_s=%.12f\n',R.t_s);
fprintf(fid,'roll0=%.15g roll1=%.15g difference=%.15g\n', ...
    R.roll0,R.roll1,R.roll_difference);
fprintf(fid,'roll_Dq_ub=%.15g branch_count=%g kmin=%g kmax=%g\n', ...
    R.roll_displacement_ub,R.branch_count,R.kmin,R.kmax);
fprintf(fid,'roll_2pi_alternative_min_abs=%.15g\n',R.alt_k_abs_min);
fprintf(fid,'omega_W0_ub=%.15g omega_Wf_ub=%.15g\n',R.r28_W0,R.omega_reach_ub);
fprintf(fid,'Iw_ub=%.15g Theta_ub=%.15g\n',R.r28_Iw,R.pitch_reach_ub);
fprintf(fid,'two_endpoint_Theta_ub_DIAGNOSTIC=%.15g\n',R.r28_two_end_pitch_ub);
fprintf(fid,'two_endpoint_roll_Dq_ub_DIAGNOSTIC=%.15g\n', ...
    R.r28_two_end_roll_delta_ub);
fprintf(fid,'energy_sqrt_start=%.15g energy_sqrt_end=%.15g\n', ...
    R.r28_energy_sqrt_start,R.r28_energy_sqrt_end);
fprintf(fid,'held_u=[%.15g %.15g %.15g %.15g]\n',R.r28_held_input);
fprintf(fid,'estimated_omega=[%.15g %.15g %.15g]\n',R.r28_estimated_omega);
fprintf(fid,'SIM_ONLY_numeric_plant_omega=[%.15g %.15g %.15g]\n',R.r28_plant_true_omega);
fprintf(fid,'SIM_ONLY_numeric_plant_euler=[%.15g %.15g %.15g]\n',R.r28_plant_true_euler);
fprintf(fid,'active_ref_mode=%s version=%g\n',R.r28_source_mode,R.r28_source_version);
fprintf(fid,'commit_attempts=%g success=%g stage_commits=%g\n', ...
    R.r28_commit_attempted,R.r28_commit_succeeded,R.r28_stage_commit_count);
fprintf(fid,'active_source_stage1_count=%g\n',R.r28_active_source_requests);
fprintf(fid,'sim_commit_success_events=%d active_source_events=%d\n', ...
    R.r28_sim_success_events,R.r28_repaired_source_stage1_events);
fprintf(fid,'\nLATEST OBSERVER STEPS (simulated true omega is NOT a proof input):\n');
fprintf(fid,'%s\n',strjoin(R.r28_ring_columns,','));
for i=1:size(R.r28_ring_data,1)
    fprintf(fid,'%s\n',strjoin(arrayfun(@(x)sprintf('%.12g',x), ...
        R.r28_ring_data(i,:),'UniformOutput',false),','));
end
fprintf(fid,'\nPIPELINE EVENTS\n');
for i=1:numel(R.r28_events)
    E=R.r28_events(i);
    fprintf(fid,'%.9f | %s | mode=%s | version=%g | %.6f ms\n', ...
        E.time_s,E.name,E.source_mode,E.source_version,E.stage_ms);
end
fprintf(fid,'\nThe exact original roll uniqueness assertion remains enforced.\n');
fprintf(fid,'Controller/plant observations are diagnostics and are not certificates.\n');
clear cc;
fprintf('[R28] captured %d observer steps, %d commit successes, %d repaired-source requests.\n', ...
    size(R.r28_ring_data,1),R.r28_sim_success_events,R.r28_repaired_source_stage1_events);
end
