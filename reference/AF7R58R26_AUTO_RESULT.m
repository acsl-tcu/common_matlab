function R26 = AF7R58R26_AUTO_RESULT()
% Run only AFTER run_flight_headless ends; no controller workspace needed.
key='AF7R58R17_LAST_CAPTURE';
assert(isappdata(0,key), ...
    '[R26] No same-session post-flight R25 capture. Check simulator completed and called r25_publish_capture().');
D=getappdata(0,key);
assert(isstruct(D)&&isfield(D,'events'), '[R26] Invalid capture format.');
E=D.events;
N={E.name};
R26=struct();
R26.sim_commit_attempted=0;
R26.sim_commit_succeeded=0;
if isfield(D,'r26_sim_commit_attempted')
    R26.sim_commit_attempted=double(D.r26_sim_commit_attempted);
end
if isfield(D,'r26_sim_commit_succeeded')
    R26.sim_commit_succeeded=double(D.r26_sim_commit_succeeded);
end
R26.sim_commit_rejected=sum(startsWith(N,'STAGE3_SIM_ONLY_ATOMIC_COMMIT_REJECT__'));
R26.precommit_pass=sum(strcmp(N,'STAGE3_SIM_ONLY_PRECOMMIT_PASS'));
R26.from_repaired=NaN; % Not inferred when R21 source-mode counter is unavailable.
if isfield(D,'r21_active_base_stage1_requests')
    R26.from_repaired=double(D.r21_active_base_stage1_requests);
end
R26.stage1_t_s=[]; R26.stage1_ms=[]; R26.stage1_cycle_ms=[];
R26.stage2_t_s=[]; R26.stage2_ms=[]; R26.stage2_cycle_ms=[];
for i=1:numel(E)
    if strcmp(E(i).name,'STAGE1_NATIVE_RETURNED')
        R26.stage1_t_s(end+1)=E(i).time_s;
        R26.stage1_ms(end+1)=E(i).stage_ms;
        R26.stage1_cycle_ms(end+1)=E(i).cycle_ms;
    elseif strcmp(E(i).name,'STAGE2_CERTIFIED_AND_PREPARED')
        R26.stage2_t_s(end+1)=E(i).time_s;
        R26.stage2_ms(end+1)=E(i).stage_ms;
        R26.stage2_cycle_ms(end+1)=E(i).cycle_ms;
    end
end
R26.events=E;
R26.original_capture=D;
R26.reference_only=true;
R26.simulation_only=true;
R26.realtime_25ms_certified=false;
R26.full_plant_tracking_certified=false;
if R26.sim_commit_succeeded>0
    R26.classification='REFERENCE_ONLY_SIM_ATOMIC_COMMIT_EXECUTED_NOT_FLIGHT_SAFE';
elseif R26.sim_commit_attempted>0
    R26.classification='SIM_ATOMIC_COMMIT_ATTEMPTED_REJECTED';
else
    R26.classification='NO_ATOMIC_COMMIT_ATTEMPT';
end
outdir=fileparts(which(mfilename));
save(fullfile(outdir,'AF7R58R26_RESULT.mat'),'R26');
fid=fopen(fullfile(outdir,'AF7R58R26_RESULT.txt'),'w');
assert(fid>=0,'[R26] Cannot write result report.');
cleanup=onCleanup(@()fclose(fid));
fprintf(fid,'R26 | SIMULATION-ONLY ATOMIC COMMIT + LAZY DIAGNOSTICS\n');
fprintf(fid,'classification=%s\n',R26.classification);
fprintf(fid,'commit attempts / success / rejection = %d / %d / %d\n', ...
    R26.sim_commit_attempted,R26.sim_commit_succeeded,R26.sim_commit_rejected);
fprintf(fid,'Stage1 counts=%d, Stage2 counts=%d, replan from REPAIRED=%d\n', ...
    numel(R26.stage1_ms),numel(R26.stage2_ms),R26.from_repaired);
fprintf(fid,'all times are diagnostic; late proposals cannot certify real-time safe activation\n');
for i=1:numel(E)
 fprintf(fid,'%s | t=%.9f | stage=%.6f | cycle=%.6f\n', ...
    E(i).name,E(i).time_s,E(i).stage_ms,E(i).cycle_ms);
end
fprintf('[R26] %s\n',R26.classification);
fprintf('[R26] commit=%d/%d, replan from REPAIRED=%d\n', ...
    R26.sim_commit_succeeded,R26.sim_commit_attempted,R26.from_repaired);
end
