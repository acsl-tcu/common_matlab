function R61=AF7R58R61_RUN()
% Only read saved real control points. Outputs right beside this script.
root=fileparts(mfilename('fullpath'));
snapshot=fullfile(root,'AF7R58R44R2_ACTIVE_SNAPSHOT.mat');
assert(isfile(snapshot),'R61:Snapshot','Place bundled R44R2 snapshot beside this script');
C=AF7R58R61_REAL_OFFSET_REJOIN(snapshot,25);
assert(C.candidate_found,'R61:Candidate','No one-sided projected C7 candidate was found');
R61=struct('status','REAL_R44R2_C7_CORRECTION_ONLY__FUTURE_HLC_AND_PHASE_NOT_BOUND', ...
    'real_correction',C,'trigger_status', ...
    'INTERVAL_GATE_AVAILABLE__REQUIRES_REAL_HLC_PLANT_ROLLOUT_AND_CERTIFIED_SPEED_BOUNDS', ...
    'production_modified',false,'production_trigger_modified',false, ...
    'production_rejoin_modified',false,'commit_allowed',false, ...
    'tube_deferred',true,'hlc_preserved',true);
matfile=fullfile(root,'AF7R58R61_OUT.mat');
txtfile=fullfile(root,'AF7R58R61_REPORT.txt');
R61.mat_file=matfile;R61.report_file=txtfile;
save(matfile,'R61','-v7');
f=fopen(txtfile,'w');assert(f>0,'R61:File','Cannot write report');
closer=onCleanup(@()fclose(f));
fprintf(f,'R61 ACTUAL COMMITTED R44R2 OFFSET | NON-COMMITTABLE\n');
fprintf(f,'status=%s\n',R61.status);
fprintf(f,'snapshot_version=%d source_t0_s=%.9f\n', ...
    C.snapshot_version,C.real_source_t0_s);
fprintf(f,'original_offset_norm_m=%.12g\n',norm(C.real_source_offset_m));
fprintf(f,'candidate_found=%d trials=%d\n',C.candidate_found,numel(C.trials));
if C.candidate_found
    P=C.candidate;
    fprintf(f,'selected_T_s=%.12g candidate_end_s=%.12g\n',P.T_s,P.t1_s);
    fprintf(f,'projected_overshoot_cp=0 (source-bound quality screen; only along source offset axis)\n');
    fprintf(f,'transverse_cp_max_m=%.12g speed_cp_UB_m_s=%.12g\n', ...
        P.max_transverse_cp_m,P.speed_cp_upper_m_s);
    fprintf(f,'start_C7_error=%.12g end_C7_error=%.12g\n', ...
        P.start_jet_error,P.end_jet_error);
end
fprintf(f,'NO NOMINAL PHASE AUTHORITY: This is NOT a spatial-path delayed rejoin.\n');
fprintf(f,'NO EKF-INITIALIZED HLC+PLANT ROLLOUT: actual-state 4s trigger NOT installed.\n');
fprintf(f,'Required next input: real live nominal jets + active phase jets;\n');
fprintf(f,'actual HLC+plant forward trajectory with certified continuous speed bounds.\n');
fprintf(f,'No safety commit, no outward rounding, no robustness tube, HLC untouched.\n');
end
