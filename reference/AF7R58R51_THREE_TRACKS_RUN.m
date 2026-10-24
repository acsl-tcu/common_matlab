% AF7R58R51: latest-source-bound THREE TRACKS. Run in reference directory.
% No source/MEX/controller/commit mutation; no fake physical safety pass.
R51=struct();
R51.schema='AF7R58R51_THREE_TRACKS_SOURCE_BOUND_V1';
R51.production_modified=false;
R51.controller_24_95_spike_in_scope=false;
R51.source=which('REPLANNING_SYSTEM');
R51.return=struct('status','NOT_RUN__SNAPSHOT_ABSENT');
R51.timing=struct('status','NOT_RUN__R23_REPORT_ABSENT');
R51.trigger=struct('status','SOURCE_ONLY__NO_LIVE_CLOSED_LOOP_TUBE');
root=pwd;
ss=fullfile(root,'AF7R58R44R2_ACTIVE_SNAPSHOT.mat');
if isfile(ss)
    R51.return=AF7R58R51_RETURN_C7_SHADOW(ss);
end
logfile=fullfile(root,'AF7R58R23_SPIKE_REPORT.txt');
if isfile(logfile)
    text23=fileread(logfile);
    pat='(?m)^\s*(\d+\.\d+)\s+(\d+\.\d+)\s+(\d+\.\d+)\s+(\d+\.\d+)\s+(\d+\.\d+)\s+(\d+\.\d+)\s+(\d+\.\d+)\s+(\d+\.\d+)';
    hits=regexp(text23,pat,'tokens');
    if ~isempty(hits)
        A=nan(numel(hits),8);
        for k=1:numel(hits)
            A(k,:)=str2double(hits{k});
        end
        R51.timing.status='HISTORICAL_FLIGHT_REPORT__NOT_A_NEW_TIMING_MEASUREMENT';
        R51.timing.top_rows_time_full_online_acq_sensor_ekf_ref_hlc=A;
        [~,i]=max(A(:,7));
        R51.timing.peak_replanner_time_s=A(i,1);
        R51.timing.peak_replanner_ms=A(i,7);
    end
end
% Existing production source ALREADY has R20 precise buckets: reuse them
% if an after-flight published capture is available, never add hotpath logs.
R51.timing.r20_capture_found=false;
if isappdata(0,'AF7R58R17_LAST_CAPTURE')
    C=getappdata(0,'AF7R58R17_LAST_CAPTURE');
    if isstruct(C) && isfield(C,'r20_profile')
        R51.timing.r20_capture_found=true;
        R51.timing.r20_profile=C.r20_profile;
        R51.timing.r20_capture_status= ...
            'FINAL_EVENT_PROFILE_ONLY__NOT_PROVED_SAME_CYCLE_WITH_HISTORICAL_SPIKE';
    end
end
if ~isempty(R51.source) && isfile(R51.source)
    sourceText=fileread(R51.source);
    R51.trigger.source_check=struct(...
        'live_estimator_geometry_reader',contains(sourceText,'get_current_physical_geometry(obj'),...
        'sensor_uses_pQ_cur',contains(sourceText,'raw_sensor_gate(pQ_cur,obj.static_obstacle_map)'),...
        'reference_trigger',contains(sourceText,'trigger_core(active,pL_ref,q_ref,time.t)'),...
        'stage1_reference_initial',contains(sourceText,'pL_a=double(pL_future(:,1))'),...
        'rolling_repaired_reference_present',contains(sourceText,'repaired_reference_packet(obj,t_now)'));
else
    R51.trigger.source_check=struct('source_unavailable',true);
end
R51.trigger.live_ekf_initialized_model_integrated=false;
R51.trigger.controller_error_tube_verified=false;
R51.trigger.continuous_wholebody_proved=false;
R51.trigger.sensor_only_known_obstacles_required=true;
R51.trigger.current_state_not_substitutable_by_reference_packet=true;
R51.trigger.status='SOURCE_BOUND_GAP_CONFIRMED__NO_PHYSICAL_PREDICTION';
R51.return_and_trigger_authority='NO_COMMIT__SHADOW_ONLY';
outdir=fullfile(root,['AF7R58R51_RESULTS_' datestr(now,'yyyymmdd_HHMMSS_FFF')]);
if ~isfolder(outdir),mkdir(outdir);end
R51.mat_file=fullfile(outdir,'AF7R58R51_OUT.mat');
R51.report_file=fullfile(outdir,'AF7R58R51_REPORT.txt');
save(R51.mat_file,'R51','-v7');
fid=fopen(R51.report_file,'w');
assert(fid>0,'R51:Write','Cannot create report.');
cleanup=onCleanup(@()fclose(fid));
fprintf(fid,'AF7R58R51 THREE TRACKS | SOURCE-BOUND | NON-PRODUCTION\n');
fprintf(fid,'production_modified=0 | 24.95s HLC spike ignored=1\n');
fprintf(fid,'source=%s\n',R51.source);
fprintf(fid,'[RETURN] %s\n',R51.return.status);
if isfield(R51.return,'generated') && R51.return.generated
    fprintf(fid,'source_time=%.9f witness_duration=%.9f candidate_end=%.9f\n',...
        R51.return.connection_time_s,R51.return.witness_duration_s,R51.return.witness_end_s);
    fprintf(fid,'old_offset_norm=%.12f CP_signed_max=%.12g speed_Bernstein_ub=%.9f\n',...
        R51.return.old_offset_norm_m,R51.return.projected_max_m, ...
        R51.return.correction_speed_Bernstein_upper_m_s);
    fprintf(fid,'start_C7_jet_max_res=%.9g\n',R51.return.start_jet_residual_max);
end
fprintf(fid,'NO live phase/wind/controller/whole-body certificate from this return witness.\n');
fprintf(fid,'[TIMING] %s\n',R51.timing.status);
if isfield(R51.timing,'peak_replanner_ms')
    fprintf(fid,'replanner_peak_t=%.9f replanner_peak_ms=%.9f\n',...
        R51.timing.peak_replanner_time_s,R51.timing.peak_replanner_ms);
end
fprintf(fid,'existing_R20_capture_found=%d\n',R51.timing.r20_capture_found);
fprintf(fid,'[TRIGGER] %s\n',R51.trigger.status);
fprintf(fid,'live_closed_loop_run=0 | full_execution_tube_proved=0\n');
fprintf(fid,'Source pairing of historical logs and current code is NOT verified.\n');
clear cleanup;
disp(['R51 REPORT: ' R51.report_file]);
disp(['R51 MAT: ' R51.mat_file]);
