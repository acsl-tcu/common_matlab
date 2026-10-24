function R54 = AF7R58R54_COLLECT()
% Run AFTER normal SimHL flight. Writes next to this source, never subfolders.
root=fileparts(mfilename('fullpath'));
R54=struct('schema','AF7R58R54_EVENT_MATCHED_R1', ...
    'production_source',which('REPLANNING_SYSTEM'), ...
    'controller_24_95_out_of_scope',true, ...
    'tracking_error_tube_deferred',true, ...
    'closed_loop_ekf_trigger_installed',false, ...
    'whole_body_c7_rejoin_proved',false, ...
    'runtime_speedup_proved',false, ...
    'capture_available',false);
if isappdata(0,'AF7R58R17_LAST_CAPTURE')
    D=getappdata(0,'AF7R58R17_LAST_CAPTURE');
    if isstruct(D) && isfield(D,'r54_stage_ring') && ...
            isfield(D,'r54_stage_ring_count')
        R54.capture_available=true;
        R54.stage_ring=D.r54_stage_ring;
        R54.stage_ring_head=D.r54_stage_ring_head;
        R54.stage_ring_count=D.r54_stage_ring_count;
        R54.stage_columns=D.r54_stage_ring_columns;
        R54.snapshot_skipped_count=D.r54_yaw_snapshot_skipped_count;
        R54.boundary=D.r54_last_trigger_boundary;
        if isfield(D,'r20_profile'),R54.r20=D.r20_profile;end
        if isfield(D,'r22_profile'),R54.r22=D.r22_profile;end
        if isfield(D,'events'),R54.events=D.events;end
    end
end
R54.mat_file=fullfile(root,'AF7R58R54_OUT.mat');
R54.report_file=fullfile(root,'AF7R58R54_REPORT.txt');
save(R54.mat_file,'R54','-v7');
f=fopen(R54.report_file,'wt');
assert(f>0,'R54:File','Cannot create R54 report in %s',root);
c=onCleanup(@()fclose(f));
fprintf(f,'AF7R58R54 | SAME-EVENT STAGE SPLIT | R53 -> R54\n');
fprintf(f,'source=%s\n',R54.production_source);
fprintf(f,'capture_available=%d\n',R54.capture_available);
fprintf(f,'track_tube=DEFERRED | actual_state_future_prediction=NOT_YET | return_commit=UNCHANGED\n');
if R54.capture_available
    fprintf(f,'yaw_snapshot_skipped_count=%d\n',R54.snapshot_skipped_count);
    n=min(64,double(R54.stage_ring_count));
    idx=mod((double(R54.stage_ring_head)-n:double(R54.stage_ring_head)-1),64)+1;
    E=double(R54.stage_ring(idx,:));
    fprintf(f,'[STAGE-EVENT-MATCHED] time stage total native/model packet/sensor mex/handles wrapper/cache\n');
    for k=1:size(E,1)
        z=E(k,:);
        if z(2)==1 || z(2)==2
            fprintf(f,'%.9f %d %.6f %.6f %.6f %.6f %.6f\n', ...
                z(1),round(z(2)),z(3),z(7),z(8),z(9),z(10));
            if z(2)==2
                fprintf(f,'  tangent=%.0f phase=%.0f delay_preserving=%.0f rejoin=%.8f delta=%.8f r53shared=%.0f r52reuse=%.0f\n', ...
                    z(13),z(14),z(15),z(16),z(17),z(19),z(20));
            end
        end
    end
    if isstruct(R54.boundary) && isfield(R54.boundary,'position_gap_m')
        fprintf(f,'[EKF-TRIGGER-SHADOW] t=%.6f gap_L=%.10g gap_q=%.10g pred_TTC=%.6f\n', ...
            R54.boundary.t,R54.boundary.position_gap_m, ...
            R54.boundary.cable_dir_gap,R54.boundary.predicted_ttc_s);
    else
        fprintf(f,'[EKF-TRIGGER-SHADOW] NO_TRIGGER_BOUNDARY_SAVED\n');
    end
else
    fprintf(f,'NO_R54_CAPTURE: run the regular simulation with R54 class, then call COLLECT after Flight.\n');
end
fprintf(f,'MAT=%s\nTXT=%s\n',R54.mat_file,R54.report_file);
fprintf('R54 results in %s\n',root);
end
