function R55 = AF7R58R55_COLLECT()
% Diagnostic-only post-Flight collector; emits directly to this M-file folder.
root=fileparts(mfilename('fullpath'));
R55=struct('schema','AF7R58R55_THREE_TRACKS_SOURCE_BOUND_V1', ...
    'production_source',which('REPLANNING_SYSTEM'), ...
    'tube_deferred',true,'controller_24_95_in_scope',false, ...
    'live_trigger_modified',false,'rejoin_commit_modified',false, ...
    'hard_realtime_proved',false,'wholebody_safety_proved_by_R55',false, ...
    'capture_available',false, 'stage_ring',[], 'stage_columns',{{}}, ...
    'trigger_ring',[],'trigger_packets',{{}},'instantaneous_known_obstacle_hits',[]);
if isappdata(0,'AF7R58R17_LAST_CAPTURE')
    D=getappdata(0,'AF7R58R17_LAST_CAPTURE');
    if isstruct(D) && isfield(D,'r54_stage_ring') && ...
            isfield(D,'r55_trigger_ring')
        R55.capture_available=true;
        n=min(64,double(D.r54_stage_ring_count));
        if n>0
            idx=mod((double(D.r54_stage_ring_head)-n:double(D.r54_stage_ring_head)-1),64)+1;
            R55.stage_ring=double(D.r54_stage_ring(idx,:));
        end
        R55.stage_columns=D.r54_stage_ring_columns;
        n=min(64,double(D.r55_trigger_ring_count));
        if n>0
            idx=mod((double(D.r55_trigger_ring_head)-n:double(D.r55_trigger_ring_head)-1),64)+1;
            R55.trigger_ring=double(D.r55_trigger_ring(idx,:));
            R55.trigger_packets=D.r55_trigger_packets(idx);
        end
    end
end
R55.trigger_columns={'time_s','reference_version', ...
    'pQ_x','pQ_y','pQ_z','pL_x','pL_y','pL_z', ...
    'q_x','q_y','q_z','ref_pL_x','ref_pL_y','ref_pL_z', ...
    'ref_q_x','ref_q_y','ref_q_z','ref_implied_pQ_x','ref_implied_pQ_y','ref_implied_pQ_z', ...
    'ref_pL_to_actual_pQ_m','ref_pL_to_actual_pL_m', ...
    'ref_implied_pQ_to_actual_pQ_m','ref_if_UAV_to_actual_pL_m', ...
    'TTC_ref_s','hit_component_code','cable_length_m','mL_latest_est_kg','known_obstacle_count'};
if ~isempty(R55.trigger_ring)
    assert(size(R55.trigger_ring,2)==numel(R55.trigger_columns), ...
        'R55:Schema','Trigger ring schema mismatch: expected 29.');
end
% Postflight ONLY: evaluate the already installed EXACT collision MEX for a
% constant observed EKF whole-body snapshot. This tests CURRENT geometry
% against the SENSOR-KNOWN obstacle mask, not future closed-loop safety.
R55.instantaneous_known_obstacle_hits=nan(size(R55.trigger_ring,1),4);
if exist('g26_future_trigger_kernel_mex','file')==3
    for i=1:numel(R55.trigger_packets)
        Q=R55.trigger_packets{i};
        if ~isstruct(Q) || ~isfield(Q,'active') || ~any(Q.active)
            continue;
        end
        try
            N=size(Q.pL_ref,2);
            pLnow=repmat(double(Q.pL_EKF(:)),1,N);
            qnow=repmat(double(Q.q_EKF(:)),1,N);
            [hit,idx,id,comp]=g26_future_trigger_kernel_mex( ...
                Q.H,Q.dt,Q.L,pLnow,qnow,Q.centers,Q.rotations, ...
                Q.axes,Q.radii,Q.ids,double(Q.active));
            R55.instantaneous_known_obstacle_hits(i,:)=[double(hit),double(idx),double(id),double(comp)];
        catch
            % Postflight checker reports inability rather than a fake pass.
        end
    end
end
R55.mat_file=fullfile(root,'AF7R58R55_OUT.mat');
R55.report_file=fullfile(root,'AF7R58R55_REPORT.txt');
save(R55.mat_file,'R55','-v7');
f=fopen(R55.report_file,'wt');
assert(f>0,'R55:IO','Cannot create report under %s',root);
c=onCleanup(@()fclose(f));
fprintf(f,'AF7R58R55 | SAME-EVENT MEX SPLIT / REFERENCE FRAME / RETURN\n');
fprintf(f,'source=%s\n',R55.production_source);
fprintf(f,'capture=%d | tube=DEFERRED | production_trigger_changed=0 | return_commit_changed=0\n',R55.capture_available);
fprintf(f,'HLC_24.95_spike_in_scope=0 | continuous_wholebody_proved_by_R55=0\n');
if R55.capture_available
    fprintf(f,'[TIMING] time stage stage_total native_mex MATLAB_packet native_parse native_plan native_result (ms)\n');
    for i=1:size(R55.stage_ring,1)
        Z=R55.stage_ring(i,:);
        if Z(2)==1
            fprintf(f,'%.6f %d %.5f %.5f %.5f %.5f %.5f %.5f\n', ...
                Z(1),round(Z(2)),Z(3),Z(9),Z(8),Z(21),Z(22),Z(23));
            fprintf(f,'  phase_seed_count=%.0f phase_certified=%.0f natural_trials=%.0f natural_selected=%.0f\n', ...
                Z(24),Z(25),Z(26),Z(27));
        elseif Z(2)==2
            fprintf(f,'%.6f %d %.5f Stage2: model=%.5f sensor=%.5f eval_handles=%.5f cache=%.5f\n', ...
                Z(1),round(Z(2)),Z(3),Z(7),Z(8),Z(9),Z(10));
            fprintf(f,'  tangent=%.0f phase=%.0f delay_preserving=%.0f actual_rejoin=%.8g delay=%.8g terminal_offset_norm=%.8g terminal_hold_start=%.8g free_rejoin=%.0f\n', ...
                Z(13),Z(14),Z(15),Z(16),Z(17),Z(24),Z(25),Z(26));
        end
    end
    if size(R55.stage_ring,1)>0 && all(isnan(R55.stage_ring(R55.stage_ring(:,2)==1,21)))
        fprintf(f,'MEX_NOT_REBUILT: native_parse/plan/result times unavailable. Recompile af7r36_single_core_mex.cpp.\n');
    end
    fprintf(f,'[FRAME] time ref_pL-to-EKFpQ ref_pL-to-EKFpL ref_implied_pQ-to-EKFpQ ref_as_UAV_implied_pL-to-EKFpL L known_count\n');
    for i=1:size(R55.trigger_ring,1)
        Z=R55.trigger_ring(i,:);
        fprintf(f,'%.6f %.8f %.8f %.8f %.8f %.6f %.0f\n',Z(1),Z(21),Z(22),Z(23),Z(24),Z(27),Z(29));
        if size(R55.instantaneous_known_obstacle_hits,1)>=i
            W=R55.instantaneous_known_obstacle_hits(i,:);
            fprintf(f,'  EKF_HELD_NOW_OBSTACLE_DIAG: hit=%.0f obstacle=%.0f component=%.0f | NOT a future physical prediction\n',W(1),W(3),W(4));
        end
    end
else
    fprintf(f,'NO_CAPTURE: simulate with R55 source, then call AF7R58R55_COLLECT\n');
end
fprintf(f,'MAT=%s\nTXT=%s\n',R55.mat_file,R55.report_file);
fprintf('R55 output files written to %s\n',root);
end
