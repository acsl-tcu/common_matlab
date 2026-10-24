function R57 = AF7R58R57_COLLECT()
% Clean-run capture, read-only AFTER Flight. Must not claim R56 event trace availability.
root=fileparts(mfilename('fullpath'));
R57=struct();
R57.schema='AF7R58R57_CLEAN_DIAGNOSTIC_SPLIT_V1';
R57.source_path=which('REPLANNING_SYSTEM');
R57.mat_file=fullfile(root,'AF7R58R57_OUT.mat');
R57.report_file=fullfile(root,'AF7R58R57_REPORT.txt');
R57.capture_available=false;
R57.detailed_trace_available=false;
R57.r58_bridge_preserved_in_source=true;
R57.r17_research_late_continuation_preserved_in_source=true;
R57.runtime_safety_certified_by_R57=false;
R57.return_modified=false;
R57.trigger_modified=false;
R57.tracking_tube_modified=false;
R57.old_r56_source_hash='SOURCE_SNAPSHOT_ONLY';
R57.run_capture=struct();
if isappdata(0,'AF7R58R17_LAST_CAPTURE')
    D=getappdata(0,'AF7R58R17_LAST_CAPTURE');
    if isstruct(D)
        R57.capture_available=true;
        keep={'schema','stage1_ms','stage1_cycle_ms','stage2_ms', ...
              'stage2_cycle_ms','commit_attempted','activation_confirmed', ...
              'stage3_precommit_ready','r27_commit_flag_at_end', ...
              'r54_stage_ring_count','r55_trigger_ring_count'};
        for k=1:numel(keep)
            if isfield(D,keep{k}), R57.run_capture.(keep{k})=D.(keep{k}); end
        end
        if isfield(D,'events')
            E=D.events;
            R57.run_capture.event_count=numel(E);
        end
        if isfield(D,'r54_stage_ring_count')
            R57.detailed_trace_available=double(D.r54_stage_ring_count)>0;
        end
    end
end
save(R57.mat_file,'R57','-v7');
f=fopen(R57.report_file,'wt');
assert(f>0,'R57:IO','Cannot write R57 report');
c=onCleanup(@()fclose(f)); %#ok<NASGU>
fprintf(f,'R57 | CLEAN FLIGHT DIAGNOSTIC OVERHEAD | POSTFLIGHT ONLY\n');
fprintf(f,'source=%s\n',R57.source_path);
fprintf(f,'capture=%d detailed_trace=%d\n',R57.capture_available,R57.detailed_trace_available);
fprintf(f,'R58R16 production bridge remains ON as in R56 (NOT disposable diagnostic).\n');
fprintf(f,'R17 late continuation mode is unchanged; its deadline behavior is RESEARCH ONLY.\n');
fprintf(f,'R47/R54/R56 event timing rings and full R55 trigger packets are skipped by default.\n');
fprintf(f,'RETURN=UNCHANGED; old held terminal, phase rejoin not proved.\n');
fprintf(f,'TRIGGER=UNCHANGED; active-reference based 4s future; EKF HLC/plant future not installed.\n');
fprintf(f,'ROBUST_TUBE=DEFERRED; whole-body execution safety NOT proved by R57.\n');
fprintf(f,'See SAME-RUN R23 spike report for full/replanner times.\n');
fprintf(f,'MAT=%s\nTXT=%s\n',R57.mat_file,R57.report_file);
end
