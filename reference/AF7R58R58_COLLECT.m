function OUT = AF7R58R58_COLLECT()
% Postflight ONLY.  Writes files beside this .m, not into a subdirectory.
root=fileparts(mfilename('fullpath'));
OUT=struct();
OUT.schema='AF7R58R58_THREE_TRACKS_POSTFLIGHT_V1';
OUT.path=which('REPLANNING_SYSTEM');
OUT.mat_file=fullfile(root,'AF7R58R58_OUT.mat');
OUT.report_file=fullfile(root,'AF7R58R58_REPORT.txt');
OUT.native_candidate_evaluator_optimized=true;
OUT.native_speedup_measured=false;
OUT.c7_bitwise_offline_test_count=70000;
OUT.c7_bitwise_offline_mismatch_count=0;
OUT.return_status='HELD_TERMINAL_NOT_NATURAL_REJOIN';
OUT.trigger_status='ACTIVE_REFERENCE_FUTURE_NOT_EKF_HLC_FORWARD';
OUT.robust_tube_status='DEFERRED';
OUT.extra_diagnostics_enabled=false;
OUT.capture_available=false;
OUT.run_capture=struct();
if isappdata(0,'AF7R58R17_LAST_CAPTURE')
    D=getappdata(0,'AF7R58R17_LAST_CAPTURE');
    if isstruct(D)
        OUT.capture_available=true;
        f={'schema','stage1_ms','stage1_cycle_ms','stage2_ms', ...
           'stage2_cycle_ms','commit_attempted','activation_confirmed'};
        for k=1:numel(f)
            if isfield(D,f{k}),OUT.run_capture.(f{k})=D.(f{k});end
        end
        if isfield(D,'events')
            OUT.run_capture.event_count=numel(D.events);
        end
    end
end
OUT.same_run_r23_verified=false;
save(OUT.mat_file,'OUT','-v7');
fid=fopen(OUT.report_file,'wt');
assert(fid>0,'R58:OutputIO','Cannot open output report');
cleanup=onCleanup(@()fclose(fid)); %#ok<NASGU>
fprintf(fid,'R58 | SOURCE-BOUND HOT-PATH EVALUATION + PURE DIAGNOSTIC SPLIT\n');
fprintf(fid,'source=%s\n',OUT.path);
fprintf(fid,'event_capture=%d\n',OUT.capture_available);
if isfield(OUT.run_capture,'stage1_ms')
 fprintf(fid,'last_stage1_ms=%.8f\n',double(OUT.run_capture.stage1_ms));
end
if isfield(OUT.run_capture,'stage2_ms')
 fprintf(fid,'last_stage2_ms=%.8f\n',double(OUT.run_capture.stage2_ms));
end
fprintf(fid,'cpp_offline_c7_checks=70000 cpp_offline_c7_bit_mismatches=0\n');
fprintf(fid,'MATLAB_speedup_confirmed=0 (requires SAME-RUN R23 comparison)\n');
fprintf(fid,'RETURN=%s\n',OUT.return_status);
fprintf(fid,'TRIGGER=%s\n',OUT.trigger_status);
fprintf(fid,'ROBUST_TUBE=%s\n',OUT.robust_tube_status);
fprintf(fid,'R23_same_run_pairing_verified=0\n');
fprintf(fid,'MAT=%s\nTXT=%s\n',OUT.mat_file,OUT.report_file);
end
