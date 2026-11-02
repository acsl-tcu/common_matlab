function D = CHECK_TRIGGER_FIRST_FLIGHT_TRACE(replanner_instance)
% First 4 REAL f-phase trigger ticks only; read-only postflight report.
% TRIGGER_FIRST_FLIGHT_TRACE=1 must be set BEFORE run_flight_headless.
% No sensor, EKF, plant, HLC, trajectory or reference evaluations occur here.
if nargin~=1 || ~isobject(replanner_instance) || ...
        ~ismethod(replanner_instance,'get_status')
    error('CHECK_TRIGGER_FIRST_FLIGHT_TRACE:Input', ...
        'Pass the actual replanner_instance after the normal Flight.');
end
Q=replanner_instance.get_status();
if ~isfield(Q,'first_flight_traces')
    error('CHECK_TRIGGER_FIRST_FLIGHT_TRACE:Version', ...
        'First-flight instrumentation is not installed.');
end
D=struct();
D.status='READ_ONLY_FIRST_FOUR_REAL_FLIGHT_TICKS__NO_SAFETY_PROOF';
D.trace_enabled=Q.first_flight_trace_enabled;
D.records=numel(Q.first_flight_traces);
D.first_flight_traces=Q.first_flight_traces;
D.preflight_ready=Q.preflight_update_ready;
D.preflight_elapsed_ms=Q.preflight_update_ms;
D.mex_preflight_status=Q.native_mex_preflight_build_status;
D.trigger_gate_status=Q.trigger_native_gate_status;
D.mex_source_check=TRIGGER_NATIVE_MODEL_SOURCE_CHECK(true);
D.mex_resolution=which('TRIGGER_SYMBOLIC_NATIVE_CORE','-all');
D.native_binary_expected_sha256= ...
    '6b76fcc4ffcd2d3d96d19d5a74f5d6e36446e6d091befa568808c6776fc8ce54';
D.restored_binary_sha_matches=strcmpi( ...
    D.mex_source_check.native_mex_sha256, D.native_binary_expected_sha256);
if D.records==0
    D.status='NO_FIRST_FLIGHT_TRACE__ENABLE_BEFORE_RUN';
end
folder=fileparts(mfilename('fullpath'));
file_mat=fullfile(folder,'TRIGGER_FIRST_FLIGHT_TRACE_RESULT.mat');
file_txt=fullfile(folder,'TRIGGER_FIRST_FLIGHT_TRACE_RESULT.txt');
save(file_mat,'D');
fid=fopen(file_txt,'wt');
if fid<0
    error('CHECK_TRIGGER_FIRST_FLIGHT_TRACE:TextOpen', ...
        'Cannot create text report in the diagnostic folder.');
end
finish=onCleanup(@()fclose(fid));
fprintf(fid,'status=%s\n',D.status);
fprintf(fid,'trace_enabled=%d records=%d\n',D.trace_enabled,D.records);
fprintf(fid,'preflight_ready=%d preflight_ms=%.6f\n', ...
    D.preflight_ready,D.preflight_elapsed_ms);
fprintf(fid,'preflight_mex=%s\n',D.mex_preflight_status);
fprintf(fid,'flight_gate=%s\n',D.trigger_gate_status);
fprintf(fid,'mex_source_pin=%s passed=%d\n', ...
    D.mex_source_check.status,D.mex_source_check.passed);
fprintf(fid,'compiled_mex_sha256=%s\n',D.mex_source_check.native_mex_sha256);
fprintf(fid,'compiled_mex_path=%s\n',D.mex_source_check.native_path);
fprintf(fid,'compiled_mex_matches_restored=%d\n',D.restored_binary_sha_matches);
fprintf(fid,'mex_resolution_paths:\n');
loc=D.mex_resolution;
if ischar(loc),loc={loc};end
for k=1:numel(loc)
    fprintf(fid,'  %s\n',char(loc{k}));
end
for k=1:D.records
    r=D.first_flight_traces(k);
    fprintf(fid,'\nTICK %d t=%.6f s status=%s\n',k,r.time_s,r.trigger_status);
    keys={'known_frame_ms','source_gate_ms','reference_now_ms', ...
        'authority_create_ms','authority_current_ms','core_wall_ms', ...
        'pre_store_total_ms'};
    for j=1:numel(keys)
        fprintf(fid,'  %-34s %12.6f ms\n',keys{j},r.(keys{j}));
    end
    C=r.core_trace;
    if isstruct(C) && ~isempty(fieldnames(C))
        fields=fieldnames(C);
        for j=1:numel(fields)
            v=C.(fields{j});
            if isnumeric(v) && isscalar(v)
                fprintf(fid,'  core.%-29s %12.6f ms\n',fields{j},v);
            end
        end
        if all(isfield(C,{'predictor_wrapped_wall_ms', ...
                'reference_window_ms','predictor_setup_ms','mex_native_call_ms'}))
            remainder=C.predictor_wrapped_wall_ms - ...
                C.reference_window_ms-C.predictor_setup_ms-C.mex_native_call_ms;
            fprintf(fid,'  core.predictor_wrapper_residual_ms  %.6f\n',remainder);
        end
    end
end
fprintf('[FIRST FLIGHT TRACE] %s | ticks=%d | pin=%s\n', ...
    D.status,D.records,D.mex_source_check.status);
if D.records>0
    disp(D.first_flight_traces(1));
end
end
