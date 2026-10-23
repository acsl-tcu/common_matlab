function R = AF7R58R39R1_AUTO_REPORT(matPath)
% AF7R58R39R1 - post-run three-track *evidence* report.
% NO guessed estimator state, model, mass, cable length, or reference jets.
% NO Flight-time I/O, no trigger/commit or safety modification.
% Operates with the saved R23 timing MAT. Uses live LOGGER if available.
if nargin == 0
    matPath = fullfile(pwd,'AF7R58R23_SPIKE_REPORT.mat');
end
assert((ischar(matPath)||isstring(matPath)) && isscalar(string(matPath)), ...
    'Provide a single explicit R23 timing MAT path.');
matPath = char(matPath);
assert(isfile(matPath), '[R39R1] Saved timing MAT not found: %s',matPath);

R = struct();
R.schema = 'AF7R58R39R1_AUTO_REPORT_V1';
R.input_timing_file = matPath;
R.timing = AF7R58R39_TIMING_CLOSURE(matPath);
R.full_cycle_closure_verified = R.timing.max_full_residual_ms <= ...
    128*eps(max(1,max(abs(R.timing.all_cycles.Full_ms))));
R.internal_closure_verified = false;
R.internal_closure_reason = 'Internal per-cycle timings not available in R23 MAT.';

% Production trigger authority is SOURCE-BOUND, not inferred from R23 timings.
R.trigger = struct();
R.trigger.status = 'PREDICTOR_NOT_CAPTURED_IN_R23_TIMING_FILE';
R.trigger.current_estimated_origin_verified = false;
R.trigger.closed_loop_dynamics_certified = false;
R.trigger.production_trigger_switched = false;
R.trigger.source = '';
src = fullfile(pwd,'REPLANNING_SYSTEM.m');
if isfile(src)
    s = fileread(src);
    R.trigger.source = src;
    % These are diagnostic source indicators, not a proof of which branch ran.
    R.trigger.reference_packet_source_present = contains(s, ...
        'obj.get_active_reference_packet(time,cha,varargin,xd_nom)');
    R.trigger.reference_future_call_present = contains(s, ...
        'obj.trigger_core(active,pL_ref,q_ref,time.t)');
    if R.trigger.reference_packet_source_present && ...
            R.trigger.reference_future_call_present
        R.trigger.status = 'SOURCE_USES_ACTIVE_REFERENCE_FUTURE_FOR_TRIGGER';
    else
        R.trigger.status = 'SOURCE_UNRECOGNIZED_NEEDS_DIRECT_INSPECTION';
    end
else
    R.trigger.status = 'PRODUCTION_SOURCE_NOT_AVAILABLE';
end

% For REAL return-shape samples use actual LOGGER rather than inventing
% Creturn: a C7 Bezier candidate is NOT reconstructible from R23 timers.
R.return_shape = struct('status','NO_LIVE_LOGGER_OR_RUNTIME_PROFILE', ...
    'signed_projection_evaluated',false, ...
    'continuous_time_shape_certified',false, ...
    'wholebody_collision_certified',false, ...
    'return_candidate_committed',false);
if evalin('base','exist(''logger'',''var'') == 1') && ...
        evalin('base','exist(''runtime_profile'',''var'') == 1')
    try
        lg = evalin('base','logger');
        rp = evalin('base','runtime_profile');
        S = load(matPath,'T');
        flightMask=(rp.phase(:)=='f');
        flightTime=double(rp.t(flightMask));flightTime=flightTime(:);
        savedTime=double(S.T.time_s(:));
        assert(isequal(size(flightTime),size(savedTime)), ...
            '[R39R1] LOGGER run and timing MAT have different sample counts.');
        assert(all(abs(flightTime-savedTime)<=... 
            32*eps(max(1,max(abs(savedTime))))), ...
            '[R39R1] LOGGER run and timing MAT are not time aligned.');
        % Source-provided R33 routine extracts the real 3D correction vector.
        C = AF7R58R33_LOGGER_RETURN_AUDIT(lg,rp);
        R.return_shape.status = 'SAMPLED_SIGNED_CORRECTION_EVALUATED';
        R.return_shape.signed_projection_evaluated = true;
        R.return_shape.max_correction_m = C.max_correction_m;
        R.return_shape.max_correction_time_s = C.reference_time_s;
        R.return_shape.min_signed_projection_m = C.min_signed_projection_m;
        R.return_shape.min_signed_projection_time_s = C.min_signed_projection_time_s;
        R.return_shape.opposite_side_sample_observed = C.opposite_side_sample_observed;
        R.return_shape.signed_axis = C.signed_axis;
        R.return_shape.continuous_time_shape_certified = false;
        R.return_shape.wholebody_collision_certified = false;
    catch ME
        R.return_shape.status = 'LIVE_LOGGER_RETURN_AUDIT_UNAVAILABLE';
        R.return_shape.error_identifier = ME.identifier;
        R.return_shape.error_message = ME.message;
    end
end

% Find stage event order and planning modes from actual event source.
R.stage_events = R.timing.stage_table;
R.stage1_count = sum(contains(string(R.stage_events.Event),'STAGE1_NATIVE_RETURNED'));
R.stage2_count = sum(contains(string(R.stage_events.Event),'STAGE2_CERTIFIED_AND_PREPARED'));
R.atomic_commit_count = sum(contains(string(R.stage_events.Event),'ATOMIC_COMMIT_PASS'));

save('AF7R58R39R1_AUTO_REPORT.mat','R');
fid=fopen('AF7R58R39R1_AUTO_REPORT.txt','wt');
assert(fid~=-1,'[R39R1] Could not open post-run text report.');
finish = onCleanup(@() fclose(fid));
fprintf(fid,'R39R1 POST-RUN AUDIT | reference only, no safety claim\n');
fprintf(fid,'input=%s\n',R.input_timing_file);
fprintf(fid,'flight_cycles=%d\n',R.timing.n_flight);
fprintf(fid,'reference_ms p50=%.9g p95=%.9g max=%.9g\n', ...
    R.timing.p50_reference_ms,R.timing.p95_reference_ms,R.timing.max_reference_ms);
fprintf(fid,'full_cycle_residual_ms=%.9g\n',R.timing.max_full_residual_ms);
fprintf(fid,'online_cycle_residual_ms=%.9g\n',R.timing.max_online_residual_ms);
fprintf(fid,'internal_closure_verified=%d\n',R.internal_closure_verified);
fprintf(fid,'trigger_status=%s\n',R.trigger.status);
fprintf(fid,'return_status=%s\n',R.return_shape.status);
fprintf(fid,'stage1=%d stage2=%d commit=%d\n', ...
    R.stage1_count,R.stage2_count,R.atomic_commit_count);
if isfield(R.return_shape,'min_signed_projection_m')
    fprintf(fid,'return_signed_min_m=%.9g at_t=%.9g\n', ...
        R.return_shape.min_signed_projection_m,R.return_shape.min_signed_projection_time_s);
end
fprintf('[R39R1] DONE | trigger: %s | return: %s\n', ...
    R.trigger.status,R.return_shape.status);
fprintf('[R39R1] Stage1/Stage2/commit=%d/%d/%d | internal closure NOT YET MEASURED\n', ...
    R.stage1_count,R.stage2_count,R.atomic_commit_count);
end
