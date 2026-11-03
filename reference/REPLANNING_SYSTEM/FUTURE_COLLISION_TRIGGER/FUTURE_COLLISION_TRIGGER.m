classdef FUTURE_COLLISION_TRIGGER < handle
    % FUTURE_COLLISION_TRIGGER  Observed-only, current-EKF prediction gate.
    %
    % IMPORTANT:
    % - This module DOES NOT implement or replace the existing HLC/plant.
    % - It does not extrapolate p+v*t, invent model coefficients, or use
    %   nominal reference positions as predicted actual positions.
    % - Runtime dt/geometry/cable values MUST arrive from a validated
    %   Flight binding on EVERY evaluate() call. Only EKF mass changes each
    %   tick; constructor caches only optional horizon=4 s.
    % - A side-effect-free predictor using the LIVE HLC + suspended-load
    %   dynamics must be supplied explicitly as authority.predictor.
    % - Without it, the result is UNKNOWN_MODEL_NOT_BOUND, never SAFE.
    % - A nominal sampled prediction does NOT certify continuous safety.
    % - Only sensor_frame.detected_obstacles may reach this class.
    % - No source-world lookup, future sensing or subset-combination caches.
    %
    % Predictor contract, one call per evaluation:
    %   pred = authority.predictor(request)
    %   request fields: time_s, horizon_s, sample_dt_s, sample_times_s,
    %     ekf_now (all 8 physical 3-vectors), payload_mass_kg,
    %     cable_length_m, reference_at (function handle).
    %   pred fields: sample_times_s (same values), uav_m (3xK),
    %     payload_m (3xK), model_description (nonempty char/string).
    %   Optional proof fields: interval_uav_excursion_m (1x[K-1]),
    %     interval_payload_excursion_m (1x[K-1]),
    %     interval_bounds_verified (true), tracking_error_included (true),
    %     estimator_error_included (true), taut_cable_verified (true).
    %   The excursion bounds must apply to every time in each interval,
    %   measured from that interval's LEFT sample, and include ALL indicated
    %   errors. Merely setting a boolean is not a proof: evidence for the
    %   external predictor and bounds must be independently audited.
    %
    % Geometry: support-plane bounds for full straight cable, UAV sphere and
    % payload sphere, for each time interval. Supports convex hull of each
    % actual obstacle shape; this can yield UNKNOWN, never unsafe SAFE.
    % Sampling alone can indicate a predicted collision but cannot prove
    % safety between samples.

    properties (SetAccess=private)
        horizon_s
        sample_dt_s = NaN
        r_uav_m = NaN
        r_payload_m = NaN
        r_cable_m = NaN
        cable_length_m = NaN
        last_result = struct()
        result_template = struct() % Immutable defaults, copied per call
        evaluation_count = 0
        total_elapsed_ms = 0
        maximum_elapsed_ms = 0
        active_request_ids = zeros(1,0)
        % Exact per-observation snapshot cache (not detection-mask presets).
        % Any source geometry or ID change triggers full revalidation.
        last_prepared_observations = []
        last_prepared_shapes = []
        % Fully validated ONCE per sensor memory geometry generation.
        % Only exact same-source static geometry may use the inexpensive
        % token/ID test. Moving geometry is always revalidated.
        last_verified_prepared_ids = zeros(1,0)
        last_verified_memory_version = []
        last_verified_geometry_version = []
        last_verified_prepared_shapes = []
        % A Flight-independent time grid is computed ONLY from the real
        % preflight dt and the externally supplied horizon. No sensor/EKF.
        static_grid_dt = NaN
        static_grid_horizon = NaN
        static_grid_values = zeros(1,0)
        static_grid_build_count = 0
        % Cached output of already VERIFIED observed shapes, never source
        % world truth. Invalidated by the observation/geometry generation.
        static_outer_ids = zeros(1,0)
        static_outer_memory_version = []
        static_outer_geometry_version = []
        static_outer_balls = zeros(5,0)
        static_outer_build_count = 0
        static_outer_reuse_count = 0
        % Differential processing of REAL, append-only observed static IDs.
        incremental_verified_appends = 0
        incremental_outer_appends = 0
    end

    methods
        function obj = FUTURE_COLLISION_TRIGGER(config)
            assert(nargin==1 && isstruct(config) && isscalar(config), ...
                'FUTURE_COLLISION_TRIGGER:Config', 'A scalar authority struct is required.');
            % Only the trigger's horizon has a module-level default.
            % No physical fallback is stored here. evaluate() requires
            % the external Flight binding with every invocation.
            if isfield(config,'horizon_s')
                obj.horizon_s=FUTURE_COLLISION_TRIGGER.positive(config.horizon_s,'horizon_s');
            else
                obj.horizon_s=4;
            end
            % Software-only constant schema; no obstacle or future state.
            obj.result_template=FUTURE_COLLISION_TRIGGER.make_result_template();
        end

        function set_horizon(obj,horizon_s)
            % Explicit external update; never silently clamp or shorten H.
            obj.horizon_s=FUTURE_COLLISION_TRIGGER.positive(horizon_s,'horizon_s');
        end

        function prepare_static_timing(obj,dt,horizon)
            % This may be run before Flight because it uses only the real
            % externally bound static time configuration, never actual
            % measurements, references, obstacles or generated motion.
            dt=FUTURE_COLLISION_TRIGGER.positive(dt,'sample_dt_s');
            horizon=FUTURE_COLLISION_TRIGGER.positive(horizon,'horizon_s');
            obj.horizon_s=horizon;
            if isequal(obj.static_grid_dt,dt) && ...
                    isequal(obj.static_grid_horizon,horizon) && ...
                    ~isempty(obj.static_grid_values)
                return
            end
            n=floor(horizon/dt);
            grid=(0:n)*dt;
            grid=grid(grid<horizon);
            grid=[grid,horizon];
            assert(isrow(grid) && numel(grid)>=2 && ...
                all(isfinite(grid)) && all(diff(grid)>0) && ...
                isequal(grid(end),horizon), ...
                'FUTURE_COLLISION_TRIGGER:StaticGrid', ...
                'Live dt/horizon do not form a valid complete time grid.');
            obj.static_grid_values=grid;
            obj.static_grid_dt=dt;
            obj.static_grid_horizon=horizon;
            obj.static_grid_build_count=obj.static_grid_build_count+1;
        end

        function out=static_preparation_status(obj)
            % Metadata only; contains no actual future/reference data.
            out=struct('dt_s',obj.static_grid_dt, ...
                'horizon_s',obj.static_grid_horizon, ...
                'grid_points',numel(obj.static_grid_values), ...
                'grid_build_count',obj.static_grid_build_count, ...
                'outer_balls_build_count',obj.static_outer_build_count, ...
                'outer_balls_reuse_count',obj.static_outer_reuse_count, ...
                'incremental_verified_appends',obj.incremental_verified_appends, ...
                'incremental_outer_appends',obj.incremental_outer_appends);
        end

        function R = evaluate(obj,time_s,ekf_now,sensor_frame,authority)
            started=tic;
            trace_first=isstruct(authority) && ...
                isfield(authority,'trace_first') && ...
                isequal(authority.trace_first,true);
            assert(nargin==5 && isscalar(time_s) && isnumeric(time_s) && ...
                isreal(time_s) && isfinite(time_s), ...
                'FUTURE_COLLISION_TRIGGER:Time','Current time must be finite.');
            ekf=FUTURE_COLLISION_TRIGGER.check_ekf(ekf_now);
            % Numeric values are supplied from the current Flight binding.
            % A separate Flight must rebind, and the EKF mass is refreshed each tick.
            assert(isstruct(authority) && isscalar(authority) && ...
                isfield(authority,'runtime') && ...
                isstruct(authority.runtime) && isscalar(authority.runtime), ...
                'FUTURE_COLLISION_TRIGGER:RuntimeAuthority', ...
                'A per-tick authority.runtime snapshot is required; no cached parameters.');
            live=authority.runtime;
            keys={'sample_dt_s','r_uav_m','r_payload_m','r_cable_m','cable_length_m'};
            assert(all(isfield(live,keys)), ...
                'FUTURE_COLLISION_TRIGGER:RuntimeAuthority', ...
                'Current dt, three body radii and cable length must be supplied each tick.');
            obj.sample_dt_s=FUTURE_COLLISION_TRIGGER.positive(live.sample_dt_s,'sample_dt_s');
            obj.r_uav_m=FUTURE_COLLISION_TRIGGER.positive(live.r_uav_m,'r_uav_m');
            obj.r_payload_m=FUTURE_COLLISION_TRIGGER.positive(live.r_payload_m,'r_payload_m');
            obj.r_cable_m=FUTURE_COLLISION_TRIGGER.nonnegative(live.r_cable_m,'r_cable_m');
            obj.cable_length_m=FUTURE_COLLISION_TRIGGER.positive(live.cable_length_m,'cable_length_m');
            if isfield(live,'horizon_s')
                obj.horizon_s=FUTURE_COLLISION_TRIGGER.positive(live.horizon_s,'horizon_s');
            end
            assert(isstruct(authority)&&isscalar(authority) && ...
                isfield(authority,'payload_mass_kg') && ...
                isfield(authority,'reference_at'), ...
                'FUTURE_COLLISION_TRIGGER:Authority', ...
                'Latest estimator mass and future reference evaluator are required.');
            mass=FUTURE_COLLISION_TRIGGER.positive(authority.payload_mass_kg,'payload_mass_kg');
            assert(isa(authority.reference_at,'function_handle'), ...
                'FUTURE_COLLISION_TRIGGER:Reference', ...
                'The unchanged nominal reference must be evaluable at future times.');
            known=FUTURE_COLLISION_TRIGGER.check_observation(sensor_frame,time_s,ekf.p);
            ids=reshape([known.id],1,[]);
            grid=obj.make_grid();
            % The SAME defaults as the previous 45+ field literal, but the
            % field-name hash/schema is built just once in the constructor.
            % Struct copy-on-write and explicit dynamic overwrites prevent
            % any state leaking between Flight ticks. No safety checks removed.
            R=obj.result_template;
            R.time_s=double(time_s);
            R.horizon_s=obj.horizon_s;
            R.sample_dt_s=obj.sample_dt_s;
            R.sample_count=numel(grid);
            R.considered_obstacle_ids=ids;
            R.considered_count=numel(ids);
            R.current_payload_mass_kg=mass;
            R.current_cable_length_m=obj.cable_length_m;
            R.current_uav_radius_m=obj.r_uav_m;
            R.current_payload_radius_m=obj.r_payload_m;
            R.current_cable_radius_m=obj.r_cable_m;
            R.flight_binding_used=isfield(authority,'authority_binding_policy');
            % A longer horizon NEVER introduces newly sensed obstacles.
            % An empty observed set does not mean the unknown world is safe.
            if isempty(known)
                R.status='NO_CURRENTLY_DETECTED_OBSTACLES';
                R.reason='No observed obstacles to evaluate; unseen world remains unknown.';
                obj.active_request_ids=zeros(1,0);
                R=obj.finish(R,started);
                return
            end
            if ~(isfield(authority,'verified_native_binding') && ...
                    isstruct(authority.verified_native_binding)) && ...
                    (~isfield(authority,'predictor') || ...
                     ~isa(authority.predictor,'function_handle'))
                R.status='UNKNOWN_MODEL_NOT_BOUND';
                R.reason='Live HLC + suspended-load dynamics predictor is not connected.';
                R.possible_risk_ids=ids; % unknown cannot be asserted clear
                obj.active_request_ids=intersect(obj.active_request_ids,ids,'stable');
                R=obj.finish(R,started);
                return
            end
            request=struct('time_s',double(time_s),'horizon_s',obj.horizon_s, ...
                'sample_dt_s',obj.sample_dt_s,'sample_times_s',grid, ...
                'ekf_now',ekf,'payload_mass_kg',mass, ...
                'cable_length_m',obj.cable_length_m, ...
                'reference_at',authority.reference_at);
            if isfield(authority,'live_physics')
                request.live_physics=authority.live_physics;
            end
            if isfield(authority,'hlc_gains')
                request.hlc_gains=authority.hlc_gains;
            end
            % Perform FIRST-USE validation BEFORE the current-tick predictor.
            % Previously this was first done AFTER the predictor, forcing the
            % very first Flight tick and each new observation generation down
            % the slow, non-fused path. This reads ONLY the current real
            % sensor/memory snapshot already checked above; no world lookup,
            % preflight observation, past-state substitution, or synthetic scan.
            % A changed/moving/invalid shape is NEVER accepted by this path.
            first_verified=false;
            if trace_first, phase_clock=tic; end
            first_use_incremental_verified=false;
            first_use_incremental_outer=false;
            has_provenance=isfield(sensor_frame,'observation_provenance') && ...
                strcmp(sensor_frame.observation_provenance, ...
                       'ACTUAL_SENSOR_HISTORY_ONLY') && ...
                isfield(sensor_frame,'memory_version') && ...
                isfield(sensor_frame,'geometry_version') && ...
                isa(sensor_frame.memory_version,'uint64') && ...
                isa(sensor_frame.geometry_version,'uint64');
            if has_provenance && ...
                    (~isequal(sensor_frame.memory_version, ...
                              obj.last_verified_memory_version) || ...
                     ~isequal(sensor_frame.geometry_version, ...
                              obj.last_verified_geometry_version) || ...
                     ~isequal(ids,obj.last_verified_prepared_ids) || ...
                     isempty(obj.last_verified_prepared_shapes)) && ...
                    isfield(sensor_frame,'prepared_sensor_shapes')
                candidate=sensor_frame.prepared_sensor_shapes;
                if isstruct(candidate) && numel(candidate)==numel(ids) && ...
                        ~isempty(candidate) && ...
                        all(isfield(candidate,{'id','is_stationary'})) && ...
                        isequal(reshape(double([candidate.id]),1,[]),ids) && ...
                        all([candidate.is_stationary])
                    verified=false;
                    old_n=numel(obj.last_verified_prepared_ids);
                    % Real OBSERVED_OBSTACLE_MEMORY is append-only for IDs.
                    % A prior static prefix is immutable until its geometry
                    % generation changes. Never trust a new prefix from the
                    % incoming packet: retain the PREVIOUSLY verified one.
                    can_append=old_n>0 && old_n<numel(ids) && ...
                        isa(obj.last_verified_memory_version,'uint64') && ...
                        isa(obj.last_verified_geometry_version,'uint64') && ...
                        isequal(sensor_frame.geometry_version, ...
                            obj.last_verified_geometry_version) && ...
                        sensor_frame.memory_version>obj.last_verified_memory_version && ...
                        sensor_frame.memory_version-obj.last_verified_memory_version== ...
                            uint64(numel(ids)-old_n) && ...
                        isequal(ids(1:old_n),obj.last_verified_prepared_ids) && ...
                        numel(obj.last_verified_prepared_shapes)==old_n && ...
                        isequal(reshape(double([candidate(1:old_n).id]),1,[]), ...
                            obj.last_verified_prepared_ids);
                    if can_append
                        first_use_incremental_verified=true;
                        % Verify only the genuinely newly observed suffix.
                        % This is valid only for the immutable static sensor
                        % geometry contract, not for arbitrary mutable data.
                        fresh=candidate(old_n+1:end);
                        verified=FUTURE_COLLISION_TRIGGER. ...
                            verified_prepared_observations(known(old_n+1:end),fresh);
                        if verified
                            candidate=[obj.last_verified_prepared_shapes fresh];
                            obj.incremental_verified_appends= ...
                                obj.incremental_verified_appends+1;
                        end
                    else
                        verified=FUTURE_COLLISION_TRIGGER. ...
                            verified_prepared_observations(known,candidate);
                    end
                    if verified
                        obj.last_verified_prepared_ids=ids;
                        obj.last_verified_memory_version= ...
                            sensor_frame.memory_version;
                        obj.last_verified_geometry_version= ...
                            sensor_frame.geometry_version;
                        obj.last_verified_prepared_shapes=candidate;
                        first_verified=true;
                    end
                end
            end
            if trace_first
                phase_prepared_verify_ms=1000*toc(phase_clock);
                phase_clock=tic;
            end
            R.geometry_first_use_verified_before_predictor=first_verified;
            % Coarse geometry is optional. It is available on the VERY FIRST
            % valid observation generation, as well as unchanged later ticks.
            % It remains a sampled-only exclusion, never a continuous SAFE.
            native_outer_ready=false;
            if isfield(sensor_frame,'observation_provenance') && ...
                    strcmp(sensor_frame.observation_provenance, ...
                        'ACTUAL_SENSOR_HISTORY_ONLY') && ...
                    isfield(sensor_frame,'memory_version') && ...
                    isfield(sensor_frame,'geometry_version') && ...
                    isa(sensor_frame.memory_version,'uint64') && ...
                    isa(sensor_frame.geometry_version,'uint64') && ...
                    isequal(sensor_frame.memory_version, ...
                        obj.last_verified_memory_version) && ...
                    isequal(sensor_frame.geometry_version, ...
                        obj.last_verified_geometry_version) && ...
                    isequal(ids,obj.last_verified_prepared_ids) && ...
                    numel(obj.last_verified_prepared_shapes)==numel(ids) && ...
                    ~isempty(obj.last_verified_prepared_shapes)
                native_shapes=obj.last_verified_prepared_shapes;
                if isstruct(native_shapes) && ...
                        all(isfield(native_shapes,{'center0', ...
                        'bounding_radius_m','is_stationary'}))
                    % A verified static observed geometry generation may
                    % reuse the IDENTICAL outer balls for the next tick.
                    % This never authorizes a new ID or an unsensed shape.
                    same_outer=isequal(ids,obj.static_outer_ids) && ...
                        isequal(sensor_frame.memory_version, ...
                                obj.static_outer_memory_version) && ...
                        isequal(sensor_frame.geometry_version, ...
                                obj.static_outer_geometry_version) && ...
                        isequal(size(obj.static_outer_balls),[5,numel(ids)]);
                    if same_outer
                        native_balls=obj.static_outer_balls;
                        native_outer_ready=true;
                        obj.static_outer_reuse_count= ...
                            obj.static_outer_reuse_count+1;
                    else
                        % Previously checked outer balls are immutable for
                        % static observed shapes. When only real NEW IDs
                        % were appended, validate/build ONLY that suffix.
                        old_outer_n=numel(obj.static_outer_ids);
                        append_outer=old_outer_n>0 && old_outer_n<numel(ids) && ...
                            isa(obj.static_outer_memory_version,'uint64') && ...
                            isa(obj.static_outer_geometry_version,'uint64') && ...
                            isequal(obj.static_outer_geometry_version, ...
                                sensor_frame.geometry_version) && ...
                            sensor_frame.memory_version>obj.static_outer_memory_version && ...
                            sensor_frame.memory_version-obj.static_outer_memory_version== ...
                                uint64(numel(ids)-old_outer_n) && ...
                            isequal(obj.static_outer_ids,ids(1:old_outer_n)) && ...
                            isequal(size(obj.static_outer_balls),[5 old_outer_n]);
                        if append_outer
                            first_use_incremental_outer=true;
                            native_balls=[obj.static_outer_balls zeros(5,numel(ids)-old_outer_n)];
                            first_outer_new=old_outer_n+1;
                        else
                            native_balls=zeros(5,numel(ids));
                            first_outer_new=1;
                        end
                        native_outer_ready=true;
                        for native_j=first_outer_new:numel(ids)
                            sh0=native_shapes(native_j);
                            c0=double(sh0.center0(:));
                            r0=double(sh0.bounding_radius_m);
                            if numel(c0)~=3 || ~all(isfinite(c0)) || ...
                                    ~isscalar(r0) || ~isfinite(r0) || r0<0 || ...
                                    ~isscalar(sh0.is_stationary) || ...
                                    ~isequal(sh0.is_stationary,true)
                                native_outer_ready=false;
                                break
                            end
                            native_balls(:,native_j)=[c0;r0;1];
                        end
                        if native_outer_ready
                            if append_outer
                                obj.incremental_outer_appends= ...
                                    obj.incremental_outer_appends+1;
                            end
                            obj.static_outer_ids=ids;
                            obj.static_outer_memory_version= ...
                                sensor_frame.memory_version;
                            obj.static_outer_geometry_version= ...
                                sensor_frame.geometry_version;
                            obj.static_outer_balls=native_balls;
                            obj.static_outer_build_count= ...
                                obj.static_outer_build_count+1;
                        else
                            obj.static_outer_ids=zeros(1,0);
                            obj.static_outer_memory_version=[];
                            obj.static_outer_geometry_version=[];
                            obj.static_outer_balls=zeros(5,0);
                        end
                    end
                    if native_outer_ready
                        request.native_sampled_outer_balls=native_balls;
                        request.native_sampled_outer_ids=ids;
                        request.native_sampled_body_radii= ...
                            [obj.r_uav_m;obj.r_payload_m;obj.r_cable_m];
                    end
                end
            end
            if trace_first
                phase_outer_build_ms=1000*toc(phase_clock);
                phase_clock=tic;
            end
            assert(isfield(authority,'numeric_tolerances'), ...
                'FUTURE_COLLISION_TRIGGER:Accuracy', ...
                'Numerical tolerances must come from live run configuration.');
            request.numeric_tolerances=authority.numeric_tolerances;
            if trace_first
                request.first_use_trace=true;
                first_pre_predictor_ms=1000*toc(started);
            end
            try
                predictor_wall_clock=tic;
                % Original real Flight authority exposes its validated
                % source binding and reference cache directly. The legacy
                % function-handle branch is kept ONLY for callers without
                % the explicit verified native interface (e.g. regression).
                % No future reference value is sampled before this tick.
                direct_native=isfield(authority,'verified_native_binding') && ...
                    isfield(authority,'native_reference_cache') && ...
                    isfield(authority,'native_reference_source') && ...
                    isfield(authority,'native_validated_now_time_s') && ...
                    isfield(authority,'native_validated_now_jet');
                if direct_native
                    reference_clock=tic;
                    abs_times=double(request.time_s)+ ...
                        double(request.sample_times_s(1:end-1));
                    contract_ok=isfield(authority,'native_reference_contract') && ...
                        strcmp(authority.native_reference_contract, ...
                          'VERIFIED_F_PHASE_NOMINAL_Z_MINUS_CABLEL');
                    if isequal(abs_times(1),double(authority.native_validated_now_time_s)) && contract_ok
                        [request.reference_matrix,~]= ...
                            authority.native_reference_cache.evaluate( ...
                                authority.native_reference_source,abs_times, ...
                                request.cable_length_m,request.reference_at, ...
                                authority.native_validated_now_jet, ...
                                'VERIFIED_F_PHASE_NOMINAL_Z_MINUS_CABLEL');
                    else
                        [request.reference_matrix,~]= ...
                            authority.native_reference_cache.evaluate( ...
                                authority.native_reference_source,abs_times, ...
                                request.cable_length_m,request.reference_at);
                    end
                    reference_ms=1000*toc(reference_clock);
                    P=TRIGGER_HLC_PLANT_PREDICTOR.predict( ...
                        request,authority.verified_native_binding);
                    P.reference_window_wall_ms=reference_ms;
                    R.predictor_dispatch='DIRECT_NATIVE_VERIFIED_BINDING';
                else
                    P=authority.predictor(request);
                    R.predictor_dispatch='LEGACY_CALLBACK';
                end
                if trace_first, phase_validate_clock=tic; end
                P=FUTURE_COLLISION_TRIGGER.validate_prediction(P,request);
                if trace_first, phase_predict_validate_ms=1000*toc(phase_validate_clock); end
                predictor_wall_ms=1000*toc(predictor_wall_clock);
            catch ME
                R.status='UNKNOWN_PREDICTOR_ERROR';
                R.reason=sprintf('%s: %s',ME.identifier,ME.message);
                R.possible_risk_ids=ids;
                obj.active_request_ids=intersect(obj.active_request_ids,ids,'stable');
                R=obj.finish(R,started);
                return
            end
            R.predictor_wall_ms=predictor_wall_ms;
            if trace_first
                R.first_use_trace=struct( ...
                    'trigger_pre_predictor_ms',first_pre_predictor_ms, ...
                    'predictor_wrapped_wall_ms',predictor_wall_ms, ...
                    'reference_window_ms',NaN, ...
                    'predictor_setup_ms',NaN, ...
                    'mex_native_call_ms',NaN, ...
                    'native_internal_hlc_ms',NaN, ...
                    'native_internal_integrator_ms',NaN, ...
                    'geometry_wall_ms',NaN, ...
                    'prepared_verify_ms',phase_prepared_verify_ms, ...
                    'native_outer_build_ms',phase_outer_build_ms, ...
                    'predictor_validation_ms',phase_predict_validate_ms, ...
                    'incremental_verify_taken',first_use_incremental_verified, ...
                    'incremental_outer_taken',first_use_incremental_outer, ...
                    'known_shape_count',numel(known));
                if isfield(P,'reference_window_wall_ms')
                    R.first_use_trace.reference_window_ms=P.reference_window_wall_ms;
                end
                if isfield(P,'setup_total_ms')
                    R.first_use_trace.predictor_setup_ms=P.setup_total_ms;
                end
                if isfield(P,'native_call_wall_ms')
                    R.first_use_trace.mex_native_call_ms=P.native_call_wall_ms;
                end
                if isfield(P,'controller_total_ms')
                    R.first_use_trace.native_internal_hlc_ms=P.controller_total_ms;
                    R.first_use_trace.native_internal_integrator_ms=P.integrator_total_ms;
                end
            end
            if isfield(P,'reference_window_wall_ms')
                R.predictor_reference_window_ms=P.reference_window_wall_ms;
            end
            R.prediction_model=P.model_description;
            if isfield(P,'controller_total_ms')
                R.predictor_hlc_time_ms=P.controller_total_ms;
                R.predictor_plant_time_ms=P.integrator_total_ms;
                R.predictor_setup_time_ms=P.setup_total_ms;
            end
            R.completed_prediction_samples=numel(P.sample_times_s);
            R.completed_prediction_horizon_s=P.sample_times_s(end);
            R.prediction_complete=true;
            R.prediction_available=true;
            % A validated predictor is a separate development/verification
            % obligation. The checks below are geometric consequences only.
            valid_bounds=FUTURE_COLLISION_TRIGGER.has_valid_bounds(P,numel(grid));
            R.conditional_external_bounds=valid_bounds;
            % Preparation uses ONLY genuinely observed/retained geometry.
            % Reuse it if and ONLY if the ENTIRE source snapshot is exactly
            % equal to the one previously validated. Any changed ID/shape/
            % orientation/motion causes full validation and preparation.
            geometry_wall_clock=tic;
            % Prefer the sensor's ALREADY prepared true-solid geometry,
            % disclosed only after the ID was actually detected. This
            % avoids first-Flight geometry construction and per-tick deep
            % comparison of an entire retained-obstacle struct array.
            % Independently check the snapshot's raw shape and pose.
            if trace_first, phase_geometry_prepare_clock=tic; end
            sensor_prepared=false;
            fast_verified_reuse=false;
            proposed=[];
            if isfield(sensor_frame,'prepared_sensor_shapes')
                proposed=sensor_frame.prepared_sensor_shapes;
                has_token=isfield(sensor_frame,'observation_provenance') && ...
                    strcmp(sensor_frame.observation_provenance, ...
                        'ACTUAL_SENSOR_HISTORY_ONLY') && ...
                    isfield(sensor_frame,'memory_version') && ...
                    isfield(sensor_frame,'geometry_version') && ...
                    isa(sensor_frame.memory_version,'uint64') && ...
                    isa(sensor_frame.geometry_version,'uint64');
                if has_token && ...
                        isequal(sensor_frame.memory_version, ...
                            obj.last_verified_memory_version) && ...
                        isequal(sensor_frame.geometry_version, ...
                            obj.last_verified_geometry_version) && ...
                        isequal(ids,obj.last_verified_prepared_ids) && ...
                        isstruct(proposed) && numel(proposed)==numel(ids) && ...
                        all(isfield(proposed,{'id','is_stationary'})) && ...
                        isequal(reshape(double([proposed.id]),1,[]),ids) && ...
                        all([proposed.is_stationary]) && ...
                        ~isempty(obj.last_verified_prepared_shapes)
                    % The source sensor has immutable static geometry for
                    % this Flight; any new ID or moving geometry invalidates
                    % the version. Always use the already verified object,
                    % not a potentially different incoming structure.
                    proposed=obj.last_verified_prepared_shapes;
                    sensor_prepared=true;
                    fast_verified_reuse=true;
                else
                    sensor_prepared=FUTURE_COLLISION_TRIGGER. ...
                        verified_prepared_observations(known,proposed);
                    if sensor_prepared && has_token && ...
                            all([proposed.is_stationary])
                        obj.last_verified_prepared_ids=ids;
                        obj.last_verified_memory_version= ...
                            sensor_frame.memory_version;
                        obj.last_verified_geometry_version= ...
                            sensor_frame.geometry_version;
                        obj.last_verified_prepared_shapes=proposed;
                    else
                        obj.last_verified_memory_version=[];
                        obj.last_verified_geometry_version=[];
                        obj.last_verified_prepared_shapes=[];
                        obj.last_verified_prepared_ids=zeros(1,0);
                    end
                end
            end
            same_observations=false;
            if sensor_prepared
                shapes=proposed;
                same_observations=true;
            else
                same_observations=~isempty(obj.last_prepared_shapes) && ...
                    isequaln(known,obj.last_prepared_observations);
                if same_observations
                    shapes=obj.last_prepared_shapes;
                else
                    try
                        shapes=OBSTACLE_GEOMETRY_QUERY.prepare(known);
                    catch ME
                        R.geometry_wall_ms=1000*toc(geometry_wall_clock);
                        R.status='UNKNOWN_GEOMETRY_PREPARATION';
                        R.reason=sprintf('%s: %s',ME.identifier,ME.message);
                        R.possible_risk_ids=ids;
                        R=obj.finish(R,started);
                        return
                    end
                    obj.last_prepared_observations=known;
                    obj.last_prepared_shapes=shapes;
                end
            end
            R.geometry_prepare_reused=same_observations;
            R.geometry_prepared_from_observed_sensor=sensor_prepared;
            if trace_first
                R.first_use_trace.geometry_prepare_ms=1000*toc(phase_geometry_prepare_clock);
            end
            R.geometry_fast_verified_reuse=fast_verified_reuse;
            % Shape-independent whole-time-interval theorem. The current
            % native predictor does NOT provide the required rigorously
            % verified excursion bounds, so this path is presently OFF.
            % No guessed tube, no arbitrary speed bound, no sampled-only SAFE.
            interval_cleared=false(1,numel(shapes));
            if valid_bounds
                try
                    I=TRIGGER_CONTINUOUS_INTERVAL_GATE.evaluate( ...
                        P.uav_m,P.payload_m,grid,shapes,obj.r_uav_m, ...
                        obj.r_payload_m,obj.r_cable_m, ...
                        P.interval_uav_excursion_m, ...
                        P.interval_payload_excursion_m);
                    interval_cleared=all(I.separated_intervals,2).';
                    R.interval_gate_status=I.status;
                    R.interval_gate_unresolved_ids=I.possibly_at_risk_ids;
                    R.interval_gate_first_possible_ttc_s=I.earliest_unresolved_left_s;
                catch ME
                    R.geometry_wall_ms=1000*toc(geometry_wall_clock);
                    R.status='UNKNOWN_INTERVAL_ENCLOSURE_QUERY';
                    R.reason=sprintf('%s: %s',ME.identifier,ME.message);
                    R.possible_risk_ids=ids;
                    R=obj.finish(R,started);
                    return
                end
            end
            % Shared broad phase for all observed solids and all sampled
            % positions. Outer balls may eliminate an exact SHAPE query,
            % but never establish contact or inter-sample safety.
            try
                if isfield(P,'native_sampled_outer_only') && ...
                        isequal(P.native_sampled_outer_only,true) && ...
                        isfield(P,'native_sampled_outer_ids') && ...
                        isequal(reshape(double(P.native_sampled_outer_ids),1,[]),ids) && ...
                        isfield(P,'native_sampled_outer_excluded') && ...
                        islogical(P.native_sampled_outer_excluded) && ...
                        numel(P.native_sampled_outer_excluded)==numel(shapes)
                    flag=reshape(P.native_sampled_outer_excluded,1,[]);
                    coarse=struct('sampled_excluded',flag, ...
                        'sampled_excluded_ids',ids(flag));
                    R.geometry_native_broadphase_used=true;
                else
                    coarse=TRIGGER_OBSERVED_BROADPHASE.evaluate( ...
                        P.uav_m,P.payload_m,shapes,obj.r_uav_m, ...
                        obj.r_payload_m,obj.r_cable_m);
                end
            catch
                % A failed optional speed gate must never suppress the
                % original narrow phase or interrupt the Flight loop.
                coarse=struct('sampled_excluded',false(1,numel(shapes)), ...
                    'sampled_excluded_ids',zeros(1,0));
            end
            if trace_first, phase_narrow_clock=tic; end
            R.geometry_sampled_broadphase_ids=coarse.sampled_excluded_ids;
            R.geometry_sampled_broadphase_exclusions= ...
                nnz(coarse.sampled_excluded)*numel(grid)*3;
            pending=zeros(1,0);sample_hits=zeros(1,0);
            material_proximity_ids=zeros(1,0);
            geometry_errors=zeros(1,0);
            geometry_error_messages=cell(1,0);
            earliest=inf;earliest_part='NONE';
            for j=1:numel(known)
                sh=shapes(j);
                % No implicit prediction of moving obstacles.
                if ~sh.is_stationary
                    pending(end+1)=double(sh.id); %#ok<AGROW>
                    continue
                end
                if coarse.sampled_excluded(j)
                    % All three true bodies are disjoint from the outer
                    % enclosing ball at EVERY SAMPLE. There is NO claim
                    % about the unsampled continuous motion. Preserve
                    % fail-closed behavior unless verified interval bounds
                    % have independently cleared the full time span.
                    if ~valid_bounds || ~interval_cleared(j)
                        pending(end+1)=double(sh.id); %#ok<AGROW>
                    end
                    continue
                end
                % One shape-INDEPENDENT sampled geometry entry point for
                % spheres and the ENTIRE finite straight cable. Every
                % obstacle shape is prepared by the same sensor geometry
                % oracle. In particular, cylinders are NOT singled out.
                try
                    [u_hit,l_hit,c_hit,c_unknown,qinfo]= ...
                        TRIGGER_UNIVERSAL_SAMPLED_GEOMETRY.evaluate( ...
                        P.uav_m,P.payload_m,sh,obj.r_uav_m, ...
                        obj.r_payload_m,obj.r_cable_m,time_s,true);
                catch ME
                    % No numeric/shape error can imply clearance.
                    geometry_errors(end+1)=double(sh.id); %#ok<AGROW>
                    geometry_error_messages{end+1}=sprintf('%s: %s', ...
                        ME.identifier,ME.message); %#ok<AGROW>
                    pending(end+1)=double(sh.id); %#ok<AGROW>
                    continue
                end
                R.geometry_exact_body_queries= ...
                    R.geometry_exact_body_queries+qinfo.body_exact_queries;
                R.geometry_exact_cable_queries= ...
                    R.geometry_exact_cable_queries+qinfo.cable_exact_queries;
                R.geometry_support_exclusions= ...
                    R.geometry_support_exclusions+qinfo.body_support_excluded+ ...
                    qinfo.cable_support_excluded;

                sampled=u_hit | l_hit | c_hit;
                index=find(sampled,1,'first');
                hit=~isempty(index);
                if hit
                    sample_hits(end+1)=double(sh.id); %#ok<AGROW>
                    if grid(index)<earliest
                        earliest=grid(index);
                        if u_hit(index)
                            earliest_part='UAV';
                        elseif l_hit(index)
                            earliest_part='PAYLOAD';
                        else
                            earliest_part='CABLE_CENTERLINE';
                        end
                    end
                end
                % An unresolved exact spatial segment query remains UNKNOWN.
                uncertain=~valid_bounds || any(c_unknown) || hit;
                if obj.r_cable_m>0 && any(c_unknown)
                    material_proximity_ids(end+1)=double(sh.id); %#ok<AGROW>
                end
                if valid_bounds && ~hit
                    % Conditional theorem for the COMPLETE temporal
                    % interval, not just the sampled times. If any such
                    % interval cannot be separated, risk remains UNKNOWN.
                    uncertain=uncertain || ~interval_cleared(j);
                end
                if uncertain
                    pending(end+1)=double(sh.id); %#ok<AGROW>
                end
            end
            if trace_first
                R.first_use_trace.geometry_narrow_loop_ms=1000*toc(phase_narrow_clock);
            end
            R.geometry_query_error_ids=geometry_errors;
            R.geometry_query_error_messages=geometry_error_messages;
            R.geometry_wall_ms=1000*toc(geometry_wall_clock);
            if trace_first
                R.first_use_trace.geometry_wall_ms=R.geometry_wall_ms;
            end
            R.sampled_collision_ids=sample_hits;
            R.sampled_cable_material_proximity_ids=material_proximity_ids;
            R.sampled_collision=~isempty(sample_hits);
            R.possible_risk_ids=pending;
            if isfinite(earliest)
                R.first_sampled_collision_ttc_s=earliest;
                R.first_sampled_collision_component=earliest_part;
            end
            if R.sampled_collision
                R.status='PREDICTED_SAMPLED_GEOMETRIC_CONTACT';
                R.reason='Predicted nominal sample intersects a known obstacle; actual closed-loop contact is not proven.';
            elseif ~isempty(geometry_errors)
                R.status='UNKNOWN_GEOMETRY_QUERY_ERROR';
                R.reason='One or more original sensed solids could not be evaluated; never assume clearance.';
            elseif ~isempty(pending)
                R.status='UNKNOWN_CONTINUOUS_INTERVAL_OR_MODEL';
                R.reason='Continuous separation of the closed-loop swept UAV/cable/payload cannot be established.';
            else
                R.status='CONDITIONAL_INTERVAL_GEOMETRY_PASS__MODEL_NOT_AUDITED';
                R.reason='Geometric separation is conditional on supplied interval bounds; no audited closed-loop safety proof.';
                R.conditional_interval_geometry_pass=true;
                % Never claim verified real closed-loop continuous safety here.
                R.continuous_observed_set_safe=false;
            end
            % An uncertified interval is treated as possible risk (fail-closed).
            % UNKNOWN from absent interval bounds is NOT itself a predicted
            % collision. Only explicitly observed nominal sample contacts may
            % raise a provisional numerical request. The final certified
            % trigger will use validated reachable-set contact instead.
            if isempty(sample_hits)
                % No nominal contact this tick. The old expression also
                % intersects active requests with the currently known IDs;
                % since requests originated from retained known IDs and
                % known IDs are append-only, this branch is equivalent.
                new_ids=zeros(1,0);
                R.duplicate_suppressed=false;
                % Preserve any active IDs for duplicate suppression if a
                % later sample still contacts them. No unproved release.
            else
                % Equivalent stable-set event transition, with no dynamic
                % setdiff/union/intersect dispatch at first contact.  This
                % helper has its own preflight isolated regression and NEVER
                % changes the physical collision/safety decision.
                [new_ids,R.duplicate_suppressed,obj.active_request_ids]= ...
                    TRIGGER_STABLE_EVENT_IDS( ...
                        sample_hits,obj.active_request_ids,ids);
            end
            R.new_event_ids=new_ids;
            R.new_event=~isempty(new_ids);
            R.numeric_request=R.new_event;
            % Never authorize trajectory commit from a trigger decision alone.
            R.safe_to_commit=false;
            R=obj.finish(R,started);
        end
    end

    methods (Access=private)
        function t=make_grid(obj)
            % Never change dt/horizon or round the flight sample clock.
            % Preflight has prepared the same formula if static parameters
            % were available; rebuild only when an external value changes.
            if ~isequal(obj.static_grid_dt,obj.sample_dt_s) || ...
                    ~isequal(obj.static_grid_horizon,obj.horizon_s) || ...
                    isempty(obj.static_grid_values)
                obj.prepare_static_timing(obj.sample_dt_s,obj.horizon_s);
            end
            t=obj.static_grid_values;
        end
        function R=finish(obj,R,started)
            R.elapsed_ms=1000*toc(started);
            obj.last_result=R;
            obj.evaluation_count=obj.evaluation_count+1;
            obj.total_elapsed_ms=obj.total_elapsed_ms+R.elapsed_ms;
            obj.maximum_elapsed_ms=max(obj.maximum_elapsed_ms,R.elapsed_ms);
        end
    end

    methods (Static,Access=private)
        function R=make_result_template()
            % Identical field names/types/ordering to the old per-tick literal.
            R=struct('time_s',NaN,'horizon_s',NaN, ...
                'sample_dt_s',NaN,'sample_count',0, ...
                'considered_obstacle_ids',zeros(1,0),'considered_count',0, ...
                'current_payload_mass_kg',NaN, ...
                'current_cable_length_m',NaN, ...
                'current_uav_radius_m',NaN, ...
                'current_payload_radius_m',NaN, ...
                'current_cable_radius_m',NaN, ...
                'runtime_authority_refreshed',true, ...
                'flight_binding_used',false, ...
                'model_and_estimator_bounds_audited',false, ...
                'prediction_complete',false, ...
                'completed_prediction_horizon_s',0, ...
                'completed_prediction_samples',0, ...
                'predictor_hlc_time_ms',NaN, ...
                'predictor_plant_time_ms',NaN, ...
                'predictor_reference_window_ms',NaN, ...
                'predictor_dispatch','NOT_INVOKED', ...
                'predictor_setup_time_ms',NaN, ...
                'status','UNKNOWN','prediction_model','', ...
                'prediction_available',false,'sampled_collision',false, ...
                'continuous_observed_set_safe',false, ...
                'conditional_interval_geometry_pass',false, ...
                'conditional_external_bounds',false, ...
                'interval_gate_status','NOT_AVAILABLE_WITHOUT_VERIFIED_BOUNDS', ...
                'interval_gate_unresolved_ids',zeros(1,0), ...
                'interval_gate_first_possible_ttc_s',NaN, ...
                'possible_risk_ids',zeros(1,0), ...
                'sampled_collision_ids',zeros(1,0), ...
                'sampled_cable_material_proximity_ids',zeros(1,0), ...
                'first_sampled_collision_ttc_s',NaN, ...
                'first_sampled_collision_component','NONE', ...
                'geometry_wall_ms',NaN, ...
                'geometry_exact_body_queries',0, ...
                'geometry_exact_cable_queries',0, ...
                'geometry_support_exclusions',0, ...
                'geometry_native_broadphase_used',false, ...
                'geometry_first_use_verified_before_predictor',false, ...
                'geometry_sampled_broadphase_exclusions',0, ...
                'geometry_sampled_broadphase_ids',zeros(1,0), ...
                'geometry_query_error_ids',zeros(1,0), ...
                'geometry_query_error_messages',{{}}, ...
                'new_event',false,'new_event_ids',zeros(1,0), ...
                'duplicate_suppressed',false, ...
                'numeric_request',false,'safe_to_commit',false, ...
                'reason','', 'elapsed_ms',NaN);
        end
        function v=positive(x,label)
            assert(isnumeric(x)&&isscalar(x)&&isreal(x)&&isfinite(x)&&x>0, ...
                'FUTURE_COLLISION_TRIGGER:ExternalValue', ...
                'External value %s must be positive and finite.',label);
            v=double(x);
        end
        function v=nonnegative(x,label)
            assert(isnumeric(x)&&isscalar(x)&&isreal(x)&&isfinite(x)&&x>=0, ...
                'FUTURE_COLLISION_TRIGGER:ExternalValue', ...
                'External value %s must be nonnegative and finite.',label);
            v=double(x);
        end
        function e=check_ekf(e)
            assert(isstruct(e)&&isscalar(e),'FUTURE_COLLISION_TRIGGER:EKF', ...
                'Current estimator state must be a scalar struct.');
            fields={'p','q','v','w','pL','vL','pT','wL'};
            for j=1:numel(fields)
                key=fields{j};
                assert(isfield(e,key) && isnumeric(e.(key)) && ...
                    isequal(size(e.(key)),[3 1]) && isreal(e.(key)) && ...
                    all(isfinite(e.(key))), ...
                    'FUTURE_COLLISION_TRIGGER:EKF','Missing current 3x1 EKF state %s.',key);
                e.(key)=double(e.(key));
            end
        end
        function obstacles=check_observation(frame,t,p)
            required={'time_s','uav_position_ekf_m','detected_obstacles', ...
                'detected_ids','detected_count'};
            assert(isstruct(frame)&&isscalar(frame) && ...
                all(isfield(frame,required)), ...
                'FUTURE_COLLISION_TRIGGER:Sensor','A real current sensor frame is required.');
            assert(isequal(double(frame.time_s),double(t)) && ...
                isequal(double(frame.uav_position_ekf_m(:)),p), ...
                'FUTURE_COLLISION_TRIGGER:NotCurrentSensor', ...
                'Only a sensor frame from this EKF state and same tick is accepted.');
            obstacles=frame.detected_obstacles;
            assert(isstruct(obstacles), 'FUTURE_COLLISION_TRIGGER:Sensor', ...
                'Detected obstacles must retain original struct type.');
            ids=reshape([obstacles.id],1,[]);
            assert(isequal(ids,reshape(frame.detected_ids,1,[])) && ...
                isequal(double(frame.detected_count),double(numel(ids))) && ...
                numel(unique(ids))==numel(ids), ...
                'FUTURE_COLLISION_TRIGGER:Sensor', ...
                'Sensor detected IDs/count and observed payload disagree.');
        end
        function yes=verified_prepared_observations(observed,prepared)
            % An optimization must NEVER accept stale/different geometry.
            % Compare the underlying static description, not the derived
            % time-varying .position field. This also works for any future
            % sensor shape without a trigger-side shape switch statement.
            yes=isstruct(prepared) && numel(prepared)==numel(observed);
            if ~yes,return;end
            needed={'id','type','dimensions','center0','rotation', ...
                    'velocity','reference_time_s','is_stationary', ...
                    'bounding_radius_m'};
            if ~all(isfield(prepared,needed)),yes=false;return;end
            for j=1:numel(observed)
                o=observed(j);p=prepared(j);
                yes=isequal(double(o.id),double(p.id)) && ...
                    isequal(o.geometry.type,p.type) && ...
                    isequaln(o.geometry.dimensions,p.dimensions) && ...
                    isequaln(double(o.initial_position(:)),double(p.center0(:))) && ...
                    isequaln(double(o.rotation),double(p.rotation)) && ...
                    isequaln(double(o.velocity(:)),double(p.velocity(:))) && ...
                    isequaln(double(o.reference_time_s),double(p.reference_time_s)) && ...
                    isequal(logical(all(o.velocity==0)),logical(p.is_stationary)) && ...
                    isscalar(p.bounding_radius_m) && ...
                    isnumeric(p.bounding_radius_m) && ...
                    isfinite(p.bounding_radius_m) && p.bounding_radius_m>=0;
                if ~yes,return;end
            end
        end
        function p=validate_prediction(p,request)
            assert(isstruct(p)&&isscalar(p) && ...
                all(isfield(p,{'sample_times_s','uav_m','payload_m','model_description'})), ...
                'FUTURE_COLLISION_TRIGGER:Prediction','Missing model prediction fields.');
            n=numel(request.sample_times_s);
            assert(isnumeric(p.sample_times_s) && ...
                isequal(reshape(double(p.sample_times_s),1,[]),request.sample_times_s) && ...
                isnumeric(p.uav_m)&&isequal(size(p.uav_m),[3 n])&& ...
                all(isfinite(p.uav_m(:)))&&isreal(p.uav_m) && ...
                isnumeric(p.payload_m)&&isequal(size(p.payload_m),[3 n])&& ...
                all(isfinite(p.payload_m(:)))&&isreal(p.payload_m), ...
                'FUTURE_COLLISION_TRIGGER:Prediction','Prediction grid or state dimensions invalid.');
            assert((ischar(p.model_description)||isstring(p.model_description)) && ...
                strlength(string(p.model_description))>0, ...
                'FUTURE_COLLISION_TRIGGER:Prediction','Nonempty model provenance is required.');
            % Must begin at the CURRENT estimator state, no nominal reset.
            assert(isequal(double(p.uav_m(:,1)),request.ekf_now.p) && ...
                isequal(double(p.payload_m(:,1)),request.ekf_now.pL), ...
                'FUTURE_COLLISION_TRIGGER:Prediction', ...
                'The model prediction must start at the current EKF state.');
            p.uav_m=double(p.uav_m);p.payload_m=double(p.payload_m);
            p.model_description=char(p.model_description);
        end
        function yes=has_valid_bounds(p,n)
            names={'interval_uav_excursion_m','interval_payload_excursion_m', ...
                'interval_bounds_verified','tracking_error_included', ...
                'estimator_error_included','taut_cable_verified'};
            yes=all(isfield(p,names));
            if ~yes,return;end
            a=p.interval_uav_excursion_m;b=p.interval_payload_excursion_m;
            yes=isnumeric(a)&&isnumeric(b)&&isreal(a)&&isreal(b) && ...
                isrow(a)&&isrow(b)&&numel(a)==n-1&&numel(b)==n-1&& ...
                all(isfinite(a))&&all(isfinite(b))&&all(a>=0)&&all(b>=0)&& ...
                isequal(p.interval_bounds_verified,true) && ...
                isequal(p.tracking_error_included,true) && ...
                isequal(p.estimator_error_included,true) && ...
                isequal(p.taut_cable_verified,true);
        end
    end
end
