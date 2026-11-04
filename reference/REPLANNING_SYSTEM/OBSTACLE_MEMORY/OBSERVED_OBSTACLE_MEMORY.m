classdef OBSERVED_OBSTACLE_MEMORY < handle
    % Retain ONLY real sensor-observed obstacles until a proved-safe release.
    % No world-definition lookup, no predicted/future observations, no TTL.
    % This version deliberately does NOT implement release: the all-future
    % closed-loop / backup no-reentry certificate is not connected yet.
    properties (SetAccess=private)
        observed_obstacles
        observed_schema_ready = false
        known_ids = zeros(1,0)
        currently_detected_ids = zeros(1,0)
        first_seen_s = zeros(1,0)
        last_seen_s = zeros(1,0)
        last_tick_s = NaN
        last_uav_position_m = NaN(3,1)
        version = uint64(0)
        observation_count = 0
        last_result = struct()
        time_total_ms = 0
        time_max_ms = 0
        % Cached ID mapping only; no world truth or detection-mask combinations.
        detected_known_indices = zeros(1,0)
        retained_ids_cache = zeros(1,0)
        % Changes when observed moving geometry is refreshed. New/static IDs
        % are covered by memory_version. Used to invalidate geometry reuse.
        geometry_version = uint64(0)
    end
    methods
        function obj = OBSERVED_OBSTACLE_MEMORY(obstacle_template)
            % Template contains NO obstacles and is used only for typing.
            assert(isstruct(obstacle_template) && isempty(obstacle_template), ...
                'OBSERVED_OBSTACLE_MEMORY:Template', ...
                'Pass only an empty obstacle struct array as template.');
            obj.observed_obstacles = reshape(obstacle_template,1,[]);
            % No observed object is present. A schema with the sensor's
            % .position field can be supplied safely before Flight, without
            % reading any sensor event or real obstacle information.
            obj.observed_schema_ready=isfield(obstacle_template,'position');
        end
        function result = update(obj,frame)
            clock = tic;
            diagnose=strcmp(getenv('REPLANNING_STAGE_TIMING'),'1');
            must = {'time_s','uav_position_ekf_m','detected_ids', ...
                    'detected_count','detected_obstacles'};
            assert(isstruct(frame)&&isscalar(frame)&&all(isfield(frame,must)), ...
                'OBSERVED_OBSTACLE_MEMORY:Frame','A genuine current sensor frame is required.');
            t=frame.time_s;
            assert(isnumeric(t)&&isscalar(t)&&isfinite(t)&&isreal(t) && ...
                (isnan(obj.last_tick_s)||t>=obj.last_tick_s), ...
                'OBSERVED_OBSTACLE_MEMORY:Time', ...
                'Sensor time must never move backward.');
            p=frame.uav_position_ekf_m;
            assert(isnumeric(p)&&isequal(size(p),[3 1]) && ...
                isreal(p)&&all(isfinite(p)), ...
                'OBSERVED_OBSTACLE_MEMORY:EKF','Same-tick 3x1 EKF position required.');
            obstacles=frame.detected_obstacles;
            assert(isstruct(obstacles), 'OBSERVED_OBSTACLE_MEMORY:Shape', ...
                'Sensor obstacles must be a struct array.');
            ids=reshape(double(frame.detected_ids),1,[]);
            assert(isnumeric(frame.detected_ids)&&isreal(frame.detected_ids) && ...
                all(isfinite(ids)) && all(ids==fix(ids)) && ...
                all(ids>0) && numel(unique(ids))==numel(ids) && ...
                numel(obstacles)==numel(ids) && ...
                isequal(reshape(double([obstacles.id]),1,[]),ids) && ...
                isequal(double(frame.detected_count),double(numel(ids))), ...
                'OBSERVED_OBSTACLE_MEMORY:SensorIDs', ...
                'Original sensor IDs and geometry payload disagree.');
            if diagnose,diagnostic_validate_ms=1000*toc(clock);end
            if isequal(double(t),obj.last_tick_s) && ...
                    isequal(ids,obj.currently_detected_ids) && ...
                    isequal(double(p),obj.last_uav_position_m)
                % Exact repeated observation: no duplicate memory event.
                result=obj.last_result;
                result.newly_acquired_ids=zeros(1,0);
                result.elapsed_ms=1000*toc(clock);
                if diagnose
                    result.diagnostic_stage_ms=[result.elapsed_ms,0,0,0];
                    result.diagnostic_same_ids=true;
                    result.diagnostic_new_id_count=0;
                end
                return
            end
            % Arming and Flight may share the same t=0 while the EKF
            % observation changes. Accept a NEW real observation at the
            % same time; do not forge a timestamp, or refuse the Flight.
            % Sensor frames can contain a derived .position field not present
            % in OBSTACLE_INFORMATION definitions. Adopt ONLY the observed
            % frame schema (not the source's obstacle list).
            if ~obj.observed_schema_ready
                % Legacy callers may provide a definition-only empty schema.
                % Perform this adaptation ONCE on the first real scan, never
                % repeat it on every empty detection during takeoff.
                obj.observed_obstacles=reshape(obstacles([]),1,[]);
                obj.observed_schema_ready=true;
            end
            newly=zeros(1,0);
            % Most frames have EXACTLY the same detected IDs as the previous
            % real scan. Keep their established positions in the known set;
            % do not repeat find()/setdiff() for unchanged sets. This is an
            % ID-cache, NOT a future sensor prediction or mask precomputation.
            same_ids=isequal(ids,obj.currently_detected_ids);
            if same_ids && numel(obj.detected_known_indices)==numel(ids)
                indices=obj.detected_known_indices;
            else
                % New real detections are appended ONCE per frame rather
                % than growing multiple struct/numeric arrays per ID.
                % The sensor has already proved the IDs are unique.
                indices=zeros(1,numel(ids));
                fresh_positions=zeros(1,numel(ids));
                n_fresh=0;
                n_old=numel(obj.known_ids);
                for i=1:numel(ids)
                    ix=find(obj.known_ids==ids(i),1);
                    if isempty(ix)
                        n_fresh=n_fresh+1;
                        fresh_positions(n_fresh)=i;
                        ix=n_old+n_fresh;
                    end
                    indices(i)=ix;
                end
                if n_fresh>0
                    fresh_positions=fresh_positions(1:n_fresh);
                    newly=ids(fresh_positions);
                    % A genuine sensor may return Kx1 while the cache is 1xK.
                    % First assemble the new row array; never mutate ID
                    % history if field compatibility or concatenation fails.
                    next_obstacles=OBSERVED_OBSTACLE_MEMORY.append_row_structs( ...
                        obj.observed_obstacles,obstacles(fresh_positions));
                    obj.observed_obstacles=next_obstacles;
                    obj.known_ids=[obj.known_ids newly];
                    obj.first_seen_s=[obj.first_seen_s repmat(double(t),1,n_fresh)];
                    obj.last_seen_s=[obj.last_seen_s repmat(double(t),1,n_fresh)];
                    % Preserve exactly one generation increment per new ID.
                    obj.version=obj.version+uint64(n_fresh);
                end
                obj.detected_known_indices=indices;
                % The known IDs are always unique and incoming IDs a subset
                % after append. The mask therefore has the SAME order and
                % values as setdiff(known_ids,ids,'stable'), including [].
                held_mask=true(1,numel(obj.known_ids));
                held_mask(indices)=false;
                obj.retained_ids_cache=obj.known_ids(held_mask);
            end
            if diagnose,diagnostic_topology_ms=1000*toc(clock);end
            % Refresh timestamps and moving descriptions only. Exact static
            % descriptions come from the sensor's immutable Flight definition
            % and MUST NOT be allocated or copied at every control tick.
            for i=1:numel(indices)
                ix=indices(i);
                stationary=isfield(obstacles,'motion') && ...
                    strcmp(obstacles(i).motion,'static');
                if ~stationary && ~isequaln(obj.observed_obstacles(ix),obstacles(i))
                    obj.observed_obstacles(ix)=obstacles(i);
                    obj.geometry_version=obj.geometry_version+uint64(1);
                end
                obj.last_seen_s(ix)=double(t);
            end
            if diagnose,diagnostic_refresh_ms=1000*toc(clock);end
            obj.currently_detected_ids=ids;
            obj.last_tick_s=double(t);
            obj.last_uav_position_m=double(p);
            obj.observation_count=obj.observation_count+1;
            held=obj.retained_ids_cache;
            elapsed_ms=1000*toc(clock);
            obj.time_total_ms=obj.time_total_ms+elapsed_ms;
            obj.time_max_ms=max(obj.time_max_ms,elapsed_ms);
            result=struct('time_s',double(t),'currently_detected_ids',ids, ...
                'known_obstacle_ids',obj.known_ids, ...
                'held_outside_current_detection_ids',held, ...
                'newly_acquired_ids',newly, ...
                'known_count',numel(obj.known_ids), ...
                'memory_version',obj.version, ...
                'geometry_version',obj.geometry_version, ...
                'automatic_release_enabled',false, ...
                'safety_release_prover_connected',false, ...
                'elapsed_ms',elapsed_ms);
            if diagnose
                % Four non-overlapping intervals; elapsed_ms is captured
                % BEFORE result struct assembly in the original source.
                result.diagnostic_stage_ms=[diagnostic_validate_ms, ...
                    diagnostic_topology_ms-diagnostic_validate_ms, ...
                    diagnostic_refresh_ms-diagnostic_topology_ms, ...
                    elapsed_ms-diagnostic_refresh_ms];
                result.diagnostic_same_ids=same_ids;
                result.diagnostic_new_id_count=numel(newly);
            end
            obj.last_result=result;
        end
        function frame = known_frame(obj,current_sensor_frame)
            % Adapter for FUTURE_COLLISION_TRIGGER's existing frame schema.
            % Every member was sourced from a genuine prior sensor frame.
            assert(isscalar(current_sensor_frame)&&isstruct(current_sensor_frame) && ...
                isfield(current_sensor_frame,'time_s') && ...
                isequal(double(current_sensor_frame.time_s),obj.last_tick_s), ...
                'OBSERVED_OBSTACLE_MEMORY:Stale', ...
                'Call update() with the current sensor before known_frame().');
            frame=current_sensor_frame;
            frame.detected_obstacles=obj.observed_obstacles;
            frame.detected_ids=obj.known_ids;
            frame.detected_count=numel(obj.known_ids);
            % Current measured distances must not be falsely assigned to
            % retained obstacles which are now outside detection.
            if isfield(current_sensor_frame,'detected_distances_m')
                measured=reshape(double(current_sensor_frame.detected_distances_m),1,[]);
                assert(numel(measured)==numel(obj.currently_detected_ids), ...
                    'OBSERVED_OBSTACLE_MEMORY:Distances', ...
                    'Current sensor distance vector and IDs disagree.');
                known_distance=nan(1,numel(obj.known_ids));
                for k=1:numel(obj.detected_known_indices)
                    known_distance(obj.detected_known_indices(k))=measured(k);
                end
                frame.detected_distances_m=known_distance;
            end
            frame.currently_detected_ids=obj.currently_detected_ids;
            frame.retained_only_ids=obj.retained_ids_cache;
            frame.known_obstacle_ids=obj.known_ids;
            frame.observation_provenance='ACTUAL_SENSOR_HISTORY_ONLY';
            frame.memory_version=obj.version;
            frame.geometry_version=obj.geometry_version;
        end
        function frame = known_frame_for_trigger(obj,current_sensor_frame)
            % Strict read-only adapter for the genuine current observation.
            % The Trigger needs the entire SENSOR-OBSERVED/RETAINED set, not
            % distances or other transient fields of only the current scan.
            % This adapter intentionally does not build a known-distance
            % vector, requery world definitions, or release retained IDs.
            mandatory={'time_s','uav_position_ekf_m','detected_ids', ...
                'detected_count','detected_obstacles'};
            assert(isstruct(current_sensor_frame) && ...
                isscalar(current_sensor_frame) && ...
                all(isfield(current_sensor_frame,mandatory)), ...
                'OBSERVED_OBSTACLE_MEMORY:TriggerFrame', ...
                'A genuine sensor frame with the current EKF is required.');
            assert(isequal(double(current_sensor_frame.time_s), ...
                       obj.last_tick_s) && ...
                   isequal(double(current_sensor_frame.uav_position_ekf_m), ...
                       obj.last_uav_position_m) && ...
                   isequal(reshape(double(current_sensor_frame.detected_ids),1,[]), ...
                       obj.currently_detected_ids), ...
                'OBSERVED_OBSTACLE_MEMORY:TriggerStale', ...
                'The current sensor observation must first update the memory.');
            % Keep this minimal: the trigger must never mistake a measured
            % distance for a retained, currently out-of-range obstacle.
            frame=struct( ...
                'time_s',double(current_sensor_frame.time_s), ...
                'uav_position_ekf_m',double(current_sensor_frame.uav_position_ekf_m), ...
                'detected_obstacles',obj.observed_obstacles, ...
                'detected_ids',obj.known_ids, ...
                'detected_count',numel(obj.known_ids), ...
                'currently_detected_ids',obj.currently_detected_ids, ...
                'retained_only_ids',obj.retained_ids_cache, ...
                'known_obstacle_ids',obj.known_ids, ...
                'observation_provenance','ACTUAL_SENSOR_HISTORY_ONLY', ...
                'memory_version',obj.version, ...
                'geometry_version',obj.geometry_version);
        end
        function status = get_status(obj)
            status=struct('known_ids',obj.known_ids, ...
                'currently_detected_ids',obj.currently_detected_ids, ...
                'retained_only_ids',obj.retained_ids_cache, ...
                'first_seen_s',obj.first_seen_s,'last_seen_s',obj.last_seen_s, ...
                'memory_version',obj.version, ...
                'geometry_version',obj.geometry_version, ...
                'last_tick_s',obj.last_tick_s, ...
                'observation_count',obj.observation_count, ...
                'automatic_release_enabled',false, ...
                'no_future_reentry_certificate_connected',false, ...
                'time_total_ms',obj.time_total_ms,'time_max_ms',obj.time_max_ms);
        end
        function release(~,varargin) %#ok<INUSD>
            error('OBSERVED_OBSTACLE_MEMORY:ReleaseNotCertified', ...
                ['A proven safe release condition has not been connected. ', ...
                 'Range exit or 4-second clearance must never erase an observed obstacle.']);
        end
    end
    methods (Static)
        function out=append_row_structs(existing,additions)
            % Shape-only normalization; preserve each field and object order.
            % Called only on actual new detections, never during a scan.
            assert(isstruct(additions), ...
                'OBSERVED_OBSTACLE_MEMORY:AppendType', ...
                'Sensor-sourced additions must be a struct array.');
            if isempty(existing)
                % Also accept the replanner's initial numeric [] sentinel.
                assert(isstruct(existing)||isequal(existing,[]), ...
                    'OBSERVED_OBSTACLE_MEMORY:AppendType', ...
                    'Existing records must be a struct array or [].');
                out=reshape(additions,1,[]);
            else
                assert(isstruct(existing), ...
                    'OBSERVED_OBSTACLE_MEMORY:AppendType', ...
                    'Existing records must be a struct array.');
                out=[reshape(existing,1,[]) reshape(additions,1,[])];
            end
        end
    end
end
