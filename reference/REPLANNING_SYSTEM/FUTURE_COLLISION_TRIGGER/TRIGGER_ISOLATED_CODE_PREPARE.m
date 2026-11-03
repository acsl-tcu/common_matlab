function R=TRIGGER_ISOLATED_CODE_PREPARE(cfg)
% Software-only isolated first-use preparation, before Arming.
% Never takes an agent, sensor, EKF, world/obstacle definition list,
% existing trigger, existing reference, or Flight logging object.
% NEVER publishes the constructed fixture, result, or request. This
% exercises the MATLAB trigger's real nonempty-known-geometry code path
% on TEMPORARY objects for JIT initialization. No actual future motion.
% It is strictly an optional speed optimization, never a safety audit.
started=tic;
R=struct('status','UNKNOWN_ISOLATED_CODE_PREPARE','passed',false,...
    'elapsed_ms',NaN,'touched_live_flight',false,'safety_proven',false,'isolated_event_path_exercised',false,...
    'reason','');
try
    assert(isstruct(cfg)&&isscalar(cfg),...
        'TRIGGER_ISOLATED_CODE_PREPARE:Cfg','A scalar static configuration is required.');
    req={'sample_dt_s','horizon_s','r_uav_m','r_payload_m','r_cable_m','cable_length_m'};
    assert(all(isfield(cfg,req)),'TRIGGER_ISOLATED_CODE_PREPARE:Cfg',...
        'Only validated static dt/radii/horizon/cable length accepted.');
    assert(all(isfinite([cfg.sample_dt_s,cfg.horizon_s,cfg.r_uav_m,...
        cfg.r_payload_m,cfg.r_cable_m,cfg.cable_length_m]))&&...
        all([cfg.sample_dt_s,cfg.horizon_s,cfg.r_uav_m,...
        cfg.r_payload_m,cfg.cable_length_m]>0)&&cfg.r_cable_m>=0,...
        'TRIGGER_ISOLATED_CODE_PREPARE:Value','Invalid static configuration.');
    % Throwaway source-shape fixture, NOT any actual environment obstacle.
    % It does NOT call OBSTACLE_INFORMATION.definitions() or sensor.observe().
    shape=OBSTACLE_INFORMATION.define(1,'sphere',[1000;1000;1000],1);
    e=struct('p',zeros(3,1),'q',zeros(3,1),'v',zeros(3,1),...
        'w',zeros(3,1),'pL',[0;0;-cfg.cable_length_m],...
        'vL',zeros(3,1),'pT',[0;0;1],'wL',zeros(3,1));
    frame=struct('time_s',0,'uav_position_ekf_m',e.p,...
        'detected_obstacles',shape,'detected_ids',1,'detected_count',1);
    temp=FUTURE_COLLISION_TRIGGER(struct('horizon_s',cfg.horizon_s));
    a=struct('runtime',cfg,'payload_mass_kg',1,...
        'reference_at',@(t)zeros(28,1),...
        'predictor',@local_numeric_fixture,...
        'numeric_tolerances',[1;1]);
    out=temp.evaluate(0,e,frame,a);
    assert(out.prediction_complete && out.prediction_available &&...
        isequal(out.considered_obstacle_ids,1) &&...
        ~out.safe_to_commit,...
        'TRIGGER_ISOLATED_CODE_PREPARE:Failed',...
        'The isolated mathematical fixture did not traverse the predictor branch.');
    % Pure ID-event branch transitions: all local variables, no live memory.
    % A fixed repeat count is a software exercise, NEVER a JIT guarantee.
    for repetition=1:8
        active=zeros(1,0);
        [~,~,active]=TRIGGER_STABLE_EVENT_IDS([],active,[]);
        [~,~,active]=TRIGGER_STABLE_EVENT_IDS(1,active,1);
        [~,~,active]=TRIGGER_STABLE_EVENT_IDS([1 2],active,[1 2]);
        [~,~,active]=TRIGGER_STABLE_EVENT_IDS(2,active,[1 2]);
        [~,~,active]=TRIGGER_STABLE_EVENT_IDS([],active,2);
    end
    % Exercise the EXACT new lean memory adapter with synthetic, throwaway
    % observation records. This warms software branch dispatch, not the
    % actual sensor/obstacle world. This does not prime flight geometry.
    for software_replay=1:8
    synthetic_memory=OBSERVED_OBSTACLE_MEMORY(shape([]));
    synthetic_frame=frame;
    synthetic_memory.update(synthetic_frame);
    thin=synthetic_memory.known_frame_for_trigger(synthetic_frame);
    assert(isequal(thin.detected_ids,1) && ...
        isequal(thin.observation_provenance,'ACTUAL_SENSOR_HISTORY_ONLY'), ...
        'TRIGGER_ISOLATED_CODE_PREPARE:LeanMemory', ...
        'Isolated first-observation adapter failed.');
    synthetic_obstacle_2=OBSTACLE_INFORMATION.define(2,'sphere', ...
        [1002;1000;1000],1);
    synthetic_frame.time_s=cfg.sample_dt_s;
    synthetic_frame.detected_obstacles=[shape synthetic_obstacle_2];
    synthetic_frame.detected_ids=[1 2];
    synthetic_frame.detected_count=2;
    synthetic_memory.update(synthetic_frame);
    thin=synthetic_memory.known_frame_for_trigger(synthetic_frame);
    assert(isequal(thin.detected_ids,[1 2]), ...
        'TRIGGER_ISOLATED_CODE_PREPARE:LeanMemory', ...
        'Isolated append-only observation adapter failed.');
    synthetic_frame.time_s=2*cfg.sample_dt_s;
    synthetic_frame.detected_obstacles=synthetic_obstacle_2;
    synthetic_frame.detected_ids=2;
    synthetic_frame.detected_count=1;
    synthetic_memory.update(synthetic_frame);
    thin=synthetic_memory.known_frame_for_trigger(synthetic_frame);
    assert(isequal(thin.detected_ids,[1 2]), ...
        'TRIGGER_ISOLATED_CODE_PREPARE:LeanMemory', ...
        'Isolated retained-obstacle adapter failed.');
    end % independent throwaway observation-memory histories
    % Exercise the exact SENSOR + MEMORY + prepared-geometry APIs with
    % independently created fictional solids, including detection loss.
    % Preflight-only, no real obstacle definitions, no actual sensor data,
    % no EKF/Flight timing, no fixed detection-mask combinations.
    external_events=TRIGGER_ISOLATED_SENSOR_EVENT_PREPARE(cfg);
    R.isolated_sensor_event_prepare_status=external_events.status;
    R.isolated_sensor_event_prepare_cycles=external_events.cycles;
    R.isolated_sensor_event_prepare_steps=external_events.steps;
    R.isolated_sensor_event_prepare_reason=external_events.reason;
    R.isolated_lean_memory_path_exercised=true;
    R.isolated_event_path_exercised=true;
    % ACTUAL compiled predictor software path on wholly synthetic inputs.
    % Numeric values come from a throwaway stationary test state and a
    % fictitious zero reference; they must never be stored/reused for Flight.
    % The preflight binding supplies ONLY real static source definitions.
    % This cannot guarantee JIT compilation or remove future Flight spikes.
    R.isolated_native_prediction_status='NOT_ATTEMPTED';
    R.isolated_native_prediction_calls=0;
    if isfield(cfg,'native_binding') && isfield(cfg,'numeric_tolerances')
        try
            b=cfg.native_binding;
            % Throwaway positive mass: source snapshots are not mutated.
            b.model_parameters(20)=1;
            b.hlc_parameter_values(b.hlc_loadmass_index)=1;
            gate=TRIGGER_NATIVE_SOURCE_PIN_CACHE('get');
            assert(isstruct(b) && ...
                isfield(b,'native_source_order_checked') && ...
                isequal(b.native_source_order_checked,true) && ...
                isstruct(gate) && isfield(gate,'passed') && ...
                isequal(gate.passed,true), ...
                'TRIGGER_ISOLATED_CODE_PREPARE:NativePin', ...
                'Pinned original native model is required for software exercise.');
            g=0:cfg.sample_dt_s:cfg.horizon_s;
            g=[g(g<cfg.horizon_s) cfg.horizon_s];
            assert(all(diff(g)>0));
            % Zero position and velocity, vertical unit cable, zero yaw:
            % these are LOCAL numeric fixtures, not possible observed data.
            req=struct('time_s',0,'sample_times_s',g, ...
                'ekf_now',e,'payload_mass_kg',1, ...
                'numeric_tolerances',cfg.numeric_tolerances, ...
                'reference_at',@(tt)zeros(28,1), ...
                'reference_matrix',zeros(28,numel(g)-1));
            for native_trial=1:8
                P=TRIGGER_HLC_PLANT_PREDICTOR.predict(req,b);
                assert(isequal(P.prediction_complete,true) && ...
                    numel(P.sample_times_s)==numel(g) && ...
                    all(isfinite(P.uav_m(:))) && ...
                    all(isfinite(P.payload_m(:))));
                R.isolated_native_prediction_calls=native_trial;
            end
            R.isolated_native_prediction_status= ...
                'SYNTHETIC_PINNED_NATIVE_PREDICTOR_EXERCISED__NO_JIT_GUARANTEE';
        catch native_error
            R.isolated_native_prediction_status= ...
                ['FULL_HORIZON_FIXTURE_NOT_COMPLETED__' native_error.identifier];
            % A synthetic 4-s trajectory can become singular even though
            % the real physical model is valid. Still exercise the SAME
            % software / MEX entrypoint over a one-sample window, without
            % inventing any Flight or future reference data.
            try
                short=struct('time_s',0,'sample_times_s', ...
                    [0 cfg.sample_dt_s], 'ekf_now',e, ...
                    'payload_mass_kg',1, ...
                    'numeric_tolerances',cfg.numeric_tolerances, ...
                    'reference_at',@(tt)zeros(28,1), ...
                    'reference_matrix',zeros(28,1));
                for native_trial=1:8
                    P=TRIGGER_HLC_PLANT_PREDICTOR.predict(short,b);
                    assert(isequal(P.prediction_complete,true));
                    R.isolated_native_prediction_calls=native_trial;
                end
                R.isolated_native_prediction_status= ...
                    'ONE_STEP_SYNTHETIC_NATIVE_EXERCISED__NOT_FULL_4S_WARM';
            catch short_error
                R.isolated_native_prediction_status= ...
                    ['NATIVE_SYNTHETIC_FIXTURE_FAILED__' short_error.identifier];
            end
        end
    end
    % Full pinned direct-native trigger preparation with local throwaway
    % TIME_VARYING_REFERENCE, observed-memory history and geometry.
    % Never imports the real Flight nominal source, EKF or obstacle truth.
    R.isolated_direct_native_status='NOT_ATTEMPTED';
    R.isolated_direct_native_calls=0;
    if isfield(cfg,'native_binding') && isfield(cfg,'numeric_tolerances')
        direct=TRIGGER_ISOLATED_DIRECT_NATIVE_PREPARE(cfg);
        R.isolated_direct_native_status=direct.status;
        R.isolated_direct_native_calls= ...
            direct.first_acquisition_calls+direct.append_calls+ ...
            direct.reuse_calls;
        R.isolated_direct_native_reason=direct.reason;
    end
    % Geometric near-contact fixture: NEVER impersonates actual sensor
    % provenance and never publishes a planner request.  Try to exercise
    % the same sampled geometry routines that are cold at first contact.
    R.isolated_near_geometry_status='NOT_RUN';
    try
        near=OBSTACLE_INFORMATION.define(2,'sphere',e.p,1);
        frame_near=frame;
        frame_near.detected_obstacles=near;
        frame_near.detected_ids=2;
        for trial=1:8
            frame_near.time_s=(trial-1)*cfg.sample_dt_s;
            temp.evaluate(frame_near.time_s,e,frame_near,a);
        end
        R.isolated_near_geometry_status='TRIED_ISOLATED_NO_PROVENANCE';
    catch near_error
        R.isolated_near_geometry_status=['NOT_COMPLETED__' near_error.identifier];
    end
    R.passed=true;
    R.status='ISOLATED_BRANCH_EXERCISE_PASS__NO_JIT_OR_SAFETY_GUARANTEE';
catch ME
    R.reason=[ME.identifier ' | ' ME.message];
    R.status='ISOLATED_JIT_PREPARE_SKIPPED__NO_FLIGHT_DATA_TOUCHED';
end
R.elapsed_ms=1000*toc(started);
end
function P=local_numeric_fixture(req)
% Deliberately stationary throwaway samples, never a physical prediction.
n=numel(req.sample_times_s);
P=struct('sample_times_s',req.sample_times_s,...
    'uav_m',repmat(req.ekf_now.p,1,n),...
    'payload_m',repmat(req.ekf_now.pL,1,n),...
    'model_description','ISOLATED_SOFTWARE_PATH_TEST__NOT_A_PHYSICAL_MODEL');
end
