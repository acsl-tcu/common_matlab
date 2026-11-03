classdef REPLANNING_SYSTEM < handle
    % REPLANNING_SYSTEM  センサ、既知障害物メモリ、未来衝突トリガの統括。
    %
    % 役割:
    %   (1) 既存の OBSTACLE_INFORMATION.definitions() を Flight 毎に1回読む。
    %   (2) 模擬センサへ検知半径・ログフラグ・障害物の原定義を渡す。
    %   (3) 各時刻の現在EKF機体位置から検知結果を作り、Sim LOGGER へ渡す。
    %
    % 現段階のトリガは将来予測の参考情報のみ。軌道変更や連続安全認証は行わない。
    % OBSTACLE_INFORMATION と OBSTACLE_SHAPES は変更せず、読み取るだけ。
    %
    % 実Flightでの使用例:
    %   replanner = REPLANNING_SYSTEM(options);  % Flight前に1回
    %   ...  % 同じtickでMOTIVE→EKF→通常の参照→HLC→plant
    %   replanner.update(time, agent);          % Sim LOGGER.logging の直前
    %   logger.logging(time, cha, agent, []);   % 既存Simの標準記録
    %
    % updateがEKFの位置取得・模擬センサ・reference.resultへの格納を統括する。
    % runは個別のセンサや障害物ログを操作しない。Sim LOGGERは変更しない。
    % HLCや通常参照の出力は変更しない。

    properties (SetAccess=private)
        obstacles
        load_count=0
        frame_count=0
        load_time_ms=NaN
        total_frame_time_ms=0
        last_logged_time_s=NaN
        SENSOR_RANGE_M=20
        last_sensor_result=struct() % ログOFFでも最新の検知結果は保持
        last_memory_result=struct() % Observed-only cumulative knowledge, never future truth
        last_trigger_result=struct() % Prediction-only; no control/trajectory alteration
        trigger_count=0
        trigger_time_total_ms=0
        trigger_time_max_ms=0
        trigger_authority_binding=[]
        % Source parity is checked from the OFFLINE audit once per Flight;
        % an unverified native engine MUST NOT consume milliseconds every tick.
        % No sensor/plant/EKF/reference changes or synthetic warmup occur.
        trigger_native_gate_checked=false
        trigger_native_gate_status='UNKNOWN_NATIVE_PARITY_NOT_VERIFIED' 
        native_mex_preflight_build_status='NOT_CHECKED'
        preflight_model_binding_ready=false
        preflight_model_binding_ms=NaN
        preflight_model_binding_status='NOT_PREBOUND'
        isolated_first_use_prepare_ms=NaN
        isolated_first_use_prepare_status='NOT_RUN'
        isolated_native_prediction_status='NOT_ATTEMPTED'
        isolated_native_prediction_calls=0
        isolated_sensor_event_prepare_status='NOT_ATTEMPTED'
        isolated_sensor_event_prepare_steps=0
        isolated_direct_native_status='NOT_ATTEMPTED'
        isolated_direct_native_calls=0
        isolated_direct_native_verified_appends=0
        isolated_direct_native_outer_appends=0
        isolated_near_geometry_cases=0
        isolated_near_geometry_status='NOT_ATTEMPTED'
        isolated_outer_authority_status='NOT_ATTEMPTED'
        isolated_outer_authority_calls=0
        isolated_mcos_current_prepare_status='NOT_ATTEMPTED'
        isolated_mcos_current_prepare_calls=0
        isolated_mcos_current_prepare_reason=''
        isolated_f_runtime_prepare_status='NOT_ATTEMPTED'
        isolated_f_runtime_prepare_calls=0
        isolated_f_runtime_prepare_reason=''
        % Non-Flight shadow performs genuine typed CURRENT-frame/authority
        % reads using the existing live agent; no 4-s future prediction,
        % event generation or publication. Runs by ACTUAL agent.cha only.
        phase_shadow_frame_calls=0
        phase_shadow_authority_calls=0
        phase_shadow_skipped_mass=0
        phase_shadow_total_ms=0
        phase_shadow_max_ms=0
        phase_shadow_last_phase=''
        phase_shadow_last_status='NOT_RUN'
        phase_shadow_last_reason=''
        preflight_timeonly_reference_status='NOT_ATTEMPTED'
        preflight_timeonly_reference_calls=0
        preflight_timeonly_direct_windows=0
        preflight_timeonly_direct_calls=0
        preflight_timeonly_contract_rejections=0
        first_flight_trace_enabled=false
        first_flight_traces=struct([])
        trigger_core=[]
        trigger_reference_cache=[] % Only exact source arguments; guarded Takeoff priming
        preflight_warmup_ms=NaN % Flight/Armingより前の準備時間（周期時間に算入しない）
        preflight_warmup_ready=false
        preflight_warmup_distance_checks=0
        % update()本体の初回実行経路を、初期化後・Arming前に一時実行する。
        % 本番センサ、Flightログ、EKFの履歴は一切進めない。
        preflight_update_ready=false
        preflight_update_ms=NaN
        preflight_update_samples_ms=[]
    end

    properties
        ENABLE_ALL_LOG=true
        LOG_OBSTACLES=true
        ENABLE_SENSOR=true
        LOG_SENSOR=true
        LOG_REPLANNER=true % update()単独の計測・Sim LOGGERへの記録
        LOG_OBSTACLE_MEMORY=true % memory state log only, observation still updates
        ENABLE_TRIGGER=true
        LOG_TRIGGER=true
        ENABLE_TAKEOFF_REFERENCE_PRIMING=false % Rejected candidate epoch; keep Flight/Takeoff periods free of unverified priming.
        % Physical spherical body radii and cable material radius: MUST be
        % specified from the actual Flight setup. NaN means UNBOUND, NOT 0.
        % There is intentionally no guessed fallback for these dimensions.
        TRIGGER_UAV_RADIUS_M=NaN
        TRIGGER_PAYLOAD_RADIUS_M=NaN
        TRIGGER_CABLE_RADIUS_M=NaN
        % Trigger horizon is internal design default, externally overridable.
        TRIGGER_HORIZON_S=4
        % Numerical solver tolerances are integration accuracy controls only,
        % not physical tracking/error inflation and not a safety certificate.
        TRIGGER_NUMERIC_ABS_TOL=NaN
        TRIGGER_NUMERIC_REL_TOL=NaN
        % Side-effect-free same-phase reference is valid only before this
        % externally supplied flight/landing transition.
        TRIGGER_FLIGHT_PHASE_END_S=NaN
    end

    properties (Access=private)
        sensor
        observed_memory
        % Only shapes disclosed by the sensor AFTER actual observation.
        % Reuses the sensor's prepared representation; never accesses
        % precomputed geometry of obstacles not yet detected.
        observed_prepared_shapes=[]
        observed_prepared_ids=zeros(1,0)
        static_frame
        all_static=false
        replanner_timing_count=0
        replanner_timing_total_ms=0
        replanner_timing_max_ms=0
        % Environment switch: REPLANNING_STAGE_TIMING=1 enables a diagnostic
        % 1x6 timing vector in the EXISTING Sim LOGGER reference.result.
        % Default OFF: no extra toc calls or per-tick arrays in production.
        stage_timing_enabled=false
        % Optional phase-local source evaluation, never a sensor/dynamics
        % warmup. A candidate epoch is only a prediction of a phase switch:
        % first Flight will independently verify it before any reuse.
        takeoff_prime_candidate_epoch=NaN
        takeoff_prime_target=NaN
        takeoff_prime_source=[]
        takeoff_prime_disabled=false
        takeoff_prime_error=''
    end

    methods
        function obj=REPLANNING_SYSTEM(options)
            if nargin<1,options=struct();end
            if ~isstruct(options)||~isscalar(options)
                error('REPLANNING_SYSTEM:Options','設定はスカラー構造体で指定してください。');
            end
            allowed={'ENABLE_ALL_LOG','LOG_OBSTACLES','ENABLE_SENSOR', ...
                'LOG_SENSOR','LOG_REPLANNER','LOG_OBSTACLE_MEMORY', ...
                'ENABLE_TRIGGER','LOG_TRIGGER','ENABLE_TAKEOFF_REFERENCE_PRIMING','TRIGGER_UAV_RADIUS_M', ...
                'TRIGGER_PAYLOAD_RADIUS_M','TRIGGER_CABLE_RADIUS_M', ...
                'TRIGGER_HORIZON_S','TRIGGER_NUMERIC_ABS_TOL', ...
                'TRIGGER_NUMERIC_REL_TOL','TRIGGER_FLIGHT_PHASE_END_S','SENSOR_RANGE_M'};
            names=fieldnames(options);
            for k=1:numel(names)
                field=names{k};value=options.(field);
                if ~ismember(field,allowed)
                    error('REPLANNING_SYSTEM:UnknownOption','未知の設定項目です: %s',field);
                end
                if strcmp(field,'SENSOR_RANGE_M')
                    REPLANNING_SYSTEM.check_range(value);
                    obj.SENSOR_RANGE_M=double(value);
                elseif ismember(field,{'TRIGGER_NUMERIC_ABS_TOL', ...
                        'TRIGGER_NUMERIC_REL_TOL','TRIGGER_FLIGHT_PHASE_END_S'})
                    REPLANNING_SYSTEM.check_range(value);
                    obj.(field)=double(value);
                elseif strcmp(field,'TRIGGER_HORIZON_S')
                    REPLANNING_SYSTEM.check_range(value);
                    obj.TRIGGER_HORIZON_S=double(value);
                elseif ismember(field,{'TRIGGER_UAV_RADIUS_M','TRIGGER_PAYLOAD_RADIUS_M'})
                    REPLANNING_SYSTEM.check_range(value);
                    obj.(field)=double(value);
                elseif strcmp(field,'TRIGGER_CABLE_RADIUS_M')
                    REPLANNING_SYSTEM.check_nonnegative(value,field);
                    obj.(field)=double(value);
                else
                    REPLANNING_SYSTEM.check_flag(value,field);
                    obj.(field)=logical(value);
                end
            end

            stage_env=strtrim(getenv('REPLANNING_STAGE_TIMING'));
            if ~(isempty(stage_env)||strcmp(stage_env,'0')||strcmp(stage_env,'1'))
                error('REPLANNING_SYSTEM:StageTimingFlag', ...
                    'REPLANNING_STAGE_TIMING must be empty, 0, or 1.');
            end
            obj.stage_timing_enabled=strcmp(stage_env,'1');
            % First 8 eligible Flight evaluations only. No file I/O, no
            % synthetic sensor/flight/ODE calls. Optional; default OFF.
            obj.first_flight_trace_enabled=strcmp( ...
                strtrim(getenv('TRIGGER_FIRST_FLIGHT_TRACE')),'1');

            stopwatch=tic;
            obj.obstacles=OBSTACLE_INFORMATION.definitions();
            OBSTACLE_INFORMATION.validate_list(obj.obstacles);
            obj.load_count=1;
            obj.load_time_ms=1000*toc(stopwatch);
            obj.all_static=all(arrayfun(@(o) strcmp(o.motion,'static'),obj.obstacles));
            if obj.all_static
                obj.static_frame=struct('time_s',0,'obstacles',obj.obstacles);
            end

            % センサの準備は一度だけ。形状定義側へ手を入れない。
            obj.sensor=SIMULATED_OBSTACLE_SENSOR(obj.obstacles,obj.SENSOR_RANGE_M, ...
                obj.ENABLE_ALL_LOG&&obj.LOG_SENSOR);
            % An empty array is ONLY a schema. Memory never reads source truth.
            obj.observed_memory=OBSERVED_OBSTACLE_MEMORY( ...
                obj.sensor.empty_observed_obstacle_schema());
            obj.trigger_core=FUTURE_COLLISION_TRIGGER(struct('horizon_s',obj.TRIGGER_HORIZON_S));
            obj.trigger_reference_cache=TRIGGER_REFERENCE_WINDOW_CACHE();
            % No world-dependent simulated observations before Flight.
            % The sensor only prepares geometry/MEX; its scan history stays empty.
            % preflight_update is a no-op; NO synthetic world and NO trial scans.
            obj.preflight_warmup_ready=false;
            obj.preflight_warmup_ms=NaN;
            obj.preflight_warmup_distance_checks=0;
        end

        function yes=logging_enabled(obj)
            yes=obj.ENABLE_ALL_LOG&&obj.LOG_OBSTACLES;
        end

        function frame=log_frame(obj,time_s)
            % 既存Flightの障害物元定義ログを互換維持。センサ結果は混入しない。
            REPLANNING_SYSTEM.check_time(time_s);
            if ~obj.logging_enabled(),frame=[];return;end
            stopwatch=tic;
            if obj.all_static
                frame=obj.static_frame;
                frame.time_s=double(time_s);
            else
                frame=OBSTACLE_INFORMATION.log_frame(time_s);
            end
            obj.frame_count=obj.frame_count+1;
            obj.last_logged_time_s=double(time_s);
            obj.total_frame_time_ms=obj.total_frame_time_ms+1000*toc(stopwatch);
        end

        function update(obj,time,agent)
            % Flightからの唯一の周期呼出し口。
            %
            % 前提：通常のSimフローで同tickのEKFが更新済みであり、
            %       参照のdo()も終了してreference.resultが確定している。
            % 保存：Simのlogger.loggingがreference.resultを記録する。
            %       このメソッドで独自の保存やLOGGER.loggingは呼ばない。
            %
            % ENABLE_SENSORとLOG_SENSORは独立：ログを切っても検知は行う。
            % ここからリプランナーの責務だけを計測する。
            % 計測値は同じtickのreference.resultに保存し、標準LOGGERが回収する。
            measure_replanner=obj.ENABLE_ALL_LOG && obj.LOG_REPLANNER;
            if measure_replanner, update_clock=tic; end
            stage_on=obj.stage_timing_enabled && measure_replanner;
            if stage_on
                stage_clock=tic;
                stage_marks=zeros(1,6);
            end
            time_s=REPLANNING_SYSTEM.require_sim_time(time);
            if ~(isobject(agent)&&isscalar(agent)&&isprop(agent,'reference'))
                error('REPLANNING_SYSTEM:Agent', ...
                    'SimのDRONE agentを渡してください。');
            end

            % 参照結果へ追加するだけ。既存のxdや制御値は上書きしない。
            reference_result=agent.reference.result;
            if ~(isstruct(reference_result)&&isscalar(reference_result))
                error('REPLANNING_SYSTEM:ReferenceResult', ...
                    'agent.reference.resultがスカラー構造体ではありません。');
            end

            if stage_on,stage_marks(1)=1000*toc(stage_clock);end
            sensor_result=[];
            if stage_on
                % Optional detailed attribution; no allocations/tic on the
                % ordinary production path with stage timing disabled.
                detail_ms=nan(1,6);
            end
            if obj.ENABLE_SENSOR
                p_now=REPLANNING_SYSTEM.require_ekf_uav_position(agent);
                if stage_on
                    stage_marks(2)=1000*toc(stage_clock);
                    sensor_memory_clock=tic;
                end
                sensor_result=obj.observe_sensor(time_s,p_now);
                if stage_on,detail_ms(1)=1000*toc(sensor_memory_clock);end
                obj.last_sensor_result=sensor_result;
                memory_result=obj.observed_memory.update(sensor_result);
                if stage_on,detail_ms(2)=1000*toc(sensor_memory_clock)-detail_ms(1);end
                % The sensor already prepared these very same shapes for
                % its own true-solid queries. Adopt a prepared shape ONLY
                % when its ID has JUST been learned from the real scan.
                % Never use a source-world lookup or a preflight scan.
                newly_learned=memory_result.newly_acquired_ids;
                if ~isempty(newly_learned)
                    shapes_now=obj.sensor.observed_prepared_only(newly_learned);
                    % Actual sensor geometry can be Kx1 or 1xK.
                    % Normalize ONCE on the acquisition event; keep ID order.
                    new_prepared=OBSERVED_OBSTACLE_MEMORY.append_row_structs( ...
                        obj.observed_prepared_shapes,shapes_now);
                    new_prepared_ids=[reshape(obj.observed_prepared_ids,1,[]) ...
                        reshape(double(newly_learned),1,[])];
                    assert(numel(new_prepared)==numel(new_prepared_ids) && ...
                        isequal(reshape(double([new_prepared.id]),1,[]), ...
                        new_prepared_ids), ...
                        'REPLANNING_SYSTEM:PreparedObservationMismatch', ...
                        'Actual observed prepared geometry and memory IDs differ.');
                    obj.observed_prepared_shapes=new_prepared;
                    obj.observed_prepared_ids=new_prepared_ids;
                end
                obj.last_memory_result=memory_result;
                if stage_on
                    detail_ms(3)=1000*toc(sensor_memory_clock)- ...
                        detail_ms(1)-detail_ms(2);
                end
            else
                if stage_on,stage_marks(2)=1000*toc(stage_clock);end
                obj.last_sensor_result=struct();
                obj.last_memory_result=struct();
            end
            if stage_on,stage_marks(3)=1000*toc(stage_clock);end

            % Removed the rejected Takeoff-period priming hot path.
            % TAKEOFF_REFERENCE.base_time is not the proven first-Flight
            % epoch. Prior priming frequently discarded all columns and
            % shifted its first-use overhead into t=0s. Keep any optional
            % software preparation strictly in preflight_update(), on
            % throwaway objects without live sensor/EKF/reference history.
            % The real first Flight reference is still evaluated at its
            % actual double-valued time argument, with no interpolation.

            % Same-typed, actual-current software shadow in Arming/Takeoff.
            % Runs AFTER genuine same-tick sensor/observer updates, BEFORE
            % ordinary phase-guarded trigger evaluation. Never publishes
            % events, touches reference epochs, uses a predicted Flight
            % phase time or changes controller/plant inputs.
            if obj.ENABLE_TRIGGER && obj.ENABLE_SENSOR
                obj.prepare_nonflight_shadow(time,agent);
            end

            % Trigger evaluation uses ONLY the sensor-derived memory frame.
            % No obstacle truth from obj.obstacles enters the trigger.
            % No future sensing, no artificial warmup, no HLC/plant mutation.
            trigger_clock=tic;
            if ~obj.ENABLE_TRIGGER
                trigger_result=struct('time_s',time_s,'status','DISABLED', ...
                    'prediction_available',false,'considered_obstacle_ids',[], ...
                    'possible_risk_ids',[],'new_event',false, ...
                    'continuous_observed_set_safe',false,'reason','Explicitly disabled');
            elseif ~obj.ENABLE_SENSOR
                trigger_result=struct('time_s',time_s,'status','UNKNOWN_SENSOR_DISABLED', ...
                    'prediction_available',false,'considered_obstacle_ids',[], ...
                    'possible_risk_ids',[],'new_event',false, ...
                    'continuous_observed_set_safe',false,'reason','Sensor disabled; unseen is not safe');
            else
                trigger_result=obj.evaluate_trigger(time,agent);
            end
            trigger_result.elapsed_ms=1000*toc(trigger_clock);
            if stage_on,detail_ms(4)=trigger_result.elapsed_ms;end
            obj.last_trigger_result=trigger_result;
            obj.trigger_count=obj.trigger_count+1;
            obj.trigger_time_total_ms=obj.trigger_time_total_ms+trigger_result.elapsed_ms;
            obj.trigger_time_max_ms=max(obj.trigger_time_max_ms,trigger_result.elapsed_ms);

            % ログONなら既存値を上書き。OFFに切り替えた場合だけ消す。
            % 毎tickのrmfield→再追加は不要なので実行しない。
            if stage_on,obstacle_log_clock=tic;end
            if obj.logging_enabled()
                if obj.all_static
                    % Time was already validated by require_sim_time().
                    % Inlining the constant-frame path avoids a second method
                    % dispatch and redundant validation on every Flight tick.
                    % Same obstacle data/clock/counters as log_frame(); direct
                    % external log_frame() calls keep their original behavior.
                    frame_clock=tic;
                    frame=obj.static_frame;
                    frame.time_s=time_s;
                    obj.frame_count=obj.frame_count+1;
                    obj.last_logged_time_s=time_s;
                    obj.total_frame_time_ms=obj.total_frame_time_ms+1000*toc(frame_clock);
                    reference_result.obstacle_information=frame;
                else
                    % Moving obstacle logging retains original code path.
                    reference_result.obstacle_information=obj.log_frame(time_s);
                end
            elseif isfield(reference_result,'obstacle_information')
                reference_result=rmfield(reference_result,'obstacle_information');
            end
            if stage_on
                detail_ms(5)=1000*toc(obstacle_log_clock);
                stage_marks(4)=1000*toc(stage_clock);
            end
            if obj.ENABLE_SENSOR && obj.ENABLE_ALL_LOG && obj.LOG_SENSOR
                reference_result.simulated_obstacle_sensor=sensor_result;
            elseif isfield(reference_result,'simulated_obstacle_sensor')
                reference_result=rmfield(reference_result,'simulated_obstacle_sensor');
            end
            % Separate current-sensor logs from learned obstacle memory.
            if obj.ENABLE_SENSOR && obj.ENABLE_ALL_LOG && obj.LOG_OBSTACLE_MEMORY
                reference_result.observed_obstacle_memory=memory_result;
            elseif isfield(reference_result,'observed_obstacle_memory')
                reference_result=rmfield(reference_result,'observed_obstacle_memory');
            end
            if obj.ENABLE_ALL_LOG && obj.LOG_TRIGGER
                reference_result.future_collision_trigger=trigger_result;
                reference_result.trigger_time_ms=trigger_result.elapsed_ms;
            else
                if isfield(reference_result,'future_collision_trigger')
                    reference_result=rmfield(reference_result,'future_collision_trigger');
                end
                if isfield(reference_result,'trigger_time_ms')
                    reference_result=rmfield(reference_result,'trigger_time_ms');
                end
            end
            if stage_on
                stage_marks(5)=1000*toc(stage_clock);
                detail_ms(6)=stage_marks(5)-stage_marks(4);
            end
            % センサ時間はこのupdate時間に含まれるため、二重加算しない。
            % 末尾の結果プロパティ代入は計測範囲外（小さな固定オーバーヘッド）。
            if measure_replanner
                update_ms=1000*toc(update_clock);
                reference_result.replanning_time_ms=update_ms;
                obj.replanner_timing_count=obj.replanner_timing_count+1;
                obj.replanner_timing_total_ms=obj.replanner_timing_total_ms+update_ms;
                obj.replanner_timing_max_ms=max(obj.replanner_timing_max_ms,update_ms);
            elseif isfield(reference_result,'replanning_time_ms')
                reference_result=rmfield(reference_result,'replanning_time_ms');
            end
            if stage_on
                stage_marks(6)=1000*toc(stage_clock);
                % Durations: input, EKF, sensor+state, obstacle log,
                % sensor attachment, timing bookkeeping. Includes toc overhead.
                % Published through the ORIGINAL LOGGER (not a shadow logger).
                reference_result.replanning_stage_ms=diff([0,stage_marks]);
                reference_result.replanning_detail_ms=detail_ms;
            elseif isfield(reference_result,'replanning_detail_ms')
                % A new Flight that has detail timing OFF must not inherit
                % previous detail data from a recycled reference struct.
                reference_result=rmfield(reference_result,'replanning_detail_ms');
            end
            agent.reference.result=reference_result;
        end

        function prepare_nonflight_shadow(obj,time,agent)
            % Standby-phase exercise of the SAME MCOS property-access path
            % as real Flight. The real sensor/memory already ran this tick.
            % The current EKF is read only; the shadow result is discarded.
            % This has no authority to predict a future phase, issue risk
            % events, modify HLC or claim any degree of physical safety.
            if ~(isprop(agent,'cha') && ...
                    (isequal(char(agent.cha),'a') || ...
                     isequal(char(agent.cha),'t')))
                return
            end
            started=tic;
            obj.phase_shadow_last_phase=char(agent.cha);
            obj.phase_shadow_last_reason='';
            try
                known=obj.get_known_obstacle_frame_for_trigger();
                actual_ids=reshape(double(obj.last_memory_result.known_obstacle_ids),1,[]);
                assert(isequal(reshape(double(known.detected_ids),1,[]),actual_ids), ...
                    'REPLANNING_SYSTEM:ShadowFrame', ...
                    'Observed-only shadow frame does not match live memory.');
                obj.phase_shadow_frame_calls=obj.phase_shadow_frame_calls+1;
                % Stage validated REAL static observed geometry gradually.
                % No source-world lookup, synthetic detection or future
                % trajectory; never commit Flight events or control changes.
                % All failures leave the original Flight validator intact.
                obj.trigger_core.prepare_shadow_observed_static( ...
                    known,REPLANNING_SYSTEM.require_sim_time(time), ...
                    obj.last_sensor_result.uav_position_ekf_m);
                % The original controller can legitimately have zero
                % payload mass before loading. current() requires positive
                % HLC mass and positive EKF mass; do not invent substitutes.
                if ~obj.preflight_model_binding_ready || ...
                        isempty(obj.trigger_authority_binding)
                    obj.phase_shadow_last_status='SKIPPED_STATIC_BINDING_NOT_READY';
                else
                    state=agent.estimator.result.state;
                    mL=double(state.mL);
                    if strcmp(class(agent.parameter),'DRONE_PARAM_SUSPENDED_LOAD')
                        hlc_mass=double(agent.parameter.loadmass);
                    else
                        hlc_mass=double(agent.parameter.get("loadmass"));
                    end
                    if ~(isscalar(mL)&&isfinite(mL)&&mL>0 && ...
                            isscalar(hlc_mass)&&isfinite(hlc_mass)&&hlc_mass>0)
                        obj.phase_shadow_skipped_mass=obj.phase_shadow_skipped_mass+1;
                        obj.phase_shadow_last_status='SKIPPED_MASS_NOT_POSITIVE';
                    else
                        % current() validates only the HANDLE at this call
                        % boundary. It never invokes the reference function.
                        % This reference is deliberately not a phase-f
                        % prediction; its output is discarded immediately.
                        unused_reference=@(unused_time) zeros(28,1); %#ok<NASGU>
                        [current_ekf,current_authority]= ...
                            obj.trigger_authority_binding.current( ...
                                time,agent,unused_reference);
                        assert(isstruct(current_ekf) && ...
                            isstruct(current_authority) && ...
                            isfield(current_authority,'verified_native_binding'), ...
                            'REPLANNING_SYSTEM:ShadowAuthority', ...
                            'Current authority returned an invalid snapshot.');
                        obj.phase_shadow_authority_calls= ...
                            obj.phase_shadow_authority_calls+1;
                        obj.phase_shadow_last_status= ...
                            'REAL_MCOSTYPE_CURRENT_DISCARDED__NO_FUTURE_PHASE';
                    end
                end
            catch ME
                obj.phase_shadow_last_status='SHADOW_PREPARE_FAILED__NO_SAFETY_CHANGE';
                obj.phase_shadow_last_reason=[ME.identifier ' | ' ME.message];
            end
            elapsed=1000*toc(started);
            obj.phase_shadow_total_ms=obj.phase_shadow_total_ms+elapsed;
            obj.phase_shadow_max_ms=max(obj.phase_shadow_max_ms,elapsed);
        end

        function result=preflight_update(obj,time,agent)
            % Before arming: code/MEX checks plus STATIC binding to the
            % REAL already-initialized controller/plant/physical parameters.
            % No EKF/result read, no future reference evaluation, no HLC.do,
            % no plant rollout, no sensor.observe, no obstacle ID access.
            % Runtime HLC mass and latest EKF mass are read ONLY during Flight.
            % On prebind failure, fail closed; NEVER retry inside update().
            started=tic;
            obj.preflight_update_ready=false;
            obj.preflight_model_binding_ready=false;
            obj.preflight_model_binding_ms=NaN;
            obj.preflight_model_binding_status='NOT_PREBOUND';
            obj.trigger_authority_binding=[];
            % Do not carry a prior Flight's reference eligibility or jets.
            obj.preflight_timeonly_direct_windows=0;
            obj.preflight_timeonly_direct_calls=0;
            obj.preflight_timeonly_contract_rejections=0;
            obj.trigger_reference_cache.clear_source_audit();
            obj.takeoff_prime_candidate_epoch=NaN;
            obj.takeoff_prime_target=NaN;
            obj.takeoff_prime_source=[];
            obj.takeoff_prime_disabled=false;
            obj.takeoff_prime_error='';
            obj.isolated_first_use_prepare_ms=NaN;
            obj.isolated_first_use_prepare_status='NOT_RUN';
            obj.isolated_native_prediction_status='NOT_ATTEMPTED';
            obj.isolated_native_prediction_calls=0;
            obj.isolated_sensor_event_prepare_status='NOT_ATTEMPTED';
            obj.isolated_sensor_event_prepare_steps=0;
            obj.isolated_direct_native_status='NOT_ATTEMPTED';
            obj.isolated_direct_native_calls=0;
            obj.isolated_direct_native_verified_appends=0;
            obj.isolated_direct_native_outer_appends=0;
            obj.isolated_near_geometry_cases=0;
            obj.isolated_near_geometry_status='NOT_ATTEMPTED';
            obj.isolated_outer_authority_status='NOT_ATTEMPTED';
            obj.isolated_outer_authority_calls=0;
            obj.isolated_mcos_current_prepare_status='NOT_ATTEMPTED';
            obj.isolated_mcos_current_prepare_calls=0;
            obj.isolated_mcos_current_prepare_reason='';
            obj.trigger_native_gate_checked=true;
            obj.trigger_native_gate_status='UNKNOWN_NATIVE_PARITY_NOT_VERIFIED';
            if obj.ENABLE_TRIGGER
                try
                    % SOFTWARE-ONLY MEX build/check, strictly BEFORE Flight.
                    % Never compile, rehash or load an audit in update().
                    build=TRIGGER_NATIVE_LOCAL_MEX('ensure');
                    obj.native_mex_preflight_build_status=build.status;
                    if ~build.ready
                        error('REPLANNING_SYSTEM:NativeMexPreflight', ...
                            'Native MEX not ready: %s (%s)', ...
                            build.status,build.reason);
                    end
                    % MEX is approved ONLY if the preflight source pin has
                    % verified either its current offline parity report or
                    % the already verified executable-code fingerprint.
                    % A fresh, different C++ numerical kernel is NOT approved.
                    pin=TRIGGER_NATIVE_SOURCE_PIN_CACHE('prepare');
                    obj.preflight_update_ready=logical(pin.passed);
                    if pin.passed
                        % Load the actual compiled native library BEFORE
                        % Flight, without running it or touching agent,
                        % estimator, sensor, obstacles or reference. This
                        % uses the source-pinned MEX 'preload' entrypoint.
                        % If the user did not explicitly REBUILD the MEX,
                        % the old binary must NOT be silently accepted.
                        preloaded=TRIGGER_SYMBOLIC_NATIVE_CORE('preload');
                        assert(isequal(preloaded,true), ...
                            'REPLANNING_SYSTEM:NativePreload', ...
                            'The pinned MEX did not confirm code-only preload.');
                        obj.trigger_native_gate_status= ...
                            'OFFLINE_PARITY_PASSED_PENDING_NATIVE_HASH_PIN';
                        report='PURE_SOURCE_PIN_AND_NATIVE_CODE_PRELOAD_PASS';
                        % The real physical model and HLC were constructed by
                        % the original runner before this method. Read their
                        % immutable definitions and validate once here; the
                        % phase-specific reference and current EKF remain
                        % strictly unqueried. This removes cold-start model
                        % binding from t=5 s rather than moving it to arming.
                        model_clock=tic;
                        geometry_source=@()struct( ...
                            'r_uav_m',obj.TRIGGER_UAV_RADIUS_M, ...
                            'r_payload_m',obj.TRIGGER_PAYLOAD_RADIUS_M, ...
                            'r_cable_m',obj.TRIGGER_CABLE_RADIUS_M, ...
                            'horizon_s',obj.TRIGGER_HORIZON_S);
                        try
                            obj.trigger_authority_binding= ...
                                TRIGGER_FLIGHT_AUTHORITY(time,agent, ...
                                    geometry_source,[]);
                            % Only Flight-independent dt/H. This is a
                            % preflight MEMORY ALLOCATION, not warmup,
                            % prediction or an obstacle query.
                            obj.trigger_core.prepare_static_timing( ...
                                obj.trigger_authority_binding.flight.sample_dt_s,...
                                obj.TRIGGER_HORIZON_S);
                            obj.preflight_model_binding_ready=true;
                            obj.preflight_model_binding_status= ...
                                'REAL_STATIC_MODEL_BOUND__NO_EKF_OR_REFERENCE';
                        catch model_error
                            obj.trigger_authority_binding=[];
                            obj.preflight_model_binding_ready=false;
                            obj.preflight_model_binding_status= ...
                                ['UNKNOWN_STATIC_BINDING__' model_error.identifier];
                            obj.preflight_update_ready=false;
                            report=obj.preflight_model_binding_status;
                        end
                        obj.preflight_model_binding_ms=1000*toc(model_clock);
                        if obj.preflight_model_binding_ready
                            % Audit the original reference function HANDLE
                            % and immutable constants before any evaluation.
                            % For the strict, stateless time-only subset,
                            % optionally exercise the scalar function CODE
                            % with argument zero, discard ALL values and
                            % never use Flight epoch or future measurements.
                            % Non-eligible source: no preflight evaluation.
                            % Audit original immutable reference before any
                            % synthetic exercise. Do not run source probe yet:
                            % the next fixture calls the cache with a dummy
                            % function handle; the ACTUAL time-only probe
                            % should be the last cache call before arming.
                            ref_fn=[];
                            source_audit=struct('eligible',false);
                            try
                                ref_fn=agent.reference.timevarying.func;
                                source_audit=obj.trigger_reference_cache.audit_immutable_source(ref_fn);
                            catch
                                % Optional source probe; fail closed by
                                % skipping it, never changing Flight gates.
                            end
                            % Pre-Arming SOFTWARE-only JIT prepare on an
                            % isolated mathematical fixture. This never
                            % reads an EKF, the sensor, an actual obstacle,
                            % real future reference, or modifies agent.
                            % Failure is nonfatal: it is NOT a safety gate.
                            pure_cfg=struct(...
                                'sample_dt_s',obj.trigger_authority_binding.flight.sample_dt_s,...
                                'horizon_s',obj.TRIGGER_HORIZON_S,...
                                'r_uav_m',obj.TRIGGER_UAV_RADIUS_M,...
                                'r_payload_m',obj.TRIGGER_PAYLOAD_RADIUS_M,...
                                'r_cable_m',obj.TRIGGER_CABLE_RADIUS_M,...
                                'cable_length_m',obj.trigger_authority_binding.flight.cable_length_m);
                            % STATIC physical/HLC source binding only.
                            % No real EKF, obstacle, future reference or
                            % Flight start epoch is supplied to the fixture.
                            pure_cfg.native_binding= ...
                                obj.trigger_authority_binding.flight;
                            pure_cfg.numeric_tolerances= ...
                                [obj.TRIGGER_NUMERIC_ABS_TOL; ...
                                 obj.TRIGGER_NUMERIC_REL_TOL];
                            pure=TRIGGER_ISOLATED_CODE_PREPARE(pure_cfg);
                            % Same-MCOS-type software first-use, BEFORE arming.
                            % Pass STATIC parameter order and the already-bound
                            % model definition only; do not read live EKF,
                            % sensor, obstacles, reference values or phase time.
                            % The fixture is a disposable DRONE/TIME/STATE_CLASS;
                            % only TRIGGER_FLIGHT_AUTHORITY.current is executed.
                            % This is NOT a Flight trigger, safety certificate,
                            % event update, or confirmed JIT compilation.
                            try
                                mcos_probe=TRIGGER_ISOLATED_MCOSTYPE_PREPARE( ...
                                    agent.parameter.parameter_order, ...
                                    obj.trigger_authority_binding.flight);
                                obj.isolated_mcos_current_prepare_status= ...
                                    mcos_probe.status;
                                obj.isolated_mcos_current_prepare_calls= ...
                                    mcos_probe.calls;
                                obj.isolated_mcos_current_prepare_reason= ...
                                    mcos_probe.reason;
                            catch ME
                                obj.isolated_mcos_current_prepare_status= ...
                                    ['NOT_COMPLETED_MCOSTYPE__' ME.identifier];
                                obj.isolated_mcos_current_prepare_calls=0;
                                obj.isolated_mcos_current_prepare_reason= ...
                                    [ME.identifier ' | ' ME.message];
                            end
                            % Exercise the EXACT Flight inner call tree
                            % on disposable genuine MCOS objects. Only the
                            % source-identified model is reused; all
                            % synthetic clocks, sensing, EKF and cache are
                            % discarded, with no Flight-phase assumption.
                            try
                                kernel_probe=TRIGGER_F_PHASE_KERNEL_PREPARE( ...
                                    agent.parameter.parameter_order, ...
                                    obj.trigger_authority_binding.flight, ...
                                    obj.TRIGGER_HORIZON_S, ...
                                    [obj.TRIGGER_NUMERIC_ABS_TOL; ...
                                     obj.TRIGGER_NUMERIC_REL_TOL]);
                                obj.isolated_f_runtime_prepare_status= ...
                                    kernel_probe.status;
                                obj.isolated_f_runtime_prepare_calls= ...
                                    kernel_probe.calls;
                                obj.isolated_f_runtime_prepare_reason= ...
                                    kernel_probe.reason;
                            catch ME
                                obj.isolated_f_runtime_prepare_status= ...
                                    ['NOT_COMPLETED__' ME.identifier];
                                obj.isolated_f_runtime_prepare_calls=0;
                                obj.isolated_f_runtime_prepare_reason= ...
                                    [ME.identifier ' | ' ME.message];
                            end
                            obj.isolated_first_use_prepare_ms=pure.elapsed_ms;
                            obj.isolated_first_use_prepare_status=pure.status;
                            if isfield(pure,'isolated_sensor_event_prepare_status')
                                obj.isolated_sensor_event_prepare_status= ...
                                    pure.isolated_sensor_event_prepare_status;
                                obj.isolated_sensor_event_prepare_steps= ...
                                    pure.isolated_sensor_event_prepare_steps;
                            end
                            if isfield(pure,'isolated_native_prediction_status')
                                obj.isolated_native_prediction_status= ...
                                    pure.isolated_native_prediction_status;
                            end
                            if isfield(pure,'isolated_native_prediction_calls')
                                obj.isolated_native_prediction_calls= ...
                                    pure.isolated_native_prediction_calls;
                            end
                            if isfield(pure,'isolated_direct_native_status')
                                obj.isolated_direct_native_status= ...
                                    pure.isolated_direct_native_status;
                            end
                            if isfield(pure,'isolated_direct_native_calls')
                                obj.isolated_direct_native_calls= ...
                                    pure.isolated_direct_native_calls;
                            end
                            if isfield(pure,'isolated_direct_native_verified_appends')
                                obj.isolated_direct_native_verified_appends= ...
                                    pure.isolated_direct_native_verified_appends;
                            end
                            if isfield(pure,'isolated_direct_native_outer_appends')
                                obj.isolated_direct_native_outer_appends= ...
                                    pure.isolated_direct_native_outer_appends;
                            end
                            if isfield(pure,'isolated_near_geometry_cases')
                                obj.isolated_near_geometry_cases= ...
                                    pure.isolated_near_geometry_cases;
                            end
                            if isfield(pure,'isolated_near_geometry_status')
                                obj.isolated_near_geometry_status= ...
                                    pure.isolated_near_geometry_status;
                            end
                            if isfield(pure,'isolated_outer_authority_status')
                                obj.isolated_outer_authority_status= ...
                                    pure.isolated_outer_authority_status;
                            end
                            if isfield(pure,'isolated_outer_authority_calls')
                                obj.isolated_outer_authority_calls= ...
                                    pure.isolated_outer_authority_calls;
                            end
                            % LAST pre-arm cache-path exercise, AFTER all
                            % fixture_jet calls. The cache and every value
                            % are disposable, with synthetic local clock
                            % and no Phase-f epoch, EKF or obstacle queries.
                            % No Flight prediction or event is authorized.
                            try
                                cfg=struct( ...
                                    'nominal',agent.reference.timevarying, ...
                                    'sample_dt_s',pure_cfg.sample_dt_s, ...
                                    'horizon_s',pure_cfg.horizon_s, ...
                                    'cable_length_m',pure_cfg.cable_length_m);
                                probe=TRIGGER_TIMEONLY_SOURCE_PREPARE( ...
                                    ref_fn,source_audit,cfg);
                                obj.preflight_timeonly_reference_status=probe.status;
                                obj.preflight_timeonly_reference_calls=probe.calls;
                                obj.preflight_timeonly_direct_windows= ...
                                    probe.direct_exact_windows;
                                obj.preflight_timeonly_direct_calls= ...
                                    probe.direct_source_calls;
                                obj.preflight_timeonly_contract_rejections= ...
                                    probe.contract_rejections;
                            catch ME
                                obj.preflight_timeonly_reference_status= ...
                                    ['SOURCE_PROBE_EXCEPTION__' ME.identifier];
                                obj.preflight_timeonly_reference_calls=0;
                            end
                        end
                    else
                        obj.trigger_native_gate_status=char(pin.status);
                        report=['PURE_SOURCE_PIN_NOT_READY__' char(pin.status)];
                    end
                catch ME
                    obj.preflight_update_ready=false;
                    obj.preflight_model_binding_ready=false;
                    obj.trigger_authority_binding=[];
                    obj.preflight_model_binding_status= ...
                        ['UNKNOWN_SOURCE_PREPARE__' ME.identifier];
                    obj.trigger_native_gate_status= ...
                        'UNKNOWN_PREFLIGHT_SOURCE_PIN_ERROR';
                    report=['UNKNOWN_SOURCE_PIN_ERROR__' ME.identifier];
                end
            else
                obj.preflight_update_ready=true;
                report='TRIGGER_DISABLED__NO_SOURCE_PIN_REQUIRED';
            end
            obj.preflight_update_ms=1000*toc(started);
            obj.preflight_update_samples_ms=[];
            result=struct('ready',obj.preflight_update_ready, ...
                'elapsed_ms',obj.preflight_update_ms, ...
                'samples_ms',[], 'status',report, ...
                'static_model_bound',obj.preflight_model_binding_ready, ...
                'static_model_binding_ms',obj.preflight_model_binding_ms, ...
                'static_model_binding_status',obj.preflight_model_binding_status);
        end

        function frame=observe_sensor(obj,time_s,current_ekf_uav_position)
            % EKF機体位置を必須入力とする。未指定なら黙って公称値を使わない。
            if ~obj.ENABLE_SENSOR
                error('REPLANNING_SYSTEM:SensorDisabled', ...
                    'ENABLE_SENSOR=false です。検知処理は無効です。');
            end
            if nargin<3
                error('REPLANNING_SYSTEM:MissingEKF', ...
                    '現在時刻のEKF機体位置を第3引数へ渡してください。');
            end
            % 設定が変わった周期だけ検証・設定する。
            desired_log=obj.ENABLE_ALL_LOG && obj.LOG_SENSOR;
            if obj.sensor.LOG_SENSOR ~= desired_log
                obj.sensor.set_log(desired_log);
            end
            frame=obj.sensor.observe(time_s,current_ekf_uav_position);
        end

        function known=get_known_obstacle_frame(obj)
            % Read-only snapshot. Contains no unseen/future source obstacles.
            if ~obj.ENABLE_SENSOR || ~isstruct(obj.last_sensor_result) || ...
                    ~isfield(obj.last_sensor_result,'time_s')
                error('REPLANNING_SYSTEM:NoSensorFrame', ...
                    'A real Flight sensor update is required.');
            end
            known=obj.observed_memory.known_frame(obj.last_sensor_result);
            % Forward ONLY previously disclosed sensor-prepared geometry.
            % The trigger validates raw type/dimensions/pose/ID before use.
            if isequal(reshape(double(known.detected_ids),1,[]), ...
                    obj.observed_prepared_ids) && ...
                    numel(obj.observed_prepared_shapes)==numel(known.detected_ids)
                known.prepared_sensor_shapes=obj.observed_prepared_shapes;
            end
        end

        function known=get_known_obstacle_frame_for_trigger(obj)
            % Narrow trigger-specific sensor-memory adapter. The existing
            % public get_known_obstacle_frame() remains bit-for-bit intact
            % for outside callers, plots and independent diagnostics.
            if ~obj.ENABLE_SENSOR || ~isstruct(obj.last_sensor_result) || ...
                    ~isfield(obj.last_sensor_result,'time_s')
                error('REPLANNING_SYSTEM:NoSensorFrame', ...
                    'The real current Flight sensor has not updated.');
            end
            known=obj.observed_memory.known_frame_for_trigger( ...
                obj.last_sensor_result);
            if isequal(reshape(double(known.detected_ids),1,[]), ...
                    obj.observed_prepared_ids) && ...
                    numel(obj.observed_prepared_shapes)==numel(known.detected_ids)
                known.prepared_sensor_shapes=obj.observed_prepared_shapes;
            end
        end

        function configure_trigger_geometry(obj,r_uav_m,r_payload_m,r_cable_m)
            % Actual physical dimensions: supplied by integrator, never inferred.
            % Must be called before a bound flight; not a safety margin.
            REPLANNING_SYSTEM.check_range(r_uav_m);
            REPLANNING_SYSTEM.check_range(r_payload_m);
            REPLANNING_SYSTEM.check_nonnegative(r_cable_m,'r_cable_m');
            obj.TRIGGER_UAV_RADIUS_M=double(r_uav_m);
            obj.TRIGGER_PAYLOAD_RADIUS_M=double(r_payload_m);
            obj.TRIGGER_CABLE_RADIUS_M=double(r_cable_m);
            obj.trigger_authority_binding=[];
        end

        function set_trigger_horizon(obj,horizon_s)
            REPLANNING_SYSTEM.check_range(horizon_s);
            obj.TRIGGER_HORIZON_S=double(horizon_s);
            obj.trigger_core.set_horizon(horizon_s);
            obj.trigger_authority_binding=[];
        end

        function R=evaluate_trigger(obj,time,agent)
            % Errors are fail-closed UNKNOWN, never silently SAFE.
            % A nominal prediction is for trigger requests, NOT safety cert.
            t=REPLANNING_SYSTEM.require_sim_time(time);
            % The same-tick observed-memory result already contains the
            % exact cumulative IDs. Do NOT build/copy the complete geometry
            % snapshot in arming/takeoff/landing or outside the certified
            % reference phase: those phases cannot run the predictor.
            % The genuine sensor.observe() and memory.update() above still
            % execute at EVERY tick, including t=0. No sensing is skipped.
            if ~isstruct(obj.last_memory_result) || ...
                    ~isscalar(obj.last_memory_result) || ...
                    ~isfield(obj.last_memory_result,'known_obstacle_ids')
                R=struct('time_s',t,'horizon_s',obj.TRIGGER_HORIZON_S, ...
                    'considered_obstacle_ids',zeros(1,0), ...
                    'prediction_available',false, ...
                    'possible_risk_ids',zeros(1,0),'new_event',false, ...
                    'continuous_observed_set_safe',false, ...
                    'status','UNKNOWN_OBSERVED_MEMORY_NOT_READY', ...
                    'reason','No genuine current sensor-memory snapshot is available.');
                return
            end
            ids=reshape(double(obj.last_memory_result.known_obstacle_ids),1,[]);
            R=struct('time_s',t,'horizon_s',obj.TRIGGER_HORIZON_S, ...
                'considered_obstacle_ids',ids,'prediction_available',false, ...
                'possible_risk_ids',ids,'new_event',false, ...
                'continuous_observed_set_safe',false, ...
                'status','UNKNOWN','reason','');
            if isempty(ids)
                R.status='NO_KNOWN_OBSTACLES';
                R.reason='No obstacle has been actually detected yet; unknown world remains unknown.';
                return
            end
            if ~all(isfinite([obj.TRIGGER_UAV_RADIUS_M, ...
                    obj.TRIGGER_PAYLOAD_RADIUS_M,obj.TRIGGER_CABLE_RADIUS_M]))
                R.status='UNKNOWN_BODY_RADII_NOT_CONFIGURED';
                R.reason='Set real spherical UAV/payload radii and cable radius in REPLANNING_SYSTEM; no defaults.';
                return
            end
            % The existing future reference is phase-dependent. A future
            % phase schedule cannot be invented; f-only nominal evaluator is
            % explicitly marked as phase-conditional, not a certified truth.
            if ~isprop(agent,'cha') || ~isequal(char(agent.cha),'f')
                R.status='UNKNOWN_NONFLIGHT_PHASE';
                R.reason='No side-effect-free verified future phase reference for takeoff/landing.';
                return
            end
            % The original future reference evaluator is f-phase-only.
            % DO NOT extend it into a future landing phase by pretending it
            % continues forever. No hidden time-based prediction shortening.
            if ~isfinite(obj.TRIGGER_FLIGHT_PHASE_END_S)
                R.status='UNKNOWN_FUTURE_PHASE_END_NOT_BOUND';
                R.reason='Flight phase endpoint must be supplied from run.';
                return
            end
            if t+obj.TRIGGER_HORIZON_S>obj.TRIGGER_FLIGHT_PHASE_END_S
                R.status='UNKNOWN_FUTURE_PHASE_CHANGE_UNMODELLED';
                R.reason='Horizon crosses real Flight-to-landing phase; current nominal f reference is not valid there.';
                return
            end
            % Never create/validate the static live model at the first f tick.
            % If pre-arm binding was unavailable, refuse prediction rather
            % than hiding the cost at t=5 or inventing physics fallbacks.
            if ~obj.preflight_model_binding_ready || ...
                    isempty(obj.trigger_authority_binding)
                R.status='UNKNOWN_PREFLIGHT_STATIC_MODEL_NOT_BOUND';
                R.reason=obj.preflight_model_binding_status;
                return
            end
            % First-use tracing is armed ONLY after all real Flight phase
            % guards. The first eight genuine predictor ticks are captured
            % into RAM, and saved OFFLINE after the Flight.
            trace_first=obj.first_flight_trace_enabled && ...
                numel(obj.first_flight_traces)<8;
            if trace_first
                trace_start=tic; trace_step=tic;
                ft=struct('time_s',t,'known_frame_ms',NaN, ...
                    'source_gate_ms',NaN,'reference_now_ms',NaN, ...
                    'authority_create_ms',NaN,'authority_current_ms',NaN, ...
                    'core_wall_ms',NaN,'pre_store_total_ms',NaN, ...
                    'trigger_status','', 'core_trace',struct());
            end
            % Only the active f-phase can reach this full geometry snapshot.
            % It is copied once for the LIVE current EKF prediction, never
            % forged or reused from a different sensor tick.
            try
                sensed=obj.get_known_obstacle_frame_for_trigger();
                if trace_first
                    ft.known_frame_ms=1000*toc(trace_step);
                    trace_step=tic;
                end
                if ~isequal(reshape(double(sensed.detected_ids),1,[]),ids)
                    R.status='UNKNOWN_MEMORY_SNAPSHOT_MISMATCH';
                    R.reason='Current sensor-memory IDs changed during evaluation.';
                    return
                end
            catch ME
                R.status='UNKNOWN_OBSERVED_GEOMETRY_BINDING';
                R.reason=sprintf('%s: %s',ME.identifier,ME.message);
                return
            end
            % Cheap, once-per-Flight gate. The supplied source parity audit
            % must already have passed OFFLINE. In particular, a failed audit
            % is never reloaded, rehashed or retried on every Flight tick.
            % A later successful audit becomes visible on the NEXT Flight.
            % If audited true, the native predictor independently pins all
            % original MATLAB source SHA-256 hashes and MEX binary SHA-256.
            % The only source/executable validation is preflight_update().
            % Never read an offline MAT file, hash or retry a verification
            % at t=5 or later. This also avoids accidentally accepting an
            % outdated audit that refers to a different rebuilt MEX.
            if ~obj.trigger_native_gate_checked
                R.status='UNKNOWN_NATIVE_PIN_PREFLIGHT_NOT_EXECUTED';
                R.reason='Preflight source/executable check is mandatory.';
                R.possible_risk_ids=ids;
                R.continuous_observed_set_safe=false;
                R.safe_to_commit=false;
                return
            end
            if ~strcmp(obj.trigger_native_gate_status, ...
                    'OFFLINE_PARITY_PASSED_PENDING_NATIVE_HASH_PIN')
                R.status='UNKNOWN_NATIVE_PARITY_NOT_VERIFIED';
                R.reason=obj.trigger_native_gate_status;
                R.possible_risk_ids=ids;
                R.continuous_observed_set_safe=false;
                R.safe_to_commit=false;
                return
            end
            if trace_first
                ft.source_gate_ms=1000*toc(trace_step);
                trace_step=tic;
            end
            try
                % The SAME source-level Flight inner path is exercised on
                % discarded REAL-MCOS synthetic fixtures BEFORE arming.
                % This is not precomputation of a phase epoch, an obstacle
                % mask, a predicted Flight state or a collision event.
                [R,inner_trace]=TRIGGER_F_PHASE_RUNTIME_KERNEL( ...
                    time,agent,sensed,obj.trigger_authority_binding, ...
                    obj.trigger_reference_cache,obj.trigger_core, ...
                    obj.TRIGGER_NUMERIC_ABS_TOL, ...
                    obj.TRIGGER_NUMERIC_REL_TOL,trace_first);
                if trace_first
                    ft.reference_now_ms=inner_trace.reference_now_ms;
                    ft.authority_create_ms=inner_trace.authority_create_ms;
                    ft.authority_current_ms=inner_trace.authority_current_ms;
                    ft.core_wall_ms=inner_trace.core_wall_ms;
                    ft.core_trace=inner_trace.core_trace;
                end
                R.future_reference_phase_certified=false;
                R.future_reference_policy='SAME_F_PHASE_NOMINAL__PHASE_CHANGE_UNVERIFIED';
                R.continuous_observed_set_safe=false;
                R.safe_to_commit=false;
                if strcmp(R.status,'CONDITIONAL_INTERVAL_GEOMETRY_PASS__MODEL_NOT_AUDITED')
                    R.status='UNKNOWN_MODEL_AND_FUTURE_PHASE_CERTIFICATE';
                end
            catch ME
                R.status='UNKNOWN_LIVE_MODEL_BINDING';
                R.reason=sprintf('%s: %s',ME.identifier,ME.message);
                R.possible_risk_ids=ids;
                R.continuous_observed_set_safe=false;
                R.safe_to_commit=false;
            end
            if trace_first
                ft.pre_store_total_ms=1000*toc(trace_start);
                ft.trigger_status=char(R.status);
                if isempty(obj.first_flight_traces)
                    obj.first_flight_traces=ft;
                else
                    obj.first_flight_traces(end+1)=ft;
                end
            end
        end

        function set_sensor_range(obj,range_m)
            % Flight途中も外部から値を更新可能。変更はセンサへ即座に反映。
            REPLANNING_SYSTEM.check_range(range_m);
            obj.sensor.set_range(range_m);
            obj.SENSOR_RANGE_M=double(range_m);
        end

        function P=predict_with_reference_cache(obj,request,original_predictor,nominal,L,validated_t,validated_jet)
            % Exact-time reuse happens only inside active Flight prediction.
            % It is NOT warmup; no diagnostic/nominal calls are injected.
            future_times=double(request.time_s)+ ...
                double(request.sample_times_s(1:end-1));
            % Separate the REAL future-reference generation cost from the
            % native rollout, especially at the first Flight tick.
            ref_started=tic;
            % No time rounding: reuse only if query time is exactly the
            % already validated same-tick absolute time. Otherwise follow
            % the prior source-evaluation path (not a guessed extrapolation).
            if isequal(future_times(1),double(validated_t))
                [ref_matrix,~]=obj.trigger_reference_cache.evaluate( ...
                    nominal,future_times,L,request.reference_at,validated_jet);
            else
                [ref_matrix,~]=obj.trigger_reference_cache.evaluate( ...
                    nominal,future_times,L,request.reference_at);
            end
            ref_elapsed_ms=1000*toc(ref_started);
            request.reference_matrix=ref_matrix;
            P=original_predictor(request);
            P.reference_window_wall_ms=ref_elapsed_ms;
        end

        function prepare_takeoff_reference_step(obj,time,agent,time_s)
            if ~obj.preflight_model_binding_ready || ...
                    isempty(obj.trigger_authority_binding)
                return
            end
            refsys=agent.reference;
            if ~isprop(refsys,'takeoff') || ~isprop(refsys,'timevarying')
                return
            end
            takeoff=refsys.takeoff;
            nominal=refsys.timevarying;
            if ~isa(takeoff,'TAKEOFF_REFERENCE') || ...
                    ~isa(nominal,'TIME_VARYING_REFERENCE') || ...
                    isempty(takeoff.base_time) || isempty(takeoff.te)
                return
            end
            dt=double(time.dt);
            target=double(takeoff.base_time)+double(takeoff.te);
            if ~isscalar(target) || ~isfinite(target) || ...
                    target<=time_s || ~isscalar(dt) || ~isfinite(dt) || dt<=0
                return
            end
            src=nominal.func;
            if isempty(obj.takeoff_prime_source) || ...
                    ~isequal(obj.takeoff_prime_target,target) || ...
                    ~isequal(obj.takeoff_prime_source,src)
                % The phase transition time is NOT guessed from obstacles
                % or a fixed 5s constant. Reproduce the current real TIME
                % object's repeated-addition clock until the original
                % takeoff reference completion time. This is a CANDIDATE
                % only, later checked against actual nominal.t.
                candidate=double(time_s);
                max_steps=ceil((target-candidate)/dt)+2;
                if max_steps>100000 || max_steps<1
                    return
                end
                for k=1:max_steps
                    if candidate>=target,break;end
                    next=candidate+dt;
                    assert(next>candidate, ...
                        'REPLANNING_SYSTEM:PrimeClock', ...
                        'Time cannot advance with the real dt.');
                    candidate=next;
                end
                if candidate<target,return;end
                obj.takeoff_prime_candidate_epoch=candidate;
                obj.takeoff_prime_target=target;
                obj.takeoff_prime_source=src;
            end
            % TIME_VARYING_REFERENCE may already have fixed its source
            % epoch in the existing Sim initialization sweep over phases.
            % If it has not, the real first Flight tick will set that
            % epoch. Handle BOTH without guessing or mutating nominal.t.
            if isempty(nominal.t)
                source_epoch=obj.takeoff_prime_candidate_epoch;
            else
                source_epoch=double(nominal.t);
            end
            L=double(agent.parameter.get("cableL"));
            obj.trigger_reference_cache.prime_takeoff_step( ...
                nominal,obj.takeoff_prime_candidate_epoch,source_epoch,dt, ...
                obj.TRIGGER_HORIZON_S,L);
        end

        function status=get_status(obj)
            % 初期読込と元障害物ログの既存フィールドは変更しない。
            status=struct();
            status.source=which('OBSTACLE_INFORMATION');
            status.count=numel(obj.obstacles);
            status.ids=[obj.obstacles.id];
            if isempty(obj.obstacles)
                status.shapes={};
            else
                shapes=[obj.obstacles.geometry];
                status.shapes={shapes.type};
            end
            status.all_static=obj.all_static;
            status.definition_load_count=obj.load_count;
            status.definition_load_ms=obj.load_time_ms;
            status.ENABLE_ALL_LOG=obj.ENABLE_ALL_LOG;
            status.LOG_OBSTACLES=obj.LOG_OBSTACLES;
            status.frame_count=obj.frame_count;
            status.total_frame_time_ms=obj.total_frame_time_ms;
            status.last_logged_time_s=obj.last_logged_time_s;
            status.ENABLE_SENSOR=obj.ENABLE_SENSOR;
            status.LOG_SENSOR=obj.LOG_SENSOR;
            status.LOG_REPLANNER=obj.LOG_REPLANNER;
            status.replanner_timing_count=obj.replanner_timing_count;
            status.replanner_timing_total_ms=obj.replanner_timing_total_ms;
            status.replanner_timing_max_ms=obj.replanner_timing_max_ms;
            status.SENSOR_RANGE_M=obj.SENSOR_RANGE_M;
            status.preflight_warmup_ready=obj.preflight_warmup_ready;
            status.preflight_warmup_ms=obj.preflight_warmup_ms;
            status.preflight_warmup_distance_checks=obj.preflight_warmup_distance_checks;
            status.preflight_warmup_mode='DISABLED_NO_WORLD_PROBES';
            status.preflight_update_mode='STATIC_MODEL_AND_OPTIONAL_DISCARDED_TIMEONLY_SCALAR_PROBE__NO_EKF_SENSOR';
            status.native_mex_preflight_build_status= ...
                obj.native_mex_preflight_build_status;
            status.stage_timing_enabled=obj.stage_timing_enabled;
            status.stage_timing_labels={'input_validation','EKF_read', ...
                'sensor_and_memory','trigger_and_obstacle_log', ...
                'result_attachment','timing_bookkeeping'};
            status.detail_timing_labels={'sensor_observe_wall', ...
                'memory_update_wall','prepared_geometry_handoff_wall', ...
                'trigger_evaluate_wall','obstacle_log_wall', ...
                'result_attachment_wall'};
            status.preflight_timeonly_reference_status= ...
                obj.preflight_timeonly_reference_status;
            status.preflight_timeonly_reference_calls= ...
                obj.preflight_timeonly_reference_calls;
            status.preflight_timeonly_direct_windows= ...
                obj.preflight_timeonly_direct_windows;
            status.preflight_timeonly_direct_calls= ...
                obj.preflight_timeonly_direct_calls;
            status.preflight_timeonly_contract_rejections= ...
                obj.preflight_timeonly_contract_rejections;
            status.preflight_update_ready=obj.preflight_update_ready;
            status.preflight_update_ms=obj.preflight_update_ms;
            status.preflight_update_samples_ms=obj.preflight_update_samples_ms;
            status.sensor_status=obj.sensor.get_status();
            status.LOG_OBSTACLE_MEMORY=obj.LOG_OBSTACLE_MEMORY;
            status.memory_status=obj.observed_memory.get_status();
            status.ENABLE_TRIGGER=obj.ENABLE_TRIGGER;
            status.LOG_TRIGGER=obj.LOG_TRIGGER;
            status.TRIGGER_HORIZON_S=obj.TRIGGER_HORIZON_S;
            status.TRIGGER_NUMERIC_ABS_TOL=obj.TRIGGER_NUMERIC_ABS_TOL;
            status.TRIGGER_NUMERIC_REL_TOL=obj.TRIGGER_NUMERIC_REL_TOL;
            status.TRIGGER_FLIGHT_PHASE_END_S=obj.TRIGGER_FLIGHT_PHASE_END_S;
            status.TRIGGER_UAV_RADIUS_M=obj.TRIGGER_UAV_RADIUS_M;
            status.TRIGGER_PAYLOAD_RADIUS_M=obj.TRIGGER_PAYLOAD_RADIUS_M;
            status.TRIGGER_CABLE_RADIUS_M=obj.TRIGGER_CABLE_RADIUS_M;
            status.trigger_count=obj.trigger_count;
            status.trigger_time_total_ms=obj.trigger_time_total_ms;
            status.trigger_time_max_ms=obj.trigger_time_max_ms;
            status.last_trigger_result=obj.last_trigger_result;
            status.trigger_native_gate_checked=obj.trigger_native_gate_checked;
            status.trigger_native_gate_status=obj.trigger_native_gate_status;
            status.trigger_flight_binding_created=~isempty(obj.trigger_authority_binding);
            status.trigger_continuous_certified=false;
            status.trigger_reference_cache=obj.trigger_reference_cache.get_status();
            status.takeoff_prime_candidate_epoch=obj.takeoff_prime_candidate_epoch;
            status.takeoff_prime_disabled=obj.takeoff_prime_disabled;
            status.takeoff_prime_error=obj.takeoff_prime_error;
            status.trigger_static_preparation=obj.trigger_core.static_preparation_status();
            status.first_flight_trace_enabled=obj.first_flight_trace_enabled;
            status.first_flight_traces=obj.first_flight_traces;
            status.preflight_model_binding_ready=obj.preflight_model_binding_ready;
            status.preflight_model_binding_ms=obj.preflight_model_binding_ms;
            status.preflight_model_binding_status=obj.preflight_model_binding_status;
            status.isolated_first_use_prepare_ms=obj.isolated_first_use_prepare_ms;
            status.isolated_first_use_prepare_status=obj.isolated_first_use_prepare_status;
            status.isolated_sensor_event_prepare_status= ...
                obj.isolated_sensor_event_prepare_status;
            status.isolated_sensor_event_prepare_steps= ...
                obj.isolated_sensor_event_prepare_steps;
            status.isolated_native_prediction_status=obj.isolated_native_prediction_status;
            status.isolated_native_prediction_calls=obj.isolated_native_prediction_calls;
            status.isolated_direct_native_status=obj.isolated_direct_native_status;
            status.isolated_direct_native_calls=obj.isolated_direct_native_calls;
            status.isolated_direct_native_verified_appends= ...
                obj.isolated_direct_native_verified_appends;
            status.isolated_direct_native_outer_appends= ...
                obj.isolated_direct_native_outer_appends;
            status.isolated_near_geometry_cases= ...
                obj.isolated_near_geometry_cases;
            status.isolated_near_geometry_status= ...
                obj.isolated_near_geometry_status;
            status.isolated_outer_authority_status= ...
                obj.isolated_outer_authority_status;
            status.isolated_outer_authority_calls= ...
                obj.isolated_outer_authority_calls;
            status.isolated_mcos_current_prepare_status= ...
                obj.isolated_mcos_current_prepare_status;
            status.isolated_mcos_current_prepare_calls= ...
                obj.isolated_mcos_current_prepare_calls;
            status.isolated_mcos_current_prepare_reason= ...
                obj.isolated_mcos_current_prepare_reason;
            status.isolated_f_runtime_prepare_status= ...
                obj.isolated_f_runtime_prepare_status;
            status.isolated_f_runtime_prepare_calls= ...
                obj.isolated_f_runtime_prepare_calls;
            status.isolated_f_runtime_prepare_reason= ...
                obj.isolated_f_runtime_prepare_reason;
            status.phase_shadow_frame_calls=obj.phase_shadow_frame_calls;
            status.phase_shadow_authority_calls=obj.phase_shadow_authority_calls;
            status.phase_shadow_skipped_mass=obj.phase_shadow_skipped_mass;
            status.phase_shadow_total_ms=obj.phase_shadow_total_ms;
            status.phase_shadow_max_ms=obj.phase_shadow_max_ms;
            status.phase_shadow_last_phase=obj.phase_shadow_last_phase;
            status.phase_shadow_last_status=obj.phase_shadow_last_status;
            status.phase_shadow_last_reason=obj.phase_shadow_last_reason;
            status.phase_shadow_geometry= ...
                obj.trigger_core.shadow_observation_preparation_status();
        end
    end

    methods (Static)
        function xd=flight_reference_from_nominal(nominal,tt,L)
            % Stable public source wrapper used by the exact same Flight
            % inner kernel and its disposable pre-arm MCOS fixture.
            xd=REPLANNING_SYSTEM.flight_reference_at(nominal,tt,L);
        end
        function xd=software_probe_flight_reference(nominal,tt,L)
            % Synthetic-only helper: execute the EXACT same mathematical
            % source wrapper as real Flight, without reading its reference.
            % Public only so the isolated pre-arm fixture can call it.
            xd=REPLANNING_SYSTEM.flight_reference_at(nominal,tt,L);
        end
    end

    methods (Static,Access=private)
        function time_s=require_sim_time(time)
            % SimのTIMEオブジェクト。テストでは同形のstructも可。
            if isobject(time)&&isscalar(time)&&isprop(time,'t')
                time_s=time.t;
            elseif isstruct(time)&&isscalar(time)&&isfield(time,'t')
                time_s=time.t;
            else
                error('REPLANNING_SYSTEM:TimeSource', ...
                    '現在のSim TIMEオブジェクト（time.t）を渡してください。');
            end
            REPLANNING_SYSTEM.check_time(time_s);
            time_s=double(time_s);
        end

        function xd=flight_reference_at(nominal,absolute_time_s,cable_length_m)
            % Actual TIME_VARYING_REFERENCE.func and its Flight epoch.
            % This is valid ONLY while the active phase remains 'f'.
            xd=nominal.func(double(absolute_time_s)-double(nominal.t));
            assert(isnumeric(xd)&&isreal(xd)&&iscolumn(xd) && ...
                numel(xd)>=4&&all(isfinite(xd)), ...
                'REPLANNING_SYSTEM:FlightReference', ...
                'Original nominal reference must supply a finite column jet.');
            xd=double(xd);
            xd(3)=xd(3)-double(cable_length_m);
        end

        function p=require_ekf_uav_position(agent)
            % 公称位置・plant真値・前tickへのフォールバックは禁止。
            if ~(isobject(agent)&&isscalar(agent)&&isprop(agent,'estimator'))
                error('REPLANNING_SYSTEM:Estimator', ...
                    'agent.estimatorがありません。');
            end
            output=agent.estimator.result;
            if ~(isstruct(output)&&isscalar(output)&&isfield(output,'state'))
                error('REPLANNING_SYSTEM:EKFUnavailable', ...
                    '現在tickのEKF推定状態stateが取得できません。');
            end
            state=output.state;
            if isobject(state)&&isscalar(state)&&isprop(state,'p')
                p=state.p;
            elseif isstruct(state)&&isscalar(state)&&isfield(state,'p')
                p=state.p;
            else
                error('REPLANNING_SYSTEM:EKFUnavailable', ...
                    'EKF状態に機体位置pがありません。');
            end
            if ~(isnumeric(p)&&isreal(p)&&numel(p)==3&&all(isfinite(p(:))))
                error('REPLANNING_SYSTEM:EKFInvalid', ...
                    'EKFの機体位置pは有限の3次元実ベクトルでなければなりません。');
            end
            p=double(p(:));
        end

        function check_time(t)
            if ~isnumeric(t)||~isscalar(t)||~isreal(t)||~isfinite(t)
                error('REPLANNING_SYSTEM:Time','時刻は有限の実数を指定してください。');
            end
        end
        function check_budget(value)
            assert(isnumeric(value)&&isscalar(value)&&isreal(value)&& ...
                (isinf(value)&&value>0 || isfinite(value)&&value>0), ...
                'REPLANNING_SYSTEM:ComputeBudget', ...
                'Computation budget must be positive milliseconds or Inf (unrestricted).');
        end

        function check_range(r)
            if ~isnumeric(r)||~isscalar(r)||~isreal(r)||~isfinite(r)||r<=0
                error('REPLANNING_SYSTEM:Range','検知半径は正の有限な実数 [m] です。');
            end
        end
        function check_nonnegative(v,name)
            if ~(isnumeric(v)&&isscalar(v)&&isreal(v)&&isfinite(v)&&v>=0)
                error('REPLANNING_SYSTEM:Nonnegative', ...
                    '%s must be a finite nonnegative physical value.',name);
            end
        end
        function check_flag(v,name)
            valid=islogical(v)&&isscalar(v);
            valid=valid || (isnumeric(v)&&isscalar(v)&&isreal(v) ...
                && isfinite(v)&&(v==0||v==1));
            if ~valid
                error('REPLANNING_SYSTEM:InvalidFlag', ...
                    '%s は true/false または 0/1 としてください。',name);
            end
        end
    end
end
