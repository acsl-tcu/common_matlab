classdef REPLANNING_SYSTEM_R5_DIAG < REPLANNING_SYSTEM
% AF7R58ZZY-R5-R3 diagnostic subclass only.
% Authority source SHA-256: 310cfdd01f9554a80b71e8d90464816a721859709ad7c4b9137cdc0f3a6eaf49
%
% IMPORTANT:
%   - This class inherits the exact production REPLANNING_SYSTEM authority.
%   - isa(obj,'REPLANNING_SYSTEM') therefore remains true for AF7R36/resource helpers.
%   - Only selected public methods are overridden to add tic/toc diagnostics.
%   - Trajectory mathematics, certificate acceptance, sensing geometry,
%     stale rejection, commit policy, and production REPLANNING_SYSTEM.m are unchanged.

    properties (Access = public)
        af7r58zzy_r5_capacity = 128;
        af7r58zzy_r5_width = 48;
        af7r58zzy_r5_ring = nan(128,48);
        af7r58zzy_r5_count = 0;
        af7r58zzy_r5_current_row = 0;
    end

    methods
        function obj = REPLANNING_SYSTEM_R5_DIAG(self,base_ref,opts)
            obj@REPLANNING_SYSTEM(self,base_ref,opts);
        end

        function result_out = do(obj,varargin)
            % =================================================================
            % 【メイン実行エントリーポイント: do】
            %   制御周期 (dt) ごとに呼び出され、衝突判定および状態出力を実行します。
            %
            % 【引数】
            %   varargin{1} : time (シミュレーション時間構造体またはオブジェクト: t, dt 等)
            %   varargin{2} : cha  (動作フェーズ文字: 'a'=Arming, 't'=Takeoff, 'f'=Flight 等)
            %   varargin{3} : state (オプション: 現在の状態量)
            %
            % 【スパイク防止処理フロー】
            %   1. 制御周期 dt の整合性チェック
            %   2. 牽引物質量推定値 mL の必須検証 (取得不可時は安全のため即座にエラー停止)
            %   3. ベース公称軌道 base_ref.do() の評価
            %   4. フェーズ判別:
            %      - 非保護フェーズ ('a', 't'):
            %        静止中 (Arming/Takeoff初期) に公称テーブル生成と JIT プライムを一括完遂。
            %        センサ・トリガー状態をゼロリセットし、公称指令値をパススルー出力。
            %      - 保護フェーズ ('f'):
            %        事前プライムの完了を厳格検証。
            %        機体位置 pQ を中心とする球状センサ (20m) で近接障害物をフィルタリング。
            %        公称テーブルから将来 H 区間の [pL, q] を O(1) スライス取得。
            %        C++ MEX カーネルを呼び出し、将来衝突判定を高速実行。
            %        結果を事前確保済み固定長フィールドへ直接上書き。
            % =================================================================
            time=varargin{1};
            cha=varargin{2};

            % AF7R58ZZY-R5: capture only the two pre-declared diagnostic windows.
            % This clone is never production authority; timing is empirical.
            r5t=double(time.t);
            r5cap=isequal(cha,'f') && ...
                ~obj.setup_exact_do_prime_active && ...
                ~obj.generator_integration_prime_active && ...
                ~obj.observer_preflight_prime_active && ...
                ((r5t>=4.95-1e-10 && r5t<=5.20+1e-10) || ...
                 (r5t>=9.80-1e-10 && r5t<=10.10+1e-10));
            r5row=0;
            r5do=[];
            r5req0=double(obj.generator_request_count);
            r5com0=double(obj.replan_commit_count);
            if r5cap
                obj.af7r58zzy_r5_count=obj.af7r58zzy_r5_count+1;
                assert(obj.af7r58zzy_r5_count<=obj.af7r58zzy_r5_capacity, ...
                    '[AF7R58ZZY-R5] diagnostic ring exhausted.');
                r5row=obj.af7r58zzy_r5_count;
                obj.af7r58zzy_r5_current_row=r5row;
                obj.af7r58zzy_r5_ring(r5row,:)=NaN;
                obj.af7r58zzy_r5_ring(r5row,1)=r5t;
                r5do=tic;
            else
                obj.af7r58zzy_r5_current_row=0;
            end

            % Single lightweight production wall timer for A/B diagnosis.
            % This remains active while the detailed multi-stage profiling
            % below is disabled.
            if obj.ENABLE_LIGHT_RUNTIME_AUDIT
                t_light_e2e=tic;
            end

            if isfield(time,'dt') && ~isempty(time.dt) && time.dt>0
                if abs(time.dt-obj.prediction_dt)>1e-12
                    obj.prediction_dt=time.dt;
                    ratio=obj.horizon_s/obj.prediction_dt;
                    assert(abs(ratio-round(ratio)) <= 1e-9*max(1,abs(ratio)), ...
                        '[REPLANNER ERROR] horizon_s/prediction_dt must be integer.');
                    obj.reference_cache_valid=false;
                    obj.nominal_table_valid=false;
                    obj.nominal_table_xd=[];
                    obj.online_prime_done=false;
                    obj.setup_exact_do_prime_done=false;
                end
            end

            if obj.ENABLE_LIGHT_RUNTIME_AUDIT
                cp_front_ms=1e3*toc(t_light_e2e);
            end

            if r5cap, r5x=tic; end
            obj.loadmass=obj.get_load_mass();
            if r5cap, obj.af7r58zzy_r5_ring(r5row,3)=1e3*toc(r5x); end

            if obj.ENABLE_LIGHT_RUNTIME_AUDIT
                cp_loadmass_ms=1e3*toc(t_light_e2e);
            end

            % -------------------------------------------------------------
            % AF7R7 observer hard-support add-on.
            % This REPLANNING_SYSTEM call occurs after sensor+EKF and before
            % controller+plant. Constructor/startup order is untouched.
            % Existing rt_light_e2e_ms automatically includes this cost.
            % -------------------------------------------------------------
            dt_observer_ms=NaN;
            if r5cap, r5x=tic; end
            if obj.ENABLE_OBSERVER_HARD_SUPPORT && ...
                    obj.observer_is_real_control_event(cha)
                if obj.ENABLE_LIGHT_RUNTIME_AUDIT || obj.ENABLE_PROFILING
                    t_observer_addon=tic;
                    obj.observer_update_same_event(time,cha);
                    dt_observer_ms=1e3*toc(t_observer_addon);
                    I=obj.observer_hard_support_internal;
                    I.runtime_last_ms=dt_observer_ms;
                    if ~isfield(I,'runtime_max_ms') || ~isfinite(I.runtime_max_ms)
                        I.runtime_max_ms=dt_observer_ms;
                    else
                        I.runtime_max_ms=max(I.runtime_max_ms,dt_observer_ms);
                    end
                    if ~isfield(I,'runtime_sum_ms') || ~isfinite(I.runtime_sum_ms)
                        I.runtime_sum_ms=0;
                    end
                    if ~isfield(I,'runtime_count') || ~isfinite(I.runtime_count)
                        I.runtime_count=0;
                    end
                    I.runtime_sum_ms=I.runtime_sum_ms+dt_observer_ms;
                    I.runtime_count=I.runtime_count+1;
                    obj.observer_hard_support_internal=I;
                else
                    % Production: preserve the exact observer update, omit only
                    % non-authoritative runtime measurement/statistics.
                    obj.observer_update_same_event(time,cha);
                end
            end
            if r5cap, obj.af7r58zzy_r5_ring(r5row,4)=1e3*toc(r5x); end

            % The replanner is constructed before the live HLC controller
            % object exists in run_flight_headless.  Command limits are
            % therefore source-bound in the constructor and the actual live
            % controller handle is verified lazily once it exists.  Flight
            % may not proceed without that live verification.
            if obj.ENABLE_PROFILING
                t_live_binding=tic;
            end
            if r5cap, r5x=tic; end
            obj.verify_live_controller_binding(isequal(cha,'f'));
            if r5cap, obj.af7r58zzy_r5_ring(r5row,5)=1e3*toc(r5x); end
            if obj.ENABLE_PROFILING
                obj.profiling.t_live_binding_ms=1e3*toc(t_live_binding);
                t_dependency=tic;
            end

            % Fast dependency check: scalar/string comparisons only.
            % No source-file parsing and no symbolic derivation is performed
            % in the Flight control cycle.
            if r5cap, r5x=tic; end
            obj.update_runtime_dependency_state_fast(cha);
            if r5cap, obj.af7r58zzy_r5_ring(r5row,6)=1e3*toc(r5x); end
            if obj.ENABLE_LIGHT_RUNTIME_AUDIT
                cp_dependency_ms=1e3*toc(t_light_e2e);
            end
            if obj.ENABLE_PROFILING
                obj.profiling.t_dependency_ms=1e3*toc(t_dependency);
            end

            if obj.ENABLE_PROFILING
                t_total=tic;
                t0=tic;
            end

            protected_phase=isequal(cha,'f');

            if protected_phase
                % NOMINAL current xd has already been evaluated exactly by
                % base_ref.do() during preflight table construction.
                % Reading it here removes repeated Flight-time reference
                % evaluation without changing the reference mathematics.
                if r5cap, r5x=tic; end
                xd_nom=obj.get_nominal_xd_from_table(time.t);
                if r5cap, obj.af7r58zzy_r5_ring(r5row,7)=1e3*toc(r5x); end
            else
                base_res=obj.base_ref.do(varargin{:});
                xd_nom=base_res.state.xd;
            end

            if obj.ENABLE_LIGHT_RUNTIME_AUDIT
                cp_base_ref_ms=1e3*toc(t_light_e2e);
            end

            if obj.ENABLE_PROFILING
                obj.profiling.t_base_ref_ms=1e3*toc(t0);
            end

            if ~protected_phase
                obj.clear_state_sensor_values(time.t);

                % AF7R58ZZU-R2 phase policy:
                %   ALL cold-path work must close in the framework's
                %   pre-mission initialization pass, before real Arming.
                %   Real Arming / Takeoff / Flight / Landing are all protected
                %   online phases and may not be used as a warm-up sink.
                %
                % The headless framework already performs an initialization
                % loop with self.cha='f' and fCha='0' before "Arming...".
                % is_flight_warmup_context() identifies exactly that pass.
                preflight_context=obj.is_flight_warmup_context();

                if preflight_context && (~obj.nominal_table_valid || ~obj.online_prime_done)
                    if ~obj.nominal_table_valid
                        obj.build_nominal_reference_table(time,varargin,xd_nom);
                    end
                    if obj.nominal_table_valid && ~obj.online_prime_done
                        obj.warmup_online_path(time,varargin,xd_nom);
                    end
                end

                % Never perform fallback warm-up in a real mission phase.
                % If framework initialization failed to close preflight, fail
                % BEFORE doing warm-up work in Arming/Takeoff/Landing.
                if isequal(cha,'a') || isequal(cha,'t') || isequal(cha,'l')
                    assert(obj.nominal_table_valid && obj.online_prime_done && ...
                        obj.setup_exact_do_prime_done && ...
                        obj.generator_backend_configuration_frozen && ...
                        obj.generator_integration_prime_done && ...
                        obj.generator_backend_kernel_prime_done && ...
                        obj.observer_preflight_prime_done && ...
                        obj.trigger_event_capture_prime_done, ...
                        ['[AF7R58ZZU-R2 PREFLIGHT] cold-path prime must finish ' ...
                         'before real Arming; Arming/Takeoff/Landing may not ' ...
                         'absorb warm-up spikes.']);
                end

                obj.write_nominal_output(xd_nom);
                result_out=obj.result;
                return;
            end

            if obj.REQUIRE_PRIME_BEFORE_TAKEOFF && ~obj.setup_exact_do_prime_active
                assert(obj.nominal_table_valid && obj.online_prime_done && ...
                    obj.setup_exact_do_prime_done && ...
                    obj.generator_backend_configuration_frozen && ...
                    obj.generator_integration_prime_done && ...
                    obj.generator_backend_kernel_prime_done && ...
                    obj.observer_preflight_prime_done && ...
                    obj.trigger_event_capture_prime_done, ...
                    ['[REPLANNER PREFLIGHT ERROR] Flight reached before pre-motion exact/generator prime completed. ' ...
                     'Prime must finish before vehicle motion.']);
                assert(~obj.setup_exact_force_hit, ...
                    '[REPLANNER ERROR] setup-only forced-hit flag leaked into Flight.');
            end

            if obj.ENABLE_LIGHT_RUNTIME_AUDIT
                cp_flight_guard_ms=1e3*toc(t_light_e2e);
            end

            % AF7R58ZP: all work below until t_hard_cycle is sensor-independent
            % preparation/authority validation.  The <=4 ms modification clock
            % starts at the first sensor-dependent operation (raw_sensor_gate),
            % matching the sensor -> trigger -> target-modification contract.
            % Full reference-node wall time remains measured externally and is
            % NOT hidden by this accounting boundary.
            if obj.ENABLE_PROFILING
                t_cert_guard=tic;
            end
            if r5cap, r5x=tic; end
            obj.assert_runtime_core_certificates_current_hot();
            if r5cap, obj.af7r58zzy_r5_ring(r5row,8)=1e3*toc(r5x); end
            if obj.ENABLE_LIGHT_RUNTIME_AUDIT
                cp_cert_guard_ms=1e3*toc(t_light_e2e);
            end

            if obj.ENABLE_PROFILING
                obj.profiling.t_cert_guard_ms=1e3*toc(t_cert_guard);
                t_geometry_eval=tic;
            end

            active_version_at_eval=double(obj.active_reference_version);
            active_mode_at_eval=char(obj.active_reference_mode);
            if r5cap, r5x=tic; end
            xd_active_now=obj.evaluate_active_reference_xd(time.t,xd_nom);
            if r5cap, obj.af7r58zzy_r5_ring(r5row,9)=1e3*toc(r5x); end

            if obj.ENABLE_LIGHT_RUNTIME_AUDIT
                cp_active_ref_ms=1e3*toc(t_light_e2e);
            end
            if r5cap, r5x=tic; end
            [pQ_cur,pL_cur,q_cur,cable_residual]=obj.get_current_physical_geometry(xd_active_now);
            if r5cap, obj.af7r58zzy_r5_ring(r5row,10)=1e3*toc(r5x); end

            if obj.ENABLE_LIGHT_RUNTIME_AUDIT
                cp_geometry_ms=1e3*toc(t_light_e2e);
                cp_base_cert_geom_ms=cp_geometry_ms;
            end

            if obj.ENABLE_PROFILING
                obj.profiling.t_geometry_eval_ms=1e3*toc(t_geometry_eval);
                tr=tic;
            end

            % Sensor-independent future-reference packet preparation.
            % It may update only the rolling reference cache; it does not read
            % or mutate the current obstacle-detection result.
            if r5cap, r5x=tic; end
            [pL_ref,q_ref,ref_source]= ...
                obj.get_active_reference_packet(time,cha,varargin,xd_nom);
            if r5cap, obj.af7r58zzy_r5_ring(r5row,11)=1e3*toc(r5x); end

            if obj.ENABLE_PROFILING
                obj.profiling.t_reference_packet_ms=1e3*toc(tr);
                ts=tic;
            end

            % Sensor-dependent synchronous attempt accounting begins here.
            % AF7R58ZZQ normal planning budget = one control period; the
            % collision-prediction horizon remains fixed at H=4 s.
            t_hard_cycle=tic;

            if r5cap, r5x=tic; end
            [sensor_active,min_dist,min_id,all_dists]= ...
                obj.raw_sensor_gate(pQ_cur,obj.static_obstacle_map);
            if r5cap, obj.af7r58zzy_r5_ring(r5row,12)=1e3*toc(r5x); end

            % Initialization and synthetic Flight-prime calls may exercise
            % the sensor branch but are forbidden from teaching the simulated
            % replanner any obstacle. Real memory begins only at the bound
            % Flight time origin.
            tf_mem=double(obj.flight_reference_time_origin);
            real_sensor_memory_event = ~obj.setup_exact_do_prime_active && ...
                ~obj.generator_integration_prime_active && ...
                isfinite(tf_mem) && ...
                double(time.t)>=tf_mem-1e-10*max(1,abs(tf_mem));
            if r5cap, r5x=tic; end
            active=obj.apply_simulated_sensor_memory( ...
                sensor_active,double(time.t),real_sensor_memory_event);
            if r5cap, obj.af7r58zzy_r5_ring(r5row,13)=1e3*toc(r5x); end

            if obj.ENABLE_PROFILING
                obj.profiling.t_sensor_ms=1e3*toc(ts);
                tt=tic;
            end

            vis_count=nnz(sensor_active);
            known_count=nnz(active);

            N=size(pL_ref,2);
            effective_horizon=(N-1)*obj.prediction_dt;

            pipeline_latched = obj.ENABLE_STAGED_REPLAN_PIPELINE && ...
                ~strcmp(char(obj.staged_replan_state),'IDLE') && ...
                isfinite(double(obj.staged_replan_collision_time)) && ...
                double(obj.staged_replan_collision_time)>double(time.t);

            if r5cap, r5x=tic; end
            if pipeline_latched
                % AF7R58ZZV:
                % The ordinary H=4 s trigger already established this static-
                % obstacle collision request on Stage 1.  During the following
                % two pipeline cycles, the active reference authority is
                % unchanged.  Re-running the full future-trigger kernel cannot
                % strengthen that already-latched request.
                %
                % Freshness is NOT skipped: raw sensing + sensor memory above
                % still execute every cycle.  Any new known obstacle changes
                % the active mask and is rejected by the staged freshness gate.
                need_replan=true;
                collision_time=double(obj.staged_replan_collision_time);
                ttc=max(0,collision_time-double(time.t));
                obs_id=double(obj.staged_replan_obstacle_id);
                comp_code=double(obj.staged_replan_component_code);
                comp_name=char(obj.staged_replan_component_name);
                pL_collision=double(obj.staged_replan_pL_collision(:));
                q_collision=double(obj.staged_replan_q_collision(:));
                pQ_collision=double(obj.staged_replan_pQ_collision(:));
                jhit=double(obj.staged_replan_hit_index);
                kernel_called=false;
                kernel_ms=0;
                obj.staged_replan_trigger_kernel_skip_count= ...
                    obj.staged_replan_trigger_kernel_skip_count+1;
            else
                [need_replan,ttc,collision_time,comp_name, ...
                    pL_collision,q_collision,pQ_collision,jhit, ...
                    obs_id,comp_code,kernel_called,kernel_ms] = ...
                    obj.trigger_core(active,pL_ref,q_ref,time.t);
            end
            if r5cap, obj.af7r58zzy_r5_ring(r5row,14)=1e3*toc(r5x); end

            if obj.setup_exact_do_prime_active && obj.setup_exact_force_hit
                force_idx=max(1,obj.setup_exact_force_hit_calls);
                if N>=2
                    j_force=1+mod(force_idx-1,2);   % current/future-hit downstream paths
                else
                    j_force=1;
                end
                obs_force=obj.obstacle_packet.ids(1);
                comp_force=1+mod(force_idx-1,3);   % PAYLOAD/UAV/CABLE result paths
                [need_replan,ttc,collision_time,comp_name, ...
                    pL_collision,q_collision,pQ_collision,jhit] = ...
                    obj.decode_trigger_result( ...
                        true,j_force,obs_force,comp_force,N,pL_ref,q_ref,time.t);
                obs_id=obs_force;
                comp_code=comp_force;
                kernel_called=true;
            end

            if obj.ENABLE_PROFILING
                obj.profiling.t_kernel_ms=kernel_ms;
                obj.profiling.t_trigger_ms=1e3*toc(tt);
            end

            if obj.ENABLE_PROFILING
                t_replan_status=tic;
            end
            % Replan generator is invoked only on a REAL Flight request edge.
            % Setup/warmup forced-hit paths never invoke the external generator.
            if r5cap, r5x=tic; end
            if need_replan
                % ---------------------------------------------------------
                % Request identity, not only a boolean rising edge.
                %
                % A repaired-reference commit increments reference_version.
                % Therefore, if the new reference still predicts a collision
                % on the next cycle, it is a NEW request and is allowed to
                % replan immediately.  Likewise, a different obstacle or
                % protected component is a new request even while the
                % previous need_replan flag remains true.
                % ---------------------------------------------------------
                requestVersion=double(obj.active_reference_version);

                sameIdentity = ...
                    obj.replan_request_active && ...
                    double(obj.replan_request_reference_version)==requestVersion && ...
                    isequaln(double(obj.replan_request_obstacle_id),double(obs_id)) && ...
                    isequaln(double(obj.replan_request_component_code),double(comp_code));

                % A hard-deadline miss is not a completed planning decision.
                % The same request remains retry-eligible while positive
                % physical lead remains. Each individual synchronous MATLAB
                % attempt must stay within one control period. The ordinary
                % collision horizon itself remains fixed at H=4 s.
                retryableDeadlineMiss = sameIdentity && ...
                    any(double(obj.generator_last_status_code)==[-2 -20 -30]) && ...
                    isfinite(double(ttc)) && ...
                    double(ttc)>double(obj.prediction_dt);

                requestIdentityChanged = ~sameIdentity || retryableDeadlineMiss;

                if requestIdentityChanged

                    % AF7R43: replay-event materialization is diagnostic.
                    % Preserve PRE-COMMIT authorities by copy-on-write now,
                    % but build the large event only after the hard
                    % generator/commit decision.
                    deferTriggerCapture = obj.CAPTURE_REAL_TRIGGER_EVENT && ...
                        ~retryableDeadlineMiss && ...
                        ~obj.setup_exact_do_prime_active && ...
                        ~obj.generator_integration_prime_active;
                    triggerAuthoritySnapshot=struct();
                    if deferTriggerCapture
                        triggerAuthoritySnapshot=struct( ...
                            'packet',obj.obstacle_packet, ...
                            'dependency_signature',obj.runtime_dependency_signature, ...
                            'speed_certificate',obj.active_speed_certificate, ...
                            'sensor_horizon_certificate',obj.sensor_horizon_certificate, ...
                            'reference_mode',char(obj.active_reference_mode), ...
                            'reference_version',double(obj.active_reference_version), ...
                            'flight_reference_time_origin',double(obj.flight_reference_time_origin));
                    end

                    if ~obj.setup_exact_do_prime_active && obj.generator_shadow_enabled
                        if obj.ENABLE_STAGED_REPLAN_PIPELINE
                            obj.start_staged_replan_pipeline( ...
                                t_hard_cycle,time,cha,varargin, ...
                                active,all_dists,ttc,collision_time, ...
                                obs_id,comp_name,comp_code, ...
                                pL_collision,q_collision,pQ_collision,jhit);
                        else
                            obj.invoke_replan_generator_shadow( ...
                                t_hard_cycle,time.t,xd_nom,xd_active_now, ...
                                pQ_cur,pL_cur,q_cur,cable_residual, ...
                                active,all_dists,pL_ref,q_ref,ref_source, ...
                                ttc,collision_time,obs_id,comp_name,comp_code, ...
                                pL_collision,q_collision,pQ_collision,jhit);
                        end
                    else
                        obj.last_replan_status=struct( ...
                            'requested',true, ...
                            'committed',false, ...
                            'reason','GENERATOR_SKIPPED_DURING_PRIME_OR_DISABLED', ...
                            'request_time',double(time.t), ...
                            'trigger_ttc',double(ttc), ...
                            'trigger_obstacle_id',double(obs_id), ...
                            'trigger_component',char(comp_name));
                    end

                    if deferTriggerCapture
                        tevent=tic;
                        obj.latch_real_trigger_event( ...
                            time.t,xd_nom,xd_active_now, ...
                            pQ_cur,pL_cur,q_cur,cable_residual, ...
                            active,all_dists,ref_source, ...
                            ttc,collision_time,obs_id,comp_name,comp_code, ...
                            pL_collision,q_collision,pQ_collision,jhit, ...
                            effective_horizon,N,triggerAuthoritySnapshot);
                        obj.trigger_event_capture_ms=1e3*toc(tevent);
                        obj.last_trigger_event.capture_runtime_ms= ...
                            double(obj.trigger_event_capture_ms);
                        if obj.trigger_event_count==1
                            obj.first_trigger_event.capture_runtime_ms= ...
                                double(obj.trigger_event_capture_ms);
                        end
                    end

                    % Store the VERSION THAT WAS REQUESTED.  If a commit
                    % occurred inside the generator, active_reference_version
                    % has already changed, so next cycle is intentionally a
                    % new identity and can replan again if still necessary.
                    obj.replan_request_active=true;
                    obj.replan_request_reference_version=requestVersion;
                    obj.replan_request_obstacle_id=double(obs_id);
                    obj.replan_request_component_code=double(comp_code);
                end
            else
                if obj.ENABLE_STAGED_REPLAN_PIPELINE && ...
                        ~strcmp(char(obj.staged_replan_state),'IDLE')
                    obj.reset_staged_replan_pipeline( ...
                        'TRIGGER_CLEARED',false);
                end
                obj.replan_request_active=false;
                obj.replan_request_reference_version=-1;
                obj.replan_request_obstacle_id=NaN;
                obj.replan_request_component_code=NaN;
            end
            if r5cap, obj.af7r58zzy_r5_ring(r5row,17)=1e3*toc(r5x); end

            if r5cap, r5x=tic; end
            if obj.ENABLE_STAGED_REPLAN_PIPELINE && ...
                    ~obj.setup_exact_do_prime_active
                obj.advance_staged_replan_pipeline( ...
                    t_hard_cycle,time.t,active,need_replan,obs_id,comp_code);
            end
            if r5cap, obj.af7r58zzy_r5_ring(r5row,23)=1e3*toc(r5x); end

            if obj.ENABLE_PROFILING
                obj.profiling.t_replan_status_ms=1e3*toc(t_replan_status);
                t_output_write=tic;
            end

            if obj.ENABLE_LIGHT_RUNTIME_AUDIT
                cp_sensor_trigger_ms=1e3*toc(t_light_e2e);
            end

            if r5cap, r5out=tic; end
            st=obj.result.state;

            if obj.ENABLE_LIGHT_RUNTIME_AUDIT
                cp_state_handle_ms=1e3*toc(t_light_e2e);
            end

            if double(obj.active_reference_version)==active_version_at_eval && ...
                    strcmp(char(obj.active_reference_mode),active_mode_at_eval)
                % Exact same active authority as the pre-sensor evaluation.
                obj.write_active_output(time.t,xd_nom,xd_active_now);
            else
                % A same-cycle commit changed the authority. Preserve the
                % legacy behavior by evaluating the newly committed reference.
                obj.write_active_output(time.t,xd_nom);
            end

            if obj.ENABLE_LIGHT_RUNTIME_AUDIT
                cp_write_active_output_ms=1e3*toc(t_light_e2e);
            end

            if obj.ENABLE_PROFILING
                obj.profiling.t_output_write_ms=1e3*toc(t_output_write);
                t_state_trigger_write=tic;
            end

            st.time=time.t;
            st.pQ=pQ_cur;
            st.detected_point=vis_count>0;

            % Variable-size detected/relevant structs are compatibility
            % placeholders only. Fixed scalar/vector fields below are the
            % production LOGGER interface.
            st.min_obstacle_id_point=min_id;
            st.min_dist_point=min_dist;
            st.detected_obstacle_count_point=vis_count;
            st.sensor_current_active_mask=double(sensor_active);
            st.sensor_known_active_mask=double(active);
            st.sensor_known_obstacle_count=double(known_count);
            st.sensor_memory_release_count=double(obj.simulated_sensor_memory_release_count);

            if obj.ENABLE_LIGHT_RUNTIME_AUDIT
                cp_state_basic_ms=1e3*toc(t_light_e2e);
            end

            % Fixed scalar trigger fields first.
            st.need_replan=need_replan;
            st.trigger_ttc=ttc;
            st.trigger_collision_time=collision_time;
            st.trigger_obstacle_id=obs_id;
            st.trigger_component_code=comp_code;
            st.trigger_effective_horizon=effective_horizon;
            st.trigger_reference_points=N;
            st.trigger_cable_residual=cable_residual;
            st.trigger_hit_index=jhit;
            st.trigger_kernel_called=double(kernel_called);

            if obj.ENABLE_LIGHT_RUNTIME_AUDIT
                cp_state_trigger_scalar_ms=1e3*toc(t_light_e2e);
            end

            % Small vectors / strings / mask-distance arrays are separated
            % because these assignments can create/copy MATLAB data.
            st.trigger_component=comp_name;
            st.trigger_pL_collision=pL_collision;
            st.trigger_q_collision=q_collision;
            st.trigger_pQ_collision=pQ_collision;
            st.trigger_reference_source=ref_source;
            st.trigger_sensor_origin=pQ_cur;
            st.trigger_active_mask=double(active);
            st.trigger_raw_distances=all_dists;

            if obj.ENABLE_LIGHT_RUNTIME_AUDIT
                cp_state_trigger_heavy_ms=1e3*toc(t_light_e2e);
            end

            if obj.ENABLE_PROFILING
                obj.profiling.t_state_trigger_write_ms=1e3*toc(t_state_trigger_write);
                t_state_cert_write=tic;
            end

            st.active_reference_mode=obj.active_reference_mode;
            st.active_reference_version=obj.active_reference_version;
            st.nominal_xd_table_used=protected_phase;

            % Full certificate structs remain authoritative in obj.*.
            % The fail-closed certificate guard above is unchanged.
            % LOGGER receives fixed-size summaries only.
            st.certificate_guard_passed=true;
            st.certificate_stale_mask = ...
                double(obj.certificate_stale.SPEED_TRIGGER_CERT) + ...
                2*double(obj.certificate_stale.SENSOR_HORIZON_CERT) + ...
                4*double(obj.certificate_stale.STOP_GEOMETRY_4MS_CERT) + ...
                8*double(obj.certificate_stale.ACTUATOR_DYNAMICS_CERT);
            st.hlc_optional_stale=logical(obj.certificate_stale.HLC_ROBUST_CERT);
            if isfield(obj.command_limit_certificate,'live_controller_verified')
                st.command_limit_live_verified= ...
                    logical(obj.command_limit_certificate.live_controller_verified);
            else
                st.command_limit_live_verified=false;
            end
            st.dependency_refresh_count=obj.dependency_refresh_count;
            st.mass_certificate_rebuild_count=obj.mass_certificate_rebuild_count;

            if obj.ENABLE_LIGHT_RUNTIME_AUDIT
                cp_state_cert_ms=1e3*toc(t_light_e2e);
            end

            if obj.ENABLE_PROFILING
                obj.profiling.t_state_cert_write_ms=1e3*toc(t_state_cert_write);
                t_state_meta_write=tic;
            end

            st.loadmass=obj.loadmass;
            st.replan_commit_count=obj.replan_commit_count;
            st.last_replan_commit_time=obj.last_replan_commit_time;
            % Keep only the small scalar generator status interface live.
            st.generator_request_count=obj.generator_request_count;
            st.generator_candidate_count=obj.generator_candidate_count;
            st.generator_validation_pass_count=obj.generator_validation_pass_count;
            st.generator_validation_fail_count=obj.generator_validation_fail_count;
            st.generator_deadline_fail_count=obj.generator_deadline_fail_count;
            st.generator_last_runtime_ms=obj.generator_last_runtime_ms;
            st.generator_last_status_code=obj.generator_last_status_code;
            st.generator_last_cycle_critical_ms=obj.generator_last_cycle_critical_ms;
            if obj.ENABLE_HOTPATH_DIAGNOSTIC_MIRROR
                st.generator_shadow_enabled=logical(obj.generator_shadow_enabled);
                st.generator_callback_installed=~isempty(obj.repaired_candidate_generator_fn);
                [flagCode,~,~]=obj.normalize_generator_backend_flag(obj.generator_backend_flag);
                st.generator_backend_flag=double(flagCode);
                st.generator_backend_last_code=double(obj.generator_backend_last_code);
                st.generator_backend_last_source_code=double(obj.generator_backend_last_source_code);
                st.generator_backend_last_callback_installed= ...
                    logical(obj.generator_backend_last_callback_installed);
                st.generator_backend_locked_code=double(obj.generator_backend_locked_code);
                st.generator_backend_selection_locked=logical(obj.generator_backend_selection_locked);
                st.generator_backend_configuration_frozen= ...
                    logical(obj.generator_backend_configuration_frozen);
                st.generator_integration_prime_done=logical(obj.generator_integration_prime_done);
                st.generator_integration_prime_ms=obj.generator_integration_prime_ms;
                st.generator_integration_prime_variants=obj.generator_integration_prime_variants;
                st.generator_context_detected_only=logical(obj.generator_context_detected_only);
                st.generator_backend_kernel_prime_done=logical(obj.generator_backend_kernel_prime_done);
                st.generator_backend_kernel_prime_ms=obj.generator_backend_kernel_prime_ms;
                st.generator_backend_kernel_prime_variants=obj.generator_backend_kernel_prime_variants;
                st.generator_shadow_proposal_count=obj.generator_shadow_proposal_count;
                st.bezier_shadow_generated_count=obj.bezier_shadow_generated_count;
                st.bezier_shadow_peak_safe_count=obj.bezier_shadow_peak_safe_count;
                st.bezier_shadow_last_reason_code=obj.bezier_shadow_last_reason_code;
                st.bezier_shadow_last_t0=obj.bezier_shadow_last_t0;
                st.bezier_shadow_last_tm=obj.bezier_shadow_last_tm;
                st.bezier_shadow_last_t1=obj.bezier_shadow_last_t1;
                st.bezier_shadow_last_obstacle_id=obj.bezier_shadow_last_obstacle_id;
                st.bezier_shadow_last_component_code=obj.bezier_shadow_last_component_code;
                st.bezier_shadow_last_displacement_m=obj.bezier_shadow_last_displacement_m;
                st.bezier_shadow_last_metric_before=obj.bezier_shadow_last_metric_before;
                st.bezier_shadow_last_metric_after=obj.bezier_shadow_last_metric_after;
                st.bezier_shadow_last_peak_min_metric=obj.bezier_shadow_last_peak_min_metric;
                st.bezier_shadow_last_peak_all_safe=logical(obj.bezier_shadow_last_peak_all_safe);
                st.bezier_shadow_last_c7_exact=logical(obj.bezier_shadow_last_c7_exact);
                st.bezier_shadow_last_d1_ub=obj.bezier_shadow_last_d1_ub;
                st.bezier_shadow_last_d2_ub=obj.bezier_shadow_last_d2_ub;
                st.bezier_shadow_last_d3_ub=obj.bezier_shadow_last_d3_ub;
                st.bezier_shadow_last_d4_ub=obj.bezier_shadow_last_d4_ub;
                st.bezier_shadow_last_d5_ub=obj.bezier_shadow_last_d5_ub;
                st.bezier_shadow_last_d6_ub=obj.bezier_shadow_last_d6_ub;
                st.bezier_shadow_last_dx=obj.bezier_shadow_last_dx;
                st.bezier_shadow_last_dy=obj.bezier_shadow_last_dy;
                st.bezier_shadow_last_dz=obj.bezier_shadow_last_dz;
                st.r9_stage_episode_ms=obj.r9_stage_episode_ms;
                st.r9_stage_synthesis_ms=obj.r9_stage_synthesis_ms;
                st.r9_stage_candidate_loop_ms=obj.r9_stage_candidate_loop_ms;
                st.r9_stage_total_internal_ms=obj.r9_stage_total_internal_ms;
                st.r9_synth_hessian_ms=obj.r9_synth_hessian_ms;
                st.r9_synth_corridor_ms=obj.r9_synth_corridor_ms;
                st.r9_synth_qp_ms=obj.r9_synth_qp_ms;
                st.r9_synth_candidate_build_ms=obj.r9_synth_candidate_build_ms;
                st.r9_synth_row_count=obj.r9_synth_row_count;
                st.r9_synth_episode_count=obj.r9_synth_episode_count;
                st.r9_synth_component_count=obj.r9_synth_component_count;
                st.r9_synth_sign_count=obj.r9_synth_sign_count;
                st.r9_hr_request_seq=obj.r9_hr_request_seq;
                st.r9_hr_callback_wall_ms=obj.r9_hr_callback_wall_ms;
                st.r9_hr_callback_thread_cycles=obj.r9_hr_callback_thread_cycles;
                st.r9_hr_callback_process_cycles=obj.r9_hr_callback_process_cycles;
                st.r9_hr_episode_wall_ms=obj.r9_hr_episode_wall_ms;
                st.r9_hr_episode_thread_cycles=obj.r9_hr_episode_thread_cycles;
                st.r9_hr_episode_process_cycles=obj.r9_hr_episode_process_cycles;
                st.r9_hr_synthesis_wall_ms=obj.r9_hr_synthesis_wall_ms;
                st.r9_hr_synthesis_thread_cycles=obj.r9_hr_synthesis_thread_cycles;
                st.r9_hr_synthesis_process_cycles=obj.r9_hr_synthesis_process_cycles;
                st.r9_hr_qp_wall_ms=obj.r9_hr_qp_wall_ms;
                st.r9_hr_qp_thread_cycles=obj.r9_hr_qp_thread_cycles;
                st.r9_hr_qp_process_cycles=obj.r9_hr_qp_process_cycles;
                st.geometry_shadow_pass=logical(obj.geometry_shadow_pass);
                st.geometry_shadow_reason_code=obj.geometry_shadow_reason_code;
                st.geometry_shadow_continuous_time=logical(obj.geometry_shadow_continuous_time);
                st.geometry_shadow_ieee754_outward_closed=logical(obj.geometry_shadow_ieee754_outward_closed);
                st.geometry_shadow_min_margin_m=obj.geometry_shadow_min_margin_m;
                st.geometry_shadow_min_payload_margin_m=obj.geometry_shadow_min_payload_margin_m;
                st.geometry_shadow_min_uav_margin_m=obj.geometry_shadow_min_uav_margin_m;
                st.geometry_shadow_min_cable_margin_m=obj.geometry_shadow_min_cable_margin_m;
                st.geometry_shadow_leaf_count=obj.geometry_shadow_leaf_count;
                st.geometry_shadow_split_count=obj.geometry_shadow_split_count;
                st.geometry_shadow_max_depth=obj.geometry_shadow_max_depth;
                st.geometry_shadow_min_alpha_lb=obj.geometry_shadow_min_alpha_lb;
                st.geometry_shadow_max_qdot_ub=obj.geometry_shadow_max_qdot_ub;
                st.geometry_shadow_max_q_arc_radius=obj.geometry_shadow_max_q_arc_radius;
                st.geometry_shadow_max_q_arc_inflation_m=obj.geometry_shadow_max_q_arc_inflation_m;
                st.geometry_shadow_resource_exhausted=logical(obj.geometry_shadow_resource_exhausted);
                st.generator_last_elapsed_before_ms=obj.generator_last_elapsed_before_ms;
                st.generator_last_budget_remaining_ms=obj.generator_last_budget_remaining_ms;
                st.generator_last_context_ms=obj.generator_last_context_ms;
                st.generator_last_callback_ms=obj.generator_last_callback_ms;
                st.generator_last_validation_ms=obj.generator_last_validation_ms;
                st.generator_last_critical_ms=obj.generator_last_critical_ms;
            end

            if obj.ENABLE_LIGHT_RUNTIME_AUDIT
                cp_state_meta_ms=1e3*toc(t_light_e2e);
            end

            if obj.ENABLE_PROFILING
                obj.profiling.t_state_meta_write_ms=1e3*toc(t_state_meta_write);
                obj.profiling.t_total_ms=1e3*toc(t_total);

                % LOGGER-safe scalar timing mirror.
                % Diagnostic only: these values do not feed any safety or
                % replanning decision.
                st.rt_live_binding_ms=obj.profiling.t_live_binding_ms;
                st.rt_dependency_ms=obj.profiling.t_dependency_ms;
                st.rt_cert_guard_ms=obj.profiling.t_cert_guard_ms;
                st.rt_geometry_eval_ms=obj.profiling.t_geometry_eval_ms;
                st.rt_replan_status_ms=obj.profiling.t_replan_status_ms;
                st.rt_output_write_ms=obj.profiling.t_output_write_ms;
                st.rt_state_trigger_write_ms=obj.profiling.t_state_trigger_write_ms;
                st.rt_state_cert_write_ms=obj.profiling.t_state_cert_write_ms;
                st.rt_state_meta_write_ms=obj.profiling.t_state_meta_write_ms;
                st.rt_base_ref_ms=obj.profiling.t_base_ref_ms;
                st.rt_sensor_ms=obj.profiling.t_sensor_ms;
                st.rt_reference_packet_ms=obj.profiling.t_reference_packet_ms;
                st.rt_kernel_ms=obj.profiling.t_kernel_ms;
                st.rt_trigger_ms=obj.profiling.t_trigger_ms;
                st.rt_total_ms=obj.profiling.t_total_ms;

                % Nested profiling is intentionally not attached to result.state.
                % Direct rt_* scalars are the runtime diagnostic interface.
            end

            if obj.ENABLE_LIGHT_RUNTIME_AUDIT
                cp_output_state_ms=1e3*toc(t_light_e2e);
                st.rt_light_e2e_ms=cp_output_state_ms;
                st.rt_observer_ms=dt_observer_ms;

                % Capture only rare slow cycles. The production total above is
                % measured before these diagnostic writes.
                if cp_output_state_ms > obj.SPIKE_CAPTURE_THRESHOLD_MS
                    st.rt_spike_capture_time=double(time.t);
                    st.rt_spike_total_ms=cp_output_state_ms;
                    st.rt_spike_stage_front_ms=cp_front_ms;
                    st.rt_spike_stage_loadmass_ms=cp_loadmass_ms-cp_front_ms;
                    st.rt_spike_stage_dependency_ms=cp_dependency_ms-cp_loadmass_ms;
                    st.rt_spike_stage_base_cert_geom_ms= ...
                        cp_base_cert_geom_ms-cp_dependency_ms;
                    st.rt_spike_stage_sensor_trigger_ms= ...
                        cp_sensor_trigger_ms-cp_base_cert_geom_ms;
                    st.rt_spike_stage_output_state_ms= ...
                        cp_output_state_ms-cp_sensor_trigger_ms;

                    % Fine split of BASE_CERT_GEOM.
                    st.rt_spike_base_ref_ms= ...
                        cp_base_ref_ms-cp_dependency_ms;
                    st.rt_spike_flight_guard_ms= ...
                        cp_flight_guard_ms-cp_base_ref_ms;
                    st.rt_spike_cert_guard_ms= ...
                        cp_cert_guard_ms-cp_flight_guard_ms;
                    st.rt_spike_active_ref_ms= ...
                        cp_active_ref_ms-cp_cert_guard_ms;
                    st.rt_spike_geometry_ms= ...
                        cp_geometry_ms-cp_active_ref_ms;

                    % Fine split of OUTPUT_STATE.
                    st.rt_spike_state_handle_ms= ...
                        cp_state_handle_ms-cp_sensor_trigger_ms;
                    st.rt_spike_write_active_output_ms= ...
                        cp_write_active_output_ms-cp_state_handle_ms;
                    st.rt_spike_state_basic_ms= ...
                        cp_state_basic_ms-cp_write_active_output_ms;
                    st.rt_spike_state_trigger_scalar_ms= ...
                        cp_state_trigger_scalar_ms-cp_state_basic_ms;
                    st.rt_spike_state_trigger_heavy_ms= ...
                        cp_state_trigger_heavy_ms-cp_state_trigger_scalar_ms;
                    st.rt_spike_state_cert_ms= ...
                        cp_state_cert_ms-cp_state_trigger_heavy_ms;
                    st.rt_spike_state_meta_ms= ...
                        cp_state_meta_ms-cp_state_cert_ms;

                    st.rt_spike_active_count=double(vis_count);
                    st.rt_spike_need_replan=double(need_replan);
                end
            end

            if r5cap
                obj.af7r58zzy_r5_ring(r5row,42)=1e3*toc(r5out);
                obj.af7r58zzy_r5_ring(r5row,2)=1e3*toc(r5do);
                obj.af7r58zzy_r5_ring(r5row,44)= ...
                    double(obj.generator_request_count)-r5req0;
                obj.af7r58zzy_r5_ring(r5row,45)= ...
                    double(obj.replan_commit_count)-r5com0;
                obj.af7r58zzy_r5_ring(r5row,46)=double(need_replan);
                obj.af7r58zzy_r5_ring(r5row,47)=double(known_count);
                obj.af7r58zzy_r5_ring(r5row,48)=double(obj.active_reference_version);
            end

            if obj.ENABLE_ALL_LOG
                if obj.LOG_SENSOR && vis_count>0
                    fprintf('[SENSOR RAW %.1fm] t=%.3f | count=%d | nearest=%g | d=%.3f m\n', ...
                        obj.trigger_dist,time.t,vis_count,min_id,min_dist);
                end
                if obj.LOG_TRIGGER && need_replan
                    fprintf('[FUTURE HIT] t=%.3f | TTC=%.3f | obs=%g | comp=%s\n', ...
                        time.t,ttc,obs_id,comp_name);
                end
            end

            result_out=obj.result;
        end


        function start_staged_replan_pipeline(obj,t_hard_cycle,time,cha,varargin_in, ...
                active,all_dists,ttc,collision_time,obs_id,comp_name,comp_code, ...
                pL_collision,q_collision,pQ_collision,jhit)
            if ~obj.ENABLE_STAGED_REPLAN_PIPELINE
                return;
            end

            assert(strcmpi(char(obj.AVOIDANCE_COMMIT_POLICY),'REFERENCE_ONLY'), ...
                ['[AF7R58ZZU] staged pipeline currently supports ' ...
                 'REFERENCE_ONLY commit only.']);
            assert(double(obj.generator_backend_locked_code)==1, ...
                '[AF7R58ZZU] staged pipeline requires current native BEZIER backend.');
            assert(isequal(double(obj.horizon_s),4.0), ...
                '[AF7R58ZZU] fixed 4-s collision horizon changed.');

            if ~strcmp(char(obj.staged_replan_state),'IDLE')
                obj.staged_replan_stale_reject_count= ...
                    obj.staged_replan_stale_reject_count+1;
                obj.reset_staged_replan_pipeline( ...
                    'REQUEST_IDENTITY_CHANGED',false);
            end

            t_now=double(time.t);
            dt=double(obj.prediction_dt);
            ta=t_now+2*dt; % exactly two compute-pipeline intervals
            if ~(isfinite(ta)&&isfinite(double(collision_time))&& ...
                    double(collision_time)>ta)
                obj.staged_replan_fail_count=obj.staged_replan_fail_count+1;
                obj.generator_validation_fail_count= ...
                    obj.generator_validation_fail_count+1;
                obj.generator_last_status_code=-60;
                obj.staged_replan_last_reason= ...
                    'INSUFFICIENT_LEAD_FOR_TWO_PERIOD_PIPELINE';
                return;
            end

            ts=tic;
            r5row=obj.af7r58zzy_r5_current_row;
            if r5row>0, r5x=tic; end
            [pL_future,q_future,ref_source,xd_nom_a,xd_active_a]= ...
                obj.staged_reference_packet_at(ta,varargin_in);
            if r5row>0, obj.af7r58zzy_r5_ring(r5row,19)=1e3*toc(r5x); end

            % For the locked AF7R36 packet, the authoritative geometry comes
            % from the active-reference batch model and obstacle packet.
            % Provide a coherent reference-boundary geometry in the compact
            % request rather than pretending a future estimator sample exists.
            pL_a=double(pL_future(:,1));
            q_a=double(q_future(:,1));
            pQ_a=pL_a-double(obj.L_cable)*q_a;
            cable_residual_a=0;

            ttc_a=double(collision_time)-ta;
            jhit_a=round(ttc_a/dt)+1;
            jhit_a=max(1,min(size(pL_future,2),jhit_a));

            elapsed_before=toc(t_hard_cycle);
            budget_remaining=max(0,dt-elapsed_before);
            if r5row>0, r5x=tic; end
            CTX=obj.build_generator_request_context_compact( ...
                ta,elapsed_before,budget_remaining, ...
                xd_nom_a,xd_active_a,pQ_a,pL_a,q_a,cable_residual_a, ...
                active,all_dists,pL_future,q_future,ref_source, ...
                ttc_a,double(collision_time),obs_id,comp_name,comp_code, ...
                pL_collision,q_collision,pQ_collision,jhit_a);
            if r5row>0, obj.af7r58zzy_r5_ring(r5row,20)=1e3*toc(r5x); end

            try
                if r5row>0, r5x=tic; end
                candidate=obj.af7r46_single_core_proposal_compact(CTX);
                if r5row>0, obj.af7r58zzy_r5_ring(r5row,21)=1e3*toc(r5x); end
            catch ME
                obj.staged_replan_fail_count=obj.staged_replan_fail_count+1;
                obj.generator_validation_fail_count= ...
                    obj.generator_validation_fail_count+1;
                obj.generator_last_status_code=-61;
                obj.staged_replan_last_reason= ...
                    ['STAGE1_NATIVE_ERROR__' ME.identifier];
                return;
            end

            stage_ms=1e3*toc(ts);
            obj.staged_replan_last_stage1_ms=stage_ms;
            if r5row>0
                obj.af7r58zzy_r5_ring(r5row,18)=stage_ms;
                z=obj.af7r58zzy_r5_ring(r5row,19:21);
                z(~isfinite(z))=0;
                obj.af7r58zzy_r5_ring(r5row,22)=max(0,stage_ms-sum(z));
            end
            obj.generator_last_runtime_ms=double(candidate.r9_stage_timing.total_internal_ms);
            obj.generator_last_callback_ms=stage_ms;
            obj.generator_last_cycle_critical_ms=1e3*toc(t_hard_cycle);
            obj.generator_request_count=obj.generator_request_count+1;
            obj.staged_replan_stage1_count=obj.staged_replan_stage1_count+1;

            if obj.generator_last_cycle_critical_ms>1e3*dt
                obj.generator_deadline_fail_count= ...
                    obj.generator_deadline_fail_count+1;
                obj.generator_last_status_code=-62;
                obj.staged_replan_fail_count=obj.staged_replan_fail_count+1;
                obj.staged_replan_last_reason='STAGE1_CONTROL_PERIOD_EXCEEDED';
                return;
            end

            if ~(isstruct(candidate)&&isfield(candidate,'success')&& ...
                    logical(candidate.success))
                obj.generator_validation_fail_count= ...
                    obj.generator_validation_fail_count+1;
                obj.generator_last_status_code=21;
                obj.staged_replan_fail_count=obj.staged_replan_fail_count+1;
                obj.staged_replan_last_reason='STAGE1_PROPOSAL_NOT_CERTIFIED';
                return;
            end

            assert(abs(double(candidate.t0)-ta)<=64*eps(max(1,abs(ta))), ...
                '[AF7R58ZZU] staged candidate t0 is not activation time.');

            obj.staged_replan_candidate=candidate;
            obj.staged_replan_rref=struct();
            obj.staged_replan_bundle=struct();
            obj.staged_replan_request_time=t_now;
            obj.staged_replan_activation_time=ta;
            obj.staged_replan_source_mode=char(obj.active_reference_mode);
            obj.staged_replan_source_version=double(obj.active_reference_version);
            obj.staged_replan_known_mask=logical(active(:).');
            obj.staged_replan_static_map=obj.static_obstacle_map;
            obj.staged_replan_dependency_key= ...
                obj.staged_reference_only_dependency_key();
            obj.staged_replan_speed_source=obj.active_speed_certificate;
            obj.staged_replan_obstacle_id=double(obs_id);
            obj.staged_replan_component_code=double(comp_code);
            obj.staged_replan_component_name=char(comp_name);
            obj.staged_replan_collision_time=double(collision_time);
            obj.staged_replan_initial_ttc=double(ttc);
            obj.staged_replan_pL_collision=double(pL_collision(:));
            obj.staged_replan_q_collision=double(q_collision(:));
            obj.staged_replan_pQ_collision=double(pQ_collision(:));
            obj.staged_replan_hit_index=double(jhit);
            obj.staged_replan_state='GENERATED';
            obj.staged_replan_last_reason='STAGE1_GENERATED';
            obj.generator_candidate_count=obj.generator_candidate_count+1;
            obj.generator_last_status_code=40;
        end


        function advance_staged_replan_pipeline(obj,t_hard_cycle,t_now, ...
                active,need_replan,obs_id,comp_code)
            state=char(obj.staged_replan_state);
            if strcmp(state,'IDLE')
                return;
            end

            r5row=obj.af7r58zzy_r5_current_row;
            if r5row>0, r5fresh=tic; end
            t_now=double(t_now);
            dt=double(obj.prediction_dt);
            ta=double(obj.staged_replan_activation_time);
            scale=max([1,abs(t_now),abs(ta)]);
            tol=64*eps(scale);

            if ~logical(need_replan)
                obj.staged_replan_stale_reject_count= ...
                    obj.staged_replan_stale_reject_count+1;
                obj.reset_staged_replan_pipeline('TRIGGER_CLEARED',false);
                return;
            end

            fresh = ...
                strcmp(char(obj.active_reference_mode), ...
                    char(obj.staged_replan_source_mode)) && ...
                double(obj.active_reference_version)== ...
                    double(obj.staged_replan_source_version) && ...
                isequal(logical(active(:).'), ...
                    logical(obj.staged_replan_known_mask(:).')) && ...
                isequaln(obj.static_obstacle_map,obj.staged_replan_static_map) && ...
                isequaln(obj.staged_reference_only_dependency_key(), ...
                    obj.staged_replan_dependency_key) && ...
                isequaln(obj.active_speed_certificate, ...
                    obj.staged_replan_speed_source) && ...
                isequaln(double(obs_id),double(obj.staged_replan_obstacle_id)) && ...
                isequaln(double(comp_code),double(obj.staged_replan_component_code));
            if r5row>0, obj.af7r58zzy_r5_ring(r5row,24)=1e3*toc(r5fresh); end

            if ~fresh
                obj.staged_replan_stale_reject_count= ...
                    obj.staged_replan_stale_reject_count+1;
                obj.generator_last_status_code=-63;
                obj.reset_staged_replan_pipeline( ...
                    'PIPELINE_SOURCE_OR_SENSOR_STALE',true);
                return;
            end

            if strcmp(state,'GENERATED')
                t_stage2=double(obj.staged_replan_request_time)+dt;
                if t_now<t_stage2-tol
                    return;
                end
                if t_now>t_stage2+tol
                    obj.staged_replan_fail_count=obj.staged_replan_fail_count+1;
                    obj.generator_last_status_code=-64;
                    obj.reset_staged_replan_pipeline( ...
                        'STAGE2_GRID_MISSED',true);
                    return;
                end

                ts=tic;
                P=obj.staged_replan_candidate;
                if r5row>0, r5x=tic; end
                Rref=obj.evaluate_reference_only_proposal_certificate(P);
                if r5row>0, obj.af7r58zzy_r5_ring(r5row,25)=1e3*toc(r5x); end
                obj.reference_only_evaluation_count= ...
                    obj.reference_only_evaluation_count+1;
                if ~(isstruct(Rref)&&isfield(Rref,'certified')&& ...
                        logical(Rref.certified))
                    obj.staged_replan_fail_count=obj.staged_replan_fail_count+1;
                    obj.generator_validation_fail_count= ...
                        obj.generator_validation_fail_count+1;
                    obj.generator_last_status_code=33;
                    obj.reset_staged_replan_pipeline( ...
                        'STAGE2_REFERENCE_CERTIFICATE_REJECT',true);
                    return;
                end

                if r5row>0, r5x=tic; end
                [ok,reason,B]=obj.prepare_staged_reference_only_bundle(P,ta);
                if r5row>0, obj.af7r58zzy_r5_ring(r5row,26)=1e3*toc(r5x); end
                obj.staged_replan_last_stage2_ms=1e3*toc(ts);
                if r5row>0
                    obj.af7r58zzy_r5_ring(r5row,36)= ...
                        obj.staged_replan_last_stage2_ms;
                end
                obj.generator_last_cycle_critical_ms=1e3*toc(t_hard_cycle);
                obj.staged_replan_stage2_count=obj.staged_replan_stage2_count+1;

                if ~ok
                    obj.staged_replan_fail_count=obj.staged_replan_fail_count+1;
                    obj.generator_validation_fail_count= ...
                        obj.generator_validation_fail_count+1;
                    obj.generator_last_status_code=31;
                    obj.reset_staged_replan_pipeline( ...
                        ['STAGE2_PREP_REJECT__' char(reason)],true);
                    return;
                end
                if obj.generator_last_cycle_critical_ms>1e3*dt
                    obj.generator_deadline_fail_count= ...
                        obj.generator_deadline_fail_count+1;
                    obj.staged_replan_fail_count=obj.staged_replan_fail_count+1;
                    obj.generator_last_status_code=-65;
                    obj.reset_staged_replan_pipeline( ...
                        'STAGE2_CONTROL_PERIOD_EXCEEDED',true);
                    return;
                end

                obj.staged_replan_rref=Rref;
                obj.staged_replan_bundle=B;
                obj.staged_replan_state='PREPARED';
                obj.staged_replan_last_reason='STAGE2_CERTIFIED_AND_PREPARED';
                obj.generator_last_status_code=41;
                return;
            end

            if strcmp(state,'PREPARED')
                if t_now<ta-tol
                    return;
                end
                if t_now>ta+tol
                    obj.staged_replan_fail_count=obj.staged_replan_fail_count+1;
                    obj.generator_last_status_code=-66;
                    obj.reset_staged_replan_pipeline( ...
                        'STAGE3_ACTIVATION_GRID_MISSED',true);
                    return;
                end

                ts=tic;
                [ok,reason]=obj.activate_staged_reference_only_bundle( ...
                    obj.staged_replan_bundle,t_now,t_hard_cycle);
                obj.staged_replan_last_stage3_ms=1e3*toc(ts);
                if r5row>0
                    obj.af7r58zzy_r5_ring(r5row,37)= ...
                        obj.staged_replan_last_stage3_ms;
                end

                if ok
                    obj.staged_replan_commit_count= ...
                        obj.staged_replan_commit_count+1;
                    obj.generator_validation_pass_count= ...
                        obj.generator_validation_pass_count+1;
                    obj.generator_last_status_code=30;
                    obj.reset_staged_replan_pipeline( ...
                        'STAGE3_ATOMIC_COMMIT_CLOSED',false);
                else
                    obj.staged_replan_fail_count=obj.staged_replan_fail_count+1;
                    obj.generator_validation_fail_count= ...
                        obj.generator_validation_fail_count+1;
                    obj.generator_last_status_code=-67;
                    obj.reset_staged_replan_pipeline( ...
                        ['STAGE3_COMMIT_REJECT__' char(reason)],true);
                end
            end
        end


        function [ok,reason,B] = prepare_staged_reference_only_bundle(obj,P,t_commit)
            ok=false;
            reason='';
            B=struct();
            r5row=obj.af7r58zzy_r5_current_row;
            if r5row>0, r5x=tic; end

            if ~obj.ENABLE_AUTO_REPLAN_COMMIT
                reason='AUTO_COMMIT_DISABLED'; return;
            end
            if ~strcmpi(char(obj.AVOIDANCE_COMMIT_POLICY),'REFERENCE_ONLY')
                reason='NOT_REFERENCE_ONLY_COMMIT_POLICY'; return;
            end
            if ~(isstruct(P)&&isfield(P,'success')&&logical(P.success))
                reason='PROPOSAL_NOT_SUCCESSFUL'; return;
            end

            [spacingOK,~,~,spacingReason]= ...
                af7r58i_commit_spacing_guard(obj,double(t_commit));
            if ~spacingOK
                reason=spacingReason; return;
            end

            G=P.geometry_shadow_certificate;
            SC=P.reference_speed_certificate;
            if ~(isstruct(G)&&isfield(G,'certified')&&logical(G.certified) && ...
                    isfield(G,'continuous_time')&&logical(G.continuous_time) && ...
                    isfield(G,'sampled_acceptance')&&~logical(G.sampled_acceptance))
                reason='REFERENCE_GEOMETRY_CERTIFICATE_NOT_CLOSED'; return;
            end
            if ~(isstruct(SC)&&isfield(SC,'certified')&&logical(SC.certified))
                reason='REFERENCE_SPEED_CERTIFICATE_NOT_CLOSED'; return;
            end

            targetVersion=double(obj.active_reference_version)+1;
            if double(SC.reference_version)~=targetVersion
                reason='REFERENCE_SPEED_CERTIFICATE_VERSION_MISMATCH'; return;
            end
            if ~(isfield(SC,'continuous_time')&&logical(SC.continuous_time) && ...
                    isfield(SC,'sampled_acceptance')&&~logical(SC.sampled_acceptance) && ...
                    isfield(SC,'specific_force_norm_LB_m_s2') && ...
                    isnumeric(SC.specific_force_norm_LB_m_s2) && ...
                    isscalar(SC.specific_force_norm_LB_m_s2) && ...
                    isfinite(double(SC.specific_force_norm_LB_m_s2)) && ...
                    double(SC.specific_force_norm_LB_m_s2)>0)
                reason='REFERENCE_ONLY_CONTINUOUS_FLATNESS_CERTIFICATE_NOT_CLOSED'; return;
            end
            if ~(isfield(P,'c7_exact')&&logical(P.c7_exact))
                reason='REFERENCE_ONLY_C7_CERTIFICATE_NOT_CLOSED'; return;
            end
            if r5row>0, obj.af7r58zzy_r5_ring(r5row,27)=1e3*toc(r5x); end

            if r5row>0, r5x=tic; end
            [resourceOK,resourceReason,batchModel,resourceMeta]= ...
                af7r58i_prepare_batch_model( ...
                    obj,P,targetVersion,double(t_commit));
            if r5row>0, obj.af7r58zzy_r5_ring(r5row,28)=1e3*toc(r5x); end
            if ~resourceOK
                reason=resourceReason; return;
            end

            % Exact rebuild is intentional in the staged-preparation cycle.
            % No cache/reuse theorem is needed for timing here.
            if r5row>0, r5x=tic; end
            sweepNew=obj.build_trigger_sweep_for_target(SC,targetVersion);
            if r5row>0, obj.af7r58zzy_r5_ring(r5row,29)=1e3*toc(r5x); end
            if r5row>0, r5x=tic; end
            packetNew=obj.prepare_obstacle_packet_fast_target( ...
                obj.obstacle_packet,sweepNew,targetVersion);
            if r5row>0, obj.af7r58zzy_r5_ring(r5row,30)=1e3*toc(r5x); end
            if r5row>0, r5x=tic; end
            sensorNew=obj.build_sensor_horizon_for_target(SC,targetVersion);
            if r5row>0, obj.af7r58zzy_r5_ring(r5row,31)=1e3*toc(r5x); end
            if ~(isstruct(sensorNew)&&isfield(sensorNew,'certified')&& ...
                    logical(sensorNew.certified))
                reason='STAGED_TARGET_SENSOR_HORIZON_OPEN'; return;
            end
            if r5row>0, r5x=tic; end
            depNew=obj.runtime_dependency_signature;
            depNew.reference_mode='REPAIRED';
            depNew.reference_version=double(targetVersion);

            [evalFn,flatEvalFn]= ...
                af7r58i_make_canonical_eval_handles(obj,batchModel);

            smoothSource='DEGREE15_ENDPOINT_REPEATED_BEZIER_C7';
            if isfield(P,'representation_code')&&double(P.representation_code)==2
                smoothSource='PIECEWISE_DEGREE15_BEZIER_C7_AFFINE_TOWING_CORRIDORS';
            end
            smooth=struct( ...
                'certified',logical(P.c7_exact), ...
                'continuous_time',true, ...
                'sampled_acceptance',false, ...
                'continuity_order',7, ...
                'reference_mode','REPAIRED', ...
                'reference_version',targetVersion, ...
                'valid_from',double(P.t0), ...
                'valid_until',double(P.t1), ...
                'source',smoothSource);

            geom=G;
            geom.reference_mode='REPAIRED';
            geom.reference_version=targetVersion;
            geom.valid_from=double(P.t0);
            geom.valid_until=double(P.t1);
            geom.reference_only_authority=true;
            geom.final_system_authority=false;

            R=struct();
            R.valid=true;
            R.backend='BEZIER_D15_C7';
            R.eval_fn=evalFn;
            R.flat_eval_fn=flatEvalFn;
            R.valid_from=double(P.t0);
            R.valid_until=Inf;
            R.reference_version=targetVersion;
            R.source='AF7R58ZZU_STAGED_REFERENCE_ONLY_COMMIT';
            R.yaw_policy='C6_INDEPENDENT';
            R.path_parameterization=struct( ...
                'kind','DIRECT_TIME_BEZIER_CORRECTION', ...
                'path_derivative_order',7, ...
                'time_derivative_order',7);
            R.rejoin_time=double(P.t1);
            R.representation=struct( ...
                'kind','BEZIER_D15_C7_TRANSLATION_BUMP', ...
                't0',double(P.t0),'tm',double(P.tm),'t1',double(P.t1), ...
                'displacement',double(P.displacement(:)));
            R.batch_flat_model=batchModel;
            R.resource_certificate=resourceMeta.contract;
            R.resource_state=resourceMeta;
            R.authority_split=struct( ...
                'controller_packet','eval_fn', ...
                'mathematical_flat_jet','flat_eval_fn', ...
                'legacy_high_derivative_slots_used_for_geometry',false);
            if r5row>0, obj.af7r58zzy_r5_ring(r5row,32)=1e3*toc(r5x); end

            % Pre-build the first packet that will be requested AFTER the
            % t_commit authority swap.  The commit cycle itself has already
            % evaluated its pre-commit packet, so the first repaired packet is
            % at t_commit+dt.  Moving this full N-point batch into Stage 2
            % removes the post-commit cold rebuild from the next Flight cycle.
            dt=double(obj.prediction_dt);
            H=double(obj.horizon_s);
            N=round(H/dt)+1;
            tCache=double(t_commit)+dt;
            tqCache=tCache+(0:N-1)*dt;
            if r5row>0, r5x=tic; end
            [Ycache,validCache]= ...
                af7r33_flat_jet_batch_model(obj,tqCache,batchModel);
            if r5row>0, obj.af7r58zzy_r5_ring(r5row,33)=1e3*toc(r5x); end
            if r5row>0, r5x=tic; end
            Ycache=double(Ycache);
            if ~(isequal(size(Ycache),[21 N]) && numel(validCache)==N && ...
                    all(logical(validCache(:))) && all(isfinite(Ycache),'all'))
                reason='STAGED_POSTCOMMIT_CACHE_BATCH_INVALID'; return;
            end
            pLcache=Ycache(1:3,:);
            sfcache=Ycache(7:9,:)+[0;0;double(obj.gravity)];
            nscache=sqrt(sum(sfcache.^2,1));
            if ~(all(isfinite(nscache))&&all(nscache>0))
                reason='STAGED_POSTCOMMIT_CACHE_FLATNESS_SINGULAR'; return;
            end
            qcache=-sfcache./nscache;
            postCache=struct( ...
                'reference_version',double(targetVersion), ...
                't0',double(tCache),'dt',dt,'H',H, ...
                'pL',pLcache,'q',qcache);
            if r5row>0, obj.af7r58zzy_r5_ring(r5row,34)=1e3*toc(r5x); end

            if r5row>0, r5x=tic; end
            B=struct( ...
                'schema','AF7R58ZZV_PREPARED_COMMIT_V2', ...
                'source_mode',char(obj.active_reference_mode), ...
                'source_version',double(obj.active_reference_version), ...
                'target_version',double(targetVersion), ...
                'activation_time',double(t_commit), ...
                'dependency_key',obj.staged_reference_only_dependency_key(), ...
                'speed_source',obj.active_speed_certificate, ...
                'known_mask',logical(obj.staged_replan_known_mask(:).'), ...
                'static_map',obj.static_obstacle_map, ...
                'reference',R, ...
                'certificates',struct( ...
                    'reference_only',true,'smoothness',smooth,'geometry',geom), ...
                'speed',SC, ...
                'sweep',sweepNew, ...
                'packet',packetNew, ...
                'sensor',sensorNew, ...
                'dep',depNew, ...
                'post_commit_reference_cache',postCache, ...
                'prevalidated',true);
            obj.staged_replan_postcommit_cache_prepare_count= ...
                obj.staged_replan_postcommit_cache_prepare_count+1;
            ok=true;
            reason='STAGED_COMMIT_BUNDLE_AND_POSTCACHE_PREPARED';
            if r5row>0, obj.af7r58zzy_r5_ring(r5row,35)=1e3*toc(r5x); end
        end


        function [ok,reason] = activate_staged_reference_only_bundle( ...
                obj,B,t_now,t_hard_cycle)
            ok=false;
            reason='';
            r5row=obj.af7r58zzy_r5_current_row;
            if r5row>0, r5x=tic; end

            if ~(isstruct(B)&&isfield(B,'schema')&& ...
                    strcmp(char(B.schema),'AF7R58ZZV_PREPARED_COMMIT_V2') && ...
                    isfield(B,'prevalidated')&&logical(B.prevalidated) && ...
                    isfield(B,'post_commit_reference_cache'))
                reason='STAGED_BUNDLE_SCHEMA_MISMATCH'; return;
            end

            scale=max([1,abs(double(t_now)),abs(double(B.activation_time))]);
            if abs(double(t_now)-double(B.activation_time))>64*eps(scale)
                reason='STAGED_ACTIVATION_TIME_MISMATCH'; return;
            end
            if ~strcmp(char(obj.active_reference_mode),char(B.source_mode)) || ...
                    double(obj.active_reference_version)~=double(B.source_version)
                reason='STAGED_SOURCE_REFERENCE_STALE'; return;
            end
            if ~isequaln(obj.staged_reference_only_dependency_key(),B.dependency_key) || ...
                    ~isequaln(obj.active_speed_certificate,B.speed_source) || ...
                    ~isequaln(obj.static_obstacle_map,B.static_map)
                reason='STAGED_DEPENDENCY_OR_MAP_STALE'; return;
            end
            if r5row>0, obj.af7r58zzy_r5_ring(r5row,38)=1e3*toc(r5x); end

            if r5row>0, r5x=tic; end
            old=struct( ...
                'mode',obj.active_reference_mode, ...
                'version',obj.active_reference_version, ...
                'ref',obj.active_repaired_reference, ...
                'cert',obj.active_repaired_certificates, ...
                'speed',obj.active_speed_certificate, ...
                'sweep',obj.trigger_sweep_inflation, ...
                'packet',obj.obstacle_packet, ...
                'sensor',obj.sensor_horizon_certificate, ...
                'dep',obj.runtime_dependency_signature, ...
                'stale',obj.certificate_stale, ...
                'level',obj.active_repaired_safety_level, ...
                'time',obj.last_replan_commit_time);
            if r5row>0, obj.af7r58zzy_r5_ring(r5row,39)=1e3*toc(r5x); end

            try
                if r5row>0, r5x=tic; end
                obj.active_repaired_reference=B.reference;
                obj.active_repaired_certificates=B.certificates;
                obj.active_reference_mode='REPAIRED';
                obj.active_reference_version=double(B.target_version);
                obj.active_repaired_safety_level='REFERENCE_ONLY';
                obj.active_speed_certificate=B.speed;
                obj.trigger_sweep_inflation=B.sweep;
                obj.obstacle_packet=B.packet;
                obj.sensor_horizon_certificate=B.sensor;
                obj.runtime_dependency_signature=B.dep;
                obj.certificate_stale=obj.empty_certificate_stale_state();
                obj.certificate_stale.ACTUATOR_DYNAMICS_CERT=true;
                obj.certificate_stale.HLC_ROBUST_CERT=true;
                obj.last_replan_commit_time=double(t_now);

                % Stage 2 already performed all expensive construction and
                % continuous certificate closure.  At Stage 3, after exact
                % source/dependency/map freshness checks above, only the
                % version/staleness hot guard is required.
                obj.assert_runtime_core_certificates_current_hot();
                if r5row>0, obj.af7r58zzy_r5_ring(r5row,40)=1e3*toc(r5x); end
                if r5row>0, r5x=tic; end

                % The main loop must still close inside its 25-ms period.
                if toc(t_hard_cycle)>double(obj.prediction_dt)
                    obj.active_reference_mode=old.mode;
                    obj.active_reference_version=old.version;
                    obj.active_repaired_reference=old.ref;
                    obj.active_repaired_certificates=old.cert;
                    obj.active_speed_certificate=old.speed;
                    obj.trigger_sweep_inflation=old.sweep;
                    obj.obstacle_packet=old.packet;
                    obj.sensor_horizon_certificate=old.sensor;
                    obj.runtime_dependency_signature=old.dep;
                    obj.certificate_stale=old.stale;
                    obj.active_repaired_safety_level=old.level;
                    obj.last_replan_commit_time=old.time;
                    obj.generator_deadline_fail_count= ...
                        obj.generator_deadline_fail_count+1;
                    reason='STAGED_ATOMIC_CONTROL_PERIOD_EXCEEDED';
                    return;
                end

                Cpost=B.post_commit_reference_cache;
                assert(isstruct(Cpost) && ...
                    double(Cpost.reference_version)==double(B.target_version) && ...
                    abs(double(Cpost.t0)-(double(t_now)+double(obj.prediction_dt)))<= ...
                        64*eps(max(1,abs(double(Cpost.t0)))), ...
                    '[AF7R58ZZV] prepared post-commit reference cache is stale.');
                obj.repaired_reference_cache=Cpost;
                obj.repaired_reference_cache_valid=true;
                obj.reference_only_prepared_packet_cache_valid=false;
                obj.reference_only_prepared_packet_cache=struct();

                obj.replan_commit_count=obj.replan_commit_count+1;
                obj.reference_only_commit_count= ...
                    obj.reference_only_commit_count+1;
                obj.replan_request_active=false;

                obj.last_replan_status=struct( ...
                    'requested',true, ...
                    'committed',true, ...
                    'safety_mode','REFERENCE_ONLY', ...
                    'safety_level','REFERENCE_ONLY', ...
                    'full_system_authority',false, ...
                    'reference_version',double(B.target_version), ...
                    'backend','BEZIER_D15_C7', ...
                    'commit_time',double(t_now), ...
                    'rejoin_time',double(B.reference.rejoin_time), ...
                    'staged_pipeline',true, ...
                    'pipeline_intervals',2);
                if r5row>0, obj.af7r58zzy_r5_ring(r5row,41)=1e3*toc(r5x); end

                ok=true;
                reason='STAGED_REFERENCE_ONLY_COMMIT_CLOSED';
            catch ME
                obj.active_reference_mode=old.mode;
                obj.active_reference_version=old.version;
                obj.active_repaired_reference=old.ref;
                obj.active_repaired_certificates=old.cert;
                obj.active_speed_certificate=old.speed;
                obj.trigger_sweep_inflation=old.sweep;
                obj.obstacle_packet=old.packet;
                obj.sensor_horizon_certificate=old.sensor;
                obj.runtime_dependency_signature=old.dep;
                obj.certificate_stale=old.stale;
                obj.active_repaired_safety_level=old.level;
                obj.last_replan_commit_time=old.time;
                reason=ME.message;
            end
        end


        function [need_replan,ttc,collision_time,comp_name, ...
                pL_collision,q_collision,pQ_collision,jhit, ...
                obs_id,comp_code,kernel_called,kernel_ms] = ...
                trigger_core(obj,active,pL_ref,q_ref,t_now)

            kernel_called=any(active);
            triggered=false;
            jhit=NaN;
            obs_id=NaN;
            comp_code=NaN;
            kernel_ms=NaN;

            if kernel_called
                % Fail closed if active reference mode/version changed without
                % installing the matching continuous-time speed certificate
                % and rebuilding the obstacle packet.
                obj.assert_trigger_packet_certificate_current();

                if obj.ENABLE_PROFILING, tk=tic; end
                r5row=obj.af7r58zzy_r5_current_row;
                if r5row>0, r5x=tic; end
                [triggered,jhit,obs_id,comp_code]= ...
                    g26_future_trigger_kernel_mex( ...
                        obj.horizon_s,obj.prediction_dt,obj.L_cable, ...
                        pL_ref,q_ref, ...
                        obj.obstacle_packet.C,obj.obstacle_packet.R, ...
                        obj.obstacle_packet.Apay,obj.obstacle_packet.Auav, ...
                        obj.obstacle_packet.Acable,obj.obstacle_packet.ids, ...
                        double(active));
                if r5row>0, obj.af7r58zzy_r5_ring(r5row,15)=1e3*toc(r5x); end
                if obj.ENABLE_PROFILING, kernel_ms=1e3*toc(tk); end
            end

            N=size(pL_ref,2);
            r5row=obj.af7r58zzy_r5_current_row;
            if r5row>0, r5x=tic; end
            [need_replan,ttc,collision_time,comp_name, ...
                pL_collision,q_collision,pQ_collision,jhit]= ...
                obj.decode_trigger_result( ...
                    triggered,jhit,obs_id,comp_code,N,pL_ref,q_ref,t_now);
            if r5row>0, obj.af7r58zzy_r5_ring(r5row,16)=1e3*toc(r5x); end
        end


    end
end
