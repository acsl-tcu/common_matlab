classdef REPLANNING_SYSTEM_R20_FAST < REPLANNING_SYSTEM_R19_FAST
% AF7R58ZZY-R20 | LEAN STAGE2 INTEGRATION
% Direct continuation of the bitwise-validated R19 authority.
%
% R20 changes only implementation redundancy in the staged REFERENCE_ONLY
% Stage2 path:
%   1) successful-candidate reference gate is checked without constructing
%      the unused read-only diagnostic struct before prepare;
%   2) the finite resource contract, which depends only on the frozen
%      nominal-table time authority, is cached once after nominal-table build;
%   3) when speed bounds differ, the exact production reuse theorem is known
%      false from its first necessary equality, so the expensive deep reuse
%      test is skipped and the speed-dependent packet is rebuilt from the
%      already-fresh Stage1 static authority;
%   4) contiguous post-commit nominal-table slices plus correction group
%      accumulation use one strict-FP direct-accumulation MEX.
%
% No trajectory equations, continuous certificates, commit timing, controller,
% HLC, plant, or obstacle geometry formula is changed.

    properties (SetAccess = private)
        r20_build_id = 'AF7R58ZZY_R20_FAST_V1';
        r20_resource_contract_cache = struct();
        r20_resource_contract_cache_valid = false;
        r20_stage2_fresh_gate_closed = false;
        r20_stage2_reference_gate_closed = false;

        r20_last_prepare_total_ms = NaN;
        r20_last_batch_model_ms = NaN;
        r20_last_packet_ms = NaN;
        r20_last_handles_ms = NaN;
        r20_last_postcache_ms = NaN;
        r20_last_bundle_ms = NaN;
        r20_last_stage2_wrapper_ms = NaN;
        r20_last_fresh_gate_ms = NaN;
        r20_last_resource_cache_used = false;
        r20_last_runtime_packet_reuse = false;
        r20_last_fresh_packet_path_used = false;
        r20_last_postcache_fast_used = false;
    end

    methods
        function obj = REPLANNING_SYSTEM_R20_FAST(self,base_ref,opts)
            obj@REPLANNING_SYSTEM_R19_FAST(self,base_ref,opts);
            assert(exist('af7r76_apply_correction_mex','file')==3, ...
                '[R20] direct correction MEX is missing. Run AF7R58ZZY_R20_INSTALL first.');
            assert(exist('af7r76_postcache_R20','file')==2, ...
                '[R20] postcache helper is missing. Run AF7R58ZZY_R20_INSTALL first.');
            assert(exist('af7r76_prepare_batch_model_R20','file')==2, ...
                '[R20] cached batch-model helper is missing. Run AF7R58ZZY_R20_INSTALL first.');
        end

        function build_nominal_reference_table(obj,time,varargin_in,xd_now)
            % Let R19 build/capture the exact nominal tables first.
            build_nominal_reference_table@REPLANNING_SYSTEM_R19_FAST( ...
                obj,time,varargin_in,xd_now);

            % Resource theorem depends only on this frozen table/time authority.
            % Cache it once; Stage2 still performs all candidate-specific gates.
            obj.r20_resource_contract_cache=struct();
            obj.r20_resource_contract_cache_valid=false;
            if obj.nominal_table_valid
                RC=af7r58i_resource_contract(obj);
                if isstruct(RC) && isfield(RC,'closed') && logical(RC.closed) && ...
                        isfield(RC,'flight_t0') && ...
                        isequal(double(RC.flight_t0),double(obj.flight_reference_time_origin)) && ...
                        isfield(RC,'reference_tend') && ...
                        isequal(double(RC.reference_tend),double(obj.nominal_table_tend)) && ...
                        isfield(RC,'dt') && ...
                        isequal(double(RC.dt),double(obj.prediction_dt))
                    obj.r20_resource_contract_cache=RC;
                    obj.r20_resource_contract_cache_valid=true;
                end
            end
        end

        function R = get_r20_performance_status(obj)
            R=struct( ...
                'prepare_total_ms',double(obj.r20_last_prepare_total_ms), ...
                'batch_model_ms',double(obj.r20_last_batch_model_ms), ...
                'packet_ms',double(obj.r20_last_packet_ms), ...
                'handles_ms',double(obj.r20_last_handles_ms), ...
                'postcache_ms',double(obj.r20_last_postcache_ms), ...
                'bundle_ms',double(obj.r20_last_bundle_ms), ...
                'stage2_wrapper_ms',double(obj.r20_last_stage2_wrapper_ms), ...
                'fresh_gate_ms',double(obj.r20_last_fresh_gate_ms), ...
                'resource_cache_used',logical(obj.r20_last_resource_cache_used), ...
                'runtime_packet_reuse',logical(obj.r20_last_runtime_packet_reuse), ...
                'fresh_packet_path_used',logical(obj.r20_last_fresh_packet_path_used), ...
                'postcache_fast_used',logical(obj.r20_last_postcache_fast_used));
            vals=[R.batch_model_ms R.packet_ms R.handles_ms R.postcache_ms R.bundle_ms];
            R.measured_subtotal_ms=sum(vals(isfinite(vals)));
            R.prepare_residual_ms=R.prepare_total_ms-R.measured_subtotal_ms;
        end

        function advance_staged_replan_pipeline(obj,t_hard_cycle,t_now, ...
                active,need_replan,obs_id,comp_code)
            % Exact parent state machine with the Stage2 read-only diagnostic
            % struct construction removed from the hot path.  The same
            % reference-layer accept/reject gate is evaluated directly.
            state=char(obj.staged_replan_state);
            if strcmp(state,'IDLE')
                return;
            end

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

            tfresh=tic;
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
            freshGateMs=1e3*toc(tfresh);
            if strcmp(state,'GENERATED')
                obj.r20_last_fresh_gate_ms=freshGateMs;
            end

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
                [refOK,~]=obj.r20_reference_only_gate(P);
                obj.reference_only_evaluation_count= ...
                    obj.reference_only_evaluation_count+1;
                if ~refOK
                    obj.staged_replan_fail_count=obj.staged_replan_fail_count+1;
                    obj.generator_validation_fail_count= ...
                        obj.generator_validation_fail_count+1;
                    obj.generator_last_status_code=33;
                    obj.reset_staged_replan_pipeline( ...
                        'STAGE2_REFERENCE_CERTIFICATE_REJECT',true);
                    return;
                end

                obj.r20_stage2_fresh_gate_closed=true;
                obj.r20_stage2_reference_gate_closed=true;
                try
                    [ok,reason,B]=obj.prepare_staged_reference_only_bundle(P,ta);
                catch ME
                    obj.r20_stage2_fresh_gate_closed=false;
                    obj.r20_stage2_reference_gate_closed=false;
                    rethrow(ME);
                end
                obj.r20_stage2_fresh_gate_closed=false;
                obj.r20_stage2_reference_gate_closed=false;

                obj.staged_replan_last_stage2_ms=1e3*toc(ts);
                obj.r20_last_stage2_wrapper_ms= ...
                    obj.staged_replan_last_stage2_ms-obj.r20_last_prepare_total_ms;
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

                % staged_replan_rref is not consumed by Stage3; keep a compact
                % successful read-only record instead of rebuilding the full
                % diagnostic mirror in the real-time path.
                obj.staged_replan_rref=struct( ...
                    'schema','REPLANNING_REFERENCE_ONLY_EVALUATION_V1', ...
                    'certified',true, ...
                    'full_system_authority',false, ...
                    'sampled_acceptance',false, ...
                    'reason','REFERENCE_ONLY_CERTIFICATE_CLOSED', ...
                    'reference_version',double(obj.active_reference_version)+1);
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
            tAll=tic;
            obj.r20_last_prepare_total_ms=NaN;
            obj.r20_last_batch_model_ms=NaN;
            obj.r20_last_packet_ms=NaN;
            obj.r20_last_handles_ms=NaN;
            obj.r20_last_postcache_ms=NaN;
            obj.r20_last_bundle_ms=NaN;
            obj.r20_last_resource_cache_used=false;
            obj.r20_last_runtime_packet_reuse=false;
            obj.r20_last_fresh_packet_path_used=false;
            obj.r20_last_postcache_fast_used=false;

            % Outside the staged R20 state machine, preserve the R19 authority.
            if ~(obj.r20_stage2_fresh_gate_closed && obj.r20_stage2_reference_gate_closed)
                [ok,reason,B]=prepare_staged_reference_only_bundle@REPLANNING_SYSTEM_R19_FAST( ...
                    obj,P,t_commit);
                obj.r20_last_prepare_total_ms=1e3*toc(tAll);
                return;
            end

            if ~obj.ENABLE_AUTO_REPLAN_COMMIT
                reason='AUTO_COMMIT_DISABLED'; return;
            end
            if ~strcmpi(char(obj.AVOIDANCE_COMMIT_POLICY),'REFERENCE_ONLY')
                reason='NOT_REFERENCE_ONLY_COMMIT_POLICY'; return;
            end

            [spacingOK,~,~,spacingReason]= ...
                af7r58i_commit_spacing_guard(obj,double(t_commit));
            if ~spacingOK
                reason=spacingReason; return;
            end

            % The successful reference-only subset gate was closed directly in
            % advance_staged_replan_pipeline(). Only Stage2-specific continuous
            % flatness/C7 requirements remain here.
            G=P.geometry_shadow_certificate;
            SC=P.reference_speed_certificate;
            targetVersion=double(obj.active_reference_version)+1;
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

            tx=tic;
            useRC=obj.r20_resource_contract_cache_valid && ...
                isstruct(obj.r20_resource_contract_cache) && ...
                isfield(obj.r20_resource_contract_cache,'closed') && ...
                logical(obj.r20_resource_contract_cache.closed) && ...
                isequal(double(obj.r20_resource_contract_cache.flight_t0), ...
                    double(obj.flight_reference_time_origin)) && ...
                isequal(double(obj.r20_resource_contract_cache.reference_tend), ...
                    double(obj.nominal_table_tend)) && ...
                isequal(double(obj.r20_resource_contract_cache.dt), ...
                    double(obj.prediction_dt));
            if useRC
                [resourceOK,resourceReason,batchModel,resourceMeta]= ...
                    af7r76_prepare_batch_model_R20( ...
                        obj,P,targetVersion,double(t_commit), ...
                        obj.r20_resource_contract_cache);
            else
                [resourceOK,resourceReason,batchModel,resourceMeta]= ...
                    af7r58i_prepare_batch_model( ...
                        obj,P,targetVersion,double(t_commit));
            end
            obj.r20_last_resource_cache_used=logical(useRC);
            obj.r20_last_batch_model_ms=1e3*toc(tx);
            if ~resourceOK
                reason=resourceReason; return;
            end

            tx=tic;
            req={'payload_speed_UB_m_s','uav_speed_UB_m_s'};
            sameSpeed=isstruct(obj.active_speed_certificate) && ...
                all(isfield(obj.active_speed_certificate,req)) && ...
                all(isfield(SC,req)) && ...
                isequal(double(obj.active_speed_certificate.payload_speed_UB_m_s), ...
                    double(SC.payload_speed_UB_m_s)) && ...
                isequal(double(obj.active_speed_certificate.uav_speed_UB_m_s), ...
                    double(SC.uav_speed_UB_m_s));

            reuseRuntimePacket=false;
            if sameSpeed
                % Same speed is only a necessary condition. Preserve the full
                % production exact-reuse theorem before reusing any authority.
                reuseRuntimePacket= ...
                    obj.can_reuse_reference_only_runtime_packet_exact( ...
                        obj.active_speed_certificate,SC, ...
                        obj.trigger_sweep_inflation,obj.obstacle_packet, ...
                        obj.sensor_horizon_certificate,obj.runtime_dependency_signature);
            end

            freshBaseOK=obj.r20_fresh_packet_base_token(obj.obstacle_packet);
            if reuseRuntimePacket
                [sweepNew,packetNew,sensorNew,depNew]= ...
                    obj.reuse_reference_only_runtime_packet_exact( ...
                        obj.trigger_sweep_inflation,obj.obstacle_packet, ...
                        obj.sensor_horizon_certificate,obj.runtime_dependency_signature, ...
                        SC,targetVersion);
            elseif ~sameSpeed && freshBaseOK
                [sweepNew,packetNew,sensorNew,depNew]= ...
                    obj.r20_rebuild_runtime_packet_from_fresh_stage( ...
                        obj.obstacle_packet,SC,targetVersion);
                obj.r20_last_fresh_packet_path_used=true;
            else
                % Same-speed theorem unexpectedly failed: fall back to the
                % unchanged production rebuild, not the lean assumption path.
                sweepNew=obj.build_trigger_sweep_for_target(SC,targetVersion);
                packetNew=obj.prepare_obstacle_packet_fast_target( ...
                    obj.obstacle_packet,sweepNew,targetVersion);
                sensorNew=obj.build_sensor_horizon_for_target(SC,targetVersion);
                if ~(isstruct(sensorNew)&&isfield(sensorNew,'certified')&& ...
                        logical(sensorNew.certified))
                    reason='STAGED_TARGET_SENSOR_HORIZON_OPEN'; return;
                end
                depNew=obj.runtime_dependency_signature;
                depNew.reference_mode='REPAIRED';
                depNew.reference_version=double(targetVersion);
            end
            obj.r20_last_runtime_packet_reuse=logical(reuseRuntimePacket);
            obj.r20_last_packet_ms=1e3*toc(tx);

            tx=tic;
            [evalFn,flatEvalFn]=af7r58i_make_canonical_eval_handles(obj,batchModel);
            obj.r20_last_handles_ms=1e3*toc(tx);

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

            tx=tic;
            dt=double(obj.prediction_dt);
            H=double(obj.horizon_s);
            N=round(H/dt)+1;
            tCache=double(t_commit)+dt;
            tqCache=tCache+(0:N-1)*dt;
            [pLcache,aLcache,validCache,fastUsed]= ...
                af7r76_postcache_R20(obj,tqCache,batchModel);
            obj.r20_last_postcache_fast_used=logical(fastUsed);
            if ~(isequal(size(pLcache),[3 N]) && isequal(size(aLcache),[3 N]) && ...
                    numel(validCache)==N && all(logical(validCache(:))) && ...
                    all(isfinite(pLcache),'all') && all(isfinite(aLcache),'all'))
                reason='STAGED_POSTCOMMIT_CACHE_BATCH_INVALID'; return;
            end
            sfcache=aLcache+[0;0;double(obj.gravity)];
            nscache=sqrt(sum(sfcache.^2,1));
            if ~(all(isfinite(nscache))&&all(nscache>0))
                reason='STAGED_POSTCOMMIT_CACHE_FLATNESS_SINGULAR'; return;
            end
            qcache=-sfcache./nscache;
            postCache=struct( ...
                'reference_version',double(targetVersion), ...
                't0',double(tCache),'dt',dt,'H',H, ...
                'pL',pLcache,'q',qcache);
            obj.r20_last_postcache_ms=1e3*toc(tx);

            tx=tic;
            B=struct( ...
                'schema','AF7R58ZZV_PREPARED_COMMIT_V2', ...
                'source_mode',char(obj.active_reference_mode), ...
                'source_version',double(obj.active_reference_version), ...
                'target_version',double(targetVersion), ...
                'activation_time',double(t_commit), ...
                'dependency_key',obj.staged_replan_dependency_key, ...
                'speed_source',obj.staged_replan_speed_source, ...
                'known_mask',logical(obj.staged_replan_known_mask(:).'), ...
                'static_map',obj.staged_replan_static_map, ...
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
            obj.r20_last_bundle_ms=1e3*toc(tx);
            obj.staged_replan_postcommit_cache_prepare_count= ...
                obj.staged_replan_postcommit_cache_prepare_count+1;
            obj.r20_last_prepare_total_ms=1e3*toc(tAll);
            ok=true;
            reason='STAGED_COMMIT_BUNDLE_AND_POSTCACHE_PREPARED';
        end
    end

    methods (Access = private)
        function [ok,reason] = r20_reference_only_gate(obj,P)
            % Exact accept/reject subset of evaluate_reference_only_proposal_certificate()
            % without constructing its read-only diagnostic struct.
            ok=false;
            reason='PROPOSAL_NOT_SUCCESSFUL';
            if ~(isstruct(P) && isfield(P,'success') && logical(P.success))
                return;
            end
            if ~isfield(P,'geometry_shadow_certificate') || ...
                    ~isfield(P,'reference_speed_certificate')
                reason='REFERENCE_GEOMETRY_CERTIFICATE_NOT_CLOSED';
                return;
            end
            G=P.geometry_shadow_certificate;
            SC=P.reference_speed_certificate;
            if ~(isstruct(G) && isfield(G,'certified') && logical(G.certified) && ...
                    isfield(G,'continuous_time') && logical(G.continuous_time) && ...
                    isfield(G,'sampled_acceptance') && ~logical(G.sampled_acceptance))
                reason='REFERENCE_GEOMETRY_CERTIFICATE_NOT_CLOSED';
                return;
            end
            if ~(isstruct(SC) && isfield(SC,'certified') && logical(SC.certified))
                reason='REFERENCE_SPEED_CERTIFICATE_NOT_CLOSED';
                return;
            end
            if ~isfield(SC,'reference_version') || ...
                    double(SC.reference_version)~=double(obj.active_reference_version)+1
                reason='REFERENCE_SPEED_CERTIFICATE_VERSION_MISMATCH';
                return;
            end
            ok=true;
            reason='REFERENCE_ONLY_CERTIFICATE_CLOSED';
        end

        function tf = r20_fresh_packet_base_token(obj,P)
            % Cheap lineage/shape token. The exact Stage1->2 freshness gate
            % separately proves map/dependency immutability for this interval.
            M=numel(obj.static_obstacle_map);
            tf=isstruct(P) && isscalar(P) && ...
                isfield(P,'reference_mode') && ...
                strcmpi(char(P.reference_mode),char(obj.active_reference_mode)) && ...
                isfield(P,'reference_version') && ...
                isequal(double(P.reference_version),double(obj.active_reference_version)) && ...
                isfield(P,'count') && isequal(double(P.count),double(M)) && ...
                isfield(P,'ids') && numel(P.ids)==M && ...
                isfield(P,'C') && isequal(size(P.C),[3 M]) && ...
                isfield(P,'R') && isequal(size(P.R),[9 M]) && ...
                isfield(P,'types') && numel(P.types)==M && ...
                isfield(P,'Araw') && isequal(size(P.Araw),[3 M]) && ...
                isfield(P,'Apay') && isequal(size(P.Apay),[3 M]) && ...
                isfield(P,'Auav') && isequal(size(P.Auav),[3 M]) && ...
                isfield(P,'Acable') && isequal(size(P.Acable),[3 M]);
        end

        function [sweep,pack,sensor,dep] = ...
                r20_rebuild_runtime_packet_from_fresh_stage(obj,oldPacket,SC,targetVersion)
            % This path is reachable only immediately after the exact Stage1->2
            % freshness gate closed. Therefore the static map and dependency
            % authority are unchanged; only speed-dependent inflation is rebuilt.
            assert(obj.r20_stage2_fresh_gate_closed, ...
                '[R20] fresh-stage packet path used without a closed freshness gate.');
            req={'payload_speed_UB_m_s','uav_speed_UB_m_s'};
            assert(isstruct(SC)&&isfield(SC,'certified')&&logical(SC.certified)&& ...
                all(isfield(SC,req)), '[R20] speed certificate unavailable.');
            VL=double(SC.payload_speed_UB_m_s);
            VQ=double(SC.uav_speed_UB_m_s);
            dt=double(obj.prediction_dt);
            assert(all(isfinite([VL VQ dt]))&&VL>=0&&VQ>=0&&dt>0, ...
                '[R20] invalid speed/dt authority.');

            sweep=struct('certified',true,'reference_mode','REPAIRED', ...
                'reference_version',double(targetVersion), ...
                'payload_m',VL*dt/2,'uav_m',VQ*dt/2, ...
                'cable_m',max(VL,VQ)*dt/2,'sensor_one_period_m',VQ*dt, ...
                'dt',dt,'source','VERSION_BOUND_ACTIVE_SPEED_CERTIFICATE');

            M=numel(obj.static_obstacle_map);
            assert(isstruct(oldPacket)&&isfield(oldPacket,'count')&& ...
                double(oldPacket.count)==M && ...
                isfield(oldPacket,'Araw')&&isequal(size(oldPacket.Araw),[3 M]), ...
                '[R20] active obstacle packet shape is stale.');
            if isfield(oldPacket,'reference_version')
                assert(isequal(double(oldPacket.reference_version), ...
                    double(obj.active_reference_version)), ...
                    '[R20] active obstacle packet version is stale.');
            end

            pack=oldPacket;
            pack.reference_mode='REPAIRED';
            pack.reference_version=double(targetVersion);
            pack.sweep_certified=true;
            pack.sweep_dt=double(sweep.dt);
            pack.delta_payload_m=double(sweep.payload_m);
            pack.delta_uav_m=double(sweep.uav_m);
            pack.delta_cable_m=double(sweep.cable_m);
            Dp=double(obj.r_load)+double(obj.d_margin)+double(sweep.payload_m);
            Dq=double(obj.r_drone)+double(obj.d_margin)+double(sweep.uav_m);
            Dc=double(obj.r_cable)+double(obj.d_margin)+double(sweep.cable_m);
            for i=1:M
                o=obj.static_obstacle_map(i);
                ar=double(pack.Araw(:,i));
                [pack.Apay(:,i),pack.Auav(:,i),pack.Acable(:,i)]= ...
                    obj.outer_ellipsoid_triplet_from_raw_cached(o,ar,Dp,Dq,Dc);
                if isfield(o,'v_center') && norm(o.v_center(:))>0
                    error('[REPLANNER ERROR] moving obstacle entered static fast packet path.');
                end
            end

            H=double(obj.horizon_s);
            Rs=double(obj.trigger_dist);
            rho=max([double(obj.r_drone),double(obj.L_cable)+double(obj.r_load), ...
                     double(obj.L_cable)+double(obj.r_cable)]);
            horizonMargin=Rs-(rho+VQ*H);
            dDetLB=Rs-VQ*dt;
            safeClear=dDetLB-rho;
            if VQ>0
                collisionLead=safeClear/VQ;
                beforeHorizon=collisionLead-H;
            else
                collisionLead=Inf;
                beforeHorizon=Inf;
            end
            sensor=struct('certified',horizonMargin>0&&safeClear>0&&beforeHorizon>0, ...
                'reference_mode','REPAIRED','reference_version',double(targetVersion), ...
                'VL',VL,'VQ',VQ,'dt',dt,'sensor_range',Rs,'horizon',H,'rho_body',rho, ...
                'sensor_horizon_margin_m',horizonMargin, ...
                'first_detection_distance_LB_m',dDetLB, ...
                'first_collision_lead_LB_s',collisionLead, ...
                'before_horizon_entry_LB_s',beforeHorizon, ...
                'source','LIVE_SPEED_SENSOR_GEOMETRY_THEOREM');
            assert(logical(sensor.certified), ...
                '[REPLANNER ERROR] fast target sensor horizon is not closed.');

            dep=obj.runtime_dependency_signature;
            dep.reference_mode='REPAIRED';
            dep.reference_version=double(targetVersion);
        end
    end
end
