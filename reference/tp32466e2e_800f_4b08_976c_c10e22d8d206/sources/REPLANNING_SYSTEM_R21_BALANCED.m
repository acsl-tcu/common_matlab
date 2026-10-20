classdef REPLANNING_SYSTEM_R21_BALANCED < REPLANNING_SYSTEM_R19_FAST
% AF7R58ZZY-R21 | THEORY-PRESERVING STAGE2/STAGE3 LOAD BALANCE
%
% R21 intentionally returns to the validated R19 numerical implementation.
% It changes only WHEN the non-authority post-commit reference cache is built:
%   Stage2: close all continuous certificates, resource/batch model, runtime
%           sweep/obstacle/sensor packet, evaluator handles, and immutable
%           reference metadata exactly as R19.
%   Stage3: after the same scheduled activation edge is reached, build the
%           first repaired {pL,q} cache for t_commit+dt, then call the exact
%           inherited atomic activation routine.
%
% The cache is not used to accept the trajectory or to close continuous-time
% geometry/C7/speed safety. Any cache/materialization/deadline failure leaves
% the current authority unchanged. Activation remains trigger+2*dt.

    properties (SetAccess = private)
        r21_build_id = 'AF7R58ZZY_R21_BALANCED_V1';
        r21_last_prepare_total_ms = NaN;
        r21_last_batch_model_ms = NaN;
        r21_last_packet_ms = NaN;
        r21_last_handles_ms = NaN;
        r21_last_bundle_core_ms = NaN;
        r21_last_stage3_postcache_ms = NaN;
        r21_last_stage3_parent_activation_ms = NaN;
        r21_last_postcache_table_used = false;
        r21_last_deferred_cache_used = false;
    end

    methods
        function obj = REPLANNING_SYSTEM_R21_BALANCED(self,base_ref,opts)
            obj@REPLANNING_SYSTEM_R19_FAST(self,base_ref,opts);
            assert(exist('af7r75_correction_posaccel_mex','file')==3, ...
                '[R21] R19 correction MEX missing. Run AF7R58ZZY_R21_INSTALL first.');
            assert(exist('af7r75_posaccel_batch_R19','file')==2, ...
                '[R21] R19 exact postcache helper missing.');
        end

        function R = get_r21_performance_status(obj)
            R=struct( ...
                'prepare_total_ms',double(obj.r21_last_prepare_total_ms), ...
                'batch_model_ms',double(obj.r21_last_batch_model_ms), ...
                'packet_ms',double(obj.r21_last_packet_ms), ...
                'handles_ms',double(obj.r21_last_handles_ms), ...
                'bundle_core_ms',double(obj.r21_last_bundle_core_ms), ...
                'stage3_postcache_ms',double(obj.r21_last_stage3_postcache_ms), ...
                'stage3_parent_activation_ms',double(obj.r21_last_stage3_parent_activation_ms), ...
                'postcache_table_used',logical(obj.r21_last_postcache_table_used), ...
                'deferred_cache_used',logical(obj.r21_last_deferred_cache_used));
            vals=[R.batch_model_ms R.packet_ms R.handles_ms R.bundle_core_ms];
            R.stage2_measured_subtotal_ms=sum(vals(isfinite(vals)));
            R.stage2_residual_ms=R.prepare_total_ms-R.stage2_measured_subtotal_ms;
        end

        function [ok,reason,B] = prepare_staged_reference_only_bundle(obj,P,t_commit)
            % Exact R19 Stage2 authority closure, except the performance-only
            % first post-commit reference cache is deferred to Stage3.
            ok=false;
            reason='';
            B=struct();
            tAll=tic;
            obj.r21_last_prepare_total_ms=NaN;
            obj.r21_last_batch_model_ms=NaN;
            obj.r21_last_packet_ms=NaN;
            obj.r21_last_handles_ms=NaN;
            obj.r21_last_bundle_core_ms=NaN;
            obj.r21_last_stage3_postcache_ms=NaN;
            obj.r21_last_stage3_parent_activation_ms=NaN;
            obj.r21_last_postcache_table_used=false;
            obj.r21_last_deferred_cache_used=false;

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

            tx=tic;
            [resourceOK,resourceReason,batchModel,resourceMeta]= ...
                af7r58i_prepare_batch_model( ...
                    obj,P,targetVersion,double(t_commit));
            obj.r21_last_batch_model_ms=1e3*toc(tx);
            if ~resourceOK
                reason=resourceReason; return;
            end

            tx=tic;
            reuseRuntimePacket= ...
                obj.can_reuse_reference_only_runtime_packet_exact( ...
                    obj.active_speed_certificate,SC, ...
                    obj.trigger_sweep_inflation,obj.obstacle_packet, ...
                    obj.sensor_horizon_certificate,obj.runtime_dependency_signature);
            if reuseRuntimePacket
                [sweepNew,packetNew,sensorNew,depNew]= ...
                    obj.reuse_reference_only_runtime_packet_exact( ...
                        obj.trigger_sweep_inflation,obj.obstacle_packet, ...
                        obj.sensor_horizon_certificate,obj.runtime_dependency_signature, ...
                        SC,targetVersion);
            else
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
            obj.r21_last_packet_ms=1e3*toc(tx);

            tx=tic;
            [evalFn,flatEvalFn]=af7r58i_make_canonical_eval_handles(obj,batchModel);
            obj.r21_last_handles_ms=1e3*toc(tx);

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

            % Deliberately no 161-point post-commit cache here. It is not an
            % acceptance certificate. Stage3 materializes it before calling
            % the unchanged inherited atomic activation routine.
            tx=tic;
            B=struct( ...
                'schema','AF7R58ZZY_R21_PREPARED_CORE_V1', ...
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
                'post_commit_cache_pending',true, ...
                'prevalidated',true);
            obj.r21_last_bundle_core_ms=1e3*toc(tx);
            obj.r21_last_prepare_total_ms=1e3*toc(tAll);
            ok=true;
            reason='STAGED_COMMIT_CORE_PREPARED__POSTCACHE_DEFERRED_TO_STAGE3';
        end

        function [ok,reason] = activate_staged_reference_only_bundle(obj,B,t_now,t_hard_cycle)
            % R21 Stage3: materialize only the deferred non-authority cache,
            % then execute the exact production atomic commit semantics.
            ok=false;
            reason='';
            obj.r21_last_stage3_postcache_ms=NaN;
            obj.r21_last_stage3_parent_activation_ms=NaN;
            obj.r21_last_postcache_table_used=false;
            obj.r21_last_deferred_cache_used=false;

            if ~(isstruct(B)&&isfield(B,'schema')&& ...
                    strcmp(char(B.schema),'AF7R58ZZY_R21_PREPARED_CORE_V1') && ...
                    isfield(B,'prevalidated')&&logical(B.prevalidated) && ...
                    isfield(B,'post_commit_cache_pending')&&logical(B.post_commit_cache_pending) && ...
                    isfield(B,'reference')&&isstruct(B.reference) && ...
                    isfield(B.reference,'batch_flat_model'))
                reason='R21_DEFERRED_BUNDLE_SCHEMA_MISMATCH';
                return;
            end

            % Same scheduled activation grid as production. No pipeline delay
            % is added by R21.
            scale=max([1,abs(double(t_now)),abs(double(B.activation_time))]);
            if abs(double(t_now)-double(B.activation_time))>64*eps(scale)
                reason='STAGED_ACTIVATION_TIME_MISMATCH';
                return;
            end

            % Materialize the performance cache before authority mutation.
            % This operation is not used to accept the candidate. Failure is
            % fail-closed: the current active reference remains untouched.
            tc=tic;
            try
                dt=double(obj.prediction_dt);
                H=double(obj.horizon_s);
                N=round(H/dt)+1;
                tCache=double(t_now)+dt;
                tqCache=tCache+(0:N-1)*dt;
                [pLcache,aLcache,validCache,tableUsed]= ...
                    af7r75_posaccel_batch_R19( ...
                        obj,tqCache,B.reference.batch_flat_model);
                if ~(isequal(size(pLcache),[3 N]) && isequal(size(aLcache),[3 N]) && ...
                        numel(validCache)==N && all(logical(validCache(:))) && ...
                        all(isfinite(pLcache),'all') && all(isfinite(aLcache),'all'))
                    reason='STAGED_POSTCOMMIT_CACHE_BATCH_INVALID';
                    obj.r21_last_stage3_postcache_ms=1e3*toc(tc);
                    return;
                end
                sfcache=aLcache+[0;0;double(obj.gravity)];
                nscache=sqrt(sum(sfcache.^2,1));
                if ~(all(isfinite(nscache))&&all(nscache>0))
                    reason='STAGED_POSTCOMMIT_CACHE_FLATNESS_SINGULAR';
                    obj.r21_last_stage3_postcache_ms=1e3*toc(tc);
                    return;
                end
                qcache=-sfcache./nscache;
                postCache=struct( ...
                    'reference_version',double(B.target_version), ...
                    't0',double(tCache),'dt',dt,'H',H, ...
                    'pL',pLcache,'q',qcache);
                obj.r21_last_postcache_table_used=logical(tableUsed);
                obj.r21_last_stage3_postcache_ms=1e3*toc(tc);
            catch ME
                obj.r21_last_stage3_postcache_ms=1e3*toc(tc);
                reason=ME.message;
                return;
            end

            % If the real control period is already exhausted, do not even
            % enter the atomic swap. This is stricter than relying on rollback.
            if toc(t_hard_cycle)>double(obj.prediction_dt)
                obj.generator_deadline_fail_count= ...
                    obj.generator_deadline_fail_count+1;
                reason='STAGED_ATOMIC_CONTROL_PERIOD_EXCEEDED';
                return;
            end

            Bfull=B;
            Bfull.schema='AF7R58ZZV_PREPARED_COMMIT_V2';
            Bfull.post_commit_reference_cache=postCache;
            Bfull=rmfield(Bfull,'post_commit_cache_pending');
            obj.r21_last_deferred_cache_used=true;
            obj.staged_replan_postcommit_cache_prepare_count= ...
                obj.staged_replan_postcommit_cache_prepare_count+1;

            ta=tic;

            % Exact production freshness gates, now applied immediately after
            % cache materialization and immediately before authority mutation.
            if ~strcmp(char(obj.active_reference_mode),char(Bfull.source_mode)) || ...
                    double(obj.active_reference_version)~=double(Bfull.source_version)
                reason='STAGED_SOURCE_REFERENCE_STALE';
                obj.r21_last_stage3_parent_activation_ms=1e3*toc(ta);
                return;
            end
            if ~isequaln(obj.staged_reference_only_dependency_key(),Bfull.dependency_key) || ...
                    ~isequaln(obj.active_speed_certificate,Bfull.speed_source) || ...
                    ~isequaln(obj.static_obstacle_map,Bfull.static_map)
                reason='STAGED_DEPENDENCY_OR_MAP_STALE';
                obj.r21_last_stage3_parent_activation_ms=1e3*toc(ta);
                return;
            end

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

            try
                obj.active_repaired_reference=Bfull.reference;
                obj.active_repaired_certificates=Bfull.certificates;
                obj.active_reference_mode='REPAIRED';
                obj.active_reference_version=double(Bfull.target_version);
                obj.active_repaired_safety_level='REFERENCE_ONLY';
                obj.active_speed_certificate=Bfull.speed;
                obj.trigger_sweep_inflation=Bfull.sweep;
                obj.obstacle_packet=Bfull.packet;
                obj.sensor_horizon_certificate=Bfull.sensor;
                obj.runtime_dependency_signature=Bfull.dep;
                obj.certificate_stale=obj.empty_certificate_stale_state();
                obj.certificate_stale.ACTUATOR_DYNAMICS_CERT=true;
                obj.certificate_stale.HLC_ROBUST_CERT=true;
                obj.last_replan_commit_time=double(t_now);

                % Identical production hot certificate/version assertion.
                obj.assert_runtime_core_certificates_current_hot();

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
                    obj.r21_last_stage3_parent_activation_ms=1e3*toc(ta);
                    return;
                end

                Cpost=Bfull.post_commit_reference_cache;
                assert(isstruct(Cpost) && ...
                    double(Cpost.reference_version)==double(Bfull.target_version) && ...
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
                    'reference_version',double(Bfull.target_version), ...
                    'backend','BEZIER_D15_C7', ...
                    'commit_time',double(t_now), ...
                    'rejoin_time',double(Bfull.reference.rejoin_time), ...
                    'staged_pipeline',true, ...
                    'pipeline_intervals',2);

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
            obj.r21_last_stage3_parent_activation_ms=1e3*toc(ta);
        end
    end
end
