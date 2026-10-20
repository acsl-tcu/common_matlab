classdef REPLANNING_SYSTEM_R18_STAGE2_DIAG < REPLANNING_SYSTEM_R17_FAST
% AF7R58ZZY-R18 | Stage2 fine-grain timing only.
% Diagnostic subclass of the already measured R17 lean implementation.
% No safety/certificate/trajectory/controller semantics are changed.

    properties (SetAccess = private)
        r18_build_id = 'AF7R58ZZY_R18_STAGE2_DIAG_V1';
        r18_last = struct();
    end

    methods
        function obj = REPLANNING_SYSTEM_R18_STAGE2_DIAG(self,base_ref,opts)
            obj@REPLANNING_SYSTEM_R17_FAST(self,base_ref,opts);
            obj.r18_last=obj.empty_r18_status();
        end

        function R=get_r18_stage2_status(obj)
            R=obj.r18_last;
            R.parent_stage2_ms=double(obj.staged_replan_last_stage2_ms);
            if isfinite(R.parent_stage2_ms) && isfinite(R.prepare_total_ms)
                R.stage2_wrapper_ms=R.parent_stage2_ms-R.prepare_total_ms;
            else
                R.stage2_wrapper_ms=NaN;
            end
        end

        function [ok,reason,B] = prepare_staged_reference_only_bundle(obj,P,t_commit)
            ok=false; reason=''; B=struct();
            Z=obj.empty_r18_status();
            totalTic=tic;

            tx=tic;
            if ~obj.ENABLE_AUTO_REPLAN_COMMIT
                reason='AUTO_COMMIT_DISABLED'; Z.precheck_ms=1e3*toc(tx); obj.finish_r18(Z,totalTic); return;
            end
            if ~strcmpi(char(obj.AVOIDANCE_COMMIT_POLICY),'REFERENCE_ONLY')
                reason='NOT_REFERENCE_ONLY_COMMIT_POLICY'; Z.precheck_ms=1e3*toc(tx); obj.finish_r18(Z,totalTic); return;
            end
            if ~(isstruct(P)&&isfield(P,'success')&&logical(P.success))
                reason='PROPOSAL_NOT_SUCCESSFUL'; Z.precheck_ms=1e3*toc(tx); obj.finish_r18(Z,totalTic); return;
            end
            [spacingOK,~,~,spacingReason]=af7r58i_commit_spacing_guard(obj,double(t_commit));
            if ~spacingOK
                reason=spacingReason; Z.precheck_ms=1e3*toc(tx); obj.finish_r18(Z,totalTic); return;
            end
            G=P.geometry_shadow_certificate;
            SC=P.reference_speed_certificate;
            if ~(isstruct(G)&&isfield(G,'certified')&&logical(G.certified) && ...
                    isfield(G,'continuous_time')&&logical(G.continuous_time) && ...
                    isfield(G,'sampled_acceptance')&&~logical(G.sampled_acceptance))
                reason='REFERENCE_GEOMETRY_CERTIFICATE_NOT_CLOSED'; Z.precheck_ms=1e3*toc(tx); obj.finish_r18(Z,totalTic); return;
            end
            if ~(isstruct(SC)&&isfield(SC,'certified')&&logical(SC.certified))
                reason='REFERENCE_SPEED_CERTIFICATE_NOT_CLOSED'; Z.precheck_ms=1e3*toc(tx); obj.finish_r18(Z,totalTic); return;
            end
            targetVersion=double(obj.active_reference_version)+1;
            if double(SC.reference_version)~=targetVersion
                reason='REFERENCE_SPEED_CERTIFICATE_VERSION_MISMATCH'; Z.precheck_ms=1e3*toc(tx); obj.finish_r18(Z,totalTic); return;
            end
            if ~(isfield(SC,'continuous_time')&&logical(SC.continuous_time) && ...
                    isfield(SC,'sampled_acceptance')&&~logical(SC.sampled_acceptance) && ...
                    isfield(SC,'specific_force_norm_LB_m_s2') && ...
                    isnumeric(SC.specific_force_norm_LB_m_s2) && isscalar(SC.specific_force_norm_LB_m_s2) && ...
                    isfinite(double(SC.specific_force_norm_LB_m_s2)) && ...
                    double(SC.specific_force_norm_LB_m_s2)>0)
                reason='REFERENCE_ONLY_CONTINUOUS_FLATNESS_CERTIFICATE_NOT_CLOSED'; Z.precheck_ms=1e3*toc(tx); obj.finish_r18(Z,totalTic); return;
            end
            if ~(isfield(P,'c7_exact')&&logical(P.c7_exact))
                reason='REFERENCE_ONLY_C7_CERTIFICATE_NOT_CLOSED'; Z.precheck_ms=1e3*toc(tx); obj.finish_r18(Z,totalTic); return;
            end
            Z.precheck_ms=1e3*toc(tx);

            tx=tic;
            [resourceOK,resourceReason,batchModel,resourceMeta]= ...
                af7r58i_prepare_batch_model(obj,P,targetVersion,double(t_commit));
            Z.batch_model_ms=1e3*toc(tx);
            if ~resourceOK
                reason=resourceReason; obj.finish_r18(Z,totalTic); return;
            end

            tx=tic;
            reuseRuntimePacket=obj.can_reuse_reference_only_runtime_packet_exact( ...
                obj.active_speed_certificate,SC,obj.trigger_sweep_inflation,obj.obstacle_packet, ...
                obj.sensor_horizon_certificate,obj.runtime_dependency_signature);
            Z.packet_reuse_gate_ms=1e3*toc(tx);
            Z.runtime_packet_reuse=logical(reuseRuntimePacket);

            if reuseRuntimePacket
                tx=tic;
                [sweepNew,packetNew,sensorNew,depNew]=obj.reuse_reference_only_runtime_packet_exact( ...
                    obj.trigger_sweep_inflation,obj.obstacle_packet,obj.sensor_horizon_certificate, ...
                    obj.runtime_dependency_signature,SC,targetVersion);
                Z.packet_reuse_apply_ms=1e3*toc(tx);
            else
                tx=tic;
                sweepNew=obj.build_trigger_sweep_for_target(SC,targetVersion);
                Z.packet_sweep_ms=1e3*toc(tx);

                tx=tic;
                packetNew=obj.prepare_obstacle_packet_fast_target(obj.obstacle_packet,sweepNew,targetVersion);
                Z.packet_obstacle_ms=1e3*toc(tx);

                tx=tic;
                sensorNew=obj.build_sensor_horizon_for_target(SC,targetVersion);
                Z.packet_sensor_ms=1e3*toc(tx);
                if ~(isstruct(sensorNew)&&isfield(sensorNew,'certified')&&logical(sensorNew.certified))
                    reason='STAGED_TARGET_SENSOR_HORIZON_OPEN'; obj.finish_r18(Z,totalTic); return;
                end

                tx=tic;
                depNew=obj.runtime_dependency_signature;
                depNew.reference_mode='REPAIRED';
                depNew.reference_version=double(targetVersion);
                Z.packet_dep_ms=1e3*toc(tx);
            end

            tx=tic;
            [evalFn,flatEvalFn]=af7r58i_make_canonical_eval_handles(obj,batchModel);
            Z.handles_ms=1e3*toc(tx);

            tx=tic;
            smoothSource='DEGREE15_ENDPOINT_REPEATED_BEZIER_C7';
            if isfield(P,'representation_code')&&double(P.representation_code)==2
                smoothSource='PIECEWISE_DEGREE15_BEZIER_C7_AFFINE_TOWING_CORRIDORS';
            end
            smooth=struct('certified',logical(P.c7_exact),'continuous_time',true, ...
                'sampled_acceptance',false,'continuity_order',7,'reference_mode','REPAIRED', ...
                'reference_version',targetVersion,'valid_from',double(P.t0),'valid_until',double(P.t1), ...
                'source',smoothSource);
            geom=G;
            geom.reference_mode='REPAIRED'; geom.reference_version=targetVersion;
            geom.valid_from=double(P.t0); geom.valid_until=double(P.t1);
            geom.reference_only_authority=true; geom.final_system_authority=false;
            R=struct();
            R.valid=true; R.backend='BEZIER_D15_C7'; R.eval_fn=evalFn; R.flat_eval_fn=flatEvalFn;
            R.valid_from=double(P.t0); R.valid_until=Inf; R.reference_version=targetVersion;
            R.source='AF7R58ZZU_STAGED_REFERENCE_ONLY_COMMIT'; R.yaw_policy='C6_INDEPENDENT';
            R.path_parameterization=struct('kind','DIRECT_TIME_BEZIER_CORRECTION', ...
                'path_derivative_order',7,'time_derivative_order',7);
            R.rejoin_time=double(P.t1);
            R.representation=struct('kind','BEZIER_D15_C7_TRANSLATION_BUMP', ...
                't0',double(P.t0),'tm',double(P.tm),'t1',double(P.t1), ...
                'displacement',double(P.displacement(:)));
            R.batch_flat_model=batchModel; R.resource_certificate=resourceMeta.contract;
            R.resource_state=resourceMeta;
            R.authority_split=struct('controller_packet','eval_fn', ...
                'mathematical_flat_jet','flat_eval_fn','legacy_high_derivative_slots_used_for_geometry',false);
            Z.reference_metadata_ms=1e3*toc(tx);

            tx=tic;
            dt=double(obj.prediction_dt); H=double(obj.horizon_s); N=round(H/dt)+1;
            tCache=double(t_commit)+dt; tqCache=tCache+(0:N-1)*dt;
            Z.postcache_grid_ms=1e3*toc(tx);

            tx=tic;
            [pLcache,aLcache,validCache,tableUsed,ZT]=af7r74_posaccel_batch_R18(obj,tqCache,batchModel);
            Z.postcache_eval_ms=1e3*toc(tx);
            Z.postcache_eval=ZT;
            Z.postcache_table_used=logical(tableUsed);

            tx=tic;
            if ~(isequal(size(pLcache),[3 N]) && isequal(size(aLcache),[3 N]) && ...
                    numel(validCache)==N && all(logical(validCache(:))) && ...
                    all(isfinite(pLcache),'all') && all(isfinite(aLcache),'all'))
                reason='STAGED_POSTCOMMIT_CACHE_BATCH_INVALID'; Z.postcache_validate_ms=1e3*toc(tx); obj.finish_r18(Z,totalTic); return;
            end
            Z.postcache_validate_ms=1e3*toc(tx);

            tx=tic;
            sfcache=aLcache+[0;0;double(obj.gravity)];
            nscache=sqrt(sum(sfcache.^2,1));
            if ~(all(isfinite(nscache))&&all(nscache>0))
                reason='STAGED_POSTCOMMIT_CACHE_FLATNESS_SINGULAR'; Z.postcache_q_ms=1e3*toc(tx); obj.finish_r18(Z,totalTic); return;
            end
            qcache=-sfcache./nscache;
            Z.postcache_q_ms=1e3*toc(tx);

            tx=tic;
            postCache=struct('reference_version',double(targetVersion),'t0',double(tCache), ...
                'dt',dt,'H',H,'pL',pLcache,'q',qcache);
            Z.postcache_struct_ms=1e3*toc(tx);

            tx=tic;
            B=struct('schema','AF7R58ZZV_PREPARED_COMMIT_V2', ...
                'source_mode',char(obj.active_reference_mode),'source_version',double(obj.active_reference_version), ...
                'target_version',double(targetVersion),'activation_time',double(t_commit), ...
                'dependency_key',obj.staged_replan_dependency_key,'speed_source',obj.staged_replan_speed_source, ...
                'known_mask',logical(obj.staged_replan_known_mask(:).'),'static_map',obj.staged_replan_static_map, ...
                'reference',R,'certificates',struct('reference_only',true,'smoothness',smooth,'geometry',geom), ...
                'speed',SC,'sweep',sweepNew,'packet',packetNew,'sensor',sensorNew,'dep',depNew, ...
                'post_commit_reference_cache',postCache,'prevalidated',true);
            Z.bundle_ms=1e3*toc(tx);

            tx=tic;
            obj.staged_replan_postcommit_cache_prepare_count=obj.staged_replan_postcommit_cache_prepare_count+1;
            Z.bookkeeping_ms=1e3*toc(tx);
            ok=true; reason='STAGED_COMMIT_BUNDLE_AND_POSTCACHE_PREPARED';
            obj.finish_r18(Z,totalTic);
        end
    end

    methods (Access = private)
        function finish_r18(obj,Z,totalTic)
            Z.prepare_total_ms=1e3*toc(totalTic);
            vals=[Z.precheck_ms Z.batch_model_ms Z.packet_reuse_gate_ms Z.packet_reuse_apply_ms ...
                Z.packet_sweep_ms Z.packet_obstacle_ms Z.packet_sensor_ms Z.packet_dep_ms ...
                Z.handles_ms Z.reference_metadata_ms Z.postcache_grid_ms Z.postcache_eval_ms ...
                Z.postcache_validate_ms Z.postcache_q_ms Z.postcache_struct_ms Z.bundle_ms Z.bookkeeping_ms];
            Z.accounted_ms=sum(vals(isfinite(vals)));
            Z.prepare_residual_ms=Z.prepare_total_ms-Z.accounted_ms;
            obj.r18_last=Z;
        end

        function Z=empty_r18_status(~)
            Z=struct('prepare_total_ms',NaN,'accounted_ms',NaN,'prepare_residual_ms',NaN, ...
                'precheck_ms',NaN,'batch_model_ms',NaN,'packet_reuse_gate_ms',NaN, ...
                'packet_reuse_apply_ms',NaN,'packet_sweep_ms',NaN,'packet_obstacle_ms',NaN, ...
                'packet_sensor_ms',NaN,'packet_dep_ms',NaN,'handles_ms',NaN, ...
                'reference_metadata_ms',NaN,'postcache_grid_ms',NaN,'postcache_eval_ms',NaN, ...
                'postcache_validate_ms',NaN,'postcache_q_ms',NaN,'postcache_struct_ms',NaN, ...
                'bundle_ms',NaN,'bookkeeping_ms',NaN,'runtime_packet_reuse',false, ...
                'postcache_table_used',false,'postcache_eval',struct());
        end
    end
end
