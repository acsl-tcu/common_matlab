classdef REPLANNING_SYSTEM_R10_DIAG < REPLANNING_SYSTEM_R8R2_DIAG
% AF7R58ZZY-R10 Stage2 direct exact-MEX connection shadow.
%
% Performance-only diagnostic subclass.  It preserves the R8-R2 observer and
% reference instrumentation and overrides ONLY the staged REFERENCE_ONLY bundle
% preparation method.  The production-equation method body is copied from the
% source-bound R5-R4 diagnostic; the sole computational substitution is the
% 161-point post-commit cache batch call:
%   af7r33_flat_jet_batch_model -> af7r33_flat_jet_batch_model_R10FAST
% The R10FAST helper is generated at audit runtime from the exact current
% AF7R33 source and is allowed only after full 21xN bitwise equivalence closes.

    properties (Access = public)
        af7r63_stage2_fast_calls = 0;
        af7r63_stage2_fast_real_calls = 0;
    end

    methods
        function obj = REPLANNING_SYSTEM_R10_DIAG(self,base_ref,opts)
            obj@REPLANNING_SYSTEM_R8R2_DIAG(self,base_ref,opts);
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
            obj.af7r63_stage2_fast_calls=obj.af7r63_stage2_fast_calls+1;
            if ~obj.setup_exact_do_prime_active && ~obj.generator_integration_prime_active
                obj.af7r63_stage2_fast_real_calls=obj.af7r63_stage2_fast_real_calls+1;
            end
            [Ycache,validCache]= ...
                af7r33_flat_jet_batch_model_R10FAST(obj,tqCache,batchModel);
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

    end
end
