classdef REPLANNING_SYSTEM_R22_ACCOUNTING_DIAG < REPLANNING_SYSTEM_R21_BALANCED
% AF7R58ZZY-R22 | ACCOUNTING-ONLY DIAGNOSTIC
% No trajectory/certificate/commit-policy change. R21 behavior is copied
% exactly in overridden staged methods; tic/toc scalars are the only added work.

    properties (SetAccess = private)
        r22_build_id = 'AF7R58ZZY_R22_ACCOUNTING_DIAG_V1';

        r22_refcert_ms = NaN;

        r22_s1_method_total_ms = NaN;
        r22_s1_future_packet_ms = NaN;
        r22_s1_context_ms = NaN;
        r22_s1_native_wall_ms = NaN;
        r22_s1_native_internal_ms = NaN;
        r22_s1_recorded_stage_ms = NaN;
        r22_s1_post_recorded_ms = NaN;
        r22_s1_residual_ms = NaN;

        r22_s2_prepare_total_ms = NaN;
        r22_s2_guard_cert_ms = NaN;
        r22_s2_batch_model_ms = NaN;
        r22_s2_packet_reuse_gate_ms = NaN;
        r22_s2_packet_action_ms = NaN;
        r22_s2_handles_ms = NaN;
        r22_s2_metadata_ms = NaN;
        r22_s2_bundle_ms = NaN;
        r22_s2_residual_ms = NaN;
        r22_s2_runtime_packet_reuse = false;

        r22_s3_total_ms = NaN;
        r22_s3_guard_ms = NaN;
        r22_s3_grid_ms = NaN;
        r22_s3_eval_ms = NaN;
        r22_s3_validate_q_ms = NaN;
        r22_s3_cache_struct_ms = NaN;
        r22_s3_predeadline_ms = NaN;
        r22_s3_bfull_ms = NaN;
        r22_s3_freshness_ms = NaN;
        r22_s3_snapshot_ms = NaN;
        r22_s3_authority_assign_ms = NaN;
        r22_s3_hot_cert_ms = NaN;
        r22_s3_deadline_ms = NaN;
        r22_s3_cache_install_ms = NaN;
        r22_s3_status_ms = NaN;
        r22_s3_residual_ms = NaN;
        r22_s3_table_used = false;
    end

    methods
        function obj = REPLANNING_SYSTEM_R22_ACCOUNTING_DIAG(self,base_ref,opts)
            obj@REPLANNING_SYSTEM_R21_BALANCED(self,base_ref,opts);
        end

        function D = get_r22_stage_status(obj)
            D=struct();
            D.build_id=obj.r22_build_id;
            D.reference_certificate_ms=double(obj.r22_refcert_ms);
            D.stage1=struct( ...
                'method_total_ms',double(obj.r22_s1_method_total_ms), ...
                'future_packet_ms',double(obj.r22_s1_future_packet_ms), ...
                'context_ms',double(obj.r22_s1_context_ms), ...
                'native_wall_ms',double(obj.r22_s1_native_wall_ms), ...
                'native_internal_ms',double(obj.r22_s1_native_internal_ms), ...
                'recorded_stage_ms',double(obj.r22_s1_recorded_stage_ms), ...
                'post_recorded_ms',double(obj.r22_s1_post_recorded_ms), ...
                'residual_ms',double(obj.r22_s1_residual_ms));
            D.stage2=struct( ...
                'prepare_total_ms',double(obj.r22_s2_prepare_total_ms), ...
                'guard_cert_ms',double(obj.r22_s2_guard_cert_ms), ...
                'batch_model_ms',double(obj.r22_s2_batch_model_ms), ...
                'packet_reuse_gate_ms',double(obj.r22_s2_packet_reuse_gate_ms), ...
                'packet_action_ms',double(obj.r22_s2_packet_action_ms), ...
                'handles_ms',double(obj.r22_s2_handles_ms), ...
                'metadata_ms',double(obj.r22_s2_metadata_ms), ...
                'bundle_ms',double(obj.r22_s2_bundle_ms), ...
                'residual_ms',double(obj.r22_s2_residual_ms), ...
                'runtime_packet_reuse',logical(obj.r22_s2_runtime_packet_reuse));
            D.stage3=struct( ...
                'total_ms',double(obj.r22_s3_total_ms), ...
                'guard_ms',double(obj.r22_s3_guard_ms), ...
                'grid_ms',double(obj.r22_s3_grid_ms), ...
                'eval_ms',double(obj.r22_s3_eval_ms), ...
                'validate_q_ms',double(obj.r22_s3_validate_q_ms), ...
                'cache_struct_ms',double(obj.r22_s3_cache_struct_ms), ...
                'predeadline_ms',double(obj.r22_s3_predeadline_ms), ...
                'bfull_ms',double(obj.r22_s3_bfull_ms), ...
                'freshness_ms',double(obj.r22_s3_freshness_ms), ...
                'snapshot_ms',double(obj.r22_s3_snapshot_ms), ...
                'authority_assign_ms',double(obj.r22_s3_authority_assign_ms), ...
                'hot_cert_ms',double(obj.r22_s3_hot_cert_ms), ...
                'deadline_ms',double(obj.r22_s3_deadline_ms), ...
                'cache_install_ms',double(obj.r22_s3_cache_install_ms), ...
                'status_ms',double(obj.r22_s3_status_ms), ...
                'residual_ms',double(obj.r22_s3_residual_ms), ...
                'table_used',logical(obj.r22_s3_table_used));
        end

        function R = evaluate_reference_only_proposal_certificate(obj,P)
            tt=tic;
            R=struct();
            R.schema='REPLANNING_REFERENCE_ONLY_EVALUATION_V1';
            R.certified=false;
            R.full_system_authority=false;
            R.sampled_acceptance=false;
            R.reason='PROPOSAL_NOT_SUCCESSFUL';
            R.reference_version=double(obj.active_reference_version)+1;
            R.geometry_certificate=struct();
            R.speed_certificate=struct();
            R.proposal_generated=false;
            R.proposal_success=false;
            R.proposal_reason_code=NaN;
            R.proposal_reason='UNKNOWN';
            R.proposal_t0=NaN;
            R.proposal_tm=NaN;
            R.proposal_t1=NaN;
            if isstruct(P)
                if isfield(P,'generated'), R.proposal_generated=logical(P.generated); end
                if isfield(P,'success'), R.proposal_success=logical(P.success); end
                if isfield(P,'reason_code')
                    R.proposal_reason_code=double(P.reason_code);
                    R.proposal_reason=obj.bezier_proposal_reason_text(P.reason_code);
                end
                if isfield(P,'t0'), R.proposal_t0=double(P.t0); end
                if isfield(P,'tm'), R.proposal_tm=double(P.tm); end
                if isfield(P,'t1'), R.proposal_t1=double(P.t1); end
            end
            if ~(isstruct(P) && isfield(P,'success') && logical(P.success))
                R.reason=['PROPOSAL_NOT_SUCCESSFUL__' R.proposal_reason];
                obj.r22_refcert_ms=1e3*toc(tt); return;
            end
            if isfield(P,'geometry_shadow_certificate'), R.geometry_certificate=P.geometry_shadow_certificate; end
            if isfield(P,'reference_speed_certificate'), R.speed_certificate=P.reference_speed_certificate; end
            G=R.geometry_certificate; SC=R.speed_certificate;
            if ~(isstruct(G) && isfield(G,'certified') && logical(G.certified) && ...
                    isfield(G,'continuous_time') && logical(G.continuous_time) && ...
                    isfield(G,'sampled_acceptance') && ~logical(G.sampled_acceptance))
                R.reason='REFERENCE_GEOMETRY_CERTIFICATE_NOT_CLOSED';
                obj.r22_refcert_ms=1e3*toc(tt); return;
            end
            if ~(isstruct(SC) && isfield(SC,'certified') && logical(SC.certified))
                R.reason='REFERENCE_SPEED_CERTIFICATE_NOT_CLOSED';
                obj.r22_refcert_ms=1e3*toc(tt); return;
            end
            if double(SC.reference_version)~=R.reference_version
                R.reason='REFERENCE_SPEED_CERTIFICATE_VERSION_MISMATCH';
                obj.r22_refcert_ms=1e3*toc(tt); return;
            end
            R.certified=true;
            R.reason='REFERENCE_ONLY_CERTIFICATE_CLOSED';
            obj.r22_refcert_ms=1e3*toc(tt);
        end

        function start_staged_replan_pipeline(obj,t_hard_cycle,time,cha,varargin_in, ...
                active,all_dists,ttc,collision_time,obs_id,comp_name,comp_code, ...
                pL_collision,q_collision,pQ_collision,jhit)
            tall=tic;
            obj.r22_s1_method_total_ms=NaN; obj.r22_s1_future_packet_ms=NaN;
            obj.r22_s1_context_ms=NaN; obj.r22_s1_native_wall_ms=NaN;
            obj.r22_s1_native_internal_ms=NaN; obj.r22_s1_recorded_stage_ms=NaN;
            obj.r22_s1_post_recorded_ms=NaN; obj.r22_s1_residual_ms=NaN;

            if ~obj.ENABLE_STAGED_REPLAN_PIPELINE, return; end
            assert(strcmpi(char(obj.AVOIDANCE_COMMIT_POLICY),'REFERENCE_ONLY'), ...
                '[AF7R58ZZU] staged pipeline currently supports REFERENCE_ONLY commit only.');
            assert(double(obj.generator_backend_locked_code)==1, ...
                '[AF7R58ZZU] staged pipeline requires current native BEZIER backend.');
            assert(isequal(double(obj.horizon_s),4.0), ...
                '[AF7R58ZZU] fixed 4-s collision horizon changed.');
            if ~strcmp(char(obj.staged_replan_state),'IDLE')
                obj.staged_replan_stale_reject_count=obj.staged_replan_stale_reject_count+1;
                obj.reset_staged_replan_pipeline('REQUEST_IDENTITY_CHANGED',false);
            end
            t_now=double(time.t); dt=double(obj.prediction_dt); ta=t_now+2*dt;
            if ~(isfinite(ta)&&isfinite(double(collision_time))&&double(collision_time)>ta)
                obj.staged_replan_fail_count=obj.staged_replan_fail_count+1;
                obj.generator_validation_fail_count=obj.generator_validation_fail_count+1;
                obj.generator_last_status_code=-60;
                obj.staged_replan_last_reason='INSUFFICIENT_LEAD_FOR_TWO_PERIOD_PIPELINE';
                obj.r22_s1_method_total_ms=1e3*toc(tall); return;
            end

            ts=tic;
            tx=tic;
            [pL_future,q_future,ref_source,xd_nom_a,xd_active_a]=obj.staged_reference_packet_at(ta,varargin_in);
            obj.r22_s1_future_packet_ms=1e3*toc(tx);

            tx=tic;
            pL_a=double(pL_future(:,1)); q_a=double(q_future(:,1));
            pQ_a=pL_a-double(obj.L_cable)*q_a; cable_residual_a=0;
            ttc_a=double(collision_time)-ta;
            jhit_a=round(ttc_a/dt)+1;
            jhit_a=max(1,min(size(pL_future,2),jhit_a));
            elapsed_before=toc(t_hard_cycle); budget_remaining=max(0,dt-elapsed_before);
            CTX=obj.build_generator_request_context_compact( ...
                ta,elapsed_before,budget_remaining,xd_nom_a,xd_active_a,pQ_a,pL_a,q_a,cable_residual_a, ...
                active,all_dists,pL_future,q_future,ref_source,ttc_a,double(collision_time),obs_id,comp_name,comp_code, ...
                pL_collision,q_collision,pQ_collision,jhit_a);
            obj.r22_s1_context_ms=1e3*toc(tx);

            tx=tic;
            try
                candidate=obj.af7r46_single_core_proposal_compact(CTX);
            catch ME
                obj.r22_s1_native_wall_ms=1e3*toc(tx);
                obj.staged_replan_fail_count=obj.staged_replan_fail_count+1;
                obj.generator_validation_fail_count=obj.generator_validation_fail_count+1;
                obj.generator_last_status_code=-61;
                obj.staged_replan_last_reason=['STAGE1_NATIVE_ERROR__' ME.identifier];
                obj.r22_s1_method_total_ms=1e3*toc(tall); return;
            end
            obj.r22_s1_native_wall_ms=1e3*toc(tx);

            stage_ms=1e3*toc(ts);
            obj.staged_replan_last_stage1_ms=stage_ms;
            obj.r22_s1_recorded_stage_ms=stage_ms;
            obj.generator_last_runtime_ms=double(candidate.r9_stage_timing.total_internal_ms);
            obj.r22_s1_native_internal_ms=obj.generator_last_runtime_ms;
            obj.generator_last_callback_ms=stage_ms;
            obj.generator_last_cycle_critical_ms=1e3*toc(t_hard_cycle);
            obj.generator_request_count=obj.generator_request_count+1;
            obj.staged_replan_stage1_count=obj.staged_replan_stage1_count+1;

            tpost=tic;
            if obj.generator_last_cycle_critical_ms>1e3*dt
                obj.generator_deadline_fail_count=obj.generator_deadline_fail_count+1;
                obj.generator_last_status_code=-62; obj.staged_replan_fail_count=obj.staged_replan_fail_count+1;
                obj.staged_replan_last_reason='STAGE1_CONTROL_PERIOD_EXCEEDED';
                obj.r22_s1_post_recorded_ms=1e3*toc(tpost); obj.r22_s1_method_total_ms=1e3*toc(tall); return;
            end
            if ~(isstruct(candidate)&&isfield(candidate,'success')&&logical(candidate.success))
                obj.generator_validation_fail_count=obj.generator_validation_fail_count+1;
                obj.generator_last_status_code=21; obj.staged_replan_fail_count=obj.staged_replan_fail_count+1;
                obj.staged_replan_last_reason='STAGE1_PROPOSAL_NOT_CERTIFIED';
                obj.r22_s1_post_recorded_ms=1e3*toc(tpost); obj.r22_s1_method_total_ms=1e3*toc(tall); return;
            end
            assert(abs(double(candidate.t0)-ta)<=64*eps(max(1,abs(ta))), ...
                '[AF7R58ZZU] staged candidate t0 is not activation time.');
            obj.staged_replan_candidate=candidate; obj.staged_replan_rref=struct(); obj.staged_replan_bundle=struct();
            obj.staged_replan_request_time=t_now; obj.staged_replan_activation_time=ta;
            obj.staged_replan_source_mode=char(obj.active_reference_mode);
            obj.staged_replan_source_version=double(obj.active_reference_version);
            obj.staged_replan_known_mask=logical(active(:).');
            obj.staged_replan_static_map=obj.static_obstacle_map;
            obj.staged_replan_dependency_key=obj.staged_reference_only_dependency_key();
            obj.staged_replan_speed_source=obj.active_speed_certificate;
            obj.staged_replan_obstacle_id=double(obs_id); obj.staged_replan_component_code=double(comp_code);
            obj.staged_replan_component_name=char(comp_name); obj.staged_replan_collision_time=double(collision_time);
            obj.staged_replan_initial_ttc=double(ttc); obj.staged_replan_pL_collision=double(pL_collision(:));
            obj.staged_replan_q_collision=double(q_collision(:)); obj.staged_replan_pQ_collision=double(pQ_collision(:));
            obj.staged_replan_hit_index=double(jhit); obj.staged_replan_state='GENERATED';
            obj.staged_replan_last_reason='STAGE1_GENERATED'; obj.generator_candidate_count=obj.generator_candidate_count+1;
            obj.generator_last_status_code=40;
            obj.r22_s1_post_recorded_ms=1e3*toc(tpost);
            obj.r22_s1_method_total_ms=1e3*toc(tall);
            known=obj.r22_s1_future_packet_ms+obj.r22_s1_context_ms+obj.r22_s1_native_wall_ms+obj.r22_s1_post_recorded_ms;
            obj.r22_s1_residual_ms=obj.r22_s1_method_total_ms-known;
        end

        function [ok,reason,B] = prepare_staged_reference_only_bundle(obj,P,t_commit)
            ok=false; reason=''; B=struct(); tAll=tic;
            obj.r22_s2_prepare_total_ms=NaN; obj.r22_s2_guard_cert_ms=NaN; obj.r22_s2_batch_model_ms=NaN;
            obj.r22_s2_packet_reuse_gate_ms=NaN; obj.r22_s2_packet_action_ms=NaN; obj.r22_s2_handles_ms=NaN;
            obj.r22_s2_metadata_ms=NaN; obj.r22_s2_bundle_ms=NaN; obj.r22_s2_residual_ms=NaN;
            obj.r22_s2_runtime_packet_reuse=false;

            tg=tic;
            if ~obj.ENABLE_AUTO_REPLAN_COMMIT, reason='AUTO_COMMIT_DISABLED'; obj.r22_s2_guard_cert_ms=1e3*toc(tg); return; end
            if ~strcmpi(char(obj.AVOIDANCE_COMMIT_POLICY),'REFERENCE_ONLY'), reason='NOT_REFERENCE_ONLY_COMMIT_POLICY'; obj.r22_s2_guard_cert_ms=1e3*toc(tg); return; end
            if ~(isstruct(P)&&isfield(P,'success')&&logical(P.success)), reason='PROPOSAL_NOT_SUCCESSFUL'; obj.r22_s2_guard_cert_ms=1e3*toc(tg); return; end
            [spacingOK,~,~,spacingReason]=af7r58i_commit_spacing_guard(obj,double(t_commit));
            if ~spacingOK, reason=spacingReason; obj.r22_s2_guard_cert_ms=1e3*toc(tg); return; end
            G=P.geometry_shadow_certificate; SC=P.reference_speed_certificate;
            if ~(isstruct(G)&&isfield(G,'certified')&&logical(G.certified)&&isfield(G,'continuous_time')&&logical(G.continuous_time)&&isfield(G,'sampled_acceptance')&&~logical(G.sampled_acceptance))
                reason='REFERENCE_GEOMETRY_CERTIFICATE_NOT_CLOSED'; obj.r22_s2_guard_cert_ms=1e3*toc(tg); return;
            end
            if ~(isstruct(SC)&&isfield(SC,'certified')&&logical(SC.certified)), reason='REFERENCE_SPEED_CERTIFICATE_NOT_CLOSED'; obj.r22_s2_guard_cert_ms=1e3*toc(tg); return; end
            targetVersion=double(obj.active_reference_version)+1;
            if double(SC.reference_version)~=targetVersion, reason='REFERENCE_SPEED_CERTIFICATE_VERSION_MISMATCH'; obj.r22_s2_guard_cert_ms=1e3*toc(tg); return; end
            if ~(isfield(SC,'continuous_time')&&logical(SC.continuous_time)&&isfield(SC,'sampled_acceptance')&&~logical(SC.sampled_acceptance)&&isfield(SC,'specific_force_norm_LB_m_s2')&&isnumeric(SC.specific_force_norm_LB_m_s2)&&isscalar(SC.specific_force_norm_LB_m_s2)&&isfinite(double(SC.specific_force_norm_LB_m_s2))&&double(SC.specific_force_norm_LB_m_s2)>0)
                reason='REFERENCE_ONLY_CONTINUOUS_FLATNESS_CERTIFICATE_NOT_CLOSED'; obj.r22_s2_guard_cert_ms=1e3*toc(tg); return;
            end
            if ~(isfield(P,'c7_exact')&&logical(P.c7_exact)), reason='REFERENCE_ONLY_C7_CERTIFICATE_NOT_CLOSED'; obj.r22_s2_guard_cert_ms=1e3*toc(tg); return; end
            obj.r22_s2_guard_cert_ms=1e3*toc(tg);

            tx=tic;
            [resourceOK,resourceReason,batchModel,resourceMeta]=af7r58i_prepare_batch_model(obj,P,targetVersion,double(t_commit));
            obj.r22_s2_batch_model_ms=1e3*toc(tx);
            if ~resourceOK, reason=resourceReason; return; end

            tx=tic;
            reuseRuntimePacket=obj.can_reuse_reference_only_runtime_packet_exact(obj.active_speed_certificate,SC,obj.trigger_sweep_inflation,obj.obstacle_packet,obj.sensor_horizon_certificate,obj.runtime_dependency_signature);
            obj.r22_s2_packet_reuse_gate_ms=1e3*toc(tx);
            obj.r22_s2_runtime_packet_reuse=logical(reuseRuntimePacket);
            tx=tic;
            if reuseRuntimePacket
                [sweepNew,packetNew,sensorNew,depNew]=obj.reuse_reference_only_runtime_packet_exact(obj.trigger_sweep_inflation,obj.obstacle_packet,obj.sensor_horizon_certificate,obj.runtime_dependency_signature,SC,targetVersion);
            else
                sweepNew=obj.build_trigger_sweep_for_target(SC,targetVersion);
                packetNew=obj.prepare_obstacle_packet_fast_target(obj.obstacle_packet,sweepNew,targetVersion);
                sensorNew=obj.build_sensor_horizon_for_target(SC,targetVersion);
                if ~(isstruct(sensorNew)&&isfield(sensorNew,'certified')&&logical(sensorNew.certified)), reason='STAGED_TARGET_SENSOR_HORIZON_OPEN'; obj.r22_s2_packet_action_ms=1e3*toc(tx); return; end
                depNew=obj.runtime_dependency_signature; depNew.reference_mode='REPAIRED'; depNew.reference_version=double(targetVersion);
            end
            obj.r22_s2_packet_action_ms=1e3*toc(tx);

            tx=tic; [evalFn,flatEvalFn]=af7r58i_make_canonical_eval_handles(obj,batchModel); obj.r22_s2_handles_ms=1e3*toc(tx);

            tx=tic;
            smoothSource='DEGREE15_ENDPOINT_REPEATED_BEZIER_C7';
            if isfield(P,'representation_code')&&double(P.representation_code)==2, smoothSource='PIECEWISE_DEGREE15_BEZIER_C7_AFFINE_TOWING_CORRIDORS'; end
            smooth=struct('certified',logical(P.c7_exact),'continuous_time',true,'sampled_acceptance',false,'continuity_order',7,'reference_mode','REPAIRED','reference_version',targetVersion,'valid_from',double(P.t0),'valid_until',double(P.t1),'source',smoothSource);
            geom=G; geom.reference_mode='REPAIRED'; geom.reference_version=targetVersion; geom.valid_from=double(P.t0); geom.valid_until=double(P.t1); geom.reference_only_authority=true; geom.final_system_authority=false;
            R=struct(); R.valid=true; R.backend='BEZIER_D15_C7'; R.eval_fn=evalFn; R.flat_eval_fn=flatEvalFn; R.valid_from=double(P.t0); R.valid_until=Inf; R.reference_version=targetVersion; R.source='AF7R58ZZU_STAGED_REFERENCE_ONLY_COMMIT'; R.yaw_policy='C6_INDEPENDENT';
            R.path_parameterization=struct('kind','DIRECT_TIME_BEZIER_CORRECTION','path_derivative_order',7,'time_derivative_order',7);
            R.rejoin_time=double(P.t1); R.representation=struct('kind','BEZIER_D15_C7_TRANSLATION_BUMP','t0',double(P.t0),'tm',double(P.tm),'t1',double(P.t1),'displacement',double(P.displacement(:)));
            R.batch_flat_model=batchModel; R.resource_certificate=resourceMeta.contract; R.resource_state=resourceMeta;
            R.authority_split=struct('controller_packet','eval_fn','mathematical_flat_jet','flat_eval_fn','legacy_high_derivative_slots_used_for_geometry',false);
            obj.r22_s2_metadata_ms=1e3*toc(tx);

            tx=tic;
            B=struct('schema','AF7R58ZZY_R21_PREPARED_CORE_V1','source_mode',char(obj.active_reference_mode),'source_version',double(obj.active_reference_version),'target_version',double(targetVersion),'activation_time',double(t_commit),'dependency_key',obj.staged_replan_dependency_key,'speed_source',obj.staged_replan_speed_source,'known_mask',logical(obj.staged_replan_known_mask(:).'),'static_map',obj.staged_replan_static_map,'reference',R,'certificates',struct('reference_only',true,'smoothness',smooth,'geometry',geom),'speed',SC,'sweep',sweepNew,'packet',packetNew,'sensor',sensorNew,'dep',depNew,'post_commit_cache_pending',true,'prevalidated',true);
            obj.r22_s2_bundle_ms=1e3*toc(tx);
            obj.r22_s2_prepare_total_ms=1e3*toc(tAll);
            known=obj.r22_s2_guard_cert_ms+obj.r22_s2_batch_model_ms+obj.r22_s2_packet_reuse_gate_ms+obj.r22_s2_packet_action_ms+obj.r22_s2_handles_ms+obj.r22_s2_metadata_ms+obj.r22_s2_bundle_ms;
            obj.r22_s2_residual_ms=obj.r22_s2_prepare_total_ms-known;
            ok=true; reason='STAGED_COMMIT_CORE_PREPARED__POSTCACHE_DEFERRED_TO_STAGE3';
        end

        function [ok,reason] = activate_staged_reference_only_bundle(obj,B,t_now,t_hard_cycle)
            ok=false; reason=''; tall=tic;
            props={'r22_s3_guard_ms','r22_s3_grid_ms','r22_s3_eval_ms','r22_s3_validate_q_ms','r22_s3_cache_struct_ms','r22_s3_predeadline_ms','r22_s3_bfull_ms','r22_s3_freshness_ms','r22_s3_snapshot_ms','r22_s3_authority_assign_ms','r22_s3_hot_cert_ms','r22_s3_deadline_ms','r22_s3_cache_install_ms','r22_s3_status_ms','r22_s3_residual_ms'};
            for kk=1:numel(props), obj.(props{kk})=NaN; end
            obj.r22_s3_total_ms=NaN; obj.r22_s3_table_used=false;

            tx=tic;
            if ~(isstruct(B)&&isfield(B,'schema')&&strcmp(char(B.schema),'AF7R58ZZY_R21_PREPARED_CORE_V1')&&isfield(B,'prevalidated')&&logical(B.prevalidated)&&isfield(B,'post_commit_cache_pending')&&logical(B.post_commit_cache_pending)&&isfield(B,'reference')&&isstruct(B.reference)&&isfield(B.reference,'batch_flat_model'))
                reason='R21_DEFERRED_BUNDLE_SCHEMA_MISMATCH'; obj.r22_s3_guard_ms=1e3*toc(tx); obj.r22_s3_total_ms=1e3*toc(tall); return;
            end
            scale=max([1,abs(double(t_now)),abs(double(B.activation_time))]);
            if abs(double(t_now)-double(B.activation_time))>64*eps(scale)
                reason='STAGED_ACTIVATION_TIME_MISMATCH'; obj.r22_s3_guard_ms=1e3*toc(tx); obj.r22_s3_total_ms=1e3*toc(tall); return;
            end
            obj.r22_s3_guard_ms=1e3*toc(tx);

            try
                tx=tic; dt=double(obj.prediction_dt); H=double(obj.horizon_s); N=round(H/dt)+1; tCache=double(t_now)+dt; tqCache=tCache+(0:N-1)*dt; obj.r22_s3_grid_ms=1e3*toc(tx);
                tx=tic; [pLcache,aLcache,validCache,tableUsed]=af7r75_posaccel_batch_R19(obj,tqCache,B.reference.batch_flat_model); obj.r22_s3_eval_ms=1e3*toc(tx); obj.r22_s3_table_used=logical(tableUsed);
                tx=tic;
                if ~(isequal(size(pLcache),[3 N])&&isequal(size(aLcache),[3 N])&&numel(validCache)==N&&all(logical(validCache(:)))&&all(isfinite(pLcache),'all')&&all(isfinite(aLcache),'all'))
                    reason='STAGED_POSTCOMMIT_CACHE_BATCH_INVALID'; obj.r22_s3_validate_q_ms=1e3*toc(tx); obj.r22_s3_total_ms=1e3*toc(tall); return;
                end
                sfcache=aLcache+[0;0;double(obj.gravity)]; nscache=sqrt(sum(sfcache.^2,1));
                if ~(all(isfinite(nscache))&&all(nscache>0)), reason='STAGED_POSTCOMMIT_CACHE_FLATNESS_SINGULAR'; obj.r22_s3_validate_q_ms=1e3*toc(tx); obj.r22_s3_total_ms=1e3*toc(tall); return; end
                qcache=-sfcache./nscache; obj.r22_s3_validate_q_ms=1e3*toc(tx);
                tx=tic; postCache=struct('reference_version',double(B.target_version),'t0',double(tCache),'dt',dt,'H',H,'pL',pLcache,'q',qcache); obj.r22_s3_cache_struct_ms=1e3*toc(tx);
            catch ME
                reason=ME.message; obj.r22_s3_total_ms=1e3*toc(tall); return;
            end

            tx=tic;
            if toc(t_hard_cycle)>double(obj.prediction_dt)
                obj.generator_deadline_fail_count=obj.generator_deadline_fail_count+1; reason='STAGED_ATOMIC_CONTROL_PERIOD_EXCEEDED'; obj.r22_s3_predeadline_ms=1e3*toc(tx); obj.r22_s3_total_ms=1e3*toc(tall); return;
            end
            obj.r22_s3_predeadline_ms=1e3*toc(tx);

            tx=tic; Bfull=B; Bfull.schema='AF7R58ZZV_PREPARED_COMMIT_V2'; Bfull.post_commit_reference_cache=postCache; Bfull=rmfield(Bfull,'post_commit_cache_pending'); obj.staged_replan_postcommit_cache_prepare_count=obj.staged_replan_postcommit_cache_prepare_count+1; obj.r22_s3_bfull_ms=1e3*toc(tx);

            tx=tic;
            if ~strcmp(char(obj.active_reference_mode),char(Bfull.source_mode)) || double(obj.active_reference_version)~=double(Bfull.source_version)
                reason='STAGED_SOURCE_REFERENCE_STALE'; obj.r22_s3_freshness_ms=1e3*toc(tx); obj.r22_s3_total_ms=1e3*toc(tall); return;
            end
            if ~isequaln(obj.staged_reference_only_dependency_key(),Bfull.dependency_key)||~isequaln(obj.active_speed_certificate,Bfull.speed_source)||~isequaln(obj.static_obstacle_map,Bfull.static_map)
                reason='STAGED_DEPENDENCY_OR_MAP_STALE'; obj.r22_s3_freshness_ms=1e3*toc(tx); obj.r22_s3_total_ms=1e3*toc(tall); return;
            end
            obj.r22_s3_freshness_ms=1e3*toc(tx);

            tx=tic;
            old=struct('mode',obj.active_reference_mode,'version',obj.active_reference_version,'ref',obj.active_repaired_reference,'cert',obj.active_repaired_certificates,'speed',obj.active_speed_certificate,'sweep',obj.trigger_sweep_inflation,'packet',obj.obstacle_packet,'sensor',obj.sensor_horizon_certificate,'dep',obj.runtime_dependency_signature,'stale',obj.certificate_stale,'level',obj.active_repaired_safety_level,'time',obj.last_replan_commit_time);
            obj.r22_s3_snapshot_ms=1e3*toc(tx);
            try
                tx=tic;
                obj.active_repaired_reference=Bfull.reference; obj.active_repaired_certificates=Bfull.certificates; obj.active_reference_mode='REPAIRED'; obj.active_reference_version=double(Bfull.target_version); obj.active_repaired_safety_level='REFERENCE_ONLY'; obj.active_speed_certificate=Bfull.speed; obj.trigger_sweep_inflation=Bfull.sweep; obj.obstacle_packet=Bfull.packet; obj.sensor_horizon_certificate=Bfull.sensor; obj.runtime_dependency_signature=Bfull.dep; obj.certificate_stale=obj.empty_certificate_stale_state(); obj.certificate_stale.ACTUATOR_DYNAMICS_CERT=true; obj.certificate_stale.HLC_ROBUST_CERT=true; obj.last_replan_commit_time=double(t_now);
                obj.r22_s3_authority_assign_ms=1e3*toc(tx);
                tx=tic; obj.assert_runtime_core_certificates_current_hot(); obj.r22_s3_hot_cert_ms=1e3*toc(tx);
                tx=tic;
                if toc(t_hard_cycle)>double(obj.prediction_dt)
                    obj.active_reference_mode=old.mode; obj.active_reference_version=old.version; obj.active_repaired_reference=old.ref; obj.active_repaired_certificates=old.cert; obj.active_speed_certificate=old.speed; obj.trigger_sweep_inflation=old.sweep; obj.obstacle_packet=old.packet; obj.sensor_horizon_certificate=old.sensor; obj.runtime_dependency_signature=old.dep; obj.certificate_stale=old.stale; obj.active_repaired_safety_level=old.level; obj.last_replan_commit_time=old.time; obj.generator_deadline_fail_count=obj.generator_deadline_fail_count+1; reason='STAGED_ATOMIC_CONTROL_PERIOD_EXCEEDED'; obj.r22_s3_deadline_ms=1e3*toc(tx); obj.r22_s3_total_ms=1e3*toc(tall); return;
                end
                obj.r22_s3_deadline_ms=1e3*toc(tx);
                tx=tic; Cpost=Bfull.post_commit_reference_cache; assert(isstruct(Cpost)&&double(Cpost.reference_version)==double(Bfull.target_version)&&abs(double(Cpost.t0)-(double(t_now)+double(obj.prediction_dt)))<=64*eps(max(1,abs(double(Cpost.t0)))),'[AF7R58ZZV] prepared post-commit reference cache is stale.'); obj.repaired_reference_cache=Cpost; obj.repaired_reference_cache_valid=true; obj.reference_only_prepared_packet_cache_valid=false; obj.reference_only_prepared_packet_cache=struct(); obj.r22_s3_cache_install_ms=1e3*toc(tx);
                tx=tic; obj.replan_commit_count=obj.replan_commit_count+1; obj.reference_only_commit_count=obj.reference_only_commit_count+1; obj.replan_request_active=false; obj.last_replan_status=struct('requested',true,'committed',true,'safety_mode','REFERENCE_ONLY','safety_level','REFERENCE_ONLY','full_system_authority',false,'reference_version',double(Bfull.target_version),'backend','BEZIER_D15_C7','commit_time',double(t_now),'rejoin_time',double(Bfull.reference.rejoin_time),'staged_pipeline',true,'pipeline_intervals',2); obj.r22_s3_status_ms=1e3*toc(tx);
                ok=true; reason='STAGED_REFERENCE_ONLY_COMMIT_CLOSED';
            catch ME
                obj.active_reference_mode=old.mode; obj.active_reference_version=old.version; obj.active_repaired_reference=old.ref; obj.active_repaired_certificates=old.cert; obj.active_speed_certificate=old.speed; obj.trigger_sweep_inflation=old.sweep; obj.obstacle_packet=old.packet; obj.sensor_horizon_certificate=old.sensor; obj.runtime_dependency_signature=old.dep; obj.certificate_stale=old.stale; obj.active_repaired_safety_level=old.level; obj.last_replan_commit_time=old.time; reason=ME.message;
            end
            obj.r22_s3_total_ms=1e3*toc(tall);
            vv=[obj.r22_s3_guard_ms obj.r22_s3_grid_ms obj.r22_s3_eval_ms obj.r22_s3_validate_q_ms obj.r22_s3_cache_struct_ms obj.r22_s3_predeadline_ms obj.r22_s3_bfull_ms obj.r22_s3_freshness_ms obj.r22_s3_snapshot_ms obj.r22_s3_authority_assign_ms obj.r22_s3_hot_cert_ms obj.r22_s3_deadline_ms obj.r22_s3_cache_install_ms obj.r22_s3_status_ms];
            obj.r22_s3_residual_ms=obj.r22_s3_total_ms-sum(vv(isfinite(vv)));
        end
    end
end
