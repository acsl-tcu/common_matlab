classdef REPLANNING_SYSTEM_R28_NATIVE_ONCE < REPLANNING_SYSTEM_R21_BALANCED
% AF7R58ZZY-R28 | NATIVE-ONCE COMPILED REPRESENTATION PIPELINE
%
% R28 preserves the R26/R28 four-cycle measurement schedule so timing changes
% cannot be hidden by moving work between control periods.  The only primary
% optimization is a representation-construction change:
%
%   * The exact AF7R36 native generator computes the degree-15 candidate,
%     continuous speed/flatness certificate and continuous whole-body geometry
%     certificate exactly as before.
%   * AFTER plan() has returned, the same native MEX constructs the R23 runtime
%     derivative packet from the same candidate control points.  No trajectory
%     acceptance condition depends on the compiled packet.
%   * Previously certified surviving groups are copied bitwise after the exact
%     same prune complement group_t1 > t_commit.  The newly generated group is
%     differentiated exactly once in native code using the already validated
%     R25 recurrence.
%   * Stage3 binds that packet to the exact R21 canonical/certificate bundle by
%     version/count/support metadata and an inductive authority invariant.  It
%     does NOT rebuild/cat all control points or compare every D0 element.
%
% Exact binding theorem used by R28:
%   Base: NOMINAL has zero correction groups.
%   Induction: a committed R28 repaired reference stores both the canonical
%   batch model and the native compiled packet at the same reference_version.
%   Both prune precisely groups with t1<=t_commit; the native packet copies each
%   surviving derivative block bitwise and appends derivatives computed from the
%   same PlanResult::cand.cp that is exported as RAW.control_points and appended
%   to the canonical model.  Hence the next committed pair remains equivalent.
%
% Intentionally unchanged:
%   - H=4 s sensing/trigger semantics
%   - degree-15 C7 candidate construction
%   - continuous-time speed/specific-force/qdot certificates
%   - continuous whole-body payload/UAV/cable geometry certificate
%   - AF7R58I resource prune/cap contract
%   - obstacle/sensor runtime certificate construction
%   - freshness/stale rejection and transactional rollback
%   - controller / estimator / plant
%   - four-cycle measurement schedule (to be reduced only after timing closes)

    properties (SetAccess = private)
        r25_build_id = 'AF7R58ZZY_R28_NATIVE_ONCE_V1';
        r23_last_compile_ms = NaN;
        r23_last_stage3_cache_ms = NaN;
        r23_last_stage3_target_guard_ms = NaN;
        r23_last_stage3_assignment_ms = NaN;
        r23_last_stage3_total_ms = NaN;
        r23_last_packet_ms = NaN;
        r23_last_current_eval_ms = NaN;
        r23_compiled_current_count = 0;
        r23_compiled_packet_count = 0;
        r23_stage3_token_valid = false;
        r23_stage3_token_time = NaN;
        r23_stage3_token_source_version = NaN;
        r23_stage3_token_target_version = NaN;
        r25_wait_freshness_skip_count = 0;
        r25_nominal_core_cache_hit_count = 0;
        r25_repaired_core_cache_hit_count = 0;
        r25_last_core_packet_ms = NaN;
        r25_stage2_fresh_token_valid = false;
        r25_stage2_fresh_token_source_version = NaN;
        r25_fast_packet_rebuild_count = 0;
        r25_last_fast_packet_rebuild_ms = NaN;
        r26_last_reference_cert_ms = NaN;
        r26_last_core_prepare_ms = NaN;
        r26_last_final_compile_ms = NaN;
        r26_last_final_activate_ms = NaN;
        r26_last_final_total_ms = NaN;
        r26_reference_cert_count = 0;
        r26_core_prepare_count = 0;
        r26_final_count = 0;
        r28_core_only_prepare_mode = false;
        r28_last_native_compile_ms = NaN;
        r28_native_compile_count = 0;
        r28_full_compile_avoided_count = 0;
        r28_lazy_postcache_commit_count = 0;
        r28_last_lazy_cache_rebuild_ms = NaN;
        r28_lazy_postcache_pending = false;
        r28_last_binding_ms = NaN;
    end

    methods
        function obj = REPLANNING_SYSTEM_R28_NATIVE_ONCE(self,base_ref,opts)
            obj@REPLANNING_SYSTEM_R21_BALANCED(self,base_ref,opts);
            assert(exist('af7r76_compiled_eval_mex','file')==3, ...
                '[R25] compiled evaluator MEX missing. Run AF7R58ZZY_R23_R3_INSTALL first.');
            assert(exist('af7r76_compile_correction_packet_R23','file')==2, ...
                '[R26] exact R23 compile helper missing.');
            assert(exist('af7r77_compile_correction_packet_R25','file')==2, ...
                '[R26] inherited R25 preflight compile helper missing.');
            assert(exist('af7r77_compile_dcp_mex','file')==3, ...
                '[R26] inherited R25 preflight derivative MEX missing.');
            assert(exist('af7r77_merge_result_reference_fast_R25','file')==2, ...
                '[R27] R25 exact fast-merge helper missing.');
            rev=af7r36_single_core_mex(5);
            assert(ischar(rev)&&strcmp(rev,'AF7R58ZZY_R28_NATIVE_ONCE_V1'), ...
                '[R28] AF7R36 native generator MEX is not the R28 exact build.');
        end

        function R = get_r23_performance_status(obj)
            R=struct( ...
                'compile_ms',double(obj.r23_last_compile_ms), ...
                'stage3_cache_ms',double(obj.r23_last_stage3_cache_ms), ...
                'stage3_target_guard_ms',double(obj.r23_last_stage3_target_guard_ms), ...
                'stage3_assignment_ms',double(obj.r23_last_stage3_assignment_ms), ...
                'stage3_total_ms',double(obj.r23_last_stage3_total_ms), ...
                'last_repaired_packet_ms',double(obj.r23_last_packet_ms), ...
                'last_current_eval_ms',double(obj.r23_last_current_eval_ms), ...
                'compiled_current_count',double(obj.r23_compiled_current_count), ...
                'compiled_packet_count',double(obj.r23_compiled_packet_count));
        end

        function R = get_r25_performance_status(obj)
            R=obj.get_r23_performance_status();
            R.build_id=char(obj.r25_build_id);
            R.wait_freshness_skip_count=double(obj.r25_wait_freshness_skip_count);
            R.nominal_core_cache_hit_count=double(obj.r25_nominal_core_cache_hit_count);
            R.repaired_core_cache_hit_count=double(obj.r25_repaired_core_cache_hit_count);
            R.last_core_packet_ms=double(obj.r25_last_core_packet_ms);
            R.fast_packet_rebuild_count=double(obj.r25_fast_packet_rebuild_count);
            R.last_fast_packet_rebuild_ms=double(obj.r25_last_fast_packet_rebuild_ms);
            R.stage2_fresh_token_valid=logical(obj.r25_stage2_fresh_token_valid);
        end

        function R = get_r26_performance_status(obj)
            R=obj.get_r25_performance_status();
            R.build_id='AF7R58ZZY_R28_NATIVE_ONCE_V1';
            R.reference_cert_ms=double(obj.r26_last_reference_cert_ms);
            R.core_prepare_ms=double(obj.r26_last_core_prepare_ms);
            R.final_compile_ms=double(obj.r26_last_final_compile_ms);
            R.final_activate_ms=double(obj.r26_last_final_activate_ms);
            R.final_total_ms=double(obj.r26_last_final_total_ms);
            R.reference_cert_count=double(obj.r26_reference_cert_count);
            R.core_prepare_count=double(obj.r26_core_prepare_count);
            R.final_count=double(obj.r26_final_count);
            R.pipeline_intervals=3;
            R.pipeline_delay_s=3*double(obj.prediction_dt);
        end

        function R = get_r28_performance_status(obj)
            R=obj.get_r26_performance_status();
            R.build_id='AF7R58ZZY_R28_NATIVE_ONCE_V1';
            R.native_compile_ms=double(obj.r28_last_native_compile_ms);
            R.native_compile_count=double(obj.r28_native_compile_count);
            R.full_compile_avoided_count=double(obj.r28_full_compile_avoided_count);
            R.binding_ms=double(obj.r28_last_binding_ms);
            R.lazy_postcache_commit_count=double(obj.r28_lazy_postcache_commit_count);
            R.last_lazy_cache_rebuild_ms=double(obj.r28_last_lazy_cache_rebuild_ms);
            R.native_once=true;
            R.external_incremental_compile=false;
            R.schedule_unchanged_from_R27=true;
        end


        function S = get_staged_pipeline_status(obj)
            S=struct();
            S.enabled=logical(obj.ENABLE_STAGED_REPLAN_PIPELINE);
            S.state=char(obj.staged_replan_state);
            S.fixed_horizon_s=double(obj.horizon_s);
            S.pipeline_intervals=3;
            S.pipeline_delay_s=3*double(obj.prediction_dt);
            S.first_trigger_entry_ttc_LB_s= ...
                double(obj.horizon_s)-double(obj.prediction_dt);
            S.remaining_ttc_after_scheduled_activation_LB_s= ...
                double(obj.horizon_s)-4*double(obj.prediction_dt);
            S.warmup_policy='PRE_ARM_INITIALIZATION_ONLY';
            S.arming_warmup_allowed=false;
            S.takeoff_warmup_allowed=false;
            S.flight_warmup_allowed=false;
            S.landing_warmup_allowed=false;
            S.stage1_count=double(obj.staged_replan_stage1_count);
            S.stage2_count=double(obj.staged_replan_stage2_count);
            S.commit_count=double(obj.staged_replan_commit_count);
            S.stale_reject_count=double(obj.staged_replan_stale_reject_count);
            S.fail_count=double(obj.staged_replan_fail_count);
            S.last_stage1_ms=double(obj.staged_replan_last_stage1_ms);
            S.last_stage2_ms=double(obj.staged_replan_last_stage2_ms);
            S.last_stage3_ms=double(obj.staged_replan_last_stage3_ms);
            S.trigger_kernel_skip_count=double(obj.staged_replan_trigger_kernel_skip_count);
            S.postcommit_cache_prepare_count= ...
                double(obj.staged_replan_postcommit_cache_prepare_count);
            S.worker_used=false;
            S.runtime_hot_guard='DEPENDENCY_BOUND_NO_PER_CYCLE_FUNC2STR';
            S.last_reason=char(obj.staged_replan_last_reason);
            S.r26_reference_cert_ms=double(obj.r26_last_reference_cert_ms);
            S.r26_core_prepare_ms=double(obj.r26_last_core_prepare_ms);
            S.r26_final_compile_ms=double(obj.r26_last_final_compile_ms);
            S.r26_final_activate_ms=double(obj.r26_last_final_activate_ms);
            S.r26_final_total_ms=double(obj.r26_last_final_total_ms);
            S.r26_reference_cert_count=double(obj.r26_reference_cert_count);
            S.r26_core_prepare_count=double(obj.r26_core_prepare_count);
            S.r26_final_count=double(obj.r26_final_count);
            S.r28_native_compile_ms=double(obj.r28_last_native_compile_ms);
            S.r28_native_compile_count=double(obj.r28_native_compile_count);
            S.r28_full_compile_avoided_count=double(obj.r28_full_compile_avoided_count);
            S.r28_binding_ms=double(obj.r28_last_binding_ms);
            S.r28_lazy_postcache_commit_count=double(obj.r28_lazy_postcache_commit_count);
            S.r28_schedule_unchanged_from_R27=true;
        end


        function [ok,reason,B] = prepare_staged_reference_only_bundle(obj,P,t_commit)
            % First close the EXACT R21 authority/certificate bundle.
            [ok,reason,B]=prepare_staged_reference_only_bundle@REPLANNING_SYSTEM_R21_BALANCED( ...
                obj,P,t_commit);
            obj.r23_last_compile_ms=NaN;
            if ~ok
                return;
            end

            % R28 Flight core-preparation mode: R21 has already closed the
            % canonical resource and continuous-time certificate bundle.  The
            % compiled packet is a dependent runtime representation produced by
            % the SAME native PlanResult.  Bind it without reconstructing all
            % control points in MATLAB.
            if obj.r28_core_only_prepare_mode
                if ~(isstruct(P)&&isfield(P,'r28_native_compiled_correction')&& ...
                        isfield(P,'r28_native_compiled_exact')&&logical(P.r28_native_compiled_exact))
                    ok=false; reason='R28_NATIVE_COMPILED_PACKET_MISSING'; B=struct(); return;
                end
                try
                    CP=P.r28_native_compiled_correction;
                    obj.r28_validate_native_binding( ...
                        CP,B.reference.batch_flat_model,P,double(B.target_version));
                    B.reference.r23_compiled_correction=CP;
                    B.reference.r23_compiled_runtime_exact=true;
                    B.reference.r23_canonical_eval_retained=true;
                    B.reference.authority_split.runtime_compiled_correction= ...
                        'R28_NATIVE_ONCE_EXACT_R23_SCHEMA';
                    B.r23_compiled_prevalidated=true;
                    B.r28_native_prevalidated=true;
                    obj.r28_full_compile_avoided_count=obj.r28_full_compile_avoided_count+1;
                    obj.r23_last_compile_ms=0;
                catch ME
                    ok=false; reason=['R28_NATIVE_BINDING_REJECT__' ME.identifier]; B=struct();
                end
                return;
            end

            % Non-Flight/preflight fallback retains the existing exact R25
            % full compiler.  This branch is not used by the real R28 staged
            % Flight path and preserves constructor/prime compatibility.
            % Only after certificate/resource closure, compile the same finite
            % canonical correction model into contiguous derivative CP arrays.
            tc=tic;
            try
                CP=af7r77_compile_correction_packet_R25( ...
                    B.reference.batch_flat_model,double(B.target_version));
                obj.r23_validate_compiled_packet_binding(CP,B.reference.batch_flat_model, ...
                    double(B.target_version));
                B.reference.r23_compiled_correction=CP;
                B.reference.r23_compiled_runtime_exact=true;
                B.reference.r23_canonical_eval_retained=true;
                B.reference.authority_split.runtime_compiled_correction= ...
                    'R23_STRICT_FP_SAME_CANONICAL_CORRECTION';
                B.r23_compiled_prevalidated=true;
                obj.r23_last_compile_ms=1e3*toc(tc);
            catch ME
                obj.r23_last_compile_ms=1e3*toc(tc);
                ok=false;
                reason=['R23_COMPILED_CORRECTION_REJECT__' ME.identifier];
                B=struct();
                return;
            end
        end

        function xd = evaluate_active_reference_xd(obj,t_now,xd_nom)
            switch upper(char(obj.active_reference_mode))
                case 'NOMINAL'
                    xd=xd_nom(:);
                    return;

                case 'REPAIRED'
                    te=tic;
                    R=obj.active_repaired_reference;
                    R=obj.r23_attach_compiled_for_preflight_prime_if_needed(R);
                    obj.r23_assert_compiled_reference_binding(R,double(t_now));

                    [base,gridOK]=obj.r23_nominal_xd_grid(double(t_now),xd_nom);
                    if ~gridOK
                        % Off-grid diagnostic/certificate calls retain the
                        % canonical function-handle authority exactly.
                        xd=R.eval_fn(double(t_now));
                        xd=xd(:);
                    else
                        if numel(base)<28
                            base=[base;zeros(28-numel(base),1)];
                        end
                        xd=af7r76_compiled_eval_mex(1,double(t_now),double(base(:)), ...
                            R.r23_compiled_correction);
                        xd=double(xd(:));
                        assert(numel(xd)>=28&&all(isfinite(xd(1:28))), ...
                            '[R23] compiled current repaired reference is invalid.');
                        obj.r23_compiled_current_count=obj.r23_compiled_current_count+1;
                    end
                    obj.r23_last_current_eval_ms=1e3*toc(te);
                    return;

                otherwise
                    error('[R23] unknown active_reference_mode: %s',char(obj.active_reference_mode));
            end
        end

        function [pL_ref,q_ref,source] = repaired_reference_packet(obj,t_now)
            tp=tic;
            N=round(obj.horizon_s/obj.prediction_dt)+1;
            dt=double(obj.prediction_dt);
            H=double(obj.horizon_s);
            t0=double(t_now);
            R=obj.active_repaired_reference;
            R=obj.r23_attach_compiled_for_preflight_prime_if_needed(R);
            obj.r23_assert_compiled_reference_binding(R,t0);

            C=obj.repaired_reference_cache;
            canReuse=obj.repaired_reference_cache_valid && isstruct(C) && ...
                isfield(C,'reference_version') && ...
                double(C.reference_version)==double(obj.active_reference_version) && ...
                isfield(C,'dt') && isequal(double(C.dt),dt) && ...
                isfield(C,'H') && isequal(double(C.H),H) && ...
                isfield(C,'t0') && isfinite(double(C.t0)) && ...
                isfield(C,'pL') && isequal(size(C.pL),[3 N]) && ...
                isfield(C,'q') && isequal(size(C.q),[3 N]);

            shiftCount=NaN;
            if canReuse
                d=(t0-double(C.t0))/dt;
                if isfinite(d)
                    shiftCount=round(d);
                    talign=double(C.t0)+double(shiftCount)*dt;
                    scale=max([1,abs(t0),abs(double(C.t0)),abs(double(shiftCount)*dt)]);
                    guard=64*eps(scale);
                    if shiftCount<0 || abs(t0-talign)>guard
                        shiftCount=NaN;
                    end
                end
            end

            if canReuse && isfinite(shiftCount) && shiftCount==0
                pL_ref=C.pL;
                q_ref=C.q;
                source=sprintf('REPAIRED_R23_CACHE_HIT:v%d:%s', ...
                    obj.active_reference_version,char(R.backend));
                obj.validate_reference_packet(pL_ref,q_ref,N);
                obj.r23_last_packet_ms=1e3*toc(tp);
                return;
            end

            if canReuse && isfinite(shiftCount) && shiftCount>=1 && shiftCount<N
                m=double(shiftCount);
                pL_ref=[C.pL(:,m+1:end),zeros(3,m)];
                q_ref=[C.q(:,m+1:end),zeros(3,m)];
                tq=t0+(N-m:N-1)*dt;
                [pTail,aTail,gridOK]=obj.r23_posaccel_grid(tq,R.r23_compiled_correction);
                if ~gridOK
                    [pTail,aTail,valid,~]=af7r75_posaccel_batch_R19(obj,tq,R.batch_flat_model);
                    assert(numel(valid)==m&&all(logical(valid(:))), ...
                        '[R23] fallback repaired tail validity failed.');
                else
                    obj.r23_compiled_packet_count=obj.r23_compiled_packet_count+1;
                end
                assert(isequal(size(pTail),[3 m])&&isequal(size(aTail),[3 m])&& ...
                    all(isfinite(pTail),'all')&&all(isfinite(aTail),'all'), ...
                    '[R23] repaired rolling tail evaluator invalid.');
                pL_ref(:,end-m+1:end)=pTail;
                sf=aTail+[0;0;double(obj.gravity)];
                ns=sqrt(sum(sf.^2,1));
                assert(all(isfinite(ns))&&all(ns>0), ...
                    '[R23] repaired rolling tail flatness singular.');
                q_ref(:,end-m+1:end)=-sf./ns;
                source=sprintf('REPAIRED_R23_ROLLING_SHIFT%d:v%d:%s', ...
                    m,obj.active_reference_version,char(R.backend));
            else
                tq=t0+(0:N-1)*dt;
                [pL_ref,aRef,gridOK]=obj.r23_posaccel_grid(tq,R.r23_compiled_correction);
                if ~gridOK
                    [pL_ref,aRef,valid,~]=af7r75_posaccel_batch_R19(obj,tq,R.batch_flat_model);
                    assert(numel(valid)==N&&all(logical(valid(:))), ...
                        '[R23] fallback repaired full packet validity failed.');
                else
                    obj.r23_compiled_packet_count=obj.r23_compiled_packet_count+1;
                end
                assert(isequal(size(pL_ref),[3 N])&&isequal(size(aRef),[3 N])&& ...
                    all(isfinite(pL_ref),'all')&&all(isfinite(aRef),'all'), ...
                    '[R23] repaired full packet evaluator invalid.');
                sf=aRef+[0;0;double(obj.gravity)];
                ns=sqrt(sum(sf.^2,1));
                assert(all(isfinite(ns))&&all(ns>0), ...
                    '[R23] repaired full packet flatness singular.');
                q_ref=-sf./ns;
                source=sprintf('REPAIRED_R23_FULL:v%d:%s', ...
                    obj.active_reference_version,char(R.backend));
            end

            obj.validate_reference_packet(pL_ref,q_ref,N);
            obj.repaired_reference_cache=struct( ...
                'reference_version',double(obj.active_reference_version), ...
                't0',t0,'dt',dt,'H',H,'pL',pL_ref,'q',q_ref);
            obj.repaired_reference_cache_valid=true;
            obj.r23_last_packet_ms=1e3*toc(tp);
            if obj.r28_lazy_postcache_pending
                obj.r28_last_lazy_cache_rebuild_ms=obj.r23_last_packet_ms;
                obj.r28_lazy_postcache_pending=false;
            end
        end

        function start_staged_replan_pipeline(obj,t_hard_cycle,time,cha,varargin_in, ...
                active,all_dists,ttc,collision_time,obs_id,comp_name,comp_code, ...
                pL_collision,q_collision,pQ_collision,jhit)
            % R26: same exact Stage1 generator, scheduled for t+3*dt so that
            % certificate, core preparation, and final compile+commit occupy
            % separate control cycles. No acceptance condition is weakened.
            if ~obj.ENABLE_STAGED_REPLAN_PIPELINE
                return;
            end

            assert(strcmpi(char(obj.AVOIDANCE_COMMIT_POLICY),'REFERENCE_ONLY'), ...
                '[R26] staged pipeline currently supports REFERENCE_ONLY commit only.');
            assert(double(obj.generator_backend_locked_code)==1, ...
                '[R26] staged pipeline requires current native BEZIER backend.');
            assert(isequal(double(obj.horizon_s),4.0), ...
                '[R26] fixed 4-s collision horizon changed.');

            if ~strcmp(char(obj.staged_replan_state),'IDLE')
                obj.staged_replan_stale_reject_count= ...
                    obj.staged_replan_stale_reject_count+1;
                obj.reset_staged_replan_pipeline('REQUEST_IDENTITY_CHANGED',false);
            end

            t_now=double(time.t);
            dt=double(obj.prediction_dt);
            ta=t_now+3*dt;
            if ~(isfinite(ta)&&isfinite(double(collision_time))&& ...
                    double(collision_time)>ta)
                obj.staged_replan_fail_count=obj.staged_replan_fail_count+1;
                obj.generator_validation_fail_count= ...
                    obj.generator_validation_fail_count+1;
                obj.generator_last_status_code=-160;
                obj.staged_replan_last_reason= ...
                    'INSUFFICIENT_LEAD_FOR_THREE_INTERVAL_PIPELINE';
                return;
            end

            ts=tic;
            [pL_future,q_future,ref_source,xd_nom_a,xd_active_a]= ...
                obj.staged_reference_packet_at(ta,varargin_in);

            pL_a=double(pL_future(:,1));
            q_a=double(q_future(:,1));
            pQ_a=pL_a-double(obj.L_cable)*q_a;
            cable_residual_a=0;

            ttc_a=double(collision_time)-ta;
            jhit_a=round(ttc_a/dt)+1;
            jhit_a=max(1,min(size(pL_future,2),jhit_a));

            elapsed_before=toc(t_hard_cycle);
            budget_remaining=max(0,dt-elapsed_before);
            CTX=obj.build_generator_request_context_compact( ...
                ta,elapsed_before,budget_remaining, ...
                xd_nom_a,xd_active_a,pQ_a,pL_a,q_a,cable_residual_a, ...
                active,all_dists,pL_future,q_future,ref_source, ...
                ttc_a,double(collision_time),obs_id,comp_name,comp_code, ...
                pL_collision,q_collision,pQ_collision,jhit_a);

            try
                candidate=obj.af7r46_single_core_proposal_compact(CTX);
            catch ME
                obj.staged_replan_fail_count=obj.staged_replan_fail_count+1;
                obj.generator_validation_fail_count= ...
                    obj.generator_validation_fail_count+1;
                obj.generator_last_status_code=-161;
                obj.staged_replan_last_reason=['R28_STAGE1_NATIVE_ERROR__' ME.identifier];
                return;
            end

            obj.generator_last_runtime_ms=double(candidate.r9_stage_timing.total_internal_ms);
            obj.generator_request_count=obj.generator_request_count+1;
            obj.staged_replan_stage1_count=obj.staged_replan_stage1_count+1;

            if ~(isstruct(candidate)&&isfield(candidate,'success')&&logical(candidate.success))
                obj.generator_validation_fail_count=obj.generator_validation_fail_count+1;
                obj.generator_last_status_code=121;
                obj.staged_replan_fail_count=obj.staged_replan_fail_count+1;
                obj.staged_replan_last_reason='R28_STAGE1_PROPOSAL_NOT_CERTIFIED';
                return;
            end

            % R28: compiled representation was constructed INSIDE the same
            % native generator call after plan() returned.  There is no second
            % MATLAB/MEX compiler call and no packet concatenation here.
            if ~(isfield(candidate,'r28_native_compiled_exact')&& ...
                    logical(candidate.r28_native_compiled_exact)&& ...
                    isfield(candidate,'r28_native_compiled_correction')&& ...
                    isstruct(candidate.r28_native_compiled_correction))
                obj.generator_validation_fail_count=obj.generator_validation_fail_count+1;
                obj.staged_replan_fail_count=obj.staged_replan_fail_count+1;
                obj.generator_last_status_code=-170;
                obj.staged_replan_last_reason='R28_NATIVE_COMPILE_PACKET_OPEN';
                return;
            end
            obj.r28_last_native_compile_ms=double(candidate.r28_native_compile_ms);
            obj.r28_native_compile_count=obj.r28_native_compile_count+1;

            % Stage1 timing includes the native representation construction
            % because it occurred inside the one AF7R36 MEX call.
            stage_ms=1e3*toc(ts);
            obj.staged_replan_last_stage1_ms=stage_ms;
            obj.generator_last_callback_ms=stage_ms;
            obj.generator_last_cycle_critical_ms=1e3*toc(t_hard_cycle);
            if obj.generator_last_cycle_critical_ms>1e3*dt
                obj.generator_deadline_fail_count=obj.generator_deadline_fail_count+1;
                obj.generator_last_status_code=-171;
                obj.staged_replan_fail_count=obj.staged_replan_fail_count+1;
                obj.staged_replan_last_reason='R28_STAGE1_CONTROL_PERIOD_EXCEEDED';
                return;
            end

            assert(abs(double(candidate.t0)-ta)<=64*eps(max(1,abs(ta))), ...
                '[R26] staged candidate t0 is not scheduled activation time.');

            obj.staged_replan_candidate=candidate;
            obj.staged_replan_rref=struct();
            obj.staged_replan_bundle=struct();
            obj.staged_replan_request_time=t_now;
            obj.staged_replan_activation_time=ta;
            obj.staged_replan_source_mode=char(obj.active_reference_mode);
            obj.staged_replan_source_version=double(obj.active_reference_version);
            obj.staged_replan_known_mask=logical(active(:).');
            obj.staged_replan_static_map=obj.static_obstacle_map;
            obj.staged_replan_dependency_key=obj.staged_reference_only_dependency_key();
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
            obj.staged_replan_last_reason='R28_STAGE1_GENERATED';
            obj.generator_candidate_count=obj.generator_candidate_count+1;
            obj.generator_last_status_code=140;
        end


        function advance_staged_replan_pipeline(obj,t_hard_cycle,t_now, ...
                active,need_replan,obs_id,comp_code)
            state=char(obj.staged_replan_state);
            if strcmp(state,'IDLE'), return; end

            t_now=double(t_now);
            dt=double(obj.prediction_dt);
            ta=double(obj.staged_replan_activation_time);
            t0=double(obj.staged_replan_request_time);
            scale=max([1,abs(t_now),abs(ta),abs(t0)]);
            tol=64*eps(scale);

            if ~logical(need_replan)
                obj.staged_replan_stale_reject_count=obj.staged_replan_stale_reject_count+1;
                obj.reset_staged_replan_pipeline('TRIGGER_CLEARED',false);
                return;
            end

            % Return before deep freshness while merely waiting.
            if strcmp(state,'GENERATED')
                due=t0+dt;
            elseif strcmp(state,'CERTIFIED')
                due=t0+2*dt;
            elseif strcmp(state,'CORE_PREPARED')
                due=ta;
            else
                obj.staged_replan_fail_count=obj.staged_replan_fail_count+1;
                obj.generator_last_status_code=-163;
                obj.reset_staged_replan_pipeline('R26_UNKNOWN_PIPELINE_STATE',true);
                return;
            end
            if t_now<due-tol
                obj.r25_wait_freshness_skip_count=obj.r25_wait_freshness_skip_count+1;
                return;
            end
            if t_now>due+tol
                obj.staged_replan_fail_count=obj.staged_replan_fail_count+1;
                obj.generator_last_status_code=-164;
                obj.reset_staged_replan_pipeline('R26_PIPELINE_GRID_MISSED',true);
                return;
            end

            % Same complete freshness authority as R25, closed immediately
            % before every stage that consumes the stored proposal/bundle.
            fresh = ...
                strcmp(char(obj.active_reference_mode),char(obj.staged_replan_source_mode)) && ...
                double(obj.active_reference_version)==double(obj.staged_replan_source_version) && ...
                isequal(logical(active(:).'),logical(obj.staged_replan_known_mask(:).')) && ...
                isequaln(obj.static_obstacle_map,obj.staged_replan_static_map) && ...
                isequaln(obj.staged_reference_only_dependency_key(),obj.staged_replan_dependency_key) && ...
                isequaln(obj.active_speed_certificate,obj.staged_replan_speed_source) && ...
                isequaln(double(obs_id),double(obj.staged_replan_obstacle_id)) && ...
                isequaln(double(comp_code),double(obj.staged_replan_component_code));
            if ~fresh
                obj.staged_replan_stale_reject_count=obj.staged_replan_stale_reject_count+1;
                obj.generator_last_status_code=-165;
                obj.reset_staged_replan_pipeline('PIPELINE_SOURCE_OR_SENSOR_STALE',true);
                return;
            end

            if strcmp(state,'GENERATED')
                ts=tic;
                P=obj.staged_replan_candidate;
                Rref=obj.evaluate_reference_only_proposal_certificate(P);
                obj.r26_last_reference_cert_ms=1e3*toc(ts);
                obj.reference_only_evaluation_count=obj.reference_only_evaluation_count+1;
                obj.r26_reference_cert_count=obj.r26_reference_cert_count+1;
                obj.staged_replan_last_stage2_ms=obj.r26_last_reference_cert_ms;
                obj.staged_replan_stage2_count=obj.staged_replan_stage2_count+1;
                obj.generator_last_cycle_critical_ms=1e3*toc(t_hard_cycle);

                if ~(isstruct(Rref)&&isfield(Rref,'certified')&&logical(Rref.certified))
                    obj.staged_replan_fail_count=obj.staged_replan_fail_count+1;
                    obj.generator_validation_fail_count=obj.generator_validation_fail_count+1;
                    obj.generator_last_status_code=133;
                    obj.reset_staged_replan_pipeline('R26_REFERENCE_CERTIFICATE_REJECT',true);
                    return;
                end
                if obj.generator_last_cycle_critical_ms>1e3*dt
                    obj.generator_deadline_fail_count=obj.generator_deadline_fail_count+1;
                    obj.staged_replan_fail_count=obj.staged_replan_fail_count+1;
                    obj.generator_last_status_code=-166;
                    obj.reset_staged_replan_pipeline('R26_REFERENCE_CERT_CONTROL_PERIOD_EXCEEDED',true);
                    return;
                end

                obj.staged_replan_rref=Rref;
                obj.staged_replan_state='CERTIFIED';
                obj.staged_replan_last_reason='R28_REFERENCE_CERTIFIED';
                obj.generator_last_status_code=141;
                return;
            end

            if strcmp(state,'CERTIFIED')
                ts=tic;
                P=obj.staged_replan_candidate;

                % Token permits the exact R25 fast obstacle-packet rebuild.
                % Core-only mode routes through this class's SAME-NAME override,
                % which legally calls the exact R21 superclass method and then
                % returns before the inherited R25 preflight compile step.
                obj.r25_stage2_fresh_token_valid=true;
                obj.r25_stage2_fresh_token_source_version=double(obj.active_reference_version);
                obj.r28_core_only_prepare_mode=true;
                c=onCleanup(@()obj.r26_clear_core_prepare_tokens()); %#ok<NASGU>
                [ok,reason,B]=obj.prepare_staged_reference_only_bundle(P,ta);
                clear c;

                obj.r26_last_core_prepare_ms=1e3*toc(ts);
                obj.r26_core_prepare_count=obj.r26_core_prepare_count+1;
                obj.generator_last_cycle_critical_ms=1e3*toc(t_hard_cycle);

                if ~ok
                    obj.staged_replan_fail_count=obj.staged_replan_fail_count+1;
                    obj.generator_validation_fail_count=obj.generator_validation_fail_count+1;
                    obj.generator_last_status_code=131;
                    obj.reset_staged_replan_pipeline(['R26_CORE_PREP_REJECT__' char(reason)],true);
                    return;
                end
                if obj.generator_last_cycle_critical_ms>1e3*dt
                    obj.generator_deadline_fail_count=obj.generator_deadline_fail_count+1;
                    obj.staged_replan_fail_count=obj.staged_replan_fail_count+1;
                    obj.generator_last_status_code=-167;
                    obj.reset_staged_replan_pipeline('R26_CORE_PREP_CONTROL_PERIOD_EXCEEDED',true);
                    return;
                end

                obj.staged_replan_bundle=B;
                obj.staged_replan_state='CORE_PREPARED';
                obj.staged_replan_last_reason='R28_CORE_PREPARED';
                obj.generator_last_status_code=142;
                return;
            end

            % CORE_PREPARED at exact scheduled activation:
            % compile the already-certified canonical correction representation
            % and then execute the unchanged R25 transactional Stage3 commit.
            tFinal=tic;
            B=obj.staged_replan_bundle;

            % R28: the exact compiled packet was already produced incrementally
            % from the generator output and rebound to the canonical Stage3
            % batch model.  Do NOT recompile the same control points here.
            if ~(isstruct(B)&&isfield(B,'r28_native_prevalidated')&& ...
                    logical(B.r28_native_prevalidated)&& ...
                    isfield(B,'reference')&&isfield(B.reference,'r23_compiled_correction'))
                obj.staged_replan_fail_count=obj.staged_replan_fail_count+1;
                obj.generator_validation_fail_count=obj.generator_validation_fail_count+1;
                obj.generator_last_status_code=-168;
                obj.reset_staged_replan_pipeline('R28_FINAL_NATIVE_BINDING_OPEN',true);
                return;
            end
            obj.r26_last_final_compile_ms=0;
            obj.r23_last_compile_ms=0;

            obj.r23_stage3_token_valid=true;
            obj.r23_stage3_token_time=t_now;
            obj.r23_stage3_token_source_version=double(obj.active_reference_version);
            obj.r23_stage3_token_target_version=double(B.target_version);

            taAct=tic;
            try
                [ok,reason]=obj.activate_staged_reference_only_bundle(B,t_now,t_hard_cycle);
            catch ME
                obj.r23_clear_stage3_token();
                rethrow(ME);
            end
            obj.r23_clear_stage3_token();
            obj.r26_last_final_activate_ms=1e3*toc(taAct);
            obj.r26_last_final_total_ms=1e3*toc(tFinal);
            obj.staged_replan_last_stage3_ms=obj.r26_last_final_total_ms;
            obj.r26_final_count=obj.r26_final_count+1;
            obj.generator_last_cycle_critical_ms=1e3*toc(t_hard_cycle);

            if ok
                obj.staged_replan_commit_count=obj.staged_replan_commit_count+1;
                obj.generator_validation_pass_count=obj.generator_validation_pass_count+1;
                obj.generator_last_status_code=130;
                if isstruct(obj.last_replan_status)
                    obj.last_replan_status.pipeline_intervals=3;
                    obj.last_replan_status.r26_load_balanced=true;
                    obj.last_replan_status.r28_native_once=true;
                    obj.last_replan_status.r28_external_incremental_compile=false;
                    obj.last_replan_status.r28_lazy_postcache=true;
                end
                obj.reset_staged_replan_pipeline('R28_FINAL_ATOMIC_COMMIT_CLOSED',false);
            else
                obj.staged_replan_fail_count=obj.staged_replan_fail_count+1;
                obj.generator_validation_fail_count=obj.generator_validation_fail_count+1;
                obj.generator_last_status_code=-169;
                obj.reset_staged_replan_pipeline(['R26_FINAL_COMMIT_REJECT__' char(reason)],true);
            end
        end


        function [ok,reason] = activate_staged_reference_only_bundle(obj,B,t_now,t_hard_cycle)
            tStage=tic;
            ok=false; reason='';
            obj.r23_last_stage3_cache_ms=NaN;
            obj.r23_last_stage3_target_guard_ms=NaN;
            obj.r23_last_stage3_assignment_ms=NaN;
            obj.r23_last_stage3_total_ms=NaN;

            if ~(obj.r23_stage3_token_valid && ...
                    isequal(double(obj.r23_stage3_token_time),double(t_now)) && ...
                    isequal(double(obj.r23_stage3_token_source_version),double(obj.active_reference_version)) && ...
                    isstruct(B)&&isfield(B,'target_version') && ...
                    isequal(double(obj.r23_stage3_token_target_version),double(B.target_version)))
                reason='R23_STAGE3_FRESHNESS_TOKEN_OPEN';
                obj.r23_last_stage3_total_ms=1e3*toc(tStage);
                return;
            end

            if ~(isstruct(B)&&isfield(B,'schema')&& ...
                    strcmp(char(B.schema),'AF7R58ZZY_R21_PREPARED_CORE_V1') && ...
                    isfield(B,'prevalidated')&&logical(B.prevalidated) && ...
                    isfield(B,'r23_compiled_prevalidated')&&logical(B.r23_compiled_prevalidated) && ...
                    isfield(B,'post_commit_cache_pending')&&logical(B.post_commit_cache_pending) && ...
                    isfield(B,'reference')&&isstruct(B.reference) && ...
                    isfield(B.reference,'r23_compiled_correction'))
                reason='R23_PREPARED_BUNDLE_SCHEMA_MISMATCH';
                obj.r23_last_stage3_total_ms=1e3*toc(tStage);
                return;
            end

            scale=max([1,abs(double(t_now)),abs(double(B.activation_time))]);
            if abs(double(t_now)-double(B.activation_time))>64*eps(scale)
                reason='STAGED_ACTIVATION_TIME_MISMATCH';
                obj.r23_last_stage3_total_ms=1e3*toc(tStage);
                return;
            end

            % R28 exact work elimination: no 161-point post-commit cache is
            % materialized on the atomic path.  The cache is explicitly
            % non-authority and is not used by any acceptance certificate.
            % After commit it is invalidated; the next real packet request uses
            % the existing exact compiled evaluator and then installs its normal
            % rolling cache.  This is demand elimination, not a shifted safety
            % computation.
            obj.r23_last_stage3_cache_ms=0;

            if toc(t_hard_cycle)>double(obj.prediction_dt)
                obj.generator_deadline_fail_count=obj.generator_deadline_fail_count+1;
                reason='STAGED_ATOMIC_CONTROL_PERIOD_EXCEEDED';
                obj.r23_last_stage3_total_ms=1e3*toc(tStage);
                return;
            end

            % Validate the TARGET authority before mutation. These are the same
            % binding facts consumed by the production hot guard, but applied
            % directly to the prepared bundle so no post-assignment duplicate
            % structural scan is required.
            tg=tic;
            [targetOK,targetReason]=obj.r23_validate_target_bundle(B);
            obj.r23_last_stage3_target_guard_ms=1e3*toc(tg);
            if ~targetOK
                reason=targetReason;
                obj.r23_last_stage3_total_ms=1e3*toc(tStage);
                return;
            end

            % Full transactional snapshot.  R23-R2 restores not only the
            % authority structs but also every cache/counter/status field that
            % can be mutated after authority assignment.  Therefore any
            % exception or deadline miss remains fail-closed with no partial
            % commit residue.
            old=struct('mode',obj.active_reference_mode, ...
                'version',obj.active_reference_version,'ref',obj.active_repaired_reference, ...
                'cert',obj.active_repaired_certificates,'speed',obj.active_speed_certificate, ...
                'sweep',obj.trigger_sweep_inflation,'packet',obj.obstacle_packet, ...
                'sensor',obj.sensor_horizon_certificate,'dep',obj.runtime_dependency_signature, ...
                'stale',obj.certificate_stale,'level',obj.active_repaired_safety_level, ...
                'time',obj.last_replan_commit_time, ...
                'repairedCache',obj.repaired_reference_cache, ...
                'repairedCacheValid',obj.repaired_reference_cache_valid, ...
                'preparedCache',obj.reference_only_prepared_packet_cache, ...
                'preparedCacheValid',obj.reference_only_prepared_packet_cache_valid, ...
                'postcacheCount',obj.staged_replan_postcommit_cache_prepare_count, ...
                'replanCount',obj.replan_commit_count, ...
                'roCount',obj.reference_only_commit_count, ...
                'requestActive',obj.replan_request_active, ...
                'lastStatus',obj.last_replan_status, ...
                'r28LazyPending',obj.r28_lazy_postcache_pending);

            ta=tic;
            try
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
                obj.r23_last_stage3_assignment_ms=1e3*toc(ta);

                % Preserve production fail-closed deadline semantics. If the
                % authority assignment itself crosses the period, roll back.
                if toc(t_hard_cycle)>double(obj.prediction_dt)
                    obj.r23_restore_authority(old);
                    obj.generator_deadline_fail_count=obj.generator_deadline_fail_count+1;
                    reason='STAGED_ATOMIC_CONTROL_PERIOD_EXCEEDED';
                    obj.r23_last_stage3_total_ms=1e3*toc(tStage);
                    return;
                end

                obj.repaired_reference_cache=struct();
                obj.repaired_reference_cache_valid=false;
                obj.reference_only_prepared_packet_cache_valid=false;
                obj.reference_only_prepared_packet_cache=struct();
                obj.r28_lazy_postcache_commit_count=obj.r28_lazy_postcache_commit_count+1;
                obj.r28_lazy_postcache_pending=true;

                obj.replan_commit_count=obj.replan_commit_count+1;
                obj.reference_only_commit_count=obj.reference_only_commit_count+1;
                obj.replan_request_active=false;
                obj.last_replan_status=struct('requested',true,'committed',true, ...
                    'safety_mode','REFERENCE_ONLY','safety_level','REFERENCE_ONLY', ...
                    'full_system_authority',false, ...
                    'reference_version',double(B.target_version), ...
                    'backend','BEZIER_D15_C7','commit_time',double(t_now), ...
                    'rejoin_time',double(B.reference.rejoin_time), ...
                    'staged_pipeline',true,'pipeline_intervals',3);

                ok=true;
                reason='STAGED_REFERENCE_ONLY_COMMIT_CLOSED';
            catch ME
                obj.r23_restore_authority(old);
                reason=ME.message;
            end
            obj.r23_last_stage3_total_ms=1e3*toc(tStage);
        end
    end

    methods

        function P = af7r46_single_core_proposal_compact(obj,CTX)
            % R25 exact reuse of already-certified AF7R33 base model.
            assert(exist('af7r36_single_core_mex','file')==3, ...
                '[AF7R46] AF7R36 native core MEX missing.');

            txPacket=tic;
            model=[];
            if strcmpi(char(obj.active_reference_mode),'NOMINAL') && ...
                    obj.nominal_batch_model_cache_valid && ...
                    isstruct(obj.nominal_batch_model_cache) && ...
                    isfield(obj.nominal_batch_model_cache,'certified') && ...
                    logical(obj.nominal_batch_model_cache.certified) && ...
                    isfield(obj.nominal_batch_model_cache,'reference_version') && ...
                    double(obj.nominal_batch_model_cache.reference_version)== ...
                        double(obj.active_reference_version)
                model=obj.nominal_batch_model_cache;
                obj.r25_nominal_core_cache_hit_count= ...
                    obj.r25_nominal_core_cache_hit_count+1;
            elseif strcmpi(char(obj.active_reference_mode),'REPAIRED')
                Ractive=obj.active_repaired_reference;
                if isstruct(Ractive) && isfield(Ractive,'batch_flat_model') && ...
                        isstruct(Ractive.batch_flat_model) && ...
                        isfield(Ractive.batch_flat_model,'certified') && ...
                        logical(Ractive.batch_flat_model.certified)
                    model=Ractive.batch_flat_model;
                    obj.r25_repaired_core_cache_hit_count= ...
                        obj.r25_repaired_core_cache_hit_count+1;
                end
            end
            pkt=af7r34_make_core_packet(obj,CTX,[],model);
            targetVersion=double(CTX.request.next_reference_version);
            pkt.r28_target_version=targetVersion;
            pkt.r28_compiled_base=obj.r28_active_compiled_packet_for_native();
            obj.r25_last_core_packet_ms=1e3*toc(txPacket);
            RAW=af7r36_single_core_mex(1,pkt);

            P=struct();
            P.shadow_proposal=true;
            P.af7r46_compact_native=true;
            P.generated=logical(RAW.success);
            P.success=logical(RAW.success);
            P.reason_code=double(RAW.reason_code);
            P.backend='BEZIER_D15_C7';
            P.t0=double(RAW.t0);
            P.tm=double(RAW.tm);
            P.trigger_collision_time=double(RAW.tm);
            P.hold_end=double(RAW.hold_end);
            P.t1=double(RAW.t1);
            P.obstacle_id=double(CTX.request.trigger_obstacle_id);
            P.component_code=double(CTX.request.trigger_component_code);
            P.execution_tracking_inflation_m=0;
            P.r9_stage_timing=struct( ...
                'episode_ms',double(RAW.episode_ms), ...
                'synthesis_ms',double(RAW.synthesis_ms), ...
                'candidate_loop_ms',double(RAW.candidate_ms), ...
                'total_internal_ms',double(RAW.total_ms));
            P.native_raw=RAW;
            P.r28_native_compiled_exact=logical(RAW.compiled_native_exact);
            P.r28_native_compile_ms=double(RAW.compiled_native_ms);
            P.r28_native_revision=char(RAW.compiled_native_revision);
            P.r28_native_compiled_correction=RAW.compiled_correction;

            if ~P.success
                return;
            end
            assert(P.r28_native_compiled_exact&& ...
                strcmp(P.r28_native_revision,'AF7R58ZZY_R28_NATIVE_ONCE_V1'), ...
                '[R28] native compiled representation was not produced by the exact R28 MEX.');

            P.c7_exact=true;
            P.representation_code=2;
            P.degree=15;
            P.piecewise_segment_t0=double(RAW.segment_t0(:).');
            P.piecewise_segment_t1=double(RAW.segment_t1(:).');
            P.piecewise_control_points=RAW.control_points;
            P.displacement_norm_m=double(RAW.max_control_norm_m);
            P.displacement=double(RAW.displacement(:));
            P.direction=double(RAW.direction(:));

            SC=struct();
            SC.certified=true;
            SC.continuous_time=true;
            SC.sampled_acceptance=false;
            SC.reference_mode='REPAIRED';
            SC.reference_version=targetVersion;
            SC.payload_speed_UB_m_s=double(RAW.speed_payload);
            SC.payload_accel_UB_m_s2=double(RAW.speed_accel);
            SC.payload_jerk_UB_m_s3=double(RAW.speed_jerk);
            SC.specific_force_norm_LB_m_s2=double(RAW.speed_sigma);
            SC.qdot_UB_rad_s=double(RAW.speed_qdot);
            SC.uav_speed_UB_m_s=double(RAW.speed_uav);
            SC.base_reference_mode=char(obj.active_reference_mode);
            SC.base_reference_version=double(obj.active_reference_version);
            if logical(RAW.speed_stageA)
                SC.source='TRIANGLE_BOUNDED_C7_REPAIRED_FLATNESS_KINEMATICS';
                SC.certificate_stage='R9_STAGE_A_GLOBAL_TRIANGLE';
            else
                SC.source='R9_LOCAL_VECTOR_COUPLED_ADAPTIVE_BERNSTEIN_FLATNESS_SPEED';
                SC.certificate_stage='R9_STAGE_B_LOCAL_VECTOR';
                SC.local_leaf_count=double(RAW.speed_leaf_count);
                SC.local_split_count=double(RAW.speed_split_count);
                SC.vector_coupled_payload_velocity=true;
                SC.suspended_load_flatness=true;
                SC.fixed_depth_cap=false;
            end
            P.reference_speed_certificate=SC;

            G=struct();
            G.mathematical_pass=logical(RAW.geometry_pass);
            G.certified=logical(RAW.geometry_pass);
            G.continuous_time=true;
            G.whole_body=true;
            G.all_detected_obstacles=true;
            G.static_obstacles_only=true;
            G.sampled_acceptance=false;
            G.finite_cover_speed_tube=true;
            G.support_function_separation=true;
            G.corrected_flatness_cable_direction=true;
            G.ieee754_outward_closed=false;
            G.reason_code=double(RAW.geometry_reason);
            G.min_payload_margin_m=double(RAW.geometry_min_payload_margin);
            G.min_uav_margin_m=double(RAW.geometry_min_uav_margin);
            G.min_cable_margin_m=double(RAW.geometry_min_cable_margin);
            G.min_margin_m=double(RAW.geometry_min_margin);
            G.leaf_count=double(RAW.geometry_leaf_count);
            G.accepted_leaf_count=double(RAW.geometry_accepted_leaf_count);
            G.split_count=double(RAW.geometry_split_count);
            G.max_depth_used=double(RAW.geometry_max_depth);
            G.max_depth_cap=Inf;
            G.resource_exhausted=false;
            G.static_separation_open=G.reason_code==111;
            G.representability_open=G.reason_code==112;
            G.cover_theorem='MIDPOINT_STATIC_SUPPORT_MINUS_SPEED_TIMES_HALFWIDTH';
            G.min_specific_force_norm_lb=double(RAW.speed_sigma);
            G.max_qdot_ub=double(RAW.speed_qdot);
            G.max_q_arc_radius=double(RAW.speed_qdot)*double(CTX.live.dt)/2;
            G.max_q_arc_inflation_m=double(CTX.live.cable_length)*G.max_q_arc_radius;
            G.tracking_inflation_m=0;
            G.reference_only_authority=true;
            G.final_system_authority=false;
            P.geometry_shadow_certificate=G;

            obj.compact_native_proposal_count=obj.compact_native_proposal_count+1;
        end

        function pack = prepare_obstacle_packet_fast_target(obj,oldPacket,sweep,targetVersion)
            % R25 exact Stage2 fast path.  Under the token, advance() has just
            % closed the complete map/dependency/speed freshness gate.
            tfFast = obj.r25_stage2_fresh_token_valid && ...
                isequal(double(obj.r25_stage2_fresh_token_source_version), ...
                        double(obj.active_reference_version));

            tFast=tic;
            if tfFast
                M=numel(obj.static_obstacle_map);
                assert(isstruct(oldPacket)&&double(oldPacket.count)==M);
                assert(isstruct(sweep)&&isfield(sweep,'certified')&&logical(sweep.certified));
                assert(isfield(oldPacket,'Araw')&&isequal(size(oldPacket.Araw),[3 M]), ...
                    '[R25] static packet raw axes unavailable.');

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
                    assert(numel(ar)==3&&all(isfinite(ar))&&all(ar>0), ...
                        '[R25] cached raw obstacle ellipsoid invalid.');
                    [pack.Apay(:,i),pack.Auav(:,i),pack.Acable(:,i)]= ...
                        obj.outer_ellipsoid_triplet_from_raw_cached(o,ar,Dp,Dq,Dc);
                    if isfield(o,'v_center') && norm(o.v_center(:))>0
                        error('[REPLANNER ERROR] moving obstacle entered static fast packet path.');
                    end
                end
                obj.r25_fast_packet_rebuild_count=obj.r25_fast_packet_rebuild_count+1;
                obj.r25_last_fast_packet_rebuild_ms=1e3*toc(tFast);
                return;
            end

            % Exact inherited fallback for setup/prime/non-Stage2 contexts.
            M=numel(obj.static_obstacle_map);
            assert(isstruct(oldPacket)&&double(oldPacket.count)==M);
            assert(isstruct(sweep)&&isfield(sweep,'certified')&&logical(sweep.certified));
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
                assert(isequal(double(pack.ids(i)),double(o.id)) && ...
                    isequal(double(pack.C(:,i)),double(o.p_center(:))) && ...
                    isequal(double(pack.R(:,i)),double(o.R_obs(:))) && ...
                    strcmpi(char(pack.types{i}),char(o.type)), ...
                    '[REPLANNER ERROR] static obstacle authority changed during fast packet rebuild.');
                ar=double(pack.Araw(:,i));
                assert(isequal(ar,double(obj.raw_enclosing_ellipsoid_axes(o))), ...
                    '[REPLANNER ERROR] raw obstacle ellipsoid changed during fast packet rebuild.');
                [pack.Apay(:,i),pack.Auav(:,i),pack.Acable(:,i)]= ...
                    obj.outer_ellipsoid_triplet_from_raw_cached(o,ar,Dp,Dq,Dc);
                if isfield(o,'v_center') && norm(o.v_center(:))>0
                    error('[REPLANNER ERROR] moving obstacle entered static fast packet path.');
                end
            end
            obj.r25_last_fast_packet_rebuild_ms=1e3*toc(tFast);
        end


    end

    methods (Access = private)
        function CP = r28_active_compiled_packet_for_native(obj)
            if strcmpi(char(obj.active_reference_mode),'NOMINAL')
                CP=struct();
                return;
            end
            assert(strcmpi(char(obj.active_reference_mode),'REPAIRED'), ...
                '[R28] unknown active reference mode for native compile.');
            R=obj.active_repaired_reference;
            R=obj.r23_attach_compiled_for_preflight_prime_if_needed(R);
            assert(isstruct(R)&&isfield(R,'r23_compiled_correction')&& ...
                isstruct(R.r23_compiled_correction)&& ...
                double(R.r23_compiled_correction.reference_version)== ...
                    double(obj.active_reference_version)&& ...
                isfield(R,'r23_compiled_runtime_exact')&&logical(R.r23_compiled_runtime_exact), ...
                '[R28] active compiled/canonical authority invariant unavailable.');
            CP=R.r23_compiled_correction;
        end

        function r28_validate_native_binding(obj,CP,model,P,targetVersion)
            % Runtime check is intentionally metadata-only.  Exact D0..D6
            % correspondence is established by construction in the native MEX
            % and bitwise install selftests.  Rebuilding all CP here would
            % recreate the R28 cost that R28 is designed to eliminate.
            tb=tic;
            assert(isstruct(CP)&&isfield(CP,'schema')&& ...
                strcmp(char(CP.schema),'AF7R58ZZY_R23_COMPILED_CORRECTION_V1')&& ...
                double(CP.reference_version)==double(targetVersion)&& ...
                logical(CP.certified_numeric_representation)&& ...
                double(CP.degree)==15&&double(CP.max_derivative_order)==6&& ...
                logical(CP.strict_support)&& ...
                strcmp(char(CP.group_addition_order),'ASCENDING_CANONICAL_GROUP_INDEX'), ...
                '[R28] native compiled packet contract failed.');
            assert(isstruct(model)&&isfield(model,'certified')&&logical(model.certified)&& ...
                isfield(model,'corrections')&&isstruct(model.corrections), ...
                '[R28] canonical model binding failed.');
            C=model.corrections; G=numel(C); S=0;
            for g=1:G
                S=S+numel(C(g).segment_t0);
            end
            assert(double(CP.group_count)==G&&double(CP.segment_count)==S, ...
                '[R28] native/canonical resource counts differ.');
            assert(G>=1&&isstruct(P)&&isfield(P,'piecewise_segment_t0')&& ...
                isfield(P,'piecewise_segment_t1'), ...
                '[R28] appended candidate metadata unavailable.');
            ns=numel(P.piecewise_segment_t0);
            assert(double(CP.group_segments(end))==ns&& ...
                isequal(typecast(double(CP.group_t0(end)),'uint64'),typecast(double(P.t0),'uint64'))&& ...
                isequal(typecast(double(CP.group_t1(end)),'uint64'),typecast(double(P.t1),'uint64')), ...
                '[R28] appended group support/count mismatch.');
            first=round(double(CP.group_first(end)));
            ix=first:(first+ns-1);
            assert(numel(ix)==ns&&ix(1)>=1&&ix(end)<=double(CP.segment_count)&& ...
                isequal(typecast(double(CP.segment_t0(ix)),'uint64'), ...
                        typecast(double(P.piecewise_segment_t0(:).'),'uint64'))&& ...
                isequal(typecast(double(CP.segment_t1(ix)),'uint64'), ...
                        typecast(double(P.piecewise_segment_t1(:).'),'uint64')), ...
                '[R28] appended segment support mismatch.');
            obj.r28_last_binding_ms=1e3*toc(tb);
        end

        function r25_clear_stage2_token(obj)
            obj.r25_stage2_fresh_token_valid=false;
            obj.r25_stage2_fresh_token_source_version=NaN;
        end


        function r26_clear_core_prepare_tokens(obj)
            obj.r25_stage2_fresh_token_valid=false;
            obj.r25_stage2_fresh_token_source_version=NaN;
            obj.r28_core_only_prepare_mode=false;
        end


        function r23_clear_stage3_token(obj)
            obj.r23_stage3_token_valid=false;
            obj.r23_stage3_token_time=NaN;
            obj.r23_stage3_token_source_version=NaN;
            obj.r23_stage3_token_target_version=NaN;
        end

        function r23_restore_authority(obj,old)
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
            if isfield(old,'repairedCache')
                obj.repaired_reference_cache=old.repairedCache;
                obj.repaired_reference_cache_valid=old.repairedCacheValid;
                obj.reference_only_prepared_packet_cache=old.preparedCache;
                obj.reference_only_prepared_packet_cache_valid=old.preparedCacheValid;
                obj.staged_replan_postcommit_cache_prepare_count=old.postcacheCount;
                obj.replan_commit_count=old.replanCount;
                obj.reference_only_commit_count=old.roCount;
                obj.replan_request_active=old.requestActive;
                obj.last_replan_status=old.lastStatus;
                if isfield(old,'r28LazyPending'), obj.r28_lazy_postcache_pending=old.r28LazyPending; end
            end
        end

        function R = r23_attach_compiled_for_preflight_prime_if_needed(obj,R)
            % R23-R3 PRE-MOTION PRIME BRIDGE ONLY.
            %
            % The inherited production prime temporarily commits a synthetic
            % REFERENCE_ONLY candidate through try_commit_reference_only_proposal()
            % and then calls repaired_reference_packet() three times to burn the
            % full-build / cache-hit / shift-1 paths.  That historical temporary
            % authority predates R23 and therefore has no runtime-only compiled
            % correction metadata.  Attach the exact compiled representation only
            % during that pre-motion prime.  Real Flight may never use this bridge.
            if isstruct(R) && isfield(R,'r23_compiled_correction')
                return;
            end

            preflightPrime = ...
                ~logical(obj.online_prime_done) && ...
                ~logical(obj.generator_backend_kernel_prime_done) && ...
                ~logical(obj.postcallback_prime_done);
            assert(preflightPrime, ...
                ['[R23-R3] missing compiled correction outside the pre-motion ' ...
                 'synthetic postcallback prime.']);

            assert(strcmpi(char(obj.active_reference_mode),'REPAIRED') && ...
                isstruct(R) && isfield(R,'valid') && logical(R.valid) && ...
                isfield(R,'reference_version') && ...
                double(R.reference_version)==double(obj.active_reference_version) && ...
                isfield(R,'batch_flat_model') && isstruct(R.batch_flat_model) && ...
                isfield(R.batch_flat_model,'certified') && ...
                logical(R.batch_flat_model.certified) && ...
                isfield(R,'eval_fn') && isa(R.eval_fn,'function_handle') && ...
                isfield(R,'flat_eval_fn') && isa(R.flat_eval_fn,'function_handle'), ...
                '[R23-R3] preflight repaired authority is not the closed canonical model.');

            CP=af7r77_compile_correction_packet_R25( ...
                R.batch_flat_model,double(R.reference_version));
            obj.r23_validate_compiled_packet_binding( ...
                CP,R.batch_flat_model,double(R.reference_version));
            R.r23_compiled_correction=CP;
            R.r23_compiled_runtime_exact=true;
            R.r23_canonical_eval_retained=true;
            if ~isfield(R,'authority_split') || ~isstruct(R.authority_split)
                R.authority_split=struct();
            end
            R.authority_split.runtime_compiled_correction= ...
                'R23_STRICT_FP_SAME_CANONICAL_CORRECTION';

            % The inherited prime snapshots/restores the entire repaired authority
            % after its three burn calls, so this metadata cannot survive into
            % real Flight.  No certificate or mathematical authority is changed.
            obj.active_repaired_reference=R;
        end


        function r23_validate_compiled_packet_binding(~,P,model,targetVersion)
            assert(isstruct(P)&&isfield(P,'schema')&& ...
                strcmp(char(P.schema),'AF7R58ZZY_R23_COMPILED_CORRECTION_V1') && ...
                isfield(P,'reference_version')&&double(P.reference_version)==double(targetVersion) && ...
                isfield(P,'certified_numeric_representation')&&logical(P.certified_numeric_representation), ...
                '[R23] compiled packet binding failed.');
            assert(isstruct(model)&&isfield(model,'certified')&&logical(model.certified)&& ...
                isfield(model,'corrections')&&isstruct(model.corrections) && ...
                double(P.group_count)==numel(model.corrections), ...
                '[R23] compiled packet/model group binding failed.');
            s=0;
            for g=1:numel(model.corrections)
                s=s+numel(model.corrections(g).segment_t0);
            end
            assert(double(P.segment_count)==s,'[R23] compiled packet/model segment binding failed.');
        end

        function r23_assert_compiled_reference_binding(obj,R,t_now)
            assert(strcmpi(char(obj.active_reference_mode),'REPAIRED'), ...
                '[R23] compiled repaired evaluator used outside REPAIRED mode.');
            assert(isstruct(R)&&isfield(R,'valid')&&logical(R.valid)&& ...
                isfield(R,'reference_version')&& ...
                double(R.reference_version)==double(obj.active_reference_version)&& ...
                isfield(R,'r23_compiled_correction')&& ...
                isfield(R,'eval_fn')&&isa(R.eval_fn,'function_handle')&& ...
                isfield(R,'flat_eval_fn')&&isa(R.flat_eval_fn,'function_handle'), ...
                '[R23] repaired reference/compiled authority binding failed.');
            assert(double(R.r23_compiled_correction.reference_version)== ...
                double(obj.active_reference_version), ...
                '[R23] compiled correction reference version is stale.');
            assert(isfinite(double(R.valid_from))&& ...
                (isfinite(double(R.valid_until))||isinf(double(R.valid_until)))&& ...
                t_now>=double(R.valid_from)&&t_now<=double(R.valid_until), ...
                '[R23] repaired reference queried outside validity interval.');
            assert(~obj.certificate_stale.SPEED_TRIGGER_CERT && ...
                ~obj.certificate_stale.SENSOR_HORIZON_CERT && ...
                ~obj.certificate_stale.STOP_GEOMETRY_4MS_CERT, ...
                '[R23] runtime core certificate stale bit is set.');
            C=obj.active_repaired_certificates;
            assert(strcmpi(char(obj.active_repaired_safety_level),'REFERENCE_ONLY') && ...
                isstruct(C)&&isfield(C,'reference_only')&&logical(C.reference_only)&& ...
                isfield(C,'smoothness')&&logical(C.smoothness.certified)&& ...
                logical(C.smoothness.continuous_time)&& ...
                isfield(C,'geometry')&&logical(C.geometry.certified)&& ...
                logical(C.geometry.continuous_time)&&~logical(C.geometry.sampled_acceptance), ...
                '[R23] reference-only continuous certificate binding failed.');
            SC=obj.active_speed_certificate;
            assert(isstruct(SC)&&isfield(SC,'certified')&&logical(SC.certified)&& ...
                strcmpi(char(SC.reference_mode),'REPAIRED')&& ...
                double(SC.reference_version)==double(obj.active_reference_version), ...
                '[R23] active speed certificate binding failed.');
        end

        function [base,ok] = r23_nominal_xd_grid(obj,t_now,xd_nom)
            ok=false; base=[];
            if ~(obj.nominal_table_valid&&isfinite(obj.nominal_table_t0)&& ...
                    isfinite(obj.nominal_table_dt)&&obj.nominal_table_dt>0)
                return;
            end
            idx=round((double(t_now)-double(obj.nominal_table_t0))/double(obj.nominal_table_dt))+1;
            if idx<1||idx>size(obj.nominal_table_xd,2), return; end
            tg=double(obj.nominal_table_t0)+(idx-1)*double(obj.nominal_table_dt);
            tol=1e-10*max(1,abs(double(t_now)));
            if abs(tg-double(t_now))>tol, return; end
            auth=double(obj.nominal_table_xd(:,idx));
            in=double(xd_nom(:));
            if ~(isequal(size(auth),size(in)) && ...
                    isequal(typecast(auth(:),'uint64'),typecast(in(:),'uint64')))
                return;
            end
            base=auth; ok=true;
        end

        function [P,A,ok] = r23_posaccel_grid(obj,tvec,CP)
            ok=false; P=[]; A=[];
            tvec=double(tvec(:).'); N=numel(tvec);
            if ~(N>=1&&all(isfinite(tvec))&&obj.nominal_table_valid&& ...
                    obj.r19_nominal_accel_table_valid&& ...
                    size(obj.r19_nominal_accel_table,2)==size(obj.nominal_table_xd,2))
                return;
            end
            dt=double(obj.nominal_table_dt);
            idx=round((tvec-double(obj.nominal_table_t0))/dt)+1;
            if any(idx<1)||any(idx>size(obj.nominal_table_xd,2)), return; end
            tg=double(obj.nominal_table_t0)+(idx-1)*dt;
            tol=1e-10*max(1,max(abs(tvec)));
            if any(abs(tg-tvec)>tol), return; end
            P0=double(obj.nominal_table_xd(1:3,idx));
            A0=double(obj.r19_nominal_accel_table(:,idx));
            assert(isequal(size(P0),[3 N])&&isequal(size(A0),[3 N])&& ...
                all(isfinite(P0),'all')&&all(isfinite(A0),'all'), ...
                '[R23] nominal position/acceleration table slice invalid.');
            [P,A]=af7r76_compiled_eval_mex(2,tvec,P0,A0,CP);
            P=double(P); A=double(A);
            assert(isequal(size(P),[3 N])&&isequal(size(A),[3 N])&& ...
                all(isfinite(P),'all')&&all(isfinite(A),'all'), ...
                '[R23] compiled position/acceleration output invalid.');
            ok=true;
        end

        function [ok,reason] = r23_validate_target_bundle(obj,B)
            ok=false; reason='R23_TARGET_BINDING_OPEN';
            try
                tv=double(B.target_version); dt=double(obj.prediction_dt);
                R=B.reference; C=B.certificates; SC=B.speed; SW=B.sweep;
                OP=B.packet; SH=B.sensor;
                assert(isstruct(R)&&isfield(R,'valid')&&logical(R.valid)&& ...
                    double(R.reference_version)==tv&& ...
                    isa(R.eval_fn,'function_handle')&&isa(R.flat_eval_fn,'function_handle')&& ...
                    isfield(R,'r23_compiled_correction')&& ...
                    double(R.r23_compiled_correction.reference_version)==tv);
                assert(isstruct(C)&&isfield(C,'reference_only')&&logical(C.reference_only)&& ...
                    isfield(C,'smoothness')&&logical(C.smoothness.certified)&& ...
                    logical(C.smoothness.continuous_time)&& ...
                    isfield(C,'geometry')&&logical(C.geometry.certified)&& ...
                    logical(C.geometry.continuous_time)&&~logical(C.geometry.sampled_acceptance));
                assert(isstruct(SC)&&logical(SC.certified)&& ...
                    strcmpi(char(SC.reference_mode),'REPAIRED')&&double(SC.reference_version)==tv&& ...
                    isfinite(double(SC.payload_speed_UB_m_s))&&double(SC.payload_speed_UB_m_s)>=0&& ...
                    isfinite(double(SC.uav_speed_UB_m_s))&&double(SC.uav_speed_UB_m_s)>=0&& ...
                    isfinite(double(SC.qdot_UB_rad_s))&&double(SC.qdot_UB_rad_s)>=0);
                assert(isstruct(SW)&&logical(SW.certified)&& ...
                    strcmpi(char(SW.reference_mode),'REPAIRED')&& ...
                    double(SW.reference_version)==tv&&double(SW.dt)==dt);
                assert(isstruct(OP)&&logical(OP.sweep_certified)&& ...
                    strcmpi(char(OP.reference_mode),'REPAIRED')&& ...
                    double(OP.reference_version)==tv&&double(OP.sweep_dt)==dt);
                assert(isstruct(SH)&&logical(SH.certified)&& ...
                    strcmpi(char(SH.reference_mode),'REPAIRED')&& ...
                    double(SH.reference_version)==tv&&double(SH.dt)==dt&& ...
                    double(SH.horizon)==double(obj.horizon_s));
                ok=true; reason='R23_TARGET_BINDING_CLOSED';
            catch ME
                reason=['R23_TARGET_BINDING_REJECT__' ME.identifier];
            end
        end
    end
end
