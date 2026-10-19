classdef REPLANNING_SYSTEM_R15_FAST < REPLANNING_SYSTEM
% AF7R58ZZY-R15 | LEAN EXACT PERFORMANCE INTEGRATION
% Direct production subclass: no R5/R8/R13 diagnostic inheritance.
% Exact changes only:
%   1) observer one-step + two-event contraction -> validated AOT/MEX kernel
%   2) AF7R33 batch de Casteljau -> validated bitwise-exact MEX helper
% No trajectory/certificate/commit-policy/controller changes.

    properties (SetAccess = private)
        r15_build_id = 'AF7R58ZZY_R15_FAST_V1';
    end

    methods
        function obj = REPLANNING_SYSTEM_R15_FAST(self,base_ref,opts)
            obj@REPLANNING_SYSTEM(self,base_ref,opts);
            assert(exist('af7r70_observer_mex','file')==3, ...
                '[R15] observer MEX is missing. Run AF7R58ZZY_R15_INSTALL first.');
            assert(exist('af7r71_decasteljau_batch_mex','file')==3, ...
                '[R15] de Casteljau MEX is missing. Run AF7R58ZZY_R15_INSTALL first.');
            assert(exist('af7r33_flat_jet_batch_model_R15FAST','file')==2, ...
                '[R15] fast AF7R33 helper is missing. Run AF7R58ZZY_R15_INSTALL first.');
        end

        function S = observer_one_step_support(obj,h,u,I)
            if obj.observer_static_setup_cache_valid
                S=af7r70_observer_mex(1,0,h,u,I,obj.observer_static_setup_cache);
                return;
            end
            S=observer_one_step_support@REPLANNING_SYSTEM(obj,h,u,I);
        end

        function O = observer_contract_two_events(obj,C0,C1,F,xhat)
            O=af7r70_observer_mex(2,0,C0,C1,F, ...
                double(xhat.p(:)),double(xhat.q(:)), ...
                double(xhat.pL(:)),double(xhat.pT(:)), ...
                double(xhat.vL(:)),double(xhat.wL(:)),double(xhat.w(:)));
        end

        function [ok,reason,B] = prepare_staged_reference_only_bundle(obj,P,t_commit)
            ok=false;
            reason='';
            B=struct();

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

            [resourceOK,resourceReason,batchModel,resourceMeta]= ...
                af7r58i_prepare_batch_model( ...
                    obj,P,targetVersion,double(t_commit));
            if ~resourceOK
                reason=resourceReason; return;
            end

            % Exact rebuild is intentional in the staged-preparation cycle.
            % No cache/reuse theorem is needed for timing here.
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
            [Ycache,validCache]= ...
                af7r33_flat_jet_batch_model_R15FAST(obj,tqCache,batchModel);
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
        end

        function [pL_ref,q_ref,source] = repaired_reference_packet(obj,t_now)
            % AF7R41 exact rolling REPAIRED future-reference packet.
            % Full rebuild uses the AF7R33 batch flat model.  On the normal
            % one-dt live path, N-1 exact columns are reused and only the new
            % tail point is evaluated.  No interpolation is used.
            obj.assert_repaired_reference_current();

            N=round(obj.horizon_s/obj.prediction_dt)+1;
            dt=double(obj.prediction_dt);
            H=double(obj.horizon_s);
            t0=double(t_now);
            R=obj.active_repaired_reference;

            assert(isfinite(dt)&&dt>0 && isfinite(H)&&H>=0, ...
                '[REPLANNER ERROR] invalid repaired packet timing parameters.');
            assert(isfield(R,'batch_flat_model') && isstruct(R.batch_flat_model) && ...
                isfield(R.batch_flat_model,'certified') && logical(R.batch_flat_model.certified), ...
                '[REPLANNER ERROR] repaired batch flat model unavailable.');
            assert(isfield(R,'flat_eval_fn') && isa(R.flat_eval_fn,'function_handle'), ...
                '[REPLANNER ERROR] repaired flat evaluator unavailable.');

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
                source=sprintf('REPAIRED_ROLLING_CACHE_HIT:v%d:%s', ...
                    obj.active_reference_version,char(R.backend));
                obj.validate_reference_packet(pL_ref,q_ref,N);
                return;
            end

            if canReuse && isfinite(shiftCount) && shiftCount>=1 && shiftCount<N
                m=double(shiftCount);
                pL_ref=[C.pL(:,m+1:end),zeros(3,m)];
                q_ref=[C.q(:,m+1:end),zeros(3,m)];

                if m==1
                    xf=double(R.flat_eval_fn(t0+(N-1)*dt));
                    xf=xf(:);
                    assert(numel(xf)>=11 && all(isfinite(xf(1:11))), ...
                        '[REPLANNER ERROR] repaired rolling tail evaluator invalid.');
                    pL_ref(:,end)=xf(1:3);
                    sf=xf(9:11)+[0;0;double(obj.gravity)];
                    ns=norm(sf);
                    assert(isfinite(ns)&&ns>0, ...
                        '[REPLANNER ERROR] repaired rolling tail flatness singular.');
                    q_ref(:,end)=-sf/ns;
                    source=sprintf('REPAIRED_ROLLING_SHIFT1:v%d:%s', ...
                        obj.active_reference_version,char(R.backend));
                else
                    tq=t0+(N-m:N-1)*dt;
                    [Y,valid]=af7r33_flat_jet_batch_model_R15FAST(obj,tq,R.batch_flat_model);
                    Y=double(Y);
                    assert(isequal(size(Y),[21 m]) && numel(valid)==m && ...
                        all(logical(valid(:))) && all(isfinite(Y),'all'), ...
                        '[REPLANNER ERROR] repaired rolling tail batch invalid.');
                    pL_ref(:,end-m+1:end)=Y(1:3,:);
                    sf=Y(7:9,:)+[0;0;double(obj.gravity)];
                    ns=sqrt(sum(sf.^2,1));
                    assert(all(isfinite(ns))&&all(ns>0), ...
                        '[REPLANNER ERROR] repaired rolling tail batch flatness singular.');
                    q_ref(:,end-m+1:end)=-sf./ns;
                    source=sprintf('REPAIRED_ROLLING_SHIFTM:v%d:%s', ...
                        obj.active_reference_version,char(R.backend));
                end
            else
                tq=t0+(0:N-1)*dt;
                [Y,valid]=af7r33_flat_jet_batch_model_R15FAST(obj,tq,R.batch_flat_model);
                Y=double(Y);
                assert(isequal(size(Y),[21 N]) && numel(valid)==N && ...
                    all(logical(valid(:))) && all(isfinite(Y),'all'), ...
                    '[REPLANNER ERROR] repaired full batch packet invalid.');
                pL_ref=Y(1:3,:);
                sf=Y(7:9,:)+[0;0;double(obj.gravity)];
                ns=sqrt(sum(sf.^2,1));
                assert(all(isfinite(ns))&&all(ns>0), ...
                    '[REPLANNER ERROR] repaired full batch flatness singular.');
                q_ref=-sf./ns;
                source=sprintf('REPAIRED_FULL_BATCH:v%d:%s', ...
                    obj.active_reference_version,char(R.backend));
            end

            obj.validate_reference_packet(pL_ref,q_ref,N);
            obj.repaired_reference_cache=struct( ...
                'reference_version',double(obj.active_reference_version), ...
                't0',t0,'dt',dt,'H',H,'pL',pL_ref,'q',q_ref);
            obj.repaired_reference_cache_valid=true;
        end

    end
end
