classdef REPLANNING_SYSTEM_R19_FAST < REPLANNING_SYSTEM
% AF7R58ZZY-R19 | NATIVE CORRECTION MAPPING + EXACT LEAN STAGE2
% Direct production subclass: no R5/R8/R13 diagnostic inheritance.
% Exact changes only:
%   1) observer one-step + two-event contraction -> validated AOT/MEX kernel
%   2) Stage2 exact runtime-packet reuse gate -> existing production theorem
%   3) repaired packet/postcache correction mapping+r={0,2} -> one strict-FP native kernel
% No trajectory/certificate/commit-policy/controller changes.

    properties (SetAccess = private)
        r19_build_id = 'AF7R58ZZY_R19_FAST_V1';
        r19_nominal_accel_table = [];
        r19_nominal_accel_table_valid = false;
        r19_capture_nominal_accel = false;
        r19_capture_nominal_accel_count = 0;
        r19_last_prepare_total_ms = NaN;
        r19_last_batch_model_ms = NaN;
        r19_last_packet_ms = NaN;
        r19_last_handles_ms = NaN;
        r19_last_postcache_ms = NaN;
        r19_last_bundle_ms = NaN;
        r19_last_runtime_packet_reuse = false;
        r19_last_postcache_table_used = false;
    end

    methods
        function obj = REPLANNING_SYSTEM_R19_FAST(self,base_ref,opts)
            obj@REPLANNING_SYSTEM(self,base_ref,opts);
            assert(exist('af7r70_observer_mex','file')==3, ...
                '[R19] observer MEX is missing. Run AF7R58ZZY_R19_INSTALL first.');
            assert(exist('af7r71_decasteljau_batch_mex','file')==3, ...
                '[R19] de Casteljau MEX is missing. Run AF7R58ZZY_R19_INSTALL first.');
            assert(exist('af7r75_correction_posaccel_mex','file')==3, ...
                '[R19] correction evaluator MEX is missing. Run AF7R58ZZY_R19_INSTALL first.');
        end

        function build_nominal_reference_table(obj,time,varargin_in,xd_now)
            % R19 captures the exact nominal acceleration values that the
            % production table builder already evaluates at every dt-grid point.
            % No extra derivative evaluation is introduced; only the already
            % computed 3-vector is retained for later O(1) Stage2 slicing.
            if obj.nominal_table_valid || ~isempty(obj.reference_packet_fn)
                build_nominal_reference_table@REPLANNING_SYSTEM( ...
                    obj,time,varargin_in,xd_now);
                return;
            end

            te=[];
            if isstruct(time)
                if isfield(time,'te'), te=time.te; end
            elseif isobject(time)
                if isprop(time,'te'), te=time.te; end
            end
            if isempty(te) || ~isscalar(te) || ~isfinite(te)
                build_nominal_reference_table@REPLANNING_SYSTEM( ...
                    obj,time,varargin_in,xd_now);
                return;
            end

            t0=obj.resolve_takeoff_end_time();
            tend=double(te)+double(obj.horizon_s);
            if tend<double(t0)
                build_nominal_reference_table@REPLANNING_SYSTEM( ...
                    obj,time,varargin_in,xd_now);
                return;
            end
            Ntab=round((tend-double(t0))/double(obj.prediction_dt))+1;
            assert(Ntab>=2,'[R19] invalid nominal acceleration-table size.');

            obj.r19_nominal_accel_table=zeros(3,Ntab);
            obj.r19_nominal_accel_table_valid=false;
            obj.r19_capture_nominal_accel_count=0;
            obj.r19_capture_nominal_accel=true;
            try
                build_nominal_reference_table@REPLANNING_SYSTEM( ...
                    obj,time,varargin_in,xd_now);
            catch ME
                obj.r19_capture_nominal_accel=false;
                obj.r19_nominal_accel_table_valid=false;
                obj.r19_nominal_accel_table=[];
                rethrow(ME);
            end
            obj.r19_capture_nominal_accel=false;

            if obj.nominal_table_valid
                assert(obj.r19_capture_nominal_accel_count==size(obj.nominal_table_xd,2) && ...
                    size(obj.r19_nominal_accel_table,2)==size(obj.nominal_table_xd,2), ...
                    '[R19] nominal acceleration capture count mismatch.');
                obj.r19_nominal_accel_table_valid=true;
            end
        end

        function [pL,q] = nominal_payload_and_q_from_xd(obj,xd,tau)
            % Exact parent algebra plus retention of the already-evaluated aL.
            assert(numel(xd)>=3, ...
                '[REPLANNER ERROR] nominal reference packet lacks position.');
            assert(nargin>=3 && isnumeric(tau) && isscalar(tau) && isfinite(tau), ...
                '[REPLANNER ERROR] exact nominal reference time tau is required.');
            assert(isa(obj.nominal_acceleration_fn,'function_handle'), ...
                ['[REPLANNER ERROR] exact nominal acceleration function is ' ...
                 'not initialized. Refresh the nominal kinematic certificate.']);

            pL=double(xd(1:3));
            aL=double(obj.nominal_acceleration_fn(double(tau)));
            aL=aL(:);
            assert(numel(aL)==3 && all(isfinite(aL)), ...
                '[REPLANNER ERROR] exact nominal acceleration evaluation failed.');

            if obj.r19_capture_nominal_accel
                k=double(obj.r19_capture_nominal_accel_count)+1;
                assert(k>=1 && k<=size(obj.r19_nominal_accel_table,2), ...
                    '[R19] nominal acceleration capture overflow.');
                obj.r19_nominal_accel_table(:,k)=aL;
                obj.r19_capture_nominal_accel_count=k;
            end

            specific=aL+[0;0;obj.gravity];
            ns=norm(specific);
            assert(isfinite(ns)&&ns>0, ...
                '[REPLANNER ERROR] nominal specific-force vector is singular.');
            q=-specific/ns;
        end

        function R = get_r19_performance_status(obj)
            R=struct( ...
                'nominal_accel_table_valid',logical(obj.r19_nominal_accel_table_valid), ...
                'nominal_accel_table_points',size(obj.r19_nominal_accel_table,2), ...
                'prepare_total_ms',double(obj.r19_last_prepare_total_ms), ...
                'batch_model_ms',double(obj.r19_last_batch_model_ms), ...
                'packet_ms',double(obj.r19_last_packet_ms), ...
                'handles_ms',double(obj.r19_last_handles_ms), ...
                'postcache_ms',double(obj.r19_last_postcache_ms), ...
                'bundle_ms',double(obj.r19_last_bundle_ms), ...
                'runtime_packet_reuse',logical(obj.r19_last_runtime_packet_reuse), ...
                'postcache_table_used',logical(obj.r19_last_postcache_table_used));
            vals=[R.batch_model_ms R.packet_ms R.handles_ms R.postcache_ms R.bundle_ms];
            R.measured_subtotal_ms=sum(vals(isfinite(vals)));
            R.prepare_residual_ms=R.prepare_total_ms-R.measured_subtotal_ms;
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
            % R19 Stage2: preserve the exact staged theorem/commit semantics,
            % but remove two implementation-only redundancies:
            %   (1) reuse the already-proven exact runtime packet theorem when
            %       its equality gate closes; otherwise execute the unchanged
            %       rebuild path;
            %   (2) prebuild the post-commit {pL,q} cache from only the flat
            %       derivative orders actually consumed: r=0 and r=2.
            ok=false;
            reason='';
            B=struct();
            r19TotalTic=tic;
            obj.r19_last_prepare_total_ms=NaN;
            obj.r19_last_batch_model_ms=NaN;
            obj.r19_last_packet_ms=NaN;
            obj.r19_last_handles_ms=NaN;
            obj.r19_last_postcache_ms=NaN;
            obj.r19_last_bundle_ms=NaN;
            obj.r19_last_runtime_packet_reuse=false;
            obj.r19_last_postcache_table_used=false;

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

            r19x=tic;
            [resourceOK,resourceReason,batchModel,resourceMeta]= ...
                af7r58i_prepare_batch_model( ...
                    obj,P,targetVersion,double(t_commit));
            obj.r19_last_batch_model_ms=1e3*toc(r19x);
            if ~resourceOK
                reason=resourceReason; return;
            end

            % Exact runtime-packet reuse theorem already exists in production.
            % It closes only under exact equality of speed bounds and all
            % static/live dependencies.  If it does not close, R19 executes
            % the same rebuild path as production/R15.
            r19x=tic;
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
            obj.r19_last_runtime_packet_reuse=logical(reuseRuntimePacket);
            obj.r19_last_packet_ms=1e3*toc(r19x);

            r19x=tic;
            [evalFn,flatEvalFn]= ...
                af7r58i_make_canonical_eval_handles(obj,batchModel);
            obj.r19_last_handles_ms=1e3*toc(r19x);

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

            % Exact first post-commit cache.  Reference packet construction
            % consumes only pL (r=0) and acceleration (r=2) to derive q.
            % R19 therefore does not compute the unused r=1,3,4,5,6 rows.
            r19x=tic;
            dt=double(obj.prediction_dt);
            H=double(obj.horizon_s);
            N=round(H/dt)+1;
            tCache=double(t_commit)+dt;
            tqCache=tCache+(0:N-1)*dt;
            [pLcache,aLcache,validCache,tableUsed]= ...
                af7r75_posaccel_batch_R19(obj,tqCache,batchModel);
            obj.r19_last_postcache_table_used=logical(tableUsed);
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
            obj.r19_last_postcache_ms=1e3*toc(r19x);

            % These three snapshots were already proven fresh immediately
            % before Stage2 by advance_staged_replan_pipeline().  Reusing them
            % avoids fresh deep struct copies without weakening the gate.
            r19x=tic;
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
            obj.r19_last_bundle_ms=1e3*toc(r19x);
            obj.staged_replan_postcommit_cache_prepare_count= ...
                obj.staged_replan_postcommit_cache_prepare_count+1;
            obj.r19_last_prepare_total_ms=1e3*toc(r19TotalTic);
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
                    [pTail,aTail,vTail,~]=af7r75_posaccel_batch_R19( ...
                        obj,t0+(N-1)*dt,R.batch_flat_model);
                    assert(numel(vTail)==1 && logical(vTail(1)) && ...
                        isequal(size(pTail),[3 1]) && isequal(size(aTail),[3 1]), ...
                        '[REPLANNER ERROR] repaired rolling tail evaluator invalid.');
                    pL_ref(:,end)=pTail;
                    sf=aTail+[0;0;double(obj.gravity)];
                    ns=norm(sf);
                    assert(isfinite(ns)&&ns>0, ...
                        '[REPLANNER ERROR] repaired rolling tail flatness singular.');
                    q_ref(:,end)=-sf/ns;
                    source=sprintf('REPAIRED_ROLLING_SHIFT1:v%d:%s', ...
                        obj.active_reference_version,char(R.backend));
                else
                    tq=t0+(N-m:N-1)*dt;
                    [pTail,aTail,valid,~]=af7r75_posaccel_batch_R19(obj,tq,R.batch_flat_model);
                    assert(isequal(size(pTail),[3 m]) && isequal(size(aTail),[3 m]) && ...
                        numel(valid)==m && all(logical(valid(:))) && ...
                        all(isfinite(pTail),'all') && all(isfinite(aTail),'all'), ...
                        '[REPLANNER ERROR] repaired rolling tail batch invalid.');
                    pL_ref(:,end-m+1:end)=pTail;
                    sf=aTail+[0;0;double(obj.gravity)];
                    ns=sqrt(sum(sf.^2,1));
                    assert(all(isfinite(ns))&&all(ns>0), ...
                        '[REPLANNER ERROR] repaired rolling tail batch flatness singular.');
                    q_ref(:,end-m+1:end)=-sf./ns;
                    source=sprintf('REPAIRED_ROLLING_SHIFTM:v%d:%s', ...
                        obj.active_reference_version,char(R.backend));
                end
            else
                tq=t0+(0:N-1)*dt;
                [pL_ref,aRef,valid,~]=af7r75_posaccel_batch_R19(obj,tq,R.batch_flat_model);
                assert(isequal(size(pL_ref),[3 N]) && isequal(size(aRef),[3 N]) && ...
                    numel(valid)==N && all(logical(valid(:))) && ...
                    all(isfinite(pL_ref),'all') && all(isfinite(aRef),'all'), ...
                    '[REPLANNER ERROR] repaired full batch packet invalid.');
                sf=aRef+[0;0;double(obj.gravity)];
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
