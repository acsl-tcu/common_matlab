classdef TRIGGER_REFERENCE_WINDOW_CACHE < handle
    % Reuses only bitwise-identical ACTUAL time arguments to nominal.func
    % MATLAB-generated, stateless, time-only nominal reference function.
    % No interpolation, extrapolation, Taylor truncation, or guessed rates.
    % Times with different arguments after subtracting Flight epoch are
    % ALWAYS freshly evaluated. Absolute timestamps may differ while the
    % actually evaluated relative double argument is exactly equal.
    % Never touches real HLC, EKF, sensor, or the Flight reference result.
    properties (Access=private)
        source=[]
        diagnostic_timing_enabled=false % Immutable per Flight, not per tick
        epoch=NaN
        cable_length=NaN
        times=zeros(1,0)
        source_arguments=zeros(1,0) % exact double arguments to nominal.func
        jets=zeros(28,0)
        source_is_stateless=false
        audited_source=[] % Preflight-only expression audit; no Flight values.
        audited_eligible=false
        audit_status='NOT_AUDITED'
        calls=0
        exact_reuses=0
        fresh_evaluations=0
        resets=0
        live_reference_mismatches=0
        validated_now_reuses=0
        aligned_partial_reuse_count=0
        general_lookup_count=0
        direct_source_evaluations=0
        direct_source_rejected=0
        % Takeoff-only evaluation of the ACTUAL audited immutable nominal
        % time function. These are NOT sensor/prediction/Flight observations.
        % Every column is accepted only if the REAL first Flight epoch,
        % function handle, cable length and exact double arguments agree.
        phase_prime_source=[]
        phase_prime_epoch=NaN
        phase_prime_first_flight_s=NaN
        phase_prime_length=NaN
        phase_prime_dt=NaN
        phase_prime_horizon=NaN
        phase_prime_arguments=zeros(1,0)
        phase_prime_jets=zeros(28,0)
        phase_prime_next=1
        phase_prime_evaluations=0
        phase_prime_accepted=0
        phase_prime_discarded=0
        phase_prime_status='NOT_STARTED'
    end
    methods
        function obj=TRIGGER_REFERENCE_WINDOW_CACHE()
            obj.diagnostic_timing_enabled=strcmp(getenv('REPLANNING_STAGE_TIMING'),'1');
        end
        function clear_source_audit(obj)
            % Begin each Flight with no inherited source authority.
            obj.audited_source=[];
            obj.audited_eligible=false;
            obj.audit_status='NOT_AUDITED';
            obj.source=[];
            obj.source_is_stateless=false;
            obj.times=zeros(1,0);
            obj.source_arguments=zeros(1,0);
            obj.jets=zeros(28,0);
            obj.discard_phase_prime('RESET_FOR_NEW_FLIGHT');
            obj.phase_prime_evaluations=0;
            obj.phase_prime_accepted=0;
            obj.phase_prime_discarded=0;
        end
        function discard_phase_prime(obj,status)
            obj.phase_prime_source=[];
            obj.phase_prime_epoch=NaN;
            obj.phase_prime_first_flight_s=NaN;
            obj.phase_prime_length=NaN;
            obj.phase_prime_dt=NaN;
            obj.phase_prime_horizon=NaN;
            obj.phase_prime_arguments=zeros(1,0);
            obj.phase_prime_jets=zeros(28,0);
            obj.phase_prime_next=1;
            obj.phase_prime_status=char(status);
        end
        function prime_takeoff_step(obj,nominal,candidate_first_flight_s,source_epoch,dt,horizon,L)
            % Real Takeoff only. One actual, audited time-only reference
            % evaluation per update; never a hidden 160-point workload.
            % No model, EKF, sensor, obstacle, HLC or controller call.
            if ~obj.audited_eligible || ...
                    ~isa(nominal,'TIME_VARYING_REFERENCE') || ...
                    ~isequal(nominal.func,obj.audited_source)
                obj.phase_prime_status='DISABLED_SOURCE_NOT_AUDITED';
                return
            end
            assert(isnumeric(candidate_first_flight_s) && ...
                isscalar(candidate_first_flight_s) && ...
                isfinite(candidate_first_flight_s) && ...
                isnumeric(source_epoch) && isscalar(source_epoch) && ...
                isfinite(source_epoch) && ...
                isnumeric(dt) && isscalar(dt) && ...
                isfinite(dt) && dt>0 && isnumeric(horizon) && ...
                isscalar(horizon) && isfinite(horizon) && horizon>0 && ...
                isnumeric(L) && isscalar(L) && isfinite(L) && L>0, ...
                'TRIGGER_REFERENCE_WINDOW_CACHE:PrimeInputs', ...
                'Actual nominal and positive static parameters are required.');
            changed=isempty(obj.phase_prime_source) || ...
                ~isequal(obj.phase_prime_source,nominal.func) || ...
                ~isequal(obj.phase_prime_epoch,double(source_epoch)) || ...
                ~isequal(obj.phase_prime_first_flight_s,double(candidate_first_flight_s)) || ...
                ~isequal(obj.phase_prime_length,double(L)) || ...
                ~isequal(obj.phase_prime_dt,double(dt)) || ...
                ~isequal(obj.phase_prime_horizon,double(horizon));
            if changed
                % SAME arithmetic/order as FUTURE_COLLISION_TRIGGER.make_grid.
                g=(0:floor(horizon/dt))*dt;
                g=g(g<horizon);g=[g,horizon];
                assert(numel(g)>=2 && all(diff(g)>0) && ...
                    isequal(g(end),horizon), ...
                    'TRIGGER_REFERENCE_WINDOW_CACHE:PrimeGrid', ...
                    'Invalid actual dt/horizon.');
                obj.phase_prime_source=nominal.func;
                obj.phase_prime_epoch=double(source_epoch);
                obj.phase_prime_first_flight_s=double(candidate_first_flight_s);
                obj.phase_prime_length=double(L);
                obj.phase_prime_dt=double(dt);
                obj.phase_prime_horizon=double(horizon);
                absolute_times=double(candidate_first_flight_s)+double(g(1:end-1));
                obj.phase_prime_arguments=double(absolute_times)-double(source_epoch);
                obj.phase_prime_jets=zeros(28,numel(obj.phase_prime_arguments));
                obj.phase_prime_next=1;
            end
            k=obj.phase_prime_next;
            if k>numel(obj.phase_prime_arguments)
                obj.phase_prime_status='READY_WAITING_FOR_REAL_FLIGHT_VERIFICATION';
                return
            end
            % Original function is known to be stateless. Evaluate EXACT
            % hypothetical first-Flight double arguments and keep results
            % private until separately verified at the actual Flight tick.
            v=nominal.func(obj.phase_prime_arguments(k));
            assert(isnumeric(v)&&isreal(v)&&iscolumn(v) && ...
                numel(v)>=4&&numel(v)<=28&&all(isfinite(v)), ...
                'TRIGGER_REFERENCE_WINDOW_CACHE:PrimeReference', ...
                'Original time-only nominal function returned invalid jets.');
            v=double(v);v(3)=v(3)-double(L);
            assert(all(isfinite(v)), ...
                'TRIGGER_REFERENCE_WINDOW_CACHE:PrimeReference', ...
                'Original cable adjustment produced nonfinite jets.');
            obj.phase_prime_jets(1:numel(v),k)=v;
            obj.phase_prime_next=k+1;
            obj.phase_prime_evaluations=obj.phase_prime_evaluations+1;
            obj.phase_prime_status='TAKEOFF_ONE_COLUMN_PER_TICK';
        end
        function R=audit_immutable_source(obj,fn)
            % PRE-ARM SOFTWARE METADATA ONLY. No reference evaluation and
            % no obstacle/EKF/Flight sample access. Runtime source changes
            % fail closed to the original no-cache evaluation path.
            R=TRIGGER_TIME_ONLY_PURITY(fn);
            obj.audited_source=fn;
            obj.audited_eligible=R.eligible;
            obj.audit_status=R.status;
        end
        function [ref,report]=evaluate(obj,nominal,absolute_times,L,reference_at,validated_now,source_contract)
            % Optional per-tick diagnostic. No cache/evaluation decisions change.
            diagnose=obj.diagnostic_timing_enabled;
            if diagnose,diagnostic_clock=tic;end
            assert(isobject(nominal)&&isscalar(nominal)&& ...
                isprop(nominal,'func')&&isprop(nominal,'t')&& ...
                isa(nominal.func,'function_handle'), ...
                'TRIGGER_REFERENCE_WINDOW_CACHE:Source', ...
                'An active original nominal reference is required.');
            assert(isnumeric(absolute_times)&&isreal(absolute_times)&& ...
                isrow(absolute_times)&&~isempty(absolute_times)&& ...
                all(isfinite(absolute_times))&&all(diff(absolute_times)>0), ...
                'TRIGGER_REFERENCE_WINDOW_CACHE:Times', ...
                'Expected strictly increasing actual absolute sample times.');
            assert(isnumeric(L)&&isscalar(L)&&isfinite(L)&&L>0 && ...
                isa(reference_at,'function_handle') && ...
                isnumeric(nominal.t)&&isscalar(nominal.t)&&isfinite(nominal.t), ...
                'TRIGGER_REFERENCE_WINDOW_CACHE:Binding', ...
                'Current flight epoch, physical cable length, reference evaluator required.');
            % Optional sixth input is the REAL, independently verified
            % current HLC reference jet. It is NOT a prediction. The caller
            % must prove the first absolute query time matches this tick.
            has_verified_now=nargin>=6 && ~isempty(validated_now);
            if has_verified_now
                verified_first=TRIGGER_REFERENCE_WINDOW_CACHE.full_jet(validated_now);
            end
            fn=nominal.func;epoch_now=double(nominal.t);
            unchanged=~isempty(obj.source)&&isequal(obj.source,fn)&& ...
                isequal(obj.epoch,epoch_now)&&isequal(obj.cable_length,double(L));
            if ~unchanged
                obj.resets=obj.resets+1;
                obj.source=fn;obj.epoch=epoch_now;
                obj.cable_length=double(L);
                % This check is O(1) with respect to expression length.
                % Pure-source syntax was audited BEFORE arming. Never
                % parse or evaluate source metadata in a Flight tick.
                % A replaced/changed reference handle uses uncached values.
                obj.source_is_stateless= ...
                    isa(nominal,'TIME_VARYING_REFERENCE') && ...
                    obj.audited_eligible && ~isempty(obj.audited_source) && ...
                    isequal(fn,obj.audited_source);
                obj.times=zeros(1,0);obj.source_arguments=zeros(1,0);
                obj.jets=zeros(28,0);
            end
            % flight_reference_at calls nominal.func(abs_time-epoch).
            % Compare EXACTLY those evaluated double arguments, not the
            % absolute clock, to avoid unnecessary ULP-induced misses.
            % This changes no source-evaluation arguments and performs no
            % interpolation, rounding or prediction-time approximation.
            source_arguments=double(absolute_times)-epoch_now;
            n=numel(absolute_times);
            % This contract is written ONLY by the real REPLANNING_SYSTEM
            % path after comparing the same-tick live reference with the
            % original TIME_VARYING_REFERENCE.func. No preflight samples,
            % no time rounding and no hidden fallback data are introduced.
            direct_exact=nargin>=7 && ischar(source_contract) && ...
                strcmp(source_contract,'VERIFIED_F_PHASE_NOMINAL_Z_MINUS_CABLEL') && ...
                has_verified_now && obj.source_is_stateless && ...
                isa(nominal,'TIME_VARYING_REFERENCE') && ...
                isequal(obj.source,nominal.func) && ...
                isequal(obj.epoch,double(nominal.t));
            if nargin>=7 && ~direct_exact
                obj.direct_source_rejected=obj.direct_source_rejected+1;
            end
            ref=zeros(28,n);
            hits=false(1,n);
            locations=zeros(1,n);
            % An OBSERVED takeoff phase may already have evaluated a
            % complete or partial first f-reference window. This is valid
            % ONLY if the REAL Flight epoch, source handle, cable length,
            % and EVERY reused double source argument are exactly equal.
            % The first real reference is independently verified at t_now.
            if ~unchanged && direct_exact && ...
                    ~isempty(obj.phase_prime_source)
                prime_match=isequal(obj.phase_prime_source,fn) && ...
                    isequal(obj.phase_prime_first_flight_s,absolute_times(1)) && ...
                    isequal(obj.phase_prime_epoch,epoch_now) && ...
                    isequal(obj.phase_prime_length,double(L)) && ...
                    numel(obj.phase_prime_arguments)==n;
                if prime_match
                    count=min(n,obj.phase_prime_next-1);
                    equal=find(typecast(source_arguments(1:count),'uint64')== ...
                        typecast(obj.phase_prime_arguments(1:count),'uint64'));
                    if ~isempty(equal)
                        hits(equal)=true;
                        locations(equal)=equal;
                        ref(:,equal)=obj.phase_prime_jets(:,equal);
                        if hits(1) && ~isequal(ref(:,1),verified_first)
                            hits(:)=false;
                            locations(:)=0;
                            ref(:)=0;
                            obj.live_reference_mismatches= ...
                                obj.live_reference_mismatches+1;
                            obj.phase_prime_status='DISCARDED_LIVE_REFERENCE_MISMATCH';
                        else
                            obj.phase_prime_accepted=obj.phase_prime_accepted+nnz(hits);
                            obj.phase_prime_status='ACCEPTED_EXACT_FIRST_FLIGHT_COLUMNS';
                        end
                    end
                else
                    obj.phase_prime_discarded=obj.phase_prime_discarded+1;
                    obj.phase_prime_status='DISCARDED_REAL_EPOCH_OR_SOURCE_DIFFERED';
                end
                % Never attempt to reuse phase-prime state after the first
                % real evaluation; the normal exact sliding cache owns it.
                obj.phase_prime_source=[];
                obj.phase_prime_arguments=zeros(1,0);
                obj.phase_prime_jets=zeros(28,0);
            elseif ~unchanged && ~isempty(obj.phase_prime_source)
                obj.phase_prime_discarded=obj.phase_prime_discarded+1;
                obj.discard_phase_prime('DISCARDED_NO_REAL_REFERENCE_CONTRACT');
            end
            if unchanged && obj.source_is_stateless && ~isempty(obj.source_arguments)
                % Exact numeric equality only; no time-rounding assumptions.
                % Exact sliding-window reuse is the normal online case.
                % First try O(N) elementwise equality and a direct column
                % shift instead of allocating a general set-membership map.
                % This applies ONLY to bitwise-equal double sample times;
                % no time rounding or approximation is introduced.
                shifted=false;
                if numel(obj.source_arguments)==n && n>1
                    % Reuse EACH exactly-equal shifted column, even when
                    % other columns differ by one ULP. Never round or
                    % identify two distinct absolute time values. A miss
                    % is evaluated anew from the real reference function.
                    equal=(source_arguments(1:n-1)==obj.source_arguments(2:n));
                    if any(equal)
                        hits(1:n-1)=equal;
                        loc=2:n;
                        locations(1:n-1)=loc.*double(equal);
                        obj.aligned_partial_reuse_count= ...
                            obj.aligned_partial_reuse_count+nnz(equal);
                        shifted=true;
                    end
                end
                if ~shifted
                    % This is necessary for non-unit shifts, arbitrary
                    % externally supplied horizons and unusual time grids.
                    % Monotone exact-time lookup: deterministic two-pointer
                    % linear scan, with the ORIGINAL ismember fallback if
                    % double subtraction loses the ordered finite contract.
                    % Equalities are exact; no rounding, missing time or
                    % nominal-trajectory approximation is introduced.
                    if all(isfinite(source_arguments)) && ...
                            all(isfinite(obj.source_arguments)) && ...
                            all(diff(source_arguments)>=0) && ...
                            all(diff(obj.source_arguments)>=0)
                        [hits,locations]=TRIGGER_EXACT_SORTED_TIME_LOOKUP( ...
                            source_arguments,obj.source_arguments);
                    else
                        [hits,locations]=ismember( ...
                            source_arguments,obj.source_arguments);
                    end
                    obj.general_lookup_count=obj.general_lookup_count+1;
                end
                if any(hits)
                    ref(:,hits)=obj.jets(:,locations(hits));
                    % Current actual reference is evaluated independently.
                    % If stale despite unchanged handle, discard all reuse.
                    if hits(1)
                        if has_verified_now
                            now=verified_first;
                        else
                            now=TRIGGER_REFERENCE_WINDOW_CACHE.full_jet( ...
                                reference_at(absolute_times(1)));
                        end
                        if ~isequal(now,ref(:,1))
                            hits(:)=false;
                            obj.live_reference_mismatches= ...
                                obj.live_reference_mismatches+1;
                        end
                    end
                end
            end
            fresh_indices=find(~hits);
            new_direct_calls=0;
            % Source dispatch is isolated from lookup/shift/cache work.
            if diagnose,source_loop_clock=tic;end
            for k=fresh_indices
                if k==1 && has_verified_now
                    ref(:,1)=verified_first;
                elseif direct_exact
                    % Preserve ORIGINAL actual double time arguments and
                    % cable subtraction order; never round 1-ULP misses.
                    v=fn(source_arguments(k));
                    assert(isnumeric(v)&&isreal(v)&&iscolumn(v) && ...
                        numel(v)>=4&&numel(v)<=28&&all(isfinite(v)), ...
                        'TRIGGER_REFERENCE_WINDOW_CACHE:DirectSource', ...
                        'The original nominal function returned an invalid jet.');
                    v=double(v);
                    v(3)=v(3)-double(L);
                    assert(all(isfinite(v)), ...
                        'TRIGGER_REFERENCE_WINDOW_CACHE:Reference', ...
                        'The live original reference must supply finite jets.');
                    % ref is already zero-initialized: padding identical to
                    % full_jet(v), without a 28x1 allocation and dispatch.
                    ref(1:numel(v),k)=v;
                    new_direct_calls=new_direct_calls+1;
                else
                    ref(:,k)=TRIGGER_REFERENCE_WINDOW_CACHE.full_jet( ...
                        reference_at(absolute_times(k)));
                end
            end
            if diagnose
                source_loop_ms=1000*toc(source_loop_clock);
            else
                source_loop_ms=NaN;
            end
            obj.direct_source_evaluations= ...
                obj.direct_source_evaluations+new_direct_calls;
            if has_verified_now
                obj.validated_now_reuses=obj.validated_now_reuses+1;
            end
            % No post-use mutation of active reference or any Flight state.
            obj.calls=obj.calls+1;
            obj.exact_reuses=obj.exact_reuses+nnz(hits);
            % Count only calls made by this cache to the original source.
            % If a previously checked current jet was supplied, no new
            % reference computation occurs for the first time sample.
            obj.fresh_evaluations=obj.fresh_evaluations+nnz(~hits) ...
                -double(has_verified_now && ~hits(1)) ...
                +double(~has_verified_now && hits(1));
            if obj.source_is_stateless
                obj.times=absolute_times;
                obj.source_arguments=source_arguments;
                obj.jets=ref;
            else
                obj.times=zeros(1,0);obj.source_arguments=zeros(1,0);
                obj.jets=zeros(28,0);
            end
            report=struct('exact_reused',nnz(hits), ...
                'fresh_generated',nnz(~hits)-double(has_verified_now && ~hits(1)), ...
                'validated_now_used',has_verified_now, ...
                'source_cached',obj.source_is_stateless);
            % Reason codes describe actual cache decisions, not their cause.
            % No claim that a long timer indicates a JIT/OS pause.
            report.diagnostic_active=diagnose;
            report.cache_reset=~unchanged;
            report.source_argument_count=n;
            report.exact_hits=nnz(hits);
            report.source_function_calls=nnz(~hits)- ...
                double(has_verified_now && ~hits(1))+ ...
                double(~has_verified_now && hits(1));
            report.source_loop_ms=source_loop_ms;
            report.direct_exact=direct_exact;
            report.full_reset_fresh=(~unchanged && nnz(~hits)==n);
            if diagnose
                report.cache_evaluate_wall_ms=1000*toc(diagnostic_clock);
            else
                report.cache_evaluate_wall_ms=NaN;
            end
        end
        function R=get_status(obj)
            R=struct('calls',obj.calls,'exact_reuses',obj.exact_reuses, ...
                'validated_now_reuses',obj.validated_now_reuses, ...
                'fresh_evaluations',obj.fresh_evaluations,'resets',obj.resets, ...
                'reference_mismatch_resets',obj.live_reference_mismatches, ...
                'active_source_cacheable',obj.source_is_stateless, ...
                'preflight_source_audit_status',obj.audit_status, ...
                'preflight_source_eligible',obj.audited_eligible, ...
                'aligned_partial_reuse_count',obj.aligned_partial_reuse_count, ...
                'general_lookup_count',obj.general_lookup_count, ...
                'direct_source_evaluations',obj.direct_source_evaluations, ...
                'direct_source_rejected',obj.direct_source_rejected, ...
                'takeoff_prime_evaluations',obj.phase_prime_evaluations, ...
                'takeoff_prime_accepted',obj.phase_prime_accepted, ...
                'takeoff_prime_discarded',obj.phase_prime_discarded, ...
                'takeoff_prime_status',obj.phase_prime_status, ...
                'takeoff_prime_first_flight_s',obj.phase_prime_first_flight_s, ...
                'cache_policy','EXACT_NOMINAL_FUNC_ARGUMENT_AFTER_FLIGHT_EPOCH__NO_APPROXIMATION');
        end
    end
    methods (Static, Access=private)
        function full=full_jet(v)
            assert(isnumeric(v)&&isreal(v)&&iscolumn(v)&& ...
                numel(v)>=4&&numel(v)<=28&&all(isfinite(v)), ...
                'TRIGGER_REFERENCE_WINDOW_CACHE:Reference', ...
                'The live original reference must supply 4..28 finite column jets.');
            full=zeros(28,1);
            full(1:numel(v))=double(v);
        end
    end
end
