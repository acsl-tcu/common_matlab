function R=TRIGGER_ORIGINAL_HELD_SUBSTEP_CHAIN(D)
% Verified-tube SEARCH for ONE actually held original controller input.
% Subdividing a hold does NOT alter the original plant or its applied input.
% The input/parameter intervals must contain the real values THROUGHOUT
% the complete hold. Every candidate slab requires strict Picard inclusion,
% taut-manifold compatibility, and a strictly positive tension lower bound.
% A success contains the mathematical original-ODE solution on ALL slabs,
% conditional on the source, rounding and physical hybrid-model premises.
% Failure is UNKNOWN, never a safe clearance or a new initial condition.
% The dyadic refinement limit is ONLY a finite computation budget.
R=struct('status','UNKNOWN_HELD_SUBDIVISION_NOT_RUN', ...
    'reason','','conditional_strict_inclusion',false, ...
    'tube_lower',nan(24,1),'tube_upper',nan(24,1), ...
    'endpoint_lower',nan(24,1),'endpoint_upper',nan(24,1), ...
    'substeps',0,'refinements_tried',0,'slabs_completed_in_last_trial',0, ...
    'total_picard_iterations',0,'minimum_tension_lower_N',NaN, ...
    'physical_hold_duration_s',NaN, ...
    'duration_source','UNVERIFIED_SCHEDULE_CLOCK', ...
    'taut_manifold',struct(),'tension',struct(), ...
    'first_failure_stage','','first_failure_reason','', ...
    'last_failure_stage','','last_failure_reason','', ...
    'true_plant_and_rounding_independently_verified',false, ...
    'safe_to_commit',false);
try
    need={'t_left_s','t_right_s','x0_lower','x0_upper', ...
          'u_lower','u_upper','param_lower','param_upper'};
    assert(isstruct(D)&&isscalar(D)&&all(isfield(D,need)), ...
        'TRIGGER_ORIGINAL_HELD_SUBSTEP_CHAIN:Fields', ...
        'Full original held-input time and state/parameter boxes required.');
    t0=double(D.t_left_s);t1=double(D.t_right_s);
    assert(isa(D.t_left_s,'double')&&isa(D.t_right_s,'double') && ...
        isscalar(t0)&&isscalar(t1)&&isfinite(t0)&&isfinite(t1)&&t1>t0, ...
        'TRIGGER_ORIGINAL_HELD_SUBSTEP_CHAIN:Clock');
    xseed=double(D.x0_lower(:)); xend=double(D.x0_upper(:));
    ul=double(D.u_lower(:)); uu=double(D.u_upper(:));
    pl=double(D.param_lower(:)); pu=double(D.param_upper(:));
    assert(isequal(size(xseed),[24 1]) && isequal(size(xend),[24 1]) && ...
        isequal(size(ul),[4 1]) && isequal(size(uu),[4 1]) && ...
        isequal(size(pl),[21 1]) && isequal(size(pu),[21 1]) && ...
        all(isfinite([xseed;xend;ul;uu;pl;pu])) && ...
        all(xseed<=xend)&&all(ul<=uu)&&all(pl<=pu) && ...
        pl(20)>0 && pl(21)>0, ...
        'TRIGGER_ORIGINAL_HELD_SUBSTEP_CHAIN:Boxes', ...
        'Finite correctly ordered original-plant boxes required.');
    U=struct('lower',ul,'upper',uu);
    P=struct('lower',pl,'upper',pu);
    % MODEL_CLASS.do advances the actual Sim plant with solver times
    % [obj.ts, obj.ts+obj.dt]. The runner clock is accumulated separately
    % by time.t=time.t+dt, so consecutive clock differences are NOT
    % guaranteed to equal the physical integration duration.
    % The source-checked Sim caller supplies verified_hold_duration_s;
    % generic callers continue using their stated interval endpoints.
    if isfield(D,'verified_hold_duration_s')
        hs=double(D.verified_hold_duration_s);
        assert(isa(D.verified_hold_duration_s,'double') && ...
            isscalar(hs) && isreal(hs) && isfinite(hs) && hs>0, ...
            'TRIGGER_ORIGINAL_HELD_SUBSTEP_CHAIN:PhysicalDuration', ...
            'Source-verified original MODEL_CLASS solver dt required.');
        tau0=0;tau1=hs;
        R.duration_source='ORIGINAL_MODEL_CLASS_SOLVER_HOLD_DT';
    else
        tau0=t0;tau1=t1;
        R.duration_source='GENERIC_BINARY64_INTERVAL_ENDPOINTS';
    end
    R.physical_hold_duration_s=tau1-tau0;
    for lev=0:6
        n=2^lev;
        R.refinements_tried=lev+1;
        % Explicit mathematical partition of the SAME original hold.
        % No segment borrows true state from the numerical plant/estimator.
        times=tau0+(tau1-tau0)*(0:n)/n;
        times(1)=tau0;times(end)=tau1;
        if ~all(isfinite(times)) || ~all(diff(times)>0)
            R.last_failure_stage='SUBSTEP_TIME_GRID';
            R.last_failure_reason='Double time grid cannot form ordered slabs.';
            break
        end
        lo=xseed;hi=xend;
        hull_lo=inf(24,1);hull_hi=-inf(24,1);
        minT=inf;maxT=-inf;
        R.slabs_completed_in_last_trial=0;
        attempt_ok=true;
        for k=1:n
            C=TRIGGER_EXACT_BINARY64_TIME_SPAN(times(k),times(k+1));
            if ~C.valid
                stage='BINARY64_SLAB_TIME'; reason=C.reason;
                attempt_ok=false;break
            end
            step=TRIGGER_ORIGINAL_PLANT_AUTO_PICARD(struct( ...
                'x0_lower',lo,'x0_upper',hi,'u_lower',ul,'u_upper',uu, ...
                'param_lower',pl,'param_upper',pu, ...
                'h_s',C.upper_s,'h_lower_s',C.lower_s));
            R.total_picard_iterations=R.total_picard_iterations+step.iterations;
            if ~step.conditional_strict_inclusion
                stage='STRICT_PICARD';
                reason=[step.status ' | ' step.reason];
                attempt_ok=false;break
            end
            X=struct('lower',step.tube_lower,'upper',step.tube_upper);
            M=TRIGGER_TAUT_CABLE_MANIFOLD_OBLIGATIONS(X,P);
            if ~M.compatible
                stage='TAUT_MANIFOLD';reason=[M.status ' | ' M.reason];
                attempt_ok=false;break
            end
            % The initial box belongs to this exact proof subslab.
            % Its dot-product and the all-time original angular-speed
            % enclosure rigorously constrain orientation drift. This
            % preserves the original physical tension >0 requirement.
            T=TRIGGER_ORIGINAL_TENSION_INTERVAL(X,U,P,struct( ...
                'x0_lower',lo,'x0_upper',hi, ...
                'h_upper_s',C.upper_s));
            if ~isfinite(T.tension_lower_N) || T.tension_lower_N<=0
                stage='POSITIVE_TENSION';
                reason=sprintf('%s | lower_N=%.17g | %s', ...
                    T.status,T.tension_lower_N,T.reason);
                attempt_ok=false;break
            end
            hull_lo=min(hull_lo,step.tube_lower);
            hull_hi=max(hull_hi,step.tube_upper);
            lo=step.endpoint_lower;hi=step.endpoint_upper;
            minT=min(minT,T.tension_lower_N);
            maxT=max(maxT,T.tension_upper_N);
            R.slabs_completed_in_last_trial=k;
        end
        if attempt_ok
            R.conditional_strict_inclusion=true;
            R.tube_lower=hull_lo;R.tube_upper=hull_hi;
            R.endpoint_lower=lo;R.endpoint_upper=hi;
            R.substeps=n;
            R.minimum_tension_lower_N=minT;
            R.taut_manifold=struct('compatible',true, ...
                'status','ALL_SUBSLABS_TAUT_COMPATIBLE__NOT_PHYSICAL_MODE_PROOF');
            R.tension=struct('tension_lower_N',minT, ...
                'tension_upper_N',maxT, ...
                'conditional_tension_positive',true, ...
                'status','ALL_SUBSLABS_POSITIVE__MODEL_CONDITIONAL');
            R.status='CONDITIONAL_ORIGINAL_HELD_SUBSLABS_COMPLETE';
            R.reason=sprintf(['All %d slabs cover the original held-input ' ...
                'time, with positive tube tension; real physics and ' ...
                'solver error are NOT certified.'],n);
            return
        end
        R.last_failure_stage=stage;
        R.last_failure_reason=sprintf('refine=%d slab=%d/%d | %s',lev,k,n,reason);
        if isempty(R.first_failure_stage)
            R.first_failure_stage=stage;
            R.first_failure_reason=R.last_failure_reason;
        end
    end
    R.status='UNKNOWN_ALL_HELD_REFINEMENTS_FAILED';
    R.reason=sprintf('First failure: %s | Last failure: %s', ...
        R.first_failure_reason,R.last_failure_reason);
catch ME
    R.status='UNKNOWN_HELD_SUBDIVISION_ERROR';
    R.reason=[ME.identifier ' | ' ME.message];
    if isempty(R.first_failure_stage)
        R.first_failure_stage='EXCEPTION';
        R.first_failure_reason=R.reason;
    end
end
end
