function R = AF7R58R81_ADAPTIVE_DELAYED_RETURN(t0,oldPayloadJets,nominalPayloadJetsAt,knownObstacles,live,opts)
% R81: Work-bounded, PHYSICAL-DURATION-UNBOUNDED candidate search for a
% degree-15 C7 connector from the actual active reference to a delayed
% nominal spatial path, in the true HLC_PAYLOAD_WORLD frame.
%
% All dynamics/geometry inputs MUST be bound by the caller to the CURRENT
% Flight. mL is the CURRENT EKF estimate at this call, not initial loadmass.
% Finite max_attempts/max_nodes are CPU work budgets, NOT maneuver-time caps.
% Returns a reference-only candidate, NEVER authorization to commit.
% No HLC, estimator, plant, source reference or production commit is changed.
assert(isnumeric(t0)&&isscalar(t0)&&isfinite(t0), '[R81] finite current t0 required.');
assert(isnumeric(oldPayloadJets)&&isequal(size(oldPayloadJets),[3,8])&& ...
    all(isfinite(oldPayloadJets(:))), '[R81] actual ACTIVE reference jets 0..7 required.');
assert(isa(nominalPayloadJetsAt,'function_handle'), ...
    '[R81] actual runtime nominal payload-jet evaluator required.');
assert(isstruct(knownObstacles)&&~isempty(knownObstacles), ...
    '[R81] certified, sensor-known oriented ellipsoids required.');
assert(isstruct(live)&&isstruct(opts), '[R81] live and work authority required.');
needed={'gravity_m_s2','cable_length_m','rQ_m','rC_m','rL_m', ...
    'finite_sensing_vQ_limit_m_s','latest_EKF_mL_kg'};
for k=1:numel(needed)
    f=needed{k};
    assert(isfield(live,f)&&isnumeric(live.(f))&&isscalar(live.(f))&& ...
        isfinite(live.(f)), '[R81] missing non-finite runtime authority: %s',f);
end
assert(live.gravity_m_s2>0&&live.cable_length_m>0&& ...
    all([live.rQ_m,live.rC_m,live.rL_m]>=0)&& ...
    live.finite_sensing_vQ_limit_m_s>0&&live.latest_EKF_mL_kg>0, ...
    '[R81] nonphysical authority; do not invent fallback.');
req={'max_attempts','max_nodes_per_check','minimum_schedule_delay_s'};
for k=1:numel(req)
    assert(isfield(opts,req{k})&&isnumeric(opts.(req{k}))&& ...
        isscalar(opts.(req{k}))&&isfinite(opts.(req{k})), ...
        '[R81] caller work authority missing: %s',req{k});
end
assert(opts.max_attempts>=1&&opts.max_attempts==fix(opts.max_attempts)&& ...
    opts.max_nodes_per_check>=1&&opts.max_nodes_per_check==fix(opts.max_nodes_per_check)&& ...
    opts.minimum_schedule_delay_s>=0, '[R81] invalid budgets/delay.');
% This is a dynamical scale, NOT an avoidance duration upper bound.
naturalScale=sqrt(live.cable_length_m/live.gravity_m_s2);
% Do not assume the active reference is the nominal reference: the old
% 0..7 jets above MUST come from the native ACTIVE state at t0.
baseDuration=naturalScale;
assert(isfinite(baseDuration)&&baseDuration>0);
R=struct('status','INCONCLUSIVE_WORK_BUDGET_OR_NO_CERTIFIED_CANDIDATE', ...
    'reference_candidate_found',false,'candidate',struct([]), ...
    'assessment',struct([]),'attempts',0,'nominal_evaluation_failures',0, ...
    'rejected_kinematic',0,'rejected_heading',0, ...
    'rejected_geometry',0,'rejected_other',0, ...
    't0_s',t0,'t1_s',NaN,'phase_delay_s',NaN, ...
    'latest_EKF_mL_kg',live.latest_EKF_mL_kg, ...
    'natural_time_scale_s',naturalScale, ...
    'continuous_time_reference_certified',false, ...
    'whole_future_certified',false, ...
    'input_torque_tension_certified',false, ...
    'tracking_observer_tube_certified',false, ...
    'production_commit_authorized',false);
level=0;
while R.attempts<opts.max_attempts
    T=baseDuration*2^level;
    if ~(isfinite(T)&&T>0&&isfinite(t0+T)&&t0+T>t0)
        R.status='INCONCLUSIVE_NUMERIC_TIME_RANGE';return;
    end
    % Enumerate increasingly delayed nominal phases; no fixed max delay.
    % A zero delay can be *considered*, but is never forced or required.
    delayList=opts.minimum_schedule_delay_s;
    if level>=1
        for j=0:level-1
            delayList(end+1)=opts.minimum_schedule_delay_s+baseDuration*2^j; %#ok<AGROW>
        end
    end
    for i=1:numel(delayList)
        if R.attempts>=opts.max_attempts,break;end
        delta=delayList(i);
        tEnd=t0+T;
        phaseAtEnd=tEnd-delta;
        if ~isfinite(phaseAtEnd)
            R.rejected_other=R.rejected_other+1;R.attempts=R.attempts+1;continue;
        end
        R.attempts=R.attempts+1;
        try
            targetJets=nominalPayloadJetsAt(phaseAtEnd);
            if ~(isnumeric(targetJets)&&isequal(size(targetJets),[3,8])&& ...
                    all(isfinite(targetJets(:))))
                R.nominal_evaluation_failures=R.nominal_evaluation_failures+1;
                continue;
            end
            % The R79 constructor enforces exact 0..7 jets on BOTH endpoints.
            C=AF7R58R79_C7_NATURAL_CONNECTOR(t0,tEnd, ...
                oldPayloadJets,targetJets,delta,live.cable_length_m);
            K=AF7R58R79_CONTINUOUS_KINEMATIC_CERT(C, ...
                live.gravity_m_s2,live.cable_length_m, ...
                live.finite_sensing_vQ_limit_m_s);
            if ~K.certified
                R.rejected_kinematic=R.rejected_kinematic+1;continue;
            end
            Y=AF7R58R80_HEADING_C6_CERT(C,opts.max_nodes_per_check);
            if ~Y.certified
                R.rejected_heading=R.rejected_heading+1;continue;
            end
            G=AF7R58R80_CONTINUOUS_SEPARATE_BODY_CERT(C,knownObstacles, ...
                live.gravity_m_s2,live.cable_length_m, ...
                live.rQ_m,live.rC_m,live.rL_m,opts.max_nodes_per_check);
            if ~G.certified
                R.rejected_geometry=R.rejected_geometry+1;continue;
            end
            % Explicitly do not promote these reference checks into a
            % closed-loop safety proof, nominal-tail proof or commit.
            R.status='CERTIFIED_REFERENCE_CONNECTOR_ONLY__NOT_COMMITTABLE';
            R.reference_candidate_found=true;
            R.candidate=C;
            R.assessment=struct('geometry',G,'kinematics',K,'heading',Y);
            R.t1_s=tEnd;
            R.phase_delay_s=delta;
            R.continuous_time_reference_certified=true;
            return;
        catch ME
            % Invalid/missing nominal authority or inconclusive calculations
            % cannot silently become an accepted trajectory.
            R.rejected_other=R.rejected_other+1;
            R.last_error_identifier=ME.identifier;
            R.last_error_message=ME.message;
        end
    end
    level=level+1;
end
end
