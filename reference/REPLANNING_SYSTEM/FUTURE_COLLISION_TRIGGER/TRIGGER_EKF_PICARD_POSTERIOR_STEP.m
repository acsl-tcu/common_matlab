function R=TRIGGER_EKF_PICARD_POSTERIOR_STEP(tracker,t_next,ekf_post,B,source)
% TRIGGER_EKF_PICARD_POSTERIOR_STEP  Link the ACTUAL EKF posterior with the
% existing 24-state interval Picard strict-inclusion obligation.
%
% B: h_s, x0_lower/upper, tube_lower/upper, rhs_lower/upper for the original
% 24-state TRUE plant on the complete control hold interval. B.rhs_* must
% enclose the REAL original RHS for ALL admissible held HLC inputs, true
% masses and disturbances. Nothing is inferred from EKF covariance.
%
% If X0+[0,h]*F strictly lies in Z and the externally supplied F enclosure
% is SOUND, true X(t) lies in Z and the endpoint enclosure follows from
% X0 + h*[Fmin,Fmax]. The actual EKF state after nonlinear update and
% projection is then subtracted. No affine EKF error approximation is used.
%
% This proves only a CONDITIONAL implication. It does not itself prove the
% source RHS enclosure, true initial inclusion, taut positive tension,
% actuator/timestamp wiring or interval rounding of the full dynamics.
R=struct('status','UNKNOWN_PICARD_TO_EKF','reason','', ...
    'strict_inclusion',false,'true_next_box',zeros(24,0), ...
    'ekf_error',struct(),'safe_to_commit',false);
try
    assert(isa(tracker,'TRIGGER_EKF_TRUE_SET_TRACKER'), ...
        'TRIGGER_EKF_PICARD_POSTERIOR_STEP:Tracker', ...
        'Original live posterior tracker required.');
    S=tracker.snapshot();
    assert(S.initialized && isstruct(B)&&isscalar(B), ...
        'TRIGGER_EKF_PICARD_POSTERIOR_STEP:Initialization', ...
        'Tracker must contain the previous true-state enclosure.');
    keys={'h_s','x0_lower','x0_upper','tube_lower','tube_upper', ...
          'rhs_lower','rhs_upper'};
    assert(all(isfield(B,keys)), ...
        'TRIGGER_EKF_PICARD_POSTERIOR_STEP:Box', ...
        'Missing true-model interval Picard obligation fields.');
    t0=double(S.time_s);t1=double(t_next);h=double(B.h_s);
    assert(isfinite(t1)&&t1>t0 && isfinite(h)&&h>0 && ...
        isequal(h,t1-t0), ...
        'TRIGGER_EKF_PICARD_POSTERIOR_STEP:Time', ...
        'Current actual hold step and Picard interval must agree exactly.');
    lo=double(B.x0_lower(:));hi=double(B.x0_upper(:));
    zl=double(B.tube_lower(:));zu=double(B.tube_upper(:));
    fl=double(B.rhs_lower(:));fu=double(B.rhs_upper(:));
    assert(isequal(size([lo hi zl zu fl fu]),[24 6]) && ...
        all(isfinite([lo;hi;zl;zu;fl;fu])) && ...
        all(lo<=hi)&&all(zl<zu)&&all(fl<=fu) && ...
        isequal(lo,S.true_lower)&&isequal(hi,S.true_upper), ...
        'TRIGGER_EKF_PICARD_POSTERIOR_STEP:Input', ...
        'Picard initial set must be exactly the tracker current true box.');
    % Enclose the full-time Picard map X0 + [0,h]*F. Positive interval h.
    low_delta=TRIGGER_FLOAT_OUTWARD_ONE_ULP(h.*min(fl,0),-1);
    high_delta=TRIGGER_FLOAT_OUTWARD_ONE_ULP(h.*max(fu,0),+1);
    candidate_lo=TRIGGER_FLOAT_OUTWARD_ONE_ULP(lo+low_delta,-1);
    candidate_hi=TRIGGER_FLOAT_OUTWARD_ONE_ULP(hi+high_delta,+1);
    if ~all(candidate_lo>zl) || ~all(candidate_hi<zu)
        R.status='UNKNOWN_PICARD_STRICT_INCLUSION_FAILED';
        R.reason='A complete true-state interval Picard enclosure was not obtained.';
        return
    end
    R.strict_inclusion=true;
    % Endpoint integral: for t1=t0+h use [h*fl,h*fu], not [0,h]*F.
    d_lo=TRIGGER_FLOAT_OUTWARD_ONE_ULP(h.*fl,-1);
    d_hi=TRIGGER_FLOAT_OUTWARD_ONE_ULP(h.*fu,+1);
    next_lo=TRIGGER_FLOAT_OUTWARD_ONE_ULP(lo+d_lo,-1);
    next_hi=TRIGGER_FLOAT_OUTWARD_ONE_ULP(hi+d_hi,+1);
    next_box=[next_lo next_hi];
    E=tracker.advance(t0,t1,ekf_post,next_box,source);
    R.true_next_box=next_box;
    R.ekf_error=E;
    R.status='CONDITIONAL_PICARD_TRUE_STATE_TO_LIVE_EKF_POSTERIOR';
    R.reason=['Only conditional: source RHS inclusion and real initial set ' ...
        'must be proven; no numeric 4s closed-loop safety certificate.'];
catch ME
    R.status='UNKNOWN_PICARD_TO_EKF';
    R.reason=[ME.identifier ' | ' ME.message];
end
end
