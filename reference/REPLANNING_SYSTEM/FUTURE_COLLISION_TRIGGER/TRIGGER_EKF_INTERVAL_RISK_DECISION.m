function R=TRIGGER_EKF_INTERVAL_RISK_DECISION(C)
% TRIGGER_EKF_INTERVAL_RISK_DECISION  Minimal conditional REPLAN gate.
% No optimization, no trajectory construction, no backup controller.
%
% Inputs are ACTUAL OBSERVED/RETAINED obstacle IDs and an endpoint distance
% lower-bound oracle for the COMPLETE nominal UAV sphere, full straight
% cable capsule and payload sphere. End-point posterior true-state error
% bounds provide rho_i >= Hausdorff(K_true(t_i), K_nom(t_i)). A verified
% bound V_k covers every point of K_true(t) through [t_k,t_{k+1}].
%
% Then true clearance >= max(0,d_nom_lower - rho).  If
%   dtrue_lower(k)+dtrue_lower(k+1) > V_k*(t_{k+1}-t_k)
% the ENTIRE interval is collision free. On the first unproved interval,
% output exactly one obstacle ID and a possible-contact time interval.
% This is NOT an exact collision time. No unobserved obstacle is consulted.
%
% All input bounds MUST be externally justified. Merely setting numbers
% in C does not certify the original EKF/plant/HLC or true geometry.
R=struct('status','UNKNOWN_REQUIRED_CERTIFICATE_MISSING', ...
    'conditional_replan_required',true, 'obstacle_id',NaN, ...
    'first_possible_ttc_left_s',NaN, ...
    'first_possible_ttc_right_s',NaN, ...
    'certified_clear_intervals',0,'evaluated_pairs',0, ...
    'safe_to_commit',false,'reason','');
try
    mandatory={'sample_times_s','observed_ids', ...
        'nominal_clearance_lower_m','full_body_endpoint_error_upper_m', ...
        'whole_body_speed_upper_mps'};
    assert(isstruct(C)&&isscalar(C)&&all(isfield(C,mandatory)), ...
        'TRIGGER_EKF_INTERVAL_RISK_DECISION:Inputs', ...
        'Observed IDs, true-shape lower clearances, EKF error and true speed bounds required.');
    t=reshape(double(C.sample_times_s),1,[]);
    ids=reshape(double(C.observed_ids),1,[]);
    d=double(C.nominal_clearance_lower_m);
    e=reshape(double(C.full_body_endpoint_error_upper_m),1,[]);
    v=reshape(double(C.whole_body_speed_upper_mps),1,[]);
    n=numel(t);m=numel(ids);
    assert(n>=2&&isequal(t(1),0)&&all(isfinite(t))&&all(diff(t)>0) && ...
        numel(unique(ids))==m && all(isfinite(ids)) && ...
        all(ids==fix(ids)) && all(ids>0) && ...
        isequal(size(d),[m n]) && all(isfinite(d(:))) && ...
        all(d(:)>=0) && numel(e)==n && all(isfinite(e)) && ...
        all(e>=0) && numel(v)==n-1 && all(isfinite(v)) && ...
        all(v>=0), ...
        'TRIGGER_EKF_INTERVAL_RISK_DECISION:Bounds', ...
        'Only finite ordered observations and nonnegative true-state bounds are accepted.');
    if isempty(ids)
        R.status='NO_OBSERVED_OR_RETAINED_OBSTACLES__UNKNOWN_WORLD';
        R.conditional_replan_required=false;
        R.reason='No known obstacle was tested; no world-level safety claim.';
        return
    end
    % Earliest-interval-first and stop at first unresolved obstacle.
    % Never process a collision search by obstacle order alone.
    for k=1:n-1
        h=TRIGGER_FLOAT_OUTWARD_ONE_ULP(t(k+1)-t(k),+1);
        move=TRIGGER_FLOAT_OUTWARD_ONE_ULP(v(k)*h,+1);
        for j=1:m
            lo0=max(0,TRIGGER_FLOAT_OUTWARD_ONE_ULP(d(j,k)-e(k),-1));
            lo1=max(0,TRIGGER_FLOAT_OUTWARD_ONE_ULP(d(j,k+1)-e(k+1),-1));
            clear_lower=TRIGGER_FLOAT_OUTWARD_ONE_ULP(lo0+lo1,-1);
            R.evaluated_pairs=R.evaluated_pairs+1;
            if ~(clear_lower>move)
                R.status='CONDITIONAL_POSSIBLE_CONTACT_INTERVAL__REPLAN';
                R.conditional_replan_required=true;
                R.obstacle_id=ids(j);
                R.first_possible_ttc_left_s=t(k);
                R.first_possible_ttc_right_s=t(k+1);
                R.certified_clear_intervals=k-1;
                R.reason='No-contact not established here; not an exact collision proof.';
                return
            end
        end
        R.certified_clear_intervals=k;
    end
    R.status='CONDITIONAL_ALL_OBSERVED_INTERVALS_SEPARATED';
    R.conditional_replan_required=false;
    R.reason=['Conditional on actual EKF true-error enclosure, original HLC/plant speed ' ...
        'bound, true-shape clearance lower bounds and justified arithmetic.'];
catch ME
    R.status='UNKNOWN_INVALID_INTERVAL_RISK_INPUT';
    R.reason=[ME.identifier ' | ' ME.message];
    R.conditional_replan_required=true;
end
end
