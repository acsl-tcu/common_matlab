function R=TRIGGER_TAUT_TEMPORAL_CLEARANCE_WITNESS(C)
% CONDITIONAL, shape-independent ALL-TIME material-body clearance theorem.
%
% Given a STATIC occupied obstacle O and the moving occupied UAV/cable/load
% body B(t), suppose the ENTIRE body's Hausdorff speed is rigorously bounded
% by V_k on [t_k,t_{k+1}]. The signed material clearance
%       c(t) = dist(B(t),O)
% is V_k-Lipschitz for any closed O (convexity NOT required).
% If independently VERIFIED lower bounds at interval endpoints are
%       d_k <= c(t_k), d_{k+1} <= c(t_{k+1}),
% then, for ALL tau in [0,h],
%   c(t_k+tau) >= max(d_k-V_k*tau, d_{k+1}-V_k*(h-tau))
%                >= (d_k+d_{k+1}-V_k*h)/2.
% Therefore a STRICTLY POSITIVE bound proves temporal separation.
%
% The matrix `sample_clearance_lower_m` MUST already include all material
% radii (UAV sphere, cable cross-section and payload sphere), true-state
% error at EACH endpoint, and a proved lower-bound distance to each genuine
% sensor-observed/retained STATIC obstacle. A nominal contact distance,
% EKF covariance, sampled-only path or an uncertified numerical oracle is
% not such a bound. `speed_upper_mps` MUST bound the true complete physical
% body on EVERY interval, not the nominal reference speed.
%
% No observation is performed here. The FINAL arithmetic of the temporal
% inequality is explicitly outward-widened one ULP PER operation, assuming
% an audited IEEE-754 binary64 round-to-nearest execution environment.
% This does NOT certify the distance oracle, ODE/Picard arithmetic, physical
% state inclusion or future applied HLC. This module NEVER enables Flight
% commitment and can only produce a CONDITIONAL mathematical witness.
R=struct('status','UNKNOWN_TEMPORAL_BODY_CLEARANCE', ...
    'conditional_clearance',false,'all_intervals_separated',false, ...
    'safe_to_commit',false,'real_flight_proved',false, ...
    'separated_intervals',false(0,0), ...
    'clearance_lower_m',zeros(0,0), ...
    'interval_duration_upper_s',zeros(1,0), ...
    'rounding_policy','IEEE754_BINARY64_OUTWARD_ONE_ULP_PER_OPERATION', ...
    'possible_risk_ids',zeros(1,0), ...
    'earliest_possible_event',struct(), ...
    'interval_count',0,'observed_count',0,'reason','');
try
    needed={'time_grid_s','observed_ids', ...
        'sample_clearance_lower_m','speed_upper_mps'};
    assert(isstruct(C)&&isscalar(C)&&all(isfield(C,needed)), ...
        'TRIGGER_TIME_CLEARANCE:Fields', ...
        'Explicit observed IDs, end-time clearance lower bounds and true speed required.');
    t=double(C.time_grid_s(:).');
    ids=double(C.observed_ids(:).');
    K=numel(t)-1;M=numel(ids);
    assert(isnumeric(C.time_grid_s)&&isreal(C.time_grid_s) && ...
        K>=1&&all(isfinite(t))&&all(diff(t)>0), ...
        'TRIGGER_TIME_CLEARANCE:Time','Strict real time grid required.');
    assert(isnumeric(C.observed_ids)&&isreal(C.observed_ids) && ...
        M>0&&all(isfinite(ids))&&all(ids>0)&& ...
        all(ids==fix(ids)) && numel(unique(ids))==M, ...
        'TRIGGER_TIME_CLEARANCE:IDs', ...
        'Use only nonempty, unique genuine observed/retained obstacle IDs.');
    D=C.sample_clearance_lower_m;
    V=C.speed_upper_mps;
    assert(isnumeric(D)&&isreal(D)&& ...
        isequal(size(D),[M K+1])&&all(isfinite(D(:))) && ...
        isnumeric(V)&&isreal(V)&& ...
        isequal(size(V),[1 K]) && ...
        all(isfinite(V(:)))&&all(V(:)>=0), ...
        'TRIGGER_TIME_CLEARANCE:Bounds', ...
        'Finite Mx(K+1) distance lower bounds and 1xK true speeds required.');
    D=double(D);V=double(V);
    % Treat each binary64 timestamp as its exact represented value.
    % Each basic operation is correctly rounded on the MATLAB IEEE-754
    % binary64 runtime, then enlarged by one representable number in the
    % required direction. Unlike a scale-dependent arbitrary eps margin,
    % this encloses the REAL arithmetic expression for FINITE inputs.
    % The physical clearance and speed inputs remain external obligations.
    h_upper=TRIGGER_FLOAT_OUTWARD_ONE_ULP(diff(t),+1);
    vh_upper=TRIGGER_FLOAT_OUTWARD_ONE_ULP(V.*h_upper,+1);
    endpoint_sum_lower=TRIGGER_FLOAT_OUTWARD_ONE_ULP( ...
        D(:,1:K)+D(:,2:K+1),-1);
    numerator_lower=TRIGGER_FLOAT_OUTWARD_ONE_ULP( ...
        endpoint_sum_lower-repmat(vh_upper,M,1),-1);
    lower=TRIGGER_FLOAT_OUTWARD_ONE_ULP( ...
        0.5*numerator_lower,-1);
    assert(all(isfinite([h_upper(:);vh_upper(:);lower(:)])), ...
        'TRIGGER_TIME_CLEARANCE:ArithmeticOverflow', ...
        'Directed arithmetic overflow: no clearance conclusion permitted.');
    % Strict >0 is sufficient because lower is already rounded OUTWARD.
    % The work remains O(M*K), with no shape-specific branches.
    sep=lower>0;
    R.interval_duration_upper_s=h_upper;
    R.interval_count=K;R.observed_count=M;
    R.clearance_lower_m=lower;
    R.separated_intervals=sep;
    R.possible_risk_ids=ids(any(~sep,2).');
    R.earliest_possible_event=TRIGGER_FIRST_UNPROVED_INTERVAL(t,ids,sep);
    R.all_intervals_separated=all(sep(:));
    R.conditional_clearance=R.all_intervals_separated;
    if R.all_intervals_separated
        R.status='CONDITIONAL_ALL_TIME_HAUSDORFF_CLEAR__NOT_FLIGHT_PROOF';
    else
        R.status='UNKNOWN_TEMPORAL_INTERVALS_NOT_SEPARATED';
    end
    R.reason=['Requires independently certified true body speed, ' ...
        'sampled material clearance lower bounds, static observed-only ' ...
        'geometry, taut body and verified IEEE-754 arithmetic semantics. ' ...
        'No Flight permission.'];
catch ME
    R.status='UNKNOWN_TEMPORAL_WITNESS_INPUT_OR_ARITHMETIC';
    R.reason=[ME.identifier ' | ' ME.message];
end
end
