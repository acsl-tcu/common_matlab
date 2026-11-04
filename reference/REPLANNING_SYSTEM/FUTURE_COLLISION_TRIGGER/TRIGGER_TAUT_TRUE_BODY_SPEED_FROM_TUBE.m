function R=TRIGGER_TAUT_TRUE_BODY_SPEED_FROM_TUBE(tube_lower,tube_upper)
% TRIGGER_TAUT_TRUE_BODY_SPEED_FROM_TUBE
% Derive (do NOT assume) a whole-body Hausdorff speed bound from the
% original 24-state all-time Picard tube. Exact original indices:
%   UAV center velocity = x(7:9); payload center velocity = x(16:18).
%
% A material point of the STRAIGHT TAUT cable capsule is
%   c(s,t) = (1-s)*q(t) + s*l(t),  0<=s<=1.
% For fixed material s, c_dot=(1-s)*q_dot+s*l_dot, whence
%   ||c_dot|| <= max(||q_dot||, ||l_dot||).
% Including constant-radius UAV/payload balls and cable capsule gives
% the SAME Hausdorff-speed upper bound for the whole physical body.
% No angular cone or giant load sphere is required.  The bound is valid
% only if the input tubes truly contain ALL actual states at ALL times,
% and the straight-taut geometry with constant radii is physically valid.
%
% Finite outward interval operations are used for the squared norm; the
% final correctly rounded sqrt is expanded by one representable value.
% The MATLAB/runtime elementary arithmetic has NOT been independently
% audited: CONDITIONAL only, never a Flight safety permission.
R=struct('status','UNKNOWN_TRUE_BODY_SPEED_TUBE', ...
    'conditional_bound',false,'uav_speed_upper_mps',[], ...
    'payload_speed_upper_mps',[], 'whole_body_speed_upper_mps',[], ...
    'safe_to_commit',false,'reason','');
try
    assert(isnumeric(tube_lower)&&isnumeric(tube_upper) && ...
        isa(tube_lower,'double')&&isa(tube_upper,'double') && ...
        isreal(tube_lower)&&isreal(tube_upper) && ...
        size(tube_lower,1)==24 && ...
        isequal(size(tube_lower),size(tube_upper)) && ...
        size(tube_lower,2)>=1 && ...
        all(isfinite(tube_lower(:))) && all(isfinite(tube_upper(:))) && ...
        all(tube_lower(:)<=tube_upper(:)), ...
        'TRIGGER_TRUE_BODY_SPEED:Tube', ...
        'Require finite ordered 24xK ALL-TIME state boxes.');
    % Entire K-column interval speed evaluation is vectorized: each
    % elementary multiplication/addition/sqrt is widened UP separately.
    % This avoids 6*K scalar interval-object constructions in hot-path
    % candidate code.  No assumption on reference trajectory is used.
    qabs=max(abs(tube_lower(7:9,:)),abs(tube_upper(7:9,:)));
    labs=max(abs(tube_lower(16:18,:)),abs(tube_upper(16:18,:)));
    qsq=TRIGGER_FLOAT_OUTWARD_ONE_ULP(qabs.*qabs,+1);
    lsq=TRIGGER_FLOAT_OUTWARD_ONE_ULP(labs.*labs,+1);
    qsum=TRIGGER_FLOAT_OUTWARD_ONE_ULP(qsq(1,:)+qsq(2,:),+1);
    qsum=TRIGGER_FLOAT_OUTWARD_ONE_ULP(qsum+qsq(3,:),+1);
    lsum=TRIGGER_FLOAT_OUTWARD_ONE_ULP(lsq(1,:)+lsq(2,:),+1);
    lsum=TRIGGER_FLOAT_OUTWARD_ONE_ULP(lsum+lsq(3,:),+1);
    vq=TRIGGER_FLOAT_OUTWARD_ONE_ULP(sqrt(qsum),+1);
    vl=TRIGGER_FLOAT_OUTWARD_ONE_ULP(sqrt(lsum),+1);
    assert(all(isfinite(vq))&&all(isfinite(vl)), ...
        'TRIGGER_TRUE_BODY_SPEED:Overflow', ...
        'Whole-body speed bound overflow.');
    R.uav_speed_upper_mps=vq;
    R.payload_speed_upper_mps=vl;
    R.whole_body_speed_upper_mps=max(vq,vl);
    R.conditional_bound=true;
    R.status='CONDITIONAL_ORIGINAL_TUBE_FULL_BODY_SPEED__NOT_FLIGHT_PROOF';
    R.reason=['The bound follows from true endpoint velocities and the ' ...
        'straight-taut cable identity. Real all-time plant/EKF inclusion, ' ...
        'constant radii, tension, and audited arithmetic are still required.'];
catch ME
    R.status='UNKNOWN_TRUE_BODY_SPEED_TUBE';
    R.reason=[ME.identifier ' | ' ME.message];
end
end
