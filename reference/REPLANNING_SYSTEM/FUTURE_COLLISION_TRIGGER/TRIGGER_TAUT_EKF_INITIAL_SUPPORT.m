function R=TRIGGER_TAUT_EKF_INITIAL_SUPPORT(D)
% Conditioned INITIAL FULL-BODY directional support from signed EKF-error
% inclusion AND the exact taut-cable manifold. Only at the initial instant.
%
% Input D.initial must be a successful TRIGGER_EKF_TAUT_INITIAL_ENCLOSURE
% result, whose raw errors must have been PROVEN by a separate estimator
% theorem. Required body radii, true cable length enclosure, normals
% are EXTERNAL physical quantities; no guessed margins or obstacle truths.
%
% Let d lie in a unit-sphere intersection of its componentwise box.
% Let axis = midpoint(dlo,dhi)/||midpoint||. In real arithmetic,
%   axis'*d >= sum(min(axis_i*dlo_i,axis_i*dhi_i)) := m,
% therefore angle(d,axis) <= acos(clamp(m,-1,1)).
% TRIGGER_TAUT_DIRECTION_CONE_SUPPORT then encloses the entire
% {pQ+s*L*d: 0<=s<=1} plus body/payload spheres. No separate payload
% uncertainty is invented: taut pL=pQ+L*d is a mandatory hypothesis.
%
% Neither EKF-error validity, taut hybrid mode, source parity nor MATLAB
% elementary-function directed rounding is proved by this routine.
R=struct('status','UNKNOWN_EKF_TAUT_INITIAL_SUPPORT', ...
    'conditional_enclosure',false,'safe_to_commit',false, ...
    'reason','','axis',[NaN;NaN;NaN], ...
    'direction_angle_upper_rad',NaN, ...
    'union_support_upper',zeros(1,0),'uav_support_upper',zeros(1,0), ...
    'cable_support_upper',zeros(1,0),'payload_support_upper',zeros(1,0));
try
    required={'initial','cable_length_lower_m','cable_length_upper_m', ...
        'r_uav_m','r_cable_m','r_payload_m','normals'};
    assert(isstruct(D)&&isscalar(D)&&all(isfield(D,required)), ...
        'TRIGGER_EKF_SUPPORT:Inputs','Explicit verified initial box and physical geometry needed.');
    E=D.initial;
    assert(isstruct(E)&&isscalar(E)&&isfield(E,'conditional_enclosure') && ...
        isequal(E.conditional_enclosure,true)&& ...
        isfield(E,'initial_lower')&&isfield(E,'initial_upper') && ...
        isequal(size(E.initial_lower),[24 1])&& ...
        isequal(size(E.initial_upper),[24 1]), ...
        'TRIGGER_EKF_SUPPORT:Initial','A completed 24-state initial enclosure is required.');
    lo=double(E.initial_lower);hi=double(E.initial_upper);
    assert(all(isfinite([lo;hi]))&&all(lo<=hi), ...
        'TRIGGER_EKF_SUPPORT:InitialBox');
    dl=lo(19:21);du=hi(19:21);
    near=max(0,max(dl,-du));far=max(abs(dl),abs(du));
    assert(sum(near.^2)<=1 && sum(far.^2)>=1, ...
        'TRIGGER_EKF_SUPPORT:UnitSphere','Direction box misses the unit sphere.');
    mid=(dl+du)/2;
    if norm(mid)>0
        axis=mid/norm(mid);
    else
        axis=[0;0;-1]; % A search direction only; not a true-state guess.
    end
    low_dot=sum(max(axis,0).*dl+min(axis,0).*du);
    alpha=acos(max(-1,min(1,low_dot)));
    G=TRIGGER_TAUT_DIRECTION_CONE_SUPPORT(struct( ...
        'uav_lower',lo(1:3),'uav_upper',hi(1:3), ...
        'initial_direction_axis',axis, ...
        'initial_direction_angle_upper_rad',alpha, ...
        'angular_speed_upper_rad_s',0,'interval_duration_s',0, ...
        'cable_length_lower_m',D.cable_length_lower_m, ...
        'cable_length_upper_m',D.cable_length_upper_m, ...
        'r_uav_m',D.r_uav_m,'r_cable_m',D.r_cable_m, ...
        'r_payload_m',D.r_payload_m,'normals',D.normals));
    if ~G.conditional_enclosure
        R.reason=G.reason;
        return
    end
    R.axis=axis;
    R.direction_angle_upper_rad=alpha;
    R.union_support_upper=G.union_support_upper;
    R.uav_support_upper=G.uav_support_upper;
    R.cable_support_upper=G.cable_support_upper;
    R.payload_support_upper=G.payload_support_upper;
    R.conditional_enclosure=true;
    R.status='CONDITIONAL_EKF_INITIAL_FULL_BODY_SUPPORT__NOT_FLIGHT_SAFETY';
    R.reason=['Exact real-arithmetic taut support at the initial instant; ' ...
        'valid ONLY when externally proved signed EKF errors, taut mode, ' ...
        'physical length/radii and sound directed rounding are established.'];
catch ME
    R.reason=[ME.identifier ' | ' ME.message];
end
end
