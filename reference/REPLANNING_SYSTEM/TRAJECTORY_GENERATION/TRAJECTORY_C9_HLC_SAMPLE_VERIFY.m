function R=TRAJECTORY_C9_HLC_SAMPLE_VERIFY(sent,internal,raw,clamped,lower,upper,expected,tolerance)
%TRAJECTORY_C9_HLC_SAMPLE_VERIFY  Pure, read-only four-input/28-ref audit.
% A single common validator is used by both the post-HLC live tick and the
% recorded Sim LOGGER audit. No reference/actuator is modified or committed.
% Expected=[] and tolerance=[] mean NO generated reference is available;
% this never grants a trajectory match or execution certificate.
% The HLC is allowed to lift yaw by integer 2*pi turns, NOT to change an
% arbitrary amount. Its historical acos(dot) yaw expression has sqrt(eps)
% conditioning near collinear headings: the yaw guard below is a numerical
% representation allowance, never a tracking tube or physical tolerance.
R=struct('status','UNKNOWN_HLC_SAMPLE_INVALID','sample_checked',false, ...
    'reference_check_available',false,'reference_match',false, ...
    'internal_nonyaw_match',false,'internal_yaw_lift_valid',false, ...
    'yaw_lift_integer_turns',NaN,'yaw_lift_residual_rad',NaN, ...
    'yaw_numeric_guard_rad',NaN,'raw_input_feasible',false, ...
    'hlc_clamp_consistent',false,'saturated',false, ...
    'minimum_raw_actuator_headroom',NaN,'max_saturation_change',NaN, ...
    'reference_max_error',NaN,'expected_clamp_4',NaN(4,1), ...
    'safe_to_commit',false,'physical_continuous_time_certified',false);
if nargin~=8, R.status='UNKNOWN_HLC_SAMPLE_INPUT_COUNT';return;end
if ~isvalidvec(sent,28)||~isvalidvec(internal,28)|| ...
        ~isvalidvec(raw,4)||~isvalidvec(clamped,4)|| ...
        ~isvalidvec(lower,4)||~isvalidvec(upper,4)
    R.status='UNKNOWN_HLC_SAMPLE_DIMENSIONS_OR_NONFINITE';return
end
sent=double(sent);internal=double(internal);raw=double(raw);
clamped=double(clamped);lower=double(lower);upper=double(upper);
if any(lower>=upper) || lower(1)<0
    R.status='UNKNOWN_HLC_INPUT_LIMITS_INVALID';return
end
if ~(isempty(expected)&&isempty(tolerance))
    if ~isvalidvec(expected,28)||~isvalidvec(tolerance,28)||any(tolerance<0)
        R.status='UNKNOWN_PLANNED_REFERENCE_OR_TOLERANCE_INVALID';return
    end
    expected=double(expected);tolerance=double(tolerance);
    R.reference_check_available=true;
    R.reference_max_error=max(abs(sent-expected));
    R.reference_match=all(abs(sent-expected)<=tolerance);
end
R.expected_clamp_4=max(lower,min(upper,raw));
R.hlc_clamp_consistent=isequal(clamped,R.expected_clamp_4);
R.raw_input_feasible=all(raw>=lower & raw<=upper);
R.saturated=~isequal(raw,clamped);
R.minimum_raw_actuator_headroom=min([raw-lower;upper-raw]);
R.max_saturation_change=max(abs(raw-clamped));
R.internal_nonyaw_match=isequal(internal([1:3 5:28]),sent([1:3 5:28]));
% Compare angles modulo integer 2*pi, not by blindly skipping channel 4.
d=internal(4)-sent(4);
k=round(d/(2*pi));
res=d-(2*pi)*k;
% The controller's sign(cross)*acos(dot) can incur sqrt(machine eps)
% angle error even when the physical two headings are aligned.
R.yaw_numeric_guard_rad=2*acos(max(-1,1-32*eps(1))) + ...
    64*eps(max([1 abs(internal(4)) abs(sent(4)) abs(d)]));
R.yaw_lift_integer_turns=k;
R.yaw_lift_residual_rad=res;
R.internal_yaw_lift_valid=isfinite(k)&&isfinite(res)&& ...
    abs(res)<=R.yaw_numeric_guard_rad;
% A changed sent reference is the primary failure even if HLC.xd still
% holds a stale internal copy of the previous reference. This makes the
% failure reason invariant to a simultaneous internal mismatch.
if R.reference_check_available && ~R.reference_match
    R.status='REJECT_ACTUAL_HLC_REFERENCE_MISMATCH';
elseif ~R.internal_nonyaw_match
    R.status='REJECT_HLC_INTERNAL_REFERENCE_CHANGED_OTHER_CHANNELS';
elseif ~R.internal_yaw_lift_valid
    R.status='REJECT_HLC_INTERNAL_YAW_NOT_EQUIVALENT';
elseif ~R.hlc_clamp_consistent
    R.status='REJECT_ACTUAL_HLC_CLAMP_MISMATCH';
elseif ~R.raw_input_feasible
    R.status='REJECT_REAL_HLC_PRECLAMP_ACTUATOR_SATURATION';
elseif R.reference_check_available
    R.status='PASS_REAL_HLC_ONE_TICK_NO_SATURATION__TRACKING_UNCERTIFIED';
    R.sample_checked=true;
else
    R.status='CHECKED_REAL_HLC_SAMPLE_NO_TRAJECTORY_REFERENCE_BINDING';
    R.sample_checked=true;
end
end

function ok=isvalidvec(x,n)
ok=isnumeric(x)&&isreal(x)&&isequal(size(x),[n 1])&& ...
    all(isfinite(x(:)));
end
