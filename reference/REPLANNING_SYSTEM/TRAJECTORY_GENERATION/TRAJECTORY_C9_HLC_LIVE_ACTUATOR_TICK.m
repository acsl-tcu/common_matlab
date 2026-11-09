function R=TRAJECTORY_C9_HLC_LIVE_ACTUATOR_TICK(binding,agent,time,expected_ref28,tolerance28)
%TRAJECTORY_C9_HLC_LIVE_ACTUATOR_TICK Read-only REAL HLC post-do snapshot.
% Caller must invoke AFTER the current tick's HLC.do and BEFORE the next
% controller update. Reads the ACTUAL HLC result.tmp (UNSATURATED) and
% result.input (SATURATED), plus the reference entering that HLC. A clamp
% can hide actuator infeasibility if only result.input is checked.
% The source limits and current EKF mL come from the live bound authority.
% No HLC/plant mutation, no claim of continuous-time tracking or torque.
clock=tic;
R=struct('status','UNKNOWN_NOT_CAPTURED','sample_checked',false, ...
    'reference_match',false,'raw_input_feasible',false, ...
    'hlc_clamp_consistent',false,'saturated',false, ...
    'internal_yaw_lift_valid',false,'yaw_lift_residual_rad',NaN, ...
    'safe_to_commit',false,'physical_continuous_time_certified',false, ...
    'time_s',NaN,'authority_snapshot',struct(), ...
    'reference_sent28',NaN(28,1),'reference_expected28',NaN(28,1), ...
    'hlc_internal_reference28',NaN(28,1), ...
    'unclamped_input_4',NaN(4,1),'clamped_input_4',NaN(4,1), ...
    'expected_clamp_4',NaN(4,1), ...
    'minimum_raw_actuator_headroom',NaN, ...
    'max_saturation_change',NaN,'reference_max_error',NaN, ...
    'elapsed_ms',NaN, ...
    'scope','ACTUAL_HLC_SINGLE_TICK_PRECLAMP_DIAGNOSTIC__NOT_HLC_PROOF');
assert(isnumeric(expected_ref28)&&isreal(expected_ref28)&& ...
    isequal(size(expected_ref28),[28 1])&&all(isfinite(expected_ref28)), ...
    'C9_HLC_TICK:Expected','Require a real 28x1 planned reference.');
assert(isnumeric(tolerance28)&&isreal(tolerance28)&& ...
    isequal(size(tolerance28),[28 1])&&all(isfinite(tolerance28))&& ...
    all(tolerance28>=0), ...
    'C9_HLC_TICK:Tolerance','Require 28 caller-owned nonnegative tolerances.');
S=TRAJECTORY_C9_HLC_FORCE_AUTHORITY.current(binding,agent,time);
R.authority_snapshot=S;
if ~S.valid
    R.status=S.status;
    R.elapsed_ms=toc(clock)*1000;return
end
R.time_s=double(time.t);
controller=agent.controller.hlc_suspended;
if ~isstruct(controller.result)|| ...
        ~all(isfield(controller.result,{'tmp','input','xd'}))
    R.status='UNKNOWN_CURRENT_HLC_RESULT_MISSING';
    R.elapsed_ms=toc(clock)*1000;return
end
try
    state=agent.reference.result.state;
    if isstruct(state)&&isfield(state,'xd')
        realref=state.xd;
    elseif isobject(state)&&isprop(state,'xd')
        realref=state.xd;
    elseif isobject(state)&&ismethod(state,'get')
        realref=state.get();
    else
        R.status='UNKNOWN_REAL_HLC_REFERENCE_NOT_AVAILABLE';
        R.elapsed_ms=toc(clock)*1000;return
    end
catch ME
    R.status=['UNKNOWN_REAL_HLC_REFERENCE_READ_' ME.identifier];
    R.elapsed_ms=toc(clock)*1000;return
end
raw=controller.result.tmp;
actual=controller.result.input;
internal=controller.result.xd;
if ~(isnumeric(realref)&&isreal(realref)&&isequal(size(realref),[28 1])&& ...
        all(isfinite(realref))&&isnumeric(internal)&& ...
        isreal(internal)&&isequal(size(internal),[28 1])&& ...
        all(isfinite(internal))&& ...
        isnumeric(raw)&&isreal(raw)&&isequal(size(raw),[4 1])&& ...
        all(isfinite(raw))&&isnumeric(actual)&&isreal(actual)&& ...
        isequal(size(actual),[4 1])&&all(isfinite(actual)))
    R.status='UNKNOWN_REAL_HLC_TICK_DIMENSIONS_OR_NONFINITE';
    R.elapsed_ms=toc(clock)*1000;return
end
R.reference_sent28=double(realref);
R.reference_expected28=double(expected_ref28);
R.hlc_internal_reference28=double(internal);
R.unclamped_input_4=double(raw);
R.clamped_input_4=double(actual);
lo=double(binding.input_lower(:));hi=double(binding.input_upper(:));
V=TRAJECTORY_C9_HLC_SAMPLE_VERIFY(double(realref),double(internal), ...
    double(raw),double(actual),lo,hi,double(expected_ref28), ...
    double(tolerance28));
R.expected_clamp_4=V.expected_clamp_4;
R.hlc_clamp_consistent=V.hlc_clamp_consistent;
R.raw_input_feasible=V.raw_input_feasible;
R.saturated=V.saturated;
R.max_saturation_change=V.max_saturation_change;
R.minimum_raw_actuator_headroom=V.minimum_raw_actuator_headroom;
R.reference_max_error=V.reference_max_error;
R.reference_match=V.reference_match;
R.internal_yaw_lift_valid=V.internal_yaw_lift_valid;
R.yaw_lift_residual_rad=V.yaw_lift_residual_rad;
R.sample_checked=V.sample_checked;
R.status=V.status;
R.elapsed_ms=toc(clock)*1000;
end
