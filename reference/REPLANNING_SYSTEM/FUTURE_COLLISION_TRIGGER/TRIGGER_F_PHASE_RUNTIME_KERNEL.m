function [R,T]=TRIGGER_F_PHASE_RUNTIME_KERNEL(time,agent,sensed,binding,cache,core,abs_tol,rel_tol,trace_on)
% Source-identical Flight f-phase INNER evaluation. The original f-only
% guards, actual observed sensor provenance and physical binding live in
% REPLANNING_SYSTEM.evaluate_trigger. A discarded real-MCOS fixture calls
% this same function pre-arm with synthetic state, reference and shapes.
% The function MUST NOT invent a phase start, alter the agent, or publish
% collision events. All source-time and physics guards remain mandatory.
% Never interpret a nominal sampled prediction as continuous safety.
if nargin<9,trace_on=false;end
if trace_on
    T=struct('reference_now_ms',NaN,'authority_create_ms',NaN, ...
        'authority_current_ms',NaN,'core_wall_ms',NaN, ...
        'core_trace',struct());
    step=tic;
else
    T=struct();
end
nominal=agent.reference.timevarying;
if ~isa(nominal.func,'function_handle') || isempty(nominal.t)
    error('REPLANNING_SYSTEM:ReferenceNotBound', ...
        'Current nominal reference function and flight epoch are unavailable.');
end
if strcmp(class(agent.parameter),'DRONE_PARAM_SUSPENDED_LOAD')
    L=double(agent.parameter.cableL);
else
    L=double(agent.parameter.get("cableL"));
end
assert(isnumeric(L)&&isscalar(L)&&isreal(L)&& ...
    isfinite(L)&&L>0 && ...
    isequal(L,binding.flight.cable_length_m), ...
    'REPLANNING_SYSTEM:LiveCableLengthChanged', ...
    'Live cable length differs from preflight plant binding.');
% This closure factory and its callsite are common to prearm and Flight;
% in prearm nominal.t is a discarded SYNTHETIC local epoch only.
future=@(tt)REPLANNING_SYSTEM.flight_reference_from_nominal(nominal,tt,L);
t=double(time.t);
assert(isnumeric(time.t)&&isscalar(time.t)&&isreal(time.t)&&isfinite(t), ...
    'REPLANNING_SYSTEM:Time','Invalid actual TIME object time.t.');
actual=agent.reference.result.state.xd;
predicted_now=future(t);
if ~isequal(size(actual),size(predicted_now)) || ...
        norm(double(actual(:))-predicted_now(:),inf)> ...
        128*eps(max(1,norm(double(actual(:)),inf)))
    error('REPLANNING_SYSTEM:ReferenceMismatch', ...
        'Future reference evaluator does not reproduce current actual HLC reference.');
end
if trace_on
    T.reference_now_ms=1000*toc(step);
    step=tic;
    T.authority_create_ms=1000*toc(step);
    step=tic;
end
[ekf,authority]=binding.current(time,agent,future);
if trace_on,T.authority_current_ms=1000*toc(step);step=tic;end
% The real preflight-pinned model, real same-tick current EKF and exact
% currently observed sensor frame are the ONLY runtime sources.
authority.native_reference_cache=cache;
authority.native_reference_source=nominal;
authority.native_validated_now_time_s=t;
authority.native_validated_now_jet=predicted_now;
authority.native_reference_contract= ...
    'VERIFIED_F_PHASE_NOMINAL_Z_MINUS_CABLEL';
authority.numeric_tolerances=[abs_tol;rel_tol];
if trace_on,authority.trace_first=true;end
R=core.evaluate(t,ekf,sensed,authority);
if trace_on
    T.core_wall_ms=1000*toc(step);
    if isfield(R,'first_use_trace'),T.core_trace=R.first_use_trace;end
end
end
