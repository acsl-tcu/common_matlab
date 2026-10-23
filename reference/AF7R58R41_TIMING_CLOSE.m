function R=AF7R58R41_TIMING_CLOSE(logger,runtime_profile)
% Event-bound, same-clock timing closure, read-only. Run AFTER Flight.
% Full iteration is attributed by R23; REPLANNING_SYSTEM.do intervals are
% partitioned by R41 only for stage and post-commit events. Missing is NaN.
assert(isa(logger,'LOGGER'),'[R41] LOGGER instance from this run required.');
P=runtime_profile;
req={'t','phase','r23_reference_s','full_iteration_s','online_before_plant_s', ...
 'sensor_acquisition_s','r23_sensor_s','r23_estimator_s','r23_controller_s', ...
 'r23_input_s','simulation_plant_s','logger_s','r23_other_s'};
assert(isstruct(P) && all(isfield(P,req)), ...
 '[R41] Missing same-run outer timing fields.');
ix=char(P.phase(:))=='f';
t=double(P.t(ix));t=t(:);
N=numel(t);
assert(N>=1 && all(isfinite(t)) && all(diff(t)>0), ...
 '[R41] Flight clock invalid.');
scale=1000;
ref=scale*double(P.r23_reference_s(ix));ref=ref(:);
full=scale*double(P.full_iteration_s(ix));full=full(:);
online=scale*double(P.online_before_plant_s(ix));online=online(:);
names={'sensor_acquisition_s','r23_sensor_s','r23_estimator_s', ...
 'r23_reference_s','r23_controller_s','r23_input_s', ...
 'simulation_plant_s','logger_s','r23_other_s'};
X=zeros(N,numel(names));
for k=1:numel(names)
    raw=P.(names{k});
    z=double(raw(ix));
    z=z(:);
    assert(numel(z)==N && all(isfinite(z)),'[R41] Invalid outer timing field %s.',names{k});
    X(:,k)=1000*z;
end
fullErr=full-sum(X,2);
assert(all(isfinite(fullErr)),'[R41] Full-cycle closure nonfinite.');
fields={'rfortyone_trace_flag','rfortyone_time_s', ...
 'rfortyone_upstream_ms','rfortyone_sensor_ms', ...
 'rfortyone_stages_ms','rfortyone_output_ms', ...
 'rfortyone_total_ms','rfortyone_closure_ms', ...
 'rfortyone_source_version'};
V=nan(N,numel(fields));
for j=1:numel(fields)
    try
        [v,~]=logger.data(1, "reference.result.state."+string(fields{j}), "", 'phase','f');
        v=double(v(:));
        assert(numel(v)==N,'[R41] LOGGER count mismatch: %s.',fields{j});
        V(:,j)=v;
    catch ME
        error('AF7R58R41:MissingLogger', ...
            '[R41] LOGGER field %s could not be read: %s',fields{j},ME.message);
    end
end
active=V(:,1)==1;
assert(any(active),'[R41] No event-bound trace recorded.');
assert(all(isfinite(V(active,:)),'all'), ...
 '[R41] Nonfinite event trace; time provenance must not be guessed.');
clockError=V(active,7)-sum(V(active,3:6),2);
loggerClockError=V(active,8);
assert(all(isfinite(clockError)) && all(isfinite(loggerClockError)), ...
 '[R41] Internal trace closure invalid.');
tol=max(1e-8,1e-6*max(1,max(abs(t))));
assert(all(abs(t(active)-V(active,2))<=tol), ...
 '[R41] Trace timestamp does not match its Flight cycle.');
outerGap=ref(active)-V(active,7);
R=struct();
R.schema='AF7R58R41_SOURCE_BOUND_EVENT_CLOSURE_V1';
R.phase_time_s=t;
R.trace_rows=table(t(active),ref(active),full(active),V(active,3),V(active,4), ...
 V(active,5),V(active,6),V(active,7),outerGap,clockError,V(active,9), ...
 'VariableNames',{'Time_s','Reference_ms','Full_ms','BeforeSensor_ms', ...
 'SensorAndTrigger_ms','StageAndRequest_ms','OutputAndState_ms', ...
 'DoTotal_ms','ReferenceWrapper_ms','DoClosureResidual_ms','ReferenceVersion'});
R.event_trace_count=nnz(active);
R.max_abs_full_closure_ms=max(abs(fullErr));
R.max_abs_do_closure_ms=max(abs(clockError));
R.max_abs_recorded_do_closure_ms=max(abs(loggerClockError));
R.online_over25_count=nnz(online>25);
R.full_over25_count=nnz(full>25);
R.full_cycle_closure_pass=R.max_abs_full_closure_ms<=1e-7;
R.do_cycle_closure_pass=R.max_abs_do_closure_ms<=1e-7;
R.wrapper_fully_attributed=false; % wrapper gap is measured, not subdivided.
R.observer_and_execution_tube_certified=false;
R.production_trigger_changed=false;
R.return_candidate_committed=false;
save('AF7R58R41_TIMING_CLOSE.mat','R');
writetable(R.trace_rows,'AF7R58R41_TIMING_EVENTS.csv');
fprintf('[R41] %d traced cycles; full/do closure %.5g / %.5g ms; maximum outer-minus-do %.6f ms\n', ...
 R.event_trace_count,R.max_abs_full_closure_ms, ...
 R.max_abs_do_closure_ms,max(outerGap));
end
