function Q=AF7R58R44_PREFLIGHT()
% No modification of REPLANNING_SYSTEM; verifies current source and requirements.
Q=struct('schema','AF7R58R44_PREFLIGHT_V1');
Q.source=which('REPLANNING_SYSTEM');
assert(~isempty(Q.source),'REPLANNING_SYSTEM.m is not on the path.');
txt=fileread(Q.source);
Q.sensor_estimated_geometry_present=contains(txt, ...
    'obj.get_current_physical_geometry(xd_active_now)');
Q.future_reference_trigger_present=contains(txt, ...
    'obj.trigger_core(active,pL_ref,q_ref,time.t)');
Q.live_mass_source_present=contains(txt, ...
    'obj.self.estimator.result.state.mL');
Q.source_hlc_class_present=~isempty(which('HLC_SUSPENDED_LOAD'));
Q.actual_pure_closed_loop_trigger_connected=false;
Q.actual_free_return_committed=false;
Q.timing_hard_bound_proved=false;
assert(Q.sensor_estimated_geometry_present && ...
    Q.future_reference_trigger_present && Q.live_mass_source_present, ...
    'Active source does not match R43; do not continue with stale assumptions.');
Q.trigger=AF7R58R44_SOURCE_AUTHORITY();
Q.status='R43_SOURCE_BOUND__TRIGGER_AND_RETURN_NOT_SWITCHED';
fprintf('[R44] R43 source bound. Trigger remains reference-future; no hidden substitute.\n');
end
