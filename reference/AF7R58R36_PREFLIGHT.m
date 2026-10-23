function Q=AF7R58R36_PREFLIGHT()
% Source-bound, production unchanged except R36 timing probes.
p=which('REPLANNING_SYSTEM');
assert(~isempty(p),'REPLANNING_SYSTEM is missing.');
s=fileread(p);
names={'r36_t_sensor_gate','r36_t_memory','r36_t_trigger','r36_t_request', ...
 'r36_t_advance','rtime_sensor_closure_ms'};
for i=1:numel(names)
 assert(contains(s,names{i}),'R36 source patch absent: %s',names{i});
end
required={'AF7R58R36_TIMING_CLOSE','AF7R58R36_ESTIMATED_ORIGIN_AUDIT', ...
 'AF7R58R36_RETURN_SIGN_AUDIT'};
for i=1:numel(required)
 assert(exist(required{i},'file')==2,'R36 helper missing: %s',required{i});
end
Q=struct('source',p,'timing_probes_installed',true, ...
 'verified_estimator_origin_trigger_production',false, ...
 'trigger_authority_changed',false,'all_body_safety_changed',false, ...
 'C7_return_candidate_auto_commit',false,'native_rebuild_required',false, ...
 'matlab_runtime_validated',false);
disp('[R36] SOURCE CHECK PASS | timing probes only; estimated-origin trigger SHADOW; no return auto-commit.');
end
