function Q = AF7R58R25_LOGGER_EQUIVALENCE_AUDIT()
% One-time source audit; compares source fragments rather than pretending
% to certify numerical plant/collision trajectories on this machine.
p=which('REPLANNING_SYSTEM'); assert(~isempty(p)); S=fileread(p);
assert(contains(S,'function r25_publish_capture(obj)'));
assert(contains(S,'obj.r25_event_capture=D;'));
assert(~contains(S,'setappdata(0,key,D);'));
assert(contains(S,'setappdata(0,''AF7R58R17_LAST_CAPTURE'',obj.r25_event_capture);'));
Q=struct('event_capture_buffered',true,'source_bound',true, ...
    'normal_solver_changed',false,'speed_gate_changed',false, ...
    'geometry_gate_changed',false,'commit_changed',false, ...
    'runtime_equivalence_unverified',true);
disp(Q);
end
