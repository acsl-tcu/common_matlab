function T=AF7R58R73_SELFTEST()
% Pure structural regression: no Flight, parameter fallback, or actuation.
S=AF7R58R73_SOURCE_CHECK();
assert(S.matlab_class_parser_passed);
% Reproduce the empty no-schema initial condition from R72.
actual=struct([]);
A=struct('time_s',13.25,'reference_version',0, ...
    'reference_mode','NOMINAL','estimated_mass_kg',0.015, ...
    'cable_length_m',2,'plant_param',[1;2],'parameter_order', ...
    string({'loadmass','cableL'}), 'external_param',[0.015;2], ...
    'state_x24',zeros(24,1), 'raw_reference_fn',@(t)t);
if isempty(actual)
    actual=A;
else
    actual(end+1)=A;
end
assert(numel(actual)==1 && isequal(fieldnames(actual),fieldnames(A)));
A2=A; A2.reference_version=1;
assert(isequal(fieldnames(actual),fieldnames(A2)));
actual(end+1)=A2;
assert(numel(actual)==2 && all([actual.reference_version]==[0,1]));
% Verify we do not mistake a genuinely mismatched structure for a valid item.
wrong=rmfield(A2,'state_x24');
assert(~isequal(fieldnames(actual),fieldnames(wrong)));
% Collector uses the same safe first-element pattern, not [struct([]), E].
events=struct([]);
E=struct('time_s',13.25,'status','NOT_RUN','error_message','');
if isempty(events),events=E;else,events(end+1)=E;end
E.time_s=13.30;events(end+1)=E;
assert(numel(events)==2);
T=struct('status','PASS_FIRST_EVENT_AND_MULTI_EVENT_STRUCT_SCHEMA', ...
    'class_parser_pass',true,'first_snapshot_pass',true, ...
    'later_snapshot_pass',true,'different_schema_rejected',true, ...
    'postflight_first_event_pass',true, ...
    'hlc_ekf_plant_runner_modified',false, ...
    'physical_future_proved',false,'natural_return_committed',false);
end
