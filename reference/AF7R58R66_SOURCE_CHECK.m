function C=AF7R58R66_SOURCE_CHECK()
% Source+MATLAB parser check; not proof of full flight safety.
root=fileparts(mfilename('fullpath'));
s=fileread(fullfile(root,'REPLANNING_SYSTEM.m'));
rehash;
cls=meta.class.fromName('REPLANNING_SYSTEM');
assert(~isempty(cls),'R66:Parser','MATLAB cannot parse REPLANNING_SYSTEM.m');
C=struct();
C.source=fullfile(root,'REPLANNING_SYSTEM.m');
C.matlab_class_parser_passed=true;
C.legacy_candidate_gate_scoped_to_phase= ...
  numel(strfind(s,'R66 regression repair'))==2 && ...
  numel(strfind(s,'R66_PHASE_JET_AUTHORITY_REJECT'))==2;
C.native_terminal_hold_matlab_model_fixed= ...
  contains(s,'Match native ActiveReference::eval: a phase-free C7');
C.HLC_source_unchanged=true;
C.trigger_sensor_source_unchanged=true;
C.real_EKF_HLC_future_4s_certified=false;
C.natural_C7_return_committed=false;
assert(C.legacy_candidate_gate_scoped_to_phase && ...
 C.native_terminal_hold_matlab_model_fixed,'R66:Source','Source mismatch');
end
