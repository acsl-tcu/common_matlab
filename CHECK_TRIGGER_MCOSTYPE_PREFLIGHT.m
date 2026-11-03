function R=CHECK_TRIGGER_MCOSTYPE_PREFLIGHT(replanner_instance)
% Disposable verification placed at common_matlab/ root; no code changes.
assert(nargin==1&&isa(replanner_instance,'REPLANNING_SYSTEM'), ...
    'CHECK_TRIGGER_MCOSTYPE_PREFLIGHT:Input', ...
    'Pass the completed Flight replanner_instance.');
s=replanner_instance.get_status();
R=struct('status','NO_MCOSTYPE_RESULT','preflight_status','', ...
    'mcos_calls',0,'reason','');
if isfield(s,'isolated_mcos_current_prepare_status')
    R.preflight_status=char(s.isolated_mcos_current_prepare_status);
end
if isfield(s,'isolated_mcos_current_prepare_reason')
    R.reason=char(s.isolated_mcos_current_prepare_reason);
end
if isfield(s,'isolated_mcos_current_prepare_calls')
    R.mcos_calls=double(s.isolated_mcos_current_prepare_calls);
end
if R.mcos_calls==8 && ...
        strcmp(R.preflight_status, ...
        'PASS_SYNTHETIC_REAL_MCOSTYPE_CURRENT_8__NO_JIT_OR_SAFETY_PROOF')
    R.status='PASS_DISCARDED_MCOSTYPE_PATH__FLIGHT_EFFECT_UNPROVED';
else
    R.status='MCOSTYPE_PREPARE_INCOMPLETE__KEEP_FLIGHT_BASELINE';
    R.reason='Read preflight_status; do not infer warm JIT or Flight safety.';
end
save('TRIGGER_MCOSTYPE_PREFLIGHT_RESULT.mat','R');
fid=fopen('TRIGGER_MCOSTYPE_PREFLIGHT_RESULT.txt','w');
if fid>=0
    fprintf(fid,'status=%s\npreflight_status=%s\nmcos_calls=%d\nreason=%s\n', ...
        R.status,R.preflight_status,R.mcos_calls,R.reason);
    fclose(fid);
end
end
