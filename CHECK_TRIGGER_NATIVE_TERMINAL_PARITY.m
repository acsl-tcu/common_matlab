function R=CHECK_TRIGGER_NATIVE_TERMINAL_PARITY(logger,agent)
% Source-bound ORIGINAL HLC+plant numerical regression AFTER a new build.
% This does not prove all-time safety, controller deadline, or EKF truth.
% The underlying byte-pinned audit is intentionally run on REAL recorded
% Flight state and references, without controlling/changing the Flight.
R=struct('status','FAIL_NATIVE_TERMINAL_PARITY', ...
 'reason','','numeric_parity_pass',false,'fresh_pin_pass',false, ...
 'source_hash_pass',false,'source_pin_status','', ...
 'original_full_horizon_records',0);
try
    assert(nargin==2 && isobject(logger) && isobject(agent), ...
        'Pass the logger and agent variables retained from a completed Flight.');
    T=TRIGGER_NATIVE_MODEL_SOURCE_CHECK(false);
    R.source_hash_pass=logical(T.cpp_source_matches_verified);
    assert(R.source_hash_pass,'Modified C++ source hash was not verified.');
    correct=fullfile(fileparts(mfilename('fullpath')), ...
        'CHECK_TRIGGER_NATIVE_MODEL_PARITY.m');
    resolved=which('CHECK_TRIGGER_NATIVE_MODEL_PARITY');
    assert(strcmpi(char(resolved),char(correct)), ...
        'The common_matlab/ numerical parity checker must be first on the MATLAB path.');
    Q=CHECK_TRIGGER_NATIVE_MODEL_PARITY(logger,agent);
    R.numeric_parity_pass=logical(Q.parity_pass);
    R.original_full_horizon_records=double(Q.full_horizon_records);
    T=TRIGGER_NATIVE_MODEL_SOURCE_CHECK(true);
    R.fresh_pin_pass=logical(T.passed) && logical(T.audit_pass);
    R.source_pin_status=T.status;
    assert(R.numeric_parity_pass && R.fresh_pin_pass, ...
        'Original HLC/plant numeric parity or fresh executable hash failed.');
    TRIGGER_NATIVE_SOURCE_PIN_CACHE('prepare');
    R.status='PASS_FRESH_NATIVE_PARITY__NOT_CONTINUOUS_SAFETY_OR_WCET';
catch ME
    R.reason=[ME.identifier ' | ' ME.message];
end
root=fileparts(mfilename('fullpath'));
save(fullfile(root,'TRIGGER_NATIVE_TERMINAL_PARITY_RESULT.mat'),'R');
fid=fopen(fullfile(root,'TRIGGER_NATIVE_TERMINAL_PARITY_RESULT.txt'),'w');
if fid>0
 fprintf(fid,['status=%s\nreason=%s\nnumeric=%d\nsource_hash=%d' ...
     '\nfresh_pin=%d\nsource_pin_status=%s\nfull_horizon_records=%d\n'], ...
     R.status,R.reason,R.numeric_parity_pass,R.source_hash_pass, ...
     R.fresh_pin_pass,R.source_pin_status, ...
     R.original_full_horizon_records);
 fclose(fid);
end
end
