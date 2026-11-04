function R=CHECK_TRIGGER_DISCARDED_INCREMENTAL_APPEND(replanner_instance)
% Verify observed-only 2=>3 append in the genuinely discarded full update.
R=struct('status','NOT_COMPLETED_DISCARDED_INCREMENTAL', ...
    'reason','','update_calls',0,'append_calls',0,'flight_safety',false);
try
    assert(nargin==1 && isa(replanner_instance,'REPLANNING_SYSTEM'), ...
        'CHECK_TRIGGER_DISCARDED_INCREMENTAL_APPEND:Input', ...
        'Provide the actual replanner_instance after a Flight.');
    S=replanner_instance.get_status();
    R.update_calls=S.isolated_full_update_prepare_calls;
    R.append_calls=S.isolated_full_update_prepare_incremental_appends;
    assert(R.update_calls==24 && R.append_calls==1 && ...
        strcmp(S.isolated_full_update_prepare_status, ...
        'PASS_DISCARDED_FULL_UPDATE_16_NONFLIGHT_8_FLIGHT__NOT_JIT_OR_SAFETY'), ...
        'CHECK_TRIGGER_DISCARDED_INCREMENTAL_APPEND:Coverage', ...
        'Genuinely new synthetic ID after two learned IDs must pass append.');
    R.status='PASS_DISCARDED_2_TO_3_NONEMPTY_APPEND__NO_JIT_OR_FLIGHT_PROOF';
catch ME
    R.reason=[ME.identifier ' | ' ME.message];
end
fid=fopen('TRIGGER_DISCARDED_INCREMENTAL_APPEND_RESULT.txt','w');
if fid>=0
    fprintf(fid,'status=%s\ncalls=%d\nappends=%d\nreason=%s\n', ...
        R.status,R.update_calls,R.append_calls,R.reason);
    fclose(fid);
end
end
