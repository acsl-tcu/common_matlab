function R=CHECK_TRIGGER_DISCARDED_KERNEL_COVERAGE(replanner_instance)
% Read-only check after ONE actual Flight; no modified runner or warmup.
R=struct('status','NOT_COMPLETED_DISCARDED_KERNEL_COVERAGE', ...
    'calls',0,'contact_and_clear_prepared',false, ...
    'reason','','safe_to_commit',false);
try
    S=replanner_instance.get_status();
    R.calls=double(S.isolated_f_runtime_prepare_calls);
    R.reason=char(S.isolated_f_runtime_prepare_reason);
    R.contact_and_clear_prepared= ...
        isequal(R.calls,8) && ...
        strcmp(char(S.isolated_f_runtime_prepare_status), ...
        'PASS_DISCARDED_FLIGHT_KERNEL_CONTACT_8_CLEAR_8__NOT_JIT_OR_SAFETY');
    if R.contact_and_clear_prepared
        R.status='PASS_SAME_FLIGHT_KERNEL_CONTACT_AND_CLEAR__NOT_JIT_OR_SAFETY';
    else
        R.status='NOT_COMPLETED_CONTACT_AND_CLEAR_SOFTWARE_PREPARE';
        R.reason=[char(S.isolated_f_runtime_prepare_status) ' | ' R.reason];
    end
catch ME
    R.reason=[ME.identifier ' | ' ME.message];
end
fid=fopen('TRIGGER_DISCARDED_KERNEL_COVERAGE_RESULT.txt','w');
if fid>=0
    fprintf(fid,'status=%s\ncalls=%d\ncontact_and_clear=%d\nreason=%s\n', ...
        R.status,R.calls,R.contact_and_clear_prepared,R.reason);
    fclose(fid);
end
save('TRIGGER_DISCARDED_KERNEL_COVERAGE_RESULT.mat','R');
end
