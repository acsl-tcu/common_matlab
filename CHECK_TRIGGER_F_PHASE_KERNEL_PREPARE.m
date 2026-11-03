function R=CHECK_TRIGGER_F_PHASE_KERNEL_PREPARE(replanner_instance)
% Disposable test in common_matlab/, requiring ONE already-run Flight.
R=struct('status','UNKNOWN_PREPARE','calls',0,'reason','');
try
    S=replanner_instance.get_status();
    R.calls=S.isolated_f_runtime_prepare_calls;
    R.status=S.isolated_f_runtime_prepare_status;
    R.reason=S.isolated_f_runtime_prepare_reason;
catch ME
    R.reason=[ME.identifier ' | ' ME.message];
end
fid=fopen('TRIGGER_F_PHASE_KERNEL_PREPARE_RESULT.txt','w');
if fid>=0
    fprintf(fid,'status=%s\ncalls=%d\nreason=%s\n', ...
        R.status,R.calls,R.reason);fclose(fid);
end
end
