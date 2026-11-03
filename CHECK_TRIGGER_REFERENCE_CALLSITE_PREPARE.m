function A=CHECK_TRIGGER_REFERENCE_CALLSITE_PREPARE()
% Disposable common_matlab/ test of candidate preflight call-site exercise.
% Synthetic time-only source; no real sensor/EKF/Flight reference values.
A=struct('status','UNKNOWN_REFERENCE_CALLSITE_FIXTURE', ...
 'callsite_cycles',0,'samples',0,'reason','');
try
    fn=@(t)[t;t.*t;sin(t);cos(t);zeros(24,1)];
    audit=TRIGGER_TIME_ONLY_PURITY(fn);
    assert(audit.eligible, ...
        'CHECK_TRIGGER_CALLSITE:Audit','Synthetic arithmetic source was rejected.');
    cfg=struct('sample_dt_s',0.025,'horizon_s',4,'cable_length_m',2);
    R=TRIGGER_TIMEONLY_SOURCE_PREPARE(fn,audit,cfg);
    A.callsite_cycles=R.callsite_cycles;
    A.samples=R.callsite_samples;
    assert(strcmp(R.status, ...
        'PASS_REFERENCE_ACTUAL_SOURCE_CALLSITE_8__NO_JIT_GUARANTEE') && ...
        R.calls==8 && A.callsite_cycles==8 && A.samples>0 && ...
        ~R.used_flight_epoch&&~R.used_sensor_or_EKF&&~R.stored_reference_values && ...
        ~R.safe_to_commit, ...
        'CHECK_TRIGGER_CALLSITE:Exercise','Candidate path did not finish: %s',R.reason);
    A.status='PASS_SYNTHETIC_SOURCE_CALLSITE__NO_JIT_GUARANTEE';
catch ME
    A.reason=[ME.identifier ' | ' ME.message];
end
save('TRIGGER_SOURCE_CALLSITE_RESULT.mat','A');
f=fopen('TRIGGER_SOURCE_CALLSITE_RESULT.txt','w');
if f>=0
    fprintf(f,'status=%s\ncycles=%d\nsamples=%d\nreason=%s\n', ...
        A.status,A.callsite_cycles,A.samples,A.reason);
    fclose(f);
end
end
