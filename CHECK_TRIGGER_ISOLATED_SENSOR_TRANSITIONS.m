function R=CHECK_TRIGGER_ISOLATED_SENSOR_TRANSITIONS()
% Disposable common_matlab/ diagnostic; synthetic IDs are isolated only.
R=TRIGGER_ISOLATED_SENSOR_EVENT_PREPARE(struct('sample_dt_s',0.025));
save('TRIGGER_ISOLATED_SENSOR_TRANSITIONS_RESULT.mat','R');
f=fopen('TRIGGER_ISOLATED_SENSOR_TRANSITIONS_RESULT.txt','w');
if f>=0
    fprintf(f,'status=%s\ncycles=%d\nsteps=%d\nphase=%s\nreason=%s\n', ...
        R.status,R.cycles,R.steps,R.phase,R.reason);
    fclose(f);
end
end
