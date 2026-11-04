function R=CHECK_TRIGGER_RECORDED_SOURCE_VS_WALL()
% COMMON_ROOT: evaluate saved per-tick source-work counters and wall-clock
% observations. These are NOT a root-cause or OS/JIT/GC/WCET proof.
root=fileparts(mfilename('fullpath'));
R=struct('status','FAIL_RECORDED_SOURCE_WALL_AUDIT', ...
    'reason','','peaks',zeros(0,8), ...
    'column_labels',{{'k','t','reference_wall_ms','source_loop_ms', ...
       'cache_reset','fresh','predictor_wall_ms','rejected'}}, ...
    'reference_spike_proved_cache_miss',false, ...
    'os_quantum_proved',false,'wcec_proved',false);
try
    S=load(fullfile(root,'TRIGGER_DIAG_RUN_01.mat'),'C');
    assert(isfield(S,'C'),'CHECK_TRIGGER_PROFILE:Source','Missing saved C capture.');
    C=S.C;
    wanted=[202,207,507,640,1299,1379,1430,1582,1707];
    X=nan(numel(wanted),8);
    for i=1:numel(wanted)
        k=wanted(i);
        assert(k<=numel(C.time_s), ...
            'CHECK_TRIGGER_PROFILE:Frames','Insufficient frames.');
        X(i,:)=[k,double(C.time_s(k)),double(C.reference_ms(k)), ...
            double(C.cache_source_loop_ms(k)),double(C.cache_reset(k)), ...
            double(C.cache_fresh(k)),double(C.predictor_ms(k)), ...
            double(C.rejected_steps(k))];
    end
    R.peaks=X;
    R.status='PASS_LOG_WORK_COUNTS_NOT_CAUSAL_WCET';
catch ME
    R.reason=[ME.identifier ' | ' ME.message];
end
save(fullfile(root,'TRIGGER_RECORDED_SOURCE_VS_WALL_RESULT.mat'),'R');
f=fopen(fullfile(root,'TRIGGER_RECORDED_SOURCE_VS_WALL_RESULT.txt'),'w');
if f>=0
    fprintf(f,'status=%s\nreason=%s\n',R.status,R.reason);
    fprintf(f,'columns=%s\n',strjoin(R.column_labels,','));
    for i=1:size(R.peaks,1)
        fprintf(f,'%g',R.peaks(i,1));
        fprintf(f,',%.9g',R.peaks(i,2:end));
        fprintf(f,'\n');
    end
    fclose(f);
end
end
