function R=CHECK_TRIGGER_THREAD_CPU_CLOCK_BUILD()
% COMMON ROOT, temporary diagnostic MEX. Compile only when explicitly run.
% Do not use this for a flight safety/WCET proof.
root=fileparts(mfilename('fullpath'));
R=struct('status','FAIL_THREAD_CPU_CLOCK_BUILD','reason','', ...
    'timestamp_monotonic',false,'thread_cpu_nondecreasing',false, ...
    'thread_cycles_nondecreasing',false,'cpu_ms_too_coarse_for_short_stages',true, ...
    'native_rebuild_required',true);
try
    assert(ispc,'TRIGGER_THREAD_CLOCK:WindowsOnly','This is a Win32 thread CPU clock.');
    src=fullfile(root,'TRIGGER_THREAD_CPU_WALL_NATIVE.cpp');
    assert(exist(src,'file')==2,'TRIGGER_THREAD_CLOCK:SourceMissing');
    clear('TRIGGER_THREAD_CPU_WALL_NATIVE');
    mex('-R2018a','-outdir',root,'-output','TRIGGER_THREAD_CPU_WALL_NATIVE',src);
    rehash;
    assert(exist('TRIGGER_THREAD_CPU_WALL_NATIVE','file')==3, ...
        'TRIGGER_THREAD_CLOCK:MexMissing','Compiled native clock is not on MATLAB path.');
    x=TRIGGER_THREAD_CPU_WALL_NATIVE();
    y=TRIGGER_THREAD_CPU_WALL_NATIVE();
    assert(isequal(size(x),[1 4]) && all(isfinite(x)) && ...
        isequal(size(y),[1 4]) && all(isfinite(y)) && ...
        x(3)==y(3),'TRIGGER_THREAD_CLOCK:InvalidNativeResult');
    R.timestamp_monotonic=(y(1)>=x(1));
    R.thread_cpu_nondecreasing=(y(2)>=x(2));
    R.thread_cycles_nondecreasing=(y(4)>=x(4));
    assert(R.timestamp_monotonic && R.thread_cpu_nondecreasing && ...
        R.thread_cycles_nondecreasing, ...
        'TRIGGER_THREAD_CLOCK:Monotonicity');
    R.status='PASS_WIN32_THREAD_CYCLES_AVAILABLE__NO_CPU_TIME_WCET';
catch ME
    R.reason=[ME.identifier ' | ' ME.message];
end
save(fullfile(root,'TRIGGER_THREAD_CPU_CLOCK_BUILD_RESULT.mat'),'R');
f=fopen(fullfile(root,'TRIGGER_THREAD_CPU_CLOCK_BUILD_RESULT.txt'),'w');
if f>=0
    fprintf(f,'status=%s\nreason=%s\n',R.status,R.reason);fclose(f);
end
end
