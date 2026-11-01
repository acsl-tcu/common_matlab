function A=CHECK_SENSOR_FLIGHT_CPU_WALL(logger,replanner_instance)
% CHECK_SENSOR_FLIGHT_CPU_WALL  Temporary post-flight CPU vs wall spike audit.
% Run once after normal Flight (the Flight sensor must be instrumented):
%   A=CHECK_SENSOR_FLIGHT_CPU_WALL(logger,replanner_instance);
% Output: SENSOR_FLIGHT_CPU_WALL_AUDIT.mat/.txt beside this script.
% This is diagnostic evidence only; no hard deadline or OS attribution proof.
%
% The native Sim LOGGER uses logger.Data and logger.k (not logger.get()).
% The real sensor provides its 15-field result unchanged; monitor rows
% are stored separately and retrieved through REPLANNING_SYSTEM.get_status().

if nargin~=2 || ~isobject(logger) || ~isscalar(logger) || ...
        ~isprop(logger,'Data') || ~isprop(logger,'k') || ...
        ~isobject(replanner_instance) || ...
        ~ismethod(replanner_instance,'get_status')
    error('CHECK_SENSOR_FLIGHT_CPU_WALL:Arguments', ...
        'Call CHECK_SENSOR_FLIGHT_CPU_WALL(logger,replanner_instance) after Flight.');
end
n=double(logger.k);
D=logger.Data;
if ~isstruct(D) || ~isscalar(D) || ~isfield(D,'t') || ...
        ~isfield(D,'agent') || ~isstruct(D.agent) || ...
        numel(D.t)<n || n<2 || n~=floor(n) || ...
        ~isfield(D.agent(1),'reference') || ...
        ~isfield(D.agent(1).reference,'result')
    error('CHECK_SENSOR_FLIGHT_CPU_WALL:Logger', ...
        'LOGGER.Data.t/agent(1).reference.result/LOGGER.k are invalid.');
end
recs=D.agent(1).reference.result;
if numel(recs)<n
    error('CHECK_SENSOR_FLIGHT_CPU_WALL:Records', ...
        'Too few reference records in normal Sim LOGGER.');
end
st=replanner_instance.get_status();
if ~isstruct(st) || ~isfield(st,'sensor_status') || ...
        ~isfield(st.sensor_status,'diag_wall_cpu_trace')
    error('CHECK_SENSOR_FLIGHT_CPU_WALL:Source', ...
        'The Flight sensor is not the CPU-wall diagnostic version.');
end
ss=st.sensor_status;
S=double(ss.diag_wall_cpu_trace);
if ~isnumeric(S) || size(S,2)~=10 || ...
        ss.diag_trace_dropped>0 || size(S,1)~=n || ...
        ss.scan_count~=n || ss.timing_count~=n
    error('CHECK_SENSOR_FLIGHT_CPU_WALL:Trace', ...
        'Trace incomplete/over capacity: rows=%d logger=%d dropped=%d.', ...
        size(S,1),n,ss.diag_trace_dropped);
end
T=nan(n,1); W=nan(n,1); U=nan(n,1); V=nan(n,1); E=false(n,1);
C=nan(n,1); NX=nan(n,1); LX=nan(n,1);
for k=1:n
    if iscell(recs),r=recs{k};else,r=recs(k);end
    if ~isstruct(r) || ...
            ~isfield(r,'simulated_obstacle_sensor') || ...
            ~isfield(r,'replanning_time_ms')
        error('CHECK_SENSOR_FLIGHT_CPU_WALL:MissingRecord', ...
            'Missing sensor/replanner record at index %d.',k);
    end
    s=r.simulated_obstacle_sensor;
    T(k)=double(s.time_s);
    W(k)=double(s.elapsed_ms);
    U(k)=double(r.replanning_time_ms);
    E(k)=~isempty(s.newly_detected_ids)||~isempty(s.left_range_ids);
    C(k)=double(s.candidates_checked);
    NX(k)=numel(s.newly_detected_ids);
    LX(k)=numel(s.left_range_ids);
end
if any(~isfinite(T)) || any(~isfinite(W)) || any(~isfinite(U)) || ...
        any(W<0) || any(U<0)
    error('CHECK_SENSOR_FLIGHT_CPU_WALL:InvalidTiming','Invalid LOGGER timing.');
end
if any(abs(T-reshape(double(D.t(1:n)),[],1))>1e-7)
    error('CHECK_SENSOR_FLIGHT_CPU_WALL:Time','LOGGER time mismatch.');
end
if any(S(:,1)~=(1:n)') || ...
        any(abs(S(:,2)-T)>1e-7) || ...
        any(abs(S(:,3)-W)>1e-6) || ...
        any(logical(S(:,5))~=E) || ...
        any(S(:,6)<0) || any(S(:,7)~=C) || ...
        any(S(:,8)~=NX) || any(S(:,9)~=LX)
    error('CHECK_SENSOR_FLIGHT_CPU_WALL:Mismatch', ...
        'CPU-wall trace does not match the original Flight LOGGER.');
end
cpu=S(:,4);
validCpu=isfinite(cpu)&cpu>=0;
if strcmp(ss.diag_cpu_status,'JAVA_THREAD_CPU_ENABLED') && ...
        ~all(validCpu)
    error('CHECK_SENSOR_FLIGHT_CPU_WALL:CPUClock', ...
        'CPU clock reported enabled but some readings are invalid.');
end
if any(validCpu) && any(~validCpu)
    clock_status='PARTIAL_CPU_DATA';
elseif all(validCpu)
    clock_status='CPU_AVAILABLE';
else
    clock_status='CPU_UNAVAILABLE';
end
cycle=nan(n,1);
if isfield(D,'system_cycle_time_ms') && numel(D.system_cycle_time_ms)>=n
    cycle=reshape(double(D.system_cycle_time_ms(1:n)),[],1);
end
% A process' Java main-thread CPU time can differ from total MATLAB CPU
% because native/worker threads are not necessarily included.
gap=nan(n,1);
gap(validCpu)=W(validCpu)-cpu(validCpu);
valid=2:n; % Record 1 = arming/preflight.
[~,ordWall]=sort(W(valid),'descend');
gapSort=gap(valid); gapSort(~isfinite(gapSort))=-Inf;
[~,ordGap]=sort(gapSort,'descend');
cycleSort=cycle(valid); cycleSort(~isfinite(cycleSort))=-Inf;
[~,ordCycle]=sort(cycleSort,'descend');
wallTop=valid(ordWall(1:min(20,numel(ordWall))));
gapTop=valid(ordGap(1:min(20,numel(ordGap))));
cycleTop=valid(ordCycle(1:min(20,numel(ordCycle))));
ev=find(E);ev=ev(ev>1);
A=struct();
A.status='PASS_WALL_TRACE__CPU_UNAVAILABLE';
if strcmp(clock_status,'CPU_AVAILABLE')
    A.status='PASS_WALL_AND_THREAD_CPU_TRACE__NOT_OS_PROOF';
elseif strcmp(clock_status,'PARTIAL_CPU_DATA')
    A.status='PASS_WALL_TRACE__PARTIAL_CPU_CLOCK';
end
A.cpu_clock_status=clock_status;
A.thread_cpu_source=ss.diag_cpu_status;
A.records=n;
A.events=numel(ev);
A.event_indices=ev;
A.event_times_s=T(ev);
A.event_columns={'index','time_s','event','sensor_wall_ms', ...
    'thread_cpu_ms','wall_minus_thread_cpu_ms','replanner_wall_ms', ...
    'whole_cycle_ms','candidates','detected_count','new_count','left_count'};
M=[(1:n)',T,double(E),W,cpu,gap,U,cycle,C,S(:,6),NX,LX];
A.event_rows=M(ev,:);
A.top_sensor_rows=M(wallTop,:);
A.top_wall_cpu_gap_rows=M(gapTop,:);
A.top_cycle_rows=M(cycleTop,:);
A.trace=M;
A.median_sensor_wall_ms=median(W(valid));
A.p95_sensor_wall_ms=prctile(W(valid),95);
A.max_sensor_wall_ms=max(W(valid));
A.max_replanner_wall_ms=max(U(valid));
if any(isfinite(cycle(valid)))
    A.max_whole_cycle_ms=max(cycle(valid));
else
    A.max_whole_cycle_ms=NaN;
end
A.cpu_median_ms=NaN;
A.cpu_max_ms=NaN;
if any(validCpu(valid))
    measured_cpu=cpu(valid);
    measured_cpu=measured_cpu(isfinite(measured_cpu) & measured_cpu>=0);
    A.cpu_median_ms=median(measured_cpu);
    A.cpu_max_ms=max(measured_cpu);
end
A.wall_gt_0p5_count=sum(W(valid)>0.5);
A.wall_gt_1_count=sum(W(valid)>1);
A.cpu_gt_0p5_count=sum(cpu(valid)>0.5);
A.cpu_gt_1_count=sum(cpu(valid)>1);
% Validate that Java ThreadMXBean advances during MATLAB's own work.
% This runs ONLY AFTER Flight; it never moves work to arming/takeoff.
A.cpu_selftest_busy_wall_ms=NaN;
A.cpu_selftest_busy_thread_ms=NaN;
A.cpu_selftest_pause_wall_ms=NaN;
A.cpu_selftest_pause_thread_ms=NaN;
A.cpu_selftest_status='NOT_RUN_CPU_UNAVAILABLE';
if any(validCpu)
    try
        bean=javaMethod('getThreadMXBean', ...
            'java.lang.management.ManagementFactory');
        c0=double(bean.getCurrentThreadCpuTime());
        q=tic;
        accumulator=0;
        while toc(q)<0.060
            accumulator=accumulator+sqrt(double(accumulator+1)); %#ok<NASGU>
        end
        c1=double(bean.getCurrentThreadCpuTime());
        A.cpu_selftest_busy_wall_ms=1000*toc(q);
        A.cpu_selftest_busy_thread_ms=(c1-c0)/1e6;
        c0=double(bean.getCurrentThreadCpuTime());
        q=tic;
        pause(0.060);
        c1=double(bean.getCurrentThreadCpuTime());
        A.cpu_selftest_pause_wall_ms=1000*toc(q);
        A.cpu_selftest_pause_thread_ms=(c1-c0)/1e6;
        if A.cpu_selftest_busy_thread_ms>5 && ...
                A.cpu_selftest_busy_thread_ms> ...
                A.cpu_selftest_pause_thread_ms+2
            A.cpu_selftest_status='THREAD_CPU_CLOCK_REACTS_TO_MATLAB_WORK';
        else
            A.cpu_selftest_status='THREAD_CPU_CLOCK_NOT_VALIDATED';
            if strcmp(A.status, ...
                    'PASS_WALL_AND_THREAD_CPU_TRACE__NOT_OS_PROOF')
                A.status='PASS_WALL_TRACE__CPU_CLOCK_NOT_VALIDATED';
            end
        end
    catch
        A.cpu_selftest_status='THREAD_CPU_SELFTEST_EXCEPTION';
        if strcmp(A.status,'PASS_WALL_AND_THREAD_CPU_TRACE__NOT_OS_PROOF')
            A.status='PASS_WALL_TRACE__CPU_CLOCK_NOT_VALIDATED';
        end
    end
end
A.cautions=['Thread CPU != full MATLAB process CPU when native/worker ', ...
    'threads run; negative wall-minus-CPU is possible due to ', ...
    'different measurement boundaries and resolution. Java calls ', ...
    'and extra trace writes perturb timing; no OS/JIT/GC proof or WCET bound.'];
root=fileparts(mfilename('fullpath'));
if isempty(root),root=pwd;end
save(fullfile(root,'SENSOR_FLIGHT_CPU_WALL_AUDIT.mat'),'A');
fid=fopen(fullfile(root,'SENSOR_FLIGHT_CPU_WALL_AUDIT.txt'),'wt');
if fid<0,error('CHECK_SENSOR_FLIGHT_CPU_WALL:Write','Cannot write TXT.');end
cleanup=onCleanup(@()fclose(fid)); %#ok<NASGU>
fprintf(fid,'%s\nCPU clock: %s (%s)\n',A.status,clock_status,ss.diag_cpu_status);
fprintf(fid,'records=%d events=%d sensor median/p95/max=%.6f %.6f %.6f ms | replanner max=%.6f ms | cycle max=%.6f ms\n', ...
    n,numel(ev),A.median_sensor_wall_ms,A.p95_sensor_wall_ms, ...
    A.max_sensor_wall_ms,A.max_replanner_wall_ms,A.max_whole_cycle_ms);
fprintf(fid,'sensor wall >0.5 / >1ms: %d / %d ; threadCPU >0.5 / >1ms: %d / %d\n', ...
    A.wall_gt_0p5_count,A.wall_gt_1_count,A.cpu_gt_0p5_count,A.cpu_gt_1_count);
fprintf(fid,'COLS: %s\n',strjoin(A.event_columns,' | '));
write_rows(fid,'EVENT',A.event_rows);
write_rows(fid,'TOP_SENSOR',A.top_sensor_rows);
if any(validCpu)
    write_rows(fid,'TOP_WALL_CPU_GAP',A.top_wall_cpu_gap_rows);
else
    fprintf(fid,'TOP_WALL_CPU_GAP: unavailable because CPU data are missing.\n');
end
if any(isfinite(cycle))
    write_rows(fid,'TOP_CYCLE',A.top_cycle_rows);
end
fprintf(fid,'POSTFLIGHT_CPU_SELFTEST: %s busy_wall_ms=%.5f busy_cpu_ms=%.5f pause_wall_ms=%.5f pause_cpu_ms=%.5f\n', ...
    A.cpu_selftest_status,A.cpu_selftest_busy_wall_ms, ...
    A.cpu_selftest_busy_thread_ms,A.cpu_selftest_pause_wall_ms, ...
    A.cpu_selftest_pause_thread_ms);
fprintf(fid,'NOTE: %s\n',A.cautions);
fprintf('[CPU-WALL] %s | Flight sensor max %.4f ms | CPU clock %s\n', ...
    A.status,A.max_sensor_wall_ms,clock_status);
end

function write_rows(fid,label,M)
for j=1:size(M,1)
    fprintf(fid,'%s',label);
    fprintf(fid,' %.8f',M(j,:));
    fprintf(fid,'\n');
end
end
