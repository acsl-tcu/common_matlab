function A=CHECK_SENSOR_NATIVE_CLOCK(logger,replanner_instance)
% CHECK_SENSOR_NATIVE_CLOCK: post-flight native thread-cycle / wall audit.
% The temporary sensor uses QueryPerformanceCounter + QueryThreadCycleTime.
% CPU cycle-derived milliseconds are only an APPROXIMATION: frequency can change.
% All logged obstacle outputs are the normal 15-field sensor result.

if nargin~=2 || ~isobject(logger) || ~isscalar(logger) || ...
        ~isprop(logger,'Data') || ~isprop(logger,'k') || ...
        ~isobject(replanner_instance) || ~ismethod(replanner_instance,'get_status')
    error('CHECK_SENSOR_NATIVE_CLOCK:Arguments', ...
        'Use CHECK_SENSOR_NATIVE_CLOCK(logger,replanner_instance) after the Flight.');
end
n=double(logger.k);
D=logger.Data;
if ~isscalar(n) || ~isfinite(n) || n~=floor(n) || n<3 || ...
        ~isstruct(D) || ~isscalar(D) || ~isfield(D,'t') || ...
        ~isfield(D,'agent') || ~isstruct(D.agent) || ...
        ~isfield(D.agent(1),'reference') || ...
        ~isfield(D.agent(1).reference,'result') || numel(D.t)<n
    error('CHECK_SENSOR_NATIVE_CLOCK:Logger','Invalid Sim LOGGER layout.');
end
rs=D.agent(1).reference.result;
if numel(rs)<n
    error('CHECK_SENSOR_NATIVE_CLOCK:Records','Missing LOGGER reference records.');
end
status=replanner_instance.get_status();
if ~isfield(status,'sensor_status') || ...
        ~isfield(status.sensor_status,'diag_native_trace')
    error('CHECK_SENSOR_NATIVE_CLOCK:Sensor', ...
        'The active Flight sensor is not the native-clock diagnostic source.');
end
ss=status.sensor_status;
trace=double(ss.diag_native_trace);
if ss.diag_native_rows~=n || ss.diag_native_dropped~=0 || ...
        size(trace,1)~=n || size(trace,2)~=10 || ...
        ss.scan_count~=n || ss.timing_count~=n
    error('CHECK_SENSOR_NATIVE_CLOCK:Trace', ...
        'Incomplete trace: rows=%d logger=%d dropped=%d.', ...
        size(trace,1),n,ss.diag_native_dropped);
end
T=nan(n,1); W=nan(n,1); U=nan(n,1); E=false(n,1);
C=nan(n,1); Count=nan(n,1); New=nan(n,1); Left=nan(n,1);
for k=1:n
    if iscell(rs),r=rs{k};else,r=rs(k);end
    if ~isstruct(r) || ~isfield(r,'simulated_obstacle_sensor') || ...
            ~isfield(r,'replanning_time_ms')
        error('CHECK_SENSOR_NATIVE_CLOCK:Record','Missing sensor/replanner at %d.',k);
    end
    z=r.simulated_obstacle_sensor;
    T(k)=double(z.time_s);
    W(k)=double(z.elapsed_ms);
    U(k)=double(r.replanning_time_ms);
    E(k)=~isempty(z.newly_detected_ids)||~isempty(z.left_range_ids);
    C(k)=double(z.candidates_checked);
    Count(k)=double(z.detected_count);
    New(k)=numel(z.newly_detected_ids);
    Left(k)=numel(z.left_range_ids);
end
if any(~isfinite(T)) || any(~isfinite(W)) || any(~isfinite(U)) || ...
        any(abs(T(:)-reshape(double(D.t(1:n)),[],1))>1e-7) || ...
        any(trace(:,1)~=(1:n)') || any(abs(trace(:,2)-T)>1e-7) || ...
        any(abs(trace(:,3)-W)>1e-6) || ...
        any(logical(trace(:,6))~=E) || any(trace(:,7)~=C) || ...
        any(trace(:,8)~=Count) || any(trace(:,9)~=New) || ...
        any(trace(:,10)~=Left)
    error('CHECK_SENSOR_NATIVE_CLOCK:Mismatch', ...
        'Native trace and real Flight LOGGER differ.');
end
native_wall=trace(:,4);
cycles=trace(:,5);
if any(~isfinite(native_wall)|native_wall<0) || ...
        any(~isfinite(cycles)|cycles<0)
    error('CHECK_SENSOR_NATIVE_CLOCK:Clock','Invalid native clock readings.');
end
clockTest=SENSOR_THREAD_CYCLE_CLOCK('selftest');
sleepWall=double(clockTest(1)); sleepCycles=double(clockTest(2));
busyWall=double(clockTest(3)); busyCycles=double(clockTest(4));
cyclesPerMs=double(clockTest(6));
validClock=all(isfinite(clockTest)) && sleepWall>=50 && ...
    busyWall>=40 && busyCycles>1e5 && ...
    sleepCycles>=0 && sleepCycles<0.15*busyCycles && cyclesPerMs>0;
if validClock
    cpu_ms_approx=cycles/cyclesPerMs;
    clockStatus='NATIVE_THREAD_CYCLES_SELFTEST_PASS';
else
    cpu_ms_approx=nan(n,1);
    clockStatus='NATIVE_THREAD_CYCLES_SELFTEST_FAIL__NO_CPU_ATTRIBUTION';
end
ratio=nan(n,1);
if validClock
    ratio=cpu_ms_approx./max(native_wall,eps);
end
cycle_time=nan(n,1);
if isfield(D,'system_cycle_time_ms') && ...
        numel(D.system_cycle_time_ms)>=n
    cycle_time=double(D.system_cycle_time_ms(1:n));
    cycle_time=cycle_time(:);
end
M=[(1:n)',T,double(E),W,native_wall,cycles, ...
    cpu_ms_approx,ratio,U,cycle_time,C,Count,New,Left];
ev=find(E);ev=ev(ev>1);
flight=(2:n)';
[~,ix]=sort(W(flight),'descend');
worst=flight(ix(1:min(20,numel(ix))));
[~,ix]=sort(native_wall(flight),'descend');
worstNative=flight(ix(1:min(20,numel(ix))));
if validClock
    eligible=flight(native_wall(flight)>=0.3);
    [~,ix]=sort(ratio(eligible),'ascend');
    minRatio=eligible(ix(1:min(20,numel(ix))));
else
    minRatio=[];
end
A=struct();
A.status='PASS_NATIVE_CLOCK_TRACE__NOT_WCET_OR_CAUSAL_PROOF';
if ~validClock,A.status='PASS_NATIVE_WALL_TRACE__CPU_CYCLE_CALIBRATION_FAILED';end
A.cpu_clock_status=clockStatus;
A.native_selftest=[sleepWall,sleepCycles,busyWall,busyCycles,cyclesPerMs];
A.sensor_source=which('SIMULATED_OBSTACLE_SENSOR');
A.records=n;
A.events=numel(ev);
A.columns={'index','time_s','event','sensor_logged_ms','sensor_native_wall_ms', ...
    'thread_cycles','cpu_ms_equivalent_approx','cpu_over_native_wall_approx', ...
    'replanner_ms','whole_cycle_ms','candidates','detected_count','new_count','left_count'};
A.event_rows=M(ev,:);
A.top_sensor_rows=M(worst,:);
A.top_native_wall_rows=M(worstNative,:);
A.low_cpu_ratio_rows=M(minRatio,:);
A.all_rows=M;
A.sensor_median_ms=median(W(flight));
A.sensor_max_ms=max(W(flight));
A.sensor_gt_0p5=sum(W(flight)>0.5);
A.sensor_gt_1=sum(W(flight)>1);
A.replanner_max_ms=max(U(flight));
A.native_wall_median_ms=median(native_wall(flight));
A.native_wall_max_ms=max(native_wall(flight));
A.native_clock_versus_logger_median_ms=median(W(flight)-native_wall(flight));
A.warning=['Native per-thread cycles omit other worker threads; CPU cycles to ', ...
    'milliseconds use a postflight busy-spin frequency approximation and ', ...
    'can drift with frequency scaling. No causal OS/JIT/GC proof or WCET.'];
root=fileparts(mfilename('fullpath'));
if isempty(root),root=pwd;end
save(fullfile(root,'SENSOR_NATIVE_THREAD_CLOCK_AUDIT.mat'),'A');
fid=fopen(fullfile(root,'SENSOR_NATIVE_THREAD_CLOCK_AUDIT.txt'),'wt');
if fid<0,error('CHECK_SENSOR_NATIVE_CLOCK:Write','Cannot write TXT report.');end
clean=onCleanup(@()fclose(fid)); %#ok<NASGU>
fprintf(fid,'%s\nclock=%s\n',A.status,clockStatus);
fprintf(fid,'records=%d events=%d sensor median/max=%.6f %.6f ms; replanner max=%.6f ms\n', ...
    n,numel(ev),A.sensor_median_ms,A.sensor_max_ms,A.replanner_max_ms);
fprintf(fid,'sensor >0.5ms=%d >1ms=%d | native wall median/max=%.6f %.6f ms\n', ...
    A.sensor_gt_0p5,A.sensor_gt_1,A.native_wall_median_ms,A.native_wall_max_ms);
fprintf(fid,'native selftest sleep_wall/cycles=%.5f/%.0f busy_wall/cycles=%.5f/%.0f cycles_per_ms=%.3f\n', ...
    sleepWall,sleepCycles,busyWall,busyCycles,cyclesPerMs);
fprintf(fid,'columns: %s\n',strjoin(A.columns,' | '));
write_rows(fid,'EVENT',A.event_rows);
write_rows(fid,'TOP_SENSOR',A.top_sensor_rows);
write_rows(fid,'TOP_NATIVE',A.top_native_wall_rows);
if validClock,write_rows(fid,'LOW_CPU_RATIO',A.low_cpu_ratio_rows);end
fprintf(fid,'NOTE: %s\n',A.warning);
fprintf('[NATIVE CLOCK] %s | max %.4f ms | %s\n',A.status,A.sensor_max_ms,clockStatus);
end

function write_rows(fid,name,matrix)
for j=1:size(matrix,1)
    fprintf(fid,'%s',name);
    fprintf(fid,' %.8f',matrix(j,:));
    fprintf(fid,'\n');
end
end
