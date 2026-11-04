function C=CHECK_TRIGGER_THREAD_CAPTURE(logger,run)
% Temporary COMMON_ROOT auditor. Save CPU vs wall marks AFTER EACH REAL FLIGHT.
% Logger is read-only. Does not touch HLC, sensor, plant or trigger state.
root=fileparts(mfilename('fullpath'));
assert(isnumeric(run)&&isscalar(run)&&ismember(run,1:3), ...
    'CHECK_TRIGGER_THREAD:Run','run must be 1,2,3.');
C=struct('status','FAIL_THREAD_CAPTURE','run',run,'reason','', ...
    'count',0,'valid_count',0,'thread_switches',0, ...
    'segments_wall_ms',zeros(0,4),'segments_cpu_ms',zeros(0,4), ...
    'segments_thread_cycles',zeros(0,4),'cycle_valid_count',0, ...
    'update_wall_ms',zeros(0,1),'trigger_wall_ms',zeros(0,1), ...
    'sensor_inner_ms',zeros(0,1),'flags',struct());
try
    assert(isobject(logger)&&isprop(logger,'Data')&&isstruct(logger.Data)&& ...
        isfield(logger.Data,'agent') && ~isempty(logger.Data.agent), ...
        'CHECK_TRIGGER_THREAD:LOGGER','Original Sim LOGGER required.');
    R=logger.Data.agent(1).reference.result;
    if ~iscell(R),R=num2cell(R(:));end
    R=R(:);N=numel(R);
    assert(N>=2,'CHECK_TRIGGER_THREAD:Frames','No flight records.');
    wall=nan(N,4);cpu=nan(N,4);cycles=nan(N,4);stable=false(N,1);
    updates=nan(N,1);triggers=nan(N,1);sensor_ms=nan(N,1);
    C.flags=struct('stage',getenv('REPLANNING_STAGE_TIMING'), ...
        'thread_cpu',getenv('REPLANNING_THREAD_CPU_DIAG'), ...
        'first_trace',getenv('TRIGGER_FIRST_FLIGHT_TRACE'), ...
        'block_experiment',getenv('TRIGGER_SAMPLED_BLOCK_EXPERIMENT'));
    for k=1:N
        r=R{k};
        if ~isstruct(r)||~isscalar(r),continue;end
        if isfield(r,'replanning_time_ms') && isnumeric(r.replanning_time_ms)
            updates(k)=double(r.replanning_time_ms);
        end
        if isfield(r,'trigger_time_ms') && isnumeric(r.trigger_time_ms) && ...
                isscalar(r.trigger_time_ms)
            triggers(k)=double(r.trigger_time_ms);
        end
        if isfield(r,'simulated_obstacle_sensor') && ...
                isstruct(r.simulated_obstacle_sensor) && ...
                isscalar(r.simulated_obstacle_sensor) && ...
                isfield(r.simulated_obstacle_sensor,'elapsed_ms')
            v=r.simulated_obstacle_sensor.elapsed_ms;
            if isnumeric(v)&&isscalar(v)&&isreal(v) && isfinite(v)
                sensor_ms(k)=double(v);
            end
        end
        if ~isfield(r,'replanning_thread_cpu_wall_ms') || ...
                ~isfield(r,'replanning_thread_same_id'),continue;end
        d=double(r.replanning_thread_cpu_wall_ms);
        if ~isequal(size(d),[4 2])||any(~isfinite(d(:))),continue;end
        wall(k,:)=d(:,1).';cpu(k,:)=d(:,2).';
        stable(k)=isequal(r.replanning_thread_same_id,true);
        if isfield(r,'replanning_thread_cycles')
            v=double(r.replanning_thread_cycles(:)).';
            if isequal(size(v),[1 4]) && ...
                    all(isfinite(v)) && all(v>=0)
                cycles(k,:)=v;
            end
        end
    end
    valid=all(isfinite(wall),2)&all(isfinite(cpu),2)& ...
        all(wall>=0,2)&all(cpu>=0,2)&stable;
    C.count=N;C.valid_count=nnz(valid);
    C.thread_switches=nnz(~stable & all(isfinite(wall),2));
    C.segments_thread_cycles=cycles;
    C.cycle_valid_count=nnz(all(isfinite(cycles),2)&stable);
    C.segments_wall_ms=wall;C.segments_cpu_ms=cpu;
    C.update_wall_ms=updates;C.trigger_wall_ms=triggers;
    C.sensor_inner_ms=sensor_ms;
    if C.valid_count==N
        C.status='CAPTURED_REAL_THREAD_CPU_WALL__NOT_WCET_OR_CAUSAL_PROOF';
    else
        C.reason=sprintf('valid=%d of %d. Set REPLANNING_THREAD_CPU_DIAG=1 and build native MEX before Flight.',C.valid_count,N);
    end
catch ME
    C.reason=[ME.identifier ' | ' ME.message];
end
save(fullfile(root,sprintf('TRIGGER_THREAD_RUN_%02d.mat',run)),'C','-v7');
f=fopen(fullfile(root,sprintf('TRIGGER_THREAD_RUN_%02d.txt',run)),'w');
if f>=0
    fprintf(f,'status=%s\nrun=%d\nframes=%d\nvalid=%d\nthread_switches=%d\ncycle_valid=%d\nreason=%s\n', ...
        C.status,C.run,C.count,C.valid_count,C.thread_switches,C.cycle_valid_count,C.reason);
    fclose(f);
end
end
