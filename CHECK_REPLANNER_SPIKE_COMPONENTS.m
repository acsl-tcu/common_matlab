function A = CHECK_REPLANNER_SPIKE_COMPONENTS(logger,replanner_instance)
% Temporary OFFLINE diagnostic. Put in common_matlab/ (project root).
% Read-only: does not call the sensor, EKF, HLC, plant, or MEX.
% For stage breakdown, set REPLANNING_STAGE_TIMING=1 BEFORE a new Flight.
% Run: A = CHECK_REPLANNER_SPIKE_COMPONENTS(logger,replanner_instance);
assert(nargin==2 && isobject(logger) && isobject(replanner_instance) && ...
    ismethod(replanner_instance,'get_status'), ...
    'CHECK_REPLANNER_SPIKE_COMPONENTS:Input', ...
    'Pass completed-flight logger and replanner_instance.');
D=logger.Data;
assert(isstruct(D) && isfield(D,'t') && isfield(D,'agent') && ...
    ~isempty(D.agent) && isfield(D.agent(1),'reference') && ...
    isfield(D.agent(1).reference,'result'), ...
    'CHECK_REPLANNER_SPIKE_COMPONENTS:Log', ...
    'The ordinary Sim LOGGER reference.result is required.');
R=D.agent(1).reference.result;
if ~iscell(R)
    R=num2cell(R(:));
else
    R=R(:);
end
n=min([double(logger.k),numel(D.t),numel(R)]);
assert(n>=1,'CHECK_REPLANNER_SPIKE_COMPONENTS:Empty','No Flight frames.');
t=double(D.t(1:n)); t=t(:);
rp=nan(n,1); tr=nan(n,1); sn=nan(n,1);
stage=nan(n,6); detail=nan(n,6);
new_ids=cell(n,1);
for k=1:n
    x=R{k};
    rp(k)=getnum(x,'replanning_time_ms');
    tr(k)=getnum(x,'trigger_time_ms');
    sensor=getfield_or_empty(x,'simulated_obstacle_sensor');
    sn(k)=getnum(sensor,'elapsed_ms');
    memory=getfield_or_empty(x,'observed_obstacle_memory');
    new_ids{k}=getids(memory,'newly_acquired_ids');
    stage(k,:)=getvec6(x,'replanning_stage_ms');
    detail(k,:)=getvec6(x,'replanning_detail_ms');
end
residual=rp-tr-sn;
S=replanner_instance.get_status();
A=struct();
A.time_s=t;
A.replanner_ms=rp;
A.trigger_ms=tr;
A.sensor_ms=sn;
A.nontrigger_nonsensor_residual_ms=residual;
A.replanning_stage_ms=stage;
A.replanning_detail_ms=detail;
A.new_ids=new_ids;
A.stage_labels={}; A.detail_labels={};
if isfield(S,'stage_timing_labels'),A.stage_labels=S.stage_timing_labels;end
if isfield(S,'detail_timing_labels'),A.detail_labels=S.detail_timing_labels;end
A.stage_rows=nnz(all(isfinite(stage),2) & all(isfinite(detail),2));
A.n=n;
if A.stage_rows==0
    A.status='NO_STAGE_TIMING_CAPTURED__SET_ENV_BEFORE_NEW_FLIGHT';
else
    A.status='STAGE_TIMING_CAPTURED__DIAGNOSTIC_OVERHEAD_INCLUDED';
end
[~,order]=sort(residual,'descend');
order=order(isfinite(residual(order)));
A.top_residual_indices=order(1:min(15,numel(order)));
A.new_detection_indices=find(~cellfun(@isempty,new_ids));
A.first_flight_indices=find(t>=4.95 & t<=5.25);
A.startup_indices=find(t<=0.15);
A.later_spike_indices=find(t>=12.55 & t<=12.75);
root=fileparts(mfilename('fullpath'));
file=fullfile(root,'REPLANNER_STAGE_SPIKE_AUDIT.txt');
fid=fopen(file,'w');
assert(fid>0,'CHECK_REPLANNER_SPIKE_COMPONENTS:File', ...
    'Cannot write the report beside the checker in common_matlab.');
cleanup=onCleanup(@()fclose(fid)); %#ok<NASGU>
pr(fid,'=== REPLANNER COMPONENT / EVENT SPIKE AUDIT ===\n');
pr(fid,'status=%s\nframes=%d\nstage_rows=%d\n',A.status,n,A.stage_rows);
pr(fid,['IMPORTANT: residual=REPLANNER - TRIGGER - SENSOR. Sensor elapsed ', ...
        'excludes some wrapper overhead; do not label residual as memory alone.\n']);
pr(fid,['STAGE timing (if present): stage1=input, stage2=EKF read, ', ...
        'stage3=sensor+memory, stage4=trigger+obstacle log, ', ...
        'stage5=result attachment, stage6=timing bookkeeping.\n']);
pr(fid,['DETAIL timing (if present): d1=sensor observe, d2=memory update, ', ...
        'd3=prepared shape handoff, d4=trigger, d5=obstacle log, ', ...
        'd6=result attachment. Units ms.\n']);
if A.stage_rows==0
    pr(fid,['To break down the existing residual, run a NEW Flight with: ', ...
            'setenv(''REPLANNING_STAGE_TIMING'',''1''). ', ...
            'This changes timing overhead, not control calculations.\n']);
end
pr(fid,'\n--- STARTUP (DO NOT SHIFT SPIKES TO ARMING/TAKEOFF) ---\n');
printrows(fid,A,A.startup_indices);
pr(fid,'\n--- FIRST FLIGHT 4.95..5.25 s ---\n');
printrows(fid,A,A.first_flight_indices);
pr(fid,'\n--- LATER SPIKE 12.55..12.75 s ---\n');
printrows(fid,A,A.later_spike_indices);
pr(fid,'\n--- TOP 15 RESIDUALS (NON-TRIGGER + NON-SENSOR) ---\n');
printrows(fid,A,A.top_residual_indices);
pr(fid,'\n--- GENUINE NEW OBSERVATION EVENTS ---\n');
printrows(fid,A,A.new_detection_indices);
save(fullfile(root,'REPLANNER_STAGE_SPIKE_AUDIT.mat'),'A','-v7');
fprintf('[CHECK_REPLANNER_SPIKE_COMPONENTS] %s\n',A.status);
fprintf('Saved REPLANNER_STAGE_SPIKE_AUDIT.txt / .mat\n');
end

function printrows(fid,A,idx)
pr(fid,['k t[s] replanner trigger sensor residual ', ...
        'stage3 stage4 stage5 d1 d2 d3 d4 d5 d6 newIDs\n']);
for k=reshape(idx,1,[])
    s=A.replanning_stage_ms(k,:);
    d=A.replanning_detail_ms(k,:);
    pr(fid,['%d %.9f %.5f %.5f %.5f %.5f ', ...
        '%.5f %.5f %.5f %.5f %.5f %.5f %.5f %.5f %.5f %s\n'], ...
        k,A.time_s(k),A.replanner_ms(k),A.trigger_ms(k), ...
        A.sensor_ms(k),A.nontrigger_nonsensor_residual_ms(k), ...
        s(3),s(4),s(5),d(1),d(2),d(3),d(4),d(5),d(6), ...
        mat2str(A.new_ids{k}));
end
end

function v=getfield_or_empty(s,key)
v=struct();
if isstruct(s) && isscalar(s) && isfield(s,key)
    v=s.(key);
end
end

function v=getnum(s,key)
v=NaN;
if isstruct(s) && isscalar(s) && isfield(s,key)
    x=s.(key);
    if isnumeric(x) && isscalar(x) && isreal(x)
        v=double(x);
    end
end
end

function v=getvec6(s,key)
v=nan(1,6);
if isstruct(s) && isscalar(s) && isfield(s,key)
    x=s.(key);
    if isnumeric(x) && isreal(x) && numel(x)==6
        v=reshape(double(x),1,6);
    end
end
end

function ids=getids(s,key)
ids=zeros(1,0);
if isstruct(s) && isscalar(s) && isfield(s,key)
    x=s.(key);
    if isnumeric(x) && isreal(x) && isvector(x)
        ids=reshape(double(x),1,[]);
    end
end
end

function pr(fid,varargin)
fprintf(fid,varargin{:});
end
