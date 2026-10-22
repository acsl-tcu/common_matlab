function T = AF7R58R23_SPIKE_REPORT(P)
% Called once AFTER completion of the headless Flight loop.
% Never informs candidate acceptance. No file writes during Flight.
req={'t','phase','online_before_plant_s','full_iteration_s', ...
    'sensor_acquisition_s','simulation_plant_s','logger_s', ...
    'r23_sensor_s','r23_estimator_s','r23_reference_s', ...
    'r23_controller_s','r23_input_s','r23_other_s'};
assert(isstruct(P),'[R23] Runtime profile must be a struct.');
for j=1:numel(req)
    assert(isfield(P,req{j}),'[R23] Missing loop metric: %s',req{j});
end
mask=find(P.phase=='f');
assert(~isempty(mask),'[R23] Flight phase has no logged cycles.');
labels={'sensor_acquisition','sensor','estimator','reference_replanner', ...
    'controller','input_transform','simulation_plant','logger','other_overhead', ...
    'online_before_plant','full_iteration'};
fields={'sensor_acquisition_s','r23_sensor_s','r23_estimator_s', ...
    'r23_reference_s','r23_controller_s','r23_input_s', ...
    'simulation_plant_s','logger_s','r23_other_s', ...
    'online_before_plant_s','full_iteration_s'};
A=nan(numel(mask),numel(fields));
for j=1:numel(fields)
    v=double(P.(fields{j}));
    assert(numel(v)>=max(mask),'[R23] Shape mismatch for %s',fields{j});
    A(:,j)=1e3*reshape(v(mask),[],1);
end
full=A(:,11);
[~,order]=sort(full,'descend');
N=min(30,numel(order));
spikeRows=order(1:N);
D=struct();
if isappdata(0,'AF7R58R17_LAST_CAPTURE')
    D=getappdata(0,'AF7R58R17_LAST_CAPTURE');
end
T=struct('schema','AF7R58R23_FULL_LOOP_DIAGNOSTIC_V1', ...
    'time_s',double(P.t(mask(:))),'phase',P.phase(mask(:)), ...
    'labels',{labels},'values_ms',A,'top_indices',spikeRows, ...
    'stage_events',D,'profile_source','HEADLESS_FULL_LOOP');
folder=fileparts(which('AF7R58R23_SPIKE_REPORT'));
assert(~isempty(folder),'[R23] Report folder missing.');
matPath=fullfile(folder,'AF7R58R23_SPIKE_REPORT.mat');
txtPath=fullfile(folder,'AF7R58R23_SPIKE_REPORT.txt');
save(matPath,'T','P','-v7');
fid=fopen(txtPath,'w','n','UTF-8');
assert(fid>=0,'[R23] Cannot write diagnostic report.');
c=onCleanup(@()fclose(fid));
fprintf(fid,'AF7R58R23 | FULL ITERATION SPIKE ATTRIBUTION | DIAGNOSTIC ONLY\n');
fprintf(fid,'No Flight disk I/O; plan safety, control, plant and commit decisions unchanged.\n');
fprintf(fid,'Recorded Flight cycles=%d\n',numel(mask));
fprintf(fid,'\nP50 / P95 / MAX BY COMPONENT (ms)\n');
for j=1:numel(fields)
    v=A(:,j);v=v(isfinite(v));
    if isempty(v),continue;end
    vs=sort(v);n=numel(vs);
    fprintf(fid,'%-23s p50=%11.6f p95=%11.6f max=%11.6f\n',labels{j}, ...
        vs(max(1,ceil(.50*n))),vs(max(1,ceil(.95*n))),vs(end));
end
fprintf(fid,'\nTOP %d FULL-CYCLE SPIKES (all ms)\n',N);
fprintf(fid,['time_s full online acquire sensor estimator reference controller ' ...
    'input plant logger other\n']);
for j=1:N
    i=spikeRows(j);
    fprintf(fid,'%.9f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f\n', ...
        T.time_s(i),A(i,11),A(i,10),A(i,1),A(i,2),A(i,3), ...
        A(i,4),A(i,5),A(i,6),A(i,7),A(i,8),A(i,9));
end
if isstruct(D) && isfield(D,'events')
    fprintf(fid,'\nREPLANNING EVENTS FROM SAME RUN\n');
    E=D.events;
    for j=1:numel(E)
        e=E(j);
        fprintf(fid,'%s | sim_t=%.9f | stage=%.6f | in_cycle=%.6f\n', ...
            e.name,e.time_s,e.stage_ms,e.cycle_ms);
    end
    if isfield(D,'r20_profile') && isstruct(D.r20_profile)
        fprintf(fid,'\nSTAGE1 NATIVE SUBSTAGES (latest captured event, ms)\n');
        f=D.r20_profile;
        fn=fieldnames(f);
        for j=1:numel(fn)
            k=fn{j};v=f.(k);
            if isnumeric(v) && isscalar(v), fprintf(fid,'%s = %.9g\n',k,v);end
        end
    end
    if isfield(D,'r22_profile') && isstruct(D.r22_profile)
        fprintf(fid,'\nR22 NATIVE/STAGE2 SUBSTAGES (ms)\n');
        f=D.r22_profile;
        fn=fieldnames(f);
        for j=1:numel(fn)
            k=fn{j};v=f.(k);
            if isnumeric(v) && isscalar(v),fprintf(fid,'%s = %.9g\n',k,v);end
        end
    end
end
fprintf(fid,'\nLIMITS: timing probes impose some overhead; this is not a hard real-time bound.\n');
fprintf(fid,'MATLAB main loop is attributable, but native MEX internals need individual native counters.\n');
fprintf('AF7R58R23 LOOP SPIKE REPORT: %s\n',txtPath);
end
