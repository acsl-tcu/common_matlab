function R=COMPARE_TRIGGER_SPIKE_SESSIONS(file_a,file_b)
% COMPARE_TRIGGER_SPIKE_SESSIONS
% Compare two saved CHECK_TRIGGER_SENSOR_SPIKE_AUDIT MAT results offline.
% No simulation, sensor, agent, HLC, MEX, or run script is touched.
% Run in MATLAB: R=COMPARE_TRIGGER_SPIKE_SESSIONS('runA.mat','runB.mat');
if nargin~=2
    error('COMPARE_TRIGGER_SPIKE_SESSIONS:Inputs', ...
        'Supply the two saved TRIGGER_SENSOR_SPIKE_AUDIT MAT paths.');
end
A=read_audit(file_a);
B=read_audit(file_b);
R=struct();
R.status='OFFLINE_TWO_SESSION_TIMING_COMPARISON__NOT_SAFETY_PROOF';
R.file_a=char(file_a);
R.file_b=char(file_b);
R.run_a_status=char(A.status);
R.run_b_status=char(B.status);
R.trigger_predictions_a=double(A.timings.predictor.count);
R.trigger_predictions_b=double(B.timings.predictor.count);
names={'cycle','replanner','sensor','trigger','predictor','geometry'};
R.timings=struct();
for j=1:numel(names)
    f=names{j};
    a=A.timings.(f); b=B.timings.(f);
    q=struct('a_n',double(a.count),'b_n',double(b.count), ...
        'a_median_ms',double(a.median_ms),'b_median_ms',double(b.median_ms), ...
        'a_p95_ms',double(a.p95_ms),'b_p95_ms',double(b.p95_ms), ...
        'a_max_ms',double(a.max_ms),'b_max_ms',double(b.max_ms), ...
        'delta_median_ms',double(b.median_ms-a.median_ms), ...
        'delta_p95_ms',double(b.p95_ms-a.p95_ms), ...
        'delta_max_ms',double(b.max_ms-a.max_ms));
    R.timings.(f)=q;
end
R.windows_a=A.windows;
R.windows_b=B.windows;
R.top_trigger_spikes_a=A.top_trigger_spikes;
R.top_trigger_spikes_b=B.top_trigger_spikes;
R.memory_audit_a=memory_status(A);
R.memory_audit_b=memory_status(B);
root=fileparts(mfilename('fullpath'));
mat_path=fullfile(root,'TRIGGER_SESSION_COMPARISON.mat');
txt_path=fullfile(root,'TRIGGER_SESSION_COMPARISON.txt');
save(mat_path,'R','-v7');
fid=fopen(txt_path,'wt');
if fid>=0
    cleaner=onCleanup(@()fclose(fid)); %#ok<NASGU>
    fprintf(fid,'STATUS=%s\nRUN_A=%s\nRUN_B=%s\n',R.status,R.file_a,R.file_b);
    fprintf(fid,'AUDIT_A=%s\nAUDIT_B=%s\n',R.run_a_status,R.run_b_status);
    fprintf(fid,'MEMORY_A=%s\nMEMORY_B=%s\n',R.memory_audit_a,R.memory_audit_b);
    fprintf(fid,'TIMING name median_a median_b delta p95_a p95_b delta max_a max_b delta [ms]\n');
    for j=1:numel(names)
        q=R.timings.(names{j});
        fprintf(fid,'%s %.6f %.6f %+.6f %.6f %.6f %+.6f %.6f %.6f %+.6f\n', ...
            names{j},q.a_median_ms,q.b_median_ms,q.delta_median_ms, ...
            q.a_p95_ms,q.b_p95_ms,q.delta_p95_ms, ...
            q.a_max_ms,q.b_max_ms,q.delta_max_ms);
    end
    fprintf(fid,'WINDOWS [start end] a_trigger_max b_trigger_max a_replanner_max b_replanner_max\n');
    for j=1:min(numel(A.windows),numel(B.windows))
        x=A.windows(j); y=B.windows(j);
        fprintf(fid,'%s %.6f %.6f %.6f %.6f\n',mat2str(x.interval_s), ...
            x.trigger_max_ms,y.trigger_max_ms,x.replanner_max_ms,y.replanner_max_ms);
    end
end
fprintf('[TRIGGER COMPARE] median %.4f vs %.4f ms, P95 %.4f vs %.4f ms, peak %.4f vs %.4f ms\n', ...
   R.timings.trigger.a_median_ms,R.timings.trigger.b_median_ms, ...
   R.timings.trigger.a_p95_ms,R.timings.trigger.b_p95_ms, ...
   R.timings.trigger.a_max_ms,R.timings.trigger.b_max_ms);
end

function A=read_audit(file)
assert(ischar(file)||isstring(file), 'COMPARE_TRIGGER_SPIKE_SESSIONS:Path', ...
    'MAT paths must be text.');
L=load(char(file),'A');
assert(isfield(L,'A')&&isstruct(L.A)&&isscalar(L.A) ...
    &&isfield(L.A,'timings')&&isfield(L.A,'windows') ...
    &&isfield(L.A,'top_trigger_spikes'), ...
    'COMPARE_TRIGGER_SPIKE_SESSIONS:Schema','File is not a trigger sensor spike audit.');
A=L.A;
end

function s=memory_status(A)
if isfield(A,'memory_log_status')
    s=char(A.memory_log_status);
elseif isfield(A,'memory_records')&&A.memory_records==0
    s='OLD_AUDIT_UNRESOLVED__EARLY_ID_CHECK_FAILED';
else
    s='OLD_AUDIT_UNRESOLVED';
end
end
