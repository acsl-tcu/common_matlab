function A = AF7R58R24_REPLANNER_ONLY_SPIKE_AUDIT(matFile)
% AF7R58R24 -- read-only replanner-only spike attribution.
% The known controller-only spike is NOT allowed to determine the ranking.
% NO production source changes, NO acceptance changes, NO time-gate bypass.
% R23 mat contains the actual 1601 Flight cycles and stage events.
if nargin < 1 || isempty(matFile)
    matFile = fullfile(fileparts(mfilename('fullpath')), ...
        'AF7R58R23_SPIKE_REPORT.mat');
end
assert(isfile(matFile), 'R24:InputMissing', ...
    'R23 MAT file not found: %s', matFile);
S = load(matFile, 'T');
assert(isfield(S,'T') && isstruct(S.T), 'R24:Schema', ...
    'R23 MAT must contain the struct T.');
T = S.T;
required = {'time_s','labels','values_ms','stage_events'};
for j = 1:numel(required)
    assert(isfield(T, required{j}), 'R24:Schema', ...
        'R23 T.%s is missing.', required{j});
end
labels = string(T.labels(:));
time_s = double(T.time_s(:));
V = double(T.values_ms);
assert(size(V,1) == numel(time_s) && size(V,2) == numel(labels), ...
    'R24:Schema', 'Incompatible time/labels/matrix dimensions.');
assert(all(isfinite(time_s)) && all(diff(time_s)>0), ...
    'R24:Time', 'Flight time must be finite and increasing.');
assert(all(isfinite(V(:))) && all(V(:)>=0), ...
    'R24:Timing', 'All durations must be finite and nonnegative.');
names = {'reference_replanner','controller','full_iteration', ...
         'online_before_plant','estimator','simulation_plant','logger'};
col = zeros(size(names));
for j = 1:numel(names)
    k = find(labels == string(names{j}));
    assert(numel(k)==1, 'R24:Schema', ...
        'Missing or duplicate channel: %s',names{j});
    col(j) = k;
end
ref = V(:,col(1)); ctrl = V(:,col(2)); full = V(:,col(3));
[~, refRank] = sort(ref, 'descend');
[~, fullRank] = sort(full, 'descend');
% The full-cycle ranking remains in the stored MAT for auditability.
% Replanner root-cause ranking uses ONLY ref, not full/controller time.
A = struct();
A.schema = 'AF7R58R24_REPLANNER_ONLY_SPIKE_AUDIT_V1';
A.source_mat = matFile;
A.cycles = numel(time_s);
A.time_s = time_s;
A.reference_ms = ref;
A.controller_ms = ctrl;
A.full_ms = full;
A.reference_rank = refRank;
A.full_rank = fullRank;
A.top_reference_time_s = time_s(refRank(1));
A.top_reference_ms = ref(refRank(1));
A.reference_p50_ms = median(ref);
A.reference_p95_ms = prctile(ref,95);
A.source_events = T.stage_events;
A.notes = { ...
    'Sorting by reference-replanner duration excludes unrelated controller spikes by construction.', ...
    'This is an offline attribution audit, not real-time safety certification.', ...
    'Parent-reference time minus Stage1/Stage2 time is an apparent non-stage remainder, not necessarily waste.'};

lines = {};
lines = add(lines, 'AF7R58R24 | REPLANNER-ONLY SPIKE AUDIT | READ-ONLY');
lines = add(lines, 'source=%s',matFile);
lines = add(lines, 'Flight cycles=%d',A.cycles);
lines = add(lines, 'No controller-based ranking, no data discarded, no commit or timing-gate changes.');
lines = add(lines, 'Reference p50/p95/max = %.6f / %.6f / %.6f ms', ...
    A.reference_p50_ms, A.reference_p95_ms, A.top_reference_ms);
lines = add(lines, '');
lines = add(lines, 'TOP REPLANNER CYCLES (sorted by reference_replanner, NOT by full_iteration)');
lines = add(lines, 't[s] ref[ms] full[ms] controller[ms] online[ms] estimator[ms] logger[ms]');
for j = 1:min(20,numel(refRank))
    k = refRank(j);
    lines = add(lines, '%.9f %.6f %.6f %.6f %.6f %.6f %.6f', ...
       time_s(k),ref(k),full(k),ctrl(k),V(k,col(4)),V(k,col(5)),V(k,col(7)));
end
lines = add(lines,'');
lines = add(lines, 'STAGE EVENTS MATCHED TO SAME SIM TIME');
if isstruct(T.stage_events) && isfield(T.stage_events,'events')
    E = T.stage_events.events;
    if ~isempty(E) && isstruct(E)
        if numel(time_s)>1, dt = median(diff(time_s)); else, dt = inf; end
        for j = 1:numel(E)
            nm = string(E(j).name);
            et = double(E(j).time_s);
            if ~isfinite(et), continue; end
            [distance,k] = min(abs(time_s-et));
            if distance>dt/4, continue; end
            if ~(contains(nm,'NATIVE_RETURNED') || contains(nm,'CERTIFIED_AND_PREPARED'))
                continue;
            end
            s = double(E(j).stage_ms);
            parent = ref(k);
            remaining = parent-s;
            lines = add(lines, ['t=%.9f event=%s reference=%.6f stage=%.6f ' ...
                'apparent_reference_minus_stage=%.6f ms'], ...
                et,char(nm),parent,s,remaining);
            if contains(nm,'STAGE1')
                A.stage1_reference_ms = parent;
                A.stage1_inner_ms = s;
                A.stage1_apparent_remainder_ms = remaining;
            elseif contains(nm,'STAGE2')
                A.stage2_reference_ms = parent;
                A.stage2_inner_ms = s;
                A.stage2_apparent_remainder_ms = remaining;
            end
        end
    end
end
lines = add(lines, '');
lines = add(lines,'CONTROLLER SPIKE IS RETAINED IN INPUT BUT NOT USED TO RANK REPLANNER COST.');
lines = add(lines,'Outer-minus-inner differences require instrumentation of pre/post Stage1/2 call sites.');
lines = add(lines,'Do not treat the subtraction as validated redundant computation.');
lines = add(lines,'R23 diagnostic has NO COMMIT; no replan-on-active evidence yet.');

outDir = fileparts(mfilename('fullpath'));
outTxt = fullfile(outDir,'AF7R58R24_REPLANNER_SPIKE_REPORT.txt');
outMat = fullfile(outDir,'AF7R58R24_REPLANNER_SPIKE_REPORT.mat');
contents = strjoin(lines,newline);
fid = fopen(outTxt,'w','n','UTF-8');
assert(fid>=0,'R24:Output','Could not open output report.');
cleanup = onCleanup(@() fclose(fid));
fprintf(fid,'%s\n',contents);
fprintf('%s\n',contents);
save(outMat,'A');
A.report_file = outTxt;
A.mat_file = outMat;
end

function lines = add(lines,fmt,varargin)
lines{end+1,1} = sprintf(fmt,varargin{:});
end
