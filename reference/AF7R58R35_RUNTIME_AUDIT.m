function R = AF7R58R35_RUNTIME_AUDIT(currentFile, baselineFile)
% AF7R58R35_RUNTIME_AUDIT
% Strict same-run, exact-cycle attribution from the already saved R23 MAT.
% No LOGGER parser, no Flight disk I/O, no modification of planning/safety.
% An optional separate baseline is compared at the same simulation time.
% This routine never treats a good median/p95 as a worst-case deadline proof.
if nargin<1 || isempty(currentFile)
    error('R35:InputRequired', 'Specify the actual current-run R23 timing MAT path.');
end
A=load(currentFile,'T');
assert(isfield(A,'T'), 'Input MAT has no R23 struct T.');
T=A.T;
required={'time_s','labels','values_ms','stage_events'};
assert(all(isfield(T,required)), 'R23 timing MAT does not have the required data.');
nameList=cellstr(T.labels);
needed={'reference_replanner','online_before_plant','full_iteration','controller','logger'};
inds=zeros(1,numel(needed));
for i=1:numel(needed)
    k=find(strcmp(nameList,needed{i}));
    assert(isscalar(k),'Missing/nonunique timing component: %s', needed{i});
    inds(i)=k;
end
time=double(T.time_s(:));
V=double(T.values_ms);
assert(size(V,1)==numel(time) && all(diff(time)>0), ...
    'Timing rows/timestamps do not agree.');
ref=V(:,inds(1)); online=V(:,inds(2)); full=V(:,inds(3));
ctrl=V(:,inds(4)); logms=V(:,inds(5));
assert(all(isfinite(V(:))) && all(V(:)>=0), ...
    'Finite nonnegative timing data required.');
% Closure of the components that are counted in full_iteration.
component_names={'sensor_acquisition','sensor','estimator', ...
  'reference_replanner','controller','input_transform', ...
  'simulation_plant','logger','other_overhead'};
component_idx=zeros(1,numel(component_names));
for k=1:numel(component_names)
    j=find(strcmp(nameList,component_names{k}));
    assert(isscalar(j),'Missing cycle component: %s',component_names{k});
    component_idx(k)=j;
end
closure=full-sum(V(:,component_idx),2);
closure_max=max(abs(closure));
% Stage attribution is by event's exact recorded cycle, never by nearest
% unrelated spike or by stale persisted LOGGER fields.
E=T.stage_events.events;
rows=struct('time_s',{},'event',{},'reference_ms',{}, ...
    'stage_ms',{},'stage_exterior_ms',{},'full_ms',{},'source_mode',{}, ...
    'source_version',{});
for k=1:numel(E)
    n=char(E(k).name);
    if ~(strcmp(n,'STAGE1_NATIVE_RETURNED') || ...
         strcmp(n,'STAGE2_CERTIFIED_AND_PREPARED') || ...
         strcmp(n,'STAGE3_SIM_ONLY_ATOMIC_COMMIT_PASS'))
        continue;
    end
    t=double(E(k).time_s);
    tol=64*eps(max(1,abs(t)));
    ind=find(abs(time-t)<=tol);
    assert(isscalar(ind), ...
        'Stage event %.17g does not map to exactly one Flight cycle.',t);
    ms=double(E(k).stage_ms);
    assert(isscalar(ms) && isfinite(ms) && ms>=0, ...
        'Nonfinite or negative stage time.');
    r=struct('time_s',t,'event',n,'reference_ms',ref(ind), ...
        'stage_ms',ms,'stage_exterior_ms',ref(ind)-ms, ...
        'full_ms',full(ind), ...
        'source_mode',char(E(k).source_mode), ...
        'source_version',double(E(k).source_version));
    rows(end+1)=r; %#ok<AGROW>
end
assert(numel(rows)>0,'No Stage1/Stage2/commit events in this run.');
Events=struct2table(rows);
Cycle=table(time,ref,online,full,ctrl,logms,closure, ...
    'VariableNames',{'time_s','reference_ms','online_ms', ...
    'full_ms','controller_ms','logger_ms','closure_error_ms'});
[~,ord]=sort(ref,'descend');
Top=Cycle(ord(1:min(30,numel(ord))),:);
% Counts are ALWAYS evaluated over all cycles. Controller-outlier counts
% are reported separately rather than quietly removed from max/p95.
Statistics=struct();
Statistics.cycle_count=numel(time);
Statistics.reference_p50_ms=median(ref);
Statistics.reference_p95_ms=prctile(ref,95);
Statistics.reference_max_ms=max(ref);
Statistics.online_over25_count=nnz(online>25);
Statistics.full_over25_count=nnz(full>25);
Statistics.full_over25_except_controller_over100_count= ...
    nnz(full>25 & ctrl<=100);
Statistics.reference_over20_count=nnz(ref>20);
Statistics.component_closure_max_abs_ms=closure_max;
Statistics.stage_event_count=height(Events);
R=struct('schema','AF7R58R35_STRICT_SAME_CYCLE_V1', ...
  'input_file',char(currentFile), 'statistics',Statistics, ...
  'stage_events',Events,'top_reference_cycles',Top,'all_cycles',Cycle, ...
  'comparison_available',false,'comparison',table());
if nargin>=2 && ~isempty(baselineFile)
    B=load(baselineFile,'T');
    assert(isfield(B,'T'),'Baseline MAT has no T.');
    BT=B.T; bi=find(strcmp(cellstr(BT.labels),'reference_replanner'));
    assert(isscalar(bi),'Baseline reference column missing.');
    bt=double(BT.time_s(:));bref=double(BT.values_ms(:,bi));
    assert(isequal(size(bt),size(time)) && ...
        all(abs(bt-time)<=64*eps(max(1,abs(time)))), ...
        'Baseline and current runs use different cycle clocks.');
    R.comparison_available=true;
    R.comparison=table(time,bref,ref,ref-bref, ...
        'VariableNames',{'time_s','baseline_ms','current_ms','delta_ms'});
end
fprintf('R35 | %d cycles | reference p50 %.4f / p95 %.4f / MAX %.4f ms\n', ...
    Statistics.cycle_count,Statistics.reference_p50_ms, ...
    Statistics.reference_p95_ms,Statistics.reference_max_ms);
fprintf('online >25 ms = %d | full >25 ms = %d | cycle closure max %.7g ms\n', ...
    Statistics.online_over25_count, Statistics.full_over25_count, ...
    closure_max);
disp('========== SAME-CYCLE STAGE ATTRIBUTION ==========');
disp(Events(:,{'time_s','event','reference_ms','stage_ms', ...
    'stage_exterior_ms','full_ms'}));
disp('========== TOP REFERENCE CYCLES ==========');
disp(Top(1:min(12,height(Top)),:));
if R.comparison_available
    disp('========== R34 VS R35 SAME SIMULATION TIMES ==========');
    for t0=[13.25 13.275 13.30 14.775 14.80 14.825]
        q=find(abs(time-t0)<=64*eps(max(1,abs(t0))));
        if isscalar(q)
            fprintf('t=%8.3f  baseline=%9.4f current=%9.4f delta=%+9.4f ms\n', ...
                t0,R.comparison.baseline_ms(q), ...
                R.comparison.current_ms(q),R.comparison.delta_ms(q));
        end
    end
end
save('AF7R58R35_TIMING_RESULT.mat','R');
writetable(Events,'AF7R58R35_STAGE_EVENTS.csv');
writetable(Top,'AF7R58R35_TOP_REFERENCE.csv');
end
