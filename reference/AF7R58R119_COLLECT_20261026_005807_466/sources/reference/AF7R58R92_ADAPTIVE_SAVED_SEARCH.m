function A=AF7R58R92_ADAPTIVE_SAVED_SEARCH(agent,snapshotTime,opts)
% New source-bound multi-segment C7 search on R88 historical snapshot.
% Explicit finite CPU-work budgets DO NOT bound physical maneuver duration.
% The generator uses only snapshot-known ellipsoids; no pre-detection map.
% NO native production commit or safety declaration of HLC/tube/tail.
needed={'max_total_attempts','max_side_assignments', ...
    'max_nodes_per_check'};
assert(isstruct(opts)&&isscalar(opts),'R92:WorkOptionsRequired');
for j=1:numel(needed)
 k=needed{j};
 assert(isfield(opts,k)&&isnumeric(opts.(k))&&isscalar(opts.(k))&& ...
     isfinite(opts.(k))&&opts.(k)>=1&&fix(opts.(k))==opts.(k), ...
     'R92:MissingPositiveWorkBudget:%s',k);
end
obj=find_replanner92(agent,5);
assert(~isempty(obj),'R92:ReplannerNotFound');
ii=AF7R58R88_SELECT_TIME(obj.r88_snapshots,snapshotTime);
assert(~isempty(ii),'R92:ExactFlightSnapshotRequired');
S=obj.r88_snapshots(ii);
assert(isfinite(S.gravity_m_s2)&&S.gravity_m_s2>0&& ...
    isfinite(S.cable_length_m)&&S.cable_length_m>0, ...
    'R92:LiveNaturalTimescaleAuthorityMissing');
base=sqrt(double(S.cable_length_m)/double(S.gravity_m_s2));
A=struct('status','INCONCLUSIVE_CANDIDATE_WORK_BUDGET', ...
    'attempts',0,'candidate_found',false,'tested_durations_s',[], ...
    'tested_phase_delays_s',[], 'best',struct([]), ...
    'attempt_log',struct('duration_s',{},'delay_s',{}, ...
        'status',{},'segment_count',{},'error',{}), ...
    'reference_version',double(S.reference_version), ...
    'known_mask_frozen',logical(S.known_mask), ...
    'estimated_mass_at_event_kg',double(S.estimated_mass_kg), ...
    'robust_tube_proved',false,'nominal_tail_proved',false, ...
    'torque_closed_loop_proved',false, ...
    'production_commit_authorized',false);
level=0;
while A.attempts<opts.max_total_attempts
 T=base*2^level;
 if ~(isfinite(T)&&T>0&&isfinite(snapshotTime+T)&&snapshotTime+T>snapshotTime)
     A.status='INCONCLUSIVE_MACHINE_TIME_RANGE';return;
 end
 delayList=0;
 if level>=1
    delayList=[delayList,base*2.^(0:level-1)]; %#ok<AGROW>
 end
 for j=1:numel(delayList)
    if A.attempts>=opts.max_total_attempts,break;end
    delta=delayList(j);
    C=opts;
    C.times_s=[snapshotTime snapshotTime+T];
    C.phase_delay_s=delta;
    C.internal_waypoints='AUTO_SUPPORT';
    A.attempts=A.attempts+1;
    A.tested_durations_s(end+1)=T; %#ok<AGROW>
    A.tested_phase_delays_s(end+1)=delta; %#ok<AGROW>
    log=struct('duration_s',T,'delay_s',delta,'status','NOT_RUN', ...
        'segment_count',0,'error','');
    try
        B=AF7R58R92_RUN_SAVED_ROUTE(agent,snapshotTime,C);
        log.status=B.status;
        if isfield(B,'route')&&isstruct(B.route)&& ...
            isfield(B.route,'num_segments')
            log.segment_count=B.route.num_segments;
        end
        if isfield(B,'assessment')&&isstruct(B.assessment)&& ...
            isfield(B.assessment,'all_segment_reference_conditions_pass')&& ...
            B.assessment.all_segment_reference_conditions_pass
           A.candidate_found=true;A.best=B;
           A.status='PARTIAL_REFERENCE_CANDIDATE_FOUND__NOT_COMMITTABLE';
           A.attempt_log(end+1)=log; %#ok<AGROW>
           return;
        end
    catch ME
        log.status='FAIL_CLOSED_EVALUATION_EXCEPTION';
        log.error=[char(ME.identifier) ':' char(ME.message)];
    end
    A.attempt_log(end+1)=log; %#ok<AGROW>
 end
 level=level+1;
end
end
function obj=find_replanner92(x,depth)
obj=[];
if isa(x,'REPLANNING_SYSTEM')&&isscalar(x),obj=x;return;end
if depth<1||isempty(x)||numel(x)~=1,return;end
if isobject(x)
    try,names=properties(x);catch,return;end
elseif isstruct(x)
    names=fieldnames(x);
elseif iscell(x)
    for i=1:numel(x)
        obj=find_replanner92(x{i},depth-1);
        if ~isempty(obj),return;end
    end
    return;
else,return;end
skip={'self','result','parameter','plant','sensor','estimator', ...
    'controller','logger','time','state','parent'};
for i=1:numel(names)
    f=names{i};if any(strcmpi(f,skip)),continue;end
    try
        obj=find_replanner92(x.(f),depth-1);
    catch
        obj=[];
    end
    if ~isempty(obj),return;end
end
end
