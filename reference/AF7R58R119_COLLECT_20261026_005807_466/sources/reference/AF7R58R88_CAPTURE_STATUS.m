function R=AF7R58R88_CAPTURE_STATUS(agent)
% Read only; do not read post-landing estimator as time-t EKF authority.
obj=r88_find_replanner_status(agent,5);
assert(~isempty(obj),'R88:LiveReplannerInstanceNotFound');
S=obj.r88_snapshots;
R=struct('status','R88_SNAPSHOT_LEDGER_READ_ONLY', ...
    'count',numel(S),'times_s',[], ...
    'reference_versions',[], 'mass_kg',[], ...
    'capture_failure_count',obj.r88_capture_failure_count, ...
    'capture_last_reason',obj.r88_capture_last_reason, ...
    'production_modified',false);
if ~isempty(S)
    R.times_s=[S.time_s];
    R.reference_versions=[S.reference_version];
    R.mass_kg=[S.estimated_mass_kg];
end
end
function obj=r88_find_replanner_status(x,depth)
obj=[];
if isa(x,'REPLANNING_SYSTEM')&&isscalar(x),obj=x;return;end
if isempty(x)||depth<1||numel(x)~=1,return;end
if isobject(x)
    try,names=properties(x);catch,return;end
elseif isstruct(x)
    names=fieldnames(x);
elseif iscell(x)
    for k=1:numel(x)
        obj=r88_find_replanner_status(x{k},depth-1);
        if ~isempty(obj),return;end
    end
    return;
else,return;end
skip={'self','result','parameter','plant','sensor','estimator', ...
    'controller','logger','time','state','parent'};
for k=1:numel(names)
    f=names{k};if any(strcmpi(f,skip)),continue;end
    try
        obj=r88_find_replanner_status(x.(f),depth-1);
        if ~isempty(obj),return;end
    catch
    end
end
end
