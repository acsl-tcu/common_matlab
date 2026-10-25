function R=AF7R58R88_RUN_AFTER_FLIGHT(agent,tglobal,opts)
% Time-bound postflight replay. Never call R87's stale live mass getter.
S=AF7R58R88_SOURCE_CHECK();
assert(S.already_installed,'R88:InstallAndRestartMATLABFirst');
obj=r88_find_replanner(agent,5);
assert(~isempty(obj),'R88:LiveReplannerInstanceNotFound');
R=obj.r88_replay_saved_version_reference_only(tglobal,opts);
assert(~R.production_committed && ~R.r81.production_commit_authorized, ...
    'R88:ReplayMayNotCommit');
end

function obj=r88_find_replanner(x,depth)
obj=[];
if isa(x,'REPLANNING_SYSTEM')&&isscalar(x),obj=x;return;end
if isempty(x)||depth<1||numel(x)~=1,return;end
if isobject(x)
    try,names=properties(x);catch,return;end
elseif isstruct(x)
    names=fieldnames(x);
elseif iscell(x)
    for k=1:numel(x)
        obj=r88_find_replanner(x{k},depth-1);
        if ~isempty(obj),return;end
    end
    return;
else,return;end
skip={'self','result','parameter','plant','sensor','estimator', ...
    'controller','logger','time','state','parent'};
for k=1:numel(names)
    f=names{k};if any(strcmpi(f,skip)),continue;end
    try
        obj=r88_find_replanner(x.(f),depth-1);
        if ~isempty(obj),return;end
    catch
    end
end
end
