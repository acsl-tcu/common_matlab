function R=AF7R58R83_RUN_AFTER_FLIGHT(root,tglobal,opts)
% Invoke the new read-only R81 live adapter from an existing post-Flight
% REPLANNING_SYSTEM instance, or locate one under the existing agent.reference.
% Never runs a flight, reads unseen obstacle truth or changes control.
assert(nargin==3,'R83:RequireRootGlobalTimeAndCallerWorkOptions');
obj=[];
if isa(root,'REPLANNING_SYSTEM')
    obj=root;
elseif (isobject(root)&&isscalar(root)) && isprop(root,'reference')
    obj=find_r83_replanner(root.reference,4);
else
    obj=find_r83_replanner(root,4);
end
assert(~isempty(obj), ...
    ['R83:LiveReplannerObjectNotFound. Supply the existing REPLANNING_SYSTEM ' ...
     'instance, or its containing agent / agent.reference. Do not create ' ...
     'a new replanner just for this audit.']);
R=obj.r83_live_r81_natural_return_reference_only(tglobal,opts);
end

function obj=find_r83_replanner(x,depth)
obj=[];
if isa(x,'REPLANNING_SYSTEM')&&isscalar(x)
    obj=x;return;
end
if depth<=0||isempty(x)||numel(x)~=1,return;end
if isstruct(x)
    names=fieldnames(x);
elseif isobject(x)
    try,names=properties(x);catch,return;end
elseif iscell(x)
    for j=1:min(numel(x),32)
        obj=find_r83_replanner(x{j},depth-1);
        if ~isempty(obj),return;end
    end
    return;
else
    return;
end
% Avoid agent -> self -> agent recursion, avoid live plant/controller state.
skip={'self','result','parameter','plant','sensor','estimator', ...
    'controller','logger','time','state','parent'};
for j=1:min(numel(names),96)
    name=names{j};
    if any(strcmpi(name,skip)),continue;end
    try
        if isstruct(x),v=x.(name);else,v=x.(name);end
        obj=find_r83_replanner(v,depth-1);
        if ~isempty(obj),return;end
    catch
        % Only resolving a debug handle; never influence safety state.
    end
end
end
