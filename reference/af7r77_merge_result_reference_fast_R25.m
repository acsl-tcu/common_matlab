function r1 = af7r77_merge_result_reference_fast_R25(r1,r2)
% AF7R58ZZY-R25 Flight-reference merge fast path.
%
% SUSPENDED_LOAD_REF_ADJUST operates on agent.reference.result.state, which is
% a STATE_CLASS handle. Therefore its returned r2.state is normally the SAME
% handle already stored in r1.state. In that case merge_result() would perform
% an unnecessary deep state_copy of all dynamic properties.
%
% R25 skips only that duplicate copy. Every non-state top-level field from r2
% is assigned exactly as merge_result would. If the state handle is not shared,
% the function falls back to the original merge_result unchanged.
%
% Calling with no inputs returns diagnostic counters.

persistent STATS
if isempty(STATS)
    STATS=struct('fast_count',0,'fallback_count',0,'last_fast',false);
end

if nargin==0
    r1=STATS;
    return;
end

if isempty(r1) || isempty(r2) || ~isstruct(r1) || ~isstruct(r2)
    STATS.fallback_count=STATS.fallback_count+1;
    STATS.last_fast=false;
    r1=merge_result(r1,r2);
    return;
end

sharedState=false;
if isfield(r1,'state') && isfield(r2,'state') && ...
        isobject(r1.state) && isobject(r2.state)
    try
        sharedState=isscalar(r1.state)&&isscalar(r2.state)&&(r1.state==r2.state);
    catch
        sharedState=false;
    end
end

if ~sharedState
    STATS.fallback_count=STATS.fallback_count+1;
    STATS.last_fast=false;
    r1=merge_result(r1,r2);
    return;
end

F=fieldnames(r2);
for j=1:numel(F)
    if ~strcmp(F{j},'state')
        r1.(F{j})=r2.(F{j});
    end
end
STATS.fast_count=STATS.fast_count+1;
STATS.last_fast=true;
end
