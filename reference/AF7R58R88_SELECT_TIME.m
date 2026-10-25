function k=AF7R58R88_SELECT_TIME(S,t)
% One exact same-event snapshot (no last-known mass, no interpolation).
assert(isnumeric(t)&&isscalar(t)&&isfinite(t),'R88:FiniteTimeRequired');
k=[];
if isempty(S),return;end
assert(isstruct(S)&&isfield(S,'time_s'),'R88:InvalidSnapshotArray');
tt=arrayfun(@(x)double(x.time_s),S);
if any(~isfinite(tt))||numel(unique(tt))~=numel(tt)
    return;  % ambiguous/invalid event clock: fail closed
end
matches=find(abs(tt-t)<=64*eps(max(1,abs(t))));
if numel(matches)==1,k=matches;end
end
