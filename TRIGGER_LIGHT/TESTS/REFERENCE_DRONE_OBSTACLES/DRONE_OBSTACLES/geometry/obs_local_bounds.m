function [lo,hi]=obs_local_bounds(C)
lo=[Inf Inf Inf];hi=[-Inf -Inf -Inf];
for i=1:numel(C)
 [a,b]=obs_component_bounds(C(i));lo=min(lo,a);hi=max(hi,b);
end
end
