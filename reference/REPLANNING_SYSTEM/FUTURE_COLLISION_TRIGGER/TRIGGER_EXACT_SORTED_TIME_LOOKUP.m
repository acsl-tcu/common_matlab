function [hits,locations]=TRIGGER_EXACT_SORTED_TIME_LOOKUP(query,known)
% Exact monotone-match candidate for ORIGINAL reference cache.
% Uses only double EQUALITY, no ULP merging, interpolation, or time shift.
% This is deliberately separate from the Flight cache until regression and
% three-Flight timings show an improvement. For finite monotone arrays,
% MATLAB ismember(query,known) must be identical, including duplicates.
assert(isa(query,'double')&&isa(known,'double') && ...
    isreal(query)&&isreal(known)&&isrow(query)&&isrow(known) && ...
    all(isfinite(query))&&all(isfinite(known)) && ...
    all(diff(query)>=0) && all(diff(known)>=0), ...
    'TRIGGER_EXACT_SORTED_TIME_LOOKUP:Input', ...
    'Finite monotone real row-double arrays required.');
hits=false(size(query));locations=zeros(size(query));
j=1;n=numel(known);
for i=1:numel(query)
    a=query(i);
    while j<=n && known(j)<a
        j=j+1;
    end
    if j<=n && known(j)==a
        hits(i)=true;
        locations(i)=j;
    end
end
end
