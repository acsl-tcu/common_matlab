function cls = af7r57_interval_sign_classify(G)
% AF7R57_INTERVAL_SIGN_CLASSIFY
% Exact three-valued classification for an outward interval G=[LB,UB].
%
% Intended Stage-B use:
%   G = VQ - VQ_limit
%
%   UB < 0  -> entire box is safe.
%   LB > 0  -> entire box violates finite sensing.
%   otherwise -> unresolved and therefore FAIL CLOSED.
%
% No tolerance is applied.

if ~(isnumeric(G) && numel(G)==2 && all(isfinite(G(:))))
    error('AF7R57:Interval','G must be a finite two-element numeric interval.');
end
G = double(G(:)).';
if G(1) > G(2)
    error('AF7R57:IntervalOrder','Interval lower bound exceeds upper bound.');
end

if G(2) < 0
    cls = 'CERTIFIED_SAFE';
elseif G(1) > 0
    cls = 'CERTIFIED_VIOLATION';
else
    cls = 'INCONCLUSIVE_FAIL_CLOSED';
end
end
