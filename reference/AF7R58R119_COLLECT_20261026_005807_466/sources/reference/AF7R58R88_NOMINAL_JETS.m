function J=AF7R58R88_NOMINAL_JETS(fns,flightT0,t)
% Frozen nominal function sources bound to the SAME Flight origin.
assert(iscell(fns)&&numel(fns)==8 && ...
    all(cellfun(@(f)isa(f,'function_handle'),fns)), ...
    'R88:EightExactNominalDerivativesRequired');
assert(isnumeric(flightT0)&&isscalar(flightT0)&&isfinite(flightT0), ...
    'R88:InvalidFlightOrigin');
assert(isnumeric(t)&&isscalar(t)&&isfinite(t)&&t>=flightT0, ...
    'R88:InvalidHistoricalTime');
tau=double(t)-double(flightT0);
J=zeros(3,8);
for d=1:8
    v=double(fns{d}(tau));v=v(:);
    assert(numel(v)==3&&all(isfinite(v)), ...
        'R88:NonfiniteNominalDerivative%d',d-1);
    J(:,d)=v;
end
end
