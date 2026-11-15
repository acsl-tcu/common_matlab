function forecast = wp_trigger_constant_velocity_enclosure(state,timeOffsets,bounds)
%WP_TRIGGER_CONSTANT_VELOCITY_ENCLOSURE Conditional kinematic reach set.
% Not a constant-input/constant-velocity plant assumption: the centerline
% p0+v0*t is ONLY the center of an acceleration-bounded reachable ball.
% True acceleration norms must obey bounds.aDrone / bounds.aLoad for ALL 4s;
% neither thrust clipping nor HLC alone establishes those constants.
% Initial position/velocity error bounds are supplied externally and nonnegative.
required={'pDrone','vDrone','pLoad','vLoad'};
for i=1:numel(required)
    if ~isfield(state,required{i}),error('Trigger:State','Missing %s',required{i});end
    x=state.(required{i});
    if ~isequal(size(x),[3,1]) || any(~isfinite(x))
        error('Trigger:State','%s must be finite 3x1',required{i});
    end
end
required={'aDrone','aLoad','posErrorDrone','posErrorLoad', ...
    'velErrorDrone','velErrorLoad'};
for i=1:numel(required)
    if ~isfield(bounds,required{i}),error('Trigger:Bounds','Missing %s',required{i});end
    x=bounds.(required{i});
    if ~isscalar(x) || ~isfinite(x) || x<0
        error('Trigger:Bounds','%s must be finite nonnegative scalar',required{i});
    end
end
t=reshape(timeOffsets,1,[]);
if numel(t)<2 || abs(t(1))>1e-12 || abs(t(end)-4)>1e-12 || ...
        any(~isfinite(t)) || any(diff(t)<=0)
    error('Trigger:Time','Offsets must strictly span [0,4] s');
end
forecast.timeOffsets=t;
forecast.pDrone=bsxfun(@plus,state.pDrone,state.vDrone*t);
forecast.pLoad=bsxfun(@plus,state.pLoad,state.vLoad*t);
forecast.rhoDrone=bounds.posErrorDrone+bounds.velErrorDrone*t+0.5*bounds.aDrone*t.^2;
forecast.rhoLoad=bounds.posErrorLoad+bounds.velErrorLoad*t+0.5*bounds.aLoad*t.^2;
forecast.accDrone=bounds.aDrone*ones(1,numel(t)-1);
forecast.accLoad=bounds.aLoad*ones(1,numel(t)-1);
% This states WHERE numerical bounds came from, not a formal proof thereof.
forecast.boundSource='external_true_acceleration_and_estimation_error_bounds';
forecast.hlcIncluded=false;
end
