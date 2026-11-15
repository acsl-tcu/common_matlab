function C=wp_six_trigger_route_cache_prepare(generator,tauGrid)
%WP_SIX_TRIGGER_ROUTE_CACHE_PREPARE Precompute known waypoint jets OFF tick.
% Uses original route generator, not future actual flight/plant trajectory.
% Rebuild only when ORIGINAL route/version/grid changes.
if ~isstruct(generator) || ~isfield(generator,'evaluate')
    error('LightTrigger:Route','Original generator.evaluate required');
end
tauGrid=reshape(tauGrid,1,[]);
if numel(tauGrid)<4 || any(~isfinite(tauGrid)) || any(diff(tauGrid)<=0)
    error('LightTrigger:Route','Invalid tau cache grid');
end
p=zeros(3,numel(tauGrid));dp=p;ddp=p;
for k=1:numel(tauGrid)
    xd=generator.evaluate(tauGrid(k));
    if ~isequal(size(xd),[28,1]) || any(~isfinite(xd(:)))
        error('LightTrigger:Route','Original route jet must have 28 finite entries');
    end
    p(:,k)=xd(1:3);dp(:,k)=xd(5:7);ddp(:,k)=xd(9:11);
end
C=struct('tauGrid',tauGrid,'p',p,'dp',dp,'ddp',ddp, ...
    'knownOriginalRouteOnly',true,'futureActualFlightUsed',false, ...
    'interpolationErrorCertified',false,'builtOutsideTick',true);
end
