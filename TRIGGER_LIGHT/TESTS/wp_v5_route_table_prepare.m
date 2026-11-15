function C=wp_v5_route_table_prepare(generator,tauGrid)
%WP_V5_ROUTE_TABLE_PREPARE Offline/route-change route samples, NOT per tick.
% generator is the existing ORIGINAL waypoint generator struct. Grid must
% cover all future virtual times; out-of-grid predictions fail closed as
% UNQUALIFIED (never silently clamp virtual time or claim collision).
assert(isstruct(generator) && isfield(generator,'evaluate'));
tauGrid=reshape(tauGrid,1,[]);
assert(numel(tauGrid)>3 && all(isfinite(tauGrid)) && ...
    all(diff(tauGrid)>0));
p=zeros(3,numel(tauGrid));dp=p;ddp=p;
for i=1:numel(tauGrid)
    xd=generator.evaluate(tauGrid(i));
    assert(isequal(size(xd),[28 1]) && all(isfinite(xd)));
    p(:,i)=xd(1:3);dp(:,i)=xd(5:7);ddp(:,i)=xd(9:11);
end
C=struct('tauGrid',tauGrid,'p',p,'dp',dp,'ddp',ddp, ...
    'knownRouteSamplesOnly',true,'certificateOfInterpolationError',false, ...
    'source','ORIGINAL_gen_wp_straight_evaluate_on_fixed_grid');
end
