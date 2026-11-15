function out=TEST_SIX_TRIGGER_SWEEP_INCLUSION()
%TEST_SIX_TRIGGER_SWEEP_INCLUSION Reference C/B/D vs swept enclosures.
% Each C/B/D shape is checked with its ORIGINAL external radius/geometry.
checks=0;worstGap=-inf;
rng(20261112);
directions=[eye(3),-eye(3),randn(3,24)];
directions=directions./sqrt(sum(directions.^2,1));
for mode={'C','B','D'}
    [~,cfg,F]=selected_test_context(mode{1},'multiple_spheres',[0,.5,4],'swing',0);
    swept=wp_six_trigger_make_sweep(cfg,F,1,mode{1});
    original=wp_system_geometry_config(mode{1});
    for tau=linspace(0,.5,21)
        exact=six_test_forecast(tau,'swing',0);
        shape=wp_system_shape_at_pose(original,exact.pDrone,exact.pLoad);
        for ix=1:numel(shape.items)
            item=shape.items(ix);
            tag=item.tag;
            if strcmp(tag,'load'),tag='payload';end
            j=find(strcmp({swept.name},tag),1);
            assert(~isempty(j));
            for n=directions
                exactH=supportItem(item,n);
                upperH=max(n'*swept(j).vertices)+swept(j).radius;
                worstGap=max(worstGap,exactH-upperH);
                assert(exactH-upperH<=1e-9);
                checks=checks+1;
            end
        end
    end
    fprintf('PASS reference shape inclusion: %s\n',mode{1});
end
out=struct('status','PASS_THREE_MODE_SWEEP_INCLUSION', ...
    'supportInequalitiesChecked',checks,'maxExactMinusSweep',worstGap, ...
    'physicalHlcTest',false);
end
function h=supportItem(item,n)
switch item.kind
    case 'sphere',h=n'*item.center+item.radius;
    case 'capsule',h=max(n'*item.p0,n'*item.p1)+item.radius;
    case 'ellipsoid',h=n'*item.center+norm(item.axes.*(item.R'*n));
    otherwise,error('LightTrigger:Test','Unsupported reference shape');
end
end
