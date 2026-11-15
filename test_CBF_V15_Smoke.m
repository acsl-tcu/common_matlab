function test_CBF_V15_Smoke()
%TEST_CBF_V15_SMOKE Verify R2 only dynamically extends THRUST, not torque.
test_CBF_SetupPaths();s=test_CBF_Fixture();
[x,u]=CBF_V12_PureHover(s);
x23=[x;u(1);zeros(3,1)];
assert(numel(x23)==23,'R2 must be 23-dimensional.');
j=0.9;tt=[0.11;-0.18;0.08];
Z=CBF_V11_RealTaylor('R2',x23,[j;tt],s,1);
assert(norm(Z(20:23,2)-[0;0;0;j])<1e-11,'Thrust chain invalid.');
P=cbf_parameter_vector(s);
expectDw=tt./P(5:7);
assert(norm(Z(5:7,2)-expectDw)<1e-10,'Torque must act DIRECTLY.');
% An altitude ABOVE z_ground can still intersect the load sphere.
xLow=x;xLow(10)=s.ground.height+s.shape.loadRadius/2;
obs=struct('type','sphere','center',[100;0;xLow(10)],'radius',0.25);
[hh,info]=cbf_geometry_core('real','B','sphere',xLow, P, ...
   cbf_obstacle_vector(obs,'sphere'),cbf_choose_cable_spheres(s));
loadRow=find(strcmp(info.labels,'ground_load'),1);
assert(~isempty(loadRow) && hh(loadRow)<0, ...
    'z>ground is insufficient; must account for load radius and margin.');
% Pure hover should satisfy all ground rows in all three approximations.
for sh={'C','B','D'}
    [ht,meta]=cbf_geometry_core('real',sh{1},'sphere',x,P, ...
        cbf_obstacle_vector(obs,'sphere'),cbf_choose_cable_spheres(s));
    gt=strncmp(meta.labels,'ground_',7);
    assert(all(ht(gt)>0),'Pure-hover ground clearance invalid.');
end
fprintf('[PASS] V15 smoke: thrust-only R2 augmentation, tau direct, ground-radius geometry.\n');
end
