function report=TEST_REFERENCE_AVOIDANCE_INSTALL()
%TEST_REFERENCE_AVOIDANCE_INSTALL Smoke-check new folder and ONE-mode math.
% NOT a substitute for original Flight or V8 performance/safety tests.
info=SETUP_REFERENCE_OBSTACLE_AVOIDANCE();
methods={'C','B','D'};
parts={'multiple_spheres','multiple_ellipsoids'};
agent=struct();
agent.parameter=struct('mass',0.9,'cableL',2.0, ...
    'gravity',9.81,'jx',0.01,'jy',0.01,'jz',0.02,'loadmass',999);
agent.estimator.result.state=struct('mL',0.14);
model=wp_six_trigger_model_snapshot(agent);
assert(model.loadMass==0.14 && agent.parameter.loadmass==999);
emptyObserved=struct('parts',struct([]));
count=0;results=cell(1,6);
for i=1:numel(methods)
    g=wp_system_geometry_config(methods{i});
    pD=[0;0;3];pL=pD-[0;0;g.cableLength];
    shape=wp_system_shape_at_pose(g,pD,pL);
    assert(~isempty(shape.items));
    for j=1:numel(parts)
        cfg=wp_six_trigger_config(methods{i},parts{j},g,model);
        obs=wp_six_trigger_prepare(emptyObserved,parts{j});
        t=0:.1:4;
        F=struct('timeOffsets',t, ...
            'pDrone',repmat(pD,1,41),'pLoad',repmat(pL,1,41), ...
            'rhoDrone',zeros(1,41),'rhoLoad',zeros(1,41), ...
            'accDrone',zeros(1,40),'accLoad',zeros(1,40), ...
            'modelLoadMass',model.loadMass);
        G=wp_six_trigger_evaluate_fast_v8(obs,cfg,F);
        assert(strcmp(G.status,'CLEAR_CONDITIONAL'), ...
            'An empty observed set should be mathematically clear');
        count=count+1;results{count}=sprintf('%s + %s',methods{i},parts{j});
    end
end
report=struct('status','PASS_FOLDER_MASS_AND_EMPTY_WORLD_6_MODES', ...
    'path',info.root,'geometryModesChecked',count, ...
    'combinationNames',{results},'observedWorldFilteredBySensor',false, ...
    'testHasNoRealHLCOrFlight',true, 'physicalSafetyCertified',false);
end
