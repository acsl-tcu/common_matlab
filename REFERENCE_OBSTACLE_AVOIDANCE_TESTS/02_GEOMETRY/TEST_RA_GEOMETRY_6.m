function R=TEST_RA_GEOMETRY_6()
% System C/B/D x observed sphere/ellipsoid = six independent checks.
methods={'C','B','D'};covers={'multiple_spheres','multiple_ellipsoids'};
agent=RA_TEST_AGENT();count=0;
for m=1:3
    g=wp_system_geometry_config(methods{m});
    % Standalone preview uses its geometry's provisional nominal cableL.
    shape=wp_system_shape_at_pose(g,[0;0;3],[0;0;3-g.cableLength]);
    if m==1,assert(numel(shape.items)==g.cableSegments+3);
    else,assert(numel(shape.items)==3);end
    for c=1:2
        cover=covers{c};
        cfg=wp_six_trigger_config(methods{m},cover,g, ...
            wp_six_trigger_model_snapshot(agent));
        known=wp_six_trigger_prepare(RA_TEST_WORLD(cover,'distant'),cover);
        cfg.useMexCChainPreclear=false;
        F=RA_TEST_FORECAST(cfg);
        G=wp_six_trigger_evaluate_fast_v8(known,cfg,F);
        assert(strcmp(G.status,'CLEAR_CONDITIONAL'), ...
            'Distant observed cover should be separated in synthetic test');
        assert(strcmp(cfg.systemMode,methods{m}));
        assert(cfg.cableLength==2 && cfg.currentLoadMass==0.14);
        count=count+1;
    end
end
assert(count==6);
R=struct('status','PASS','testedModes',count, ...
    'onlyNominalSyntheticGeometry',true,'physicalSafetyCertified',false);
end
