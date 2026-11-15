function out = TEST_SIX_TRIGGER_BASIC()
%TEST_SIX_TRIGGER_BASIC Run 6 INDEPENDENT single-mode test suites.
%  One call NEVER applies more than one C/B/D x sphere/ellipsoid choice.
modes={'C','B','D'};covers={'multiple_spheres','multiple_ellipsoids'};
assertions=0;
for mi=1:3
    for ci=1:2
        mode=modes{mi};cover=covers{ci};
        [model,cfg,F]=selected_test_context(mode,cover);
        assert(strcmp(cfg.systemMode,mode));assert(strcmp(cfg.obstacleApproximation,cover));
        assert(cfg.cableLength==model.cableLength);
        assert(cfg.currentLoadMass==model.loadMass);
        [S,E]=six_test_fixture([100;60;20],[.5;.3;.2]);
        known=pickPrepared(S,E,cover);
        R=wp_six_trigger_evaluate(known,cfg,F);
        assert(strcmp(R.status,'CLEAR_CONDITIONAL') && R.narrowphasePairs==0);
        assert(strcmp(R.systemMode,mode) && strcmp(R.obstacleApproximation,cover));
        assert(~isfield(R,'six'),'Production call must NOT evaluate six modes');
        assertions=assertions+1;

        % Mid-interval cable collision; 3 sample times, but one hit between.
        [~,cfg,F]=selected_test_context(mode,cover,[0,1,4]);
        [S,E]=six_test_fixture([.5;0;2],[.19;.19;.19]);
        R=wp_six_trigger_evaluate(pickPrepared(S,E,cover),cfg,F);
        assert(strcmp(R.status,'POSSIBLE_CONTACT'));assert(R.firstInterval(1)==0);
        assertions=assertions+1;

        % Off-reference displacement must influence the actual swept body.
        [~,cfg,F]=selected_test_context(mode,cover,[0,1,4],'straight',.65);
        [S,E]=six_test_fixture([.5;.65;2],[.19;.19;.19]);
        R=wp_six_trigger_evaluate(pickPrepared(S,E,cover),cfg,F);
        assert(strcmp(R.status,'POSSIBLE_CONTACT'));
        assertions=assertions+1;

        % Floor with no obstacle parts.
        [S,E]=six_test_fixture(zeros(3,0),zeros(3,0),true);
        F.pDrone(3,:)=1.1;F.pLoad(3,:)=-.9;
        R=wp_six_trigger_evaluate(pickPrepared(S,E,cover),cfg,F);
        assert(strcmp(R.status,'POSSIBLE_CONTACT') && strcmp(R.firstObstacle,'ground'));
        assertions=assertions+1;

        % Missing upper bound must not turn into a CLEAR.
        Fbad=rmfield(F,'rhoLoad');
        R=wp_six_trigger_evaluate(known,cfg,Fbad);
        assert(strcmp(R.status,'UNKNOWN'));
        assertions=assertions+1;

        % Stale forecast must not reuse old estimated mass as truth.
        Fbad=F;Fbad.modelLoadMass=0.0556;
        R=wp_six_trigger_evaluate(known,cfg,Fbad);
        assert(strcmp(R.status,'UNKNOWN'));
        assertions=assertions+1;

        % Unsupported motion is reported as UNKNOWN.
        moving=known;moving.dynamicUnsupported=true;
        R=wp_six_trigger_evaluate(moving,cfg,F);
        assert(strcmp(R.status,'UNKNOWN'));
        assertions=assertions+1;

        % Existing external geometry is always used; do not assume radii.
        changed=wp_system_geometry_config(mode);
        changed.droneRadius=changed.droneRadius*1.18;
        altered=wp_six_trigger_config(mode,cover,changed,model);
        assert(altered.droneRadius==changed.droneRadius);
        assert(cfg.droneRadius~=altered.droneRadius);
        assertions=assertions+1;

        % The *actual external* drone and load radii must influence the
        % very first broad-phase test (not just the final narrow phase).
        [~,cfgR,FR]=selected_test_context(mode,cover);
        [SD,ED]=six_test_fixture([1.5;.52;3],[.08;.08;.08]);
        [SL,EL]=six_test_fixture([1.5;.48;1],[.08;.08;.08]);
        droneNear=pickPrepared(SD,ED,cover);
        loadNear=pickPrepared(SL,EL,cover);
        RD0=wp_six_trigger_evaluate(droneNear,cfgR,FR);
        RL0=wp_six_trigger_evaluate(loadNear,cfgR,FR);
        assert(strcmp(RD0.status,'CLEAR_CONDITIONAL'));
        assert(strcmp(RL0.status,'CLEAR_CONDITIONAL'));
        enlargedDrone=wp_system_geometry_config(mode);
        enlargedDrone.droneRadius=0.60;
        cfgDrone=wp_six_trigger_config(mode,cover,enlargedDrone,model);
        RD1=wp_six_trigger_evaluate(droneNear,cfgDrone,FR);
        assert(strcmp(RD1.status,'POSSIBLE_CONTACT'));
        enlargedLoad=wp_system_geometry_config(mode);
        enlargedLoad.loadRadius=0.55;
        cfgLoad=wp_six_trigger_config(mode,cover,enlargedLoad,model);
        RL1=wp_six_trigger_evaluate(loadNear,cfgLoad,FR);
        assert(strcmp(RL1.status,'POSSIBLE_CONTACT'));
        assertions=assertions+1;

        % Fixed nonuniform times: only ONE selected obstacle approximation.
        [S,E]=six_test_fixture([1.5;0;2],[.35;.2;.2]);
        selected=pickPrepared(S,E,cover);
        o=struct('baseStep',.4,'scanStep',.05,'maxExtraPerPart',8);
        plan=wp_six_trigger_fixed_plan(selected,cfg,@nominalCenters,o);
        plan2=wp_six_trigger_fixed_plan(selected,cfg,@nominalCenters,o);
        assert(isequal(plan.timeOffsets,plan2.timeOffsets));
        assert(all(plan.perPartSelectedCount<=8));
        assert(all(ismember(plan.baseSites,plan.timeOffsets)));
        assertions=assertions+1;

        % Different mode shapes, but NEVER compute six during one call.
        F=six_test_forecast([0,.5,4],'straight',0);
        if mode=='C',expected=cfg.cableSegments+3;else,expected=3;end
        bodies=wp_six_trigger_make_sweep(cfg,F,1,mode);
        assert(numel(bodies)==expected);
        assertions=assertions+1;
        fprintf('PASS independently: %s × %s\n',mode,cover);
    end
end
out=struct('status','PASS_6_INDEPENDENT_MODE_SUITS', ...
    'numSelectedCalls',6,'assertionGroups',assertions, ...
    'singleSelectedModePerCall',true, ...
    'actualHlcBoundDerived',false,'physicalSafetyCertified',false);
end
function known=pickPrepared(S,E,cover)
if strcmp(cover,'multiple_spheres'),known=wp_six_trigger_prepare(S,cover);
else,known=wp_six_trigger_prepare(E,cover);end
end
function [pD,pL]=nominalCenters(t)
pD=[t;0;3];pL=[t;0;1];
end
