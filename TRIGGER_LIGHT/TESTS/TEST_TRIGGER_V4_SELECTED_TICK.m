function R=TEST_TRIGGER_V4_SELECTED_TICK()
%TEST_TRIGGER_V4_SELECTED_TICK Read-only estimated mass and cache audit.
% Provider in this test returns an ANALYTIC trajectory, not a real HLC
% forecast. Timing includes callback when supplied as a function handle.
agent.parameter=struct('mass',.762,'cableL',2,'gravity',9.81, ...
    'jx',.022,'jy',.029,'jz',.048,'loadmass',999);
agent.estimator.result.state=struct('mL',.14);
external=wp_system_geometry_config('B');
[~,E]=six_test_fixture([30;0;2],[.3;.2;.15]);
provider=@(model) analyticForecast(model);
[r,cache]=wp_six_trigger_run_selected_v4(agent,E,external,'B', ...
    'multiple_ellipsoids',provider,1,[]);
assert(strcmp(r.status,'CLEAR_CONDITIONAL'));
assert(r.geometrySelectedOnly && r.obstacleCacheRebuilt);
assert(r.modelLoadMassUsed==.14 && agent.parameter.loadmass==999);
assert(isfinite(r.selectedTriggerElapsedMs));
[r2,cache]=wp_six_trigger_run_selected_v4(agent,E,external,'B', ...
    'multiple_ellipsoids',provider,1,cache);
assert(~r2.obstacleCacheRebuilt);
[r3,~]=wp_six_trigger_run_selected_v4(agent,E,external,'B', ...
    'multiple_ellipsoids',provider,2,cache);
assert(r3.obstacleCacheRebuilt);
% Old caches require rebuilding, rather than accidentally trusting a
% missing ellipsoid-AABB provenance.
old=rmfield(cache,'formatVersion');
[r4,~]=wp_six_trigger_run_selected_v4(agent,E,external,'B', ...
    'multiple_ellipsoids',provider,1,old);
assert(r4.obstacleCacheRebuilt);
R=struct('status','PASS_V4_ONE_SELECTED_MODE_ESTIMATED_MASS_AND_CACHE', ...
    'estimatedMass',r.modelLoadMassUsed,'oldCacheRebuilt',true, ...
    'geometryOnlySynthetic',true,'physicalPredictionCertified',false);
end
function F=analyticForecast(model)
F=six_test_forecast(0:.1:4,'straight',0);
F.modelLoadMass=model.loadMass;
end
