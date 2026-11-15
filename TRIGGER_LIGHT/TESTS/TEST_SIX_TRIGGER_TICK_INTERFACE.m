function out=TEST_SIX_TRIGGER_TICK_INTERFACE()
%TEST_SIX_TRIGGER_TICK_INTERFACE Test read-only model/mass and cache contract.
agent.parameter=struct('mass',0.762,'cableL',2,'gravity',9.81, ...
    'jx',0.022,'jy',0.029,'jz',0.048,'loadmass',999);
agent.estimator.result.state=struct('mL',0.14);
external=wp_system_geometry_config('B');
[S,~]=six_test_fixture([30;0;2],[.3;.2;.15]);
provider=@(model) buildForecast(model);
[R,cache]=wp_six_trigger_run_selected(agent,S,external,'B', ...
    'multiple_spheres',provider,1,[]);
assert(strcmp(R.status,'CLEAR_CONDITIONAL'));
assert(R.geometrySelectedOnly && R.obstacleCacheRebuilt);
assert(isfinite(R.selectedTriggerElapsedMs));
assert(cache.version==1);
assert(agent.parameter.loadmass==999);
assert(R.modelLoadMassUsed==0.14);
[R2,cache]=wp_six_trigger_run_selected(agent,S,external,'B', ...
    'multiple_spheres',provider,1,cache);
assert(~R2.obstacleCacheRebuilt);
[R3,~]=wp_six_trigger_run_selected(agent,S,external,'B', ...
    'multiple_spheres',provider,2,cache);
assert(R3.obstacleCacheRebuilt);

% No mass fallback: the current estimator mL is always authoritative.
agent.estimator.result.state=rmfield(agent.estimator.result.state,'mL');
failed=false;
try
    wp_six_trigger_model_snapshot(agent);
catch err
    failed=strcmp(err.identifier,'LightTrigger:Model');
end
assert(failed,'Missing current EKF mL MUST fail instead of using parameter.loadmass');

out=struct('status','PASS_SELECTED_TICK_MODEL_AND_CACHE', ...
    'cacheReused',true,'currentEstimatedMassUsed',true, ...
    'parametersNotWritten',true,'providerTimeIncluded',true, ...
    'actualHlcPredictionProven',false);
fprintf('PASS one-mode tick, cache reuse, mandatory current mL, no parameter write\n');
end
function F=buildForecast(model)
F=six_test_forecast(linspace(0,4,41),'straight',0);
F.modelLoadMass=model.loadMass;
end
