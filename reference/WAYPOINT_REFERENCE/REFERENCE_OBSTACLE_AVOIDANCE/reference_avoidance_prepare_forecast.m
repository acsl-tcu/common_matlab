function P=reference_avoidance_prepare_forecast( ...
    gains,pastRelativeTimes,filterTime,velocityWeight,routeGenerator,tauGrid)
%REFERENCE_AVOIDANCE_PREPARE_FORECAST Build reusable caches OFF 25ms tick.
% gains are the ORIGINAL HLC gains; routeGenerator is the ORIGINAL route.
% velocityWeight is supplied by caller, not guessed from physics.
P=struct();
P.modalCache=wp_six_trigger_pv_cache_prepare( ...
    gains,pastRelativeTimes,filterTime,velocityWeight);
P.routeCache=wp_six_trigger_route_cache_prepare_v8( ...
    routeGenerator,tauGrid);
P.builtOutsideTick=true;
P.safetyCertified=false;
end
