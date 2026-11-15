function C=wp_six_trigger_route_cache_prepare_v8(generator,tauGrid)
%WP_SIX_TRIGGER_ROUTE_CACHE_PREPARE_V8 PCHIP slopes OFF the 25ms tick.
% Identical nominal interpolation as V7 interp1(...,'pchip') evaluated
% at arbitrary admissible tau; no future real state used. If the original
% route is changed, rebuild this cache outside the real-time path.
C=wp_six_trigger_route_cache_prepare(generator,tauGrid);
C.ppP=pchip(C.tauGrid,C.p);
C.ppDP=pchip(C.tauGrid,C.dp);
C.ppDDP=pchip(C.tauGrid,C.ddp);
C.pchipCoefficientsPreparedOffTick=true;
end
