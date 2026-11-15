function Q = RA_AUDIT_ROUTE_CHANGE()
% TEST ONLY: show whether existing V8 cache can be silently stale after a route change.
% No route replacement is made in Flight. Known scheduled future route is allowed;
% an unannounced online replan is NOT predictable with the old cache.
old = gen_wp_straight('speed',0.65);
new = gen_wp_ra_audit_line_curve('speed',0.65);
tauGrid=0:0.025:10;
cOld=wp_six_trigger_route_cache_prepare_v8(old,tauGrid);
cNew=wp_six_trigger_route_cache_prepare_v8(new,tauGrid);
q=linspace(1,8,101);
oldPred=ppval(cOld.ppP,q);newPred=ppval(cNew.ppP,q);
% A newly assigned original generator is NOT automatically reflected in old cache.
delta=sqrt(sum((oldPred-newPred).^2,1));
Q=struct('status','NOT_TESTED','maxStaleRouteError_m',max(delta), ...
    'meanStaleRouteError_m',mean(delta), ...
    'requiresExplicitCacheRebuild',true, ...
    'requiresHistoryResetOrControlledHandoff',true, ...
    'predictionOfUnknownFutureRouteChangesPossible',false, ...
    'productionModified',false);
if Q.maxStaleRouteError_m>0.1
 Q.status='PASS_STALE_CACHE_RISK_EXPOSED__ONLINE_REPLAN_UNSUPPORTED';
else
 Q.status='FAIL_STALE_CACHE_TEST_INCONCLUSIVE';
end
end
