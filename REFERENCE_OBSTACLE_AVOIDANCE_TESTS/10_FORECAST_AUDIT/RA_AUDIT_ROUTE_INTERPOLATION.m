function Q=RA_AUDIT_ROUTE_INTERPOLATION()
% Compare 0.025s precomputed PCHIP with the original known generator at
% off-grid arbitrary virtual times. NO true plant or online future data.
names={'gen_wp_straight','gen_wp_gentle_turn','gen_wp_circle', ...
       'gen_wp_saddle','gen_wp_ra_audit_line_curve'};
speed=[1.5,.6,.55,.55,.65];
r=struct('name',{},'maxP_m',{},'maxDP_mps',{},'maxDDP_mps2',{},'status',{});
for k=1:numel(names)
 g=feval(names{k},'speed',speed(k));
 tauGrid=0:.025:12;
 C=wp_six_trigger_route_cache_prepare_v8(g,tauGrid);
 tq=linspace(0,11.9,207)+.000043;
 eP=0;eV=0;eA=0;
 for j=1:numel(tq)
    x=g.evaluate(tq(j));
    eP=max(eP,norm(ppval(C.ppP,tq(j))-x(1:3)));
    eV=max(eV,norm(ppval(C.ppDP,tq(j))-x(5:7)));
    eA=max(eA,norm(ppval(C.ppDDP,tq(j))-x(9:11)));
 end
 r(end+1)=struct('name',names{k},'maxP_m',eP, ...
   'maxDP_mps',eV,'maxDDP_mps2',eA,'status','MEASURED_NOT_CERTIFIED'); %#ok<AGROW>
end
Q=struct('status','PASS_ROUTE_INTERPOLATION_MEASURED_ONLY', ...
 'records',r,'cacheGrid_s',.025,'validOnlyForKnownUnchangedRoutes',true, ...
 'strictInterpolationErrorBoundCertified',false);
end
