function plan = wp_six_trigger_fixed_plan(known,cfg,nominalCenters,options)
%WP_SIX_TRIGGER_FIXED_PLAN One reusable sampling schedule for ONE cover.
% Rebuild when the active reference or the observed obstacle memory changes.
% Density selection cannot certify any interval as collision-free.
if nargin<4,options=struct();end
plan=wp_trigger_plan_once(known,cfg,nominalCenters,options);
plan.systemMode=cfg.systemMode;
plan.obstacleApproximation=cfg.obstacleApproximation;
end
