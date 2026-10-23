function R = AF7R58R44_SOURCE_AUTHORITY(P)
% R44: source-bound prerequisite check for a true estimated-state 4-s trigger.
% READ ONLY. Does NOT change the live trigger, the controller or the plant.
if nargin < 1
    if evalin('base','exist(''replanner_instance'',''var'') == 1')
        P=evalin('base','replanner_instance');
    else
        P=[];
    end
end
R=struct();
R.schema='AF7R58R44_SOURCE_AUTHORITY_V1';
R.production_trigger_switched=false;
R.closed_loop_forecast_certified=false;
R.paths=struct();
names={'REPLANNING_SYSTEM','HLC_SUSPENDED_LOAD','Model_Suspended_Load', ...
       'Load_HL','Load_mL_HL','with_load_model_euler_for_HL', ...
       'with_load_model_mL_euler_for_HL','get_model_name'};
for k=1:numel(names)
    R.paths.(names{k})=which(names{k});
end
R.plant_rhs_available=~isempty(R.paths.Load_HL) || ...
    ~isempty(R.paths.with_load_model_euler_for_HL);
R.estimator_rhs_available=~isempty(R.paths.Load_mL_HL) || ...
    ~isempty(R.paths.with_load_model_mL_euler_for_HL);
R.hlc_class_available=~isempty(R.paths.HLC_SUSPENDED_LOAD);
R.live_instance_found=~isempty(P);
R.estimator_state_complete=false;
R.controller_binding_present=false;
R.latest_mass_kg=NaN;
R.horizon_s=NaN;
R.prediction_dt_s=NaN;
R.sensor_range_m=NaN;
if isempty(P)
    R.status='LIVE_REPLANNER_UNAVAILABLE_IN_BASE_WORKSPACE';
    return;
end
assert(isa(P,'REPLANNING_SYSTEM'), 'Live instance is not REPLANNING_SYSTEM.');
try
    R.loaded_plant_rhs=func2str(P.self.plant.method);
    R.plant_rhs_available=isa(P.self.plant.method,'function_handle');
catch
    R.loaded_plant_rhs='';
end
R.active_mode=char(P.active_reference_mode);
R.active_version=double(P.active_reference_version);
R.horizon_s=double(P.horizon_s);
R.prediction_dt_s=double(P.prediction_dt);
R.sensor_range_m=double(P.trigger_dist);
S=P.self.estimator.result.state;
fieldNames={'p','q','v','w','pL','vL','pT','wL','mL'};
found=false(size(fieldNames));
for k=1:numel(fieldNames)
    found(k)=isprop(S,fieldNames{k});
end
R.estimator_missing=fieldNames(~found);
R.estimator_state_complete=all(found);
if R.estimator_state_complete
    R.latest_mass_kg=double(S.mL);
    R.estimated_uav_p=double(S.p(:));
    R.estimated_uav_v=double(S.v(:));
    R.estimated_load_p=double(S.pL(:));
    R.estimated_load_v=double(S.vL(:));
    R.estimated_cable_q=double(S.pT(:));
    R.estimated_cable_omega=double(S.wL(:));
    R.estimated_angular_velocity=double(S.w(:));
    R.estimated_attitude=double(S.q(:));
    assert(isscalar(R.latest_mass_kg) && isfinite(R.latest_mass_kg) && ...
       R.latest_mass_kg>0, 'Latest mL estimator state invalid.');
    R.initial_cable_geometry_residual_m=norm( ...
       R.estimated_load_p-R.estimated_uav_p- ...
       double(P.L_cable)*R.estimated_cable_q);
else
    R.initial_cable_geometry_residual_m=NaN;
end
% Presence of an HLC object is NOT equivalent to a pure-state HLC
% evaluation function. The prediction must not mutate the real estimator.
try
    R.controller_binding_present=isprop(P.self,'controller') && ...
       ~isempty(P.self.controller);
catch
    R.controller_binding_present=false;
end
R.required_model_equations= ...
   'exact Load_HL state ODE + actual HLC map + input saturation/ZOH';
R.note='Reference packet cannot substitute for an estimated-state closed-loop rollout.';
if ~R.plant_rhs_available
    R.status='PLANT_RHS_SOURCE_INTERFACE_NOT_AVAILABLE_TO_VERIFIER';
elseif ~R.estimator_state_complete
    R.status='ESTIMATED_25D_STATE_INCOMPLETE';
else
    R.status='STATE_AND_SOURCE_DISCOVERED__PREDICTOR_NOT_YET_CERTIFIED';
end
end
