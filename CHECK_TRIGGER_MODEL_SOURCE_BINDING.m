function S=CHECK_TRIGGER_MODEL_SOURCE_BINDING(agent,time)
% Read-only LIVE source audit. Use at a FLIGHT tick for valid mass evidence.
% After landing, mL can legitimately be 0 due to state manager; this is
% flagged as UNAVAILABLE_FOR_TAUT_PREDICTION, NOT misreported as model loss.
% Does NOT certify a continuous-time collision prediction.
S=struct('status','UNTESTED','time_s',NaN,'phase','', ...
 'active_rhs','','active_solver','', ...
 'ekf_mass_kg',NaN,'raw_ekf_mass_kg',NaN, ...
 'hlc_mass_source_kg',NaN,'plant_configured_mass_kg',NaN, ...
 'controller_mass_equals_estimator',false, ...
 'flight_dt_s',NaN,'plant_dt_s',NaN,'hlc_dt_s',NaN, ...
 'active_hlc_class','','plant_class','', ...
 'controller_uses_original_parameter',false, ...
 'prediction_mass_policy','CURRENT_EKF_FOR_PLANT__CURRENT_PARAMETER_FOR_HLC', ...
 'certified_continuous_time',false,'model_binding_readonly',true, ...
 'diagnostic_error','','source_errors',struct(), ...
 'trigger_connected_to_replanner',false, ...
 'obstacle_memory_connected_to_trigger',false, ...
 'mass_valid_for_taut_model',false, ...
 'landing_mass_policy_detected',false);
assert(nargin==2,'CHECK_TRIGGER_MODEL_SOURCE_BINDING:Arguments', ...
 'Provide existing live agent and time.');
issues={};
try
 S.time_s=double(time.t); S.flight_dt_s=double(time.dt);
 if isprop(agent,'cha'), S.phase=char(agent.cha);end
catch ME
 issues{end+1}=sprintf('time:%s %s',ME.identifier,ME.message); %#ok<AGROW>
end
try
 plant=agent.plant;S.plant_class=class(plant);
 S.active_rhs=func2str(plant.method);
 S.active_solver=func2str(plant.solver);
 S.plant_dt_s=double(plant.dt);
catch ME
 issues{end+1}=sprintf('plant:%s %s',ME.identifier,ME.message); %#ok<AGROW>
end
try
 ctrl=agent.controller.hlc_suspended;S.active_hlc_class=class(ctrl);
 S.hlc_dt_s=double(ctrl.param.dt);
 S.controller_uses_original_parameter=isa(ctrl,'HLC_SUSPENDED_LOAD');
catch ME
 issues{end+1}=sprintf('HLC:%s %s',ME.identifier,ME.message); %#ok<AGROW>
end
try
 S.ekf_mass_kg=double(agent.estimator.result.state.mL);
catch ME
 issues{end+1}=sprintf('EKF:%s %s',ME.identifier,ME.message); %#ok<AGROW>
end
try
 S.raw_ekf_mass_kg=double(agent.estimator.ekf.result.state.mL);
catch ME
 issues{end+1}=sprintf('EKFraw:%s %s',ME.identifier,ME.message); %#ok<AGROW>
end
try
 S.hlc_mass_source_kg=double(agent.parameter.get("loadmass"));
catch ME
 issues{end+1}=sprintf('HLCmass:%s %s',ME.identifier,ME.message); %#ok<AGROW>
end
try
 order=string(agent.parameter.parameter_order);
 assert(numel(unique(order))==numel(order),'Parameter order duplicate.');
 idx=find(order=="loadmass");
 assert(isscalar(idx),'No unique loadmass in parameter order.');
 pv=double(agent.plant.param);
 assert(isvector(pv)&&numel(pv)==numel(order), ...
     'Plant parameter vector/order mismatch.');
 S.plant_configured_mass_kg=double(pv(idx));
catch ME
 issues{end+1}=sprintf('plantmass:%s %s',ME.identifier,ME.message); %#ok<AGROW>
end
S.mass_valid_for_taut_model=isscalar(S.ekf_mass_kg) && ...
 isreal(S.ekf_mass_kg)&&isfinite(S.ekf_mass_kg)&&S.ekf_mass_kg>0;
S.controller_mass_equals_estimator=isequal(S.ekf_mass_kg,S.hlc_mass_source_kg);
S.landing_mass_policy_detected=strcmp(S.phase,'l')&& ...
 isequal(S.ekf_mass_kg,0);
if ~(isscalar(S.flight_dt_s)&&isscalar(S.plant_dt_s)&& ...
 isscalar(S.hlc_dt_s)&&isfinite(S.flight_dt_s)&& ...
 S.flight_dt_s>0&&isequal(S.flight_dt_s,S.plant_dt_s)&& ...
 isequal(S.flight_dt_s,S.hlc_dt_s))
 issues{end+1}='time: flight/plant/HLC dt invalid or unequal.'; %#ok<AGROW>
end
if ~isempty(issues)
 S.status='FAIL_SOURCE_BINDING';
 S.diagnostic_error=strjoin(issues,' | ');
elseif ~S.mass_valid_for_taut_model
 S.status='MODEL_SOURCES_PRESENT__EKF_MASS_UNUSABLE_IN_THIS_PHASE';
 S.diagnostic_error='EKF mass <=0 or non-finite: taut closed-loop prediction unavailable at this tick.';
elseif ~(isscalar(S.hlc_mass_source_kg)&&isfinite(S.hlc_mass_source_kg)&& ...
         S.hlc_mass_source_kg>0)
 S.status='MODEL_SOURCES_PRESENT__CURRENT_HLC_MASS_UNUSABLE';
 S.diagnostic_error='Original HLC parameter.loadmass is not positive in this phase.';
elseif ~(isscalar(S.plant_configured_mass_kg)&&isfinite(S.plant_configured_mass_kg)&& ...
         S.plant_configured_mass_kg>0)
 S.status='FAIL_PLANT_MASS_SOURCE';
 S.diagnostic_error='Original plant configured mass is invalid.';
elseif ~S.controller_uses_original_parameter
 S.status='FAIL_ACTIVE_HLC';
 S.diagnostic_error='Current controller is not HLC_SUSPENDED_LOAD.';
else
 S.status='PASS_LIVE_MODEL_SOURCES__NOT_CONTINUOUS_CERTIFIED';
end
end
