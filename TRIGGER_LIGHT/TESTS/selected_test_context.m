function [model,cfg,F] = selected_test_context(mode,cover,timeOffsets,trajectory,offset)
%TEST helper: synthetic agent/provider. NOT a real Flight parameter source.
if nargin<3 || isempty(timeOffsets),timeOffsets=linspace(0,4,41);end
if nargin<4,trajectory='straight';end
if nargin<5,offset=0;end
agent.parameter=struct('mass',0.762,'cableL',2.0,'gravity',9.81, ...
    'jx',0.02237568,'jy',0.02985236,'jz',0.0480374, ...
    'loadmass',999); % intentionally wrong, MUST NOT be used as forecast mL
agent.estimator.result.state=struct('mL',0.14);
model=wp_six_trigger_model_snapshot(agent);
assert(isequal(model.loadMass,0.14));
assert(isequal(agent.parameter.loadmass,999), ...
    'Model snapshot must NEVER modify any original parameter');
external=wp_system_geometry_config(mode);
cfg=wp_six_trigger_config(mode,cover,external,model);
F=six_test_forecast(timeOffsets,trajectory,offset);
F.modelLoadMass=model.loadMass;
end
