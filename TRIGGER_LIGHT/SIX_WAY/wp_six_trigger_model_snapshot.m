function model = wp_six_trigger_model_snapshot(agent)
%WP_SIX_TRIGGER_MODEL_SNAPSHOT Read current flight parameters without editing them.
% The CURRENT EKF/state-manager mL is treated as model truth for this tick.
% NO fallback to the parameter's loadmass and NO parameter.set call.
% Other values are read from the original agent.parameter provider.
%
% A plain struct provider is allowed ONLY to make unit tests independent of
% the original MATLAB project. In Flight use agent.parameter as supplied.
if ~isfield_or_prop(agent,'parameter') || ~isfield_or_prop(agent,'estimator')
    error('LightTrigger:Model','Agent must expose parameter and estimator');
end
estimator = agent.estimator;
if ~isfield_or_prop(estimator,'result') || ...
        ~isfield_or_prop(estimator.result,'state')
    error('LightTrigger:Model','Current post-update estimated state is missing');
end
state = estimator.result.state;
if ~isfield_or_prop(state,'mL')
    error('LightTrigger:Model','Current estimated mL is REQUIRED; no fallback');
end
mL = state.mL;
if ~isscalar(mL) || ~isnumeric(mL) || ~isfinite(mL) || mL <= 0
    error('LightTrigger:Model','Flight load mass estimate must be finite and positive');
end
p = agent.parameter;
model = struct();
model.loadMass = double(mL);
model.droneMass = getPhysical(p,'mass');
model.cableLength = getPhysical(p,'cableL');
model.gravity = getPhysical(p,'gravity');
model.inertia = [getPhysical(p,'jx');getPhysical(p,'jy');getPhysical(p,'jz')];
model.estimatedMassUsedAsModelTruth = true;
model.estimatorMassIsPhysicalTruthCertified = false;
model.readOnly = true;
if any([model.droneMass,model.cableLength,model.gravity,model.inertia(:)']<=0)
    error('LightTrigger:Model','Non-positive physical model parameter');
end
end

function v = getPhysical(p,name)
if isstruct(p)
    if ~isfield(p,name),error('LightTrigger:Model','Missing model parameter %s',name);end
    v=p.(name);
else
    % Existing PARAMETER_CLASS.get uses name-based lookup.
    if ~ismethod(p,'get')
        error('LightTrigger:Model','Parameter provider must implement get');
    end
    v=p.get(string(name));
end
if ~isnumeric(v) || ~isscalar(v) || ~isreal(v) || ~isfinite(v)
    error('LightTrigger:Model','Invalid model parameter %s',name);
end
v=double(v);
end

function yes=isfield_or_prop(x,name)
if isstruct(x), yes=isfield(x,name);else, yes=isprop(x,name);end
end
