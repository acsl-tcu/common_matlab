function binding=wp_six_trigger_original_model_binding(agent,model)
%WP_SIX_TRIGGER_ORIGINAL_MODEL_BINDING Locate the EXISTING 24-state RHS.
% This function never edits agent or uses plant's true payload mass.
% Full physical vector comes from agent.parameter; element 20 is replaced
% IN A LOCAL COPY with the CURRENT estimator model mass.
%
% Required external source, per supplied Model_Suspended_Load.m:
%    get_model_name('Load_HL') => with_load_model_euler_for_HL
% State order: [p(3);qRPY(3);v(3);w(3);pL(3);vL(3);pT(3);wL(3)]
% Parameter order: 21 original fields, loadmass at 20, cableL at 21.
if nargin~=2 || ~isstruct(model)
    error('LightTrigger:Binding','Need actual agent and model snapshot');
end
if ~(exist('get_model_name','file')==2) || ~(exist('with_load_model_euler_for_HL','file')==2)
    error('LightTrigger:MissingOriginalRHS', ...
        'Add ORIGINAL get_model_name and with_load_model_euler_for_HL to MATLAB path');
end
resolved=char(get_model_name('Load_HL'));
if ~strcmp(resolved,'with_load_model_euler_for_HL')
    error('LightTrigger:OriginalRHSChanged','Load_HL now resolves to %s; inspect first',resolved);
end
p=agent.parameter.get('all','row');
p=reshape(double(p),1,[]);
if numel(p)~=21 || any(~isfinite(p))
    error('LightTrigger:OriginalParameterOrder','Original model must supply 21 finite parameters');
end
% Verify that the parameter order is compatible with the named fields;
% no hard-coded drone mass/gravity/cable length is introduced.
checks=[1,6,7,8,9,21];
expected=[model.droneMass;model.inertia(:);model.gravity;model.cableLength];
if any(abs(p(checks)'-expected)>32*eps(max(1,abs(expected))))
    error('LightTrigger:ParameterOrder','Original parameter slots do not match named model fields');
end
p(20)=model.loadMass; % snapshot only; NEVER agent.parameter.set
if ~(p(20)>0 && p(1)>0 && p(21)>0)
    error('LightTrigger:OriginalParameter','Invalid estimated-mass model');
end
binding=struct('rhs',str2func(resolved),'physicalParameterRow',p, ...
    'modelSource',which(resolved),'modelResolver',which('get_model_name'), ...
    'stateDimension',24,'inputDimension',4,'physicalParameterCount',21, ...
    'estimatedMassUsed',model.loadMass, ...
    'plantTrueMassRead',false,'agentParameterModified',false);
end
