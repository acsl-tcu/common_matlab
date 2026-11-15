function out=TEST_TRIGGER_V2_SOURCE_DISCOVERY()
%TEST_TRIGGER_V2_SOURCE_DISCOVERY Report model/HLC RHS availability ONLY.
% This test does not simulate original Flight or certify model equivalence.
keys={'Model_Suspended_Load','HLC_SUSPENDED_LOAD', ...
    'Controller_HL_Suspended_Load','Load_HL','Load_mL_HL', ...
    'get_model_name','PARAMETER_CLASS'};
out=struct();
for i=1:numel(keys)
    out.(keys{i})=which(keys{i});
end
out.resolvedDynamicsIdentifier='';
out.resolverStatus='NOT_AVAILABLE';
if exist('get_model_name','file')==2
    try
        id=get_model_name('Load_HL');
        if ischar(id) || isstring(id)
            out.resolvedDynamicsIdentifier=char(id);
        else
            out.resolvedDynamicsIdentifier=class(id);
        end
        out.resolverStatus='RESOLVED_NAME_NOT_YET_AUDITED';
    catch e
        out.resolverStatus=['RESOLVER_FAILED_' e.identifier];
    end
end
out.actualRhsEquationEquivalenceCertified=false;
out.actualClosedLoopFutureBoundCertified=false;
fprintf('Original 24-state RHS identifier: %s (%s)\n', ...
    out.resolvedDynamicsIdentifier,out.resolverStatus);
end
