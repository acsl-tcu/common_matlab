function R=TEST_TRIGGER_V6_SOURCE_CONTRACT()
%TEST_TRIGGER_V6_SOURCE_CONTRACT Fail early on stale V3/V4 rollout source.
% This is a SOURCE contract audit, not a substitute for the numeric
% 6-second rollout and not a physical accuracy/safety certification.
% The V6 six-second reference needs original 24-state/HLC to remain intact
% while the PRIVATE predictor records the six governor filter states.
source=which('wp_six_trigger_predict_hlc_4s');
if isempty(source)
    error('V6:MissingPredictor', ...
        'wp_six_trigger_predict_hlc_4s.m is missing from MATLAB path');
end
allMatches=which('wp_six_trigger_predict_hlc_4s','-all');
if ischar(allMatches),allMatches={allMatches};end
if isstring(allMatches),allMatches=cellstr(allMatches);end
if numel(allMatches)>1
    warning('V6:ShadowedPredictor', ...
        'More than one predict_hlc_4s implementation on path; executing %s',source);
end
if ~endsWith(lower(source),'.m')
    error('V6:PredictorNotMFile', ...
        'Cannot statically audit selected predictor source: %s',source);
end
src=fileread(source);
needed={'governorRateState','governorSettings','governorCommand', ...
    'virtualTime','zHist(:,k+1)=z'};
missing={};
for k=1:numel(needed)
    if ~contains(src,needed{k})
        missing{end+1}=needed{k}; %#ok<AGROW>
    end
end
if ~isempty(missing)
    error('V6:PredictorVersionMismatch', ...
        ['Installed original-HLC rollout predates V6 governor-trace API. ' ...
        'Apply TRIGGER_LIGHT/SIX_WAY/wp_six_trigger_predict_hlc_4s.m ' ...
        'from the V6.1 patch. Missing: %s. Loaded: %s'], ...
        strjoin(missing,', '),source);
end
R=struct('status','PASS_V6_SOURCE_CONTRACT_STATIC_CHECK', ...
    'resolvedPredictorPath',source,'allResolvedPaths',{allMatches}, ...
    'requiredFields',{needed},'runtimeSixSecondValidated',false, ...
    'physicalPredictionErrorCertified',false);
end
