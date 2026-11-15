function R=RUN_RA_OPTIONAL_96_TRACE_REPLAY(repeats)
% Original V7 vs V8 96 cases, saved actual original-HLC nominal traces.
% This requires DRONE, DRONE_PARAM_SUSPENDED_LOAD, original HLC controller,
% original gen_wp_straight on the MATLAB path. It is NOT run by default.
if nargin<1,repeats=3;end
mustHave={'DRONE','DRONE_PARAM_SUSPENDED_LOAD','Controller_HL_Suspended_Load', ...
    'gen_wp_straight'};
missing={};
for j=1:numel(mustHave)
    if isempty(which(mustHave{j})),missing{end+1}=mustHave{j};end %#ok<AGROW>
end
if ~isempty(missing)
    R=struct('status','SKIPPED_ORIGINAL_HLC_DEPENDENCIES_NOT_ON_PATH', ...
        'missing',{missing},'physicalSafetyCertified',false);
    return
end
R=TEST_TRIGGER_V8_FASTPATH_AND_STREAM(repeats);
R.importedOriginalV8Test=true;
R.nominalOnly=true;
end
