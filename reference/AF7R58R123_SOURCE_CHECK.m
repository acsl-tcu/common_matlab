function S=AF7R58R123_SOURCE_CHECK()
% Read-only source/dependency checks for the new end-to-end rehearsal.
req={'REPLANNING_SYSTEM','AF7R58R74_REGISTRY', ...
    'AF7R58R74_FROZEN_EVENT_PREVIEW','AF7R58R120_BIND_R71_TRIGGER', ...
    'AF7R58R118_EKF_TRIGGER','AF7R58R121_WHOLEBODY_PLANNER', ...
    'AF7R58R122_BIND_LIVE_C7','AF7R58R122_C7_TO_RAW_XD28', ...
    'AF7R58R102_LIVE_INPUT_AUTHORITY', ...
    'AF7R58R123_EVENT_OBSTACLES', ...
    'AF7R58R123_FROZEN_PHYSICAL_TRIGGER', ...
    'AF7R58R123_ACTIVATION_SEAM','AF7R58R123_REPLAY_CORE', ...
    'AF7R58R123_LIVE_REHEARSAL'};
S=struct('status','MISSING_DEPENDENCIES','paths',struct([]), ...
    'old_flight_modified',false);
missing={};
for k=1:numel(req)
    f=which(req{k});
    if isempty(f),missing{end+1}=req{k}; %#ok<AGROW>
      fprintf('[R123][MISSING] %s\n',req{k});
    else
      fprintf('[R123][FOUND] %s = %s\n',req{k},f);
    end
end
if isempty(missing)
    S.status='PASS_R123_SOURCE_DEPENDENCIES__NO_FLIGHT_MUTATION';
else
    S.status='BLOCKED_R123_MISSING_SOURCE_DEPENDENCIES';
end
S.missing=missing;
S.old_flight_modified=false;
end
