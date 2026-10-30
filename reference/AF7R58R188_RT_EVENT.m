function [seenVersion,seenIds,E]=AF7R58R188_RT_EVENT(seenVersion,seenIds,R,knownIds,version,t,H)
% Runtime same-tick conditional warning -> one event per obstacle/version.
% Returns diagnostic/request intent only. NEVER certifies physical safety.
E=struct('status','UNKNOWN_INPUT','new_event',false,'repeated',false,...
 'unknown',true,'obstacle_id',NaN,'component_code',0,...
 'first_ttc_s',NaN,'time_s',t,'reference_version',version,...
 'model_only',true,'full_closed_loop_safety_proved',false,...
 'production_commit_authorized',false);
if ~isstruct(R)||~isscalar(R)||~isnumeric(knownIds)||...
 ~isscalar(version)||~isfinite(version)||...
 ~isscalar(t)||~isfinite(t)||~isscalar(H)||~isfinite(H)||H<=0||...
 ~isnumeric(seenIds)||~isscalar(seenVersion)
 return;
end
knownIds=double(knownIds(:).');
if any(~isfinite(knownIds))||numel(unique(knownIds))~=numel(knownIds)
 E.status='UNKNOWN_SENSOR_IDS';return;
end
% Changing reference version rearms this obstacle once, including on return.
if isnan(seenVersion)||seenVersion~=version
 seenVersion=version;seenIds=zeros(1,0);
end
if ~isfield(R,'used_current_EKF')||~isequal(R.used_current_EKF,true)||...
 ~isfield(R,'reference_version')||...
 ~isequal(double(R.reference_version),double(version))
 E.status='UNKNOWN_EKF_OR_REFERENCE_VERSION';return;
end
r182Risk=isfield(R,'r182_model_unseparated') && ...
 isequal(R.r182_model_unseparated,true);
numericRisk=isfield(R,'numeric_request') && ...
 isequal(R.numeric_request,true);
if r182Risk
 idName='r182_first_obstacle_id';
 partName='r182_first_component_code';
 timeName='r182_first_unseparated_time_s';
 E.status='MODEL_INTERVAL_UNSEPARATED__NOT_PROVEN_COLLISION';
elseif numericRisk
 idName='first_obstacle_id';
 partName='first_component_code';
 timeName='first_ttc_s';
 E.status='NUMERIC_WITNESS__NOT_PROVEN_COLLISION';
else
 if isfield(R,'r182_conditional_clear') && ...
         isequal(R.r182_conditional_clear,true)
   E.status='CONDITIONAL_MODEL_CLEAR__EXECUTION_NOT_PROVED';
   E.unknown=false;
 else
   E.status='UNKNOWN_PREVIEW_OR_INTERVAL';
 end
 return;
end
if ~all(isfield(R,{idName,partName,timeName}))
 E.status='UNKNOWN_MISSING_WITNESS';return;
end
id=double(R.(idName)); part=double(R.(partName));
if strcmp(timeName,'r182_first_unseparated_time_s')
  firstTTC=double(R.(timeName))-t;
else
  firstTTC=double(R.(timeName));
end
if ~isscalar(id)||~isfinite(id)||~isscalar(part)||...
 ~ismember(part,1:3)||~isscalar(firstTTC)||~isfinite(firstTTC)||...
 firstTTC < -128*eps(max(1,abs(t))) ||...
 firstTTC > H+128*eps(max(1,abs(t))) ||~any(knownIds==id)
 E.status='UNKNOWN_WITNESS_NOT_IN_SAME_TICK_4S_KNOWN_SET';return;
end
E.unknown=false;E.obstacle_id=id;E.component_code=part;
E.first_ttc_s=max(0,firstTTC);
% An obstacle can reappear under a NEW reference version, but a changed
% affected body part alone is not a second episode for the same version.
if any(seenIds==id)
 E.repeated=true;
 E.status='SAME_OBSTACLE_REFERENCE_EVENT_ALREADY_ISSUED';
 return;
end
seenIds(end+1)=id;
E.new_event=true;
end
