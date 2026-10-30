function [ledger,E]=AF7R58R187_RT_EPISODE(ledger,R,knownIds,version,t)
% Current-control-cycle obstacle event ledger. Pure, deterministic O(n).
% Separates model warning from execution-certified collision triggering.
% Does NOT authorize HLC actuation or a new reference commit.
E=struct('status','UNKNOWN_INPUT','new_event',false,'repeated',false,...
 'unknown',true,'obstacle_id',NaN,'component_code',0,...
 'first_ttc_s',NaN,'model_conditional_only',true,...
 'execution_reach_proved',false,'production_commit_authorized',false);
if ~isstruct(R)||~isscalar(R)||~isnumeric(knownIds)||...
 ~isscalar(version)||~isfinite(version)||~isscalar(t)||~isfinite(t)||...
 ~isstruct(ledger)||~all(isfield(ledger,{'version','obstacle_id'}))
 return;
end
knownIds=double(knownIds(:));
if any(~isfinite(knownIds))||numel(unique(knownIds))~=numel(knownIds)
 E.status='UNKNOWN_SENSOR_LEDGER';return;
end
if isfield(R,'used_current_EKF') && ~isequal(R.used_current_EKF,true)
 E.status='UNKNOWN_NOT_CURRENT_EKF';return;
end
if isfield(R,'reference_version') && ...
 ~isequal(double(R.reference_version),double(version))
 E.status='UNKNOWN_REFERENCE_VERSION_MISMATCH';return;
end
modelRisk=isfield(R,'r182_model_unseparated') && ...
 isequal(R.r182_model_unseparated,true);
numericRisk=isfield(R,'numeric_request') && isequal(R.numeric_request,true);
clearModel=isfield(R,'r182_conditional_clear') && ...
 isequal(R.r182_conditional_clear,true);
if modelRisk
 idField='r182_first_obstacle_id';codeField='r182_first_component_code';
 if isfield(R,'r182_first_unseparated_time_s') && ...
  isscalar(R.r182_first_unseparated_time_s) && ...
  isfinite(R.r182_first_unseparated_time_s) && ...
  double(R.r182_first_unseparated_time_s)>=t
  E.first_ttc_s=double(R.r182_first_unseparated_time_s)-t;
 end
elseif numericRisk
 idField='first_obstacle_id';codeField='first_component_code';
 if isfield(R,'first_ttc_s'),E.first_ttc_s=double(R.first_ttc_s);end
else
 if clearModel
  E.status='MODEL_CONDITIONAL_CLEAR__NO_TRUE_HLC_REACH_PROOF';E.unknown=false;
 else
  E.status='UNKNOWN_NUMERIC_OR_INTERVAL_REACH';
 end
 return;
end
if ~isfield(R,idField)||~isfield(R,codeField)
 E.status='UNKNOWN_RISK_WITNESS';return;
end
obs=double(R.(idField));body=double(R.(codeField));
if ~isscalar(obs)||~isfinite(obs)||~isscalar(body)||...
 ~ismember(body,1:3)||~any(knownIds==obs)
 E.status='UNKNOWN_WITNESS_NOT_IN_SAME_TICK_SENSOR_MEMORY';return;
end
E.unknown=false;E.obstacle_id=obs;E.component_code=body;
E.model_conditional_only=true;
if modelRisk
 E.status='MODEL_INTERVAL_UNSEPARATED__NEW_WARNING_NOT_PROVEN_COLLISION';
else
 E.status='SAMPLED_NUMERIC_WARNING__NOT_PROVEN_COLLISION';
end
% Obstacle ID, not body: changing body witness must NOT retrigger the
% same fixed-reference obstacle every tick. A new reference version can
% legitimately create a new episode (including during spatial return).
if ~isempty(ledger)
 present=([ledger.version]==version)&([ledger.obstacle_id]==obs);
 if any(present)
  E.status='REPEATED_SAME_OBSTACLE_AND_REFERENCE__SUPPRESSED';
  E.repeated=true;return;
 end
end
ledger(end+1)=struct('version',version,'obstacle_id',obs);
E.new_event=true;
end
