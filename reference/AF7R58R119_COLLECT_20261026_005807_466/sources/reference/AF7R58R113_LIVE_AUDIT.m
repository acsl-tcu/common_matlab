function A=AF7R58R113_LIVE_AUDIT(agentOrReplanner)
% Read-only audit of the ACTUAL running R111 native trigger, generator and
% Stage1/2/3. No Flight rerun, plot dependency, log, or engine switching.
P=[];
if isa(agentOrReplanner,'REPLANNING_SYSTEM')
 P=agentOrReplanner;
else
 try
  base=agentOrReplanner.reference;
  if isa(base,'REPLANNING_SYSTEM')
   P=base;
  else
   n=properties(base);
   for k=1:numel(n)
    try
     value=base.(n{k});
     if isa(value,'REPLANNING_SYSTEM'),P=value;break;end
    catch
    end
   end
  end
 catch
 end
end
A=struct('status','ACTIVE_REPLANNER_INSTANCE_NOT_FOUND', ...
 'actual_replanner_source',which('REPLANNING_SYSTEM'), ...
 'instance_found',false,'engine_armed',NaN, ...
 'trigger',struct([]),'pipeline',struct([]), ...
 'input_certificates',struct([]),'engine_mode','UNKNOWN', ...
 'full_hlc_execution_proved',false);
if isempty(P),return;end
A.instance_found=true;
try,A.engine_armed=logical(P.r108_dual_engine_armed);catch,end
if isequal(A.engine_armed,false)
 A.engine_mode='LEGACY_TRIGGER_AND_NATIVE_MEX_GENERATOR';
elseif isequal(A.engine_armed,true)
 A.engine_mode='DUAL_DISPATCH_ARMED__VERIFY_CALLBACKS';
end
try,A.trigger=P.get_last_trigger_event();catch ME,A.trigger=struct('error',ME.message);end
% Read counters directly. get_staged_pipeline_status() materializes lazy
% diagnostics; this audit intentionally does NOT modify the running object.
fields={'staged_replan_state','staged_replan_stage1_count', ...
 'staged_replan_stage2_count','staged_replan_commit_count', ...
 'staged_replan_last_reason','staged_replan_last_stage1_ms', ...
 'staged_replan_last_stage2_ms','staged_replan_last_stage3_ms', ...
 'staged_replan_fail_count','active_reference_version', ...
 'ENABLE_AF7R58R17_DIAG_CONTINUE_LATE'};
S=struct();
for j=1:numel(fields)
 try
  v=P.(fields{j});
  if isstring(v),v=char(v);end
  S.(fields{j})=v;
 catch
  S.(fields{j})='NOT_PUBLIC_OR_UNAVAILABLE';
 end
end
A.pipeline=S;
try
 C=P.get_runtime_certificate_status();
 D=struct('command_limit',struct([]), ...
   'actuator_dynamics',struct([]),'hlc_robust',struct([]), ...
   'live_controller_verified',false,'live_gain_verified',false, ...
   'robust_margin_switch',NaN,'backup_switch',NaN);
 for key={'command_limit','actuator_dynamics','hlc_robust', ...
          'live_controller_verified','live_gain_verified'}
  if isfield(C,key{1}),D.(key{1})=C.(key{1});end
 end
 try,D.robust_margin_switch=P.ENABLE_ROBUST_TRACKING_MARGIN;catch,end
 try,D.backup_switch=P.ENABLE_FAILURE_NO_SOLUTION_BACKUP;catch,end
 A.input_certificates=D;
 if isstruct(D.actuator_dynamics)&&isfield(D.actuator_dynamics,'certified')
  okAct=logical(D.actuator_dynamics.certified);
  okRob=isstruct(D.hlc_robust) && ...
   isfield(D.hlc_robust,'certified') && ...
   logical(D.hlc_robust.certified);
  A.full_hlc_execution_proved=okAct&&okRob&& ...
   logical(D.live_controller_verified)&&logical(D.live_gain_verified);
 end
catch ME
 A.input_certificates=struct('error',ME.message);
end
A.status='READ_ONLY_LIVE_TRIGGER_STAGE_AND_INPUT_STATUS';
end
