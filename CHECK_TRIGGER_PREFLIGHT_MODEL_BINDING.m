function R=CHECK_TRIGGER_PREFLIGHT_MODEL_BINDING(replanner_instance)
% Read-only Flight-independent static binding status. NO synthetic update.
assert(nargin==1 && isobject(replanner_instance) && ...
    ismethod(replanner_instance,'get_status'), ...
    'CHECK_TRIGGER_PREFLIGHT_MODEL_BINDING:Input', ...
    'Pass the actual replanner_instance after the normal Flight.');
Q=replanner_instance.get_status();
R=struct();
R.preflight_model_binding_ready=Q.preflight_model_binding_ready;
R.preflight_model_binding_ms=Q.preflight_model_binding_ms;
R.preflight_model_binding_status=Q.preflight_model_binding_status;
R.preflight_update_ms=Q.preflight_update_ms;
R.first_flight_traces=Q.first_flight_traces;
R.last_trigger_status=Q.last_trigger_result.status;
R.native_gate_status=Q.trigger_native_gate_status;
R.status='READ_ONLY_STATIC_BINDING_AND_FIRST_TICK__NOT_SAFETY_PROOF';
folder=fileparts(mfilename('fullpath'));
save(fullfile(folder,'TRIGGER_PREFLIGHT_MODEL_BINDING.mat'),'R');
fid=fopen(fullfile(folder,'TRIGGER_PREFLIGHT_MODEL_BINDING.txt'),'wt');
assert(fid>=0,'Cannot create binding report.');
closer=onCleanup(@()fclose(fid));
fprintf(fid,'status=%s\n',R.status);
fprintf(fid,'preflight_model_binding_ready=%d\n',R.preflight_model_binding_ready);
fprintf(fid,'preflight_model_binding_ms=%.6f\n',R.preflight_model_binding_ms);
fprintf(fid,'preflight_model_binding_status=%s\n',R.preflight_model_binding_status);
fprintf(fid,'preflight_update_ms=%.6f\n',R.preflight_update_ms);
fprintf(fid,'native_gate=%s\n',R.native_gate_status);
if ~isempty(R.first_flight_traces)
  t=R.first_flight_traces(1);
  fprintf(fid,'first_t=%.6f\n',t.time_s);
  fprintf(fid,'first_authority_create_ms=%.6f\n',t.authority_create_ms);
  fprintf(fid,'first_authority_current_ms=%.6f\n',t.authority_current_ms);
  fprintf(fid,'first_core_ms=%.6f\n',t.core_wall_ms);
  fprintf(fid,'first_total_ms=%.6f\n',t.pre_store_total_ms);
end
fprintf('[TRIGGER PREBIND] %s | ready=%d | model prebind=%.3f ms\n', ...
  R.status,R.preflight_model_binding_ready,R.preflight_model_binding_ms);
end
