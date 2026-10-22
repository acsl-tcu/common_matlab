function T=AF7R58R22_AUTO_RESULT()
% Source-bound observational R22 measurements after simulation.
% Uses existing root app-data capture; no workspace instance required.
T=AF7R58R21_AUTO_RESULT();
assert(isfield(T,'r22_profile') && isstruct(T.r22_profile), ...
    '[R22] Missing R22 profile; check REPLANNING_SYSTEM source binding.');
F=T.r22_profile;
keys={'stage2_model_ms','stage2_sensor_ms', ...
      'stage2_eval_handles_ms','stage2_cache_ms', ...
      'stage2_native_eval_packet_ms','stage2_native_eval_ms', ...
      'r58_build_ms','r58_homotopy_ms', ...
      'r58_certificate_ms','r58_shadow_ms'};
for j=1:numel(keys)
    k=keys{j};
    assert(isfield(F,k) && isnumeric(F.(k)) && isscalar(F.(k)), ...
        '[R22] Missing timing field %s',k);
end
% Select the exact stage event rather than the mutable "last cycle" field.
E=T.events;
if isempty(E),names={};else,names={E.name};end
a=find(strcmp(names,'STAGE1_NATIVE_RETURNED'),1,'first');
b=find(strcmp(names,'STAGE2_CERTIFIED_AND_PREPARED'),1,'first');
T.r22_stage1_cycle_event_ms=NaN;
T.r22_stage2_cycle_event_ms=NaN;
if ~isempty(a),T.r22_stage1_cycle_event_ms=double(E(a).cycle_ms);end
if ~isempty(b),T.r22_stage2_cycle_event_ms=double(E(b).cycle_ms);end
T.r22_diagnostic_only=true;
T.r22_actual_commit=false;
T.r22_deadline_unchanged=true;
folder=fileparts(mfilename('fullpath'));
report=fullfile(folder,'AF7R58R22_RESULT.txt');
fid=fopen(report,'w','n','UTF-8');
assert(fid>=0,'[R22] Cannot create report file.');
c=onCleanup(@()fclose(fid));
fprintf(fid,'AF7R58R22 | DIRECT LOWJET HESSIAN + SHARED EVAL PACKET | NO COMMIT\n');
fprintf(fid,'classification=%s\n',T.classification);
fprintf(fid,'stage1_ms=%.9g stage2_ms=%.9g\n',T.stage1_ms,T.stage2_ms);
fprintf(fid,'stage1_cycle_event_ms=%.9g stage2_cycle_event_ms=%.9g\n', ...
    T.r22_stage1_cycle_event_ms,T.r22_stage2_cycle_event_ms);
fprintf(fid,'stage1_deadline_would_reject=%d stage2_deadline_would_reject=%d\n', ...
    T.deadline_stage1_would_reject,T.deadline_stage2_would_reject);
P=T.r20_profile;
fprintf(fid,'stage1_native_packet_ms=%.9g stage1_native_mex_ms=%.9g\n', ...
    P.native_packet_ms,P.native_mex_ms);
fprintf(fid,'stage2_total_bundle_ms=%.9g\n',P.stage2_bundle_ms);
for j=1:numel(keys)
    k=keys{j};
    fprintf(fid,'%s=%.9g\n',k,double(F.(k)));
end
fprintf(fid,'native_connected=%g native_success=%g\n',T.native_connected,T.native_success);
fprintf(fid,'speed_heading_geometry=%g %g %g\n', ...
    T.native_speed_pass,T.native_heading_pass,T.native_geometry_pass);
fprintf(fid,'active_source_stage1_requests=%d\n',T.r21_active_base_stage1_requests);
fprintf(fid,'active_source_stage2_prepared=%d\n',T.r21_active_base_stage2_prepared);
fprintf(fid,'ACTUAL_COMMIT=0; 25ms deadline remains unchanged; diagnostic bypass is not flight safe.\n');
fprintf(fid,'\nEVENTS\n');
for i=1:numel(E)
    e=E(i);
    fprintf(fid,'%s | %.12g | %.9g | %.9g\n',e.name,e.time_s,e.stage_ms,e.cycle_ms);
end
save(fullfile(folder,'AF7R58R22_RESULT.mat'),'T');
fprintf('========== R22 RESULT ==========\n');
fprintf('classification: %s\n',T.classification);
fprintf('Stage1/2: %.6f / %.6f ms; stage cycles: %.6f / %.6f ms\n', ...
    T.stage1_ms,T.stage2_ms,T.r22_stage1_cycle_event_ms,T.r22_stage2_cycle_event_ms);
fprintf('MEX R58 build/homotopy: %.6f / %.6f ms\n',F.r58_build_ms,F.r58_homotopy_ms);
fprintf('Stage2 model/sensor/handles/cache: %.6f / %.6f / %.6f / %.6f ms\n', ...
    F.stage2_model_ms,F.stage2_sensor_ms,F.stage2_eval_handles_ms,F.stage2_cache_ms);
fprintf('Report: %s\n',report);
end
