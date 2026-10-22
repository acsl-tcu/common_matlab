function T = AF7R58R16_AUTO_RESULT()
% R16 native low-jet -> production reference proposal bridge diagnostics.
% MATLAB process root-appdata only; never used for trajectory acceptance.
key='AF7R58R15_LAST_STATUS';
assert(isappdata(0,key), ...
    '[R16] no same-process stage1 capture; run the simulation first.');
S=getappdata(0,key);
assert(isstruct(S) && isfield(S,'r16_requested') && S.r16_requested==1, ...
    '[R16] bridge not enabled in captured simulation.');
required={'r16_connected','r16_native_success','r16_native_geometry', ...
    'r16_native_candidate_family','last_stage1_ms'};
assert(all(isfield(S,required)), '[R16] incomplete native candidate capture.');
T=struct();
T.bridge_requested=S.r16_requested;
T.native_connected=S.r16_connected;
T.native_success=S.r16_native_success;
T.native_geometry=S.r16_native_geometry;
T.candidate_family=S.r16_native_candidate_family;
T.stage1_ms=S.last_stage1_ms;
T.shadow_ms=S.r58_shadow_ms;
T.lowjet_dof=S.r58r14_active_reduced_dof;
T.full_dof=S.r58r14_full_reduced_dof;
T.speed_pass=S.r58_speed_pass;
T.heading_pass=S.r58_heading_pass;
T.geometry_pass=S.r58_geometry_pass;
T.stage1_status='UNKNOWN';
T.stage1_cycle_ms=NaN;
if isfield(S,'r16_stage1_status'),T.stage1_status=S.r16_stage1_status;end
if isfield(S,'r16_stage1_cycle_ms'),T.stage1_cycle_ms=S.r16_stage1_cycle_ms;end
T.stage2_status='NOT_REACHED';
T.stage2_ms=NaN;
T.stage3_status='NOT_REACHED';
T.stage3_ms=NaN;
if isfield(S,'r16_stage2_status'),T.stage2_status=S.r16_stage2_status;end
if isfield(S,'r16_stage2_ms'),T.stage2_ms=S.r16_stage2_ms;end
if isfield(S,'r16_stage3_status'),T.stage3_status=S.r16_stage3_status;end
if isfield(S,'r16_stage3_ms'),T.stage3_ms=S.r16_stage3_ms;end
T.reference_only=true;
T.activation_confirmed=false;
if isfield(S,'r16_activation_confirmed') && ...
        S.r16_activation_confirmed==1 && ...
        strcmp(T.stage3_status,'STAGE3_ATOMIC_COMMIT_CLOSED')
    T.activation_confirmed=true;
end
T.requires_stage2_stage3_verification=true;
T.full_execution_certified=false;
folder=fileparts(mfilename('fullpath'));
save(fullfile(folder,'AF7R58R16_CAPTURE.mat'),'T','S');
fid=fopen(fullfile(folder,'AF7R58R16_RESULT.txt'),'w');
assert(fid>=0,'[R16] cannot write result report.');
cleanup=onCleanup(@()fclose(fid));
fprintf(fid,'AF7R58R16 | NATIVE LOWJET REFERENCE BRIDGE\n');
fprintf(fid,'bridge_requested=%d native_connected=%d native_success=%d\n', ...
 T.bridge_requested,T.native_connected,T.native_success);
fprintf(fid,'family=%s\n',T.candidate_family);
fprintf(fid,'speed/heading/geometry=%d/%d/%d\n',T.speed_pass,T.heading_pass,T.geometry_pass);
fprintf(fid,'lowjet/full_dof=%.0f/%.0f\n',T.lowjet_dof,T.full_dof);
fprintf(fid,'stage1/shadow ms=%.7f / %.7f\n',T.stage1_ms,T.shadow_ms);
fprintf(fid,'stage1_status=%s cycle_ms=%.7f\n',T.stage1_status,T.stage1_cycle_ms);
fprintf(fid,'stage2_status=%s ms=%.7f\n',T.stage2_status,T.stage2_ms);
fprintf(fid,'stage3_status=%s ms=%.7f\n',T.stage3_status,T.stage3_ms);
fprintf(fid,'activation_confirmed=%d\n',T.activation_confirmed);
fprintf('R16: native_connected=%d, stage1_status=%s, cycle=%.6f ms\n', ...
 T.native_connected,T.stage1_status,T.stage1_cycle_ms);
end
