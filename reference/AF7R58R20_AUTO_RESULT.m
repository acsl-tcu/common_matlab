function T = AF7R58R20_AUTO_RESULT()
% R20: inspect live in-memory R17 research-only event log after simulation.
% No Flight disk writes, no workspace handles, no MEX call, no commit.
% Data belongs only to the current MATLAB process and is lost on restart.
key='AF7R58R17_LAST_CAPTURE';
assert(isappdata(0,key), ...
    ['[R20] No in-process R20 capture. Run the complete simulation ', ...
     'in this same MATLAB process with the preconfigured R20 source.']);
D=getappdata(0,key);
assert(isstruct(D) && isfield(D,'schema') && ...
    strcmp(char(D.schema),'AF7R58R17_DIAG_ONLY_NO_COMMIT_V1') && ...
    isfield(D,'events') && isfield(D,'r20_profile'), ...
    '[R20] Incompatible or incomplete capture.');
E=D.events;
if isempty(E), names={}; else, names={E.name}; end
has=@(n)any(strcmp(names,n));
startswith=@(n)any(strncmp(names,n,length(n)));
T=D;
T.deadline_stage1_would_reject=has('STAGE1_25MS_WOULD_REJECT');
T.deadline_stage2_would_reject=has('STAGE2_25MS_WOULD_REJECT');
T.stage1_generated=has('STAGE1_GENERATED');
T.stage2_entered=has('STAGE2_ENTERED');
T.stage2_prepared=has('STAGE2_CERTIFIED_AND_PREPARED');
T.stage3_readonly_entered=startswith('STAGE3_PRECOMMIT_');
T.actual_commit_attempted=false;
T.actual_reference_activation=false;
if has('STAGE3_PRECOMMIT_READY_NO_COMMIT')
    T.classification='STAGE2_PASS__STAGE3_PRECOMMIT_PASS__NOT_COMMITTED';
elseif T.stage3_readonly_entered
    T.classification='STAGE3_READONLY_REJECT__NOT_COMMITTED';
elseif T.stage2_prepared
    T.classification='STAGE2_PREPARED__STAGE3_NOT_REACHED';
elseif T.stage2_entered
    T.classification='STAGE2_ENTERED__REJECTED_OR_EXCEPTION';
elseif T.stage1_generated
    T.classification='STAGE1_GENERATED__STAGE2_NOT_REACHED';
elseif startswith('STAGE1_')
    T.classification='STAGE1_ENTERED__NOT_GENERATED';
else
    T.classification='STAGE1_NOT_OBSERVED';
end
folder=fileparts(mfilename('fullpath'));
report=fullfile(folder,'AF7R58R20_RESULT.txt');
fid=fopen(report,'w','n','UTF-8');
assert(fid>=0,'[R20] Cannot create report file.');
c=onCleanup(@()fclose(fid));
fprintf(fid,'AF7R58R20 | NO FLIGHT DISK IO | NO COMMIT | DIAGNOSTIC\n');
fprintf(fid,'classification=%s\n',T.classification);
fprintf(fid,'stage1_generated=%d stage2_prepared=%d stage3_readonly=%d\n', ...
    T.stage1_generated,T.stage2_prepared,T.stage3_readonly_entered);
fprintf(fid,'stage1_deadline_reject=%d stage2_deadline_reject=%d\n', ...
    T.deadline_stage1_would_reject,T.deadline_stage2_would_reject);
fprintf(fid,'stage1_ms=%.9g stage2_ms=%.9g\n',T.stage1_ms,T.stage2_ms);
F=T.r20_profile;
keys={ ...
 'stage1_future_ms','stage1_context_ms','stage1_native_call_ms', ...
 'native_packet_ms','native_mex_ms','native_wrapper_ms','native_internal_ms', ...
 'stage2_reference_cert_ms','stage2_bundle_ms','stage2_other_ms'};
for i=1:numel(keys)
    k=keys{i};fprintf(fid,'%s=%.9g\n',k,double(F.(k)));
end
if isfield(T,'native_exception_captured') && T.native_exception_captured
    fprintf(fid,'native_exception=%s: %s\n', ...
        char(T.native_exception_identifier),char(T.native_exception_message));
end
fprintf(fid,'native connected/success: %.9g / %.9g\n', ...
    r20_field(T,'native_connected'),r20_field(T,'native_success'));
fprintf(fid,'speed/heading/geometry=%.9g / %.9g / %.9g\n', ...
    r20_field(T,'native_speed_pass'),r20_field(T,'native_heading_pass'), ...
    r20_field(T,'native_geometry_pass'));
fprintf(fid,'\nEVENTS\n');
for i=1:numel(E)
    fprintf(fid,'%s | %.12g | %.9g | %.9g\n', ...
        E(i).name,E(i).time_s,E(i).stage_ms,E(i).cycle_ms);
end
save(fullfile(folder,'AF7R58R20_RESULT.mat'),'T');
fprintf('========== R20 RESULT ==========\n');
fprintf('classification: %s\n',T.classification);
fprintf('Stage1/Stage2: %.6f / %.6f ms\n',T.stage1_ms,T.stage2_ms);
fprintf('Native packet/MEX/wrapper: %.6f / %.6f / %.6f ms\n', ...
    F.native_packet_ms,F.native_mex_ms,F.native_wrapper_ms);
fprintf('Stage2 certificate/bundle/other: %.6f / %.6f / %.6f ms\n', ...
    F.stage2_reference_cert_ms,F.stage2_bundle_ms,F.stage2_other_ms);
fprintf('report: %s\n',report);
end
function x=r20_field(S,k)
x=NaN;
if isfield(S,k)&&isnumeric(S.(k))&&isscalar(S.(k)),x=double(S.(k));end
end
