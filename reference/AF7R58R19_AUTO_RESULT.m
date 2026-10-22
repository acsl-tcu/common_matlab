function R19 = AF7R58R19_AUTO_RESULT()
% AF7R58R19: post-run, disk-backed R17/R18 result; no live handles.
% Diagnostics only. All authoritative safety/commit rules unchanged.
R19=AF7R58R18_AUTO_RESULT();
folder=fileparts(mfilename('fullpath'));
report=fullfile(folder,'AF7R58R19_RESULT.txt');
fid=fopen(report,'w','n','UTF-8');
assert(fid>=0,'[R19] Could not open diagnostic report.');
c=onCleanup(@()fclose(fid));
fprintf(fid,'AF7R58R19 | NATIVE BRIDGE SCALAR TYPE FIX TRIAGE | NO COMMIT\n');
fprintf(fid,'classification=%s\n',char(R19.classification));
fprintf(fid,'stage1_generated=%d\n',logical(R19.stage1_generated));
fprintf(fid,'stage2_entered=%d\n',logical(R19.stage2_entered));
fprintf(fid,'stage2_prepared=%d\n',logical(R19.stage2_prepared));
fprintf(fid,'stage3_readonly_entered=%d\n',logical(R19.stage3_readonly_entered));
fprintf(fid,'deadline_stage1_would_reject=%d\n',logical(R19.deadline_stage1_would_reject));
fprintf(fid,'deadline_stage2_would_reject=%d\n',logical(R19.deadline_stage2_would_reject));
fprintf(fid,'native_connected=%.9g\n',r19_num(R19,'native_connected'));
fprintf(fid,'native_success=%.9g\n',r19_num(R19,'native_success'));
fprintf(fid,'speed/heading/geometry=%.9g / %.9g / %.9g\n', ...
    r19_num(R19,'native_speed_pass'),r19_num(R19,'native_heading_pass'), ...
    r19_num(R19,'native_geometry_pass'));
fprintf(fid,'stage1_ms=%.9g\n',r19_num(R19,'stage1_ms'));
fprintf(fid,'stage2_ms=%.9g\n',r19_num(R19,'stage2_ms'));
fprintf(fid,'stage3_precommit_reason=%s\n',r19_str(R19,'stage3_precommit_reason'));
fprintf(fid,'native_exception_captured=%d\n',logical(R19.native_exception_captured));
if R19.native_exception_captured
    fprintf(fid,'native_exception_identifier=%s\n',char(R19.native_exception_identifier));
    fprintf(fid,'native_exception_message=%s\n',char(R19.native_exception_message));
end
fprintf(fid,'commit_attempted=0 activation_confirmed=0\n');
fprintf(fid,'R19 is diagnostic-only: 25-ms deadline bypass does not grant flight safety.\n');
if isfield(R19,'events')
    fprintf(fid,'\nTIMELINE\n');
    for k=1:numel(R19.events)
        E=R19.events(k);
        fprintf(fid,'%s | %.12g | %.9g | %.9g\n', ...
            char(E.name),double(E.time_s),double(E.stage_ms),double(E.cycle_ms));
    end
end
save(fullfile(folder,'AF7R58R19_RESULT.mat'),'R19');
fprintf('========== R19 RESULT ==========\n');
fprintf('classification: %s\n',char(R19.classification));
fprintf('Stage1 generated / Stage2 prepared / Stage3 readonly = %d / %d / %d\n', ...
    R19.stage1_generated,R19.stage2_prepared,R19.stage3_readonly_entered);
fprintf('Native connected / success = %.9g / %.9g\n', ...
    r19_num(R19,'native_connected'),r19_num(R19,'native_success'));
fprintf('Stage1 / Stage2 = %.6f / %.6f ms\n', ...
    r19_num(R19,'stage1_ms'),r19_num(R19,'stage2_ms'));
fprintf('Native exception = %d\n',logical(R19.native_exception_captured));
if R19.native_exception_captured
    fprintf('Exception message: %s\n',char(R19.native_exception_message));
end
fprintf('report: %s\n',report);
end

function x=r19_num(S,k)
x=NaN;
if isfield(S,k) && isnumeric(S.(k)) && isscalar(S.(k))
    x=double(S.(k));
end
end

function x=r19_str(S,k)
x='NOT_AVAILABLE';
if isfield(S,k) && (ischar(S.(k)) || isstring(S.(k)))
    x=char(S.(k));
end
end
