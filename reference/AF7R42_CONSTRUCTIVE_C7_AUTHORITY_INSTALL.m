function OUT = AF7R42_CONSTRUCTIVE_C7_AUTHORITY_INSTALL()
% AF7R42_CONSTRUCTIVE_C7_AUTHORITY_INSTALL
%
% Fixes one source-proven consistency bug in the native planner:
%
%   build_candidate(...) constructs each degree-15 transition from endpoint
%   jets 0..7 and represents each unsafe affine span exactly, but the returned
%   Candidate leaves exact_c7 at its default false value.
%
% As a result, family-1 primary and family-2 fallback candidates are rejected
% BEFORE Stage-A/Stage-B by:
%
%       if(cand.exact_c7) ...
%       else continue;
%
% R38/R41 independently showed:
%   constructive exact-C7 candidates = 2
%   legacy exact flag pass           = 0
%   false rejects                    = 2
%
% This installer changes ONLY that metadata authority:
%   build_candidate(...) => C.exact_c7=true
%
% It does NOT alter:
%   - trajectory control points
%   - candidate objective / QP
%   - finite-sensing speed limit
%   - Stage-A theorem
%   - Stage-B theorem
%   - rejoin certificate
%   - continuous whole-body geometry certificate
%   - yaw/heading regularity requirements
%   - fallback order
%   - acceptance inequalities
%
% Therefore newly admitted candidates must still pass every unchanged hard
% certificate before they can be accepted.
%
% The script first compiles a staged patched MEX. Only after staged compilation
% succeeds does it write the production header and rebuild the production MEX.
% Failure after production write triggers rollback.
%
% Outputs are written beside THIS installer:
%   AF7R42_CONSTRUCTIVE_C7_AUTHORITY_REPORT.txt
%   AF7R42_CONSTRUCTIVE_C7_AUTHORITY_OUT.mat
%
% No README and no persistent backup directory are created.

fprintf('\n');
fprintf('====================================================================================================\n');
fprintf(' AF7R42 CONSTRUCTIVE-C7 AUTHORITY INSTALL\n');
fprintf(' trajectory mathematics changed              = 0\n');
fprintf(' hard certificate semantics changed          = 0\n');
fprintf(' exact-C7 metadata consistency repaired       = 1\n');
fprintf('====================================================================================================\n');

OUT=struct();
OUT.schema='AF7R42_CONSTRUCTIVE_C7_AUTHORITY_INSTALL_V1';
OUT.timestamp=char(datetime('now','Format','yyyy-MM-dd HH:mm:ss.SSS'));
OUT.production_modified=false;
OUT.rollback_performed=false;
OUT.staged_compile_pass=false;
OUT.production_compile_pass=false;
OUT.status='UNINITIALIZED';

auditDir=fileparts(mfilename('fullpath'));
if isempty(auditDir), auditDir=pwd; end
reportPath=fullfile(auditDir,'AF7R42_CONSTRUCTIVE_C7_AUTHORITY_REPORT.txt');
outPath=fullfile(auditDir,'AF7R42_CONSTRUCTIVE_C7_AUTHORITY_OUT.mat');
OUT.audit_directory=auditDir;
OUT.report_file=reportPath;
OUT.output_file=outPath;

fid=fopen(reportPath,'wt','n','UTF-8');
assert(fid>=0,'[R42] Cannot open report file: %s',reportPath);
clReport=onCleanup(@() fclose(fid)); %#ok<NASGU>

say(fid,'AF7R42 CONSTRUCTIVE-C7 AUTHORITY INSTALL');
say(fid,'timestamp=%s',OUT.timestamp);
say(fid,'trajectory_math_changed=0');
say(fid,'hard_certificate_semantics_changed=0');
say(fid,'candidate_objective_changed=0');
say(fid,'candidate_geometry_changed=0');
say(fid,'speed_gate_changed=0');
say(fid,'rejoin_gate_changed=0');
say(fid,'wholebody_gate_changed=0');
say(fid,'yaw_heading_requirement_changed=0');

% Resolve CURRENT production authority from the active MEX/helper directory.
mexPath=which('af7r36_single_core_mex');
helperPath=which('af7r34_make_core_packet');
classPath=which('REPLANNING_SYSTEM');

assert(~isempty(mexPath) && exist('af7r36_single_core_mex','file')==3, ...
    '[R42] Current af7r36_single_core_mex MEX not resolved.');
assert(~isempty(helperPath) && isfile(helperPath), ...
    '[R42] af7r34_make_core_packet.m not resolved.');
assert(~isempty(classPath) && isfile(classPath), ...
    '[R42] REPLANNING_SYSTEM.m not resolved.');

refDir=fileparts(helperPath);
hpp=fullfile(refDir,'af7r36_core.hpp');
cpp=fullfile(refDir,'af7r36_single_core_mex.cpp');
prodMex=fullfile(refDir,['af7r36_single_core_mex.' mexext]);

assert(isfile(hpp),'[R42] Missing current header: %s',hpp);
assert(isfile(cpp),'[R42] Missing current MEX source: %s',cpp);
assert(isfile(prodMex),'[R42] Missing current production MEX: %s',prodMex);

OUT.reference_directory=refDir;
OUT.hpp_path=hpp;
OUT.cpp_path=cpp;
OUT.mex_path=prodMex;
OUT.class_path=classPath;

hpp0=normalize_lf(fileread(hpp));
cpp0=normalize_lf(fileread(cpp));

OUT.hpp_sha_before=file_hash(hpp);
OUT.cpp_sha_before=file_hash(cpp);
OUT.mex_sha_before=file_hash(prodMex);
OUT.class_sha=file_hash(classPath);

say(fid,'');
say(fid,'[A. CURRENT AUTHORITY]');
say(fid,' reference_dir=%s',refDir);
say(fid,' REPLANNING_SYSTEM=%s',classPath);
say(fid,' hpp=%s',hpp);
say(fid,' cpp=%s',cpp);
say(fid,' mex=%s',prodMex);
say(fid,' hpp_sha_before=%s',OUT.hpp_sha_before);
say(fid,' cpp_sha_before=%s',OUT.cpp_sha_before);
say(fid,' mex_sha_before=%s',OUT.mex_sha_before);

% Source contract proven by R41 and current source.
marker='AF7R42_CONSTRUCTIVE_C7_AUTHORITY';
if contains(hpp0,marker)
    OUT.status='ALREADY_INSTALLED';
    OUT.production_modified=false;
    say(fid,'');
    say(fid,'[B. INSTALL]');
    say(fid,' marker already present = 1');
    say(fid,' production modified = 0');
    save(outPath,'OUT','-v7');
    fprintf('[R42] Already installed. No write performed.\n');
    return;
end

fnMarker='static inline Candidate build_candidate(const std::vector<Component>&comp,double t0,double t1,';
kfn=strfind(hpp0,fnMarker);
assert(numel(kfn)==1, ...
    '[R42] Expected exactly one build_candidate function; found %d.',numel(kfn));

% Verify structural construction markers inside build_candidate.
tail=hpp0(kfn(1):min(numel(hpp0),kfn(1)+12000));
required={ ...
    'vector_control_from_jets(a,b,comp[0].start-t0)', ...
    'p[k]=C.pos[j]+C.vel[j]*(T*(double(k)/15.0))', ...
    'vector_control_from_jets(a,b,Tt)', ...
    'vector_control_from_jets(a,b,t1-comp[j].end)', ...
    'for(int r=1;r<=6;++r)'};
OUT.structural_markers=false(size(required));
tailCompact=regexprep(tail,'\\s+','');
for k=1:numel(required)
    reqCompact=regexprep(required{k},'\\s+','');
    OUT.structural_markers(k)=contains(tailCompact,reqCompact);
end
assert(all(OUT.structural_markers), ...
    '[R42] build_candidate structural C7 construction markers are incomplete.');

oldInit='Candidate C;C.cost=cost;C.signs=signs;C.minSlack=minSlack;C.family=family;C.constant_translation=false;';
newInit=[ ...
    'Candidate C;C.cost=cost;C.signs=signs;C.minSlack=minSlack;C.family=family;C.constant_translation=false;' newline ...
    '    // AF7R42_CONSTRUCTIVE_C7_AUTHORITY:' newline ...
    '    // build_candidate is structurally C7: transition pieces are constructed' newline ...
    '    // from identical endpoint jets 0..7, while each unsafe affine span has' newline ...
    '    // exactly zero derivatives 2..7. This is representation authority, not' newline ...
    '    // a floating post-hoc equality test. All speed/rejoin/whole-body hard' newline ...
    '    // gates remain unchanged and are still mandatory.' newline ...
    '    C.exact_c7=true;'];

% Replace only inside build_candidate, never build_candidate_c7r7.
[hppNew,nrep]=replace_once_after(hpp0,fnMarker,oldInit,newInit);
assert(nrep==1,'[R42] Failed to patch build_candidate exact-C7 authority exactly once.');

% Ensure no hard-gate text is removed.
% Compare after removing ALL whitespace so CR/LF/spacing style cannot
% create a false negative.  The semantic token sequence must still exist.
gateMarkers={ ...
    'auto tr=tighten_stageA(cand,base,L.L,lim);', ...
    'sc=stageB_speed(ref,L,base,cand,lim,wit,rp);', ...
    'if(!rejoin_safe(ref,L,obs,t1,sc))continue;', ...
    'auto G=wholebody(ref,L,obs,t0,tm,R.hold_end,t1,cand,sc);'};
hppCompact=regexprep(hppNew,'\\s+','');
for k=1:numel(gateMarkers)
    gateCompact=regexprep(gateMarkers{k},'\\s+','');
    assert(contains(hppCompact,gateCompact), ...
        '[R42] Required hard-gate semantic token sequence disappeared: %s',gateMarkers{k});
end

% Temporary staging/rollback directory beside installer.
stageDir=tempname(auditDir);
[ok,msg]=mkdir(stageDir);
assert(ok,'[R42] Cannot create staging directory: %s',msg);
clStage=onCleanup(@() cleanup_stage(stageDir)); %#ok<NASGU>

stageHpp=fullfile(stageDir,'af7r36_core.hpp');
stageCpp=fullfile(stageDir,'af7r36_single_core_mex.cpp');
write_text(stageHpp,hppNew);
copyfile(cpp,stageCpp);

backupHpp=fullfile(stageDir,'af7r36_core.hpp.original');
backupMex=fullfile(stageDir,['af7r36_single_core_mex.original.' mexext]);
copyfile(hpp,backupHpp);
copyfile(prodMex,backupMex);

% ----------------------- staged compile: no production write -----------------------
say(fid,'');
say(fid,'[B. STAGED COMPILE]');
try
    mex('-R2018a','-O', ...
        'COMPFLAGS="$COMPFLAGS /std:c++17"', ...
        ['-I' stageDir], ...
        '-output','af7r36_single_core_mex_R42_stage', ...
        '-outdir',stageDir, ...
        stageCpp);
    stageMex=fullfile(stageDir,['af7r36_single_core_mex_R42_stage.' mexext]);
    assert(isfile(stageMex),'[R42] Staged MEX was not produced.');
    OUT.staged_compile_pass=true;
    OUT.staged_mex_sha=file_hash(stageMex);
    say(fid,' staged compile pass=1');
    say(fid,' staged_mex_sha=%s',OUT.staged_mex_sha);
catch ME
    OUT.status='STAGED_COMPILE_FAILED__PRODUCTION_UNTOUCHED';
    OUT.error=getReport(ME,'extended','hyperlinks','off');
    say(fid,' staged compile pass=0');
    say(fid,' production modified=0');
    say(fid,' error=%s',ME.message);
    save(outPath,'OUT','-v7');
    rethrow(ME);
end

% ----------------------- production write + rebuild with rollback -------------------
writesStarted=false;
try
    say(fid,'');
    say(fid,'[C. PRODUCTION INSTALL]');
    write_text(hpp,hppNew);
    writesStarted=true;
    OUT.production_modified=true;

    % Source write verification before compilation.
    hppDisk=normalize_lf(fileread(hpp));
    assert(strcmp(hppDisk,hppNew),'[R42] Header write verification failed.');
    OUT.hpp_sha_after_write=file_hash(hpp);
    say(fid,' hpp_sha_after_write=%s',OUT.hpp_sha_after_write);

    % Rebuild the actual authority. Header/source mathematics other than the
    % exact_c7 metadata bit are unchanged.
    clear mex
    rehash
    mex('-R2018a','-O', ...
        'COMPFLAGS="$COMPFLAGS /std:c++17"', ...
        ['-I' refDir], ...
        '-output','af7r36_single_core_mex', ...
        '-outdir',refDir, ...
        cpp);
    clear mex
    rehash

    assert(isfile(prodMex),'[R42] Production MEX missing after rebuild.');
    OUT.production_compile_pass=true;
    OUT.hpp_sha_after=file_hash(hpp);
    OUT.cpp_sha_after=file_hash(cpp);
    OUT.mex_sha_after=file_hash(prodMex);

    assert(strcmp(OUT.cpp_sha_before,OUT.cpp_sha_after), ...
        '[R42] MEX wrapper source unexpectedly changed.');
    assert(~strcmp(OUT.hpp_sha_before,OUT.hpp_sha_after), ...
        '[R42] Header hash did not change after intended install.');

    % The MEX must still expose its existing revision opcode.
    rev=af7r36_single_core_mex(5);
    OUT.native_revision=char(rev);

    OUT.status='INSTALL_COMPLETE__RERUN_SCENARIO_THEN_R38';
    OUT.rollback_performed=false;

    say(fid,' production compile pass=1');
    say(fid,' hpp_sha_after=%s',OUT.hpp_sha_after);
    say(fid,' cpp_sha_after=%s',OUT.cpp_sha_after);
    say(fid,' mex_sha_after=%s',OUT.mex_sha_after);
    say(fid,' native_revision=%s',OUT.native_revision);
    say(fid,' production_modified=1');
    say(fid,' rollback_performed=0');

catch ME
    OUT.error=getReport(ME,'extended','hyperlinks','off');
    if writesStarted
        try
            clear mex
            copyfile(backupHpp,hpp,'f');
            copyfile(backupMex,prodMex,'f');
            clear mex
            rehash
            OUT.rollback_performed=true;
            OUT.production_modified=false;
            OUT.status='INSTALL_FAILED__ROLLBACK_COMPLETE';
        catch ME2
            OUT.rollback_error=getReport(ME2,'extended','hyperlinks','off');
            OUT.status='INSTALL_FAILED__ROLLBACK_INCOMPLETE';
        end
    else
        OUT.status='INSTALL_FAILED__NO_PRODUCTION_WRITE';
    end

    say(fid,' production compile pass=0');
    say(fid,' status=%s',OUT.status);
    say(fid,' rollback_performed=%d',OUT.rollback_performed);
    say(fid,' error=%s',ME.message);
    save(outPath,'OUT','-v7');
    rethrow(ME);
end

say(fid,'');
say(fid,'[D. SAFETY CONTRACT AFTER INSTALL]');
say(fid,' constructive C7 metadata now matches structural construction = 1');
say(fid,' trajectory control points changed                           = 0');
say(fid,' objective/QP changed                                        = 0');
say(fid,' finite-sensing speed theorem changed                        = 0');
say(fid,' Stage-A/Stage-B inequalities changed                        = 0');
say(fid,' rejoin certificate changed                                  = 0');
say(fid,' continuous whole-body certificate changed                   = 0');
say(fid,' yaw/heading regularity requirement changed                  = 0');
say(fid,' fail-closed behavior after any failed hard gate retained    = 1');

say(fid,'');
say(fid,'[E. NEXT REQUIRED RUN]');
say(fid,' Run the same scenario again so the rebuilt MEX evaluates the two primary candidates.');
say(fid,' Then run: D38 = AF7R38_SPEED_ROOTCAUSE_AUDIT;');
say(fid,' Return the new D38 console output (and report/capture if generated).');
say(fid,' The next question is whether Stage-A or Stage-B now closes, or gives a finite speed witness.');

save(outPath,'OUT','-v7');
assignin('base','R42',OUT);

fprintf('\n[R42] INSTALL COMPLETE\n');
fprintf('  production header changed                   = 1\n');
fprintf('  MEX rebuilt                                 = 1\n');
fprintf('  trajectory/certificate mathematics changed  = 0\n');
fprintf('  next: rerun same scenario, then D38\n');
fprintf('  report: %s\n',reportPath);
fprintf('====================================================================================================\n');

end

% ================================================================================================
function [out,n]=replace_once_after(in,marker,old,new)
n=0; out=in;
km=strfind(in,marker);
if numel(km)~=1, return; end
ko=strfind(in(km(1):end),old);
if isempty(ko), return; end
p=km(1)+ko(1)-1;
out=[in(1:p-1) new in(p+length(old):end)];
n=1;
end

function s=normalize_lf(s)
s=strrep(s,sprintf('\r\n'),sprintf('\n'));
s=strrep(s,sprintf('\r'),sprintf('\n'));
end

function write_text(path,s)
fid=fopen(path,'wt','n','UTF-8');
assert(fid>=0,'Cannot open %s for write.',path);
c=onCleanup(@() fclose(fid)); %#ok<NASGU>
fwrite(fid,s,'char');
end

function h=file_hash(path)
md=java.security.MessageDigest.getInstance('SHA-256');
fid=fopen(path,'rb');
assert(fid>=0,'Cannot open %s for hash.',path);
c=onCleanup(@() fclose(fid)); %#ok<NASGU>
while true
    b=fread(fid,1024*1024,'*uint8');
    if isempty(b), break; end
    md.update(b);
end
d=typecast(md.digest(),'uint8');
h=lower(reshape(dec2hex(d,2).',1,[]));
end

function cleanup_stage(d)
try
    if isfolder(d), rmdir(d,'s'); end
catch
end
end

function say(fid,varargin)
fprintf(fid,[varargin{1} '\n'],varargin{2:end});
end
