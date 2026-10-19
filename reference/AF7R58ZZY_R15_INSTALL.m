function AF7R58ZZY_R15_INSTALL
% AF7R58ZZY-R15 transactional lean performance integration.
% Installs a direct REPLANNING_SYSTEM subclass and switches only the runner
% constructor. REPLANNING_SYSTEM.m itself is never modified.

EXPECTED_SOURCE_SHA='310cfdd01f9554a80b71e8d90464816a721859709ad7c4b9137cdc0f3a6eaf49';
EXPECTED_RUNNER_SHA='d4d15d081ea8a6e3073b0a63ce36e47f983777ac2c81db194384de69f5e5fc37';
EXPECTED_HELPER_SHA='1f57a0066acb75859dfe565b3c38a99d5c272b8afae4698131d7aeeb70093889';
scriptPath=mfilename('fullpath'); scriptDir=fileparts(scriptPath);
srcPath=which('REPLANNING_SYSTEM'); runnerPath=which('run_flight_headless');
helperPath=which('af7r33_flat_jet_batch_model');
assert(~isempty(srcPath)&&isfile(srcPath),'[R15] REPLANNING_SYSTEM.m not found.');
assert(~isempty(runnerPath)&&isfile(runnerPath),'[R15] run_flight_headless.m not found.');
assert(~isempty(helperPath)&&isfile(helperPath),'[R15] AF7R33 helper not found.');
assert(strcmpi(sha256_file_r15(srcPath),EXPECTED_SOURCE_SHA),'[R15] production source SHA mismatch.');
runnerSHA=sha256_file_r15(runnerPath);
helperSHA=sha256_file_r15(helperPath);
assert(strcmpi(helperSHA,EXPECTED_HELPER_SHA),'[R15] AF7R33 helper SHA mismatch.');
root=fileparts(runnerPath);

% Runner may be pristine or already R15-installed. Never accept another edit.
runnerText=fileread(runnerPath);
anchor='replanner_instance = REPLANNING_SYSTEM(agent,nominal_ref,replan_opts);';
installed='replanner_instance = REPLANNING_SYSTEM_R15_FAST(agent,nominal_ref,replan_opts);';
if contains(runnerText,installed)
    assert(count(runnerText,installed)==1,'[R15] runner R15 anchor is not unique.');
    fprintf('[R15] already installed. Verifying files only.\n');
else
    assert(strcmpi(runnerSHA,EXPECTED_RUNNER_SHA),'[R15] runner SHA mismatch before install.');
    assert(count(runnerText,anchor)==1,'[R15] pristine runner anchor count != 1.');
end

% Build exact AF7R33 fast helper from the bound production helper source.
src=fileread(helperPath);
fastSrc=build_fast_helper_r15(src);
fastPath=fullfile(root,'af7r33_flat_jet_batch_model_R15FAST.m');
write_utf8_r15(fastPath,fastSrc);

% Install lean direct subclass source.
classSrc=fullfile(scriptDir,'REPLANNING_SYSTEM_R15_FAST.m');
classDst=fullfile(root,'REPLANNING_SYSTEM_R15_FAST.m');
copyfile(classSrc,classDst,'f');

% Compile exact kernels into the runner root. Strict FP first; fail closed.
compile_mex_r15(fullfile(scriptDir,'af7r70_observer_mex.cpp'),root,'af7r70_observer_mex');
compile_mex_r15(fullfile(scriptDir,'af7r71_decasteljau_batch_mex.cpp'),root,'af7r71_decasteljau_batch_mex');
addpath(root,'-begin'); rehash;
clear REPLANNING_SYSTEM_R15_FAST af7r70_observer_mex af7r71_decasteljau_batch_mex af7r33_flat_jet_batch_model_R15FAST;
rehash;
assert(exist('af7r70_observer_mex','file')==3,'[R15] observer MEX install failed.');
assert(exist('af7r71_decasteljau_batch_mex','file')==3,'[R15] de Casteljau MEX install failed.');
assert(exist('af7r33_flat_jet_batch_model_R15FAST','file')==2,'[R15] fast helper install failed.');
assert(exist('REPLANNING_SYSTEM_R15_FAST','class')==8,'[R15] class install failed.');

% Transactional runner switch. Base REPLANNING_SYSTEM.m remains byte-exact.
backupPath=fullfile(root,'run_flight_headless.R15_PREINSTALL_BACKUP.m');
if ~contains(runnerText,installed)
    assert(~isfile(backupPath),'[R15] backup already exists; resolve it before install.');
    copyfile(runnerPath,backupPath);
    patched=strrep(runnerText,anchor,installed);
    write_utf8_r15(runnerPath,patched);
end
rehash;
assert(strcmpi(sha256_file_r15(srcPath),EXPECTED_SOURCE_SHA),'[R15] production source changed during install.');
newRunner=fileread(runnerPath);
assert(count(newRunner,installed)==1 && ~contains(newRunner,anchor),'[R15] runner switch did not close.');

manifest=fullfile(root,'AF7R58ZZY_R15_INSTALL_MANIFEST.txt');
fid=fopen(manifest,'w'); assert(fid>=0); c=onCleanup(@()fclose(fid));
fprintf(fid,'status=R15_INSTALLED\n');
fprintf(fid,'productionSourceModified=0\n');
fprintf(fid,'sourceSHA=%s\n',sha256_file_r15(srcPath));
fprintf(fid,'runnerSHA=%s\n',sha256_file_r15(runnerPath));
fprintf(fid,'helperOriginalSHA=%s\n',helperSHA);
fprintf(fid,'r15ClassSHA=%s\n',sha256_file_r15(classDst));
fprintf(fid,'r15FastHelperSHA=%s\n',sha256_file_r15(fastPath));
fprintf(fid,'observerCppSHA=%s\n',sha256_file_r15(fullfile(scriptDir,'af7r70_observer_mex.cpp')));
fprintf(fid,'decasteljauCppSHA=%s\n',sha256_file_r15(fullfile(scriptDir,'af7r71_decasteljau_batch_mex.cpp')));
fprintf(fid,'runnerBackup=%s\n',backupPath);
fprintf(fid,'controllerChanged=0\ntrajectoryMathChanged=0\ncertificateAcceptanceChanged=0\ncommitPolicyChanged=0\n');
fprintf('\n[R15] INSTALLED. REPLANNING_SYSTEM.m unchanged. Runner now uses REPLANNING_SYSTEM_R15_FAST.\n');
fprintf('[R15] Next normal command: run_flight_headless\n');
fprintf('[R15] Rollback command: AF7R58ZZY_R15_ROLLBACK\n');
end

function out=build_fast_helper_r15(src)
pat='(?m)^[ \t]*function[^\n]*\<af7r33_flat_jet_batch_model\>\s*\([^\n]*$';
[s0,e0]=regexp(src,pat,'start','end','once'); assert(~isempty(s0),'[R15] AF7R33 top declaration not found.');
top=src(s0:e0);
top2=regexprep(top,'(?<![A-Za-z0-9_])af7r33_flat_jet_batch_model(?=\s*\()','af7r33_flat_jet_batch_model_R15FAST','once');
src2=[src(1:s0-1) top2 src(e0+1:end)];
[decl,outVar,inVars,s,e]=locate_local_r15(src2,'decasteljau_batch');
orig='decasteljau_batch_r15_original';
renamed=regexprep(decl,'(?<![A-Za-z0-9_])decasteljau_batch(?=\s*\()',orig,'once');
a1=inVars{1}; a2=inVars{2};
wrapper=sprintf(['function %s=decasteljau_batch(%s,%s)%c' ...
    '%s=af7r71_decasteljau_batch_mex(%s,%s);%c' ...
    'end%c%c'],outVar,a1,a2,10,outVar,a1,a2,10,10,10);
out=[src2(1:s-1) wrapper renamed src2(e+1:end)];
end

function [decl,outVar,inVars,s,e]=locate_local_r15(src,name)
pat=['(?m)^[ \t]*function[^\n]*\<' regexptranslate('escape',name) '\>\s*\([^\n]*$'];
[s,e]=regexp(src,pat,'start','end','once'); assert(~isempty(s),'[R15] local %s not found.',name);
decl=src(s:e);
expr=['function\s+(?:\[?\s*)?([A-Za-z]\w*)\s*(?:\]?)\s*=\s*' regexptranslate('escape',name) '\s*\(([^)]*)\)'];
tok=regexp(decl,expr,'tokens','once'); assert(~isempty(tok),'[R15] unsupported local declaration.');
outVar=strtrim(tok{1}); args=strtrim(tok{2});
if isempty(args), inVars={}; else, inVars=strtrim(strsplit(args,',')); end
assert(numel(inVars)==2,'[R15] de Casteljau signature changed.');
end

function compile_mex_r15(cppPath,outDir,outName)
old=pwd; c=onCleanup(@()cd(old)); cd(outDir);
if ispc
    mex('-R2018a','COMPFLAGS=$COMPFLAGS /fp:strict',cppPath,'-output',outName,'-outdir',outDir);
else
    mex('-R2018a','CXXFLAGS=$CXXFLAGS -fno-fast-math -ffp-contract=off',cppPath,'-output',outName,'-outdir',outDir);
end
end

function h=sha256_file_r15(p)
fid=fopen(p,'rb'); assert(fid>=0,'[R15] cannot open %s.',p); c=onCleanup(@()fclose(fid));
b=fread(fid,Inf,'*uint8'); md=javaMethod('getInstance','java.security.MessageDigest','SHA-256');
md.update(typecast(b,'int8')); h=lower(reshape(dec2hex(typecast(md.digest(),'uint8'),2).',1,[]));
end

function write_utf8_r15(p,s)
fid=fopen(p,'w','n','UTF-8'); assert(fid>=0,'[R15] cannot write %s.',p); c=onCleanup(@()fclose(fid)); fwrite(fid,s,'char');
end
