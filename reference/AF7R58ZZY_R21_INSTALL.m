function AF7R58ZZY_R21_INSTALL
% AF7R58ZZY-R21 theory-preserving Stage2/Stage3 load-balance installer.
% Returns to R19 numerical authority and defers ONLY the non-authority first
% post-commit reference cache from Stage2 to Stage3 before atomic activation.

EXPECTED_SOURCE_SHA='310cfdd01f9554a80b71e8d90464816a721859709ad7c4b9137cdc0f3a6eaf49';
EXPECTED_HELPER_SHA='1f57a0066acb75859dfe565b3c38a99d5c272b8afae4698131d7aeeb70093889';
EXPECTED_PRISTINE_RUNNER_SHA='d4d15d081ea8a6e3073b0a63ce36e47f983777ac2c81db194384de69f5e5fc37';

scriptDir=fileparts(mfilename('fullpath'));
srcPath=which('REPLANNING_SYSTEM');
runnerPath=which('run_flight_headless');
helperPath=which('af7r33_flat_jet_batch_model');
assert(~isempty(srcPath)&&isfile(srcPath),'[R21] REPLANNING_SYSTEM.m not found.');
assert(~isempty(runnerPath)&&isfile(runnerPath),'[R21] run_flight_headless.m not found.');
assert(~isempty(helperPath)&&isfile(helperPath),'[R21] AF7R33 helper not found.');
assert(strcmpi(sha256_file_r21(srcPath),EXPECTED_SOURCE_SHA),'[R21] production source SHA mismatch.');
assert(strcmpi(sha256_file_r21(helperPath),EXPECTED_HELPER_SHA),'[R21] AF7R33 helper SHA mismatch.');
root=fileparts(runnerPath);

% Source-bound theorem check: production performs the hot certificate/version
% assertion before consuming the prepared post-commit cache. Therefore that
% cache is not an acceptance certificate; it is installed only after authority
% safety closure. Refuse installation if this source relation changes.
Rtxt=fileread(srcPath);
tokA='obj.assert_runtime_core_certificates_current_hot();';
tokB='Cpost=B.post_commit_reference_cache;';
tokC='obj.repaired_reference_cache=Cpost;';
a=strfind(Rtxt,tokA); b=strfind(Rtxt,tokB); c=strfind(Rtxt,tokC);
assert(numel(a)>=1&&numel(b)==1&&numel(c)==1,'[R21] production postcache authority tokens changed.');
a=a(find(a<b(1),1,'last'));
assert(~isempty(a)&&a<b(1)&&b(1)<c(1), ...
    '[R21] production postcache is no longer downstream of runtime certificate closure.');

baseAnchor='replanner_instance = REPLANNING_SYSTEM(agent,nominal_ref,replan_opts);';
r15Anchor='replanner_instance = REPLANNING_SYSTEM_R15_FAST(agent,nominal_ref,replan_opts);';
r16Anchor='replanner_instance = REPLANNING_SYSTEM_R16_FAST(agent,nominal_ref,replan_opts);';
r17Anchor='replanner_instance = REPLANNING_SYSTEM_R17_FAST(agent,nominal_ref,replan_opts);';
r18Anchor='replanner_instance = REPLANNING_SYSTEM_R18_STAGE2_DIAG(agent,nominal_ref,replan_opts);';
r19Anchor='replanner_instance = REPLANNING_SYSTEM_R19_FAST(agent,nominal_ref,replan_opts);';
r20Anchor='replanner_instance = REPLANNING_SYSTEM_R20_FAST(agent,nominal_ref,replan_opts);';
r21Anchor='replanner_instance = REPLANNING_SYSTEM_R21_BALANCED(agent,nominal_ref,replan_opts);';
txt=fileread(runnerPath);

if contains(txt,r21Anchor)
    assert(count(txt,r21Anchor)==1,'[R21] R21 runner anchor is not unique.');
    mode='ALREADY_R21'; oldAnchor='';
elseif contains(txt,r20Anchor)
    assert(count(txt,r20Anchor)==1,'[R21] R20 runner anchor is not unique.');
    mode='FROM_R20'; oldAnchor=r20Anchor;
elseif contains(txt,r19Anchor)
    assert(count(txt,r19Anchor)==1,'[R21] R19 runner anchor is not unique.');
    mode='FROM_R19'; oldAnchor=r19Anchor;
elseif contains(txt,r18Anchor)
    assert(count(txt,r18Anchor)==1,'[R21] R18 runner anchor is not unique.');
    mode='FROM_R18'; oldAnchor=r18Anchor;
elseif contains(txt,r17Anchor)
    assert(count(txt,r17Anchor)==1,'[R21] R17 runner anchor is not unique.');
    mode='FROM_R17'; oldAnchor=r17Anchor;
elseif contains(txt,r16Anchor)
    assert(count(txt,r16Anchor)==1,'[R21] R16 runner anchor is not unique.');
    mode='FROM_R16'; oldAnchor=r16Anchor;
elseif contains(txt,r15Anchor)
    assert(count(txt,r15Anchor)==1,'[R21] R15 runner anchor is not unique.');
    mode='FROM_R15'; oldAnchor=r15Anchor;
elseif contains(txt,baseAnchor)
    assert(count(txt,baseAnchor)==1,'[R21] pristine runner anchor is not unique.');
    assert(strcmpi(sha256_file_r21(runnerPath),EXPECTED_PRISTINE_RUNNER_SHA),'[R21] pristine runner SHA mismatch.');
    mode='FROM_PRISTINE'; oldAnchor=baseAnchor;
else
    error('[R21] runner is not a recognized pristine/R15/R16/R17/R18/R19/R20/R21 authority.');
end

files={ ...
    'REPLANNING_SYSTEM_R19_FAST.m', ...
    'REPLANNING_SYSTEM_R21_BALANCED.m', ...
    'af7r75_posaccel_batch_R19.m', ...
    'af7r75_posaccel_core_R19.m', ...
    'af7r72_posaccel_core_R16.m', ...
    'af7r73_posaccel_core_R17.m'};
for k=1:numel(files)
    s=fullfile(scriptDir,files{k});
    assert(isfile(s),'[R21] package file missing: %s',files{k});
    copyfile(s,fullfile(root,files{k}),'f');
end

compile_mex_r21(fullfile(scriptDir,'af7r70_observer_mex.cpp'),root,'af7r70_observer_mex');
compile_mex_r21(fullfile(scriptDir,'af7r71_decasteljau_batch_mex.cpp'),root,'af7r71_decasteljau_batch_mex');
compile_mex_r21(fullfile(scriptDir,'af7r75_correction_posaccel_mex.cpp'),root,'af7r75_correction_posaccel_mex');
addpath(root,'-begin'); rehash;
clear af7r75_posaccel_batch_R19 af7r75_posaccel_core_R19 af7r72_posaccel_core_R16 af7r73_posaccel_core_R17;
rehash;

assert(exist('af7r70_observer_mex','file')==3,'[R21] observer MEX install failed.');
assert(exist('af7r71_decasteljau_batch_mex','file')==3,'[R21] de Casteljau MEX install failed.');
assert(exist('af7r75_correction_posaccel_mex','file')==3,'[R21] R19 correction MEX install failed.');
assert(exist('af7r75_posaccel_batch_R19','file')==2,'[R21] R19 batch helper install failed.');
assert(exist('REPLANNING_SYSTEM_R19_FAST','class')==8,'[R21] R19 parent class parse/install failed.');
assert(exist('REPLANNING_SYSTEM_R21_BALANCED','class')==8,'[R21] R21 class parse/install failed.');

% Same R19 numerical evaluator gate. R21 does not introduce a new evaluator.
selftest_r21_r19_exact();

Ctxt=fileread(fullfile(root,'REPLANNING_SYSTEM_R21_BALANCED.m'));
assert(~contains(Ctxt,'REPLANNING_SYSTEM_R20_FAST'),'[R21] failed R20 authority leaked into R21 inheritance.');
assert(contains(Ctxt,'classdef REPLANNING_SYSTEM_R21_BALANCED < REPLANNING_SYSTEM_R19_FAST'), ...
    '[R21] R19 authority inheritance missing.');
assert(contains(Ctxt,'''post_commit_cache_pending'',true'),'[R21] deferred-cache marker missing.');
assert(contains(Ctxt,'tCache=double(t_now)+dt;'),'[R21] Stage3 cache timing is not t_commit+dt.');
assert(contains(Ctxt,'obj.assert_runtime_core_certificates_current_hot();'), ...
    '[R21] production hot certificate assertion missing from atomic activation copy.');
assert(contains(Ctxt,'''pipeline_intervals'',2'),'[R21] pipeline interval contract changed.');

backupPath=fullfile(root,'run_flight_headless.R21_PREINSTALL_BACKUP.m');
if ~strcmp(mode,'ALREADY_R21')
    assert(~isfile(backupPath),'[R21] R21 runner backup already exists; resolve before install.');
    copyfile(runnerPath,backupPath);
    patched=strrep(txt,oldAnchor,r21Anchor);
    assert(count(patched,r21Anchor)==1,'[R21] runner replacement count invalid.');
    write_utf8_r21(runnerPath,patched);
end
rehash;

assert(strcmpi(sha256_file_r21(srcPath),EXPECTED_SOURCE_SHA),'[R21] production source changed during install.');
assert(strcmpi(sha256_file_r21(helperPath),EXPECTED_HELPER_SHA),'[R21] AF7R33 helper changed during install.');
newtxt=fileread(runnerPath);
assert(count(newtxt,r21Anchor)==1,'[R21] runner switch did not close.');

manifest=fullfile(root,'AF7R58ZZY_R21_INSTALL_MANIFEST.txt');
fid=fopen(manifest,'w'); assert(fid>=0); cc=onCleanup(@()fclose(fid));
fprintf(fid,'status=R21_INSTALLED\n');
fprintf(fid,'installMode=%s\n',mode);
fprintf(fid,'productionSourceModified=0\n');
fprintf(fid,'sourceSHA=%s\n',sha256_file_r21(srcPath));
fprintf(fid,'runnerSHA=%s\n',sha256_file_r21(runnerPath));
fprintf(fid,'r21ClassSHA=%s\n',sha256_file_r21(fullfile(root,'REPLANNING_SYSTEM_R21_BALANCED.m')));
fprintf(fid,'runnerBackup=%s\n',backupPath);
fprintf(fid,'baseNumericalAuthority=R19\n');
fprintf(fid,'controllerChanged=0\ntrajectoryMathChanged=0\ncertificateAcceptanceChanged=0\ncommitPolicyChanged=0\n');
fprintf(fid,'pipelineIntervals=2\nactivationDelayChanged=0\n');
fprintf(fid,'deferredItem=POST_COMMIT_REFERENCE_CACHE_ONLY\n');
fprintf(fid,'workShiftedToArmingTakeoffLanding=0\n');

fprintf('\n[R21] INSTALLED | mode=%s\n',mode);
fprintf('[R21] R20 discarded: numerical authority is R19.\n');
fprintf('[R21] R19 native correction exactness self-test CLOSED.\n');
fprintf('[R21] Source audit CLOSED: post-commit cache is downstream of certificate hot guard.\n');
fprintf('[R21] Activation remains trigger+2*dt; no Arming/Takeoff/Landing work added.\n');
fprintf('[R21] Run normally: run_flight_headless\n');
fprintf('[R21] Rollback target is R19: AF7R58ZZY_R21_ROLLBACK\n');
end

function selftest_r21_r19_exact()
t=[0.25 0.5 0.999999 1.0 1.125 1.25 1.5 1.75 2.0 2.125 2.25 2.5 2.75 3.0 3.000001 3.4];
N=numel(t); P=zeros(3,N); A=zeros(3,N);
for k=1:N
    P(:,k)=[1+t(k); -0.0; 3-0.125*t(k)];
    A(:,k)=[0.1+k*eps; -0.2; 0.3-2*k*eps];
end
model=struct('schema','AF7R33_BATCH_FLAT_MODEL_V1','certified',true, ...
    'corrections',struct('t0',{},'t1',{},'segment_t0',{},'segment_t1',{},'control_points',{}));
model.corrections(1)=synthetic_correction_r21(1.0,2.5, 0.013,3);
model.corrections(2)=synthetic_correction_r21(1.5,3.0,-0.021,2);
model.corrections(3)=synthetic_correction_r21(2.0,3.4, 0.007,4);
[Pr,Ar,Vr]=af7r73_posaccel_core_R17(P,A,t,model);
[Pn,An,Vn]=af7r75_posaccel_core_R19(P,A,t,model);
assert(isequal(Vr,Vn) && bitwise_r21(Pr,Pn) && bitwise_r21(Ar,An), ...
    '[R21] R19 native correction synthetic bitwise gate failed.');
end

function C=synthetic_correction_r21(t0,t1,scale,S)
edges=linspace(t0,t1,S+1);
C=struct(); C.t0=t0; C.t1=t1;
C.segment_t0=edges(1:end-1); C.segment_t1=edges(2:end);
C.control_points=cell(1,S);
for s=1:S
    Q=zeros(3,16);
    for j=1:16
        Q(:,j)=scale*[j+0.1*s; (j*j)/17+0.03*s; ((-1)^j)*(j+0.2*s)/9];
    end
    C.control_points{s}=Q;
end
end

function tf=bitwise_r21(a,b)
a=double(a); b=double(b);
tf=isequal(size(a),size(b)) && isequal(typecast(a(:),'uint64'),typecast(b(:),'uint64'));
end

function compile_mex_r21(cppPath,outDir,outName)
assert(isfile(cppPath),'[R21] missing C++ source: %s',cppPath);
old=pwd; c=onCleanup(@()cd(old)); cd(outDir);
if ispc
    mex('-R2018a','COMPFLAGS=$COMPFLAGS /fp:strict',cppPath,'-output',outName,'-outdir',outDir);
else
    mex('-R2018a','CXXFLAGS=$CXXFLAGS -fno-fast-math -ffp-contract=off',cppPath,'-output',outName,'-outdir',outDir);
end
end

function h=sha256_file_r21(p)
fid=fopen(p,'rb'); assert(fid>=0,'[R21] cannot open %s.',p); c=onCleanup(@()fclose(fid));
b=fread(fid,Inf,'*uint8');
md=javaMethod('getInstance','java.security.MessageDigest','SHA-256');
md.update(typecast(b,'int8'));
h=lower(reshape(dec2hex(typecast(md.digest(),'uint8'),2).',1,[]));
end

function write_utf8_r21(p,s)
fid=fopen(p,'w','n','UTF-8'); assert(fid>=0,'[R21] cannot write %s.',p); c=onCleanup(@()fclose(fid)); fwrite(fid,s,'char');
end
