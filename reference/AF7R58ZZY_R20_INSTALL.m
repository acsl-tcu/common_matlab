function AF7R58ZZY_R20_INSTALL
% AF7R58ZZY-R20 lean Stage2 installer.
% Source-bound, fail-closed, runner switch occurs only after compile/self-tests.

EXPECTED_SOURCE_SHA='310cfdd01f9554a80b71e8d90464816a721859709ad7c4b9137cdc0f3a6eaf49';
EXPECTED_HELPER_SHA='1f57a0066acb75859dfe565b3c38a99d5c272b8afae4698131d7aeeb70093889';
EXPECTED_PREPARE_SHA='92f093168f1df95661c5e7f2e75005fe439406eb3ec22e53fd1f25d6bdfb8a15';
EXPECTED_PRISTINE_RUNNER_SHA='d4d15d081ea8a6e3073b0a63ce36e47f983777ac2c81db194384de69f5e5fc37';

scriptDir=fileparts(mfilename('fullpath'));
srcPath=which('REPLANNING_SYSTEM');
runnerPath=which('run_flight_headless');
helperPath=which('af7r33_flat_jet_batch_model');
preparePath=which('af7r58i_prepare_batch_model');
assert(~isempty(srcPath)&&isfile(srcPath),'[R20] REPLANNING_SYSTEM.m not found.');
assert(~isempty(runnerPath)&&isfile(runnerPath),'[R20] run_flight_headless.m not found.');
assert(~isempty(helperPath)&&isfile(helperPath),'[R20] AF7R33 helper not found.');
assert(~isempty(preparePath)&&isfile(preparePath),'[R20] af7r58i_prepare_batch_model.m not found.');
assert(strcmpi(sha256_file_r20(srcPath),EXPECTED_SOURCE_SHA),'[R20] production source SHA mismatch.');
assert(strcmpi(sha256_file_r20(helperPath),EXPECTED_HELPER_SHA),'[R20] AF7R33 helper SHA mismatch.');
assert(strcmpi(sha256_file_r20(preparePath),EXPECTED_PREPARE_SHA),'[R20] prepare-batch source SHA mismatch.');
root=fileparts(runnerPath);

baseAnchor='replanner_instance = REPLANNING_SYSTEM(agent,nominal_ref,replan_opts);';
r15Anchor='replanner_instance = REPLANNING_SYSTEM_R15_FAST(agent,nominal_ref,replan_opts);';
r16Anchor='replanner_instance = REPLANNING_SYSTEM_R16_FAST(agent,nominal_ref,replan_opts);';
r17Anchor='replanner_instance = REPLANNING_SYSTEM_R17_FAST(agent,nominal_ref,replan_opts);';
r18Anchor='replanner_instance = REPLANNING_SYSTEM_R18_STAGE2_DIAG(agent,nominal_ref,replan_opts);';
r19Anchor='replanner_instance = REPLANNING_SYSTEM_R19_FAST(agent,nominal_ref,replan_opts);';
r20Anchor='replanner_instance = REPLANNING_SYSTEM_R20_FAST(agent,nominal_ref,replan_opts);';
txt=fileread(runnerPath);

if contains(txt,r20Anchor)
    assert(count(txt,r20Anchor)==1,'[R20] R20 runner anchor is not unique.');
    mode='ALREADY_R20'; oldAnchor='';
elseif contains(txt,r19Anchor)
    assert(count(txt,r19Anchor)==1,'[R20] R19 runner anchor is not unique.');
    mode='FROM_R19'; oldAnchor=r19Anchor;
elseif contains(txt,r18Anchor)
    assert(count(txt,r18Anchor)==1,'[R20] R18 runner anchor is not unique.');
    mode='FROM_R18'; oldAnchor=r18Anchor;
elseif contains(txt,r17Anchor)
    assert(count(txt,r17Anchor)==1,'[R20] R17 runner anchor is not unique.');
    mode='FROM_R17'; oldAnchor=r17Anchor;
elseif contains(txt,r16Anchor)
    assert(count(txt,r16Anchor)==1,'[R20] R16 runner anchor is not unique.');
    mode='FROM_R16'; oldAnchor=r16Anchor;
elseif contains(txt,r15Anchor)
    assert(count(txt,r15Anchor)==1,'[R20] R15 runner anchor is not unique.');
    mode='FROM_R15'; oldAnchor=r15Anchor;
elseif contains(txt,baseAnchor)
    assert(count(txt,baseAnchor)==1,'[R20] pristine runner anchor is not unique.');
    assert(strcmpi(sha256_file_r20(runnerPath),EXPECTED_PRISTINE_RUNNER_SHA),'[R20] pristine runner SHA mismatch.');
    mode='FROM_PRISTINE'; oldAnchor=baseAnchor;
else
    error('[R20] runner is not a recognized pristine/R15/R16/R17/R18/R19/R20 authority.');
end

files={ ...
    'REPLANNING_SYSTEM_R19_FAST.m', ...
    'REPLANNING_SYSTEM_R20_FAST.m', ...
    'af7r72_posaccel_core_R16.m', ...
    'af7r73_posaccel_core_R17.m', ...
    'af7r75_posaccel_batch_R19.m', ...
    'af7r75_posaccel_core_R19.m', ...
    'af7r76_postcache_R20.m'};
for k=1:numel(files)
    s=fullfile(scriptDir,files{k});
    assert(isfile(s),'[R20] package file missing: %s',files{k});
    copyfile(s,fullfile(root,files{k}),'f');
end

% Generate the cached-contract helper directly from the exact production
% prepare-batch source. The ONLY semantic change is replacing the repeated
% resource-contract computation with an explicit already-closed RC argument.
Ptxt=fileread(preparePath);
oldSig='function [ok,reason,model,meta] = af7r58i_prepare_batch_model(replanner,P,targetVersion,t_commit)';
newSig='function [ok,reason,model,meta] = af7r76_prepare_batch_model_R20(replanner,P,targetVersion,t_commit,RC)';
assert(count(Ptxt,oldSig)==1,'[R20] prepare-batch function signature changed.');
Ptxt=strrep(Ptxt,oldSig,newSig);
oldRC='RC=af7r58i_resource_contract(replanner);';
newRC='% R20: RC is the exact cached closed contract for the same frozen time authority.';
assert(count(Ptxt,oldRC)==1,'[R20] resource-contract call count changed.');
Ptxt=strrep(Ptxt,oldRC,newRC);
write_utf8_r20(fullfile(root,'af7r76_prepare_batch_model_R20.m'),Ptxt);

compile_mex_r20(fullfile(scriptDir,'af7r70_observer_mex.cpp'),root,'af7r70_observer_mex');
compile_mex_r20(fullfile(scriptDir,'af7r71_decasteljau_batch_mex.cpp'),root,'af7r71_decasteljau_batch_mex');
compile_mex_r20(fullfile(scriptDir,'af7r75_correction_posaccel_mex.cpp'),root,'af7r75_correction_posaccel_mex');
compile_mex_r20(fullfile(scriptDir,'af7r76_apply_correction_mex.cpp'),root,'af7r76_apply_correction_mex');
addpath(root,'-begin'); rehash;
clear af7r76_postcache_R20 af7r76_prepare_batch_model_R20;
rehash;

assert(exist('af7r70_observer_mex','file')==3,'[R20] observer MEX install failed.');
assert(exist('af7r71_decasteljau_batch_mex','file')==3,'[R20] de Casteljau MEX install failed.');
assert(exist('af7r75_correction_posaccel_mex','file')==3,'[R20] R19 correction MEX install failed.');
assert(exist('af7r76_apply_correction_mex','file')==3,'[R20] direct correction MEX install failed.');
assert(exist('af7r76_postcache_R20','file')==2,'[R20] postcache helper install failed.');
assert(exist('af7r76_prepare_batch_model_R20','file')==2,'[R20] cached batch-model helper install failed.');
assert(nargin('af7r76_prepare_batch_model_R20')==5,'[R20] cached batch-model helper signature mismatch.');
assert(exist('REPLANNING_SYSTEM_R19_FAST','class')==8,'[R20] R19 parent class parse/install failed.');
assert(exist('REPLANNING_SYSTEM_R20_FAST','class')==8,'[R20] R20 class parse/install failed.');

% Fail closed before runner switch unless direct native accumulation is
% bitwise-identical to the R19 group-order MATLAB accumulation authority.
selftest_r20_native_exact();

classText=fileread(fullfile(root,'REPLANNING_SYSTEM_R20_FAST.m'));
assert(contains(classText,'af7r76_prepare_batch_model_R20'),'[R20] cached resource helper not connected.');
assert(contains(classText,'af7r76_postcache_R20'),'[R20] direct postcache helper not connected.');
assert(~contains(classText,'af7r36_single_core_mex(2'),'[R20] rejected AF7R36 op=2 leaked into class.');
assert(contains(classText,'r20_rebuild_runtime_packet_from_fresh_stage'),'[R20] fresh packet path missing.');

backupPath=fullfile(root,'run_flight_headless.R20_PREINSTALL_BACKUP.m');
if ~strcmp(mode,'ALREADY_R20')
    assert(~isfile(backupPath),'[R20] R20 runner backup already exists; resolve before install.');
    copyfile(runnerPath,backupPath);
    patched=strrep(txt,oldAnchor,r20Anchor);
    assert(count(patched,r20Anchor)==1,'[R20] runner replacement count invalid.');
    write_utf8_r20(runnerPath,patched);
end
rehash;

assert(strcmpi(sha256_file_r20(srcPath),EXPECTED_SOURCE_SHA),'[R20] production source changed during install.');
assert(strcmpi(sha256_file_r20(helperPath),EXPECTED_HELPER_SHA),'[R20] AF7R33 helper changed during install.');
assert(strcmpi(sha256_file_r20(preparePath),EXPECTED_PREPARE_SHA),'[R20] prepare-batch source changed during install.');
newtxt=fileread(runnerPath);
assert(count(newtxt,r20Anchor)==1,'[R20] runner switch did not close.');

manifest=fullfile(root,'AF7R58ZZY_R20_INSTALL_MANIFEST.txt');
fid=fopen(manifest,'w'); assert(fid>=0); c=onCleanup(@()fclose(fid));
fprintf(fid,'status=R20_INSTALLED\n');
fprintf(fid,'installMode=%s\n',mode);
fprintf(fid,'productionSourceModified=0\n');
fprintf(fid,'sourceSHA=%s\n',sha256_file_r20(srcPath));
fprintf(fid,'prepareSourceSHA=%s\n',sha256_file_r20(preparePath));
fprintf(fid,'runnerSHA=%s\n',sha256_file_r20(runnerPath));
fprintf(fid,'r20ClassSHA=%s\n',sha256_file_r20(fullfile(root,'REPLANNING_SYSTEM_R20_FAST.m')));
fprintf(fid,'r20PostcacheSHA=%s\n',sha256_file_r20(fullfile(root,'af7r76_postcache_R20.m')));
fprintf(fid,'r20CachedBatchSHA=%s\n',sha256_file_r20(fullfile(root,'af7r76_prepare_batch_model_R20.m')));
fprintf(fid,'runnerBackup=%s\n',backupPath);
fprintf(fid,'controllerChanged=0\ntrajectoryMathChanged=0\ncertificateAcceptanceChanged=0\ncommitPolicyChanged=0\n');
fprintf(fid,'resourceContractMemoized=1\nreferenceGateStructRemovedFromHotPath=1\n');
fprintf(fid,'freshPacketSpeedDependentRebuild=1\ndirectCorrectionAccumulation=1\n');
fprintf(fid,'AF7R36op2Used=0\n');

fprintf('\n[R20] INSTALLED | mode=%s\n',mode);
fprintf('[R20] Direct-correction bitwise self-test CLOSED before runner switch.\n');
fprintf('[R20] Cached batch helper generated from exact production source SHA.\n');
fprintf('[R20] REPLANNING_SYSTEM.m/controller/HLC/plant unchanged.\n');
fprintf('[R20] Run normally: run_flight_headless\n');
fprintf('[R20] Rollback: AF7R58ZZY_R20_ROLLBACK\n');
end

function selftest_r20_native_exact()
t=[0.25 0.5 0.999999 1.0 1.125 1.25 1.5 1.75 2.0 2.125 2.25 2.5 2.75 3.0 3.000001 3.4];
N=numel(t);
P=zeros(3,N); A=zeros(3,N);
for k=1:N
    P(:,k)=[1+t(k); -0.0; 3-0.125*t(k)];
    A(:,k)=[0.1+k*eps; -0.2; 0.3-2*k*eps];
end
model=struct('schema','AF7R33_BATCH_FLAT_MODEL_V1','certified',true, ...
    'corrections',struct('t0',{},'t1',{},'segment_t0',{},'segment_t1',{},'control_points',{}));
model.corrections(1)=synthetic_correction_r20(1.0,2.5, 0.013,3);
model.corrections(2)=synthetic_correction_r20(1.5,3.0,-0.021,2);
model.corrections(3)=synthetic_correction_r20(2.0,3.4, 0.007,4);
[P19,A19,V19]=af7r75_posaccel_core_R19(P,A,t,model);
[P20,A20,V20]=af7r76_apply_correction_mex(P,A,t,model.corrections);
assert(isequal(V19,V20) && bitwise_r20(P19,P20) && bitwise_r20(A19,A20), ...
    '[R20] direct-accumulation synthetic bitwise gate failed.');

for c=1:12
    tt=linspace(0.07*c,4.2+0.031*c,47+c);
    PP=zeros(3,numel(tt)); AA=zeros(3,numel(tt));
    for k=1:numel(tt)
        PP(:,k)=[0.2*c+tt(k); -1+0.03*k; 2-0.17*tt(k)];
        AA(:,k)=[0.01*k; -0.02*c; 0.04+eps*k];
    end
    mm=struct('schema','AF7R33_BATCH_FLAT_MODEL_V1','certified',true, ...
        'corrections',struct('t0',{},'t1',{},'segment_t0',{},'segment_t1',{},'control_points',{}));
    mm.corrections(1)=synthetic_correction_r20(0.5+0.03*c,2.6+0.02*c,0.004*c,2+mod(c,3));
    mm.corrections(2)=synthetic_correction_r20(1.1+0.01*c,3.5+0.01*c,-0.003*c,3);
    if mod(c,2)==0
        mm.corrections(3)=synthetic_correction_r20(1.7,4.0,0.0017*c,4);
    end
    [P1,A1,V1]=af7r75_posaccel_core_R19(PP,AA,tt,mm);
    [P2,A2,V2]=af7r76_apply_correction_mex(PP,AA,tt,mm.corrections);
    assert(isequal(V1,V2) && bitwise_r20(P1,P2) && bitwise_r20(A1,A2), ...
        '[R20] direct-accumulation multi-case bitwise gate failed at case %d.',c);
end
end

function C=synthetic_correction_r20(t0,t1,scale,S)
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

function tf=bitwise_r20(a,b)
a=double(a); b=double(b);
tf=isequal(size(a),size(b)) && isequal(typecast(a(:),'uint64'),typecast(b(:),'uint64'));
end

function compile_mex_r20(cppPath,outDir,outName)
assert(isfile(cppPath),'[R20] missing C++ source: %s',cppPath);
old=pwd; c=onCleanup(@()cd(old)); cd(outDir);
if ispc
    mex('-R2018a','COMPFLAGS=$COMPFLAGS /fp:strict',cppPath,'-output',outName,'-outdir',outDir);
else
    mex('-R2018a','CXXFLAGS=$CXXFLAGS -fno-fast-math -ffp-contract=off',cppPath,'-output',outName,'-outdir',outDir);
end
end

function h=sha256_file_r20(p)
fid=fopen(p,'rb'); assert(fid>=0,'[R20] cannot open %s.',p); c=onCleanup(@()fclose(fid));
b=fread(fid,Inf,'*uint8');
md=javaMethod('getInstance','java.security.MessageDigest','SHA-256');
md.update(typecast(b,'int8'));
h=lower(reshape(dec2hex(typecast(md.digest(),'uint8'),2).',1,[]));
end

function write_utf8_r20(p,s)
fid=fopen(p,'w','n','UTF-8'); assert(fid>=0,'[R20] cannot write %s.',p); c=onCleanup(@()fclose(fid)); fwrite(fid,s,'char');
end
