function AF7R58ZZY_R19_INSTALL
% AF7R58ZZY-R19 transactional install.
% Purpose: replace R17/R18 MATLAB per-point correction segment mapping with
% one strict-FP native correction evaluator while preserving exact R17
% group accumulation and the existing safety/certificate/commit semantics.
%
% REPLANNING_SYSTEM.m is never modified. Only the runner constructor is
% switched after compile + bitwise self-test close.

EXPECTED_SOURCE_SHA='310cfdd01f9554a80b71e8d90464816a721859709ad7c4b9137cdc0f3a6eaf49';
EXPECTED_HELPER_SHA='1f57a0066acb75859dfe565b3c38a99d5c272b8afae4698131d7aeeb70093889';
EXPECTED_PRISTINE_RUNNER_SHA='d4d15d081ea8a6e3073b0a63ce36e47f983777ac2c81db194384de69f5e5fc37';

scriptDir=fileparts(mfilename('fullpath'));
srcPath=which('REPLANNING_SYSTEM');
runnerPath=which('run_flight_headless');
helperPath=which('af7r33_flat_jet_batch_model');
assert(~isempty(srcPath)&&isfile(srcPath),'[R19] REPLANNING_SYSTEM.m not found.');
assert(~isempty(runnerPath)&&isfile(runnerPath),'[R19] run_flight_headless.m not found.');
assert(~isempty(helperPath)&&isfile(helperPath),'[R19] AF7R33 helper not found.');
assert(strcmpi(sha256_file_r19(srcPath),EXPECTED_SOURCE_SHA),'[R19] production source SHA mismatch.');
assert(strcmpi(sha256_file_r19(helperPath),EXPECTED_HELPER_SHA),'[R19] AF7R33 helper SHA mismatch.');
root=fileparts(runnerPath);

baseAnchor='replanner_instance = REPLANNING_SYSTEM(agent,nominal_ref,replan_opts);';
r15Anchor='replanner_instance = REPLANNING_SYSTEM_R15_FAST(agent,nominal_ref,replan_opts);';
r16Anchor='replanner_instance = REPLANNING_SYSTEM_R16_FAST(agent,nominal_ref,replan_opts);';
r17Anchor='replanner_instance = REPLANNING_SYSTEM_R17_FAST(agent,nominal_ref,replan_opts);';
r18Anchor='replanner_instance = REPLANNING_SYSTEM_R18_STAGE2_DIAG(agent,nominal_ref,replan_opts);';
r19Anchor='replanner_instance = REPLANNING_SYSTEM_R19_FAST(agent,nominal_ref,replan_opts);';
txt=fileread(runnerPath);

if contains(txt,r19Anchor)
    assert(count(txt,r19Anchor)==1,'[R19] R19 runner anchor is not unique.');
    mode='ALREADY_R19'; oldAnchor='';
elseif contains(txt,r18Anchor)
    assert(count(txt,r18Anchor)==1,'[R19] R18 runner anchor is not unique.');
    mode='FROM_R18'; oldAnchor=r18Anchor;
elseif contains(txt,r17Anchor)
    assert(count(txt,r17Anchor)==1,'[R19] R17 runner anchor is not unique.');
    mode='FROM_R17'; oldAnchor=r17Anchor;
elseif contains(txt,r16Anchor)
    assert(count(txt,r16Anchor)==1,'[R19] R16 runner anchor is not unique.');
    mode='FROM_R16'; oldAnchor=r16Anchor;
elseif contains(txt,r15Anchor)
    assert(count(txt,r15Anchor)==1,'[R19] R15 runner anchor is not unique.');
    mode='FROM_R15'; oldAnchor=r15Anchor;
elseif contains(txt,baseAnchor)
    assert(count(txt,baseAnchor)==1,'[R19] pristine runner anchor is not unique.');
    assert(strcmpi(sha256_file_r19(runnerPath),EXPECTED_PRISTINE_RUNNER_SHA),'[R19] pristine runner SHA mismatch.');
    mode='FROM_PRISTINE'; oldAnchor=baseAnchor;
else
    error('[R19] runner is not a recognized pristine/R15/R16/R17/R18/R19 authority.');
end

files={ ...
    'REPLANNING_SYSTEM_R19_FAST.m', ...
    'af7r75_posaccel_batch_R19.m', ...
    'af7r75_posaccel_core_R19.m', ...
    'af7r72_posaccel_core_R16.m', ...
    'af7r73_posaccel_core_R17.m'};
for k=1:numel(files)
    s=fullfile(scriptDir,files{k});
    assert(isfile(s),'[R19] package file missing: %s',files{k});
    copyfile(s,fullfile(root,files{k}),'f');
end

compile_mex_r19(fullfile(scriptDir,'af7r70_observer_mex.cpp'),root,'af7r70_observer_mex');
compile_mex_r19(fullfile(scriptDir,'af7r71_decasteljau_batch_mex.cpp'),root,'af7r71_decasteljau_batch_mex');
compile_mex_r19(fullfile(scriptDir,'af7r75_correction_posaccel_mex.cpp'),root,'af7r75_correction_posaccel_mex');
addpath(root,'-begin'); rehash;
clear af7r75_posaccel_batch_R19 af7r75_posaccel_core_R19 af7r72_posaccel_core_R16 af7r73_posaccel_core_R17;
rehash;

assert(exist('af7r70_observer_mex','file')==3,'[R19] observer MEX install failed.');
assert(exist('af7r71_decasteljau_batch_mex','file')==3,'[R19] de Casteljau MEX install failed.');
assert(exist('af7r75_correction_posaccel_mex','file')==3,'[R19] correction MEX install failed.');
assert(exist('af7r75_posaccel_batch_R19','file')==2,'[R19] batch helper install failed.');
assert(exist('af7r75_posaccel_core_R19','file')==2,'[R19] core helper install failed.');
assert(exist('REPLANNING_SYSTEM_R19_FAST','class')==8,'[R19] class parse/install failed.');
assert(nargin('af7r75_posaccel_batch_R19')==3 && nargout('af7r75_posaccel_batch_R19')==4, ...
    '[R19] batch helper signature mismatch.');
assert(nargin('af7r75_posaccel_core_R19')==4 && nargout('af7r75_posaccel_core_R19')==3, ...
    '[R19] core helper signature mismatch.');

% Fail closed before runner switch unless the new native correction path is
% bitwise-identical to the already-validated R17 correction path.
selftest_r19_native_exact();

classText=fileread(fullfile(root,'REPLANNING_SYSTEM_R19_FAST.m'));
assert(contains(classText,'build_nominal_reference_table@REPLANNING_SYSTEM'),'[R19] nominal-table parent call missing.');
assert(contains(classText,'af7r75_posaccel_batch_R19'),'[R19] native batch helper not connected.');
assert(~contains(classText,'af7r74_posaccel_batch_R18'),'[R19] R18 diagnostic helper leaked into lean class.');

backupPath=fullfile(root,'run_flight_headless.R19_PREINSTALL_BACKUP.m');
if ~strcmp(mode,'ALREADY_R19')
    assert(~isfile(backupPath),'[R19] R19 runner backup already exists; resolve before install.');
    copyfile(runnerPath,backupPath);
    patched=strrep(txt,oldAnchor,r19Anchor);
    assert(count(patched,r19Anchor)==1,'[R19] runner replacement count invalid.');
    write_utf8_r19(runnerPath,patched);
end
rehash;

assert(strcmpi(sha256_file_r19(srcPath),EXPECTED_SOURCE_SHA),'[R19] production source changed during install.');
assert(strcmpi(sha256_file_r19(helperPath),EXPECTED_HELPER_SHA),'[R19] AF7R33 helper changed during install.');
newtxt=fileread(runnerPath);
assert(count(newtxt,r19Anchor)==1,'[R19] runner switch did not close.');
assert(~contains(newtxt,r18Anchor) && ~contains(newtxt,r17Anchor) && ...
    ~contains(newtxt,r16Anchor) && ~contains(newtxt,r15Anchor) && ~contains(newtxt,baseAnchor), ...
    '[R19] old constructor anchor remains after install.');

manifest=fullfile(root,'AF7R58ZZY_R19_INSTALL_MANIFEST.txt');
fid=fopen(manifest,'w'); assert(fid>=0); c=onCleanup(@()fclose(fid));
fprintf(fid,'status=R19_INSTALLED\n');
fprintf(fid,'installMode=%s\n',mode);
fprintf(fid,'productionSourceModified=0\n');
fprintf(fid,'sourceSHA=%s\n',sha256_file_r19(srcPath));
fprintf(fid,'runnerSHA=%s\n',sha256_file_r19(runnerPath));
fprintf(fid,'r19ClassSHA=%s\n',sha256_file_r19(fullfile(root,'REPLANNING_SYSTEM_R19_FAST.m')));
fprintf(fid,'r19BatchSHA=%s\n',sha256_file_r19(fullfile(root,'af7r75_posaccel_batch_R19.m')));
fprintf(fid,'r19CoreSHA=%s\n',sha256_file_r19(fullfile(root,'af7r75_posaccel_core_R19.m')));
fprintf(fid,'runnerBackup=%s\n',backupPath);
fprintf(fid,'controllerChanged=0\ntrajectoryMathChanged=0\ncertificateAcceptanceChanged=0\ncommitPolicyChanged=0\n');
fprintf(fid,'workShiftedToArmingTakeoffLanding=0\n');
fprintf(fid,'nativeCorrectionOnly=1\nnominalNativeEvaluatorUsed=0\n');
fprintf(fid,'r17GroupAccumulationOrderPreserved=1\n');

fprintf('\n[R19] INSTALLED | mode=%s\n',mode);
fprintf('[R19] Native correction exactness self-test CLOSED before runner switch.\n');
fprintf('[R19] AF7R36 op=2 is NOT used; nominal-table bitwise authority is preserved.\n');
fprintf('[R19] Run normally: run_flight_headless\n');
fprintf('[R19] Rollback: AF7R58ZZY_R19_ROLLBACK\n');
end

function selftest_r19_native_exact()
% Multiple groups, overlapping supports, multiple segments, exact boundaries,
% negative-zero nominal entries and nonuniform deterministic control points.
t=[0.25 0.5 0.999999 1.0 1.125 1.25 1.5 1.75 2.0 2.125 2.25 2.5 2.75 3.0 3.000001 3.4];
N=numel(t);
P=zeros(3,N); A=zeros(3,N);
for k=1:N
    P(:,k)=[1+t(k); -0.0; 3-0.125*t(k)];
    A(:,k)=[0.1+k*eps; -0.2; 0.3-2*k*eps];
end
model=struct('schema','AF7R33_BATCH_FLAT_MODEL_V1','certified',true, ...
    'corrections',struct('t0',{},'t1',{},'segment_t0',{},'segment_t1',{},'control_points',{}));
model.corrections(1)=synthetic_correction_r19(1.0,2.5, 0.013,3);
model.corrections(2)=synthetic_correction_r19(1.5,3.0,-0.021,2);
model.corrections(3)=synthetic_correction_r19(2.0,3.4, 0.007,4);

[Pr,Ar,Vr]=af7r73_posaccel_core_R17(P,A,t,model);
[Pn,An,Vn]=af7r75_posaccel_core_R19(P,A,t,model);
assert(isequal(Vr,Vn) && bitwise_r19(Pr,Pn) && bitwise_r19(Ar,An), ...
    '[R19] native correction synthetic bitwise gate failed.');

% Repeat with shifted/scaled supports to exercise different mapping patterns.
for c=1:8
    tt=linspace(0.1*c,4.0+0.07*c,53+c);
    PP=zeros(3,numel(tt)); AA=zeros(3,numel(tt));
    for k=1:numel(tt)
        PP(:,k)=[0.2*c+tt(k); -1+0.03*k; 2-0.17*tt(k)];
        AA(:,k)=[0.01*k; -0.02*c; 0.04+eps*k];
    end
    mm=struct('schema','AF7R33_BATCH_FLAT_MODEL_V1','certified',true, ...
        'corrections',struct('t0',{},'t1',{},'segment_t0',{},'segment_t1',{},'control_points',{}));
    mm.corrections(1)=synthetic_correction_r19(0.5+0.03*c,2.6+0.02*c,0.004*c,2+mod(c,3));
    mm.corrections(2)=synthetic_correction_r19(1.1+0.01*c,3.5+0.01*c,-0.003*c,3);
    [P1,A1,V1]=af7r73_posaccel_core_R17(PP,AA,tt,mm);
    [P2,A2,V2]=af7r75_posaccel_core_R19(PP,AA,tt,mm);
    assert(isequal(V1,V2) && bitwise_r19(P1,P2) && bitwise_r19(A1,A2), ...
        '[R19] native correction multi-case bitwise gate failed at case %d.',c);
end
end

function C=synthetic_correction_r19(t0,t1,scale,S)
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

function tf=bitwise_r19(a,b)
a=double(a); b=double(b);
tf=isequal(size(a),size(b)) && isequal(typecast(a(:),'uint64'),typecast(b(:),'uint64'));
end

function compile_mex_r19(cppPath,outDir,outName)
assert(isfile(cppPath),'[R19] missing C++ source: %s',cppPath);
old=pwd; c=onCleanup(@()cd(old)); cd(outDir);
if ispc
    mex('-R2018a','COMPFLAGS=$COMPFLAGS /fp:strict',cppPath,'-output',outName,'-outdir',outDir);
else
    mex('-R2018a','CXXFLAGS=$CXXFLAGS -fno-fast-math -ffp-contract=off',cppPath,'-output',outName,'-outdir',outDir);
end
end

function h=sha256_file_r19(p)
fid=fopen(p,'rb'); assert(fid>=0,'[R19] cannot open %s.',p); c=onCleanup(@()fclose(fid));
b=fread(fid,Inf,'*uint8');
md=javaMethod('getInstance','java.security.MessageDigest','SHA-256');
md.update(typecast(b,'int8'));
h=lower(reshape(dec2hex(typecast(md.digest(),'uint8'),2).',1,[]));
end

function write_utf8_r19(p,s)
fid=fopen(p,'w','n','UTF-8'); assert(fid>=0,'[R19] cannot write %s.',p); c=onCleanup(@()fclose(fid)); fwrite(fid,s,'char');
end
