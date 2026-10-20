function AF7R58ZZY_R17_INSTALL
% AF7R58ZZY-R17 transactional install.
% Keeps REPLANNING_SYSTEM.m unchanged. Runner constructor only is switched.
% R17 removes the R16 scalar-nominal batch cost by retaining the exact
% nominal acceleration values already computed during the existing nominal
% table build, then using O(1) table slices in Stage2/postcommit packet work.

EXPECTED_SOURCE_SHA='310cfdd01f9554a80b71e8d90464816a721859709ad7c4b9137cdc0f3a6eaf49';
EXPECTED_PRISTINE_RUNNER_SHA='d4d15d081ea8a6e3073b0a63ce36e47f983777ac2c81db194384de69f5e5fc37';
EXPECTED_HELPER_SHA='1f57a0066acb75859dfe565b3c38a99d5c272b8afae4698131d7aeeb70093889';
EXPECTED_R16_CLASS_SHA='4de7134d283047b650f2a28daaf5ac3f352c29dbb1d4119cad11214d93559ebb';

scriptPath=mfilename('fullpath');
scriptDir=fileparts(scriptPath);
srcPath=which('REPLANNING_SYSTEM');
runnerPath=which('run_flight_headless');
helperPath=which('af7r33_flat_jet_batch_model');
assert(~isempty(srcPath)&&isfile(srcPath),'[R17] REPLANNING_SYSTEM.m not found.');
assert(~isempty(runnerPath)&&isfile(runnerPath),'[R17] run_flight_headless.m not found.');
assert(~isempty(helperPath)&&isfile(helperPath),'[R17] AF7R33 helper not found.');
assert(strcmpi(sha256_file_r17(srcPath),EXPECTED_SOURCE_SHA), ...
    '[R17] production source SHA mismatch.');
assert(strcmpi(sha256_file_r17(helperPath),EXPECTED_HELPER_SHA), ...
    '[R17] AF7R33 helper SHA mismatch.');
root=fileparts(runnerPath);

baseAnchor='replanner_instance = REPLANNING_SYSTEM(agent,nominal_ref,replan_opts);';
r15Anchor='replanner_instance = REPLANNING_SYSTEM_R15_FAST(agent,nominal_ref,replan_opts);';
r16Anchor='replanner_instance = REPLANNING_SYSTEM_R16_FAST(agent,nominal_ref,replan_opts);';
r17Anchor='replanner_instance = REPLANNING_SYSTEM_R17_FAST(agent,nominal_ref,replan_opts);';
runnerText=fileread(runnerPath);

mode='';
if contains(runnerText,r17Anchor)
    assert(count(runnerText,r17Anchor)==1,'[R17] R17 runner anchor is not unique.');
    mode='ALREADY_R17';
elseif contains(runnerText,r16Anchor)
    assert(count(runnerText,r16Anchor)==1,'[R17] R16 runner anchor is not unique.');
    r16ClassPath=which('REPLANNING_SYSTEM_R16_FAST');
    assert(~isempty(r16ClassPath)&&isfile(r16ClassPath), ...
        '[R17] current runner names R16 but the R16 class is missing.');
    assert(strcmpi(sha256_file_r17(r16ClassPath),EXPECTED_R16_CLASS_SHA), ...
        '[R17] current R16 class differs from the R16-R2 authority just measured.');
    mode='FROM_R16';
elseif contains(runnerText,r15Anchor)
    assert(count(runnerText,r15Anchor)==1,'[R17] R15 runner anchor is not unique.');
    mode='FROM_R15';
elseif contains(runnerText,baseAnchor)
    assert(count(runnerText,baseAnchor)==1,'[R17] pristine runner anchor is not unique.');
    assert(strcmpi(sha256_file_r17(runnerPath),EXPECTED_PRISTINE_RUNNER_SHA), ...
        '[R17] pristine runner SHA mismatch.');
    mode='FROM_PRISTINE';
else
    error('[R17] runner is not pristine/R15/R16/R17 authority.');
end

files={ ...
    'REPLANNING_SYSTEM_R17_FAST.m', ...
    'af7r73_posaccel_batch_R17.m', ...
    'af7r73_posaccel_core_R17.m', ...
    'af7r72_posaccel_core_R16.m'};
for k=1:numel(files)
    s=fullfile(scriptDir,files{k});
    assert(isfile(s),'[R17] package file missing: %s',files{k});
    copyfile(s,fullfile(root,files{k}),'f');
end

compile_mex_r17(fullfile(scriptDir,'af7r70_observer_mex.cpp'),root,'af7r70_observer_mex');
compile_mex_r17(fullfile(scriptDir,'af7r71_decasteljau_batch_mex.cpp'),root,'af7r71_decasteljau_batch_mex');
addpath(root,'-begin');
rehash;
clear af7r73_posaccel_batch_R17 af7r73_posaccel_core_R17 af7r72_posaccel_core_R16;
rehash;

assert(exist('af7r70_observer_mex','file')==3,'[R17] observer MEX install failed.');
assert(exist('af7r71_decasteljau_batch_mex','file')==3,'[R17] de Casteljau MEX install failed.');
assert(exist('af7r73_posaccel_batch_R17','file')==2,'[R17] table batch helper install failed.');
assert(exist('af7r73_posaccel_core_R17','file')==2,'[R17] correction core install failed.');
assert(exist('af7r72_posaccel_core_R16','file')==2,'[R17] exact fallback core install failed.');
assert(exist('REPLANNING_SYSTEM_R17_FAST','class')==8,'[R17] class install failed.');
assert(nargin('af7r73_posaccel_batch_R17')==3 && nargout('af7r73_posaccel_batch_R17')==4, ...
    '[R17] table helper signature mismatch.');
assert(nargin('af7r73_posaccel_core_R17')==4 && nargout('af7r73_posaccel_core_R17')==3, ...
    '[R17] correction core signature mismatch.');

selftest_r17_correction_core();
classText=fileread(fullfile(root,'REPLANNING_SYSTEM_R17_FAST.m'));
assert(contains(classText,'build_nominal_reference_table@REPLANNING_SYSTEM'), ...
    '[R17] superclass nominal-table call missing.');
assert(contains(classText,'r17_nominal_accel_table'), ...
    '[R17] nominal acceleration table missing.');
assert(contains(classText,'af7r73_posaccel_batch_R17'), ...
    '[R17] Stage2/postcommit helper connection missing.');
assert(~contains(classText,'af7r72_posaccel_batch_R16'), ...
    '[R17] old R16 scalar batch helper remains connected.');

backupPath=fullfile(root,'run_flight_headless.R17_PREINSTALL_BACKUP.m');
if ~strcmp(mode,'ALREADY_R17')
    assert(~isfile(backupPath),'[R17] R17 runner backup already exists; resolve it before install.');
    copyfile(runnerPath,backupPath);
    switch mode
        case 'FROM_R16'
            patched=strrep(runnerText,r16Anchor,r17Anchor);
        case 'FROM_R15'
            patched=strrep(runnerText,r15Anchor,r17Anchor);
        case 'FROM_PRISTINE'
            patched=strrep(runnerText,baseAnchor,r17Anchor);
        otherwise
            error('[R17] internal runner mode error.');
    end
    write_utf8_r17(runnerPath,patched);
end
rehash;

assert(strcmpi(sha256_file_r17(srcPath),EXPECTED_SOURCE_SHA), ...
    '[R17] production source changed during install.');
assert(strcmpi(sha256_file_r17(helperPath),EXPECTED_HELPER_SHA), ...
    '[R17] original AF7R33 helper changed during install.');
newRunner=fileread(runnerPath);
assert(count(newRunner,r17Anchor)==1,'[R17] runner switch did not close.');
assert(~contains(newRunner,r16Anchor) && ~contains(newRunner,r15Anchor) && ...
    ~contains(newRunner,baseAnchor), ...
    '[R17] an old replanner constructor anchor remains after install.');

manifest=fullfile(root,'AF7R58ZZY_R17_INSTALL_MANIFEST.txt');
fid=fopen(manifest,'w'); assert(fid>=0); c=onCleanup(@()fclose(fid));
fprintf(fid,'status=R17_INSTALLED\n');
fprintf(fid,'installMode=%s\n',mode);
fprintf(fid,'productionSourceModified=0\n');
fprintf(fid,'sourceSHA=%s\n',sha256_file_r17(srcPath));
fprintf(fid,'runnerSHA=%s\n',sha256_file_r17(runnerPath));
fprintf(fid,'originalAF7R33SHA=%s\n',sha256_file_r17(helperPath));
fprintf(fid,'r17ClassSHA=%s\n',sha256_file_r17(fullfile(root,'REPLANNING_SYSTEM_R17_FAST.m')));
fprintf(fid,'r17BatchSHA=%s\n',sha256_file_r17(fullfile(root,'af7r73_posaccel_batch_R17.m')));
fprintf(fid,'r17CoreSHA=%s\n',sha256_file_r17(fullfile(root,'af7r73_posaccel_core_R17.m')));
fprintf(fid,'runnerBackup=%s\n',backupPath);
fprintf(fid,'controllerChanged=0\n');
fprintf(fid,'trajectoryMathChanged=0\n');
fprintf(fid,'certificateAcceptanceChanged=0\n');
fprintf(fid,'commitPolicyChanged=0\n');
fprintf(fid,'workShiftedToArmingTakeoffLanding=0\n');
fprintf(fid,'newNominalDerivativeEvaluationsDuringTableBuild=0\n');
fprintf(fid,'nominalAccelerationRetentionOnly=1\n');
fprintf(fid,'postcacheDerivativeOrders=0,2\n');
fprintf(fid,'stage2LedgerLightweight=1\n');

fprintf('\n[R17] INSTALLED. REPLANNING_SYSTEM.m unchanged.\n');
fprintf('[R17] Runner now uses REPLANNING_SYSTEM_R17_FAST.\n');
fprintf('[R17] Run normally: run_flight_headless\n');
fprintf('[R17] Rollback: AF7R58ZZY_R17_ROLLBACK\n');
end

function selftest_r17_correction_core()
N=17;
t=linspace(0.25,3.0,N);
P=zeros(3,N); A=zeros(3,N);
for k=1:N
    P(:,k)=[1+t(k);2-0.5*t(k);3+0.25*t(k)];
    A(:,k)=[0.1;0.2;-0.3]+k*eps;
end
model=struct('schema','AF7R33_BATCH_FLAT_MODEL_V1','certified',true, ...
    'corrections',struct('t0',{},'t1',{},'segment_t0',{},'segment_t1',{},'control_points',{}));
model.corrections(1)=synthetic_correction_r17(1.0,2.0,0.01);
model.corrections(2)=synthetic_correction_r17(1.5,2.5,-0.02);
[P1,A1,V]=af7r73_posaccel_core_R17(P,A,t,model);
[Pr,Ar]=scalar_correction_reference_r17(P,A,t,model);
assert(all(V) && bitwise_double_r17(P1,Pr) && bitwise_double_r17(A1,Ar), ...
    '[R17] correction-core synthetic bitwise gate failed.');
end

function C=synthetic_correction_r17(t0,t1,scale)
tm=(t0+t1)/2;
C=struct(); C.t0=t0; C.t1=t1;
C.segment_t0=[t0 tm]; C.segment_t1=[tm t1];
C.control_points=cell(1,2);
for s=1:2
    Q=zeros(3,16);
    for j=1:16
        Q(:,j)=scale*[j; j^2/16; (-1)^j*j/8]*(1+0.1*s);
    end
    C.control_points{s}=Q;
end
end

function [P,A]=scalar_correction_reference_r17(P,A,tvec,model)
P=double(P); A=double(A); tvec=double(tvec(:).');
for k=1:numel(tvec)
    tt=tvec(k);
    for g=1:numel(model.corrections)
        C=model.corrections(g);
        if tt<=C.t0 || tt>=C.t1, continue; end
        s0=double(C.segment_t0(:)); s1=double(C.segment_t1(:));
        is=find(tt<=s1,1,'first'); if isempty(is), continue; end
        if tt<s0(is)
            if is==1, continue; end
            is=is-1;
        end
        T=s1(is)-s0(is); u=(tt-s0(is))/T; u=max(0,min(1,u));
        Q0=double(C.control_points{is});
        P(:,k)=P(:,k)+scalar_decasteljau_r17(Q0,u);
        Q1=(15/T)*diff(Q0,1,2); Q2=(14/T)*diff(Q1,1,2);
        A(:,k)=A(:,k)+scalar_decasteljau_r17(Q2,u);
    end
end
end

function y=scalar_decasteljau_r17(Q,u)
Q=double(Q);
while size(Q,2)>1
    Q=(1-u)*Q(:,1:end-1)+u*Q(:,2:end);
end
y=Q(:,1);
end

function tf=bitwise_double_r17(a,b)
a=double(a); b=double(b);
tf=isequal(size(a),size(b)) && ...
    isequal(typecast(a(:),'uint64'),typecast(b(:),'uint64'));
end

function compile_mex_r17(cppPath,outDir,outName)
assert(isfile(cppPath),'[R17] missing C++ source: %s',cppPath);
old=pwd; c=onCleanup(@()cd(old)); cd(outDir);
if ispc
    mex('-R2018a','COMPFLAGS=$COMPFLAGS /fp:strict',cppPath,'-output',outName,'-outdir',outDir);
else
    mex('-R2018a','CXXFLAGS=$CXXFLAGS -fno-fast-math -ffp-contract=off',cppPath,'-output',outName,'-outdir',outDir);
end
end

function h=sha256_file_r17(p)
fid=fopen(p,'rb'); assert(fid>=0,'[R17] cannot open %s.',p); c=onCleanup(@()fclose(fid));
b=fread(fid,Inf,'*uint8');
md=javaMethod('getInstance','java.security.MessageDigest','SHA-256');
md.update(typecast(b,'int8'));
h=lower(reshape(dec2hex(typecast(md.digest(),'uint8'),2).',1,[]));
end

function write_utf8_r17(p,s)
fid=fopen(p,'w','n','UTF-8'); assert(fid>=0,'[R17] cannot write %s.',p); c=onCleanup(@()fclose(fid));
fwrite(fid,s,'char');
end
