function AF7R58ZZY_R16_R2_INSTALL
% AF7R58ZZY-R16-R2 transactional Stage2/postcommit exact-performance install.
% REPLANNING_SYSTEM.m itself is never modified.  Only the runner constructor
% is switched to the direct subclass REPLANNING_SYSTEM_R16_FAST.

EXPECTED_SOURCE_SHA='310cfdd01f9554a80b71e8d90464816a721859709ad7c4b9137cdc0f3a6eaf49';
EXPECTED_PRISTINE_RUNNER_SHA='d4d15d081ea8a6e3073b0a63ce36e47f983777ac2c81db194384de69f5e5fc37';
EXPECTED_HELPER_SHA='1f57a0066acb75859dfe565b3c38a99d5c272b8afae4698131d7aeeb70093889';

scriptPath=mfilename('fullpath');
scriptDir=fileparts(scriptPath);
srcPath=which('REPLANNING_SYSTEM');
runnerPath=which('run_flight_headless');
helperPath=which('af7r33_flat_jet_batch_model');
assert(~isempty(srcPath)&&isfile(srcPath),'[R16-R2] REPLANNING_SYSTEM.m not found.');
assert(~isempty(runnerPath)&&isfile(runnerPath),'[R16-R2] run_flight_headless.m not found.');
assert(~isempty(helperPath)&&isfile(helperPath),'[R16-R2] AF7R33 helper not found.');
assert(strcmpi(sha256_file_r16(srcPath),EXPECTED_SOURCE_SHA), ...
    '[R16-R2] production source SHA mismatch.');
assert(strcmpi(sha256_file_r16(helperPath),EXPECTED_HELPER_SHA), ...
    '[R16-R2] AF7R33 helper SHA mismatch.');
root=fileparts(runnerPath);

baseAnchor='replanner_instance = REPLANNING_SYSTEM(agent,nominal_ref,replan_opts);';
r15Anchor='replanner_instance = REPLANNING_SYSTEM_R15_FAST(agent,nominal_ref,replan_opts);';
r16Anchor='replanner_instance = REPLANNING_SYSTEM_R16_FAST(agent,nominal_ref,replan_opts);';
runnerText=fileread(runnerPath);

% Accept only the pristine runner, the immediately preceding R15 install,
% or an already-installed R16 runner.  Any unrelated edit fails closed.
mode='';
if contains(runnerText,r16Anchor)
    assert(count(runnerText,r16Anchor)==1,'[R16-R2] R16 runner anchor is not unique.');
    mode='ALREADY_R16';
elseif contains(runnerText,r15Anchor)
    assert(count(runnerText,r15Anchor)==1,'[R16-R2] R15 runner anchor is not unique.');
    r15Backup=fullfile(root,'run_flight_headless.R15_PREINSTALL_BACKUP.m');
    assert(isfile(r15Backup),'[R16-R2] R15 pristine-runner backup is missing.');
    assert(strcmpi(sha256_file_r16(r15Backup),EXPECTED_PRISTINE_RUNNER_SHA), ...
        '[R16-R2] R15 backup does not match the authoritative pristine runner.');
    mode='FROM_R15';
elseif contains(runnerText,baseAnchor)
    assert(count(runnerText,baseAnchor)==1,'[R16-R2] pristine runner anchor is not unique.');
    assert(strcmpi(sha256_file_r16(runnerPath),EXPECTED_PRISTINE_RUNNER_SHA), ...
        '[R16-R2] pristine runner SHA mismatch.');
    mode='FROM_PRISTINE';
else
    error('[R16-R2] runner is neither pristine, R15-installed, nor R16-installed.');
end

% Install direct subclass + reduced r=0/r=2 evaluator.
classSrc=fullfile(scriptDir,'REPLANNING_SYSTEM_R16_FAST.m');
helperSrc=fullfile(scriptDir,'af7r72_posaccel_batch_R16.m');
coreSrc=fullfile(scriptDir,'af7r72_posaccel_core_R16.m');
assert(isfile(classSrc)&&isfile(helperSrc)&&isfile(coreSrc),'[R16-R2] package source is incomplete.');
classDst=fullfile(root,'REPLANNING_SYSTEM_R16_FAST.m');
helperDst=fullfile(root,'af7r72_posaccel_batch_R16.m');
coreDst=fullfile(root,'af7r72_posaccel_core_R16.m');
copyfile(classSrc,classDst,'f');
copyfile(helperSrc,helperDst,'f');
copyfile(coreSrc,coreDst,'f');

% Compile the two already validated strict-FP kernels used by R16.
compile_mex_r16(fullfile(scriptDir,'af7r70_observer_mex.cpp'),root,'af7r70_observer_mex');
compile_mex_r16(fullfile(scriptDir,'af7r71_decasteljau_batch_mex.cpp'),root,'af7r71_decasteljau_batch_mex');
addpath(root,'-begin');
rehash;
clear af7r72_posaccel_batch_R16 af7r72_posaccel_core_R16 af7r70_observer_mex af7r71_decasteljau_batch_mex;
rehash;
assert(exist('af7r70_observer_mex','file')==3,'[R16-R2] observer MEX install failed.');
assert(exist('af7r71_decasteljau_batch_mex','file')==3,'[R16-R2] de Casteljau MEX install failed.');
assert(exist('af7r72_posaccel_batch_R16','file')==2,'[R16-R2] reduced flat-jet helper install failed.');
assert(exist('af7r72_posaccel_core_R16','file')==2,'[R16-R2] reduced flat-jet core install failed.');
assert(exist('REPLANNING_SYSTEM_R16_FAST','class')==8,'[R16-R2] class install failed.');

% Basic parse/signature checks before touching the runner.
assert(nargin('af7r72_posaccel_batch_R16')==3,'[R16-R2] reduced helper signature mismatch.');
assert(nargout('af7r72_posaccel_batch_R16')==3,'[R16-R2] reduced helper output signature mismatch.');
assert(nargin('af7r72_posaccel_core_R16')==5 && nargout('af7r72_posaccel_core_R16')==3, ...
    '[R16-R2] reduced core signature mismatch.');
selftest_reduced_core_r16();

backupPath=fullfile(root,'run_flight_headless.R16_PREINSTALL_BACKUP.m');
if ~strcmp(mode,'ALREADY_R16')
    assert(~isfile(backupPath),'[R16-R2] R16 backup already exists; resolve it before install.');
    copyfile(runnerPath,backupPath);
    switch mode
        case 'FROM_R15'
            patched=strrep(runnerText,r15Anchor,r16Anchor);
        case 'FROM_PRISTINE'
            patched=strrep(runnerText,baseAnchor,r16Anchor);
        otherwise
            error('[R16-R2] internal runner mode error.');
    end
    write_utf8_r16(runnerPath,patched);
end
rehash;

assert(strcmpi(sha256_file_r16(srcPath),EXPECTED_SOURCE_SHA), ...
    '[R16-R2] production source changed during install.');
assert(strcmpi(sha256_file_r16(helperPath),EXPECTED_HELPER_SHA), ...
    '[R16-R2] original AF7R33 helper changed during install.');
newRunner=fileread(runnerPath);
assert(count(newRunner,r16Anchor)==1, '[R16-R2] runner switch did not close.');
assert(~contains(newRunner,r15Anchor) && ~contains(newRunner,baseAnchor), ...
    '[R16-R2] old runner constructor anchor remains after install.');

manifest=fullfile(root,'AF7R58ZZY_R16_R2_INSTALL_MANIFEST.txt');
fid=fopen(manifest,'w'); assert(fid>=0); c=onCleanup(@()fclose(fid));
fprintf(fid,'status=R16_R2_INSTALLED\n');
fprintf(fid,'installMode=%s\n',mode);
fprintf(fid,'productionSourceModified=0\n');
fprintf(fid,'sourceSHA=%s\n',sha256_file_r16(srcPath));
fprintf(fid,'runnerSHA=%s\n',sha256_file_r16(runnerPath));
fprintf(fid,'originalAF7R33SHA=%s\n',sha256_file_r16(helperPath));
fprintf(fid,'r16R2ClassSHA=%s\n',sha256_file_r16(classDst));
fprintf(fid,'r16R2ReducedHelperSHA=%s\n',sha256_file_r16(helperDst));
fprintf(fid,'r16R2ReducedCoreSHA=%s\n',sha256_file_r16(coreDst));
fprintf(fid,'observerCppSHA=%s\n',sha256_file_r16(fullfile(scriptDir,'af7r70_observer_mex.cpp')));
fprintf(fid,'decasteljauCppSHA=%s\n',sha256_file_r16(fullfile(scriptDir,'af7r71_decasteljau_batch_mex.cpp')));
fprintf(fid,'runnerBackup=%s\n',backupPath);
fprintf(fid,'controllerChanged=0\n');
fprintf(fid,'trajectoryMathChanged=0\n');
fprintf(fid,'certificateAcceptanceChanged=0\n');
fprintf(fid,'commitPolicyChanged=0\n');
fprintf(fid,'workShiftedToArmingTakeoffLanding=0\n');
fprintf(fid,'stage2ExactRuntimePacketReuse=1\n');
fprintf(fid,'postcacheDerivativeOrders=0,2\n');
fprintf(fid,'nominalFlatEvaluation=SCALAR_TIME_AUTHORITY\n');

fprintf('\n[R16-R2] INSTALLED. REPLANNING_SYSTEM.m unchanged.\n');
fprintf('[R16-R2] Runner now uses REPLANNING_SYSTEM_R16_FAST.\n');
fprintf('[R16-R2] Run normally: run_flight_headless\n');
fprintf('[R16-R2] Rollback: AF7R58ZZY_R16_R2_ROLLBACK\n');
end

function selftest_reduced_core_r16()
% Independent synthetic bitwise gate for the exact r={0,2} restriction.
flat0=@(t)[0.0;0.0;t.*1.5+3.0];
flat2=@(t)[0.0;0.0;0.0.*t];
model=struct();
model.schema='AF7R33_BATCH_FLAT_MODEL_V1';
model.certified=true;
model.flight_reference_time_origin=0.25;
model.corrections=struct('t0',{},'t1',{},'segment_t0',{},'segment_t1',{},'control_points',{});
model.corrections(1)=synthetic_correction_r16(1.0,2.0,0.01);
model.corrections(2)=synthetic_correction_r16(1.5,2.5,-0.02);
t=[0.25 0.999999 1.0 1.2 1.5 1.75 2.0 2.2 2.5 2.500001 3.0];
[P,A,V]=af7r72_posaccel_core_R16(flat0,flat2,0.25,t,model);
[Pr,Ar]=scalar_reference_r16(flat0,flat2,0.25,t,model);
assert(all(V) && bitwise_double_r16(P,Pr) && bitwise_double_r16(A,Ar), ...
    '[R16-R2] scalar-authority r={0,2} synthetic bitwise gate failed.');
end

function C=synthetic_correction_r16(t0,t1,scale)
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

function [P,A]=scalar_reference_r16(flat0,flat2,tf0,tvec,model)
tvec=double(tvec(:).'); N=numel(tvec); P=zeros(3,N); A=zeros(3,N);
for k=1:N
    tau=tvec(k)-tf0; if tau<0, tau=0; end
    p=double(flat0(tau)); a=double(flat2(tau)); p=p(:); a=a(:);
    for g=1:numel(model.corrections)
        C=model.corrections(g); tt=tvec(k);
        if tt<=C.t0 || tt>=C.t1, continue; end
        s0=double(C.segment_t0(:)); s1=double(C.segment_t1(:));
        is=find(tt<=s1,1,'first'); if isempty(is), continue; end
        if tt<s0(is)
            if is==1, continue; end
            is=is-1;
        end
        T=s1(is)-s0(is); u=(tt-s0(is))/T; u=max(0,min(1,u));
        Q0=double(C.control_points{is});
        p=p+scalar_decasteljau_r16(Q0,u);
        Q1=(15/T)*diff(Q0,1,2); Q2=(14/T)*diff(Q1,1,2);
        a=a+scalar_decasteljau_r16(Q2,u);
    end
    P(:,k)=p; A(:,k)=a;
end
end

function y=scalar_decasteljau_r16(Q,u)
Q=double(Q);
while size(Q,2)>1
    Q=(1-u)*Q(:,1:end-1)+u*Q(:,2:end);
end
y=Q(:,1);
end

function tf=bitwise_double_r16(a,b)
a=double(a); b=double(b);
tf=isequal(size(a),size(b)) && ...
    isequal(typecast(a(:),'uint64'),typecast(b(:),'uint64'));
end

function compile_mex_r16(cppPath,outDir,outName)
assert(isfile(cppPath),'[R16-R2] missing C++ source: %s',cppPath);
old=pwd; c=onCleanup(@()cd(old)); cd(outDir);
if ispc
    mex('-R2018a','COMPFLAGS=$COMPFLAGS /fp:strict',cppPath,'-output',outName,'-outdir',outDir);
else
    mex('-R2018a','CXXFLAGS=$CXXFLAGS -fno-fast-math -ffp-contract=off',cppPath,'-output',outName,'-outdir',outDir);
end
end

function h=sha256_file_r16(p)
fid=fopen(p,'rb'); assert(fid>=0,'[R16-R2] cannot open %s.',p); c=onCleanup(@()fclose(fid));
b=fread(fid,Inf,'*uint8');
md=javaMethod('getInstance','java.security.MessageDigest','SHA-256');
md.update(typecast(b,'int8'));
h=lower(reshape(dec2hex(typecast(md.digest(),'uint8'),2).',1,[]));
end

function write_utf8_r16(p,s)
fid=fopen(p,'w','n','UTF-8'); assert(fid>=0,'[R16-R2] cannot write %s.',p); c=onCleanup(@()fclose(fid));
fwrite(fid,s,'char');
end
