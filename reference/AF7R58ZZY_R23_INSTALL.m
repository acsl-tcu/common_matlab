function AF7R58ZZY_R23_INSTALL
% AF7R58ZZY-R23 theory-preserving compiled-correction installer.
%
% Safety/trajectory theorem is NOT changed. The installer refuses to switch
% the runner unless source binding and strict-FP bitwise evaluator gates close.

EXPECTED_SOURCE_SHA='310cfdd01f9554a80b71e8d90464816a721859709ad7c4b9137cdc0f3a6eaf49';
EXPECTED_HELPER_SHA='1f57a0066acb75859dfe565b3c38a99d5c272b8afae4698131d7aeeb70093889';
EXPECTED_PRISTINE_RUNNER_SHA='d4d15d081ea8a6e3073b0a63ce36e47f983777ac2c81db194384de69f5e5fc37';

scriptDir=fileparts(mfilename('fullpath'));
srcPath=which('REPLANNING_SYSTEM');
runnerPath=which('run_flight_headless');
helperPath=which('af7r33_flat_jet_batch_model');
canonPath=which('af7r58i_eval_canonical_reference');
tvPath=which('TIME_VARYING_REFERENCE');
assert(~isempty(srcPath)&&isfile(srcPath),'[R23] REPLANNING_SYSTEM.m not found.');
assert(~isempty(runnerPath)&&isfile(runnerPath),'[R23] run_flight_headless.m not found.');
assert(~isempty(helperPath)&&isfile(helperPath),'[R23] AF7R33 helper not found.');
assert(~isempty(canonPath)&&isfile(canonPath),'[R23] canonical evaluator source not found.');
assert(~isempty(tvPath)&&isfile(tvPath),'[R23] TIME_VARYING_REFERENCE.m not found.');
assert(strcmpi(sha256_file_r23(srcPath),EXPECTED_SOURCE_SHA),'[R23] production source SHA mismatch.');
assert(strcmpi(sha256_file_r23(helperPath),EXPECTED_HELPER_SHA),'[R23] AF7R33 helper SHA mismatch.');
root=fileparts(runnerPath);

% -------------------------------------------------------------------------
% Source theorem binding.
% -------------------------------------------------------------------------
Rtxt=fileread(srcPath);
Ctxt=compact_r23(fileread(canonPath));
Ttxt=compact_r23(fileread(tvPath));
assert(contains(Ctxt,'forg=1:numel(model.corrections)') && ...
    contains(Ctxt,'D=correction_derivatives(tg,C,6);') && ...
    contains(Ctxt,'xd(4*r+(1:3))=xd(4*r+(1:3))+D(:,r+1);'), ...
    '[R23] canonical ordered-addition authority changed.');
assert(contains(Ctxt,'ift<=double(C.t0)||t>=double(C.t1),return;end') || ...
       contains(Ctxt,'ift<=C.t0||t>=C.t1,return;end'), ...
    '[R23] canonical strict-support authority changed.');
assert(contains(Ttxt,'obj.result.state.xd=obj.func(t);'), ...
    '[R23] TIME_VARYING_REFERENCE nominal xd authority changed.');

% do() must close the runtime hot certificate guard before current repaired
% evaluation and future-reference packet construction.
pGuard=strfind(Rtxt,'obj.assert_runtime_core_certificates_current_hot();');
pEval=strfind(Rtxt,'xd_active_now=obj.evaluate_active_reference_xd(time.t,xd_nom);');
pPkt=strfind(Rtxt,'obj.get_active_reference_packet(time,cha,varargin,xd_nom);');
assert(~isempty(pGuard)&&numel(pEval)==1&&numel(pPkt)>=1, ...
    '[R23] production guard/evaluate/packet tokens changed.');
idxGuard=find(pGuard<pEval(1),1,'last');
assert(~isempty(idxGuard)&&pGuard(idxGuard)<pEval(1)&&pEval(1)<pPkt(1), ...
    '[R23] production guard/evaluate/packet ordering changed.');
assert(contains(Rtxt,'r = obj.base_ref.do(args{:});') && ...
    contains(Rtxt,'xd_eval = r.state.xd;') && ...
    contains(Rtxt,'obj.nominal_table_xd=xdtab;'), ...
    '[R23] nominal-table exact base-reference authority changed.');

% Production staged freshness authority must still include all source/sensor
% dependencies R23 copies into its single freshness gate.
requiredFresh={ ...
    'isequal(logical(active(:).''),', ...
    'isequaln(obj.static_obstacle_map,obj.staged_replan_static_map)', ...
    'obj.staged_reference_only_dependency_key()', ...
    'obj.active_speed_certificate', ...
    'obj.staged_replan_obstacle_id', ...
    'obj.staged_replan_component_code'};
for k=1:numel(requiredFresh)
    assert(contains(Rtxt,requiredFresh{k}),'[R23] production freshness authority token changed: %s',requiredFresh{k});
end

% -------------------------------------------------------------------------
% Runner authority recognition.
% -------------------------------------------------------------------------
anchors={ ...
    'REPLANNING_SYSTEM_R23_COMPILED','R23'; ...
    'REPLANNING_SYSTEM_R22_ACCOUNTING_DIAG','R22'; ...
    'REPLANNING_SYSTEM_R21_BALANCED','R21'; ...
    'REPLANNING_SYSTEM_R20_FAST','R20'; ...
    'REPLANNING_SYSTEM_R19_FAST','R19'; ...
    'REPLANNING_SYSTEM_R18_STAGE2_DIAG','R18'; ...
    'REPLANNING_SYSTEM_R17_FAST','R17'; ...
    'REPLANNING_SYSTEM_R16_FAST','R16'; ...
    'REPLANNING_SYSTEM_R15_FAST','R15'; ...
    'REPLANNING_SYSTEM','PRISTINE'};
newAnchor='replanner_instance = REPLANNING_SYSTEM_R23_COMPILED(agent,nominal_ref,replan_opts);';
txt=fileread(runnerPath);
mode=''; oldAnchor='';
for k=1:size(anchors,1)
    a=sprintf('replanner_instance = %s(agent,nominal_ref,replan_opts);',anchors{k,1});
    if contains(txt,a)
        assert(count(txt,a)==1,'[R23] runner authority anchor is not unique.');
        mode=['FROM_' anchors{k,2}]; oldAnchor=a; break;
    end
end
assert(~isempty(mode),'[R23] runner is not a recognized authority.');
if strcmp(mode,'FROM_PRISTINE')
    assert(strcmpi(sha256_file_r23(runnerPath),EXPECTED_PRISTINE_RUNNER_SHA), ...
        '[R23] pristine runner SHA mismatch.');
end

% -------------------------------------------------------------------------
% Install exact parent authorities + R23 implementation.
% -------------------------------------------------------------------------
files={ ...
    'REPLANNING_SYSTEM_R19_FAST.m', ...
    'REPLANNING_SYSTEM_R21_BALANCED.m', ...
    'REPLANNING_SYSTEM_R23_COMPILED.m', ...
    'af7r75_posaccel_batch_R19.m', ...
    'af7r75_posaccel_core_R19.m', ...
    'af7r72_posaccel_core_R16.m', ...
    'af7r73_posaccel_core_R17.m', ...
    'af7r76_compile_correction_packet_R23.m'};
for k=1:numel(files)
    s=fullfile(scriptDir,files{k});
    assert(isfile(s),'[R23] package file missing: %s',files{k});
    copyfile(s,fullfile(root,files{k}),'f');
end

compile_mex_r23(fullfile(scriptDir,'af7r70_observer_mex.cpp'),root,'af7r70_observer_mex');
compile_mex_r23(fullfile(scriptDir,'af7r71_decasteljau_batch_mex.cpp'),root,'af7r71_decasteljau_batch_mex');
compile_mex_r23(fullfile(scriptDir,'af7r75_correction_posaccel_mex.cpp'),root,'af7r75_correction_posaccel_mex');
compile_mex_r23(fullfile(scriptDir,'af7r76_compiled_eval_mex.cpp'),root,'af7r76_compiled_eval_mex');
addpath(root,'-begin'); rehash;
clear af7r76_compile_correction_packet_R23 af7r75_posaccel_batch_R19 af7r75_posaccel_core_R19;
rehash;
assert(exist('af7r70_observer_mex','file')==3,'[R23] observer MEX install failed.');
assert(exist('af7r71_decasteljau_batch_mex','file')==3,'[R23] de Casteljau MEX install failed.');
assert(exist('af7r75_correction_posaccel_mex','file')==3,'[R23] R19 correction MEX install failed.');
assert(exist('af7r76_compiled_eval_mex','file')==3,'[R23] compiled correction MEX install failed.');
assert(exist('REPLANNING_SYSTEM_R19_FAST','class')==8,'[R23] R19 parent class parse failed.');
assert(exist('REPLANNING_SYSTEM_R21_BALANCED','class')==8,'[R23] R21 parent class parse failed.');
assert(exist('REPLANNING_SYSTEM_R23_COMPILED','class')==8,'[R23] R23 class parse failed.');

% -------------------------------------------------------------------------
% Strict-FP algebra/evaluation gate BEFORE runner switch.
% -------------------------------------------------------------------------
selftest_r23_compiled_exact();

% Static R23 invariants.
Q=fileread(fullfile(root,'REPLANNING_SYSTEM_R23_COMPILED.m'));
Qc=compact_r23(Q);
assert(contains(Q,'classdef REPLANNING_SYSTEM_R23_COMPILED < REPLANNING_SYSTEM_R21_BALANCED'), ...
    '[R23] R21 inheritance missing.');
assert(~contains(Q,'REPLANNING_SYSTEM_R20_FAST'),'[R23] failed R20 authority leaked into R23.');
assert(contains(Q,'prepare_staged_reference_only_bundle@REPLANNING_SYSTEM_R21_BALANCED'), ...
    '[R23] exact R21 Stage2 authority closure is not retained.');
assert(contains(Q,'R.eval_fn')&&contains(Q,'R.flat_eval_fn'), ...
    '[R23] canonical evaluator handles are not retained.');
assert(~contains(Q,'Bfull=B;'),'[R23] Bfull materialization unexpectedly present.');
assert(~contains(Q,'obj.assert_runtime_core_certificates_current_hot();'), ...
    '[R23] duplicate post-assignment hot guard unexpectedly present.');
assert(count(Qc,'fresh=')==1,'[R23] staged freshness gate is not singular.');
assert(contains(Q,'STAGED_ATOMIC_CONTROL_PERIOD_EXCEEDED'), ...
    '[R23] fail-closed control-period rollback contract missing.');
assert(contains(Q,'''pipeline_intervals'',2'), ...
    '[R23] two-interval activation contract missing.');

% -------------------------------------------------------------------------
% Transactional runner switch only after all gates close.
% -------------------------------------------------------------------------
backupPath=fullfile(root,'run_flight_headless.R23_PREINSTALL_BACKUP.m');
if ~contains(txt,newAnchor)
    assert(~isfile(backupPath),'[R23] R23 runner backup already exists; resolve before install.');
    copyfile(runnerPath,backupPath);
    patched=strrep(txt,oldAnchor,newAnchor);
    assert(count(patched,newAnchor)==1,'[R23] runner replacement count invalid.');
    write_utf8_r23(runnerPath,patched);
else
    mode='ALREADY_R23';
end
rehash;

assert(strcmpi(sha256_file_r23(srcPath),EXPECTED_SOURCE_SHA),'[R23] production source changed during install.');
assert(strcmpi(sha256_file_r23(helperPath),EXPECTED_HELPER_SHA),'[R23] AF7R33 helper changed during install.');
newtxt=fileread(runnerPath);
assert(count(newtxt,newAnchor)==1,'[R23] runner switch did not close.');

manifest=fullfile(root,'AF7R58ZZY_R23_INSTALL_MANIFEST.txt');
fid=fopen(manifest,'w'); assert(fid>=0); cc=onCleanup(@()fclose(fid));
fprintf(fid,'status=R23_INSTALLED\n');
fprintf(fid,'installMode=%s\n',mode);
fprintf(fid,'productionSourceModified=0\n');
fprintf(fid,'sourceSHA=%s\n',sha256_file_r23(srcPath));
fprintf(fid,'runnerSHA=%s\n',sha256_file_r23(runnerPath));
fprintf(fid,'r23ClassSHA=%s\n',sha256_file_r23(fullfile(root,'REPLANNING_SYSTEM_R23_COMPILED.m')));
fprintf(fid,'runnerBackup=%s\n',backupPath);
fprintf(fid,'trajectoryMathChanged=0\n');
fprintf(fid,'continuousGeometryCertificateChanged=0\n');
fprintf(fid,'C7CertificateChanged=0\n');
fprintf(fid,'speedSensorCertificateChanged=0\n');
fprintf(fid,'controllerChanged=0\n');
fprintf(fid,'activationDelayChanged=0\n');
fprintf(fid,'pipelineIntervals=2\n');
fprintf(fid,'canonicalEvalHandlesRetained=1\n');
fprintf(fid,'compiledRepresentationOnly=1\n');
fprintf(fid,'strictFPBitwiseGate=1\n');
fprintf(fid,'stage3SingleFreshnessGate=1\n');
fprintf(fid,'deadlineRollbackRetained=1\n');

fprintf('\n[R23] INSTALLED | mode=%s\n',mode);
fprintf('[R23] Strict-FP compiled correction bitwise self-test CLOSED before runner switch.\n');
fprintf('[R23] Canonical eval_fn/flat_eval_fn retained as mathematical authority.\n');
fprintf('[R23] Continuous geometry/C7/speed/sensor certificates unchanged.\n');
fprintf('[R23] Stage3 freshness remains fail-closed and is evaluated once immediately before commit.\n');
fprintf('[R23] Activation remains trigger+2*dt; deadline rollback retained.\n');
fprintf('[R23] Run: run_flight_headless\n');
fprintf('[R23] Then: AF7R58ZZY_R23_REPORT\n');
end

function selftest_r23_compiled_exact()
% Multiple overlapping groups, multiple segments, exact boundaries and
% near-boundary points. Compare BOTH full r=0..6 state correction and r={0,2}.
t=[0.25 0.5 0.999999 1.0 1.000001 1.125 1.25 1.5 1.75 2.0 ...
   2.000001 2.125 2.25 2.5 2.75 3.0 3.000001 3.4];
N=numel(t);
base=zeros(28,N);
for n=1:N
    for r=0:6
        base(4*r+(1:3),n)=[r+1+0.031*t(n); -2*r+0.017*t(n); 0.2*r-0.019*t(n)];
        base(4*r+4,n)=0.05*(r+1)+0.001*t(n);
    end
end
P0=base(1:3,:); A0=base(9:11,:);
model=struct('schema','AF7R33_BATCH_FLAT_MODEL_V1','certified',true, ...
    'corrections',struct('t0',{},'t1',{},'segment_t0',{},'segment_t1',{},'control_points',{}));
model.corrections(1)=synthetic_correction_r23(1.0,2.5, 0.013,3);
model.corrections(2)=synthetic_correction_r23(1.5,3.0,-0.021,2);
model.corrections(3)=synthetic_correction_r23(2.0,3.4, 0.007,4);
CP=af7r76_compile_correction_packet_R23(model,7);
Yold=legacy_full_r23(base,t,model);
[Pold,Aold]=legacy_pa_r23(P0,A0,t,model);
Ynew=af7r76_compiled_eval_mex(1,t,base,CP);
[Pnew,Anew]=af7r76_compiled_eval_mex(2,t,P0,A0,CP);
assert(bitwise_r23(Yold,Ynew),'[R23] full r=0..6 compiled evaluator bitwise gate failed.');
assert(bitwise_r23(Pold,Pnew),'[R23] r=0 compiled evaluator bitwise gate failed.');
assert(bitwise_r23(Aold,Anew),'[R23] r=2 compiled evaluator bitwise gate failed.');
end

function Y=legacy_full_r23(base,t,model)
Y=double(base);
for g=1:numel(model.corrections)
    C=model.corrections(g);
    for n=1:numel(t)
        D=legacy_derivatives_r23(t(n),C,6);
        for r=0:6
            Y(4*r+(1:3),n)=Y(4*r+(1:3),n)+D(:,r+1);
        end
    end
end
end

function [P,A]=legacy_pa_r23(P,A,t,model)
P=double(P); A=double(A);
for g=1:numel(model.corrections)
    C=model.corrections(g);
    for n=1:numel(t)
        D=legacy_derivatives_r23(t(n),C,2);
        P(:,n)=P(:,n)+D(:,1);
        A(:,n)=A(:,n)+D(:,3);
    end
end
end

function D=legacy_derivatives_r23(t,C,maxOrder)
D=zeros(3,maxOrder+1);
if t<=double(C.t0)||t>=double(C.t1), return; end
s0=double(C.segment_t0(:)); s1=double(C.segment_t1(:)); CP=C.control_points;
is=find(t<=s1,1,'first');
if isempty(is), return; end
if t<s0(is)
    if is==1, return; end
    is=is-1;
end
T=s1(is)-s0(is); u=(t-s0(is))/T; u=max(0,min(1,u));
Q=double(CP{is}); deg=15; D(:,1)=legacy_decasteljau_r23(Q,u);
for r=1:maxOrder
    Q=(deg/T)*diff(Q,1,2); deg=deg-1;
    D(:,r+1)=legacy_decasteljau_r23(Q,u);
end
end

function y=legacy_decasteljau_r23(C,u)
C=double(C);
while size(C,2)>1
    C=(1-u)*C(:,1:end-1)+u*C(:,2:end);
end
y=C(:,1);
end

function C=synthetic_correction_r23(t0,t1,scale,S)
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

function tf=bitwise_r23(a,b)
a=double(a); b=double(b);
tf=isequal(size(a),size(b)) && isequal(typecast(a(:),'uint64'),typecast(b(:),'uint64'));
end

function compile_mex_r23(cppPath,outDir,outName)
assert(isfile(cppPath),'[R23] missing C++ source: %s',cppPath);
old=pwd; c=onCleanup(@()cd(old)); cd(outDir);
if ispc
    mex('-R2018a','COMPFLAGS=$COMPFLAGS /fp:strict',cppPath,'-output',outName,'-outdir',outDir);
else
    mex('-R2018a','CXXFLAGS=$CXXFLAGS -fno-fast-math -ffp-contract=off',cppPath,'-output',outName,'-outdir',outDir);
end
end

function s=compact_r23(s)
s=regexprep(s,'\s+','');
end

function h=sha256_file_r23(p)
fid=fopen(p,'rb'); assert(fid>=0,'[R23] cannot open %s.',p); c=onCleanup(@()fclose(fid));
b=fread(fid,Inf,'*uint8');
md=javaMethod('getInstance','java.security.MessageDigest','SHA-256');
md.update(typecast(b,'int8'));
h=lower(reshape(dec2hex(typecast(md.digest(),'uint8'),2).',1,[]));
end

function write_utf8_r23(p,s)
fid=fopen(p,'w','n','UTF-8'); assert(fid>=0,'[R23] cannot write %s.',p); c=onCleanup(@()fclose(fid)); fwrite(fid,s,'char');
end
