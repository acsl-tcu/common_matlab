function AF7R58ZZY_R18_INSTALL
% R18 diagnostic install: switch R17 lean implementation to fine-grain Stage2 timing.
% REPLANNING_SYSTEM.m remains byte-identical. No controller/trajectory/certificate changes.

EXPECTED_SOURCE_SHA='310cfdd01f9554a80b71e8d90464816a721859709ad7c4b9137cdc0f3a6eaf49';
EXPECTED_R17_CLASS_SHA='4fb7e70ec38a0ded4975846d0e0e758ad70946db5cb92b0d313bc21560f50d4d';
EXPECTED_R17_BATCH_SHA='4b1e0d7fdec84354c2ce65d28d1ee50bec522ab94ac3feb68b2b347ff25c8c65';
EXPECTED_R17_CORE_SHA='0bc12e7e2c9241b308e731c5639f5deeb159035b4b3acfbfd2d178157493e755';

scriptDir=fileparts(mfilename('fullpath'));
srcPath=which('REPLANNING_SYSTEM');
runnerPath=which('run_flight_headless');
r17Class=which('REPLANNING_SYSTEM_R17_FAST');
r17Batch=which('af7r73_posaccel_batch_R17');
r17Core=which('af7r73_posaccel_core_R17');
assert(~isempty(srcPath)&&isfile(srcPath),'[R18] production source missing.');
assert(~isempty(runnerPath)&&isfile(runnerPath),'[R18] runner missing.');
assert(~isempty(r17Class)&&isfile(r17Class),'[R18] R17 class missing.');
assert(~isempty(r17Batch)&&isfile(r17Batch),'[R18] R17 batch helper missing.');
assert(~isempty(r17Core)&&isfile(r17Core),'[R18] R17 core helper missing.');
assert(strcmpi(sha256_file_r18(srcPath),EXPECTED_SOURCE_SHA),'[R18] production source SHA mismatch.');
assert(strcmpi(sha256_file_r18(r17Class),EXPECTED_R17_CLASS_SHA),'[R18] R17 class SHA mismatch.');
assert(strcmpi(sha256_file_r18(r17Batch),EXPECTED_R17_BATCH_SHA),'[R18] R17 batch SHA mismatch.');
assert(strcmpi(sha256_file_r18(r17Core),EXPECTED_R17_CORE_SHA),'[R18] R17 core SHA mismatch.');
assert(exist('af7r70_observer_mex','file')==3,'[R18] R17 observer MEX missing.');
assert(exist('af7r71_decasteljau_batch_mex','file')==3,'[R18] R17 de Casteljau MEX missing.');

root=fileparts(runnerPath);
files={'REPLANNING_SYSTEM_R18_STAGE2_DIAG.m','af7r74_posaccel_batch_R18.m','af7r74_posaccel_core_R18.m'};
for k=1:numel(files)
    p=fullfile(scriptDir,files{k}); assert(isfile(p),'[R18] package file missing: %s',files{k});
    copyfile(p,fullfile(root,files{k}),'f');
end
addpath(root,'-begin'); rehash;
clear af7r74_posaccel_batch_R18 af7r74_posaccel_core_R18;
assert(exist('REPLANNING_SYSTEM_R18_STAGE2_DIAG','class')==8,'[R18] class parse/install failed.');
assert(nargin('af7r74_posaccel_batch_R18')==3 && nargout('af7r74_posaccel_batch_R18')==5, ...
    '[R18] batch helper signature mismatch.');
assert(nargin('af7r74_posaccel_core_R18')==4 && nargout('af7r74_posaccel_core_R18')==4, ...
    '[R18] core helper signature mismatch.');
selftest_r18_core();

r17Anchor='replanner_instance = REPLANNING_SYSTEM_R17_FAST(agent,nominal_ref,replan_opts);';
r18Anchor='replanner_instance = REPLANNING_SYSTEM_R18_STAGE2_DIAG(agent,nominal_ref,replan_opts);';
txt=fileread(runnerPath);
if contains(txt,r18Anchor)
    assert(count(txt,r18Anchor)==1,'[R18] R18 runner anchor is not unique.');
    mode='ALREADY_R18';
elseif contains(txt,r17Anchor)
    assert(count(txt,r17Anchor)==1,'[R18] R17 runner anchor is not unique.');
    mode='FROM_R17';
    backup=fullfile(root,'run_flight_headless.R18_PREINSTALL_BACKUP.m');
    assert(~isfile(backup),'[R18] backup already exists; resolve before install.');
    copyfile(runnerPath,backup);
    txt=strrep(txt,r17Anchor,r18Anchor);
    write_utf8_r18(runnerPath,txt);
else
    error('[R18] runner must currently use R17 or already use R18.');
end
rehash;
assert(strcmpi(sha256_file_r18(srcPath),EXPECTED_SOURCE_SHA),'[R18] production source changed during install.');
newtxt=fileread(runnerPath);
assert(count(newtxt,r18Anchor)==1 && ~contains(newtxt,r17Anchor),'[R18] runner switch failed.');

fprintf('\n[R18] INSTALLED | mode=%s\n',mode);
fprintf('[R18] Diagnostic only: Stage2 fine-grain tic/toc.\n');
fprintf('[R18] R17 remains the performance authority; R18 timing adds instrumentation overhead.\n');
fprintf('[R18] Run: run_flight_headless\n');
fprintf('[R18] Rollback: AF7R58ZZY_R18_ROLLBACK\n');
end

function selftest_r18_core()
N=23; t=linspace(0.25,3.0,N);
P=zeros(3,N); A=zeros(3,N);
for k=1:N
    P(:,k)=[1+t(k);2-0.5*t(k);3+0.25*t(k)];
    A(:,k)=[0.1;0.2;-0.3]+k*eps;
end
model=struct('schema','AF7R33_BATCH_FLAT_MODEL_V1','certified',true, ...
    'corrections',struct('t0',{},'t1',{},'segment_t0',{},'segment_t1',{},'control_points',{}));
model.corrections(1)=synthetic_correction_r18(1.0,2.0,0.01);
model.corrections(2)=synthetic_correction_r18(1.5,2.5,-0.02);
[P17,A17,V17]=af7r73_posaccel_core_R17(P,A,t,model);
[P18,A18,V18,T]=af7r74_posaccel_core_R18(P,A,t,model);
assert(isequal(V17,V18) && bitwise_r18(P17,P18) && bitwise_r18(A17,A18), ...
    '[R18] diagnostic core changed numerical output.');
assert(isstruct(T)&&isfinite(T.total_ms),'[R18] timing self-test invalid.');
end

function C=synthetic_correction_r18(t0,t1,scale)
tm=(t0+t1)/2; C=struct(); C.t0=t0; C.t1=t1;
C.segment_t0=[t0 tm]; C.segment_t1=[tm t1]; C.control_points=cell(1,2);
for s=1:2
    Q=zeros(3,16);
    for j=1:16, Q(:,j)=scale*[j;j^2/16;(-1)^j*j/8]*(1+0.1*s); end
    C.control_points{s}=Q;
end
end

function tf=bitwise_r18(a,b)
a=double(a); b=double(b);
tf=isequal(size(a),size(b)) && isequal(typecast(a(:),'uint64'),typecast(b(:),'uint64'));
end

function h=sha256_file_r18(p)
fid=fopen(p,'rb'); assert(fid>=0,'[R18] cannot open %s',p); c=onCleanup(@()fclose(fid));
b=fread(fid,Inf,'*uint8'); md=javaMethod('getInstance','java.security.MessageDigest','SHA-256');
md.update(typecast(b,'int8')); h=lower(reshape(dec2hex(typecast(md.digest(),'uint8'),2).',1,[]));
end

function write_utf8_r18(p,s)
fid=fopen(p,'w','n','UTF-8'); assert(fid>=0,'[R18] cannot write %s',p); c=onCleanup(@()fclose(fid)); fwrite(fid,s,'char');
end
