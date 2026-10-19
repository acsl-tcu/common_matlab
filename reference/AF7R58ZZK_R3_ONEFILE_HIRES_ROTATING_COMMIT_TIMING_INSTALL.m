function OUT = AF7R58ZZK_R3_ONEFILE_HIRES_ROTATING_COMMIT_TIMING_INSTALL()
% AF7R58ZZK-R3
% High-resolution, low-perturbation rotating-window timing install.
%
% PRIMARY CLOCKS (NO tic/toc FOR SUBSTAGE ATTRIBUTION)
%   1) QueryPerformanceCounter
%   2) QueryThreadCycleTime(GetCurrentThread)
%   3) QueryProcessCycleTime(GetCurrentProcess)
%
% PERTURBATION CONTROL
%   Exactly ONE timing window is selected per real commit attempt.
%   Each selected window gets only TWO MEX mark calls (start/end).
%
% SAFETY / SENSING
%   Diagnostic only. 4 ms deadline, acceptance, certificate math,
%   trajectory/generator math, and finite-sensing contract are unchanged.

PRE_SHA='2a0013a583cbcff3afd62e4495fa5e784427132390616ec5886000f134f2c05f';

OUT=struct('schema','AF7R58ZZK_R3_HIRES_ROTATING_TIMING_V3', ...
    'production_modified',false,'rollback_performed',false, ...
    'status','OPEN','error',struct());

pR=which('REPLANNING_SYSTEM');
assert(~isempty(pR)&&isfile(pR),'[AF7R58ZZK-R3] REPLANNING_SYSTEM unresolved.');
root=fileparts(pR);
outdir=tempname(root); [ok,msg]=mkdir(outdir); assert(ok,'%s',msg);
report=fullfile(outdir,'AF7R58ZZK_R3_REPORT.txt');
matp=fullfile(outdir,'AF7R58ZZK_R3_OUT.mat');
fid=fopen(report,'wt','n','UTF-8'); assert(fid>=0);
cf=onCleanup(@()safe_fclose(fid)); %#ok<NASGU>

say(fid,'====================================================================================================');
say(fid,' AF7R58ZZK-R3 | HIGH-RES ROTATING COMMIT TIMING INSTALL');
say(fid,'====================================================================================================');
say(fid,'[AUTHORITY] QPC=1 | threadCycles=1 | processCycles=1 | ticTocSubstage=0');
say(fid,'[PERTURBATION] oneWindowPerRequest=1 | marksPerSelectedWindow=2 | fixedMexRing=1');
say(fid,'[SAFETY] deadlineChanged=0 | acceptanceChanged=0 | certificateMathChanged=0 | generatorMathChanged=0');
say(fid,'[SENSING] detectedOnlyGeneratorRequired=1 | unseenTruthPreplan=0');

backup=fullfile(outdir,'REPLANNING_SYSTEM_PRE_AF7R58ZZK_R3.m');

try
    assert(ispc,'[AF7R58ZZK-R3] Windows QPC/cycle APIs required.');

    pre=file_hash(pR);
    OUT.pre_sha=pre;
    OUT.production_path=pR;
    OUT.backup_path=backup;
    say(fid,''); say(fid,'[A. EXACT CURRENT SOURCE]');
    say(fid,' path = %s',pR);
    say(fid,' SHA  = %s',pre);
    assert(strcmp(pre,PRE_SHA), ...
        '[AF7R58ZZK-R3] source differs from exact AF7R58ZZH authority.');

    R=read_utf8(pR);
    assert(contains(R,'The generator receives ONLY sensor-known obstacles:'));
    assert(contains(R,'''detected_only_obstacle_contract'',true'));
    assert(contains(R,'Initial known set is empty.'));
    assert(contains(R,'raw_sensor_gate alone may admit a previously unknown obstacle.'));
    assert(~contains(R,'AF7R58ZZK_R3_ROTATING_WINDOW'), ...
        '[AF7R58ZZK-R3] instrumentation already present.');

    copyfile(pR,backup,'f');

    cpp=fullfile(outdir,'af7r58zzk_trace_mex.cpp');
    write_utf8(cpp,cpp_source());

    say(fid,''); say(fid,'[B. HIGH-RES TRACE MEX]');
    clear af7r58zzk_trace_mex
    rehash
    pMex=which('af7r58zzk_trace_mex');
    mexOK=false;
    if ~isempty(pMex)
        try
            af7r58zzk_trace_mex('reset');
            ss=af7r58zzk_trace_mex('stamp');
            mexOK=isa(ss,'uint64')&&numel(ss)==4&&all(ss>0);
        catch
            mexOK=false;
        end
    end
    if ~mexOK
        oldpwd=pwd;
        cd(root);
        cleanupPwd=onCleanup(@()cd(oldpwd)); %#ok<NASGU>
        mex('-R2018a','-outdir',root,cpp);
        clear af7r58zzk_trace_mex
        rehash
        pMex=which('af7r58zzk_trace_mex');
    end

    assert(~isempty(pMex),'[AF7R58ZZK-R3] trace MEX did not resolve.');
    OUT.mex_path=pMex;
    OUT.mex_sha=file_hash(pMex);
    say(fid,' MEX = %s',pMex);
    say(fid,' SHA = %s',OUT.mex_sha);

    af7r58zzk_trace_mex('reset');
    ss=af7r58zzk_trace_mex('stamp');
    assert(isa(ss,'uint64')&&numel(ss)==4&&all(ss>0), ...
        '[AF7R58ZZK-R3] trace MEX self-test failed.');

    ms='function [ok,reason] = try_commit_reference_only_proposal';
    me='function F = build_execution_aware_foundation_snapshot';
    ps=strfind(R,ms); pe=strfind(R,me);
    assert(numel(ps)==1&&numel(pe)==1&&pe>ps, ...
        '[AF7R58ZZK-R3] commit method boundaries not unique/ordered.');
    F=R(ps:pe-1);

    entryAnchor=sprintf('            ok=false;\n            reason='''';\n');
    entryInsert=[entryAnchor ...
        sprintf('\n            %% AF7R58ZZK_R3_ROTATING_WINDOW: diagnostic only.\n') ...
        sprintf(['            %% Rotate by completed commit-path outcomes, not raw requests.\n' ...
                 '            af7r58zzk_attempt=double(obj.generator_validation_fail_count)+ ...\n' ...
                 '                double(obj.generator_validation_pass_count)+1;\n' ...
                 '            af7r58zzk_slot=min(7,max(1,ceil(af7r58zzk_attempt/2)));\n'])];
    F=replace_once(F,entryAnchor,entryInsert);

    % Window 1: resource model.
    F=insert_before_once(F, ...
        '            [resourceOK,resourceReason,batchModel,resourceMeta]= ...', ...
        mark_start(1));
    resourceBlock=sprintf(['            if ~resourceOK\n' ...
        '                reason=resourceReason;\n' ...
        '                return;\n' ...
        '            end\n']);
    F=insert_after_once(F,resourceBlock,mark_end(1,0));

    % Window 2: exact runtime reuse + prepared-cache gate.
    F=insert_before_once(F,'            reuseRuntimePacket= ...',mark_start(2));
    gate2=sprintf(['            if nargin>=4 && ~isempty(t_hard_cycle) && ...\n' ...
        '                    toc(t_hard_cycle)>double(obj.hard_modify_deadline_s)\n' ...
        '                reason=''REFERENCE_ONLY_DEADLINE_AFTER_REUSE_GATE'';\n' ...
        '                return;\n' ...
        '            end\n']);
    F=insert_before_once(F,gate2,mark_end(2,0));

    % Window 3: packet reuse / retarget / rebuild.
    % R2: 'if reuseRuntimePacket' occurs twice in the method:
    %   (a) pre-transaction packet preparation (wanted),
    %   (b) post-commit reuse/rebuild counter update (must NOT be timed here).
    % Select only the first occurrence after the reuseRuntimePacket assignment
    % and before the preparation-deadline token.
    pAssign=strfind(F,'            reuseRuntimePacket= ...');
    pDeadline=strfind(F,'REFERENCE_ONLY_COMMIT_PREPARATION_DEADLINE_EXCEEDED');
    assert(numel(pAssign)==1 && numel(pDeadline)==1 && pAssign<pDeadline, ...
        '[AF7R58ZZK-R3] Window-3 bounding anchors not unique/ordered.');
    pIfAll=strfind(F,'            if reuseRuntimePacket');
    pIf=pIfAll(pIfAll>pAssign & pIfAll<pDeadline);
    assert(numel(pIf)==1, ...
        '[AF7R58ZZK-R3] expected exactly one pre-transaction reuse branch, found %d.',numel(pIf));
    F=[F(1:pIf-1) mark_start(3) F(pIf:end)];

    p=strfind(F,'REFERENCE_ONLY_COMMIT_PREPARATION_DEADLINE_EXCEEDED');
    assert(numel(p)==1);
    gateStart=find_gate_start(F,p);
    F=[F(1:gateStart-1) mark_end(3,0) F(gateStart:end)];

    % Window 4: canonical evaluator handles.
    F=insert_before_once(F,'            [evalFn,flatEvalFn]= ...',mark_start(4));
    evalCall=sprintf(['            [evalFn,flatEvalFn]= ...\n' ...
        '                af7r58i_make_canonical_eval_handles(obj,batchModel);\n']);
    F=insert_after_once(F,evalCall,mark_end(4,0));

    % Window 5: certificate metadata.
    F=insert_before_once(F, ...
        '            % C7 endpoint/peak construction is exact by repeated endpoint', ...
        mark_start(5));
    p=strfind(F,'REFERENCE_ONLY_DEADLINE_AFTER_CERT_METADATA');
    assert(numel(p)==1);
    gateStart=find_gate_start(F,p);
    F=[F(1:gateStart-1) mark_end(5,0) F(gateStart:end)];

    % Window 6: rollback snapshot.
    F=insert_before_once(F,'            old=struct( ...',mark_start(6));
    p=strfind(F,'REFERENCE_ONLY_DEADLINE_AFTER_ROLLBACK_SNAPSHOT');
    assert(numel(p)==1);
    gateStart=find_gate_start(F,p);
    F=[F(1:gateStart-1) mark_end(6,0) F(gateStart:end)];

    % Window 7: atomic transaction. Find try AFTER rollback deadline gate.
    p=strfind(F,'REFERENCE_ONLY_DEADLINE_AFTER_ROLLBACK_SNAPSHOT');
    assert(numel(p)==1);
    tail=F(p:end);
    q=strfind(tail,sprintf('\n            try\n'));
    assert(~isempty(q),'[AF7R58ZZK-R3] transaction try anchor missing.');
    ptry=p+q(1); % points to first space after newline
    F=[F(1:ptry-1) mark_start(7) F(ptry:end)];

    okAnchor=sprintf('                ok=true;\n                reason=''REFERENCE_ONLY_COMMIT_CLOSED'';');
    F=insert_before_once(F,okAnchor,mark_end(7,0));

    catchAnchor='            catch ME';
    F=insert_after_once(F,catchAnchor, ...
        sprintf(['\n                if af7r58zzk_slot==7\n' ...
        '                    af7r58zzk_trace_mex(''mark'',uint64(obj.generator_request_count),uint64(72),uint64(1));\n' ...
        '                end']));

    assert(count_exact(F,'af7r58zzk_trace_mex(''mark''')==15, ...
        '[AF7R58ZZK-R3] expected exactly 15 mark call sites.');
    assert(count_exact(F,'AF7R58ZZK_R3_ROTATING_WINDOW')==1);

    % R2 semantic placement audit: start mark 31 must precede the preparation
    % reuse branch, while the independent post-commit counter branch remains
    % after active authority mutation.
    pM31=strfind(F,'''mark'',uint64(obj.generator_request_count),uint64(31),uint64(0)');
    pReuseAll=strfind(F,'            if reuseRuntimePacket');
    pMut=strfind(F,'obj.active_repaired_reference=R;');
    assert(numel(pM31)==1 && numel(pReuseAll)==2 && numel(pMut)==1, ...
        '[AF7R58ZZK-R3] semantic placement count mismatch.');
    assert(pM31 < pReuseAll(1) && pReuseAll(1) < pMut && pMut < pReuseAll(2), ...
        '[AF7R58ZZK-R3] Window-3 instrumentation attached to wrong reuse branch.');

    R2=[R(1:ps-1) F R(pe:end)];

    assert(contains(R2,'The generator receives ONLY sensor-known obstacles:'));
    assert(contains(R2,'''detected_only_obstacle_contract'',true'));
    assert(contains(R2,'Initial known set is empty.'));
    assert(contains(R2,'raw_sensor_gate alone may admit a previously unknown obstacle.'));
    assert(count_exact(R2,'REFERENCE_ONLY_DEADLINE_AFTER_RESOURCE_MODEL')==1);
    assert(count_exact(R2,'REFERENCE_ONLY_DEADLINE_AFTER_REUSE_GATE')==1);
    assert(count_exact(R2,'REFERENCE_ONLY_COMMIT_PREPARATION_DEADLINE_EXCEEDED')==1);
    assert(count_exact(R2,'REFERENCE_ONLY_DEADLINE_AFTER_EVAL_HANDLES')==1);
    assert(count_exact(R2,'REFERENCE_ONLY_DEADLINE_AFTER_CERT_METADATA')==1);
    assert(count_exact(R2,'REFERENCE_ONLY_DEADLINE_AFTER_ROLLBACK_SNAPSHOT')==1);
    assert(contains(R2,'finished after the hard online modification deadline.'));

    % ------------------------------------------------------------------
    % FULL PREFLIGHT BEFORE THE FIRST PRODUCTION WRITE.
    % Stage exact patched bytes, compute post SHA from staged file,
    % parse the entire patched class under a temporary class name, then
    % and only then write REPLANNING_SYSTEM.m.
    % ------------------------------------------------------------------
    stageDir=fullfile(outdir,'AF7R58ZZK_R3_STAGE');
    [ok,msg]=mkdir(stageDir); assert(ok,'%s',msg);
    staged=fullfile(stageDir,'REPLANNING_SYSTEM.m');
    write_utf8(staged,R2);
    expectedPost=file_hash(staged);
    OUT.expected_post_sha=expectedPost;

    parseDir=fullfile(outdir,'AF7R58ZZK_R3_PARSE');
    [ok,msg]=mkdir(parseDir); assert(ok,'%s',msg);
    parseName='AF7R58ZZK_R3_PARSE_REPLANNING_SYSTEM';
    parseText=strrep(R2,'classdef REPLANNING_SYSTEM < handle', ...
        ['classdef ' parseName ' < handle']);
    assert(~strcmp(parseText,R2), ...
        '[AF7R58ZZK-R3] temporary class rename failed.');
    parseFile=fullfile(parseDir,[parseName '.m']);
    write_utf8(parseFile,parseText);
    addpath(parseDir,'-begin');
    parseCleanup=onCleanup(@()safe_rmpath(parseDir)); %#ok<NASGU>
    eval(['clear ' parseName]);
    mc=meta.class.fromName(parseName);
    assert(~isempty(mc),'[AF7R58ZZK-R3] patched full-class parse failed.');
    eval(['clear ' parseName]);

    say(fid,''); say(fid,'[C. PREFLIGHT BEFORE PRODUCTION WRITE]');
    say(fid,' exact pre-source SHA                   = 1');
    say(fid,' Window-3 bounded branch selection      = 1');
    say(fid,' staged patched source SHA              = %s',expectedPost);
    say(fid,' full patched class parse               = 1');
    say(fid,' production writes                      = 0');

    % Production modification starts only after all preflight gates passed.
    write_utf8(pR,R2);
    OUT.production_modified=true;
    OUT.post_sha=file_hash(pR);
    assert(strcmp(OUT.post_sha,expectedPost), ...
        '[AF7R58ZZK-R3] post-write bytes differ from staged preflight bytes.');

    af7r58zzk_trace_mex('reset');

    say(fid,''); say(fid,'[D. INSTALL RESULT]');
    say(fid,' post SHA                               = %s',OUT.post_sha);
    say(fid,' staged/post SHA exact match            = 1');
    say(fid,' source instrumentation mark sites      = 15');
    say(fid,' timing rotation uses commit attempts   = 1');
    say(fid,' MEX marks per selected request window  = 2');
    say(fid,' tic/toc used for substage timing       = 0');
    say(fid,' 4-ms deadline changed                  = 0');
    say(fid,' sensor-known-only contract retained    = 1');

    OUT.status='HIRES_ROTATING_TIMING_R3_INSTALLED__CLEAN_RERUN_REQUIRED';

catch ME
    OUT.error=struct('identifier',ME.identifier,'message',ME.message,'stack',{ME.stack});
    if OUT.production_modified && isfile(backup)
        try
            copyfile(backup,pR,'f');
            OUT.rollback_performed=true;
            OUT.production_modified=false;
        catch
        end
    end
    OUT.status='INSTALL_FAILED__ROLLBACK_ATTEMPTED';
    say(fid,''); say(fid,'[INSTALL FAILURE]');
    say(fid,' identifier = %s',ME.identifier);
    say(fid,' message    = %s',ME.message);
    say(fid,' rollback   = %d',OUT.rollback_performed);
end

OUT.report_path=report;
OUT.mat_path=matp;
save(matp,'OUT','-v7');
say(fid,'');
say(fid,'RETURN: %s',report);
say(fid,'RETURN: %s',matp);
say(fid,'status = %s',OUT.status);
assignin('base','AF7R58ZZK_R3_OUT',OUT);
end

function s=mark_start(k)
s=sprintf(['            if af7r58zzk_slot==%d\n' ...
    '                af7r58zzk_trace_mex(''mark'',uint64(obj.generator_request_count),uint64(%d),uint64(0));\n' ...
    '            end\n'],k,10*k+1);
end

function s=mark_end(k,aux)
s=sprintf(['            if af7r58zzk_slot==%d\n' ...
    '                af7r58zzk_trace_mex(''mark'',uint64(obj.generator_request_count),uint64(%d),uint64(%d));\n' ...
    '            end\n'],k,10*k+2,aux);
end

function p0=find_gate_start(F,tokenPos)
before=F(1:tokenPos);
pat=sprintf('            if nargin>=4 && ~isempty(t_hard_cycle) && ...\n');
q=strfind(before,pat);
assert(~isempty(q),'[AF7R58ZZK-R3] deadline gate start not found.');
p0=q(end);
end

function R=insert_before_once(R,anchor,ins)
p=strfind(R,anchor);
assert(numel(p)==1,'[AF7R58ZZK-R3] insert-before anchor count=%d.',numel(p));
R=[R(1:p-1) ins R(p:end)];
end

function R=insert_after_once(R,anchor,ins)
p=strfind(R,anchor);
assert(numel(p)==1,'[AF7R58ZZK-R3] insert-after anchor count=%d.',numel(p));
e=p+length(anchor)-1;
R=[R(1:e) ins R(e+1:end)];
end

function R=replace_once(R,a,b)
p=strfind(R,a);
assert(numel(p)==1,'[AF7R58ZZK-R3] replace anchor count=%d.',numel(p));
R=[R(1:p-1) b R(p+length(a):end)];
end

function n=count_exact(s,pat)
n=numel(strfind(s,pat));
end

function s=cpp_source()
s = [ ...
'#include "mex.h"' newline ...
'#define WIN32_LEAN_AND_MEAN' newline ...
'#include <windows.h>' newline ...
'#include <stdint.h>' newline ...
'#include <string.h>' newline ...
'' newline ...
'struct Event { uint64_t seq, code, qpc, thr, proc, aux; };' newline ...
'static const size_t CAP=1024;' newline ...
'static Event ringbuf[CAP];' newline ...
'static size_t head=0, countEv=0;' newline ...
'static uint64_t qpc_now(){ LARGE_INTEGER x; QueryPerformanceCounter(&x); return (uint64_t)x.QuadPart; }' newline ...
'static uint64_t qpc_freq(){ LARGE_INTEGER x; QueryPerformanceFrequency(&x); return (uint64_t)x.QuadPart; }' newline ...
'static uint64_t thread_cycles(){ ULONG64 x=0; QueryThreadCycleTime(GetCurrentThread(),&x); return (uint64_t)x; }' newline ...
'static uint64_t process_cycles(){ ULONG64 x=0; QueryProcessCycleTime(GetCurrentProcess(),&x); return (uint64_t)x; }' newline ...
'static uint64_t scalar_u64(const mxArray* a){ if(mxIsUint64(a)&&mxGetNumberOfElements(a)==1) return *((uint64_t*)mxGetData(a)); return (uint64_t)mxGetScalar(a); }' newline ...
'static void push(uint64_t seq,uint64_t code,uint64_t aux){ Event e; e.seq=seq; e.code=code; e.qpc=qpc_now(); e.thr=thread_cycles(); e.proc=process_cycles(); e.aux=aux; ringbuf[head]=e; head=(head+1)%CAP; if(countEv<CAP) ++countEv; }' newline ...
'static mxArray* u64row4(uint64_t a,uint64_t b,uint64_t c,uint64_t d){ mxArray* y=mxCreateNumericMatrix(1,4,mxUINT64_CLASS,mxREAL); uint64_t* p=(uint64_t*)mxGetData(y); p[0]=a; p[1]=b; p[2]=c; p[3]=d; return y; }' newline ...
'void mexFunction(int nlhs,mxArray* plhs[],int nrhs,const mxArray* prhs[]){' newline ...
' if(nrhs<1||!mxIsChar(prhs[0])) mexErrMsgIdAndTxt("AF7R58ZZK:Args","command required");' newline ...
' char cmd[32]; mxGetString(prhs[0],cmd,sizeof(cmd));' newline ...
' if(strcmp(cmd,"reset")==0){ head=0; countEv=0; memset(ringbuf,0,sizeof(ringbuf)); return; }' newline ...
' if(strcmp(cmd,"mark")==0){ if(nrhs<4) mexErrMsgIdAndTxt("AF7R58ZZK:Args","mark needs seq,code,aux"); push(scalar_u64(prhs[1]),scalar_u64(prhs[2]),scalar_u64(prhs[3])); return; }' newline ...
' if(strcmp(cmd,"stamp")==0){ uint64_t a=qpc_now(); uint64_t b=thread_cycles(); uint64_t c=process_cycles(); uint64_t d=qpc_freq(); if(nlhs>0) plhs[0]=u64row4(a,b,c,d); return; }' newline ...
' if(strcmp(cmd,"dump")==0){ const char* fn[]={"freq","events"}; mxArray* st=mxCreateStructMatrix(1,1,2,fn); mxArray* fq=mxCreateNumericMatrix(1,1,mxUINT64_CLASS,mxREAL); *((uint64_t*)mxGetData(fq))=qpc_freq(); mxSetField(st,0,"freq",fq); mwSize N=(mwSize)countEv; mxArray* ev=mxCreateNumericMatrix(N,6,mxUINT64_CLASS,mxREAL); uint64_t* p=(uint64_t*)mxGetData(ev); size_t start=(head+CAP-countEv)%CAP; for(mwSize r=0;r<N;++r){ const Event& e=ringbuf[(start+r)%CAP]; p[r+N*0]=e.seq; p[r+N*1]=e.code; p[r+N*2]=e.qpc; p[r+N*3]=e.thr; p[r+N*4]=e.proc; p[r+N*5]=e.aux; } mxSetField(st,0,"events",ev); if(nlhs>0) plhs[0]=st; else mxDestroyArray(st); return; }' newline ...
' mexErrMsgIdAndTxt("AF7R58ZZK:Command","unknown command");' newline ...
'}' newline ...
];
end

function h=file_hash(path)
md=java.security.MessageDigest.getInstance('SHA-256');
fid=fopen(path,'rb'); assert(fid>=0); c=onCleanup(@()fclose(fid));
while true
    b=fread(fid,1024*1024,'*uint8'); if isempty(b), break; end
    md.update(b);
end
u=typecast(md.digest(),'uint8');
h=lower(reshape(dec2hex(u,2).',1,[]));
end

function s=read_utf8(path)
fid=fopen(path,'rt','n','UTF-8'); assert(fid>=0); c=onCleanup(@()fclose(fid));
s=fread(fid,'*char')';
end

function write_utf8(path,s)
fid=fopen(path,'wt','n','UTF-8'); assert(fid>=0); c=onCleanup(@()fclose(fid));
fwrite(fid,s,'char');
end

function say(fid,varargin)
fprintf(fid,[varargin{1} '\n'],varargin{2:end});
fprintf([varargin{1} '\n'],varargin{2:end});
end

function safe_fclose(fid)
try, fclose(fid); catch, end
end

function safe_rmpath(p)
try
    rmpath(p);
catch
end
end
