function OUT = AF7R58ZZM_REMOVE_R3_TIMING_INSTRUMENTATION()
% AF7R58ZZM
% Remove ONLY AF7R58ZZK-R3 diagnostic timing instrumentation and restore
% the exact AF7R58ZZH authority source.
%
% This does NOT change trajectory/certificate/generator/sensing mathematics.
% It removes only:
%   - R3 rotating timing selector
%   - 7 start marks
%   - 7 end marks
%   - one catch-path end mark
%
% Exact current instrumented source SHA (from AF7R58ZZL-R4):
%   0f62bade234b28c3e9b128c6080e2109e09e75d404236e3077b10176b89afbad
%
% Exact restored AF7R58ZZH raw UTF-8/LF SHA:
%   2a0013a583cbcff3afd62e4495fa5e784427132390616ec5886000f134f2c05f

PRE_SHA='0f62bade234b28c3e9b128c6080e2109e09e75d404236e3077b10176b89afbad';
POST_SHA='2a0013a583cbcff3afd62e4495fa5e784427132390616ec5886000f134f2c05f';

OUT=struct('schema','AF7R58ZZM_REMOVE_R3_TIMING_V1', ...
    'production_modified',false,'rollback_performed',false, ...
    'status','OPEN','error',struct());

pR=which('REPLANNING_SYSTEM');
assert(~isempty(pR)&&isfile(pR), ...
    '[AF7R58ZZM] REPLANNING_SYSTEM unresolved.');
root=fileparts(pR);

outdir=tempname(root);
[ok,msg]=mkdir(outdir);
assert(ok,'%s',msg);

report=fullfile(outdir,'AF7R58ZZM_REPORT.txt');
matp=fullfile(outdir,'AF7R58ZZM_OUT.mat');
backup=fullfile(outdir,'REPLANNING_SYSTEM_R3_INSTRUMENTED_BACKUP.m');

fid=fopen(report,'wt','n','UTF-8');
assert(fid>=0);
cf=onCleanup(@()safe_fclose(fid)); %#ok<NASGU>

say(fid,'====================================================================================================');
say(fid,' AF7R58ZZM | REMOVE R3 HIGH-RES TIMING INSTRUMENTATION');
say(fid,'====================================================================================================');
say(fid,'[AUTHORITY] exactCurrentSHA=1 | reverseStripExact=1 | parseBeforeWrite=1 | rollbackOnFailure=1');
say(fid,'[SAFETY] trajectoryMathChanged=0 | certificateMathChanged=0 | generatorMathChanged=0 | deadlineChanged=0');
say(fid,'[SENSING] detectedOnlyGeneratorRequired=1 | unseenTruthPreplan=0');

writeStarted=false;

try
    pre=file_hash(pR);
    OUT.pre_sha=pre;
    OUT.production_path=pR;

    say(fid,'');
    say(fid,'[A. CURRENT INSTRUMENTED SOURCE]');
    say(fid,' path = %s',pR);
    say(fid,' SHA  = %s',pre);

    assert(strcmp(pre,PRE_SHA), ...
        '[AF7R58ZZM] current source is not the exact R3-instrumented authority.');

    R=read_utf8_textmode(pR);

    assert(count_exact(R,'AF7R58ZZK_R3_ROTATING_WINDOW')==1, ...
        '[AF7R58ZZM] R3 selector marker not unique.');
    assert(count_exact(R,'af7r58zzk_trace_mex(''mark''')==15, ...
        '[AF7R58ZZM] R3 timing mark count is not 15.');

    assert(contains(R,'The generator receives ONLY sensor-known obstacles:'));
    assert(contains(R,'''detected_only_obstacle_contract'',true'));
    assert(contains(R,'Initial known set is empty.'));
    assert(contains(R,'raw_sensor_gate alone may admit a previously unknown obstacle.'));

    copyfile(pR,backup,'f');
    OUT.backup_path=backup;

    [R2,stripOK,D]=strip_r3_instrumentation(R);
    OUT.strip_detail=D;
    assert(stripOK,'[AF7R58ZZM] exact R3 reverse-strip failed.');

    assert(~contains(R2,'AF7R58ZZK_R3_ROTATING_WINDOW'));
    assert(~contains(R2,'af7r58zzk_trace_mex(''mark'''));

    % Stage as exact UTF-8 bytes with LF. Do NOT use Windows text-mode write,
    % otherwise CRLF normalization would change the exact ZZH raw SHA.
    staged=fullfile(outdir,'REPLANNING_SYSTEM_RESTORED_ZZH.m');
    write_utf8_binary_lf(staged,R2);
    stageSHA=file_hash(staged);
    OUT.staged_post_sha=stageSHA;

    assert(strcmp(stageSHA,POST_SHA), ...
        '[AF7R58ZZM] reverse-stripped source does not equal exact ZZH authority.');

    % Full parse before first production write.
    parseDir=fullfile(outdir,'PARSE');
    [ok,msg]=mkdir(parseDir);
    assert(ok,'%s',msg);

    parseName='AF7R58ZZM_PARSE_REPLANNING_SYSTEM';
    parseText=strrep(R2,'classdef REPLANNING_SYSTEM < handle', ...
        ['classdef ' parseName ' < handle']);
    assert(~strcmp(parseText,R2), ...
        '[AF7R58ZZM] temporary class rename failed.');

    parseFile=fullfile(parseDir,[parseName '.m']);
    write_utf8_binary_lf(parseFile,parseText);

    addpath(parseDir,'-begin');
    cp=onCleanup(@()safe_rmpath(parseDir)); %#ok<NASGU>
    eval(['clear ' parseName]);
    mc=meta.class.fromName(parseName);
    assert(~isempty(mc),'[AF7R58ZZM] restored full class parse failed.');
    eval(['clear ' parseName]);

    say(fid,'');
    say(fid,'[B. PREFLIGHT BEFORE PRODUCTION WRITE]');
    say(fid,' R3 selector removed                   = 1');
    say(fid,' all 15 R3 mark sites removed         = 1');
    say(fid,' exact restored ZZH SHA               = 1');
    say(fid,' full restored class parse            = 1');
    say(fid,' finite-sensing contract retained     = 1');
    say(fid,' production writes                    = 0');

    % First and only production write.
    writeStarted=true;
    write_utf8_binary_lf(pR,R2);
    OUT.production_modified=true;

    post=file_hash(pR);
    OUT.post_sha=post;
    assert(strcmp(post,POST_SHA), ...
        '[AF7R58ZZM] production post-write SHA mismatch.');

    say(fid,'');
    say(fid,'[C. RESTORE RESULT]');
    say(fid,' post SHA                             = %s',post);
    say(fid,' exact AF7R58ZZH restored             = 1');
    say(fid,' timing instrumentation remaining     = 0');
    say(fid,' 4-ms deadline changed                = 0');
    say(fid,' safety/certificate math changed      = 0');
    say(fid,' finite-sensing contract retained     = 1');
    say(fid,' MEX file deleted                     = 0');
    say(fid,' NOTE: af7r58zzk_trace_mex may remain on disk but is no longer called.');

    OUT.status='R3_TIMING_REMOVED__EXACT_ZZH_RESTORED__CLEAN_RERUN_REQUIRED';

catch ME
    OUT.error=struct('identifier',ME.identifier, ...
        'message',ME.message,'stack',{ME.stack});

    if writeStarted && isfile(backup)
        try
            copyfile(backup,pR,'f');
            OUT.rollback_performed=true;
            OUT.production_modified=false;
        catch
        end
    end

    OUT.status='REMOVE_FAILED__ROLLBACK_ATTEMPTED';

    say(fid,'');
    say(fid,'[FAILURE]');
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

assignin('base','AF7R58ZZM_OUT',OUT);
end

function [R,ok,D]=strip_r3_instrumentation(R)
ok=false;
D=struct('entry',0,'mark_start',zeros(1,7),'mark_end',zeros(1,7), ...
    'catch_end',0,'residual_marker',true,'residual_mark_call',true);

entry=sprintf([ ...
    '\n            %% AF7R58ZZK_R3_ROTATING_WINDOW: diagnostic only.\n' ...
    '            %% Rotate by completed commit-path outcomes, not raw requests.\n' ...
    '            af7r58zzk_attempt=double(obj.generator_validation_fail_count)+ ...\n' ...
    '                double(obj.generator_validation_pass_count)+1;\n' ...
    '            af7r58zzk_slot=min(7,max(1,ceil(af7r58zzk_attempt/2)));\n']);

D.entry=count_exact(R,entry);
if D.entry~=1, return; end
R=replace_once(R,entry,'');

for k=1:7
    a=mark_start(k);
    D.mark_start(k)=count_exact(R,a);
    if D.mark_start(k)~=1, return; end
    R=replace_once(R,a,'');

    b=mark_end(k,0);
    D.mark_end(k)=count_exact(R,b);
    if D.mark_end(k)~=1, return; end
    R=replace_once(R,b,'');
end

catchEnd=sprintf([ ...
    '\n                if af7r58zzk_slot==7\n' ...
    '                    af7r58zzk_trace_mex(''mark'',uint64(obj.generator_request_count),uint64(72),uint64(1));\n' ...
    '                end']);

D.catch_end=count_exact(R,catchEnd);
if D.catch_end~=1, return; end
R=replace_once(R,catchEnd,'');

D.residual_marker=contains(R,'AF7R58ZZK_R3_ROTATING_WINDOW');
D.residual_mark_call=contains(R,'af7r58zzk_trace_mex(''mark''');

if D.residual_marker || D.residual_mark_call
    return;
end

ok=true;
end

function s=mark_start(k)
s=sprintf([ ...
    '            if af7r58zzk_slot==%d\n' ...
    '                af7r58zzk_trace_mex(''mark'',uint64(obj.generator_request_count),uint64(%d),uint64(0));\n' ...
    '            end\n'],k,10*k+1);
end

function s=mark_end(k,aux)
s=sprintf([ ...
    '            if af7r58zzk_slot==%d\n' ...
    '                af7r58zzk_trace_mex(''mark'',uint64(obj.generator_request_count),uint64(%d),uint64(%d));\n' ...
    '            end\n'],k,10*k+2,aux);
end

function R=replace_once(R,a,b)
p=strfind(R,a);
assert(numel(p)==1,'[AF7R58ZZM] reverse-strip pattern count=%d.',numel(p));
R=[R(1:p-1) b R(p+length(a):end)];
end

function n=count_exact(s,pat)
n=numel(strfind(s,pat));
end

function s=read_utf8_textmode(path)
fid=fopen(path,'rt','n','UTF-8');
assert(fid>=0);
c=onCleanup(@()fclose(fid)); %#ok<NASGU>
s=fread(fid,'*char')';
end

function write_utf8_binary_lf(path,s)
% The input string is canonical LF text. Binary mode preserves those bytes.
bytes=unicode2native(s,'UTF-8');
fid=fopen(path,'wb');
assert(fid>=0);
c=onCleanup(@()fclose(fid)); %#ok<NASGU>
fwrite(fid,uint8(bytes),'uint8');
end

function h=file_hash(path)
md=java.security.MessageDigest.getInstance('SHA-256');
fid=fopen(path,'rb');
assert(fid>=0);
c=onCleanup(@()fclose(fid)); %#ok<NASGU>
while true
    b=fread(fid,1024*1024,'*uint8');
    if isempty(b), break; end
    md.update(b);
end
u=typecast(md.digest(),'uint8');
h=lower(reshape(dec2hex(u,2).',1,[]));
end

function say(fid,varargin)
fprintf(fid,[varargin{1} '\n'],varargin{2:end});
fprintf([varargin{1} '\n'],varargin{2:end});
end

function safe_rmpath(p)
try
    rmpath(p);
catch
end
end

function safe_fclose(fid)
try
    fclose(fid);
catch
end
end
