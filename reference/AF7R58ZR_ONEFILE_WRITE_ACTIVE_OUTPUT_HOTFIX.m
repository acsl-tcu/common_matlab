function OUT = AF7R58ZR_ONEFILE_WRITE_ACTIVE_OUTPUT_HOTFIX()
% AF7R58ZR
% One-line hotfix for AF7R58ZP write_active_output.
%
% Bug:
%   varargin_xd is the declared numeric 4th input itself, not a cell array.
%   Therefore varargin_xd{1} is invalid for a double vector.
%
% Fix:
%   xd=double(varargin_xd(:));
%
% No trajectory, certificate, trigger, cache, observer, controller, or
% deadline mathematics are changed.

PRE_SHA  = 'afdab70a7a68f545dac3dede9814c95b703cb7823b9da5374ed05004735a1c11';
POST_SHA = 'c7baa3c7dda80bbacb412443a035fd7c5ae7afbc6c95a46eddc4d82b69d88289';

OUT=struct();
OUT.schema='AF7R58ZR_WRITE_ACTIVE_OUTPUT_HOTFIX_V1';
OUT.production_modified=false;
OUT.rollback_performed=false;
OUT.status='OPEN';
OUT.error=struct();

pR=which('REPLANNING_SYSTEM');
assert(~isempty(pR)&&isfile(pR),'[AF7R58ZR] REPLANNING_SYSTEM unresolved.');
root=fileparts(pR);
outdir=tempname(root);
[ok,msg]=mkdir(outdir); assert(ok,'%s',msg);
report=fullfile(outdir,'AF7R58ZR_REPORT.txt');
fid=fopen(report,'wt','n','UTF-8'); assert(fid>=0);
cf=onCleanup(@()safe_fclose(fid)); %#ok<NASGU>

say(fid,'====================================================================================================');
say(fid,' AF7R58ZR | WRITE_ACTIVE_OUTPUT ONE-LINE HOTFIX');
say(fid,'====================================================================================================');
say(fid,'[AUTHORITY] exactPostZPSHA=1 | oneExactReplacement=1 | parseBeforeWrite=1 | rollback=1');
say(fid,'[MATH] trajectoryChanged=0 | certificateChanged=0 | triggerChanged=0 | cacheChanged=0');

backupDir=tempname(root);
backupReady=false;
writesStarted=false;

try
    sha=file_hash(pR);
    OUT.pre_sha=sha;
    OUT.production_path=pR;

    say(fid,'');
    say(fid,'[A. CURRENT SOURCE]');
    say(fid,' path = %s',pR);
    say(fid,' SHA  = %s',sha);
    say(fid,' exact AF7R58ZP source = %d',strcmp(sha,PRE_SHA));
    assert(strcmp(sha,PRE_SHA), ...
        '[AF7R58ZR] current source is not exact AF7R58ZP authority.');

    raw=read_utf8(pR);
    old='xd=double(varargin_xd{1}(:));';
    new='xd=double(varargin_xd(:));';

    n=count(raw,old);
    say(fid,'');
    say(fid,'[B. EXACT BUG BINDING]');
    say(fid,' buggy expression count = %d',n);
    assert(n==1,'[AF7R58ZR] buggy expression count=%d, expected 1.',n);

    patched=strrep(raw,old,new);

    assert(count(patched,old)==0,'[AF7R58ZR] buggy cell indexing remains.');
    assert(count(patched,new)==1,'[AF7R58ZR] corrected expression count is not 1.');

    restored=strrep(patched,new,old);
    assert(strcmp(restored,raw), ...
        '[AF7R58ZR] patch is not exactly reversible to original source.');

    parseDir=fullfile(outdir,'AF7R58ZR_PREFLIGHT');
    [ok,msg]=mkdir(parseDir); assert(ok,'%s',msg);
    parseName='AF7R58ZR_PARSE_REPLANNING_SYSTEM';
    parseText=strrep(patched,'classdef REPLANNING_SYSTEM < handle', ...
        ['classdef ' parseName ' < handle']);
    parseFile=fullfile(parseDir,[parseName '.m']);
    write_utf8(parseFile,parseText);
    addpath(parseDir,'-begin');
    cp=onCleanup(@()safe_rmpath(parseDir)); %#ok<NASGU>
    eval(['clear ' parseName]);
    mc=meta.class.fromName(parseName);
    assert(~isempty(mc),'[AF7R58ZR] patched class parse failed.');
    eval(['clear ' parseName]);

    payload=fullfile(outdir,'REPLANNING_SYSTEM_AF7R58ZR_PAYLOAD.m');
    write_utf8(payload,patched);
    payloadSha=file_hash(payload);
    OUT.payload_sha=payloadSha;

    say(fid,'');
    say(fid,'[C. PREFLIGHT BEFORE WRITE]');
    say(fid,' exact one-line replacement = 1');
    say(fid,' reversible source check     = 1');
    say(fid,' full MATLAB class parse     = 1');
    say(fid,' payload SHA                 = %s',payloadSha);
    say(fid,' expected post SHA           = %s',POST_SHA);
    assert(strcmp(payloadSha,POST_SHA), ...
        '[AF7R58ZR] payload SHA mismatch.');
    say(fid,' production writes           = 0');

    [ok,msg]=mkdir(backupDir); assert(ok,'%s',msg);
    backupReady=true;
    OUT.backup_directory=backupDir;
    copyfile(pR,fullfile(backupDir,'REPLANNING_SYSTEM.m'));

    writesStarted=true;
    write_utf8(pR,patched);
    rehash;

    postSha=file_hash(pR);
    assert(strcmp(postSha,POST_SHA),'[AF7R58ZR] post-write SHA mismatch.');

    OUT.post_sha=postSha;
    OUT.production_modified=true;
    OUT.status='WRITE_ACTIVE_OUTPUT_HOTFIX_INSTALLED__RERUN_NOW';

    say(fid,'');
    say(fid,'[D. INSTALL RESULT]');
    say(fid,' production modified = 1');
    say(fid,' rollback performed   = 0');
    say(fid,' corrected line       = xd=double(varargin_xd(:));');
    say(fid,' post SHA             = %s',postSha);

catch ME
    if backupReady && writesStarted
        try
            copyfile(fullfile(backupDir,'REPLANNING_SYSTEM.m'),pR,'f');
            rehash;
            OUT.rollback_performed=true;
        catch
        end
    end
    OUT.production_modified=false;
    OUT.status='INSTALL_FAILED__ROLLBACK_COMPLETE';
    OUT.error=struct('identifier',ME.identifier,'message',ME.message,'stack',{ME.stack});

    say(fid,'');
    say(fid,'[INSTALL FAILURE]');
    say(fid,' identifier          = %s',ME.identifier);
    say(fid,' message             = %s',ME.message);
    say(fid,' rollback performed  = %d',OUT.rollback_performed);
    say(fid,' production modified = 0');
end

save(fullfile(outdir,'AF7R58ZR_OUT.mat'),'OUT','-v7');
assignin('base','AF7R58ZR_OUT',OUT);
say(fid,'');
say(fid,'RETURN: %s',report);
say(fid,'RETURN: %s',fullfile(outdir,'AF7R58ZR_OUT.mat'));
say(fid,'status = %s',OUT.status);
end


function s=read_utf8(p)
fid=fopen(p,'rb'); assert(fid>=0);
c=onCleanup(@()fclose(fid)); %#ok<NASGU>
b=fread(fid,Inf,'*uint8');
s=native2unicode(b.','UTF-8');
end


function write_utf8(p,s)
fid=fopen(p,'wb'); assert(fid>=0);
c=onCleanup(@()fclose(fid)); %#ok<NASGU>
b=unicode2native(s,'UTF-8');
n=fwrite(fid,b,'uint8');
assert(n==numel(b));
end


function h=file_hash(p)
fid=fopen(p,'rb'); assert(fid>=0);
c=onCleanup(@()fclose(fid)); %#ok<NASGU>
b=fread(fid,Inf,'*uint8');
m=java.security.MessageDigest.getInstance('SHA-256');
m.update(b);
d=typecast(m.digest(),'uint8');
h=lower(reshape(dec2hex(d,2).',1,[]));
end


function safe_rmpath(p)
try
    q=strsplit(path,pathsep);
    if any(strcmpi(q,p)),rmpath(p);end
catch
end
end


function safe_fclose(fid)
try
    if fid>=0,fclose(fid);end
catch
end
end


function say(fid,fmt,varargin)
s=sprintf(fmt,varargin{:});
fprintf(1,'%s\n',s);
fprintf(fid,'%s\n',s);
end
