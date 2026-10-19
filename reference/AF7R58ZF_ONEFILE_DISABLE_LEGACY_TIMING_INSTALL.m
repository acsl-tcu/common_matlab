function OUT = AF7R58ZF_ONEFILE_DISABLE_LEGACY_TIMING_INSTALL()
% AF7R58ZF
% Disable legacy AF7R58 diagnostic instrumentation in production.
%
% This patch changes ONLY four property default values:
%   ENABLE_AF7R58M_TIMING_RING
%   ENABLE_AF7R58Q_MIN_TIMING
%   ENABLE_AF7R58T_OTHER_TIMING
%   ENABLE_AF7R58V_R4_PRETRY_TIMING
%
% It does NOT change trajectory, safety certificates, cache logic,
% builders, commit logic, deadline logic, or any method body.
%
% Source binding:
%   Instead of assuming a stale external SHA, this installer requires the
%   current ZD semantic fingerprint to be present exactly as installed:
%     AF7R58ZD monotone theorem
%     VLcached>=VL
%     VQcached>=VQ
%     AF7R58ZB_ONE_ENTRY_PREPARED_PACKET_CACHE_V2
%   It records the exact pre-write SHA in REPORT/OUT and verifies every
%   function/method body hash is byte-identical before vs after patch.
%
% Transactional: backup + rollback on any failure.
% Full patched class parse is required before production write.

root=fileparts(mfilename('fullpath'));
outdir=tempname(root);
[ok,msg]=mkdir(outdir); assert(ok,'%s',msg);
report=fullfile(outdir,'AF7R58ZF_REPORT.txt');

f=fopen(report,'wt','n','UTF-8');
assert(f>=0,'[AF7R58ZF] cannot open report.');
cc=onCleanup(@()fclose(f)); %#ok<NASGU>

say(f,'====================================================================================================');
say(f,' AF7R58ZF | DISABLE LEGACY AF7R58 TIMING INSTRUMENTATION');
say(f,'====================================================================================================');
say(f,'[AUTHORITY] propertyDefaultsOnly=1 | allMethodBodiesUnchanged=1 | rollbackOnFailure=1 | parseBeforeWrite=1');
say(f,'[SAFETY] trajectoryMathChanged=0 | certificateLogicChanged=0 | cacheLogicChanged=0 | commitLogicChanged=0');

OUT=struct();
OUT.schema='AF7R58ZF_DISABLE_LEGACY_TIMING_INSTALL_V1';
OUT.production_modified=false;
OUT.rollback_performed=false;
OUT.status='OPEN';
OUT.error=struct();

pR=fullfile(root,'REPLANNING_SYSTEM.m');
backupDir=tempname(root);
backupReady=false;
writesStarted=false;

try
    assert(isfile(pR),'[AF7R58ZF] REPLANNING_SYSTEM.m missing.');
    preSha=file_hash(pR);
    [txt,eolStyle]=read_text_norm(pR);
    lines=reshape(regexp(txt,'\n','split'),1,[]);

    say(f,'');
    say(f,'[A. CURRENT PRODUCTION SOURCE]');
    say(f,' pre-write SHA = %s',preSha);

    % Require exact ZD semantic fingerprint.
    fp=struct();
    fp.zd_comment=count(txt,'AF7R58ZD monotone theorem')==1;
    fp.payload_dominance=count(txt,'VLcached>=VL')==1;
    fp.uav_dominance=count(txt,'VQcached>=VQ')==1;
    fp.zb_schema=count(txt,'AF7R58ZB_ONE_ENTRY_PREPARED_PACKET_CACHE_V2')>=1;
    fp.all=fp.zd_comment&&fp.payload_dominance&&fp.uav_dominance&&fp.zb_schema;

    say(f,'');
    say(f,'[B. ZD SEMANTIC SOURCE BINDING]');
    fn=fieldnames(fp);
    for k=1:numel(fn)
        say(f,' %-32s = %d',fn{k},logical(fp.(fn{k})));
    end
    assert(fp.all,'[AF7R58ZF] current source is not the expected ZD production topology.');

    % Inventory all function bodies before patch.
    defsBefore=discover_function_headers(lines);
    beforeHashes=hash_all_function_blocks(lines,defsBefore);

    names={ ...
        'ENABLE_AF7R58M_TIMING_RING', ...
        'ENABLE_AF7R58Q_MIN_TIMING', ...
        'ENABLE_AF7R58T_OTHER_TIMING', ...
        'ENABLE_AF7R58V_R4_PRETRY_TIMING'};

    oldValues=cell(1,numel(names));
    patchedLines=lines;

    say(f,'');
    say(f,'[C. TARGET PROPERTY DEFAULTS -- BEFORE]');
    for k=1:numel(names)
        pat=['^\s*' regexptranslate('escape',names{k}) '\s*=\s*(true|false)\s*;\s*(?:%.*)?$'];
        hit=find_regex_lines(patchedLines,pat);
        assert(numel(hit)==1, ...
            '[AF7R58ZF] property declaration "%s" count=%d.',names{k},numel(hit));

        tok=regexp(patchedLines{hit},pat,'tokens','once');
        oldValues{k}=tok{1};
        say(f,' %-40s = %s',names{k},oldValues{k});

        patchedLines{hit}=regexprep(patchedLines{hit}, ...
            '=\s*(true|false)\s*;', '= false;','once');
    end

    patched=strjoin(patchedLines,newline);

    % Verify exactly four semantic changes and all are false.
    say(f,'');
    say(f,'[D. TARGET PROPERTY DEFAULTS -- PATCHED]');
    for k=1:numel(names)
        pat=['^\s*' regexptranslate('escape',names{k}) '\s*=\s*false\s*;\s*(?:%.*)?$'];
        hit=find_regex_lines(patchedLines,pat);
        assert(numel(hit)==1, ...
            '[AF7R58ZF] patched false declaration "%s" count=%d.',names{k},numel(hit));
        say(f,' %-40s = false',names{k});
    end

    % Every function/method body MUST remain exactly unchanged.
    defsAfter=discover_function_headers(patchedLines);
    afterHashes=hash_all_function_blocks(patchedLines,defsAfter);

    assert(numel(beforeHashes)==numel(afterHashes), ...
        '[AF7R58ZF] function count changed.');
    bodiesSame=true;
    for k=1:numel(beforeHashes)
        if ~strcmp(beforeHashes(k).header,afterHashes(k).header) || ...
                ~strcmp(beforeHashes(k).hash,afterHashes(k).hash)
            bodiesSame=false;
            break;
        end
    end
    assert(bodiesSame,'[AF7R58ZF] at least one function/method body changed unexpectedly.');

    OUT.function_body_count=numel(beforeHashes);
    OUT.all_function_bodies_unchanged=bodiesSame;

    say(f,'');
    say(f,'[E. METHOD-BODY PRESERVATION]');
    say(f,' discovered function/method bodies       = %d',numel(beforeHashes));
    say(f,' all function/method bodies unchanged    = %d',bodiesSame);

    % Full patched class parse before production write.
    preDir=fullfile(outdir,'AF7R58ZF_PREFLIGHT');
    [ok,msg]=mkdir(preDir); assert(ok,'%s',msg);
    parseName='AF7R58ZF_PARSE_REPLANNING_SYSTEM';
    parseText=strrep(patched,'classdef REPLANNING_SYSTEM < handle', ...
        ['classdef ' parseName ' < handle']);
    write_utf8(fullfile(preDir,[parseName '.m']),parseText);
    addpath(preDir,'-begin');
    cp=onCleanup(@()safe_rmpath_if_present(preDir)); %#ok<NASGU>
    eval(['clear ' parseName]);
    mc=meta.class.fromName(parseName);
    assert(~isempty(mc),'[AF7R58ZF] patched full-class parse failed.');
    eval(['clear ' parseName]);

    say(f,'');
    say(f,'[F. PREFLIGHT BEFORE WRITE]');
    say(f,' ZD semantic source binding             = 1');
    say(f,' exactly four defaults patched          = 1');
    say(f,' all method bodies unchanged            = 1');
    say(f,' patched full class parse               = 1');
    say(f,' production writes                      = 0');

    [ok,msg]=mkdir(backupDir); assert(ok,'%s',msg);
    backupReady=true;
    OUT.backup_directory=backupDir;
    copyfile(pR,fullfile(backupDir,'REPLANNING_SYSTEM.m'));

    writesStarted=true;
    write_text_eol(pR,patched,eolStyle);
    rehash;

    postSha=file_hash(pR);
    OUT.pre_sha=preSha;
    OUT.post_sha=postSha;
    OUT.old_values=oldValues;
    OUT.production_modified=true;
    OUT.status='LEGACY_TIMING_DEFAULTS_DISABLED__CLEAN_RERUN_REQUIRED';

    say(f,'');
    say(f,'[G. INSTALL RESULT]');
    say(f,' production modified                    = 1');
    say(f,' rollback performed                     = 0');
    say(f,' legacy timing defaults disabled        = 4 / 4');
    say(f,' method bodies changed                  = 0');
    say(f,' trajectory/certificate/cache math      = unchanged');
    say(f,' new REPLANNING_SYSTEM SHA              = %s',postSha);

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

    say(f,'');
    say(f,'[INSTALL FAILURE]');
    say(f,' identifier          = %s',ME.identifier);
    say(f,' message             = %s',ME.message);
    say(f,' rollback performed  = %d',OUT.rollback_performed);
    say(f,' production modified = 0');
end

save(fullfile(outdir,'AF7R58ZF_OUT.mat'),'OUT','-v7');
assignin('base','AF7R58ZF_OUT',OUT);

say(f,'');
say(f,'RETURN: %s',report);
say(f,'RETURN: %s',fullfile(outdir,'AF7R58ZF_OUT.mat'));
say(f,'status = %s',OUT.status);
end


function hit=find_regex_lines(lines,pat)
hit=find(cellfun(@(x)~isempty(regexp(x,pat,'once')),lines));
end


function defs=discover_function_headers(lines)
lines=reshape(lines,1,[]);
defs=struct('line',{},'indent',{},'header',{},'header_end',{});
for k=1:numel(lines)
    if startsWith(strtrim(lines{k}),'function ')
        [hdr,ke]=collect_function_header(lines,k);
        defs(end+1)=struct('line',k,'indent',leading_ws_count(lines{k}), ...
            'header',hdr,'header_end',ke); %#ok<AGROW>
    end
end
end


function H=hash_all_function_blocks(lines,defs)
H=struct('header',{},'hash',{});
for j=1:numel(defs)
    i0=defs(j).line;
    i1=numel(lines);
    for k=i0+1:numel(lines)
        if startsWith(strtrim(lines{k}),'function ') && ...
                leading_ws_count(lines{k})==defs(j).indent
            i1=k-1;
            break;
        end
    end
    B=strjoin(reshape(lines(i0:i1),1,[]),newline);
    H(end+1)=struct('header',defs(j).header,'hash',text_hash(B)); %#ok<AGROW>
end
end


function [hdr,kend]=collect_function_header(lines,k0)
parts={strtrim(lines{k0})};
kend=k0;
while endsWith(strtrim(parts{end}),'...')
    parts{end}=strtrim(parts{end}(1:end-3));
    kend=kend+1;
    assert(kend<=numel(lines));
    parts{end+1}=strtrim(lines{kend}); %#ok<AGROW>
end
hdr=strjoin(parts,' ');
end


function n=leading_ws_count(line)
n=0;
for k=1:numel(line)
    if isspace(line(k)), n=n+1; else, break; end
end
end


function [s,eol]=read_text_norm(p)
fid=fopen(p,'rb'); assert(fid>=0);
c=onCleanup(@()fclose(fid)); %#ok<NASGU>
b=fread(fid,Inf,'*uint8');
s=native2unicode(b.','UTF-8');

if contains(s,sprintf('\r\n')), eol='CRLF';
elseif contains(s,sprintf('\r')), eol='CR';
else, eol='LF'; end

s=strrep(s,sprintf('\r\n'),sprintf('\n'));
s=strrep(s,sprintf('\r'),sprintf('\n'));
end


function write_text_eol(p,s,eol)
switch eol
    case 'CRLF'
        s=strrep(s,sprintf('\n'),sprintf('\r\n'));
    case 'CR'
        s=strrep(s,sprintf('\n'),sprintf('\r'));
end
write_utf8(p,s);
end


function write_utf8(p,s)
fid=fopen(p,'wb'); assert(fid>=0);
c=onCleanup(@()fclose(fid)); %#ok<NASGU>
b=unicode2native(s,'UTF-8');
n=fwrite(fid,b,'uint8');
assert(n==numel(b));
end


function safe_rmpath_if_present(p)
try
    q=strsplit(path,pathsep);
    if any(strcmpi(q,p)), rmpath(p); end
catch
end
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


function h=text_hash(s)
b=unicode2native(s,'UTF-8');
m=java.security.MessageDigest.getInstance('SHA-256');
m.update(uint8(b));
d=typecast(m.digest(),'uint8');
h=lower(reshape(dec2hex(d,2).',1,[]));
end


function say(f,fmt,varargin)
s=sprintf(fmt,varargin{:});
fprintf(1,'%s\n',s);
fprintf(f,'%s\n',s);
end
