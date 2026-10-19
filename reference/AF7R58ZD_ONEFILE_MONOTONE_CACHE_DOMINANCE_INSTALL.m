function OUT = AF7R58ZD_ONEFILE_MONOTONE_CACHE_DOMINANCE_INSTALL()
% AF7R58ZD
% Exact source-bound patch on top of AF7R58ZB.
%
% Purpose:
%   The ZB cache was safe but had 0/7 hits because payload speed bound
%   changed on every cache check. ZD replaces exact speed equality in the
%   PREPARED-cache gate with theorem-safe monotone dominance:
%
%       cached VL >= current VL
%       cached VQ >= current VQ
%
%   The cached sweep / obstacle packet / sensor certificate are validated
%   against the CACHED bounds, not the smaller current bounds. Therefore the
%   reused packet remains at least as conservative as required now.
%
% Important:
%   - ACTIVE runtime reuse gate is unchanged.
%   - Builders are unchanged.
%   - Geometry / trajectory generation is unchanged.
%   - If either current bound increases above cached, reuse still FAILS CLOSED.
%   - Live numeric deps, obstacle map and active authority still require exact
%     equality as in ZB.
%   - Full runtime dependency signature is still recaptured live on cache hit.
%
% Exact pre-source SHA:
%   4767fd66d50247a5d848eba2e6a3b3af30f098c3a9fc3b8a206b4d50c62a4e72

root=fileparts(mfilename('fullpath'));
outdir=tempname(root);
[ok,msg]=mkdir(outdir); assert(ok,'%s',msg);
report=fullfile(outdir,'AF7R58ZD_REPORT.txt');

f=fopen(report,'wt','n','UTF-8');
assert(f>=0,'[AF7R58ZD] cannot open report.');
cc=onCleanup(@()fclose(f)); %#ok<NASGU>

say(f,'====================================================================================================');
say(f,' AF7R58ZD | MONOTONE CONSERVATIVE PREPARED-CACHE DOMINANCE INSTALL');
say(f,'====================================================================================================');
say(f,'[AUTHORITY] sourceBound=1 | rollbackOnFailure=1 | parseBeforeWrite=1');
say(f,'[SAFETY] cachedSpeedDominatesCurrent=1 | exactLiveDeps=1 | exactObstacleMap=1 | failClosedOnIncrease=1');

OUT=struct();
OUT.schema='AF7R58ZD_MONOTONE_CACHE_DOMINANCE_INSTALL_V1';
OUT.production_modified=false;
OUT.rollback_performed=false;
OUT.status='OPEN';
OUT.error=struct();
OUT.output_directory=outdir;

pR=fullfile(root,'REPLANNING_SYSTEM.m');
expected='4767fd66d50247a5d848eba2e6a3b3af30f098c3a9fc3b8a206b4d50c62a4e72';

backupDir=tempname(root);
backupReady=false;
writesStarted=false;

try
    assert(isfile(pR),'[AF7R58ZD] REPLANNING_SYSTEM.m missing.');
    h0=file_hash(pR);

    say(f,'');
    say(f,'[A. EXACT ZB SOURCE BINDING]');
    say(f,' source SHA exact = %d | %s',strcmp(h0,expected),h0);
    assert(strcmp(h0,expected),'[AF7R58ZD] production source is not exact ZB.');

    [txt,eolStyle]=read_text_norm(pR);
    lines=reshape(regexp(txt,'\n','split'),1,[]);
    defs=discover_function_headers(lines);

    protected={ ...
        'can_reuse_reference_only_runtime_packet_exact', ...
        'reuse_reference_only_runtime_packet_exact', ...
        'build_trigger_sweep_for_target', ...
        'prepare_obstacle_packet_fast_target', ...
        'build_sensor_horizon_for_target'};

    before=struct();
    say(f,'');
    say(f,'[B. PROTECTED FUNCTION HASHES -- BEFORE]');
    for k=1:numel(protected)
        [b,i0,i1,~]=method_block_by_name(lines,defs,protected{k});
        key=matlab.lang.makeValidName(protected{k});
        before.(key)=text_hash(strjoin(b,newline));
        say(f,' %-48s L%d:L%d | %s',protected{k},i0,i1,before.(key));
    end

    [G,g0,g1,~]=method_block_by_name(lines,defs, ...
        'can_reuse_reference_only_prepared_packet_cache_exact');
    G=reshape(G,1,[]);
    oldGateHash=text_hash(strjoin(G,newline));
    OUT.cache_gate_hash_before=oldGateHash;

    % Exact ZB anchors.
    pVL=find_unique_contains(G,'VL=double(newSC.payload_speed_UB_m_s);');
    pVQ=find_unique_contains(G,'VQ=double(newSC.uav_speed_UB_m_s);');
    pR5=find_unique_contains(G,'reason=5;');
    pEqVL=find_unique_contains(G, ...
        'if ~isequal(double(oldSC.payload_speed_UB_m_s),VL), return; end');
    pR6=find_unique_contains(G,'reason=6;');
    pEqVQ=find_unique_contains(G, ...
        'if ~isequal(double(oldSC.uav_speed_UB_m_s),VQ), return; end');
    pDt=find_unique_contains(G,'dt=live(1);');
    pPay=find_unique_contains(G,'pay=VL*dt/2;');
    pUav=find_unique_contains(G,'uav=VQ*dt/2;');
    pCab=find_unique_contains(G,'cab=max(VL,VQ)*dt/2;');
    pSweepPeriod=find_unique_contains(G, ...
        'isequal(double(S.sensor_one_period_m),VQ*dt))');
    pH=find_unique_contains(G,'H=live(6); Rs=live(7); L=live(8);');
    pHMargin=find_unique_contains(G,'horizonMargin=Rs-(rho+VQ*H);');
    pDet=find_unique_contains(G,'dDetLB=Rs-VQ*dt;');
    pIfVQ=find_unique_contains(G,'if VQ>0');
    pLead=find_unique_contains(G,'collisionLead=safeClear/VQ;');
    pTVL=find_unique_contains(G, ...
        'isfield(T,''VL'') && isequal(double(T.VL),VL) && ...');
    pTVQ=find_unique_contains(G, ...
        'isfield(T,''VQ'') && isequal(double(T.VQ),VQ) && ...');

    assert(pVL<pVQ && pVQ<pR5 && pR5<pEqVL && pEqVL<pR6 && ...
        pR6<pEqVQ && pEqVQ<pDt && pDt<pPay && pPay<pUav && ...
        pUav<pCab && pCab<pSweepPeriod && pSweepPeriod<pH && ...
        pH<pHMargin && pHMargin<pDet && pDet<pIfVQ && ...
        pIfVQ<pLead && pLead<pTVL && pTVL<pTVQ, ...
        '[AF7R58ZD] ZB cache-gate topology is unexpected.');

    % Insert cached bounds after current bounds.
    cachedCode={ ...
        '            VLcached=double(oldSC.payload_speed_UB_m_s);'; ...
        '            VQcached=double(oldSC.uav_speed_UB_m_s);'};
    G=insert_after(G,pVQ,cachedCode);

    % Re-find after insertion and replace exact equality by dominance.
    pEqVL=find_unique_contains(G, ...
        'if ~isequal(double(oldSC.payload_speed_UB_m_s),VL), return; end');
    G=replace_line(G,pEqVL, ...
        '            if ~(isfinite(VLcached)&&VLcached>=VL), return; end');

    pEqVQ=find_unique_contains(G, ...
        'if ~isequal(double(oldSC.uav_speed_UB_m_s),VQ), return; end');
    G=replace_line(G,pEqVQ, ...
        '            if ~(isfinite(VQcached)&&VQcached>=VQ), return; end');

    % Validate cached artifacts against the cached dominating speed bounds.
    replacements={ ...
        'pay=VL*dt/2;','pay=VLcached*dt/2;'; ...
        'uav=VQ*dt/2;','uav=VQcached*dt/2;'; ...
        'cab=max(VL,VQ)*dt/2;','cab=max(VLcached,VQcached)*dt/2;'; ...
        'isequal(double(S.sensor_one_period_m),VQ*dt))', ...
            'isequal(double(S.sensor_one_period_m),VQcached*dt))'; ...
        'horizonMargin=Rs-(rho+VQ*H);','horizonMargin=Rs-(rho+VQcached*H);'; ...
        'dDetLB=Rs-VQ*dt;','dDetLB=Rs-VQcached*dt;'; ...
        'if VQ>0','if VQcached>0'; ...
        'collisionLead=safeClear/VQ;','collisionLead=safeClear/VQcached;'; ...
        'isfield(T,''VL'') && isequal(double(T.VL),VL) && ...', ...
            'isfield(T,''VL'') && isequal(double(T.VL),VLcached) && ...'; ...
        'isfield(T,''VQ'') && isequal(double(T.VQ),VQ) && ...', ...
            'isfield(T,''VQ'') && isequal(double(T.VQ),VQcached) && ...'};

    for k=1:size(replacements,1)
        j=find_unique_contains(G,replacements{k,1});
        G=replace_line(G,j,strrep(G{j},replacements{k,1},replacements{k,2}));
    end

    % Add explicit theorem comment to source.
    pReason5=find_unique_contains(G,'reason=5;');
    theoremComment={ ...
        '            % AF7R58ZD monotone theorem: sweep radii are nondecreasing in'; ...
        '            % payload/UAV speed upper bounds. A cached larger bound therefore'; ...
        '            % remains conservative for a smaller current bound.'};
    G=insert_before(G,pReason5,theoremComment);

    lines=replace_range(lines,g0,g1,G);
    patched=strjoin(lines,newline);

    patchedLines=reshape(regexp(patched,'\n','split'),1,[]);
    patchedDefs=discover_function_headers(patchedLines);
    [PG,~,~,~]=method_block_by_name(patchedLines,patchedDefs, ...
        'can_reuse_reference_only_prepared_packet_cache_exact');
    gateText=strjoin(PG,newline);

    inv=struct();
    inv.cached_bounds_defined= ...
        count(gateText,'VLcached=double(oldSC.payload_speed_UB_m_s);')==1 && ...
        count(gateText,'VQcached=double(oldSC.uav_speed_UB_m_s);')==1;
    inv.payload_dominance= ...
        count(gateText,'VLcached>=VL')==1 && ...
        ~contains(gateText,'isequal(double(oldSC.payload_speed_UB_m_s),VL)');
    inv.uav_dominance= ...
        count(gateText,'VQcached>=VQ')==1 && ...
        ~contains(gateText,'isequal(double(oldSC.uav_speed_UB_m_s),VQ)');
    inv.cached_sweep_validation= ...
        contains(gateText,'pay=VLcached*dt/2;') && ...
        contains(gateText,'uav=VQcached*dt/2;') && ...
        contains(gateText,'cab=max(VLcached,VQcached)*dt/2;') && ...
        contains(gateText,'S.sensor_one_period_m),VQcached*dt');
    inv.cached_sensor_validation= ...
        contains(gateText,'horizonMargin=Rs-(rho+VQcached*H);') && ...
        contains(gateText,'dDetLB=Rs-VQcached*dt;') && ...
        contains(gateText,'if VQcached>0') && ...
        contains(gateText,'collisionLead=safeClear/VQcached;') && ...
        contains(gateText,'T.VL),VLcached') && ...
        contains(gateText,'T.VQ),VQcached');
    inv.exact_live_dependencies_preserved= ...
        contains(gateText,'isequal(stored,live)') && ...
        contains(gateText,'isequaln(K.static_obstacle_map,obj.static_obstacle_map)');
    inv.fail_closed_on_bound_increase= ...
        contains(gateText,'VLcached>=VL') && contains(gateText,'VQcached>=VQ');
    inv.all=all(structfun(@(x)logical(x),inv));
    OUT.static_invariants=inv;

    say(f,'');
    say(f,'[C. MONOTONE CACHE THEOREM PREFLIGHT]');
    fn=fieldnames(inv);
    for k=1:numel(fn)
        say(f,' %-46s = %d',fn{k},logical(inv.(fn{k})));
    end
    assert(inv.all,'[AF7R58ZD] monotone cache invariant failed.');

    % Protected functions must remain text-identical.
    after=struct();
    protectedEqual=true;
    say(f,'');
    say(f,'[D. PROTECTED FUNCTION HASHES -- PATCHED]');
    for k=1:numel(protected)
        [b,j0,j1,~]=method_block_by_name(patchedLines,patchedDefs,protected{k});
        key=matlab.lang.makeValidName(protected{k});
        after.(key)=text_hash(strjoin(b,newline));
        same=strcmp(before.(key),after.(key));
        protectedEqual=protectedEqual && same;
        say(f,' %-48s unchanged=%d | L%d:L%d | %s', ...
            protected{k},same,j0,j1,after.(key));
    end
    assert(protectedEqual,'[AF7R58ZD] protected active reuse/builder changed.');
    OUT.protected_unchanged=protectedEqual;

    % Full patched class parse before production write.
    preDir=fullfile(outdir,'AF7R58ZD_PREFLIGHT');
    [ok,msg]=mkdir(preDir); assert(ok,'%s',msg);
    parseName='AF7R58ZD_PARSE_REPLANNING_SYSTEM';
    parseText=strrep(patched,'classdef REPLANNING_SYSTEM < handle', ...
        ['classdef ' parseName ' < handle']);
    write_utf8(fullfile(preDir,[parseName '.m']),parseText);
    addpath(preDir,'-begin');
    cleanupPath=onCleanup(@()safe_rmpath_if_present(preDir)); %#ok<NASGU>
    eval(['clear ' parseName]);
    mc=meta.class.fromName(parseName);
    assert(~isempty(mc),'[AF7R58ZD] patched full class parse failed.');
    eval(['clear ' parseName]);

    say(f,'');
    say(f,'[E. PREFLIGHT BEFORE WRITE]');
    say(f,' monotone cache theorem                 = 1');
    say(f,' exact live dependency invalidation     = 1');
    say(f,' exact obstacle source invalidation     = 1');
    say(f,' protected production builders          = 1');
    say(f,' full patched class parse               = 1');
    say(f,' production writes                      = 0');

    [ok,msg]=mkdir(backupDir); assert(ok,'%s',msg);
    backupReady=true;
    OUT.backup_directory=backupDir;
    copyfile(pR,fullfile(backupDir,'REPLANNING_SYSTEM.m'));

    writesStarted=true;
    write_text_eol(pR,patched,eolStyle);
    rehash;

    h1=file_hash(pR);
    OUT.source_hash_before=h0;
    OUT.source_hash_after=h1;
    OUT.cache_gate_hash_after=text_hash(strjoin(PG,newline));
    OUT.production_modified=true;
    OUT.status='MONOTONE_CACHE_DOMINANCE_INSTALL_COMPLETE__CLEAN_RERUN_REQUIRED';

    say(f,'');
    say(f,'[F. INSTALL RESULT]');
    say(f,' production modified                    = 1');
    say(f,' rollback performed                     = 0');
    say(f,' exact equality -> conservative dominance= 1');
    say(f,' fail closed when current bound increases= 1');
    say(f,' trajectory geometry changed            = 0');
    say(f,' active runtime reuse gate changed      = 0');
    say(f,' builders changed                       = 0');
    say(f,' new REPLANNING_SYSTEM SHA              = %s',h1);

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

save(fullfile(outdir,'AF7R58ZD_OUT.mat'),'OUT','-v7');
assignin('base','AF7R58ZD_OUT',OUT);

say(f,'');
say(f,'[RETURN THESE TWO FILES]');
say(f,' RETURN: %s',report);
say(f,' RETURN: %s',fullfile(outdir,'AF7R58ZD_OUT.mat'));
say(f,' status = %s',OUT.status);
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


function [b,i0,i1,hdr]=method_block_by_name(lines,defs,name)
hits=[];
for k=1:numel(defs)
    if header_has_exact_method_name(defs(k).header,name)
        hits(end+1)=k; %#ok<AGROW>
    end
end
assert(numel(hits)==1,'[AF7R58ZD] method "%s" count=%d.',name,numel(hits));
d=defs(hits(1));
i0=d.line; hdr=d.header; i1=numel(lines);
for k=i0+1:numel(lines)
    if startsWith(strtrim(lines{k}),'function ') && ...
            leading_ws_count(lines{k})==d.indent
        i1=k-1; break;
    end
end
b=reshape(lines(i0:i1),1,[]);
end


function tf=header_has_exact_method_name(header,name)
tf=false;
loc=strfind(header,name);
for ii=1:numel(loc)
    p=loc(ii); q=p+numel(name);
    if p>1 && is_ident_char(header(p-1)), continue; end
    if q<=numel(header) && is_ident_char(header(q)), continue; end
    while q<=numel(header) && isspace(header(q)), q=q+1; end
    if q<=numel(header) && header(q)=='(', tf=true; return; end
end
end


function [hdr,kend]=collect_function_header(lines,k0)
parts={strtrim(lines{k0})}; kend=k0;
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


function tf=is_ident_char(c)
tf=isstrprop(c,'alphanum') || c=='_';
end


function j=find_unique_contains(B,token)
v=find(cellfun(@(x)contains(x,token),B));
assert(numel(v)==1,'[AF7R58ZD] token "%s" count=%d.',token,numel(v));
j=v(1);
end


function j=find_unique_regex(B,pat)
v=find(cellfun(@(x)~isempty(regexp(x,pat,'once')),B));
assert(numel(v)==1,'[AF7R58ZD] regex "%s" count=%d.',pat,numel(v));
j=v(1);
end


function c=insert_before(c,j,newLines)
c=reshape(c,1,[]); newLines=reshape(newLines,1,[]);
c=[c(1:j-1),newLines,c(j:end)];
end


function c=insert_after(c,j,newLines)
c=reshape(c,1,[]); newLines=reshape(newLines,1,[]);
c=[c(1:j),newLines,c(j+1:end)];
end


function c=replace_line(c,j,newLine)
c=reshape(c,1,[]);
c{j}=char(newLine);
end


function c=replace_range(c,i0,i1,newLines)
c=reshape(c,1,[]); newLines=reshape(newLines,1,[]);
c=[c(1:i0-1),newLines,c(i1+1:end)];
end


function [s,eol]=read_text_norm(p)
fid=fopen(p,'rb'); assert(fid>=0);
cl=onCleanup(@()fclose(fid)); %#ok<NASGU>
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
    case 'CRLF', s=strrep(s,sprintf('\n'),sprintf('\r\n'));
    case 'CR',   s=strrep(s,sprintf('\n'),sprintf('\r'));
end
write_utf8(p,s);
end


function write_utf8(p,s)
fid=fopen(p,'wb'); assert(fid>=0);
cl=onCleanup(@()fclose(fid)); %#ok<NASGU>
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
cl=onCleanup(@()fclose(fid)); %#ok<NASGU>
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
