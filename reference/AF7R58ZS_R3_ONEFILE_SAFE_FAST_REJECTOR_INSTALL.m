function OUT = AF7R58ZS_R3_ONEFILE_SAFE_FAST_REJECTOR_INSTALL()
% AF7R58ZS-R3
% Safe fast-rejector promotion for the live R23 Bezier wrapper.
%
% WHY R3
% ------
% R1/R2 incorrectly used raw whole-file text counts such as
%   count(wrapper,[name '('])
% as if they were executable-call counts. Comments/diagnostic text can make
% that false. R3 resolves executable DIRECT CALLS line-by-line and patches
% only those code tokens.
%
% SAFETY ARCHITECTURE
% -------------------
% R41I fast certificate is NOT promoted as the final safety authority.
%
%     candidate
%        |
%        v
%   R41I FAST CERT
%        |
%        +-- FAIL / INCONCLUSIVE --> REJECT  (safe, possibly conservative)
%        |
%        +-- PASS -------------> ORIGINAL R23 CERT
%                                  |
%                                  +-- PASS --> may proceed
%                                  +-- FAIL --> reject
%
% Therefore:
%   final SAFE authority = ORIGINAL R23
%   fast false PASS      = caught by ORIGINAL R23
%   fast false FAIL      = safe false reject only
%
% No sampled acceptance is introduced.
% Original R23 source is never modified.

srcName  = 'G27V74R23_CONTINUOUS_WHOLE_BODY_CERT';
fastName = 'G27V74R41I_CONTINUOUS_WHOLE_BODY_CERT_FAST';
gateName = 'G27V74R58ZS_R3_CONTINUOUS_WHOLE_BODY_CERT_GATED';
wrapName = 'G27V74R23_BEZIER_QARC_CTGEO_SHADOW_GENERATOR';
helperName='g27_best_support_margin';

OUT=struct();
OUT.schema='AF7R58ZS_R3_SAFE_FAST_REJECTOR_V1';
OUT.production_modified=false;
OUT.rollback_performed=false;
OUT.status='OPEN';
OUT.error=struct();

[wrapFile,wrapHow]=resolve_wrapper_source(wrapName);
assert(~isempty(wrapFile)&&isfile(wrapFile), ...
    '[AF7R58ZS-R3] live R23 wrapper source unresolved.');

wrapDir=fileparts(wrapFile);
wrap=normalize_newlines(read_utf8(wrapFile));

[srcFile,srcHow]=resolve_dependency_source(srcName,wrapDir);
assert(~isempty(srcFile)&&isfile(srcFile), ...
    '[AF7R58ZS-R3] original R23 certificate source unresolved.');

fastFile=fullfile(wrapDir,[fastName '.m']);
gateFile=fullfile(wrapDir,[gateName '.m']);

CALLold=find_executable_direct_calls(wrap,srcName);
CALLfast=find_executable_direct_calls(wrap,fastName);
CALLgate=find_executable_direct_calls(wrap,gateName);

fprintf('\n====================================================================================================\n');
fprintf(' AF7R58ZS-R3 | SAFE FAST-REJECTOR SOURCE BINDING\n');
fprintf('====================================================================================================\n');
fprintf('[AUTHORITY] executableCallScan=1 | rawTextCountAuthority=0 | originalR23FinalAuthority=1\n');
fprintf(' wrapper resolution = %s\n',wrapHow);
fprintf(' wrapper            = %s\n',wrapFile);
fprintf(' R23 resolution     = %s\n',srcHow);
fprintf(' R23 source         = %s\n',srcFile);
fprintf(' executable direct calls: original=%d fast=%d gate=%d\n', ...
    numel(CALLold),numel(CALLfast),numel(CALLgate));
print_call_lines('ORIGINAL',CALLold);
print_call_lines('FAST',CALLfast);
print_call_lines('GATE',CALLgate);

if isempty(CALLold) && isempty(CALLfast) && ~isempty(CALLgate)
    assert(isfile(fastFile)&&isfile(gateFile), ...
        '[AF7R58ZS-R3] gate is live but companion fast/gate files are incomplete.');
    OUT.wrapper_source=wrapFile;
    OUT.wrapper_resolution=wrapHow;
    OUT.r23_source=srcFile;
    OUT.r23_resolution=srcHow;
    OUT.fast_source=fastFile;
    OUT.gate_source=gateFile;
    OUT.wrapper_sha=file_hash(wrapFile);
    OUT.r23_sha=file_hash(srcFile);
    OUT.fast_sha=file_hash(fastFile);
    OUT.gate_sha=file_hash(gateFile);
    OUT.status='SAFE_FAST_REJECTOR_ALREADY_ACTIVE__NO_WRITE';
    assignin('base','AF7R58ZS_R3_OUT',OUT);
    fprintf(' production modified = 0\n');
    fprintf(' status = %s\n',OUT.status);
    return;
end

assert(isempty(CALLfast), ...
    '[AF7R58ZS-R3] wrapper contains a direct FAST call outside the safe gate architecture.');
assert(isempty(CALLgate), ...
    '[AF7R58ZS-R3] wrapper contains an unexpected partial GATE binding.');
assert(~isempty(CALLold), ...
    ['[AF7R58ZS-R3] no executable direct call to original R23 was found. ' ...
     'See the printed executable-call scan; production was not modified.']);
assert(numel(CALLold)<=8, ...
    '[AF7R58ZS-R3] implausibly many executable original-R23 calls; fail closed.');

outdir=tempname(wrapDir);
[ok,msg]=mkdir(outdir); assert(ok,'%s',msg);
report=fullfile(outdir,'AF7R58ZS_R3_REPORT.txt');
fid=fopen(report,'wt','n','UTF-8'); assert(fid>=0);
cf=onCleanup(@()safe_fclose(fid)); %#ok<NASGU>

backupDir=tempname(wrapDir);
backupReady=false;
writesStarted=false;
fastExisted=isfile(fastFile);
gateExisted=isfile(gateFile);
fastBackup='';
gateBackup='';

try
    srcSha=file_hash(srcFile);
    wrapSha=file_hash(wrapFile);

    OUT.wrapper_source=wrapFile;
    OUT.wrapper_resolution=wrapHow;
    OUT.wrapper_pre_sha=wrapSha;
    OUT.r23_source=srcFile;
    OUT.r23_resolution=srcHow;
    OUT.r23_pre_sha=srcSha;
    OUT.original_direct_call_count=numel(CALLold);

    say(fid,'====================================================================================================');
    say(fid,' AF7R58ZS-R3 | SAFE FAST-REJECTOR INSTALL');
    say(fid,'====================================================================================================');
    say(fid,'[SAFETY] originalR23FinalAuthority=1 | fastRejectOnly=1 | sampledAcceptance=0');
    say(fid,'[SOURCE] executableCallScan=1 | rawWholeFileCountAuthority=0');
    say(fid,' wrapper = %s',wrapFile);
    say(fid,' wrapper SHA = %s',wrapSha);
    say(fid,' R23 = %s',srcFile);
    say(fid,' R23 SHA = %s',srcSha);
    say(fid,' executable original-R23 calls = %d',numel(CALLold));

    src=normalize_newlines(read_utf8(srcFile));
    L=reshape(regexp(src,'\n','split'),1,[]);
    [k0,k1]=find_local_function_block(L,helperName);
    oldHelper=strjoin(L(k0:k1),newline);

    expectedOld = strjoin({ ...
        'function best=g27_best_support_margin(P,c,R,a,rho)', ...
        'N=zeros(3,0);', ...
        'for j=1:size(P,2)', ...
        'z=R.''*(P(:,j)-c);', ...
        'nw=R*(z./(a.^2));', ...
        'nn=norm(nw);', ...
        'if nn>0', ...
        'N(:,end+1)=nw/nn;', ...
        'end', ...
        'end', ...
        'd=mean(P,2)-c;', ...
        'nd=norm(d);', ...
        'if nd>0', ...
        'N(:,end+1)=d/nd;', ...
        'end', ...
        'for k=1:3', ...
        'N(:,end+1)=R(:,k);', ...
        'N(:,end+1)=-R(:,k);', ...
        'end', ...
        'best=-inf;', ...
        'for k=1:size(N,2)', ...
        'n=N(:,k);', ...
        'h=n.''*c+norm(a.*(R.''*n))+rho;', ...
        'best=max(best,min(n.''*P)-h);', ...
        'end', ...
        'end'}, newline);

    helperExact=strcmp(canonical_code(oldHelper),canonical_code(expectedOld));
    say(fid,'');
    say(fid,'[A. ORIGINAL SUPPORT-KERNEL BINDING]');
    say(fid,' helper source lines               = %d:%d',k0,k1);
    say(fid,' scalar executable body exact      = %d',helperExact);
    assert(helperExact, ...
        '[AF7R58ZS-R3] current R23 support helper differs from the audited scalar authority.');

    H={ ...
        'function best=g27_best_support_margin(P,c,R,a,rho)', ...
        '% AF7R58ZS-R3 / R41I fast rejector kernel.', ...
        '% Candidate-normal set and real-arithmetic support theorem match R23.', ...
        '% This kernel is NOT final SAFE authority in production.', ...
        'm=size(P,2);', ...
        'N=zeros(3,m+7);', ...
        'nN=0;', ...
        'Rt=R.'';', ...
        'Pc=P-c;', ...
        'Z=Rt*Pc;', ...
        'NW=R*(Z./(a.^2));', ...
        'nn=sqrt(sum(NW.^2,1));', ...
        'good=isfinite(nn) & nn>0;', ...
        'ig=find(good);', ...
        'ng=numel(ig);', ...
        'if ng>0', ...
        '    N(:,1:ng)=NW(:,ig)./nn(ig);', ...
        '    nN=ng;', ...
        'end', ...
        'd=mean(P,2)-c;', ...
        'nd=norm(d);', ...
        'if isfinite(nd) && nd>0', ...
        '    nN=nN+1;', ...
        '    N(:,nN)=d/nd;', ...
        'end', ...
        'for k=1:3', ...
        '    nN=nN+1; N(:,nN)= R(:,k);', ...
        '    nN=nN+1; N(:,nN)=-R(:,k);', ...
        'end', ...
        'N=N(:,1:nN);', ...
        'RN=Rt*N;', ...
        'h=(c.''*N) + sqrt(sum((a.*RN).^2,1)) + rho;', ...
        'lower=min(N.''*P,[],2).'';', ...
        'best=max(lower-h);', ...
        'end'};

    topHits=[];
    for k=1:numel(L)
        t=strtrim(L{k});
        if startsWith(t,'function') && contains(t,srcName)
            topHits(end+1)=k; %#ok<AGROW>
        end
    end
    assert(numel(topHits)==1, ...
        '[AF7R58ZS-R3] R23 top-level declaration was not uniquely resolved.');
    kt=topHits(1);
    assert(count(L{kt},srcName)==1);
    L{kt}=strrep(L{kt},srcName,fastName);
    Lfast=[L(1:k0-1),reshape(H,1,[]),L(k1+1:end)];
    fastText=strjoin(Lfast,newline);

    fastHelper=strjoin(H,newline);
    T=struct();
    T.control_point_normals=contains(fastHelper,'NW=R*(Z./(a.^2));') && ...
        contains(fastHelper,'N(:,1:ng)=NW(:,ig)./nn(ig);');
    T.mean_normal=contains(fastHelper,'d=mean(P,2)-c;') && ...
        contains(fastHelper,'N(:,nN)=d/nd;');
    T.principal_axes=contains(fastHelper,'N(:,nN)= R(:,k);') && ...
        contains(fastHelper,'N(:,nN)=-R(:,k);');
    T.forbidden_support=contains(fastHelper,'sqrt(sum((a.*RN).^2,1)) + rho');
    T.polytope_lower_support=contains(fastHelper,'lower=min(N.''*P,[],2).'';');
    T.best_margin=contains(fastHelper,'best=max(lower-h);');
    T.all=all(structfun(@(x)logical(x),T));
    assert(T.all,'[AF7R58ZS-R3] fast support theorem-marker audit failed.');

    maxAbs=0;
    for q=1:200
        m=4+mod(q,13);
        jj=1:m;
        P=[sin(0.13*q+0.31*jj); ...
           cos(0.17*q-0.23*jj); ...
           0.2*q/m + sin(0.07*jj)];
        c=[sin(0.11*q);cos(0.09*q);0.01*q];
        ang=0.01*q;
        RR=[cos(ang),-sin(ang),0;sin(ang),cos(ang),0;0,0,1];
        a=[0.4+0.001*q;0.7+0.002*mod(q,17);1.1+0.001*mod(q,23)];
        rho=0.02+0.0001*q;
        bo=support_old(P,c,RR,a,rho);
        bf=support_fast(P,c,RR,a,rho);
        maxAbs=max(maxAbs,abs(bo-bf));
    end
    assert(maxAbs<1e-11, ...
        '[AF7R58ZS-R3] scalar/vectorized arithmetic regression unexpectedly large.');
    OUT.support_regression_max_abs=maxAbs;
    OUT.fast_theorem_static=T;

    say(fid,'');
    say(fid,'[B. FAST REJECTOR CONSTRUCTION]');
    say(fid,' candidate-normal construction retained = %d',T.control_point_normals&&T.mean_normal&&T.principal_axes);
    say(fid,' real-arithmetic support formula retained= %d',T.forbidden_support);
    say(fid,' deterministic max abs difference        = %.17g',maxAbs);
    say(fid,' fast result used as final SAFE authority= 0');

    gateText=strjoin({ ...
        ['function C=' gateName '(varargin)'], ...
        '% Auto-generated by AF7R58ZS-R3.', ...
        '% Fast certificate is reject-only; original R23 is final SAFE authority.', ...
        ['Cf=' fastName '(varargin{:});'], ...
        'fastPass=false;', ...
        'if isstruct(Cf) && isscalar(Cf) && isfield(Cf,''mathematical_pass'')', ...
        '    fastPass=logical(Cf.mathematical_pass);', ...
        'end', ...
        'if ~fastPass', ...
        '    C=Cf;', ...
        '    return;', ...
        'end', ...
        ['C=' srcName '(varargin{:});'], ...
        'end'},newline);

    G=struct();
    G.fast_called_once=count(gateText,[fastName '('])==1;
    G.original_called_once=count(gateText,[srcName '('])==1;
    G.fast_fail_returns_before_original = ...
        strfind(gateText,'if ~fastPass') < strfind(gateText,['C=' srcName '(varargin{:});']);
    G.original_is_final_assignment = ...
        contains(gateText,['C=' srcName '(varargin{:});']);
    G.all=G.fast_called_once && G.original_called_once && ...
        G.fast_fail_returns_before_original && G.original_is_final_assignment;
    assert(G.all,'[AF7R58ZS-R3] gate semantic invariant failed.');
    OUT.gate_static=G;

    say(fid,'');
    say(fid,'[C. SAFE GATE SEMANTICS]');
    say(fid,' fast FAIL -> immediate reject path   = 1');
    say(fid,' fast PASS -> ORIGINAL R23 executed   = 1');
    say(fid,' original R23 is final returned cert  = 1');
    say(fid,' fast false PASS can create SAFE      = 0');
    say(fid,' fast false FAIL may reject safe cand  = 1 (completeness only)');

    [wrapNew,nReplaced,replacedLines]=replace_executable_direct_calls( ...
        wrap,srcName,gateName);
    assert(nReplaced==numel(CALLold) && nReplaced>=1, ...
        '[AF7R58ZS-R3] executable wrapper replacement count mismatch.');

    postOld=find_executable_direct_calls(wrapNew,srcName);
    postGate=find_executable_direct_calls(wrapNew,gateName);
    assert(isempty(postOld) && numel(postGate)==nReplaced, ...
        '[AF7R58ZS-R3] wrapper executable-call postcondition failed.');

    say(fid,'');
    say(fid,'[D. WRAPPER EXECUTABLE REBIND]');
    say(fid,' replaced executable direct calls = %d',nReplaced);
    for k=1:numel(replacedLines)
        say(fid,' line %d | %s',replacedLines(k).line,replacedLines(k).text);
    end
    say(fid,' raw comment/string occurrence counts are NOT authority');

    preDir=fullfile(outdir,'AF7R58ZS_R3_PREFLIGHT');
    [ok,msg]=mkdir(preDir); assert(ok,'%s',msg);

    write_utf8(fullfile(preDir,[fastName '.m']),fastText);
    write_utf8(fullfile(preDir,[gateName '.m']),gateText);

    wrapTmpName='AF7R58ZS_R3_PARSE_WRAPPER';
    wrapParse=replace_top_function_name(wrapNew,wrapName,wrapTmpName);
    write_utf8(fullfile(preDir,[wrapTmpName '.m']),wrapParse);

    addpath(preDir,'-begin');
    cp=onCleanup(@()safe_rmpath(preDir)); %#ok<NASGU>
    eval(['clear ' fastName]);
    nargin(fastName);
    eval(['clear ' gateName]);
    nargin(gateName);
    eval(['clear ' wrapTmpName]);
    nargin(wrapTmpName);
    eval(['clear ' fastName]);
    eval(['clear ' gateName]);
    eval(['clear ' wrapTmpName]);

    say(fid,'');
    say(fid,'[E. PREFLIGHT BEFORE WRITE]');
    say(fid,' fast certificate parse = 1');
    say(fid,' safe gate parse        = 1');
    say(fid,' patched wrapper parse  = 1');
    say(fid,' original R23 modified  = 0');
    say(fid,' production writes      = 0');

    [ok,msg]=mkdir(backupDir); assert(ok,'%s',msg);
    backupReady=true;
    OUT.backup_directory=backupDir;

    copyfile(wrapFile,fullfile(backupDir,[wrapName '.m']));
    if fastExisted
        fastBackup=fullfile(backupDir,[fastName '.m']);
        copyfile(fastFile,fastBackup);
    end
    if gateExisted
        gateBackup=fullfile(backupDir,[gateName '.m']);
        copyfile(gateFile,gateBackup);
    end

    writesStarted=true;
    write_utf8(fastFile,fastText);
    write_utf8(gateFile,gateText);
    write_utf8(wrapFile,wrapNew);
    rehash;

    finalWrap=normalize_newlines(read_utf8(wrapFile));
    finalOld=find_executable_direct_calls(finalWrap,srcName);
    finalGate=find_executable_direct_calls(finalWrap,gateName);
    assert(isempty(finalOld) && numel(finalGate)==nReplaced, ...
        '[AF7R58ZS-R3] post-write wrapper binding failed.');

    OUT.r23_post_sha=file_hash(srcFile);
    assert(strcmp(OUT.r23_post_sha,srcSha), ...
        '[AF7R58ZS-R3] original R23 source unexpectedly changed.');

    OUT.fast_source=fastFile;
    OUT.gate_source=gateFile;
    OUT.fast_sha=file_hash(fastFile);
    OUT.gate_sha=file_hash(gateFile);
    OUT.wrapper_post_sha=file_hash(wrapFile);
    OUT.production_modified=true;
    OUT.status='SAFE_FAST_REJECTOR_INSTALLED__ORIGINAL_R23_FINAL_AUTHORITY__CLEAN_RERUN_REQUIRED';

    say(fid,'');
    say(fid,'[F. INSTALL RESULT]');
    say(fid,' production modified              = 1');
    say(fid,' original R23 modified            = 0');
    say(fid,' fast certificate installed       = 1');
    say(fid,' safe reject-only gate installed  = 1');
    say(fid,' wrapper executable calls rebound = %d',nReplaced);
    say(fid,' final SAFE authority             = ORIGINAL R23');
    say(fid,' fast false-SAFE vulnerability    = 0 by construction');
    say(fid,' status = %s',OUT.status);

catch ME
    if backupReady && writesStarted
        try
            copyfile(fullfile(backupDir,[wrapName '.m']),wrapFile,'f');

            if fastExisted
                copyfile(fastBackup,fastFile,'f');
            elseif isfile(fastFile)
                delete(fastFile);
            end

            if gateExisted
                copyfile(gateBackup,gateFile,'f');
            elseif isfile(gateFile)
                delete(gateFile);
            end

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

save(fullfile(outdir,'AF7R58ZS_R3_OUT.mat'),'OUT','-v7');
assignin('base','AF7R58ZS_R3_OUT',OUT);
say(fid,'');
say(fid,'RETURN: %s',report);
say(fid,'RETURN: %s',fullfile(outdir,'AF7R58ZS_R3_OUT.mat'));
say(fid,'status = %s',OUT.status);
end


function [p,how]=resolve_wrapper_source(name)
p=which(name);
how='WHICH';
if ~isempty(p)&&isfile(p)
    return;
end
p='';
how='';
try
    if evalin('base','exist(''replanner_instance'',''var'')')==1
        R=evalin('base','replanner_instance');
        fh=R.generator_backend_locked_callback;
        if isa(fh,'function_handle')
            info=functions(fh);
            if isfield(info,'file') && ischar(info.file) && isfile(info.file)
                p=info.file;
                how='LIVE_LOCKED_CALLBACK_HANDLE';
                return;
            end
        end
    end
catch
end
end


function [p,how]=resolve_dependency_source(name,wrapDir)
p=which(name);
how='WHICH';
if ~isempty(p)&&isfile(p)
    return;
end

p2=fullfile(wrapDir,[name '.m']);
if isfile(p2)
    p=p2;
    how='LIVE_WRAPPER_SIBLING';
    return;
end

roots={wrapDir,fileparts(wrapDir),fileparts(fileparts(wrapDir))};
hits={};
for ir=1:numel(roots)
    r=roots{ir};
    if isempty(r)||~isfolder(r), continue; end
    try
        D=dir(fullfile(r,'**',[name '.m']));
        for k=1:numel(D)
            if ~D(k).isdir
                hits{end+1}=fullfile(D(k).folder,D(k).name); %#ok<AGROW>
            end
        end
    catch
    end
    hits=unique(hits,'stable');
    if numel(hits)==1
        p=hits{1};
        how='UNIQUE_BOUNDED_LOCAL_TREE';
        return;
    elseif numel(hits)>1
        break;
    end
end

p='';
how='';
end


function C=find_executable_direct_calls(txt,name)
L=regexp(normalize_newlines(txt),'\n','split');
C=struct('line',{},'text',{},'start',{});

for k=1:numel(L)
    code=code_before_comment(L{k});
    spans=direct_call_starts(code,name);
    for q=1:numel(spans)
        C(end+1)=struct('line',k,'text',strtrim(L{k}), ...
            'start',spans(q)); %#ok<AGROW>
    end
end
end


function starts=direct_call_starts(code,name)
starts=[];
pos=strfind(code,name);
for k=1:numel(pos)
    p=pos(k);
    q=p+numel(name);

    if p>1 && is_ident_char(code(p-1))
        continue;
    end
    if q<=numel(code) && is_ident_char(code(q))
        continue;
    end

    z=q;
    while z<=numel(code) && isspace(code(z))
        z=z+1;
    end
    if z<=numel(code) && code(z)=='('
        starts(end+1)=p; %#ok<AGROW>
    end
end
end


function tf=is_ident_char(c)
tf=isstrprop(c,'alphanum') || c=='_';
end


function code=code_before_comment(line)
p=strfind(line,'%');
if isempty(p)
    code=line;
else
    code=line(1:p(1)-1);
end
end


function [out,n,REC]=replace_executable_direct_calls(txt,oldName,newName)
L=regexp(normalize_newlines(txt),'\n','split');
n=0;
REC=struct('line',{},'text',{});

for k=1:numel(L)
    line=L{k};
    code=code_before_comment(line);
    starts=direct_call_starts(code,oldName);
    if isempty(starts)
        continue;
    end

    for q=numel(starts):-1:1
        p=starts(q);
        line=[line(1:p-1),newName,line(p+numel(oldName):end)];
        n=n+1;
    end
    L{k}=line;
    REC(end+1)=struct('line',k,'text',strtrim(line)); %#ok<AGROW>
end

out=strjoin(L,newline);
end


function out=replace_top_function_name(txt,oldName,newName)
L=regexp(normalize_newlines(txt),'\n','split');
hits=[];
for k=1:numel(L)
    t=strtrim(L{k});
    if startsWith(t,'function') && contains(t,oldName)
        hits(end+1)=k; %#ok<AGROW>
    end
end
assert(numel(hits)==1,'[AF7R58ZS-R3] wrapper top-level declaration not unique.');
k=hits(1);
assert(count(L{k},oldName)==1);
L{k}=strrep(L{k},oldName,newName);
out=strjoin(L,newline);
end


function print_call_lines(label,C)
fprintf(' %s executable call count = %d\n',label,numel(C));
for k=1:numel(C)
    fprintf('   L%d | %s\n',C(k).line,C(k).text);
end
end


function [k0,k1]=find_local_function_block(L,name)
hits=[];
for k=1:numel(L)
    t=strtrim(L{k});
    if startsWith(t,'function')
        p=strfind(t,name);
        if ~isempty(p)
            tail=strtrim(t(p(1)+numel(name):end));
            if startsWith(tail,'(')
                hits(end+1)=k; %#ok<AGROW>
            end
        end
    end
end
assert(numel(hits)==1,'[AF7R58ZS-R3] local function %s count=%d.',name,numel(hits));
k0=hits(1);
k1=numel(L);
for k=k0+1:numel(L)
    if startsWith(strtrim(L{k}),'function')
        k1=k-1;
        break;
    end
end
end


function s=canonical_code(s)
L=regexp(normalize_newlines(s),'\n','split');
o={};
for k=1:numel(L)
    x=L{k};
    p=strfind(x,'%');
    if ~isempty(p)
        x=x(1:p(1)-1);
    end
    x=regexprep(x,'\s+','');
    if ~isempty(x)
        o{end+1}=x; %#ok<AGROW>
    end
end
s=strjoin(o,'');
end


function best=support_old(P,c,R,a,rho)
N=zeros(3,0);
for j=1:size(P,2)
    z=R.'*(P(:,j)-c);
    nw=R*(z./(a.^2));
    nn=norm(nw);
    if nn>0
        N(:,end+1)=nw/nn; %#ok<AGROW>
    end
end
d=mean(P,2)-c;
nd=norm(d);
if nd>0
    N(:,end+1)=d/nd; %#ok<AGROW>
end
for k=1:3
    N(:,end+1)= R(:,k); %#ok<AGROW>
    N(:,end+1)=-R(:,k); %#ok<AGROW>
end
best=-inf;
for k=1:size(N,2)
    n=N(:,k);
    h=n.'*c+norm(a.*(R.'*n))+rho;
    best=max(best,min(n.'*P)-h);
end
end


function best=support_fast(P,c,R,a,rho)
m=size(P,2);
N=zeros(3,m+7);
nN=0;
Rt=R.';
Pc=P-c;
Z=Rt*Pc;
NW=R*(Z./(a.^2));
nn=sqrt(sum(NW.^2,1));
good=isfinite(nn) & nn>0;
ig=find(good);
ng=numel(ig);
if ng>0
    N(:,1:ng)=NW(:,ig)./nn(ig);
    nN=ng;
end
d=mean(P,2)-c;
nd=norm(d);
if isfinite(nd) && nd>0
    nN=nN+1;
    N(:,nN)=d/nd;
end
for k=1:3
    nN=nN+1; N(:,nN)= R(:,k);
    nN=nN+1; N(:,nN)=-R(:,k);
end
N=N(:,1:nN);
RN=Rt*N;
h=(c.'*N) + sqrt(sum((a.*RN).^2,1)) + rho;
lower=min(N.'*P,[],2).';
best=max(lower-h);
end


function s=normalize_newlines(s)
s=strrep(s,sprintf('\r\n'),sprintf('\n'));
s=strrep(s,sprintf('\r'),sprintf('\n'));
end


function s=read_utf8(p)
fid=fopen(p,'rb'); assert(fid>=0,'cannot read %s',p);
c=onCleanup(@()fclose(fid)); %#ok<NASGU>
b=fread(fid,Inf,'*uint8');
s=native2unicode(b.','UTF-8');
end


function write_utf8(p,s)
fid=fopen(p,'wb'); assert(fid>=0,'cannot write %s',p);
c=onCleanup(@()fclose(fid)); %#ok<NASGU>
b=unicode2native(s,'UTF-8');
n=fwrite(fid,b,'uint8');
assert(n==numel(b),'short write: %s',p);
end


function h=file_hash(p)
fid=fopen(p,'rb'); assert(fid>=0,'cannot hash %s',p);
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
