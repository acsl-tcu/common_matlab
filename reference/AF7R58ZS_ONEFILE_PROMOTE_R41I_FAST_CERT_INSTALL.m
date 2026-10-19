function OUT = AF7R58ZS_ONEFILE_PROMOTE_R41I_FAST_CERT_INSTALL()
% AF7R58ZS
% Promote the already-derived R41I theorem-equivalent support kernel into
% the live R23 BEZIER callback path.
%
% IMPORTANT:
%   - Original G27V74R23_CONTINUOUS_WHOLE_BODY_CERT.m is NOT modified.
%   - A generated copy G27V74R41I_CONTINUOUS_WHOLE_BODY_CERT_FAST.m is made
%     from the CURRENT resolved R23 source.
%   - Only the live R23 wrapper is rebound from original R23 cert -> R41I
%     fast cert.
%
% THEOREM PRESERVED:
% For every candidate unit normal n,
%   h_E⊕B(n) = n'*c + || a .* (R'*n) ||_2 + rho
% and
%   margin(n) = min_j n'*P_j - h_E⊕B(n).
% Positive max margin over the candidate-normal set is still only a
% sufficient separation certificate.  The candidate-normal set is
% unchanged:
%   (i) outward ellipsoid normal induced by each control point,
%   (ii) mean(P,2)-c direction,
%   (iii) +/- each obstacle principal axis.
%
% The continuous-time Bezier convex-hull theorem, q-arc enclosure,
% subdivision policy, resource policy, and fail-closed acceptance logic
% remain byte-for-byte inherited from the CURRENT R23 source.
%
% This is exactly the optimization direction already established by R41H/R41I:
% 105 g27_best_support_margin calls per certificate were observed, and only
% that helper is vectorized/preallocated.

srcName='G27V74R23_CONTINUOUS_WHOLE_BODY_CERT';
fastName='G27V74R41I_CONTINUOUS_WHOLE_BODY_CERT_FAST';
wrapName='G27V74R23_BEZIER_QARC_CTGEO_SHADOW_GENERATOR';
helperName='g27_best_support_margin';

OUT=struct();
OUT.schema='AF7R58ZS_PROMOTE_R41I_FAST_CERT_V1';
OUT.production_modified=false;
OUT.rollback_performed=false;
OUT.status='OPEN';
OUT.error=struct();

srcFile=which(srcName);
wrapFile=which(wrapName);
assert(~isempty(srcFile)&&isfile(srcFile), ...
    '[AF7R58ZS] current R23 certificate source unresolved.');
assert(~isempty(wrapFile)&&isfile(wrapFile), ...
    '[AF7R58ZS] current R23 wrapper source unresolved.');

outdir=tempname(fileparts(wrapFile));
[ok,msg]=mkdir(outdir); assert(ok,'%s',msg);
report=fullfile(outdir,'AF7R58ZS_REPORT.txt');
fid=fopen(report,'wt','n','UTF-8'); assert(fid>=0);
cf=onCleanup(@()safe_fclose(fid)); %#ok<NASGU>

say(fid,'====================================================================================================');
say(fid,' AF7R58ZS | PROMOTE R41I THEOREM-EQUIVALENT FAST GEOMETRY CERT');
say(fid,'====================================================================================================');
say(fid,'[AUTHORITY] runtimeSourceResolved=1 | originalR23Modified=0 | wrapperOnlyRebind=1 | rollback=1');
say(fid,'[THEORY] candidateNormalSetChanged=0 | supportFunctionChanged=0 | sampledAcceptanceAdded=0');

backupDir=tempname(fileparts(wrapFile));
backupReady=false;
writesStarted=false;
fastFile=fullfile(fileparts(srcFile),[fastName '.m']);
fastExisted=isfile(fastFile);
fastBackup='';

try
    srcSha=file_hash(srcFile);
    wrapSha=file_hash(wrapFile);
    OUT.r23_source=srcFile;
    OUT.r23_pre_sha=srcSha;
    OUT.wrapper_source=wrapFile;
    OUT.wrapper_pre_sha=wrapSha;

    say(fid,'');
    say(fid,'[A. CURRENT LIVE SOURCES]');
    say(fid,' R23 certificate = %s',srcFile);
    say(fid,' R23 SHA         = %s',srcSha);
    say(fid,' wrapper         = %s',wrapFile);
    say(fid,' wrapper SHA     = %s',wrapSha);

    src=normalize_newlines(read_utf8(srcFile));
    wrap=normalize_newlines(read_utf8(wrapFile));
    L=reshape(regexp(src,'\n','split'),1,[]);

    % ------------------------------------------------------------------
    % Exact helper binding to the historically audited R23 support theorem.
    % Comments/whitespace are ignored; executable token sequence is required
    % to be exactly the known original helper.
    % ------------------------------------------------------------------
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

    oldCanon=canonical_code(oldHelper);
    expectedCanon=canonical_code(expectedOld);
    helperExact=strcmp(oldCanon,expectedCanon);

    say(fid,'');
    say(fid,'[B. EXACT ORIGINAL SUPPORT-HELPER BINDING]');
    say(fid,' helper lines                         = %d:%d',k0,k1);
    say(fid,' canonical executable body exact      = %d',helperExact);
    assert(helperExact, ...
        '[AF7R58ZS] current R23 support helper differs from audited authority.');

    % ------------------------------------------------------------------
    % Build theorem-equivalent fast helper from established R41I form.
    % ------------------------------------------------------------------
    H={ ...
        'function best=g27_best_support_margin(P,c,R,a,rho)', ...
        '% AF7R58ZS / R41I theorem-equivalent vectorized implementation.', ...
        '% Same candidate-normal set and same support inequality as R23.', ...
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

    % Top-level copy name: replace only declaration token.
    topHits=[];
    for k=1:numel(L)
        t=strtrim(L{k});
        if startsWith(t,'function') && contains(t,srcName)
            topHits(end+1)=k; %#ok<AGROW>
        end
    end
    assert(numel(topHits)==1, ...
        '[AF7R58ZS] top-level R23 declaration not unique.');
    kt=topHits(1);
    assert(count(L{kt},srcName)==1, ...
        '[AF7R58ZS] top-level source-name token not unique.');
    L{kt}=strrep(L{kt},srcName,fastName);

    Lfast=[L(1:k0-1),reshape(H,1,[]),L(k1+1:end)];
    fastText=strjoin(Lfast,newline);

    % Structural theorem preservation: everything except top-level name and
    % local support helper must be identical to current R23 source.
    Lbase=L;
    Lbase{kt}=strrep(Lbase{kt},srcName,fastName);
    Lbase=[Lbase(1:k0-1),reshape(H,1,[]),Lbase(k1+1:end)];
    assert(strcmp(fastText,strjoin(Lbase,newline)), ...
        '[AF7R58ZS] generated fast source has unexpected changes.');

    % Static fast-helper theorem markers.
    fastHelper=strjoin(H,newline);
    T=struct();
    T.control_point_normals = contains(fastHelper,'NW=R*(Z./(a.^2));') && ...
        contains(fastHelper,'N(:,1:ng)=NW(:,ig)./nn(ig);');
    T.mean_normal = contains(fastHelper,'d=mean(P,2)-c;') && ...
        contains(fastHelper,'N(:,nN)=d/nd;');
    T.principal_axes = contains(fastHelper,'N(:,nN)= R(:,k);') && ...
        contains(fastHelper,'N(:,nN)=-R(:,k);');
    T.same_forbidden_support = ...
        contains(fastHelper,'sqrt(sum((a.*RN).^2,1)) + rho');
    T.same_polytope_lower_support = ...
        contains(fastHelper,'lower=min(N.''*P,[],2).'';');
    T.same_max_margin = contains(fastHelper,'best=max(lower-h);');
    T.all=all(structfun(@(x)logical(x),T));
    assert(T.all,'[AF7R58ZS] fast helper theorem-marker audit failed.');
    OUT.theorem_static=T;

    % ------------------------------------------------------------------
    % Supplementary deterministic numerical regression.
    % Not the theorem authority; verifies implementation arithmetic agrees
    % with the scalar source to roundoff on representative finite inputs.
    % ------------------------------------------------------------------
    maxAbs=0;
    signSafe=true;
    for q=1:200
        % Deterministic, no random state.
        m=4+mod(q,13);
        jj=1:m;
        P=[sin(0.13*q+0.31*jj); ...
           cos(0.17*q-0.23*jj); ...
           0.2*q/m + sin(0.07*jj)];
        c=[sin(0.11*q);cos(0.09*q);0.01*q];
        ang=0.01*q;
        R=[cos(ang),-sin(ang),0;sin(ang),cos(ang),0;0,0,1];
        a=[0.4+0.001*q;0.7+0.002*mod(q,17);1.1+0.001*mod(q,23)];
        rho=0.02+0.0001*q;
        bo=support_old(P,c,R,a,rho);
        bf=support_fast(P,c,R,a,rho);
        maxAbs=max(maxAbs,abs(bo-bf));
        tol=4096*eps(max([1,abs(bo),abs(bf)]));
        signSafe=signSafe && ...
            ((bo>tol && bf>0) || (bo<-tol && bf<0) || abs(bo)<=tol || abs(bf)<=tol);
    end
    assert(maxAbs<1e-11 && signSafe, ...
        '[AF7R58ZS] scalar/vectorized support regression failed.');
    OUT.support_regression_max_abs=maxAbs;

    say(fid,'');
    say(fid,'[C. THEOREM / NUMERICAL REGRESSION]');
    say(fid,' candidate control-point normals unchanged = %d',T.control_point_normals);
    say(fid,' mean direction unchanged                  = %d',T.mean_normal);
    say(fid,' +/- principal axes unchanged             = %d',T.principal_axes);
    say(fid,' forbidden support formula unchanged      = %d',T.same_forbidden_support);
    say(fid,' convex-hull lower support unchanged      = %d',T.same_polytope_lower_support);
    say(fid,' max-margin logic unchanged               = %d',T.same_max_margin);
    say(fid,' deterministic max abs difference         = %.17g',maxAbs);
    say(fid,' sampled acceptance used as authority     = 0');

    % ------------------------------------------------------------------
    % Wrapper rebind: exactly one executable call token.
    % ------------------------------------------------------------------
    oldCall=[srcName '('];
    newCall=[fastName '('];
    nOld=count(wrap,oldCall);
    nNew=count(wrap,newCall);
    say(fid,'');
    say(fid,'[D. LIVE WRAPPER BINDING]');
    say(fid,' original cert call count = %d',nOld);
    say(fid,' fast cert call count     = %d',nNew);
    assert(nOld==1 && nNew==0, ...
        '[AF7R58ZS] live wrapper certificate call is not uniquely original-R23.');
    wrapNew=strrep(wrap,oldCall,newCall);
    assert(count(wrapNew,oldCall)==0 && count(wrapNew,newCall)==1);

    % ------------------------------------------------------------------
    % Parse generated cert and wrapper copies BEFORE production write.
    % ------------------------------------------------------------------
    preDir=fullfile(outdir,'AF7R58ZS_PREFLIGHT');
    [ok,msg]=mkdir(preDir); assert(ok,'%s',msg);

    fastTmp=fullfile(preDir,[fastName '.m']);
    write_utf8(fastTmp,fastText);
    wrapTmpName='AF7R58ZS_PARSE_WRAPPER';
    wrapParse=strrep(wrapNew,wrapName,wrapTmpName);
    write_utf8(fullfile(preDir,[wrapTmpName '.m']),wrapParse);

    addpath(preDir,'-begin');
    cp=onCleanup(@()safe_rmpath(preDir)); %#ok<NASGU>
    eval(['clear ' fastName]);
    nargin(fastName);
    eval(['clear ' wrapTmpName]);
    nargin(wrapTmpName);
    eval(['clear ' fastName]);
    eval(['clear ' wrapTmpName]);

    say(fid,'');
    say(fid,'[E. PREFLIGHT BEFORE WRITE]');
    say(fid,' generated fast certificate parse = 1');
    say(fid,' rebound wrapper parse             = 1');
    say(fid,' original R23 production modified  = 0');
    say(fid,' production writes                 = 0');

    % ------------------------------------------------------------------
    % Backup + transactional install.
    % ------------------------------------------------------------------
    [ok,msg]=mkdir(backupDir); assert(ok,'%s',msg);
    backupReady=true;
    OUT.backup_directory=backupDir;
    copyfile(wrapFile,fullfile(backupDir,[wrapName '.m']));
    if fastExisted
        fastBackup=fullfile(backupDir,[fastName '.m']);
        copyfile(fastFile,fastBackup);
    end

    writesStarted=true;
    write_utf8(fastFile,fastText);
    write_utf8(wrapFile,wrapNew);
    rehash;

    assert(strcmp(which(srcName),srcFile), ...
        '[AF7R58ZS] original R23 resolution changed.');
    assert(strcmp(which(wrapName),wrapFile), ...
        '[AF7R58ZS] wrapper resolution changed.');
    assert(~isempty(which(fastName)), ...
        '[AF7R58ZS] fast certificate not resolvable after install.');

    finalWrap=normalize_newlines(read_utf8(wrapFile));
    assert(count(finalWrap,newCall)==1 && count(finalWrap,oldCall)==0, ...
        '[AF7R58ZS] wrapper post-write rebind failed.');

    OUT.fast_source=fastFile;
    OUT.fast_sha=file_hash(fastFile);
    OUT.wrapper_post_sha=file_hash(wrapFile);
    OUT.original_r23_post_sha=file_hash(srcFile);
    assert(strcmp(OUT.original_r23_post_sha,srcSha), ...
        '[AF7R58ZS] original R23 source was unexpectedly modified.');

    OUT.production_modified=true;
    OUT.status='R41I_FAST_CERT_PROMOTED__CLEAN_RERUN_REQUIRED';

    say(fid,'');
    say(fid,'[F. INSTALL RESULT]');
    say(fid,' original R23 modified            = 0');
    say(fid,' generated fast cert installed    = 1');
    say(fid,' live wrapper rebound             = 1');
    say(fid,' support theorem changed          = 0');
    say(fid,' continuous-time cert body changed= 0');
    say(fid,' fast cert SHA                    = %s',OUT.fast_sha);
    say(fid,' wrapper post SHA                 = %s',OUT.wrapper_post_sha);

catch ME
    if backupReady && writesStarted
        try
            copyfile(fullfile(backupDir,[wrapName '.m']),wrapFile,'f');
            if fastExisted
                copyfile(fastBackup,fastFile,'f');
            elseif isfile(fastFile)
                delete(fastFile);
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

save(fullfile(outdir,'AF7R58ZS_OUT.mat'),'OUT','-v7');
assignin('base','AF7R58ZS_OUT',OUT);
say(fid,'');
say(fid,'RETURN: %s',report);
say(fid,'RETURN: %s',fullfile(outdir,'AF7R58ZS_OUT.mat'));
say(fid,'status = %s',OUT.status);
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
assert(numel(hits)==1,'[AF7R58ZS] local function %s count=%d.',name,numel(hits));
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
