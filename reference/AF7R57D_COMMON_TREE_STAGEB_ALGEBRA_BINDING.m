function OUT = AF7R57D_COMMON_TREE_STAGEB_ALGEBRA_BINDING()
% AF7R57D_COMMON_TREE_STAGEB_ALGEBRA_BINDING
% -----------------------------------------------------------------------------
% Read-only bridge from the closed AF7R57C runtime schema to the actual
% COMMON_DYADIC_REFINEMENT_INTERVAL_STAGEB theorem.
%
% This program:
%   1) loads the exact AF7R57C captures;
%   2) constructs the exact common endpoint-union refinement from lower/upper
%      leaf partitions, per candidate and per segment, with no tolerance;
%   3) proves exact [0,1] cover and exact refinement of both endpoint trees;
%      dyadic splitter provenance is checked separately from native source;
%   4) audits the returned Stage-B algebraic identities (qdot and VQ);
%   5) source-binds the remaining VL/AL/JL/sigma/control-point algebra by
%      searching the actual AF7R56 native/build sources and printing exact
%      source excerpts with SHA-256;
%   6) refuses to claim the terminal-boundary theorem until that algebra is
%      source-bound.
%
% No production modification. No production t1 selection. No sampled
% acceptance.
% -----------------------------------------------------------------------------

clc;
fprintf('\n====================================================================================================\n');
fprintf(' AF7R57D V2 | EXACT COMMON TREE + STAGE-B ALGEBRA / DYADIC-PROVENANCE BINDING\n');
fprintf('====================================================================================================\n');
fprintf('[AUTHORITY] readOnly=1 | exactDyadic=1 | sourceBoundAlgebra=1 | sampledAcceptance=0 | productionModified=0\n');

OUT=struct();
OUT.schema='AF7R57D_COMMON_TREE_STAGEB_ALGEBRA_BINDING_V2';
OUT.closed=false;
OUT.production_modified=false;
OUT.production_t1_selected=false;
OUT.continuous_terminal_boundary_proof=false;

%% A. Load AF7R57C authority
fprintf('\n[A. AF7R57C AUTHORITY]\n');

C=[];
if evalin('base','exist(''AF7R57C_SNAPSHOT_BOUND_SCHEMA_CAPTURE_OUT'',''var'')==1')
    C=evalin('base','AF7R57C_SNAPSHOT_BOUND_SCHEMA_CAPTURE_OUT');
    cSource='BASE:AF7R57C_SNAPSHOT_BOUND_SCHEMA_CAPTURE_OUT';
else
    cfile=af7r57d_find_file('AF7R57C_SNAPSHOT_BOUND_SCHEMA_CAPTURE_OUT.mat');
    assert(~isempty(cfile),'[AF7R57D] AF7R57C output not found.');
    S=load(cfile,'OUT');
    C=S.OUT;
    cSource=cfile;
end

assert(isstruct(C) && isfield(C,'closed') && logical(C.closed), ...
    '[AF7R57D] AF7R57C authority is not closed.');
assert(isfield(C,'trace_records') && iscell(C.trace_records), ...
    '[AF7R57D] AF7R57C trace records missing.');
assert(isfield(C,'kkt_records') && iscell(C.kkt_records), ...
    '[AF7R57D] AF7R57C KKT records missing.');

TR=C.trace_records;
KR=C.kkt_records;

fprintf(' authority source                         = %s\n',cSource);
fprintf(' AF7R57C closed                          = 1\n');
fprintf(' trace records                           = %d\n',numel(TR));
fprintf(' KKT records                             = %d\n',numel(KR));

cand=unique(cellfun(@(q)double(q.c),TR));
fprintf(' candidate ids                           = %s\n',mat2str(cand));

assert(numel(cand)>=1,'[AF7R57D] no candidate trace records.');

%% B. Bracket / KKT consistency
fprintf('\n[B. FIXED BOUNDARY / KKT CONSISTENCY]\n');

allLo=cellfun(@(q)double(q.lo),TR);
allHi=cellfun(@(q)double(q.hi),TR);
lo=allLo(1); hi=allHi(1);

sameBracket=all(allLo==lo) && all(allHi==hi);
fprintf(' t1 interval                             = [%.17g, %.17g]\n',lo,hi);
fprintf(' interval width                          = %.17g\n',hi-lo);
fprintf(' all trace records same bracket          = %d\n',sameBracket);

kktClosed=true;
for i=1:numel(KR)
    c=double(KR{i}.c);
    K=KR{i}.K{c};
    q=logical(K.closed)&&logical(K.spd)&&logical(K.lambda_positive)&& ...
      logical(K.inactive_strict) && isequal(double(K.t_interval(:).'),[lo hi]);
    kktClosed=kktClosed&&q;
    fprintf(' candidate %g KKT closed/SPD/lambda+/inactive = %d/%d/%d/%d\n', ...
        c,logical(K.closed),logical(K.spd),logical(K.lambda_positive),logical(K.inactive_strict));
    fprintf(' candidate %g inactive slack LB          = %.17g\n', ...
        c,double(K.min_inactive_slack_lb));
end

fprintf(' all-candidate interval KKT closed       = %d\n',kktClosed);
assert(sameBracket&&kktClosed,'[AF7R57D] bracket/KKT authority mismatch.');

%% C. Exact common endpoint-union refinement
fprintf('\n[C. EXACT COMMON ENDPOINT-UNION REFINEMENT]\n');

TREES=cell(1,max(cand));
treeClosed=true;

for c=cand(:).'
    r=af7r57d_record_for_candidate(TR,c);
    L=r.TL{c};
    U=r.TU{c};

    assert(isstruct(L)&&isstruct(U),'[AF7R57D] missing endpoint trace.');

    [common,audit]=af7r57d_common_tree(L,U);
    TREES{c}=common;

    fprintf('\n candidate %d\n',c);
    fprintf('  lower leaves                           = %d\n',numel(L.leaf_seg));
    fprintf('  upper leaves                           = %d\n',numel(U.leaf_seg));
    fprintf('  common leaves                          = %d\n',size(common,1));
    fprintf('  exact lower cover                      = %d\n',audit.lower_cover);
    fprintf('  exact upper cover                      = %d\n',audit.upper_cover);
    fprintf('  exact common cover                     = %d\n',audit.common_cover);
    fprintf('  common refines lower exactly           = %d\n',audit.refines_lower);
    fprintf('  common refines upper exactly           = %d\n',audit.refines_upper);
    fprintf('  common boundaries from endpoint union  = %d\n',audit.boundary_union_exact);
    fprintf('  tolerance/snapping used                = 0\n');

    segs=unique(common(:,1)).';
    for s=segs
        n=nnz(common(:,1)==s);
        fprintf('   segment %g common leaves              = %d\n',s,n);
    end

    treeClosed=treeClosed && audit.closed;
end

fprintf('\n exact common refinement kernel closed    = %d\n',treeClosed);
assert(treeClosed,'[AF7R57D] exact common refinement failed.');

%% D. Returned Stage-B algebra identities
fprintf('\n[D. RETURNED STAGE-B ALGEBRA IDENTITIES]\n');

algClosed=true;
maxQdotErr=0;
maxVqErr=0;
tested=0;

for c=cand(:).'
    r=af7r57d_record_for_candidate(TR,c);
    ends={r.TL{c},r.TU{c}};

    for e=1:2
        T=ends{e};
        Lc=double(r.pkt.cable_length);

        for j=1:numel(T.leaf_vl)
            vl=double(T.leaf_vl(j));
            jl=double(T.leaf_jl(j));
            sig=double(T.leaf_sigma(j));
            qd=double(T.leaf_qdot(j));
            vq=double(T.leaf_vq(j));

            if sig>0 && isfinite(jl)
                qref=jl/sig;
            else
                qref=Inf;
            end

            if isfinite(qd)&&isfinite(qref)
                eq=abs(qd-qref);
                maxQdotErr=max(maxQdotErr,eq);
                qok=eq<=32*eps(max(1,abs(qref)));
            else
                qok=isinf(qd)&&isinf(qref);
            end

            vref=vl+Lc*qref;
            if isfinite(vq)&&isfinite(vref)
                ev=abs(vq-vref);
                maxVqErr=max(maxVqErr,ev);
                vok=ev<=64*eps(max(1,abs(vref)));
            else
                vok=isinf(vq)&&isinf(vref);
            end

            algClosed=algClosed&&qok&&vok;
            tested=tested+1;
        end
    end
end

fprintf(' leaf algebra cases tested               = %d\n',tested);
fprintf(' qdot = JL/sigma (sigma>0 else Inf)      = %d\n',algClosed);
fprintf(' VQ = VL + L*qdot                        = %d\n',algClosed);
fprintf(' max finite qdot identity error          = %.17g\n',maxQdotErr);
fprintf(' max finite VQ identity error            = %.17g\n',maxVqErr);

%% E. Native/build source search
fprintf('\n[E. STAGE-B SOURCE SEARCH]\n');

mexPath=which('af7r56_trace_mex');
assert(~isempty(mexPath),'[AF7R57D] af7r56_trace_mex missing.');

fprintf(' trace MEX                               = %s\n',mexPath);

roots=af7r57d_source_roots(mexPath);
tokens={ ...
    'leaf_vl', ...
    'leaf_al', ...
    'leaf_jl', ...
    'leaf_sigma', ...
    'leaf_qdot', ...
    'leaf_vq', ...
    'leaf_control_points', ...
    'witness_max', ...
    'finite_witness', ...
    'specific_force', ...
    'sigma', ...
    'qdot', ...
    'leaf_u0', ...
    'leaf_u1', ...
    'split_count', ...
    'midpoint', ...
    'split'};

H=af7r57d_source_hits(roots,tokens);

fprintf(' source files with Stage-B token hits     = %d\n',numel(H));

for i=1:numel(H)
    fprintf('\n ---- SOURCE %d ----\n',i);
    fprintf(' file                                    = %s\n',H(i).file);
    fprintf(' SHA-256                                 = %s\n',H(i).sha256);
    fprintf(' matched tokens                          = %s\n',strjoin(H(i).tokens,', '));
    fprintf(' hit line count                          = %d\n',numel(H(i).lines));

    show=unique(H(i).lines);
    if numel(show)>80
        show=show(1:80);
    end

    txt=splitlines(string(af7r57d_normalize_newlines(fileread(H(i).file))));
    for k=1:numel(show)
        a=max(1,show(k)-2);
        b=min(numel(txt),show(k)+3);
        fprintf('  [context around L%d]\n',show(k));
        for q=a:b
            fprintf('   L%-5d %s\n',q,char(txt(q)));
        end
    end
end

% A source binding is considered found only if at least one actual source
% file exposes the core returned leaf field names, not merely a binary MEX.
fieldTokens={'leaf_vl','leaf_al','leaf_jl','leaf_sigma','leaf_qdot','leaf_vq'};
bound=false(1,numel(fieldTokens));
for i=1:numel(H)
    for k=1:numel(fieldTokens)
        bound(k)=bound(k)||any(strcmp(H(i).tokens,fieldTokens{k}));
    end
end
sourceAlgebraBound=all(bound);

% "Dyadic" is a provenance statement about the adaptive splitter.  It must
% be established from the actual trace/native source, not from a denominator
% test on returned floating u coordinates.
provTokens={'leaf_u0','leaf_u1','split_count'};
provBound=false(1,numel(provTokens));
splitSemantic=false;

for i=1:numel(H)
    for k=1:numel(provTokens)
        provBound(k)=provBound(k)||any(strcmp(H(i).tokens,provTokens{k}));
    end
    splitSemantic=splitSemantic || any(strcmp(H(i).tokens,'midpoint')) || ...
        any(strcmp(H(i).tokens,'split'));
end

dyadicProvenanceBound=all(provBound)&&splitSemantic;

fprintf('\n source field binding vector             = %s\n',mat2str(bound));
fprintf(' Stage-B native algebra source-bound     = %d\n',sourceAlgebraBound);
fprintf(' splitter provenance field vector        = %s\n',mat2str(provBound));
fprintf(' splitter midpoint/split source present  = %d\n',splitSemantic);
fprintf(' dyadic provenance source-bound          = %d\n',dyadicProvenanceBound);

%% F. Control-point endpoint variation diagnostic
fprintf('\n[F. ENDPOINT CONTROL-POINT VARIATION]\n');

for c=cand(:).'
    r=af7r57d_record_for_candidate(TR,c);
    L=r.TL{c};
    U=r.TU{c};

    n=min(numel(L.leaf_control_points),numel(U.leaf_control_points));
    sameTopo = n==numel(L.leaf_control_points) && n==numel(U.leaf_control_points) && ...
        isequal(L.leaf_seg,U.leaf_seg) && isequal(L.leaf_u0,U.leaf_u0) && isequal(L.leaf_u1,U.leaf_u1);

    fprintf(' candidate %d endpoint leaf topology identical = %d\n',c,sameTopo);

    if sameTopo
        dmax=0;
        for j=1:n
            dmax=max(dmax,max(abs(double(L.leaf_control_points{j}(:))- ...
                                  double(U.leaf_control_points{j}(:)))));
        end
        fprintf(' candidate %d max endpoint CP delta       = %.17g\n',c,dmax);
    else
        fprintf(' candidate %d CP delta requires common-tree re-extraction = 1\n',c);
    end
end

%% G. Formal gate
fprintf('\n[G. FORMAL GATE]\n');

srcStable=true;
for i=1:numel(H)
    srcStable=srcStable&&strcmpi(H(i).sha256,af7r57d_sha256_file(H(i).file));
end

preTheoremClosed=treeClosed&&kktClosed&&algClosed;

fprintf(' interval KKT authority closed           = %d\n',kktClosed);
fprintf(' exact common endpoint tree closed       = %d\n',treeClosed);
fprintf(' returned qdot/VQ algebra closed         = %d\n',algClosed);
fprintf(' Stage-B native algebra source-bound     = %d\n',sourceAlgebraBound);
fprintf(' dyadic provenance source-bound          = %d\n',dyadicProvenanceBound);
fprintf(' source SHA unchanged                    = %d\n',srcStable);
fprintf(' interval Stage-B theorem implemented    = 0\n');
fprintf(' outward Stage-B leaf interval proof     = 0\n');
fprintf(' continuous terminal-boundary proof      = 0\n');
fprintf(' production t1 selected                  = 0\n');
fprintf(' production source modified              = 0\n');
fprintf(' unresolved interval policy              = FAIL CLOSED\n');

if sourceAlgebraBound && dyadicProvenanceBound
    next='IMPLEMENT_OUTWARD_COMMON_TREE_STAGEB_FROM_BOUND_NATIVE_ALGEBRA';
elseif sourceAlgebraBound
    next='BIND_ADAPTIVE_SPLITTER_DYADIC_PROVENANCE_THEN_IMPLEMENT_INTERVAL_STAGEB';
else
    next='LOCATE_NATIVE_STAGEB_ALGEBRA_SOURCE_OR_BUILD_GENERATOR';
end

fprintf(' next action                              = %s\n',next);
fprintf('====================================================================================================\n');

OUT.closed=preTheoremClosed&&srcStable;
OUT.common_trees=TREES;
OUT.common_tree_closed=treeClosed;
OUT.kkt_closed=kktClosed;
OUT.returned_algebra_closed=algClosed;
OUT.source_hits=H;
OUT.native_algebra_source_bound=sourceAlgebraBound;
OUT.dyadic_provenance_source_bound=dyadicProvenanceBound;
OUT.next_action=next;
OUT.production_modified=false;
OUT.production_t1_selected=false;
OUT.continuous_terminal_boundary_proof=false;

assignin('base','AF7R57D_COMMON_TREE_STAGEB_ALGEBRA_BINDING_OUT',OUT);
save('AF7R57D_COMMON_TREE_STAGEB_ALGEBRA_BINDING_OUT.mat','OUT','-v7.3');
end


function r=af7r57d_record_for_candidate(TR,c)
ids=cellfun(@(q)double(q.c),TR);
k=find(ids==c,1,'last');
assert(~isempty(k),'[AF7R57D] candidate record missing.');
r=TR{k};
end


function [C,A]=af7r57d_common_tree(L,U)
PL=af7r57d_partition(L);
PU=af7r57d_partition(U);

A=struct();
A.lower_cover=af7r57d_exact_cover(PL);
A.upper_cover=af7r57d_exact_cover(PU);

segments=unique([PL(:,1);PU(:,1)]);
C=zeros(0,3);

for s=segments(:).'
    xl=PL(PL(:,1)==s,2:3);
    xu=PU(PU(:,1)==s,2:3);

    B=unique([xl(:);xu(:)]);
    B=sort(B);

    assert(~isempty(B)&&B(1)==0&&B(end)==1, ...
        '[AF7R57D] endpoint partition is not exact [0,1].');

    for j=1:numel(B)-1
        assert(B(j)<B(j+1),'[AF7R57D] non-positive common leaf.');
        C(end+1,:)=[s B(j) B(j+1)]; %#ok<AGROW>
    end
end

A.common_cover=af7r57d_exact_cover(C);
A.refines_lower=af7r57d_refines(C,PL);
A.refines_upper=af7r57d_refines(C,PU);
A.boundary_union_exact=af7r57d_boundary_union_exact(C,PL,PU);
A.closed=A.lower_cover&&A.upper_cover&&A.common_cover&& ...
    A.refines_lower&&A.refines_upper&&A.boundary_union_exact;
end


function P=af7r57d_partition(T)
s=double(T.leaf_seg(:));
u0=double(T.leaf_u0(:));
u1=double(T.leaf_u1(:));

assert(numel(s)==numel(u0)&&numel(u0)==numel(u1)&&~isempty(s));
P=[s u0 u1];

P=sortrows(P,[1 2 3]);

assert(all(isfinite(P(:)))&&all(P(:,2)>=0)&&all(P(:,3)<=1)&&all(P(:,2)<P(:,3)));
end


function ok=af7r57d_exact_cover(P)
ok=true;
segs=unique(P(:,1)).';

for s=segs
    Q=P(P(:,1)==s,:);
    Q=sortrows(Q,2);

    ok=ok && Q(1,2)==0 && Q(end,3)==1;

    for j=1:size(Q,1)-1
        ok=ok && Q(j,3)==Q(j+1,2);
    end
end
end


function ok=af7r57d_refines(C,P)
ok=true;

for i=1:size(C,1)
    s=C(i,1); a=C(i,2); b=C(i,3);
    Q=P(P(:,1)==s,:);
    hit=find(Q(:,2)<=a & Q(:,3)>=b);

    if numel(hit)~=1
        ok=false;
        return;
    end
end
end


function ok=af7r57d_boundary_union_exact(C,PL,PU)
% Exact topology check only.  The common partition may use only endpoint
% doubles that occur verbatim in the lower/upper partitions.  No tolerance,
% snapping, rounding, or approximate merge is permitted.
ok=true;
segments=unique([PL(:,1);PU(:,1)]).';

for s=segments
    B=unique([PL(PL(:,1)==s,2:3); PU(PU(:,1)==s,2:3)]);
    B=unique(B(:));

    Q=C(C(:,1)==s,2:3);
    CQ=unique(Q(:));

    if numel(B)~=numel(CQ) || ~isequal(B(:),CQ(:))
        ok=false;
        return;
    end
end
end


function roots=af7r57d_source_roots(mexPath)
r0=fileparts(mexPath);
roots={r0};

p=fileparts(r0);
if ~isempty(p), roots{end+1}=p; end %#ok<AGROW>

u=getenv('USERPROFILE');
if ~isempty(u)
    q=fullfile(u,'Documents','GitHub','common_matlab');
    if isfolder(q), roots{end+1}=q; end %#ok<AGROW>
end

roots=unique(roots,'stable');
end


function H=af7r57d_source_hits(roots,tokens)
H=struct('file',{},'sha256',{},'tokens',{},'lines',{});

ext={'.m','.c','.cc','.cpp','.cxx','.h','.hh','.hpp','.txt'};

seen={};

for r=1:numel(roots)
    root=roots{r};
    if ~isfolder(root), continue; end

    D=dir(fullfile(root,'**','*'));

    for i=1:numel(D)
        if D(i).isdir, continue; end

        fn=fullfile(D(i).folder,D(i).name);
        [~,~,e]=fileparts(fn);

        if ~any(strcmpi(e,ext)), continue; end
        if D(i).bytes>8e6, continue; end
        if any(strcmp(seen,fn)), continue; end
        seen{end+1}=fn; %#ok<AGROW>

        % Prefer AF7R56/Stage-B/build sources; otherwise require a strong token.
        nm=lower(D(i).name);
        likely=contains(nm,'af7r56')||contains(nm,'stageb')||contains(nm,'stage_b')||contains(nm,'trace');

        try
            s=fileread(fn);
        catch
            continue;
        end

        hitTokens={};
        lineSet=[];

        L=splitlines(string(af7r57d_normalize_newlines(s)));

        for k=1:numel(tokens)
            rows=find(contains(L,tokens{k}));

            if ~isempty(rows)
                hitTokens{end+1}=tokens{k}; %#ok<AGROW>
                lineSet=[lineSet;double(rows(:))]; %#ok<AGROW>
            end
        end

        strong=any(ismember(hitTokens,{'leaf_vl','leaf_al','leaf_jl','leaf_sigma','leaf_qdot','leaf_vq','leaf_control_points'}));

        if ~isempty(hitTokens) && (likely||strong)
            q=struct();
            q.file=fn;
            q.sha256=af7r57d_sha256_file(fn);
            q.tokens=unique(hitTokens,'stable');
            q.lines=unique(lineSet(:).');
            H(end+1)=q; %#ok<AGROW>
        end
    end
end
end


function f=af7r57d_find_file(name)
f='';

p=which(name);
if ~isempty(p)&&isfile(p)
    f=p;
    return;
end

roots={pwd,fileparts(mfilename('fullpath'))};

p=which('AF7R56B_STAGEB_BOUNDARY_TRACE');
if ~isempty(p), roots{end+1}=fileparts(p); end %#ok<AGROW>

for i=1:numel(roots)
    q=fullfile(roots{i},name);
    if isfile(q)
        f=q;
        return;
    end
end
end


function s=af7r57d_normalize_newlines(s)
s=strrep(s,sprintf('\r\n'),sprintf('\n'));
s=strrep(s,sprintf('\r'),sprintf('\n'));
end


function h=af7r57d_sha256_file(fn)
md=java.security.MessageDigest.getInstance('SHA-256');
fid=fopen(fn,'rb');
assert(fid>=0,'[AF7R57D] cannot open source file.');
c=onCleanup(@() fclose(fid));

while true
    b=fread(fid,1024*1024,'*uint8');
    if isempty(b), break; end
    md.update(b);
end

raw=typecast(md.digest(),'uint8');
h=lower(reshape(dec2hex(raw,2).',1,[]));
end
