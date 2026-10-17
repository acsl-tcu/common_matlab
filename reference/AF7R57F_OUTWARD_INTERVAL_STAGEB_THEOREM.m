function OUT = AF7R57F_OUTWARD_INTERVAL_STAGEB_THEOREM()
% AF7R57F_OUTWARD_INTERVAL_STAGEB_THEOREM
% -----------------------------------------------------------------------------
% Full outward interval Stage-B audit over the actual fixed boundary bracket.
%
% Authority:
%   AF7R57C  : same-run snapshot + runtime trace/KKT schema
%   AF7R57E4 : strict native/MATLAB/prime source binding
%
% Acceptance:
%   G = VQ_StageB - VQ_limit
%   sup(G) < 0 and sigma_LB > 0 => CERTIFIED_SAFE
%   inf(G) > 0                  => CERTIFIED_STAGEB_VIOLATION
%   otherwise                   => INCONCLUSIVE_FAIL_CLOSED
%
% IMPORTANT:
%   This theorem certifies the Stage-B finite-sensing inequality over the
%   entire KKT box and t1 bracket.  It does NOT use sampled acceptance.
%   If any required interval remains unresolved, production t1 stays unselected.
% -----------------------------------------------------------------------------

clc;
fprintf('\n====================================================================================================\n');
fprintf(' AF7R57F V3 | STABLE-DERIVATIVE OUTWARD INTERVAL STAGE-B THEOREM\n');
fprintf('====================================================================================================\n');
fprintf('[AUTHORITY] intervalT1=1 | intervalKKT=1 | commonTree=1 | outwardRounding=1 | sampledAcceptance=0 | productionModified=0\n');

OUT=struct();
OUT.schema='AF7R57F_STABLE_DERIVATIVE_OUTWARD_INTERVAL_STAGEB_THEOREM_V3';
OUT.closed=false;
OUT.production_modified=false;
OUT.production_t1_selected=false;
OUT.continuous_terminal_boundary_proof=false;

%% A. Load closed authorities
fprintf('\n[A. CLOSED AUTHORITIES]\n');

C=af7r57f_get_authority('AF7R57C_SNAPSHOT_BOUND_SCHEMA_CAPTURE_OUT', ...
                        'AF7R57C_SNAPSHOT_BOUND_SCHEMA_CAPTURE_OUT.mat');

assert(isstruct(C)&&isfield(C,'closed')&&logical(C.closed), ...
    '[AF7R57F] AF7R57C authority is not closed.');

fprintf(' AF7R57C closed                          = 1\n');

V4present=false;
if evalin('base','exist(''AF7R57E_V4_STRICT_SOURCE_BINDING_OUT'',''var'')==1')
    V4tmp=evalin('base','AF7R57E_V4_STRICT_SOURCE_BINDING_OUT');
    V4present=isstruct(V4tmp)&&isfield(V4tmp,'closed')&&logical(V4tmp.closed);
end
fprintf(' V4 closed result present in base        = %d\n',V4present);
fprintf(' V4 MAT required for theorem             = 0\n');

% Re-bind the exact sources that V4 closed.  If any source changed after V4,
% this theorem must not reuse the old source-binding result.
hpp=af7r57f_find_source('af7r56_trace_core.hpp');
mexcpp0=af7r57f_find_source('af7r56_trace_mex.cpp');
rps=which('REPLANNING_SYSTEM');
prime=which('AF7R56B_PRIME_TRACE_REFERENCE');

assert(~isempty(hpp)&&~isempty(mexcpp0)&&~isempty(rps)&&~isempty(prime), ...
    '[AF7R57F] bound source path unavailable.');

expectedHpp='9f1c7054bbac0dadc2e82fbb107183937143b39431d8996ee13a620cc62ef821';
expectedMex='cef987c012982982d2fc4381419781770fe9304c6a2ba4f1d7cff1bba2746f23';
expectedRps='223d34c49d7148b8ea14ff4ab9c420d409560c0650ac19e9b0f99318887090c9';
expectedPrime='62f56ba16b01ce4b566eb9369fc19fa5bb254e6993e6b5b1c4e2062040a0c01c';

sourceSHAClosed = ...
    strcmpi(af7r57f_sha256_file(hpp),expectedHpp) && ...
    strcmpi(af7r57f_sha256_file(mexcpp0),expectedMex) && ...
    strcmpi(af7r57f_sha256_file(rps),expectedRps) && ...
    strcmpi(af7r57f_sha256_file(prime),expectedPrime);

fprintf(' V4 bound source SHA still current       = %d\n',sourceSHAClosed);
fprintf(' V4 source authority reconstructed       = %d\n',sourceSHAClosed);
assert(sourceSHAClosed,'[AF7R57F] a V4-bound source changed after source binding.');

assert(evalin('base','exist(''replanner_instance'',''var'')==1'), ...
    '[AF7R57F] replanner_instance missing.');
replanner=evalin('base','replanner_instance');
assert(isa(replanner,'REPLANNING_SYSTEM'));

%% B. Build / bind theorem MEX
fprintf('\n[B. OUTWARD INTERVAL MEX]\n');

root=fileparts(mfilename('fullpath'));
cpp=fullfile(root,'af7r57f_v3_interval_stageb_mex.cpp');
assert(isfile(cpp),'[AF7R57F] interval MEX source missing.');

srcHash=af7r57f_sha256_file(cpp);
fprintf(' interval MEX source                    = %s\n',cpp);
fprintf(' interval MEX source SHA-256            = %s\n',srcHash);

mexName='af7r57f_v3_interval_stageb_mex';
needBuild=exist(mexName,'file')~=3;

if ~needBuild
    mexFile=which(mexName);
    needBuild=isempty(mexFile) || dir(mexFile).datenum < dir(cpp).datenum;
end

if needBuild
    mex('-R2018a','-outdir',root,cpp);
    rehash;
end

assert(exist(mexName,'file')==3,'[AF7R57F] interval MEX build failed.');
fprintf(' interval MEX available                 = 1\n');
fprintf(' outward primitive                      = std::nextafter\n');
fprintf(' derivative ordering                    = SEGMENT_DERIVATIVE_THEN_LEAF_RESTRICTION\n');
fprintf(' derivative/restriction identity        = EXACT_BEZIER_COMMUTATION\n');

%% C. Same-run packet / nominal authority
fprintf('\n[C. SAME-RUN ACTIVE REFERENCE AUTHORITY]\n');

TR=C.trace_records;
KR=C.kkt_records;
assert(iscell(TR)&&iscell(KR)&&~isempty(TR)&&~isempty(KR));

pkt=TR{1}.pkt;

F=replanner.nominal_flat_derivative_fn(1:7);
assert(iscell(F)&&numel(F)>=7&&all(cellfun(@(f)isa(f,'function_handle'),F)));
S=cellfun(@func2str,F,'UniformOutput',false);
sig=af7r57f_sha256_text(strjoin(S,char(10)));

fprintf(' current nominal signature              = %s\n',sig);
fprintf(' packet nominal signature               = %s\n',char(pkt.nominal_signature));
fprintf(' nominal signature equal                = %d\n',strcmp(sig,char(pkt.nominal_signature)));
assert(strcmp(sig,char(pkt.nominal_signature)), ...
    '[AF7R57F] nominal source changed after AF7R57C.');

repairCount=numel(pkt.repair_t0);
fprintf(' immutable prior repair count           = %d\n',repairCount);

% Current theorem engine is rigorously implemented for the authority state
% with no previously active repair segments. Never ignore nonempty repairs.
if repairCount~=0 || ~isempty(pkt.repair_t1) || ~isempty(pkt.repair_seg0) || ...
        ~isempty(pkt.repair_seg1) || ~isempty(pkt.repair_cp)
    error('AF7R57F:RepairIntervalUnsupported', ...
        ['Prior immutable repairs are present. This V1 theorem engine refuses ' ...
         'to ignore them: FAIL CLOSED.']);
end
fprintf(' active-reference interval scope         = NOMINAL_ONLY_BY_RUNTIME_AUTHORITY\n');

%% D. Common exact endpoint-union trees
fprintf('\n[D. EXACT COMMON TREES]\n');

cand=unique(cellfun(@(q)double(q.c),TR));
TREES=cell(1,max(cand));

for c=cand(:).'
    R=af7r57f_record(TR,c);
    [tree,audit]=af7r57f_common_tree(R.TL{c},R.TU{c});
    assert(audit.closed,'[AF7R57F] common tree not exact.');
    TREES{c}=tree;
    minDu=min(tree(:,3)-tree(:,2));
    fprintf(' candidate %d common leaves             = %d | exact=%d | min du=%.17g\n', ...
        c,size(tree,1),audit.closed,minDu);
end

%% E. Outward interval evaluation
fprintf('\n[E. OUTWARD INTERVAL STAGE-B]\n');

RESULT=cell(1,max(cand));
allKKT=true;
allResolved=true;
candidateSafe=false(1,max(cand));
candidateReject=false(1,max(cand));
candidateInconclusive=false(1,max(cand));

for c=cand(:).'
    rk=af7r57f_record(KR,c);
    rt=af7r57f_record(TR,c);

    K=rk.K{c};
    U=rk.U;
    assert(logical(K.closed)&&logical(K.spd)&&logical(K.lambda_positive)&&logical(K.inactive_strict));
    allKKT=allKKT&&logical(K.closed);

    zI=double(K.z_interval);
    tI=double(K.t_interval(:).');
    cs=double(U.component_start(:).');
    ce=double(U.component_end(:).');
    tree=TREES{c};

    assert(numel(cs)==numel(ce)&&numel(cs)>=1);
    assert(size(zI,1)==6*numel(cs)&&size(zI,2)==2, ...
        '[AF7R57F] z interval / component count mismatch.');

    limL=double(rt.TL{c}.limit);
    limU=double(rt.TU{c}.limit);
    assert(limL==limU,'[AF7R57F] endpoint Stage-B limits differ.');
    limit=limL;

    % Exact outward leaf midpoint-time intervals from the MEX.
    Tm=af7r57f_v3_interval_stageb_mex(0,tI,double(U.t0),cs,ce,tree,0);

    n=size(tree,1);
    VC=zeros(3,n); AC=zeros(3,n);
    vRad=zeros(n,1); aRad=zeros(n,1);

    for j=1:n
        tc=Tm.tc_center(j);
        tau=max(0,double(tc)-double(pkt.flight_t0));

        v=F{2}(tau); a=F{3}(tau);
        v=double(v(:)); a=double(a(:));
        assert(numel(v)==3&&numel(a)==3&&all(isfinite(v))&&all(isfinite(a)));

        VC(:,j)=v; AC(:,j)=a;

        % Continuous-time reference variation around the point value:
        % ||v(t)-v(tc)|| <= A_base |t-tc|
        % ||a(t)-a(tc)|| <= J_base |t-tc|
        vRad(j)=double(pkt.base_payload_accel)*double(Tm.tc_radius(j));
        aRad(j)=double(pkt.base_payload_jerk)*double(Tm.tc_radius(j));
    end

    R=af7r57f_v3_interval_stageb_mex(1,zI,tI,double(U.t0),cs,ce,tree, ...
        double(pkt.base_payload_accel),double(pkt.base_payload_jerk), ...
        double(pkt.gravity),double(pkt.cable_length), ...
        VC,AC,vRad,aRad,limit,0);

    cls=double(R.class_code(:));
    nSafe=nnz(cls==-1);
    nViol=nnz(cls==1);
    nInc=nnz(cls==0);

    candidateSafe(c)=nSafe==numel(cls);
    candidateReject(c)=nViol>0;
    candidateInconclusive(c)=~candidateSafe(c)&&~candidateReject(c);
    allResolved=allResolved&&(nInc==0);

    fprintf('\n candidate %d\n',c);
    fprintf('  KKT full-bracket closed               = %d\n',logical(K.closed));
    fprintf('  common leaves                         = %d\n',numel(cls));
    fprintf('  safe / violation / inconclusive       = %d / %d / %d\n',nSafe,nViol,nInc);
    fprintf('  candidate uniformly Stage-B safe      = %d\n',candidateSafe(c));
    fprintf('  candidate uniformly rejected by leaf  = %d\n',candidateReject(c));
    fprintf('  candidate interval-inconclusive       = %d\n',candidateInconclusive(c));

    [gHi,ig]=max(R.G(:,2));
    [gLo,jg]=min(R.G(:,1));
    [sLo,is]=min(R.sigma(:,1));
    [vqHi,iv]=max(R.vq(:,2));

    fprintf('  max G upper                           = %.17g | leaf=%d seg=%g u=[%.17g %.17g]\n', ...
        gHi,ig,R.seg(ig),R.u0(ig),R.u1(ig));
    fprintf('  min G lower                           = %.17g | leaf=%d\n',gLo,jg);
    fprintf('  min sigma lower                       = %.17g | leaf=%d\n',sLo,is);
    fprintf('  max VQ upper                          = %.17g | leaf=%d\n',vqHi,iv);

    RESULT{c}=R;
end

%% F. Terminal-boundary theorem classification
fprintf('\n[F. TERMINAL-BOUNDARY THEOREM]\n');

safeCandidates=find(candidateSafe);
rejectCandidates=find(candidateReject);
incCandidates=find(candidateInconclusive);

fprintf(' uniformly safe candidates              = %s\n',mat2str(safeCandidates));
fprintf(' uniformly rejected candidates          = %s\n',mat2str(rejectCandidates));
fprintf(' interval-inconclusive candidates       = %s\n',mat2str(incCandidates));
fprintf(' all-candidate KKT closed               = %d\n',allKKT);

% A full-bracket safe production selection exists only if at least one fixed
% candidate branch is Stage-B safe on every common leaf for the entire
% t1 x z interval box.
uniformSafeExists=~isempty(safeCandidates);

% The continuous terminal-boundary theorem is considered closed only when
% every candidate branch is classified without interval ambiguity.
continuousClosed=allResolved&&allKKT;

fprintf(' all leaf boxes sign-resolved           = %d\n',allResolved);
fprintf(' continuous terminal-boundary proof     = %d\n',continuousClosed);
fprintf(' full-bracket uniform-safe branch exists= %d\n',uniformSafeExists);

% Keep production selection separate. This theorem never installs t1.
fprintf(' production t1 selected                 = 0\n');
fprintf(' production source modified             = 0\n');

if continuousClosed && uniformSafeExists
    next='PRODUCTION_T1_SELECTION_SHADOW_GATE';
elseif ~continuousClosed
    next='FAIL_CLOSED__INTERVAL_REFINEMENT_REQUIRED';
else
    next='FAIL_CLOSED__NO_UNIFORM_SAFE_BRANCH_ON_BOUNDARY_BRACKET';
end

fprintf(' next action                             = %s\n',next);
fprintf('====================================================================================================\n');

OUT.closed=continuousClosed;
OUT.derivative_ordering='SEGMENT_DERIVATIVE_THEN_LEAF_RESTRICTION';
OUT.derivative_restriction_identity='EXACT_BEZIER_COMMUTATION';
OUT.v4_mat_required=false;
OUT.v4_source_authority_reconstructed=sourceSHAClosed;
OUT.interval_stageb_theorem_implemented=true;
OUT.outward_stageb_leaf_interval_proof=continuousClosed;
OUT.continuous_terminal_boundary_proof=continuousClosed;
OUT.uniform_safe_branch_exists=uniformSafeExists;
OUT.safe_candidates=safeCandidates;
OUT.rejected_candidates=rejectCandidates;
OUT.inconclusive_candidates=incCandidates;
OUT.results=RESULT;
OUT.common_trees=TREES;
OUT.next_action=next;
OUT.production_t1_selected=false;
OUT.production_modified=false;

assignin('base','AF7R57F_OUTWARD_INTERVAL_STAGEB_THEOREM_OUT',OUT);
save(fullfile(root,'AF7R57F_OUTWARD_INTERVAL_STAGEB_THEOREM_OUT.mat'),'OUT','-v7.3');
end


function A=af7r57f_get_authority(varName,fileName)
% Numerical authority loader. Prefer the base-workspace object from the
% closed run. If absent, locate the MAT next to its generating audit or on
% the MATLAB path; never assume the current working directory.

if evalin('base',sprintf('exist(''%s'',''var'')==1',varName))
    A=evalin('base',varName);
    return;
end

cand={};

p=which(fileName);
if ~isempty(p),cand{end+1}=p;end %#ok<AGROW>

cand{end+1}=fullfile(pwd,fileName); %#ok<AGROW>
cand{end+1}=fullfile(fileparts(mfilename('fullpath')),fileName); %#ok<AGROW>

if startsWith(varName,'AF7R57C_')
    q=which('AF7R57C_SNAPSHOT_BOUND_SCHEMA_CAPTURE');
    if ~isempty(q)
        cand{end+1}=fullfile(fileparts(q),fileName); %#ok<AGROW>
    end
end

u=getenv('USERPROFILE');
if ~isempty(u)
    ref=fullfile(u,'Documents','GitHub','common_matlab','reference');
    cand{end+1}=fullfile(ref,fileName); %#ok<AGROW>
end

p='';
for i=1:numel(cand)
    if ~isempty(cand{i})&&isfile(cand{i})
        p=cand{i};
        break;
    end
end

assert(~isempty(p), ...
    '[AF7R57F] numerical authority unavailable: %s',fileName);

S=load(p,'OUT');
assert(isfield(S,'OUT'),'[AF7R57F] authority MAT has no OUT: %s',p);
A=S.OUT;
end


function r=af7r57f_record(R,c)
ids=cellfun(@(q)double(q.c),R);
k=find(ids==c,1,'last');
assert(~isempty(k));r=R{k};
end


function [C,A]=af7r57f_common_tree(L,U)
PL=sortrows([double(L.leaf_seg(:)) double(L.leaf_u0(:)) double(L.leaf_u1(:))],[1 2 3]);
PU=sortrows([double(U.leaf_seg(:)) double(U.leaf_u0(:)) double(U.leaf_u1(:))],[1 2 3]);

segs=unique([PL(:,1);PU(:,1)]);
C=zeros(0,3);
for s=segs(:).'
    B=unique([PL(PL(:,1)==s,2:3);PU(PU(:,1)==s,2:3)]);
    B=sort(unique(B(:)));
    assert(B(1)==0&&B(end)==1);
    for j=1:numel(B)-1
        assert(B(j)<B(j+1));
        C(end+1,:)=[s B(j) B(j+1)]; %#ok<AGROW>
    end
end

A=struct();
A.lower_cover=af7r57f_cover(PL);
A.upper_cover=af7r57f_cover(PU);
A.common_cover=af7r57f_cover(C);
A.refines_lower=af7r57f_refines(C,PL);
A.refines_upper=af7r57f_refines(C,PU);
A.boundary_union=af7r57f_boundary_union(C,PL,PU);
A.closed=A.lower_cover&&A.upper_cover&&A.common_cover&& ...
    A.refines_lower&&A.refines_upper&&A.boundary_union;
end


function ok=af7r57f_cover(P)
ok=true;
for s=unique(P(:,1)).'
    Q=sortrows(P(P(:,1)==s,:),2);
    ok=ok&&Q(1,2)==0&&Q(end,3)==1;
    for j=1:size(Q,1)-1
        ok=ok&&Q(j,3)==Q(j+1,2);
    end
end
end


function ok=af7r57f_refines(C,P)
ok=true;
for i=1:size(C,1)
    Q=P(P(:,1)==C(i,1),:);
    hit=find(Q(:,2)<=C(i,2)&Q(:,3)>=C(i,3));
    if numel(hit)~=1,ok=false;return;end
end
end


function ok=af7r57f_boundary_union(C,L,U)
ok=true;
for s=unique([L(:,1);U(:,1)]).'
    B=sort(unique([L(L(:,1)==s,2:3);U(U(:,1)==s,2:3)]));
    B=sort(unique(B(:)));
    Q=C(C(:,1)==s,2:3);
    CQ=sort(unique(Q(:)));
    if ~isequal(B(:),CQ(:)),ok=false;return;end
end
end


function fn=af7r57f_find_source(name)
fn='';
p=which(name);
if ~isempty(p)&&isfile(p)
    fn=p;
    return;
end

roots={pwd,fileparts(mfilename('fullpath'))};
p=which('af7r56_trace_mex');
if ~isempty(p),roots{end+1}=fileparts(p);end %#ok<AGROW>
p=which('AF7R56B_STAGEB_BOUNDARY_TRACE');
if ~isempty(p),roots{end+1}=fileparts(p);end %#ok<AGROW>
u=getenv('USERPROFILE');
if ~isempty(u)
    roots{end+1}=fullfile(u,'Documents','GitHub','common_matlab','reference'); %#ok<AGROW>
end

for i=1:numel(roots)
    q=fullfile(roots{i},name);
    if isfile(q)
        fn=q;
        return;
    end
end
end


function h=af7r57f_sha256_text(s)
md=java.security.MessageDigest.getInstance('SHA-256');
md.update(uint8(unicode2native(char(s),'UTF-8')));
d=typecast(md.digest(),'uint8');
h=lower(reshape(dec2hex(d,2).',1,[]));
end


function h=af7r57f_sha256_file(fn)
md=java.security.MessageDigest.getInstance('SHA-256');
fid=fopen(fn,'rb');assert(fid>=0);
c=onCleanup(@()fclose(fid));
while true
    b=fread(fid,1024*1024,'*uint8');
    if isempty(b),break;end
    md.update(b);
end
d=typecast(md.digest(),'uint8');
h=lower(reshape(dec2hex(d,2).',1,[]));
end
