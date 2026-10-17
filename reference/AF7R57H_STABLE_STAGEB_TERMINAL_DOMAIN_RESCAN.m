function OUT = AF7R57H_STABLE_STAGEB_TERMINAL_DOMAIN_RESCAN()
% AF7R57H_STABLE_STAGEB_TERMINAL_DOMAIN_RESCAN
% -----------------------------------------------------------------------------
% Re-map the old AF7R49B t1 diagnostic lattice with the corrected
% derivative-first Stage-B certificate.
%
% IMPORTANT:
%   * The old B49 t1 values are used ONLY as search locations.
%   * Old B49 success/failure is NOT reused as safety authority.
%   * QP candidates are replayed by the native shadow all_diag probe.
%   * Speed is certified by stable derivative-first Bezier bounds.
%   * A non-certified point is UNRESOLVED, never declared physically unsafe.
%   * Rejoin/geometry are printed only as diagnostics in this stage because
%     the old native core may short-circuit after its old speed gate.
%   * No production t1 is selected.
% -----------------------------------------------------------------------------

clc;
fprintf('\n====================================================================================================\n');
fprintf(' AF7R57H V2 | ACCEPTED-LEAF STABLE-STAGEB TERMINAL DOMAIN RESCAN\n');
fprintf('====================================================================================================\n');
fprintf('[AUTHORITY] B49GridOnly=1 | nativeCandidates=1 | stableDerivativeStageB=1 | sampledAcceptance=0 | productionModified=0\n');

OUT=struct();
OUT.schema='AF7R57H_STABLE_STAGEB_TERMINAL_DOMAIN_RESCAN_V2';
OUT.closed=false;
OUT.production_modified=false;
OUT.production_t1_selected=false;
OUT.continuous_terminal_boundary_proof=false;

%% A. Closed authorities
fprintf('\n[A. CLOSED AUTHORITIES]\n');

assert(evalin('base','exist(''replanner_instance'',''var'')==1'), ...
    '[AF7R57H] replanner_instance missing.');
replanner=evalin('base','replanner_instance');
assert(isa(replanner,'REPLANNING_SYSTEM'));

C=af7r57h_get_var_or_mat( ...
    'AF7R57C_SNAPSHOT_BOUND_SCHEMA_CAPTURE_OUT', ...
    'AF7R57C_SNAPSHOT_BOUND_SCHEMA_CAPTURE_OUT.mat');
assert(isstruct(C)&&isfield(C,'closed')&&logical(C.closed), ...
    '[AF7R57H] AF7R57C not closed.');

G=af7r57h_get_var_or_mat( ...
    'AF7R57G_NATIVE_WITNESS_CONSISTENCY_AUDIT_OUT', ...
    'AF7R57G_NATIVE_WITNESS_CONSISTENCY_AUDIT_OUT.mat');
assert(isstruct(G)&&isfield(G,'closed')&&logical(G.closed), ...
    '[AF7R57H] AF7R57G witness consistency not closed.');
assert(isfield(G,'diagnosis') && ...
    strcmp(char(G.diagnosis),'NUMERICAL_SPLIT_THEN_DIFFERENTIATE_WITNESS_ARTIFACT'), ...
    '[AF7R57H] R57G did not close the native witness artifact.');

[E,M,AUX,authoritySource]=AF7R52_LOAD_TRUE_TRIGGER_AUTHORITY(replanner); %#ok<ASGLU>
assert(isstruct(E)&&isfield(E,'valid')&&logical(E.valid));
assert(isstruct(M)&&isfield(M,'certified')&&logical(M.certified));
assert(isstruct(AUX)&&isfield(AUX,'B49')&&isstruct(AUX.B49), ...
    '[AF7R57H] AF7R52 snapshot has no B49 authority.');

B49=AUX.B49;
assert(isfield(B49,'t1')&&isfield(B49,'success')&&isfield(B49,'reason'));

TR=C.trace_records;
assert(iscell(TR)&&~isempty(TR));
pkt=TR{1}.pkt;
tU=double(TR{1}.tU);

fprintf(' AF7R57C closed                          = 1\n');
fprintf(' AF7R57G split-artifact closure         = 1\n');
fprintf(' AF7R52 authority source                = %s\n',char(string(authoritySource)));
fprintf(' fixed source-domain tU                 = %.17g\n',tU);

%% B. Shadow probe / nominal source
fprintf('\n[B. NATIVE CANDIDATE PROBE]\n');

assert(exist('af7r57g_trace_probe_mex','file')==3, ...
    ['[AF7R57H] af7r57g_trace_probe_mex missing. Run the closed AF7R57G V4 ' ...
     'audit first in this MATLAB session/path.']);

[F,expr,signature]=af7r57h_nominal_ast(replanner,M);
ok=af7r57g_trace_probe_mex(0,expr,signature);
assert(logical(ok),'[AF7R57H] shadow probe AST prime failed.');

fprintf(' shadow probe available                  = 1\n');
fprintf(' nominal signature                       = %s\n',signature);
fprintf(' packet nominal signature equal          = %d\n', ...
    strcmp(signature,char(pkt.nominal_signature)));
assert(strcmp(signature,char(pkt.nominal_signature)));

%% C. Old B49 lattice -- coordinates only
fprintf('\n[C. B49 SEARCH COORDINATES ONLY]\n');

tgrid=double(B49.t1(:).');
oldPass=logical(B49.success(:).');
oldReason=double(B49.reason(:).');

assert(numel(tgrid)==numel(oldPass)&&numel(oldPass)==numel(oldReason)&&~isempty(tgrid));
assert(all(diff(tgrid)>0),'[AF7R57H] B49 grid is not strictly increasing.');

fprintf(' B49 point count                         = %d\n',numel(tgrid));
fprintf(' B49 range                               = [%.17g %.17g]\n',tgrid(1),tgrid(end));
fprintf(' B49 pass/fail used as safety authority  = 0\n');

% Old terminal component, for comparison only.
d=diff([false oldPass false]);
obeg=find(d==1); ofin=find(d==-1)-1;
oterm=find(ofin==numel(oldPass),1,'last');
if isempty(oterm)
    oldLeft=NaN;
else
    oldLeft=tgrid(obeg(oterm));
end
fprintf(' old terminal-component left grid point  = %.17g\n',oldLeft);
fprintf(' known false boundary from R56B/R57G     = [%.17g %.17g]\n', ...
    double(TR{1}.lo),double(TR{1}.hi));

%% D. Stable Stage-B rescan
fprintf('\n[D. STABLE STAGE-B RESCAN]\n');
fprintf(' #   t1[s]           oldP oldR  cand  stableSafe unresolved  bestMargin      oldRej oldGeom\n');

N=numel(tgrid);
safe=false(1,N);
unresolved=false(1,N);
candCount=zeros(1,N);
bestMargin=-inf(1,N);
bestCand=nan(1,N);
oldRejoinAny=false(1,N);
oldGeomAny=false(1,N);
DETAIL=cell(1,N);

limit=af7r57h_limit(pkt);

for k=1:N
    t1=tgrid(k);
    R=af7r57g_trace_probe_mex(4,pkt,tU,t1);

    nc=double(R.all_candidate_count);
    candCount(k)=nc;

    rec=struct('t1',t1,'candidate_count',nc,'candidate',[]);
    if nc<1
        unresolved(k)=true;
        DETAIL{k}=rec;
        fprintf('%3d %16.9f   %d   %3g   %3d      0          1      %+ .6e      0      0\n', ...
            k,t1,oldPass(k),oldReason(k),nc,NaN);
        continue;
    end

    csafe=false(1,nc);
    cinc=false(1,nc);
    cmargin=-inf(1,nc);
    CC=cell(1,nc);

    if isfield(R,'all_candidate_rejoin_pass')
        oldRejoinAny(k)=any(logical(R.all_candidate_rejoin_pass));
    end
    if isfield(R,'all_candidate_geometry_pass')
        oldGeomAny(k)=any(logical(R.all_candidate_geometry_pass));
    end

    for c=1:nc
        cp=R.all_candidate_control_points{c};
        s0=double(R.all_candidate_segment_t0{c}(:).');
        s1=double(R.all_candidate_segment_t1{c}(:).');

        S=af7r57h_stable_speed_candidate(cp,s0,s1,pkt,F,limit);

        csafe(c)=logical(S.certified);
        cinc(c)=logical(S.unresolved);
        cmargin(c)=double(S.margin);
        CC{c}=S;
    end

    safe(k)=any(csafe);
    unresolved(k)=~safe(k) && (any(cinc)||all(~csafe));

    if any(isfinite(cmargin))
        [bestMargin(k),ic]=max(cmargin);
        bestCand(k)=ic;
    end

    rec.candidate=CC;
    rec.safe_candidates=find(csafe);
    rec.unresolved_candidates=find(cinc);
    DETAIL{k}=rec;

    fprintf('%3d %16.9f   %d   %3g   %3d      %d          %d      %+ .6e      %d      %d\n', ...
        k,t1,oldPass(k),oldReason(k),nc,safe(k),unresolved(k),bestMargin(k), ...
        oldRejoinAny(k),oldGeomAny(k));
end

%% E. Corrected SPEED-safe connected components
fprintf('\n[E. CORRECTED SPEED-SAFE TOPOLOGY]\n');

dd=diff([false safe false]);
beg=find(dd==1);
fin=find(dd==-1)-1;

fprintf(' stable-speed certified components       = %d\n',numel(beg));
for c=1:numel(beg)
    tail = fin(c)==N;
    fprintf('  component %d = [%.17g %.17g] | points=%d%s\n', ...
        c,tgrid(beg(c)),tgrid(fin(c)),fin(c)-beg(c)+1, ...
        af7r57h_if(tail,' | CONTAINS tU',''));
end

term=find(fin==N,1,'last');
terminalExists=~isempty(term);

if terminalExists
    kL=beg(term);
    newLeft=tgrid(kL);
    if kL>1
        leftProbe=tgrid(kL-1);
        leftProbeSafe=safe(kL-1);
        leftProbeUnresolved=unresolved(kL-1);
    else
        leftProbe=NaN;
        leftProbeSafe=false;
        leftProbeUnresolved=false;
    end
else
    kL=NaN;
    newLeft=NaN;
    leftProbe=NaN;
    leftProbeSafe=false;
    leftProbeUnresolved=false;
end

fprintf(' terminal speed component exists         = %d\n',terminalExists);
fprintf(' corrected terminal speed-left grid t1   = %.17g\n',newLeft);
fprintf(' point immediately below corrected comp  = %.17g\n',leftProbe);
fprintf(' below point speed-safe / unresolved     = %d / %d\n', ...
    leftProbeSafe,leftProbeUnresolved);

fakeBoundaryMoved = terminalExists && isfinite(oldLeft) && newLeft<oldLeft;
fprintf(' corrected left moved earlier than old   = %d\n',fakeBoundaryMoved);

%% F. Corrected boundary-neighborhood details
fprintf('\n[F. CORRECTED BOUNDARY-NEIGHBORHOOD DETAILS]\n');

if terminalExists
    idsDetail=unique(max(1,kL-1):min(N,kL+1),'stable');
    for kk=idsDetail
        fprintf('\n t1 = %.17g | stableSafe=%d | unresolved=%d | candidates=%d\n', ...
            tgrid(kk),safe(kk),unresolved(kk),candCount(kk));

        rec=DETAIL{kk};
        if isstruct(rec) && isfield(rec,'candidate') && ~isempty(rec.candidate)
            for cc=1:numel(rec.candidate)
                q=rec.candidate{cc};
                fprintf(['  cand %d | cert=%d unresolved=%d | margin=%+.17g | ' ...
                         'maxVQ=%.17g | minSigma=%.17g | leaves=%d accepted=%d ' ...
                         'splits=%d coarseRefined=%d | %s\n'], ...
                    cc,logical(q.certified),logical(q.unresolved),double(q.margin), ...
                    double(q.max_vq),double(q.min_sigma),double(q.leaf_count), ...
                    double(q.accepted_leaf_count),double(q.split_count), ...
                    double(q.coarse_inconclusive_count),char(q.reason));
            end
        end
    end
end

%% G. Formal scope / next theorem
fprintf('\n[G. FORMAL SCOPE]\n');

% This rescan does NOT prove the production boundary.  It proves only that
% the old grid topology must be replaced for the speed gate and supplies a
% new left-neighborhood search bracket.
rescanClosed = terminalExists && safe(end);

if ~rescanClosed
    next='FAIL_CLOSED__NO_STABLE_SPEED_COMPONENT_CONTAINING_tU';
elseif kL==1
    next='EXTEND_SOURCE_DERIVED_LEFT_DOMAIN_OR_PROVE_HOLDEND_BOUNDARY';
else
    next='BUILD_INTERVAL_BOUNDARY_THEOREM_AROUND_CORRECTED_SPEED_LEFT_NEIGHBORHOOD';
end

fprintf(' stable-speed domain rescan closed       = %d\n',rescanClosed);
fprintf(' full rejoin/geometry boundary proved    = 0\n');
fprintf(' continuous terminal-boundary proof      = 0\n');
fprintf(' production t1 selected                  = 0\n');
fprintf(' production source modified              = 0\n');
fprintf(' next action                             = %s\n',next);
fprintf('====================================================================================================\n');

OUT.closed=rescanClosed;
OUT.t1=tgrid;
OUT.old_success=oldPass;
OUT.old_reason=oldReason;
OUT.stable_speed_safe=safe;
OUT.unresolved=unresolved;
OUT.candidate_count=candCount;
OUT.best_speed_margin=bestMargin;
OUT.best_candidate=bestCand;
OUT.old_rejoin_any_diagnostic=oldRejoinAny;
OUT.old_geometry_any_diagnostic=oldGeomAny;
OUT.detail=DETAIL;
OUT.component_begin_index=beg;
OUT.component_end_index=fin;
OUT.terminal_component_index=term;
OUT.corrected_terminal_speed_left_grid_t1=newLeft;
OUT.corrected_left_predecessor_t1=leftProbe;
OUT.old_terminal_left_grid_t1=oldLeft;
OUT.false_boundary_moved_earlier=fakeBoundaryMoved;
OUT.full_rejoin_geometry_boundary_proved=false;
OUT.continuous_terminal_boundary_proof=false;
OUT.production_t1_selected=false;
OUT.production_modified=false;
OUT.next_action=next;

assignin('base','AF7R57H_STABLE_STAGEB_TERMINAL_DOMAIN_RESCAN_OUT',OUT);
save('AF7R57H_STABLE_STAGEB_TERMINAL_DOMAIN_RESCAN_OUT.mat','OUT','-v7.3');
end


function S=af7r57h_stable_speed_candidate(CP,seg0,seg1,pkt,F,limit)
% Continuous sufficient certificate using parent-segment derivatives first.
% No split-then-differentiate operation appears here.

S=struct('certified',false,'unresolved',false,'margin',-Inf, ...
    'max_vq',Inf,'min_sigma',-Inf,'leaf_count',0,'accepted_leaf_count',0, ...
    'split_count',0,'coarse_inconclusive_count',0, ...
    'repr_fail',false,'reason','');

assert(iscell(CP)&&numel(CP)==numel(seg0)&&numel(seg0)==numel(seg1));

% IMPORTANT: aggregate ONLY terminal accepted leaves.  A coarse parent leaf
% with VQ=Inf or VQ>=limit is an inconclusive enclosure that is refined; it
% must never contaminate the final certified VQ aggregate.
acceptedMaxVQ=0;
acceptedMinSigma=Inf;
leafCount=0;
acceptedLeafCount=0;
splitCount=0;
coarseInconclusiveCount=0;

for is=1:numel(CP)
    P=double(CP{is});
    T=double(seg1(is)-seg0(is));
    if ~(size(P,1)==3&&size(P,2)==16&&isfinite(T)&&T>0)
        S.unresolved=true;S.reason='INVALID_PARENT_SEGMENT';return;
    end

    D1=af7r57h_deriv_cp(P,T,1);
    D2=af7r57h_deriv_cp(P,T,2);
    D3=af7r57h_deriv_cp(P,T,3);

    % Stack rows: [u0 u1]. Start from whole normalized segment.
    stack=[0 1];

    while ~isempty(stack)
        u0=stack(end,1);u1=stack(end,2);stack(end,:)=[];
        leafCount=leafCount+1;

        C1=af7r57h_restrict(D1,u0,u1);
        C2=af7r57h_restrict(D2,u0,u1);
        C3=af7r57h_restrict(D3,u0,u1);

        ta=seg0(is)+T*u0;
        tb=seg0(is)+T*u1;
        tc=0.5*(ta+tb);
        h=0.5*(tb-ta);

        if ~(isfinite(tc)&&tc>=ta&&tc<=tb)
            S.unresolved=true;S.repr_fail=true;S.reason='TIME_REPRESENTABILITY';return;
        end

        tau=max(0,tc-double(pkt.flight_t0));
        v0=double(F{2}(tau));v0=v0(:);
        a0=double(F{3}(tau));a0=a0(:);

        vl=max(vecnorm(C1+v0,2,1))+double(pkt.base_payload_accel)*h;
        jl=double(pkt.base_payload_jerk)+max(vecnorm(C3,2,1));
        d2=af7r57h_decasteljau(C2,0.5);
        sigma=norm(a0+d2+[0;0;double(pkt.gravity)])-jl*h;

        if isfinite(sigma)&&sigma>0
            qdot=jl/sigma;
            vq=vl+double(pkt.cable_length)*qdot;
        else
            vq=Inf;
        end

        if isfinite(vq)&&vq<limit&&isfinite(sigma)&&sigma>0
            acceptedMaxVQ=max(acceptedMaxVQ,vq);
            acceptedMinSigma=min(acceptedMinSigma,sigma);
            acceptedLeafCount=acceptedLeafCount+1;
            continue;
        end

        % This coarse box is inconclusive only.  Do not aggregate its bound
        % into the eventual accepted-leaf certificate.
        coarseInconclusiveCount=coarseInconclusiveCount+1;

        % Fail closed but refine if binary64 time subdivision is meaningful.
        um=0.5*(u0+u1);
        tm=seg0(is)+T*um;

        if ~(isfinite(um)&&um>u0&&um<u1&&isfinite(tm)&&tm>ta&&tm<tb)
            S.unresolved=true;
            S.repr_fail=true;
            S.reason='UNRESOLVED_AT_BINARY64_REPRESENTABILITY';
            S.max_vq=acceptedMaxVQ;S.min_sigma=acceptedMinSigma;
            S.leaf_count=leafCount;S.accepted_leaf_count=acceptedLeafCount;
            S.split_count=splitCount;S.coarse_inconclusive_count=coarseInconclusiveCount;
            if acceptedLeafCount>0
                S.margin=limit-acceptedMaxVQ;
            else
                S.margin=-Inf;
            end;
            return;
        end

        % Computational ceiling is NOT an acceptance tolerance. Exceeding it
        % yields unresolved/fail-closed only.
        if splitCount>=200000
            S.unresolved=true;
            S.reason='OFFLINE_RESCAN_SPLIT_BUDGET_EXHAUSTED_FAIL_CLOSED';
            S.max_vq=acceptedMaxVQ;S.min_sigma=acceptedMinSigma;
            S.leaf_count=leafCount;S.accepted_leaf_count=acceptedLeafCount;
            S.split_count=splitCount;S.coarse_inconclusive_count=coarseInconclusiveCount;
            if acceptedLeafCount>0
                S.margin=limit-acceptedMaxVQ;
            else
                S.margin=-Inf;
            end;
            return;
        end

        stack(end+1,:)=[um u1]; %#ok<AGROW>
        stack(end+1,:)=[u0 um]; %#ok<AGROW>
        splitCount=splitCount+1;
    end
end

S.max_vq=acceptedMaxVQ;
S.min_sigma=acceptedMinSigma;
S.leaf_count=leafCount;
S.accepted_leaf_count=acceptedLeafCount;
S.split_count=splitCount;
S.coarse_inconclusive_count=coarseInconclusiveCount;
S.margin=limit-acceptedMaxVQ;

if acceptedLeafCount>0 && isfinite(acceptedMaxVQ) && ...
        isfinite(acceptedMinSigma) && acceptedMinSigma>0 && S.margin>0
    S.certified=true;
    S.unresolved=false;
    S.reason='STABLE_DERIVATIVE_FIRST_CONTINUOUS_SPEED_CERTIFIED';
else
    S.certified=false;
    S.unresolved=true;
    S.reason='FINAL_ACCEPTED_LEAF_AGGREGATE_NOT_STRICTLY_CERTIFIED';
end
end


function lim=af7r57h_limit(pkt)
rho=max([double(pkt.r_drone), ...
         double(pkt.cable_length)+double(pkt.r_load), ...
         double(pkt.cable_length)+double(pkt.r_cable)]);
lim=(double(pkt.sensor_range)-rho)/(double(pkt.horizon)+double(pkt.dt));
assert(isfinite(lim)&&lim>0);
end


function D=af7r57h_deriv_cp(P,T,order)
D=double(P);
deg=size(D,2)-1;
for r=1:order
    D=(D(:,2:end)-D(:,1:end-1))*(deg/T);
    deg=deg-1;
end
end


function y=af7r57h_decasteljau(P,u)
D=double(P);
while size(D,2)>1
    D=D(:,1:end-1)*(1-u)+D(:,2:end)*u;
end
y=D(:,1);
end


function [L,R]=af7r57h_split(P,u)
T=double(P);m=size(T,2);
L=zeros(size(T));R=zeros(size(T));
L(:,1)=T(:,1);R(:,m)=T(:,m);
for k=2:m
    U=T(:,1:end-1)*(1-u)+T(:,2:end)*u;
    T=U;
    L(:,k)=T(:,1);
    R(:,m-k+1)=T(:,end);
end
end


function Q=af7r57h_restrict(P,u0,u1)
if u0==0&&u1==1,Q=P;return;end
[L,~]=af7r57h_split(P,u1);
if u0==0,Q=L;return;end
alpha=u0/u1;
[~,Q]=af7r57h_split(L,alpha);
end


function [F,expr,signature]=af7r57h_nominal_ast(replanner,M)
F=replanner.nominal_flat_derivative_fn(1:7);
assert(iscell(F)&&numel(F)>=7&&all(cellfun(@(f)isa(f,'function_handle'),F)));

S=cellfun(@func2str,F,'UniformOutput',false);
signature=af7r57h_sha256_text(strjoin(S,char(10)));
assert(strcmp(signature,char(M.nominal_signature)), ...
    '[AF7R57H] nominal source differs from snapshot authority.');

expr=cell(21,1);kk=0;
for r=1:7
    ss=strtrim(S{r});
    tok=regexp(ss,'^@\(([A-Za-z]\w*)\)(.*)$','tokens','once');
    assert(~isempty(tok)&&numel(tok)==2);
    body=strtrim(char(tok{2}));
    body=strrep(body,'.^','^');
    body=strrep(body,'.*','*');
    body=strrep(body,'./','/');
    z=str2sym(body);z=z(:);
    assert(numel(z)==3);
    t=sym('t','real');
    sv0=symvar(z);assert(numel(sv0)<=1);
    if ~isempty(sv0),z=subs(z,sv0(1),t);end
    sv=symvar(z);
    for j=1:numel(sv),assert(strcmp(char(sv(j)),'t'));end
    for j=1:3,kk=kk+1;expr{kk}=char(z(j));end
end
end


function A=af7r57h_get_var_or_mat(varName,fileName)
if evalin('base',sprintf('exist(''%s'',''var'')==1',varName))
    A=evalin('base',varName);return;
end

cand={which(fileName),fullfile(pwd,fileName), ...
      fullfile(fileparts(mfilename('fullpath')),fileName)};

u=getenv('USERPROFILE');
if ~isempty(u)
    cand{end+1}=fullfile(u,'Documents','GitHub','common_matlab','reference',fileName); %#ok<AGROW>
end

p='';
for i=1:numel(cand)
    if ~isempty(cand{i})&&isfile(cand{i}),p=cand{i};break;end
end
assert(~isempty(p),'[AF7R57H] authority unavailable: %s',fileName);
S=load(p,'OUT');A=S.OUT;
end


function s=af7r57h_if(tf,a,b)
if tf,s=a;else,s=b;end
end


function h=af7r57h_sha256_text(s)
md=java.security.MessageDigest.getInstance('SHA-256');
md.update(uint8(unicode2native(char(s),'UTF-8')));
d=typecast(md.digest(),'uint8');
h=lower(reshape(dec2hex(d,2).',1,[]));
end
