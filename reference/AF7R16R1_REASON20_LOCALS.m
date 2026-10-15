function OUTD = AF7R16R1_REASON20_LOCALS()
% Run only at reason 20 inside builtin_bezier_d15_c7_proposal.
% Replays the corridor construction read-only with detailed stage diagnostics.

P=evalin('caller','OUT');
CTX=evalin('caller','CTX');
holdEnd=evalin('caller','holdEnd');
corridor=evalin('caller','corridor');
wReturned=evalin('caller','w');
tailDiag=evalin('caller','tailDiag');

O=CTX.obstacles;
LIV=CTX.live;
M=double(O.count);

fprintf('\n');
fprintf('====================================================================================================\n');
fprintf(' AF7R16R1 | REAL-FLIGHT REASON-20 CORRIDOR-QP LOCAL DIAGNOSTIC\n');
fprintf('====================================================================================================\n');
fprintf(' obstacle count                          = %d\n',M);
fprintf(' t0 / tm / holdEnd [s]                  = %.12g / %.12g / %.12g\n', ...
    double(P.t0),double(P.tm),double(holdEnd));
fprintf(' returned w finite                       = %d\n', ...
    isnumeric(wReturned)&&numel(wReturned)==3&&all(isfinite(double(wReturned(:)))));
fprintf(' returned corridor certified             = %d\n',field_bool(corridor,'certified'));
fprintf(' nominal safe-tail certified             = %d\n',field_bool(tailDiag,'certified'));

% Nominal path progress, exactly matching production logic.
x0=double(P.base_flat_eval_fn(double(P.tm))); x0=x0(:);
xh=double(P.base_flat_eval_fn(double(holdEnd))); xh=xh(:);
progress=nan(3,1);
if numel(x0)>=7 && numel(xh)>=7
    progress=xh(1:3)-x0(1:3);
end
if all(isfinite(progress)) && norm(progress)<=128*eps(max(1,norm(x0(1:3))))
    xm=double(P.base_flat_eval_fn(0.5*(double(P.tm)+double(holdEnd))));
    xm=xm(:);
    if numel(xm)>=7
        progress=xm(5:7);
    end
end
if all(isfinite(progress)) && norm(progress)<=128*eps(max(1,norm(progress))) && ...
        isfield(CTX.request,'trigger_pL_collision') && ...
        isfield(CTX.request,'pL_current')
    progress=double(CTX.request.trigger_pL_collision(:))- ...
        double(CTX.request.pL_current(:));
end

fprintf('\n[A. PATH PROGRESS USED FOR LATERAL SUBSPACE]\n');
fprintf(' progress                                = [% .12g % .12g % .12g]\n',progress);
fprintf(' ||progress||                            = %.12g\n',norm(progress));

% Active speed certificate from the actual stopped REPLANNING_SYSTEM
% object.  No base-workspace variable-name assumption is allowed.
obj=evalin('caller','obj');
assert(~isempty(obj),'[AF7R16R1] stopped REPLANNING_SYSTEM object unavailable.');
SC=obj.active_speed_certificate;

isSetupPrime=logical(obj.setup_exact_do_prime_active);
isGeneratorPrime=logical(obj.generator_integration_prime_active);
fprintf(' setup_exact_do_prime_active             = %d\n',isSetupPrime);
fprintf(' generator_integration_prime_active      = %d\n',isGeneratorPrime);
fprintf(' REAL Flight request stop                = %d\n',~isSetupPrime && ~isGeneratorPrime);
assert(~isSetupPrime && ~isGeneratorPrime, ...
    '[AF7R16R1] prime/setup invocation was not supposed to reach this conditional breakpoint.');
VL=double(SC.payload_speed_UB_m_s);
VQ=double(SC.uav_speed_UB_m_s);
VC=max(VL,VQ);
track=0;
if isfield(P,'execution_tracking_inflation_m') && ...
        isfinite(double(P.execution_tracking_inflation_m))
    track=max(0,double(P.execution_tracking_inflation_m));
end

A=zeros(M,3);
b=zeros(M,1);
N=zeros(3,M);
bpAll=nan(M,1);
bmAll=nan(M,1);
okpAll=false(M,1);
okmAll=false(M,1);
choice=zeros(M,1);

fprintf('\n[B. PER-OBSTACLE CORRIDOR CONSTRUCTION]\n');
fprintf(' obs | ok+/ok- | b+ [m]      b- [m]      chosen | n\n');

stageOK=true;
firstFail='NONE';

for ii=1:M
    R=reshape(double(O.rotations_flat(:,ii)),3,3);
    a=double(O.base_axes(:,ii));
    if any(~isfinite([R(:);a])) || any(a<=0)
        stageOK=false;
        firstFail=sprintf('INVALID_OBSTACLE_GEOMETRY_%d',ii);
        break;
    end

    n0=lateral_normal(R,a,progress);
    if any(~isfinite(n0)) || norm(n0)<=0
        stageOK=false;
        firstFail=sprintf('LATERAL_NORMAL_FAILED_%d',ii);
        break;
    end

    [bp,okp,diagp]=required_projection(P,CTX,holdEnd,ii,n0,VL,VQ,VC,track);
    [bm,okm,diagm]=required_projection(P,CTX,holdEnd,ii,-n0,VL,VQ,VC,track);
    bpAll(ii)=bp; bmAll(ii)=bm;
    okpAll(ii)=okp; okmAll(ii)=okm;

    if ~(okp&&okm)
        stageOK=false;
        if ~okp
            firstFail=sprintf('PLUS_PROJECTION_FAILED_OBS_%d__%s',ii,diagp.reason);
        else
            firstFail=sprintf('MINUS_PROJECTION_FAILED_OBS_%d__%s',ii,diagm.reason);
        end
        fprintf('%4d | %d/%d    | %-11.4g %-11.4g FAILED\n',ii,okp,okm,bp,bm);
        break;
    end

    if max(0,bp)<=max(0,bm)
        n=n0; bi=bp; choice(ii)=+1;
    else
        n=-n0; bi=bm; choice(ii)=-1;
    end

    eta=64*eps(max(1,abs(bi)));
    bi=bi+eta;

    A(ii,:)=n(:).';
    b(ii)=bi;
    N(:,ii)=n(:);

    fprintf('%4d | %d/%d    | %-11.4g %-11.4g %+6d | [% .4g % .4g % .4g]\n', ...
        ii,okp,okm,bp,bm,choice(ii),n);
end

fprintf('\n[C. GLOBAL HALFSPACE SYSTEM]\n');
fprintf(' corridor rows constructed              = %d / %d\n',sum(vecnorm(A,2,2)>0),M);

if stageOK
    [wQ,active,okQ]=solve_qp3(A,b);
    fprintf(' QP feasible                             = %d\n',okQ);
    if okQ
        slack=A*wQ-b;
        fprintf(' QP w [m]                               = [% .12g % .12g % .12g]\n',wQ);
        fprintf(' ||w|| [m]                              = %.12g\n',norm(wQ));
        fprintf(' active set                             = [%s]\n',sprintf(' %d',active));
        fprintf(' min slack [m]                          = %.12g\n',min(slack));
    else
        wQ=nan(3,1);
        active=[];
        fprintf(' QP w                                   = INFEASIBLE/UNRESOLVED\n');
    end
else
    okQ=false; wQ=nan(3,1); active=[];
    fprintf(' QP not reached                         = 1\n');
end

fprintf('\n[D. LOCAL-SIGN CHOICE CONSISTENCY]\n');
fprintf(' first construction failure             = %s\n',firstFail);

% For modest M, exhaustively check all +/- normal signs. This is
% DIAGNOSTIC ONLY and never used as production safety authority.
globalSignFeasible=NaN;
bestAltNorm=NaN;
bestAltSigns=[];
if stageOK && M<=16
    bestAlt=inf;
    nmask=2^M;
    for mask=0:nmask-1
        AA=zeros(M,3);
        bb=zeros(M,1);
        valid=true;
        signs=zeros(M,1);
        for ii=1:M
            R=reshape(double(O.rotations_flat(:,ii)),3,3);
            a=double(O.base_axes(:,ii));
            n0=lateral_normal(R,a,progress);
            if bitget(mask,ii)
                n=n0; bi=bpAll(ii); signs(ii)=+1;
            else
                n=-n0; bi=bmAll(ii); signs(ii)=-1;
            end
            if ~isfinite(bi)
                valid=false; break;
            end
            bi=bi+64*eps(max(1,abs(bi)));
            AA(ii,:)=n(:).'; bb(ii)=bi;
        end
        if ~valid, continue; end
        [wc,~,okc]=solve_qp3(AA,bb);
        if okc && norm(wc)<bestAlt
            bestAlt=norm(wc);
            bestAltSigns=signs;
        end
    end
    globalSignFeasible=isfinite(bestAlt);
    if globalSignFeasible
        bestAltNorm=bestAlt;
    end
    fprintf(' any alternative sign assignment feasible = %d\n',globalSignFeasible);
    if globalSignFeasible
        fprintf(' best alternative sign ||w|| [m]       = %.12g\n',bestAltNorm);
        fprintf(' best alternative signs                = [%s]\n',sprintf(' %+d',bestAltSigns));
    end
else
    fprintf(' alternative sign enumeration           = skipped (M>16 or earlier failure)\n');
end

fprintf('\n[E. CLASSIFICATION]\n');
if ~stageOK
    classification='CORRIDOR_REQUIREMENT_CONSTRUCTION_FAILED_BEFORE_QP';
elseif ~okQ && isequal(globalSignFeasible,1)
    classification='LOCAL_PER_OBSTACLE_SIGN_CHOICE_MAKES_GLOBAL_QP_INFEASIBLE';
elseif ~okQ
    classification='SELECTED_LATERAL_NORMAL_FAMILY_HAS_NO_COMMON_TRANSLATION';
elseif okQ
    classification='REPLAY_QP_FEASIBLE__PRODUCTION_RETURN_PATH_MISMATCH_REVIEW';
else
    classification='UNRESOLVED';
end
fprintf(' classification                         = %s\n',classification);

fprintf('\n[F. THEORY INTERPRETATION]\n');
fprintf([' A per-obstacle choice between n and -n is a DISJUNCTIVE corridor choice.\n' ...
         ' Choosing each sign independently by its local translation cost is not\n' ...
         ' equivalent to minimizing the globally feasible translation. If the\n' ...
         ' diagnostic finds an alternative feasible sign assignment, the next\n' ...
         ' production formulation must solve the sign/corridor choice jointly,\n' ...
         ' not by a case-specific rule or multiplier.\n']);

OUTD=struct();
OUTD.schema='AF7R16R1_REAL_FLIGHT_REASON20_CORRIDOR_DIAG_V1';
OUTD.obstacle_count=M;
OUTD.progress=progress;
OUTD.bp=bpAll;
OUTD.bm=bmAll;
OUTD.local_choice=choice;
OUTD.A=A;
OUTD.b=b;
OUTD.stage_ok=stageOK;
OUTD.first_failure=firstFail;
OUTD.local_qp_ok=okQ;
OUTD.local_qp_w=wQ;
OUTD.local_qp_active=active;
OUTD.alternative_sign_feasible=globalSignFeasible;
OUTD.best_alternative_norm=bestAltNorm;
OUTD.best_alternative_signs=bestAltSigns;
OUTD.classification=classification;
assignin('base','AF7R16R1_REASON20_LOCALS_OUT',OUTD);

fprintf('====================================================================================================\n');
end

function n=lateral_normal(R,a,progress)
R=double(R); a=double(a(:)); progress=double(progress(:));
Q=R*diag(1./(a.^2))*R.';
Q=0.5*(Q+Q.');
if numel(progress)==3 && all(isfinite(progress)) && ...
        norm(progress)>128*eps(max(1,norm(progress)))
    v=progress/norm(progress);
    Pm=eye(3)-v*v.';
    Qp=0.5*(Pm*Q*Pm+(Pm*Q*Pm).');
    [V,D]=eig(Qp,'vector');
    [~,ix]=max(real(D));
    n=Pm*real(V(:,ix));
    if norm(n)<=128*eps(max(1,norm(Qp,'fro')))
        [V,D]=eig(Q,'vector');
        [~,ix]=max(real(D));
        n=real(V(:,ix));
    end
else
    [V,D]=eig(Q,'vector');
    [~,ix]=max(real(D));
    n=real(V(:,ix));
end
n=n/norm(n);
end

function [breq,ok,DG]=required_projection(P,CTX,holdEnd,obsIndex,n,VL,VQ,VC,track)
LIV=CTX.live; O=CTX.obstacles;
n=double(n(:)); nn=norm(n);
DG=struct('reason','NONE','cells',0,'max_req',-inf);
ok=false; breq=-inf;
if ~(numel(n)==3&&all(isfinite(n))&&isfinite(nn)&&nn>0)
    DG.reason='INVALID_NORMAL'; return;
end
n=n/nn;
c=double(O.centers(:,obsIndex));
R=reshape(double(O.rotations_flat(:,obsIndex)),3,3);
a=double(O.base_axes(:,obsIndex));
hObs=dot(n,c)+norm(a.*(R.'*n));

ta=double(P.tm); dt=double(LIV.dt);
if holdEnd<=ta+8*eps(max(1,ta))
    [req,valid,why]=cell_requirement(P,ta,0,LIV,hObs,n,VL,VQ,VC,track);
    DG.cells=1; DG.reason=why;
    if valid, breq=req; DG.max_req=req; ok=true; end
    return;
end

while ta<holdEnd-8*eps(max(1,holdEnd))
    tb=min(holdEnd,ta+dt);
    tc=0.5*(ta+tb); half=0.5*(tb-ta);
    [req,valid,why]=cell_requirement(P,tc,half,LIV,hObs,n,VL,VQ,VC,track);
    DG.cells=DG.cells+1;
    if ~valid
        DG.reason=why; return;
    end
    breq=max(breq,req);
    ta=tb;
end
DG.max_req=breq;
DG.reason='OK';
ok=isfinite(breq);
end

function [req,ok,why]=cell_requirement(P,tc,half,LIV,hObs,n,VL,VQ,VC,track)
xd=P.base_flat_eval_fn(double(tc));
xd=double(xd(:));
req=NaN; ok=false; why='UNKNOWN';
if numel(xd)<11 || any(~isfinite(xd(1:11)))
    why='BASE_FLAT_INVALID'; return;
end
pL=xd(1:3);
sf=xd(9:11)+[0;0;double(LIV.gravity)];
ns=norm(sf);
if ~(isfinite(ns)&&ns>0)
    why='SPECIFIC_FORCE_INVALID'; return;
end
q=-sf/ns;
pQ=pL-double(LIV.cable_length)*q;

rhoL=double(LIV.r_load)+double(LIV.d_margin)+track+VL*half;
rhoQ=double(LIV.r_drone)+double(LIV.d_margin)+track+VQ*half;
rhoC=double(LIV.r_cable)+double(LIV.d_margin)+track+VC*half;

reqL=hObs+rhoL-dot(n,pL);
reqQ=hObs+rhoQ-dot(n,pQ);
reqC=hObs+rhoC-min(dot(n,pQ),dot(n,pL));
req=max([reqL,reqQ,reqC]);
ok=isfinite(req);
if ok, why='OK'; else, why='NONFINITE_REQ'; end
end

function [w,active,ok]=solve_qp3(A,b)
A=double(A); b=double(b(:)); m=size(A,1);
w=nan(3,1); active=[]; ok=false;
if size(A,2)~=3 || numel(b)~=m || any(~isfinite(A(:))) || any(~isfinite(b))
    return;
end
tol0=256*eps(max([1;abs(b)]));
if all(-b>=-tol0)
    w=zeros(3,1); active=[]; ok=true; return;
end
best=inf; bestW=nan(3,1); bestS=[];
for kk=1:min(3,m)
    S=nchoosek(1:m,kk);
    for jj=1:size(S,1)
        ids=S(jj,:);
        AS=A(ids,:); bs=b(ids);
        G=AS*AS.';
        if rcond(G)<=1e3*eps(max(1,norm(G,'fro'))), continue; end
        lam=G\bs;
        tolLam=256*eps(max([1;abs(lam);abs(bs)]));
        if any(lam < -tolLam), continue; end
        wc=AS.'*lam;
        tolFeas=256*eps(max([1;abs(b);abs(A*wc)]));
        if any(A*wc < b-tolFeas), continue; end
        val=dot(wc,wc);
        if val<best
            best=val; bestW=wc; bestS=ids;
        end
    end
end
if isfinite(best)&&all(isfinite(bestW))
    w=bestW; active=bestS; ok=true;
end
end

function tf=field_bool(S,name)
tf=false;
try
    tf=isstruct(S)&&isfield(S,name)&&logical(S.(name));
catch
end
end
