function OUTD = AF7R16R2_REAL_FLIGHT_REASON20_LOCALS()
% Run only while stopped at reason 20 after ALL prime-done flags are true.

obj=evalin('caller','obj');
P=evalin('caller','OUT');
CTX=evalin('caller','CTX');
holdEnd=evalin('caller','holdEnd');
corridor=evalin('caller','corridor');
wReturned=evalin('caller','w');
tailDiag=evalin('caller','tailDiag');

flags=struct( ...
    'online_prime_done',logical(obj.online_prime_done), ...
    'setup_exact_do_prime_done',logical(obj.setup_exact_do_prime_done), ...
    'generator_integration_prime_done',logical(obj.generator_integration_prime_done), ...
    'generator_backend_kernel_prime_done',logical(obj.generator_backend_kernel_prime_done), ...
    'trigger_event_capture_prime_done',logical(obj.trigger_event_capture_prime_done), ...
    'trigger_event_count',double(obj.trigger_event_count), ...
    'setup_exact_do_prime_active',logical(obj.setup_exact_do_prime_active), ...
    'generator_integration_prime_active',logical(obj.generator_integration_prime_active));

trueFlight = flags.online_prime_done && flags.setup_exact_do_prime_done && ...
    flags.generator_integration_prime_done && ...
    flags.generator_backend_kernel_prime_done && ...
    flags.trigger_event_capture_prime_done && flags.trigger_event_count>=1 && ...
    ~flags.setup_exact_do_prime_active && ...
    ~flags.generator_integration_prime_active;

fprintf('\n====================================================================================================\n');
fprintf(' AF7R16R2 | TRUE FLIGHT REASON-20 LOCAL STATE\n');
fprintf('====================================================================================================\n');
fprintf(' request time t0 [s]                    = %.12g\n',double(P.t0));
fprintf(' collision time tm [s]                  = %.12g\n',double(P.tm));
fprintf(' holdEnd [s]                            = %.12g\n',double(holdEnd));
fprintf(' obstacle count                         = %d\n',double(CTX.obstacles.count));
fprintf(' returned w finite                      = %d\n', ...
    isnumeric(wReturned)&&numel(wReturned)==3&&all(isfinite(double(wReturned(:)))));
fprintf(' returned corridor certified            = %d\n',field_bool(corridor,'certified'));
fprintf(' nominal safe-tail certified             = %d\n',field_bool(tailDiag,'certified'));
fprintf('\n[A. PRIME/FIGHT AUTHORITY]\n');
fprintf(' online_prime_done                      = %d\n',flags.online_prime_done);
fprintf(' setup_exact_do_prime_done              = %d\n',flags.setup_exact_do_prime_done);
fprintf(' generator_integration_prime_done       = %d\n',flags.generator_integration_prime_done);
fprintf(' generator_backend_kernel_prime_done    = %d\n',flags.generator_backend_kernel_prime_done);
fprintf(' trigger_event_capture_prime_done       = %d\n',flags.trigger_event_capture_prime_done);
fprintf(' trigger_event_count                    = %.0f\n',flags.trigger_event_count);
fprintf(' setup_exact_do_prime_active            = %d\n',flags.setup_exact_do_prime_active);
fprintf(' generator_integration_prime_active     = %d\n',flags.generator_integration_prime_active);
fprintf(' TRUE Flight stop                       = %d\n',trueFlight);
assert(trueFlight,'[AF7R16R2] this is not a fully post-prime Flight invocation.');

% Current implementation details if present.
fprintf('\n[B. CORRIDOR RETURN]\n');
if isstruct(corridor)
    f=fieldnames(corridor);
    for k=1:numel(f)
        name=f{k};
        v=corridor.(name);
        if isnumeric(v) && isscalar(v)
            fprintf(' %-38s = %.12g\n',name,double(v));
        elseif islogical(v) && isscalar(v)
            fprintf(' %-38s = %d\n',name,v);
        elseif ischar(v) || (isstring(v)&&isscalar(v))
            fprintf(' %-38s = %s\n',name,char(v));
        end
    end
end

% Recompute the R6 corridor construction read-only from the actual Flight CTX.
O=CTX.obstacles;
M=double(O.count);
LIV=CTX.live;
SC=obj.active_speed_certificate;
obj.validate_active_speed_certificate(SC);

x0=double(P.base_flat_eval_fn(double(P.tm))); x0=x0(:);
xh=double(P.base_flat_eval_fn(double(holdEnd))); xh=xh(:);
progress=xh(1:3)-x0(1:3);
if norm(progress)<=128*eps(max(1,norm(x0(1:3))))
    xm=double(P.base_flat_eval_fn(0.5*(double(P.tm)+double(holdEnd)))); xm=xm(:);
    if numel(xm)>=7, progress=xm(5:7); end
end
if norm(progress)<=128*eps(max(1,norm(progress))) && ...
        isfield(CTX.request,'trigger_pL_collision') && ...
        isfield(CTX.request,'pL_current')
    progress=double(CTX.request.trigger_pL_collision(:))-double(CTX.request.pL_current(:));
end

fprintf('\n[C. ACTUAL FLIGHT PATH PROGRESS]\n');
fprintf(' progress                               = [% .12g % .12g % .12g]\n',progress);
fprintf(' ||progress||                           = %.12g\n',norm(progress));

VL=double(SC.payload_speed_UB_m_s);
VQ=double(SC.uav_speed_UB_m_s);
VC=max(VL,VQ);
track=0;
if isfield(P,'execution_tracking_inflation_m') && ...
        isfinite(double(P.execution_tracking_inflation_m))
    track=max(0,double(P.execution_tracking_inflation_m));
end

A=zeros(M,3); b=zeros(M,1); N=zeros(3,M);
bpAll=nan(M,1); bmAll=nan(M,1); choice=zeros(M,1);
okAll=true;

fprintf('\n[D. ACTUAL FLIGHT PER-OBSTACLE +/- CORRIDOR]\n');
fprintf(' obs | b+ [m]        b- [m]        local sign | n0\n');

for ii=1:M
    R=reshape(double(O.rotations_flat(:,ii)),3,3);
    a=double(O.base_axes(:,ii));
    n0=lateral_normal(R,a,progress);
    [bp,okp]=required_projection(P,CTX,holdEnd,ii,n0,VL,VQ,VC,track);
    [bm,okm]=required_projection(P,CTX,holdEnd,ii,-n0,VL,VQ,VC,track);
    bpAll(ii)=bp; bmAll(ii)=bm;
    if ~(okp&&okm), okAll=false; break; end

    if max(0,bp)<=max(0,bm)
        n=n0; bi=bp; choice(ii)=1;
    else
        n=-n0; bi=bm; choice(ii)=-1;
    end
    bi=bi+64*eps(max(1,abs(bi)));
    A(ii,:)=n(:).'; b(ii)=bi; N(:,ii)=n;
    fprintf('%4d | %-13.6g %-13.6g %+10d | [% .5g % .5g % .5g]\n', ...
        ii,bp,bm,choice(ii),n0);
end

fprintf('\n[E. R6 LOCAL-SIGN QP]\n');
if okAll
    [wLocal,~,okLocal]=solve_qp3(A,b);
else
    wLocal=nan(3,1); okLocal=false;
end
fprintf(' local-sign QP feasible                 = %d\n',okLocal);
if okLocal
    fprintf(' local-sign w [m]                       = [% .12g % .12g % .12g]\n',wLocal);
    fprintf(' ||w_local|| [m]                        = %.12g\n',norm(wLocal));
end

% Exact global disjunctive active-set replay, independent of production source.
[wGlobal,sGlobal,okGlobal,activeGlobal]=solve_disjunctive3(N_from_base(O,progress),bpAll,bmAll);

fprintf('\n[F. GLOBAL DISJUNCTIVE REPLAY]\n');
fprintf(' globally feasible                      = %d\n',okGlobal);
if okGlobal
    fprintf(' global w [m]                           = [% .12g % .12g % .12g]\n',wGlobal);
    fprintf(' ||w_global|| [m]                       = %.12g\n',norm(wGlobal));
    fprintf(' global signs                           = [%s]\n',sprintf(' %+d',sGlobal));
    fprintf(' active oriented ids                    = [%s]\n',sprintf(' %d',activeGlobal));
end

if ~okAll
    cls='CORRIDOR_REQUIREMENT_CONSTRUCTION_FAILED';
elseif ~okLocal && okGlobal
    cls='ACTUAL_FLIGHT_LOCAL_SIGN_GREEDY_CAUSES_FALSE_INFEASIBILITY';
elseif ~okLocal && ~okGlobal
    cls='ACTUAL_FLIGHT_FIXED_NORMAL_FAMILY_HAS_NO_COMMON_TRANSLATION';
elseif okLocal
    cls='ACTUAL_FLIGHT_REASON20_NOT_REPRODUCED_BY_R6_REPLAY';
else
    cls='UNRESOLVED';
end

fprintf('\n[G. CLASSIFICATION]\n');
fprintf(' classification                         = %s\n',cls);
fprintf('====================================================================================================\n');

OUTD=struct('schema','AF7R16R2_TRUE_FLIGHT_REASON20_V1', ...
    'true_flight',trueFlight,'t0',double(P.t0),'tm',double(P.tm), ...
    'hold_end',double(holdEnd),'obstacle_count',M,'progress',progress, ...
    'bp',bpAll,'bm',bmAll,'local_choice',choice, ...
    'local_qp_ok',okLocal,'local_w',wLocal, ...
    'global_qp_ok',okGlobal,'global_w',wGlobal, ...
    'global_signs',sGlobal,'classification',cls);
assignin('base','AF7R16R2_REAL_FLIGHT_REASON20_OUT',OUTD);
end

function N0=N_from_base(O,progress)
M=double(O.count);
N0=zeros(3,M);
for ii=1:M
    R=reshape(double(O.rotations_flat(:,ii)),3,3);
    a=double(O.base_axes(:,ii));
    N0(:,ii)=lateral_normal(R,a,progress);
end
end

function n=lateral_normal(R,a,progress)
Q=R*diag(1./(a(:).^2))*R.';
Q=0.5*(Q+Q.');
if norm(progress)>128*eps(max(1,norm(progress)))
    v=progress/norm(progress);
    Pm=eye(3)-v*v.';
    Qp=0.5*(Pm*Q*Pm+(Pm*Q*Pm).');
    [V,D]=eig(Qp,'vector'); [~,ix]=max(real(D));
    n=Pm*real(V(:,ix));
    if norm(n)<=128*eps(max(1,norm(Qp,'fro')))
        [V,D]=eig(Q,'vector'); [~,ix]=max(real(D)); n=real(V(:,ix));
    end
else
    [V,D]=eig(Q,'vector'); [~,ix]=max(real(D)); n=real(V(:,ix));
end
n=n/norm(n);
end

function [breq,ok]=required_projection(P,CTX,holdEnd,obsIndex,n,VL,VQ,VC,track)
LIV=CTX.live; O=CTX.obstacles;
n=n(:)/norm(n);
c=double(O.centers(:,obsIndex));
R=reshape(double(O.rotations_flat(:,obsIndex)),3,3);
a=double(O.base_axes(:,obsIndex));
hObs=dot(n,c)+norm(a.*(R.'*n));
breq=-inf; ok=false;
ta=double(P.tm); dt=double(LIV.dt);
while ta<holdEnd-8*eps(max(1,holdEnd))
    tb=min(holdEnd,ta+dt); tc=0.5*(ta+tb); half=0.5*(tb-ta);
    xd=double(P.base_flat_eval_fn(tc)); xd=xd(:);
    if numel(xd)<11 || any(~isfinite(xd(1:11))), return; end
    pL=xd(1:3); sf=xd(9:11)+[0;0;double(LIV.gravity)];
    if ~(norm(sf)>0), return; end
    q=-sf/norm(sf); pQ=pL-double(LIV.cable_length)*q;
    rhoL=double(LIV.r_load)+double(LIV.d_margin)+track+VL*half;
    rhoQ=double(LIV.r_drone)+double(LIV.d_margin)+track+VQ*half;
    rhoC=double(LIV.r_cable)+double(LIV.d_margin)+track+VC*half;
    req=max([hObs+rhoL-dot(n,pL), ...
             hObs+rhoQ-dot(n,pQ), ...
             hObs+rhoC-min(dot(n,pQ),dot(n,pL))]);
    if ~isfinite(req), return; end
    breq=max(breq,req); ta=tb;
end
ok=isfinite(breq);
end

function [w,active,ok]=solve_qp3(A,b)
m=size(A,1); w=nan(3,1); active=[]; ok=false;
tol0=256*eps(max([1;abs(b(:))]));
if all(-b>=-tol0), w=zeros(3,1); ok=true; return; end
best=inf;
for k=1:min(3,m)
    S=nchoosek(1:m,k);
    for j=1:size(S,1)
        ids=S(j,:); AS=A(ids,:); bs=b(ids);
        G=AS*AS.';
        if rcond(G)<=1e3*eps(max(1,norm(G,'fro'))), continue; end
        lam=G\bs;
        if any(lam < -256*eps(max([1;abs(lam);abs(bs)]))), continue; end
        wc=AS.'*lam;
        tol=256*eps(max([1;abs(b(:));abs(A*wc)]));
        if any(A*wc < b-tol), continue; end
        val=dot(wc,wc);
        if val<best, best=val; w=wc; active=ids; ok=true; end
    end
end
end

function [wBest,sBest,ok,activeBest]=solve_disjunctive3(N,bp,bm)
M=size(N,2);
AO=[N.';-N.']; bO=[bp(:);bm(:)];
obs=[(1:M)';(1:M)'];
wBest=nan(3,1); sBest=zeros(M,1); activeBest=[]; ok=false; best=inf;

[tf,s]=feas(zeros(3,1));
if tf, wBest=zeros(3,1); sBest=s; ok=true; return; end

for k=1:min(3,M)
    S=nchoosek(1:2*M,k);
    for j=1:size(S,1)
        ids=S(j,:);
        if numel(unique(obs(ids)))<k, continue; end
        A=AO(ids,:); b=bO(ids); G=A*A.';
        if rcond(G)<=1e3*eps(max(1,norm(G,'fro'))), continue; end
        lam=G\b;
        if any(lam < -256*eps(max([1;abs(lam);abs(b)]))), continue; end
        wc=A.'*lam;
        [tf,s]=feas(wc);
        if tf && dot(wc,wc)<best
            best=dot(wc,wc); wBest=wc; sBest=s; activeBest=ids; ok=true;
        end
    end
end

    function [tf,s]=feas(w)
        pr=N.'*w;
        tol=256*eps(max([ones(M,1),abs(pr),abs(bp(:)),abs(bm(:))],[],2));
        sp=pr>=bp(:)-tol; sm=-pr>=bm(:)-tol;
        tf=all(sp|sm); s=zeros(M,1);
        if ~tf, return; end
        for ii=1:M
            if sp(ii)&&~sm(ii), s(ii)=1;
            elseif sm(ii)&&~sp(ii), s(ii)=-1;
            elseif pr(ii)-bp(ii)>=-pr(ii)-bm(ii), s(ii)=1;
            else, s(ii)=-1;
            end
        end
    end
end

function tf=field_bool(S,name)
tf=false;
try
    tf=isstruct(S)&&isfield(S,name)&&logical(S.(name));
catch
end
end
