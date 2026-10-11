%% Audit_CBF_SupportPlane_KKT_RuntimeSafe.m
% Runtime-safe symbolic audit for suspended-load CBF formulations.
%
% Goal
% ----
% Avoid repeating the very expensive direct symbolic inverse formulation.
% Compare only tractable candidates first:
%   B0 fixed-metric quadratic baseline
%   B1 single-system ellipsoid + separating/supporting hyperplane
%   B2 cable ellipsoid + separating/supporting hyperplane
%   B3 fixed UAV ellipsoid + separating/supporting hyperplane
%   B4 fixed load ellipsoid + separating/supporting hyperplane
%
% The obstacle remains an arbitrarily oriented 3-D ellipsoid.
%
% IMPORTANT
% ---------
% nH is treated as an external separating-hyperplane normal in this AUDIT.
% This tests symbolic complexity only.  A final safety-complete controller
% must add a valid update/dynamic-extension/optimization law for nH.
%
% Direct symbolic inverse barriers are intentionally NOT run by default.
% Previous logs already showed severe expression growth.

clearvars; clc;

%% ========================================================================
%% 0. SETTINGS -- conservative runtime guards
%% ========================================================================
MAX_RELATIVE_DEGREE = 6;
SOFT_STEP_LIMIT_SEC = 15;       % after a completed step > this, stop that model
TOTAL_MODEL_LIMIT_SEC = 75;     % do not launch a new step after this cumulative time
MAX_PHI_CHARS_BEFORE_LG = 2e6;  % pre-gate: do not launch Lg on a huge expression
MAX_PHI_CHARS_BEFORE_DRIFT = 8e5; % pre-gate before creating next drift derivative
RUN_HLC_SANITY_AUDIT = true;

RUN_B0_FIXED_METRIC = true;
RUN_B1_SINGLE_SUPPORT = true;
RUN_B2_CABLE_SUPPORT = true;
RUN_B3_UAV_SUPPORT = true;
RUN_B4_LOAD_SUPPORT = true;

% KKT / minimum-distance audits.  These do NOT expand inv(Jz) symbolically.
% They build the KKT residual and Jacobians, then benchmark the structure needed
% for the KKT-solution ODE  zdot = -Jz\(Jx*xdot).
RUN_K1_UAV_KKT = true;
RUN_K2_LOAD_KKT = true;
RUN_K3_SINGLE_KKT = true;
RUN_K4_CABLE_KKT = true;
KKT_MAX_EXPR_CHARS = 2e6;

OUT_DIR = fullfile(pwd,'generated_cbf_audit');
if ~exist(OUT_DIR,'dir'), mkdir(OUT_DIR); end
stamp = datestr(now,'yyyymmdd_HHMMSS');
LOG_FILE = fullfile(OUT_DIR,['CBF_support_kkt_audit_' stamp '.txt']);
MAT_FILE = fullfile(OUT_DIR,['CBF_support_kkt_audit_' stamp '.mat']);
CSV_FILE = fullfile(OUT_DIR,['CBF_support_kkt_audit_' stamp '.csv']);

diary off; diary(LOG_FILE); cleanupDiary=onCleanup(@() diary('off'));
LogMsg('============================================================');
LogMsg('CBF SUPPORT-PLANE + KKT RUNTIME-SAFE AUDIT START');
LogMsg('MATLAB version: %s',version);
LogMsg('maxRD=%d softStep=%.1fs totalModel=%.1fs',MAX_RELATIVE_DEGREE,SOFT_STEP_LIMIT_SEC,TOTAL_MODEL_LIMIT_SEC);
LogMsg('preGate: Lg chars<=%.0f, drift chars<=%.0f',MAX_PHI_CHARS_BEFORE_LG,MAX_PHI_CHARS_BEFORE_DRIFT);
LogMsg('Direct symbolic inverse formulations are disabled in this audit.');
LogMsg('Log file: %s',LOG_FILE);
LogMsg('============================================================');

%% ========================================================================
%% 1. SAME SYMBOLIC PLANT AS ORIGINAL HLC GENERATOR
%% ========================================================================
syms p1 p2 p3 dp1 dp2 dp3 ddp1 ddp2 ddp3 q0 q1 q2 q3 o1 o2 o3 real
syms u u1 u2 u3 u4 T1 T2 T3 T4 real
syms m Lx Ly lx ly jx jy jz gravity km1 km2 km3 km4 k1 k2 k3 k4 real
syms mL l Length cableL rotor_r real
syms pl1 pl2 pl3 dpl1 dpl2 dpl3 ol1 ol2 ol3 real
syms pT1 pT2 pT3 dstx dsty real
syms obj
syms xd1(t) xd2(t) xd3(t) xd4(t) v1(t)
syms t real

xd=[xd1(t),xd2(t),xd3(t),xd4(t)];
q=[q0;q1;q2;q3]; ob=[o1;o2;o3]; pl=[pl1;pl2;pl3]; dpl=[dpl1;dpl2;dpl3];
pT=[pT1;pT2;pT3]; ol=[ol1;ol2;ol3]; x=[q;ob;pl;dpl;pT;ol];
physicalParamFull=[m,Lx,Ly,lx,ly,jx,jy,jz,gravity,km1,km2,km3,km4,k1,k2,k3,k4,rotor_r,mL,cableL,dstx,dsty];

LogMsg('[MODEL] FLxyDst START'); ts=tic; f=FLxyDst(x,physicalParamFull); LogMsg('[MODEL] FLxyDst %.3f s',toc(ts));
LogMsg('[MODEL] GLxyDst START'); ts=tic; g=GLxyDst(x,physicalParamFull); LogMsg('[MODEL] GLxyDst %.3f s',toc(ts));

%% ========================================================================
%% 2. ORIGINAL FIRST-LAYER EMBEDDING
%% ========================================================================
h1=pl3-xd(3);
dh1=LieD(h1,f,x)+diff(h1,t);
alpha1=LieD(dh1,f,x)+diff(dh1,t);
beta1=simplify(LieD(dh1,g,x));
H=[1/beta1(1),-beta1(2)/beta1(1),-beta1(3)/beta1(1),-beta1(4)/beta1(1);zeros(3,1),eye(3)];
LogMsg('[HLC] FG simplify START'); ts=tic;
FG=simplify(f+g*H*[-alpha1+v1(t);u2;u3;u4]);
LogMsg('[HLC] FG simplify %.3f s chars=%d',toc(ts),symCharCount(FG));
LogMsg('[HLC] g1 simplify START'); ts=tic;
g1=simplify(MyCoeff(FG,[u2;u3;u4]));
LogMsg('[HLC] g1 simplify %.3f s chars=%d',toc(ts),symCharCount(g1));
f1=subs(FG,[u2,u3,u4],[0,0,0]);

if RUN_HLC_SANITY_AUDIT
    LogMsg('--- HLC SANITY ---');
    AuditSize('x',x,[19 1]); AuditSize('f1',f1,[19 1]); AuditSize('g1',g1,[19 3]);
    AuditNonzero('beta1(1)',beta1(1));
    AuditPrefixZero('load-x',pl1-xd(1),f1,g1,x,t,3);
    AuditPrefixZero('load-y',pl2-xd(2),f1,g1,x,t,3);
end

%% ========================================================================
%% 3. GEOMETRY
%% ========================================================================
% Arbitrarily oriented obstacle ellipsoid.
syms obs_x obs_y obs_z obs_a obs_b obs_c real
syms obs_roll obs_pitch obs_yaw real
cObs=[obs_x;obs_y;obs_z];
RObs=RotZYX(obs_roll,obs_pitch,obs_yaw);
SObs=RObs*diag([obs_a^2,obs_b^2,obs_c^2])*RObs.';

% External separating hyperplane normal.  Final controller must define its
% update law / dynamic extension.
syms nh1 nh2 nh3 real
nH=[nh1;nh2;nh3];

% System centers.
pU=pl-cableL*pT;
pMid=pl-(cableL/2)*pT;

% Single system ellipsoid: axisymmetric along cable direction.
syms single_a single_b positive
SSingle=single_b^2*eye(3)+(single_a^2-single_b^2)*(pT*pT.');

% Cable ellipsoid: axisymmetric along cable direction.
syms cable_a cable_b positive
SCable=cable_b^2*eye(3)+(cable_a^2-cable_b^2)*(pT*pT.');

% Fixed outer ellipsoid for UAV and fixed load ellipsoid.
syms uav_a uav_b uav_c load_a load_b load_c positive
SUav=diag([uav_a^2,uav_b^2,uav_c^2]);
SLoad=diag([load_a^2,load_b^2,load_c^2]);

% Fixed-metric baseline, supplied as an external constant symmetric matrix.
syms Q11 Q22 Q33 Q12 Q13 Q23 real
Qfix=[Q11 Q12 Q13;Q12 Q22 Q23;Q13 Q23 Q33];
dFix=pMid-cObs;
h_B0=dFix.'*Qfix*dFix-1;

% Support-plane signed separation margins.
h_B1=SupportPlaneBarrier(pMid,SSingle,cObs,SObs,nH);
h_B2=SupportPlaneBarrier(pMid,SCable,cObs,SObs,nH);
h_B3=SupportPlaneBarrier(pU,SUav,cObs,SObs,nH);
h_B4=SupportPlaneBarrier(pl,SLoad,cObs,SObs,nH);


%% ------------------------------------------------------------------------
%% KKT minimum-distance formulation (Thirugnanam-Zeng-Sreenath style audit)
%% ------------------------------------------------------------------------
% IMPORTANT:
%  * We do NOT symbolically form inv(Jz).
%  * At runtime the closest-point KKT solution z* is tracked and the ODE is
%       Jz * zdot = -Jx * xdot
%    solved numerically (8x8 linear solve for ellipsoid-vs-ellipsoid).
%  * This section audits expression sizes and whether Jx*g1 is structurally
%    zero/nonzero.  It is intentionally separated from the support-plane HOCBF.
%  * The KKT equations below assume the closest points lie on both ellipsoid
%    boundaries and the associated multipliers are positive. Runtime code must
%    monitor residual, multiplier sign, Jz conditioning, and solution validity.

syms ya1 ya2 ya3 yb1 yb2 yb3 lamA lamB real
yA=[ya1;ya2;ya3]; yB=[yb1;yb2;yb3];
zK=[yA;yB;lamA;lamB];
syms kkt_dsafe positive

% Obstacle inverse-shape metric is EXTERNAL. It is computed numerically from
% the arbitrary obstacle pose/axes, avoiding symbolic inversion of R*D*R'.
syms PB11 PB22 PB33 PB12 PB13 PB23 real
PB=[PB11 PB12 PB13;PB12 PB22 PB23;PB13 PB23 PB33];

% Fixed UAV/load inverse metrics are simple diagonal forms.
PUav=diag([1/uav_a^2,1/uav_b^2,1/uav_c^2]);
PLoad=diag([1/load_a^2,1/load_b^2,1/load_c^2]);

% Axisymmetric inverse metrics for single/cable ellipsoids. Under ||pT||=1,
% inv(b^2 I + (a^2-b^2)pT*pT') = (1/b^2)I + (1/a^2-1/b^2)pT*pT'.
% This avoids symbolic matrix inversion entirely.
PSingle=(1/single_b^2)*eye(3)+(1/single_a^2-1/single_b^2)*(pT*pT.');
PCable=(1/cable_b^2)*eye(3)+(1/cable_a^2-1/cable_b^2)*(pT*pT.');

K1=BuildEllipsoidKKT('K1 UAV/KKT',pU,PUav,cObs,PB,yA,yB,lamA,lamB,zK,x,g1,kkt_dsafe,KKT_MAX_EXPR_CHARS);
K2=BuildEllipsoidKKT('K2 load/KKT',pl,PLoad,cObs,PB,yA,yB,lamA,lamB,zK,x,g1,kkt_dsafe,KKT_MAX_EXPR_CHARS);
K3=BuildEllipsoidKKT('K3 single/KKT',pMid,PSingle,cObs,PB,yA,yB,lamA,lamB,zK,x,g1,kkt_dsafe,KKT_MAX_EXPR_CHARS);
K4=BuildEllipsoidKKT('K4 cable/KKT',pMid,PCable,cObs,PB,yA,yB,lamA,lamB,zK,x,g1,kkt_dsafe,KKT_MAX_EXPR_CHARS);

LogMsg('--- GEOMETRY EXPRESSION SIZES ---');
LogBarrierBuild('B0 fixed metric',h_B0);
LogBarrierBuild('B1 single support',h_B1);
LogBarrierBuild('B2 cable support',h_B2);
LogBarrierBuild('B3 UAV support',h_B3);
LogBarrierBuild('B4 load support',h_B4);
LogMsg('[NOTE] obstacle orientation remains arbitrary via Rz*Ry*Rx.');
LogMsg('[NOTE] nH is external in this audit; this is complexity evaluation, not yet the final safety proof.');

%% ========================================================================
%% 4. RUNTIME-SAFE AUDIT -- cheap candidates first
%% ========================================================================
opts.maxRD=MAX_RELATIVE_DEGREE;
opts.softStep=SOFT_STEP_LIMIT_SEC;
opts.totalLimit=TOTAL_MODEL_LIMIT_SEC;
opts.maxPhiBeforeLg=MAX_PHI_CHARS_BEFORE_LG;
opts.maxPhiBeforeDrift=MAX_PHI_CHARS_BEFORE_DRIFT;
results=struct();

if RUN_B0_FIXED_METRIC
    fprintf('\n=== B0 FIXED METRIC BASELINE ===\n');
    results.B0=AuditBarrierSafe('B0 fixed-metric',h_B0,f1,g1,x,t,opts);
end
if RUN_B3_UAV_SUPPORT
    fprintf('\n=== B3 FIXED UAV ELLIPSOID + SUPPORT PLANE ===\n');
    results.B3=AuditBarrierSafe('B3 UAV/support',h_B3,f1,g1,x,t,opts);
end
if RUN_B4_LOAD_SUPPORT
    fprintf('\n=== B4 FIXED LOAD ELLIPSOID + SUPPORT PLANE ===\n');
    results.B4=AuditBarrierSafe('B4 load/support',h_B4,f1,g1,x,t,opts);
end
if RUN_B1_SINGLE_SUPPORT
    fprintf('\n=== B1 SINGLE SYSTEM ELLIPSOID + SUPPORT PLANE ===\n');
    results.B1=AuditBarrierSafe('B1 single/support',h_B1,f1,g1,x,t,opts);
end
if RUN_B2_CABLE_SUPPORT
    fprintf('\n=== B2 CABLE ELLIPSOID + SUPPORT PLANE ===\n');
    results.B2=AuditBarrierSafe('B2 cable/support',h_B2,f1,g1,x,t,opts);
end


%% ========================================================================
%% 4B. KKT STRUCTURE / RUNTIME-COST AUDIT
%% ========================================================================
kktResults=struct();
if RUN_K1_UAV_KKT,    kktResults.K1=AuditKKTData(K1); end
if RUN_K2_LOAD_KKT,   kktResults.K2=AuditKKTData(K2); end
if RUN_K3_SINGLE_KKT, kktResults.K3=AuditKKTData(K3); end
if RUN_K4_CABLE_KKT,  kktResults.K4=AuditKKTData(K4); end

%% ========================================================================
%% 5. SUMMARY
%% ========================================================================
names=fieldnames(results); N=numel(names);
Formulation=strings(N,1); DetectedRD=nan(N,1); RDLB=nan(N,1); Status=strings(N,1);
TotalSec=nan(N,1); MaxStepSec=nan(N,1); LastPhiChars=nan(N,1);
for i=1:N
    r=results.(names{i}); Formulation(i)=string(r.name); DetectedRD(i)=r.detectedRD;
    RDLB(i)=r.relativeDegreeLowerBound; Status(i)=string(r.status); TotalSec(i)=r.totalSec;
    MaxStepSec(i)=r.maxStepSec; LastPhiChars(i)=r.lastPhiChars;
end
Summary=table(Formulation,DetectedRD,RDLB,Status,TotalSec,MaxStepSec,LastPhiChars);
fprintf('\n================ SUPPORT AUDIT SUMMARY ================\n'); disp(Summary);
writetable(Summary,CSV_FILE);

% KKT summary is printed separately because it is a structural audit rather
% than a direct relative-degree sweep.
kNames=fieldnames(kktResults);
if ~isempty(kNames)
    KN=numel(kNames); KKTName=strings(KN,1); FChars=nan(KN,1); JzChars=nan(KN,1); JxChars=nan(KN,1); JxgZero=false(KN,1); BuildSec=nan(KN,1);
    for ii=1:KN
        rr=kktResults.(kNames{ii}); KKTName(ii)=string(rr.name); FChars(ii)=rr.FChars; JzChars(ii)=rr.JzChars; JxChars(ii)=rr.JxChars; JxgZero(ii)=rr.JxgZero; BuildSec(ii)=rr.buildSec;
    end
    KKTSummary=table(KKTName,FChars,JzChars,JxChars,JxgZero,BuildSec);
    fprintf('\n================ KKT STRUCTURE SUMMARY ================\n'); disp(KKTSummary);
    writetable(KKTSummary,fullfile(OUT_DIR,['CBF_kkt_structure_' stamp '.csv']));
else
    KKTSummary=table();
end
save(MAT_FILE,'results','Summary','kktResults','KKTSummary','opts');
LogMsg('Saved MAT: %s',MAT_FILE); LogMsg('Saved CSV: %s',CSV_FILE); LogMsg('AUDIT END');

%% ========================================================================
%% LOCAL FUNCTIONS
%% ========================================================================

function data=BuildEllipsoidKKT(name,cA,PA,cB,PB,yA,yB,lamA,lamB,zK,x,g1,dsafe,maxChars)
    % Active-boundary KKT system for min 1/2||yA-yB||^2 between ellipsoids
    % (yA-cA)'PA(yA-cA)<=1 and (yB-cB)'PB(yB-cB)<=1.
    % Multipliers use the convention grad objective + lambda*grad constraint.
    ts=tic;
    r=yA-yB;
    eA=yA-cA; eB=yB-cB;
    gA=eA.'*PA*eA-1;
    gB=eB.'*PB*eB-1;
    F=[r + 2*lamA*PA*eA; -r + 2*lamB*PB*eB; gA; gB];
    hdist=r.'*r-dsafe^2;
    LogMsg('[KKT][%s] residual F built | chars=%d',name,symCharCount(F));
    if symCharCount(F)>maxChars
        LogMsg('[KKT][%s][WARN] F exceeds char gate; Jacobian build skipped.',name);
        data=struct('name',name,'F',F,'Jz',sym([]),'Jx',sym([]),'hz',jacobian(hdist,zK), ...
            'FChars',symCharCount(F),'JzChars',NaN,'JxChars',NaN,'JxgZero',false,'buildSec',toc(ts),'status','F_too_large');
        return;
    end
    tj=tic; Jz=jacobian(F,zK); tJz=toc(tj);
    LogMsg('[KKT][%s] Jz=%dx%d built %.3fs chars=%d',name,size(Jz,1),size(Jz,2),tJz,symCharCount(Jz));
    tj=tic; Jx=jacobian(F,x); tJx=toc(tj);
    LogMsg('[KKT][%s] Jx=%dx%d built %.3fs chars=%d',name,size(Jx,1),size(Jx,2),tJx,symCharCount(Jx));
    hz=jacobian(hdist,zK);
    % Do not solve Jz\\(Jx*g1) symbolically. Only detect whether plant inputs
    % enter the implicit KKT derivative RHS before the numerical 8x8 solve.
    tj=tic; Jxg=Jx*g1; tJxg=toc(tj); z=IsSymbolicZero(Jxg);
    LogMsg('[KKT][%s] Jx*g1 built %.3fs chars=%d zero=%d',name,tJxg,symCharCount(Jxg),z);
    data=struct('name',name,'F',F,'Jz',Jz,'Jx',Jx,'hz',hz,'Jxg',Jxg, ...
        'FChars',symCharCount(F),'JzChars',symCharCount(Jz),'JxChars',symCharCount(Jx), ...
        'JxgZero',z,'buildSec',toc(ts),'status','ok');
end

function out=AuditKKTData(data)
    fprintf('\n=== %s ===\n',data.name);
    LogMsg('[KKT][%s] status=%s build=%.3fs Fchars=%g Jzchars=%g Jxchars=%g JxgZero=%d', ...
        data.name,data.status,data.buildSec,data.FChars,data.JzChars,data.JxChars,data.JxgZero);
    if strcmp(data.status,'ok')
        LogMsg('[KKT][%s] Runtime rule: evaluate Jz,Jx numerically and solve Jz\\RHS; NEVER form inv(Jz).',data.name);
        LogMsg('[KKT][%s] Required runtime monitors: ||F||, min(lambda), cond/rcond(Jz), distance h, solve time.',data.name);
        LogMsg('[KKT][%s] If Jx*g1 is zero, control does not enter hdot through the KKT ODE at first derivative; higher-order/extended analysis is required.',data.name);
    end
    out=rmfield(data,intersect(fieldnames(data),{'F','Jz','Jx','hz','Jxg'}));
end

function h=SupportPlaneBarrier(cA,SA,cB,SB,n)
    % Signed separation margin along n. Scale-invariant if n is normalized;
    % final implementation should enforce/parameterize ||n|| consistently.
    h=n.'*(cB-cA)-sqrt(n.'*SA*n)-sqrt(n.'*SB*n);
end

function out=AuditBarrierSafe(name,h,f,g,x,t,opts)
    tModel=tic; phi=h; maxStep=0; detectedRD=NaN; lowerBound=0;
    status="maxRD_reached_without_input";
    history=struct('r',{},'LgSec',{},'LgZero',{},'DriftSec',{},'phiChars',{},'LgChars',{});
    LogMsg('[%s] AUDIT START',name); LogExpr(name,'h',h);
    for r=1:opts.maxRD
        elapsed=toc(tModel); phiChars=symCharCount(phi);
        if elapsed>opts.totalLimit
            status="stopped_total_time_before_next_step"; LogMsg('[STOP][%s] total %.1fs > %.1fs',name,elapsed,opts.totalLimit); break;
        end
        if phiChars>opts.maxPhiBeforeLg
            status="stopped_pre_gate_phi_before_Lg";
            LogMsg('[STOP][%s] phi_%d chars=%d > Lg pre-gate %.0f; rd>%d is established.',name,r-1,phiChars,opts.maxPhiBeforeLg,lowerBound); break;
        end

        LogMsg('[%s] r=%d Lg(phi_%d) START | phiChars=%d',name,r,r-1,phiChars); drawnow;
        ts=tic; Lgphi=LieD(phi,g,x); lgSec=toc(ts); maxStep=max(maxStep,lgSec);
        lgZero=IsSymbolicZero(Lgphi); lgChars=symCharCount(Lgphi);
        LogMsg('[%s] r=%d Lg DONE %.3fs | zero=%d | chars=%d',name,r,lgSec,lgZero,lgChars);
        rec.r=r; rec.LgSec=lgSec; rec.LgZero=lgZero; rec.DriftSec=NaN; rec.phiChars=phiChars; rec.LgChars=lgChars;
        if ~lgZero
            detectedRD=r; lowerBound=r; status="relative_degree_detected"; history(end+1)=rec; %#ok<AGROW>
            LogMsg('[PASS][%s] control appears at derivative %d.',name,r); break;
        end
        lowerBound=r;
        if lgSec>opts.softStep
            status="stopped_after_slow_Lg"; history(end+1)=rec; %#ok<AGROW>
            LogMsg('[STOP][%s] Lg %.1fs > %.1fs; proven rd>%d.',name,lgSec,opts.softStep,r); break;
        end
        if phiChars>opts.maxPhiBeforeDrift
            status="stopped_pre_gate_phi_before_drift"; history(end+1)=rec; %#ok<AGROW>
            LogMsg('[STOP][%s] phi chars=%d > drift pre-gate %.0f; next derivative skipped.',name,phiChars,opts.maxPhiBeforeDrift); break;
        end

        LogMsg('[%s] phi_%d=Dbar(phi_%d) START',name,r,r-1); drawnow;
        ts=tic; phiNext=Dbar(phi,f,x,t); driftSec=toc(ts); maxStep=max(maxStep,driftSec); rec.DriftSec=driftSec;
        history(end+1)=rec; %#ok<AGROW>
        nextChars=symCharCount(phiNext); LogMsg('[%s] phi_%d DONE %.3fs | chars=%d',name,r,driftSec,nextChars);
        phi=phiNext;
        if driftSec>opts.softStep
            status="stopped_after_slow_drift"; LogMsg('[STOP][%s] drift %.1fs > %.1fs; proven rd>%d.',name,driftSec,opts.softStep,r); break;
        end
    end
    out=struct('name',name,'detectedRD',detectedRD,'relativeDegreeLowerBound',lowerBound,'status',status,...
        'totalSec',toc(tModel),'maxStepSec',maxStep,'lastPhiChars',symCharCount(phi),'history',history);
    LogMsg('[%s] AUDIT END status=%s detectedRD=%s rdLB=%d total=%.3fs maxStep=%.3fs',name,char(status),num2str(detectedRD),lowerBound,out.totalSec,maxStep);
end

function LogBarrierBuild(name,h), LogMsg('[GEOM] %-22s chars=%d symvars=%d',name,symCharCount(h),numel(symvar(h))); end
function AuditPrefixZero(name,h,f,g,x,t,nOrders)
    phi=h; for r=1:nOrders
        ts=tic; z=LieD(phi,g,x); dt=toc(ts); isz=IsSymbolicZero(z);
        LogMsg('[HLC-AUDIT] %s Lg(phi_%d): zero=%d time=%.3fs',name,r-1,isz,dt);
        if ~isz, return; end; phi=Dbar(phi,f,x,t);
    end
end
function LogExpr(name,label,expr), LogMsg('[STAT][%s] %s size=%s symvars=%d chars=%d',name,label,mat2str(size(expr)),numel(symvar(expr)),symCharCount(expr)); end
function tf=IsSymbolicZero(M)
    tf=true; for k=1:numel(M)
        try, z=isAlways(M(k)==0,'Unknown','false'); catch, z=isequal(M(k),sym(0)); end
        if ~z, tf=false; return; end
    end
end
function LogMsg(fmt,varargin), fprintf('[%s] ',datestr(now,'HH:MM:SS.FFF')); fprintf(fmt,varargin{:}); fprintf('\n'); drawnow; end
function AuditSize(name,value,expected), if isequal(size(value),expected), LogMsg('[PASS] size(%s)=%s',name,mat2str(size(value))); else, LogMsg('[ERROR] size(%s)=%s expected=%s',name,mat2str(size(value)),mat2str(expected)); end; end
function AuditNonzero(name,value), if IsSymbolicZero(value), LogMsg('[ERROR] %s is symbolically zero.',name); else, LogMsg('[PASS] %s is not symbolically zero.',name); end; end
function y=Dbar(h,f,x,t), y=LieD(h,f,x)+diff(h,t); end
function pd=pdiff(flist,vars), flist=flist(:); vars=vars(:); pdiff_tmp=@(flist,vars) arrayfun(@(func) arrayfun(@(xx) diff(func,xx),vars)',flist,'UniformOutput',false); pd=cell2sym(pdiff_tmp(flist,vars)); end
function dh=LieD(h,f,x), x=x(:); dh=pdiff(h,x)*f; end
function mat=MyCoeff(M,vars)
    vars=vars(:); mat=coder.nullcopy(sym(zeros(length(M),length(vars))));
    for i=1:length(M), for j=1:length(vars), mat(i,j)=subs(M(i)-subs(M(i),vars(j),0),vars(j),1); end, end
end
function R=RotZYX(roll,pitch,yaw)
    cr=cos(roll); sr=sin(roll); cp=cos(pitch); sp=sin(pitch); cy=cos(yaw); sy=sin(yaw);
    Rx=[1 0 0;0 cr -sr;0 sr cr]; Ry=[cp 0 sp;0 1 0;-sp 0 cp]; Rz=[cy -sy 0;sy cy 0;0 0 1]; R=Rz*Ry*Rx;
end
function n=symCharCount(expr), try, n=sum(double(strlength(string(expr))),'all'); catch, try, n=numel(char(expr)); catch, n=-1; end, end; end
