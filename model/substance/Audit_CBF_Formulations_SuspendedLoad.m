%% Audit_CBF_Formulations_SuspendedLoad.m
% CBF formulation complexity audit for the supplied suspended-load HLC.
%
% Purpose
% -------
% Compare several collision-barrier REPRESENTATIONS before committing to a
% full HOCBF generator.  This file intentionally does NOT generate the final
% controller.  It measures where symbolic Lie derivatives become expensive.
%
% Compared formulations (same HLC f1,g1):
%   F1  SPHERE / ellipsoid obstacle, current Minkowski-outer inverse form
%   F2  SINGLE ELLIPSOID, current Minkowski-outer inverse form
%   F3  SINGLE ELLIPSOID, separating/supporting-hyperplane form
%       (support normal n is an EXTERNAL PARAMETER in this audit)
%   F4  FIXED-METRIC quadratic ellipsoid barrier, h = dc'Qdc-1
%       (Q is an EXTERNAL symmetric parameter in this audit)
%
% IMPORTANT THEORY NOTES
% ----------------------
% F3 is a complexity audit inspired by supporting/separating-hyperplane CBF
% literature.  Treating n as externally supplied/constant during Lie
% differentiation is NOT yet the full dynamic-extension construction.
% A safety-complete implementation must specify how n evolves/is optimized.
%
% F4 is only a valid continuous-time CBF for the represented geometry when Q
% is truly constant with respect to the plant state, e.g. a fixed outer
% ellipsoid.  Freezing a state-dependent Q(pT) each sample would omit Qdot and
% is therefore NOT claimed here as an exact CBF.  F4 is included to isolate
% the symbolic cost of the inverse/shape dependence.
%
% The script auto-stops a formulation AFTER an expensive completed step.
% MATLAB Symbolic operations cannot in general be interrupted safely from
% within the same script, so no timer can abort an operation already running.
%
% Output
% ------
%   generated_cbf_audit/CBF_formulation_audit_YYYYMMDD_HHMMSS.txt
%   generated_cbf_audit/CBF_formulation_audit_YYYYMMDD_HHMMSS.mat
%   generated_cbf_audit/CBF_formulation_audit_YYYYMMDD_HHMMSS.csv
%
% Based on the user's supplied HLC generation structure:
%   x = [q; ob; pl; dpl; pT; ol]
%   FG = simplify(f + g*H*[-alpha1+v1;u2;u3;u4])
%   g1 = simplify(MyCoeff(FG,[u2;u3;u4]))
%   f1 = subs(FG,[u2,u3,u4],[0,0,0])

clearvars; clc;

%% ========================================================================
%% 0. SETTINGS
%% ========================================================================
MAX_RELATIVE_DEGREE = 6;
SOFT_STEP_LIMIT_SEC = 45;    % stop model after a completed step exceeds this
TOTAL_MODEL_LIMIT_SEC = 180; % stop model between steps after this cumulative time
GROWTH_LIMIT = 4.0;          % optional growth warning/stop factor
MIN_GROWTH_BASE_SEC = 5;     % only use growth rule if previous step >= this
RUN_HLC_SANITY_AUDIT = true;
RUN_GEOMETRY_SANITY_AUDIT = true;

RUN_F1_SPHERE_CURRENT       = true;
RUN_F2_SINGLE_CURRENT       = true;
RUN_F3_SINGLE_SUPPORT       = true;
RUN_F4_FIXED_METRIC         = true;

OUT_DIR = fullfile(pwd,'generated_cbf_audit');
if ~exist(OUT_DIR,'dir'), mkdir(OUT_DIR); end
stamp = datestr(now,'yyyymmdd_HHMMSS');
LOG_FILE = fullfile(OUT_DIR,['CBF_formulation_audit_' stamp '.txt']);
MAT_FILE = fullfile(OUT_DIR,['CBF_formulation_audit_' stamp '.mat']);
CSV_FILE = fullfile(OUT_DIR,['CBF_formulation_audit_' stamp '.csv']);

diary off;
diary(LOG_FILE);
cleanupDiary = onCleanup(@() diary('off'));

LogMsg('============================================================');
LogMsg('CBF FORMULATION AUDIT START');
LogMsg('MATLAB version: %s',version);
LogMsg('maxRD=%d, softStep=%.1fs, totalModel=%.1fs, growthLimit=%.2f', ...
    MAX_RELATIVE_DEGREE,SOFT_STEP_LIMIT_SEC,TOTAL_MODEL_LIMIT_SEC,GROWTH_LIMIT);
LogMsg('Log file: %s',LOG_FILE);
LogMsg('============================================================');

%% ========================================================================
%% 1. SYMBOLIC PLANT: SAME STATE/PARAMETER CONVENTION AS SUPPLIED GENERATOR
%% ========================================================================
syms p1 p2 p3 dp1 dp2 dp3 ddp1 ddp2 ddp3 q0 q1 q2 q3 o1 o2 o3 real
syms u u1 u2 u3 u4 T1 T2 T3 T4 real
syms m Lx Ly lx ly jx jy jz gravity km1 km2 km3 km4 k1 k2 k3 k4 real
syms mL l Length cableL rotor_r real
syms pl1 pl2 pl3 dpl1 dpl2 dpl3 ol1 ol2 ol3 real
syms pT1 pT2 pT3 real
syms dstx dsty real
syms obj
syms xd1(t) xd2(t) xd3(t) xd4(t) v1(t)
syms t real

xd = [xd1(t),xd2(t),xd3(t),xd4(t)];
p   = [p1;p2;p3]; %#ok<NASGU>
dp  = [dp1;dp2;dp3]; %#ok<NASGU>
ddp = [ddp1;ddp2;ddp3]; %#ok<NASGU>
q   = [q0;q1;q2;q3];
ob  = [o1;o2;o3];
pl  = [pl1;pl2;pl3];
dpl = [dpl1;dpl2;dpl3];
ol  = [ol1;ol2;ol3];
pT  = [pT1;pT2;pT3];
x   = [q;ob;pl;dpl;pT;ol];

physicalParamFull = [m,Lx,Ly,lx,ly,jx,jy,jz,gravity,km1,km2,km3,km4,k1,k2,k3,k4,rotor_r,mL,cableL,dstx,dsty];

LogMsg('[MODEL] Calling FLxyDst ...'); ts=tic;
f = FLxyDst(x,physicalParamFull);
LogMsg('[MODEL] FLxyDst done in %.3f s',toc(ts));
LogMsg('[MODEL] Calling GLxyDst ...'); ts=tic;
g = GLxyDst(x,physicalParamFull);
LogMsg('[MODEL] GLxyDst done in %.3f s',toc(ts));
LogMsg('[MODEL] size(x)=%s size(f)=%s size(g)=%s',mat2str(size(x)),mat2str(size(f)),mat2str(size(g)));

%% ========================================================================
%% 2. REBUILD ONLY HLC FIRST-LAYER EMBEDDED DYNAMICS f1,g1
%% ========================================================================
h1 = pl3-xd(3);
LogMsg('[HLC-1] dh1/alpha1/beta1 ...'); ts=tic;
dh1 = LieD(h1,f,x)+diff(h1,t);
alpha1 = LieD(dh1,f,x)+diff(dh1,t);
beta1 = simplify(LieD(dh1,g,x));
LogMsg('[HLC-1] first-layer derivatives done in %.3f s',toc(ts));

H = [1/beta1(1),-beta1(2)/beta1(1),-beta1(3)/beta1(1),-beta1(4)/beta1(1); zeros(3,1),eye(3)];
LogMsg('[HLC-1] FG simplify START'); ts=tic;
FG = simplify(f+g*H*[-alpha1+v1(t);u2;u3;u4]);
LogMsg('[HLC-1] FG simplify done in %.3f s | chars=%d',toc(ts),symCharCount(FG));
LogMsg('[HLC-1] g1 simplify START'); ts=tic;
g1 = simplify(MyCoeff(FG,[u2;u3;u4]));
LogMsg('[HLC-1] g1 simplify done in %.3f s | chars=%d',toc(ts),symCharCount(g1));
f1 = subs(FG,[u2,u3,u4],[0,0,0]);
LogMsg('[HLC-1] f1 formed. size(f1)=%s size(g1)=%s',mat2str(size(f1)),mat2str(size(g1)));

if RUN_HLC_SANITY_AUDIT
    LogMsg('--- HLC SANITY AUDIT START ---');
    AuditSize('x',x,[19 1]);
    AuditSize('f1',f1,[19 1]);
    AuditSize('g1',g1,[19 3]);
    AuditSize('beta1',beta1,[1 4]);
    AuditNonzero('beta1(1)',beta1(1));

    % Cheap known-output checks only; do not rebuild alpha2/beta2 here.
    h2 = pl1-xd(1); h3 = pl2-xd(2);
    LogMsg('[HLC-AUDIT] Cheap prefix test for load x/y: first 3 input Lie terms should be zero.');
    AuditPrefixZero('load-x',h2,f1,g1,x,t,3);
    AuditPrefixZero('load-y',h3,f1,g1,x,t,3);
    LogMsg('--- HLC SANITY AUDIT END ---');
end

%% ========================================================================
%% 3. OBSTACLE + SYSTEM GEOMETRY
%% ========================================================================
syms obs_x obs_y obs_z obs_a obs_b obs_c real
syms obs_roll obs_pitch obs_yaw real
syms betaM positive
cObs = [obs_x;obs_y;obs_z];
RObs = RotZYX(obs_roll,obs_pitch,obs_yaw);
SObs = RObs*diag([obs_a^2,obs_b^2,obs_c^2])*RObs.';

% system centers
pU   = pl-cableL*pT;
pMid = pl-(cableL/2)*pT;

% Generic sphere along cable
syms lambdaSphere real
syms sphere_r positive
cSphere = pl-lambdaSphere*cableL*pT;
SSphere = sphere_r^2*eye(3);

% Single axisymmetric system ellipsoid along cable direction
syms single_a single_b positive
SSingle = single_b^2*eye(3)+(single_a^2-single_b^2)*(pT*pT.');
cSingle = pMid;

% Supporting/separating hyperplane normal (external audit parameter)
syms nh1 nh2 nh3 real
nH = [nh1;nh2;nh3];

% Fixed symmetric collision metric (external audit parameter)
syms Q11 Q22 Q33 Q12 Q13 Q23 real
Qfix = [Q11 Q12 Q13; Q12 Q22 Q23; Q13 Q23 Q33];

LogMsg('[GEOM] Building four audit barriers ...');

% F1: current inverse form for sphere vs arbitrary ellipsoid obstacle
ts=tic;
h_F1 = EllipsoidSeparationBarrier(cSphere,SSphere,cObs,SObs,betaM);
LogMsg('[GEOM][F1] sphere/current-inverse built %.3f s | chars=%d',toc(ts),symCharCount(h_F1));

% F2: current inverse form for pT-dependent single ellipsoid
ts=tic;
h_F2 = EllipsoidSeparationBarrier(cSingle,SSingle,cObs,SObs,betaM);
LogMsg('[GEOM][F2] single/current-inverse built %.3f s | chars=%d',toc(ts),symCharCount(h_F2));

% F3: supporting/separating-hyperplane form.
% Positive means the chosen normal separates the shapes with positive margin.
% Support radius of E(c,S) along n is sqrt(n' S n).
ts=tic;
h_F3 = nH.'*(cObs-cSingle) - sqrt(nH.'*SSingle*nH) - sqrt(nH.'*SObs*nH);
LogMsg('[GEOM][F3] single/support-plane built %.3f s | chars=%d',toc(ts),symCharCount(h_F3));

% F4: fixed metric benchmark. Qfix is assumed supplied externally and constant
% w.r.t. x for this audit.
dcFix = cSingle-cObs;
ts=tic;
h_F4 = dcFix.'*Qfix*dcFix - 1;
LogMsg('[GEOM][F4] fixed-metric quadratic built %.3f s | chars=%d',toc(ts),symCharCount(h_F4));

if RUN_GEOMETRY_SANITY_AUDIT
    LogMsg('--- GEOMETRY SANITY AUDIT START ---');
    AuditScalar('h_F1',h_F1); AuditScalar('h_F2',h_F2); AuditScalar('h_F3',h_F3); AuditScalar('h_F4',h_F4);
    AuditNoInputDependence('h_F1',h_F1,[u2 u3 u4]);
    AuditNoInputDependence('h_F2',h_F2,[u2 u3 u4]);
    AuditNoInputDependence('h_F3',h_F3,[u2 u3 u4]);
    AuditNoInputDependence('h_F4',h_F4,[u2 u3 u4]);
    LogMsg('[GEOM][F3] nH is intentionally external in this audit; full supporting-plane CBF needs n dynamics/optimization.');
    LogMsg('[GEOM][F4] Qfix is intentionally external/constant; do NOT freeze Q(pT) in a final exact CBF without accounting for Qdot.');
    LogMsg('[GEOM] obstacle is arbitrary oriented ellipsoid via Rz*Ry*Rx.');
    LogMsg('[GEOM] assumptions at runtime: ||pT||=1; obstacle axes>0; betaM>0; nH nonzero (normally unit).');
    LogMsg('--- GEOMETRY SANITY AUDIT END ---');
end

%% ========================================================================
%% 4. FORMULATION AUDIT
%% ========================================================================
opts.maxRD = MAX_RELATIVE_DEGREE;
opts.softStep = SOFT_STEP_LIMIT_SEC;
opts.totalLimit = TOTAL_MODEL_LIMIT_SEC;
opts.growthLimit = GROWTH_LIMIT;
opts.minGrowthBase = MIN_GROWTH_BASE_SEC;

results = struct();
if RUN_F1_SPHERE_CURRENT
    fprintf('\n=== F1: SPHERE + CURRENT INVERSE ELLIPSOID FORM ===\n');
    results.F1 = AuditBarrier('F1 sphere/current-inverse',h_F1,f1,g1,x,t,opts);
end
if RUN_F2_SINGLE_CURRENT
    fprintf('\n=== F2: SINGLE ELLIPSOID + CURRENT INVERSE FORM ===\n');
    results.F2 = AuditBarrier('F2 single/current-inverse',h_F2,f1,g1,x,t,opts);
end
if RUN_F3_SINGLE_SUPPORT
    fprintf('\n=== F3: SINGLE ELLIPSOID + SUPPORTING HYPERPLANE FORM ===\n');
    results.F3 = AuditBarrier('F3 single/support-plane',h_F3,f1,g1,x,t,opts);
end
if RUN_F4_FIXED_METRIC
    fprintf('\n=== F4: FIXED-METRIC QUADRATIC ELLIPSOID FORM ===\n');
    results.F4 = AuditBarrier('F4 fixed-metric',h_F4,f1,g1,x,t,opts);
end

%% ========================================================================
%% 5. SUMMARY TABLE + SAVE
%% ========================================================================
names = fieldnames(results);
Formulation = strings(numel(names),1);
DetectedRD = nan(numel(names),1);
RDLB = nan(numel(names),1);
Status = strings(numel(names),1);
TotalSec = nan(numel(names),1);
MaxStepSec = nan(numel(names),1);
LastPhiChars = nan(numel(names),1);
for i=1:numel(names)
    r = results.(names{i});
    Formulation(i)=string(r.name);
    DetectedRD(i)=r.detectedRD;
    RDLB(i)=r.relativeDegreeLowerBound;
    Status(i)=string(r.status);
    TotalSec(i)=r.totalSec;
    MaxStepSec(i)=r.maxStepSec;
    LastPhiChars(i)=r.lastPhiChars;
end
Summary = table(Formulation,DetectedRD,RDLB,Status,TotalSec,MaxStepSec,LastPhiChars);

fprintf('\n================ AUDIT SUMMARY ================\n');
disp(Summary);
writetable(Summary,CSV_FILE);
save(MAT_FILE,'results','Summary','opts');
LogMsg('Saved MAT: %s',MAT_FILE);
LogMsg('Saved CSV: %s',CSV_FILE);
LogMsg('CBF FORMULATION AUDIT END');

%% ========================================================================
%% LOCAL FUNCTIONS
%% ========================================================================
function out = AuditBarrier(name,h,f,g,x,t,opts)
    tModel=tic;
    phi=h;
    lastLgSec=NaN; lastDriftSec=NaN;
    maxStep=0;
    detectedRD=NaN;
    lowerBound=0;
    status="maxRD_reached_without_input";
    history = struct('r',{},'LgSec',{},'LgZero',{},'DriftSec',{},'phiChars',{},'LgChars',{});

    LogMsg('[%s] AUDIT START',name);
    LogExpr(name,'h',h);

    for r=1:opts.maxRD
        elapsed=toc(tModel);
        if elapsed > opts.totalLimit
            status="stopped_total_time_before_next_step";
            LogMsg('[STOP][%s] cumulative %.1f s > total limit %.1f s before r=%d.',name,elapsed,opts.totalLimit,r);
            break;
        end

        % Growth predictor: avoid launching another likely catastrophic Lg.
        if r>=3 && isfinite(lastLgSec) && lastLgSec>opts.softStep
            status="stopped_after_slow_previous_Lg";
            LogMsg('[STOP][%s] previous Lg %.1f s exceeded soft limit %.1f s; not launching r=%d.',name,lastLgSec,opts.softStep,r);
            break;
        end

        LogMsg('[%s] r=%d Lg(phi_%d) START',name,r,r-1); drawnow;
        ts=tic; Lgphi=LieD(phi,g,x); lgSec=toc(ts); maxStep=max(maxStep,lgSec);
        lgZero=IsSymbolicZero(Lgphi);
        lgChars=symCharCount(Lgphi);
        LogMsg('[%s] r=%d Lg DONE %.3f s | zero=%d | chars=%d',name,r,lgSec,lgZero,lgChars);

        if isfinite(lastLgSec) && lastLgSec>=opts.minGrowthBase
            growth=lgSec/max(lastLgSec,eps);
            LogMsg('[%s] r=%d Lg time growth x%.2f',name,r,growth);
            if growth>=opts.growthLimit
                LogMsg('[WARN][%s] Lg growth x%.2f >= %.2f',name,growth,opts.growthLimit);
            end
        end
        lastLgSec=lgSec;

        rec.r=r; rec.LgSec=lgSec; rec.LgZero=lgZero; rec.DriftSec=NaN;
        rec.phiChars=symCharCount(phi); rec.LgChars=lgChars;

        if ~lgZero
            detectedRD=r; lowerBound=r;
            status="relative_degree_detected";
            history(end+1)=rec; %#ok<AGROW>
            LogMsg('[PASS][%s] control appears at derivative %d -> rd=%d',name,r,r);
            break;
        end
        lowerBound=r; % if Lg(phi_{r-1})=0, actual rd > r

        if lgSec > opts.softStep
            status="stopped_after_slow_Lg";
            history(end+1)=rec; %#ok<AGROW>
            LogMsg('[STOP][%s] Lg %.1f s > soft limit %.1f s. Proven rd > %d; drift to next order skipped.',name,lgSec,opts.softStep,r);
            break;
        end

        if toc(tModel)>opts.totalLimit
            status="stopped_total_time_after_Lg";
            history(end+1)=rec; %#ok<AGROW>
            LogMsg('[STOP][%s] cumulative time limit reached after Lg; proven rd > %d.',name,r);
            break;
        end

        LogMsg('[%s] phi_%d=Dbar(phi_%d) START',name,r,r-1); drawnow;
        ts=tic; phiNext=Dbar(phi,f,x,t); driftSec=toc(ts); maxStep=max(maxStep,driftSec);
        rec.DriftSec=driftSec;
        history(end+1)=rec; %#ok<AGROW>
        LogMsg('[%s] phi_%d DONE %.3f s | chars=%d',name,r,driftSec,symCharCount(phiNext));

        if isfinite(lastDriftSec) && lastDriftSec>=opts.minGrowthBase
            growth=driftSec/max(lastDriftSec,eps);
            LogMsg('[%s] phi drift time growth x%.2f',name,growth);
            if growth>=opts.growthLimit
                LogMsg('[WARN][%s] drift growth x%.2f >= %.2f',name,growth,opts.growthLimit);
            end
        end
        lastDriftSec=driftSec;
        phi=phiNext;

        if driftSec > opts.softStep
            status="stopped_after_slow_drift";
            LogMsg('[STOP][%s] drift %.1f s > soft limit %.1f s. Proven rd > %d; next Lg skipped.',name,driftSec,opts.softStep,r);
            break;
        end
    end

    totalSec=toc(tModel);
    out=struct();
    out.name=name;
    out.detectedRD=detectedRD;
    out.relativeDegreeLowerBound=lowerBound;
    out.status=status;
    out.totalSec=totalSec;
    out.maxStepSec=maxStep;
    out.lastPhiChars=symCharCount(phi);
    out.history=history;
    LogMsg('[%s] AUDIT END status=%s detectedRD=%s rdLowerBound=%d total=%.3f s maxStep=%.3f s', ...
        name,char(status),num2str(detectedRD),lowerBound,totalSec,maxStep);
end

function h = EllipsoidSeparationBarrier(cA,SA,cB,SB,betaM)
    Ssum=(1+1/betaM)*SA+(1+betaM)*SB;
    dc=cA-cB;
    h=dc.'*(Ssum\dc)-1;
end

function AuditPrefixZero(name,h,f,g,x,t,nOrders)
    phi=h;
    for r=1:nOrders
        ts=tic; z=LieD(phi,g,x); dt=toc(ts);
        isz=IsSymbolicZero(z);
        LogMsg('[HLC-AUDIT] %s Lg(phi_%d): zero=%d time=%.3f s',name,r-1,isz,dt);
        if ~isz, return; end
        phi=Dbar(phi,f,x,t);
    end
end

function LogExpr(name,label,expr)
    LogMsg('[STAT][%s] %s size=%s symvars=%d chars=%d',name,label,mat2str(size(expr)),numel(symvar(expr)),symCharCount(expr));
end

function tf=IsSymbolicZero(M)
    tf=true;
    for k=1:numel(M)
        try
            z=isAlways(M(k)==0,'Unknown','false');
        catch
            z=isequal(M(k),sym(0));
        end
        if ~z, tf=false; return; end
    end
end

function LogMsg(fmt,varargin)
    fprintf('[%s] ',datestr(now,'HH:MM:SS.FFF'));
    fprintf(fmt,varargin{:}); fprintf('\n'); drawnow;
end

function AuditSize(name,value,expected)
    if isequal(size(value),expected)
        LogMsg('[PASS] size(%s)=%s',name,mat2str(size(value)));
    else
        LogMsg('[ERROR] size(%s)=%s expected=%s',name,mat2str(size(value)),mat2str(expected));
    end
end

function AuditScalar(name,value)
    if isscalar(value), LogMsg('[PASS] %s is scalar.',name); else, LogMsg('[ERROR] %s is not scalar.',name); end
end

function AuditNonzero(name,value)
    if IsSymbolicZero(value), LogMsg('[ERROR] %s is symbolically zero.',name); else, LogMsg('[PASS] %s is not symbolically zero.',name); end
end

function AuditNoInputDependence(name,h,inputs)
    vars=symvar(h);
    dep=false;
    for i=1:numel(inputs)
        if any(arrayfun(@(z) isequal(z,inputs(i)),vars)), dep=true; end
    end
    if dep, LogMsg('[ERROR] %s has direct control-symbol dependence.',name); else, LogMsg('[PASS] %s has no direct u2/u3/u4 dependence.',name); end
end

function y=Dbar(h,f,x,t)
    y=LieD(h,f,x)+diff(h,t);
end

function pd=pdiff(flist,vars)
    flist=flist(:); vars=vars(:);
    pdiff_tmp=@(flist,vars) arrayfun(@(func) arrayfun(@(xx) diff(func,xx),vars)',flist,'UniformOutput',false);
    pd=cell2sym(pdiff_tmp(flist,vars));
end

function dh=LieD(h,f,x)
    x=x(:); dh=pdiff(h,x)*f;
end

function mat=MyCoeff(M,vars)
    vars=vars(:);
    mat=coder.nullcopy(sym(zeros(length(M),length(vars))));
    for i=1:length(M)
        for j=1:length(vars)
            mat(i,j)=subs(M(i)-subs(M(i),vars(j),0),vars(j),1);
        end
    end
end

function R=RotZYX(roll,pitch,yaw)
    cr=cos(roll); sr=sin(roll); cp=cos(pitch); sp=sin(pitch); cy=cos(yaw); sy=sin(yaw);
    Rx=[1 0 0;0 cr -sr;0 sr cr];
    Ry=[cp 0 sp;0 1 0;-sp 0 cp];
    Rz=[cy -sy 0;sy cy 0;0 0 1];
    R=Rz*Ry*Rx;
end

function n=symCharCount(expr)
    try
        n=strlength(string(expr));
        n=sum(double(n),'all');
    catch
        try, n=numel(char(expr)); catch, n=-1; end
    end
end
