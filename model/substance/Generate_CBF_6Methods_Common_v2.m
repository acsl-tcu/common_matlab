%% Generate_CBF_6Methods_Common.m
% Common symbolic generator / registry for six suspended-load CBF candidates.
%
% Geometry x formulation matrix
%   S1: single ellipsoid + support/separating hyperplane
%   S2: multi-sphere      + support/separating hyperplane
%   S3: multi-ellipsoid   + support/separating hyperplane
%   K1: single ellipsoid + KKT minimum distance
%   K2: multi-sphere      + KKT minimum distance
%   K3: multi-ellipsoid   + KKT minimum distance
%
% Design principles
% -----------------
% 1) Keep the original HLC first-layer embedding exactly aligned with the
%    supplied generator: FG=simplify(...), g1=simplify(MyCoeff(...)).
% 2) Never build a symbolic matrix inverse for collision geometry or Jz.
% 3) Obstacle orientation is arbitrary.  For support-plane primitives the
%    obstacle shape SObs is parameterized by roll/pitch/yaw.  For KKT
%    primitives the inverse obstacle metric PB is passed from numeric runtime
%    preprocessing.
% 4) Multi-sphere is generated as ONE generic component with lambdaSphere and
%    sphere_r; instantiate it multiple times at runtime.
% 5) Multi-ellipsoid is generated as UAV / cable / load components.
% 6) The support-plane normal nH is still an external auxiliary variable in
%    this generator.  A final controller must supply a valid update/dynamic
%    extension and maintain ||nH||=1.
% 7) The KKT primitives expose F,Jz,Jx and hdist.  At runtime solve linear
%    systems with Jz\rhs; NEVER compute inv(Jz).
%
% This file intentionally generates reusable primitives + metadata.  It does
% not yet claim a complete safety proof for either auxiliary-variable law.

clearvars; clc;

%% ========================================================================
%% 0. SETTINGS
%% ========================================================================
GENERATE_MATLAB_PRIMITIVES = true;
RUN_NEW_S2_SUPPORT_RD_AUDIT = true;   % only the new generic multi-sphere case
RUN_FULL_SUPPORT_RD_AUDIT = false;    % S1/S3 were already audited; optional rerun
MAX_RELATIVE_DEGREE = 6;
SOFT_STEP_LIMIT_SEC = 15;
TOTAL_MODEL_LIMIT_SEC = 75;
MAX_PHI_CHARS_BEFORE_LG = 2e6;
MAX_PHI_CHARS_BEFORE_DRIFT = 8e5;

OUT_DIR = fullfile(pwd,'generated_cbf_6methods');
if ~exist(OUT_DIR,'dir'), mkdir(OUT_DIR); end
stamp = datestr(now,'yyyymmdd_HHMMSS');
LOG_FILE = fullfile(OUT_DIR,['CBF6_generate_' stamp '.txt']);
META_FILE = fullfile(OUT_DIR,['CBF6_metadata_' stamp '.mat']);
COMP_CSV = fullfile(OUT_DIR,['CBF6_components_' stamp '.csv']);
METHOD_CSV = fullfile(OUT_DIR,['CBF6_methods_' stamp '.csv']);

% Benchmark output schema/template for later closed-loop comparison.
BENCHMARK_CSV = fullfile(OUT_DIR,'CBF6_runtime_benchmark.csv');

% Diary
if exist(LOG_FILE,'file'), delete(LOG_FILE); end
diary(LOG_FILE); cleanupDiary=onCleanup(@() diary('off'));
LogMsg('============================================================');
LogMsg('CBF SIX-METHOD COMMON GENERATOR START');
LogMsg('MATLAB version: %s',version);
LogMsg('Output dir: %s',OUT_DIR);
LogMsg('Methods: S1 S2 S3 K1 K2 K3');
LogMsg('Direct symbolic inverse collision formulations: DISABLED');
LogMsg('============================================================');

%% ========================================================================
%% 1. ORIGINAL SYMBOLIC PLANT / HLC FIRST-LAYER EMBEDDING
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

LogMsg('[MODEL] FLxyDst START'); ts=tic; f=FLxyDst(x,physicalParamFull); LogMsg('[MODEL] FLxyDst %.3fs',toc(ts));
LogMsg('[MODEL] GLxyDst START'); ts=tic; g=GLxyDst(x,physicalParamFull); LogMsg('[MODEL] GLxyDst %.3fs',toc(ts));

h1=pl3-xd(3);
dh1=LieD(h1,f,x)+diff(h1,t);
alpha1=LieD(dh1,f,x)+diff(dh1,t);
beta1=simplify(LieD(dh1,g,x));
H=[1/beta1(1),-beta1(2)/beta1(1),-beta1(3)/beta1(1),-beta1(4)/beta1(1);zeros(3,1),eye(3)];
LogMsg('[HLC] FG simplify START'); ts=tic;
FG=simplify(f+g*H*[-alpha1+v1(t);u2;u3;u4]);
LogMsg('[HLC] FG simplify %.3fs chars=%d',toc(ts),symCharCount(FG));
LogMsg('[HLC] g1 simplify START'); ts=tic;
g1=simplify(MyCoeff(FG,[u2;u3;u4]));
LogMsg('[HLC] g1 simplify %.3fs chars=%d',toc(ts),symCharCount(g1));
f1=subs(FG,[u2,u3,u4],[0,0,0]);

AuditSize('x',x,[19 1]); AuditSize('f1',f1,[19 1]); AuditSize('g1',g1,[19 3]); AuditNonzero('beta1(1)',beta1(1));

%% ========================================================================
%% 2. COMMON GEOMETRY
%% ========================================================================
% Arbitrarily oriented obstacle ellipsoid.
syms obs_x obs_y obs_z obs_a obs_b obs_c real
syms obs_roll obs_pitch obs_yaw real
cObs=[obs_x;obs_y;obs_z];
obsPose=[obs_x obs_y obs_z obs_a obs_b obs_c obs_roll obs_pitch obs_yaw];
RObs=RotZYX(obs_roll,obs_pitch,obs_yaw);
SObs=RObs*diag([obs_a^2,obs_b^2,obs_c^2])*RObs.';

% External unit normal for support-plane candidates.
syms nh1 nh2 nh3 real
nH=[nh1;nh2;nh3];

% Centers used by the geometry approximations.
pU=pl-cableL*pT;
pMid=pl-(cableL/2)*pT;

% Single whole-system ellipsoid and cable ellipsoid are axisymmetric around pT.
syms single_a single_b positive
syms cable_a cable_b positive
SSingle=single_b^2*eye(3)+(single_a^2-single_b^2)*(pT*pT.');
SCable=cable_b^2*eye(3)+(cable_a^2-cable_b^2)*(pT*pT.');

% Fixed outer UAV and load ellipsoids.
syms uav_a uav_b uav_c load_a load_b load_c positive
SUav=diag([uav_a^2,uav_b^2,uav_c^2]);
SLoad=diag([load_a^2,load_b^2,load_c^2]);

% Generic sphere located along the cable/load/UAV line.
% lambdaSphere=0 -> load center, lambdaSphere=1 -> UAV center.
syms lambdaSphere sphere_r positive
pSphere=pl-lambdaSphere*cableL*pT;
SSphere=sphere_r^2*eye(3);

%% ========================================================================
%% 3. SUPPORT-PLANE FAMILY (S1,S2,S3)
%% ========================================================================
% S1: one system ellipsoid.
hS1 = SupportPlaneBarrier(pMid,SSingle,cObs,SObs,nH);

% S2: generic sphere component; instantiate many lambda/radius pairs at runtime.
hS2_generic = SupportPlaneBarrier(pSphere,SSphere,cObs,SObs,nH);

% S3: UAV + cable + load ellipsoids.
hS3_uav   = SupportPlaneBarrier(pU,SUav,cObs,SObs,nH);
hS3_cable = SupportPlaneBarrier(pMid,SCable,cObs,SObs,nH);
hS3_load  = SupportPlaneBarrier(pl,SLoad,cObs,SObs,nH);

LogMsg('--- SUPPORT PRIMITIVE SIZES ---');
LogBarrierBuild('S1 single/support',hS1);
LogBarrierBuild('S2 sphere/support generic',hS2_generic);
LogBarrierBuild('S3 UAV/support',hS3_uav);
LogBarrierBuild('S3 cable/support',hS3_cable);
LogBarrierBuild('S3 load/support',hS3_load);

%% ========================================================================
%% 4. KKT FAMILY (K1,K2,K3)
%% ========================================================================
% Closest-point variables and active-boundary multipliers.
syms ya1 ya2 ya3 yb1 yb2 yb3 lamA lamB real
yA=[ya1;ya2;ya3]; yB=[yb1;yb2;yb3]; zK=[yA;yB;lamA;lamB];
syms kkt_dsafe positive

% Obstacle inverse metric is supplied NUMERICALLY at runtime after applying
% arbitrary obstacle orientation. This is a symmetric 3x3 matrix encoded by 6 values.
syms PB11 PB22 PB33 PB12 PB13 PB23 real
PBvec=[PB11 PB22 PB33 PB12 PB13 PB23];
PB=[PB11 PB12 PB13;PB12 PB22 PB23;PB13 PB23 PB33];

PUav=diag([1/uav_a^2,1/uav_b^2,1/uav_c^2]);
PLoad=diag([1/load_a^2,1/load_b^2,1/load_c^2]);
PSingle=(1/single_b^2)*eye(3)+(1/single_a^2-1/single_b^2)*(pT*pT.');
PCable=(1/cable_b^2)*eye(3)+(1/cable_a^2-1/cable_b^2)*(pT*pT.');
PSphere=(1/sphere_r^2)*eye(3);

K1 = BuildEllipsoidKKT('K1 single/KKT',pMid,PSingle,cObs,PB,yA,yB,lamA,lamB,zK,x,g1,kkt_dsafe);
K2 = BuildEllipsoidKKT('K2 sphere/KKT generic',pSphere,PSphere,cObs,PB,yA,yB,lamA,lamB,zK,x,g1,kkt_dsafe);
K3u= BuildEllipsoidKKT('K3 UAV/KKT',pU,PUav,cObs,PB,yA,yB,lamA,lamB,zK,x,g1,kkt_dsafe);
K3c= BuildEllipsoidKKT('K3 cable/KKT',pMid,PCable,cObs,PB,yA,yB,lamA,lamB,zK,x,g1,kkt_dsafe);
K3l= BuildEllipsoidKKT('K3 load/KKT',pl,PLoad,cObs,PB,yA,yB,lamA,lamB,zK,x,g1,kkt_dsafe);

%% ========================================================================
%% 5. METHOD REGISTRY -- THE SIX FINAL CANDIDATES
%% ========================================================================
Methods = MethodDef('S1','single_ellipsoid','support_plane',{'single'},'external nH; final n-dynamics pending');
Methods(2)=MethodDef('S2','multi_sphere','support_plane',{'sphere_generic'},'instantiate N spheres via lambdaSphere/radius');
Methods(3)=MethodDef('S3','multi_ellipsoid','support_plane',{'uav','cable','load'},'three support constraints per obstacle');
Methods(4)=MethodDef('K1','single_ellipsoid','kkt_min_distance',{'single'},'8-state KKT closest-point auxiliary system');
Methods(5)=MethodDef('K2','multi_sphere','kkt_min_distance',{'sphere_generic'},'one KKT system per sphere/obstacle pair');
Methods(6)=MethodDef('K3','multi_ellipsoid','kkt_min_distance',{'uav','cable','load'},'one KKT system per component/obstacle pair');

% Prior observed support-plane relative degrees from the validated audit log.
% These are metadata only; set RUN_FULL_SUPPORT_RD_AUDIT=true to recompute.
Methods(1).ObservedRD='4';
Methods(2).ObservedRD='audit_this_run';
Methods(3).ObservedRD='UAV=4,Cable=4,Load=6';
Methods(4).ObservedRD='pending KKT extended-system audit';
Methods(5).ObservedRD='pending KKT extended-system audit';
Methods(6).ObservedRD='pending KKT extended-system audit';

%% ========================================================================
%% 6. OPTIONAL SUPPORT-PLANE RELATIVE-DEGREE AUDIT
%% ========================================================================
opts.maxRD=MAX_RELATIVE_DEGREE;
opts.softStep=SOFT_STEP_LIMIT_SEC;
opts.totalLimit=TOTAL_MODEL_LIMIT_SEC;
opts.maxPhiBeforeLg=MAX_PHI_CHARS_BEFORE_LG;
opts.maxPhiBeforeDrift=MAX_PHI_CHARS_BEFORE_DRIFT;
SupportAudit=struct();

if RUN_NEW_S2_SUPPORT_RD_AUDIT
    fprintf('\n=== S2 GENERIC SPHERE + SUPPORT PLANE RELATIVE-DEGREE AUDIT ===\n');
    SupportAudit.S2=AuditBarrierSafe('S2 sphere/support generic',hS2_generic,f1,g1,x,t,opts);
    if ~isnan(SupportAudit.S2.detectedRD)
        Methods(2).ObservedRD=num2str(SupportAudit.S2.detectedRD);
    else
        Methods(2).ObservedRD=['>' num2str(SupportAudit.S2.relativeDegreeLowerBound)];
    end
end

if RUN_FULL_SUPPORT_RD_AUDIT
    SupportAudit.S1=AuditBarrierSafe('S1 single/support',hS1,f1,g1,x,t,opts);
    SupportAudit.S3_uav=AuditBarrierSafe('S3 UAV/support',hS3_uav,f1,g1,x,t,opts);
    SupportAudit.S3_cable=AuditBarrierSafe('S3 cable/support',hS3_cable,f1,g1,x,t,opts);
    SupportAudit.S3_load=AuditBarrierSafe('S3 load/support',hS3_load,f1,g1,x,t,opts);
end

%% ========================================================================
%% 7. GENERATE REUSABLE PRIMITIVE FUNCTIONS
%% ========================================================================
if GENERATE_MATLAB_PRIMITIVES
    LogMsg('--- MATLAB FUNCTION GENERATION START ---');
    % Support-plane barriers. Keep inputs grouped for easier runtime use.
    GenerateFun(hS1,fullfile(OUT_DIR,'CBF6_S1_support_h.m'),'h', ...
        {x,obsPose,nH,[single_a single_b cableL]});
    GenerateFun(hS2_generic,fullfile(OUT_DIR,'CBF6_S2_support_h_genericSphere.m'),'h', ...
        {x,obsPose,nH,[lambdaSphere sphere_r cableL]});
    GenerateFun(hS3_uav,fullfile(OUT_DIR,'CBF6_S3_support_h_uav.m'),'h', ...
        {x,obsPose,nH,[uav_a uav_b uav_c cableL]});
    GenerateFun(hS3_cable,fullfile(OUT_DIR,'CBF6_S3_support_h_cable.m'),'h', ...
        {x,obsPose,nH,[cable_a cable_b cableL]});
    GenerateFun(hS3_load,fullfile(OUT_DIR,'CBF6_S3_support_h_load.m'),'h', ...
        {x,obsPose,nH,[load_a load_b load_c]});

    % KKT primitives.  PBvec is runtime-preprocessed obstacle inverse metric.
    GenerateKKTPrimitive(K1,fullfile(OUT_DIR,'CBF6_K1_KKT_single.m'), ...
        x,zK,cObs,PBvec,[single_a single_b cableL kkt_dsafe]);
    GenerateKKTPrimitive(K2,fullfile(OUT_DIR,'CBF6_K2_KKT_genericSphere.m'), ...
        x,zK,cObs,PBvec,[lambdaSphere sphere_r cableL kkt_dsafe]);
    GenerateKKTPrimitive(K3u,fullfile(OUT_DIR,'CBF6_K3_KKT_uav.m'), ...
        x,zK,cObs,PBvec,[uav_a uav_b uav_c cableL kkt_dsafe]);
    GenerateKKTPrimitive(K3c,fullfile(OUT_DIR,'CBF6_K3_KKT_cable.m'), ...
        x,zK,cObs,PBvec,[cable_a cable_b cableL kkt_dsafe]);
    GenerateKKTPrimitive(K3l,fullfile(OUT_DIR,'CBF6_K3_KKT_load.m'), ...
        x,zK,cObs,PBvec,[load_a load_b load_c kkt_dsafe]);
    LogMsg('--- MATLAB FUNCTION GENERATION END ---');
end

%% ========================================================================
%% 8. COMPONENT + METHOD SUMMARY TABLES
%% ========================================================================
ComponentID = ["S1.single";"S2.sphere_generic";"S3.uav";"S3.cable";"S3.load"; ...
               "K1.single";"K2.sphere_generic";"K3.uav";"K3.cable";"K3.load"];
Formulation = [repmat("support_plane",5,1);repmat("kkt_min_distance",5,1)];
Geometry = ["single_ellipsoid";"multi_sphere";repmat("multi_ellipsoid",3,1); ...
            "single_ellipsoid";"multi_sphere";repmat("multi_ellipsoid",3,1)];
PrimitiveChars=[symCharCount(hS1);symCharCount(hS2_generic);symCharCount(hS3_uav);symCharCount(hS3_cable);symCharCount(hS3_load); ...
                K1.FChars;K2.FChars;K3u.FChars;K3c.FChars;K3l.FChars];
JzChars=[nan(5,1);K1.JzChars;K2.JzChars;K3u.JzChars;K3c.JzChars;K3l.JzChars];
JxChars=[nan(5,1);K1.JxChars;K2.JxChars;K3u.JxChars;K3c.JxChars;K3l.JxChars];
JxgZero=[false(5,1);K1.JxgZero;K2.JxgZero;K3u.JxgZero;K3c.JxgZero;K3l.JxgZero];
ComponentSummary=table(ComponentID,Geometry,Formulation,PrimitiveChars,JzChars,JxChars,JxgZero);
writetable(ComponentSummary,COMP_CSV);

MethodID=string({Methods.ID})';
Geometry=string({Methods.Geometry})';
Formulation=string({Methods.Formulation})';
Components=string(cellfun(@(c) strjoin(c,'+'),{Methods.Components},'UniformOutput',false))';
ObservedRD=string({Methods.ObservedRD})';
CurrentSafetyStatus=string({Methods.SafetyStatus})';
MethodSummary=table(MethodID,Geometry,Formulation,Components,ObservedRD,CurrentSafetyStatus);
writetable(MethodSummary,METHOD_CSV);

fprintf('\n================ SIX-METHOD REGISTRY ================\n'); disp(MethodSummary);
fprintf('\n================ COMPONENT STRUCTURE ================\n'); disp(ComponentSummary);

%% ========================================================================
%% 9. CREATE EMPTY RUNTIME BENCHMARK FILE
%% ========================================================================
if ~exist(BENCHMARK_CSV,'file')
    T=BenchmarkSchema();
    writetable(T,BENCHMARK_CSV);
    LogMsg('Created runtime benchmark CSV schema: %s',BENCHMARK_CSV);
else
    LogMsg('Runtime benchmark CSV already exists: %s',BENCHMARK_CSV);
end

save(META_FILE,'Methods','MethodSummary','ComponentSummary','SupportAudit','opts');
LogMsg('Saved metadata: %s',META_FILE);
LogMsg('Saved method CSV: %s',METHOD_CSV);
LogMsg('Saved component CSV: %s',COMP_CSV);
LogMsg('GENERATOR END');

%% ========================================================================
%% LOCAL FUNCTIONS
%% ========================================================================
function s=MethodDef(id,geometry,formulation,components,note)
    s=struct('ID',id,'Geometry',geometry,'Formulation',formulation, ...
        'Components',{components},'ObservedRD','', ...
        'SafetyStatus','candidate: auxiliary dynamics / final proof pending', ...
        'Note',note);
end

function data=BuildEllipsoidKKT(name,cA,PA,cB,PB,yA,yB,lamA,lamB,zK,x,g1,dsafe)
    ts=tic;
    r=yA-yB; eA=yA-cA; eB=yB-cB;
    gA=eA.'*PA*eA-1; gB=eB.'*PB*eB-1;
    F=[r+2*lamA*PA*eA; -r+2*lamB*PB*eB; gA; gB];
    hdist=r.'*r-dsafe^2;
    Jz=jacobian(F,zK); Jx=jacobian(F,x); hz=jacobian(hdist,zK); Jxg=Jx*g1;
    data=struct('name',name,'F',F,'Jz',Jz,'Jx',Jx,'hz',hz,'hdist',hdist,'Jxg',Jxg, ...
        'FChars',symCharCount(F),'JzChars',symCharCount(Jz),'JxChars',symCharCount(Jx), ...
        'JxgZero',IsSymbolicZero(Jxg),'buildSec',toc(ts));
    LogMsg('[KKT] %-24s build=%.3fs F=%d Jz=%d Jx=%d JxgZero=%d', ...
        name,data.buildSec,data.FChars,data.JzChars,data.JxChars,data.JxgZero);
end

function h=SupportPlaneBarrier(cA,SA,cB,SB,n)
    h=n.'*(cB-cA)-sqrt(n.'*SA*n)-sqrt(n.'*SB*n);
end

function GenerateFun(expr,filePath,outName,vars)
    ts=tic;
    matlabFunction(expr,'file',filePath,'Vars',vars,'Outputs',{outName},'Optimize',true);
    LogMsg('[GEN] %s %.3fs',filePath,toc(ts));
end

function GenerateKKTPrimitive(data,filePath,x,zK,cObs,PBvec,paramVec)
    ts=tic;
    matlabFunction(data.F,data.Jz,data.Jx,data.hdist, ...
        'file',filePath,'Vars',{x,zK,cObs,PBvec,paramVec}, ...
        'Outputs',{'F','Jz','Jx','hdist'},'Optimize',true);
    LogMsg('[GEN-KKT] %s %.3fs',filePath,toc(ts));
end

function T=BenchmarkSchema()
    MethodID=strings(0,1); Scenario=strings(0,1); TimeSec=zeros(0,1); ObstacleID=zeros(0,1);
    ConstraintCount=zeros(0,1); CbfEvalSec=zeros(0,1); AuxUpdateSec=zeros(0,1); QpSolveSec=zeros(0,1);
    MinH=zeros(0,1); MinGeomDistance=zeros(0,1); InterventionNorm=zeros(0,1); TrackingErrorNorm=zeros(0,1);
    ExitFlag=zeros(0,1); MaxActuatorRatio=zeros(0,1); KKTResidualNorm=zeros(0,1); KKTJzRcond=zeros(0,1);
    KKTMinLambda=zeros(0,1); SupportNormError=zeros(0,1); SupportNdotNorm=zeros(0,1); Notes=strings(0,1);
    T=table(MethodID,Scenario,TimeSec,ObstacleID,ConstraintCount,CbfEvalSec,AuxUpdateSec,QpSolveSec, ...
        MinH,MinGeomDistance,InterventionNorm,TrackingErrorNorm,ExitFlag,MaxActuatorRatio, ...
        KKTResidualNorm,KKTJzRcond,KKTMinLambda,SupportNormError,SupportNdotNorm,Notes);
end

function out=AuditBarrierSafe(name,h,f,g,x,t,opts)
    tModel=tic; phi=h; maxStep=0; detectedRD=NaN; lowerBound=0; status="maxRD_reached_without_input";
    history=struct('r',{},'LgSec',{},'LgZero',{},'DriftSec',{},'phiChars',{},'LgChars',{});
    LogMsg('[%s] AUDIT START',name);
    for r=1:opts.maxRD
        phiChars=symCharCount(phi);
        if toc(tModel)>opts.totalLimit, status="stopped_total_time"; break; end
        if phiChars>opts.maxPhiBeforeLg, status="stopped_phi_gate_Lg"; break; end
        ts=tic; Lgphi=LieD(phi,g,x); lgSec=toc(ts); maxStep=max(maxStep,lgSec);
        z=IsSymbolicZero(Lgphi); rec=struct('r',r,'LgSec',lgSec,'LgZero',z,'DriftSec',NaN,'phiChars',phiChars,'LgChars',symCharCount(Lgphi));
        LogMsg('[%s] r=%d Lg %.3fs zero=%d phiChars=%d',name,r,lgSec,z,phiChars);
        if ~z, detectedRD=r; lowerBound=r; status="relative_degree_detected"; history(end+1)=rec; break; end %#ok<AGROW>
        lowerBound=r;
        if lgSec>opts.softStep, status="stopped_after_slow_Lg"; history(end+1)=rec; break; end %#ok<AGROW>
        if phiChars>opts.maxPhiBeforeDrift, status="stopped_phi_gate_drift"; history(end+1)=rec; break; end %#ok<AGROW>
        ts=tic; phiNext=LieD(phi,f,x)+diff(phi,t); driftSec=toc(ts); maxStep=max(maxStep,driftSec); rec.DriftSec=driftSec; history(end+1)=rec; %#ok<AGROW>
        LogMsg('[%s] phi_%d drift %.3fs chars=%d',name,r,driftSec,symCharCount(phiNext));
        phi=phiNext;
        if driftSec>opts.softStep, status="stopped_after_slow_drift"; break; end
    end
    out=struct('name',name,'detectedRD',detectedRD,'relativeDegreeLowerBound',lowerBound,'status',status, ...
        'totalSec',toc(tModel),'maxStepSec',maxStep,'lastPhiChars',symCharCount(phi),'history',history);
    LogMsg('[%s] AUDIT END status=%s RD=%s total=%.3fs',name,char(status),num2str(detectedRD),out.totalSec);
end

function LogBarrierBuild(name,h), LogMsg('[GEOM] %-28s chars=%d symvars=%d',name,symCharCount(h),numel(symvar(h))); end
function LogMsg(fmt,varargin), fprintf('[%s] ',datestr(now,'HH:MM:SS.FFF')); fprintf(fmt,varargin{:}); fprintf('\n'); drawnow; end
function AuditSize(name,value,expected), if isequal(size(value),expected), LogMsg('[PASS] size(%s)=%s',name,mat2str(size(value))); else, LogMsg('[ERROR] size(%s)=%s expected=%s',name,mat2str(size(value)),mat2str(expected)); end, end
function AuditNonzero(name,value), if IsSymbolicZero(value), LogMsg('[ERROR] %s is symbolically zero.',name); else, LogMsg('[PASS] %s is not symbolically zero.',name); end, end
function tf=IsSymbolicZero(M), tf=true; for k=1:numel(M), try, z=isAlways(M(k)==0,'Unknown','false'); catch, z=isequal(M(k),sym(0)); end, if ~z, tf=false; return; end, end, end
function pd=pdiff(flist,vars), flist=flist(:); vars=vars(:); pdiff_tmp=@(flist,vars) arrayfun(@(func) arrayfun(@(xx) diff(func,xx),vars)',flist,'UniformOutput',false); pd=cell2sym(pdiff_tmp(flist,vars)); end
function dh=LieD(h,f,x), x=x(:); dh=pdiff(h,x)*f; end
function mat=MyCoeff(M,vars), vars=vars(:); mat=coder.nullcopy(sym(zeros(length(M),length(vars)))); for i=1:length(M), for j=1:length(vars), mat(i,j)=subs(M(i)-subs(M(i),vars(j),0),vars(j),1); end, end, end
function R=RotZYX(roll,pitch,yaw), cr=cos(roll); sr=sin(roll); cp=cos(pitch); sp=sin(pitch); cy=cos(yaw); sy=sin(yaw); Rx=[1 0 0;0 cr -sr;0 sr cr]; Ry=[cp 0 sp;0 1 0;-sp 0 cp]; Rz=[cy -sy 0;sy cy 0;0 0 1]; R=Rz*Ry*Rx; end
function n=symCharCount(expr), try, n=sum(double(strlength(string(expr))),'all'); catch, try, n=numel(char(expr)); catch, n=-1; end, end, end
