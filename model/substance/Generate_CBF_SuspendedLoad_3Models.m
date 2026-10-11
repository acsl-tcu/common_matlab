%% Generate_CBF_SuspendedLoad_3Models.m
% CBF generator for a quadrotor with a cable-suspended load.
%
% Based on the same symbolic HLC construction used in the supplied
% controller-generation source.  This file generates geometry/HOCBF data
% for three system approximations against an arbitrarily oriented ellipsoid
% obstacle:
%
%   MODEL 1 : one prolate ellipsoid enclosing the suspended system
%   MODEL 2 : multiple spheres placed along the cable (one generic sphere
%             function is generated; evaluate it for several lambda values)
%   MODEL 3 : multiple ellipsoids (UAV / cable / load)
%
% Obstacle model (common to all three):
%   E_o = {p | (p-c_o)' inv(S_o) (p-c_o) <= 1}
%   S_o = R_o diag([a_o^2 b_o^2 c_o^2]) R_o'
% where R_o is an arbitrary ZYX roll-pitch-yaw orientation.
%
% Ellipsoid-versus-ellipsoid safety surrogate:
%   The Minkowski sum is outer-bounded by
%   S_sum = (1+1/betaM) S_sys + (1+betaM) S_obs, betaM>0.
%   h = dc' inv(S_sum) dc - 1 >= 0
% is therefore a conservative sufficient separation condition.
%
% IMPORTANT DESIGN CHOICE
% -----------------------
% HOCBFs are first derived w.r.t. the 2nd-layer physical inputs
% u234=[u2;u3;u4] in
%       xdot = f1(x,t) + g1(x,t) u234.
% At runtime convert the inequality to the HLC virtual inputs v=[v2;v3;v4]
% using
%       u234 = beta2 \ (v - alpha2).
% If the generated HOCBF gives
%       Au*u234 + bu >= 0,
% then numerically compute
%       Av = Au / beta2;
%       cv = bu - Av*alpha2;
% and enforce
%       Av*v + cv >= 0.
% This avoids symbolically differentiating beta2^{-1} inside the HOCBF.
%
% NOTE ON THE LOAD SHAPE
% ----------------------
% The supplied model has no independent payload attitude state.  Therefore
% MODEL 3 uses a world-fixed load ellipsoid.  If payload attitude is added
% later, replace S_load by R_load*diag(axis.^2)*R_load'.
%
% NOTE ON SAFETY MARGIN
% ---------------------
% For this first generator, include the desired geometric safety margin in
% the supplied system/obstacle semi-axis parameters.  Do not change these
% parameters online if a strict continuous-time CBF interpretation is
% required.

clearvars;
clc;

%% ========================================================================
%% 0. USER OPTIONS
%% ========================================================================
MAX_RELATIVE_DEGREE = 6;
GENERATE_SINGLE_ELLIPSOID = true;
GENERATE_MULTI_SPHERE     = true;
GENERATE_MULTI_ELLIPSOID  = true;
GENERATE_ALPHA2           = true;

% simplify() on high-order expressions can dominate generation time.
% Keep false initially.  The generated expressions are still exact.
USE_SIMPLIFY_EACH_STEP = false;

OUT_DIR = fullfile(fileparts(mfilename('fullpath')), 'generated_cbf');
if ~exist(OUT_DIR, 'dir')
    mkdir(OUT_DIR);
end

%% ========================================================================
%% 1. NONLINEAR SUSPENDED-LOAD MODEL (same state ordering as supplied file)
%% ========================================================================
syms p1 p2 p3 dp1 dp2 dp3 ddp1 ddp2 ddp3 q0 q1 q2 q3 o1 o2 o3 real
syms u u1 u2 u3 u4 T1 T2 T3 T4 real
syms m Lx Ly lx ly jx jy jz gravity km1 km2 km3 km4 k1 k2 k3 k4 real
syms mL l Length cableL rotor_r real
syms pl1 pl2 pl3 dpl1 dpl2 dpl3 ol1 ol2 ol3 real
syms pT1 pT2 pT3 real
syms obj
syms dstx dsty real

syms xd1(t) xd2(t) xd3(t) xd4(t) v1(t)
syms t real

xd = [xd1(t), xd2(t), xd3(t), xd4(t)];

p   = [p1;p2;p3]; %#ok<NASGU>
dp  = [dp1;dp2;dp3]; %#ok<NASGU>
ddp = [ddp1;ddp2;ddp3]; %#ok<NASGU>
q   = [q0;q1;q2;q3];
ob  = [o1;o2;o3];
pl  = [pl1;pl2;pl3];
dpl = [dpl1;dpl2;dpl3];
ol  = [ol1;ol2;ol3];
pT  = [pT1;pT2;pT3];

x = [q;ob;pl;dpl;pT;ol];

physicalParam_full = [m,Lx,Ly,lx,ly,jx,jy,jz,gravity,km1,km2,km3,km4,k1,k2,k3,k4,rotor_r,mL,cableL,dstx,dsty];

% These two project functions are the same ones used by the supplied HLC
% generator and must be on the MATLAB path.
f = FLxyDst(x,physicalParam_full);
g = GLxyDst(x,physicalParam_full);

physicalParam = [m,jx,jy,jz,gravity,mL,cableL,dstx,dsty];

%% ========================================================================
%% 2. REBUILD HLC FIRST LAYER AND SECOND-LAYER AFFINE SYSTEM
%% ========================================================================
h1     = pl3 - xd(3);
dh1    = LieD(h1,f,x) + diff(h1,t);
alpha1 = LieD(dh1,f,x) + diff(dh1,t);
beta1  = LieD(dh1,g,x);

H = [1/beta1(1), -beta1(2)/beta1(1), -beta1(3)/beta1(1), -beta1(4)/beta1(1); ...
     zeros(3,1), eye(3)];

FG = f + g*H*[-alpha1 + v1(t); u2; u3; u4];
g1 = MyCoeff(FG,[u2;u3;u4]);
f1 = subs(FG,[u2,u3,u4],[0,0,0]);

%% Existing 2nd-layer outputs: needed to generate alpha2/beta2 mapping
h2 = pl1 - xd(1);
h3 = pl2 - xd(2);
[~,~,yaw] = Quat2Eul(q);
h4 = yaw - xd(4);

% x / y relative-degree chains
dh2  = Dbar(h2, f1, x, t);
dh3  = Dbar(h3, f1, x, t);
d2h2 = Dbar(dh2,f1,x,t);
d2h3 = Dbar(dh3,f1,x,t);
d3h2 = Dbar(d2h2,f1,x,t);
d3h3 = Dbar(d2h3,f1,x,t);
d4h2 = Dbar(d3h2,f1,x,t);
d4h3 = Dbar(d3h3,f1,x,t);
d5h2 = Dbar(d4h2,f1,x,t);
d5h3 = Dbar(d4h3,f1,x,t);

% yaw chain
dh4 = Dbar(h4,f1,x,t);

alpha2 = [Dbar(d5h2,f1,x,t); ...
          Dbar(d5h3,f1,x,t); ...
          Dbar(dh4, f1,x,t)];

beta2 = [LieD(d5h2,g1,x); ...
         LieD(d5h3,g1,x); ...
         LieD(dh4, g1,x)];

%% ========================================================================
%% 3. NUMERIC-SUBSTITUTION SYMBOLS (same convention as supplied generator)
%% ========================================================================
% Reference derivatives
syms Xd1 Xd2 Xd3 Xd4 dXd1 dXd2 dXd3 dXd4 real
syms d2Xd1 d2Xd2 d2Xd3 d2Xd4 d3Xd1 d3Xd2 d3Xd3 d3Xd4 real
syms d4Xd1 d4Xd2 d4Xd3 d4Xd4 d5Xd1 d5Xd2 d5Xd3 d5Xd4 real
syms d6Xd1 d6Xd2 d6Xd3 d6Xd4 real

XD = {Xd1 Xd2 Xd3 Xd4, ...
      dXd1 dXd2 dXd3 dXd4, ...
      d2Xd1 d2Xd2 d2Xd3 d2Xd4, ...
      d3Xd1 d3Xd2 d3Xd3 d3Xd4, ...
      d4Xd1 d4Xd2 d4Xd3 d4Xd4, ...
      d5Xd1 d5Xd2 d5Xd3 d5Xd4, ...
      d6Xd1 d6Xd2 d6Xd3 d6Xd4};

xdRef = [xd, diff(xd,t), diff(xd,t,2), diff(xd,t,3), ...
         diff(xd,t,4), diff(xd,t,5), diff(xd,t,6)];

% First-layer virtual input and its derivatives
syms V1 dV1 d2V1 d3V1 d4V1 d5V1 real
V1v = {V1 dV1 d2V1 d3V1 d4V1 d5V1};
vInput1 = [v1(t), diff(v1(t),t), diff(v1(t),t,2), ...
           diff(v1(t),t,3), diff(v1(t),t,4), diff(v1(t),t,5)];

% Preserve the same reverse-substitution convention used in the supplied file
XDf       = flip(XD);
V1vf      = flip(V1v);
xdReff    = flip(xdRef);
vInput1f  = flip(vInput1);

% HOCBF gains.  Only Kappa(1:rd) are used for a barrier of relative degree rd.
syms kap1 kap2 kap3 kap4 kap5 kap6 positive
Kappa = [kap1 kap2 kap3 kap4 kap5 kap6];

%% ========================================================================
%% 4. ARBITRARILY ORIENTED ELLIPSOID OBSTACLE
%% ========================================================================
% Obstacle pose and semi-axes.  One obstacle is evaluated per function call;
% stack constraints for all currently detected obstacles at runtime.
syms obs_x obs_y obs_z obs_a obs_b obs_c real
syms obs_roll obs_pitch obs_yaw real
syms betaM positive

cObs = [obs_x;obs_y;obs_z];
RObs = RotZYX(obs_roll,obs_pitch,obs_yaw);
SObs = RObs*diag([obs_a^2,obs_b^2,obs_c^2])*RObs.';

% betaM is the outer-Minkowski parameter.  A useful fixed choice is betaM=1.
% A tighter minimum-trace choice for two fixed-shape ellipsoids is
% betaM = sqrt(trace(Ssys)/trace(SObs)); rotation does not change the trace.
ObsParam = [obs_x obs_y obs_z obs_a obs_b obs_c obs_roll obs_pitch obs_yaw betaM];

%% ========================================================================
%% 5. SYSTEM GEOMETRY: COMMON CENTERS
%% ========================================================================
% In the supplied implementation pT=(pL-pU)/||pL-pU||, therefore
% pU = pL - cableL*pT.
pU   = pl - cableL*pT;
pMid = pl - (cableL/2)*pT;

%% ========================================================================
%% 6. MODEL 1: SINGLE PROLATE ELLIPSOID
%% ========================================================================
% Axisymmetric ellipsoid along the cable direction.  This avoids constructing
% an arbitrary transverse basis and is smooth in pT.
syms single_a single_b positive
Ssingle = single_b^2*eye(3) + (single_a^2-single_b^2)*(pT*pT.');
csingle = pMid;

h_single = EllipsoidSeparationBarrier(csingle,Ssingle,cObs,SObs,betaM);

%% ========================================================================
%% 7. MODEL 2: GENERIC SPHERE ALONG THE CABLE
%% ========================================================================
% lambdaSphere = 0 -> load center, 1 -> UAV center.
% Call the generated function repeatedly with a set of lambdas/radii to form
% a multiple-sphere approximation.
syms lambdaSphere real
syms sphere_r positive
csphere = pl - lambdaSphere*cableL*pT;
Ssphere = sphere_r^2*eye(3);
h_sphere = EllipsoidSeparationBarrier(csphere,Ssphere,cObs,SObs,betaM);

%% ========================================================================
%% 8. MODEL 3: MULTIPLE ELLIPSOIDS (UAV / CABLE / LOAD)
%% ========================================================================
% UAV ellipsoid follows the vehicle attitude q.
syms uav_a uav_b uav_c positive
Rq = RodriguesQuaternion(q);
Suav = Rq*diag([uav_a^2,uav_b^2,uav_c^2])*Rq.';
h_uav = EllipsoidSeparationBarrier(pU,Suav,cObs,SObs,betaM);

% Cable as an axisymmetric prolate ellipsoid aligned with pT.
syms cable_a cable_b positive
Scable = cable_b^2*eye(3) + (cable_a^2-cable_b^2)*(pT*pT.');
h_cable = EllipsoidSeparationBarrier(pMid,Scable,cObs,SObs,betaM);

% No payload attitude exists in the supplied model: use world-fixed axes.
syms load_a load_b load_c positive
Sload = diag([load_a^2,load_b^2,load_c^2]);
h_load = EllipsoidSeparationBarrier(pl,Sload,cObs,SObs,betaM);

%% ========================================================================
%% 9. RELATIVE-DEGREE AUDIT + HOCBF CONSTRUCTION
%% ========================================================================
% Each result satisfies
%       Au*u234 + bu >= 0,  u234=[u2;u3;u4].
% Runtime mapping to v=[v2;v3;v4]:
%       Av = Au / beta2_num;
%       cv = bu - Av*alpha2_num;
%       Av*v + cv >= 0.

results = struct();

if GENERATE_SINGLE_ELLIPSOID
    fprintf('\n=== MODEL 1: single ellipsoid ===\n');
    results.single = BuildHOCBF(h_single,f1,g1,x,t,Kappa,MAX_RELATIVE_DEGREE,USE_SIMPLIFY_EACH_STEP);
    fprintf('relative degree = %d\n',results.single.rd);
end

if GENERATE_MULTI_SPHERE
    fprintf('\n=== MODEL 2: generic cable sphere ===\n');
    results.sphere = BuildHOCBF(h_sphere,f1,g1,x,t,Kappa,MAX_RELATIVE_DEGREE,USE_SIMPLIFY_EACH_STEP);
    fprintf('relative degree = %d\n',results.sphere.rd);
end

if GENERATE_MULTI_ELLIPSOID
    fprintf('\n=== MODEL 3A: UAV ellipsoid ===\n');
    results.uav = BuildHOCBF(h_uav,f1,g1,x,t,Kappa,MAX_RELATIVE_DEGREE,USE_SIMPLIFY_EACH_STEP);
    fprintf('relative degree = %d\n',results.uav.rd);

    fprintf('\n=== MODEL 3B: cable ellipsoid ===\n');
    results.cable = BuildHOCBF(h_cable,f1,g1,x,t,Kappa,MAX_RELATIVE_DEGREE,USE_SIMPLIFY_EACH_STEP);
    fprintf('relative degree = %d\n',results.cable.rd);

    fprintf('\n=== MODEL 3C: load ellipsoid ===\n');
    results.load = BuildHOCBF(h_load,f1,g1,x,t,Kappa,MAX_RELATIVE_DEGREE,USE_SIMPLIFY_EACH_STEP);
    fprintf('relative degree = %d\n',results.load.rd);
end

%% ========================================================================
%% 10. GENERATE NUMERIC MATLAB FUNCTIONS
%% ========================================================================
fprintf('\nGenerating MATLAB functions in:\n  %s\n',OUT_DIR);

% Common substitution from symbolic time functions to numeric reference/v1 data
SUB_FROM = [xdReff vInput1f];
SUB_TO   = [XDf V1vf];

% Existing HLC mapping data used by the runtime CBF filter.
if GENERATE_ALPHA2
    alpha2_num = subs(alpha2,SUB_FROM,SUB_TO);
    matlabFunction(alpha2_num, ...
        'file',fullfile(OUT_DIR,'CBF_Alpha2_SuspendedLoadxyDst.m'), ...
        'vars',{obj,x,cell2sym(XD),cell2sym(V1v),physicalParam}, ...
        'outputs',{'alpha2'});
end

% Also generate beta2 here so this folder is self-contained for the CBF layer.
beta2_num = subs(beta2,SUB_FROM,SUB_TO);
matlabFunction(beta2_num, ...
    'file',fullfile(OUT_DIR,'CBF_Beta2_SuspendedLoadxyDst.m'), ...
    'vars',{obj,x,cell2sym(XD),cell2sym(V1v),physicalParam}, ...
    'outputs',{'beta2'});

if GENERATE_SINGLE_ELLIPSOID
    geom = [single_a single_b];
    GenerateBarrierFunction(results.single,'CBF_SingleEllipsoid_HOCBF.m', ...
        OUT_DIR,obj,x,XD,V1v,physicalParam,ObsParam,geom,Kappa,SUB_FROM,SUB_TO);
end

if GENERATE_MULTI_SPHERE
    geom = [lambdaSphere sphere_r];
    GenerateBarrierFunction(results.sphere,'CBF_GenericSphere_HOCBF.m', ...
        OUT_DIR,obj,x,XD,V1v,physicalParam,ObsParam,geom,Kappa,SUB_FROM,SUB_TO);
end

if GENERATE_MULTI_ELLIPSOID
    GenerateBarrierFunction(results.uav,'CBF_UAVEllipsoid_HOCBF.m', ...
        OUT_DIR,obj,x,XD,V1v,physicalParam,ObsParam,[uav_a uav_b uav_c],Kappa,SUB_FROM,SUB_TO);

    GenerateBarrierFunction(results.cable,'CBF_CableEllipsoid_HOCBF.m', ...
        OUT_DIR,obj,x,XD,V1v,physicalParam,ObsParam,[cable_a cable_b],Kappa,SUB_FROM,SUB_TO);

    GenerateBarrierFunction(results.load,'CBF_LoadEllipsoid_HOCBF.m', ...
        OUT_DIR,obj,x,XD,V1v,physicalParam,ObsParam,[load_a load_b load_c],Kappa,SUB_FROM,SUB_TO);
end

%% Save metadata for checking which relative degree MATLAB found.
meta = struct();
fn = fieldnames(results);
for i=1:numel(fn)
    meta.(fn{i}).relative_degree = results.(fn{i}).rd;
end
save(fullfile(OUT_DIR,'CBF_generation_metadata.mat'),'meta');

fprintf('\nDone.\n');
fprintf(['Runtime CBF conversion:\n' ...
         '  Av = Au / beta2;\n' ...
         '  cv = bu - Av*alpha2;\n' ...
         '  enforce Av*[v2;v3;v4] + cv >= 0.\n']);

%% ========================================================================
%% LOCAL FUNCTIONS
%% ========================================================================
function out = BuildHOCBF(h,f,g,x,t,Kappa,maxRD,useSimplify)
% Determine uniform relative degree against g and build a linear-alpha HOCBF.
%
% psi_0 = h
% psi_i = Dbar(psi_{i-1}) + kappa_i psi_{i-1}, i=1,...,r-1
% final constraint:
%   Au*u + bu >= 0
%   Au = Lg psi_{r-1}
%   bu = Dbar(psi_{r-1}) + kappa_r psi_{r-1}

    phi = h;
    rd = 0;

    for r = 1:maxRD
        Lgphi = LieD(phi,g,x);
        if useSimplify
            Lgphi_test = simplify(Lgphi);
        else
            Lgphi_test = Lgphi;
        end

        if ~IsSymbolicZero(Lgphi_test)
            rd = r;
            break;
        end

        phi = Dbar(phi,f,x,t);
        if useSimplify
            phi = simplify(phi);
        end
    end

    if rd == 0
        error('No control dependence detected through relative degree %d.',maxRD);
    end

    psi = h;
    psi_chain = sym(zeros(rd,1));
    psi_chain(1) = h;

    for i = 1:(rd-1)
        psi = Dbar(psi,f,x,t) + Kappa(i)*psi;
        if useSimplify
            psi = simplify(psi);
        end
        psi_chain(i+1) = psi;
    end

    Au = LieD(psi,g,x);
    bu = Dbar(psi,f,x,t) + Kappa(rd)*psi;

    if useSimplify
        Au = simplify(Au);
        bu = simplify(bu);
    end

    out = struct('h',h,'psi',psi,'Au',Au,'bu',bu,'rd',rd,'psi_chain',psi_chain);
end

function GenerateBarrierFunction(res,fileName,outDir,obj,x,XD,V1v,physicalParam,ObsParam,GeomParam,Kappa,SUB_FROM,SUB_TO)
% Generate one numerical function.  Outputs:
%   h   : geometric barrier value
%   psi : last pre-input HOCBF recursion state psi_{r-1}
%   Au  : row multiplying physical 2nd-layer input [u2 u3 u4]'
%   bu  : drift/HOCBF term, such that Au*u234 + bu >= 0

    h_num   = subs(res.h,  SUB_FROM,SUB_TO);
    psi_num = subs(res.psi,SUB_FROM,SUB_TO);
    Au_num  = subs(res.Au, SUB_FROM,SUB_TO);
    bu_num  = subs(res.bu, SUB_FROM,SUB_TO);

    matlabFunction(h_num,psi_num,Au_num,bu_num, ...
        'file',fullfile(outDir,fileName), ...
        'vars',{obj,x,cell2sym(XD),cell2sym(V1v),physicalParam,ObsParam,GeomParam,Kappa}, ...
        'outputs',{'h','psi','Au','bu'});
end

function h = EllipsoidSeparationBarrier(cA,SA,cB,SB,betaM)
% Conservative ellipsoid-separation barrier using an outer ellipsoid of the
% Minkowski sum.  Ellipsoids use shape-matrix form
%   E(c,S)={c+S^(1/2)z : ||z||<=1}.
%
% Since both ellipsoids are centrally symmetric, collision corresponds to
% cA-cB belonging to E(0,SA) (+) E(0,SB).  The chosen Ssum contains that
% Minkowski sum, so h>=0 is a sufficient collision-free condition.

    Ssum = (1 + 1/betaM)*SA + (1 + betaM)*SB;
    dc = cA-cB;

    % Avoid inv() in the written formula where possible.
    h = dc.'*(Ssum\dc) - 1;
end

function tf = IsSymbolicZero(M)
% Conservative symbolic zero test for a matrix/vector.
    tf = true;
    for k=1:numel(M)
        try
            z = isAlways(M(k)==0,'Unknown','false');
        catch
            z = isequal(M(k),sym(0));
        end
        if ~z
            tf = false;
            return;
        end
    end
end

function y = Dbar(h,f,x,t)
% Total derivative along a time-varying drift f.
    y = LieD(h,f,x) + diff(h,t);
end

function pd = pdiff(flist,vars)
    flist = flist(:);
    vars = vars(:);
    pdiff_tmp = @(flist,vars) arrayfun(@(func) arrayfun(@(xx) diff(func,xx),vars)',flist,'UniformOutput',false);
    pd = cell2sym(pdiff_tmp(flist,vars));
end

function dh = LieD(h,f,x)
    x = x(:);
    dh = pdiff(h,x)*f;
end

function mat = MyCoeff(M,vars)
    vars = vars(:);
    mat = sym(zeros(length(M),length(vars)));
    for i=1:length(M)
        for j=1:length(vars)
            mat(i,j) = subs(M(i)-subs(M(i),vars(j),0),vars(j),1);
        end
    end
end

function [eul,th,psi] = Quat2Eul(q,varargin)
    switch nargin
        case 1
            q0=q(1); q1=q(2); q2=q(3); q3=q(4);
        case 4
            q0=q; q1=varargin{1}; q2=varargin{2}; q3=varargin{3};
    end
    phi = atan2(2*(q0*q1+q2*q3),(q0^2-q1^2-q2^2+q3^2));
    th  = asin(2*(q0*q2-q1*q3));
    psi = atan2(2*(q0*q3+q1*q2),(q0^2+q1^2-q2^2-q3^2));
    switch nargout
        case 1
            eul=[phi,th,psi];
        case 3
            eul=phi;
    end
end

function R = RodriguesQuaternion(q)
    q=q(:);
    q0=q(1); q1=q(2); q2=q(3); q3=q(4);
    E=[-q1,q0,-q3,q2; -q2,q3,q0,-q1; -q3,-q2,q1,q0];
    L=[-q1,q0,q3,-q2; -q2,-q3,q0,q1; -q3,q2,-q1,q0];
    R=E*L.';
end

function R = RotZYX(roll,pitch,yaw)
% World rotation for obstacle orientation: Rz(yaw)*Ry(pitch)*Rx(roll)
    cr=cos(roll);  sr=sin(roll);
    cp=cos(pitch); sp=sin(pitch);
    cy=cos(yaw);   sy=sin(yaw);

    Rx=[1 0 0; 0 cr -sr; 0 sr cr];
    Ry=[cp 0 sp; 0 1 0; -sp 0 cp];
    Rz=[cy -sy 0; sy cy 0; 0 0 1];
    R=Rz*Ry*Rx;
end
