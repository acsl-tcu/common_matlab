function OUT = AF7R34C_CORRIDOR_BRANCH_FORENSIC_AUDIT(replanner)
clc;
fprintf('\n====================================================================================================\n');
fprintf(' AF7R34C | PHYSICAL CORRIDOR-BRANCH / COMMON-OBJECTIVE FORENSIC AUDIT\n');
fprintf('====================================================================================================\n');
fprintf('[AUTHORITY] readOnly=1 | commonMATLABRows=1 | commonMATLABObjective=1 | productionModification=0\n');

assert(isa(replanner,'REPLANNING_SYSTEM'));
assert(exist('af7r34_single_core_mex','file')==3,'[AF7R34C] R14 core MEX missing.');

P0=AF7R34_PRIME_SINGLE_CORE_REFERENCE(replanner);
assert(P0.closed,'[AF7R34C] nominal AST prime did not close.');

E=replanner.get_first_trigger_event();
assert(isstruct(E)&&isfield(E,'valid')&&logical(E.valid)&& ...
    logical(E.real_flight_request_edge),'[AF7R34C] no stored true trigger.');

CTX=af7r32_replay_context(replanner,E);

PL=replanner.builtin_bezier_d15_c7_proposal(CTX);
[PN,RN]=af7r34_single_core_proposal(replanner,CTX);

assert(logical(PL.success)&&logical(PN.success), ...
    '[AF7R34C] both legacy and native proposals must be certified.');
EL=PL.unsafe_episode_certificate;
EN=PN.unsafe_episode_certificate;
assert(double(EL.episode_count)==1 && double(EL.component_count)==1, ...
    '[AF7R34C] this forensic audit expects the current 1-episode / 1-component true trigger.');
assert(double(EN.episode_count)==1 && double(EN.component_count)==1);

ep=EL.episodes(1);
tca=double(ep.start_s);
tcb=double(ep.end_s);

% MATLAB authority for the separator normal and both physical halfspaces.
xa=double(PL.base_flat_eval_fn(tca)); xa=xa(:);
xb=double(PL.base_flat_eval_fn(tcb)); xb=xb(:);
progress=xb(1:3)-xa(1:3);
if norm(progress)<=128*eps(max(1,norm(xa(1:3))))
    xm=double(PL.base_flat_eval_fn(0.5*(tca+tcb))); xm=xm(:);
    progress=xm(5:7);
end

ii=double(ep.obstacle_index);
R=reshape(double(CTX.obstacles.rotations_flat(:,ii)),3,3);
a=double(CTX.obstacles.base_axes(:,ii));
n0=replanner.bezier_lateral_separator_normal(R,a,progress);
n0=double(n0(:))/norm(double(n0(:)));

nvar=6;
[Ap,bp,okp]=replanner.bezier_piecewise_episode_corridor_rows( ...
    PL,CTX,EL,1,n0,nvar);
[Am,bm,okm]=replanner.bezier_piecewise_episode_corridor_rows( ...
    PL,CTX,EL,1,-n0,nvar);
assert(okp&&okm,'[AF7R34C] MATLAB corridor rows failed.');

zL=pack_z(PL);
zN=pack_z(PN);

[sLp,passLp,tolLp]=slack_test(Ap,bp,zL);
[sLm,passLm,tolLm]=slack_test(Am,bm,zL);
[sNp,passNp,tolNp]=slack_test(Ap,bp,zN);
[sNm,passNm,tolNm]=slack_test(Am,bm,zN);

branchL=branch_code(passLp,passLm);
branchN=branch_code(passNp,passNm);

% Common objective evaluation: SAME MATLAB H, SAME arithmetic for both z.
Hs=replanner.piecewise_minimum_snap_hessian( ...
    EL.components,double(PL.t0),double(PL.t1));
H=kron(double(Hs),eye(3));
H=0.5*(H+H.');

JL=0.5*zL.'*H*zL;
JN=0.5*zN.'*H*zN;

% Conservative binary64 rounding envelope for evaluating the already-double
% quadratic form. This is an implementation comparison guard only; it is not
% a safety margin and is never used for collision acceptance.
n=numel(zL);
g=gamma_n(max(1,4*n*n+4*n+16));
absFormL=0.5*sum(abs((zL*zL.').*H),'all');
absFormN=0.5*sum(abs((zN*zN.').*H),'all');
eL=g*max(1,absFormL);
eN=g*max(1,absFormN);
IL=[JL-eL,JL+eL];
IN=[JN-eN,JN+eN];
objectiveIntervalsOverlap=max(IL(1),IN(1))<=min(IL(2),IN(2));

% Compare native branch using the current MATLAB safety authority.
baseSC=replanner.active_speed_certificate;
LIV=CTX.live;
rhoBody=max([double(LIV.r_drone), ...
    double(LIV.cable_length)+double(LIV.r_load), ...
    double(LIV.cable_length)+double(LIV.r_cable)]);
VQlimit=(double(LIV.sensor_range)-rhoBody)/(double(LIV.horizon)+double(LIV.dt));
targetVersion=double(CTX.request.next_reference_version);

[dubTight,DB]=replanner.tighten_piecewise_derivative_bounds_for_speed( ...
    PN.piecewise_segment_t0,PN.piecewise_segment_t1,PN.piecewise_control_points, ...
    baseSC,VQlimit,6);

speedClosed=false;
SC=struct('certified',false);
if isstruct(DB)&&isfield(DB,'certified')&&logical(DB.certified)
    PT=PN;
    PT.correction_derivative_ub=dubTight;
    SCtri=replanner.build_bezier_repaired_speed_certificate(PT,targetVersion);
    if isstruct(SCtri)&&isfield(SCtri,'certified')&&logical(SCtri.certified)&& ...
            double(SCtri.uav_speed_UB_m_s)<VQlimit
        SC=SCtri; speedClosed=true;
    end
end
if ~speedClosed
    [SClocal,~]=replanner.build_piecewise_local_vector_speed_certificate( ...
        PN,PN.piecewise_segment_t0,PN.piecewise_segment_t1, ...
        PN.piecewise_control_points,CTX,baseSC,VQlimit,targetVersion);
    if isstruct(SClocal)&&isfield(SClocal,'certified')&&logical(SClocal.certified)&& ...
            double(SClocal.uav_speed_UB_m_s)<VQlimit
        SC=SClocal; speedClosed=true;
    end
end

PR=PN;
PR.reference_speed_certificate=SC;
rejoinClosed=false; geometryClosed=false; G=struct();
if speedClosed
    rejoinClosed=replanner.reference_rejoin_cell_continuously_safe(PR,PR.t1,CTX);
end
if rejoinClosed
    G=replanner.build_static_continuous_wholebody_shadow_certificate(CTX,PR);
    geometryClosed=isstruct(G)&&isfield(G,'mathematical_pass')&&logical(G.mathematical_pass);
end

% Exact disjunction is invariant to n0 -> -n0 because BOTH orientations are
% enumerated. Therefore a difference of base eigenvector sign is only a label
% change; the physical branch is determined here using the common MATLAB rows.
physicalOpposite=(branchL==1 && branchN==-1) || (branchL==-1 && branchN==1);
physicalSame=(branchL==branchN)&&(abs(branchL)==1);

closed=(abs(branchL)==1)&&(abs(branchN)==1)&& ...
    objectiveIntervalsOverlap&&speedClosed&&rejoinClosed&&geometryClosed;

fprintf(' MATLAB base normal n0                    = [%.17g %.17g %.17g]^T\n',n0);
fprintf(' legacy/native stored sign label          = %.0f / %.0f\n', ...
    double(PL.piecewise_corridor_signs(1)),double(PN.piecewise_corridor_signs(1)));
fprintf('\n[COMMON MATLAB PHYSICAL HALFSPACE TEST]\n');
fprintf(' legacy plus/minus pass                   = %d / %d\n',passLp,passLm);
fprintf(' native plus/minus pass                   = %d / %d\n',passNp,passNm);
fprintf(' legacy physical branch code              = %d  (+1=+n, -1=-n)\n',branchL);
fprintf(' native physical branch code              = %d  (+1=+n, -1=-n)\n',branchN);
fprintf(' physical same / opposite                 = %d / %d\n',physicalSame,physicalOpposite);
fprintf(' legacy plus/minus min slack              = %.17g / %.17g\n',sLp,sLm);
fprintf(' native plus/minus min slack              = %.17g / %.17g\n',sNp,sNm);
fprintf(' feasibility tolerances L+/L-/N+/N-       = %.3g / %.3g / %.3g / %.3g\n', ...
    tolLp,tolLm,tolNp,tolNm);

fprintf('\n[COMMON MATLAB OBJECTIVE]\n');
fprintf(' J(legacy) / J(native)                    = %.17g / %.17g\n',JL,JN);
fprintf(' |Jlegacy-Jnative|                        = %.17g\n',abs(JL-JN));
fprintf(' legacy objective interval                = [%.17g, %.17g]\n',IL(1),IL(2));
fprintf(' native objective interval                = [%.17g, %.17g]\n',IN(1),IN(2));
fprintf(' binary64 objective intervals overlap     = %d\n',objectiveIntervalsOverlap);

fprintf('\n[INDEPENDENT MATLAB SAFETY RE-CERTIFICATION]\n');
fprintf(' speed / rejoin / whole-body              = %d / %d / %d\n', ...
    speedClosed,rejoinClosed,geometryClosed);
if geometryClosed
    fprintf(' native recertified min whole-body margin = %.17g m\n',double(G.min_margin_m));
end

fprintf('\n physical/certificate equivalence closed  = %d\n',closed);
fprintf(' production source modified               = 0\n');
fprintf('====================================================================================================\n');

OUT=struct('schema','AF7R34C_CORRIDOR_BRANCH_FORENSIC_AUDIT_V1', ...
    'closed',closed,'legacy_branch',branchL,'native_branch',branchN, ...
    'physical_same',physicalSame,'physical_opposite',physicalOpposite, ...
    'legacy_objective',JL,'native_objective',JN, ...
    'legacy_objective_interval',IL,'native_objective_interval',IN, ...
    'objective_intervals_overlap',objectiveIntervalsOverlap, ...
    'speed_closed',speedClosed,'rejoin_closed',rejoinClosed, ...
    'geometry_closed',geometryClosed,'native_geometry',G, ...
    'native_raw',RN);
assignin('base','AF7R34C_CORRIDOR_BRANCH_FORENSIC_AUDIT_OUT',OUT);
end

function z=pack_z(P)
p=double(P.piecewise_component_position);
v=double(P.piecewise_component_velocity);
assert(size(p,2)==1&&size(v,2)==1,'[AF7R34C] current audit expects one component.');
z=[p(:,1);v(:,1)];
end

function [m,pass,tol]=slack_test(A,b,z)
A=double(A); b=double(b(:)); z=double(z(:));
q=A*z;
sl=q-b;
scale=max([1;abs(q(:));abs(b(:))]);
tol=512*eps(scale);
m=min(sl);
pass=all(sl>=-tol);
end

function c=branch_code(pp,pm)
if pp&&~pm, c=1;
elseif pm&&~pp, c=-1;
elseif pp&&pm, c=2;
else, c=0;
end
end

function g=gamma_n(n)
u=eps/2;
nu=n*u;
if nu>=1
    g=Inf;
else
    g=nu/(1-nu);
end
end
