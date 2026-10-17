function OUT = AF7R57G_NATIVE_WITNESS_CONSISTENCY_AUDIT()
% AF7R57G_NATIVE_WITNESS_CONSISTENCY_AUDIT
% -----------------------------------------------------------------------------
% Resolve the apparent contradiction:
%
%   AF7R56B lower endpoint:
%       finite witness > VQ limit
%
%   AF7R57F V3:
%       full bracket outward Stage-B safe
%
% The native adaptive Stage-B implementation repeatedly split the correction
% control polygon and then differentiated the tiny split leaf.  At the lower
% endpoint, the witness leaf is extremely small.  This audit compares:
%
%   A) native split-leaf evaluation (exactly reproduces AF7R56B storage)
%   B) direct evaluation of the ORIGINAL candidate segment at the same time
%   C) stable derivative-first Stage-B bound on the same physical leaf
%   D) R57F V3 outward interval bound on the same leaf
%
% No production state/source is modified and no t1 is selected.
% -----------------------------------------------------------------------------

clc;
fprintf('\n====================================================================================================\n');
fprintf(' AF7R57G V4 | C++17 NATIVE CANDIDATE PROBE / WITNESS CONSISTENCY AUDIT\n');
fprintf('====================================================================================================\n');
fprintf('[AUTHORITY] readOnly=1 | nativeAllDiagProbe=1 | sameRun=1 | sampledAcceptance=0 | productionModified=0\n');

OUT=struct();
OUT.schema='AF7R57G_NATIVE_WITNESS_CONSISTENCY_AUDIT_V4';
OUT.closed=false;
OUT.production_modified=false;
OUT.production_t1_selected=false;
OUT.continuous_terminal_boundary_proof=false;

%% A. Authorities
fprintf('\n[A. AUTHORITIES]\n');

C=af7r57g_get_C();
assert(isstruct(C)&&isfield(C,'closed')&&logical(C.closed), ...
    '[AF7R57G] AF7R57C authority not closed.');

assert(evalin('base','exist(''replanner_instance'',''var'')==1'), ...
    '[AF7R57G] replanner_instance missing.');
replanner=evalin('base','replanner_instance');
assert(isa(replanner,'REPLANNING_SYSTEM'));

[E,M,AUX,authoritySource]=AF7R52_LOAD_TRUE_TRIGGER_AUTHORITY(replanner); %#ok<ASGLU>
assert(isstruct(E)&&isfield(E,'valid')&&logical(E.valid));
assert(isstruct(M)&&isfield(M,'certified')&&logical(M.certified));

P=AF7R56B_PRIME_TRACE_REFERENCE(replanner,M);
assert(isstruct(P)&&isfield(P,'closed')&&logical(P.closed));

fprintf(' AF7R57C closed                          = 1\n');
fprintf(' AF7R52 authority source                 = %s\n',char(string(authoritySource)));
fprintf(' native trace prime closed               = 1\n');

TR=C.trace_records;
KR=C.kkt_records;

r1=af7r57g_rec(TR,1);
k1=af7r57g_rec(KR,1);
r2=af7r57g_rec(TR,2);

pkt=r1.pkt;
lo=double(r1.lo);
hi=double(r1.hi);
tU=double(r1.tU);

fprintf(' bracket                                 = [%.17g %.17g]\n',lo,hi);

%% B. Build diagnostics-only shadow trace probe + endpoint replays
fprintf('\n[B. NATIVE all_diag CANDIDATE PROBE]\n');

src=af7r57g_find_source('af7r56_trace_mex.cpp');
assert(~isempty(src)&&isfile(src),'[AF7R57G] af7r56_trace_mex.cpp missing.');
srcHash0=af7r57g_sha256_file(src);
expected='cef987c012982982d2fc4381419781770fe9304c6a2ba4f1d7cff1bba2746f23';
fprintf(' production trace source                 = %s\n',src);
fprintf(' production trace SHA-256                = %s\n',srcHash0);
fprintf(' expected source SHA equal               = %d\n',strcmpi(srcHash0,expected));
assert(strcmpi(srcHash0,expected),'[AF7R57G] production trace source changed.');

root=fileparts(mfilename('fullpath'));
shadowCpp=fullfile(root,'af7r57g_trace_probe_mex.cpp');
probeName='af7r57g_trace_probe_mex';
af7r57g_write_probe_source(src,shadowCpp);

needBuild=exist(probeName,'file')~=3;
if ~needBuild
    q=which(probeName);
    needBuild=isempty(q)||dir(q).datenum<dir(shadowCpp).datenum;
end
if needBuild
    inc=['-I' fileparts(src)];

    % af7r56_trace_core.hpp uses C++17 language features.  The production
    % trace MEX was built with that language level, so the diagnostics-only
    % shadow translation unit must use the same minimum standard.
    if ispc
        stdflag='COMPFLAGS=$COMPFLAGS /std:c++17';
    else
        stdflag='CXXFLAGS=$CXXFLAGS -std=c++17';
    end

    fprintf(' shadow compiler standard                = C++17\n');
    fprintf(' shadow compiler flag                    = %s\n',stdflag);

    mex('-R2018a',inc,stdflag,'-outdir',root,'-output',probeName,shadowCpp);
    rehash;
end
assert(exist(probeName,'file')==3,'[AF7R57G] shadow candidate probe build failed.');

srcHash1=af7r57g_sha256_file(src);
fprintf(' shadow probe MEX available              = 1\n');
fprintf(' production source SHA unchanged         = %d\n',strcmpi(srcHash0,srcHash1));
assert(strcmpi(srcHash0,srcHash1));

[F,expr,signature]=af7r57g_nominal_ast(replanner,M);
ok=af7r57g_trace_probe_mex(0,expr,signature);
assert(logical(ok),'[AF7R57G] shadow probe nominal AST prime failed.');
fprintf(' shadow nominal AST prime                = 1\n');
fprintf(' shadow nominal signature                = %s\n',signature);

RL=af7r57g_trace_probe_mex(4,pkt,tU,lo);
RU=af7r57g_trace_probe_mex(4,pkt,tU,hi);

fprintf(' lower success / reason                  = %d / %.0f\n',logical(RL.success),double(RL.reason_code));
fprintf(' upper success / reason                  = %d / %.0f\n',logical(RU.success),double(RU.reason_code));
fprintf(' lower selected component_position empty = %d\n',isempty(RL.component_position));
fprintf(' lower all-candidate count               = %.0f\n',double(RL.all_candidate_count));
fprintf(' upper all-candidate count               = %.0f\n',double(RU.all_candidate_count));
assert(double(RL.all_candidate_count)>=1&&double(RU.all_candidate_count)>=1);

zL=double(RL.all_candidate_z{1}(:));
zU=double(RU.all_candidate_z{1}(:));
zI=af7r57g_normalize_interval_matrix(double(k1.K{1}.z_interval),numel(zL),'K.z_interval');
zLin=all(zL>=zI(:,1)&zL<=zI(:,2));
zUin=all(zU>=zI(:,1)&zU<=zI(:,2));

signL=double(RL.all_candidate_signs{1}(:).');
signU=double(RU.all_candidate_signs{1}(:).');
actL=double(RL.all_candidate_active_set{1}(:).');
actU=double(RU.all_candidate_active_set{1}(:).');
lamL=double(RL.all_candidate_lambda{1}(:));
lamU=double(RU.all_candidate_lambda{1}(:));
lamI=double(k1.K{1}.lambda(:).');
activeTarget=double(k1.K{1}.active_index)-1;

signClosed=isequal(signL,double(k1.K{1}.sign_tag))&&isequal(signU,double(k1.K{1}.sign_tag));
activeClosed=isequal(actL,activeTarget)&&isequal(actU,activeTarget);
lambdaClosed=all(lamL>=lamI(1)&lamL<=lamI(2))&&all(lamU>=lamI(1)&lamU<=lamI(2));

TL=r1.TL{1};
traceDiagClosed=logical(RL.all_candidate_finite_witness(1))&& ...
    af7r57g_same_number(double(RL.all_candidate_witness_max(1)),double(TL.witness_max));

fprintf(' size(zL) / size(zU)                     = %s / %s\n',mat2str(size(zL)),mat2str(size(zU)));
fprintf(' normalized size(K.z_interval)           = %s\n',mat2str(size(zI)));
fprintf(' lower candidate z inside KKT box        = %d\n',zLin);
fprintf(' upper candidate z inside KKT box        = %d\n',zUin);
fprintf(' lower / upper sign                      = %s / %s\n',mat2str(signL),mat2str(signU));
fprintf(' KKT sign tag                            = %.0f\n',double(k1.K{1}.sign_tag));
fprintf(' lower / upper active(native)            = %s / %s\n',mat2str(actL),mat2str(actU));
fprintf(' KKT active(native)                      = %.0f\n',activeTarget);
fprintf(' lower / upper lambda                    = %s / %s\n',mat2str(lamL,17),mat2str(lamU,17));
fprintf(' KKT lambda interval                     = %s\n',mat2str(lamI,17));
fprintf(' lower all_diag witness / trace witness  = %.17g / %.17g\n',double(RL.all_candidate_witness_max(1)),double(TL.witness_max));

branchIdentity=zLin&&zUin&&signClosed&&activeClosed&&lambdaClosed&&traceDiagClosed;
fprintf(' candidate-1 native branch identity      = %d\n',branchIdentity);

seg0L=double(RL.all_candidate_segment_t0{1}(:).');
seg1L=double(RL.all_candidate_segment_t1{1}(:).');
seg0U=double(RU.all_candidate_segment_t0{1}(:).');
seg1U=double(RU.all_candidate_segment_t1{1}(:).');
cpL=RL.all_candidate_control_points{1};
cpU=RU.all_candidate_control_points{1};
segTraceClosed=isequal(seg0L,double(TL.candidate_seg0(:).'))&& ...
    isequal(seg1L,double(TL.candidate_seg1(:).'))&& ...
    isequal(seg0U,double(r1.TU{1}.candidate_seg0(:).'))&& ...
    isequal(seg1U,double(r1.TU{1}.candidate_seg1(:).'));
fprintf(' lower / upper native segment count      = %d / %d\n',numel(cpL),numel(cpU));
fprintf(' candidate segment arrays match trace    = %d\n',segTraceClosed);
assert(segTraceClosed,'[AF7R57G] native all_diag candidate does not match trace segments.');

%% C. Lower native finite-witness leaf
fprintf('\n[C. LOWER NATIVE FINITE-WITNESS LEAF]\n');

TL=r1.TL{1};
[witStored,iw]=max(double(TL.leaf_witness(:)));

seg=double(TL.leaf_seg(iw));       % native 0-based segment
u0=double(TL.leaf_u0(iw));
u1=double(TL.leaf_u1(iw));
ta=double(TL.leaf_ta(iw));
tb=double(TL.leaf_tb(iw));
cpLeaf=double(TL.leaf_control_points{iw});

fprintf(' witness leaf index                     = %d\n',iw);
fprintf(' segment / u                            = %.0f / [%.17g %.17g]\n',seg,u0,u1);
fprintf(' leaf time                              = [%.17g %.17g]\n',ta,tb);
fprintf(' leaf duration                          = %.17g\n',tb-ta);
fprintf(' stored witness                         = %.17g\n',witStored);
fprintf(' Stage-B limit                          = %.17g\n',double(TL.limit));

%% D. Reproduce native split-leaf arithmetic exactly
fprintf('\n[D. NATIVE SPLIT-LEAF REPRODUCTION]\n');

F=replanner.nominal_flat_derivative_fn(1:7);
assert(iscell(F)&&numel(F)>=7);

Nsplit=af7r57g_leaf_from_cp(cpLeaf,ta,tb,pkt,F);

stored=struct( ...
    'vl',double(TL.leaf_vl(iw)), ...
    'al',double(TL.leaf_al(iw)), ...
    'jl',double(TL.leaf_jl(iw)), ...
    'sigma',double(TL.leaf_sigma(iw)), ...
    'qdot',double(TL.leaf_qdot(iw)), ...
    'vq',double(TL.leaf_vq(iw)), ...
    'witness',double(TL.leaf_witness(iw)));

fn={'vl','al','jl','sigma','qdot','vq','witness'};
repro=true;
for j=1:numel(fn)
    a=stored.(fn{j});
    b=Nsplit.(fn{j});
    ok=af7r57g_same_number(a,b);
    repro=repro&&ok;
    fprintf(' %-8s stored / reproduced / match       = %.17g / %.17g / %d\n', ...
        fn{j},a,b,ok);
end
fprintf(' native split-leaf reproduction closed   = %d\n',repro);

%% E. Direct ORIGINAL-candidate evaluation at the same physical time
fprintf('\n[E. DIRECT ORIGINAL-CANDIDATE EVALUATION]\n');

assert(seg+1<=numel(cpL),'[AF7R57G] native segment index invalid.');
cpParent=double(cpL{seg+1});
s0=double(seg0L(seg+1));
s1=double(seg1L(seg+1));
Ts=s1-s0;
tc=0.5*(ta+tb);
u=(tc-s0)/Ts;

D1=af7r57g_deriv_cp(cpParent,Ts,1);
D2=af7r57g_deriv_cp(cpParent,Ts,2);
D3=af7r57g_deriv_cp(cpParent,Ts,3);

d1direct=af7r57g_decasteljau(D1,u);
d2direct=af7r57g_decasteljau(D2,u);
d3direct=af7r57g_decasteljau(D3,u);

Tl=tb-ta;
L1=af7r57g_deriv_cp(cpLeaf,Tl,1);
L2=af7r57g_deriv_cp(cpLeaf,Tl,2);
L3=af7r57g_deriv_cp(cpLeaf,Tl,3);
d1split=af7r57g_decasteljau(L1,0.5);
d2split=af7r57g_decasteljau(L2,0.5);
d3split=af7r57g_decasteljau(L3,0.5);

tau=max(0,tc-double(pkt.flight_t0));
v0=double(F{2}(tau));v0=v0(:);
a0=double(F{3}(tau));a0=a0(:);
j0=double(F{4}(tau));j0=j0(:);

Wdirect=af7r57g_physical_witness(v0,a0,j0,d1direct,d2direct,d3direct, ...
    double(pkt.gravity),double(pkt.cable_length));
Wsplit=af7r57g_physical_witness(v0,a0,j0,d1split,d2split,d3split, ...
    double(pkt.gravity),double(pkt.cable_length));

fprintf(' midpoint tc                            = %.17g\n',tc);
fprintf(' parent segment u                       = %.17g\n',u);
fprintf(' ||d1_split-d1_direct||                 = %.17g\n',norm(d1split-d1direct));
fprintf(' ||d2_split-d2_direct||                 = %.17g\n',norm(d2split-d2direct));
fprintf(' ||d3_split-d3_direct||                 = %.17g\n',norm(d3split-d3direct));
fprintf(' stored split witness                   = %.17g\n',witStored);
fprintf(' reproduced split witness               = %.17g\n',Wsplit);
fprintf(' direct original-candidate witness      = %.17g\n',Wdirect);
fprintf(' direct witness - limit                 = %.17g\n',Wdirect-double(TL.limit));
fprintf(' split witness - direct witness         = %.17g\n',Wsplit-Wdirect);

directSafe=Wdirect<double(TL.limit);

%% F. Stable derivative-first bound on exactly the same physical leaf
fprintf('\n[F. STABLE DERIVATIVE-FIRST LEAF BOUND]\n');

S=af7r57g_stable_leaf_bound(cpParent,s0,s1,u0,u1,pkt,F);

fprintf(' stable VL                              = %.17g\n',S.vl);
fprintf(' stable JL                              = %.17g\n',S.jl);
fprintf(' stable sigma                           = %.17g\n',S.sigma);
fprintf(' stable qdot                            = %.17g\n',S.qdot);
fprintf(' stable VQ                              = %.17g\n',S.vq);
fprintf(' stable VQ - limit                      = %.17g\n',S.vq-double(TL.limit));
fprintf(' stable VQ >= direct witness            = %d\n',S.vq>=Wdirect);
fprintf(' stable VQ < limit                      = %d\n',S.vq<double(TL.limit));

stableClosed=isfinite(S.vq)&&S.sigma>0&&S.vq>=Wdirect&&S.vq<double(TL.limit);

%% G. R57F V3 outward interval on the native witness leaf
fprintf('\n[G. R57F V3 OUTWARD CONTAINMENT]\n');

assert(exist('af7r57f_v3_interval_stageb_mex','file')==3, ...
    '[AF7R57G] af7r57f_v3_interval_stageb_mex missing.');

U=k1.U;
cs=double(U.component_start(:).');
ce=double(U.component_end(:).');
tree=[seg u0 u1];
tIp=[lo lo];

Tm=af7r57f_v3_interval_stageb_mex(0,tIp,double(U.t0),cs,ce,tree,0);

VC=zeros(3,1);AC=zeros(3,1);
tauI=max(0,double(Tm.tc_center)-double(pkt.flight_t0));
VC(:,1)=double(F{2}(tauI));VC(:,1)=VC(:,1);
AC(:,1)=double(F{3}(tauI));AC(:,1)=AC(:,1);
vRad=double(pkt.base_payload_accel)*double(Tm.tc_radius);
aRad=double(pkt.base_payload_jerk)*double(Tm.tc_radius);

zPoint=[zL zL];
RI=af7r57f_v3_interval_stageb_mex(1,zPoint,tIp,double(U.t0),cs,ce,tree, ...
    double(pkt.base_payload_accel),double(pkt.base_payload_jerk), ...
    double(pkt.gravity),double(pkt.cable_length),VC,AC,vRad,aRad, ...
    double(TL.limit),0);

fprintf(' interval VL                            = [%.17g %.17g]\n',RI.vl(1,1),RI.vl(1,2));
fprintf(' interval JL                            = [%.17g %.17g]\n',RI.jl(1,1),RI.jl(1,2));
fprintf(' interval sigma                         = [%.17g %.17g]\n',RI.sigma(1,1),RI.sigma(1,2));
fprintf(' interval VQ                            = [%.17g %.17g]\n',RI.vq(1,1),RI.vq(1,2));
fprintf(' interval G                             = [%.17g %.17g]\n',RI.G(1,1),RI.G(1,2));
fprintf(' interval VQ upper >= direct witness    = %d\n',RI.vq(1,2)>=Wdirect);
fprintf(' interval VQ upper < limit              = %d\n',RI.vq(1,2)<double(TL.limit));

intervalClosed=RI.sigma(1,1)>0 && RI.vq(1,2)>=Wdirect && ...
    RI.vq(1,2)<double(TL.limit);

%% H. Candidate-2 stable violation control
fprintf('\n[H. CANDIDATE-2 CONTROL]\n');

TL2=r2.TL{2};
[w2,i2]=max(double(TL2.leaf_witness(:)));
du2=double(TL2.leaf_u1(i2)-TL2.leaf_u0(i2));
fprintf(' candidate-2 max witness                = %.17g\n',w2);
fprintf(' candidate-2 witness-limit              = %.17g\n',w2-double(TL2.limit));
fprintf(' candidate-2 witness leaf du            = %.17g\n',du2);
fprintf(' candidate-2 split count                = %.0f\n',double(TL2.split_count));

%% I. Formal diagnosis
fprintf('\n[I. FORMAL DIAGNOSIS]\n');

splitArtifact = branchIdentity && segTraceClosed && repro && directSafe && stableClosed && ...
    intervalClosed && Wsplit>=double(TL.limit) && ...
    abs(Wsplit-witStored)<=1e3*eps(max(1,abs(witStored)));

fprintf(' candidate-1 branch identity closed     = %d\n',branchIdentity);
fprintf(' native parent/trace segment identity    = %d\n',segTraceClosed);
fprintf(' native split reproduction closed       = %d\n',repro);
fprintf(' direct original candidate is safe      = %d\n',directSafe);
fprintf(' stable leaf bound closed               = %d\n',stableClosed);
fprintf(' outward interval contains direct eval  = %d\n',intervalClosed);
fprintf(' lower native witness is split artifact = %d\n',splitArtifact);

if splitArtifact
    diagnosis='NUMERICAL_SPLIT_THEN_DIFFERENTIATE_WITNESS_ARTIFACT';
    next='REINSTATE_R57F_V3_WITH_NATIVE_WITNESS_ARTIFACT_CLOSURE';
else
    diagnosis='UNRESOLVED_CONTRADICTION_FAIL_CLOSED';
    next='DO_NOT_SELECT_T1__INVESTIGATE_CANDIDATE_OR_STAGEB_MAPPING';
end

fprintf(' diagnosis                              = %s\n',diagnosis);
fprintf(' continuous terminal-boundary proof     = 0\n');
fprintf(' production t1 selected                 = 0\n');
fprintf(' production source modified             = 0\n');
fprintf(' next action                            = %s\n',next);
fprintf('====================================================================================================\n');

OUT.closed=splitArtifact;
OUT.shadow_cpp_standard='C++17';
OUT.diagnosis=diagnosis;
OUT.branch_identity_closed=branchIdentity;
OUT.parent_trace_identity_closed=segTraceClosed;
OUT.native_split_reproduction_closed=repro;
OUT.direct_witness=Wdirect;
OUT.split_witness=Wsplit;
OUT.stable_bound=S;
OUT.interval_bound=RI;
OUT.production_t1_selected=false;
OUT.production_modified=false;
OUT.continuous_terminal_boundary_proof=false;
OUT.next_action=next;

assignin('base','AF7R57G_NATIVE_WITNESS_CONSISTENCY_AUDIT_OUT',OUT);
save('AF7R57G_NATIVE_WITNESS_CONSISTENCY_AUDIT_OUT.mat','OUT','-v7.3');
end


function C=af7r57g_get_C()
if evalin('base','exist(''AF7R57C_SNAPSHOT_BOUND_SCHEMA_CAPTURE_OUT'',''var'')==1')
    C=evalin('base','AF7R57C_SNAPSHOT_BOUND_SCHEMA_CAPTURE_OUT');
    return;
end
p=which('AF7R57C_SNAPSHOT_BOUND_SCHEMA_CAPTURE_OUT.mat');
assert(~isempty(p)&&isfile(p),'[AF7R57G] AF7R57C authority unavailable.');
S=load(p,'OUT');C=S.OUT;
end


function r=af7r57g_rec(R,c)
ids=cellfun(@(q)double(q.c),R);
k=find(ids==c,1,'last');
assert(~isempty(k));r=R{k};
end


function D=af7r57g_deriv_cp(P,T,order)
D=double(P);
deg=size(D,2)-1;
for r=1:order
    assert(T>0&&size(D,2)>=2);
    D=(D(:,2:end)-D(:,1:end-1))*(deg/T);
    deg=deg-1;
end
end


function y=af7r57g_decasteljau(P,u)
D=double(P);
while size(D,2)>1
    D=D(:,1:end-1)*(1-u)+D(:,2:end)*u;
end
y=D(:,1);
end


function [L,R]=af7r57g_split(P,u)
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


function Q=af7r57g_restrict(P,u0,u1)
if u0==0&&u1==1
    Q=P;return;
end
[L,~]=af7r57g_split(P,u1);
if u0==0
    Q=L;return;
end
alpha=u0/u1;
[~,Q]=af7r57g_split(L,alpha);
end


function R=af7r57g_leaf_from_cp(C0,ta,tb,pkt,F)
T=tb-ta;h=0.5*T;tc=0.5*(ta+tb);
C1=af7r57g_deriv_cp(C0,T,1);
C2=af7r57g_deriv_cp(C0,T,2);
C3=af7r57g_deriv_cp(C0,T,3);

tau=max(0,tc-double(pkt.flight_t0));
v0=double(F{2}(tau));v0=v0(:);
a0=double(F{3}(tau));a0=a0(:);
j0=double(F{4}(tau));j0=j0(:);

R.vl=max(vecnorm(C1+v0,2,1))+double(pkt.base_payload_accel)*h;
R.al=max(vecnorm(C2+a0,2,1))+double(pkt.base_payload_jerk)*h;
d1=af7r57g_decasteljau(C1,0.5);
d2=af7r57g_decasteljau(C2,0.5);
d3=af7r57g_decasteljau(C3,0.5);
R.jl=double(pkt.base_payload_jerk)+max(vecnorm(C3,2,1));
sm=a0+d2+[0;0;double(pkt.gravity)];
R.sigma=norm(sm)-R.jl*h;
if isfinite(R.sigma)&&R.sigma>0
    R.qdot=R.jl/R.sigma;
    R.vq=R.vl+double(pkt.cable_length)*R.qdot;
else
    R.qdot=Inf;R.vq=Inf;
end
R.witness=af7r57g_physical_witness(v0,a0,j0,d1,d2,d3, ...
    double(pkt.gravity),double(pkt.cable_length));
end


function W=af7r57g_physical_witness(v0,a0,j0,d1,d2,d3,g,L)
vm=v0+d1;am=a0+d2;jm=j0+d3;
s=am+[0;0;g];ns=norm(s);
if ~(isfinite(ns)&&ns>0),W=NaN;return;end
q=-s/ns;
proj=jm-q*(q.'*jm);
qd=-proj/ns;
W=norm(vm-qd*L);
end


function S=af7r57g_stable_leaf_bound(P,s0,s1,u0,u1,pkt,F)
T=s1-s0;
D1=af7r57g_deriv_cp(P,T,1);
D2=af7r57g_deriv_cp(P,T,2);
D3=af7r57g_deriv_cp(P,T,3);
C1=af7r57g_restrict(D1,u0,u1);
C2=af7r57g_restrict(D2,u0,u1);
C3=af7r57g_restrict(D3,u0,u1);

ta=s0+(s1-s0)*u0;
tb=s0+(s1-s0)*u1;
h=0.5*(tb-ta);tc=0.5*(ta+tb);
tau=max(0,tc-double(pkt.flight_t0));
v0=double(F{2}(tau));v0=v0(:);
a0=double(F{3}(tau));a0=a0(:);

S.vl=max(vecnorm(C1+v0,2,1))+double(pkt.base_payload_accel)*h;
S.al=max(vecnorm(C2+a0,2,1))+double(pkt.base_payload_jerk)*h;
S.jl=double(pkt.base_payload_jerk)+max(vecnorm(C3,2,1));
d2=af7r57g_decasteljau(C2,0.5);
sm=a0+d2+[0;0;double(pkt.gravity)];
S.sigma=norm(sm)-S.jl*h;
if S.sigma>0
    S.qdot=S.jl/S.sigma;
    S.vq=S.vl+double(pkt.cable_length)*S.qdot;
else
    S.qdot=Inf;S.vq=Inf;
end
end


function af7r57g_write_probe_source(src,dst)
raw=fileread(src);
needle1='"all_candidate_geometry_pass","all_candidate_geometry_margin",';
insert1=['"all_candidate_geometry_pass","all_candidate_geometry_margin",' ...
    '"all_candidate_z","all_candidate_segment_t0","all_candidate_segment_t1","all_candidate_control_points",'];
assert(af7r57g_count(raw,needle1)==1,'[AF7R57G] probe field-list anchor not unique.');
raw=strrep(raw,needle1,insert1);

needle2='set("all_candidate_geometry_margin",diag_vector(R.all_diag,4));';
probeCode=[needle2 newline ...
'    {' newline ...
'      mwSize NC=(mwSize)R.all_diag.size();' newline ...
'      mxArray* CZ=mxCreateCellMatrix(1,NC);' newline ...
'      mxArray* CS0=mxCreateCellMatrix(1,NC);' newline ...
'      mxArray* CS1=mxCreateCellMatrix(1,NC);' newline ...
'      mxArray* CCP=mxCreateCellMatrix(1,NC);' newline ...
'      for(mwIndex ci=0;ci<NC;++ci){' newline ...
'        const auto &C=R.all_diag[(size_t)ci].cand;' newline ...
'        mwSize nc=(mwSize)C.pos.size();' newline ...
'        mxArray* Z=mxCreateDoubleMatrix(6*nc,1,mxREAL);double* pz=mxGetPr(Z);' newline ...
'        for(mwIndex j=0;j<nc;++j){' newline ...
'          pz[6*j+0]=C.pos[j].x;pz[6*j+1]=C.pos[j].y;pz[6*j+2]=C.pos[j].z;' newline ...
'          pz[6*j+3]=C.vel[j].x;pz[6*j+4]=C.vel[j].y;pz[6*j+5]=C.vel[j].z;' newline ...
'        }' newline ...
'        mxSetCell(CZ,ci,Z);' newline ...
'        mxArray* A0=mxCreateDoubleMatrix(1,(mwSize)C.seg0.size(),mxREAL);' newline ...
'        mxArray* A1=mxCreateDoubleMatrix(1,(mwSize)C.seg1.size(),mxREAL);' newline ...
'        for(mwIndex j=0;j<(mwSize)C.seg0.size();++j)mxGetPr(A0)[j]=C.seg0[j];' newline ...
'        for(mwIndex j=0;j<(mwSize)C.seg1.size();++j)mxGetPr(A1)[j]=C.seg1[j];' newline ...
'        mxSetCell(CS0,ci,A0);mxSetCell(CS1,ci,A1);' newline ...
'        mxArray* PC=mxCreateCellMatrix(1,(mwSize)C.cp.size());' newline ...
'        for(mwIndex ss=0;ss<(mwSize)C.cp.size();++ss){' newline ...
'          const auto &PP=C.cp[ss];mxArray* Q=mxCreateDoubleMatrix(3,(mwSize)PP.size(),mxREAL);double* pq=mxGetPr(Q);' newline ...
'          for(mwIndex k=0;k<(mwSize)PP.size();++k){pq[3*k+0]=PP[k].x;pq[3*k+1]=PP[k].y;pq[3*k+2]=PP[k].z;}' newline ...
'          mxSetCell(PC,ss,Q);' newline ...
'        }' newline ...
'        mxSetCell(CCP,ci,PC);' newline ...
'      }' newline ...
'      set("all_candidate_z",CZ);set("all_candidate_segment_t0",CS0);' newline ...
'      set("all_candidate_segment_t1",CS1);set("all_candidate_control_points",CCP);' newline ...
'    }'];
assert(af7r57g_count(raw,needle2)==1,'[AF7R57G] probe serializer anchor not unique.');
raw=strrep(raw,needle2,probeCode);

fid=fopen(dst,'w');assert(fid>=0,'[AF7R57G] cannot create shadow probe source.');
c=onCleanup(@()fclose(fid));fwrite(fid,raw,'char');
end


function n=af7r57g_count(s,pat)
n=numel(strfind(s,pat)); %#ok<STREMP>
end


function [F,expr,signature]=af7r57g_nominal_ast(replanner,M)
F=replanner.nominal_flat_derivative_fn(1:7);
assert(iscell(F)&&numel(F)>=7&&all(cellfun(@(f)isa(f,'function_handle'),F)));
S=cellfun(@func2str,F,'UniformOutput',false);
signature=af7r57g_sha256_text(strjoin(S,char(10)));
assert(strcmp(signature,char(M.nominal_signature)),'[AF7R57G] nominal source mismatch.');
expr=cell(21,1);kk=0;
for r=1:7
    ss=strtrim(S{r});tok=regexp(ss,'^@\(([A-Za-z]\w*)\)(.*)$','tokens','once');
    assert(~isempty(tok)&&numel(tok)==2);
    body=strtrim(char(tok{2}));body=strrep(body,'.^','^');body=strrep(body,'.*','*');body=strrep(body,'./','/');
    z=str2sym(body);z=z(:);assert(numel(z)==3);t=sym('t','real');sv0=symvar(z);assert(numel(sv0)<=1);
    if ~isempty(sv0),z=subs(z,sv0(1),t);end
    sv=symvar(z);for j=1:numel(sv),assert(strcmp(char(sv(j)),'t'));end
    for j=1:3,kk=kk+1;expr{kk}=char(z(j));end
end
end


function fn=af7r57g_find_source(name)
fn='';p=which(name);if ~isempty(p)&&isfile(p),fn=p;return;end
roots={pwd,fileparts(mfilename('fullpath'))};
p=which('af7r56_trace_mex');if ~isempty(p),roots{end+1}=fileparts(p);end %#ok<AGROW>
u=getenv('USERPROFILE');if ~isempty(u),roots{end+1}=fullfile(u,'Documents','GitHub','common_matlab','reference');end %#ok<AGROW>
for i=1:numel(roots),q=fullfile(roots{i},name);if isfile(q),fn=q;return;end,end
end


function h=af7r57g_sha256_text(x)
md=java.security.MessageDigest.getInstance('SHA-256');md.update(uint8(unicode2native(char(x),'UTF-8')));
d=typecast(md.digest(),'uint8');h=lower(reshape(dec2hex(d,2).',1,[]));
end


function h=af7r57g_sha256_file(fn)
md=java.security.MessageDigest.getInstance('SHA-256');fid=fopen(fn,'rb');assert(fid>=0);c=onCleanup(@()fclose(fid));
while true,b=fread(fid,1024*1024,'*uint8');if isempty(b),break;end,md.update(b);end
d=typecast(md.digest(),'uint8');h=lower(reshape(dec2hex(d,2).',1,[]));
end


function I=af7r57g_normalize_interval_matrix(Iraw,n,name)
Iraw=double(Iraw);

assert(isnumeric(Iraw)&&isreal(Iraw)&&all(isfinite(Iraw(:))), ...
    '[AF7R57G] %s must be finite real numeric.',name);

if isequal(size(Iraw),[n 2])
    I=Iraw;
elseif isequal(size(Iraw),[2 n])
    I=Iraw.';
elseif isvector(Iraw) && numel(Iraw)==2*n
    % Do not silently guess arbitrary vector packing.  Only accept the two
    % canonical interval matrix orientations above.
    error('AF7R57G:IntervalShape', ...
        '[AF7R57G] %s is vectorized; canonical Nx2 or 2xN shape required.',name);
else
    error('AF7R57G:IntervalShape', ...
        '[AF7R57G] %s shape %s incompatible with z dimension %d.', ...
        name,mat2str(size(Iraw)),n);
end

assert(all(I(:,1)<=I(:,2)), ...
    '[AF7R57G] %s contains reversed intervals.',name);
end


function ok=af7r57g_same_number(a,b)
if isinf(a)||isinf(b)
    ok=isinf(a)&&isinf(b)&&sign(a)==sign(b);return;
end
if isnan(a)||isnan(b)
    ok=isnan(a)&&isnan(b);return;
end
tol=1024*eps(max([1 abs(a) abs(b)]));
ok=abs(a-b)<=tol;
end
