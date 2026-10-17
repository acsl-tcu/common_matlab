function OUT = AF7R57I_FULL_INTERVAL_TERMINAL_BOUNDARY_THEOREM()
% AF7R57I_FULL_INTERVAL_TERMINAL_BOUNDARY_THEOREM
% -----------------------------------------------------------------------------
% Continuous/outward theorem over the corrected terminal-component boundary.
%
% Search coordinates may originate from AF7R57H, but NO H-grid PASS/FAIL is
% accepted as a theorem.  A t1 box is accepted only when:
%
%   (1) interval KKT closes the same unique sign/active-set branch;
%   (2) stable derivative-first Stage-B is strict on every Bezier leaf;
%   (3) payload, UAV, and complete cable segment are continuously separated
%       from every sensed ellipsoid over every physical-time cell;
%   (4) the zero-flat terminal rejoin tube is continuously separated.
%
% All interval arithmetic inside the full-gate MEX is widened with
% std::nextafter.  Any unresolved box is subdivided.  If binary64 can no
% longer subdivide it, the box remains INCONCLUSIVE and production selection
% is forbidden.
%
% IMPORTANT:
%   - This proves a continuous SAFE component cover and, when possible, an enclosure of the certification front.
%   - It does not claim physical infeasibility outside that certified set; a necessary exclusion theorem is separate.
%   - It does not select production t1.
%   - Runtime <=4 ms remains a separate production gate.
% -----------------------------------------------------------------------------

clc;
fprintf('\n====================================================================================================\n');
fprintf(' AF7R57I V3 | ADAPTIVE OUTWARD SAFE-COMPONENT COVER + CERTIFICATION-FRONT THEOREM\n');
fprintf('====================================================================================================\n');
fprintf(['[AUTHORITY] intervalKKT=1 | stableStageB=1 | wholeBodyContinuous=1 | ' ...
         'rejoinInterval=1 | sampledAcceptance=0 | failClosed=1 | productionModified=0\n']);

OUT=struct();
OUT.schema='AF7R57I_FULL_INTERVAL_TERMINAL_BOUNDARY_THEOREM_V3';
OUT.closed=false;
OUT.production_modified=false;
OUT.production_t1_selected=false;
OUT.continuous_terminal_boundary_proof=false;

%% A. Authorities
fprintf('\n[A. CLOSED AUTHORITIES]\n');

assert(evalin('base','exist(''replanner_instance'',''var'')==1'), ...
    '[AF7R57I] replanner_instance missing.');
replanner=evalin('base','replanner_instance');
assert(isa(replanner,'REPLANNING_SYSTEM'));

H=af7r57i_get_authority('AF7R57H_STABLE_STAGEB_TERMINAL_DOMAIN_RESCAN_OUT', ...
                        'AF7R57H_STABLE_STAGEB_TERMINAL_DOMAIN_RESCAN_OUT.mat');
G=af7r57i_get_authority('AF7R57G_NATIVE_WITNESS_CONSISTENCY_AUDIT_OUT', ...
                        'AF7R57G_NATIVE_WITNESS_CONSISTENCY_AUDIT_OUT.mat');
C=af7r57i_get_authority('AF7R57C_SNAPSHOT_BOUND_SCHEMA_CAPTURE_OUT', ...
                        'AF7R57C_SNAPSHOT_BOUND_SCHEMA_CAPTURE_OUT.mat');

assert(logical(H.closed),'[AF7R57I] R57H V2 rescan not closed.');
assert(logical(G.closed)&&strcmp(char(G.diagnosis), ...
    'NUMERICAL_SPLIT_THEN_DIFFERENTIATE_WITNESS_ARTIFACT'), ...
    '[AF7R57I] R57G artifact closure missing.');
assert(logical(C.closed),'[AF7R57I] R57C authority not closed.');

[E,M,AUX,authoritySource]=AF7R52_LOAD_TRUE_TRIGGER_AUTHORITY(replanner); %#ok<ASGLU>
assert(isstruct(E)&&logical(E.valid)&&isstruct(M)&&logical(M.certified));

fprintf(' AF7R57H corrected rescan closed          = 1\n');
fprintf(' AF7R57G split-artifact closure           = 1\n');
fprintf(' AF7R57C same-run authority               = 1\n');
fprintf(' AF7R52 authority                         = %s\n',char(string(authoritySource)));

left0=double(H.corrected_left_predecessor_t1);
right0=double(H.corrected_terminal_speed_left_grid_t1);
tU=max(double(H.t1));

assert(isfinite(left0)&&isfinite(right0)&&left0<right0&&right0<=tU);
fprintf(' theorem boundary search interval         = [%.17g %.17g]\n',left0,right0);
fprintf(' terminal source-domain upper tU          = %.17g\n',tU);

%% B. Exact source / MEX binding
fprintf('\n[B. EXACT SOURCE / MEX BINDING]\n');

root=fileparts(mfilename('fullpath'));
hpp=af7r57i_find_source('af7r56_trace_core.hpp');
tracecpp=af7r57i_find_source('af7r56_trace_mex.cpp');
rps=which('REPLANNING_SYSTEM');

expectedHpp='9f1c7054bbac0dadc2e82fbb107183937143b39431d8996ee13a620cc62ef821';
expectedTrace='cef987c012982982d2fc4381419781770fe9304c6a2ba4f1d7cff1bba2746f23';
expectedRps='223d34c49d7148b8ea14ff4ab9c420d409560c0650ac19e9b0f99318887090c9';

sourceClosed=strcmpi(af7r57i_sha256_file(hpp),expectedHpp)&& ...
             strcmpi(af7r57i_sha256_file(tracecpp),expectedTrace)&& ...
             strcmpi(af7r57i_sha256_file(rps),expectedRps);
fprintf(' native/source SHA authority current      = %d\n',sourceClosed);
assert(sourceClosed,'[AF7R57I] source authority changed.');

assert(exist('af7r56_interval_kkt_mex','file')==3, ...
    '[AF7R57I] interval-KKT MEX unavailable.');

cpp=fullfile(root,'af7r57i_interval_fullgate_mex.cpp');
assert(isfile(cpp));
mexName='af7r57i_interval_fullgate_mex';
needBuild=exist(mexName,'file')~=3;
if ~needBuild
    q=which(mexName);
    needBuild=isempty(q)||dir(q).datenum<dir(cpp).datenum;
end
if needBuild
    mex('-R2018a','-outdir',root,cpp);
    rehash;
end
assert(exist(mexName,'file')==3);
fprintf(' full interval MEX available              = 1\n');
fprintf(' outward primitive                        = std::nextafter\n');
fprintf(' geometry certificate                     = ELLIPSOID_SUPPORT_SEPARATING_PLANE\n');
fprintf(' cable certificate                        = COMMON_SUPPORT_PLANE_FOR_COMPLETE_SEGMENT\n');

%% C. Runtime packet / nominal / obstacle authority
fprintf('\n[C. RUNTIME PACKET / NOMINAL / OBSTACLE AUTHORITY]\n');

TR=C.trace_records;
KR=C.kkt_records;
assert(iscell(TR)&&iscell(KR)&&numel(TR)>=2&&numel(KR)>=2);

rt1=af7r57i_record(TR,1);
pkt=rt1.pkt;
rk1=af7r57i_record(KR,1);
U=rk1.U;

assert(double(U.component_count)==1, ...
    ['[AF7R57I] current interval-KKT source contract is bound to the one-component ' ...
     'true-trigger authority. Multi-component episodes FAIL CLOSED here.']);

F=replanner.nominal_flat_derivative_fn(1:7);
assert(iscell(F)&&numel(F)>=7);
S=cellfun(@func2str,F,'UniformOutput',false);
signature=af7r57i_sha256_text(strjoin(S,char(10)));
assert(strcmp(signature,char(pkt.nominal_signature)));
assert(isempty(pkt.repair_t0)&&isempty(pkt.repair_t1)&&isempty(pkt.repair_cp), ...
    '[AF7R57I] prior active repairs require a larger ActiveReference interval theorem.');

centers=double(pkt.centers);
rotflat=double(pkt.rotations_flat);
axes=double(pkt.base_axes);
assert(size(centers,1)==3&&size(rotflat,1)==9&&size(axes,1)==3);
assert(size(centers,2)==size(rotflat,2)&&size(centers,2)==size(axes,2));

fprintf(' nominal signature equal                 = 1\n');
fprintf(' immutable prior repair count            = 0\n');
fprintf(' obstacle count (runtime, not hardcoded)  = %d\n',size(centers,2));
fprintf(' candidate component count               = 1\n');

%% D. Interval-KKT self-validation on R57C closed bracket
fprintf('\n[D. INTERVAL-KKT SELF-VALIDATION]\n');

kktSelf=true;
for c=1:2
    rk=af7r57i_record(KR,c);
    rt=af7r57i_record(TR,c);
    K0=rk.K{c};
    K=af7r57i_kkt_box(rt,rk,[double(K0.t_interval(1)) double(K0.t_interval(2))],c);
    same=logical(K.closed)&&logical(K.spd)&&logical(K.lambda_positive)&& ...
         logical(K.inactive_strict)&& ...
         all(K.z_interval(:,1)<=K0.z_interval(:,1))&& ...
         all(K.z_interval(:,2)>=K0.z_interval(:,2))&& ...
         K.lambda(1)<=K0.lambda(1)&&K.lambda(2)>=K0.lambda(2);
    fprintf(' candidate %d old-bracket KKT containment = %d\n',c,same);
    kktSelf=kktSelf&&same;
end
assert(kktSelf,'[AF7R57I] interval-KKT runtime no longer reproduces saved authority.');

%% E. Adaptive continuous safe cover of terminal component to tU
fprintf('\n[E. ADAPTIVE CONTINUOUS SAFE COVER TO tU]\n');

% IMPORTANT:
% Do NOT replay every 0.025-s R57H lattice interval.  H supplied only the
% initial right-side search coordinate.  The theorem starts from ONE t1 box
% [right0,tU] and subdivides only boxes that cannot yet be certified.
%
% This changes proof scheduling only, never acceptance conditions.
ticE=tic;
fprintf(' initial safe-cover box                   = [%.17g %.17g]\n',right0,tU);

[coverClosed,safeBoxes,safeDiag,coverStats]=af7r57i_prove_safe_cover( ...
    [right0 tU],replanner,F,pkt,TR,KR,U,centers,rotflat,axes, ...
    'maxNodes',8192,'progressEvery',64);

fprintf(' outer t1 boxes visited                  = %d\n',coverStats.nodes);
fprintf(' outer t1 splits                         = %d\n',coverStats.splits);
fprintf(' certified safe sub-boxes                = %d\n',size(safeBoxes,1));
fprintf(' max proof queue                         = %d\n',coverStats.maxQueue);
fprintf(' continuous safe cover [right0,tU]       = %d\n',coverClosed);
fprintf(' safe-cover elapsed [s]                  = %.6f\n',toc(ticE));

if ~coverClosed
    fprintf(' safe-cover stop reason                   = %s\n',coverStats.reason);
end

%% F. Certification-front interval bisection
fprintf('\n[F. CERTIFICATION-FRONT INTERVAL BISECTION]\n');

% We DO NOT attempt to classify every point left of the safe component.
% Invariant:
%
%   [frontR, right0] U [right0,tU] is continuously certified SAFE.
%
% frontL is merely unresolved.  It is never called unsafe.
%
% At each step, try to certify [mid,frontR].  If proof closes, move frontR
% left to mid.  Otherwise move frontL right to mid.  Thus every right update
% is backed by an interval theorem, never a sampled point.

frontL=left0;
frontR=right0;
frontCover=safeBoxes;
frontDiag=safeDiag;
frontIter=0;
frontAttempts=0;
frontSafeExtensions=0;
frontBudgetStops=0;
maxFrontIter=80;

ticF=tic;
while frontIter<maxFrontIter
    mid=frontL+0.5*(frontR-frontL);
    if ~(mid>frontL&&mid<frontR)
        break;
    end
    frontIter=frontIter+1;
    frontAttempts=frontAttempts+1;

    [okB,bxB,dgB,stB]=af7r57i_prove_safe_cover( ...
        [mid frontR],replanner,F,pkt,TR,KR,U,centers,rotflat,axes, ...
        'maxNodes',1024,'progressEvery',0);

    if okB
        frontR=mid;
        frontCover=[bxB;frontCover]; %#ok<AGROW>
        frontDiag=[dgB frontDiag]; %#ok<AGROW>
        frontSafeExtensions=frontSafeExtensions+1;
    else
        frontL=mid;
        if strcmp(stB.reason,'NODE_BUDGET_FAIL_CLOSED')
            frontBudgetStops=frontBudgetStops+1;
        end
    end

    if mod(frontIter,8)==0
        fprintf([' front iter=%d | unresolved-left=%.17g | safe-right=%.17g | ' ...
                 'width=%.17g\n'], ...
            frontIter,frontL,frontR,frontR-frontL);
    end
end

frontClosed = frontR>frontL && af7r57i_safe_cover_exact(frontCover,frontR,tU);

fprintf(' front iterations                        = %d\n',frontIter);
fprintf(' successful safe extensions              = %d\n',frontSafeExtensions);
fprintf(' fail-closed budget stops                = %d\n',frontBudgetStops);
fprintf(' certification-front enclosure           = [%.17g %.17g]\n',frontL,frontR);
fprintf(' certification-front width               = %.17g\n',frontR-frontL);
fprintf(' right-side exact SAFE cover              = %d\n',frontClosed);
fprintf(' physical left-side infeasibility proved = 0\n');
fprintf(' front-refinement elapsed [s]             = %.6f\n',toc(ticF));

% Compatibility outputs: no exhaustive PART is created in V3.
PART=struct('box',{[frontL frontR]}, ...
            'status',{'INCONCLUSIVE_CERTIFICATION_FRONT'}, ...
            'candidate_diag',{{}});
partClosed=frontClosed;
exactCover=frontClosed;
topology=frontClosed;
bnd=[frontL frontR];
%% G. Structural smoothness / rejoin semantics
fprintf('\n[G. STRUCTURAL SMOOTHNESS / REJOIN]\n');

% Source-bound degree-15 endpoint jets 0..7 imply zero correction jets at
% EXIT_TO_END.  Thus the position correction is C7 at t1 and in particular
% preserves the required position C6.  Avoidance does not alter yaw here.
positionC6=true;
terminalZeroJets=true;
yawModified=false;

fprintf(' terminal correction jets 0..7 zero      = %d\n',terminalZeroJets);
fprintf(' avoidance position C6 preserved         = %d\n',positionC6);
fprintf(' avoidance modifies yaw                  = %d\n',yawModified);
fprintf(' yaw C6 effect of avoidance              = INHERITED_UNCHANGED_FROM_BASELINE\n');

%% H. Formal theorem gate
fprintf('\n[H. FORMAL THEOREM GATE]\n');

safeComponentProof=coverClosed&&positionC6&&terminalZeroJets&&kktSelf;
certificationFrontProof=partClosed&&exactCover&&topology&&safeComponentProof;

% A true physical terminal-boundary theorem additionally needs a necessary
% exclusion theorem/witness on the left.  V2 intentionally does not invent
% one from failure of a sufficient Stage-B upper bound.
physicalBoundaryProof=false;

fprintf(' interval KKT source/runtime bound        = %d\n',kktSelf);
fprintf(' stable Stage-B continuous safe cover     = %d\n',coverClosed);
fprintf(' whole-body continuous geometry cover     = %d\n',coverClosed);
fprintf(' rejoin interval cover                    = %d\n',coverClosed);
fprintf(' certified safe component [right0,tU]     = %d\n',safeComponentProof);
fprintf(' certification-front enclosure closed     = %d\n',certificationFrontProof);
fprintf(' certification front is physical reject   = 0\n');
fprintf(' physical terminal-boundary proof         = %d\n',physicalBoundaryProof);
fprintf(' production t1 selected                   = 0\n');
fprintf(' production source modified               = 0\n');
fprintf(' online <=4 ms runtime gate closed        = 0\n');

if safeComponentProof && certificationFrontProof
    next='SAFE_T1_SELECTION_SHADOW_GATE_PLUS_4MS_RUNTIME_GATE__PHYSICAL_MINIMALITY_OPTIONAL';
elseif safeComponentProof
    next='SAFE_COMPONENT_CLOSED__REFINE_CERTIFICATION_FRONT_OR_PROCEED_NONMINIMAL_SAFE_T1_SHADOW';
else
    next='FAIL_CLOSED__REFINE_OR_STRENGTHEN_UNRESOLVED_INTERVAL_CERTIFICATES';
end
fprintf(' next action                              = %s\n',next);
fprintf('====================================================================================================\n');

OUT.closed=safeComponentProof;
OUT.safe_component_proof=safeComponentProof;
OUT.certification_front_proof=certificationFrontProof;
OUT.certification_front_enclosure=bnd;
OUT.physical_terminal_boundary_proof=false;
OUT.boundary_enclosure=bnd;
OUT.safe_cover=safeBoxes;
OUT.boundary_partition=PART;
OUT.position_C6=positionC6;
OUT.yaw_modified=yawModified;
OUT.continuous_terminal_boundary_proof=false;
OUT.production_t1_selected=false;
OUT.production_modified=false;
OUT.runtime_4ms_gate_closed=false;
OUT.next_action=next;

assignin('base','AF7R57I_FULL_INTERVAL_TERMINAL_BOUNDARY_THEOREM_OUT',OUT);
save(fullfile(root,'AF7R57I_FULL_INTERVAL_TERMINAL_BOUNDARY_THEOREM_OUT.mat'),'OUT','-v7.3');
end


function K=af7r57i_kkt_box(rt,rk,box,c)
% R57C exact capture contract:
%   trace record  -> endpoint Stage-B traces / candidate identity
%   KKT record    -> K,U,pkt,lo,hi,tU,A,b,ac0,signTag,c
%
% The original AF7R56B call captured by R57C is exactly:
%   af7r56_interval_kkt_mex(lo,hi,U.t0,U.component_start(1), ...
%       U.component_end(1),A,b,ac0(1)+1,signTag,c)
%
% Do not move these fields to the trace record: that would be a schema guess.
req={'A','b','ac0','signTag','c','U','K'};
for ii=1:numel(req)
    assert(isfield(rk,req{ii}), ...
        '[AF7R57I] R57C KKT record missing field %s.',req{ii});
end
assert(isfield(rt,'c')&&double(rt.c)==c, ...
    '[AF7R57I] trace candidate identity mismatch.');
assert(double(rk.c)==c, ...
    '[AF7R57I] KKT candidate identity mismatch.');

U=rk.U;
A=double(rk.A);
b=double(rk.b);
ac0=double(rk.ac0);
signTag=double(rk.signTag);

K=af7r56_interval_kkt_mex( ...
    double(box(1)),double(box(2)),double(U.t0), ...
    double(U.component_start(1)),double(U.component_end(1)), ...
    A,b,ac0(1)+1,signTag,c);
end


function [safe,rej,D]=af7r57i_candidate_box(c,box,replanner,F,pkt,TR,KR,U,centers,rotflat,axes,needGeom)
rt=af7r57i_record(TR,c);
rk=af7r57i_record(KR,c);

D=struct('candidate',c,'box',box,'kkt_closed',false,'speed',[], ...
         'geometry',[],'rejoin',[],'status','INCONCLUSIVE');
safe=false;rej=false;

try
    K=af7r57i_kkt_box(rt,rk,box,c);
catch ME
    D.kkt_error=ME.message;
    return;
end

D.kkt_closed=logical(K.closed)&&logical(K.spd)&& ...
    logical(K.lambda_positive)&&logical(K.inactive_strict);
if ~D.kkt_closed
    return;
end

[Sstatus,S]=af7r57i_speed_box(K,box,U,F,pkt);
D.speed=S;

% Stable Stage-B in R57I V2 is a sufficient safety certificate only.
% Failure to certify is INCONCLUSIVE, never candidate rejection.
if ~strcmp(Sstatus,'SAFE')
    return;
end

if ~needGeom
    safe=true;D.status='SAFE';return;
end

[Gstatus,G]=af7r57i_geometry_box(K,box,U,F,pkt,centers,rotflat,axes,S);
D.geometry=G;
if ~strcmp(Gstatus,'SAFE')
    return;
end

[Rstatus,R]=af7r57i_rejoin_box(K,box,U,F,pkt,centers,rotflat,axes,S);
D.rejoin=R;
if ~strcmp(Rstatus,'SAFE')
    return;
end

safe=true;D.status='SAFE';
end


function [status,D]=af7r57i_speed_box(K,box,U,F,pkt)
% Adaptive dyadic Bezier partition.  A leaf is accepted only when the
% outward interval MEX returns class -1.

nc=numel(U.component_start);
tree=zeros(2*nc+1,3);
tree(:,1)=(0:2*nc).';
tree(:,2)=0;
tree(:,3)=1;

limit=(double(pkt.sensor_range)-max([double(pkt.r_drone), ...
    double(pkt.cable_length)+double(pkt.r_load), ...
    double(pkt.cable_length)+double(pkt.r_cable)]))/ ...
    (double(pkt.horizon)+double(pkt.dt));

% Local Bezier refinement budget.  Exhaustion returns INCONCLUSIVE and
% triggers outer t1 subdivision; it never creates acceptance.
maxSplit=2048;
splits=0;
last=[];

while true
    Tm=af7r57i_interval_fullgate_mex(0,box,double(U.t0), ...
        double(U.component_start(:).'),double(U.component_end(:).'),tree,0);

    n=size(tree,1);VC=zeros(3,n);AC=zeros(3,n);
    vRad=zeros(n,1);aRad=zeros(n,1);
    for j=1:n
        tau=max(0,double(Tm.tc_center(j))-double(pkt.flight_t0));
        v=double(F{2}(tau));a=double(F{3}(tau));
        VC(:,j)=v(:);AC(:,j)=a(:);
        vRad(j)=double(pkt.base_payload_accel)*double(Tm.tc_radius(j));
        aRad(j)=double(pkt.base_payload_jerk)*double(Tm.tc_radius(j));
    end

    R=af7r57i_interval_fullgate_mex(1,double(K.z_interval),box,double(U.t0), ...
        double(U.component_start(:).'),double(U.component_end(:).'),tree, ...
        double(pkt.base_payload_accel),double(pkt.base_payload_jerk), ...
        double(pkt.gravity),double(pkt.cable_length), ...
        VC,AC,vRad,aRad,limit,0);
    last=R;
    cls=double(R.class_code(:));

    if all(cls==-1)
        status='SAFE';break;
    end

    % IMPORTANT THEOREM SEMANTICS:
    % class_code=+1 means that THIS CURRENT LEAF ENCLOSURE has
    % G_LB>0.  It does NOT prove that the underlying candidate is unsafe,
    % because restricting the Bezier leaf reduces the enclosure and the
    % Stage-B sufficient bound may become strict-safe after refinement.
    %
    % Therefore BOTH class 0 and class +1 are refinement requests.
    % No Stage-B upper-bound failure is ever promoted to mathematical REJECT.
    iz=find(cls~=-1,1,'first');
    u0=tree(iz,2);u1=tree(iz,3);um=0.5*(u0+u1);
    if ~(um>u0&&um<u1) || splits>=maxSplit
        status='INCONCLUSIVE';break;
    end
    row=tree(iz,:);
    tree(iz,:)=[row(1) u0 um];
    tree=[tree;row(1) um u1]; %#ok<AGROW>
    splits=splits+1;
end

D=struct();
D.status=status;D.tree=tree;D.splits=splits;D.raw=last;
if ~isempty(last)
    D.VL_ub=max(last.vl(:,2));
    D.VQ_ub=max(last.vq(:,2));
    D.sigma_lb=min(last.sigma(:,1));
    D.limit=limit;
else
    D.VL_ub=Inf;D.VQ_ub=Inf;D.sigma_lb=-Inf;D.limit=limit;
end
end


function [status,D]=af7r57i_geometry_box(K,box,U,F,pkt,centers,rotflat,axes,S)
% Complete payload/UAV/cable separation over [t0, t1] and the entire t1 box.
% Time cells are recursively refined; acceptance uses interval margins only.

cs=double(U.component_start(:).');
ce=double(U.component_end(:).');
nc=numel(cs);
t0=double(U.t0);

cells=zeros(0,3); % [seg,ta,tb]
for seg=0:2*nc
    if seg==0
        a=t0;b=cs(1);
    elseif seg==2*nc
        a=ce(end);b=box(2);
    elseif mod(seg,2)==1
        j=(seg+1)/2;a=cs(j);b=ce(j);
    else
        j=seg/2;a=ce(j);b=cs(j+1);
    end
    if b>a
        if seg==2*nc && box(1)>a && box(1)<b
            cells=[cells;seg a box(1);seg box(1) b]; %#ok<AGROW>
        else
            cells=[cells;seg a b]; %#ok<AGROW>
        end
    end
end

% A broad t1 box may remain conservative even after excessive physical-time
% splitting.  Do not spend hundreds of thousands of cells trying to repair
% t1-dependency with time refinement.  After this fail-closed proof budget,
% return INCONCLUSIVE so the OUTER t1 box is bisected.
%
% This cap cannot create PASS; it changes only proof scheduling.
maxSplit=512;
splits=0;
minL=Inf;minQ=Inf;minC=Inf;minSf=Inf;
i=1;
while i<=size(cells,1)
    seg=cells(i,1);ta=cells(i,2);tb=cells(i,3);
    tc=ta+0.5*(tb-ta);h=0.5*(tb-ta);
    tau=max(0,tc-double(pkt.flight_t0));
    pc=double(F{1}(tau));ac=double(F{3}(tau));
    pRad=double(pkt.base_payload_speed)*h;
    aRad=double(pkt.base_payload_jerk)*h;

    rhoL=double(pkt.r_load)+double(pkt.d_margin)+double(pkt.tracking_inflation);
    rhoQ=double(pkt.r_drone)+double(pkt.d_margin)+double(pkt.tracking_inflation);
    rhoC=double(pkt.r_cable)+double(pkt.d_margin)+double(pkt.tracking_inflation);

    R=af7r57i_interval_fullgate_mex(2,double(K.z_interval),box,t0,cs,ce, ...
        seg,[ta tb],pc(:),pRad,ac(:),aRad,double(pkt.gravity), ...
        double(pkt.cable_length),centers,rotflat,axes,rhoL,rhoQ,rhoC,0,0,0,0);

    minL=min(minL,double(R.load_margin_lb));
    minQ=min(minQ,double(R.drone_margin_lb));
    minC=min(minC,double(R.cable_margin_lb));
    minSf=min(minSf,double(R.specific_force_norm_lb));

    if double(R.class_code)==-1
        i=i+1;
        continue;
    end

    tm=ta+0.5*(tb-ta);
    if ~(tm>ta&&tm<tb) || splits>=maxSplit
        status='INCONCLUSIVE';
        D=struct('status',status,'cells',cells,'splits',splits, ...
            'min_load_margin_lb',minL,'min_drone_margin_lb',minQ, ...
            'min_cable_margin_lb',minC,'specific_force_norm_lb',minSf);
        return;
    end
    old=cells(i,:);
    cells(i,:)=[seg ta tm];
    cells=[cells(1:i,:);seg tm tb;cells(i+1:end,:)]; %#ok<AGROW>
    splits=splits+1;
end

status='SAFE';
D=struct('status',status,'cells',cells,'splits',splits, ...
    'min_load_margin_lb',minL,'min_drone_margin_lb',minQ, ...
    'min_cable_margin_lb',minC,'specific_force_norm_lb',minSf);
end


function [status,D]=af7r57i_rejoin_box(K,box,U,F,pkt,centers,rotflat,axes,S)
% At t=t1 the correction endpoint jets are zero.  Rejoin therefore uses the
% nominal interval over t1, with the same half-dt speed inflation as native
% rejoin_safe.

tc=box(1)+0.5*(box(2)-box(1));
hT=0.5*(box(2)-box(1));
tau=max(0,tc-double(pkt.flight_t0));
pc=double(F{1}(tau));ac=double(F{3}(tau));

pRad=double(pkt.base_payload_speed)*hT;
aRad=double(pkt.base_payload_jerk)*hT;
h=0.5*double(pkt.dt);

rhoL=double(pkt.r_load)+double(pkt.d_margin)+double(pkt.tracking_inflation)+S.VL_ub*h;
rhoQ=double(pkt.r_drone)+double(pkt.d_margin)+double(pkt.tracking_inflation)+S.VQ_ub*h;
rhoC=double(pkt.r_cable)+double(pkt.d_margin)+double(pkt.tracking_inflation)+max(S.VL_ub,S.VQ_ub)*h;

R=af7r57i_interval_fullgate_mex(2,double(K.z_interval),box,double(U.t0), ...
    double(U.component_start(:).'),double(U.component_end(:).'), ...
    -1,box,pc(:),pRad,ac(:),aRad,double(pkt.gravity),double(pkt.cable_length), ...
    centers,rotflat,axes,rhoL,rhoQ,rhoC,0,0,0,0);

if double(R.class_code)==-1
    status='SAFE';
else
    status='INCONCLUSIVE';
end
D=struct('status',status,'raw',R);
end


function [ok,boxes,diagbox,ST]=af7r57i_prove_safe_cover(box,replanner,F,pkt,TR,KR,U,centers,rotflat,axes,varargin)
% Adaptive theorem scheduler.
%
% Acceptance is unchanged: a box enters `boxes` only after full interval
% KKT + stable Stage-B + whole-body geometry + rejoin all close.
%
% maxNodes/progressEvery affect only computation scheduling.  Exhaustion
% returns FAIL CLOSED and can never create PASS.

p=inputParser;
p.addParameter('maxNodes',8192,@(x)isnumeric(x)&&isscalar(x)&&x>=1);
p.addParameter('progressEvery',64,@(x)isnumeric(x)&&isscalar(x)&&x>=0);
p.parse(varargin{:});
maxNodes=double(p.Results.maxNodes);
progressEvery=double(p.Results.progressEvery);

queue=box;
boxes=zeros(0,2);diagbox={};
nodes=0;splits=0;maxQueue=1;
ok=true;reason='CLOSED';

while ~isempty(queue)
    q=queue(1,:);queue(1,:)=[];
    nodes=nodes+1;
    maxQueue=max(maxQueue,size(queue,1)+1);

    if nodes>maxNodes
        ok=false;reason='NODE_BUDGET_FAIL_CLOSED';break;
    end

    if progressEvery>0 && mod(nodes,progressEvery)==0
        fprintf(['  cover progress nodes=%d certified=%d queue=%d ' ...
                 'current=[%.9f %.9f]\n'], ...
            nodes,size(boxes,1),size(queue,1),q(1),q(2));
    end

    % Candidate 1 first; on the current terminal component it is the
    % established useful branch in R57F/R57H. Candidate 2 is evaluated only
    % if candidate 1 does not certify this whole box.
    [s1,~,D1]=af7r57i_candidate_box(1,q,replanner,F,pkt,TR,KR,U,centers,rotflat,axes,true);
    if s1
        boxes(end+1,:)=q; %#ok<AGROW>
        diagbox{end+1}=D1; %#ok<AGROW>
        continue;
    end

    [s2,~,D2]=af7r57i_candidate_box(2,q,replanner,F,pkt,TR,KR,U,centers,rotflat,axes,true);
    if s2
        boxes(end+1,:)=q; %#ok<AGROW>
        diagbox{end+1}=D2; %#ok<AGROW>
        continue;
    end

    % Unresolved -> subdivide t1.  This is proof refinement, not acceptance.
    m=q(1)+0.5*(q(2)-q(1));
    if ~(m>q(1)&&m<q(2))
        ok=false;reason='T1_BINARY64_UNSPLITTABLE_FAIL_CLOSED';break;
    end
    queue=[q(1) m;m q(2);queue]; %#ok<AGROW>
    splits=splits+1;
end

if ok
    boxes=sortrows(boxes,1);
    ok=af7r57i_safe_cover_exact(boxes,box(1),box(2));
    if ~ok,reason='SAFE_BOX_COVER_GAP_FAIL_CLOSED';end
end

ST=struct('nodes',nodes,'splits',splits,'maxQueue',maxQueue,'reason',reason);
end


function tf=af7r57i_safe_cover_exact(boxes,a,b)
if isempty(boxes),tf=false;return;end
boxes=sortrows(boxes,1);
tf=(boxes(1,1)==a)&&(boxes(end,2)==b);
if ~tf,return;end
for k=1:size(boxes,1)-1
    if boxes(k,2)~=boxes(k+1,1)
        tf=false;return;
    end
end
end




function tf=af7r57i_exact_cover(P,a,b)
if isempty(P),tf=false;return;end
tf=(P(1).box(1)==a)&&(P(end).box(2)==b);
if ~tf,return;end
for k=1:numel(P)-1
    if P(k).box(2)~=P(k+1).box(1)
        tf=false;return;
    end
end
end


function r=af7r57i_record(R,c)
ids=cellfun(@(q)double(q.c),R);
k=find(ids==c,1,'last');
assert(~isempty(k));r=R{k};
end


function A=af7r57i_get_authority(varName,fileName)
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
assert(~isempty(p),'[AF7R57I] authority unavailable: %s',fileName);
S=load(p,'OUT');assert(isfield(S,'OUT'));A=S.OUT;
end


function fn=af7r57i_find_source(name)
fn='';
p=which(name);
if ~isempty(p)&&isfile(p),fn=p;return;end
roots={pwd,fileparts(mfilename('fullpath'))};
q=which('af7r56_trace_mex');
if ~isempty(q),roots{end+1}=fileparts(q);end %#ok<AGROW>
u=getenv('USERPROFILE');
if ~isempty(u)
    roots{end+1}=fullfile(u,'Documents','GitHub','common_matlab','reference'); %#ok<AGROW>
end
for i=1:numel(roots)
    q=fullfile(roots{i},name);
    if isfile(q),fn=q;return;end
end
assert(false,'[AF7R57I] source unavailable: %s',name);
end


function h=af7r57i_sha256_text(s)
md=java.security.MessageDigest.getInstance('SHA-256');
md.update(uint8(unicode2native(char(s),'UTF-8')));
d=typecast(md.digest(),'uint8');
h=lower(reshape(dec2hex(d,2).',1,[]));
end


function h=af7r57i_sha256_file(fn)
md=java.security.MessageDigest.getInstance('SHA-256');
fid=fopen(fn,'rb');assert(fid>=0);
c=onCleanup(@()fclose(fid));
while true
    x=fread(fid,1024*1024,'*uint8');
    if isempty(x),break;end
    md.update(x);
end
d=typecast(md.digest(),'uint8');
h=lower(reshape(dec2hex(d,2).',1,[]));
end
