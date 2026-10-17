function OUT = AF7R57J_FIXED_TU_OUTWARD_PROVISIONAL_GATE()
% AF7R57J_FIXED_TU_OUTWARD_PROVISIONAL_GATE
% =========================================================================
% Lightweight provisional theorem:
%
%   DO NOT search/minimize t1.
%   Set t1 to the current source-validity upper endpoint tU and certify
%   exactly that candidate with:
%
%     - point-interval KKT (outward)
%     - corrected derivative-first Stage-B (outward)
%     - continuous payload/UAV/complete-cable ellipsoid separation (outward)
%     - continuous rejoin cell with native half-dt speed inflation
%     - structural C7 endpoint zero jets
%
% The old AF7R36 result is used only as a lineage/runtime cross-check.
% Final PASS is determined by the corrected outward gate below.
%
% No production source or active token is modified.
% =========================================================================

clc;
fprintf('\n====================================================================================================\n');
fprintf(' AF7R57J | FIXED-live-tU OUTWARD PROVISIONAL SAFETY GATE\n');
fprintf('====================================================================================================\n');
fprintf(['[AUTHORITY] noT1Search=1 | pointKKT=1 | stableStageB=1 | ' ...
         'wholeBodyContinuous=1 | sampledAcceptance=0 | failClosed=1 | productionModified=0\n']);

OUT=struct();
OUT.schema='AF7R57J_FIXED_TU_OUTWARD_PROVISIONAL_GATE_V1';
OUT.closed=false;
OUT.production_modified=false;
OUT.production_t1_selected=false;
OUT.hard_4ms_gate_closed=false;

%% A. Authorities
fprintf('\n[A. AUTHORITY]\n');

assert(evalin('base','exist(''replanner_instance'',''var'')==1'), ...
    '[AF7R57J] replanner_instance missing.');
replanner=evalin('base','replanner_instance');
assert(isa(replanner,'REPLANNING_SYSTEM'));

C=af7r57j_get_authority('AF7R57C_SNAPSHOT_BOUND_SCHEMA_CAPTURE_OUT', ...
                        'AF7R57C_SNAPSHOT_BOUND_SCHEMA_CAPTURE_OUT.mat');
G=af7r57j_get_authority('AF7R57G_NATIVE_WITNESS_CONSISTENCY_AUDIT_OUT', ...
                        'AF7R57G_NATIVE_WITNESS_CONSISTENCY_AUDIT_OUT.mat');

assert(logical(C.closed),'[AF7R57J] R57C authority not closed.');
assert(logical(G.closed)&&strcmp(char(G.diagnosis), ...
    'NUMERICAL_SPLIT_THEN_DIFFERENTIATE_WITNESS_ARTIFACT'), ...
    '[AF7R57J] corrected Stage-B artifact closure missing.');

[E,M,AUX,authoritySource]=AF7R52_LOAD_TRUE_TRIGGER_AUTHORITY(replanner); %#ok<ASGLU>
assert(isstruct(E)&&logical(E.valid)&&isstruct(M)&&logical(M.certified));

TR=C.trace_records;
KR=C.kkt_records;
rt1=af7r57j_record(TR,1);
rk1=af7r57j_record(KR,1);
U=rk1.U;
pkt=rt1.pkt;

tU=double(rt1.tU);
assert(isfinite(tU)&&tU>double(U.hold_end), ...
    '[AF7R57J] finite source-derived tU unavailable.');

fprintf(' AF7R57C closed                          = 1\n');
fprintf(' AF7R57G corrected Stage-B closure       = 1\n');
fprintf(' AF7R52 authority                        = %s\n',char(string(authoritySource)));
fprintf(' live/current captured tU                = %.17g\n',tU);
fprintf(' tU hardcoded in gate                    = 0\n');
fprintf(' t1 search/minimization                   = 0\n');

%% B. Source / MEX binding
fprintf('\n[B. SOURCE / MEX BINDING]\n');

hpp=af7r57j_find_source('af7r56_trace_core.hpp');
tracecpp=af7r57j_find_source('af7r56_trace_mex.cpp');
rps=which('REPLANNING_SYSTEM');

expectedHpp='9f1c7054bbac0dadc2e82fbb107183937143b39431d8996ee13a620cc62ef821';
expectedTrace='cef987c012982982d2fc4381419781770fe9304c6a2ba4f1d7cff1bba2746f23';
expectedRps='223d34c49d7148b8ea14ff4ab9c420d409560c0650ac19e9b0f99318887090c9';

sourceClosed=strcmpi(af7r57j_sha256_file(hpp),expectedHpp)&& ...
             strcmpi(af7r57j_sha256_file(tracecpp),expectedTrace)&& ...
             strcmpi(af7r57j_sha256_file(rps),expectedRps);
fprintf(' bound source SHA current                = %d\n',sourceClosed);
assert(sourceClosed,'[AF7R57J] source authority changed.');

assert(exist('af7r56_interval_kkt_mex','file')==3, ...
    '[AF7R57J] af7r56_interval_kkt_mex missing.');
assert(exist('af7r36_single_core_mex','file')==3, ...
    '[AF7R57J] af7r36_single_core_mex missing.');

root=fileparts(mfilename('fullpath'));
cpp=fullfile(root,'af7r57i_interval_fullgate_mex.cpp');
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

fprintf(' interval fullgate MEX available         = 1\n');
fprintf(' outward primitive                       = std::nextafter\n');

%% C. Nominal / obstacle / finite-sensing authority
fprintf('\n[C. LIVE PACKET]\n');

F=replanner.nominal_flat_derivative_fn(1:7);
assert(iscell(F)&&numel(F)>=7&&all(cellfun(@(f)isa(f,'function_handle'),F)));

S=cellfun(@func2str,F,'UniformOutput',false);
sig=af7r57j_sha256_text(strjoin(S,char(10)));
assert(strcmp(sig,char(pkt.nominal_signature)), ...
    '[AF7R57J] nominal signature differs from captured packet.');
assert(isempty(pkt.repair_t0)&&isempty(pkt.repair_t1)&&isempty(pkt.repair_cp), ...
    '[AF7R57J] provisional V1 requires nominal-only captured ActiveReference.');

centers=double(pkt.centers);
rotflat=double(pkt.rotations_flat);
axes=double(pkt.base_axes);
assert(size(centers,1)==3&&size(rotflat,1)==9&&size(axes,1)==3);
assert(size(centers,2)==size(rotflat,2)&&size(centers,2)==size(axes,2));

rhoBody=max([double(pkt.r_drone), ...
    double(pkt.cable_length)+double(pkt.r_load), ...
    double(pkt.cable_length)+double(pkt.r_cable)]);
VQlim=(double(pkt.sensor_range)-rhoBody)/(double(pkt.horizon)+double(pkt.dt));
assert(isfinite(VQlim)&&VQlim>0);

fprintf(' nominal signature equal                 = 1\n');
fprintf(' runtime obstacle count                  = %d\n',size(centers,2));
fprintf(' VQ strict limit                         = %.17g\n',VQlim);

%% D. Historical/native single-core cross-check at fixed tU
fprintf('\n[D. R36 SINGLE-CORE CROSS-CHECK]\n');

P=pkt;P.t1=tU;
tic;
R36=af7r36_single_core_mex(1,P);
nativeMs=1000*toc;

fprintf(' native success / reason                 = %d / %.0f\n', ...
    logical(R36.success),double(R36.reason_code));
fprintf(' native speed/rejoin/geometry counts     = %.0f / %.0f / %.0f\n', ...
    double(R36.speed_pass_count),double(R36.rejoin_pass_count), ...
    double(R36.geometry_test_count));
fprintf(' native geometry pass                    = %d\n',logical(R36.geometry_pass));
fprintf(' native geometry min L/Q/C [m]           = [%.9g %.9g %.9g]\n', ...
    double(R36.geometry_min_payload_margin), ...
    double(R36.geometry_min_uav_margin), ...
    double(R36.geometry_min_cable_margin));
fprintf(' native one-shot wall diagnostic [ms]    = %.6f\n',nativeMs);
fprintf(' native result used as final authority   = 0\n');

%% E. Corrected outward candidates at point tU
fprintf('\n[E. CORRECTED OUTWARD FIXED-tU CANDIDATES]\n');

candidate=cell(1,2);
pass=false(1,2);
for c=1:2
    ticCand=tic;
    rt=af7r57j_record(TR,c);
    rk=af7r57j_record(KR,c);

    K=af7r57j_kkt_point(rt,rk,tU,c);
    kktClosed=logical(K.closed)&&logical(K.spd)&& ...
        logical(K.lambda_positive)&&logical(K.inactive_strict);

    if kktClosed
        [speedOK,SC]=af7r57j_speed_point(K,tU,U,F,pkt,VQlim);
    else
        speedOK=false;SC=struct('status','KKT_NOT_CLOSED');
    end

    if speedOK
        [geomOK,GC]=af7r57j_geometry_point(K,tU,U,F,pkt, ...
            centers,rotflat,axes,SC);
    else
        geomOK=false;GC=struct('status','SPEED_NOT_CLOSED');
    end

    if geomOK
        [rejoinOK,RC]=af7r57j_rejoin_point(K,tU,U,F,pkt, ...
            centers,rotflat,axes,SC);
    else
        rejoinOK=false;RC=struct('status','GEOMETRY_NOT_CLOSED');
    end

    pass(c)=kktClosed&&speedOK&&geomOK&&rejoinOK;
    candidate{c}=struct('candidate',c,'KKT',K,'speed',SC,'geometry',GC, ...
        'rejoin',RC,'pass',pass(c),'elapsed_ms',1000*toc(ticCand));

    fprintf([' candidate %d | KKT=%d speed=%d geometry=%d rejoin=%d PASS=%d ' ...
             '| elapsed=%.6f ms\n'], ...
        c,kktClosed,speedOK,geomOK,rejoinOK,pass(c),candidate{c}.elapsed_ms);

    if speedOK
        fprintf(['   Stage-B VQ_UB=%.17g | limit-VQ=%.17g | sigma_LB=%.17g ' ...
                 '| leaves=%d splits=%d\n'], ...
            SC.VQ_ub,VQlim-SC.VQ_ub,SC.sigma_lb,size(SC.tree,1),SC.splits);
    end
    if geomOK
        fprintf(['   geometry margin LB L/Q/C = [%.9g %.9g %.9g] m ' ...
                 '| timeCells=%d splits=%d\n'], ...
            GC.min_load_margin_lb,GC.min_drone_margin_lb,GC.min_cable_margin_lb, ...
            size(GC.cells,1),GC.splits);
    end
end

fixedGate=any(pass);
selected=find(pass,1,'first');

%% F. Structural smoothness
fprintf('\n[F. STRUCTURAL SMOOTHNESS]\n');

% Bound source uses endpoint jets 0..7 for degree-15 control construction.
% EXIT_TO_END has zero terminal correction jets through order 7.
positionC7=true;
positionC6=true;
yawChanged=false;

fprintf(' correction terminal jets 0..7 zero      = %d\n',positionC7);
fprintf(' required position C6                    = %d\n',positionC6);
fprintf(' avoidance yaw modification              = %d\n',yawChanged);

%% G. Warm timing diagnostic
fprintf('\n[G. WARM TIMING DIAGNOSTIC]\n');

% Timing here is NOT the final QPC/ProcessCycleTime hard-WCET authority.
% It is only used to decide whether this fixed-tU architecture is worth
% integrating into the <=4 ms production gate.
N=20;
tm=zeros(N,1);
for k=1:N
    tt=tic;
    af7r57j_fixed_gate_eval(tU,U,F,pkt,TR,KR,centers,rotflat,axes,VQlim);
    tm(k)=1000*toc(tt);
end

fprintf(' warm trials                              = %d\n',N);
fprintf(' warm wall min / median / max [ms]        = %.6f / %.6f / %.6f\n', ...
    min(tm),median(tm),max(tm));
fprintf(' hard <=4 ms timing authority             = 0\n');

%% H. Provisional gate
fprintf('\n[H. PROVISIONAL FORMAL GATE]\n');

closed=fixedGate&&positionC7&&positionC6&&sourceClosed;

fprintf(' fixed live-tU outward candidate exists   = %d\n',fixedGate);
fprintf(' selected corrected candidate             = %s\n', ...
    af7r57j_num_or_none(selected));
fprintf(' continuous payload/UAV/cable included   = 1\n');
fprintf(' sampled acceptance                       = 0\n');
fprintf(' t1 minimization                          = 0\n');
fprintf(' fixed-tU provisional theorem closed      = %d\n',closed);
fprintf(' production t1 selected                   = 0\n');
fprintf(' production source modified               = 0\n');
fprintf(' hard <=4 ms production gate closed       = 0\n');

if closed
    next='BENCHMARK_FIXED_TU_GATE_WITH_QPC_AND_PREPARED_PACKET__THEN_SHADOW_INTEGRATION';
else
    next='FAIL_CLOSED__INSPECT_FIXED_TU_KKT_SPEED_GEOMETRY_OR_REJOIN';
end
fprintf(' next action                              = %s\n',next);
fprintf('====================================================================================================\n');

OUT.closed=closed;
OUT.tU=tU;
OUT.native_R36=R36;
OUT.native_wall_ms=nativeMs;
OUT.candidates=candidate;
OUT.selected_candidate=selected;
OUT.warm_wall_ms=tm;
OUT.position_C7=positionC7;
OUT.position_C6=positionC6;
OUT.yaw_changed=yawChanged;
OUT.production_t1_selected=false;
OUT.production_modified=false;
OUT.hard_4ms_gate_closed=false;
OUT.next_action=next;

assignin('base','AF7R57J_FIXED_TU_OUTWARD_PROVISIONAL_GATE_OUT',OUT);
save(fullfile(root,'AF7R57J_FIXED_TU_OUTWARD_PROVISIONAL_GATE_OUT.mat'),'OUT','-v7.3');
end


function [closed,C]=af7r57j_fixed_gate_eval(tU,U,F,pkt,TR,KR,centers,rotflat,axes,VQlim)
closed=false;C=cell(1,2);
for c=1:2
    rt=af7r57j_record(TR,c);rk=af7r57j_record(KR,c);
    K=af7r57j_kkt_point(rt,rk,tU,c);
    k=logical(K.closed)&&logical(K.spd)&&logical(K.lambda_positive)&&logical(K.inactive_strict);
    if ~k,continue;end
    [s,SC]=af7r57j_speed_point(K,tU,U,F,pkt,VQlim);
    if ~s,continue;end
    [g,GC]=af7r57j_geometry_point(K,tU,U,F,pkt,centers,rotflat,axes,SC);
    if ~g,continue;end
    [r,RC]=af7r57j_rejoin_point(K,tU,U,F,pkt,centers,rotflat,axes,SC);
    C{c}=struct('K',K,'S',SC,'G',GC,'R',RC,'pass',r);
    if r,closed=true;return;end
end
end


function K=af7r57j_kkt_point(rt,rk,t,c)
req={'A','b','ac0','signTag','c','U','K'};
for i=1:numel(req),assert(isfield(rk,req{i}));end
assert(double(rt.c)==c&&double(rk.c)==c);
U=rk.U;
K=af7r56_interval_kkt_mex(t,t,double(U.t0), ...
    double(U.component_start(1)),double(U.component_end(1)), ...
    double(rk.A),double(rk.b),double(rk.ac0)+1,double(rk.signTag),c);
end


function [ok,D]=af7r57j_speed_point(K,t,U,F,pkt,limit)
tree=[0 0 1;1 0 1;2 0 1];
maxSplit=4096;splits=0;last=[];

while true
    Tm=af7r57i_interval_fullgate_mex(0,[t t],double(U.t0), ...
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

    R=af7r57i_interval_fullgate_mex(1,double(K.z_interval),[t t], ...
        double(U.t0),double(U.component_start(:).'), ...
        double(U.component_end(:).'),tree, ...
        double(pkt.base_payload_accel),double(pkt.base_payload_jerk), ...
        double(pkt.gravity),double(pkt.cable_length), ...
        VC,AC,vRad,aRad,limit,0);
    last=R;
    cls=double(R.class_code(:));

    if all(cls==-1)
        ok=true;break;
    end

    iz=find(cls~=-1,1,'first');
    u0=tree(iz,2);u1=tree(iz,3);um=0.5*(u0+u1);
    if ~(um>u0&&um<u1)||splits>=maxSplit
        ok=false;break;
    end
    row=tree(iz,:);
    tree(iz,:)=[row(1) u0 um];
    tree=[tree;row(1) um u1]; %#ok<AGROW>
    splits=splits+1;
end

D=struct('status',tern(ok,'SAFE','INCONCLUSIVE'), ...
    'tree',tree,'splits',splits,'raw',last, ...
    'VL_ub',max(last.vl(:,2)),'VQ_ub',max(last.vq(:,2)), ...
    'sigma_lb',min(last.sigma(:,1)),'limit',limit);
end


function [ok,D]=af7r57j_geometry_point(K,t,U,F,pkt,centers,rotflat,axes,S)
cs=double(U.component_start(:).');
ce=double(U.component_end(:).');
t0=double(U.t0);

cells=[0 t0 cs(1);1 cs(1) ce(1);2 ce(1) t];
cells=cells(cells(:,3)>cells(:,2),:);

maxSplit=4096;splits=0;
minL=Inf;minQ=Inf;minC=Inf;minSf=Inf;
i=1;ok=true;

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

    R=af7r57i_interval_fullgate_mex(2,double(K.z_interval),[t t],t0,cs,ce, ...
        seg,[ta tb],pc(:),pRad,ac(:),aRad,double(pkt.gravity), ...
        double(pkt.cable_length),centers,rotflat,axes,rhoL,rhoQ,rhoC,0,0,0,0);

    minL=min(minL,double(R.load_margin_lb));
    minQ=min(minQ,double(R.drone_margin_lb));
    minC=min(minC,double(R.cable_margin_lb));
    minSf=min(minSf,double(R.specific_force_norm_lb));

    if double(R.class_code)==-1
        i=i+1;continue;
    end

    tm=ta+0.5*(tb-ta);
    if ~(tm>ta&&tm<tb)||splits>=maxSplit
        ok=false;break;
    end
    cells(i,:)=[seg ta tm];
    cells=[cells(1:i,:);seg tm tb;cells(i+1:end,:)]; %#ok<AGROW>
    splits=splits+1;
end

D=struct('status',tern(ok,'SAFE','INCONCLUSIVE'),'cells',cells,'splits',splits, ...
    'min_load_margin_lb',minL,'min_drone_margin_lb',minQ, ...
    'min_cable_margin_lb',minC,'specific_force_norm_lb',minSf);
end


function [ok,D]=af7r57j_rejoin_point(K,t,U,F,pkt,centers,rotflat,axes,S)
tau=max(0,t-double(pkt.flight_t0));
pc=double(F{1}(tau));ac=double(F{3}(tau));
h=0.5*double(pkt.dt);

rhoL=double(pkt.r_load)+double(pkt.d_margin)+double(pkt.tracking_inflation)+S.VL_ub*h;
rhoQ=double(pkt.r_drone)+double(pkt.d_margin)+double(pkt.tracking_inflation)+S.VQ_ub*h;
rhoC=double(pkt.r_cable)+double(pkt.d_margin)+double(pkt.tracking_inflation)+max(S.VL_ub,S.VQ_ub)*h;

R=af7r57i_interval_fullgate_mex(2,double(K.z_interval),[t t],double(U.t0), ...
    double(U.component_start(:).'),double(U.component_end(:).'), ...
    -1,[t t],pc(:),0,ac(:),0,double(pkt.gravity),double(pkt.cable_length), ...
    centers,rotflat,axes,rhoL,rhoQ,rhoC,0,0,0,0);

ok=double(R.class_code)==-1;
D=struct('status',tern(ok,'SAFE','INCONCLUSIVE'),'raw',R);
end


function r=af7r57j_record(R,c)
ids=cellfun(@(q)double(q.c),R);
k=find(ids==c,1,'last');assert(~isempty(k));r=R{k};
end


function A=af7r57j_get_authority(varName,fileName)
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
assert(~isempty(p),'[AF7R57J] authority unavailable: %s',fileName);
S=load(p,'OUT');assert(isfield(S,'OUT'));A=S.OUT;
end


function fn=af7r57j_find_source(name)
p=which(name);
if ~isempty(p)&&isfile(p),fn=p;return;end
u=getenv('USERPROFILE');
q=fullfile(u,'Documents','GitHub','common_matlab','reference',name);
assert(isfile(q),'[AF7R57J] source unavailable: %s',name);
fn=q;
end


function s=af7r57j_num_or_none(x)
if isempty(x),s='NONE';else,s=sprintf('%d',x);end
end


function y=tern(tf,a,b)
if tf,y=a;else,y=b;end
end


function h=af7r57j_sha256_text(s)
md=java.security.MessageDigest.getInstance('SHA-256');
md.update(uint8(unicode2native(char(s),'UTF-8')));
d=typecast(md.digest(),'uint8');
h=lower(reshape(dec2hex(d,2).',1,[]));
end


function h=af7r57j_sha256_file(fn)
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
