function OUT = AF7R57K_CORRECTED_NATIVE_CORE_AUDIT()
% AF7R57K_CORRECTED_NATIVE_CORE_AUDIT
% =========================================================================
% Build a SHADOW copy of the native AF7 core in which ONLY Stage-B adaptive
% subdivision is corrected from
%
%       split position CP -> differentiate short leaf
%
% to
%
%       differentiate parent segment once -> split derivative CPs
%
% using the exact Bezier identity
%
%       D_t^r ( R P ) = R ( D_t^r P ).
%
% Candidate/QP/C7/rejoin/whole-body geometry source is otherwise unchanged.
% Production source and active reference token are never modified.
% =========================================================================

clc;
fprintf('\n====================================================================================================\n');
fprintf(' AF7R57K V2 | CORRECTED NATIVE SHADOW + CELL-AWARE EXACT IDENTITY\n');
fprintf('====================================================================================================\n');
fprintf(['[AUTHORITY] derivativeRestrictionIdentity=1 | nativeCoreShadow=1 | ' ...
         'wholeBodyUnchanged=1 | sampledAcceptance=0 | productionModified=0\n']);

OUT=struct();
OUT.schema='AF7R57K_CORRECTED_NATIVE_CORE_AUDIT_V2';
OUT.closed=false;
OUT.production_modified=false;
OUT.production_t1_selected=false;
OUT.hard_4ms_gate_closed=false;

%% A. Authorities
fprintf('\n[A. CLOSED AUTHORITIES]\n');

assert(evalin('base','exist(''replanner_instance'',''var'')==1'), ...
    '[AF7R57K] replanner_instance missing.');
replanner=evalin('base','replanner_instance');
assert(isa(replanner,'REPLANNING_SYSTEM'));

C=get_authority('AF7R57C_SNAPSHOT_BOUND_SCHEMA_CAPTURE_OUT', ...
                'AF7R57C_SNAPSHOT_BOUND_SCHEMA_CAPTURE_OUT.mat');
G=get_authority('AF7R57G_NATIVE_WITNESS_CONSISTENCY_AUDIT_OUT', ...
                'AF7R57G_NATIVE_WITNESS_CONSISTENCY_AUDIT_OUT.mat');
assert(logical(C.closed));
assert(logical(G.closed)&&strcmp(char(G.diagnosis), ...
    'NUMERICAL_SPLIT_THEN_DIFFERENTIATE_WITNESS_ARTIFACT'));

[E,M,AUX,authoritySource]=AF7R52_LOAD_TRUE_TRIGGER_AUTHORITY(replanner); %#ok<ASGLU>
assert(isstruct(E)&&logical(E.valid)&&isstruct(M)&&logical(M.certified));

fprintf(' AF7R57C closed                          = 1\n');
fprintf(' AF7R57G numerical-artifact closure      = 1\n');
fprintf(' AF7R52 authority                        = %s\n',char(string(authoritySource)));

%% B. Exact source binding
fprintf('\n[B. EXACT SOURCE BINDING]\n');

core=find_source('af7r56_trace_core.hpp');
mexsrc=find_source('af7r56_trace_mex.cpp');
rps=which('REPLANNING_SYSTEM');

expectedCore='9f1c7054bbac0dadc2e82fbb107183937143b39431d8996ee13a620cc62ef821';
expectedMex ='cef987c012982982d2fc4381419781770fe9304c6a2ba4f1d7cff1bba2746f23';
expectedRps ='223d34c49d7148b8ea14ff4ab9c420d409560c0650ac19e9b0f99318887090c9';

coreSHA=sha256_file(core);
mexSHA=sha256_file(mexsrc);
rpsSHA=sha256_file(rps);

sourceClosed=strcmpi(coreSHA,expectedCore)&&strcmpi(mexSHA,expectedMex)&&strcmpi(rpsSHA,expectedRps);
fprintf(' core source                              = %s\n',core);
fprintf(' core SHA current                        = %d\n',strcmpi(coreSHA,expectedCore));
fprintf(' serializer SHA current                  = %d\n',strcmpi(mexSHA,expectedMex));
fprintf(' REPLANNING_SYSTEM SHA current           = %d\n',strcmpi(rpsSHA,expectedRps));
assert(sourceClosed,'[AF7R57K] bound source changed.');

%% C. Generate derivative-first shadow source
fprintf('\n[C. GENERATE SHADOW CORE]\n');

root=fileparts(mfilename('fullpath'));
shadowH=fullfile(root,'af7r57k_trace_core.hpp');
shadowCpp=fullfile(root,'af7r57k_corrected_core_mex.cpp');
shadowName='af7r57k_corrected_core_mex';

src=fileread(core);

stableSpeed = stable_speed_source();
stableTrace = stable_trace_source();

[src,n1]=replace_cpp_function(src, ...
    'static inline SpeedCert stageB_speed(',stableSpeed);
[src,n2]=replace_cpp_function(src, ...
    'static inline StageBTrace stageB_trace(',stableTrace);

assert(n1==1&&n2==1,'[AF7R57K] Stage-B patch count mismatch.');

headerChanged=write_if_changed(shadowH,src);

cpp=fileread(mexsrc);
oldInc='#include "af7r56_trace_core.hpp"';
newInc='#include "af7r57k_trace_core.hpp"';
assert(count(string(cpp),oldInc)==1,'[AF7R57K] include anchor mismatch.');
cpp=strrep(cpp,oldInc,newInc);
cppChanged=write_if_changed(shadowCpp,cpp);

fprintf(' shadow header content changed            = %d\n',headerChanged);
fprintf(' shadow MEX source content changed        = %d\n',cppChanged);

fprintf(' stageB_speed patched                    = %d\n',n1);
fprintf(' stageB_trace patched                    = %d\n',n2);
fprintf(' shadow header SHA                       = %s\n',sha256_file(shadowH));
fprintf(' shadow MEX source SHA                   = %s\n',sha256_file(shadowCpp));

% Source-level contract: old split->make_vleaf sequence must be absent from
% corrected functions, while derivative CP splitting must be present.
s2=fileread(shadowH);
stableTokens=contains(s2,'struct StableVLeaf') && ...
             contains(s2,'split_half(old.C1)') && ...
             contains(s2,'split_half(old.C2)') && ...
             contains(s2,'split_half(old.C3)') && ...
             contains(s2,'make_stable_parent_leaf');
fprintf(' derivative-first source contract         = %d\n',stableTokens);
assert(stableTokens);

%% D. Build shadow MEX
fprintf('\n[D. BUILD SHADOW MEX]\n');

needBuild=exist(shadowName,'file')~=3 || headerChanged || cppChanged;
if ~needBuild
    q=which(shadowName);
    needBuild=isempty(q);
end
if needBuild
    inc=['-I' root];
    if ispc
        stdflag='COMPFLAGS=$COMPFLAGS /std:c++17';
    else
        stdflag='CXXFLAGS=$CXXFLAGS -std=c++17';
    end
    mex('-R2018a',inc,stdflag,'-outdir',root,'-output',shadowName,shadowCpp);
    rehash;
end
assert(exist(shadowName,'file')==3);
fprintf(' corrected shadow MEX available          = 1\n');

% Production inputs must still be unchanged after source generation/build.
assert(strcmpi(sha256_file(core),expectedCore));
assert(strcmpi(sha256_file(mexsrc),expectedMex));
assert(strcmpi(sha256_file(rps),expectedRps));
fprintf(' production source SHA unchanged         = 1\n');

%% E. Prime current nominal AST
fprintf('\n[E. PRIME NOMINAL AST]\n');

[expr,signature]=nominal_ast(replanner,M);
ok=af7r57k_corrected_core_mex(0,expr,signature);
assert(logical(ok),'[AF7R57K] corrected shadow AST prime failed.');

fprintf(' nominal signature                       = %s\n',signature);
fprintf(' nominal signature equal                 = 1\n');

%% F. Fixed live/source-derived tU packet
fprintf('\n[F. FIXED live-tU PACKET]\n');

TR=C.trace_records;
rt1=record_for_candidate(TR,1);
pkt=rt1.pkt;
tU=double(rt1.tU);

assert(strcmp(signature,char(pkt.nominal_signature)));
assert(isfinite(tU));
fprintf(' tU                                      = %.17g\n',tU);
fprintf(' tU hardcoded                            = 0\n');
fprintf(' obstacle count                          = %d\n',size(pkt.centers,2));

%% G. Original vs corrected native core
fprintf('\n[G. ORIGINAL vs CORRECTED NATIVE CORE]\n');

P=pkt;P.t1=tU;

tic;
Rold=af7r36_single_core_mex(1,P);
oldOne=1000*toc;

tic;
Rnew=af7r57k_corrected_core_mex(4,pkt,tU,tU);
newOne=1000*toc;

fprintf(' old R36 success / reason                = %d / %.0f\n', ...
    logical(Rold.success),double(Rold.reason_code));
fprintf(' corrected success / reason              = %d / %.0f\n', ...
    logical(Rnew.success),double(Rnew.reason_code));
fprintf(' old / corrected one-shot [ms]           = %.6f / %.6f\n',oldOne,newOne);

% Candidate synthesis identity before speed/geometry acceptance.
id=compare_candidate_identity(Rold,Rnew);
fprintf(' candidate segment-time identity         = %d\n',id.segment_times);
fprintf(' candidate control-point identity        = %d\n',id.control_points);
fprintf(' candidate control-point max |diff|      = %.17g\n',id.control_point_max_abs_error);
fprintf(' candidate corridor-sign identity        = %d\n',id.signs);
fprintf(' candidate active-set identity           = %d\n',id.active_set);
fprintf(' candidate synthesis identity            = %d\n',id.all);
assert(id.all,'[AF7R57K] corrected core changed candidate synthesis.');

fprintf('\n[H. CORRECTED CERTIFICATES]\n');

rhoBody=max([double(pkt.r_drone), ...
    double(pkt.cable_length)+double(pkt.r_load), ...
    double(pkt.cable_length)+double(pkt.r_cable)]);
VQlim=(double(pkt.sensor_range)-rhoBody)/(double(pkt.horizon)+double(pkt.dt));

speedClosed=logical(Rnew.success) && isfinite(double(Rnew.speed_uav)) && ...
    isfinite(double(Rnew.speed_sigma)) && ...
    double(Rnew.speed_sigma)>0 && double(Rnew.speed_uav)<VQlim;

geomClosed=logical(Rnew.geometry_pass) && ...
    double(Rnew.geometry_min_payload_margin)>0 && ...
    double(Rnew.geometry_min_uav_margin)>0 && ...
    double(Rnew.geometry_min_cable_margin)>0;

rejoinClosed=double(Rnew.rejoin_pass_count)>=1;

fprintf(' corrected Stage-B VQ_UB                = %.17g\n',double(Rnew.speed_uav));
fprintf(' strict VQ limit                         = %.17g\n',VQlim);
fprintf(' corrected speed margin                  = %.17g\n',VQlim-double(Rnew.speed_uav));
fprintf(' corrected sigma_LB                      = %.17g\n',double(Rnew.speed_sigma));
fprintf(' corrected speed certificate             = %d\n',speedClosed);
fprintf(' corrected geometry L/Q/C [m]            = [%.9g %.9g %.9g]\n', ...
    double(Rnew.geometry_min_payload_margin), ...
    double(Rnew.geometry_min_uav_margin), ...
    double(Rnew.geometry_min_cable_margin));
fprintf(' corrected whole-body geometry           = %d\n',geomClosed);
fprintf(' corrected rejoin                        = %d\n',rejoinClosed);

% Optional cross-check against the already closed R57J outward point speed.
outwardContains=false;
J=[];
if evalin('base','exist(''AF7R57J_FIXED_TU_OUTWARD_PROVISIONAL_GATE_OUT'',''var'')==1')
    J=evalin('base','AF7R57J_FIXED_TU_OUTWARD_PROVISIONAL_GATE_OUT');
elseif isfile(fullfile(root,'AF7R57J_FIXED_TU_OUTWARD_PROVISIONAL_GATE_OUT.mat'))
    q=load(fullfile(root,'AF7R57J_FIXED_TU_OUTWARD_PROVISIONAL_GATE_OUT.mat'),'OUT');
    J=q.OUT;
end
if isstruct(J)&&isfield(J,'candidates')&&~isempty(J.candidates) && ...
        numel(J.candidates)>=1 && isstruct(J.candidates{1}) && ...
        isfield(J.candidates{1},'speed') && isfield(J.candidates{1}.speed,'VQ_ub')
    oq=double(J.candidates{1}.speed.VQ_ub);
    outwardContains=double(Rnew.speed_uav)<=oq && oq<VQlim;
    fprintf(' R57J outward VQ_UB                     = %.17g\n',oq);
    fprintf(' corrected native <= outward VQ_UB      = %d\n',outwardContains);
else
    fprintf(' R57J outward point cross-check           = NOT_AVAILABLE_IN_BASE\n');
end

%% I. Warm native timing
fprintf('\n[I. WARM NATIVE TIMING]\n');

% Prime caches.
for k=1:10
    af7r57k_corrected_core_mex(4,pkt,tU,tU);
end

N=200;
tm=zeros(N,1);
for k=1:N
    tt=tic;
    af7r57k_corrected_core_mex(4,pkt,tU,tU);
    tm(k)=1000*toc(tt);
end

q50=median(tm);
q95=prctile(tm,95);
mx=max(tm);

fprintf(' warm trials                              = %d\n',N);
fprintf(' corrected min / median / p95 / max [ms] = %.6f / %.6f / %.6f / %.6f\n', ...
    min(tm),q50,q95,mx);
fprintf(' <=4 ms observed all warm trials          = %d\n',all(tm<4));
fprintf(' hard QPC/ProcessCycleTime authority      = 0\n');

%% J. Formal provisional gate
fprintf('\n[J. FORMAL PROVISIONAL GATE]\n');

mathClosed=id.all&&speedClosed&&geomClosed&&rejoinClosed&&sourceClosed;
if ~isempty(J) && isstruct(J) && isfield(J,'closed') && logical(J.closed)
    % If outward point authority is available, require containment.
    mathClosed=mathClosed&&outwardContains;
end

fprintf(' candidate synthesis unchanged            = %d\n',id.all);
fprintf(' corrected derivative-first speed         = %d\n',speedClosed);
fprintf(' continuous whole-body geometry           = %d\n',geomClosed);
fprintf(' continuous rejoin                        = %d\n',rejoinClosed);
fprintf(' sampled acceptance                       = 0\n');
fprintf(' fixed-tU corrected native theorem        = %d\n',mathClosed);
fprintf(' production source modified               = 0\n');
fprintf(' production t1 selected                   = 0\n');
fprintf(' hard <=4 ms timing gate closed           = 0\n');

if mathClosed && all(tm<4)
    next='BUILD_QPC_PROCESSTIME_4MS_GATE_FOR_CORRECTED_NATIVE_SHADOW';
elseif mathClosed
    next='PROFILE_CORRECTED_NATIVE_SHADOW_INTERNAL_PHASES_BEFORE_QPC_GATE';
else
    next='FAIL_CLOSED__INSPECT_CORRECTED_SPEED_GEOMETRY_REJOIN_OR_IDENTITY';
end
fprintf(' next action                              = %s\n',next);
fprintf('====================================================================================================\n');

OUT.closed=mathClosed;
OUT.tU=tU;
OUT.original=Rold;
OUT.corrected=Rnew;
OUT.identity=id;
OUT.speed_closed=speedClosed;
OUT.geometry_closed=geomClosed;
OUT.rejoin_closed=rejoinClosed;
OUT.outward_point_contains=outwardContains;
OUT.warm_ms=tm;
OUT.production_modified=false;
OUT.production_t1_selected=false;
OUT.hard_4ms_gate_closed=false;
OUT.next_action=next;

assignin('base','AF7R57K_CORRECTED_NATIVE_CORE_AUDIT_OUT',OUT);
save(fullfile(root,'AF7R57K_CORRECTED_NATIVE_CORE_AUDIT_OUT.mat'),'OUT','-v7.3');
end


function S=stable_speed_source()
S = [
"struct StableVLeaf{" newline ...
"    VLeaf f; Poly C1,C2,C3;" newline ...
"};" newline ...
"static inline StableVLeaf make_stable_leaf(const ActiveReference&ref,const Live&L,const SpeedCert&base," newline ...
"                                           const Poly&C0,const Poly&C1,const Poly&C2,const Poly&C3," newline ...
"                                           double ta,double tb,int seg){" newline ...
"    StableVLeaf z;VLeaf&f=z.f;f.seg=seg;f.ta=ta;f.tb=tb;f.C0=C0;z.C1=C1;z.C2=C2;z.C3=C3;" newline ...
"    double T=tb-ta;if(!(fin(T)&&T>0)||C1.empty()||C2.empty()||C3.empty()){f.vq=std::numeric_limits<double>::infinity();return z;}" newline ...
"    double tc=0.5*(ta+tb),h=0.5*T;auto x=ref.eval(tc);Vec3 v0=x[1],a0=x[2],j0=x[3];" newline ...
"    f.vl=0;for(auto v:C1)f.vl=std::max(f.vl,norm(v+v0));f.vl+=base.A*h;" newline ...
"    f.al=0;for(auto a:C2)f.al=std::max(f.al,norm(a+a0));f.al+=base.J*h;" newline ...
"    Vec3 d1=decasteljau(C1,0.5),d2=decasteljau(C2,0.5),d3=decasteljau(C3,0.5);" newline ...
"    f.jl=base.J+poly_norm_ub(C3);" newline ...
"    Vec3 sm=a0+d2+Vec3{0,0,L.g};f.sigma=norm(sm)-f.jl*h;" newline ...
"    if(!(fin(f.sigma)&&f.sigma>0)){f.qdot=f.vq=std::numeric_limits<double>::infinity();}" newline ...
"    else{f.qdot=f.jl/f.sigma;f.vq=f.vl+L.L*f.qdot;}" newline ...
"    Vec3 vm=v0+d1,am=a0+d2,jm=j0+d3,s=am+Vec3{0,0,L.g};double ns=norm(s);" newline ...
"    if(fin(ns)&&ns>0){Vec3 q=s*(-1.0/ns);Vec3 proj=jm-q*dot(q,jm);Vec3 qd=proj*(-1.0/ns);f.wit=norm(vm-qd*L.L);}" newline ...
"    return z;" newline ...
"}" newline ...
"static inline StableVLeaf make_stable_parent_leaf(const ActiveReference&ref,const Live&L,const SpeedCert&base," newline ...
"                                                  const Poly&C0,double ta,double tb,int seg){" newline ...
"    double T=tb-ta;Poly C1=deriv_poly(C0,T,1),C2=deriv_poly(C0,T,2),C3=deriv_poly(C0,T,3);" newline ...
"    return make_stable_leaf(ref,L,base,C0,C1,C2,C3,ta,tb,seg);" newline ...
"}" newline ...
"static inline std::pair<StableVLeaf,StableVLeaf> split_stable_leaf(const ActiveReference&ref,const Live&L,const SpeedCert&base," newline ...
"                                                                  const StableVLeaf&old,double tm){" newline ...
"    auto p0=split_half(old.f.C0);auto p1=split_half(old.C1);auto p2=split_half(old.C2);auto p3=split_half(old.C3);" newline ...
"    return {make_stable_leaf(ref,L,base,p0.first,p1.first,p2.first,p3.first,old.f.ta,tm,old.f.seg)," newline ...
"            make_stable_leaf(ref,L,base,p0.second,p1.second,p2.second,p3.second,tm,old.f.tb,old.f.seg)};" newline ...
"}" newline ...
"static inline SpeedCert stageB_speed(const ActiveReference&ref,const Live&L,const SpeedCert&base," newline ...
"                                     const Candidate&C,double lim,bool&finiteWitness,bool&repr," newline ...
"                                     double* witnessMaxOut=nullptr){" newline ...
"    std::vector<StableVLeaf> leaves;for(size_t s=0;s<C.cp.size();++s)leaves.push_back(make_stable_parent_leaf(ref,L,base,C.cp[s],C.seg0[s],C.seg1[s],(int)s));" newline ...
"    if(witnessMaxOut){*witnessMaxOut=-std::numeric_limits<double>::infinity();for(auto&z:leaves)if(fin(z.f.wit))*witnessMaxOut=std::max(*witnessMaxOut,z.f.wit);}" newline ...
"    int splits=0;" newline ...
"    while(true){" newline ...
"        bool all=true;for(auto&z:leaves)if(!(fin(z.f.vq)&&z.f.vq<lim)){all=false;break;}if(all)break;" newline ...
"        for(auto&z:leaves)if(fin(z.f.wit)&&z.f.wit>=lim){finiteWitness=true;if(witnessMaxOut)*witnessMaxOut=std::max(*witnessMaxOut,z.f.wit);return {};}" newline ...
"        int bi=-1;double bs=-std::numeric_limits<double>::infinity();" newline ...
"        for(int i=0;i<(int)leaves.size();++i)if(!fin(leaves[i].f.vq)||leaves[i].f.vq>=lim){double sc=fin(leaves[i].f.vq)?leaves[i].f.vq-lim:std::numeric_limits<double>::infinity();if(sc>bs){bs=sc;bi=i;}}" newline ...
"        if(bi<0)break;auto old=leaves[bi];double tm=0.5*(old.f.ta+old.f.tb);" newline ...
"        if(!(fin(tm)&&tm>old.f.ta&&tm<old.f.tb)){repr=true;return {};}" newline ...
"        auto sp=split_stable_leaf(ref,L,base,old,tm);leaves[bi]=std::move(sp.first);leaves.push_back(std::move(sp.second));++splits;" newline ...
"        if(witnessMaxOut){if(fin(leaves[bi].f.wit))*witnessMaxOut=std::max(*witnessMaxOut,leaves[bi].f.wit);if(fin(leaves.back().f.wit))*witnessMaxOut=std::max(*witnessMaxOut,leaves.back().f.wit);}" newline ...
"    }" newline ...
"    SpeedCert s;s.certified=true;s.stageA=false;s.leaf_count=(int)leaves.size();s.split_count=splits;" newline ...
"    s.VQ=0;s.VL=0;s.A=0;s.J=0;s.sigma=std::numeric_limits<double>::infinity();s.qdot=0;" newline ...
"    for(auto&z:leaves){auto&f=z.f;if(!fin(f.vq)||!fin(f.sigma)||f.sigma<=0){s.certified=false;return s;}s.VQ=std::max(s.VQ,f.vq);s.VL=std::max(s.VL,f.vl);s.A=std::max(s.A,f.al);s.J=std::max(s.J,f.jl);s.sigma=std::min(s.sigma,f.sigma);s.qdot=std::max(s.qdot,f.qdot);}" newline ...
"    if(!(s.VQ<lim&&s.sigma>0))s.certified=false;return s;" newline ...
"}" newline ...
];
S=char(join(S,""));
end


function S=stable_trace_source()
S = [
"static inline StageBTrace stageB_trace(const ActiveReference&ref,const Live&L,const SpeedCert&base," newline ...
"                                       const Candidate&C,double lim){" newline ...
"    StageBTrace R;std::vector<StableVLeaf> leaves;" newline ...
"    for(size_t s=0;s<C.cp.size();++s){auto z=make_stable_parent_leaf(ref,L,base,C.cp[s],C.seg0[s],C.seg1[s],(int)s);if(fin(z.f.wit))R.witness_max=std::max(R.witness_max,z.f.wit);leaves.push_back(std::move(z));}" newline ...
"    while(true){" newline ...
"        bool all=true;for(auto&z:leaves)if(!(fin(z.f.vq)&&z.f.vq<lim)){all=false;break;}if(all)break;" newline ...
"        for(auto&z:leaves)if(fin(z.f.wit)&&z.f.wit>=lim){R.finiteWitness=true;R.leaves.clear();for(auto&q:leaves)R.leaves.push_back(q.f);return R;}" newline ...
"        int bi=-1;double bs=-std::numeric_limits<double>::infinity();" newline ...
"        for(int i=0;i<(int)leaves.size();++i)if(!fin(leaves[i].f.vq)||leaves[i].f.vq>=lim){double sc=fin(leaves[i].f.vq)?leaves[i].f.vq-lim:std::numeric_limits<double>::infinity();if(sc>bs){bs=sc;bi=i;}}" newline ...
"        if(bi<0)break;auto old=leaves[bi];double tm=0.5*(old.f.ta+old.f.tb);" newline ...
"        if(!(fin(tm)&&tm>old.f.ta&&tm<old.f.tb)){R.repr=true;R.leaves.clear();for(auto&q:leaves)R.leaves.push_back(q.f);return R;}" newline ...
"        auto sp=split_stable_leaf(ref,L,base,old,tm);leaves[bi]=std::move(sp.first);leaves.push_back(std::move(sp.second));++R.splits;" newline ...
"        if(fin(leaves[bi].f.wit))R.witness_max=std::max(R.witness_max,leaves[bi].f.wit);if(fin(leaves.back().f.wit))R.witness_max=std::max(R.witness_max,leaves.back().f.wit);" newline ...
"    }" newline ...
"    SpeedCert s;s.certified=true;s.stageA=false;s.leaf_count=(int)leaves.size();s.split_count=R.splits;" newline ...
"    s.VQ=0;s.VL=0;s.A=0;s.J=0;s.sigma=std::numeric_limits<double>::infinity();s.qdot=0;" newline ...
"    R.leaves.clear();" newline ...
"    for(auto&z:leaves){auto&f=z.f;R.leaves.push_back(f);if(!fin(f.vq)||!fin(f.sigma)||f.sigma<=0){s.certified=false;R.speed=s;return R;}s.VQ=std::max(s.VQ,f.vq);s.VL=std::max(s.VL,f.vl);s.A=std::max(s.A,f.al);s.J=std::max(s.J,f.jl);s.sigma=std::min(s.sigma,f.sigma);s.qdot=std::max(s.qdot,f.qdot);}" newline ...
"    if(!(s.VQ<lim&&s.sigma>0))s.certified=false;R.speed=s;R.certified=s.certified;return R;" newline ...
"}" newline ...
];
S=char(join(S,""));
end


function [src,n]=replace_cpp_function(src,marker,repl)
p=strfind(src,marker);
assert(numel(p)==1,'[AF7R57K] function marker count for %s is %d.',marker,numel(p));
p=p(1);
ob=find(src(p:end)=='{',1,'first')+p-1;
assert(~isempty(ob));
depth=0;cb=NaN;
for k=ob:numel(src)
    ch=src(k);
    if ch=='{'
        depth=depth+1;
    elseif ch=='}'
        depth=depth-1;
        if depth==0
            cb=k;break;
        end
    end
end
assert(isfinite(cb),'[AF7R57K] unmatched C++ braces.');
src=[src(1:p-1) repl src(cb+1:end)];
n=1;
end


function id=compare_candidate_identity(A,B)
% Serializer contract:
%   control_points is a 1xN CELL ARRAY, each cell a 3x16 double matrix.
% Compare the candidate before/after the Stage-B-only patch exactly.
%
% No tolerance is used for the Boolean identity.  max_abs_error is reported
% diagnostically and must be exactly zero when identity closes.

id=struct('segment_times',false,'control_points',false, ...
    'control_point_max_abs_error',Inf, ...
    'signs',false,'active_set',false,'all',false);

if isfield(A,'segment_t0')&&isfield(B,'segment_t0')&& ...
   isfield(A,'segment_t1')&&isfield(B,'segment_t1')
    id.segment_times=isequaln(double(A.segment_t0),double(B.segment_t0)) && ...
                     isequaln(double(A.segment_t1),double(B.segment_t1));
end

if isfield(A,'control_points')&&isfield(B,'control_points')
    [id.control_points,id.control_point_max_abs_error]= ...
        exact_control_point_cells(A.control_points,B.control_points);
end

if isfield(A,'corridor_signs')&&isfield(B,'corridor_signs')
    id.signs=isequaln(double(A.corridor_signs),double(B.corridor_signs));
end

if isfield(A,'qp_active_set')&&isfield(B,'qp_active_set')
    id.active_set=isequaln(double(A.qp_active_set),double(B.qp_active_set));
end

id.all=id.segment_times&&id.control_points && ...
       (~(isfield(A,'corridor_signs')&&isfield(B,'corridor_signs'))||id.signs) && ...
       (~(isfield(A,'qp_active_set')&&isfield(B,'qp_active_set'))||id.active_set);
end


function [eq,err]=exact_control_point_cells(A,B)
eq=false;err=Inf;

% Accept the documented serializer cell form. Numeric fallback is supported
% only to make the audit robust to an equivalent future serializer.
if iscell(A) || iscell(B)
    if ~(iscell(A)&&iscell(B)&&isequal(size(A),size(B)))
        return;
    end
    err=0;
    for k=1:numel(A)
        if ~(isnumeric(A{k})&&isnumeric(B{k})&&isequal(size(A{k}),size(B{k})))
            eq=false;err=Inf;return;
        end
        a=double(A{k});b=double(B{k});
        if isempty(a)&&isempty(b)
            e=0;
        elseif any(~isfinite(a(:))) || any(~isfinite(b(:)))
            if ~isequaln(a,b),eq=false;err=Inf;return;end
            e=0;
        else
            e=max(abs(a(:)-b(:)));
        end
        err=max(err,e);
        if ~isequaln(a,b)
            eq=false;return;
        end
    end
    eq=true;
    return;
end

if isnumeric(A)&&isnumeric(B)&&isequal(size(A),size(B))
    a=double(A);b=double(B);
    eq=isequaln(a,b);
    if isempty(a)&&isempty(b)
        err=0;
    elseif all(isfinite(a(:)))&&all(isfinite(b(:)))
        err=max(abs(a(:)-b(:)));
    elseif eq
        err=0;
    end
end
end


function [expr,signature]=nominal_ast(replanner,M)
F=replanner.nominal_flat_derivative_fn(1:7);
assert(iscell(F)&&numel(F)>=7&&all(cellfun(@(f)isa(f,'function_handle'),F)));
S=cellfun(@func2str,F,'UniformOutput',false);
signature=sha256_text(strjoin(S,char(10)));
assert(strcmp(signature,char(M.nominal_signature)));

expr=cell(21,1);kk=0;
for r=1:7
    ss=strtrim(S{r});
    tok=regexp(ss,'^@\(([A-Za-z]\w*)\)(.*)$','tokens','once');
    assert(~isempty(tok)&&numel(tok)==2);
    body=strtrim(char(tok{2}));
    body=strrep(body,'.^','^');body=strrep(body,'.*','*');body=strrep(body,'./','/');
    z=str2sym(body);z=z(:);assert(numel(z)==3);
    t=sym('t','real');sv0=symvar(z);assert(numel(sv0)<=1);
    if ~isempty(sv0),z=subs(z,sv0(1),t);end
    sv=symvar(z);for j=1:numel(sv),assert(strcmp(char(sv(j)),'t'));end
    for j=1:3,kk=kk+1;expr{kk}=char(z(j));end
end
end


function r=record_for_candidate(R,c)
ids=cellfun(@(q)double(q.c),R);
k=find(ids==c,1,'last');assert(~isempty(k));r=R{k};
end


function A=get_authority(varName,fileName)
if evalin('base',sprintf('exist(''%s'',''var'')==1',varName))
    A=evalin('base',varName);return;
end
cand={which(fileName),fullfile(pwd,fileName),fullfile(fileparts(mfilename('fullpath')),fileName)};
u=getenv('USERPROFILE');
if ~isempty(u),cand{end+1}=fullfile(u,'Documents','GitHub','common_matlab','reference',fileName);end %#ok<AGROW>
p='';
for i=1:numel(cand)
    if ~isempty(cand{i})&&isfile(cand{i}),p=cand{i};break;end
end
assert(~isempty(p),'[AF7R57K] authority unavailable: %s',fileName);
q=load(p,'OUT');assert(isfield(q,'OUT'));A=q.OUT;
end


function fn=find_source(name)
fn='';
p=which(name);
if ~isempty(p)&&isfile(p),fn=p;return;end
roots={pwd,fileparts(mfilename('fullpath'))};
u=getenv('USERPROFILE');
if ~isempty(u),roots{end+1}=fullfile(u,'Documents','GitHub','common_matlab','reference');end %#ok<AGROW>
for i=1:numel(roots)
    q=fullfile(roots{i},name);
    if isfile(q),fn=q;return;end
end
assert(false,'[AF7R57K] source unavailable: %s',name);
end


function changed=write_if_changed(fn,txt)
changed=true;
if isfile(fn)
    old=fileread(fn);
    if isequal(old,txt)
        changed=false;
        return;
    end
end
fid=fopen(fn,'w');assert(fid>=0);
c=onCleanup(@()fclose(fid)); %#ok<NASGU>
fwrite(fid,txt,'char');
end


function h=sha256_text(s)
md=java.security.MessageDigest.getInstance('SHA-256');
md.update(uint8(unicode2native(char(s),'UTF-8')));
d=typecast(md.digest(),'uint8');
h=lower(reshape(dec2hex(d,2).',1,[]));
end


function h=sha256_file(fn)
md=java.security.MessageDigest.getInstance('SHA-256');
fid=fopen(fn,'rb');assert(fid>=0);c=onCleanup(@()fclose(fid));
while true
    x=fread(fid,1024*1024,'*uint8');if isempty(x),break;end;md.update(x);
end
d=typecast(md.digest(),'uint8');
h=lower(reshape(dec2hex(d,2).',1,[]));
end
