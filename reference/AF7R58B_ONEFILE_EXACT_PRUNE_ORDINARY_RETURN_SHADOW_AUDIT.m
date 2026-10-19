function OUT = AF7R58B_ONEFILE_EXACT_PRUNE_ORDINARY_RETURN_SHADOW_AUDIT()
% AF7R58B_ONEFILE_EXACT_PRUNE_ORDINARY_RETURN_SHADOW_AUDIT
% =========================================================================
% PURPOSE
%   Move past AF7R58A source-location false negatives and implement, in
%   immutable shadows only, the two closed mathematical operations that are
%   now justified:
%
%   (1) EXACT EXPIRED-CORRECTION PRUNING
%         C_g.t1 <= t_now  =>  C_g contributes exactly zero for all t>=t_now
%       because AF7R36 ActiveReference uses strict support t0 < t < t1.
%
%   (2) ORDINARY OP=1 RESOURCE REJECTION
%       Over-G / over-S is returned as success=false with resource reason,
%       rather than escaping through the outer exception wrapper.
%
% IMPORTANT
%   - AF7R36 core definitions live in af7r36_core.hpp, while op dispatch and
%     MATLAB serialization live in af7r36_single_core_mex.cpp. Both are bound.
%   - GMAX_TEST/SMAX_TEST are AUDIT-ONLY. No production cap is selected.
%   - No production source/MEX is modified or overwritten.
%   - No sampled safety acceptance is introduced.
%   - No production end-to-end 4 ms theorem is claimed.
% =========================================================================

clc;
fprintf('\n====================================================================================================\n');
fprintf(' AF7R58B | ONE-FILE EXACT EXPIRED PRUNE + ORDINARY OP1 RESOURCE RETURN SHADOW AUDIT\n');
fprintf('====================================================================================================\n');
fprintf('[AUTHORITY] headerPlusWrapper=1 | exactPrune=1 | ordinaryReturnShadow=1 | noCapGuess=1 | productionModified=0\n');

root=fileparts(mfilename('fullpath'));
self=[mfilename('fullpath') '.m'];
cpp=fullfile(root,'af7r36_single_core_mex.cpp');
hpp=fullfile(root,'af7r36_core.hpp');
a33=fullfile(root,'af7r33_make_batch_model_from_proposal.m');
assert(isfile(cpp),'[AF7R58B] af7r36_single_core_mex.cpp missing.');
assert(isfile(hpp),'[AF7R58B] af7r36_core.hpp missing.');
assert(isfile(a33),'[AF7R58B] af7r33_make_batch_model_from_proposal.m missing.');

fprintf(' entrypoint                               = %s\n',self);
fprintf(' entrypoint SHA-256                        = %s\n',sha256_file(self));

cpp0=fileread(cpp); hpp0=fileread(hpp); a330=fileread(a33);
ncpp=compact_source(cpp0); nhpp=compact_source(hpp0); na33=compact_source(a330);
expectedCpp='4f9778723aa3aa339d5167b90845fcfcaa8564cae68326ff1e4ed504c7e6daea';
expectedA33='f0e9f487cd270430d191871366c84595be586b6da0fb7b93eb98f4dfc25b46fd';
cppCurrent=strcmp(sha256_file(cpp),expectedCpp);
a33Current=strcmp(sha256_file(a33),expectedA33);
includeBound=contains(ncpp,'#include"af7r36_core.hpp"');
activeBound=contains(nhpp,'structActiveReference{') && ...
    contains(nhpp,'if(!(t>C.t0&&t<C.t1))continue;');
planBound=contains(nhpp,'structPlanResult{') && ...
    contains(nhpp,'boolsuccess=false;intreason=0;');
appendBound=contains(na33,'model.corrections(end+1)=C;');

fprintf('\n[A. EXACT SOURCE OWNERSHIP / BINDING]\n');
fprintf(' AF7R36 wrapper SHA current              = %d | %s\n',cppCurrent,sha256_file(cpp));
fprintf(' AF7R36 core header SHA                  = %s\n',sha256_file(hpp));
fprintf(' AF7R33 SHA current                      = %d | %s\n',a33Current,sha256_file(a33));
fprintf(' wrapper includes af7r36_core.hpp        = %d\n',includeBound);
fprintf(' ActiveReference strict support in header= %d\n',activeBound);
fprintf(' PlanResult success/reason in header     = %d\n',planBound);
fprintf(' AF7R33 append semantics current          = %d\n',appendBound);
sourceClosed=cppCurrent&&a33Current&&includeBound&&activeBound&&planBound&&appendBound;
fprintf(' header+wrapper source authority closed   = %d\n',sourceClosed);
assert(sourceClosed,'[AF7R58B] current header/wrapper/source authority not closed.');

% -------------------------------------------------------------------------
% B. Build exact parent clone for native evaluator identity checks.
% -------------------------------------------------------------------------
parentMex='af7r58b_parent_eval_mex';
shadowMex='af7r58b_ordinary_return_mex';
shadowCpp=fullfile(root,'af7r58b_ordinary_return_mex.cpp');

% Generate shadow wrapper from exact current cpp.
GMAX_TEST=4; SMAX_TEST=8;
patched=patch_op1_ordinary_return(cpp0,GMAX_TEST,SMAX_TEST);
write_text_binary(shadowCpp,patched);
assert(strcmp(sha256_file(cpp),expectedCpp),'[AF7R58B] production cpp changed during shadow generation.');

fprintf('\n[B. IMMUTABLE SHADOW GENERATION]\n');
fprintf(' shadow cpp                              = %s\n',shadowCpp);
fprintf(' shadow cpp SHA-256                      = %s\n',sha256_file(shadowCpp));
fprintf(' Gmax/Smax                               = %d / %d | AUDIT TEST ONLY\n',GMAX_TEST,SMAX_TEST);
fprintf(' production cap selected                 = 0\n');
fprintf(' production source modified              = 0\n');

cd0=pwd; c=onCleanup(@()cd(cd0)); cd(root);
safe_clear_mex(parentMex); safe_clear_mex(shadowMex);
safe_delete_mex(parentMex,root); safe_delete_mex(shadowMex,root);
if ispc
    cxx17arg='COMPFLAGS="$COMPFLAGS /std:c++17"';
    cxx17label='MSVC /std:c++17';
else
    cxx17arg='CXXFLAGS="$CXXFLAGS -std=c++17"';
    cxx17label='CXX -std=c++17';
end
fprintf('\n[C. BUILD UNIQUE SHADOW MEX FILES]\n');
fprintf(' C++ standard                           = %s\n',cxx17label);
fprintf(' building parent evaluator              = %s\n',parentMex);
mex('-R2018a',cxx17arg,'-output',parentMex,cpp);
fprintf(' building ordinary-return shadow         = %s\n',shadowMex);
mex('-R2018a',cxx17arg,'-output',shadowMex,shadowCpp);
assert(exist(parentMex,'file')==3 && exist(shadowMex,'file')==3,'[AF7R58B] MEX build failed.');
fprintf(' parent clone built                      = 1\n');
fprintf(' ordinary-return shadow built            = 1\n');

expr=repmat({'0'},1,21);
sig='AF7R58B_ZERO_NOMINAL';
feval(parentMex,0,expr,sig);
feval(shadowMex,0,expr,sig);

% -------------------------------------------------------------------------
% D. Exact expired pruning on a nonzero canonical repair packet.
% -------------------------------------------------------------------------
P=build_prune_packet(sig);
tNow=4.0;
Pp=prune_expired_packet(P,tNow);

G0=numel(P.repair_t0); S0=size(P.repair_cp,2);
G1=numel(Pp.repair_t0); S1=size(Pp.repair_cp,2);
removedG=G0-G1; removedS=S0-S1;

% The theorem authority is source/order logic. Native eval is an exact
% implementation-equivalence regression on many future times, NOT safety
% acceptance by sampling.
tFuture=unique([tNow,linspace(tNow,9,401)]);
Y0=feval(parentMex,2,P,tFuture);
Y1=feval(parentMex,2,Pp,tFuture);
futureBitwise=isequaln(Y0,Y1);
futureMaxErr=max(abs(double(Y0(:))-double(Y1(:))));

% Demonstrate pruning can change the past; this is why t_now must be an
% explicit commit/evaluation lower bound and not guessed from P.t0.
tPast=linspace(1.05,tNow-0.05,97);
Z0=feval(parentMex,2,P,tPast);
Z1=feval(parentMex,2,Pp,tPast);
pastDifferent=~isequaln(Z0,Z1);

pruneTheorem=activeBound;
fprintf('\n[D. EXACT EXPIRED-CORRECTION PRUNING]\n');
fprintf(' explicit prune time t_now [s]           = %.9g\n',tNow);
fprintf(' before G/S                              = %d / %d\n',G0,S0);
fprintf(' after  G/S                              = %d / %d\n',G1,S1);
fprintf(' removed expired groups/segments         = %d / %d\n',removedG,removedS);
fprintf(' theorem: t1<=t_now => inactive for all future t = %d\n',pruneTheorem);
fprintf(' future native eval bitwise identical    = %d\n',futureBitwise);
fprintf(' future max numeric error                = %.3e\n',futureMaxErr);
fprintf(' past may differ (expected)              = %d\n',pastDifferent);
fprintf(' sampled safety acceptance used          = 0\n');
fprintf(' approximation/resampling used           = 0\n');
assert(pruneTheorem&&futureBitwise&&futureMaxErr==0,'[AF7R58B] exact prune equivalence failed.');

% -------------------------------------------------------------------------
% E. Ordinary op=1 over-G / over-S resource returns.
% Minimal packets intentionally omit live/speed/obstacle fields. A correct
% pre-parse resource return must return BEFORE any of those are required.
% -------------------------------------------------------------------------
PG=minimal_op1_packet(sig);
PG.repair_t0=zeros(1,GMAX_TEST+1);
PG.repair_t1=[]; PG.repair_offsets=[];
PG.repair_seg0=[]; PG.repair_seg1=[]; PG.repair_cp=zeros(1,1);
[okG,RG,msgG]=call_no_exception(@()feval(shadowMex,1,PG));

PS=minimal_op1_packet(sig);
PS.repair_t0=0; PS.repair_t1=0; PS.repair_offsets=[0 0];
PS.repair_cp=zeros(48,SMAX_TEST+1); PS.repair_seg0=[]; PS.repair_seg1=[];
[okS,RS,msgS]=call_no_exception(@()feval(shadowMex,1,PS));

gReturn=okG&&isstruct(RG)&&field_bool(RG,'success')==false&&field_num(RG,'reason_code')==218;
sReturn=okS&&isstruct(RS)&&field_bool(RS,'success')==false&&field_num(RS,'reason_code')==219;
wrapperFieldsG=has_failure_wrapper_fields(RG);
wrapperFieldsS=has_failure_wrapper_fields(RS);

fprintf('\n[E. ORDINARY OP=1 RESOURCE RETURN]\n');
fprintf(' over-G returned without exception       = %d\n',okG);
fprintf(' over-G success/reason                   = %d / %.0f\n',field_bool(RG,'success'),field_num(RG,'reason_code'));
fprintf(' over-G AF7R46 failure fields complete   = %d\n',wrapperFieldsG);
if ~okG, fprintf(' over-G exception                         = %s\n',msgG); end
fprintf(' over-S returned without exception       = %d\n',okS);
fprintf(' over-S success/reason                   = %d / %.0f\n',field_bool(RS,'success'),field_num(RS,'reason_code'));
fprintf(' over-S AF7R46 failure fields complete   = %d\n',wrapperFieldsS);
if ~okS, fprintf(' over-S exception                         = %s\n',msgS); end
ordinaryClosed=gReturn&&sReturn&&wrapperFieldsG&&wrapperFieldsS;
fprintf(' ordinary resource-return transport closed= %d\n',ordinaryClosed);
fprintf(' outer exception transport used          = %d\n',~(okG&&okS));
assert(ordinaryClosed,'[AF7R58B] ordinary op1 resource return failed.');

% -------------------------------------------------------------------------
% F. Admitted semantics remain unchanged. Shadow op=2 is not modified.
% -------------------------------------------------------------------------
tAll=linspace(0,9,301);
A0=feval(parentMex,2,P,tAll);
A1=feval(shadowMex,2,P,tAll);
admittedSame=isequaln(A0,A1);
fprintf('\n[F. ADMITTED REFERENCE SEMANTICS]\n');
fprintf(' parent-vs-shadow op2 bitwise identical  = %d\n',admittedSame);
fprintf(' mathematical trajectory modified        = %d\n',~admittedSame);
assert(admittedSame,'[AF7R58B] admitted semantics changed.');

% -------------------------------------------------------------------------
% G. Production-cap theorem: derive FORM, not numeric cap.
% -------------------------------------------------------------------------
% Exact pruning removes dead history. A finite simultaneous future chain
% still needs source-bound contracts:
%   Tcorr_max       : max correction support duration
%   Dcommit_min     : minimum time between committed repair versions
%   Cmax            : max simultaneously certified unsafe components
%
% Then one conservative finite contract is
%   Gmax <= ceil(Tcorr_max/Dcommit_min)+1
% and with AF7R57M candidate structure S_group <= 2*Cmax+1,
%   Smax <= Gmax*(2*Cmax+1).
% No values are selected until all three quantities are source/live bound.
fprintf('\n[G. PRODUCTION RESOURCE-CAP DERIVATION CONTRACT]\n');
fprintf(' exact expired pruning removes dead history = 1\n');
fprintf(' required live/source quantities          = Tcorr_max, Dcommit_min, Cmax\n');
fprintf(' Gmax derivation                          = ceil(Tcorr_max/Dcommit_min)+1\n');
fprintf(' per-group segment bound                  = 2*Cmax+1\n');
fprintf(' Smax derivation                          = Gmax*(2*Cmax+1)\n');
fprintf(' numeric production Gmax/Smax selected   = 0\n');
fprintf(' current observed G/S used as cap         = 0\n');
fprintf(' audit-only 4/8 promoted to cap           = 0\n');

% -------------------------------------------------------------------------
% H. Diagnostic timing of pruning itself only.
% -------------------------------------------------------------------------
N=5000; tp=zeros(1,N);
for k=1:N
    q=tic; prune_expired_packet(P,tNow); tp(k)=1e3*toc(q);
end
sp=sort(tp); p95i=max(1,ceil(.95*N));
fprintf('\n[H. PRUNE IMPLEMENTATION TIMING - DIAGNOSTIC ONLY]\n');
fprintf(' iterations                              = %d\n',N);
fprintf(' median / p95 / max [ms]                = %.6f / %.6f / %.6f\n',median(tp),sp(p95i),max(tp));
fprintf(' production end-to-end 4 ms upgraded     = 0\n');

closed=sourceClosed&&pruneTheorem&&futureBitwise&&ordinaryClosed&&admittedSame;
fprintf('\n[I. FINAL GATE]\n');
fprintf(' header+wrapper authority closed          = %d\n',sourceClosed);
fprintf(' exact expired-prune theorem/impl closed  = %d\n',pruneTheorem&&futureBitwise);
fprintf(' ordinary op1 resource return closed      = %d\n',ordinaryClosed);
fprintf(' admitted semantics unchanged             = %d\n',admittedSame);
fprintf(' production caps selected                 = 0\n');
fprintf(' production source modified               = 0\n');
fprintf(' production MEX overwritten               = 0\n');
fprintf(' production 4 ms theorem claimed          = 0\n');
fprintf(' AF7R58B PRUNE/RETURN SHADOW CLOSURE      = %d\n',closed);
fprintf(' next action                               = SOURCE_BIND_TCORR_MAX_DCOMMIT_MIN_CMAX__DERIVE_GMAX_SMAX__THEN_INSTALL_PRODUCTION_GUARD_AND_PRUNE__THEN_END_TO_END_QPC\n');
fprintf('====================================================================================================\n');

OUT=struct();
OUT.schema='AF7R58B_EXACT_PRUNE_ORDINARY_RETURN_SHADOW_V1';
OUT.closed=closed;
OUT.cpp_sha256=sha256_file(cpp);
OUT.hpp_sha256=sha256_file(hpp);
OUT.a33_sha256=sha256_file(a33);
OUT.shadow_cpp=shadowCpp;
OUT.shadow_sha256=sha256_file(shadowCpp);
OUT.prune_time=tNow;
OUT.before_G=G0; OUT.before_S=S0; OUT.after_G=G1; OUT.after_S=S1;
OUT.future_bitwise_equivalence=futureBitwise;
OUT.future_max_error=futureMaxErr;
OUT.ordinary_overG=gReturn; OUT.ordinary_overS=sReturn;
OUT.admitted_semantics_unchanged=admittedSame;
OUT.production_caps_selected=false;
OUT.production_modified=false;
OUT.production_4ms_claimed=false;
OUT.prune_timing_ms=tp;
OUT.entrypoint=self;
OUT.entrypoint_sha256=sha256_file(self);
assignin('base','AF7R58B_ONEFILE_EXACT_PRUNE_ORDINARY_RETURN_SHADOW_AUDIT_OUT',OUT);
save(fullfile(root,'AF7R58B_ONEFILE_EXACT_PRUNE_ORDINARY_RETURN_SHADOW_AUDIT_OUT.mat'),'OUT');
end

% =========================================================================
% C++ shadow patch.
% =========================================================================
function patched=patch_op1_ordinary_return(src,GMAX,SMAX)
% Add an AF7R46-compatible ordinary failure struct helper immediately before
% mexFunction, then replace only the op=1 dispatch block. op=0/op=2 remain
% byte-identical source text.
anchor='void mexFunction(';
p=strfind(src,anchor);
assert(numel(p)==1,'[AF7R58B] mexFunction anchor count !=1.');
helper=sprintf([ ...
'// AF7R58B ordinary resource-failure transport -- SHADOW ONLY\n' ...
'static mxArray* af7r58b_resource_result(int reason,double t0,double tm,double t1){\n' ...
'    const char*f[]={"success","reason_code","t0","tm","hold_end","t1","episode_ms","synthesis_ms","candidate_ms","total_ms"};\n' ...
'    mxArray*S=mxCreateStructMatrix(1,1,10,f);\n' ...
'    mxSetField(S,0,"success",logicalScalar(false));\n' ...
'    mxSetField(S,0,"reason_code",dscalar((double)reason));\n' ...
'    mxSetField(S,0,"t0",dscalar(t0));mxSetField(S,0,"tm",dscalar(tm));\n' ...
'    mxSetField(S,0,"hold_end",dscalar(tm));mxSetField(S,0,"t1",dscalar(t1));\n' ...
'    mxSetField(S,0,"episode_ms",dscalar(0));mxSetField(S,0,"synthesis_ms",dscalar(0));\n' ...
'    mxSetField(S,0,"candidate_ms",dscalar(0));mxSetField(S,0,"total_ms",dscalar(0));\n' ...
'    return S;\n}\n\n']);
src=[src(1:p-1) helper src(p:end)];

s=strfind(src,'if(op==1){');
assert(numel(s)==1,'[AF7R58B] op1 block count !=1.');
u=strfind(src,'mexErrMsgIdAndTxt("AF7R34:Op"');
u=u(u>s(1)); assert(~isempty(u),'[AF7R58B] AF7R34:Op anchor missing.');
u=u(1);

block=sprintf([ ...
'if(op==1){\n' ...
'            if(nrhs!=2) mexErrMsgIdAndTxt("AF7R34:Arity","plan op: op, packet.");\n' ...
'            const mxArray*P=prhs[1];\n' ...
'            std::string sig=str(fld(P,"nominal_signature"),"nominal_signature");\n' ...
'            if(!G_compiled||sig!=G_nominal.signature) mexErrMsgIdAndTxt("AF7R34:Reference","compiled nominal signature mismatch.");\n' ...
'            double t0=scalar(fld(P,"t0"),"t0"),tm=scalar(fld(P,"tm"),"tm"),t1=scalar(fld(P,"t1"),"t1");\n' ...
'            constexpr size_t AF7R58B_GMAX=%d; constexpr size_t AF7R58B_SMAX=%d;\n' ...
'            const mxArray*rt0=fld(P,"repair_t0"); const size_t G=mxGetNumberOfElements(rt0);\n' ...
'            if(G>AF7R58B_GMAX){if(nlhs>0)plhs[0]=af7r58b_resource_result(218,t0,tm,t1);return;}\n' ...
'            const mxArray*rt1=fld(P,"repair_t1"); const mxArray*off=fld(P,"repair_offsets");\n' ...
'            if(mxGetNumberOfElements(rt1)!=G) mexErrMsgIdAndTxt("AF7R58B:MalformedRepairT1","repair_t1 count mismatch before repair copy.");\n' ...
'            if(mxGetNumberOfElements(off)!=G+1) mexErrMsgIdAndTxt("AF7R58B:MalformedRepairOffsets","repair_offsets count mismatch before repair copy.");\n' ...
'            const mxArray*cp=fld(P,"repair_cp");\n' ...
'            if(!mxIsDouble(cp)||mxIsComplex(cp)||mxGetM(cp)!=48) mexErrMsgIdAndTxt("AF7R58B:MalformedRepairCP","repair_cp must be real double 48xS.");\n' ...
'            const size_t S=mxGetN(cp);\n' ...
'            if(S>AF7R58B_SMAX){if(nlhs>0)plhs[0]=af7r58b_resource_result(219,t0,tm,t1);return;}\n' ...
'            const mxArray*s0=fld(P,"repair_seg0"); const mxArray*s1=fld(P,"repair_seg1");\n' ...
'            if(mxGetNumberOfElements(s0)!=S||mxGetNumberOfElements(s1)!=S) mexErrMsgIdAndTxt("AF7R58B:MalformedRepairSegments","repair segment count mismatch before repair copy.");\n' ...
'            ActiveReference R=parse_ref(P);Live L=parse_live(P);SpeedCert B=parse_speed(P);auto O=parse_obs(P);\n' ...
'            PlanResult Q=plan(R,L,B,O,t0,tm,t1);plhs[0]=result_struct(Q,t0,tm,t1,O);return;\n' ...
'        }\n' ...
'        '],GMAX,SMAX);
patched=[src(1:s(1)-1) block src(u:end)];
end

% =========================================================================
% Canonical packet / exact prune implementation.
% =========================================================================
function P=build_prune_packet(sig)
P=struct(); P.nominal_signature=sig; P.flight_t0=0;
% G1 expired nonzero, G2 expired empty, G3 active at tNow=4, G4 future.
P.repair_t0=[1 3.2 3.5 6];
P.repair_t1=[3 3.8 6 8];
P.repair_offsets=[0 2 2 4 6];
P.repair_seg0=[1 2 3.5 4.5 6 7];
P.repair_seg1=[2 3 4.5 6 7 8];
J0=zeros(3,8);
J1=zeros(3,8); J1(:,1)=[0.4;-0.1;0.2]; J1(:,2)=[0.08;0.03;-0.02];
J2=zeros(3,8); J2(:,1)=[-0.25;0.32;0.10]; J2(:,2)=[-0.03;0.07;0.02];
J3=zeros(3,8); J3(:,1)=[0.18;0.12;-0.16]; J3(:,2)=[0.04;-0.02;0.03];
C=cell(1,6);
C{1}=hermite_d15(J0,J1,1); C{2}=hermite_d15(J1,J0,1);
C{3}=hermite_d15(J0,J2,1); C{4}=hermite_d15(J2,J0,1.5);
C{5}=hermite_d15(J0,J3,1); C{6}=hermite_d15(J3,J0,1);
P.repair_cp=zeros(48,6);
for k=1:6, P.repair_cp(:,k)=C{k}(:); end
end

function Q=prune_expired_packet(P,tNow)
% Exact group deletion only. Keep group iff its open support may intersect
% [tNow,+inf), i.e. repair_t1 > tNow. No fitting or control-point changes.
G=numel(P.repair_t0);
assert(numel(P.repair_t1)==G && numel(P.repair_offsets)==G+1);
S=size(P.repair_cp,2);
assert(size(P.repair_cp,1)==48 && numel(P.repair_seg0)==S && numel(P.repair_seg1)==S);
keep=P.repair_t1>tNow;
Q=P;
Q.repair_t0=P.repair_t0(keep); Q.repair_t1=P.repair_t1(keep);
Q.repair_offsets=zeros(1,nnz(keep)+1);
seg0=[]; seg1=[]; cp=zeros(48,0); q=0; j=0;
for g=1:G
    if ~keep(g), continue; end
    j=j+1; Q.repair_offsets(j)=q;
    a=P.repair_offsets(g)+1; b=P.repair_offsets(g+1);
    if b>=a
        idx=a:b;
        seg0=[seg0 P.repair_seg0(idx)]; %#ok<AGROW>
        seg1=[seg1 P.repair_seg1(idx)]; %#ok<AGROW>
        cp=[cp P.repair_cp(:,idx)]; %#ok<AGROW>
        q=q+numel(idx);
    end
end
Q.repair_offsets(end)=q;
Q.repair_seg0=seg0; Q.repair_seg1=seg1; Q.repair_cp=cp;
end

function P=minimal_op1_packet(sig)
P=struct(); P.nominal_signature=sig; P.t0=1; P.tm=2; P.t1=4;
P.repair_t0=[]; P.repair_t1=[]; P.repair_offsets=0;
P.repair_seg0=[]; P.repair_seg1=[]; P.repair_cp=zeros(48,0);
end

function B=hermite_d15(JL,JR,T)
n=15; B=zeros(3,16);
for r=0:7
    ff=fallfac(n,r); d=JL(:,r+1)*(T^r)/ff; acc=zeros(3,1);
    for k=0:r-1, acc=acc+((-1)^(r-k))*nchoosek(r,k)*B(:,k+1); end
    B(:,r+1)=d-acc;
end
C=zeros(3,8);
for r=0:7
    ff=fallfac(n,r); d=(((-1)^r)*JR(:,r+1))*(T^r)/ff; acc=zeros(3,1);
    for k=0:r-1, acc=acc+((-1)^(r-k))*nchoosek(r,k)*C(:,k+1); end
    C(:,r+1)=d-acc;
end
for k=0:7, B(:,16-k)=C(:,k+1); end
end

function v=fallfac(n,r)
v=1; for k=0:r-1, v=v*(n-k); end
end

% =========================================================================
% Utilities.
% =========================================================================
function [ok,R,msg]=call_no_exception(f)
ok=false; R=struct(); msg='';
try, R=f(); ok=true; catch ME, msg=[ME.identifier ' | ' ME.message]; end
end

function tf=has_failure_wrapper_fields(R)
req={'success','reason_code','t0','tm','hold_end','t1','episode_ms','synthesis_ms','candidate_ms','total_ms'};
tf=isstruct(R)&&isscalar(R);
if ~tf, return; end
for k=1:numel(req), tf=tf&&isfield(R,req{k}); end
end

function v=field_num(S,f)
v=NaN; if isstruct(S)&&isfield(S,f), q=double(S.(f)); if isscalar(q),v=q;end,end
end

function v=field_bool(S,f)
v=false; if isstruct(S)&&isfield(S,f), q=S.(f); if isscalar(q),v=logical(q);end,end
end

function y=compact_source(x)
y=regexprep(x,'\s+','');
end

function safe_clear_mex(name)
try, eval(['clear ' name]); catch, end
end

function safe_delete_mex(name,root)
f=fullfile(root,[name '.' mexext]); if isfile(f), try, delete(f); catch, end, end
end

function write_text_binary(f,s)
fid=fopen(f,'wb'); assert(fid>=0,'cannot write %s',f); c=onCleanup(@()fclose(fid)); fwrite(fid,uint8(s),'uint8');
end

function h=sha256_file(f)
fid=fopen(f,'rb'); assert(fid>=0,'cannot open %s',f); c=onCleanup(@()fclose(fid));
b=fread(fid,Inf,'*uint8'); md=java.security.MessageDigest.getInstance('SHA-256'); md.update(b);
d=typecast(md.digest(),'uint8'); h=lower(reshape(dec2hex(d,2).',1,[]));
end
