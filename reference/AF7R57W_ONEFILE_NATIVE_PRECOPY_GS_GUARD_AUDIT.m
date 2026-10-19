function OUT = AF7R57W_ONEFILE_NATIVE_PRECOPY_GS_GUARD_AUDIT()
% AF7R57W_ONEFILE_NATIVE_PRECOPY_GS_GUARD_AUDIT
% ========================================================================
% PURPOSE
%   Transplant the already-closed AF7R57V V2 canonical repair resource
%   theorem into the ACTUAL production native parent source
%   af7r36_single_core_mex.cpp, but ONLY as immutable shadow artifacts.
%
%   This audit patches parse_ref() BEFORE all dvec repair-array copies with:
%       1) shape-only schema checks for every repair array,
%       2) independent G and S resource caps,
%       3) reject-only resource errors.
%
%   It then builds TWO unique MEX files from the same exact parent source:
%       af7r57w_parent_clone_mex   : exact unmodified parent source
%       af7r57w_precopy_guard_mex  : parent + pre-copy shape/resource gate
%
%   A nonzero repaired-reference packet is generated inside THIS FILE.
%   No production source/MEX is overwritten.
%
% IMPORTANT
%   GMAX_TEST=4 and SMAX_TEST=8 are AUDIT-ONLY values. They are NOT selected
%   as production caps. This audit closes guard placement/semantics only.
%
%   Safety acceptance is not sampled here. op=2 is used only to prove that
%   admitted ActiveReference evaluation is unchanged by the guard.
% ========================================================================

clc;
fprintf('\n====================================================================================================\n');
fprintf(' AF7R57W | ONE-FILE NATIVE PRE-COPY G/S + SCHEMA GUARD SHADOW AUDIT\n');
fprintf('====================================================================================================\n');
fprintf('[AUTHORITY] productionParent=AF7R36 | immutableShadow=1 | preCopyShapeGate=1 | rejectOnly=1 | productionModified=0\n');

root=fileparts(mfilename('fullpath'));
self=[mfilename('fullpath') '.m'];
fprintf(' entrypoint                               = %s\n',self);
fprintf(' entrypoint SHA-256                        = %s\n',sha256_file(self));

parentCpp=fullfile(root,'af7r36_single_core_mex.cpp');
assert(isfile(parentCpp),'[AF7R57W] af7r36_single_core_mex.cpp missing.');
parentSHA=sha256_file(parentCpp);
expectedParentSHA='4f9778723aa3aa339d5167b90845fcfcaa8564cae68326ff1e4ed504c7e6daea';

fprintf('\n[A. EXACT ACTUAL PRODUCTION-NATIVE PARENT]\n');
fprintf(' source                                   = %s\n',parentCpp);
fprintf(' SHA-256                                  = %s\n',parentSHA);
fprintf(' expected current SHA match               = %d\n',strcmp(parentSHA,expectedParentSHA));
assert(strcmp(parentSHA,expectedParentSHA), ...
    '[AF7R57W] actual AF7R36 parent changed; refuse stale patch.');

GMAX_TEST=4;
SMAX_TEST=8;
fprintf(' Gmax / Smax                              = %d / %d | AUDIT TEST ONLY\n',GMAX_TEST,SMAX_TEST);
fprintf(' production caps selected                 = 0\n');

% -------------------------------------------------------------------------
% B. Generate guarded shadow source by deterministic source insertion.
% -------------------------------------------------------------------------
src=fileread(parentCpp);
parseAnchor='static ActiveReference parse_ref';
p0=strfind(src,parseAnchor);
assert(numel(p0)==1,'[AF7R57W] parse_ref declaration count != 1.');

dvecAnchor='auto offs=dvec';
pd=strfind(src,dvecAnchor);
pd=pd(pd>p0);
assert(~isempty(pd),'[AF7R57W] repair dvec anchor not found after parse_ref.');
pd=pd(1);

% Guard intentionally uses only mxArray shape metadata before original dvec.
% All repair-array lengths are checked here, not only G and S, so malformed
% oversized arrays cannot induce unbounded dvec copies before rejection.
guard = sprintf([ ...
'    // AF7R57W PRE-COPY REPAIR RESOURCE/SCHEMA GATE -- AUDIT SHADOW ONLY\n' ...
'    constexpr size_t AF7R57W_MAX_REPAIR_GROUPS=%d;\n' ...
'    constexpr size_t AF7R57W_MAX_REPAIR_SEGMENTS=%d;\n' ...
'    const mxArray* af7r57w_rt0A=fld(P,"repair_t0");\n' ...
'    const size_t af7r57w_G=mxGetNumberOfElements(af7r57w_rt0A);\n' ...
'    if(af7r57w_G>AF7R57W_MAX_REPAIR_GROUPS)\n' ...
'        mexErrMsgIdAndTxt("AF7R57W:ResourceMaxRepairGroups","repair group resource cap exceeded before repair-array copy.");\n' ...
'    const mxArray* af7r57w_rt1A=fld(P,"repair_t1");\n' ...
'    const mxArray* af7r57w_offA=fld(P,"repair_offsets");\n' ...
'    if(mxGetNumberOfElements(af7r57w_rt1A)!=af7r57w_G)\n' ...
'        mexErrMsgIdAndTxt("AF7R57W:MalformedRepairT1Count","repair_t1 count mismatch before repair-array copy.");\n' ...
'    if(mxGetNumberOfElements(af7r57w_offA)!=af7r57w_G+1)\n' ...
'        mexErrMsgIdAndTxt("AF7R57W:MalformedRepairOffsetCount","repair_offsets count mismatch before repair-array copy.");\n' ...
'    const mxArray* af7r57w_cpA=fld(P,"repair_cp");\n' ...
'    if(!mxIsDouble(af7r57w_cpA)||mxIsComplex(af7r57w_cpA)||mxGetM(af7r57w_cpA)!=48)\n' ...
'        mexErrMsgIdAndTxt("AF7R57W:MalformedRepairCPShape","repair_cp must be real double 48xS before repair-array copy.");\n' ...
'    const size_t af7r57w_S=mxGetN(af7r57w_cpA);\n' ...
'    if(af7r57w_S>AF7R57W_MAX_REPAIR_SEGMENTS)\n' ...
'        mexErrMsgIdAndTxt("AF7R57W:ResourceMaxRepairSegments","repair segment resource cap exceeded before repair-array copy.");\n' ...
'    const mxArray* af7r57w_s0A=fld(P,"repair_seg0");\n' ...
'    const mxArray* af7r57w_s1A=fld(P,"repair_seg1");\n' ...
'    if(mxGetNumberOfElements(af7r57w_s0A)!=af7r57w_S)\n' ...
'        mexErrMsgIdAndTxt("AF7R57W:MalformedRepairSeg0Count","repair_seg0 count mismatch before repair-array copy.");\n' ...
'    if(mxGetNumberOfElements(af7r57w_s1A)!=af7r57w_S)\n' ...
'        mexErrMsgIdAndTxt("AF7R57W:MalformedRepairSeg1Count","repair_seg1 count mismatch before repair-array copy.");\n' ...
'    // AF7R57W END PRE-COPY GATE\n'],GMAX_TEST,SMAX_TEST);

patched=[src(1:pd-1) guard src(pd:end)];
shadowCpp=fullfile(root,'af7r57w_precopy_guard_mex.cpp');
write_text_binary(shadowCpp,patched);

fprintf('\n[B. IMMUTABLE SHADOW SOURCE GENERATION]\n');
fprintf(' guarded source                          = %s\n',shadowCpp);
fprintf(' guarded source SHA-256                   = %s\n',sha256_file(shadowCpp));
fprintf(' production source modified               = %d\n',~strcmp(sha256_file(parentCpp),parentSHA));
assert(strcmp(sha256_file(parentCpp),parentSHA),'[AF7R57W] parent source was modified.');

% Source-order theorem: every shape/resource check precedes original dvec.
pg=strfind(patched,'AF7R57W PRE-COPY REPAIR RESOURCE/SCHEMA GATE');
pCopy=strfind(patched,dvecAnchor);
pG=strfind(patched,'AF7R57W_MAX_REPAIR_GROUPS');
pS=strfind(patched,'AF7R57W_MAX_REPAIR_SEGMENTS');
pRt1=strfind(patched,'MalformedRepairT1Count');
pOff=strfind(patched,'MalformedRepairOffsetCount');
pCp=strfind(patched,'MalformedRepairCPShape');
pS0=strfind(patched,'MalformedRepairSeg0Count');
pS1=strfind(patched,'MalformedRepairSeg1Count');
sourceOrder = numel(pg)==1 && ~isempty(pCopy) && ...
    first_after(pG,pg)<pCopy(1) && first_after(pS,pg)<pCopy(1) && ...
    first_after(pRt1,pg)<pCopy(1) && first_after(pOff,pg)<pCopy(1) && ...
    first_after(pCp,pg)<pCopy(1) && first_after(pS0,pg)<pCopy(1) && ...
    first_after(pS1,pg)<pCopy(1);

fprintf(' all repair shape/cap checks before dvec  = %d\n',sourceOrder);
assert(sourceOrder,'[AF7R57W] guard is not entirely before original repair dvec copies.');

% -------------------------------------------------------------------------
% C. Build exact parent clone and guarded MEX under unique names.
% -------------------------------------------------------------------------
parentMex='af7r57w_parent_clone_mex';
guardMex='af7r57w_precopy_guard_mex';
cd0=pwd;
cleanup=onCleanup(@()cd(cd0));
cd(root);

safe_clear_mex(parentMex);
safe_clear_mex(guardMex);
safe_delete_mex(parentMex,root);
safe_delete_mex(guardMex,root);

fprintf('\n[C. BUILD UNIQUE SHADOW MEX FILES]\n');
fprintf(' building exact parent clone             = %s\n',parentMex);
mex('-R2018a','-output',parentMex,parentCpp);
fprintf(' building guarded shadow                 = %s\n',guardMex);
mex('-R2018a','-output',guardMex,shadowCpp);
assert(exist(parentMex,'file')==3 && exist(guardMex,'file')==3, ...
    '[AF7R57W] one or both shadow MEX builds failed.');

fprintf(' parent clone built                       = 1\n');
fprintf(' guarded shadow built                     = 1\n');

% -------------------------------------------------------------------------
% D. Compile a deterministic zero nominal AST into both MEX instances.
%    This is only an eval baseline; nonzero repair carries the tested signal.
% -------------------------------------------------------------------------
expr=repmat({'0'},1,21);
sig='AF7R57W_ZERO_NOMINAL_FOR_REPAIR_EQUIVALENCE';
feval(parentMex,0,expr,sig);
feval(guardMex,0,expr,sig);

% Build a fully nonzero, C7 piecewise correction packet inside this file.
P=build_nonzero_packet(sig);
G=numel(P.repair_t0);
S=size(P.repair_cp,2);
fprintf('\n[D. NONZERO REPAIR PACKET]\n');
fprintf(' G / S                                   = %d / %d\n',G,S);
fprintf(' empty repair group present              = %d\n',any(diff(P.repair_offsets)==0));
fprintf(' nonzero control point present           = %d\n',any(P.repair_cp(:)~=0));
fprintf(' under audit caps                        = %d\n',G<=GMAX_TEST && S<=SMAX_TEST);
assert(G<=GMAX_TEST && S<=SMAX_TEST && any(P.repair_cp(:)~=0));

% -------------------------------------------------------------------------
% E. Admitted semantics: guarded == exact parent on op=2.
% -------------------------------------------------------------------------
t=linspace(0,8,257);
Y0=feval(parentMex,2,P,t);
Y1=feval(guardMex,2,P,t);
bitwiseSame=isequaln(Y0,Y1);
maxErr=max(abs(double(Y0(:))-double(Y1(:))));

fprintf('\n[E. ADMITTED-PACKET EXACT EQUIVALENCE]\n');
fprintf(' parent-vs-guarded bitwise identical      = %d\n',bitwiseSame);
fprintf(' maximum numeric difference              = %.3e\n',maxErr);
fprintf(' mathematical trajectory modified        = %d\n',~bitwiseSame);
assert(bitwiseSame && maxErr==0,'[AF7R57W] admitted semantics changed.');

% -------------------------------------------------------------------------
% F. Runtime resource rejects. These packets are intentionally malformed
%    after the resource/shape decision point, proving early rejection order.
% -------------------------------------------------------------------------
PoverG=P;
PoverG.repair_t0=zeros(1,GMAX_TEST+1);       % should reject immediately
% Deliberately make later fields inconsistent; they must not be copied/read.
PoverG.repair_t1=[];
PoverG.repair_offsets=[];
PoverG.repair_seg0=[];
PoverG.repair_seg1=[];
PoverG.repair_cp=zeros(1,1);                 % also malformed, but must be unseen
[eG,idG]=expect_error(@()feval(guardMex,2,PoverG,0), ...
    'AF7R57W:ResourceMaxRepairGroups');

PoverS=P;
PoverS.repair_t0=0; PoverS.repair_t1=0; PoverS.repair_offsets=[0 0];
PoverS.repair_cp=zeros(48,SMAX_TEST+1);      % S resource reject
PoverS.repair_seg0=[];                       % later malformed, must be unseen
PoverS.repair_seg1=[];
[eS,idS]=expect_error(@()feval(guardMex,2,PoverS,0), ...
    'AF7R57W:ResourceMaxRepairSegments');

fprintf('\n[F. OVER-RESOURCE PRE-COPY REJECTION]\n');
fprintf(' over-G rejected with resource ID        = %d | %s\n',eG,idG);
fprintf(' over-S rejected with resource ID        = %d | %s\n',eS,idS);
assert(eG&&eS,'[AF7R57W] resource rejection regression failed.');

% -------------------------------------------------------------------------
% G. Malformed under-cap schemas are rejected before dvec/data copy.
%    Include deliberately huge inconsistent metadata arrays to make the
%    finite-work point explicit: shape query is O(1), dvec is never reached.
% -------------------------------------------------------------------------
PbadOff=P;
PbadOff.repair_offsets=zeros(1,100000);      % huge malformed metadata array
[eOff,idOff]=expect_error(@()feval(guardMex,2,PbadOff,0), ...
    'AF7R57W:MalformedRepairOffsetCount');

PbadSeg=P;
PbadSeg.repair_seg0=zeros(1,100000);          % huge malformed segment array
[eSeg,idSeg]=expect_error(@()feval(guardMex,2,PbadSeg,0), ...
    'AF7R57W:MalformedRepairSeg0Count');

PbadCp=P;
PbadCp.repair_cp=zeros(47,S);                 % wrong rows, under S cap
[eCp,idCp]=expect_error(@()feval(guardMex,2,PbadCp,0), ...
    'AF7R57W:MalformedRepairCPShape');

fprintf('\n[G. MALFORMED-SCHEMA PRE-COPY FAIL-CLOSED]\n');
fprintf(' huge repair_offsets rejected pre-copy   = %d | %s\n',eOff,idOff);
fprintf(' huge repair_seg0 rejected pre-copy      = %d | %s\n',eSeg,idSeg);
fprintf(' wrong repair_cp rows rejected pre-copy  = %d | %s\n',eCp,idCp);
assert(eOff&&eSeg&&eCp,'[AF7R57W] malformed pre-copy gate regression failed.');

% -------------------------------------------------------------------------
% H. Reject-only theorem and finite-work theorem.
% -------------------------------------------------------------------------
finiteWorkClosed = sourceOrder && eG && eS && eOff && eSeg && eCp;
rejectOnlyClosed = bitwiseSame && eG && eS;

fprintf('\n[H. FORMAL NATIVE PREPARSE THEOREM]\n');
fprintf(' shape-only pre-admission work           = O(1) fixed-field metadata queries\n');
fprintf(' admitted repair copies bounded by       = O(Gmax + Smax)\n');
fprintf(' parser/evaluator structural work        = O(G + S + C15x8) after admission\n');
fprintf(' malformed oversized vectors copied      = 0 before rejection\n');
fprintf(' admitted semantics unchanged            = %d\n',bitwiseSame);
fprintf(' guarded acceptance subset of parent     = 1\n');
fprintf(' finite preparse work theorem closed      = %d\n',finiteWorkClosed);
fprintf(' reject-only semantics theorem closed     = %d\n',rejectOnlyClosed);

% -------------------------------------------------------------------------
% I. Diagnostic timing only. This is NOT the end-to-end Flight 4 ms theorem.
% -------------------------------------------------------------------------
N=2000;
tParent=zeros(1,N); tGuard=zeros(1,N);
for k=1:50
    feval(parentMex,2,P,t); feval(guardMex,2,P,t);
end
for k=1:N
    z=tic; feval(parentMex,2,P,t); tParent(k)=1e3*toc(z);
    z=tic; feval(guardMex,2,P,t);  tGuard(k)=1e3*toc(z);
end
sp=sort(tParent); sg=sort(tGuard);
p95i=max(1,ceil(.95*N));

fprintf('\n[I. OP=2 PARSE+EVAL TIMING - DIAGNOSTIC ONLY]\n');
fprintf(' iterations                               = %d\n',N);
fprintf(' parent median / p95 / max [ms]          = %.6f / %.6f / %.6f\n',median(tParent),sp(p95i),max(tParent));
fprintf(' guard  median / p95 / max [ms]          = %.6f / %.6f / %.6f\n',median(tGuard),sg(p95i),max(tGuard));
fprintf(' median guard overhead [ms]               = %.6f\n',median(tGuard)-median(tParent));
fprintf(' production end-to-end 4 ms upgraded     = 0\n');

% -------------------------------------------------------------------------
% J. Final gate.
% -------------------------------------------------------------------------
closed=sourceOrder && bitwiseSame && finiteWorkClosed && rejectOnlyClosed;
fprintf('\n[J. FINAL GATE]\n');
fprintf(' actual AF7R36 parent SHA bound          = 1\n');
fprintf(' nonzero repair admitted equivalence      = %d\n',bitwiseSame);
fprintf(' independent G/S pre-copy reject         = %d\n',eG&&eS);
fprintf(' malformed oversized schema pre-copy reject= %d\n',eOff&&eSeg&&eCp);
fprintf(' finite preparse theorem closed           = %d\n',finiteWorkClosed);
fprintf(' reject-only theorem closed               = %d\n',rejectOnlyClosed);
fprintf(' production cap values selected           = 0\n');
fprintf(' production source modified               = 0\n');
fprintf(' production MEX overwritten               = 0\n');
fprintf(' production 4 ms theorem claimed          = 0\n');
fprintf(' NATIVE PRE-COPY GUARD SHADOW CLOSURE     = %d\n',closed);
fprintf(' next action                               = DERIVE_PRODUCTION_GMAX_SMAX_FROM_REFERENCE_LIFECYCLE__THEN_INSTALL_SAME_GATE__THEN_END_TO_END_QPC\n');
fprintf('====================================================================================================\n');

assert(closed,'[AF7R57W] native pre-copy guard shadow closure failed.');

OUT=struct();
OUT.schema='AF7R57W_ONEFILE_NATIVE_PRECOPY_GS_GUARD_V1';
OUT.closed=closed;
OUT.parent_sha256=parentSHA;
OUT.shadow_cpp=shadowCpp;
OUT.shadow_sha256=sha256_file(shadowCpp);
OUT.Gmax_test=GMAX_TEST;
OUT.Smax_test=SMAX_TEST;
OUT.nonzero_G=G;
OUT.nonzero_S=S;
OUT.bitwise_equivalence=bitwiseSame;
OUT.max_equivalence_error=maxErr;
OUT.overG_rejected=eG;
OUT.overS_rejected=eS;
OUT.malformed_precopy_closed=eOff&&eSeg&&eCp;
OUT.finite_preparse_closed=finiteWorkClosed;
OUT.reject_only_closed=rejectOnlyClosed;
OUT.parent_timing_ms=tParent;
OUT.guard_timing_ms=tGuard;
OUT.production_caps_selected=false;
OUT.production_modified=false;
OUT.production_4ms_claimed=false;
OUT.entrypoint=self;
OUT.entrypoint_sha256=sha256_file(self);
assignin('base','AF7R57W_ONEFILE_NATIVE_PRECOPY_GS_GUARD_AUDIT_OUT',OUT);
save(fullfile(root,'AF7R57W_ONEFILE_NATIVE_PRECOPY_GS_GUARD_AUDIT_OUT.mat'),'OUT');
end

% =========================================================================
% Local canonical nonzero repaired-reference construction.
% =========================================================================
function P=build_nonzero_packet(sig)
P=struct();
P.nominal_signature=sig;
P.flight_t0=0;

% Three groups: group 2 is deliberately empty, proving G and S independent.
P.repair_t0=[1 4 5];
P.repair_t1=[3 5 7];
P.repair_offsets=[0 2 2 4]; % zero-based offsets, matching AF7R34/native parser
P.repair_seg0=[1 2 5 6];
P.repair_seg1=[2 3 6 7];

J0=zeros(3,8);
Jm1=zeros(3,8); Jm1(:,1)=[0.35;-0.20;0.15]; Jm1(:,2)=[0.12;0.04;-0.03];
Jm2=zeros(3,8); Jm2(:,1)=[-0.22;0.28;0.11]; Jm2(:,2)=[-0.05;0.08;0.02];

C=cell(1,4);
C{1}=hermite_d15(J0,Jm1,1);
C{2}=hermite_d15(Jm1,J0,1);
C{3}=hermite_d15(J0,Jm2,1);
C{4}=hermite_d15(Jm2,J0,1);

P.repair_cp=zeros(48,4);
for k=1:4
    assert(isequal(size(C{k}),[3 16]));
    P.repair_cp(:,k)=C{k}(:);
end
end

function B=hermite_d15(JL,JR,T)
% Exact algebraic degree-15 Hermite -> Bernstein construction for endpoint
% physical-time jets r=0..7. Left 8 and right 8 control points do not overlap.
n=15;
assert(isequal(size(JL),[3 8]) && isequal(size(JR),[3 8]));
assert(isscalar(T)&&isfinite(T)&&T>0);
B=zeros(3,16);

% Left endpoint via forward differences.
for r=0:7
    ff=fallfac(n,r);
    d=JL(:,r+1)*(T^r)/ff;
    acc=zeros(3,1);
    for k=0:r-1
        acc=acc+((-1)^(r-k))*nchoosek(r,k)*B(:,k+1);
    end
    B(:,r+1)=d-acc;
end

% Right endpoint: reverse curve C(v)=B(1-v), whose left jets are (-1)^r JR.
C=zeros(3,8);
for r=0:7
    ff=fallfac(n,r);
    d=(((-1)^r)*JR(:,r+1))*(T^r)/ff;
    acc=zeros(3,1);
    for k=0:r-1
        acc=acc+((-1)^(r-k))*nchoosek(r,k)*C(:,k+1);
    end
    C(:,r+1)=d-acc;
end
for k=0:7
    B(:,16-k)=C(:,k+1);
end
end

function v=fallfac(n,r)
v=1;
for k=0:r-1, v=v*(n-k); end
end

% =========================================================================
% Audit utilities.
% =========================================================================
function [ok,id]=expect_error(f,expectedId)
ok=false; id='NO_ERROR';
try
    f();
catch ME
    id=ME.identifier;
    ok=strcmp(id,expectedId);
end
end

function p=first_after(v,a)
z=v(v>=a);
assert(~isempty(z)); p=z(1);
end

function safe_clear_mex(name)
try
    eval(['clear ' name]);
catch
end
end

function safe_delete_mex(name,root)
f=fullfile(root,[name '.' mexext]);
if isfile(f)
    try, delete(f); catch, end
end
end

function write_text_binary(f,s)
fid=fopen(f,'wb'); assert(fid>=0,'cannot write %s',f);
c=onCleanup(@()fclose(fid));
fwrite(fid,uint8(s),'uint8');
end

function h=sha256_file(f)
fid=fopen(f,'rb'); assert(fid>=0,'cannot open %s',f);
c=onCleanup(@()fclose(fid));
b=fread(fid,Inf,'*uint8');
md=java.security.MessageDigest.getInstance('SHA-256');
md.update(b);
d=typecast(md.digest(),'uint8');
h=lower(reshape(dec2hex(d,2).',1,[]));
end
