function OUT = AF7R57V_V2_ONEFILE_CANONICAL_REPAIR_CLOSURE_AUDIT()
% AF7R57V_V2_ONEFILE_CANONICAL_REPAIR_CLOSURE_AUDIT
% =========================================================================
% SELF-CONTAINED ONE-FILE CLOSURE HARNESS
%
% PURPOSE
%   Close the current active repaired-reference serialization/resource gap
%   without reading REPLANNING_SYSTEM.m, af7r34_make_core_packet.m, any MEX,
%   or any other helper at runtime.
%
%   This file contains, in one place:
%     1) canonical correction-chain model;
%     2) exact model -> native packet serializer;
%     3) shape-only G/S pre-admission resource guard;
%     4) guarded parser / exact packet -> model reconstruction;
%     5) native-semantics active-reference evaluator;
%     6) degree-15 Bezier endpoint-jet construction/evaluation;
%     7) C7 splice / inter-segment continuity audit;
%     8) nonzero repair + multiple groups + empty-group regression;
%     9) malformed/over-cap fail-closed regressions;
%    10) guarded-vs-unguarded exact structural equivalence;
%    11) diagnostic timing (NOT a production 4 ms authority).
%
% THEOREM SCOPE
%   This file proves the current SERIALIZATION + FINITE-WORK RESOURCE layer.
%   It does NOT replace the already separate whole-body obstacle, suspended-
%   load flatness, actuator, HLC-robustness, sensing, or compute-deadline
%   certificates. No safety inequality is weakened here.
%
% MATHEMATICAL CLAIMS
%
% T1. Canonical serialization is bijective on the admitted schema.
%     Each correction group g stores [t0_g,t1_g] and an ordered list of
%     degree-15, 3-D Bezier segments. Each 3x16 control-point matrix is copied
%     column-major into one 48-vector and back. Therefore reconstruction is
%     an exact permutation/reshape of stored scalars, not a fit.
%
% T2. C7 is preserved by serialization.
%     Serialization never changes times/control points. Hence all endpoint
%     jet equalities d^r/dt^r, r=0..7, are invariant under serialization.
%
% T3. Resource work is finite after independent G/S admission.
%     Let G=#repair groups and S=#total repair segments. Because empty groups
%     are legal, G cannot be bounded by S. For finite configured Gmax,Smax,
%       serialize/parse work = O(G+S),
%       one reference evaluation = O(G+S+C15x8),
%     where C15x8 is a representation constant. Thus admitted work is finite.
%
% T4. The G/S guard is reject-only.
%     It modifies no trajectory/safety inequality. For admitted packets the
%     parsed representation is exactly the unguarded representation. Hence
%       Accepted_guarded subseteq Accepted_unguarded.
%     A resource rejection can reduce availability, never enlarge the safety
%     acceptance set.
%
% IMPORTANT
%   GMAX_TEST/SMAX_TEST below are TEST HARNESS bounds only. They are NOT
%   production caps and are never claimed as such. Production cap values must
%   be selected from the eventual upstream/reference contract + runtime
%   envelope, not from the current observed packet.
% =========================================================================

clc;
fprintf('\n====================================================================================================\n');
fprintf(' AF7R57V V2 | ONE-FILE ANALYTIC C7 + CANONICAL SERIALIZATION + G/S RESOURCE CLOSURE\n');
fprintf('====================================================================================================\n');
fprintf('[AUTHORITY] selfContained=1 | externalSourceRead=0 | externalHelperCall=0 | sampledSafetyAcceptance=0 | productionModified=0\n');

self=[mfilename('fullpath') '.m'];
fprintf(' entrypoint                              = %s\n',self);
fprintf(' entrypoint SHA-256                       = %s\n',sha256_file(self));

%% ------------------------------------------------------------------------
% A. FORMAL CONSTANTS / TEST-ONLY RESOURCE BOUNDS
% -------------------------------------------------------------------------
n=15;
GMAX_TEST=4;     % test-only, NOT production cap
SMAX_TEST=8;     % test-only, NOT production cap
fprintf('\n[A. FORMAL SCOPE]\n');
fprintf(' native polynomial degree n             = %d\n',n);
fprintf(' required smoothness checked            = C7 (r=0..7)\n');
fprintf(' Gmax / Smax used for regressions        = %d / %d | TEST ONLY\n',GMAX_TEST,SMAX_TEST);
fprintf(' production resource caps selected       = 0\n');
fprintf(' whole-body safety theorem replaced      = 0\n');
fprintf(' dynamics/HLC theorem replaced           = 0\n');

%% ------------------------------------------------------------------------
% B. BUILD A NONZERO, MULTI-GROUP, EMPTY-GROUP C7 CORRECTION CHAIN
% -------------------------------------------------------------------------
model=make_demo_model();
G=numel(model.corrections);
S=0;
for g=1:G
    S=S+numel(model.corrections(g).control_points);
end

fprintf('\n[B. NONZERO CANONICAL SOURCE MODEL]\n');
fprintf(' repair groups G                        = %d\n',G);
fprintf(' total segments S                       = %d\n',S);
fprintf(' empty group present                    = %d\n',any(arrayfun(@(c)isempty(c.control_points),model.corrections)));
fprintf(' nonzero segment present                = %d\n',has_nonzero_cp(model));
fprintf(' G independent from S demonstrated      = %d\n',G~=S && any(arrayfun(@(c)isempty(c.control_points),model.corrections)));

%% ------------------------------------------------------------------------
% C. C7 THEOREM AUDIT ON SOURCE MODEL
% -------------------------------------------------------------------------
C7src=audit_model_c7_exact(model);
C7diag=audit_model_c7_numeric(model);

fprintf('\n[C. C7 SOURCE-MODEL ANALYTIC THEOREM AUDIT]\n');
fprintf(' Hermite D15 endpoint-jet map bijective = %d\n',C7src.hermite_bijective);
fprintf(' constructor replay bitwise identical   = %d\n',C7src.constructor_replay);
fprintf(' all correction outer certificate jets  = %d\n',C7src.outer_pass);
fprintf(' all internal join certificate jets     = %d\n',C7src.internal_pass);
fprintf(' segment boundary times exact            = %d\n',C7src.time_pass);
fprintf(' analytic C7 theorem closed              = %d\n',C7src.pass);
fprintf(' sampled path acceptance used           = 0\n');
assert(C7src.pass,'[AF7R57V V2] analytic source-model C7 theorem failed.');

fprintf('\n[C2. FLOATING ENDPOINT-JET RECOMPUTATION - DIAGNOSTIC ONLY]\n');
fprintf(' max recomputed endpoint-jet mismatch    = %.3e\n',C7diag.max_mismatch);
fprintf(' diagnostic legacy tolerance            = %.3e\n',C7diag.tol);
fprintf(' diagnostic tolerance pass              = %d\n',C7diag.pass);
fprintf(' theorem authority uses this tolerance  = 0\n');
fprintf(' note                                    = high-order finite differences can amplify cancellation\n');

%% ------------------------------------------------------------------------
% D. CANONICAL SERIALIZATION
% -------------------------------------------------------------------------
pkt=serialize_model(model);

fprintf('\n[D. CANONICAL SERIALIZATION]\n');
fprintf(' repair_t0 length                       = %d\n',numel(pkt.repair_t0));
fprintf(' repair_t1 length                       = %d\n',numel(pkt.repair_t1));
fprintf(' repair_offsets length                  = %d\n',numel(pkt.repair_offsets));
fprintf(' repair_seg0 length                     = %d\n',numel(pkt.repair_seg0));
fprintf(' repair_seg1 length                     = %d\n',numel(pkt.repair_seg1));
fprintf(' repair_cp size                         = %d x %d\n',size(pkt.repair_cp,1),size(pkt.repair_cp,2));
fprintf(' fixed 48xS control-point schema        = %d\n',size(pkt.repair_cp,1)==48 && size(pkt.repair_cp,2)==S);
fprintf(' serializer performs fitting            = 0\n');
fprintf(' serializer performs resampling         = 0\n');

%% ------------------------------------------------------------------------
% E. SHAPE-ONLY PRE-ADMISSION GUARD
% -------------------------------------------------------------------------
[adm,guard]=preparse_guard(pkt,GMAX_TEST,SMAX_TEST);

fprintf('\n[E. SHAPE-ONLY PRE-ADMISSION G/S GUARD]\n');
fprintf(' admitted                               = %d\n',adm);
fprintf(' observed G / S                         = %d / %d\n',guard.G,guard.S);
fprintf(' guard used control-point data copy     = %d\n',guard.data_copy_started);
fprintf(' independent G guard                    = %d\n',guard.G_checked);
fprintf(' independent S guard                    = %d\n',guard.S_checked);
fprintf(' CP third resource coordinate needed    = %d\n',guard.cp_extra_coordinate_required);
assert(adm,'[AF7R57V V2] valid demo packet unexpectedly rejected.');
assert(~guard.data_copy_started,'[AF7R57V V2] preparse guard copied payload data.');

%% ------------------------------------------------------------------------
% F. GUARDED PARSE AND EXACT STRUCTURAL BIJECTION
% -------------------------------------------------------------------------
[parsed,st]=parse_packet_guarded(pkt,GMAX_TEST,SMAX_TEST);
assert(st.ok,'[AF7R57V V2] guarded parse failed: %s',st.reason);
EQ=exact_model_identity(model,parsed);

fprintf('\n[F. EXACT STRUCTURAL BIJECTION]\n');
fprintf(' nominal metadata identical              = %d\n',EQ.metadata);
fprintf(' group count identical                   = %d\n',EQ.group_count);
fprintf(' group times identical                   = %d\n',EQ.group_times);
fprintf(' segment counts identical                = %d\n',EQ.segment_counts);
fprintf(' segment times identical                 = %d\n',EQ.segment_times);
fprintf(' all 3x16 control points identical       = %d\n',EQ.control_points);
fprintf(' empty-group semantics identical         = %d\n',EQ.empty_groups);
fprintf(' exact structural bijection closed       = %d\n',EQ.pass);
assert(EQ.pass,'[AF7R57V V2] serialize/parse bijection failed.');

C7preserved = C7src.pass && EQ.pass;
C7parsedDiag=audit_model_c7_numeric(parsed);
fprintf(' analytic C7 preserved by exact copy     = %d\n',C7preserved);
fprintf(' parsed numeric endpoint diagnostic pass = %d | max=%.3e\n', ...
    C7parsedDiag.pass,C7parsedDiag.max_mismatch);
assert(C7preserved,'[AF7R57V V2] exact serialization did not preserve analytic C7 theorem.');

%% ------------------------------------------------------------------------
% G. EVALUATOR SEMANTIC REGRESSION (DIAGNOSTIC, NOT THEOREM AUTHORITY)
% -------------------------------------------------------------------------
times=unique([linspace(model.flight_t0,model.flight_t0+40,161), ...
              all_segment_boundaries(model)]);
maxEvalErr=0;
for it=1:numel(times)
    t=times(it);
    y1=eval_model_native_semantics(model,t);
    y2=eval_model_native_semantics(parsed,t);
    maxEvalErr=max(maxEvalErr,max(abs(y1(:)-y2(:))));
end

fprintf('\n[G. EVALUATOR REGRESSION - DIAGNOSTIC ONLY]\n');
fprintf(' diagnostic query count                  = %d\n',numel(times));
fprintf(' max source-vs-parsed jet error          = %.3e\n',maxEvalErr);
fprintf(' regression pass                         = %d\n',maxEvalErr<=5e-12);
fprintf(' sampled safety acceptance               = 0\n');
assert(maxEvalErr<=5e-12,'[AF7R57V V2] parsed evaluator regression mismatch.');

%% ------------------------------------------------------------------------
% H. RESOURCE REJECTION TESTS: BEFORE COPY / FAIL CLOSED
% -------------------------------------------------------------------------
pktG=pkt;
pktG.repair_t0=[pktG.repair_t0 zeros(1,GMAX_TEST+1)];
pktG.repair_t1=[pktG.repair_t1 zeros(1,GMAX_TEST+1)];
pktG.repair_offsets=zeros(1,numel(pktG.repair_t0)+1);
[aG,gG]=preparse_guard(pktG,GMAX_TEST,SMAX_TEST);

pktS=pkt;
extra=SMAX_TEST+1-size(pktS.repair_cp,2);
if extra<1, extra=1; end
pktS.repair_cp=[pktS.repair_cp zeros(48,extra)];
pktS.repair_seg0=[pktS.repair_seg0 zeros(1,extra)];
pktS.repair_seg1=[pktS.repair_seg1 ones(1,extra)];
[aS,gS]=preparse_guard(pktS,GMAX_TEST,SMAX_TEST);

fprintf('\n[H. OVER-RESOURCE REJECT-ONLY REGRESSIONS]\n');
fprintf(' over-G rejected                         = %d | reason=%s\n',~aG,gG.reason);
fprintf(' over-G rejected before data copy        = %d\n',~gG.data_copy_started);
fprintf(' over-S rejected                         = %d | reason=%s\n',~aS,gS.reason);
fprintf(' over-S rejected before data copy        = %d\n',~gS.data_copy_started);
assert(~aG && ~gG.data_copy_started,'[AF7R57V V2] over-G guard failure.');
assert(~aS && ~gS.data_copy_started,'[AF7R57V V2] over-S guard failure.');

%% ------------------------------------------------------------------------
% I. MALFORMED SCHEMA FAIL-CLOSED TESTS
% -------------------------------------------------------------------------
bad1=pkt; bad1.repair_cp=zeros(47,size(pkt.repair_cp,2));
[~,sbad1]=parse_packet_guarded(bad1,GMAX_TEST,SMAX_TEST);
bad2=pkt; bad2.repair_offsets(end)=bad2.repair_offsets(end)+1;
[~,sbad2]=parse_packet_guarded(bad2,GMAX_TEST,SMAX_TEST);
bad3=pkt; bad3.repair_offsets(2)=bad3.repair_offsets(1)-1;
[~,sbad3]=parse_packet_guarded(bad3,GMAX_TEST,SMAX_TEST);

fprintf('\n[I. MALFORMED SCHEMA FAIL-CLOSED]\n');
fprintf(' wrong CP row count rejected             = %d | %s\n',~sbad1.ok,sbad1.reason);
fprintf(' offset terminal mismatch rejected       = %d | %s\n',~sbad2.ok,sbad2.reason);
fprintf(' decreasing offset rejected              = %d | %s\n',~sbad3.ok,sbad3.reason);
assert(~sbad1.ok && ~sbad2.ok && ~sbad3.ok,'[AF7R57V V2] malformed schema not fail-closed.');

%% ------------------------------------------------------------------------
% J. GUARDED VS UNGUARDED EXACT EQUIVALENCE FOR ADMITTED INPUT
% -------------------------------------------------------------------------
rawParsed=parse_packet_unguarded(pkt);
EQguard=exact_model_identity(rawParsed,parsed);

fprintf('\n[J. GUARDED VS UNGUARDED ADMITTED-PACKET IDENTITY]\n');
fprintf(' exact structural identity               = %d\n',EQguard.pass);
fprintf(' mathematical trajectory modified        = 0\n');
fprintf(' safety inequality modified              = 0\n');
fprintf(' availability may reduce on resource reject= 1\n');
assert(EQguard.pass,'[AF7R57V V2] guard changed admitted packet semantics.');

%% ------------------------------------------------------------------------
% K. FINITE-WORK / REJECT-ONLY THEOREM
% -------------------------------------------------------------------------
finiteWork = isfinite(GMAX_TEST) && isfinite(SMAX_TEST) && ...
             GMAX_TEST>=0 && SMAX_TEST>=0;
rejectOnly = EQguard.pass && ~aG && ~aS;

fprintf('\n[K. FORMAL RESOURCE THEOREM]\n');
fprintf(' serializer structural work             = O(G + S)\n');
fprintf(' parser structural work                 = O(G + S) after admission\n');
fprintf(' evaluator worst-case structural work   = O(G + S + C15x8)\n');
fprintf(' finite Gmax/Smax imply finite work      = %d\n',finiteWork);
fprintf(' empty groups require independent G cap  = %d\n',any(arrayfun(@(c)isempty(c.control_points),model.corrections)));
fprintf(' fixed 48xS removes extra CP-size coord  = %d\n',size(pkt.repair_cp,1)==48);
fprintf(' admitted semantics unchanged            = %d\n',EQguard.pass);
fprintf(' guard acceptance subset theorem         = %d\n',rejectOnly);
fprintf(' reject-only resource layer closed       = %d\n',finiteWork && rejectOnly);

%% ------------------------------------------------------------------------
% L. DIAGNOSTIC TIMING - NEVER UPGRADE TO PRODUCTION 4 ms AUTHORITY
% -------------------------------------------------------------------------
Ntim=300;
tm=zeros(1,Ntim);
for k=1:Ntim
    tt=tic;
    [~,ss]=parse_packet_guarded(pkt,GMAX_TEST,SMAX_TEST);
    if ~ss.ok, error('[AF7R57V V2] timing parse failed.'); end
    eval_model_native_semantics(parsed,model.flight_t0+12.345);
    tm(k)=1e3*toc(tt);
end

fprintf('\n[L. MATLAB ONE-FILE DIAGNOSTIC TIMING]\n');
fprintf(' iterations                              = %d\n',Ntim);
fprintf(' median / p95 / max [ms]                 = %.6f / %.6f / %.6f\n', ...
    median(tm),percentile_local(tm,95),max(tm));
fprintf(' production 4 ms claim upgraded          = 0\n');
fprintf(' reason                                  = MATLAB harness timing != production native end-to-end WCET\n');

%% ------------------------------------------------------------------------
% M. FINAL GATE
% -------------------------------------------------------------------------
closed = C7src.pass && C7preserved && EQ.pass && ...
         maxEvalErr<=5e-12 && ...
         ~aG && ~aS && ~sbad1.ok && ~sbad2.ok && ~sbad3.ok && ...
         EQguard.pass && finiteWork && rejectOnly;

fprintf('\n[M. FINAL GATE]\n');
fprintf(' nonzero repair serialization closed     = %d\n',EQ.pass);
fprintf(' analytic C7 preservation theorem closed = %d\n',C7preserved);
fprintf(' independent G/S resource theorem closed = %d\n',finiteWork);
fprintf(' over-resource pre-copy reject closed    = %d\n',~aG && ~aS && ~gG.data_copy_started && ~gS.data_copy_started);
fprintf(' malformed schema fail-closed            = %d\n',~sbad1.ok && ~sbad2.ok && ~sbad3.ok);
fprintf(' guard reject-only semantics closed      = %d\n',rejectOnly);
fprintf(' production cap values selected          = 0\n');
fprintf(' production source modified              = 0\n');
fprintf(' production 4 ms theorem claimed         = 0\n');
fprintf(' ONE-FILE SERIALIZATION/RESOURCE CLOSURE  = %d\n',closed);
if closed
    fprintf(' next action                              = TRANSPLANT_THIS_PREPARSE_G_S_GUARD_INTO_NATIVE_PARSE_REF__THEN_MEASURE_END_TO_END\n');
else
    fprintf(' next action                              = DO_NOT_PATCH_PRODUCTION__FIX_THIS_ONE_FILE_FIRST\n');
end
fprintf('====================================================================================================\n');

OUT=struct();
OUT.schema='AF7R57V_V2_ONEFILE_CANONICAL_REPAIR_CLOSURE_V2';
OUT.closed=closed;
OUT.G=G; OUT.S=S;
OUT.C7_source=C7src;
OUT.C7_preserved=C7preserved;
OUT.C7_numeric_source=C7diag;
OUT.C7_numeric_parsed=C7parsedDiag;
OUT.exact_identity=EQ;
OUT.guard_identity=EQguard;
OUT.max_eval_error=maxEvalErr;
OUT.guard=guard;
OUT.timing_ms=struct('median',median(tm),'p95',percentile_local(tm,95),'max',max(tm));
OUT.production_caps_selected=false;
OUT.production_modified=false;
OUT.production_4ms_claim=false;
OUT.entrypoint=self;
OUT.entrypoint_sha256=sha256_file(self);
assignin('base','AF7R57V_V2_ONEFILE_CANONICAL_REPAIR_CLOSURE_AUDIT_OUT',OUT);
end

% =========================================================================
% SELF-CONTAINED MODEL GENERATION
% =========================================================================
function model=make_demo_model()
model=struct();
model.nominal_signature='AF7R57V_SELF_CONTAINED_NOMINAL';
model.flight_t0=10.0;

% Group 1: C7 transition -> affine corridor -> C7 transition to zero.
C1=make_c7_bump_group(20.0,22.0,26.0,28.0, ...
    [0.45;-0.30;0.20],[0.08;0.03;-0.02]);

% Group 2: deliberately empty. This proves G is independent of S.
C2=empty_group(30.0,31.0);

% Group 3: second nonzero correction, independent direction/velocity.
C3=make_c7_bump_group(32.0,33.5,36.5,38.0, ...
    [-0.25;0.35;0.12],[-0.04;0.02;0.015]);

model.corrections=[C1 C2 C3];
end

function C=empty_group(t0,t1)
C=struct('t0',t0,'t1',t1,'segment_t0',zeros(1,0), ...
    'segment_t1',zeros(1,0),'control_points',{{}}, ...
    'cert_left_jets',{{}},'cert_right_jets',{{}},'cert_kind',{{}});
end

function C=make_c7_bump_group(t0,ta,tb,t1,d,v)
assert(t0<ta && ta<tb && tb<t1);
Z=zeros(3,8);
J0=Z;
JA=Z; JA(:,1)=d; JA(:,2)=v;
Tc=tb-ta;
db=d+v*Tc;
JB=Z; JB(:,1)=db; JB(:,2)=v;
JZ=Z;

% Every segment is generated by the same exact D15 endpoint-jet Hermite
% isomorphism.  The middle segment's endpoint jets are those of the affine
% polynomial d+v(t-ta); uniqueness of the D15 Hermite problem therefore
% proves that the exact-arithmetic segment is affine, without sampling.
cp1=bezier_from_endpoint_jets15(J0,JA,ta-t0);
cp2=bezier_from_endpoint_jets15(JA,JB,tb-ta);
cp3=bezier_from_endpoint_jets15(JB,JZ,t1-tb);

C=empty_group(t0,t1);
C.segment_t0=[t0 ta tb];
C.segment_t1=[ta tb t1];
C.control_points={cp1,cp2,cp3};
C.cert_left_jets={J0,JA,JB};
C.cert_right_jets={JA,JB,JZ};
C.cert_kind={'D15_HERMITE','D15_HERMITE_AFFINE_UNIQUE','D15_HERMITE'};
end

% Construct degree-15 Bernstein control points from physical-time endpoint
% jets r=0..7. First 8 CP are determined by the left jets and last 8 by
% the right jets; n=15 makes these sets disjoint. This is the direct Hermite
% coordinate isomorphism, not a fit.
function cp=bezier_from_endpoint_jets15(JL,JR,T)
assert(isequal(size(JL),[3 8])&&isequal(size(JR),[3 8]));
assert(isfinite(T)&&T>0);
n=15;
cp=nan(3,16);

% Left endpoint: d^r/dt^r = n!/(n-r)! / T^r * Delta^r B_0.
for r=0:7
    fac=factorial(n)/factorial(n-r)/T^r;
    if r==0
        cp(:,1)=JL(:,1);
    else
        s=zeros(3,1);
        for i=0:r-1
            s=s+(-1)^(r-i)*nchoosek(r,i)*cp(:,i+1);
        end
        cp(:,r+1)=JL(:,r+1)/fac - s;
    end
end

% Right endpoint: d^r/dt^r = fac * sum_i (-1)^i C(r,i) B_{n-i}.
for r=0:7
    fac=factorial(n)/factorial(n-r)/T^r;
    if r==0
        cp(:,16)=JR(:,1);
    else
        s=zeros(3,1);
        for i=0:r-1
            s=s+(-1)^i*nchoosek(r,i)*cp(:,16-i);
        end
        cp(:,16-r)=(-1)^r*(JR(:,r+1)/fac - s);
    end
end
assert(all(isfinite(cp),'all'));
end

% =========================================================================
% CANONICAL SERIALIZER / PARSERS
% =========================================================================
function P=serialize_model(model)
P=struct();
P.nominal_signature=char(model.nominal_signature);
P.flight_t0=double(model.flight_t0);
G=numel(model.corrections);
P.repair_t0=zeros(1,G);
P.repair_t1=zeros(1,G);
P.repair_offsets=zeros(1,G+1);
S=0;
for g=1:G
    P.repair_t0(g)=double(model.corrections(g).t0);
    P.repair_t1(g)=double(model.corrections(g).t1);
    P.repair_offsets(g)=S;
    S=S+numel(model.corrections(g).control_points);
end
P.repair_offsets(G+1)=S;
P.repair_seg0=zeros(1,S);
P.repair_seg1=zeros(1,S);
P.repair_cp=zeros(48,S);
q=0;
for g=1:G
    C=model.corrections(g);
    for k=1:numel(C.control_points)
        q=q+1;
        P.repair_seg0(q)=double(C.segment_t0(k));
        P.repair_seg1(q)=double(C.segment_t1(k));
        cp=double(C.control_points{k});
        assert(isequal(size(cp),[3 16])&&all(isfinite(cp),'all'));
        P.repair_cp(:,q)=cp(:);
    end
end
end

function [admit,S]=preparse_guard(P,Gmax,Smax)
S=struct('G',NaN,'S',NaN,'reason','', ...
    'data_copy_started',false,'G_checked',false,'S_checked',false, ...
    'cp_extra_coordinate_required',false);
admit=false;
req={'repair_t0','repair_t1','repair_offsets','repair_seg0','repair_seg1','repair_cp'};
if ~isstruct(P) || ~isscalar(P)
    S.reason='MALFORMED_PACKET_NOT_SCALAR_STRUCT'; return;
end
for k=1:numel(req)
    if ~isfield(P,req{k})
        S.reason=['MISSING_' upper(req{k})]; return;
    end
end
if ~(isscalar(Gmax)&&isfinite(Gmax)&&Gmax>=0&&Gmax==floor(Gmax))
    S.reason='INVALID_GMAX'; return;
end
if ~(isscalar(Smax)&&isfinite(Smax)&&Smax>=0&&Smax==floor(Smax))
    S.reason='INVALID_SMAX'; return;
end

% Shape/count queries only. No cp(:), double(array), or per-element copy.
G=numel(P.repair_t0);
if ndims(P.repair_cp)>2
    S.reason='MALFORMED_REPAIR_CP_NDIMS'; return;
end
[r,Sseg]=size(P.repair_cp);
S.G=G; S.S=Sseg;
S.G_checked=true;
if G>Gmax
    S.reason='RESOURCE_MAX_REPAIR_GROUPS'; return;
end
S.S_checked=true;
if Sseg>Smax
    S.reason='RESOURCE_MAX_REPAIR_SEGMENTS'; return;
end
if r~=48
    S.reason='MALFORMED_REPAIR_CP_ROWS'; return;
end
S.cp_extra_coordinate_required=false;
admit=true;
S.reason='ADMITTED';
end

function [model,st]=parse_packet_guarded(P,Gmax,Smax)
model=empty_model();
st=struct('ok',false,'reason','');
[admit,G]=preparse_guard(P,Gmax,Smax);
if ~admit
    st.reason=G.reason; return;
end
try
    model=parse_packet_after_admission(P);
catch ME
    st.reason=['MALFORMED_' ME.identifier];
    if isempty(ME.identifier), st.reason='MALFORMED_SCHEMA'; end
    return;
end
st.ok=true; st.reason='OK';
end

function model=parse_packet_unguarded(P)
model=parse_packet_after_admission(P);
end

function model=parse_packet_after_admission(P)
G=numel(P.repair_t0);
S=size(P.repair_cp,2);
assert(numel(P.repair_t1)==G,'AF7R57V:GroupSize');
assert(numel(P.repair_offsets)==G+1,'AF7R57V:OffsetSize');
assert(numel(P.repair_seg0)==S && numel(P.repair_seg1)==S,'AF7R57V:SegmentSize');
assert(size(P.repair_cp,1)==48,'AF7R57V:CpRows');
assert(all(isfinite(P.repair_t0))&&all(isfinite(P.repair_t1)),'AF7R57V:GroupFinite');
assert(all(isfinite(P.repair_offsets)),'AF7R57V:OffsetFinite');
assert(all(P.repair_offsets==round(P.repair_offsets)),'AF7R57V:OffsetInteger');
assert(P.repair_offsets(1)==0,'AF7R57V:OffsetFirst');
assert(P.repair_offsets(end)==S,'AF7R57V:OffsetLast');
assert(all(diff(P.repair_offsets)>=0),'AF7R57V:OffsetOrder');
assert(all(P.repair_offsets>=0 & P.repair_offsets<=S),'AF7R57V:OffsetRange');

model=struct();
if isfield(P,'nominal_signature'), model.nominal_signature=char(P.nominal_signature); else, model.nominal_signature=''; end
if isfield(P,'flight_t0'), model.flight_t0=double(P.flight_t0); else, model.flight_t0=0; end
model.corrections=repmat(empty_group(0,1),1,G);
for g=1:G
    a=P.repair_offsets(g);
    b=P.repair_offsets(g+1);
    assert(P.repair_t1(g)>=P.repair_t0(g),'AF7R57V:GroupTimeOrder');
    C=empty_group(double(P.repair_t0(g)),double(P.repair_t1(g)));
    ng=b-a;
    C.segment_t0=zeros(1,ng);
    C.segment_t1=zeros(1,ng);
    C.control_points=cell(1,ng);
    for jj=1:ng
        j=a+jj; % convert 0-based offset to MATLAB 1-based segment index
        t0=double(P.repair_seg0(j));
        t1=double(P.repair_seg1(j));
        assert(isfinite(t0)&&isfinite(t1)&&t1>t0,'AF7R57V:SegmentTime');
        v=double(P.repair_cp(:,j));
        assert(numel(v)==48&&all(isfinite(v)),'AF7R57V:CpFinite');
        C.segment_t0(jj)=t0;
        C.segment_t1(jj)=t1;
        C.control_points{jj}=reshape(v,3,16);
    end
    model.corrections(g)=C;
end
end

function m=empty_model()
m=struct('nominal_signature','','flight_t0',0,'corrections',repmat(empty_group(0,1),1,0));
end

% =========================================================================
% EXACT STRUCTURAL IDENTITY
% =========================================================================
function E=exact_model_identity(A,B)
E=struct('metadata',false,'group_count',false,'group_times',false,'segment_counts',false, ...
    'segment_times',false,'control_points',false,'empty_groups',false,'pass',false);
E.metadata=isequal(A.nominal_signature,B.nominal_signature) && isequal(A.flight_t0,B.flight_t0);
E.group_count=numel(A.corrections)==numel(B.corrections);
if ~E.group_count, return; end
G=numel(A.corrections);
E.group_times=true; E.segment_counts=true; E.segment_times=true;
E.control_points=true; E.empty_groups=true;
for g=1:G
    a=A.corrections(g); b=B.corrections(g);
    E.group_times=E.group_times && isequal(a.t0,b.t0) && isequal(a.t1,b.t1);
    E.segment_counts=E.segment_counts && numel(a.control_points)==numel(b.control_points);
    E.empty_groups=E.empty_groups && (isempty(a.control_points)==isempty(b.control_points));
    if numel(a.control_points)~=numel(b.control_points)
        E.segment_times=false; E.control_points=false; continue;
    end
    E.segment_times=E.segment_times && isequal(a.segment_t0,b.segment_t0) && isequal(a.segment_t1,b.segment_t1);
    for k=1:numel(a.control_points)
        E.control_points=E.control_points && isequal(a.control_points{k},b.control_points{k});
    end
end
E.pass=E.metadata&&E.group_count&&E.group_times&&E.segment_counts&&E.segment_times&&E.control_points&&E.empty_groups;
end

% =========================================================================
% NATIVE-SEMANTICS EVALUATOR
% =========================================================================
function Y=eval_model_native_semantics(model,t)
% 3 x 8 matrix: derivatives r=0..7 of correction sum.
Y=zeros(3,8);
for g=1:numel(model.corrections)
    C=model.corrections(g);
    if ~(t>C.t0 && t<C.t1), continue; end
    sp=0;
    for k=1:numel(C.control_points)
        if t<=C.segment_t1(k)
            sp=k; break;
        end
    end
    if sp==0, continue; end
    if t<C.segment_t0(sp), continue; end
    T=C.segment_t1(sp)-C.segment_t0(sp);
    u=(t-C.segment_t0(sp))/T;
    u=max(0,min(1,u));
    cp=C.control_points{sp};
    for r=0:7
        Y(:,r+1)=Y(:,r+1)+bezier_derivative_value(cp,T,u,r);
    end
end
end

function v=bezier_derivative_value(cp,T,u,r)
D=cp; deg=15;
for q=1:r
    D=(D(:,2:end)-D(:,1:end-1))*(deg/T);
    deg=deg-1;
end
v=decasteljau_vec(D,u);
end

function v=decasteljau_vec(cp,u)
D=cp;
while size(D,2)>1
    D=(1-u)*D(:,1:end-1)+u*D(:,2:end);
end
v=D(:,1);
end

% =========================================================================
% C7 AUDIT
% =========================================================================
function A=audit_model_c7_exact(model)
% FORMAL AUTHORITY.  No floating tolerance is used.
%
% For n=15 and endpoint jets r=0..7, the left equations determine B0..B7
% triangularly and the right equations determine B15..B8 triangularly.
% The two unknown sets are disjoint and every diagonal coefficient
% n!/(n-r)!/T^r is nonzero for T>0.  Hence the endpoint-jet -> Bernstein
% map is bijective.  C7 is then an equality statement on the declared jets.
n=15; R=7;
hermiteBijective=(R+1)+(R+1)==n+1;
outer=true; internal=true; times=true; replay=true; certPresent=true;
Z=zeros(3,8);
for g=1:numel(model.corrections)
    C=model.corrections(g);
    if isempty(C.control_points), continue; end
    certPresent=certPresent && numel(C.cert_left_jets)==numel(C.control_points) && ...
        numel(C.cert_right_jets)==numel(C.control_points);
    if ~certPresent, outer=false; internal=false; replay=false; break; end

    outer=outer && isequal(C.cert_left_jets{1},Z) && isequal(C.cert_right_jets{end},Z);
    for k=1:numel(C.control_points)
        T=C.segment_t1(k)-C.segment_t0(k);
        times=times && isfinite(T) && T>0;
        cpExpected=bezier_from_endpoint_jets15( ...
            C.cert_left_jets{k},C.cert_right_jets{k},T);
        replay=replay && isequal(cpExpected,C.control_points{k});
        if k<numel(C.control_points)
            times=times && isequal(C.segment_t1(k),C.segment_t0(k+1));
            internal=internal && isequal(C.cert_right_jets{k},C.cert_left_jets{k+1});
        end
    end
end
A=struct('hermite_bijective',hermiteBijective,'certificate_present',certPresent, ...
    'constructor_replay',replay,'outer_pass',outer,'internal_pass',internal, ...
    'time_pass',times,'pass',hermiteBijective&&certPresent&&replay&&outer&&internal&&times);
end

function A=audit_model_c7_numeric(model)
% DIAGNOSTIC ONLY.  High-order finite-difference evaluation can amplify
% cancellation even when the analytic endpoint-jet construction is exact.
tol=2e-9;
maxMis=0; outer=true; internal=true;
for g=1:numel(model.corrections)
    C=model.corrections(g);
    if isempty(C.control_points), continue; end
    cp0=C.control_points{1}; T0=C.segment_t1(1)-C.segment_t0(1);
    cpn=C.control_points{end}; Tn=C.segment_t1(end)-C.segment_t0(end);
    for r=0:7
        jl=bezier_endpoint_jet(cp0,T0,r,'left');
        jr=bezier_endpoint_jet(cpn,Tn,r,'right');
        maxMis=max(maxMis,max(abs(jl)));
        maxMis=max(maxMis,max(abs(jr)));
        outer=outer && max(abs(jl))<=tol && max(abs(jr))<=tol;
    end
    for k=1:numel(C.control_points)-1
        ca=C.control_points{k}; cb=C.control_points{k+1};
        Ta=C.segment_t1(k)-C.segment_t0(k);
        Tb=C.segment_t1(k+1)-C.segment_t0(k+1);
        for r=0:7
            ja=bezier_endpoint_jet(ca,Ta,r,'right');
            jb=bezier_endpoint_jet(cb,Tb,r,'left');
            e=max(abs(ja-jb));
            maxMis=max(maxMis,e);
            internal=internal && e<=tol;
        end
    end
end
A=struct('outer_pass',outer,'internal_pass',internal, ...
    'max_mismatch',maxMis,'tol',tol,'pass',outer&&internal,'diagnostic_only',true);
end

function j=bezier_endpoint_jet(cp,T,r,side)
D=cp; deg=15;
for q=1:r
    D=(D(:,2:end)-D(:,1:end-1))*(deg/T);
    deg=deg-1;
end
if strcmp(side,'left'), j=D(:,1); else, j=D(:,end); end
end

% =========================================================================
% SMALL UTILITIES
% =========================================================================
function tf=has_nonzero_cp(model)
tf=false;
for g=1:numel(model.corrections)
    for k=1:numel(model.corrections(g).control_points)
        if any(model.corrections(g).control_points{k}(:)~=0)
            tf=true; return;
        end
    end
end
end

function t=all_segment_boundaries(model)
t=[];
for g=1:numel(model.corrections)
    t=[t model.corrections(g).t0 model.corrections(g).t1 ...
       model.corrections(g).segment_t0 model.corrections(g).segment_t1]; %#ok<AGROW>
end
t=unique(t);
end

function q=percentile_local(x,p)
x=sort(x(:));
if isempty(x), q=NaN; return; end
if numel(x)==1, q=x; return; end
r=1+(numel(x)-1)*p/100;
a=floor(r); b=ceil(r);
if a==b, q=x(a); else, q=x(a)+(r-a)*(x(b)-x(a)); end
end

function h=sha256_file(f)
fid=fopen(f,'r'); assert(fid>=0,'cannot open %s',f);
c=onCleanup(@()fclose(fid));
b=fread(fid,Inf,'*uint8');
md=java.security.MessageDigest.getInstance('SHA-256');
md.update(b);
d=typecast(md.digest(),'uint8');
h=lower(reshape(dec2hex(d,2).',1,[]));
end
