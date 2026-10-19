function OUT = AF7R57O_GLOBAL_THEORY_CONSISTENCY_AUDIT()
% AF7R57O_GLOBAL_THEORY_CONSISTENCY_AUDIT
% =========================================================================
% READ-ONLY GLOBAL CONSISTENCY / SCOPE RE-AUDIT FOR AF7 R57 CHAIN.
%
% This audit deliberately does NOT create a new safety theorem.  It checks
% whether the already-closed R57G/R57I/R57K/V10/V11/V12 statements have
% drifted in meaning relative to the exact source and the production
% REPLANNING_SYSTEM interface.
%
% Main separation:
%   (1) continuous-time mathematical safety theorem,
%   (2) finite algorithmic-resource theorem,
%   (3) fail-closed MEX transport,
%   (4) platform wall-time qualification,
%   (5) final production commit authority (tracking/HLC/actuator/etc.).
%
% No source is modified.  No MEX is built.  No trajectory is generated.
% =========================================================================

clc;
fprintf('\n====================================================================================================\n');
fprintf(' AF7R57O | GLOBAL THEORY / SOURCE / SCOPE CONSISTENCY RE-AUDIT\n');
fprintf('====================================================================================================\n');
fprintf('[AUTHORITY] readOnly=1 | sourceBound=1 | noNewAcceptance=1 | noTimingClaimUpgrade=1 | productionModified=0\n');

selfFile=[mfilename('fullpath') '.m'];
assert(isfile(selfFile),'[R57O] immutable entrypoint unavailable.');
selfSHA=sha256_file(selfFile);
fprintf(' immutable entrypoint                    = %s\n',selfFile);
fprintf(' immutable entrypoint SHA-256             = %s\n',selfSHA);

OUT=struct();
OUT.schema='AF7R57O_GLOBAL_THEORY_CONSISTENCY_AUDIT_V1';
OUT.entrypoint=selfFile;
OUT.entrypoint_sha256=selfSHA;
OUT.audit_closed=false;
OUT.production_ready=false;
OUT.production_modified=false;

%% A. Historical authorities
fprintf('\n[A. HISTORICAL AUTHORITY CHAIN]\n');
K=get_authority('AF7R57K_CORRECTED_NATIVE_CORE_AUDIT_OUT', ...
    'AF7R57K_CORRECTED_NATIVE_CORE_AUDIT_OUT.mat');
L=get_authority('AF7R57L_QPC_HARD4MS_CURRENT_PACKET_AUDIT_OUT', ...
    'AF7R57L_QPC_HARD4MS_CURRENT_PACKET_AUDIT_OUT.mat');
M=get_authority('AF7R57M_SOURCE_COMPLEXITY_CONTRACT_AUDIT_OUT', ...
    'AF7R57M_SOURCE_COMPLEXITY_CONTRACT_AUDIT_OUT.mat');
V10=get_authority('AF7R57N_V10_EXACT_OP4_EPISODE_RETURN_AUDIT_OUT', ...
    'AF7R57N_V10_EXACT_OP4_EPISODE_RETURN_AUDIT_OUT.mat');
V11=get_authority('AF7R57N_V11_TAILSAFE_RESOURCE_CLOSURE_AUDIT_OUT', ...
    'AF7R57N_V11_TAILSAFE_RESOURCE_CLOSURE_AUDIT_OUT.mat');
V12=get_authority('AF7R57N_V12_RESOURCE_DOMINANCE_BASIS_AUDIT_OUT', ...
    'AF7R57N_V12_RESOURCE_DOMINANCE_BASIS_AUDIT_OUT.mat');
C=get_authority('AF7R57C_SNAPSHOT_BOUND_SCHEMA_CAPTURE_OUT', ...
    'AF7R57C_SNAPSHOT_BOUND_SCHEMA_CAPTURE_OUT.mat');

assert(isstruct(K)&&logical(K.closed),'[R57O] R57K theorem authority not closed.');
assert(isstruct(L)&&isfield(L,'current_packet_hard_4ms')&&logical(L.current_packet_hard_4ms), ...
    '[R57O] R57L current-packet timing authority not closed.');
assert(isstruct(M)&&logical(M.closed),'[R57O] R57M structural authority not closed.');
assert(isstruct(V10)&&logical(V10.guard_mechanism_closed)&&logical(V10.exception_free_resource_transport), ...
    '[R57O] V10 resource transport authority not closed.');
assert(isstruct(V11)&&logical(V11.guard_mechanism_closed)&&logical(V11.exception_free_resource_transport), ...
    '[R57O] V11 TailSafe resource authority not closed.');
assert(isstruct(V12)&&logical(V12.closed),'[R57O] V12 source-dominance audit did not close.');
assert(isstruct(C)&&logical(C.closed),'[R57O] R57C packet authority unavailable.');

fprintf(' R57K corrected fixed-tU theorem          = 1\n');
fprintf(' R57L prepared-current-packet hard 4 ms   = 1\n');
fprintf(' R57M structural growth audit             = 1\n');
fprintf(' V10 exception-free resource transport    = 1\n');
fprintf(' V11 TailSafe guard mechanism             = 1\n');
fprintf(' V12 split/depth dominance audit          = 1\n');

%% B. Exact source bindings
fprintf('\n[B. EXACT SOURCE BINDING]\n');
core0=find_source('af7r56_trace_core.hpp');
ser0=find_source('af7r56_trace_mex.cpp');
coreK=find_source('af7r57k_trace_core.hpp');
serK=find_source('af7r57k_corrected_core_mex.cpp');
core11=find_source('af7r57n_v11_trace_core.hpp');
ser11=find_source('af7r57n_v11_guarded_core_mex.cpp');
rps=which('REPLANNING_SYSTEM');
assert(~isempty(rps)&&isfile(rps),'[R57O] active REPLANNING_SYSTEM unresolved.');

expected=struct( ...
    'core0','9f1c7054bbac0dadc2e82fbb107183937143b39431d8996ee13a620cc62ef821', ...
    'ser0','cef987c012982982d2fc4381419781770fe9304c6a2ba4f1d7cff1bba2746f23', ...
    'coreK','4f99d3ba7bdd5645a851a1c2236a8239753631d47998760cc6939a0655a344e1', ...
    'serK','3b774a4aa1b18646182d1f7fa6d01bd2c6669e5ad2e6a8200f702f2f8e6f8597', ...
    'core11','4d1b4d8a74bdad3c1798c8ce76330ebfc2652d0d5ae16016bf374fb2796c87ae', ...
    'ser11','ad9b80615b0c944bbaabd0a62f318aec60debec815e06e0229fd7b7d837a8c1e', ...
    'rps','223d34c49d7148b8ea14ff4ab9c420d409560c0650ac19e9b0f99318887090c9');

sha=struct('core0',sha256_file(core0),'ser0',sha256_file(ser0), ...
    'coreK',sha256_file(coreK),'serK',sha256_file(serK), ...
    'core11',sha256_file(core11),'ser11',sha256_file(ser11), ...
    'rps',sha256_file(rps));
fn=fieldnames(expected);
for i=1:numel(fn)
    assert(strcmpi(sha.(fn{i}),expected.(fn{i})), ...
        '[R57O] exact source SHA changed: %s',fn{i});
end
fprintf(' original core / serializer current       = 1 / 1\n');
fprintf(' R57K corrected core / serializer current = 1 / 1\n');
fprintf(' V11 guarded core / serializer current    = 1 / 1\n');
fprintf(' REPLANNING_SYSTEM current                = 1\n');

S0=normalize_lf(fileread(core0));
SK=normalize_lf(fileread(coreK));
S11=normalize_lf(fileread(core11));
C0=normalize_lf(fileread(ser0));
CK=normalize_lf(fileread(serK));
C11=normalize_lf(fileread(ser11));
RPS=normalize_lf(fileread(rps));

%% C. R57K delta scope + R57I semantics
fprintf('\n[C. SAFETY-MATHEMATICS DRIFT AUDIT]\n');
kAudit=find_source('AF7R57K_CORRECTED_NATIVE_CORE_AUDIT.m');
kAuditSHA=sha256_file(kAudit);
assert(strcmpi(kAuditSHA,'49079e4014e21b6e5d36129322220766f543061614337351b094a3958d5c5fb2'), ...
    '[R57O] R57K generator/audit source changed.');
KA=normalize_lf(fileread(kAudit));

stageBGenerationBound = token_count(KA,"replace_cpp_function(src, ...")>=2 && ...
    contains(KA,"'static inline SpeedCert stageB_speed('") && ...
    contains(KA,"'static inline StageBTrace stageB_trace('") && ...
    contains(KA,'#include "af7r56_trace_core.hpp"') && ...
    contains(KA,'#include "af7r57k_trace_core.hpp"');
assert(stageBGenerationBound,'[R57O] R57K Stage-B-only generator contract not found.');

% The corrected source must retain derivative-first restriction and must not
% reinterpret a sufficient upper-bound failure as physical infeasibility.
SB=extract_cpp_function(SK,'static inline SpeedCert stageB_speed(');
derivativeFirst=contains(SK,'split_half(old.C1)')&&contains(SK,'split_half(old.C2)')&& ...
    contains(SK,'split_half(old.C3)')&&contains(SK,'make_stable_parent_leaf');
necessaryWitnessReject=contains(SB,'if(fin(z.f.wit)&&z.f.wit>=lim)')&& ...
    contains(SB,'finiteWitness=true');
upperFailureRefines=contains(SB,'if(!(fin(z.f.vq)&&z.f.vq<lim))')&& ...
    contains(SB,'split_stable_leaf')&&contains(SB,'leaves.push_back');
strictFinalBound=contains(SB,'s.VQ<lim&&s.sigma>0');

candidateIdentity = isfield(K,'identity') && logical(K.identity.segment_times) && ...
    logical(K.identity.control_points) && double(K.identity.control_point_max_abs_error)==0 && ...
    logical(K.identity.signs);
mathPreserved = derivativeFirst&&necessaryWitnessReject&&upperFailureRefines&&strictFinalBound&& ...
    logical(K.speed_closed)&&logical(K.geometry_closed)&&logical(K.rejoin_closed)&&candidateIdentity;
assert(mathPreserved,'[R57O] corrected safety-math preservation audit failed.');

fprintf(' R57K source-generation scope Stage-B     = 1\n');
fprintf(' derivative-before-restriction preserved = 1\n');
fprintf(' sufficient VQ-bound failure refines     = 1\n');
fprintf(' direct physical witness reject retained = 1\n');
fprintf(' strict VQ/sigma final condition retained = 1\n');
fprintf(' candidate synthesis exact identity       = 1\n');
fprintf(' whole-body/rejoin parent theorem retained= 1\n');

%% D. Guard semantics: reject-only / no acceptance weakening
fprintf('\n[D. RESOURCE-GUARD SEMANTIC AUDIT]\n');
rejectOnlyTokens={ ...
    'resource_fail(201)','resource_fail(202)','resource_fail(203)', ...
    'resource_fail(204)','resource_fail(205)','resource_fail(206)', ...
    'resource_fail(207)','resource_fail(208)','resource_fail(209)', ...
    'resource_fail(210)','resource_fail(211)','resource_fail(212)', ...
    'resource_fail(213)','resource_fail(214)','resource_fail(215)', ...
    'resource_fail(216)','resource_fail(217)'};
allReasons=true;
for i=1:numel(rejectOnlyTokens)
    allReasons=allReasons&&contains([S11 C11],rejectOnlyTokens{i});
end
sticky=contains(S11,'if(resource_exceeded()){R.success=false;R.reason=resource_reason();') && ...
       contains(C11,'resource_reason()==202||resource_reason()==203') && ...
       contains(C11,'resource_reason()==216||resource_reason()==217') && ...
       contains(C11,'PlanResult Qr;Qr.success=false;Qr.reason=resource_reason();') && ...
       contains(C11,'PlanResult Qt;Qt.success=false;Qt.reason=resource_reason();');
noProductionMutation = strcmpi(sha.rps,expected.rps);
assert(allReasons&&sticky&&noProductionMutation,'[R57O] resource guard reject-only contract failed.');

fprintf(' resource reasons 201..217 present        = 1\n');
fprintf(' sticky top-level fail-closed semantics   = 1\n');
fprintf(' ordinary-return transport is reject-only= 1\n');
fprintf(' production source unchanged             = 1\n');

%% E. V12 dominance theorem: valid lemma, over-strong global interpretation
fprintf('\n[E. V12 RESOURCE-DOMINANCE REINTERPRETATION]\n');
U=V11.discovery_usage;
prefixCurrent = double(U.episode_splits)<=double(U.episode_leaves) && ...
    double(U.stageA_splits)<=double(U.stageA_leaves) && ...
    double(U.stageB_splits)<=double(U.stageB_leaves) && ...
    double(U.geometry_splits)<=double(U.geometry_leaves) && ...
    double(U.tail_splits)<=double(U.tail_leaves) && ...
    double(U.geometry_depth)<=double(U.geometry_splits);
sourceDominance = contains(S11,'++G_RU.episode_leaves;') && contains(S11,'++G_RU.episode_splits;') && ...
    contains(S11,'G_RU.stageA_splits;++G_RU.stageA_leaves;') && ...
    contains(S11,'++G_RU.stageB_splits;++G_RU.stageB_leaves;') && ...
    contains(S11,'++G_RU.geometry_leaves;') && contains(S11,'++G_RU.geometry_splits;') && ...
    contains(C11,'++G_RU.tail_leaves;') && contains(C11,'++G_RU.tail_splits;');
dominanceLemmaValid=prefixCurrent&&sourceDominance;
assert(dominanceLemmaValid,'[R57O] V12 split/leaf dominance lemma failed.');

% Terminology correction: sign_branch_upper depends on E; qp_subset_upper
% depends on QP dimensions/row count; candidate count is downstream of sign
% enumeration.  Therefore the 11 coordinates are not algebraically
% independent in the strict mathematical sense.
strictIndependence=false;
fprintf(' split/depth dominance lemma              = 1  [KEEP]\n');
fprintf(' strict algebraic independence of 11 axes = 0  [RENAME]\n');
fprintf(' preferred term                           = GUARDED/NONDOMINATED RESOURCE COORDINATES\n');

%% F. Active-reference / pre-admission resource completeness
fprintf('\n[F. ACTIVE-REFERENCE PRE-ADMISSION RESOURCE AUDIT]\n');
AR=extract_cpp_struct(SK,'struct ActiveReference {');
activeRepairVector=contains(AR,'std::vector<Correction> repairs');
repairLoop=contains(AR,'for(const auto& C:repairs)');
segmentLoop=contains(AR,'for(const auto& s:C.seg)');
assert(activeRepairVector&&repairLoop&&segmentLoop, ...
    '[R57O] ActiveReference repair-evaluation structure changed.');

op4=parse_op4_block(C11);
parseRefPos=strfind(op4,'ActiveReference R=parse_ref(P)');
parseLivePos=strfind(op4,'Live L=parse_live(P)');
assert(isscalar(parseRefPos)&&isscalar(parseLivePos),'[R57O] op4 parse order unresolved.');
parseRefBeforeGuard=parseRefPos<parseLivePos;
resourceConfiguredInLive=contains(C11,'resource_configure(rc,nobs);');
repairCapPresent=contains(C11,'resource_max_repairs')||contains(C11,'max_repairs')|| ...
    contains(C11,'resource_max_repair_segments')||contains(C11,'max_repair_segments');

rt=record_for_candidate(C.trace_records,1);
pkt=rt.pkt;
currentRepairCount=numel(double(pkt.repair_t0));
currentRepairSegments=numel(double(pkt.repair_seg0));
currentRepairZero=(currentRepairCount==0)&&(currentRepairSegments==0);

% Even if production eventually proves that only one active repaired version
% can be serialized, that theorem is not yet present in the native resource
% contract.  Do not assume unbounded growth, but do not claim it bounded either.
repairResourceClosure = (~parseRefBeforeGuard) && repairCapPresent;

fprintf(' ActiveReference contains repair vector   = %d\n',activeRepairVector);
fprintf(' eval loops over repairs / segments       = %d / %d\n',repairLoop,segmentLoop);
fprintf(' op4 parse_ref before parse_live guard    = %d\n',parseRefBeforeGuard);
fprintf(' explicit repair/segment resource cap     = %d\n',repairCapPresent);
fprintf(' current authority repairs / segments     = %d / %d\n',currentRepairCount,currentRepairSegments);
fprintf(' current timing authority is zero-repair  = %d\n',currentRepairZero);
fprintf(' global repaired-reference work closed    = %d\n',repairResourceClosure);

%% G. Corridor discretization / row-construction work
fprintf('\n[G. CORRIDOR CONSTRUCTION RESOURCE AUDIT]\n');
corridorLoop=contains(SK,'while(ta<te-8*epsd()*std::max(1.0,std::fabs(te)))');
corridorDt=contains(SK,'tb=std::min(te,ta+L.dt)');
corridorCap=contains(S11,'max_corridor')||contains(S11,'corridor_cells')||contains(S11,'corridor_rows');
qpCap=contains(S11,'resource_comb_sum_cap')&&contains(S11,'max_qp_subset_tests');

% A QP subset cap bounds active-set enumeration after A,b exist.  It does
% not by itself prove a bound on the preceding dt-cell corridor construction.
% A separate source theorem may later prove corridor cells <= an existing
% episode resource.  Until then this is OPEN, not a failure of safety math.
corridorDominanceProved=false;
corridorResourceClosure = ~corridorLoop || corridorCap || corridorDominanceProved;

fprintf(' dt-cell corridor loop present            = %d\n',corridorLoop&&corridorDt);
fprintf(' QP combinatorial pre-admission present   = %d\n',qpCap);
fprintf(' explicit corridor cell/row cap present  = %d\n',corridorCap);
fprintf(' corridor<=existing-resource proof bound = %d\n',corridorDominanceProved);
fprintf(' corridor finite-work closure             = %d\n',corridorResourceClosure);

%% H. Obstacle model scope
fprintf('\n[H. OBSTACLE-MODEL SCOPE AUDIT]\n');
ctx=extract_between(RPS, ...
    'function CTX = build_generator_request_context_compact', ...
    'function prime_generator_integration_variants');
ctxStaticPose=contains(ctx,"'centers'")&&contains(ctx,"'rotations_flat'")&&contains(ctx,"'base_axes'");
ctxLinearVelocity=contains(ctx,"'velocities'")||contains(ctx,"'center_velocities'")||contains(ctx,"'velocity'");
ctxAngularVelocity=contains(ctx,"'angular_velocities'")||contains(ctx,"'angular_velocity'");
nativeStaticPose=contains(CK,'fld(P,"centers")')&&contains(CK,'fld(P,"rotations_flat")')&&contains(CK,'fld(P,"base_axes")');
nativeLinearVelocity=contains(CK,'fld(P,"velocities")')||contains(CK,'fld(P,"center_velocities")')||contains(CK,'fld(P,"velocity")');
nativeAngularVelocity=contains(CK,'fld(P,"angular_velocities")')||contains(CK,'fld(P,"angular_velocity")');

staticPoseClosed=ctxStaticPose&&nativeStaticPose;
movingObstacleClosed=ctxLinearVelocity&&ctxAngularVelocity&&nativeLinearVelocity&&nativeAngularVelocity;

fprintf(' arbitrary center/R/axes packet present  = %d\n',staticPoseClosed);
fprintf(' linear obstacle velocity carried         = %d\n',ctxLinearVelocity&&nativeLinearVelocity);
fprintf(' angular obstacle velocity carried        = %d\n',ctxAngularVelocity&&nativeAngularVelocity);
fprintf(' current R57 moving/rotating model closed = %d\n',movingObstacleClosed);
if ~movingObstacleClosed
    fprintf(' scope note                               = CURRENT R57 NATIVE CERTIFICATE IS STATIC-POSE ELLIPSOID SCOPE\n');
end

%% I. Active speed + final commit authority
fprintf('\n[I. ACTIVE-SPEED / TOWING-DYNAMICS / COMMIT AUTHORITY]\n');
activeSpeedContext=contains(ctx,'SC=obj.active_speed_certificate;') && ...
    contains(ctx,'obj.validate_active_speed_certificate(SC);') && ...
    contains(ctx,"'active_payload_speed_ub_m_s'") && ...
    contains(ctx,"'active_uav_speed_ub_m_s'") && ...
    contains(ctx,"'active_qdot_ub_rad_s'");
commitBlock=extract_between(RPS,'function commit_repaired_reference', ...
    'function '); % helper selects the *next* function after the matched one
versionBoundSpeed=contains(commitBlock,"strcmpi(char(sp.reference_mode),'REPAIRED')") && ...
    contains(commitBlock,'double(sp.reference_version)==newVersion') && ...
    contains(commitBlock,'obj.active_speed_certificate=sp;') && ...
    contains(commitBlock,'obj.validate_active_speed_certificate(obj.active_speed_certificate);');
fullCommitRequires=contains(commitBlock,'dynamics_certificate') && ...
    contains(commitBlock,'hlc_robust_certificate') && ...
    contains(commitBlock,'backup_certificate') && ...
    contains(commitBlock,'position_c6_certificate') && ...
    contains(commitBlock,'yaw_c6_certificate') && ...
    contains(commitBlock,'finite_sensing_certificate') && ...
    contains(commitBlock,'compute_deadline_certificate') && ...
    contains(commitBlock,'dependency_certificate') && ...
    contains(commitBlock,'obj.validate_repaired_candidate');

assert(activeSpeedContext&&versionBoundSpeed&&fullCommitRequires, ...
    '[R57O] production commit/speed fail-closed architecture drifted.');

% R57K itself certifies kinematics/geometry/rejoin of this fixed-tU core;
% the production commit interface still requires the additional towing
% tracking/HLC/actuator/backup certificates.  Keep those layers distinct.
fprintf(' active speed bound is current-reference  = 1\n');
fprintf(' repaired speed certificate version-bound= 1\n');
fprintf(' final commit requires dynamics/HLC/etc. = 1\n');
fprintf(' R57K alone is full final commit authority= 0\n');
fprintf(' production interface fails closed on it = 1\n');

%% J. Timing-scope audit
fprintf('\n[J. 4-ms TIMING-SCOPE AUDIT]\n');
lSrc=find_source('AF7R57L_QPC_HARD4MS_CURRENT_PACKET_AUDIT.m');
lSHA=sha256_file(lSrc);
assert(strcmpi(lSHA,'cb6c3e3abd0c93af17d830465253eff6dce9c54410154b3564750e74ca434f69'), ...
    '[R57O] R57L timing source changed.');
LS=normalize_lf(fileread(lSrc));
timed=extract_matlab_function(LS,'function [a,R,b]=timed_call(pkt,tU)');
coreOnlyTiming=contains(timed,'a=af7r57l_qpc_stamp_mex();') && ...
    contains(timed,'R=af7r57k_corrected_core_mex(4,pkt,tU,tU);') && ...
    contains(timed,'b=af7r57l_qpc_stamp_mex();');
productionHasPreCoreWork=contains(RPS,'obj.evaluate_active_reference_xd') && ...
    contains(RPS,'obj.raw_sensor_gate') && ...
    contains(RPS,'obj.get_active_reference_packet') && ...
    contains(RPS,'build_generator_request_context_compact');
endToEndTimingClosed = false;

fprintf(' R57L QPC encloses corrected op4 only     = %d\n',coreOnlyTiming);
fprintf(' Flight path has pre/core/post work       = %d\n',productionHasPreCoreWork);
fprintf(' prepared-packet core <4 ms qualification = %d\n',logical(L.current_packet_hard_4ms));
fprintf(' sensor-to-commit-ready <=4 ms closed     = %d\n',endToEndTimingClosed);
fprintf(' OS-scheduling WCET theorem               = 0\n');

%% K. Corrected interpretation / truth table
fprintf('\n[K. CORRECTED THEORY STATUS TABLE]\n');
status=struct();
status.r57g_numerical_artifact_correction='CLOSED';
status.r57i_sufficient_bound_semantics='CLOSED_AND_PRESERVED';
status.r57k_fixed_tu_continuous_core='CLOSED_FOR_BOUND_AUTHORITY';
status.r57k_candidate_synthesis_identity='CLOSED';
status.v10_v11_failclosed_transport='CLOSED';
status.v12_split_depth_dominance='CLOSED';
status.v12_strict_independent_basis='RETRACT_NAME';
status.global_op4_finite_work_basis=ternary(repairResourceClosure&&corridorResourceClosure, ...
    'CLOSED','OPEN');
status.repaired_reference_parse_work=ternary(repairResourceClosure,'CLOSED','OPEN');
status.corridor_construction_work=ternary(corridorResourceClosure,'CLOSED','OPEN');
status.static_pose_ellipsoid_scope=ternary(staticPoseClosed,'CLOSED','OPEN');
status.moving_rotating_obstacle_scope=ternary(movingObstacleClosed,'CLOSED','OPEN');
status.full_towing_dynamics_hlc_authority='OPEN_OUTSIDE_R57_CORE__FAILCLOSED_AT_COMMIT';
status.prepared_packet_core_4ms='CLOSED_EMPIRICAL_HARD_TRIAL_GATE';
status.end_to_end_4ms='OPEN';
status.production_caps='OPEN';
status.production_general_4ms='OPEN';

print_status('R57G numerical artifact correction',status.r57g_numerical_artifact_correction);
print_status('R57I sufficient-bound semantics',status.r57i_sufficient_bound_semantics);
print_status('R57K fixed-tU continuous core',status.r57k_fixed_tu_continuous_core);
print_status('V10/V11 resource transport',status.v10_v11_failclosed_transport);
print_status('V12 split/depth dominance lemma',status.v12_split_depth_dominance);
print_status('V12 11-axis strict independence',status.v12_strict_independent_basis);
print_status('global op4 finite-work basis',status.global_op4_finite_work_basis);
print_status('repaired-reference parse/eval work',status.repaired_reference_parse_work);
print_status('corridor construction work',status.corridor_construction_work);
print_status('static-pose ellipsoid theorem scope',status.static_pose_ellipsoid_scope);
print_status('moving/rotating obstacle scope',status.moving_rotating_obstacle_scope);
print_status('full towing dynamics/HLC authority',status.full_towing_dynamics_hlc_authority);
print_status('prepared-packet core 4ms',status.prepared_packet_core_4ms);
print_status('end-to-end 4ms',status.end_to_end_4ms);

% Explicitly downgrade only the over-broad V12 interpretation.  Do not
% invalidate its prefix invariant theorem.
v12InterpretationCorrection = dominanceLemmaValid && ...
    (~repairResourceClosure || ~corridorResourceClosure || ~strictIndependence);

fprintf('\n[L. FORMAL RE-AUDIT GATE]\n');
fprintf(' historical safety mathematics preserved = %d\n',mathPreserved);
fprintf(' resource reject semantics preserved      = %d\n',allReasons&&sticky);
fprintf(' V12 dominance lemma preserved            = %d\n',dominanceLemmaValid);
fprintf(' V12 global finite-work interpretation downgraded = %d\n',v12InterpretationCorrection);
fprintf(' production source modified               = 0\n');
fprintf(' production-ready theorem                 = 0\n');

next='CLOSE_PREPARSE_ACTIVE_REFERENCE_AND_CORRIDOR_RESOURCE_WORK__THEN_DECIDE_MOVING_OBSTACLE_SCOPE__THEN_END_TO_END_4MS';
fprintf(' next action                              = %s\n',next);
fprintf('====================================================================================================\n');

OUT.audit_closed=true;
OUT.production_ready=false;
OUT.production_modified=false;
OUT.source_sha=sha;
OUT.math_preserved=mathPreserved;
OUT.resource_reject_semantics_preserved=allReasons&&sticky;
OUT.v12_dominance_lemma_valid=dominanceLemmaValid;
OUT.v12_strict_independence_valid=false;
OUT.v12_global_finite_work_interpretation_accepted=~v12InterpretationCorrection;
OUT.active_reference=struct( ...
    'repair_vector_present',activeRepairVector, ...
    'repair_loop_present',repairLoop, ...
    'segment_loop_present',segmentLoop, ...
    'parse_ref_before_resource_configure',parseRefBeforeGuard, ...
    'explicit_repair_cap_present',repairCapPresent, ...
    'current_repair_count',currentRepairCount, ...
    'current_repair_segments',currentRepairSegments, ...
    'global_resource_closed',repairResourceClosure);
OUT.corridor=struct('dt_cell_loop_present',corridorLoop&&corridorDt, ...
    'explicit_resource_cap_present',corridorCap, ...
    'dominance_proved',corridorDominanceProved, ...
    'global_resource_closed',corridorResourceClosure);
OUT.obstacle_scope=struct('static_pose_closed',staticPoseClosed, ...
    'moving_rotating_closed',movingObstacleClosed);
OUT.active_speed_version_binding_closed=activeSpeedContext&&versionBoundSpeed;
OUT.final_commit_failclosed_requirements_closed=fullCommitRequires;
OUT.prepared_packet_core_timing_closed=logical(L.current_packet_hard_4ms)&&coreOnlyTiming;
OUT.end_to_end_4ms_closed=endToEndTimingClosed;
OUT.status=status;
OUT.next_action=next;

root=fileparts(mfilename('fullpath'));
assignin('base','AF7R57O_GLOBAL_THEORY_CONSISTENCY_AUDIT_OUT',OUT);
save(fullfile(root,'AF7R57O_GLOBAL_THEORY_CONSISTENCY_AUDIT_OUT.mat'),'OUT','-v7.3');
end

% ========================================================================
function r=record_for_candidate(R,c)
assert(iscell(R),'[R57O] trace_records is not a cell array.');
ids=cellfun(@(q)double(q.c),R);
k=find(ids==c,1,'last');
assert(~isempty(k),'[R57O] requested trace candidate absent.');
r=R{k};
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
assert(~isempty(p),'[R57O] authority unavailable: %s',fileName);
q=load(p,'OUT');assert(isfield(q,'OUT'),'[R57O] OUT missing in %s',p);A=q.OUT;
end

function fn=find_source(name)
p=which(name);
if ~isempty(p)&&isfile(p),fn=p;return;end
selfRoot=fileparts(mfilename('fullpath'));
roots={pwd,selfRoot,fileparts(selfRoot)};
u=getenv('USERPROFILE');
if ~isempty(u)
    roots{end+1}=fullfile(u,'Documents','GitHub','common_matlab','reference'); %#ok<AGROW>
    roots{end+1}=fullfile(u,'Documents','GitHub','common_matlab'); %#ok<AGROW>
end
for i=1:numel(roots)
    q=fullfile(roots{i},name);
    if isfile(q),fn=q;return;end
end
assert(false,'[R57O] source unavailable: %s',name);
end

function B=extract_cpp_function(S,marker)
S=char(S);p=strfind(S,marker);assert(numel(p)==1,'[R57O] C++ function marker count mismatch: %s',marker);
p=p(1);ob=find(S(p:end)=='{',1,'first')+p-1;assert(~isempty(ob));
d=0;cb=NaN;
for k=ob:numel(S)
    if S(k)=='{',d=d+1;elseif S(k)=='}',d=d-1;if d==0,cb=k;break;end,end,end
assert(isfinite(cb),'[R57O] unmatched braces for %s',marker);B=S(p:cb);
end

function B=extract_cpp_struct(S,marker)
B=extract_cpp_function(S,marker);
end

function B=parse_op4_block(S)
marker='if(op==4){';p=strfind(S,marker);assert(numel(p)==1,'[R57O] op4 marker mismatch.');
p=p(1);ob=p+length(marker)-1;d=0;cb=NaN;
for k=ob:numel(S)
    if S(k)=='{',d=d+1;elseif S(k)=='}',d=d-1;if d==0,cb=k;break;end,end,end
assert(isfinite(cb),'[R57O] op4 braces unmatched.');B=S(p:cb);
end

function B=extract_between(S,startMarker,endMarker)
% If endMarker is exactly 'function ', find the next MATLAB function line
% after startMarker. Otherwise use the first explicit end marker after start.
S=char(S);p=strfind(S,startMarker);assert(~isempty(p),'[R57O] start marker missing: %s',startMarker);p=p(1);
if strcmp(endMarker,'function ')
    tail=S(p+length(startMarker):end);
    q=regexp(tail,'\n\s*function\s+','start','once');
    assert(~isempty(q),'[R57O] next MATLAB function not found after %s',startMarker);
    e=p+length(startMarker)+q-2;
else
    qAll=strfind(S,endMarker);qAll=qAll(qAll>p);assert(~isempty(qAll),'[R57O] end marker missing: %s',endMarker);e=qAll(1)-1;
end
B=S(p:e);
end

function B=extract_matlab_function(S,marker)
B=extract_between(S,marker,'function ');
end

function n=token_count(s,a)
s=char(s);a=char(a);if isempty(a),n=0;else,n=numel(strfind(s,a));end %#ok<STREMP>
n=double(n);
end

function s=normalize_lf(s)
s=char(s);s=strrep(s,sprintf('\r\n'),sprintf('\n'));s=strrep(s,sprintf('\r'),sprintf('\n'));
end

function print_status(name,val)
fprintf(' %-39s = %s\n',name,char(string(val)));
end

function y=ternary(c,a,b)
if c,y=a;else,y=b;end
end

function h=sha256_file(fn)
md=java.security.MessageDigest.getInstance('SHA-256');
fid=fopen(fn,'rb');assert(fid>=0,'[R57O] cannot open %s',fn);c=onCleanup(@()fclose(fid)); %#ok<NASGU>
while true
    x=fread(fid,1024*1024,'*uint8');if isempty(x),break;end;md.update(x);
end
d=typecast(md.digest(),'uint8');h=lower(reshape(dec2hex(d,2).',1,[]));
end
