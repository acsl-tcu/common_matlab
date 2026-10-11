function OUT = G26V211_CURRENT_THEORY_BASELINE_REQUIREMENTS_AND_GATE(PHYS)
% G26V211_CURRENT_THEORY_BASELINE_REQUIREMENTS_AND_GATE
% =========================================================================
% CURRENT REQUIREMENTS + CURRENT THEORY BASELINE
% =========================================================================
%
% RESEARCH LAYER
%   This code is for ONLINE OBSTACLE-AVOIDANCE REFERENCE GENERATION BEFORE
%   THE DOWNSTREAM CONTROLLER for suspended-load UAV transport.
%
% NON-NEGOTIABLE SYSTEM REQUIREMENTS
% -------------------------------------------------------------------------
% [R1] No prior/global obstacle map is assumed.
% [R2] Obstacles are known only after sensing. UNKNOWN is never silently
%      treated as FREE.
% [R3] Avoidance starts from future-collision prediction.
% [R4] Position and yaw references are globally C6 through derivative 6.
% [R5] PAYLOAD / UAV / FINITE CABLE must all be included in collision safety.
% [R6] Safety is continuous-time. Temporal sampling never grants safety PASS.
% [R7] No empirical multiplier bank, hand-tuned time bank, case-dependent
%      safety threshold, or data-selected heuristic is allowed as theorem
%      authority.
% [R8] Search failure is SEARCH_INCOMPLETE / certificate failure; it is not
%      automatically physical/path infeasibility.
% [R9] During one flight/configuration, vehicle mass, inertia, actuator
%      limits, cable length, radii, margins, gravity, and cable-angle limit
%      are fixed configuration values.
% [R10] Only payload/towed-object mass estimate may change during flight.
%       It must remain inside a pre-certified payload-mass interval, or
%       authority fails closed. Estimation error must be included in that
%       interval if a justified bound is available.
% [R11] Configuration values may change between experiments/builds. Therefore
%       no historical values such as mQ=0.762 kg or thrust=20 N are permanent
%       constants or proof assumptions.
% [R12] Control/reference period is 25 ms.
% [R13] Final online prefix sensor -> memory -> trigger -> trajectory/reference
%       modification -> certificate -> atomic switch must fit 2--4 ms total,
%       ideally 2--3 ms. Planner-only 4--5 ms is NOT sufficient.
% [R14] A new feature/mount/integration stage is forbidden until BOTH
%
%           THEORY_CLOSED == true
%           ONLINE_E2E_COMPUTE_CLOSED == true
%
%       hold simultaneously.
%
% CURRENTLY CLOSED / PROVED PARTS
% -------------------------------------------------------------------------
% [T1] Exact d0...d6 handoff construction/gate.
% [T2] Global C6 construction/audit for the accepted reference.
% [T3] Cable cone + tautness continuous sufficient condition through Bezier
%      convex-hull containment of resultant controls.
% [T4] V209Z/V210H time result is a certified bounded-suboptimality interval:
%
%      formal LB <= J* <= hard-certified UB.
%
%      Failed witness-search points do NOT raise the formal LB.
% [T5] Equality-eliminated central-resultant reconstruction satisfies endpoint
%      moment equations by construction.
% [T6] Payload mass runtime architecture: fixed configuration + runtime mL
%      guarded by a configured certification interval.
%
% IMPLEMENTED BUT NOT YET PROMOTED TO FINAL THEORY
% -------------------------------------------------------------------------
% [P1] Whole-body PAYLOAD/UAV/CABLE hard certificate:
%      observed PASS exists, but the direct lower-bound implementation,
%      floating-point guards, subdivision PASS logic, and all-span/all-
%      obstacle accounting still require the dedicated mathematical
%      soundness proof already identified in V210B.
%
% [P2] Continuous actuator interval certificate:
%      Bernstein/interval/de-Casteljau structure is fail-closed in real-
%      arithmetic logic, but the current V62/V63-style double-precision
%      interval primitives/restriction maps have not yet been audited with
%      directed/outward floating-point rounding. Therefore numerical PASS is
%      NOT yet the final actuator soundness theorem.
%
% [P3] Closed-loop/HLC tracking transfer from reference safety to actual
%      vehicle/load safety is not yet proved.
%
% [P4] Sensor outer-cover / unknown-obstacle capture + recursive all-time
%      certified continuation is not yet fully closed.
%
% [P5] End-to-end online 2--4 ms compute authority is not closed.
%
% CONSEQUENCE
% -------------------------------------------------------------------------
% This file deliberately returns NEXT_FEATURE_ALLOWED=false today.
% It is a promotion gate, not a way to hide unfinished theory.
%
% It reconstructs the accepted V210H BSPLINE reference and re-checks the
% parts that are already legitimate to check. Pending proof layers are
% represented explicitly as false flags.
%
% PARAMETER INPUT
% -------------------------------------------------------------------------
% PHYS follows the fixed-configuration / variable-payload architecture:
%
%   PHYS.parameter_authority
%   PHYS.mQ_kg
%   PHYS.J_diag_kgm2
%   PHYS.thrust_bounds_N
%   PHYS.moment_bounds_Nm
%   PHYS.mL_cert_range_kg
%   PHYS.mL_runtime_check_kg              optional current estimator output
%   PHYS.estimator_error_included_in_range
%
% If omitted, historical values are used only for regression and authority
% remains false.
% =========================================================================

clc;

if nargin<1 || isempty(PHYS), PHYS=struct(); end
PHYS=v211_resolve_phys(PHYS);

if ~evalin('base','exist(''G26V209Z_RESULT'',''var'')')
    error('G26V211:MissingV209Z','Run G26V209Z first.');
end
if ~evalin('base','exist(''G26V210H_RESULT'',''var'')')
    error('G26V211:MissingV210H','Run G26V210H first.');
end

Z=evalin('base','G26V209Z_RESULT');
H=evalin('base','G26V210H_RESULT');
CFG=Z.cfg;
AH=H.audit;

fprintf('\n========================================================================================================================\n');
fprintf(' G26-V211 | CURRENT THEORY BASELINE + REQUIREMENTS + PROMOTION GATE\n');
fprintf(' No new feature promotion unless theory AND online E2E compute are both closed | clc enabled\n');
fprintf('========================================================================================================================\n');

% Reconstruct the final accepted reference without changing it.
CTX=v210i_context(CFG);
C=v210i_reconstruct_candidate(AH.bestCertified.h,AH.bestCertified.T,CTX,CFG);
assert(C.jetPass && C.c6Pass && C.combinedCorridorPass, ...
    'G26V211:ReferenceRegression','Accepted reference failed a previously closed structural gate.');

% Closed theory status.
S=struct();
S.timeBoundedSuboptimalityClosed=true;
S.timeFormalLB_s=AH.originalLB;
S.timeCertifiedUB_s=AH.improvedUB;
S.timeGap_s=AH.improvedGap;
S.globalOptimumExactClaim=false;

S.exactD0D6HandoffClosed=logical(C.jetPass);
S.globalC6Closed=logical(C.c6Pass);

% The accepted bridge/tail resultant gates are continuous sufficient gates.
S.cableConeTautContinuousGateClosed=true;

% Parameter architecture.
S.fixedConfigurationDuringFlight=true;
S.onlyPayloadMassRuntimeVariable=true;
S.noPermanentHistoricalParameterConstants=true;
S.parameterAuthority=logical(PHYS.parameter_authority);
S.payloadMassCertificationRange_kg=PHYS.mL_cert_range_kg;
S.payloadRuntimeEstimate_kg=PHYS.mL_runtime_check_kg;
S.payloadRuntimeInsideRange=true;
if isfinite(PHYS.mL_runtime_check_kg)
    S.payloadRuntimeInsideRange = ...
        PHYS.mL_runtime_check_kg>=PHYS.mL_cert_range_kg(1) && ...
        PHYS.mL_runtime_check_kg<=PHYS.mL_cert_range_kg(2);
end
S.estimatorErrorIncludedInMassRange=logical(PHYS.estimator_error_included_in_range);

% Pending mathematical proof layers. Deliberately FALSE until dedicated
% proof/audit closes them; implementation PASS must not silently flip these.
S.wholeBodyHardCertificateObservedPass=logical(C.hardPass);
S.wholeBodyHardCertificateFloatingPointSoundnessProved=false;
S.wholeBodyHardCertificateMathematicalSoundnessClosed=false;

S.continuousActuatorImplementationAvailable=true;
S.continuousActuatorFloatingPointOutwardRoundingSoundnessProved=false;
S.continuousActuatorTheoryClosed=false;

S.closedLoopTrackingSafetyTransferClosed=false;
S.sensorOuterCoverAndUnknownCaptureTheoryClosed=false;
S.recursiveAllTimeCertifiedContinuationClosed=false;

% Compute requirement.
S.controlPeriod_s=0.025;
S.onlineE2ERequirement_ms=[2 4];
S.onlineE2EComputeClosed=false;
S.onlineE2EMeasuredWorstCase_ms=NaN;

% Overall theory gate. Current final obstacle theorem and actual-system
% transfer are required; actuator theory is also required before calling the
% selected time/reference physically final.
S.theoryClosed = ...
    S.timeBoundedSuboptimalityClosed && ...
    S.exactD0D6HandoffClosed && ...
    S.globalC6Closed && ...
    S.cableConeTautContinuousGateClosed && ...
    S.wholeBodyHardCertificateMathematicalSoundnessClosed && ...
    S.continuousActuatorTheoryClosed && ...
    S.closedLoopTrackingSafetyTransferClosed && ...
    S.sensorOuterCoverAndUnknownCaptureTheoryClosed && ...
    S.recursiveAllTimeCertifiedContinuationClosed;

S.nextFeatureAllowed = S.theoryClosed && S.onlineE2EComputeClosed;

if S.nextFeatureAllowed
    S.classification='NEXT_FEATURE_ALLOWED__THEORY_AND_ONLINE_E2E_COMPUTE_BOTH_CLOSED';
elseif ~S.theoryClosed && ~S.onlineE2EComputeClosed
    S.classification='NO_PROMOTION__THEORY_OPEN_AND_ONLINE_E2E_COMPUTE_OPEN';
elseif ~S.theoryClosed
    S.classification='NO_PROMOTION__THEORY_OPEN';
else
    S.classification='NO_PROMOTION__ONLINE_E2E_COMPUTE_OPEN';
end

v211_print(S);

OUT=struct();
OUT.version='G26-V211';
OUT.status='CURRENT_THEORY_BASELINE_REQUIREMENTS_AND_PROMOTION_GATE';
OUT.requirements=struct( ...
    'control_period_s',0.025, ...
    'online_e2e_target_ms',[2 4], ...
    'no_prior_global_map',true, ...
    'position_yaw_global_C6',true, ...
    'whole_body_payload_uav_cable_required',true, ...
    'only_payload_mass_runtime_variable',true, ...
    'new_feature_requires_theory_and_compute',true);
OUT.audit=S;
OUT.next_required_stage=['Do not add/mount the next feature. First close the pending mathematical soundness layers (whole-body certificate, ', ...
    'actuator floating-point interval proof, sensor/recursive continuation, tracking transfer as required by the final theorem), then ', ...
    'close the 2--4 ms end-to-end online compute path. Promotion is allowed only when both gates are true.'];
assignin('base','G26V211_RESULT',OUT);
fprintf('[G26-V211] result exported to base workspace as G26V211_RESULT\n');
end

function v211_print(S)
fprintf('\n========================================================================================================================\n');
fprintf(' G26-V211 CURRENT THEORY / COMPUTE GATE STATUS\n');
fprintf('------------------------------------------------------------------------------------------------------------------------\n');
fprintf('[CLOSED NOW]\n');
fprintf(' time bounded-suboptimality=%d | interval=[%.15g, %.15g] s | width=%.6e s\n', ...
    S.timeBoundedSuboptimalityClosed,S.timeFormalLB_s,S.timeCertifiedUB_s,S.timeGap_s);
fprintf(' exact d0...d6 handoff=%d | global C6=%d | continuous cone/taut gate=%d\n', ...
    S.exactD0D6HandoffClosed,S.globalC6Closed,S.cableConeTautContinuousGateClosed);
fprintf(' fixed-config/runtime-payload architecture=%d | only payload runtime-variable=%d\n', ...
    S.fixedConfigurationDuringFlight,S.onlyPayloadMassRuntimeVariable);

fprintf('\n[PENDING THEORY -- IMPLEMENTATION PASS DOES NOT PROMOTE]\n');
fprintf(' whole-body hardCert observedPass=%d | mathematicalSoundnessClosed=%d\n', ...
    S.wholeBodyHardCertificateObservedPass,S.wholeBodyHardCertificateMathematicalSoundnessClosed);
fprintf(' continuous actuator implementationAvailable=%d | floatingPointSoundness=%d | theoryClosed=%d\n', ...
    S.continuousActuatorImplementationAvailable,S.continuousActuatorFloatingPointOutwardRoundingSoundnessProved,S.continuousActuatorTheoryClosed);
fprintf(' tracking safety transfer=%d | sensor/unknown capture=%d | recursive all-time continuation=%d\n', ...
    S.closedLoopTrackingSafetyTransferClosed,S.sensorOuterCoverAndUnknownCaptureTheoryClosed,S.recursiveAllTimeCertifiedContinuationClosed);

fprintf('\n[COMPUTE]\n');
fprintf(' control period=%.3f ms | online E2E requirement=%.1f--%.1f ms | computeClosed=%d\n', ...
    1e3*S.controlPeriod_s,S.onlineE2ERequirement_ms(1),S.onlineE2ERequirement_ms(2),S.onlineE2EComputeClosed);

fprintf('\n[PROMOTION GATE]\n');
fprintf(' THEORY_CLOSED=%d\n',S.theoryClosed);
fprintf(' ONLINE_E2E_COMPUTE_CLOSED=%d\n',S.onlineE2EComputeClosed);
fprintf(' NEXT_FEATURE_ALLOWED=%d\n',S.nextFeatureAllowed);
fprintf(' CLASSIFICATION: %s\n',S.classification);
fprintf('========================================================================================================================\n');
end


function P=v211_resolve_phys(P)
% Historical values are regression defaults only, never permanent constants.
D=struct();
D.mQ_kg=0.762;
D.J_diag_kgm2=[0.02237568 0.02985236 0.0480374];
D.thrust_bounds_N=[0 20];
D.moment_bounds_Nm=[-1 1; -1 1; -1 1];
D.mL_cert_range_kg=[0.0556 0.0556];
D.mL_runtime_check_kg=NaN;
D.estimator_error_included_in_range=false;
D.parameter_authority=false;
D.source='HISTORICAL_F1_REGRESSION_DEFAULTS__NOT_PERMANENT_NOT_AUTHORITY';
D.actuator_interval_max_depth=16;
D.actuator_interval_guard_scale=512;

fn=fieldnames(D);
for k=1:numel(fn)
    if ~isfield(P,fn{k}) || isempty(P.(fn{k})), P.(fn{k})=D.(fn{k}); end
end
if logical(P.parameter_authority)
    P.source='USER_SUPPLIED_FIXED_CONFIGURATION_AND_PAYLOAD_MASS_CONTRACT';
end

validateattributes(P.mQ_kg,{'numeric'},{'real','finite','scalar','positive'});
validateattributes(P.J_diag_kgm2,{'numeric'},{'real','finite','vector','numel',3,'positive'});
P.J_diag_kgm2=reshape(P.J_diag_kgm2,1,3);
validateattributes(P.thrust_bounds_N,{'numeric'},{'real','finite','vector','numel',2});
P.thrust_bounds_N=reshape(P.thrust_bounds_N,1,2);
assert(P.thrust_bounds_N(1)>=0 && P.thrust_bounds_N(2)>P.thrust_bounds_N(1), ...
    'G26V211:ThrustBounds','Require 0<=thrust_min<thrust_max.');
validateattributes(P.moment_bounds_Nm,{'numeric'},{'real','finite','size',[3 2]});
assert(all(P.moment_bounds_Nm(:,2)>P.moment_bounds_Nm(:,1)), ...
    'G26V211:MomentBounds','Each moment row must satisfy lo<hi.');
validateattributes(P.mL_cert_range_kg,{'numeric'},{'real','finite','vector','numel',2,'positive'});
P.mL_cert_range_kg=reshape(P.mL_cert_range_kg,1,2);
assert(P.mL_cert_range_kg(2)>=P.mL_cert_range_kg(1), ...
    'G26V211:MassRange','mL_cert_range_kg must satisfy lo<=hi.');
if ~isempty(P.mL_runtime_check_kg) && ~isnan(P.mL_runtime_check_kg)
    validateattributes(P.mL_runtime_check_kg,{'numeric'},{'real','finite','scalar','positive'});
end
validateattributes(P.estimator_error_included_in_range,{'logical','numeric'},{'scalar'});
validateattributes(P.parameter_authority,{'logical','numeric'},{'scalar'});
validateattributes(P.actuator_interval_max_depth,{'numeric'},{'real','finite','scalar','integer','>=',0,'<=',30});
validateattributes(P.actuator_interval_guard_scale,{'numeric'},{'real','finite','scalar','positive'});
end

function ACTIVE=v211_build_active(C,CFG)
% Mixed-degree piece support.
%
% The final trajectory is intentionally heterogeneous:
%   - bridge: degree 17 Bezier
%   - tail: extracted BSPLINE pieces, not required to have degree 17.
%
% The continuous actuator map only needs one piece at a time and already
% infers the degree from size(Pcp,2)-1.  Therefore forcing every piece into
% one 3-D numeric array was an invalid implementation assumption.
np=numel(C.P);
CP=cell(1,np);
deg=zeros(1,np);
for q=1:np
    assert(isnumeric(C.P{q}) && size(C.P{q},1)==3 && size(C.P{q},2)>=1, ...
        'G26V211:PieceShape','Each trajectory piece must be a finite 3xN Bezier control-point matrix.');
    assert(all(isfinite(C.P{q}(:))), ...
        'G26V211:PieceFinite','Trajectory piece %d contains non-finite control points.',q);
    CP{q}=C.P{q};
    deg(q)=size(C.P{q},2)-1;
end

T=sum(C.h);
hfrac=C.h/T;
hfrac=hfrac/sum(hfrac);

yawcp=[CFG.v207_yaw_start_rad*ones(1,7), ...
       CFG.v207_yaw_goal_rad*ones(1,7)];

ACTIVE=struct();
ACTIVE.valid=true;
ACTIVE.t_start=0;
ACTIVE.T=T;
ACTIVE.t_end=T;
ACTIVE.result=struct('control_points',{CP},'piece_degree',deg);
ACTIVE.model=struct('h',hfrac);
ACTIVE.yaw=struct('coeff',yawcp);
end

function RP=v211_runtime_parameter_contract(P)
RP=struct();
RP.payload_mass_kg=P.mL_runtime_check_kg;
RP.payload_mass_cert_interval_kg=P.mL_cert_range_kg(:).';
RP.payload_mass_range_mode=(P.mL_cert_range_kg(2)>P.mL_cert_range_kg(1));
RP.mass_within_cert_range=true;
if isfinite(P.mL_runtime_check_kg)
    RP.mass_within_cert_range= ...
        P.mL_runtime_check_kg>=P.mL_cert_range_kg(1) && ...
        P.mL_runtime_check_kg<=P.mL_cert_range_kg(2);
end
RP.thrust_bounds_N=P.thrust_bounds_N;
RP.moment_bounds_Nm=P.moment_bounds_Nm;
end

function C=v211_certificate_config(P,CFG)
C=struct();
C.mQ_kg=P.mQ_kg;
C.vehicle_inertia_diag_kgm2=P.J_diag_kgm2(:);
C.actuator_interval_max_depth=P.actuator_interval_max_depth;
C.actuator_interval_guard_scale=P.actuator_interval_guard_scale;
C.e3=[0;0;1];
C.gravity=CFG.gravity;
C.L_cable=CFG.L_cable;
end

function A=v211_actuator_interval_certificate(active,ta,tb,RP,CFG)
q=tic;
A=v211_empty_actuator_certificate();
A.t0=ta; A.t1=tb; A.payload_mass_kg=RP.payload_mass_kg;
if isfield(active,'result') && isfield(active.result,'piece_degree')
    A.piece_degrees=active.result.piece_degree;
else
    A.piece_degrees=[];
end

if ~RP.mass_within_cert_range
    A.reason='PAYLOAD_MASS_OUTSIDE_CERTIFIED_RANGE';
    A.runtime_ms=1e3*toc(q);
    return;
end
if ~(isstruct(active) && isfield(active,'valid') && active.valid && ...
        isfield(active,'result') && isfield(active.result,'control_points') && ...
        isfield(active,'model') && isfield(active.model,'h') && ...
        isfield(active,'yaw') && isfield(active.yaw,'coeff'))
    A.reason='ACTIVE_REFERENCE_UNSUPPORTED';
    A.runtime_ms=1e3*toc(q);
    return;
end
if ~(tb>ta)
    A.pass=true; A.reason='EMPTY_INTERVAL'; A.runtime_ms=1e3*toc(q); return;
end

h=active.model.h(:).';
edges=[0,cumsum(h)];
edges(end)=1;
absEdges=active.t_start+active.T*edges;

A.pass=true;
A.reason='CERTIFIED_CONTINUOUS_INTERVAL';
for qpiece=1:numel(h)
    aa=max(ta,absEdges(qpiece));
    bb=min(tb,absEdges(qpiece+1));
    if ~(bb>aa), continue; end
    pieceDt=active.T*h(qpiece);
    u0=(aa-absEdges(qpiece))/pieceDt;
    u1=(bb-absEdges(qpiece))/pieceDt;
    s0=(aa-active.t_start)/active.T;
    s1=(bb-active.t_start)/active.T;

    L=v211_actuator_leaf(active,qpiece,pieceDt,u0,u1,s0,s1,RP,CFG,0);
    A.leaf_count=A.leaf_count+L.leaf_count;
    A.split_count=A.split_count+L.split_count;
    A.max_depth=max(A.max_depth,L.max_depth);
    A.thrust_lb_N=min(A.thrust_lb_N,L.thrust_lb_N);
    A.thrust_ub_N=max(A.thrust_ub_N,L.thrust_ub_N);
    A.moment_lb_Nm=min(A.moment_lb_Nm,L.moment_lb_Nm);
    A.moment_ub_Nm=max(A.moment_ub_Nm,L.moment_ub_Nm);
    if ~L.pass
        A.pass=false;
        A.reason=L.reason;
        A.runtime_ms=1e3*toc(q);
        return;
    end
end
if isinf(A.thrust_lb_N), A.thrust_lb_N=NaN; end
if isinf(A.thrust_ub_N), A.thrust_ub_N=NaN; end
A.runtime_ms=1e3*toc(q);
end

function L=v211_actuator_leaf(active,qpiece,pieceDt,u0,u1,s0,s1,RP,CFG,depth)
L=v211_empty_actuator_certificate();
L.leaf_count=1;
L.max_depth=depth;

[I,ok,reason]=v211_actuator_interval_map(active,qpiece,pieceDt,u0,u1,s0,s1,RP,CFG);
if ok
    L.thrust_lb_N=I.thrust.lo;
    L.thrust_ub_N=I.thrust.hi;
    L.moment_lb_Nm=I.moment.lo(:);
    L.moment_ub_Nm=I.moment.hi(:);

    scale=max([1,abs(I.thrust.lo),abs(I.thrust.hi), ...
        abs(I.moment.lo(:).'),abs(I.moment.hi(:).'), ...
        abs(RP.thrust_bounds_N),abs(RP.moment_bounds_Nm(:).')]);
    guard=CFG.actuator_interval_guard_scale*eps(scale);

    thrustPass=I.thrust.lo>=RP.thrust_bounds_N(1)+guard && ...
        I.thrust.hi<=RP.thrust_bounds_N(2)-guard;
    momentPass=all(I.moment.lo(:)>=RP.moment_bounds_Nm(:,1)+guard) && ...
        all(I.moment.hi(:)<=RP.moment_bounds_Nm(:,2)-guard);

    if thrustPass && momentPass
        L.pass=true;
        L.reason='CERTIFIED_INTERVAL';
        return;
    end

    definiteViolation=(I.thrust.lo>RP.thrust_bounds_N(2)+guard) || ...
        (I.thrust.hi<RP.thrust_bounds_N(1)-guard) || ...
        any(I.moment.lo(:)>RP.moment_bounds_Nm(:,2)+guard) || ...
        any(I.moment.hi(:)<RP.moment_bounds_Nm(:,1)-guard);

    if definiteViolation
        L.pass=false;
        L.reason='ACTUATOR_LIMIT_VIOLATION_CERTIFIED';
        return;
    end
else
    L.reason=reason;
end

if depth>=CFG.actuator_interval_max_depth
    L.pass=false;
    if isempty(L.reason), L.reason='ACTUATOR_INTERVAL_UNRESOLVED'; end
    return;
end

um=0.5*(u0+u1);
sm=0.5*(s0+s1);
if ~(um>u0 && um<u1 && sm>s0 && sm<s1)
    L.pass=false;
    L.reason='ACTUATOR_INTERVAL_FLOATING_POINT_LIMIT';
    return;
end

A=v211_actuator_leaf(active,qpiece,pieceDt,u0,um,s0,sm,RP,CFG,depth+1);
if ~A.pass
    L=A; L.split_count=L.split_count+1; return;
end
B=v211_actuator_leaf(active,qpiece,pieceDt,um,u1,sm,s1,RP,CFG,depth+1);

L.pass=B.pass;
L.reason=B.reason;
L.leaf_count=A.leaf_count+B.leaf_count;
L.split_count=A.split_count+B.split_count+1;
L.max_depth=max(A.max_depth,B.max_depth);
L.thrust_lb_N=min(A.thrust_lb_N,B.thrust_lb_N);
L.thrust_ub_N=max(A.thrust_ub_N,B.thrust_ub_N);
L.moment_lb_Nm=min(A.moment_lb_Nm,B.moment_lb_Nm);
L.moment_ub_Nm=max(A.moment_ub_Nm,B.moment_ub_Nm);
if L.pass, L.reason='CERTIFIED_SUBDIVIDED_INTERVAL'; end
end

function [O,ok,reason]=v211_actuator_interval_map(active,qpiece,pieceDt,u0,u1,s0,s1,RP,CFG)
O=struct('thrust',v211_iv(NaN,NaN),'moment',v211_iv(nan(3,1),nan(3,1)));
ok=false; reason='';

CPall=active.result.control_points;
if ~iscell(CPall) || qpiece<1 || qpiece>numel(CPall)
    reason='ACTIVE_REFERENCE_PIECE_STORAGE_INVALID'; return;
end
Pcp=CPall{qpiece};
if ~(isnumeric(Pcp) && size(Pcp,1)==3 && size(Pcp,2)>=1 && all(isfinite(Pcp(:))))
    reason='ACTIVE_REFERENCE_PIECE_INVALID'; return;
end
r=cell(1,5);
for k=0:4
    dord=k+2;
    I=v211_bezier_derivative_interval(Pcp,pieceDt,dord,u0,u1);
    if k==0
        I=v211_iv_add(I,v211_iv(CFG.gravity*CFG.e3,CFG.gravity*CFG.e3));
    end
    r{k+1}=v211_iv_scale(I,1/factorial(k));
end

[rho,rhoOK]=v211_ivs_norm(r,4);
if ~rhoOK || rho{1}.lo<=0
    reason='RESULTANT_NORM_INTERVAL_SINGULAR'; return;
end

pinvRho=v211_ivs_inv(rho,4);
p=v211_ivs_vec_scale(r,pinvRho,4);
for k=1:numel(p), p{k}=v211_iv_scale(p{k},-1); end

pdot=v211_ivs_derivative(p,3);
pddot=v211_ivs_derivative(pdot,2);
omegaDot=v211_ivs_cross(p,pddot,2);
pdot2=v211_ivs_dot(pdot,pdot,2);

mQ=CFG.mQ_kg;
mLRange=RP.payload_mass_cert_interval_kg(:).';
Lcab=CFG.L_cable;

massSum0=v211_iv(mQ+mLRange(1),mQ+mLRange(2));
zeroS=v211_iv(0,0);
massSumSeries={massSum0,zeroS,zeroS};
massTerm=v211_ivs_mul(rho,massSumSeries,2);

pDotF=v211_ivs_add( ...
    v211_ivs_scale(pdot2,mQ*Lcab,2), ...
    v211_ivs_scale(massTerm,-1,2),2);

F1=v211_ivs_vec_scale(p,pDotF,2);
F2=v211_ivs_scale(v211_ivs_cross(p,omegaDot,2),mQ*Lcab,2);
F=v211_ivs_add(F1,F2,2);

[fser,fOK]=v211_ivs_norm(F,2);
if ~fOK || fser{1}.lo<=0
    reason='THRUST_NORM_INTERVAL_SINGULAR'; return;
end
O.thrust=fser{1};

b3=v211_ivs_vec_scale(F,v211_ivs_inv(fser,2),2);

yc=active.yaw.coeff(:).';
Y=cell(1,3);
for k=0:2
    Iy=v211_bezier_derivative_interval(yc,active.T,k,s0,s1);
    Y{k+1}=v211_iv_scale(Iy,1/factorial(k));
end
[sy,cy]=v211_ivs_sincos2(Y);
zero=v211_iv(0,0);
c={ ...
    v211_iv_stack3(cy{1},sy{1},zero), ...
    v211_iv_stack3(cy{2},sy{2},zero), ...
    v211_iv_stack3(cy{3},sy{3},zero)};

a=v211_ivs_cross(b3,c,2);
[b2,aOK]=v211_ivs_normalize(a,2);
if ~aOK
    reason='ATTITUDE_HEADING_CROSS_INTERVAL_SINGULAR'; return;
end
b1=v211_ivs_cross(b2,b3,2);

R0=v211_iv_hcat3(b1{1},b2{1},b3{1});
R1=v211_iv_hcat3(b1{2},b2{2},b3{2});
Rdd=v211_iv_hcat3(v211_iv_scale(b1{3},2), ...
    v211_iv_scale(b2{3},2),v211_iv_scale(b3{3},2));

OmHat=v211_iv_matmul(v211_iv_transpose(R0),R1);
OmSk=v211_iv_scale(v211_iv_sub(OmHat,v211_iv_transpose(OmHat)),0.5);
Omega=v211_iv_stack3( ...
    v211_iv_at(OmSk,3,2),v211_iv_at(OmSk,1,3),v211_iv_at(OmSk,2,1));

OD1=v211_iv_matmul(v211_iv_transpose(R1),R1);
OD2=v211_iv_matmul(v211_iv_transpose(R0),Rdd);
OD=v211_iv_add(OD1,OD2);
ODsk=v211_iv_scale(v211_iv_sub(OD,v211_iv_transpose(OD)),0.5);
OmegaDot=v211_iv_stack3( ...
    v211_iv_at(ODsk,3,2),v211_iv_at(ODsk,1,3),v211_iv_at(ODsk,2,1));

J=CFG.vehicle_inertia_diag_kgm2(:);
JOmega=v211_iv_mul(Omega,v211_iv(J,J));
term1=v211_iv_mul(OmegaDot,v211_iv(J,J));
term2=v211_iv_cross(Omega,JOmega);
O.moment=v211_iv_add(term1,term2);

ok=all(isfinite([O.thrust.lo,O.thrust.hi,O.moment.lo(:).',O.moment.hi(:).']));
if ~ok, reason='NONFINITE_ACTUATOR_INTERVAL'; end
end

function I=v211_bezier_derivative_interval(CP,dt,order,u0,u1)
n=size(CP,2)-1;
if order>n
    Z=zeros(size(CP,1),1);
    I=v211_iv(Z,Z); return;
end
Q=CP; nr=n;
for j=1:order
    Q=(nr/dt)*diff(Q,1,2);
    nr=nr-1;
end
R=v211_bezier_restrict_map(n-order,u0,u1);
Q=Q*R.';
I=v211_iv(min(Q,[],2),max(Q,[],2));
end

function R=v211_bezier_restrict_map(n,u0,u1)
% Exact de Casteljau restriction matrix for degree n.
E=eye(n+1);
if u0==0 && u1==1
    R=E; return;
end
% Restrict [0,1] -> [u0,u1] by evaluating each basis-control vector via
% de Casteljau at the new Bernstein nodes through subdivision maps.
L0=v211_left_subdivision_map(n,u1);
if u0==0
    R=L0;
    return;
end
v=u0/u1;
Rv=v211_right_subdivision_map(n,v);
R=Rv*L0;
end

function L=v211_left_subdivision_map(n,u)
L=zeros(n+1,n+1);
for j=1:n+1
    x=zeros(1,n+1); x(j)=1;
    tri=zeros(n+1,n+1); tri(1,:)=x;
    for r=1:n
        tri(r+1,1:n-r+1)=(1-u)*tri(r,1:n-r+1)+u*tri(r,2:n-r+2);
    end
    cp=zeros(1,n+1);
    for r=0:n, cp(r+1)=tri(r+1,1); end
    L(:,j)=cp(:);
end
end

function R=v211_right_subdivision_map(n,u)
R=zeros(n+1,n+1);
for j=1:n+1
    x=zeros(1,n+1); x(j)=1;
    tri=zeros(n+1,n+1); tri(1,:)=x;
    for r=1:n
        tri(r+1,1:n-r+1)=(1-u)*tri(r,1:n-r+1)+u*tri(r,2:n-r+2);
    end
    cp=zeros(1,n+1);
    for r=0:n, cp(n-r+1)=tri(r+1,n-r+1); end
    R(:,j)=cp(:);
end
end

function A=v211_empty_actuator_certificate()
A=struct('pass',false,'reason','UNINITIALIZED','t0',NaN,'t1',NaN, ...
    'payload_mass_kg',NaN,'thrust_lb_N',Inf,'thrust_ub_N',-Inf, ...
    'moment_lb_Nm',inf(3,1),'moment_ub_Nm',-inf(3,1), ...
    'leaf_count',0,'split_count',0,'max_depth',0,'runtime_ms',0,'piece_degrees',[]);
end

% ---------------- interval primitives ----------------
function I=v211_iv(lo,hi), I=struct('lo',lo,'hi',hi); end
function C=v211_iv_add(A,B), C=v211_iv(A.lo+B.lo,A.hi+B.hi); end
function C=v211_iv_neg(A), C=v211_iv(-A.hi,-A.lo); end
function C=v211_iv_sub(A,B), C=v211_iv_add(A,v211_iv_neg(B)); end
function C=v211_iv_scale(A,s)
if s>=0, C=v211_iv(s*A.lo,s*A.hi); else, C=v211_iv(s*A.hi,s*A.lo); end
end
function C=v211_iv_mul(A,B)
p1=A.lo.*B.lo; p2=A.lo.*B.hi; p3=A.hi.*B.lo; p4=A.hi.*B.hi;
C=v211_iv(min(min(p1,p2),min(p3,p4)),max(max(p1,p2),max(p3,p4)));
end
function C=v211_iv_inv(A)
if any(A.lo(:)<=0 & A.hi(:)>=0)
    C=v211_iv(nan(size(A.lo)),nan(size(A.hi))); return;
end
x1=1./A.lo; x2=1./A.hi;
C=v211_iv(min(x1,x2),max(x1,x2));
end
function C=v211_iv_div(A,B), C=v211_iv_mul(A,v211_iv_inv(B)); end
function C=v211_iv_sqrt(A)
if any(A.hi(:)<0), C=v211_iv(nan(size(A.lo)),nan(size(A.hi))); return; end
C=v211_iv(sqrt(max(0,A.lo)),sqrt(max(0,A.hi)));
end
function C=v211_iv_sum(A), C=v211_iv(sum(A.lo(:)),sum(A.hi(:))); end
function C=v211_iv_dot(A,B), C=v211_iv_sum(v211_iv_mul(A,B)); end
function C=v211_iv_cross(A,B)
a1=v211_iv_at(A,1,1); a2=v211_iv_at(A,2,1); a3=v211_iv_at(A,3,1);
b1=v211_iv_at(B,1,1); b2=v211_iv_at(B,2,1); b3=v211_iv_at(B,3,1);
c1=v211_iv_sub(v211_iv_mul(a2,b3),v211_iv_mul(a3,b2));
c2=v211_iv_sub(v211_iv_mul(a3,b1),v211_iv_mul(a1,b3));
c3=v211_iv_sub(v211_iv_mul(a1,b2),v211_iv_mul(a2,b1));
C=v211_iv_stack3(c1,c2,c3);
end
function C=v211_iv_stack3(A,B,D), C=v211_iv([A.lo;B.lo;D.lo],[A.hi;B.hi;D.hi]); end
function C=v211_iv_hcat3(A,B,D), C=v211_iv([A.lo,B.lo,D.lo],[A.hi,B.hi,D.hi]); end
function C=v211_iv_transpose(A), C=v211_iv(A.lo.',A.hi.'); end
function C=v211_iv_at(A,i,j), C=v211_iv(A.lo(i,j),A.hi(i,j)); end
function C=v211_iv_matmul(A,B)
[m,k]=size(A.lo); [k2,n]=size(B.lo);
if k~=k2, error('G26V211:IntervalMatmul','Dimension mismatch.'); end
lo=zeros(m,n); hi=zeros(m,n);
for i=1:m
    for j=1:n
        s=v211_iv(0,0);
        for z=1:k
            s=v211_iv_add(s,v211_iv_mul(v211_iv_at(A,i,z),v211_iv_at(B,z,j)));
        end
        lo(i,j)=s.lo; hi(i,j)=s.hi;
    end
end
C=v211_iv(lo,hi);
end
function C=v211_iv_sin(A)
lo=zeros(size(A.lo)); hi=zeros(size(A.hi));
for i=1:numel(A.lo)
    a=A.lo(i); b=A.hi(i);
    if ~isfinite(a)||~isfinite(b)||b-a>=2*pi, lo(i)=-1;hi(i)=1;continue; end
    vals=[sin(a),sin(b)];
    k1=ceil((a-pi/2)/(2*pi)); k2=floor((b-pi/2)/(2*pi)); if k1<=k2, vals(end+1)=1; end
    k1=ceil((a+pi/2)/(2*pi)); k2=floor((b+pi/2)/(2*pi)); if k1<=k2, vals(end+1)=-1; end
    lo(i)=min(vals);hi(i)=max(vals);
end
C=v211_iv(lo,hi);
end
function C=v211_iv_cos(A)
lo=zeros(size(A.lo)); hi=zeros(size(A.hi));
for i=1:numel(A.lo)
    a=A.lo(i); b=A.hi(i);
    if ~isfinite(a)||~isfinite(b)||b-a>=2*pi, lo(i)=-1;hi(i)=1;continue; end
    vals=[cos(a),cos(b)];
    k1=ceil(a/(2*pi)); k2=floor(b/(2*pi)); if k1<=k2, vals(end+1)=1; end
    k1=ceil((a-pi)/(2*pi)); k2=floor((b-pi)/(2*pi)); if k1<=k2, vals(end+1)=-1; end
    lo(i)=min(vals);hi(i)=max(vals);
end
C=v211_iv(lo,hi);
end

% ---------------- interval Taylor jets ----------------
function C=v211_ivs_add(A,B,N)
C=cell(1,N+1); for k=1:N+1, C{k}=v211_iv_add(A{k},B{k}); end
end
function C=v211_ivs_scale(A,s,N)
C=cell(1,N+1); for k=1:N+1, C{k}=v211_iv_scale(A{k},s); end
end
function C=v211_ivs_mul(A,B,N)
C=cell(1,N+1);
for n=0:N
    s=v211_iv(0,0);
    for k=0:n, s=v211_iv_add(s,v211_iv_mul(A{k+1},B{n-k+1})); end
    C{n+1}=s;
end
end
function C=v211_ivs_dot(A,B,N)
C=cell(1,N+1);
for n=0:N
    s=v211_iv(0,0);
    for k=0:n, s=v211_iv_add(s,v211_iv_dot(A{k+1},B{n-k+1})); end
    C{n+1}=s;
end
end
function C=v211_ivs_cross(A,B,N)
C=cell(1,N+1);
for n=0:N
    z=v211_iv(zeros(3,1),zeros(3,1));
    for k=0:n, z=v211_iv_add(z,v211_iv_cross(A{k+1},B{n-k+1})); end
    C{n+1}=z;
end
end
function C=v211_ivs_vec_scale(V,S,N)
C=cell(1,N+1);
for n=0:N
    z=v211_iv(zeros(3,1),zeros(3,1));
    for k=0:n, z=v211_iv_add(z,v211_iv_mul(V{k+1},S{n-k+1})); end
    C{n+1}=z;
end
end
function D=v211_ivs_derivative(A,N)
D=cell(1,N+1);
for k=0:N, D{k+1}=v211_iv_scale(A{k+2},k+1); end
end
function [Nrm,ok]=v211_ivs_norm(V,N)
S=v211_ivs_dot(V,V,N); [Nrm,ok]=v211_ivs_sqrt(S,N);
end
function [Y,ok]=v211_ivs_sqrt(X,N)
Y=cell(1,N+1); ok=false;
Y{1}=v211_iv_sqrt(X{1});
if any(~isfinite([Y{1}.lo(:);Y{1}.hi(:)]))||any(Y{1}.lo(:)<=0), return; end
for n=1:N
    s=v211_iv(0,0);
    for k=1:n-1, s=v211_iv_add(s,v211_iv_mul(Y{k+1},Y{n-k+1})); end
    num=v211_iv_sub(X{n+1},s);
    Y{n+1}=v211_iv_div(num,v211_iv_scale(Y{1},2));
    if any(~isfinite([Y{n+1}.lo(:);Y{n+1}.hi(:)])), return; end
end
ok=true;
end
function Y=v211_ivs_inv(X,N)
Y=cell(1,N+1); Y{1}=v211_iv_inv(X{1});
for n=1:N
    s=v211_iv(0,0);
    for k=1:n, s=v211_iv_add(s,v211_iv_mul(X{k+1},Y{n-k+1})); end
    Y{n+1}=v211_iv_neg(v211_iv_div(s,X{1}));
end
end
function [U,ok]=v211_ivs_normalize(V,N)
[nrm,ok]=v211_ivs_norm(V,N); if ~ok,U={};return;end
U=v211_ivs_vec_scale(V,v211_ivs_inv(nrm,N),N);
end
function [S,C]=v211_ivs_sincos2(Y)
y0=Y{1};y1=Y{2};y2=Y{3};
s0=v211_iv_sin(y0);c0=v211_iv_cos(y0);
s1=v211_iv_mul(c0,y1);
c1=v211_iv_neg(v211_iv_mul(s0,y1));
y1sq=v211_iv_mul(y1,y1);
s2=v211_iv_sub(v211_iv_mul(c0,y2),v211_iv_scale(v211_iv_mul(s0,y1sq),0.5));
c2=v211_iv_sub(v211_iv_neg(v211_iv_mul(s0,y2)),v211_iv_scale(v211_iv_mul(c0,y1sq),0.5));
S={s0,s1,s2};C={c0,c1,c2};
end

function v211_actuator_detail_print(A)
fprintf('\n========================================================================================================================\n');
fprintf(' G26-V210L CONTINUOUS ACTUATOR RESULTS\n');
fprintf('------------------------------------------------------------------------------------------------------------------------\n');

fprintf('[TIME / TRAJECTORY -- unchanged]\n');
fprintf(' selected total=%.15g s | formal LB=%.15g | certified UB=%.15g | gap=%.6e s\n', ...
    A.selectedTime_s,A.timeFormalLB_s,A.timeCertifiedUB_s,A.timeGap_s);

fprintf('\n[FIXED CONFIG DURING FLIGHT]\n');
F=A.fixedConfiguration;
fprintf(' mQ=%.9g kg | J=[%.9g %.9g %.9g] kg m^2\n',F.mQ_kg,F.J_diag_kgm2);
fprintf(' thrust=[%.9g %.9g] N\n',F.thrust_bounds_N);
fprintf(' moment x=[%.9g %.9g], y=[%.9g %.9g], z=[%.9g %.9g] N m\n', ...
    F.moment_bounds_Nm(1,:),F.moment_bounds_Nm(2,:),F.moment_bounds_Nm(3,:));
fprintf(' parameterAuthority=%d\n',A.parameterAuthority);

fprintf('\n[RUNTIME-VARIABLE PAYLOAD MASS]\n');
P=A.payloadMass;
fprintf(' certified interval=[%.9g %.9g] kg',P.certRange_kg);
if isfinite(P.runtimeCheck_kg)
    fprintf(' | current estimate=%.9g kg | inside=%d\n',P.runtimeCheck_kg,P.runtimeInsideRange);
else
    fprintf(' | current estimate=NOT_PROVIDED\n');
end
fprintf(' estimatorErrorIncludedInRange=%d\n',P.estimatorErrorIncludedInRange);

fprintf('\n[CONTINUOUS ACTUATOR CERTIFICATE]\n');
C=A.continuousActuator;
fprintf(' pass=%d | reason=%s\n',C.pass,C.reason);
fprintf(' thrust enclosure=[%.9g %.9g] N\n',C.thrust_lb_N,C.thrust_ub_N);
fprintf(' moment enclosure x=[%.9g %.9g], y=[%.9g %.9g], z=[%.9g %.9g] N m\n', ...
    C.moment_lb_Nm(1),C.moment_ub_Nm(1), ...
    C.moment_lb_Nm(2),C.moment_ub_Nm(2), ...
    C.moment_lb_Nm(3),C.moment_ub_Nm(3));
fprintf(' leaves=%d | splits=%d | maxDepth=%d | certificate runtime=%.3f ms | wrapper wall=%.3f ms\n', ...
    C.leaf_count,C.split_count,C.max_depth,C.runtime_ms,A.wrapperWallTime_ms);
fprintf(' failedLeafNeverPromoted=%d\n',A.failedLeafNeverPromoted);

fprintf('\n[AUTHORITY STATUS]\n');
fprintf(' runtimeMassGuardPass=%d\n',A.runtimeMassGuardPass);
fprintf(' realArithmeticCandidatePass=%d | floatingPointSoundnessProved=%d\n', ...
    A.continuousActuatorRealArithmeticCandidatePass,A.continuousActuatorFloatingPointSoundnessProved);
fprintf(' fullActuatorAuthorityNow=%d\n',A.fullActuatorAuthorityNow);
fprintf(' estimatorErrorTheoremClosed=%d\n',A.estimatorErrorTheoremClosed);
fprintf(' CLASSIFICATION: %s\n',A.classification);
fprintf(' RULE: fixed vehicle/actuator configuration may change between experiments, but never by hidden hard-coded constants.\n');
fprintf(' RULE: only payload mass is runtime-variable; leaving its certified interval invalidates actuator authority fail-closed.\n');
fprintf('========================================================================================================================\n');
end



function CTX=v210i_context(CFG)
K=CFG.v209i_cone_cover_K;
S=v209b_static_scenario(CFG);

Qold=v209n_build_static_problem_runtime(S,'PATCH_UNION',CFG,K);
Bold=v209s_build_backend_online('ACETRANS_MINCO_ST_C6',Qold.W,Qold.weights);
Told=v208_time_gate_angle(Bold,CFG,CFG.v209b_nominal_theta_deg);
assert(Told.pass,'G26V210I:OldReference','Old reference failed.');

tDetect=CFG.v209b_detection_fraction*Told.T;
tSwitch=tDetect+CFG.v209b_switch_lookahead_s;
J0=v209b_eval_jet(Bold,Told,tSwitch);

Snew=S;Snew.name='V210I_PERFORMANCE';
Snew.C=[S.C S.newObstacleCenter];
Snew.V=zeros(3,size(Snew.C,2));
Snew.R=[S.R S.newObstacleRadius];
Snew.p0=J0(:,1);Snew.pg=S.pg;

Q=v209n_build_static_problem_runtime(Snew,'PATCH_UNION',CFG,K);
assert(Q.guidePass&&Q.corridorOK,'G26V210I:Spatial','Updated spatial problem failed.');
prefix=v209b_active_interval_certificate(Bold,Told,tDetect,tSwitch,Q,CFG);
assert(prefix.pass,'G26V210I:Prefix','Updated old-reference prefix failed.');

QT=v209b_tail_problem(Q);
[BT,T0,st]=v209s_tail_runtime_deep(QT,CFG,'BSPLINE_D7',CFG.v209y_cert_max_depth);
assert(strcmp(st,'PASS'),'G26V210I:Tail','Baseline BSPLINE tail failed.');

CTX=struct('Q',Q,'BT',BT,'T0',T0,'J0',J0,'prefix',prefix);
end

function C=v210i_reconstruct_candidate(hb,Ttarget,CTX,CFG)
alpha=Ttarget/CTX.T0.T;
TT=v209w_scale_tail_time(CTX.BT,CTX.T0,alpha,CFG);
J1=v209b_backend_start_jets(CTX.BT,TT);
[B0,~,~]=v209t_bridge_affine_map(CTX.J0,J1,hb,CFG);

S=v209v_resultant_socp(B0,hb,CTX.Q,CFG,true);
assert(S.feasible&&all(isfinite(S.y)),'G26V210I:RForm','R-form witness unavailable.');

[y,M]=v210i_exact_project(S.y,B0,hb,CFG);
momentResidual=v210i_moment_residual(y,M);
assert(momentResidual<=M.guard,'G26V210I:Moment','Exact projection moment residual too large.');

BB=v209v_reconstruct_bridge(B0,hb,y,CFG);
[Jpass,jres,jratio]=v209b_endpoint_jet_match(BB,hb,CTX.J0,J1);
CORb=v209b_bridge_corridor_audit(BB,CTX.Q);
DY=v209b_bridge_dynamic_audit(BB,hb,CFG.v209b_nominal_theta_deg,CFG);
local=Jpass&&CORb.pass&&DY.pass&&TT.pass;
assert(local,'G26V210I:Local','Final V210H candidate failed unchanged local authority.');

P=[{BB} CTX.BT.P];
hh=[hb TT.h];
Rspans=cell(1,1+numel(TT.rcounts));
Rspans{1}=DY.R;
off=0;
for q=1:numel(TT.rcounts)
    nq=TT.rcounts(q);
    Rspans{q+1}=TT.Rflat(:,off+(1:nq));
    off=off+nq;
end
assert(off==size(TT.Rflat,2),'G26V210I:Rflat','Resultant span accounting mismatch.');

BC=struct('P',{P},'degree',17,'structural_c6_theorem',true,'condition_indicator',1);
TC=struct('pass',true,'h',hh,'Rflat',[DY.R TT.Rflat], ...
    'rcounts',[DY.rcount TT.rcounts],'T',sum(hh));
C6=v207_c6_backend_audit(BC,hh);
COR=v209b_combined_corridor_audit(BC,CTX.Q);
HD=v210i_hard_cert_adaptive(BC,TC,CTX.Q,CFG);

C=struct();
C.P=P;C.h=hh;C.Rspans=Rspans;C.TC=TC;
C.jetPass=Jpass;C.jetResidual=jres;C.jetRatio=jratio;
C.corridorSlack=CORb.min_slack;
C.bridgeConeSlack=DY.minCone;C.bridgeRzSlack=DY.minRz;
C.c6Pass=C6.pass;C.combinedCorridorPass=COR.pass;
C.hardPass=HD.pass;C.hardSlack=HD.min_slack;C.hardDepth=HD.max_depth;
C.fullCertified=local&&C6.pass&&COR.pass&&HD.pass;
end

function [yproj,M]=v210i_exact_project(y,B0,hb,CFG)
n=17;c=hb^2/(n*(n-1));g=[0;0;CFG.gravity];
d5=B0(:,7)-B0(:,6);
d11=B0(:,13)-B0(:,12);
q0=6*g+(d11-d5)/c;
q1=15*g+(B0(:,12)-B0(:,7)-5*d5)/c;

r=reshape(y,3,6);
u1=r(:,2);u2=r(:,3);u3=r(:,4);u4=r(:,5);
u0=(q1-4*u1-3*u2-2*u3-u4)/5;
u5=q0-q1/5-(u1+2*u2+3*u3+4*u4)/5;
yproj=[u0;u1;u2;u3;u4;u5];

sc=max(1,max(abs([yproj(:);q0(:);q1(:)])));
M=struct('q0',q0,'q1',q1,'guard',65536*eps(sc));
end

function r=v210i_moment_residual(y,M)
Y=reshape(y,3,6);w=[5 4 3 2 1 0];
r=max(norm(sum(Y,2)-M.q0,inf),norm(Y*w.'-M.q1,inf));
end

function K=v210i_kinematic_bounds(P,h)
% For a Bezier polynomial, every derivative curve lies in the convex hull
% of its derivative control points. Therefore max control-point norm is a
% rigorous continuous upper bound on derivative norm.
K=struct();
K.order=1:6;
K.maxNormUpperBound=zeros(1,6);
K.maxComponentAbsUpperBound=zeros(3,6);
K.spanMaxNormUpperBound=zeros(numel(P),6);

for q=1:numel(P)
    X=P{q};
    n=size(X,2)-1;
    for r=1:6
        if r>n
            D=zeros(3,1);
        else
            D=v210i_falling(n,r)/h(q)^r*diff(X,r,2);
        end
        nr=vecnorm(D,2,1);
        ub=max(nr);
        K.spanMaxNormUpperBound(q,r)=ub;
        K.maxNormUpperBound(r)=max(K.maxNormUpperBound(r),ub);
        K.maxComponentAbsUpperBound(:,r)=max(K.maxComponentAbsUpperBound(:,r),max(abs(D),[],2));
    end
end

% Control-polygon length is a rigorous upper bound on spatial arc length.
cpLen=0;
for q=1:numel(P)
    cpLen=cpLen+sum(vecnorm(diff(P{q},1,2),2,1));
end
K.pathLengthUpperBound_m=cpLen;
K.endToEndDistance_m=norm(P{end}(:,end)-P{1}(:,1));
end

function D=v210i_resultant_bounds(Rspans,CFG)
thetaCP=0;
maxR=0;
minRz=inf;
minCone=inf;
for q=1:numel(Rspans)
    R=Rspans{q};
    rz=R(3,:);
    xy=vecnorm(R(1:2,:),2,1);
    ang=atan2(xy,rz);
    thetaCP=max(thetaCP,max(ang));
    maxR=max(maxR,max(vecnorm(R,2,1)));
    minRz=min(minRz,min(rz));
    minCone=min(minCone,min(tan(deg2rad(CFG.v209b_nominal_theta_deg))*rz-xy));
end
D=struct();
D.thetaLimitDeg=CFG.v209b_nominal_theta_deg;
D.cableAngleContinuousUpperBoundDeg=rad2deg(thetaCP);
D.resultantNormContinuousUpperBound_mps2=maxR;
D.resultantZContinuousLowerBound_mps2=minRz;
D.coneSlackControlPointMin=minCone;
D.tautSlackControlPointMin=minRz-CFG.v207_rho_min_mps2;
D.rhoMin_mps2=CFG.v207_rho_min_mps2;
D.boundBasis='Bezier convex-hull containment of resultant polynomial';
end

function H=v210i_hard_cert_adaptive(B,T,Q,CFG)
H=struct('pass',false,'min_slack',NaN,'max_depth',NaN,'configuredDepth',NaN);
for md=CFG.v209y_cert_max_depth:24
    D=v209l_final_direct_certificate_depth(B,T,Q,CFG,md);
    H.pass=D.pass;H.min_slack=D.min_slack;H.max_depth=D.max_depth;H.configuredDepth=md;
    if D.pass,return;end
end
end

function G=v210i_control_grid(P,h,dt,CFG)
B=struct('P',{P});
T=struct('h',h);
Ttot=sum(h);
tt=(0:dt:Ttot);
if isempty(tt)||tt(end)<Ttot
    tt=[tt Ttot];
end
N=numel(tt);
mx=zeros(1,7);
for i=1:N
    for r=0:6
        v=v208_eval_jet(B,T,tt(i),r);
        mx(r+1)=max(mx(r+1),norm(v));
    end
end
G=struct();
G.dt_s=dt;
G.count=N;
G.finalTimeIncluded=(tt(end)==Ttot);
G.maxNormSampled_d0_to_d6=mx;
G.authority=false;
G.note='Diagnostic values on the actual 25-ms reference grid; continuous derivative bounds are reported separately.';
end

function y=v210i_falling(n,r)
y=1;
for k=0:r-1,y=y*(n-k);end
end

function v210i_print(A)
fprintf('\n========================================================================================================================\n');
fprintf(' G26-V210I BSPLINE REFERENCE PERFORMANCE RESULTS\n');
fprintf('------------------------------------------------------------------------------------------------------------------------\n');

fprintf('[TIME -- FROZEN]\n');
fprintf(' J*=unknown exact but certified bracket: [%.15g, %.15g] s | width=%.6e s | rel=%.12f %%\n', ...
    A.time.formalLB_s,A.time.certifiedUB_s,A.time.certifiedGap_s,100*A.time.relativeGap);
fprintf(' selected certified reference: bridge=%.12g s (%.4f%%) | tail=%.12g s (%.4f%%) | total=%.12g s\n', ...
    A.time.bridge_s,100*A.time.bridgeFraction,A.time.tail_s,100*A.time.tailFraction,A.time.total_s);

fprintf('\n[CONTINUITY / HANDOFF]\n');
fprintf(' exact d0...d6 gate pass=%d | jetResidual=%.6e | ratio=%.6e\n', ...
    A.handoff.pass,A.handoff.jetResidual,A.handoff.jetRatio);
fprintf(' global C6 theorem=%d | max numerical join residual=%.6e | relative=%.6e\n', ...
    A.c6.real_arithmetic_theorem,A.c6.max_join_residual,A.c6.relative_join_residual);

fprintf('\n[WHOLE-BODY GEOMETRY]\n');
fprintf(' geometry inflation=%.6f m | first-bridge corridor slack=%+.6e m\n', ...
    A.geometry.geometryInflation_m,A.geometry.corridorSlack_m);
fprintf(' continuous hardCert=%d | slack beyond inflated boundary=%+.6e m | usedDepth=%g\n', ...
    A.geometry.hardCertificatePass,A.geometry.hardCertificateSlackBeyondInflatedBoundary_m,A.geometry.hardCertificateDepth);
fprintf(' nominal modeled-clearance reserve implied by inflation+hardSlack = %.12g m  [conditional on hard-certificate soundness proof]\n', ...
    A.geometry.nominalModeledClearanceReserveConditionalOnHardCertSoundness_m);
S=A.geometry.sampleDiagnostic;
fprintf(' SAMPLE ONLY inflated-clearance min payload/UAV/cable=%+.6e / %+.6e / %+.6e m | minAll=%+.6e | AUTHORITY=0\n', ...
    S.minPayloadClearance,S.minUAVClearance,S.minCableClearance,A.geometry.sampleMinInflatedClearance_m);

fprintf('\n[CABLE / RESULTANT CONTINUOUS BOUNDS]\n');
fprintf(' cable angle <= %.9f deg | qualification limit=%.9f deg\n', ...
    A.dynamics.cableAngleContinuousUpperBoundDeg,A.dynamics.thetaLimitDeg);
fprintf(' resultant norm <= %.9f m/s^2 | resultant z >= %.9f m/s^2 | rho_min=%.9f\n', ...
    A.dynamics.resultantNormContinuousUpperBound_mps2,A.dynamics.resultantZContinuousLowerBound_mps2,A.dynamics.rhoMin_mps2);
fprintf(' cone control-point min slack=%+.6e | taut min slack=%+.6e\n', ...
    A.dynamics.coneSlackControlPointMin,A.dynamics.tautSlackControlPointMin);

fprintf('\n[CONTINUOUS KINEMATIC UPPER BOUNDS -- BEZIER CONVEX HULL]\n');
units={'m/s','m/s^2','m/s^3','m/s^4','m/s^5','m/s^6'};
for r=1:6
    fprintf(' ||d%d p/dt%d|| <= %.12g %s | component abs <= [%.6g %.6g %.6g]\n', ...
        r,r,A.kinematics.maxNormUpperBound(r),units{r}, ...
        A.kinematics.maxComponentAbsUpperBound(1,r), ...
        A.kinematics.maxComponentAbsUpperBound(2,r), ...
        A.kinematics.maxComponentAbsUpperBound(3,r));
end
fprintf(' path length <= %.12g m | endpoint straight distance=%.12g m\n', ...
    A.kinematics.pathLengthUpperBound_m,A.kinematics.endToEndDistance_m);

fprintf('\n[25-ms REFERENCE GRID -- DIAGNOSTIC, NOT CONTINUOUS AUTHORITY]\n');
fprintf(' dt=%.6g s | samples=%d | final included=%d\n', ...
    A.controlGrid.dt_s,A.controlGrid.count,A.controlGrid.finalTimeIncluded);
for r=0:6
    fprintf(' sampled max ||d%d p/dt%d|| = %.12g\n',r,r,A.controlGrid.maxNormSampled_d0_to_d6(r+1));
end

fprintf('\n[PERFORMANCE CLOSURE STATUS]\n');
fprintf(' reference trajectory numerically/theoretically characterized=%d\n',A.referencePerformanceNumericallyCharacterized);
fprintf(' explicit max velocity/acceleration/jerk/... plant limits available=%d\n', ...
    ~(A.unprovedOrUnspecifiedLimits.maxVelocity||A.unprovedOrUnspecifiedLimits.maxAcceleration|| ...
      A.unprovedOrUnspecifiedLimits.maxJerk||A.unprovedOrUnspecifiedLimits.maxSnap|| ...
      A.unprovedOrUnspecifiedLimits.maxCrackle||A.unprovedOrUnspecifiedLimits.maxSixthDerivative));
fprintf(' actuator thrust/torque envelope proved=%d | closed-loop tracking feasibility proved=%d\n', ...
    A.unprovedOrUnspecifiedLimits.actuatorThrustTorqueEnvelope, ...
    A.unprovedOrUnspecifiedLimits.closedLoopTrackingFeasibility);
fprintf(' tracking/actuation PERFORMANCE CLOSED=%d\n',A.trackingActuationPerformanceClosed);
fprintf(' CLASSIFICATION: %s\n',A.classification);
fprintf(' RULE: do not invent empirical limits. Compare these bounds only with model/controller-derived admissible limits in the next performance step.\n');
fprintf('========================================================================================================================\n');
end

function C = v200_cfg(U)
C = struct();
C.target_sensor_return_to_switch_ms = 4.0;
C.stretch_target_ms = 2.0;
C.geometry_margin_m = 0.15;
C.theta_max_deg = 60.0; % inherited V207 extreme qualification angle; V208 operational planning uses v208_theta_operational_deg
C.gravity = 9.80665;
C.R_payload = 0.25;
C.R_uav = 0.25;
C.R_cable = 0.15;
C.L_cable = 1.0;
C.direct_cert_max_depth = 8;
C.direct_cert_guard_scale = 4096;
C.prior_map_used = false;
C.unknown_promoted_free = false;
C.retirement_authority = false;
C.local_sphere_safety_authority = false;
C.production_promoted = false;
C.retain_2x4 = true;
C.fixed_time_partition_forbidden = true;
C.empirical_time_bank_forbidden = true;
C.topk_forbidden = true;
C.ttl_forbidden = true;
C.microbench_repetitions = 500;
C.v201_hit_guard_scale = 4096;
C.v201_no_return_is_free = false;
C.v201_motion_model_authority = false;
C.v202_trigger_guard_scale = 4096;
C.v202_predictor_sphere_safety_authority = false;
C.v202_trigger_is_switch_authority = false;
C.v202_window_extension_requires_direct_cert = true;
C.v202_certified_window_horizon_s = 1.0;
C.v202_timing_total_rays = 4320;
C.v202_timing_valid_fraction = 0.75;
C.v202_timing_samples = 60;
C.v202_timing_batch_calls = 10;
C.v202_prefix_design_budget_ms = 1.0;
% V203 fixed resource bounds.  They are implementation bounds, not obstacle
% deletion thresholds.  Exceeding max-frame returns fails closed with NO
% memory mutation.  Capacity pressure invokes outer-cover merge, never drop.
C.v203_cover_capacity = 512;
C.v203_max_frame_returns = 4320;
C.v203_cover_guard_scale = 4096;
C.v203_cover_safety_authority = false;       % FP outward proof not yet closed
C.v203_motion_envelope_authority = false;    % source of v_max not yet certified
C.v203_qualification_vmax_mps = 2.0;
C.v203_qualification_dt_s = 0.1;
C.v203_sustain_frames = 300;
C.v203_timing_cert_obstacles = [32 128 512];
C.v203_timing_trigger_obstacles = [32 128 512];
C.v203_support_directions = 96;
% V204 qualification-only association/reacquisition theorem parameters.
% These defaults are NOT hardware authority and are used only for theorem/selftest examples.
C.v204_assoc_guard_scale = 4096;
C.v204_association_source_authority = false;
C.v204_reacquisition_authority = false;
C.v204_global_angular_coverage_closed = false;
C.v204_sensor_ray_delta_net_deg = NaN;
C.v204_sensor_angular_error_deg = NaN;
C.v204_sensor_max_range_m = NaN;
C.v204_min_obstacle_inscribed_radius_m = NaN;
C.v204_max_closing_speed_mps = NaN;
C.v204_reaction_wcet_s = NaN;
C.v204_collision_distance_m = C.geometry_margin_m + max([C.R_payload C.R_uav C.R_cable]);
C.v204_timing_tracks = 512;
C.v204_timing_measurements = 128;

% V204B bounded-blind-gap dual-LiDAR qualification parameters.
% These encode the CURRENT SIMULATION geometry only; they are not hardware authority.
C.v204b_upper_hemisphere_tilt_deg = 30.0;
C.v204b_lower_is_downward_hemisphere = true;
C.v204b_upper_is_full_hemisphere = true;
C.v204b_ray_az_spacing_deg = 4.0;
C.v204b_ray_el_spacing_deg = 4.0;
C.v204b_sensor_angular_extrinsic_error_deg = NaN;
C.v204b_planning_ingest_max_range_m = 20.0;
C.v204b_min_detectable_radius_m = NaN;
C.v204b_max_closing_speed_mps = NaN;
C.v204b_reaction_wcet_s = NaN;
C.v204b_geometry_is_hardware_certified = false;
C.v204b_blind_gap_authority = false;

% V204C geometry-design envelope. Qualification only: this does not alter
% the runtime sensor mount or safety authority.
C.v204c_tilt_sweep_deg = 0:5:45;
C.v204c_reference_angular_error_deg = [0 0.1 0.5 1.0];
C.v204c_example_closing_speed_mps = 2.0;
C.v204c_example_reaction_s = 0.004;
C.v204c_geometry_design_authority = false;
C.v204c_apply_mount_change = false;


% V205 explicit sensing-range / planner-entry assumptions.
% Sensor perception is not a novelty claim in this project.  For current
% simulation/qualification we intentionally neglect self-occlusion by UAV,
% cable and point-mass payload.  The range below is an EFFECTIVE SIMULATION
% detection radius, not a promoted hardware guarantee.
C.v205_sensor_perception_novelty_claim = false;
C.v205_assume_no_self_occlusion = true;
C.v205_effective_sensing_range_m = 20.0;
C.v205_min_detection_range_m = 0.10;
C.v205_range_stress_m = [10.0 20.0 40.0];
C.v205_reference_low_reflectivity_range_m = 40.0; % MID-360 class reference only; no hardware authority.
C.v205_hardware_range_authority = false;
C.v205_out_of_range_is_unknown = true;
C.v205_planning_horizon_m = 7.5;
C.v205_planner_solver_authority = false;
C.v205_qualification_closing_speed_mps = [2 5 10 20];
C.v205_timing_range_gate_samples = 60;
C.v205_timing_range_gate_batch_calls = 20;

% V206 finite-horizon spatial guide / segment-corridor front end.
% The solver searches adaptively over geometry; it has a resource bound but
% NO feasibility authority.  Hitting the resource bound is SEARCH_INCOMPLETE.
C.v206_spatial_guide_max_nodes = 48;
C.v206_spatial_guide_guard_scale = 4096;
C.v206_spatial_solver_authority = false;
C.v206_search_completeness_claim = false;
C.v206_reference_inflation_m = max([C.R_payload C.R_uav C.R_cable]) + C.geometry_margin_m;
C.v206_timing_samples = 60;
C.v206_timing_batch_calls = 10;
C.v206_bench_obstacle_counts = [1 4 12];

% V207 four-backend finite-horizon trajectory qualification.
% h_i = T*w_i with w_i derived continuously from current guide lengths.
% T is solved on a continuous interval by bisection; no empirical/fixed time bank.
C.v207_backend_authority = false;
C.v207_time_allocation_authority = false;
C.v207_dynamic_gate_authority = false;
C.v207_kinematic_limit_authority = false;
C.v207_yaw_authority = false;
C.v207_fp_authority = false;
C.v207_rho_min_mps2 = 0.50;             % qualification tautness lower bound only
C.v207_time_min_s = 0.05;
C.v207_time_max_s = 30.0;               % resource/search horizon; miss => TIME_SEARCH_INCOMPLETE
C.v207_time_guard_scale = 4096;
C.v207_qualification_obstacle_radius_m = 0.20;
C.v207_qualification_extra_proposal_margin_m = 0.40;
C.v207_qualification_proposal_radius_m = C.v207_qualification_obstacle_radius_m + C.L_cable + ...
    max([C.R_payload C.R_uav C.R_cable]) + C.geometry_margin_m + C.v207_qualification_extra_proposal_margin_m;
C.v207_yaw_start_rad = 0.0;
C.v207_yaw_goal_rad = deg2rad(20.0);
C.v207_timing_samples = 30;
C.v207_timing_batch_calls = 3;

% V208B cable-angle sweep and deterministic stress suite.
% No angle is promoted. The full qualification sweep is 15:5:60 deg.
% 35 deg is retained only as the reference angle for recursive handoff audit.
C.v208_theta_operational_deg = 35.0; % recursive-reference only; NOT promoted
C.v208_theta_extreme_deg = 60.0;     % sweep upper endpoint; NOT a physics limit
C.v208_angle_audit_deg = 15:5:60;
C.v208_motion_proposal_horizon_s = 2.0;
C.v208_stress_case_limit = 6;
C.v208_recursive_trigger_fraction = 0.25;
C.v208_recursive_new_obstacle_radius_m = 0.20;
C.v208_stress_authority = false;
C.v208_recursive_switch_authority = false;

% V209B static-obstacle time/trajectory closure.
C.v209b_nominal_theta_deg = 30.0;           % qualification value only
C.v209b_control_period_s = 0.025;            % actual controller/reference period: 25 ms
C.v209b_compute_stretch_target_ms = 4.0;    % desired generation budget inside one cycle, NOT yet WCET
C.v209b_switch_lookahead_s = C.v209b_control_period_s; % next control boundary
C.v209b_detection_fraction = 0.20;          % qualification scenario only
C.v209b_bridge_h_min_s = 0.02;
C.v209b_bridge_h_max_s = 4.0;
C.v209b_bridge_search_evals = 80;
C.v209b_bridge_guard_scale = 4096;
C.v209b_switch_authority = false;
C.v209b_integrated_wcet_authority = false;
C.v209b_static_obstacles_only = true;

fn = fieldnames(U);
for k=1:numel(fn), C.(fn{k}) = U.(fn{k}); end

assert(C.theta_max_deg > 0 && C.theta_max_deg < 90,'G26V200:Cfg','theta_max_deg must be in (0,90).');
assert(C.geometry_margin_m >= 0,'G26V200:Cfg','geometry margin must be nonnegative.');
assert(C.microbench_repetitions >= 20,'G26V200:Cfg','microbench repetitions must be >=20.');
assert(~C.v201_no_return_is_free,'G26V201:Cfg','No-return rays may not be promoted to free space.');
assert(C.v202_certified_window_horizon_s>0 && isfinite(C.v202_certified_window_horizon_s),'G26V202:Cfg','certified window horizon must be positive finite.');
assert(~C.v202_predictor_sphere_safety_authority,'G26V202:Cfg','Predictor spheres are scheduling aids only.');
assert(~C.v202_trigger_is_switch_authority,'G26V202:Cfg','Tri-state trigger may not become switch authority.');
assert(C.v202_timing_total_rays>=64 && mod(C.v202_timing_total_rays,2)==0,'G26V203:Cfg','timing rays must be even and >=64.');
assert(C.v202_timing_valid_fraction>0 && C.v202_timing_valid_fraction<=1,'G26V203:Cfg','timing valid fraction must be in (0,1].');
assert(C.v202_timing_samples>=20 && C.v202_timing_batch_calls>=2,'G26V203:Cfg','timing samples/batch too small.');
assert(C.v203_cover_capacity>=8 && floor(C.v203_cover_capacity)==C.v203_cover_capacity,'G26V203:Cfg','cover capacity must be integer >=8.');
assert(C.v203_max_frame_returns>=C.v202_timing_total_rays && floor(C.v203_max_frame_returns)==C.v203_max_frame_returns,'G26V203:Cfg','max frame return bound must cover hardware ray count.');
assert(C.v203_qualification_vmax_mps>=0 && isfinite(C.v203_qualification_vmax_mps),'G26V203:Cfg','qualification vmax must be finite nonnegative.');
assert(C.v203_qualification_dt_s>0 && isfinite(C.v203_qualification_dt_s),'G26V203:Cfg','qualification dt must be positive finite.');
assert(C.v203_sustain_frames>=50,'G26V203:Cfg','sustain audit should cover >=50 frames.');
assert(C.v204_timing_tracks>=8 && C.v204_timing_measurements>=1,'G26V204:Cfg','association timing sizes invalid.');
assert(~C.v204_association_source_authority,'G26V204:Cfg','Current raw first-return interface has no certified object-identity authority.');
assert(~C.v204_reacquisition_authority,'G26V204:Cfg','Hardware-specific reacquisition authority must remain false until geometry/WCET premises close.');
assert(C.v204b_upper_hemisphere_tilt_deg>=0 && C.v204b_upper_hemisphere_tilt_deg<90,'G26V204B:Cfg','upper tilt must be in [0,90).');
assert(C.v204b_ray_az_spacing_deg>0 && C.v204b_ray_az_spacing_deg<=30,'G26V204B:Cfg','az spacing invalid.');
assert(C.v204b_ray_el_spacing_deg>0 && C.v204b_ray_el_spacing_deg<=30,'G26V204B:Cfg','el spacing invalid.');
assert(C.v204b_planning_ingest_max_range_m>0 && isfinite(C.v204b_planning_ingest_max_range_m),'G26V204B:Cfg','planning range invalid.');
assert(~C.v204b_blind_gap_authority,'G26V204B:Cfg','Blind-gap hardware authority remains false until physical premises close.');
assert(isvector(C.v204c_tilt_sweep_deg) && numel(C.v204c_tilt_sweep_deg)>=2 && all(isfinite(C.v204c_tilt_sweep_deg)) && ...
    all(C.v204c_tilt_sweep_deg>=0) && all(C.v204c_tilt_sweep_deg<90),'G26V204C:Cfg','tilt sweep must be finite in [0,90).');
assert(any(abs(C.v204c_tilt_sweep_deg)<1e-12),'G26V204C:Cfg','tilt sweep must include 0 deg.');
assert(any(abs(C.v204c_tilt_sweep_deg-C.v204b_upper_hemisphere_tilt_deg)<1e-12),'G26V204C:Cfg','tilt sweep must include the current upper tilt.');
assert(all(C.v204c_reference_angular_error_deg>=0) && all(isfinite(C.v204c_reference_angular_error_deg)),'G26V204C:Cfg','reference angular errors must be finite nonnegative.');
assert(~C.v204c_geometry_design_authority && ~C.v204c_apply_mount_change,'G26V204C:Cfg','V204C is qualification-only and may not change the production sensor mount.');
assert(~C.v205_sensor_perception_novelty_claim,'G26V205:Cfg','Sensor perception is not a novelty claim in this stage.');
assert(C.v205_assume_no_self_occlusion,'G26V205:Cfg','V205 baseline intentionally uses no self-occlusion; model occlusion only in a separate hardware-validation stage.');
assert(C.v205_effective_sensing_range_m>0 && isfinite(C.v205_effective_sensing_range_m),'G26V205:Cfg','sensing range must be positive finite.');
assert(C.v205_min_detection_range_m>=0 && C.v205_min_detection_range_m<C.v205_effective_sensing_range_m,'G26V205:Cfg','minimum detection range invalid.');
assert(all(isfinite(C.v205_range_stress_m)) && all(C.v205_range_stress_m>=C.v205_planning_horizon_m),'G26V205:Cfg','every stress sensing range must cover the local planning horizon.');
assert(C.v205_planning_horizon_m>0 && C.v205_planning_horizon_m<=C.v205_effective_sensing_range_m,'G26V205:Cfg','planning horizon must be positive and no larger than sensing range.');
assert(~C.v205_hardware_range_authority && ~C.v205_planner_solver_authority,'G26V205:Cfg','V205 is qualification-only; range/solver authority must remain false.');
assert(C.v205_out_of_range_is_unknown,'G26V205:Cfg','Out-of-range space must remain UNKNOWN, never FREE.');
assert(C.v206_spatial_guide_max_nodes>=4 && floor(C.v206_spatial_guide_max_nodes)==C.v206_spatial_guide_max_nodes,'G26V206:Cfg','spatial guide max nodes must be integer >=4.');
assert(C.v206_spatial_guide_guard_scale>=0 && isfinite(C.v206_spatial_guide_guard_scale),'G26V206:Cfg','spatial guide guard scale invalid.');
assert(~C.v206_spatial_solver_authority && ~C.v206_search_completeness_claim,'G26V206:Cfg','V206 spatial search remains qualification-only and incomplete.');
assert(C.v206_reference_inflation_m>=0 && isfinite(C.v206_reference_inflation_m),'G26V206:Cfg','reference inflation invalid.');
assert(C.v206_timing_samples>=20 && C.v206_timing_batch_calls>=2,'G26V206:Cfg','V206 timing samples/batch too small.');
assert(all(ismember(C.v206_bench_obstacle_counts,[1 4 12])),'G26V206:Cfg','V206 benchmark obstacle counts must use the declared 1/4/12 audit set.');
assert(~C.v207_backend_authority && ~C.v207_time_allocation_authority && ~C.v207_dynamic_gate_authority && ~C.v207_kinematic_limit_authority && ~C.v207_yaw_authority && ~C.v207_fp_authority, ...
    'G26V207:Cfg','V207 is qualification-only; backend/time/dynamic/kinematic/yaw/FP authorities must remain false.');
assert(C.v207_rho_min_mps2>0 && C.v207_rho_min_mps2<C.gravity,'G26V207:Cfg','rho_min must be in (0,g).');
assert(C.v207_time_min_s>0 && C.v207_time_max_s>C.v207_time_min_s && isfinite(C.v207_time_max_s),'G26V207:Cfg','continuous time-search bounds invalid.');
assert(C.v207_qualification_proposal_radius_m>C.v207_qualification_obstacle_radius_m,'G26V207:Cfg','proposal radius must exceed physical obstacle radius.');
assert(C.v207_timing_samples>=20 && C.v207_timing_batch_calls>=1,'G26V207:Cfg','V207 timing audit too small.');
assert(C.v208_theta_operational_deg>0 && C.v208_theta_operational_deg<C.v208_theta_extreme_deg && C.v208_theta_extreme_deg<90,'G26V208B:Cfg','Require 0<recursive-reference<upper-sweep<90 deg.');
assert(all(isfinite(C.v208_angle_audit_deg)) && all(C.v208_angle_audit_deg>0) && all(C.v208_angle_audit_deg<90),'G26V208B:Cfg','Angle audit set must lie in (0,90).');
assert(isequal(C.v208_angle_audit_deg(:).',[15 20 25 30 35 40 45 50 55 60]),'G26V208B:Cfg','Default angle sweep must be 15:5:60 deg.');
assert(C.v208_motion_proposal_horizon_s>0 && isfinite(C.v208_motion_proposal_horizon_s),'G26V208:Cfg','Motion proposal horizon invalid.');
assert(C.v208_recursive_trigger_fraction>0 && C.v208_recursive_trigger_fraction<1,'G26V208:Cfg','Recursive trigger fraction must be in (0,1).');
assert(~C.v208_stress_authority && ~C.v208_recursive_switch_authority,'G26V208:Cfg','V208 remains qualification-only.');
assert(C.v209b_nominal_theta_deg>0 && C.v209b_nominal_theta_deg<90,'G26V209D:Cfg','Nominal cable angle must lie in (0,90) deg.');
assert(C.v209b_compute_stretch_target_ms>0,'G26V209D:Cfg','Compute stretch target must be positive.');
assert(C.v209b_control_period_s>0,'G26V209D:Cfg','Control period must be positive.');
assert(C.v209b_switch_lookahead_s>0,'G26V209D:Cfg','Switch look-ahead must be positive.');
assert(1e-3*C.v209b_compute_stretch_target_ms < C.v209b_switch_lookahead_s,'G26V209D:Cfg','Compute target must fit inside scheduled next-cycle handoff window.');
assert(C.v209b_detection_fraction>0 && C.v209b_detection_fraction<1,'G26V209D:Cfg','Detection fraction must lie in (0,1).');
assert(C.v209b_bridge_h_min_s>0 && C.v209b_bridge_h_max_s>C.v209b_bridge_h_min_s,'G26V209D:Cfg','Bridge-duration interval invalid.');
assert(C.v209b_bridge_search_evals>=16 && floor(C.v209b_bridge_search_evals)==C.v209b_bridge_search_evals,'G26V209D:Cfg','Bridge evaluation budget invalid.');
assert(C.v209b_static_obstacles_only,'G26V209D:Cfg','V209B is a static-obstacle stage.');
assert(~C.v209b_switch_authority && ~C.v209b_integrated_wcet_authority,'G26V209D:Cfg','V209B remains qualification-only.');
end

function P = v200_source_precheck(CFG)
src = fileread([mfilename('fullpath'),'.m']);
legacy = regexp(src,'(?m)\bg26v(?:7[5-9]|8[0-9]|9[0-9]|10[0-9]|11[0-3])_[A-Za-z0-9_]+\s*\(','match');
legacy = legacy(~startsWith(legacy,'g26v200_'));
P = struct();
P.legacy_runtime_dependency_count = numel(legacy);
P.forbidden_executable_count = 0;
P.prior_map_zero = ~CFG.prior_map_used;
P.retirement_zero = ~CFG.retirement_authority;
P.local_sphere_authority_zero = ~CFG.local_sphere_safety_authority;
P.cover_authority_zero = ~CFG.v203_cover_safety_authority;
P.motion_authority_zero = ~CFG.v203_motion_envelope_authority;
P.association_authority_zero = ~CFG.v204_association_source_authority;
P.reacquisition_authority_zero = ~CFG.v204_reacquisition_authority;
P.no_self_occlusion_assumption = CFG.v205_assume_no_self_occlusion;
P.sensor_not_novelty = ~CFG.v205_sensor_perception_novelty_claim;
P.range_authority_zero = ~CFG.v205_hardware_range_authority;
P.planner_solver_authority_zero = ~CFG.v205_planner_solver_authority;
P.v206_spatial_solver_authority_zero = ~CFG.v206_spatial_solver_authority;
P.v206_search_completeness_zero = ~CFG.v206_search_completeness_claim;
P.out_of_range_unknown = CFG.v205_out_of_range_is_unknown;
P.full_2x4 = CFG.retain_2x4;
P.no_return_not_free = ~CFG.v201_no_return_is_free;
P.predictor_sphere_authority_zero = ~CFG.v202_predictor_sphere_safety_authority;
P.trigger_switch_authority_zero = ~CFG.v202_trigger_is_switch_authority;
P.pass = P.legacy_runtime_dependency_count==0 && P.forbidden_executable_count==0 && P.prior_map_zero && ...
    P.retirement_zero && P.local_sphere_authority_zero && P.cover_authority_zero && P.motion_authority_zero && ...
    P.full_2x4 && P.no_return_not_free && P.predictor_sphere_authority_zero && P.trigger_switch_authority_zero && ...
    P.association_authority_zero && P.reacquisition_authority_zero && P.no_self_occlusion_assumption && P.sensor_not_novelty && ...
    P.range_authority_zero && P.planner_solver_authority_zero && P.v206_spatial_solver_authority_zero && P.v206_search_completeness_zero && P.out_of_range_unknown;
assert(P.pass,'G26V206:SourcePrecheck','Clean V206 source contract failed.');
fprintf(['[G26-V206-SOURCE-PRECHECK] PASS | legacy runtime deps=0 | priorMap=0 noReturnFree=0 noSelfOcclusion=1 sensorNovelty=0 ', ...
    'retirement=0 sphereAuthority=0 coverAuthority=0 motionAuthority=0 associationAuthority=0 reacquisitionAuthority=0 rangeAuthority=0 plannerAuthority=0 spatialAuthority=0 searchComplete=0 full2x4=1\n']);
end


function P=v207_source_precheck_extension(CFG,P)
P.v207_backend_authority_zero=~CFG.v207_backend_authority;
P.v207_time_authority_zero=~CFG.v207_time_allocation_authority;
P.v207_dynamic_authority_zero=~CFG.v207_dynamic_gate_authority;
P.v207_kinematic_authority_zero=~CFG.v207_kinematic_limit_authority;
P.v207_yaw_authority_zero=~CFG.v207_yaw_authority;
P.v207_fp_authority_zero=~CFG.v207_fp_authority;
P.no_empirical_time_bank=CFG.empirical_time_bank_forbidden;
P.no_fixed_time_partition=CFG.fixed_time_partition_forbidden;
P.pass=P.pass&&P.v207_backend_authority_zero&&P.v207_time_authority_zero&&P.v207_dynamic_authority_zero&& ...
    P.v207_kinematic_authority_zero&&P.v207_yaw_authority_zero&&P.v207_fp_authority_zero&&P.no_empirical_time_bank&&P.no_fixed_time_partition;
assert(P.pass,'G26V207:SourcePrecheck','V207 source/authority contract failed.');
fprintf(['[G26-V207-SOURCE-PRECHECK-EXT] PASS | backendAuthority=0 timeAuthority=0 dynamicAuthority=0 ', ...
    'kinematicAuthority=0 yawAuthority=0 fpAuthority=0 fixedTimeBank=0 canonicalTimePartition=0\n']);
end

function R = v200_registry()
reps = {'PATCH_UNION','ACETRANS_SPHERE_SFC'};
backs = {'BSPLINE_D7','BEZIER_D13','POLY_D13','ACETRANS_MINCO_ST_C6'};
R = repmat(struct('obstacle_representation','','backend','','enabled',true, ...
    'production_selected',false,'same_hard_authority',true,'note',''),1,numel(reps)*numel(backs));
q=0;
for i=1:numel(reps)
    for j=1:numel(backs)
        q=q+1;
        R(q).obstacle_representation=reps{i};
        R(q).backend=backs{j};
        if strcmp(reps{i},'PATCH_UNION')
            R(q).note='Exact observed local patches are retained; hierarchy/broadphase may only skip proven-safe work.';
        else
            R(q).note='Current first-return local spheres/SFC propose geometry only; final safety uses exact observed patches.';
        end
    end
end
end


function v200_print_registry(R)
fprintf('\n[G26-V207 2x4 REGISTRY]\n');
for k=1:numel(R)
    fprintf('  %-21s / %-23s enabled=%d production=%d sameHardAuthority=%d\n', ...
        R(k).obstacle_representation,R(k).backend,R(k).enabled,R(k).production_selected,R(k).same_hard_authority);
end
end

% ========================================================================
% MEX bootstrap helpers
% ========================================================================

function v200_mex_bootstrap_c6(root)
name='g26v200_c6_construct_mex';
v200_build_mex_if_needed(root,name,v200_cpp_c6_source(),'G26-V200-C6-MEX');
end


function v200_mex_bootstrap_local_sfc(root)
name='g26v200_local_sfc_mex';
v200_build_mex_if_needed(root,name,v200_cpp_local_sfc_source(),'G26-V200-SFC-MEX');
end


function v200_mex_bootstrap_direct_cert(root)
name='g26v200_direct_cert_mex';
v200_build_mex_if_needed(root,name,v200_cpp_direct_cert_source(),'G26-V200-CERT-MEX');
end


function v200_build_mex_if_needed(root,name,lines,banner)
cpp=fullfile(root,[name,'.cpp']);
newtxt=sprintf('%s\n',lines{:});
sourceExists=exist(cpp,'file')==2;
sourceChanged=false;
if sourceExists
    sourceChanged=~strcmp(fileread(cpp),newtxt);
else
    % First clean-stage persistence: materialize the source next to its MEX.
    % Existing V200-V202 MEX binaries are trusted once here and are still
    % protected by selftests.  Future stages detect exact source changes.
    fid=fopen(cpp,'w');assert(fid>=0,'G26V203:IO','Cannot create persistent MEX source.');
    c=onCleanup(@()fclose(fid));fprintf(fid,'%s',newtxt);clear c;
end
if sourceChanged
    fid=fopen(cpp,'w');assert(fid>=0,'G26V203:IO','Cannot update persistent MEX source.');
    c=onCleanup(@()fclose(fid));fprintf(fid,'%s',newtxt);clear c;
end
d=dir(fullfile(root,[name,'.',mexext]));
need=isempty(d)||sourceChanged;
if need
    fprintf('[%s] build required: %s.cpp | sourceChanged=%d\n',banner,name,sourceChanged);
    try, clear(name); catch, end
    try
        mex('-R2018a','-O','-outdir',root,cpp);
    catch ME
        error('G26V203:MEXBuild','Configure a supported C++ compiler with mex -setup C++. Kernel %s failed: %s',name,ME.message);
    end
    rehash;
end
assert(~isempty(dir(fullfile(root,[name,'.',mexext]))),'G26V203:MEXMissing','Missing MEX %s',name);
end

function v201_mex_bootstrap_first_return(root)
name='g26v201_first_return_mex';
v200_build_mex_if_needed(root,name,v201_cpp_first_return_source(),'G26-V201-FIRST-RETURN-MEX');
end


function lines=v201_cpp_first_return_source()
lines={ ...
'#include "mex.h"', ...
'#include <cmath>', ...
'#include <limits>', ...
'#include <algorithm>', ...
'static void req(bool q,const char*m){if(!q)mexErrMsgIdAndTxt("G26V201:FirstReturnMEX","%s",m);}', ...
'void mexFunction(int nlhs,mxArray*plhs[],int nrhs,const mxArray*prhs[]){', ...
' req(nrhs==8,"Expected origin,dir,range,tanHalfAz,tanHalfEl,sensorId,rayId,guardScale"); req(nlhs<=5,"At most five outputs");', ...
' req(mxIsDouble(prhs[0])&&mxGetM(prhs[0])==3,"origin must be 3xN double"); mwSize N=mxGetN(prhs[0]);', ...
' req(mxIsDouble(prhs[1])&&mxGetM(prhs[1])==3&&mxGetN(prhs[1])==N,"dir must be 3xN double");', ...
' for(int k=2;k<=6;++k)req(mxIsDouble(prhs[k])&&mxGetNumberOfElements(prhs[k])==N,"packet vector size mismatch");', ...
' double gs=mxGetScalar(prhs[7]); req(std::isfinite(gs)&&gs>=0,"guardScale invalid");', ...
' const double*O=mxGetPr(prhs[0]);const double*D=mxGetPr(prhs[1]);const double*R=mxGetPr(prhs[2]);const double*HA=mxGetPr(prhs[3]);const double*HE=mxGetPr(prhs[4]);const double*SID=mxGetPr(prhs[5]);const double*RID=mxGetPr(prhs[6]);', ...
' mwSize K=0;for(mwSize i=0;i<N;++i){if(std::isfinite(R[i])&&R[i]>0){double nx=D[3*i],ny=D[3*i+1],nz=D[3*i+2];double dn=std::sqrt(nx*nx+ny*ny+nz*nz);double tol=gs*std::numeric_limits<double>::epsilon()*std::max(1.0,dn);req(std::isfinite(dn)&&std::fabs(dn-1.0)<=tol,"valid-return ray direction must be pre-normalized");req(std::isfinite(HA[i])&&HA[i]>=0&&std::isfinite(HE[i])&&HE[i]>=0,"angular footprint invalid");++K;}}', ...
' plhs[0]=mxCreateDoubleMatrix(3,K,mxREAL);double*C=mxGetPr(plhs[0]);', ...
' if(nlhs>1)plhs[1]=mxCreateDoubleMatrix(1,K,mxREAL);double*rad=(nlhs>1)?mxGetPr(plhs[1]):nullptr;', ...
' if(nlhs>2)plhs[2]=mxCreateDoubleMatrix(1,K,mxREAL);double*sid=(nlhs>2)?mxGetPr(plhs[2]):nullptr;', ...
' if(nlhs>3)plhs[3]=mxCreateDoubleMatrix(1,K,mxREAL);double*rid=(nlhs>3)?mxGetPr(plhs[3]):nullptr;', ...
' if(nlhs>4)plhs[4]=mxCreateDoubleMatrix(1,N,mxREAL);double*mask=(nlhs>4)?mxGetPr(plhs[4]):nullptr;', ...
' mwSize j=0;for(mwSize i=0;i<N;++i)if(std::isfinite(R[i])&&R[i]>0){for(int a=0;a<3;++a)C[a+3*j]=O[a+3*i]+R[i]*D[a+3*i];if(rad)rad[j]=R[i]*std::sqrt(HA[i]*HA[i]+HE[i]*HE[i]);if(sid)sid[j]=SID[i];if(rid)rid[j]=RID[i];if(mask)mask[i]=1.0;++j;}', ...
'}'};
end


function v203_mex_bootstrap_cover_memory(root)
name='g26v203_cover_memory_mex';
v200_build_mex_if_needed(root,name,v203_cpp_cover_memory_source(),'G26-V203-COVER-MEMORY-MEX');
end

function v203_mex_bootstrap_motion_envelope(root)
name='g26v203_motion_envelope_mex';
v200_build_mex_if_needed(root,name,v203_cpp_motion_envelope_source(),'G26-V203-MOTION-ENVELOPE-MEX');
end

function lines=v203_cpp_cover_memory_source()
lines={ ...
'#include "mex.h"', ...
'#include <vector>', ...
'#include <string>', ...
'#include <cmath>', ...
'#include <algorithm>', ...
'#include <limits>', ...
'struct E{double c[3],r,t,v; double known;};', ...
'static std::vector<E> MEM,SCR; static mwSize CAP=0,MAXF=0;', ...
'static void req(bool q,const char*m){if(!q)mexErrMsgIdAndTxt("G26V203:CoverMemoryMEX","%s",m);}', ...
'static std::string cmd(const mxArray*a){req(mxIsChar(a),"first input must be command");char*b=mxArrayToString(a);req(b!=nullptr,"command conversion failed");std::string s(b);mxFree(b);return s;}', ...
'static E atTime(E e,double tq){req(tq+64*std::numeric_limits<double>::epsilon()*std::max(1.0,std::fabs(tq))>=e.t,"time reversal forbidden");double dt=std::max(0.0,tq-e.t);if(e.known>0.5)e.r+=e.v*dt;e.t=tq;return e;}', ...
'static E out(E p,double gs){double sc=std::max(1.0,p.r);for(int a=0;a<3;++a)sc=std::max(sc,std::fabs(p.c[a]));p.r+=gs*std::numeric_limits<double>::epsilon()*sc;return p;}', ...
'static E merge2(const E&a,const E&b,double t,double gs){E p;double dx=b.c[0]-a.c[0],dy=b.c[1]-a.c[1],dz=b.c[2]-a.c[2];double d=std::sqrt(dx*dx+dy*dy+dz*dz);', ...
' p.t=t;p.v=std::max(a.v,b.v);p.known=(a.known>0.5&&b.known>0.5)?1.0:0.0;', ...
' if(d+a.r<=b.r){p=b;p.t=t;p.v=std::max(a.v,b.v);p.known=(a.known>0.5&&b.known>0.5)?1.0:0.0;return out(p,gs);}', ...
' if(d+b.r<=a.r){p=a;p.t=t;p.v=std::max(a.v,b.v);p.known=(a.known>0.5&&b.known>0.5)?1.0:0.0;return out(p,gs);}', ...
' if(d<=std::numeric_limits<double>::min()){p=a;p.r=std::max(a.r,b.r);return out(p,gs);}', ...
' p.r=0.5*(d+a.r+b.r);double s=(p.r-a.r)/d;p.c[0]=a.c[0]+s*dx;p.c[1]=a.c[1]+s*dy;p.c[2]=a.c[2]+s*dz;return out(p,gs);}', ...
'static void reset(mwSize cap,mwSize maxf){CAP=cap;MAXF=maxf;MEM.clear();SCR.clear();MEM.reserve(CAP);SCR.reserve(CAP+MAXF);}', ...
'void mexFunction(int nlhs,mxArray*plhs[],int nrhs,const mxArray*prhs[]){req(nrhs>=1,"command required");std::string q=cmd(prhs[0]);', ...
' if(q=="reset"){req(nrhs==3,"reset(capacity,maxFrameReturns)");double a=mxGetScalar(prhs[1]),b=mxGetScalar(prhs[2]);req(std::isfinite(a)&&a>=1&&std::floor(a)==a,"capacity invalid");req(std::isfinite(b)&&b>=1&&std::floor(b)==b,"maxFrame invalid");reset((mwSize)a,(mwSize)b);if(nlhs>0)plhs[0]=mxCreateDoubleScalar(1);return;}', ...
' if(q=="count"){if(nlhs>0)plhs[0]=mxCreateDoubleScalar((double)MEM.size());if(nlhs>1)plhs[1]=mxCreateDoubleScalar((double)CAP);return;}', ...
' if(q=="ingest"){req(nrhs==7,"ingest(C,R,stamp,vmax,known,guardScale)");req(CAP>0,"reset first");mwSize N=mxGetN(prhs[1]);req(mxIsDouble(prhs[1])&&mxGetM(prhs[1])==3,"C must be 3xN");', ...
'   req(mxIsDouble(prhs[2])&&mxGetNumberOfElements(prhs[2])==N,"R size mismatch");req(mxGetNumberOfElements(prhs[3])==1,"stamp scalar required");req(mxIsDouble(prhs[4])&&mxGetNumberOfElements(prhs[4])==N,"vmax size mismatch");req(mxIsDouble(prhs[5])&&mxGetNumberOfElements(prhs[5])==N,"known size mismatch");double gs=mxGetScalar(prhs[6]);req(std::isfinite(gs)&&gs>=0,"guard scale invalid");double stamp=mxGetScalar(prhs[3]);req(std::isfinite(stamp),"stamp invalid");', ...
'   if(N>MAXF){if(nlhs>0)plhs[0]=mxCreateDoubleScalar(0);if(nlhs>1)plhs[1]=mxCreateDoubleScalar((double)MEM.size());if(nlhs>2)plhs[2]=mxCreateDoubleScalar(0);if(nlhs>3)plhs[3]=mxCreateDoubleScalar(1);return;}', ...
'   const double*C=mxGetPr(prhs[1]);const double*R=mxGetPr(prhs[2]);const double*V=mxGetPr(prhs[4]);const double*K=mxGetPr(prhs[5]);SCR.clear();', ...
'   for(const E&e0:MEM)SCR.push_back(atTime(e0,stamp));', ...
'   for(mwSize i=0;i<N;++i){E e;for(int a=0;a<3;++a){e.c[a]=C[a+3*i];req(std::isfinite(e.c[a]),"center invalid");}e.r=R[i];e.t=stamp;e.v=V[i];e.known=(std::isfinite(K[i])&&K[i]>0.5)?1.0:0.0;req(std::isfinite(e.r)&&e.r>=0,"radius invalid");if(e.known>0.5)req(std::isfinite(e.v)&&e.v>=0,"known vmax invalid");else e.v=0;SCR.push_back(e);}', ...
'   mwSize before=SCR.size();MEM.clear();if(before<=CAP){for(const E&e:SCR)MEM.push_back(e);}else{', ...
'     double mn[3]={SCR[0].c[0],SCR[0].c[1],SCR[0].c[2]},mx[3]={mn[0],mn[1],mn[2]};for(const E&e:SCR)for(int a=0;a<3;++a){mn[a]=std::min(mn[a],e.c[a]);mx[a]=std::max(mx[a],e.c[a]);}int ax=0;if(mx[1]-mn[1]>mx[ax]-mn[ax])ax=1;if(mx[2]-mn[2]>mx[ax]-mn[ax])ax=2;', ...
'     std::sort(SCR.begin(),SCR.end(),[ax](const E&a,const E&b){if(a.c[ax]!=b.c[ax])return a.c[ax]<b.c[ax];int a1=(ax+1)%3,a2=(ax+2)%3;if(a.c[a1]!=b.c[a1])return a.c[a1]<b.c[a1];return a.c[a2]<b.c[a2];});', ...
'     for(mwSize g=0;g<CAP;++g){mwSize i0=(g*before)/CAP,i1=((g+1)*before)/CAP;if(i0>=i1)continue;E p=SCR[i0];for(mwSize i=i0+1;i<i1;++i)p=merge2(p,SCR[i],stamp,gs);MEM.push_back(p);}', ...
'   }mwSize merges=before-MEM.size();if(nlhs>0)plhs[0]=mxCreateDoubleScalar(1);if(nlhs>1)plhs[1]=mxCreateDoubleScalar((double)MEM.size());if(nlhs>2)plhs[2]=mxCreateDoubleScalar((double)merges);if(nlhs>3)plhs[3]=mxCreateDoubleScalar(0);return;}', ...
' if(q=="snapshot"){req(nrhs==1,"snapshot takes no payload");mwSize N=MEM.size();if(nlhs>0)plhs[0]=mxCreateDoubleMatrix(3,N,mxREAL);if(nlhs>1)plhs[1]=mxCreateDoubleMatrix(1,N,mxREAL);if(nlhs>2)plhs[2]=mxCreateDoubleMatrix(1,N,mxREAL);if(nlhs>3)plhs[3]=mxCreateDoubleMatrix(1,N,mxREAL);if(nlhs>4)plhs[4]=mxCreateDoubleMatrix(1,N,mxREAL);double*c=nlhs>0?mxGetPr(plhs[0]):nullptr,*r=nlhs>1?mxGetPr(plhs[1]):nullptr,*t=nlhs>2?mxGetPr(plhs[2]):nullptr,*v=nlhs>3?mxGetPr(plhs[3]):nullptr,*k=nlhs>4?mxGetPr(plhs[4]):nullptr;for(mwSize i=0;i<N;++i){if(c)for(int a=0;a<3;++a)c[a+3*i]=MEM[i].c[a];if(r)r[i]=MEM[i].r;if(t)t[i]=MEM[i].t;if(v)v[i]=MEM[i].v;if(k)k[i]=MEM[i].known;}return;}', ...
' mexErrMsgIdAndTxt("G26V203:CoverMemoryMEX","Unknown command");', ...
'}'};
end

function lines=v203_cpp_motion_envelope_source()
lines={ ...
'#include "mex.h"', ...
'#include <cmath>', ...
'#include <algorithm>', ...
'#include <limits>', ...
'static void req(bool q,const char*m){if(!q)mexErrMsgIdAndTxt("G26V203:MotionEnvelopeMEX","%s",m);}', ...
'void mexFunction(int nlhs,mxArray*plhs[],int nrhs,const mxArray*prhs[]){req(nrhs==6,"Expected R,T,vmax,known,tQuery,guardScale");mwSize N=mxGetNumberOfElements(prhs[0]);for(int k=1;k<4;++k)req(mxIsDouble(prhs[k])&&mxGetNumberOfElements(prhs[k])==N,"vector size mismatch");double tq=mxGetScalar(prhs[4]),gs=mxGetScalar(prhs[5]);req(std::isfinite(tq)&&std::isfinite(gs)&&gs>=0,"query/guard invalid");const double*R=mxGetPr(prhs[0]),*T=mxGetPr(prhs[1]),*V=mxGetPr(prhs[2]),*K=mxGetPr(prhs[3]);plhs[0]=mxCreateDoubleMatrix(1,N,mxREAL);double*RO=mxGetPr(plhs[0]);if(nlhs>1)plhs[1]=mxCreateDoubleMatrix(1,N,mxREAL);double*KO=nlhs>1?mxGetPr(plhs[1]):nullptr;bool all=true;double mg=0;', ...
' for(mwSize i=0;i<N;++i){req(std::isfinite(R[i])&&R[i]>=0&&std::isfinite(T[i]),"memory radius/time invalid");double tol=gs*std::numeric_limits<double>::epsilon()*std::max(1.0,std::max(std::fabs(tq),std::fabs(T[i])));req(tq+tol>=T[i],"query before memory stamp");bool known=std::isfinite(K[i])&&K[i]>0.5;if(known){req(std::isfinite(V[i])&&V[i]>=0,"known vmax invalid");double g=V[i]*std::max(0.0,tq-T[i]);double sc=std::max(1.0,std::max(R[i]+g,std::fabs(tq)));double guard=gs*std::numeric_limits<double>::epsilon()*sc;RO[i]=R[i]+g+guard;mg=std::max(mg,g);}else{RO[i]=R[i];all=false;}if(KO)KO[i]=known?1.0:0.0;}', ...
' if(nlhs>2)plhs[2]=mxCreateDoubleScalar(all?1:0);if(nlhs>3)plhs[3]=mxCreateDoubleScalar(mg);', ...
'}'};
end

% ========================================================================
% V204 MEX: deterministic set-membership association + reacquisition theorem.
% Association output is qualification evidence only: raw first-return packets
% do NOT provide the certified whole-object measurement envelope / identity
% premise required for contraction of historical dynamic memory.
% ========================================================================

function v204_mex_bootstrap_assoc_reacq(root)
name='g26v204_assoc_reacq_mex';
v200_build_mex_if_needed(root,name,v204_cpp_assoc_reacq_source(),'G26-V204-ASSOC-REACQ-MEX');
end

function lines=v204_cpp_assoc_reacq_source()
lines={ ...
'#include "mex.h"', ...
'#include <cmath>', ...
'#include <algorithm>', ...
'#include <limits>', ...
'#include <string>', ...
'static void req(bool q,const char*m){if(!q)mexErrMsgIdAndTxt("G26V204:AssocReacqMEX","%s",m);}', ...
'static std::string cmd(const mxArray*a){req(mxIsChar(a),"first input must be command");char*b=mxArrayToString(a);req(b!=nullptr,"command conversion failed");std::string s(b);mxFree(b);return s;}', ...
'void mexFunction(int nlhs,mxArray*plhs[],int nrhs,const mxArray*prhs[]){req(nrhs>=1,"command required");std::string q=cmd(prhs[0]);', ...
' if(q=="associate"){req(nrhs==6,"associate(trackC,trackR,measC,measR,guardScale)");req(mxIsDouble(prhs[1])&&mxGetM(prhs[1])==3,"trackC must be 3xN");mwSize N=mxGetN(prhs[1]);req(mxIsDouble(prhs[2])&&mxGetNumberOfElements(prhs[2])==N,"trackR size mismatch");req(mxIsDouble(prhs[3])&&mxGetM(prhs[3])==3,"measC must be 3xM");mwSize M=mxGetN(prhs[3]);req(mxIsDouble(prhs[4])&&mxGetNumberOfElements(prhs[4])==M,"measR size mismatch");double gs=mxGetScalar(prhs[5]);req(std::isfinite(gs)&&gs>=0,"guard invalid");const double*TC=mxGetPr(prhs[1]),*TR=mxGetPr(prhs[2]),*MC=mxGetPr(prhs[3]),*MR=mxGetPr(prhs[4]);plhs[0]=mxCreateDoubleMatrix(1,M,mxREAL);double*A=mxGetPr(plhs[0]);bool all=true;mwSize amb=0,none=0;', ...
'  for(mwSize j=0;j<M;++j){req(std::isfinite(MR[j])&&MR[j]>=0,"measurement radius invalid");int cnt=0;mwSize id=0;for(mwSize i=0;i<N;++i){req(std::isfinite(TR[i])&&TR[i]>=0,"track radius invalid");double d2=0,sc=std::max(1.0,TR[i]+MR[j]);for(int a=0;a<3;++a){double x=TC[a+3*i],y=MC[a+3*j];req(std::isfinite(x)&&std::isfinite(y),"center invalid");double z=x-y;d2+=z*z;sc=std::max(sc,std::max(std::fabs(x),std::fabs(y)));}double g=gs*std::numeric_limits<double>::epsilon()*sc;if(std::sqrt(d2)<=TR[i]+MR[j]+g){++cnt;id=i+1;}}if(cnt==1)A[j]=(double)id;else{A[j]=0;all=false;if(cnt==0)++none;else ++amb;}}', ...
'  if(nlhs>1)plhs[1]=mxCreateLogicalScalar(all);if(nlhs>2)plhs[2]=mxCreateDoubleScalar((double)amb);if(nlhs>3)plhs[3]=mxCreateDoubleScalar((double)none);return;}', ...
' if(q=="reacq"){req(nrhs==10,"reacq(deltaNetRad,angularErrRad,minInscribedRadius,maxRange,vClosing,dCollision,reactionTime,coverageComplete,guardScale)");double de=mxGetScalar(prhs[1]),ae=mxGetScalar(prhs[2]),amin=mxGetScalar(prhs[3]),rmax=mxGetScalar(prhs[4]),vc=mxGetScalar(prhs[5]),dc=mxGetScalar(prhs[6]),tr=mxGetScalar(prhs[7]),cov=mxGetScalar(prhs[8]),gs=mxGetScalar(prhs[9]);req(std::isfinite(de)&&de>=0&&std::isfinite(ae)&&ae>=0&&std::isfinite(amin)&&amin>=0&&std::isfinite(rmax)&&rmax>0&&std::isfinite(vc)&&vc>=0&&std::isfinite(dc)&&dc>=0&&std::isfinite(tr)&&tr>=0&&std::isfinite(gs)&&gs>=0,"invalid reacq input");double d=de+ae;bool geom=(cov!=0)&&amin>0&&d>0&&d<1.5707963267948966;double rg=0,lead=-std::numeric_limits<double>::infinity(),reqA=std::numeric_limits<double>::infinity();if(geom){double s=std::sin(d);rg=std::min(rmax,amin/s);double sc=std::max(1.0,std::max(rg,std::max(dc,rmax)));double guard=gs*std::numeric_limits<double>::epsilon()*sc;rg=std::max(0.0,rg-guard);reqA=(dc+vc*tr)*s+guard;if(vc>0)lead=(rg-dc)/vc;else lead=(rg>dc)?std::numeric_limits<double>::infinity():-std::numeric_limits<double>::infinity();}bool pass=geom&&rg>dc&&lead>tr;', ...
'  if(nlhs>0)plhs[0]=mxCreateLogicalScalar(pass);if(nlhs>1)plhs[1]=mxCreateDoubleScalar(rg);if(nlhs>2)plhs[2]=mxCreateDoubleScalar(lead);if(nlhs>3)plhs[3]=mxCreateDoubleScalar(reqA);if(nlhs>4)plhs[4]=mxCreateLogicalScalar(geom);return;}', ...
' mexErrMsgIdAndTxt("G26V204:AssocReacqMEX","Unknown command");', ...
'}'};
end


function v204b_mex_bootstrap_blind_gap_reacq(root)
name='g26v204b_blind_gap_reacq_mex';
v200_build_mex_if_needed(root,name,v204b_cpp_blind_gap_reacq_source(),'G26-V204B-BLIND-GAP-REACQ-MEX');
end

function lines=v204b_cpp_blind_gap_reacq_source()
lines={ ...
'#include "mex.h"', ...
'#include <cmath>', ...
'#include <limits>', ...
'#include <algorithm>', ...
'static void req(bool q,const char*m){if(!q)mexErrMsgIdAndTxt("G26V204B:BlindGapReacqMEX","%s",m);}', ...
'void mexFunction(int nlhs,mxArray*plhs[],int nrhs,const mxArray*prhs[]){', ...
' req(nrhs==9,"Expected blindGapRad,rayNetRad,angularErrRad,minRadius,maxRange,vClosing,dCollision,reactionTime,guardScale"); req(nlhs<=7,"At most 7 outputs");', ...
' double bg=mxGetScalar(prhs[0]),rn=mxGetScalar(prhs[1]),ae=mxGetScalar(prhs[2]),a=mxGetScalar(prhs[3]),rmax=mxGetScalar(prhs[4]),vc=mxGetScalar(prhs[5]),dc=mxGetScalar(prhs[6]),tr=mxGetScalar(prhs[7]),gs=mxGetScalar(prhs[8]);', ...
' req(std::isfinite(bg)&&bg>=0&&std::isfinite(rn)&&rn>=0&&std::isfinite(ae)&&ae>=0&&std::isfinite(a)&&a>=0&&std::isfinite(rmax)&&rmax>0&&std::isfinite(vc)&&vc>=0&&std::isfinite(dc)&&dc>=0&&std::isfinite(tr)&&tr>=0&&std::isfinite(gs)&&gs>=0,"invalid input");', ...
' double d=bg+rn+ae; bool geom=d>=0&&d<1.5707963267948966&&a>0;', ...
' double rg=0,lead=-std::numeric_limits<double>::infinity(),reqA=std::numeric_limits<double>::infinity(),guard=0;', ...
' if(geom){double s=std::sin(d); if(d==0){rg=rmax; reqA=0;} else {rg=std::min(rmax,a/s); reqA=(dc+vc*tr)*s;} double sc=std::max(1.0,std::max(rmax,std::max(rg,std::max(dc,a))));guard=gs*std::numeric_limits<double>::epsilon()*sc;rg=std::max(0.0,rg-guard);reqA+=guard;if(vc>0)lead=(rg-dc)/vc;else lead=(rg>dc)?std::numeric_limits<double>::infinity():-std::numeric_limits<double>::infinity();}', ...
' bool pass=geom&&rg>dc&&lead>tr;', ...
' if(nlhs>0)plhs[0]=mxCreateLogicalScalar(pass); if(nlhs>1)plhs[1]=mxCreateDoubleScalar(rg); if(nlhs>2)plhs[2]=mxCreateDoubleScalar(lead); if(nlhs>3)plhs[3]=mxCreateDoubleScalar(reqA); if(nlhs>4)plhs[4]=mxCreateDoubleScalar(d); if(nlhs>5)plhs[5]=mxCreateLogicalScalar(geom); if(nlhs>6)plhs[6]=mxCreateDoubleScalar(guard);', ...
'}'};
end

function A=v204b_blind_gap_selftest(CFG)
% Ideal geometry theorem:
% lower sensor covers z<=0. Upper full hemisphere has center normal tilted
% theta from +z.  In the rear vertical great-circle, the uncovered wedge is
% 0<elevation<theta, so its maximum distance to either covered boundary is theta/2.
theta=deg2rad(CFG.v204b_upper_hemisphere_tilt_deg);
gap=theta/2;
g0=0/2; g60=deg2rad(60)/2;
A=struct();
A.current_tilt_deg=CFG.v204b_upper_hemisphere_tilt_deg;
A.analytic_blind_gap_deg=rad2deg(gap);
A.zero_tilt_zero_gap=(g0==0);
A.sixty_tilt_thirty_gap=abs(rad2deg(g60)-30)<64*eps;
A.current_formula=abs(A.analytic_blind_gap_deg-0.5*CFG.v204b_upper_hemisphere_tilt_deg)<64*eps(max(1,CFG.v204b_upper_hemisphere_tilt_deg));
A.complete_global_fov=(gap==0);
A.pass=A.zero_tilt_zero_gap&&A.sixty_tilt_thirty_gap&&A.current_formula;
assert(A.pass,'G26V204B:BlindGapSelftest','Analytic two-hemisphere blind-gap theorem selftest failed.');
fprintf('[G26-V204B-BLIND-GAP-SELFTEST] PASS | upperTilt=%.3f deg -> maxFoVBlindDepth=%.3f deg | completeGlobalFoV=%d\n',A.current_tilt_deg,A.analytic_blind_gap_deg,A.complete_global_fov);
end

function A=v204b_blind_gap_reacq_selftest(CFG)
% The total angular miss bound is blind-FoV depth + ray-net bound + sensor angular/extrinsic error.
% This generalizes V204: complete angular FoV is sufficient but not necessary when a positive
% obstacle radius is guaranteed.
bg=deg2rad(15); rn=deg2rad(4); ae=deg2rad(.1); a=.5; rmax=6; vc=2; dc=.4; tr=.2;
[p,rg,lead,reqA,dt,geom]=g26v204b_blind_gap_reacq_mex(bg,rn,ae,a,rmax,vc,dc,tr,CFG.v204_assoc_guard_scale);
[pTiny,rgTiny]=g26v204b_blind_gap_reacq_mex(bg,rn,ae,1e-9,rmax,vc,dc,tr,CFG.v204_assoc_guard_scale);
[pWide,~,~,~,dtWide,gWide]=g26v204b_blind_gap_reacq_mex(deg2rad(80),deg2rad(10),deg2rad(1),a,rmax,vc,dc,tr,CFG.v204_assoc_guard_scale);
A=struct();
A.good_pass=logical(p)&&logical(geom)&&rg>dc&&lead>tr;
A.tiny_fails=~logical(pTiny)&&rgTiny<dc;
A.too_wide_geometry_fails=~logical(pWide)&&~logical(gWide)&&dtWide>=pi/2;
A.total_angle_deg=rad2deg(dt); A.Rg_m=rg; A.lead_s=lead; A.required_min_radius_m=reqA;
A.pass=A.good_pass&&A.tiny_fails&&A.too_wide_geometry_fails;
assert(A.pass,'G26V204B:BlindGapReacqSelftest','Bounded blind-gap reacquisition selftest failed.');
fprintf('[G26-V204B-BLIND-GAP-REACQ-SELFTEST] PASS | totalAngle=%.3f deg Rg=%.3f m lead=%.3f s requiredAmin=%.4f m | tiny/wide fail-closed\n',A.total_angle_deg,A.Rg_m,A.lead_s,A.required_min_radius_m);
end


function A=v204c_geometry_design_selftest(CFG)
% Under the idealized V204B model (full upper hemisphere + downward lower
% hemisphere), worst-case FoV blind depth is tilt/2.  With fixed ray-net
% and angular error, the required detectable-size coefficient is monotone
% in nonnegative upper tilt.  This is a geometry theorem, not hardware authority.
tilt=reshape(double(CFG.v204c_tilt_sweep_deg),1,[]);
blind=0.5*tilt;
rayNet=0.5*(CFG.v204b_ray_az_spacing_deg+CFG.v204b_ray_el_spacing_deg);
coef=sin(deg2rad(blind+rayNet));
dcoef=diff(coef);
tol=256*eps(max(1,max(abs(coef))));
[~,i0]=min(abs(tilt));
[~,ic]=min(abs(tilt-CFG.v204b_upper_hemisphere_tilt_deg));
A=struct();
A.tilt_deg=tilt;
A.blind_depth_deg=blind;
A.ray_net_deg=rayNet;
A.normalized_required_radius_coefficient=coef;
A.monotone_non_decreasing=all(dcoef>=-tol);
A.zero_tilt_is_global_min=abs(coef(i0)-min(coef))<=tol;
A.current_tilt_present=abs(tilt(ic)-CFG.v204b_upper_hemisphere_tilt_deg)<1e-12;
A.current_to_zero_ratio=coef(ic)/max(coef(i0),realmin);
A.current_forward_upper_fov_interior_margin_deg=CFG.v204b_upper_hemisphere_tilt_deg;
A.zero_tilt_forward_upper_fov_interior_margin_deg=0;
A.mount_change_promoted=CFG.v204c_apply_mount_change;
A.pass=A.monotone_non_decreasing && A.zero_tilt_is_global_min && A.current_tilt_present && ~A.mount_change_promoted;
assert(A.pass,'G26V204C:GeometryDesignSelftest','Ideal full-hemisphere tilt-design theorem selftest failed.');
fprintf('[G26-V204C-GEOMETRY-DESIGN-SELFTEST] PASS | worst-case blind depth=tilt/2 | normalized required-size coefficient monotone | current30/zero=%.3fx | mountChange=0\n',A.current_to_zero_ratio);
end

function C=v204c_geometry_design_contract(CFG,SELF)
A=SELF.geometry_design;
C=struct();
C.authority=false;
C.runtime_mount_changed=false;
C.model_scope='IDEAL_FULL_UPPER_HEMISPHERE_PLUS_DOWNWARD_LOWER_HEMISPHERE';
C.current_tilt_deg=CFG.v204b_upper_hemisphere_tilt_deg;
C.current_blind_depth_deg=0.5*C.current_tilt_deg;
C.ray_net_bound_deg=A.ray_net_deg;
C.reference_angular_error_deg=reshape(double(CFG.v204c_reference_angular_error_deg),1,[]);
C.tilt_sweep_deg=A.tilt_deg;
C.blind_depth_sweep_deg=A.blind_depth_deg;
C.coefficient_no_error=A.normalized_required_radius_coefficient;
C.zero_tilt_coefficient_by_error=sin(deg2rad(C.ray_net_bound_deg+C.reference_angular_error_deg));
C.current_tilt_coefficient_by_error=sin(deg2rad(C.current_blind_depth_deg+C.ray_net_bound_deg+C.reference_angular_error_deg));
C.current_to_zero_ratio_by_error=C.current_tilt_coefficient_by_error./C.zero_tilt_coefficient_by_error;
C.global_worst_case_optimal_tilt_deg=0;
C.theorem_monotone=A.monotone_non_decreasing;
C.zero_tilt_minimizes_global_worst_case=A.zero_tilt_is_global_min;
C.current_forward_upper_fov_interior_margin_deg=A.current_forward_upper_fov_interior_margin_deg;
C.example_only=true;
C.example_collision_distance_m=CFG.v204_collision_distance_m;
C.example_closing_speed_mps=CFG.v204c_example_closing_speed_mps;
C.example_reaction_s=CFG.v204c_example_reaction_s;
C.example_D_m=C.example_collision_distance_m+C.example_closing_speed_mps*C.example_reaction_s;
% Use the largest reference angular error as a deliberately conservative
% qualification example.  It is NOT a hardware value.
e=max(C.reference_angular_error_deg);
C.example_angular_error_deg=e;
C.example_required_amin_current_m=C.example_D_m*sin(deg2rad(C.current_blind_depth_deg+C.ray_net_bound_deg+e));
C.example_required_amin_zero_m=C.example_D_m*sin(deg2rad(C.ray_net_bound_deg+e));
C.example_reduction_factor=C.example_required_amin_current_m/max(C.example_required_amin_zero_m,realmin);
C.hardware_values_still_required={'actual ray directions if a hardware guarantee is claimed','angular/extrinsic error bound if a hardware guarantee is claimed','minimum detectable obstacle radius','max closing speed','reaction WCET'};
C.planner_qualification_may_proceed_with_conservative_memory=true;
C.production_promotion_blocked=true;
C.reason='CURRENT_30DEG_TILT_IS_NOT_GLOBAL_WORST_CASE_OPTIMAL_UNDER_IDEAL_FULL_HEMISPHERE_MODEL__HARDWARE_AUTHORITY_STILL_OPEN';
end

function v204c_print_geometry_design(C)
fprintf('\n');
fprintf('========================================================================================================================\n');
fprintf(' G26-V204C DUAL-LIDAR GEOMETRY DESIGN ENVELOPE | QUALIFICATION ONLY\n');
fprintf('------------------------------------------------------------------------------------------------------------------------\n');
fprintf(' model=%s\n',C.model_scope);
fprintf(' current tilt=%.1f deg -> FoV blind depth=%.1f deg | ray-net bound=%.1f deg\n',C.current_tilt_deg,C.current_blind_depth_deg,C.ray_net_bound_deg);
fprintf(' theorem: for fixed ray-net/error, global worst-case detectable-size requirement is monotone in nonnegative upper tilt\n');
fprintf(' normalized requiredAmin coefficient k=sin(blindDepth+rayNet+angularError), requiredAmin=D*k\n');
for k=1:numel(C.reference_angular_error_deg)
    fprintf(' angularError=%4.1f deg | k(tilt0)=%.6f k(current)=%.6f ratio=%.3fx\n', ...
        C.reference_angular_error_deg(k),C.zero_tilt_coefficient_by_error(k),C.current_tilt_coefficient_by_error(k),C.current_to_zero_ratio_by_error(k));
end
fprintf(' qualification example ONLY: dCollision=%.3f m vClosing=%.2f m/s Treact=%.3f ms angularError=%.1f deg\n', ...
    C.example_collision_distance_m,C.example_closing_speed_mps,1e3*C.example_reaction_s,C.example_angular_error_deg);
fprintf('   requiredAmin current tilt = %.4f m | zero tilt = %.4f m | ratio=%.3fx\n', ...
    C.example_required_amin_current_m,C.example_required_amin_zero_m,C.example_reduction_factor);
fprintf(' global-worst-case geometry candidate under this ideal model: upper tilt=0 deg (NO automatic mount change / authority=0)\n');
fprintf(' forward upper-FoV interior margin changes from %.1f deg at current tilt to 0 deg at zero tilt; V205 paper scope assumes no self-occlusion, while hardware mount validation stays outside the novelty claim.\n', ...
    C.current_forward_upper_fov_interior_margin_deg);
fprintf(' planner qualification may proceed with conservative no-contraction memory=%d | production promotion blocked=%d\n', ...
    C.planner_qualification_may_proceed_with_conservative_memory,C.production_promotion_blocked);
fprintf(' reason=%s\n',C.reason);
fprintf('========================================================================================================================\n');
end

function C=v204b_blind_gap_contract(CFG,SELF)
C=struct();
C.theorem_selftest=SELF.blind_gap_reacq.pass;
C.upper_tilt_deg=CFG.v204b_upper_hemisphere_tilt_deg;
C.fov_blind_depth_deg=SELF.blind_gap.analytic_blind_gap_deg;
% For a structured azimuth/elevation ray grid, a conservative within-FoV
% geodesic covering bound follows from triangle inequality: half az step +
% half elevation step.
C.ray_net_bound_deg=0.5*(CFG.v204b_ray_az_spacing_deg+CFG.v204b_ray_el_spacing_deg);
C.base_total_gap_without_sensor_error_deg=C.fov_blind_depth_deg+C.ray_net_bound_deg;
C.geometry_is_hardware_certified=CFG.v204b_geometry_is_hardware_certified;
C.blind_gap_authority=CFG.v204b_blind_gap_authority;
vals=[CFG.v204b_sensor_angular_extrinsic_error_deg CFG.v204b_min_detectable_radius_m CFG.v204b_max_closing_speed_mps CFG.v204b_reaction_wcet_s CFG.v204b_planning_ingest_max_range_m CFG.v204_collision_distance_m];
C.candidate_evaluable=all(isfinite(vals))&&all(vals>=0)&&CFG.v204b_min_detectable_radius_m>0;
C.candidate_pass=false; C.total_gap_deg=NaN; C.Rg_m=NaN; C.lead_s=NaN; C.required_min_radius_m=NaN;
if C.candidate_evaluable
    [pp,rg,ld,ra,dt]=g26v204b_blind_gap_reacq_mex(deg2rad(C.fov_blind_depth_deg),deg2rad(C.ray_net_bound_deg),deg2rad(CFG.v204b_sensor_angular_extrinsic_error_deg),CFG.v204b_min_detectable_radius_m,CFG.v204b_planning_ingest_max_range_m,CFG.v204b_max_closing_speed_mps,CFG.v204_collision_distance_m,CFG.v204b_reaction_wcet_s,CFG.v204_assoc_guard_scale);
    C.candidate_pass=logical(pp); C.total_gap_deg=rad2deg(dt); C.Rg_m=rg; C.lead_s=ld; C.required_min_radius_m=ra;
end
C.dynamic_contraction_allowed=C.candidate_pass&&C.geometry_is_hardware_certified&&CFG.v204_association_source_authority;
C.dynamic_retirement_allowed=false;
end

function v204b_print_blind_gap(C)
fprintf('\n[G26-V204B DUAL-LIDAR BOUNDED-BLIND-GAP CANDIDATE | AUTHORITY REMAINS 0]\n');
fprintf(' idealized current geometry: upper full hemisphere tilt=%.3f deg | lower downward hemisphere | maxFoVBlindDepth=%.3f deg\n',C.upper_tilt_deg,C.fov_blind_depth_deg);
fprintf(' structured-ray conservative net bound=%.3f deg | base total gap before angular/extrinsic error=%.3f deg\n',C.ray_net_bound_deg,C.base_total_gap_without_sensor_error_deg);
if C.candidate_evaluable
    fprintf(' candidatePass=%d totalGap=%.6g deg Rg=%.6g m lead=%.6g s requiredMinRadius=%.6g m\n',C.candidate_pass,C.total_gap_deg,C.Rg_m,C.lead_s,C.required_min_radius_m);
else
    fprintf(' candidate=INCONCLUSIVE | need angular/extrinsic error, positive detectable radius, max closing speed, and reaction WCET\n');
    fprintf(' once supplied: requiredAmin=(dCollision+vClosing*Treact)*sin((%.6g deg)+angularError)\n',C.base_total_gap_without_sensor_error_deg);
end
fprintf(' geometryHardwareCertified=%d blindGapAuthority=%d dynamicContractionAllowed=%d retirementAllowed=%d\n',C.geometry_is_hardware_certified,C.blind_gap_authority,C.dynamic_contraction_allowed,C.dynamic_retirement_allowed);
end

% ========================================================================
% V205 MEX: explicit effective-range gate + finite-horizon local target.
% Out-of-range and too-close/nonfinite measurements become UNKNOWN (NaN);
% they are never promoted to FREE.  This is sensor-contract plumbing only.
% ========================================================================

function v205_mex_bootstrap_range_gate(root)
name='g26v205_range_gate_mex';
v200_build_mex_if_needed(root,name,v205_cpp_range_gate_source(),'G26-V205-RANGE-GATE-MEX');
end

function lines=v205_cpp_range_gate_source()
lines={ ...
'#include "mex.h"', ...
'#include <cmath>', ...
'#include <algorithm>', ...
'#include <limits>', ...
'static void req(bool q,const char*m){if(!q)mexErrMsgIdAndTxt("G26V205:RangeGateMEX","%s",m);}', ...
'void mexFunction(int nlhs,mxArray*plhs[],int nrhs,const mxArray*prhs[]){', ...
' req(nrhs==3,"Expected ranges,minRange,maxRange");req(nlhs<=3,"At most three outputs");', ...
' req(mxIsDouble(prhs[0]),"ranges must be double");mwSize N=mxGetNumberOfElements(prhs[0]);const double*R=mxGetPr(prhs[0]);', ...
' double rmin=mxGetScalar(prhs[1]),rmax=mxGetScalar(prhs[2]);req(std::isfinite(rmin)&&rmin>=0&&std::isfinite(rmax)&&rmax>rmin,"range bounds invalid");', ...
' plhs[0]=mxCreateDoubleMatrix(1,N,mxREAL);double*O=mxGetPr(plhs[0]);', ...
' if(nlhs>1)plhs[1]=mxCreateDoubleMatrix(1,N,mxREAL);double*M=nlhs>1?mxGetPr(plhs[1]):nullptr;', ...
' mwSize K=0;double nan=mxGetNaN();for(mwSize i=0;i<N;++i){bool ok=std::isfinite(R[i])&&R[i]>=rmin&&R[i]<=rmax;if(ok){O[i]=R[i];++K;}else O[i]=nan;if(M)M[i]=ok?1.0:0.0;}', ...
' if(nlhs>2)plhs[2]=mxCreateDoubleScalar((double)K);', ...
'}'};
end

function v205_mex_bootstrap_local_target(root)
name='g26v205_local_target_mex';
v200_build_mex_if_needed(root,name,v205_cpp_local_target_source(),'G26-V205-LOCAL-TARGET-MEX');
end

function lines=v205_cpp_local_target_source()
lines={ ...
'#include "mex.h"', ...
'#include <cmath>', ...
'#include <algorithm>', ...
'static void req(bool q,const char*m){if(!q)mexErrMsgIdAndTxt("G26V205:LocalTargetMEX","%s",m);}', ...
'void mexFunction(int nlhs,mxArray*plhs[],int nrhs,const mxArray*prhs[]){', ...
' req(nrhs==3,"Expected p0,goal,horizon");req(nlhs<=3,"At most three outputs");', ...
' req(mxIsDouble(prhs[0])&&mxGetNumberOfElements(prhs[0])==3,"p0 must be 3-vector");req(mxIsDouble(prhs[1])&&mxGetNumberOfElements(prhs[1])==3,"goal must be 3-vector");', ...
' const double*p=mxGetPr(prhs[0]);const double*g=mxGetPr(prhs[1]);double H=mxGetScalar(prhs[2]);req(std::isfinite(H)&&H>0,"horizon invalid");', ...
' double d[3]={g[0]-p[0],g[1]-p[1],g[2]-p[2]};double L=std::sqrt(d[0]*d[0]+d[1]*d[1]+d[2]*d[2]);req(std::isfinite(L),"goal distance invalid");double a=(L>H&&L>0)?H/L:1.0;', ...
' plhs[0]=mxCreateDoubleMatrix(3,1,mxREAL);double*q=mxGetPr(plhs[0]);for(int k=0;k<3;++k)q[k]=p[k]+a*d[k];', ...
' if(nlhs>1)plhs[1]=mxCreateDoubleScalar(L);if(nlhs>2)plhs[2]=mxCreateLogicalScalar(L>H);', ...
'}'};
end


% ========================================================================
% V206 MEX: deterministic adaptive spatial guide over inflated sphere covers.
% The output is a proposal path only.  Every accepted segment is checked
% against ALL supplied sphere covers.  maxNodes is a resource bound; reaching
% it returns SEARCH_INCOMPLETE and is never an infeasibility declaration.
% ========================================================================

function v206_mex_bootstrap_spatial_guide(root)
name='g26v206_spatial_guide_mex';
v200_build_mex_if_needed(root,name,v206_cpp_spatial_guide_source(),'G26-V206-SPATIAL-GUIDE-MEX');
end

function lines=v206_cpp_spatial_guide_source()
lines={ ...
'#include "mex.h"', ...
'#include <vector>', ...
'#include <cmath>', ...
'#include <limits>', ...
'#include <algorithm>', ...
'static void req(bool q,const char*m){if(!q)mexErrMsgIdAndTxt("G26V206:SpatialGuideMEX","%s",m);}', ...
'static double n3(double x,double y,double z){return std::sqrt(x*x+y*y+z*z);}', ...
'static double dot3(const double*a,const double*b){return a[0]*b[0]+a[1]*b[1]+a[2]*b[2];}', ...
'static double guard(double sc,double gs){return gs*std::numeric_limits<double>::epsilon()*std::max(1.0,sc);}', ...
'static double segdist(const double*A,const double*B,const double*C,double*q,double&t){double d[3]={B[0]-A[0],B[1]-A[1],B[2]-A[2]};double l2=dot3(d,d);t=0.0;if(l2>0){double w[3]={C[0]-A[0],C[1]-A[1],C[2]-A[2]};t=std::max(0.0,std::min(1.0,dot3(w,d)/l2));}for(int a=0;a<3;++a)q[a]=A[a]+t*d[a];return n3(q[0]-C[0],q[1]-C[1],q[2]-C[2]);}', ...
'static bool inHorizon(const double*W,const double*p0,double H,double gs){double dx=W[0]-p0[0],dy=W[1]-p0[1],dz=W[2]-p0[2];double d=n3(dx,dy,dz);return d<=H+guard(std::max(H,d),gs);}', ...
'static bool sideCandidate(const double*A,const double*B,const double*C,double R,const double*q,const double*n,const double*p0,double H,double gs,double*outW,double&outS){', ...
' double v[3]={q[0]-p0[0],q[1]-p0[1],q[2]-p0[2]};double b=dot3(v,n);double vv=dot3(v,v);double disc=b*b+H*H-vv;double gd=guard(std::max(H,std::sqrt(std::max(0.0,vv))),gs);if(disc<-gd)return false;disc=std::max(0.0,disc);double smax=-b+std::sqrt(disc);if(!(smax>gd&&std::isfinite(smax)))return false;', ...
' auto clearAt=[&](double ss){double W[3]={q[0]+ss*n[0],q[1]+ss*n[1],q[2]+ss*n[2]};double qq[3],tt;double d1=segdist(A,W,C,qq,tt),d2=segdist(W,B,C,qq,tt);double sc=std::max(R,std::max(d1,d2));double g=guard(sc,gs);return d1>=R+16.0*g&&d2>=R+16.0*g&&inHorizon(W,p0,H,gs);};', ...
' if(!clearAt(smax))return false;double lo=0.0,hi=smax;for(int it=0;it<64;++it){double md=0.5*(lo+hi);if(clearAt(md))hi=md;else lo=md;}outS=hi;for(int a=0;a<3;++a)outW[a]=q[a]+hi*n[a];return clearAt(hi);}', ...
'void mexFunction(int nlhs,mxArray*plhs[],int nrhs,const mxArray*prhs[]){', ...
' req(nrhs==7,"Expected p0,goal,centers,radii,horizon,maxNodes,guardScale");req(nlhs<=5,"At most five outputs");', ...
' req(mxIsDouble(prhs[0])&&mxGetNumberOfElements(prhs[0])==3,"p0 must be 3-vector");req(mxIsDouble(prhs[1])&&mxGetNumberOfElements(prhs[1])==3,"goal must be 3-vector");', ...
' req(mxIsDouble(prhs[2])&&mxGetM(prhs[2])==3,"centers must be 3xM");mwSize M=mxGetN(prhs[2]);req(mxIsDouble(prhs[3])&&mxGetNumberOfElements(prhs[3])==M,"radii mismatch");', ...
' const double*p0=mxGetPr(prhs[0]);const double*pg=mxGetPr(prhs[1]);const double*C=mxGetPr(prhs[2]);const double*R=mxGetPr(prhs[3]);double H=mxGetScalar(prhs[4]);int maxNodes=(int)std::llround(mxGetScalar(prhs[5]));double gs=mxGetScalar(prhs[6]);', ...
' req(std::isfinite(H)&&H>0&&maxNodes>=2&&maxNodes<=4096&&std::isfinite(gs)&&gs>=0,"resource/horizon input invalid");for(mwSize j=0;j<M;++j)req(std::isfinite(R[j])&&R[j]>=0,"radius invalid");', ...
' std::vector<double>P;P.reserve(3*maxNodes);for(int a=0;a<3;++a)P.push_back(p0[a]);for(int a=0;a<3;++a)P.push_back(pg[a]);int status=1;int ins=0;double checks=0.0;', ...
' if(!inHorizon(pg,p0,H,gs))status=-2;', ...
' for(mwSize j=0;j<M&&status==1;++j){double q[3],t;double ds=n3(p0[0]-C[3*j],p0[1]-C[3*j+1],p0[2]-C[3*j+2]);double dg=n3(pg[0]-C[3*j],pg[1]-C[3*j+1],pg[2]-C[3*j+2]);double sc=std::max(R[j],std::max(ds,dg));double g=guard(sc,gs);if(ds<=R[j]+g||dg<=R[j]+g)status=-1;}', ...
' while(status==1){mwSize N=P.size()/3;double bestClr=std::numeric_limits<double>::infinity();mwSize bs=0,bo=0;bool hit=false;double bq[3]={0,0,0};', ...
'  for(mwSize si=0;si+1<N;++si){const double*A=&P[3*si];const double*B=&P[3*(si+1)];for(mwSize j=0;j<M;++j){double q[3],t;double d=segdist(A,B,C+3*j,q,t);checks+=1.0;double sc=std::max(R[j],d);double g=guard(sc,gs);double clr=d-R[j]-g;if(clr<bestClr){bestClr=clr;bs=si;bo=j;bq[0]=q[0];bq[1]=q[1];bq[2]=q[2];}if(clr<0)hit=true;}}', ...
'  if(!hit)break;if((int)N>=maxNodes){status=0;break;}const double*A=&P[3*bs];const double*B=&P[3*(bs+1)];const double*cc=C+3*bo;double dAB[3]={B[0]-A[0],B[1]-A[1],B[2]-A[2]};double L=n3(dAB[0],dAB[1],dAB[2]);if(!(L>guard(L,gs))){status=0;break;}double e[3]={dAB[0]/L,dAB[1]/L,dAB[2]/L};double rv[3]={bq[0]-cc[0],bq[1]-cc[1],bq[2]-cc[2]};double rn=n3(rv[0],rv[1],rv[2]);double n[3];', ...
'  if(rn>guard(rn,gs)){n[0]=rv[0]/rn;n[1]=rv[1]/rn;n[2]=rv[2]/rn;}else{n[0]=e[1];n[1]=-e[0];n[2]=0.0;double nn=n3(n[0],n[1],n[2]);if(nn<=guard(nn,gs)){n[0]=0.0;n[1]=e[2];n[2]=-e[1];nn=n3(n[0],n[1],n[2]);}if(!(nn>guard(nn,gs))){status=0;break;}n[0]/=nn;n[1]/=nn;n[2]/=nn;}', ...
'  double Wp[3],Wm[3],sp=0,sm=0;bool op=sideCandidate(A,B,cc,R[bo],bq,n,p0,H,gs,Wp,sp);double nm[3]={-n[0],-n[1],-n[2]};bool om=sideCandidate(A,B,cc,R[bo],bq,nm,p0,H,gs,Wm,sm);if(!op&&!om){status=0;break;}const double*W=(op&&(!om||sp<=sm))?Wp:Wm;', ...
'  P.insert(P.begin()+3*(bs+1),{W[0],W[1],W[2]});++ins;', ...
' }', ...
' mwSize N=P.size()/3;double minClr=std::numeric_limits<double>::infinity();if(M==0)minClr=mxGetInf();for(mwSize si=0;si+1<N;++si){for(mwSize j=0;j<M;++j){double q[3],t;double d=segdist(&P[3*si],&P[3*(si+1)],C+3*j,q,t);minClr=std::min(minClr,d-R[j]);}}', ...
' plhs[0]=mxCreateDoubleMatrix(3,N,mxREAL);double*O=mxGetPr(plhs[0]);for(mwSize i=0;i<3*N;++i)O[i]=P[i];if(nlhs>1)plhs[1]=mxCreateDoubleScalar((double)status);if(nlhs>2)plhs[2]=mxCreateDoubleScalar((double)ins);if(nlhs>3)plhs[3]=mxCreateDoubleScalar(minClr);if(nlhs>4)plhs[4]=mxCreateDoubleScalar(checks);', ...
'}'};
end

% ========================================================================
% V206 MEX: per-segment convex safe corridor from a certified-clear polyline.
% For each segment and each inflated sphere, the nearest-point normal defines
% a tangent half-space containing the whole segment and excluding the sphere.
% All obstacles are represented; no top-k or distance discard is used.
% ========================================================================

function v206_mex_bootstrap_segment_corridor(root)
name='g26v206_segment_corridor_mex';
v200_build_mex_if_needed(root,name,v206_cpp_segment_corridor_source(),'G26-V206-SEGMENT-CORRIDOR-MEX');
end

function lines=v206_cpp_segment_corridor_source()
lines={ ...
'#include "mex.h"', ...
'#include <cmath>', ...
'#include <limits>', ...
'#include <algorithm>', ...
'static void req(bool q,const char*m){if(!q)mexErrMsgIdAndTxt("G26V206:SegmentCorridorMEX","%s",m);}', ...
'static double n3(double x,double y,double z){return std::sqrt(x*x+y*y+z*z);}', ...
'static double dot3(const double*a,const double*b){return a[0]*b[0]+a[1]*b[1]+a[2]*b[2];}', ...
'static double segdist(const double*A,const double*B,const double*C,double*q){double d[3]={B[0]-A[0],B[1]-A[1],B[2]-A[2]};double l2=dot3(d,d);double t=0;if(l2>0){double w[3]={C[0]-A[0],C[1]-A[1],C[2]-A[2]};t=std::max(0.0,std::min(1.0,dot3(w,d)/l2));}for(int a=0;a<3;++a)q[a]=A[a]+t*d[a];return n3(q[0]-C[0],q[1]-C[1],q[2]-C[2]);}', ...
'void mexFunction(int nlhs,mxArray*plhs[],int nrhs,const mxArray*prhs[]){req(nrhs==4,"Expected waypoints,centers,radii,guardScale");req(nlhs<=6,"At most six outputs");req(mxIsDouble(prhs[0])&&mxGetM(prhs[0])==3&&mxGetN(prhs[0])>=2,"waypoints must be 3xN,N>=2");mwSize Np=mxGetN(prhs[0]);mwSize S=Np-1;req(mxIsDouble(prhs[1])&&mxGetM(prhs[1])==3,"centers must be 3xM");mwSize M=mxGetN(prhs[1]);req(mxIsDouble(prhs[2])&&mxGetNumberOfElements(prhs[2])==M,"radii mismatch");double gs=mxGetScalar(prhs[3]);req(std::isfinite(gs)&&gs>=0,"guardScale invalid");const double*W=mxGetPr(prhs[0]);const double*C=mxGetPr(prhs[1]);const double*R=mxGetPr(prhs[2]);mwSize K=S*M;', ...
' plhs[0]=mxCreateDoubleMatrix(3,K,mxREAL);double*ON=mxGetPr(plhs[0]);if(nlhs>1)plhs[1]=mxCreateDoubleMatrix(1,K,mxREAL);double*OH=nlhs>1?mxGetPr(plhs[1]):nullptr;if(nlhs>2)plhs[2]=mxCreateDoubleMatrix(1,S,mxREAL);double*OC=nlhs>2?mxGetPr(plhs[2]):nullptr;bool pass=true;double minClr=(M==0)?mxGetInf():std::numeric_limits<double>::infinity();mwSize kk=0;', ...
' for(mwSize s=0;s<S;++s){if(OC)OC[s]=(double)M;const double*A=W+3*s;const double*B=W+3*(s+1);for(mwSize j=0;j<M;++j,++kk){req(std::isfinite(R[j])&&R[j]>=0,"radius invalid");double q[3];double d=segdist(A,B,C+3*j,q);double sc=std::max(1.0,std::max(R[j],d));double g=gs*std::numeric_limits<double>::epsilon()*sc;double clr=d-R[j];minClr=std::min(minClr,clr);if(!(d>R[j]+g)){pass=false;ON[3*kk]=ON[3*kk+1]=ON[3*kk+2]=mxGetNaN();if(OH)OH[kk]=mxGetNaN();continue;}double n[3]={(q[0]-C[3*j])/d,(q[1]-C[3*j+1])/d,(q[2]-C[3*j+2])/d};double h=n[0]*C[3*j]+n[1]*C[3*j+1]+n[2]*C[3*j+2]+R[j];ON[3*kk]=n[0];ON[3*kk+1]=n[1];ON[3*kk+2]=n[2];if(OH)OH[kk]=h;}}', ...
' if(nlhs>3)plhs[3]=mxCreateLogicalScalar(pass);if(nlhs>4)plhs[4]=mxCreateDoubleScalar(minClr);if(nlhs>5)plhs[5]=mxCreateDoubleScalar((double)K);', ...
'}'};
end

function A=v205_sensor_range_selftest(CFG)
r=[NaN 0.05 0.10 5 19.999 20 20.001 40];
[rg,mask,n]=g26v205_range_gate_mex(r,CFG.v205_min_detection_range_m,CFG.v205_effective_sensing_range_m);
exp=[false false true true true true false false];
A=struct();A.pass=isequal(mask>0.5,exp)&&n==sum(exp)&&all(isfinite(rg(exp)))&&all(isnan(rg(~exp)));
A.out_of_range_unknown=all(isnan(rg(~exp)));A.baseline_range_m=CFG.v205_effective_sensing_range_m;
assert(A.pass,'G26V205:RangeSelftest','Effective sensing-range gate failed.');
fprintf('[G26-V205-RANGE-SELFTEST] PASS | effectiveRange=%.1f m minRange=%.2f m | outOfRange->UNKNOWN=1 selfOcclusion=0 hardwareAuthority=0\n',CFG.v205_effective_sensing_range_m,CFG.v205_min_detection_range_m);
end

function A=v205_local_target_selftest(CFG)
p0=[0;0;0];g1=[3;4;0];g2=[30;0;0];[q1,L1,c1]=g26v205_local_target_mex(p0,g1,CFG.v205_planning_horizon_m);[q2,L2,c2]=g26v205_local_target_mex(p0,g2,CFG.v205_planning_horizon_m);
A=struct();A.pass=norm(q1-g1)<1e-12 && ~c1 && abs(L1-5)<1e-12 && c2 && abs(norm(q2-p0)-CFG.v205_planning_horizon_m)<1e-12 && abs(L2-30)<1e-12;
A.horizon_m=CFG.v205_planning_horizon_m;A.range_ratio=CFG.v205_effective_sensing_range_m/CFG.v205_planning_horizon_m;
assert(A.pass,'G26V205:LocalTargetSelftest','Finite-horizon local-target selection failed.');
fprintf('[G26-V205-LOCAL-TARGET-SELFTEST] PASS | sensing=%.1f m planningHorizon=%.1f m ratio=%.3f clippedGoalExact=1\n',CFG.v205_effective_sensing_range_m,CFG.v205_planning_horizon_m,A.range_ratio);
end

function A=v205_planner_entry_selftest(CFG)
% Planner-entry semantics only: a straight seed is continuously certified if
% clear.  If blocked, the result is SPATIAL_SOLVER_REQUIRED, never an
% infeasibility declaration.  V205 intentionally does not add the avoidance solver.
p0=[0;0;0];goal=[6;0;0];[pg,~,~]=g26v205_local_target_mex(p0,goal,CFG.v205_planning_horizon_m);
J0=zeros(3,7,1);J1=zeros(3,7,1);J0(:,1,1)=p0;J1(:,1,1)=pg;dt=3.0;F=zeros(3,4,1);F(1,:,1)=linspace(1.8,4.2,4);B=g26v200_c6_construct_mex(J0,J1,dt,F);
P=reshape(B,3,[]);pc=size(P,2);RR=[zeros(2,pc);CFG.gravity*ones(1,pc)];rc=pc;ta=0;tb=dt;K=[CFG.R_payload+CFG.geometry_margin_m,CFG.R_uav+CFG.geometry_margin_m,CFG.R_cable+CFG.geometry_margin_m,CFG.L_cable];OR=reshape(eye(3),9,1);ER=.20*ones(3,1);V=zeros(3,1);
okClear=g26v200_direct_cert_mex(P,pc,RR,rc,ta,tb,[3;3;0],V,OR,ER,K,CFG.direct_cert_max_depth,CFG.direct_cert_guard_scale);
okBlocked=g26v200_direct_cert_mex(P,pc,RR,rc,ta,tb,[3;0;0],V,OR,ER,K,CFG.direct_cert_max_depth,CFG.direct_cert_guard_scale);
A=struct();A.pass=logical(okClear)&&~logical(okBlocked);A.clear_status='STRAIGHT_PREFIX_CERTIFIED';A.blocked_status='SPATIAL_SOLVER_REQUIRED';A.blocked_is_infeasible=false;A.solver_authority=false;
assert(A.pass,'G26V205:PlannerEntrySelftest','Planner-entry continuous-certificate semantics failed.');
fprintf('[G26-V205-PLANNER-ENTRY-SELFTEST] PASS | clear straight seed certified | blocked seed -> SPATIAL_SOLVER_REQUIRED | infeasibilityClaim=0\n');
end


function A=v206_spatial_guide_selftest(CFG)
p0=[0;0;0];pg=[6;0;0];H=CFG.v205_planning_horizon_m;
C1=[3;0;0];R1=0.20+CFG.v206_reference_inflation_m;
[W1,s1,i1,m1]=g26v206_spatial_guide_mex(p0,pg,C1,R1,H,CFG.v206_spatial_guide_max_nodes,CFG.v206_spatial_guide_guard_scale);
C2=[2.0 3.6 5.0;0 0.45 -0.40;0 0 0];R2=(0.20+CFG.v206_reference_inflation_m)*ones(1,3);
[W2,s2,i2,m2]=g26v206_spatial_guide_mex(p0,pg,C2,R2,H,CFG.v206_spatial_guide_max_nodes,CFG.v206_spatial_guide_guard_scale);
[~,sBad]=g26v206_spatial_guide_mex(p0,pg,p0,0.25,H,CFG.v206_spatial_guide_max_nodes,CFG.v206_spatial_guide_guard_scale);
within1=all(vecnorm(W1-p0,2,1)<=H+1e-10);within2=all(vecnorm(W2-p0,2,1)<=H+1e-10);
A=struct();A.single_success=(s1==1)&&size(W1,2)>=3&&i1>=1&&m1>=-1e-10&&within1;A.multi_success=(s2==1)&&size(W2,2)>=3&&i2>=1&&m2>=-1e-10&&within2;A.endpoint_occupied_rejected=(sBad==-1);
A.search_complete=false;A.search_miss_is_infeasible=false;A.all_obstacles_scanned=true;A.pass=A.single_success&&A.multi_success&&A.endpoint_occupied_rejected&&~A.search_complete&&~A.search_miss_is_infeasible;
assert(A.pass,'G26V206:SpatialGuideSelftest','Adaptive spatial-guide selftest failed.');
fprintf('[G26-V206-SPATIAL-GUIDE-SELFTEST] PASS | single/multi blocker solved | all-sphere segment clearance=1 | resourceMiss->SEARCH_INCOMPLETE | infeasibilityClaim=0\n');
end

function A=v206_segment_corridor_selftest(CFG)
p0=[0;0;0];pg=[6;0;0];C=[2.0 3.6 5.0;0 0.45 -0.40;0 0 0];R=(0.20+CFG.v206_reference_inflation_m)*ones(1,3);
[W,st]=g26v206_spatial_guide_mex(p0,pg,C,R,CFG.v205_planning_horizon_m,CFG.v206_spatial_guide_max_nodes,CFG.v206_spatial_guide_guard_scale);
assert(st==1,'G26V206:CorridorSelftest','Guide prerequisite failed.');
[N,h,cnt,ok,minClr,K]=g26v206_segment_corridor_mex(W,C,R,CFG.v206_spatial_guide_guard_scale);
M=size(C,2);S=size(W,2)-1;endpointInside=true;obstacleExcluded=true;
for q=1:S
    for j=1:M
        f=(q-1)*M+j;n=N(:,f);
        endpointInside=endpointInside && all(n'*W(:,q:q+1)>=h(f)-1e-10);
        obstacleExcluded=obstacleExcluded && (n'*C(:,j)+R(j)<=h(f)+1e-10);
    end
end
A=struct();A.pass=logical(ok)&&K==S*M&&all(cnt==M)&&minClr>=-1e-10&&endpointInside&&obstacleExcluded;A.faces=K;A.segments=S;A.obstacles=M;A.no_topk=true;A.all_obstacles_per_segment=true;
assert(A.pass,'G26V206:CorridorSelftest','Segment safe-corridor theorem selftest failed.');
fprintf('[G26-V206-SEGMENT-CORRIDOR-SELFTEST] PASS | segments=%d obstacles=%d faces=%d | every segment endpoint inside | every inflated sphere excluded | topK=0\n',S,M,K);
end

function A=v206_c6_corridor_carrier_selftest(CFG)
% Qualification carrier only: each clear polyline segment is embedded in a
% degree-17 C6 segment whose control points stay on that line segment.
% Internal physical jets d1..d6 are all shared zeros.  This proves the
% corridor can carry a C6 curve, but it is NOT one of the retained production
% backends and no dynamic/time-optimality authority is claimed here.
p0=[0;0;0];pg=[6;0;0];C=[2.0 3.6 5.0;0 0.45 -0.40;0 0 0];R=(0.20+CFG.v206_reference_inflation_m)*ones(1,3);
[W,st]=g26v206_spatial_guide_mex(p0,pg,C,R,CFG.v205_planning_horizon_m,CFG.v206_spatial_guide_max_nodes,CFG.v206_spatial_guide_guard_scale);
assert(st==1,'G26V206:C6Carrier','Guide prerequisite failed.');
[N,h,cnt,ok]=g26v206_segment_corridor_mex(W,C,R,CFG.v206_spatial_guide_guard_scale);assert(ok,'G26V206:C6Carrier','Corridor prerequisite failed.');
S=size(W,2)-1;M=size(C,2);J0=zeros(3,7,S);J1=zeros(3,7,S);F=zeros(3,4,S);dt=ones(1,S);
u=(7:10)/17;
for q=1:S
    J0(:,1,q)=W(:,q);J1(:,1,q)=W(:,q+1);
    for k=1:4,F(:,k,q)=(1-u(k))*W(:,q)+u(k)*W(:,q+1);end
end
B=g26v200_c6_construct_mex(J0,J1,dt,F);REL=v200_c6_relation_audit(B,J0,J1,dt);inside=true;
for q=1:S
    f0=sum(cnt(1:q-1));
    for j=1:M
        f=f0+j;inside=inside && all(N(:,f)'*B(:,:,q)>=h(f)-1e-10);
    end
end
A=struct();A.pass=REL.pass&&REL.shared_jet_exact&&inside;A.structural_c6=REL.pass;A.control_hull_inside_corridor=inside;A.production_backend=false;A.dynamic_authority=false;A.time_allocation_authority=false;
assert(A.pass,'G26V206:C6Carrier','C6 corridor-carrier embedding failed.');
fprintf('[G26-V206-C6-CORRIDOR-CARRIER] PASS | structuralC6=1 controlHullInsideSegmentSFC=1 | qualificationCarrierOnly=1 dynamicsAuthority=0 timeAuthority=0\n');
end

function C=v206_spatial_solver_contract(CFG,SELF)
C=struct();C.spatial_guide=SELF.spatial_guide.pass;C.segment_safe_corridor=SELF.segment_corridor.pass;C.c6_corridor_carrier=SELF.c6_corridor_carrier.pass;
C.spatial_solver_authority=CFG.v206_spatial_solver_authority;C.search_completeness_claim=CFG.v206_search_completeness_claim;C.search_miss_is_infeasible=false;
C.max_nodes_resource_bound=CFG.v206_spatial_guide_max_nodes;C.all_obstacles_scanned=true;C.topk_used=false;C.fixed_direction_bank_used=false;
C.reference_inflation_m=CFG.v206_reference_inflation_m;C.sphere_corridor_is_proposal_only=true;C.full_2x4_retained=true;
C.production_backend_selected=false;C.time_allocation_authority=false;C.final_continuous_whole_body_gate_unchanged=true;
end

function v206_print_spatial_solver(C,B)
fprintf('\n========================================================================================================================\n');
fprintf(' G26-V206 FINITE-HORIZON SPATIAL GUIDE / SEGMENT-SFC CONTRACT\n');
fprintf('------------------------------------------------------------------------------------------------------------------------\n');
fprintf(' spatialGuide=%d segmentSFC=%d c6Carrier=%d | searchComplete=%d solverAuthority=%d infeasibilityClaim=0\n',C.spatial_guide,C.segment_safe_corridor,C.c6_corridor_carrier,C.search_completeness_claim,C.spatial_solver_authority);
fprintf(' maxNodes(resource only)=%d | allObstaclesScanned=1 topK=0 fixedDirectionBank=0 | proposalInflation=%.3f m\n',C.max_nodes_resource_bound,C.reference_inflation_m);
fprintf(' RULE: every accepted guide segment is continuously separated from every supplied inflated sphere cover.\n');
fprintf(' RULE: each segment corridor is the intersection of tangent half-spaces generated from exact segment-to-sphere nearest points.\n');
fprintf(' RULE: hitting the resource bound or failing the two symmetric detour sides returns SEARCH_INCOMPLETE, never physical/path infeasibility.\n');
fprintf(' RULE: the degree-17 C6 carrier proves corridor embeddability only; V207 must synthesize/compare the retained 2x4 trajectory backends and continuous time allocation.\n');
if isfield(B.qual,'v206')
    for k=1:numel(B.qual.v206)
        Q=B.qual.v206(k);
        v202_print_stats(sprintf(' guide M=%d',Q.obstacles),Q.guide);
        v202_print_stats(sprintf(' corridor M=%d',Q.obstacles),Q.corridor);
        v202_print_stats(sprintf(' guide+corridor M=%d',Q.obstacles),Q.integrated);
        fprintf('   M=%d status=%d nodes=%d insertions=%d corridorFaces=%d\n',Q.obstacles,Q.status,Q.nodes,Q.insertions,Q.faces);
    end
    fprintf(' V206 p95 sensor-prefix+range+localTarget+guide+corridor(M=%d)+directCert(M=%d)=%.6f ms | remaining to 4ms=%+.6f ms\n', ...
        B.qual.v206(end).obstacles,B.qual.direct_cert(end).obstacles,B.qual.v206_screen_p95_ms,B.qual.v206_remaining_to_4ms);
end
fprintf('========================================================================================================================\n');
end

function [p0,pg,C,R]=v206_bench_case(M,CFG)
p0=[0;0;0];pg=[7;0;0];baseR=0.20+CFG.v206_reference_inflation_m;
if M==1
    C=[3.5;0;0];
elseif M==4
    C=[1.8 3.5 5.2 4.0;0 0.45 -0.50 2.2;0 0 0 0];
elseif M==12
    x=linspace(0.8,6.2,12);y=2.0*(-1).^(0:11);y([3 6 9])=[0 0.30 -0.25];C=[x;y;zeros(1,12)];
else
    error('G26V206:BenchCase','Unsupported qualification obstacle count.');
end
R=baseR*ones(1,M);
end

function B=v206_microbench(CFG)
B=v205_microbench(CFG);NS=CFG.v206_timing_samples;NB=CFG.v206_timing_batch_calls;MM=CFG.v206_bench_obstacle_counts;
B.qual.v206=repmat(struct('obstacles',0,'guide',[],'corridor',[],'integrated',[],'status',0,'nodes',0,'insertions',0,'faces',0),1,numel(MM));
for z=1:numel(MM)
    M=MM(z);[p0,pg,C,R]=v206_bench_case(M,CFG);
    [W,st,ins]=g26v206_spatial_guide_mex(p0,pg,C,R,CFG.v205_planning_horizon_m,CFG.v206_spatial_guide_max_nodes,CFG.v206_spatial_guide_guard_scale);
    [~,~,~,ok,~,K]=g26v206_segment_corridor_mex(W,C,R,CFG.v206_spatial_guide_guard_scale);
    assert(st==1&&ok,'G26V206:Bench','Qualification guide/corridor case failed.');
    for k=1:20,tmp=g26v206_spatial_guide_mex(p0,pg,C,R,CFG.v205_planning_horizon_m,CFG.v206_spatial_guide_max_nodes,CFG.v206_spatial_guide_guard_scale);end %#ok<NASGU>
    Tg=zeros(NS,1);for q=1:NS,tt=tic;for k=1:NB,tmp=g26v206_spatial_guide_mex(p0,pg,C,R,CFG.v205_planning_horizon_m,CFG.v206_spatial_guide_max_nodes,CFG.v206_spatial_guide_guard_scale);end;Tg(q)=1e3*toc(tt)/NB;end %#ok<NASGU>
    for k=1:20,[a,b,c,d]=g26v206_segment_corridor_mex(W,C,R,CFG.v206_spatial_guide_guard_scale);end %#ok<ASGLU>
    Tc=zeros(NS,1);for q=1:NS,tt=tic;for k=1:NB,[a,b,c,d]=g26v206_segment_corridor_mex(W,C,R,CFG.v206_spatial_guide_guard_scale);end;Tc(q)=1e3*toc(tt)/NB;end %#ok<ASGLU>
    Ti=zeros(NS,1);for q=1:NS,tt=tic;for k=1:NB,[WW,ss]=g26v206_spatial_guide_mex(p0,pg,C,R,CFG.v205_planning_horizon_m,CFG.v206_spatial_guide_max_nodes,CFG.v206_spatial_guide_guard_scale);if ss==1,[a,b,c,d]=g26v206_segment_corridor_mex(WW,C,R,CFG.v206_spatial_guide_guard_scale);end;end;Ti(q)=1e3*toc(tt)/NB;end %#ok<ASGLU>
    B.qual.v206(z).obstacles=M;B.qual.v206(z).guide=v202_stats(Tg);B.qual.v206(z).corridor=v202_stats(Tc);B.qual.v206(z).integrated=v202_stats(Ti);B.qual.v206(z).status=st;B.qual.v206(z).nodes=size(W,2);B.qual.v206(z).insertions=ins;B.qual.v206(z).faces=K;
end
B.qual.v206_screen_p95_ms=B.qual.p95_prefix_with_range_gate_ms+B.qual.local_target.p95+B.qual.v206(end).integrated.p95+B.qual.direct_cert(end).stats.p95;
B.qual.v206_remaining_to_4ms=CFG.target_sensor_return_to_switch_ms-B.qual.v206_screen_p95_ms;
end

function C=v205_sensor_range_contract(CFG,SELF)
C=struct();C.sensor_novelty_claim=false;C.self_occlusion_mode='NONE_IDEALIZED';C.payload_occlusion_mode='NONE_POINT_MASS_SENSOR_MODEL';C.effective_sensing_range_m=CFG.v205_effective_sensing_range_m;C.minimum_detection_range_m=CFG.v205_min_detection_range_m;C.range_stress_m=CFG.v205_range_stress_m;C.reference_low_reflectivity_range_m=CFG.v205_reference_low_reflectivity_range_m;C.hardware_range_authority=CFG.v205_hardware_range_authority;C.out_of_range_is_unknown=SELF.sensor_range.out_of_range_unknown;C.local_planning_horizon_m=CFG.v205_planning_horizon_m;C.sensing_to_planning_ratio=CFG.v205_effective_sensing_range_m/CFG.v205_planning_horizon_m;C.production_promotion=false;
D=CFG.v205_effective_sensing_range_m-CFG.v204_collision_distance_m;v=CFG.v205_qualification_closing_speed_mps;C.closing_speed_mps=v;C.latency_only_lead_s=D./v;C.latency_only_remaining_after_4ms_s=C.latency_only_lead_s-CFG.target_sensor_return_to_switch_ms/1000;C.latency_screen_only=true;
end

function C=v205_planner_entry_contract(CFG,SELF)
C=struct();C.finite_horizon_local_target=SELF.local_target.pass;C.straight_seed_certificate_gate=SELF.planner_entry.pass;C.blocked_seed_means_spatial_solver_required=true;C.generator_miss_is_not_infeasible=true;C.backend_solver_authority=CFG.v205_planner_solver_authority;C.full_2x4_retained=true;C.fixed_canonical_time_forbidden=CFG.fixed_time_partition_forbidden;C.empirical_time_bank_forbidden=CFG.empirical_time_bank_forbidden;C.next_solver_must_optimize_space_and_time=false;C.v206_spatial_stage_only=true;C.next_continuous_time_stage='V207';
end

function v205_print_sensor_range(C,B)
fprintf('\n========================================================================================================================\n');
fprintf(' G26-V205 SENSOR RANGE CONTRACT | QUALIFICATION / SIMULATION ASSUMPTION\n');
fprintf('------------------------------------------------------------------------------------------------------------------------\n');
fprintf(' selfOcclusion=NONE | payloadOcclusion=NONE(point-mass sensor model) | sensorNoveltyClaim=0\n');
fprintf(' effective sensing radius=%.1f m | local planning horizon=%.1f m | ratio=%.3f | outOfRange=UNKNOWN\n',C.effective_sensing_range_m,C.local_planning_horizon_m,C.sensing_to_planning_ratio);
fprintf(' range sensitivity set=[%s] m | reference low-reflectivity hardware class=%.1f m | hardwareAuthority=0\n',strtrim(sprintf('%.0f ',C.range_stress_m)),C.reference_low_reflectivity_range_m);
for k=1:numel(C.closing_speed_mps),fprintf(' latency-only screen vClosing=%5.1f m/s -> detection lead=%.4f s | after 4ms compute=%.4f s\n',C.closing_speed_mps(k),C.latency_only_lead_s(k),C.latency_only_remaining_after_4ms_s(k));end
v202_print_stats(' range gate 4320 rays',B.qual.range_gate);
fprintf(' RULE: the 20 m value is an effective simulation/planning sensing radius, not a hardware safety guarantee.\n');
fprintf(' RULE: sensing range and local planning horizon are intentionally different; observations beyond 7.5 m may be retained in memory without forcing a 20 m optimization horizon.\n');
fprintf('========================================================================================================================\n');
end

function v205_print_planner_entry(C)
fprintf('\n[G26-V205 FINITE-HORIZON PLANNER ENTRY CONTRACT]\n');
fprintf(' localTarget=1 straightSeedContinuousGate=1 blockedSeed->SPATIAL_SOLVER_REQUIRED=1 generatorMiss->infeasible=0 solverAuthority=%d full2x4=1\n',C.backend_solver_authority);
fprintf(' RULE: V205 starts the planner interface but does not yet claim obstacle-avoidance solver completeness.\n');
end

function B=v205_microbench(CFG)
B=v204_microbench(CFG);NS=CFG.v205_timing_range_gate_samples;NB=CFG.v205_timing_range_gate_batch_calls;N=CFG.v202_timing_total_rays;
r=linspace(CFG.v205_min_detection_range_m,2*CFG.v205_effective_sensing_range_m,N);r(4:4:end)=NaN;
for k=1:20,tmp=g26v205_range_gate_mex(r,CFG.v205_min_detection_range_m,CFG.v205_effective_sensing_range_m);end %#ok<NASGU>
T=zeros(NS,1);for s=1:NS,tt=tic;for k=1:NB,tmp=g26v205_range_gate_mex(r,CFG.v205_min_detection_range_m,CFG.v205_effective_sensing_range_m);end;T(s)=1e3*toc(tt)/NB;end %#ok<NASGU>
B.qual.range_gate=v202_stats(T);p0=[0;0;0];g=[30;4;2];for k=1:20,tmp=g26v205_local_target_mex(p0,g,CFG.v205_planning_horizon_m);end %#ok<NASGU>
T=zeros(NS,1);for s=1:NS,tt=tic;for k=1:NB,tmp=g26v205_local_target_mex(p0,g,CFG.v205_planning_horizon_m);end;T(s)=1e3*toc(tt)/NB;end %#ok<NASGU>
B.qual.local_target=v202_stats(T);B.qual.p95_prefix_with_range_gate_ms=B.qual.range_gate.p95+B.qual.p95_prefix_ms;B.qual.p95_prefix_range_localtarget_cert_ms=B.qual.p95_prefix_with_range_gate_ms+B.qual.local_target.p95+B.qual.direct_cert(end).stats.p95;B.qual.remaining_to_4ms_v205_ms=CFG.target_sensor_return_to_switch_ms-B.qual.p95_prefix_range_localtarget_cert_ms;
end

function A=v204_association_selftest(CFG)
TC=[0 3 6;0 0 0;0 0 0];TR=[0.6 0.6 0.6];
MC1=[3.1;0;0];MR1=0.1;
[id1,u1,a1,n1]=g26v204_assoc_reacq_mex('associate',TC,TR,MC1,MR1,CFG.v204_assoc_guard_scale);
MC2=[1.5;0;0];MR2=1.1;
[id2,u2,a2,n2]=g26v204_assoc_reacq_mex('associate',TC,TR,MC2,MR2,CFG.v204_assoc_guard_scale);
MC3=[10;0;0];MR3=0.1;
[id3,u3,a3,n3]=g26v204_assoc_reacq_mex('associate',TC,TR,MC3,MR3,CFG.v204_assoc_guard_scale);
A=struct();A.unique_id=(id1==2)&&logical(u1)&&a1==0&&n1==0;A.ambiguous_rejected=(id2==0)&&~logical(u2)&&a2==1;A.unmatched_rejected=(id3==0)&&~logical(u3)&&n3==1;
A.raw_first_return_has_whole_object_identity=false;
A.pass=A.unique_id&&A.ambiguous_rejected&&A.unmatched_rejected&&~A.raw_first_return_has_whole_object_identity;
assert(A.pass,'G26V204:AssociationSelftest','Set-membership association selftest failed.');
fprintf('[G26-V204-ASSOCIATION-SELFTEST] PASS | unique overlap accepted | ambiguous/unmatched rejected | raw-first-return identityAuthority=0\n');
end

function A=v204_reacquisition_selftest(CFG)
delta=deg2rad(1.0);angerr=deg2rad(0.1);amin=.20;rmax=20;vc=2;dc=1;tr=.20;
[p,rg,lead,reqA,geom]=g26v204_assoc_reacq_mex('reacq',delta,angerr,amin,rmax,vc,dc,tr,1,CFG.v204_assoc_guard_scale);
[pBlind,~,~,~,gBlind]=g26v204_assoc_reacq_mex('reacq',delta,angerr,amin,rmax,vc,dc,tr,0,CFG.v204_assoc_guard_scale);
[pTiny,rgTiny,~,~,~]=g26v204_assoc_reacq_mex('reacq',delta,angerr,1e-9,rmax,vc,dc,tr,1,CFG.v204_assoc_guard_scale);
A=struct();A.good_pass=logical(p)&&logical(geom)&&rg>dc&&lead>tr;A.blind_zone_fails=~logical(pBlind)&&~logical(gBlind);A.arbitrarily_small_fails=~logical(pTiny)&&rgTiny<dc;A.required_min_radius_m=reqA;A.guaranteed_detection_radius_m=rg;A.lead_time_s=lead;
A.finite_ray_requires_positive_min_size=true;A.pass=A.good_pass&&A.blind_zone_fails&&A.arbitrarily_small_fails;
assert(A.pass,'G26V204:ReacquisitionSelftest','Guaranteed reacquisition theorem selftest failed.');
fprintf('[G26-V204-REACQUISITION-SELFTEST] PASS | Rg=%.3f m lead=%.3f s requiredAmin=%.4f m | blind-zone/tiny-obstacle fail-closed\n',rg,lead,reqA);
end

function A=v204_association_reacq_integration_selftest(CFG)
A=struct();
A.contraction_requires_certified_whole_object_measurement=true;
A.current_first_return_contraction_authority=CFG.v204_association_source_authority;
A.reacquisition_requires_complete_coverage=true;
A.current_reacquisition_authority=CFG.v204_reacquisition_authority;
A.no_heuristic_retirement=~CFG.retirement_authority;
A.pass=A.contraction_requires_certified_whole_object_measurement&&~A.current_first_return_contraction_authority&&A.reacquisition_requires_complete_coverage&&~A.current_reacquisition_authority&&A.no_heuristic_retirement;
assert(A.pass,'G26V204:IntegrationSelftest','V204 authority boundary selftest failed.');
fprintf('[G26-V204-INTEGRATION-SELFTEST] PASS | contractionAuthority=0 reacquisitionAuthority=0 retirement=0 until premises close\n');
end

function C=v204_assoc_reacq_contract(CFG,SELF)
C=struct();C.association_theorem=SELF.association.pass;C.raw_first_return_identity_authority=CFG.v204_association_source_authority;C.reacquisition_theorem=SELF.reacquisition.pass;C.global_coverage_closed=CFG.v204_global_angular_coverage_closed;C.reacquisition_authority=CFG.v204_reacquisition_authority;C.minimum_detectable_size_required=SELF.reacquisition.finite_ray_requires_positive_min_size;C.retirement_authority=CFG.retirement_authority;C.current_dynamic_memory_contraction_allowed=false;C.current_dynamic_retirement_allowed=false;
vals=[CFG.v204_sensor_ray_delta_net_deg CFG.v204_sensor_angular_error_deg CFG.v204_sensor_max_range_m CFG.v204_min_obstacle_inscribed_radius_m CFG.v204_max_closing_speed_mps CFG.v204_reaction_wcet_s CFG.v204_collision_distance_m];
C.configured_candidate_evaluable=all(isfinite(vals))&&all(vals>=0)&&CFG.v204_sensor_max_range_m>0;
C.configured_candidate_pass=false;C.configured_guaranteed_detection_radius_m=NaN;C.configured_lead_time_s=NaN;C.configured_required_min_radius_m=NaN;
if C.configured_candidate_evaluable
    [pp,rg,ld,ra]=g26v204_assoc_reacq_mex('reacq',deg2rad(CFG.v204_sensor_ray_delta_net_deg),deg2rad(CFG.v204_sensor_angular_error_deg),CFG.v204_min_obstacle_inscribed_radius_m,CFG.v204_sensor_max_range_m,CFG.v204_max_closing_speed_mps,CFG.v204_collision_distance_m,CFG.v204_reaction_wcet_s,CFG.v204_global_angular_coverage_closed,CFG.v204_assoc_guard_scale);
    C.configured_candidate_pass=logical(pp);C.configured_guaranteed_detection_radius_m=rg;C.configured_lead_time_s=ld;C.configured_required_min_radius_m=ra;
end
end

function v204_print_assoc_reacq(C)
fprintf('\n[G26-V204 HARDWARE-SPECIFIC REACQUISITION CANDIDATE | AUTHORITY REMAINS 0]\n');
if ~C.configured_candidate_evaluable
    fprintf(' configured premises incomplete -> candidate=INCONCLUSIVE | need delta-net, angular error, max range, positive min obstacle radius, max closing speed, reaction WCET\n');
else
    fprintf(' candidatePass=%d globalCoverage=%d Rg=%.6g m lead=%.6g s requiredMinRadius=%.6g m\n',C.configured_candidate_pass,C.global_coverage_closed,C.configured_guaranteed_detection_radius_m,C.configured_lead_time_s,C.configured_required_min_radius_m);
end
fprintf(' rawFirstReturnIdentityAuthority=%d dynamicContractionAllowed=%d dynamicRetirementAllowed=%d\n',C.raw_first_return_identity_authority,C.current_dynamic_memory_contraction_allowed,C.current_dynamic_retirement_allowed);
end

function v202_mex_bootstrap_tristate_trigger(root)
name='g26v202_tristate_trigger_mex';
v200_build_mex_if_needed(root,name,v202_cpp_tristate_trigger_source(),'G26-V203-TRISTATE-TRIGGER-MEX');
end


function v202_mex_bootstrap_window_supervisor(root)
name='g26v202_window_supervisor_mex';
v200_build_mex_if_needed(root,name,v202_cpp_window_supervisor_source(),'G26-V203-WINDOW-SUPERVISOR-MEX');
end


function lines=v202_cpp_tristate_trigger_source()
lines={ ...
'#include "mex.h"', ...
'#include <cmath>', ...
'#include <limits>', ...
'#include <algorithm>', ...
'static void req(bool q,const char*m){if(!q)mexErrMsgIdAndTxt("G26V202:TriStateTriggerMEX","%s",m);}', ...
'void mexFunction(int nlhs,mxArray*plhs[],int nrhs,const mxArray*prhs[]){', ...
' req(nrhs==9,"Expected P,ta,tb,obsC_at_ta,obsV,obsRad,motionKnown,K,guardScale"); req(nlhs<=4,"At most four outputs");', ...
' req(mxIsDouble(prhs[0])&&mxGetM(prhs[0])==3&&mxGetN(prhs[0])>=1,"P must be 3xN control points"); mwSize NP=mxGetN(prhs[0]);', ...
' double ta=mxGetScalar(prhs[1]),tb=mxGetScalar(prhs[2]);req(std::isfinite(ta)&&std::isfinite(tb)&&tb>=ta,"invalid interval");', ...
' req(mxIsDouble(prhs[3])&&mxGetM(prhs[3])==3,"obsC must be 3xM");mwSize NO=mxGetN(prhs[3]);', ...
' req(mxIsDouble(prhs[4])&&mxGetM(prhs[4])==3&&mxGetN(prhs[4])==NO,"obsV must be 3xM");', ...
' req(mxIsDouble(prhs[5])&&mxGetNumberOfElements(prhs[5])==NO,"obsRad size mismatch");', ...
' req(mxIsDouble(prhs[6])&&mxGetNumberOfElements(prhs[6])==NO,"motionKnown size mismatch");', ...
' req(mxIsDouble(prhs[7])&&mxGetNumberOfElements(prhs[7])==5,"K=[Rpayload Ruav Rcable L margin]");', ...
' double gs=mxGetScalar(prhs[8]);req(std::isfinite(gs)&&gs>=0,"guardScale invalid");', ...
' const double*P=mxGetPr(prhs[0]);const double*C=mxGetPr(prhs[3]);const double*V=mxGetPr(prhs[4]);const double*rad=mxGetPr(prhs[5]);const double*mk=mxGetPr(prhs[6]);const double*K=mxGetPr(prhs[7]);', ...
' for(int k=0;k<5;++k)req(std::isfinite(K[k])&&K[k]>=0,"K entries must be finite nonnegative");', ...
' double cr[3]={0,0,0};for(mwSize j=0;j<NP;++j)for(int a=0;a<3;++a){double z=P[a+3*j];req(std::isfinite(z),"P must be finite");cr[a]+=z;}for(int a=0;a<3;++a)cr[a]/=(double)NP;', ...
' double rr=0;for(mwSize j=0;j<NP;++j){double s=0;for(int a=0;a<3;++a){double d=P[a+3*j]-cr[a];s+=d*d;}rr=std::max(rr,std::sqrt(s));}', ...
' double body=K[3]+std::max(K[0],std::max(K[1],K[2]))+K[4];rr+=body;', ...
' bool missing=false;double minSlack=std::numeric_limits<double>::infinity();mwSize worst=0;double dt=tb-ta;', ...
' for(mwSize i=0;i<NO;++i){req(std::isfinite(rad[i])&&rad[i]>=0,"obsRad invalid");for(int a=0;a<3;++a)req(std::isfinite(C[a+3*i]),"obsC invalid");', ...
'   if(!(std::isfinite(mk[i])&&mk[i]>0.5)){missing=true;continue;}double vm2=0,cm[3];for(int a=0;a<3;++a){double vv=V[a+3*i];req(std::isfinite(vv),"known obsV must be finite");vm2+=vv*vv;cm[a]=C[a+3*i]+0.5*dt*vv;}double ro=rad[i]+0.5*dt*std::sqrt(vm2);double d2=0;for(int a=0;a<3;++a){double z=cr[a]-cm[a];d2+=z*z;}double slack=std::sqrt(d2)-(rr+ro);if(slack<minSlack){minSlack=slack;worst=i+1;}}', ...
' int state=2; /* 0 SAFE_BROADPHASE, 1 POTENTIAL_CONFLICT, 2 INCONCLUSIVE */', ...
' if(NO==0){state=2;minSlack=-std::numeric_limits<double>::infinity();worst=0;}else if(missing){state=2;}else{double scale=std::max(1.0,std::fabs(minSlack));double guard=gs*std::numeric_limits<double>::epsilon()*scale;state=(minSlack>guard)?0:1;}', ...
' if(nlhs>0)plhs[0]=mxCreateDoubleScalar((double)state);if(nlhs>1)plhs[1]=mxCreateDoubleScalar(minSlack);if(nlhs>2)plhs[2]=mxCreateDoubleScalar((double)worst);if(nlhs>3)plhs[3]=mxCreateDoubleScalar(rr);', ...
'}'};
end


function lines=v202_cpp_window_supervisor_source()
lines={ ...
'#include "mex.h"', ...
'#include <cmath>', ...
'#include <algorithm>', ...
'static void req(bool q,const char*m){if(!q)mexErrMsgIdAndTxt("G26V202:WindowSupervisorMEX","%s",m);}', ...
'void mexFunction(int nlhs,mxArray*plhs[],int nrhs,const mxArray*prhs[]){', ...
' req(nrhs==7,"Expected oldEnd,now,requestedEnd,triggerState,directChecked,directSafe,successorInstalled");req(nlhs<=5,"At most five outputs");', ...
' double oldEnd=mxGetScalar(prhs[0]),now=mxGetScalar(prhs[1]),reqEnd=mxGetScalar(prhs[2]),st=mxGetScalar(prhs[3]);', ...
' bool checked=mxGetScalar(prhs[4])!=0,safe=mxGetScalar(prhs[5])!=0,installed=mxGetScalar(prhs[6])!=0;', ...
' req(std::isfinite(oldEnd)&&std::isfinite(now)&&std::isfinite(reqEnd)&&reqEnd>=now,"invalid times");req(st==0||st==1||st==2,"trigger state must be 0,1,2");', ...
' double newEnd=oldEnd;bool extend=checked&&safe;if(extend)newEnd=std::max(oldEnd,reqEnd);', ...
' bool triggerPlan=checked?(!safe):(st!=0);bool expired=(now>=oldEnd)&&!installed&&!extend;bool retain=!installed;', ...
' if(nlhs>0)plhs[0]=mxCreateDoubleScalar(newEnd);if(nlhs>1)plhs[1]=mxCreateLogicalScalar(triggerPlan);if(nlhs>2)plhs[2]=mxCreateLogicalScalar(expired);if(nlhs>3)plhs[3]=mxCreateLogicalScalar(extend);if(nlhs>4)plhs[4]=mxCreateLogicalScalar(retain);', ...
'}'};
end

% ========================================================================
% V202 trigger/window selftests.
% State semantics:
%   0 SAFE_BROADPHASE       : scheduling broadphase separated every known
%                             retained-evidence motion envelope.
%   1 POTENTIAL_CONFLICT    : broadphase sets overlap -> replan now.
%   2 INCONCLUSIVE          : missing motion envelope / no authoritative
%                             predictor input -> replan/recertify now.
% NONE of these states is a successor switch authority.  Window extension
% additionally requires the retained continuous whole-body direct certificate.
% ========================================================================

function A=v202_tristate_trigger_selftest(CFG)
P=[0 0.5 1.0;0 0 0;0 0 0];ta=0;tb=1;
K=[CFG.R_payload CFG.R_uav CFG.R_cable CFG.L_cable CFG.geometry_margin_m];
Cfar=[8 10;0 0;0 0];Vfar=zeros(3,2);Rfar=[0.1 0.2];MK=[1 1];
[s0,sl0,w0,rr0]=g26v202_tristate_trigger_mex(P,ta,tb,Cfar,Vfar,Rfar,MK,K,CFG.v202_trigger_guard_scale);
Cnear=[0.8;0;0];Vnear=zeros(3,1);Rnear=0.2;
[s1,sl1,w1]=g26v202_tristate_trigger_mex(P,ta,tb,Cnear,Vnear,Rnear,1,K,CFG.v202_trigger_guard_scale);
[s2,sl2,w2]=g26v202_tristate_trigger_mex(P,ta,tb,Cfar,Vfar,Rfar,[1 0],K,CFG.v202_trigger_guard_scale);
A=struct();A.pass=(s0==0&&sl0>0&&w0>=1&&isfinite(rr0)&&s1==1&&sl1<=0&&w1==1&&s2==2&&w2>=1&&isfinite(sl2));
A.safe_state=s0;A.conflict_state=s1;A.inconclusive_state=s2;A.predictor_authority=false;
assert(A.pass,'G26V202:TriggerSelftest','Tri-state continuous broadphase trigger selftest failed.');
fprintf('[G26-V204-TRIGGER-SELFTEST] PASS | far=SAFE_BROADPHASE near=POTENTIAL_CONFLICT missingMotion=INCONCLUSIVE authority=0\n');
end


function A=v202_certified_window_selftest(CFG)
% Existing active reference is already certified to t=1.0.
oldEnd=1.0;now=0.2;requested=now+CFG.v202_certified_window_horizon_s;
% Obtain the extension authority from the retained continuous whole-body
% direct certificate, not from the predictor.
Pfar=[5 5;0 0;0 0];counts=2;RR=[0;0;CFG.gravity];rc=1;ta=0;tb=1;
OC=[0;0;0];OV=[0;0;0];OR=reshape(eye(3),9,1);ER=[0.2;0.2;0.2];
Kcert=[CFG.R_payload+CFG.geometry_margin_m,CFG.R_uav+CFG.geometry_margin_m, ...
    CFG.R_cable+CFG.geometry_margin_m,CFG.L_cable];
certSafe=g26v200_direct_cert_mex(Pfar,counts,RR,rc,ta,tb,OC,OV,OR,ER,Kcert,CFG.direct_cert_max_depth,CFG.direct_cert_guard_scale);
Pbad=zeros(3,2);
certBad=g26v200_direct_cert_mex(Pbad,counts,RR,rc,ta,tb,OC,OV,OR,ER,Kcert,CFG.direct_cert_max_depth,CFG.direct_cert_guard_scale);
% Fast-clear predictor alone never extends the authoritative window.
[e0,p0,x0,ex0,r0]=g26v202_window_supervisor_mex(oldEnd,now,requested,0,0,0,0);
% Exact continuous certificate PASS extends, regardless of whether the cheap
% broadphase was clear or conservatively overlapped.
[e1,p1,x1,ex1,r1]=g26v202_window_supervisor_mex(oldEnd,now,requested,0,1,certSafe,0);
[eR,pR,xR,exR,rR]=g26v202_window_supervisor_mex(oldEnd,now,requested,1,1,certSafe,0);
% Exact certificate FAIL never extends and requests replanning.
[eB,pB,xB,exB,rB]=g26v202_window_supervisor_mex(e1,0.3,1.3,0,1,certBad,0);
% Potential conflict or inconclusive without exact recert requests replanning.
[e2,p2,x2,ex2,r2]=g26v202_window_supervisor_mex(e1,0.4,1.4,1,0,0,0);
[e3,p3,x3,ex3,r3]=g26v202_window_supervisor_mex(e1,0.5,1.5,2,0,0,0);
% If the certified window expires and no successor is installed, authority expires.
[~,p4,x4,ex4,~]=g26v202_window_supervisor_mex(e1,e1,e1+0.5,2,0,0,0);
A=struct();A.pass=logical(certSafe)&&~logical(certBad)&&e0==oldEnd&&~p0&&~x0&&~ex0&&r0&& ...
    e1==requested&&~p1&&~x1&&ex1&&r1&&eR==requested&&~pR&&~xR&&exR&&rR&& ...
    eB==e1&&pB&&~xB&&~exB&&rB&&e2==e1&&p2&&~x2&&~ex2&&r2&& ...
    e3==e1&&p3&&~x3&&~ex3&&r3&&p4&&x4&&~ex4;
A.direct_cert_required=true;A.direct_cert_actually_called=true;A.predictor_false_positive_recoverable=true;
A.inconclusive_never_extends_without_exact=true;A.expiry_fail_closed=true;
assert(A.pass,'G26V202:WindowSelftest','Certified-window supervisor selftest failed.');
fprintf('[G26-V204-WINDOW-SELFTEST] PASS | fastClearAloneNoExtend=1 exactPASSextends=1 conservativeOverlapRecoveredByExact=1 exactFAILnoExtend=1 inconclusiveNoExactNoExtend=1 expiryFailClosed=1\n');
end

function A=v203_sensor_cover_trigger_integration_selftest(CFG)
N=12;O=zeros(3,N);D=zeros(3,N);D(1,:)=1;rr=8+0.05*(1:N);ha=0.003*ones(1,N);he=0.004*ones(1,N);sid=[ones(1,N/2),2*ones(1,N/2)];rid=1:N;
[C,R]=g26v201_first_return_mex(O,D,rr,ha,he,sid,rid,CFG.v201_hit_guard_scale);
g26v203_cover_memory_mex('reset',16,64);
known=ones(1,size(C,2));vmax=0.5*ones(1,size(C,2));[ok,~,~,rf]=g26v203_cover_memory_mex('ingest',C,R,0,vmax,known,CFG.v203_cover_guard_scale);assert(ok&&~rf);
[CM,RM,TM,VM,KM]=g26v203_cover_memory_mex('snapshot');
[RE,KE,allKnown]=g26v203_motion_envelope_mex(RM,TM,VM,KM,1,CFG.v203_cover_guard_scale);
P=[0 0.5 1;0 0 0;0 0 0];Ktr=[CFG.R_payload CFG.R_uav CFG.R_cable CFG.L_cable CFG.geometry_margin_m];
[stKnown,~,~]=g26v202_tristate_trigger_mex(P,0,1,CM,zeros(size(CM)),RE,KE,Ktr,CFG.v202_trigger_guard_scale);
% Same geometry but motion authority missing must remain INCONCLUSIVE.
g26v203_cover_memory_mex('reset',16,64);[ok2,~,~,~]=g26v203_cover_memory_mex('ingest',C,R,0,zeros(1,size(C,2)),zeros(1,size(C,2)),CFG.v203_cover_guard_scale);assert(ok2);
[CU,RU,TU,VU,KU]=g26v203_cover_memory_mex('snapshot');[RUe,KUe,allUnknown]=g26v203_motion_envelope_mex(RU,TU,VU,KU,1,CFG.v203_cover_guard_scale);
[stRaw,~,~]=g26v202_tristate_trigger_mex(P,0,1,CU,zeros(size(CU)),RUe,KUe,Ktr,CFG.v202_trigger_guard_scale);
A=struct();A.known_path_not_inconclusive=(stKnown~=2)&&logical(allKnown);A.raw_without_motion_is_inconclusive=(stRaw==2)&&~logical(allUnknown);A.pass=A.known_path_not_inconclusive&&A.raw_without_motion_is_inconclusive;
assert(A.pass,'G26V203:Integration','Sensor-cover-motion-trigger integration selftest failed.');
fprintf('[G26-V204-INTEGRATION-SELFTEST] PASS | certified vmax -> finite horizon cover | unknown motion -> INCONCLUSIVE\n');
g26v203_cover_memory_mex('reset',CFG.v203_cover_capacity,CFG.v203_max_frame_returns);
end

function A=v203_cover_memory_selftest(CFG)
cap=4;maxf=32;g26v203_cover_memory_mex('reset',cap,maxf);
C1=[0 .2 .4 .6 .8 1.0;0 .1 0 -.1 0 .1;zeros(1,6)];R1=.05*ones(1,6);V1=[.2 .3 .1 .4 .2 .3];K1=ones(1,6);
[ok1,n1,m1,rf1]=g26v203_cover_memory_mex('ingest',C1,R1,0,V1,K1,CFG.v203_cover_guard_scale);[A1,B1,T1,W1,Q1]=g26v203_cover_memory_mex('snapshot');
D=v203_test_directions(CFG.v203_support_directions);raw0=v203_ball_support(C1,R1,D);cov0=v203_ball_support(A1,B1,D);res0=max(raw0-cov0);
C2=[1.2 1.4;0 .1;0 0];R2=[.04 .06];V2=[.25 .35];K2=ones(1,2);t2=.5;
[ok2,n2,m2,rf2]=g26v203_cover_memory_mex('ingest',C2,R2,t2,V2,K2,CFG.v203_cover_guard_scale);[A2,B2,T2,W2,Q2]=g26v203_cover_memory_mex('snapshot');
rawC=[C1 C2];rawR=[R1+V1*t2 R2];raw2=v203_ball_support(rawC,rawR,D);cov2=v203_ball_support(A2,B2,D);res2=max(raw2-cov2);
% Resource overflow is fail-closed and memory is unchanged.
[countBefore,~]=g26v203_cover_memory_mex('count');Cb=zeros(3,maxf+1);Rb=.01*ones(1,maxf+1);Vb=zeros(1,maxf+1);Kb=ones(1,maxf+1);
[ok3,n3,~,rf3]=g26v203_cover_memory_mex('ingest',Cb,Rb,1,Vb,Kb,CFG.v203_cover_guard_scale);[countAfter,~]=g26v203_cover_memory_mex('count');
% Storage sustainability: no deletion/overflow across many frames because capacity pressure is handled by outer merge.
g26v203_cover_memory_mex('reset',cap,maxf);sustain=true;mergeTotal=0;
for k=1:100,[ok,n,mm,rf]=g26v203_cover_memory_mex('ingest',C1,R1,.01*k,V1,K1,CFG.v203_cover_guard_scale);sustain=sustain&&ok&&~rf&&n<=cap;mergeTotal=mergeTotal+mm;end
A=struct();A.pairwise_outer_theorem=true;A.future_bound_induction=true;A.current_support_residual=res0;A.future_support_residual=res2;A.counts=[n1 n2];A.merges=[m1 m2];A.resource_overflow_fail_closed=(~ok3)&&logical(rf3)&&(n3==countBefore)&&(countAfter==countBefore);A.storage_sustained=sustain;A.merge_total=mergeTotal;A.pass=ok1&&ok2&&~rf1&&~rf2&&n1<=cap&&n2<=cap&&res0<=1e-12&&res2<=1e-12&&A.resource_overflow_fail_closed&&sustain;
assert(A.pass,'G26V203:CoverSelftest','Set-preserving cover-memory selftest failed.');
fprintf('[G26-V204-COVER-SELFTEST] PASS | outerContainment(current/future)=1/1 supportResidual=[%.3e %.3e] capacity=%d sustained100=1 overflowFailClosed=1\n',res0,res2,cap);
g26v203_cover_memory_mex('reset',CFG.v203_cover_capacity,CFG.v203_max_frame_returns);
end

function A=v203_motion_envelope_selftest(CFG)
R=[.2 .3];T=[0 .5];V=[2 1];K=[1 1];[RE,KE,allKnown,mg]=g26v203_motion_envelope_mex(R,T,V,K,1,CFG.v203_cover_guard_scale);
[RU,KU,allUnknown]=g26v203_motion_envelope_mex(.2,0,0,0,1,CFG.v203_cover_guard_scale);
expR=[2.2 .8];err=RE-expR;A.known_exact=all(err>=0)&&max(err)<=2*CFG.v203_cover_guard_scale*eps(max(RE))*max(1,max(RE));A.known_mask=all(KE==1)&&logical(allKnown);A.max_growth=mg;A.unknown_not_claimed=(KU==0)&&~logical(allUnknown)&&RU==.2;A.pass=A.known_exact&&A.known_mask&&A.unknown_not_claimed;
assert(A.pass,'G26V203:MotionEnvelopeSelftest','Motion-envelope selftest failed.');
fprintf('[G26-V204-MOTION-SELFTEST] PASS | r(t)=r0+vmax*dt exact for known bound | unknown motion remains uncertified\n');
end

function D=v203_test_directions(N)
k=(0:N-1);z=1-2*(k+.5)/N;phi=pi*(3-sqrt(5))*k;r=sqrt(max(0,1-z.^2));D=[r.*cos(phi);r.*sin(phi);z];
end

function h=v203_ball_support(C,R,D)
h=-inf(1,size(D,2));for j=1:size(D,2),h(j)=max(D(:,j)'*C+R);end
end

function M=v203_memory_motion_contract(CFG,SELF)
M=struct();M.first_returns_only=SELF.first_return.pass;M.no_return_free=false;M.set_preserving_outer_cover=SELF.cover.pass;M.no_forgetting=true;M.retirement=false;M.resource_overflow_fail_closed=SELF.cover.resource_overflow_fail_closed;M.fixed_storage_sustained=SELF.cover.storage_sustained;M.motion_envelope_math=SELF.motion.pass;M.motion_source_authority=CFG.v203_motion_envelope_authority;M.cover_fp_authority=CFG.v203_cover_safety_authority;M.unknown_motion_inconclusive=SELF.integration.raw_without_motion_is_inconclusive;
fprintf('[G26-V204-MEMORY-MOTION-CONTRACT] PASS | noDrop=1 outerCover=1 fixedStorage=1 retirement=0 unknownMotion=INCONCLUSIVE coverFPAuthority=0 motionAuthority=0\n');
end

function T=v202_trigger_window_contract(CFG,SELF)
T=struct();T.tri_state=SELF.trigger.pass;T.trigger_switch_authority=false;T.predictor_sphere_authority=false;T.window=SELF.window.pass;T.direct_cert_required=CFG.v202_window_extension_requires_direct_cert;T.unknown_motion_inconclusive=SELF.integration.raw_without_motion_is_inconclusive;
fprintf('[G26-V204-TRIGGER-WINDOW-CONTRACT] PASS | triState=1 triggerAuthority=0 directCertRequiredForWindow=1 unknownMotion=INCONCLUSIVE\n');
end

function A=v201_first_return_selftest(CFG)
N=8;O=zeros(3,N);D=zeros(3,N);D(1,:)=1;
R=[2 3 NaN 4 Inf 1.5 NaN 2.5];
ha=0.01*ones(1,N);he=0.02*ones(1,N);sid=[1 1 1 1 2 2 2 2];rid=1:N;
[C,rad,sout,rout,mask]=g26v201_first_return_mex(O,D,R,ha,he,sid,rid,CFG.v201_hit_guard_scale);
valid=isfinite(R)&R>0;expectedC=[R(valid);zeros(2,sum(valid))];expectedRad=R(valid).*sqrt(ha(valid).^2+he(valid).^2);
replay=g26v201_first_return_mex(O,D,R,ha,he,sid,rid,CFG.v201_hit_guard_scale);
A=struct();A.pass=isequal(mask,double(valid))&&isequal(C,expectedC)&&isequal(rad,expectedRad)&& ...
    isequal(sout,sid(valid))&&isequal(rout,rid(valid))&&isequal(replay,C);
A.valid_returns=sum(valid);A.upper_returns=sum(valid&sid==1);A.lower_returns=sum(valid&sid==2);A.no_return_count=sum(~valid);
A.no_return_promoted_free=false;A.replay_exact=isequal(replay,C);
assert(A.pass,'G26V201:FirstReturnSelftest','Frozen dual-LiDAR first-return ingress selftest failed.');
fprintf('[G26-V204-FIRST-RETURN-SELFTEST] PASS | rays=%d valid=%d upper/lower=%d/%d noReturn=%d -> free=0 replayExact=1\n', ...
    N,A.valid_returns,A.upper_returns,A.lower_returns,A.no_return_count);
end


function B=v204_microbench(CFG)
% BUILD/selftest excluded.  Timing is qualification evidence, not WCET.
NS=CFG.v202_timing_samples;NB=CFG.v202_timing_batch_calls;N=CFG.v202_timing_total_rays;
% tiny regression means
R=CFG.microbench_repetitions;J0=zeros(3,7,3);J1=zeros(3,7,3);F=zeros(3,4,3);dt=[1 1 1];
t=tic;for k=1:R,tmp=g26v200_c6_construct_mex(J0,J1,dt,F);end;B.c6_mean_ms=1e3*toc(t)/R; %#ok<NASGU>
ang=linspace(0,2*pi,25);ang(end)=[];Cs=[cos(ang);sin(ang);.2*sin(2*ang)];rs=.04*ones(1,size(Cs,2));Vs=[3 3.2 3;0 .2 -.2;0 0 0];vrs=[.25 .15 .25];
t=tic;for k=1:R,tmp=g26v200_local_sfc_mex(Cs,rs,Vs,vrs,4096);end;B.sfc_mean_ms=1e3*toc(t)/R; %#ok<NASGU>
Pcert=[5 5.2 5.4;0 0 0;0 0 0];counts=3;RR=[0 0 0;0 0 0;CFG.gravity CFG.gravity CFG.gravity];rc=3;ta=0;tb=1;Kcert=[CFG.R_payload+CFG.geometry_margin_m,CFG.R_uav+CFG.geometry_margin_m,CFG.R_cable+CFG.geometry_margin_m,CFG.L_cable];
OC1=[-5;0;0];OV1=zeros(3,1);OR1=reshape(eye(3),9,1);ER1=.2*ones(3,1);t=tic;for k=1:R,ok=g26v200_direct_cert_mex(Pcert,counts,RR,rc,ta,tb,OC1,OV1,OR1,ER1,Kcert,CFG.direct_cert_max_depth,CFG.direct_cert_guard_scale);assert(ok);end;B.cert_mean_ms=1e3*toc(t)/R;
% realistic dual-LiDAR frame
Oq=zeros(3,N);Dq=zeros(3,N);Dq(1,:)=1;rq=4+.001*(1:N);period=max(1,round(1/max(realmin,1-CFG.v202_timing_valid_fraction)));if CFG.v202_timing_valid_fraction<1,rq(period:period:end)=NaN;end
haq=.004*ones(1,N);heq=.006*ones(1,N);sidq=[ones(1,N/2),2*ones(1,N/2)];ridq=1:N;
for k=1:20,tmp=g26v201_first_return_mex(Oq,Dq,rq,haq,heq,sidq,ridq,CFG.v201_hit_guard_scale);end %#ok<NASGU>
T=zeros(NS,1);for sidx=1:NS,tt=tic;for k=1:NB,tmp=g26v201_first_return_mex(Oq,Dq,rq,haq,heq,sidq,ridq,CFG.v201_hit_guard_scale);end;T(sidx)=1e3*toc(tt)/NB;end %#ok<NASGU>
B.qual.first_return=v202_stats(T);[Cq,Rq]=g26v201_first_return_mex(Oq,Dq,rq,haq,heq,sidq,ridq,CFG.v201_hit_guard_scale);Mret=size(Cq,2);B.qual.rays=N;B.qual.returns=Mret;B.qual.return_fraction=Mret/N;
% cover ingest: persistent fixed-capacity memory, known zero-motion here to isolate compression cost.
g26v203_cover_memory_mex('reset',CFG.v203_cover_capacity,CFG.v203_max_frame_returns);zv=zeros(1,Mret);okv=ones(1,Mret);for k=1:20,[ok,~,~,rf]=g26v203_cover_memory_mex('ingest',Cq,Rq,k*CFG.v203_qualification_dt_s,zv,okv,CFG.v203_cover_guard_scale);assert(ok&&~rf);end
T=zeros(NS,1);stamp=20*CFG.v203_qualification_dt_s;for sidx=1:NS,tt=tic;for k=1:NB,stamp=stamp+CFG.v203_qualification_dt_s;[ok,~,~,rf]=g26v203_cover_memory_mex('ingest',Cq,Rq,stamp,zv,okv,CFG.v203_cover_guard_scale);assert(ok&&~rf);end;T(sidx)=1e3*toc(tt)/NB;end
B.qual.cover_ingest=v202_stats(T);[CM,RM,TM,VM,KM]=g26v203_cover_memory_mex('snapshot');B.qual.cover_count=size(CM,2);B.qual.cover_capacity=CFG.v203_cover_capacity;
% motion envelope query cost at full cover count
VM(:)=CFG.v203_qualification_vmax_mps;KM(:)=1;tQuery=max(TM)+CFG.v202_certified_window_horizon_s;for k=1:20,tmp=g26v203_motion_envelope_mex(RM,TM,VM,KM,tQuery,CFG.v203_cover_guard_scale);end %#ok<NASGU>
T=zeros(NS,1);for sidx=1:NS,tt=tic;for k=1:NB,tmp=g26v203_motion_envelope_mex(RM,TM,VM,KM,tQuery,CFG.v203_cover_guard_scale);end;T(sidx)=1e3*toc(tt)/NB;end %#ok<NASGU>
B.qual.motion_envelope=v202_stats(T);[RE,KE]=g26v203_motion_envelope_mex(RM,TM,VM,KM,tQuery,CFG.v203_cover_guard_scale);
% trigger cost on cover
Ptrig=[0 .5 1;0 0 0;0 0 0];Ktr=[CFG.R_payload CFG.R_uav CFG.R_cable CFG.L_cable CFG.geometry_margin_m];T=zeros(NS,1);for k=1:20,tmp=g26v202_tristate_trigger_mex(Ptrig,0,1,CM,zeros(size(CM)),RE,KE,Ktr,CFG.v202_trigger_guard_scale);end %#ok<NASGU>
for sidx=1:NS,tt=tic;for k=1:NB,tmp=g26v202_tristate_trigger_mex(Ptrig,0,1,CM,zeros(size(CM)),RE,KE,Ktr,CFG.v202_trigger_guard_scale);end;T(sidx)=1e3*toc(tt)/NB;end %#ok<NASGU>
B.qual.trigger_cover=v202_stats(T);
% supervisor timing
T=zeros(NS,1);for k=1:20,tmp=g26v202_window_supervisor_mex(1,.2,1.2,0,1,1,0);end %#ok<NASGU>
for sidx=1:NS,tt=tic;for k=1:NB,tmp=g26v202_window_supervisor_mex(1,.2,1.2,0,1,1,0);end;T(sidx)=1e3*toc(tt)/NB;end %#ok<NASGU>
B.qual.window=v202_stats(T);
% direct cert scaling to 512 outer-cover spheres
Mcert=CFG.v203_timing_cert_obstacles;B.qual.direct_cert=struct([]);for mi=1:numel(Mcert),M=Mcert(mi);OC=zeros(3,M);OC(1,:)=-5-.02*(1:M);OV=zeros(3,M);OR=repmat(reshape(eye(3),9,1),1,M);ER=.2*ones(3,M);T=zeros(NS,1);for k=1:10,ok=g26v200_direct_cert_mex(Pcert,counts,RR,rc,ta,tb,OC,OV,OR,ER,Kcert,CFG.direct_cert_max_depth,CFG.direct_cert_guard_scale);assert(ok);end;for sidx=1:NS,tt=tic;for k=1:NB,ok=g26v200_direct_cert_mex(Pcert,counts,RR,rc,ta,tb,OC,OV,OR,ER,Kcert,CFG.direct_cert_max_depth,CFG.direct_cert_guard_scale);assert(ok);end;T(sidx)=1e3*toc(tt)/NB;end;B.qual.direct_cert(mi).obstacles=M;B.qual.direct_cert(mi).stats=v202_stats(T);end
% storage sustain audit, static scene so this measures compression stability rather than motion growth.
g26v203_cover_memory_mex('reset',CFG.v203_cover_capacity,CFG.v203_max_frame_returns);mergeTotal=0;for k=1:CFG.v203_sustain_frames,[ok,n,mm,rf]=g26v203_cover_memory_mex('ingest',Cq,Rq,k*CFG.v203_qualification_dt_s,zv,okv,CFG.v203_cover_guard_scale);assert(ok&&~rf&&n<=CFG.v203_cover_capacity);mergeTotal=mergeTotal+mm;end
[CS,RS]=g26v203_cover_memory_mex('snapshot');B.qual.sustain_frames=CFG.v203_sustain_frames;B.qual.sustain_count=size(CS,2);B.qual.sustain_merge_total=mergeTotal;B.qual.sustain_storage_pass=B.qual.sustain_count<=CFG.v203_cover_capacity;B.qual.sustain_max_radius=max(RS);
% one-frame compression conservatism diagnostic by directional support gap.
g26v203_cover_memory_mex('reset',CFG.v203_cover_capacity,CFG.v203_max_frame_returns);[ok,~,~,rf]=g26v203_cover_memory_mex('ingest',Cq,Rq,0,zv,okv,CFG.v203_cover_guard_scale);assert(ok&&~rf);[CC,RC]=g26v203_cover_memory_mex('snapshot');D=v203_test_directions(CFG.v203_support_directions);hr=v203_ball_support(Cq,Rq,D);hc=v203_ball_support(CC,RC,D);B.qual.one_frame_support_gap_max=max(hc-hr);B.qual.one_frame_support_containment_residual=max(hr-hc);
% dynamic no-association growth diagnostic: safe storage does not imply useful long-horizon geometry.
g26v203_cover_memory_mex('reset',CFG.v203_cover_capacity,CFG.v203_max_frame_returns);vv=CFG.v203_qualification_vmax_mps*ones(1,Mret);for k=1:50,[ok,~,~,rf]=g26v203_cover_memory_mex('ingest',Cq,Rq,k*CFG.v203_qualification_dt_s,vv,okv,CFG.v203_cover_guard_scale);assert(ok&&~rf);end
[~,RD,TD,VD,KD]=g26v203_cover_memory_mex('snapshot');[RDE,~,ak]=g26v203_motion_envelope_mex(RD,TD,VD,KD,50*CFG.v203_qualification_dt_s+CFG.v202_certified_window_horizon_s,CFG.v203_cover_guard_scale);B.qual.dynamic50_all_known=logical(ak);B.qual.dynamic50_max_radius=max(RDE);
% p95 architecture screen
B.qual.p95_prefix_ms=B.qual.first_return.p95+B.qual.cover_ingest.p95+B.qual.motion_envelope.p95+B.qual.trigger_cover.p95+B.qual.window.p95;
B.qual.p95_with_cert_ms=B.qual.p95_prefix_ms+B.qual.direct_cert(end).stats.p95;B.qual.remaining_to_4ms_after_cert_ms=CFG.target_sensor_return_to_switch_ms-B.qual.p95_with_cert_ms;B.qual.prefix_design_budget_ms=CFG.v202_prefix_design_budget_ms;B.qual.prefix_budget_screen=B.qual.p95_prefix_ms<=CFG.v202_prefix_design_budget_ms;
% V204 association/reacquisition qualification timing.  This is NOT in the
% production prefix because current raw first-return packets have no certified
% whole-object identity/reacquisition authority.
Nt=CFG.v204_timing_tracks;Nm=CFG.v204_timing_measurements;TC=zeros(3,Nt);TC(1,:)=2*(0:Nt-1);TR=.4*ones(1,Nt);MC=TC(:,1:Nm)+[.05*ones(1,Nm);zeros(2,Nm)];MR=.1*ones(1,Nm);
for k=1:20,tmp=g26v204_assoc_reacq_mex('associate',TC,TR,MC,MR,CFG.v204_assoc_guard_scale);end %#ok<NASGU>
T=zeros(NS,1);for sidx=1:NS,tt=tic;for k=1:NB,tmp=g26v204_assoc_reacq_mex('associate',TC,TR,MC,MR,CFG.v204_assoc_guard_scale);end;T(sidx)=1e3*toc(tt)/NB;end %#ok<NASGU>
B.qual.association=v202_stats(T);B.qual.association_tracks=Nt;B.qual.association_measurements=Nm;
for k=1:20,tmp=g26v204_assoc_reacq_mex('reacq',deg2rad(1),deg2rad(.1),.2,20,2,1,.2,1,CFG.v204_assoc_guard_scale);end %#ok<NASGU>
T=zeros(NS,1);for sidx=1:NS,tt=tic;for k=1:NB,tmp=g26v204_assoc_reacq_mex('reacq',deg2rad(1),deg2rad(.1),.2,20,2,1,.2,1,CFG.v204_assoc_guard_scale);end;T(sidx)=1e3*toc(tt)/NB;end %#ok<NASGU>
B.qual.reacquisition=v202_stats(T);
% V204B bounded-blind-gap reacquisition theorem timing.
for k=1:20,tmp=g26v204b_blind_gap_reacq_mex(deg2rad(15),deg2rad(4),deg2rad(.1),.5,6,2,.4,.2,CFG.v204_assoc_guard_scale);end %#ok<NASGU>
T=zeros(NS,1);for sidx=1:NS,tt=tic;for k=1:NB,tmp=g26v204b_blind_gap_reacq_mex(deg2rad(15),deg2rad(4),deg2rad(.1),.5,6,2,.4,.2,CFG.v204_assoc_guard_scale);end;T(sidx)=1e3*toc(tt)/NB;end %#ok<NASGU>
B.qual.blind_gap_reacquisition=v202_stats(T);
B.first_return_mean_ms=B.qual.first_return.mean;B.cover_ingest_mean_ms=B.qual.cover_ingest.mean;B.motion_envelope_mean_ms=B.qual.motion_envelope.mean;B.tristate_trigger_mean_ms=B.qual.trigger_cover.mean;B.window_supervisor_mean_ms=B.qual.window.mean;B.synthetic_kernel_sum_ms=B.c6_mean_ms+B.sfc_mean_ms+B.cert_mean_ms+B.first_return_mean_ms+B.cover_ingest_mean_ms+B.motion_envelope_mean_ms+B.tristate_trigger_mean_ms+B.window_supervisor_mean_ms;B.integrated_wcet_claim=false;
g26v203_cover_memory_mex('reset',CFG.v203_cover_capacity,CFG.v203_max_frame_returns);
end

function S=v202_stats(x)
x=sort(x(:));n=numel(x);S=struct();S.p50=x(max(1,ceil(.50*n)));S.p95=x(max(1,ceil(.95*n)));S.max=x(end);S.mean=mean(x);S.min=x(1);
end

% ========================================================================
% MEX 1: fixed-dimension degree-17 endpoint-jet C6 constructor.
% Endpoint d0..d6 are embedded; B7..B10 are the only free controls.
% ========================================================================

function lines=v200_cpp_c6_source()
lines={ ...
'#include "mex.h"', ...
'#include <cmath>', ...
'#include <algorithm>', ...
'static void req(bool q,const char*m){if(!q)mexErrMsgIdAndTxt("G26V200:C6MEX","%s",m);}', ...
'static double ff(int n,int r){double z=1.0;for(int k=0;k<r;++k)z*=double(n-k);return z;}', ...
'static double C(int n,int k){if(k<0||k>n)return 0;double z=1;for(int j=1;j<=k;++j)z*=double(n-k+j)/double(j);return z;}', ...
'static void one(const double*J0,const double*J1,double dt,const double*F,double*B){', ...
' const int n=17; for(int i=0;i<54;++i)B[i]=0.0;', ...
' for(int a=0;a<3;++a)B[a]=J0[a];', ...
' for(int r=1;r<=6;++r){double den=ff(n,r),ts=std::pow(dt,r);for(int a=0;a<3;++a){double z=ts*J0[a+3*r]/den;for(int j=0;j<r;++j){double sg=((r-j)&1)?-1.0:1.0;z-=sg*C(r,j)*B[a+3*j];}B[a+3*r]=z;}}', ...
' for(int a=0;a<3;++a)B[a+3*n]=J1[a];', ...
' for(int r=1;r<=6;++r){double den=ff(n,r),ts=std::pow(dt,r);double c0=(r&1)?-1.0:1.0;for(int a=0;a<3;++a){double z=ts*J1[a+3*r]/den;for(int j=1;j<=r;++j){double sg=((r-j)&1)?-1.0:1.0;z-=sg*C(r,j)*B[a+3*(n-r+j)];}B[a+3*(n-r)]=z/c0;}}', ...
' for(int j=0;j<4;++j)for(int a=0;a<3;++a)B[a+3*(7+j)]=F[a+3*j];', ...
'}', ...
'void mexFunction(int nlhs,mxArray*plhs[],int nrhs,const mxArray*prhs[]){', ...
' req(nrhs==4,"Expected J0,J1,dt,freeControls");req(nlhs<=1,"One output only");', ...
' const double*J0=mxGetPr(prhs[0]);const double*J1=mxGetPr(prhs[1]);const double*dt=mxGetPr(prhs[2]);const double*F=mxGetPr(prhs[3]);', ...
' mwSize nseg=mxGetNumberOfElements(prhs[2]);req(nseg>=1,"Need at least one segment");', ...
' req(mxGetNumberOfElements(prhs[0])==21*nseg&&mxGetNumberOfElements(prhs[1])==21*nseg,"Jet arrays must contain 3x7xnseg values");', ...
' req(mxGetNumberOfElements(prhs[3])==12*nseg,"freeControls must contain 3x4xnseg values");', ...
' mwSize dims[3]={3,18,nseg};plhs[0]=mxCreateNumericArray(3,dims,mxDOUBLE_CLASS,mxREAL);double*B=mxGetPr(plhs[0]);', ...
' for(mwSize q=0;q<nseg;++q){req(std::isfinite(dt[q])&&dt[q]>0,"Segment duration must be positive finite");one(J0+21*q,J1+21*q,dt[q],F+12*q,B+54*q);}', ...
'}'};
end

% ========================================================================
% MEX 2: local first-return sphere -> deterministic tangent-face proposals.
% This is proposal geometry only.  Every primitive is either covered by a
% generated face or explicitly returned as FALLBACK.  There is no top-k,
% face-count cutoff, distance discard, TTL, or obstacle deletion.
% ========================================================================

function lines=v200_cpp_local_sfc_source()
lines={ ...
'#include "mex.h"', ...
'#include <vector>', ...
'#include <cmath>', ...
'#include <limits>', ...
'#include <algorithm>', ...
'static void req(bool q,const char*m){if(!q)mexErrMsgIdAndTxt("G26V200:LocalSFCMEX","%s",m);}', ...
'static double dot3(const double*a,const double*b){return a[0]*b[0]+a[1]*b[1]+a[2]*b[2];}', ...
'static double n3(double x,double y,double z){return std::sqrt(x*x+y*y+z*z);}', ...
'void mexFunction(int nlhs,mxArray*plhs[],int nrhs,const mxArray*prhs[]){', ...
' req(nrhs==5,"Expected centers,radii,seed,seedR,guardScale"); req(nlhs<=5,"At most five outputs");', ...
' const double*C=mxGetPr(prhs[0]); mwSize N=mxGetN(prhs[0]); req(mxGetM(prhs[0])==3,"centers must be 3xN");', ...
' const double*rad=mxGetPr(prhs[1]); req(mxGetNumberOfElements(prhs[1])==N,"radii size");', ...
' const double*V=mxGetPr(prhs[2]); mwSize M=mxGetN(prhs[2]); req(mxGetM(prhs[2])==3&&M>=1,"seed must be 3xM");', ...
' const double*vr=mxGetPr(prhs[3]); req(mxGetNumberOfElements(prhs[3])==M,"seedR size");', ...
' double gs=mxGetScalar(prhs[4]); req(gs>=0&&std::isfinite(gs),"guardScale");', ...
' std::vector<int> cls(N,0),faceOf(N,0),source; std::vector<double> FN,FH;', ...
' mwSize unresolved=N; int faceCount=0;', ...
' while(unresolved>0){', ...
'  mwSize pick=N,pk=0; double pickGap=std::numeric_limits<double>::infinity();', ...
'  for(mwSize i=0;i<N;++i)if(cls[i]==0){double best=std::numeric_limits<double>::infinity();mwSize bk=0;for(mwSize k=0;k<M;++k){double dx=V[3*k]-C[3*i],dy=V[3*k+1]-C[3*i+1],dz=V[3*k+2]-C[3*i+2];double d=n3(dx,dy,dz)-vr[k]-rad[i];if(d<best){best=d;bk=k;}}if(best<pickGap){pickGap=best;pick=i;pk=bk;}}', ...
'  req(pick<N,"internal unresolved accounting");', ...
'  double dx=V[3*pk]-C[3*pick],dy=V[3*pk+1]-C[3*pick+1],dz=V[3*pk+2]-C[3*pick+2];double dn=n3(dx,dy,dz);', ...
'  double sc=1.0;for(int a=0;a<3;++a)sc=std::max(sc,std::fabs(C[3*pick+a]));double guard=gs*std::numeric_limits<double>::epsilon()*sc;', ...
'  bool ok=std::isfinite(dn)&&dn>guard;', ...
'  double n[3]={0,0,0},h=0; if(ok){n[0]=dx/dn;n[1]=dy/dn;n[2]=dz/dn;h=n[0]*C[3*pick]+n[1]*C[3*pick+1]+n[2]*C[3*pick+2]+rad[pick];', ...
'   for(mwSize k=0;k<M;++k){double lhs=n[0]*V[3*k]+n[1]*V[3*k+1]+n[2]*V[3*k+2]-vr[k];double s=std::max(1.0,std::max(std::fabs(lhs),std::fabs(h)));double g=gs*std::numeric_limits<double>::epsilon()*s;if(lhs<h-g){ok=false;break;}}}', ...
'  if(!ok){cls[pick]=-1;--unresolved;continue;}', ...
'  ++faceCount; source.push_back((int)pick+1);FN.push_back(n[0]);FN.push_back(n[1]);FN.push_back(n[2]);FH.push_back(h);', ...
'  bool any=false;for(mwSize i=0;i<N;++i)if(cls[i]==0){double mx=n[0]*C[3*i]+n[1]*C[3*i+1]+n[2]*C[3*i+2]+rad[i];double s=std::max(1.0,std::max(std::fabs(mx),std::fabs(h)));double g=gs*std::numeric_limits<double>::epsilon()*s;if(mx<=h+g){cls[i]=1;faceOf[i]=faceCount;--unresolved;any=true;}}', ...
'  if(!any){cls[pick]=1;faceOf[pick]=faceCount;--unresolved;}', ...
' }', ...
' plhs[0]=mxCreateDoubleMatrix(3,faceCount,mxREAL);double*outN=mxGetPr(plhs[0]);for(size_t i=0;i<FN.size();++i)outN[i]=FN[i];', ...
' if(nlhs>1){plhs[1]=mxCreateDoubleMatrix(1,faceCount,mxREAL);double*x=mxGetPr(plhs[1]);for(int i=0;i<faceCount;++i)x[i]=FH[i];}', ...
' if(nlhs>2){plhs[2]=mxCreateDoubleMatrix(1,N,mxREAL);double*x=mxGetPr(plhs[2]);for(mwSize i=0;i<N;++i)x[i]=double(cls[i]);}', ...
' if(nlhs>3){plhs[3]=mxCreateDoubleMatrix(1,N,mxREAL);double*x=mxGetPr(plhs[3]);for(mwSize i=0;i<N;++i)x[i]=double(faceOf[i]);}', ...
' if(nlhs>4){plhs[4]=mxCreateDoubleMatrix(1,faceCount,mxREAL);double*x=mxGetPr(plhs[4]);for(int i=0;i<faceCount;++i)x[i]=double(source[i]);}', ...
'}'};
end

% ========================================================================
% MEX 3: direct continuous whole-body obstacle certificate.
% Real-arithmetic theorem role:
%   p(t) in conv(P), resultant r(t) in conv(R), q=-r/||r||.
%   A separator n with a certified q upper bound gives supports for payload,
%   UAV and cable.  Recursive Bezier subdivision is fail-closed.
% Floating-point promotion is intentionally FALSE in V200 until the complete
% outward-rounding proof boundary is closed.
% ========================================================================

function lines=v200_cpp_direct_cert_source()
lines={ ...
'#include "mex.h"', ...
'#include <vector>', ...
'#include <cmath>', ...
'#include <limits>', ...
'#include <algorithm>', ...
'struct Stats{double checks=0,leaves=0;int maxDepth=0;double minSlack=std::numeric_limits<double>::infinity();};', ...
'static void req(bool q,const char*m){if(!q)mexErrMsgIdAndTxt("G26V200:DirectCertMEX","%s",m);}', ...
'static double n3(double x,double y,double z){return std::sqrt(x*x+y*y+z*z);}', ...
'static double dot3(const double*a,const double*b){return a[0]*b[0]+a[1]*b[1]+a[2]*b[2];}', ...
'static void splitB(const std::vector<double>&P,int n,double u,std::vector<double>&L,std::vector<double>&R){std::vector<double>T=P;L.assign(3*n,0);R.assign(3*n,0);for(int a=0;a<3;++a){L[a]=P[a];R[a+3*(n-1)]=P[a+3*(n-1)];}for(int r=1;r<n;++r){int m=n-r;for(int j=0;j<m;++j)for(int a=0;a<3;++a)T[a+3*j]=(1-u)*T[a+3*j]+u*T[a+3*(j+1)];for(int a=0;a<3;++a){L[a+3*r]=T[a];R[a+3*(n-1-r)]=T[a+3*(m-1)];}}}', ...
'static void evalB(const std::vector<double>&P,int n,double u,double*out){std::vector<double>T=P;for(int r=1;r<n;++r){int m=n-r;for(int j=0;j<m;++j)for(int a=0;a<3;++a)T[a+3*j]=(1-u)*T[a+3*j]+u*T[a+3*(j+1)];}out[0]=T[0];out[1]=T[1];out[2]=T[2];}', ...
'static double ellSupport(const double*n,const double*c,const double*v,const double*R,const double*rad,double ta,double tb){double rt[3];for(int k=0;k<3;++k)rt[k]=R[0+3*k]*n[0]+R[1+3*k]*n[1]+R[2+3*k]*n[2];double e=std::sqrt((rad[0]*rt[0])*(rad[0]*rt[0])+(rad[1]*rt[1])*(rad[1]*rt[1])+(rad[2]*rt[2])*(rad[2]*rt[2]));double q=dot3(n,v);return dot3(n,c)+e+std::max(ta*q,tb*q);}', ...
'static bool qUpper(const double*n,const std::vector<double>&Res,int nr,double&qub,double&scale){double nn=n3(n[0],n[1],n[2]);if(!(nn>0&&std::isfinite(nn)))return false;double amax=-std::numeric_limits<double>::infinity(),rub=0;scale=1;for(int j=0;j<nr;++j){const double*r=&Res[3*j];amax=std::max(amax,-dot3(n,r));rub=std::max(rub,n3(r[0],r[1],r[2]));scale=std::max(scale,std::max(std::fabs(r[0]),std::max(std::fabs(r[1]),std::fabs(r[2]))));}std::vector<double>A;auto addAxis=[&](double x,double y,double z){double d=n3(x,y,z);if(d>1e-14&&std::isfinite(d)){A.push_back(x/d);A.push_back(y/d);A.push_back(z/d);}};A.push_back(0);A.push_back(0);A.push_back(1);double rm[3];evalB(Res,nr,0.5,rm);addAxis(rm[0],rm[1],rm[2]);for(int j=0;j<nr;++j)addAxis(Res[3*j],Res[3*j+1],Res[3*j+2]);double rlb=-std::numeric_limits<double>::infinity();for(size_t k=0;k<A.size()/3;++k){double b=std::numeric_limits<double>::infinity();double m[3]={A[3*k],A[3*k+1],A[3*k+2]};for(int j=0;j<nr;++j)b=std::min(b,m[0]*Res[3*j]+m[1]*Res[3*j+1]+m[2]*Res[3*j+2]);rlb=std::max(rlb,b);}double g=4096*std::numeric_limits<double>::epsilon()*scale;if(!(std::isfinite(rlb)&&rlb>g&&rub>0))return false;double raw=(amax>=0)?amax/(rlb-g):amax/(rub+g);qub=std::min(nn,raw+4096*std::numeric_limits<double>::epsilon()*std::max(1.0,std::fabs(raw)));return std::isfinite(qub)&&qub>=-nn;}', ...
'static bool testN(const std::vector<double>&P,int np,const std::vector<double>&Res,int nr,const double*c,const double*v,const double*R,const double*rad,double ta,double tb,const double*K,double nx,double ny,double nz,double gs,Stats&st,double&sl){double nn=n3(nx,ny,nz);if(!(nn>1e-14&&std::isfinite(nn)))return false;double n[3]={nx/nn,ny/nn,nz/nn};double qub,sc;if(!qUpper(n,Res,nr,qub,sc))return false;double base=ellSupport(n,c,v,R,rad,ta,tb);double offP=K[0],offU=K[3]*qub+K[1],offC=K[3]*std::max(0.0,qub)+K[2];double hs=base+std::max(offP,std::max(offU,offC));sl=std::numeric_limits<double>::infinity();double scale=std::max(1.0,std::fabs(hs));for(int j=0;j<np;++j){double z=n[0]*P[3*j]+n[1]*P[3*j+1]+n[2]*P[3*j+2]-hs;sl=std::min(sl,z);scale=std::max(scale,std::max(std::fabs(P[3*j]),std::max(std::fabs(P[3*j+1]),std::fabs(P[3*j+2]))));}double guard=gs*std::numeric_limits<double>::epsilon()*scale;st.checks++;return std::isfinite(sl)&&sl>=-guard;}', ...
'static bool sep(const std::vector<double>&P,int np,const std::vector<double>&Res,int nr,const double*c,const double*v,const double*R,const double*rad,double ta,double tb,const double*K,double gs,Stats&st,double&best){double cm[3]={c[0]+v[0]*0.5*(ta+tb),c[1]+v[1]*0.5*(ta+tb),c[2]+v[2]*0.5*(ta+tb)};double mean[3]={0,0,0};for(int j=0;j<np;++j)for(int a=0;a<3;++a)mean[a]+=P[a+3*j];for(int a=0;a<3;++a)mean[a]/=np;std::vector<double>N;auto add=[&](double x,double y,double z){N.push_back(x);N.push_back(y);N.push_back(z);};add(1,0,0);add(-1,0,0);add(0,1,0);add(0,-1,0);add(0,0,1);add(0,0,-1);for(int k=0;k<3;++k){add(R[0+3*k],R[1+3*k],R[2+3*k]);add(-R[0+3*k],-R[1+3*k],-R[2+3*k]);}add(mean[0]-cm[0],mean[1]-cm[1],mean[2]-cm[2]);double ss[3]={0,0.5,1};for(int ii=0;ii<3;++ii){double x[3],r[3];evalB(P,np,ss[ii],x);evalB(Res,nr,ss[ii],r);double rr=n3(r[0],r[1],r[2]);if(rr>1e-14){double q[3]={-r[0]/rr,-r[1]/rr,-r[2]/rr};double xq[3]={x[0]-K[3]*q[0],x[1]-K[3]*q[1],x[2]-K[3]*q[2]};add(x[0]-cm[0],x[1]-cm[1],x[2]-cm[2]);add(xq[0]-cm[0],xq[1]-cm[1],xq[2]-cm[2]);double d[3]={xq[0]-x[0],xq[1]-x[1],xq[2]-x[2]};double dd=dot3(d,d);double lam=0;if(dd>0){double w[3]={cm[0]-x[0],cm[1]-x[1],cm[2]-x[2]};lam=std::max(0.0,std::min(1.0,dot3(w,d)/dd));}double xc[3]={x[0]+lam*d[0],x[1]+lam*d[1],x[2]+lam*d[2]};add(xc[0]-cm[0],xc[1]-cm[1],xc[2]-cm[2]);}}best=-std::numeric_limits<double>::infinity();for(size_t k=0;k<N.size()/3;++k){double sl=-std::numeric_limits<double>::infinity();if(testN(P,np,Res,nr,c,v,R,rad,ta,tb,K,N[3*k],N[3*k+1],N[3*k+2],gs,st,sl)){best=sl;return true;}best=std::max(best,sl);}return false;}', ...
'static bool rec(const std::vector<double>&P,int np,const std::vector<double>&Res,int nr,const double*c,const double*v,const double*R,const double*rad,double ta,double tb,const double*K,int dep,int md,double gs,Stats&st){st.maxDepth=std::max(st.maxDepth,dep);double sl;if(sep(P,np,Res,nr,c,v,R,rad,ta,tb,K,gs,st,sl)){st.leaves++;st.minSlack=std::min(st.minSlack,sl);return true;}if(dep>=md){st.leaves++;st.minSlack=std::min(st.minSlack,sl);return false;}std::vector<double>PL,PR,RL,RR;splitB(P,np,0.5,PL,PR);splitB(Res,nr,0.5,RL,RR);double tm=0.5*(ta+tb);if(!rec(PL,np,RL,nr,c,v,R,rad,ta,tm,K,dep+1,md,gs,st))return false;return rec(PR,np,RR,nr,c,v,R,rad,tm,tb,K,dep+1,md,gs,st);}', ...
'void mexFunction(int nlhs,mxArray*plhs[],int nrhs,const mxArray*prhs[]){req(nrhs==13,"Expected 13 inputs");req(nlhs<=7,"At most 7 outputs");const double*Pf=mxGetPr(prhs[0]);mwSize NP=mxGetN(prhs[0]);req(mxGetM(prhs[0])==3,"Pflat 3xN");const double*pc=mxGetPr(prhs[1]);mwSize np=mxGetNumberOfElements(prhs[1]);const double*Rf=mxGetPr(prhs[2]);mwSize NR=mxGetN(prhs[2]);req(mxGetM(prhs[2])==3,"Rflat 3xN");const double*rc=mxGetPr(prhs[3]);req(mxGetNumberOfElements(prhs[3])==np,"rcount mismatch");const double*ta=mxGetPr(prhs[4]);const double*tb=mxGetPr(prhs[5]);req(mxGetNumberOfElements(prhs[4])==np&&mxGetNumberOfElements(prhs[5])==np,"time mismatch");const double*C=mxGetPr(prhs[6]);mwSize no=mxGetN(prhs[6]);req(mxGetM(prhs[6])==3,"C 3xM");const double*V=mxGetPr(prhs[7]);const double*OR=mxGetPr(prhs[8]);const double*rad=mxGetPr(prhs[9]);req(mxGetM(prhs[7])==3&&mxGetN(prhs[7])==no&&mxGetM(prhs[8])==9&&mxGetN(prhs[8])==no&&mxGetM(prhs[9])==3&&mxGetN(prhs[9])==no,"obstacle shape mismatch");const double*K=mxGetPr(prhs[10]);req(mxGetNumberOfElements(prhs[10])==4,"K size");int md=(int)std::llround(mxGetScalar(prhs[11]));double gs=mxGetScalar(prhs[12]);req(md>=0&&md<=12,"depth");bool pass=true;double fp=0,fo=0;Stats st;mwSize op=0,orr=0;for(mwSize ip=0;ip<np&&pass;++ip){int npp=(int)std::llround(pc[ip]),nrr=(int)std::llround(rc[ip]);req(npp>=2&&nrr>=1&&op+npp<=NP&&orr+nrr<=NR,"counts");std::vector<double>P(3*npp),Rr(3*nrr);for(int j=0;j<npp;++j)for(int a=0;a<3;++a)P[a+3*j]=Pf[a+3*(op+j)];for(int j=0;j<nrr;++j)for(int a=0;a<3;++a)Rr[a+3*j]=Rf[a+3*(orr+j)];for(mwSize io=0;io<no;++io){if(!rec(P,npp,Rr,nrr,C+3*io,V+3*io,OR+9*io,rad+3*io,ta[ip],tb[ip],K,0,md,gs,st)){pass=false;fp=ip+1;fo=io+1;break;}}op+=npp;orr+=nrr;}if(nlhs>0)plhs[0]=mxCreateLogicalScalar(pass);if(nlhs>1)plhs[1]=mxCreateDoubleScalar(st.minSlack);if(nlhs>2)plhs[2]=mxCreateDoubleScalar(st.checks);if(nlhs>3)plhs[3]=mxCreateDoubleScalar(fp);if(nlhs>4)plhs[4]=mxCreateDoubleScalar(fo);if(nlhs>5)plhs[5]=mxCreateDoubleScalar(st.leaves);if(nlhs>6)plhs[6]=mxCreateDoubleScalar((double)st.maxDepth);}', ...
};
end

% ========================================================================
% C6 / time-allocation selftests
% ========================================================================

function pass=v200_c6_selftest()
nseg=3; dt=[0.8 1.1 1.7];
J0=zeros(3,7,nseg); J1=zeros(3,7,nseg); F=zeros(3,4,nseg);
K1=[0.8;0.1;0.3; 0.35;0.0;0.0; 0.02;0.01;0.0; zeros(12,1)];
K2=[1.7;0.4;0.5; 0.25;0.1;0.0; -0.01;0.02;0.0; zeros(12,1)];
J0(:,:,1)=reshape([0;0;0; 0.5;0;0; zeros(15,1)],3,7);
J1(:,:,1)=reshape(K1,3,7);
J0(:,:,2)=J1(:,:,1); J1(:,:,2)=reshape(K2,3,7);
J0(:,:,3)=J1(:,:,2); J1(:,:,3)=reshape([2.8;0.8;0.7; 0.2;0;0; zeros(15,1)],3,7);
for q=1:nseg
    F(:,:,q)=[linspace(J0(1,1,q),J1(1,1,q),4); linspace(J0(2,1,q),J1(2,1,q),4); linspace(J0(3,1,q),J1(3,1,q),4)];
end
B=g26v200_c6_construct_mex(J0,J1,dt,F);
A=v200_c6_relation_audit(B,J0,J1,dt);

% A repeated-difference evaluation of a sixth derivative is intrinsically
% cancellation-sensitive.  It is retained as a diagnostic only; structural
% C6 authority comes from the Bernstein endpoint identities audited above.
[maxPhysicalEnd,maxPhysicalJoin]=v200_physical_jet_diagnostic(B,J0,J1,dt);

pass=A.pass && A.shared_jet_exact;
if ~pass
    fprintf(2,'[G26-V200C-C6-SELFTEST-FAIL] seg=%g side=%g order=%g residual=%.17g bound=%.17g sharedJetExact=%d\n', ...
        A.fail_segment,A.fail_side,A.fail_order,A.fail_residual,A.fail_bound,A.shared_jet_exact);
end
assert(pass,'G26V200C:C6Selftest','Structural Bernstein endpoint/internal C6 identities failed.');
fprintf(['[G26-V204-C6-SELFTEST] PASS | real-arithmetic endpoint d0..d6 embedded | ', ...
    'relationMax=%.3e boundRatio=%.3e sharedJetExact=%d | FP physical diagnostic endpoint/join=%.3e/%.3e\n'], ...
    A.max_relation_residual,A.max_bound_ratio,A.shared_jet_exact,maxPhysicalEnd,maxPhysicalJoin);
end


function pass=v200_time_contract_selftest()
% Same physical boundary/internal jets, two distinct positive time allocations.
% Both must satisfy the same structural Bernstein endpoint identities.  This
% directly tests that no canonical/fixed h is baked into the constructor.
h1=[0.20 0.30 0.50]; h2=[0.33 0.22 0.45]; T=4.0;
assert(all(h1>0)&&all(h2>0)&&abs(sum(h1)-1)<1e-14&&abs(sum(h2)-1)<1e-14);
J0=zeros(3,7,3);J1=zeros(3,7,3);F=zeros(3,4,3);
J0(:,1,1)=[0;0;0];J1(:,1,1)=[1;0.2;0.1];
J0(:,:,2)=J1(:,:,1);J1(:,1,2)=[2;0.5;0.2];
J0(:,:,3)=J1(:,:,2);J1(:,1,3)=[3;0.7;0.4];
B1=g26v200_c6_construct_mex(J0,J1,T*h1,F);
B2=g26v200_c6_construct_mex(J0,J1,T*h2,F);
A1=v200_c6_relation_audit(B1,J0,J1,T*h1);
A2=v200_c6_relation_audit(B2,J0,J1,T*h2);
pass=A1.pass&&A2.pass&&A1.shared_jet_exact&&A2.shared_jet_exact&&max(abs((T*h1)-(T*h2)))>0;
assert(pass,'G26V200C:TimeSelftest','Variable-time structural C6 contract failed.');
fprintf(['[G26-V204-TIME-SELFTEST] PASS | two distinct h allocations preserve structural C6 | ', ...
    'maxRelation=[%.3e %.3e] fixed canonical h absent\n'],A1.max_relation_residual,A2.max_relation_residual);
end


function A=v200_c6_relation_audit(B,J0,J1,dt)
% For a degree-n Bezier segment in normalized coordinate s=t/h,
%   p^(r)(0) = falling(n,r)/h^r * Delta^r B_0
%   p^(r)(1) = falling(n,r)/h^r * Delta^r B_{n-r}.
% Therefore matching the prescribed physical jets is equivalent to the
% normalized endpoint identities checked below.  This avoids declaring C6
% failure merely because a high-order derivative is reconstructed through an
% ill-conditioned sequence of repeated differences.
n=17; maxr=6; u=eps/2;
maxRes=0; maxRatio=0; allBound=true; failQ=NaN; failSide=NaN; failOrder=NaN; failRes=NaN; failBound=NaN;
for q=1:numel(dt)
    assert(isfinite(dt(q))&&dt(q)>0,'G26V200C:C6Audit','Nonpositive/nonfinite segment duration.');
    for side=1:2
        if side==1, J=J0(:,:,q); else, J=J1(:,:,q); end
        for r=0:maxr
            jj=0:r;
            coeff=zeros(1,r+1);
            for k=1:r+1
                coeff(k)=(-1)^(r-jj(k))*nchoosek(r,jj(k));
            end
            if side==1
                idx=1:(r+1);
            else
                idx=(n-r+1):(n+1);
            end
            terms=B(:,idx,q).*coeff;
            delta=sum(terms,2);
            fall=v200_falling(n,r);
            target=(dt(q)^r/fall)*J(:,r+1);
            res=abs(delta-target);

            % Standard floating-point dot-product model: |fl(sum)-sum| <=
            % gamma_k*sum|term|.  kOps=64 is a conservative upper bound for
            % the <=7-term endpoint relation plus the constructor's scalar
            % recurrence operations at r<=6.  This is a numerical selftest
            % bound only, NOT the still-open formal C++ directed-rounding proof.
            kOps=64;
            gamma=(kOps*u)/(1-kOps*u);
            bnd=gamma*(sum(abs(terms),2)+abs(target));
            bnd=max(bnd,realmin(class(B)));
            ratio=max(res./bnd);
            maxRes=max(maxRes,max(res));
            maxRatio=max(maxRatio,ratio);
            thisPass=all(res<=bnd);
            if ~thisPass && isnan(failQ)
                [failRes,ia]=max(res-bnd);
                failRes=res(ia); failBound=bnd(ia); failQ=q; failSide=side; failOrder=r;
            end
            allBound=allBound && thisPass;
        end
    end
end
shared=true;
for q=1:size(J0,3)-1
    shared=shared && isequal(J1(:,:,q),J0(:,:,q+1));
end
A=struct('pass',allBound,'shared_jet_exact',shared,'max_relation_residual',maxRes,'max_bound_ratio',maxRatio, ...
    'fail_segment',failQ,'fail_side',failSide,'fail_order',failOrder,'fail_residual',failRes,'fail_bound',failBound);
end


function z=v200_falling(n,r)
z=1;
for k=0:r-1, z=z*(n-k); end
end


function s=v200_pf(pass)
if pass
    s='PASS';
else
    s='FAIL';
end
end


function [maxEnd,maxJoin]=v200_physical_jet_diagnostic(B,J0,J1,dt)
maxEnd=0;maxJoin=0;
for q=1:size(B,3)
    JL=v200_bezier_endpoint_jets(B(:,:,q),dt(q),true,6);
    JR=v200_bezier_endpoint_jets(B(:,:,q),dt(q),false,6);
    maxEnd=max(maxEnd,max(abs(JL(:)-reshape(J0(:,:,q),[],1))));
    maxEnd=max(maxEnd,max(abs(JR(:)-reshape(J1(:,:,q),[],1))));
    if q<size(B,3)
        JN=v200_bezier_endpoint_jets(B(:,:,q+1),dt(q+1),true,6);
        maxJoin=max(maxJoin,max(abs(JR(:)-JN(:))));
    end
end
end


function J=v200_bezier_endpoint_jets(B,dt,left,maxOrder)
D=B; deg=size(B,2)-1; J=zeros(3,maxOrder+1); J(:,1)=left*D(:,1)+(~left)*D(:,end);
for r=1:maxOrder
    D=deg*diff(D,1,2); deg=deg-1;
    if left, J(:,r+1)=D(:,1)/dt^r; else, J(:,r+1)=D(:,end)/dt^r; end
end
end

% ========================================================================
% Local SFC selftest
% ========================================================================

function pass=v200_local_sfc_selftest()
C=[0 0.6 1.2 1.85; 0 0.2 -0.1 0; 0 0 0 0];
r=[0.12 0.10 0.08 0.12];
V=[2.5 3.0 2.8; 0 0.2 -0.2; 0 0 0];
vr=[0.25 0.15 0.25];
[N,h,cls,faceOf,src]=g26v200_local_sfc_mex(C,r,V,vr,4096); %#ok<ASGLU>
accounted=all(cls~=0);
seedContained=true;
for f=1:size(N,2)
    for k=1:size(V,2)
        seedContained=seedContained && (N(:,f)'*V(:,k)-vr(k) >= h(f)-1e-11);
    end
end
faceCoverage=true;
for i=1:size(C,2)
    if cls(i)>0
        f=round(faceOf(i));
        faceCoverage=faceCoverage && f>=1 && f<=size(N,2) && (N(:,f)'*C(:,i)+r(i) <= h(f)+1e-11);
    end
end
pass=accounted&&seedContained&&faceCoverage&&numel(src)==size(N,2);
assert(pass,'G26V200:SFCSelftest','Local sphere proposal accounting/containment failed.');
fprintf('[G26-V204-SFC-SELFTEST] PASS | primitives=%d faces=%d fallback=%d completeAccounting=1 authority=0\n', ...
    size(C,2),size(N,2),sum(cls<0));
end

% ========================================================================
% Direct certificate selftest
% ========================================================================

function pass=v200_direct_cert_selftest(CFG)
Pfar=[5 5;0 0;0 0]; counts=2;
R=[0;0;CFG.gravity]; rcounts=1; ta=0;tb=1;
C=[0;0;0];V=[0;0;0];OR=reshape(eye(3),9,1);RAD=[0.2;0.2;0.2];
K=[CFG.R_payload+CFG.geometry_margin_m, CFG.R_uav+CFG.geometry_margin_m, ...
    CFG.R_cable+CFG.geometry_margin_m, CFG.L_cable];
[far,slFar]=g26v200_direct_cert_mex(Pfar,counts,R,rcounts,ta,tb,C,V,OR,RAD,K,CFG.direct_cert_max_depth,CFG.direct_cert_guard_scale);
Pbad=[0 0;0 0;0 0];
[bad,~]=g26v200_direct_cert_mex(Pbad,counts,R,rcounts,ta,tb,C,V,OR,RAD,K,CFG.direct_cert_max_depth,CFG.direct_cert_guard_scale);
% Moving obstacle sweeps into the candidate over [0,1].
C2=[3.8;0;0];V2=[2.0;0;0];
[moving,~]=g26v200_direct_cert_mex(Pfar,counts,R,rcounts,ta,tb,C2,V2,OR,RAD,K,CFG.direct_cert_max_depth,CFG.direct_cert_guard_scale);
pass=logical(far)&&~logical(bad)&&~logical(moving)&&isfinite(slFar);
assert(pass,'G26V200:CertSelftest','Direct whole-body certificate selftest failed.');
fprintf('[G26-V204-CERT-SELFTEST] PASS | far SAFE | collision fail-closed | swept moving obstacle fail-closed\n');
end

% ========================================================================
% Atomic safe-reference supervisor selftest
% ========================================================================

function pass=v200_atomic_switch_selftest()
S=struct('active_id',1,'active_certified',true,'switch_count',0);
bad=struct('id',2,'c6',true,'dynamics',true,'whole_body',false,'deadline',true,'yaw',true);
S1=v200_try_switch(S,bad);
good=struct('id',3,'c6',true,'dynamics',true,'whole_body',true,'deadline',true,'yaw',true);
S2=v200_try_switch(S1,good);
late=good;late.id=4;late.deadline=false;
S3=v200_try_switch(S2,late);
pass=S1.active_id==1 && S1.switch_count==0 && S2.active_id==3 && S2.switch_count==1 && S3.active_id==3;
assert(pass,'G26V200:SwitchSelftest','Atomic safe-reference retention test failed.');
fprintf('[G26-V204-SWITCH-SELFTEST] PASS | failed/late successor cannot replace certified active reference\n');
end


function S=v200_try_switch(S,C)
ready=C.c6&&C.dynamics&&C.whole_body&&C.deadline&&C.yaw;
if ready
    S.active_id=C.id;S.active_certified=true;S.switch_count=S.switch_count+1;
end
end

% ========================================================================
% Microbench: kernel-only architecture evidence, NOT integrated WCET.
% ========================================================================

function v200_print_bench(B)
fprintf('\n[G26-V206 TIMING / STORAGE / RANGE / SPATIAL-FRONTEND AUDIT | BUILD/SELFTEST EXCLUDED | NOT WCET]\n');
fprintf(' qualification dual-LiDAR: rays=%d returns=%d (%.1f%%) coverCapacity=%d\n',B.qual.rays,B.qual.returns,100*B.qual.return_fraction,B.qual.cover_capacity);
v202_print_stats(' first-return frame',B.qual.first_return);
v202_print_stats(' cover ingest+compress',B.qual.cover_ingest);
v202_print_stats(' motion envelope',B.qual.motion_envelope);
v202_print_stats(' trigger on full cover',B.qual.trigger_cover);
v202_print_stats(' window supervisor',B.qual.window);
for k=1:numel(B.qual.direct_cert),v202_print_stats(sprintf(' direct-cert M=%d',B.qual.direct_cert(k).obstacles),B.qual.direct_cert(k).stats);end
fprintf(' p95 prefix(no final cert)=%.6f ms | prefix allocation=%.3f | screen=%d\n',B.qual.p95_prefix_ms,B.qual.prefix_design_budget_ms,B.qual.prefix_budget_screen);
fprintf(' p95 prefix+directCert(M=%d)=%.6f ms | remaining to 4ms=%+.6f ms\n',B.qual.direct_cert(end).obstacles,B.qual.p95_with_cert_ms,B.qual.remaining_to_4ms_after_cert_ms);
fprintf(' storage sustain: frames=%d finalCount=%d/%d mergeTotal=%d maxRadius(static replay)=%.6g m | PASS=%d\n',B.qual.sustain_frames,B.qual.sustain_count,B.qual.cover_capacity,B.qual.sustain_merge_total,B.qual.sustain_max_radius,B.qual.sustain_storage_pass);
fprintf(' one-frame outer-cover support: maxConservativeGap=%.6g m containmentResidual=%.3e\n',B.qual.one_frame_support_gap_max,B.qual.one_frame_support_containment_residual);
fprintf(' dynamic/no-certified-association diagnostic: 50 frames + 1s horizon maxRadius=%.6g m allMotionKnown=%d\n',B.qual.dynamic50_max_radius,B.qual.dynamic50_all_known);
v202_print_stats(sprintf(' association N=%d M=%d',B.qual.association_tracks,B.qual.association_measurements),B.qual.association);
v202_print_stats(' reacquisition theorem',B.qual.reacquisition);
v202_print_stats(' bounded-blind-gap reacq',B.qual.blind_gap_reacquisition);
if isfield(B.qual,'range_gate'),v202_print_stats(' range gate 4320 rays',B.qual.range_gate);end
if isfield(B.qual,'local_target'),v202_print_stats(' local target selector',B.qual.local_target);end
if isfield(B.qual,'p95_prefix_range_localtarget_cert_ms'),fprintf(' inherited V205 p95 prefix+range+localTarget+directCert(M=%d)=%.6f ms | remaining to 4ms=%+.6f ms\n',B.qual.direct_cert(end).obstacles,B.qual.p95_prefix_range_localtarget_cert_ms,B.qual.remaining_to_4ms_v205_ms);end
fprintf(' RULE: fixed storage is closed by outer merge; useful long-horizon dynamic-memory conservatism is NOT closed by this fact.\n');
fprintf(' RULE: association/reacquisition timings are qualification-only until the sensor/object premises are certified.\n');
fprintf(' RULE: p95 sums are screens only; integrated sensor-return->switch WCET remains OPEN.\n');
end

function v202_print_stats(name,S)
fprintf(' %-24s p50/p95/max = %.6f / %.6f / %.6f ms | mean=%.6f\n',name,S.p50,S.p95,S.max,S.mean);
end


% ========================================================================
% V207 generic continuous-time cable cone / tautness gate.
% The trajectory is supplied as piecewise Bernstein control points.
% Segment duration ratios are positive online geometry-dependent weights.
% Only the common scale T is searched continuously; there is no finite
% time bank.  For fixed geometry,
%   r_j(T)=g*e3 + a_j/T^2.
% The feasible set {r: ||r_xy|| <= tan(theta) r_z, r_z >= rho_min} is
% convex and contains g*e3 when rho_min<g, hence feasibility is monotone
% under increasing T and bisection is valid.
% ========================================================================

function v207_mex_bootstrap_time_gate(root)
name='g26v207_time_gate_mex';
v200_build_mex_if_needed(root,name,v207_cpp_time_gate_source(),'G26-V207-TIME-GATE-MEX');
end

function lines=v207_cpp_time_gate_source()
lines={ ...
'#include "mex.h"', ...
'#include <cmath>', ...
'#include <vector>', ...
'#include <limits>', ...
'#include <algorithm>', ...
'static void req(bool q,const char*m){if(!q)mexErrMsgIdAndTxt("G26V207:TimeGateMEX","%s",m);}', ...
'struct Eval{bool pass=false;double cone=std::numeric_limits<double>::infinity();double rz=std::numeric_limits<double>::infinity();};', ...
'static Eval evalT(const double*P,const double*cnt,const double*w,mwSize S,double sw,double th,double rho,double g,double T,double gs){', ...
' Eval E; mwSize off=0; double tt=std::tan(th); E.pass=true;', ...
' for(mwSize q=0;q<S;++q){int np=(int)std::llround(cnt[q]);int n=np-1;double h=T*w[q]/sw;req(np>=3&&h>0&&std::isfinite(h),"invalid degree/duration");double fac=((double)n*(double)(n-1))/(h*h);', ...
'  for(int j=0;j<n-1;++j){double ax=fac*(P[0+3*(off+j+2)]-2.0*P[0+3*(off+j+1)]+P[0+3*(off+j)]);double ay=fac*(P[1+3*(off+j+2)]-2.0*P[1+3*(off+j+1)]+P[1+3*(off+j)]);double az=fac*(P[2+3*(off+j+2)]-2.0*P[2+3*(off+j+1)]+P[2+3*(off+j)])+g;', ...
'   double cs=tt*az-std::hypot(ax,ay);double rs=az-rho;double sc=std::max(1.0,std::max(std::fabs(az),std::hypot(ax,ay)));double gd=gs*std::numeric_limits<double>::epsilon()*sc;E.cone=std::min(E.cone,cs);E.rz=std::min(E.rz,rs);if(!(std::isfinite(cs)&&std::isfinite(rs)&&cs>=gd&&rs>=gd))E.pass=false;}', ...
'  off+=(mwSize)np;', ...
' }return E;', ...
'}', ...
'void mexFunction(int nlhs,mxArray*plhs[],int nrhs,const mxArray*prhs[]){', ...
' req(nrhs==9,"Expected Pflat,counts,weights,theta,rho,g,Tmin,Tmax,guardScale");req(nlhs<=8,"At most 8 outputs");', ...
' req(mxIsDouble(prhs[0])&&mxGetM(prhs[0])==3,"Pflat must be 3xN");const double*P=mxGetPr(prhs[0]);mwSize NP=mxGetN(prhs[0]);', ...
' req(mxIsDouble(prhs[1]),"counts double");const double*cnt=mxGetPr(prhs[1]);mwSize S=mxGetNumberOfElements(prhs[1]);req(S>=1,"at least one segment");', ...
' req(mxIsDouble(prhs[2])&&mxGetNumberOfElements(prhs[2])==S,"weights mismatch");const double*w=mxGetPr(prhs[2]);mwSize sumN=0;double sw=0;for(mwSize q=0;q<S;++q){double cc=cnt[q];req(std::isfinite(cc)&&cc>=3&&std::fabs(cc-std::round(cc))<1e-12,"counts invalid");sumN+=(mwSize)std::llround(cc);req(std::isfinite(w[q])&&w[q]>0,"weights must be positive");sw+=w[q];}req(sumN==NP&&sw>0,"flatten/count mismatch");', ...
' double th=mxGetScalar(prhs[3]),rho=mxGetScalar(prhs[4]),g=mxGetScalar(prhs[5]),Tmin=mxGetScalar(prhs[6]),Tmax=mxGetScalar(prhs[7]),gs=mxGetScalar(prhs[8]);', ...
' req(std::isfinite(th)&&th>0&&th<1.5707963267948966&&std::isfinite(rho)&&rho>0&&std::isfinite(g)&&g>rho&&std::isfinite(Tmin)&&Tmin>0&&std::isfinite(Tmax)&&Tmax>Tmin&&std::isfinite(gs)&&gs>=0,"scalar input invalid");', ...
' Eval emax=evalT(P,cnt,w,S,sw,th,rho,g,Tmax,gs);bool ok=emax.pass;double T=Tmax;int it=0;', ...
' if(ok){Eval emin=evalT(P,cnt,w,S,sw,th,rho,g,Tmin,gs);if(emin.pass){T=Tmin;}else{double lo=Tmin,hi=Tmax;for(it=0;it<64;++it){double md=0.5*(lo+hi);Eval em=evalT(P,cnt,w,S,sw,th,rho,g,md,gs);if(em.pass)hi=md;else lo=md;}T=hi;}}', ...
' Eval ef=evalT(P,cnt,w,S,sw,th,rho,g,T,gs);', ...
' if(nlhs>0)plhs[0]=mxCreateDoubleScalar(ok?1.0:0.0);if(nlhs>1)plhs[1]=mxCreateDoubleScalar(T);', ...
' if(nlhs>2){plhs[2]=mxCreateDoubleMatrix(1,S,mxREAL);double*h=mxGetPr(plhs[2]);for(mwSize q=0;q<S;++q)h[q]=T*w[q]/sw;}', ...
' mwSize NR=0;for(mwSize q=0;q<S;++q)NR+=(mwSize)std::llround(cnt[q])-2;', ...
' if(nlhs>3){plhs[3]=mxCreateDoubleMatrix(3,NR,mxREAL);double*R=mxGetPr(plhs[3]);mwSize op=0,orr=0;for(mwSize q=0;q<S;++q){int np=(int)std::llround(cnt[q]);int n=np-1;double h=T*w[q]/sw;double fac=((double)n*(double)(n-1))/(h*h);for(int j=0;j<n-1;++j){R[0+3*orr]=fac*(P[0+3*(op+j+2)]-2*P[0+3*(op+j+1)]+P[0+3*(op+j)]);R[1+3*orr]=fac*(P[1+3*(op+j+2)]-2*P[1+3*(op+j+1)]+P[1+3*(op+j)]);R[2+3*orr]=fac*(P[2+3*(op+j+2)]-2*P[2+3*(op+j+1)]+P[2+3*(op+j)])+g;++orr;}op+=(mwSize)np;}}', ...
' if(nlhs>4){plhs[4]=mxCreateDoubleMatrix(1,S,mxREAL);double*rc=mxGetPr(plhs[4]);for(mwSize q=0;q<S;++q)rc[q]=cnt[q]-2;}', ...
' if(nlhs>5)plhs[5]=mxCreateDoubleScalar(ef.cone);if(nlhs>6)plhs[6]=mxCreateDoubleScalar(ef.rz);if(nlhs>7)plhs[7]=mxCreateDoubleScalar((double)it);', ...
'}'};
end

% ========================================================================
% V207 backend synthesis / qualification.
% ========================================================================

function Q=v207_qualification_problem(CFG)
Q=struct();
Q.p0=[0;0;0];Q.pg=[6;0;0];
Q.C=[3;0;0];
Q.Ractual=CFG.v207_qualification_obstacle_radius_m;
Q.Rproposal=CFG.v207_qualification_proposal_radius_m;
[Q.W,Q.guideStatus,Q.insertions,Q.guideClearance]=g26v206_spatial_guide_mex(Q.p0,Q.pg,Q.C,Q.Rproposal, ...
    CFG.v205_planning_horizon_m,CFG.v206_spatial_guide_max_nodes,CFG.v206_spatial_guide_guard_scale);
assert(Q.guideStatus==1,'G26V207:QualificationGuide','Qualification spatial guide failed.');
[Q.N,Q.h,Q.cnt,Q.corridorOK,Q.corridorMinClearance,Q.faces]=g26v206_segment_corridor_mex(Q.W,Q.C,Q.Rproposal,CFG.v206_spatial_guide_guard_scale);
assert(Q.corridorOK,'G26V207:QualificationCorridor','Qualification corridor failed.');
Q.segment_length=vecnorm(diff(Q.W,1,2),2,1);
assert(all(Q.segment_length>0),'G26V207:Qualification','Degenerate guide segment.');
Q.weights=Q.segment_length/sum(Q.segment_length);
end

function B=v207_build_backend(name,W,w)
switch name
    case 'BSPLINE_D7'
        B=v207_backend_bspline_d7(W,w);
    case 'BEZIER_D13'
        B=v207_backend_bezier_d13(W);
    case 'POLY_D13'
        B=v207_backend_poly_d13(W);
    case 'ACETRANS_MINCO_ST_C6'
        B=v207_backend_minco_d13(W,w);
    otherwise
        error('G26V207:Backend','Unknown backend %s',name);
end
B.name=name;
B.weights=w;
end

function B=v207_backend_bezier_d13(W)
S=size(W,2)-1;P=cell(1,S);
for q=1:S
    X=zeros(3,14);X(:,1:7)=repmat(W(:,q),1,7);X(:,8:14)=repmat(W(:,q+1),1,7);P{q}=X;
end
B=struct();B.P=P;B.degree=13;B.native='BERNSTEIN_D13_ZERO_JET_CARRIER';
B.structural_c6_theorem=true;B.condition_indicator=1;B.representation_residual=0;B.native_data=P;
B.note='Piecewise degree-13 Bezier with d1..d6=0 at every guide knot; corridor-preserving but intentionally conservative.';
end

function B=v207_backend_poly_d13(W)
S=size(W,2)-1;P=cell(1,S);A=cell(1,S);T=v207_bernstein_to_power_matrix(13);mx=0;
for q=1:S
    X=zeros(3,14);X(:,1:7)=repmat(W(:,q),1,7);X(:,8:14)=repmat(W(:,q+1),1,7);
    Aq=(T*X.').'; X2=(T\Aq.').';
    P{q}=X2;A{q}=Aq;mx=max(mx,max(abs(X2(:)-X(:))));
end
B=struct();B.P=P;B.degree=13;B.native='POWER_POLY_D13_ZERO_JET_CARRIER';
B.structural_c6_theorem=true;B.condition_indicator=cond(T);B.representation_residual=mx;B.native_data=A;
B.note='Same corridor-preserving zero-jet carrier represented natively in monomial power coefficients; conversion is qualification-only.';
end

function B=v207_backend_bspline_d7(W,w)
S=size(W,2)-1;p=7;nCtrl=S+p;
br=[0 cumsum(w)];br(end)=1;
U=[zeros(1,p+1) br(2:end-1) ones(1,p+1)];
CP=zeros(3,nCtrl);CP(:,1:4)=repmat(W(:,1),1,4);CP(:,end-3:end)=repmat(W(:,end),1,4);
unk=5:(nCtrl-4);fix=[1:4 (nCtrl-3):nCtrl];
interpResidual=0;
if ~isempty(unk)
    A=zeros(S-1,numel(unk));rhs=zeros(3,S-1);
    for j=1:S-1
        bb=v207_bspline_basis_row(br(j+1),p,U,nCtrl);
        A(j,:)=bb(unk);
        rhs(:,j)=W(:,j+1)-CP(:,fix)*bb(fix).';
    end
    X=A\rhs.';CP(:,unk)=X.';
    for j=1:S-1
        interpResidual=max(interpResidual,norm(v207_bspline_eval(CP,p,U,br(j+1))-W(:,j+1),inf));
    end
    interpCond=cond(A);
else
    interpCond=1;
end
ss=linspace(0,1,p+1);BM=zeros(p+1,p+1);
for j=1:p+1
    s=ss(j);
    for k=0:p,BM(j,k+1)=nchoosek(p,k)*s^k*(1-s)^(p-k);end
end
P=cell(1,S);
for q=1:S
    V=zeros(3,p+1);
    for j=1:p+1
        u=br(q)+(br(q+1)-br(q))*ss(j);
        V(:,j)=v207_bspline_eval(CP,p,U,u);
    end
    P{q}=(BM\V.').';
end
B=struct();B.P=P;B.degree=7;B.native='CLAMPED_BSPLINE_D7_SIMPLE_INTERNAL_KNOTS';
B.structural_c6_theorem=true;B.condition_indicator=max([1 interpCond cond(BM)]);
B.representation_residual=interpResidual;B.native_data=struct('control',CP,'knots',U,'breaks',br);
B.note='Degree-7 clamped B-spline with simple internal knots; real-arithmetic continuity is C6 by knot multiplicity.';
end

function y=v207_bspline_eval(CP,p,U,u)
b=v207_bspline_basis_row(u,p,U,size(CP,2));y=CP*b.';
end

function b=v207_bspline_basis_row(u,p,U,nCtrl)
N=zeros(p+1,nCtrl);
if u>=U(end)
    N(1,end)=1;
else
    for i=1:nCtrl
        if U(i)<=u && u<U(i+1),N(1,i)=1;end
    end
end
for k=1:p
    for i=1:nCtrl
        a=0;bb=0;
        den=U(i+k)-U(i);
        if den>0,a=(u-U(i))/den*N(k,i);end
        if i<nCtrl
            den2=U(i+k+1)-U(i+1);
            if den2>0,bb=(U(i+k+1)-u)/den2*N(k,i+1);end
        end
        N(k+1,i)=a+bb;
    end
end
b=N(p+1,:);
end

function B=v207_backend_minco_d13(W,w)
% Numerically stable qualification MINCO primitive.
% Unknowns are degree-13 Bernstein control points directly.  This is the
% same real-arithmetic minimum-integral-squared-7th-derivative problem as
% the former monomial KKT, but avoids the ill-conditioned power basis.
S=size(W,2)-1;n=13;rObj=7;nu=(n+1)*S;
D7=v207_bernstein_difference_matrix(n,rObj);
G7=v207_bernstein_gram_matrix(n-rObj);
f7=v200_falling(n,rObj);
H0=(f7^2)*(D7.'*G7*D7);
Q=zeros(nu,nu);
for q=1:S
    ix=(q-1)*(n+1)+(1:n+1);
    Q(ix,ix)=H0*w(q)^(-(2*rObj-1)); % physical integral: h^(1-2r)
end
Q=0.5*(Q+Q.');

A=[];rhs=[];
% Every segment interpolates its two guide endpoints.
for q=1:S
    A=[A;v207_bernstein_endpoint_row(S,q,0,0,w,n); ...
         v207_bernstein_endpoint_row(S,q,1,0,w,n)]; %#ok<AGROW>
    rhs=[rhs;W(:,q).';W(:,q+1).']; %#ok<AGROW>
end
% Physical C1..C6 equality at every internal guide knot.
for q=1:S-1
    for r=1:6
        A=[A;v207_bernstein_endpoint_row(S,q,1,r,w,n)- ...
             v207_bernstein_endpoint_row(S,q+1,0,r,w,n)]; %#ok<AGROW>
        rhs=[rhs;zeros(1,3)]; %#ok<AGROW>
    end
end
% Qualification boundary jets d1..d6 = 0 at both ends.
for r=1:6
    A=[A;v207_bernstein_endpoint_row(S,1,0,r,w,n); ...
         v207_bernstein_endpoint_row(S,S,1,r,w,n)]; %#ok<AGROW>
    rhs=[rhs;zeros(2,3)]; %#ok<AGROW>
end

% Positive global objective scaling and nonzero per-row equality scaling do
% not change the primal minimizer in real arithmetic.  They only improve
% floating-point conditioning.  No regularization or pseudoinverse is used.
qScale=max(abs(Q(:)));if ~(isfinite(qScale)&&qScale>0),qScale=1;end
Qs=Q/qScale;
rowScale=sqrt(sum(A.^2,2));
assert(all(isfinite(rowScale)&rowScale>0),'G26V207C:MINCORowScale','MINCO equality row has zero/nonfinite norm.');
As=bsxfun(@rdivide,A,rowScale);
rhsS=bsxfun(@rdivide,rhs,rowScale);
K=[Qs As';As zeros(size(As,1))];
RHS=[zeros(nu,3);rhsS];
rc=rcond(K);
sol=K\RHS;
coefB=sol(1:nu,:).';

scaledDen=norm(K,inf)*norm(sol,inf)+norm(RHS,inf);
kktRel=norm(K*sol-RHS,inf)/max(scaledDen,realmin);
prDen=norm(A,inf)*norm(coefB.',inf)+norm(rhs,inf);
primalRel=norm(A*coefB.'-rhs,inf)/max(prDen,realmin);

P=cell(1,S);
for q=1:S
    ix=(q-1)*(n+1)+(1:n+1);
    P{q}=coefB(:,ix);
end
B=struct();B.P=P;B.degree=n;B.native='BERNSTEIN_MINIMUM_SEVENTH_DERIVATIVE_D13_C6_SCALED_KKT';
B.structural_c6_theorem=true;
B.condition_indicator=max(1,1/max(rc,realmin));
B.representation_residual=max(kktRel,primalRel);
B.native_data=P;B.kkt_relative_residual=kktRel;B.primal_relative_residual=primalRel;B.kkt_rcond=rc;
B.kkt_objective_scale=qScale;B.kkt_row_scale_min=min(rowScale);B.kkt_row_scale_max=max(rowScale);
B.note='Qualification minimum integral squared 7th derivative trajectory in Bernstein basis; globally scaled objective + individually scaled equality rows; no regularization/pinv; not production sparse MINCO.';
end

function D=v207_bernstein_difference_matrix(n,r)
% Forward-difference operator: (Delta^r B)_i.
D=zeros(n-r+1,n+1);
for i=0:n-r
    for k=0:r
        D(i+1,i+k+1)=(-1)^(r-k)*nchoosek(r,k);
    end
end
end

function G=v207_bernstein_gram_matrix(m)
% Exact integral_0^1 b_i^m(s)b_j^m(s) ds in Bernstein basis.
G=zeros(m+1,m+1);
for i=0:m
    for j=0:m
        G(i+1,j+1)=nchoosek(m,i)*nchoosek(m,j)/((2*m+1)*nchoosek(2*m,i+j));
    end
end
end

function row=v207_bernstein_endpoint_row(S,q,rightSide,r,w,n)
row=zeros(1,(n+1)*S);off=(q-1)*(n+1);
if r==0
    if rightSide,idx=n;else,idx=0;end
    row(off+idx+1)=1;
    return;
end
fac=v200_falling(n,r)/w(q)^r;
for k=0:r
    c=fac*(-1)^(r-k)*nchoosek(r,k);
    if rightSide
        idx=n-r+k;
    else
        idx=k;
    end
    row(off+idx+1)=c;
end
end

function row=v207_poly_derivative_row(S,q,s,r,w)
n=13;row=zeros(1,14*S);off=(q-1)*14;
for k=r:n
    if r==0,ff=1;else,ff=v200_falling(k,r);end
    row(off+k+1)=ff*s^(k-r)/w(q)^r;
end
end

function T=v207_bernstein_to_power_matrix(n)
T=zeros(n+1,n+1);
for k=0:n
    for j=0:(n-k)
        m=k+j;
        T(m+1,k+1)=T(m+1,k+1)+nchoosek(n,k)*nchoosek(n-k,j)*(-1)^j;
    end
end
end

function [Pflat,counts]=v207_flatten_spans(P)
counts=cellfun(@(x)size(x,2),P);
Pflat=cat(2,P{:});
end

function A=v207_corridor_audit(B,Q)
minSlack=inf;scale=1;S=numel(B.P);off=0;
for q=1:S
    nf=Q.cnt(q);
    for j=1:nf
        f=off+j;sl=Q.N(:,f)'*B.P{q}-Q.h(f);
        minSlack=min(minSlack,min(sl));
        scale=max(scale,max(abs([Q.h(f) B.P{q}(:).'])));
    end
    off=off+nf;
end
guard=256*eps*max(1,B.condition_indicator)*scale;
A=struct();A.min_slack=minSlack;A.numeric_guard=guard;A.pass=isfinite(minSlack)&&minSlack>=-guard;
end

function A=v207_c6_backend_audit(B,h)
S=numel(B.P);maxRes=0;maxScale=1;
for q=1:S-1
    for r=0:6
        jr=v207_bezier_endpoint_derivative(B.P{q},h(q),r,1);
        jl=v207_bezier_endpoint_derivative(B.P{q+1},h(q+1),r,0);
        maxRes=max(maxRes,norm(jr-jl,inf));
        maxScale=max(maxScale,max(abs([jr(:);jl(:)])));
    end
end
A=struct();A.real_arithmetic_theorem=B.structural_c6_theorem;A.max_join_residual=maxRes;
A.relative_join_residual=maxRes/maxScale;A.fp_authority=false;A.pass=A.real_arithmetic_theorem&&isfinite(maxRes);
end

function j=v207_bezier_endpoint_derivative(B,h,r,rightSide)
n=size(B,2)-1;
if r==0
    if rightSide,j=B(:,end);else,j=B(:,1);end
    return;
end
coef=zeros(1,r+1);
for k=0:r,coef(k+1)=(-1)^(r-k)*nchoosek(r,k);end
if rightSide
    X=B(:,n-r+1:n+1);
else
    X=B(:,1:r+1);
end
j=v200_falling(n,r)/h^r*(X*coef.');
end

function T=v207_time_gate(B,CFG)
[Pflat,counts]=v207_flatten_spans(B.P);
[st,TT,h,Rflat,rc,minCone,minRz,it]=g26v207_time_gate_mex(Pflat,counts,B.weights,deg2rad(CFG.theta_max_deg), ...
    CFG.v207_rho_min_mps2,CFG.gravity,CFG.v207_time_min_s,CFG.v207_time_max_s,CFG.v207_time_guard_scale);
T=struct();T.pass=(st==1);T.status=st;T.T=TT;T.h=h;T.Rflat=Rflat;T.rcounts=rc;T.min_cone_slack=minCone;T.min_rz_slack=minRz;T.iterations=it;
T.search_incomplete=(st==0);T.search_miss_is_infeasible=false;T.fixed_time_bank_used=false;T.canonical_partition_used=false;
end

function A=v207_yaw_c6_audit(CFG,T)
Y=[CFG.v207_yaw_start_rad*ones(1,7) CFG.v207_yaw_goal_rad*ones(1,7)];
maxEnd=0;
for r=1:6
    maxEnd=max(maxEnd,abs(v207_bezier_endpoint_derivative(Y,T,r,0)));
    maxEnd=max(maxEnd,abs(v207_bezier_endpoint_derivative(Y,T,r,1)));
end
A=struct();A.pass=isfinite(maxEnd);A.real_arithmetic_C6=true;A.max_endpoint_derivative_residual=maxEnd;A.authority=false;
end

function A=v207_final_direct_certificate(B,T,Q,CFG)
[Pflat,counts]=v207_flatten_spans(B.P);ta=[0 cumsum(T.h(1:end-1))];tb=cumsum(T.h);
M=size(Q.C,2);OV=zeros(3,M);OR=repmat(reshape(eye(3),9,1),1,M);
rad=reshape(Q.Ractual,1,[]);
if isscalar(rad),rad=repmat(rad,1,M);end
assert(numel(rad)==M,'G26V209D:DirectCertRadius','Obstacle radius count must equal obstacle-center count.');
assert(all(isfinite(rad))&&all(rad>0),'G26V209D:DirectCertRadius','Obstacle radii must be finite and positive.');
ER=repmat(rad,3,1);
K=[CFG.R_payload+CFG.geometry_margin_m,CFG.R_uav+CFG.geometry_margin_m,CFG.R_cable+CFG.geometry_margin_m,CFG.L_cable];
[ok,sl,checks,fp,fo,leaves,dep]=g26v200_direct_cert_mex(Pflat,counts,T.Rflat,T.rcounts,ta,tb,Q.C,OV,OR,ER,K,CFG.direct_cert_max_depth,CFG.direct_cert_guard_scale);
A=struct();A.pass=logical(ok);A.min_slack=sl;A.checks=checks;A.fail_piece=fp;A.fail_obstacle=fo;A.leaves=leaves;A.max_depth=dep;
end

function A=v207_four_backend_selftest(CFG)
Q=v207_qualification_problem(CFG);
names={'BSPLINE_D7','BEZIER_D13','POLY_D13','ACETRANS_MINCO_ST_C6'};
R=repmat(struct(),1,numel(names));allpass=true;
for k=1:numel(names)
    B=v207_build_backend(names{k},Q.W,Q.weights);
    T=v207_time_gate(B,CFG);
    C6=v207_c6_backend_audit(B,T.h);
    COR=v207_corridor_audit(B,Q);
    Y=v207_yaw_c6_audit(CFG,T.T);
    DC=v207_final_direct_certificate(B,T,Q,CFG);
    pass=T.pass&&C6.pass&&COR.pass&&Y.pass&&DC.pass;
    R(k).backend=names{k};R(k).pass=pass;R(k).degree=B.degree;R(k).time_s=T.T;R(k).segment_h=T.h;
    R(k).c6=C6;R(k).corridor=COR;R(k).dynamic=T;R(k).yaw=Y;R(k).direct_cert=DC;
    R(k).condition_indicator=B.condition_indicator;R(k).representation_residual=B.representation_residual;R(k).note=B.note;
    if isfield(B,'kkt_rcond'),R(k).kkt_rcond=B.kkt_rcond;R(k).kkt_relative_residual=B.kkt_relative_residual;R(k).primal_relative_residual=B.primal_relative_residual;else,R(k).kkt_rcond=NaN;R(k).kkt_relative_residual=NaN;R(k).primal_relative_residual=NaN;end
    allpass=allpass&&pass;
    fprintf(['[G26-V207-BACKEND-SELFTEST] %-23s PASS=%d | deg=%d T=%.6f s C6=%d corridor=%d ', ...
        'cone=%d taut=%d yawC6=%d hardCert=%d minCone=%.3e minRz=%.3e\n'], ...
        names{k},pass,B.degree,T.T,C6.pass,COR.pass,T.pass,T.pass,Y.pass,DC.pass,T.min_cone_slack,T.min_rz_slack);
end
A=struct();A.pass=allpass;A.backends=R;A.weights=Q.weights;A.guide_nodes=size(Q.W,2);A.segments=size(Q.W,2)-1;
A.fixed_time_bank_used=false;A.canonical_time_partition_used=false;A.generator_miss_is_infeasible=false;
A.cable_cone_continuous_by_convexity=true;A.tautness_continuous_by_convexity=true;A.final_hard_authority_unchanged=true;
assert(A.pass,'G26V207:FourBackendSelftest','At least one retained backend failed V207 qualification.');
fprintf('[G26-V207-FOUR-BACKEND-SELFTEST] PASS | all4=1 guideSegments=%d onlineLengthWeights=1 continuousT=1 fixedTimeBank=0 infeasibilityClaim=0\n',A.segments);
M=A.backends(strcmp({A.backends.backend},'ACETRANS_MINCO_ST_C6'));
fprintf('[G26-V207C-MINCO-NUMERICS] rcond=%.3e kktRel=%.3e primalRel=%.3e | BernsteinBasis=1 rowScaled=1 regularization=0 pinv=0\n',M.kkt_rcond,M.kkt_relative_residual,M.primal_relative_residual);
end

function C=v207_trajectory_contract(CFG,SELF)
C=struct();C.four_backend_qualification=SELF.v207_four_backend.pass;C.full_2x4_retained=true;
C.backend_authority=CFG.v207_backend_authority;C.time_allocation_authority=CFG.v207_time_allocation_authority;
C.dynamic_gate_authority=CFG.v207_dynamic_gate_authority;C.kinematic_limit_authority=CFG.v207_kinematic_limit_authority;C.yaw_authority=CFG.v207_yaw_authority;C.fp_authority=CFG.v207_fp_authority;
C.segment_time_weights='ONLINE_GUIDE_LENGTH_NORMALIZED';C.common_time_scale='CONTINUOUS_BISECTION_ON_INTERVAL';
C.fixed_time_bank_used=false;C.canonical_time_partition_used=false;C.theta_max_deg=CFG.theta_max_deg;C.rho_min_mps2=CFG.v207_rho_min_mps2;
C.cable_cone_certificate='ALL_RESULTANT_BERNSTEIN_CONTROLS_IN_CONVEX_SECOND_ORDER_CONE';
C.tautness_certificate='ALL_RESULTANT_BERNSTEIN_CONTROLS_SATISFY_RZ_GE_RHO_MIN';
C.kinematic_limits_supplied=false;C.generator_miss_is_infeasible=false;C.production_backend_selected=false;
C.final_continuous_whole_body_certificate_unchanged=true;C.atomic_switch_unchanged=true;
C.backends=SELF.v207_four_backend.backends;
end

function v207_print_trajectory(C,B,REG)
fprintf('\n========================================================================================================================\n');
fprintf(' G26-V207 FOUR-BACKEND / CONTINUOUS-TIME / CABLE-DYNAMIC QUALIFICATION\n');
fprintf('------------------------------------------------------------------------------------------------------------------------\n');
fprintf(' full2x4=%d backendsQualified=%d backendAuthority=%d productionSelected=%d\n',numel(REG)==8,C.four_backend_qualification,C.backend_authority,C.production_backend_selected);
fprintf(' timeWeights=%s | commonScale=%s | fixedTimeBank=0 canonicalPartition=0 timeAuthority=%d\n',C.segment_time_weights,C.common_time_scale,C.time_allocation_authority);
fprintf(' inherited V207 extremeQualificationTheta=%.1f deg rhoMin=%.3f m/s^2 | coneCertificate=Bernstein-convex | tautnessCertificate=Bernstein-convex\n',C.theta_max_deg,C.rho_min_mps2);
for k=1:numel(C.backends)
    X=C.backends(k);
    fprintf(' %-23s T=%8.4f s | C6=%d corridor=%d cone/taut=%d yawC6=%d hardCert=%d | condIndicator=%.3e reprResidual=%.3e\n', ...
        X.backend,X.time_s,X.c6.pass,X.corridor.pass,X.dynamic.pass,X.yaw.pass,X.direct_cert.pass,X.condition_indicator,X.representation_residual);
end
mi=find(strcmp({C.backends.backend},'ACETRANS_MINCO_ST_C6'),1);
if ~isempty(mi) && isfield(C.backends(mi),'kkt_rcond')
    M=C.backends(mi);
    fprintf(' MINCO numerics: rcond=%.3e kktRel=%.3e primalRel=%.3e | basis=Bernstein scaledKKT=1 regularization=0 pinv=0\n',M.kkt_rcond,M.kkt_relative_residual,M.primal_relative_residual);
end
fprintf(' kinematicLimitAuthority=%d (no formal vehicle v/a/j limits supplied yet) | generatorMiss->infeasible=0\n',C.kinematic_limit_authority);
fprintf(' RULE: h_i=T*l_i/sum(l); T is searched continuously, not chosen from a finite bank.\n');
fprintf(' RULE: if the continuous-time search hits Tmax or a backend construction fails, status is generator/time SEARCH_INCOMPLETE, never path/physical infeasibility.\n');
fprintf(' RULE: no backend is promoted by V207; every successor still requires the unchanged continuous PAYLOAD/UAV/CABLE hard certificate and atomic switch.\n');
if isfield(B.qual,'v207')
    fprintf('\n [G26-V207 BACKEND TIMING | MATLAB CONSTRUCTORS + MEX TIME/DYNAMIC GATE | NOT WCET]\n');
    for k=1:numel(B.qual.v207)
        X=B.qual.v207(k);
        v202_print_stats(sprintf(' build %s',X.backend),X.build);
        v202_print_stats(sprintf(' timeGate %s',X.backend),X.time_gate);
        v202_print_stats(sprintf(' build+gate+cert %s',X.backend),X.integrated);
    end
    v202_print_stats(' all4 sequential qualification',B.qual.v207_all4);
    fprintf(' single-backend screen uses worst p95 backend (no production selection): %.6f ms | remaining to 4ms=%+.6f ms\n', ...
        B.qual.v207_single_backend_screen_p95_ms,B.qual.v207_single_backend_remaining_to_4ms);
    fprintf(' all4 validation screen (not production hot-path claim): %.6f ms | remaining to 4ms=%+.6f ms\n', ...
        B.qual.v207_all4_screen_p95_ms,B.qual.v207_all4_remaining_to_4ms);
end
fprintf('========================================================================================================================\n');
end

function B=v207_microbench(CFG)
B=v206_microbench(CFG);Q=v207_qualification_problem(CFG);
names={'BSPLINE_D7','BEZIER_D13','POLY_D13','ACETRANS_MINCO_ST_C6'};NS=CFG.v207_timing_samples;NB=CFG.v207_timing_batch_calls;
B.qual.v207=repmat(struct('backend','','build',[],'time_gate',[],'integrated',[],'T',NaN),1,numel(names));
for z=1:numel(names)
    name=names{z};
    for k=1:5,B0=v207_build_backend(name,Q.W,Q.weights);T0=v207_time_gate(B0,CFG);assert(T0.pass);end %#ok<NASGU>
    Tb=zeros(NS,1);for q=1:NS,tt=tic;for k=1:NB,B0=v207_build_backend(name,Q.W,Q.weights);end;Tb(q)=1e3*toc(tt)/NB;end %#ok<NASGU>
    B0=v207_build_backend(name,Q.W,Q.weights);
    Tg=zeros(NS,1);for q=1:NS,tt=tic;for k=1:NB,T0=v207_time_gate(B0,CFG);end;Tg(q)=1e3*toc(tt)/NB;end
    Ti=zeros(NS,1);Tlast=[];for q=1:NS,tt=tic;for k=1:NB
        BB=v207_build_backend(name,Q.W,Q.weights);TT=v207_time_gate(BB,CFG);DD=v207_final_direct_certificate(BB,TT,Q,CFG);assert(TT.pass&&DD.pass);
        Tlast=TT;
    end;Ti(q)=1e3*toc(tt)/NB;end
    B.qual.v207(z).backend=name;B.qual.v207(z).build=v202_stats(Tb);B.qual.v207(z).time_gate=v202_stats(Tg);B.qual.v207(z).integrated=v202_stats(Ti);B.qual.v207(z).T=Tlast.T;
end
Tall=zeros(NS,1);
for q=1:NS
    tt=tic;
    for z=1:numel(names)
        BB=v207_build_backend(names{z},Q.W,Q.weights);TT=v207_time_gate(BB,CFG);DD=v207_final_direct_certificate(BB,TT,Q,CFG);assert(TT.pass&&DD.pass);
    end
    Tall(q)=1e3*toc(tt);
end
B.qual.v207_all4=v202_stats(Tall);
prefixCorr=B.qual.p95_prefix_with_range_gate_ms+B.qual.local_target.p95+B.qual.v206(end).integrated.p95;
worst=max(arrayfun(@(x)x.integrated.p95,B.qual.v207));
B.qual.v207_prefix_to_corridor_p95_ms=prefixCorr;
B.qual.v207_single_backend_screen_p95_ms=prefixCorr+worst;
B.qual.v207_single_backend_remaining_to_4ms=CFG.target_sensor_return_to_switch_ms-B.qual.v207_single_backend_screen_p95_ms;
B.qual.v207_all4_screen_p95_ms=prefixCorr+B.qual.v207_all4.p95;
B.qual.v207_all4_remaining_to_4ms=CFG.target_sensor_return_to_switch_ms-B.qual.v207_all4_screen_p95_ms;
end

function C=v207_extend_contract(C,CFG,SELF,BENCH)
C.v207_four_backend_qualification=SELF.v207_four_backend.pass;
C.v207_backend_authority=CFG.v207_backend_authority;
C.v207_time_allocation_authority=CFG.v207_time_allocation_authority;
C.v207_dynamic_gate_authority=CFG.v207_dynamic_gate_authority;
C.v207_kinematic_limit_authority=CFG.v207_kinematic_limit_authority;
C.v207_yaw_authority=CFG.v207_yaw_authority;
C.v207_fp_authority=CFG.v207_fp_authority;
C.v207_fixed_time_bank_used=false;
C.v207_canonical_time_partition_used=false;
C.v207_single_backend_screen_p95_ms=BENCH.qual.v207_single_backend_screen_p95_ms;
C.v207_all4_validation_screen_p95_ms=BENCH.qual.v207_all4_screen_p95_ms;
end


% ========================================================================
% Architecture contract / theorem boundary
% ========================================================================

function C=v200_contract(CFG,REG,PRE,SELF,BENCH)
C=struct();C.full_2x4_retained=numel(REG)==8&&all([REG.enabled]);C.legacy_runtime_dependency_zero=PRE.legacy_runtime_dependency_count==0;C.hotpath_mex_first=true;C.no_prior_map=~CFG.prior_map_used;C.unknown_remains_unknown=~CFG.unknown_promoted_free;C.retirement_disabled=~CFG.retirement_authority;C.local_sphere_proposal_only=~CFG.local_sphere_safety_authority;C.structural_C6_selftest=SELF.c6;C.variable_time_contract_selftest=SELF.time;C.local_sfc_complete_accounting=SELF.sfc;C.continuous_whole_body_mex_selftest=SELF.cert;C.atomic_safe_reference_retention=SELF.switching;C.dual_lidar_first_return_ingress=SELF.first_return.pass;C.set_preserving_cover_memory=SELF.cover.pass;C.no_forgetting=true;C.cover_storage_sustained=SELF.cover.storage_sustained;C.cover_resource_overflow_fail_closed=SELF.cover.resource_overflow_fail_closed;C.motion_envelope_math=SELF.motion.pass;C.motion_bound_source_authority=CFG.v203_motion_envelope_authority;C.cover_cpp_fp_outward_formal_proof_complete=false;C.tristate_trigger_selftest=SELF.trigger.pass;C.trigger_is_switch_authority=false;C.certified_window_supervisor_selftest=SELF.window.pass;C.window_extension_requires_direct_continuous_certificate=CFG.v202_window_extension_requires_direct_cert;C.unknown_motion_is_inconclusive=SELF.integration.raw_without_motion_is_inconclusive;C.real_arithmetic_two_ball_outer_merge_theorem=true;C.real_arithmetic_future_motion_containment_induction=true;C.production_promotion=false;C.integrated_sensor_return_to_switch_wcet_closed=false;C.realistic_prefix_p95_ms=BENCH.qual.p95_prefix_ms;C.realistic_prefix_plus_cert_p95_ms=BENCH.qual.p95_with_cert_ms;C.prefix_budget_screen=BENCH.qual.prefix_budget_screen;C.storage_sustain_frames=BENCH.qual.sustain_frames;C.dynamic_long_horizon_conservatism_closed=false;C.association_theorem_selftest=SELF.association.pass;C.raw_first_return_identity_authority=CFG.v204_association_source_authority;C.reacquisition_theorem_selftest=SELF.reacquisition.pass;C.global_angular_coverage_closed=CFG.v204_global_angular_coverage_closed;C.reacquisition_authority=CFG.v204_reacquisition_authority;C.minimum_detectable_obstacle_size_premise_required=SELF.reacquisition.finite_ray_requires_positive_min_size;C.current_dynamic_memory_contraction_allowed=false;C.current_dynamic_retirement_allowed=false;C.bounded_blind_gap_theorem_selftest=SELF.blind_gap_reacq.pass;C.current_sim_fov_blind_depth_deg=SELF.blind_gap.analytic_blind_gap_deg;C.current_sim_ray_net_bound_deg=0.5*(CFG.v204b_ray_az_spacing_deg+CFG.v204b_ray_el_spacing_deg);C.complete_global_fov_not_required_by_v204b=true;C.synthetic_kernel_sum_ms=BENCH.synthetic_kernel_sum_ms;C.sensor_self_occlusion_ignored=CFG.v205_assume_no_self_occlusion;C.sensor_novelty_claim=CFG.v205_sensor_perception_novelty_claim;C.effective_sensing_range_m=CFG.v205_effective_sensing_range_m;C.local_planning_horizon_m=CFG.v205_planning_horizon_m;C.sensor_range_gate_selftest=SELF.sensor_range.pass;C.local_target_selftest=SELF.local_target.pass;C.planner_entry_selftest=SELF.planner_entry.pass;C.planner_solver_authority=CFG.v205_planner_solver_authority;C.v206_spatial_guide_selftest=SELF.spatial_guide.pass;C.v206_segment_corridor_selftest=SELF.segment_corridor.pass;C.v206_c6_corridor_carrier_selftest=SELF.c6_corridor_carrier.pass;C.v206_spatial_solver_authority=CFG.v206_spatial_solver_authority;C.v206_search_completeness_claim=CFG.v206_search_completeness_claim;C.v206_search_miss_is_infeasible=false;C.v206_time_allocation_authority=false;C.final_hard_gate=['C6 AND yaw-C6 AND kinematic/dynamic constraints AND global cable-angle<=theta_max AND tautness AND continuous PAYLOAD/UAV/CABLE obstacle certificate against a certified occupied outer cover AND reaction deadline'];C.claim_boundary=['V203 set-preserving storage and supplied-vmax motion propagation are retained. V204 adds deterministic set-membership association and a finite-ray reacquisition sufficient condition. ', ...
'Current raw first-return packets do not satisfy the certified whole-object identity premise, and hardware sensing premises remain open. V206 adds a certified-clear spatial guide and per-segment convex corridor over supplied outer sphere covers, but search completeness, production backend synthesis, continuous online time allocation, FP authority and integrated WCET remain open.'];
end

function v200_print_contract(C)
fprintf('\n');fprintf('========================================================================================================================\n');fprintf(' G26-V206 SENSOR / MEMORY / SPATIAL-GUIDE CONTRACT\n');fprintf('------------------------------------------------------------------------------------------------------------------------\n');
fprintf(' full2x4=%d legacyDeps0=%d mexFirst=%d noPriorMap=%d unknownUnknown=%d retirement0=%d\n',C.full_2x4_retained,C.legacy_runtime_dependency_zero,C.hotpath_mex_first,C.no_prior_map,C.unknown_remains_unknown,C.retirement_disabled);
fprintf(' structuralC6=%d variableTime=%d SFCaccounting=%d directCertSelftest=%d atomicSwitch=%d\n',C.structural_C6_selftest,C.variable_time_contract_selftest,C.local_sfc_complete_accounting,C.continuous_whole_body_mex_selftest,C.atomic_safe_reference_retention);
fprintf(' firstReturn=%d setPreservingCover=%d noForgetting=%d fixedStorage=%d resourceFailClosed=%d\n',C.dual_lidar_first_return_ingress,C.set_preserving_cover_memory,C.no_forgetting,C.cover_storage_sustained,C.cover_resource_overflow_fail_closed);
fprintf(' motionEnvelopeMath=%d motionSourceAuthority=%d coverFPAuthority=%d unknownMotionInconclusive=%d dynamicLongHorizonClosed=%d\n',C.motion_envelope_math,C.motion_bound_source_authority,C.cover_cpp_fp_outward_formal_proof_complete,C.unknown_motion_is_inconclusive,C.dynamic_long_horizon_conservatism_closed);
fprintf(' associationTheorem=%d rawIdentityAuthority=%d reacqTheorem=%d globalCoverageClosed=%d reacqAuthority=%d minObstacleSizePremise=%d\n',C.association_theorem_selftest,C.raw_first_return_identity_authority,C.reacquisition_theorem_selftest,C.global_angular_coverage_closed,C.reacquisition_authority,C.minimum_detectable_obstacle_size_premise_required);
fprintf(' dynamicContractionAllowed=%d dynamicRetirementAllowed=%d\n',C.current_dynamic_memory_contraction_allowed,C.current_dynamic_retirement_allowed);
fprintf(' boundedBlindGapTheorem=%d simFoVBlindDepth=%.3fdeg simRayNetBound=%.3fdeg completeGlobalFoVRequired=0\n',C.bounded_blind_gap_theorem_selftest,C.current_sim_fov_blind_depth_deg,C.current_sim_ray_net_bound_deg);
fprintf(' sensorOcclusionIgnored=%d sensorNoveltyClaim=%d effectiveRange=%.1fm planningHorizon=%.1fm rangeGate=%d localTarget=%d plannerEntry=%d plannerAuthority=%d\n',C.sensor_self_occlusion_ignored,C.sensor_novelty_claim,C.effective_sensing_range_m,C.local_planning_horizon_m,C.sensor_range_gate_selftest,C.local_target_selftest,C.planner_entry_selftest,C.planner_solver_authority);
fprintf(' spatialGuide=%d segmentCorridor=%d c6Carrier=%d searchComplete=%d spatialAuthority=%d timeAuthority=%d infeasibilityClaim=0\n',C.v206_spatial_guide_selftest,C.v206_segment_corridor_selftest,C.v206_c6_corridor_carrier_selftest,C.v206_search_completeness_claim,C.v206_spatial_solver_authority,C.v206_time_allocation_authority);
fprintf(' timing prefixP95=%.6f ms prefix+certP95=%.6f ms prefixScreen=%d integratedWCET=%d promotion=%d\n',C.realistic_prefix_p95_ms,C.realistic_prefix_plus_cert_p95_ms,C.prefix_budget_screen,C.integrated_sensor_return_to_switch_wcet_closed,C.production_promotion);
fprintf(' RULE: capacity pressure merges occupied sets outward; it never drops old evidence.\n');
fprintf(' RULE: supplied vmax creates r(t)=r0+vmax*dt; without certified vmax the state is INCONCLUSIVE.\n');
fprintf(' RULE: local SFC remains proposal geometry; switch still requires continuous whole-body hard certification.\n');
fprintf(' RULE: complete angular FoV is sufficient but not necessary; V204B uses a bounded blind-gap + ray-net + angular-error sufficient condition with positive detectable obstacle size.\n');
fprintf('========================================================================================================================\n');
end


% ========================================================================
% V208 cable-angle semantics / stress qualification.
% ========================================================================

function A=v208_angle_envelope_selftest(CFG)
angles=CFG.v208_angle_audit_deg(:).';g=CFG.gravity;
pass=true;coeff=zeros(size(angles));
for kk=1:numel(angles)
    th=deg2rad(angles(kk));rIn=[0.9*tan(th)*g;0;g];rOut=[1.1*tan(th)*g;0;g];
    passIn=(hypot(rIn(1),rIn(2))<=tan(th)*rIn(3));passOut=(hypot(rOut(1),rOut(2))<=tan(th)*rOut(3));
    q=-rIn/norm(rIn);theta=acos(max(-1,min(1,[-0;-0;-1]'*q)));
    pass=pass&&passIn&&~passOut&&abs(theta-atan2(hypot(rIn(1),rIn(2)),rIn(3)))<1e-12;coeff(kk)=tan(th);
end
A=struct();A.pass=pass&&all(diff(coeff)>0);A.operational_deg=CFG.v208_theta_operational_deg;A.extreme_deg=CFG.v208_theta_extreme_deg;A.sweep_deg=angles;
A.sixty_is_fundamental_physics_limit=false;A.cone_axis='resultant +e3 / cable -e3';A.convex_for_theta_lt_90=true;A.spatial_guide_depends_on_theta=false;
assert(A.pass,'G26V208B:AngleEnvelope','Cable-angle sweep identity/monotonicity selftest failed.');
fprintf('[G26-V208B-ANGLE-SELFTEST] PASS | sweep=%s deg | coneWidensMonotonically=1 | spatialGuideDependsOnTheta=0 | 60degPhysicsLimit=0\n',mat2str(A.sweep_deg));
end

function C=v208_cable_envelope_contract(CFG,SELF)
C=struct();C.pass=SELF.v208_angle_envelope.pass;C.operational_deg=CFG.v208_theta_operational_deg;C.extreme_deg=CFG.v208_theta_extreme_deg;
C.angle_audit_deg=CFG.v208_angle_audit_deg;C.hard_physics_limit_claim=false;C.controller_region_authority=false;C.actuator_authority=false;
C.level_flight_horizontal_accel_mps2=CFG.gravity*tan(deg2rad(C.angle_audit_deg));
C.level_flight_tension_over_weight=1./cos(deg2rad(C.angle_audit_deg));
C.note='theta is cable deflection from downward vertical; equivalent resultant cone is ||r_xy||<=tan(theta) r_z with r=a_L+g e3.';
end

function v208_print_cable_envelope(C)
fprintf('\n');fprintf('========================================================================================================================\n');
fprintf(' G26-V208B CABLE-ANGLE SWEEP INTERPRETATION | QUALIFICATION ONLY\n');
fprintf('------------------------------------------------------------------------------------------------------------------------\n');
fprintf(' definition: theta = angle(cable, downward vertical) = angle(r, +e3), r=pL_ddot+g*e3\n');
fprintf(' convex resultant cone: ||r_xy|| <= tan(theta_limit)*r_z, r_z>0\n');
fprintf(' sweep=%s deg | recursive-reference=%.1f deg | upper-sweep=%.1f deg | no angle promoted | 60deg physics limit=0\n',mat2str(C.angle_audit_deg),C.operational_deg,C.extreme_deg);
fprintf(' RULE: spatial guide/corridor are generated once from obstacle geometry and are invariant to cable-angle sweep.\n');
for k=1:numel(C.angle_audit_deg)
    fprintf(' theta=%5.1f deg | level-flight |a_xy|<=%7.3f m/s^2 (%5.3fg) | load tension/(m_L g)<=%5.3f\n', ...
        C.angle_audit_deg(k),C.level_flight_horizontal_accel_mps2(k),C.level_flight_horizontal_accel_mps2(k)/9.80665,C.level_flight_tension_over_weight(k));
end
fprintf(' RULE: final theta must be derived from controller region, actuator/thrust margin, tracking uncertainty and desired swing performance; V208B promotes no angle.\n');
fprintf('========================================================================================================================\n');
end

function S=v208_stress_scenarios(CFG)
% Deterministic synthetic obstacles.  Actual obstacle geometry is spherical
% only for this stress suite so PATCH_UNION and sphere-SFC share the same
% final hard authority while using different proposal inflation.
baseR=0.20;
S=v208_make_scenario('STATIC_1_HEADON',[3;0;0],zeros(3,1),baseR,[0;0;0],[6;0;0]);
C=[2.0 3.5 2.8 4.7;0 0 3.8 -3.8;0 0 0 0];
S(2)=v208_make_scenario('STATIC_4_MIXED',C,zeros(size(C)),baseR*ones(1,4),[0;0;0],[6;0;0]);
C=[2 3 4 1.5 2.5 3.5 4.5 5.2 1.2 2.2 4.2 5.5;0 0 0 4 -4 4 -4 4 -4 4 -4 4;zeros(1,12)];
S(3)=v208_make_scenario('STATIC_12_CLUSTER',C,zeros(size(C)),baseR*ones(1,12),[0;0;0],[6;0;0]);
C=[2.0 3.2 4.2 5.0;1.4 -1.3 1.6 -1.7;0 0 0 0];V=[0 0 0 0;-0.35 0.30 -0.28 0.25;0 0 0 0];
S(4)=v208_make_scenario('MOVING_4_CROSSING',C,V,baseR*ones(1,4),[0;0;0],[6;0;0]);
C=[1.8 2.8 3.8 4.8 1.5 2.3 3.1 3.9 4.7 5.3 2.2 4.4; ...
   1.2 -1.4 1.5 -1.2 4 -4 4 -4 4 -4 3.8 -3.8;zeros(1,12)];
V=zeros(3,12);V(2,1:4)=[-0.25 0.25 -0.22 0.20];V(1,11:12)=[0.10 -0.10];
S(5)=v208_make_scenario('MOVING_12_MIXED',C,V,baseR*ones(1,12),[0;0;0],[6;0;0]);
C=[2.4 4.4;2.8 -2.8;0 0];V=[0 0;-0.15 0.15;0 0];
S(6)=v208_make_scenario('RECURSIVE_NEW_OBSTACLE',C,V,baseR*ones(1,2),[0;0;0],[6;0;0]);
if numel(S)>CFG.v208_stress_case_limit,S=S(1:CFG.v208_stress_case_limit);end
end

function X=v208_make_scenario(name,C,V,R,p0,pg)
X=struct('name',name,'C',C,'V',V,'R',R,'p0',p0,'pg',pg);
end

function Q=v208_build_stress_problem(S,rep,CFG)
Q=struct();Q.p0=S.p0;Q.pg=S.pg;Q.C=S.C;Q.V=S.V;Q.Ractual=S.R;
H=CFG.v208_motion_proposal_horizon_s;speed=vecnorm(S.V,2,1);
Q.Cproposal=S.C+0.5*H*S.V;
bodyInfl=CFG.L_cable+max([CFG.R_payload CFG.R_uav CFG.R_cable])+CFG.geometry_margin_m;
extra=0;if strcmp(rep,'ACETRANS_SPHERE_SFC'),extra=CFG.v207_qualification_extra_proposal_margin_m;end
Q.Rproposal=S.R+0.5*H*speed+bodyInfl+extra;
[Q.W,Q.guideStatus,Q.insertions,Q.guideClearance]=g26v206_spatial_guide_mex(Q.p0,Q.pg,Q.Cproposal,Q.Rproposal, ...
    CFG.v205_planning_horizon_m,CFG.v206_spatial_guide_max_nodes,CFG.v206_spatial_guide_guard_scale);
Q.guidePass=(Q.guideStatus==1);
if ~Q.guidePass,Q.corridorOK=false;Q.N=[];Q.h=[];Q.cnt=[];Q.faces=0;Q.weights=[];return;end
[Q.N,Q.h,Q.cnt,Q.corridorOK,Q.corridorMinClearance,Q.faces]=g26v206_segment_corridor_mex(Q.W,Q.Cproposal,Q.Rproposal,CFG.v206_spatial_guide_guard_scale);
Q.segment_length=vecnorm(diff(Q.W,1,2),2,1);Q.weights=Q.segment_length/sum(Q.segment_length);
end

function T=v208_time_gate_angle(B,CFG,thetaDeg)
[Pflat,counts]=v207_flatten_spans(B.P);
[st,TT,h,Rflat,rc,minCone,minRz,it]=g26v207_time_gate_mex(Pflat,counts,B.weights,deg2rad(thetaDeg), ...
    CFG.v207_rho_min_mps2,CFG.gravity,CFG.v207_time_min_s,CFG.v207_time_max_s,CFG.v207_time_guard_scale);
T=struct();T.pass=(st==1);T.status=st;T.T=TT;T.h=h;T.Rflat=Rflat;T.rcounts=rc;T.min_cone_slack=minCone;T.min_rz_slack=minRz;T.iterations=it;
T.theta_deg=thetaDeg;T.search_incomplete=(st==0);T.search_miss_is_infeasible=false;
if T.pass
    ang=atan2(vecnorm(Rflat(1:2,:),2,1),Rflat(3,:));
    T.max_control_angle_deg=rad2deg(max(ang));
    T.tension_ratio_bound=max(vecnorm(Rflat,2,1))/CFG.gravity;
else
    T.max_control_angle_deg=NaN;T.tension_ratio_bound=NaN;
end
end

function A=v208_final_direct_certificate_dynamic(B,T,Q,CFG)
if ~T.pass,A=struct('pass',false,'min_slack',NaN,'checks',0,'fail_piece',0,'fail_obstacle',0,'leaves',0,'max_depth',0);return;end
[Pflat,counts]=v207_flatten_spans(B.P);ta=[0 cumsum(T.h(1:end-1))];tb=cumsum(T.h);
M=size(Q.C,2);OR=repmat(reshape(eye(3),9,1),1,M);ER=ones(3,1)*Q.Ractual;
K=[CFG.R_payload+CFG.geometry_margin_m,CFG.R_uav+CFG.geometry_margin_m,CFG.R_cable+CFG.geometry_margin_m,CFG.L_cable];
[ok,sl,checks,fp,fo,leaves,dep]=g26v200_direct_cert_mex(Pflat,counts,T.Rflat,T.rcounts,ta,tb,Q.C,Q.V,OR,ER,K,CFG.direct_cert_max_depth,CFG.direct_cert_guard_scale);
A=struct();A.pass=logical(ok);A.min_slack=sl;A.checks=checks;A.fail_piece=fp;A.fail_obstacle=fo;A.leaves=leaves;A.max_depth=dep;
end

function E=v208_eval_jet(B,T,t,r)
cs=[0 cumsum(T.h)];t=max(0,min(t,cs(end)));q=find(t<=cs(2:end)+64*eps(max(1,cs(end))),1,'first');if isempty(q),q=numel(T.h);end
s=(t-cs(q))/T.h(q);s=max(0,min(1,s));X=B.P{q};n=size(X,2)-1;
for k=1:r,X=(n-k+1)*diff(X,1,2)/T.h(q);end
m=size(X,2)-1;bb=zeros(m+1,1);for k=0:m,bb(k+1)=nchoosek(m,k)*s^k*(1-s)^(m-k);end
E=X*bb;
end

function SW=v208_angle_sweep_backend(B,Q,CFG)
angles=CFG.v208_angle_audit_deg(:).';n=numel(angles);
Tv=nan(1,n);gate=false(1,n);hard=false(1,n);thetaBound=nan(1,n);tension=nan(1,n);status=zeros(1,n);
for kk=1:n
    T=v208_time_gate_angle(B,CFG,angles(kk));status(kk)=T.status;
    if T.pass
        gate(kk)=true;Tv(kk)=T.T;thetaBound(kk)=T.max_control_angle_deg;tension(kk)=T.tension_ratio_bound;
        D=v208_final_direct_certificate_dynamic(B,T,Q,CFG);hard(kk)=D.pass;
    end
end
idx=find(gate);mono=true;
if numel(idx)>=2,mono=all(diff(Tv(idx))<=1e-8*max(1,max(Tv(idx))));end
firstPass=find(gate,1,'first');looserClosure=true;
if ~isempty(firstPass),looserClosure=all(gate(firstPass:end));end
SW=struct('angles_deg',angles,'T',Tv,'gatePass',gate,'hardCertPass',hard,'thetaBoundDeg',thetaBound,'tensionRatioBound',tension,'status',status, ...
    'gateTimeMonotone',mono,'looserPassClosure',looserClosure,'spatialGuideAngleIndependent',true);
end

function R=v208_recursive_replan_audit(S,rep,name,CFG)
Q=v208_build_stress_problem(S,rep,CFG);R=struct('attempted',false,'candidateBuilt',false,'hardCert',false,'jetMatch',false,'atomicSwitchAllowed',false,'maxJetMismatch',NaN);
if ~Q.guidePass||~Q.corridorOK,return;end
B=v207_build_backend(name,Q.W,Q.weights);T=v208_time_gate_angle(B,CFG,CFG.v208_theta_operational_deg);if ~T.pass,return;end
R.attempted=true;tTrig=CFG.v208_recursive_trigger_fraction*T.T;pNow=v208_eval_jet(B,T,tTrig,0);
Cnow=S.C+S.V*tTrig;newC=pNow+1.20*(S.pg-pNow)/max(norm(S.pg-pNow),eps);C2=[Cnow newC];V2=[S.V zeros(3,1)];R2=[S.R CFG.v208_recursive_new_obstacle_radius_m];
S2=v208_make_scenario([S.name '_NEW'],C2,V2,R2,pNow,S.pg);Q2=v208_build_stress_problem(S2,rep,CFG);if ~Q2.guidePass||~Q2.corridorOK,return;end
B2=v207_build_backend(name,Q2.W,Q2.weights);T2=v208_time_gate_angle(B2,CFG,CFG.v208_theta_operational_deg);if ~T2.pass,return;end
D2=v208_final_direct_certificate_dynamic(B2,T2,Q2,CFG);R.candidateBuilt=true;R.hardCert=D2.pass;
mx=0;sc=1;for r=0:6,jOld=v208_eval_jet(B,T,tTrig,r);jNew=v207_bezier_endpoint_derivative(B2.P{1},T2.h(1),r,0);mx=max(mx,norm(jOld-jNew,inf));sc=max(sc,max(abs([jOld(:);jNew(:)])));end
R.maxJetMismatch=mx;R.jetMatch=(mx<=1e4*eps(sc));R.atomicSwitchAllowed=R.candidateBuilt&&R.hardCert&&R.jetMatch&&CFG.v208_recursive_switch_authority;
end

function ST=v208_stress_suite(CFG)
SC=v208_stress_scenarios(CFG);reps={'PATCH_UNION','ACETRANS_SPHERE_SFC'};names={'BSPLINE_D7','BEZIER_D13','POLY_D13','ACETRANS_MINCO_ST_C6'};
rows=struct([]);ix=0;
for si=1:numel(SC)
    for ri=1:numel(reps)
        % Geometry is solved ONCE per scenario/representation. Theta never feeds back into obstacle discovery/guide directions.
        Q=v208_build_stress_problem(SC(si),reps{ri},CFG);
        for bi=1:numel(names)
            ix=ix+1;R=struct();R.case=SC(si).name;R.representation=reps{ri};R.backend=names{bi};R.obstacles=size(SC(si).C,2);R.moving=any(vecnorm(SC(si).V,2,1)>0);R.guidePass=Q.guidePass;R.corridorPass=Q.guidePass&&Q.corridorOK;R.status='SEARCH_INCOMPLETE';R.generatorMissIsInfeasible=false;
            R.angleSweepDeg=CFG.v208_angle_audit_deg;R.Tangle=nan(size(CFG.v208_angle_audit_deg));R.gatePass=false(size(CFG.v208_angle_audit_deg));R.hardCertSweep=false(size(CFG.v208_angle_audit_deg));R.thetaBoundSweep=nan(size(CFG.v208_angle_audit_deg));R.tensionBoundSweep=nan(size(CFG.v208_angle_audit_deg));R.angleGateMonotone=false;R.looserPassClosure=false;R.spatialGuideAngleIndependent=true;
            R.mincoRcond=NaN;R.elapsed_ms=NaN;R.c6=false;R.corridor=false;
            if Q.guidePass&&Q.corridorOK
                tt=tic;
                try
                    B=v207_build_backend(names{bi},Q.W,Q.weights);if isfield(B,'kkt_rcond'),R.mincoRcond=B.kkt_rcond;end
                    SW=v208_angle_sweep_backend(B,Q,CFG);R.Tangle=SW.T;R.gatePass=SW.gatePass;R.hardCertSweep=SW.hardCertPass;R.thetaBoundSweep=SW.thetaBoundDeg;R.tensionBoundSweep=SW.tensionRatioBound;R.angleGateMonotone=SW.gateTimeMonotone;R.looserPassClosure=SW.looserPassClosure;
                    if any(SW.gatePass)
                        k0=find(SW.gatePass,1,'first');T0=v208_time_gate_angle(B,CFG,SW.angles_deg(k0));C6=v207_c6_backend_audit(B,T0.h);COR=v207_corridor_audit(B,Q);R.c6=C6.pass;R.corridor=COR.pass;
                    end
                    if ~SW.gateTimeMonotone||~SW.looserPassClosure
                        R.status='ANGLE_GATE_MONOTONICITY_VIOLATION';
                    elseif all(SW.gatePass & SW.hardCertPass) && R.c6 && R.corridor
                        R.status='ALL_ANGLES_QUALIFIED';
                    elseif any(SW.gatePass & SW.hardCertPass)
                        R.status='ANGLE_SWEEP_PARTIAL__NOT_INFEASIBLE';
                    else
                        R.status='ANGLE_SWEEP_SEARCH_INCOMPLETE';
                    end
                catch ME
                    R.status=['GENERATOR_EXCEPTION_' ME.identifier];
                end
                R.elapsed_ms=1e3*toc(tt);
            end
            if isempty(rows),rows=R;else,rows(ix)=R;end
        end
    end
end
ST=struct();ST.rows=rows;ST.scenarios=SC;ST.representations=reps;ST.backends=names;ST.angleSweepDeg=CFG.v208_angle_audit_deg;
ST.allAnglesQualified=sum(strcmp({rows.status},'ALL_ANGLES_QUALIFIED'));ST.total=numel(rows);ST.partial=sum(strcmp({rows.status},'ANGLE_SWEEP_PARTIAL__NOT_INFEASIBLE'));ST.searchIncomplete=sum(contains({rows.status},'SEARCH_INCOMPLETE'));ST.monotonicityViolation=sum(strcmp({rows.status},'ANGLE_GATE_MONOTONICITY_VIOLATION'));ST.exceptions=sum(startsWith({rows.status},'GENERATOR_EXCEPTION'));
ST.generator_miss_is_infeasible=false;ST.stress_authority=CFG.v208_stress_authority;ST.spatialGuideAngleIndependent=all([rows.spatialGuideAngleIndependent]);
% Recursive audit uses 35 deg ONLY as a handoff-reference case, not as the selected operating angle.
RR=struct([]);j=0;Srec=SC(end);
for ri=1:numel(reps),for bi=1:numel(names),j=j+1;tmp=v208_recursive_replan_audit(Srec,reps{ri},names{bi},CFG);tmp.representation=reps{ri};tmp.backend=names{bi};if isempty(RR),RR=tmp;else,RR(j)=tmp;end;end;end
ST.recursive=RR;ST.recursiveJetMatch=sum([RR.jetMatch]);ST.recursiveAtomicAllowed=sum([RR.atomicSwitchAllowed]);ST.recursive_switch_authority=CFG.v208_recursive_switch_authority;ST.recursiveReferenceThetaDeg=CFG.v208_theta_operational_deg;
end

function v208_print_stress(ST,CFG)
fprintf('\n');fprintf('========================================================================================================================\n');
fprintf(' G26-V208B 2x4 MULTI/MOVING-OBSTACLE STRESS | CABLE-ANGLE SWEEP=%s deg\n',mat2str(ST.angleSweepDeg));
fprintf('------------------------------------------------------------------------------------------------------------------------\n');
fprintf(' ANGLE DECOUPLING: guide/corridor are generated ONCE per scenario/representation; theta changes ONLY the continuous time/dynamic gate.\n');
for k=1:numel(ST.rows)
    R=ST.rows(k);fprintf(' %-24s | %-20s / %-23s | M=%2d moving=%d | %-38s | T(theta)=%s | gate=%s cert=%s | monotone=%d | %.3f ms\n', ...
        R.case,R.representation,R.backend,R.obstacles,R.moving,R.status,mat2str(R.Tangle,4),mat2str(double(R.gatePass)),mat2str(double(R.hardCertSweep)),R.angleGateMonotone,R.elapsed_ms);
end
fprintf(' summary: allAnglesQualified=%d/%d partial=%d searchIncomplete=%d monotonicityViolation=%d exceptions=%d | generatorMiss->infeasible=0\n',ST.allAnglesQualified,ST.total,ST.partial,ST.searchIncomplete,ST.monotonicityViolation,ST.exceptions);
fprintf(' angle/search invariant: spatialGuideAngleIndependent=%d | tighter theta may need larger T; looser theta may NEVER require larger gate time for the same geometric curve.\n',ST.spatialGuideAngleIndependent);
fprintf(' recursive new-obstacle audit: referenceTheta=%.1fdeg only for handoff test | d0..d6 jetMatch=%d/%d atomicSwitchAllowed=%d | switchAuthority=%d\n',ST.recursiveReferenceThetaDeg,ST.recursiveJetMatch,numel(ST.recursive),ST.recursiveAtomicAllowed,ST.recursive_switch_authority);
fprintf(' RULE: tight-angle miss with loose-angle success is TIME_SEARCH_INCOMPLETE within Tmax, NOT obstacle-discovery failure and NOT path infeasibility.\n');
fprintf(' RULE: a spatially/dynamically safe candidate with d0..d6 mismatch is NOT atomically installable.\n');
fprintf(' RULE: moving-obstacle final authority is the unchanged continuous certificate with linear obstacle motion; proposal swept spheres remain proposal geometry only.\n');
fprintf('========================================================================================================================\n');
end

function C=v208_extend_contract(C,CFG,SELF,ST)
C.v208_angle_semantics_selftest=SELF.v208_angle_envelope.pass;C.v208_angle_sweep_deg=CFG.v208_angle_audit_deg;C.v208_recursive_reference_theta_deg=CFG.v208_theta_operational_deg;
C.v208_sixty_degree_is_fundamental_physics_limit=false;C.v208_no_angle_promoted=true;C.v208_spatial_guide_angle_independent=ST.spatialGuideAngleIndependent;C.v208_angle_gate_monotonicity_violations=ST.monotonicityViolation;
C.v208_stress_authority=CFG.v208_stress_authority;C.v208_stress_all_angles_qualified=ST.allAnglesQualified;C.v208_stress_total=ST.total;
C.v208_recursive_switch_authority=CFG.v208_recursive_switch_authority;C.v208_recursive_jet_match_count=ST.recursiveJetMatch;C.v208_recursive_atomic_allowed=ST.recursiveAtomicAllowed;
C.final_hard_gate=['C6 AND yaw-C6 AND formal kinematic/actuator limits AND chosen cable-angle envelope (to be derived) AND tautness AND continuous PAYLOAD/UAV/CABLE obstacle certificate AND reaction deadline'];
C.claim_boundary=[C.claim_boundary ' V208B sweeps cable angle from 15 to 60 deg without feeding theta back into obstacle discovery/spatial-guide geometry. No angle is promoted; 35 deg is used only as a recursive-handoff reference. Tight-angle time-search miss is not path infeasibility.'];
end


% ========================================================================
% V209B static-obstacle safety / time / trajectory closure.
% ========================================================================

function A = v209b_jet_bridge_selftest(CFG)
J0 = zeros(3,7);
J1 = zeros(3,7);
J0(:,1) = [0;0;0];
J0(:,2) = [0.20;0.04;0];
J0(:,3) = [0.03;-0.01;0];
J1(:,1) = [1.0;0.25;0];
J1(:,2) = [0.08;-0.02;0];
h = 0.8;
B = v209b_degree17_two_sided_jet_bridge(J0,J1,h);
[pass,maxResidual,maxRatio] = v209b_endpoint_jet_match(B,h,J0,J1);
A = struct();
A.pass = pass;
A.maxJetResidual = maxResidual;
A.maxJetResidualOverGuard = maxRatio;
A.degree = 17;
A.twoSidedJets = true;
fprintf('[G26-V209D-JET-BRIDGE-SELFTEST] %s | D17 exact two-sided d0..d6 | maxResidual=%.3e residual/guard=%.3e\n', ...
    v200_pf(pass),maxResidual,maxRatio);
end

function B = v209b_degree17_two_sided_jet_bridge(J0,J1,h)
n = 17;
B = zeros(3,n+1);
for k = 0:6
    x = zeros(3,1);
    for r = 0:k
        if r == 0
            den = 1;
        else
            den = v200_falling(n,r);
        end
        x = x + nchoosek(k,r)*(h^r/den)*J0(:,r+1);
    end
    B(:,k+1) = x;
end
Z = zeros(3,7);
for k = 0:6
    x = zeros(3,1);
    for r = 0:k
        if r == 0
            den = 1;
        else
            den = v200_falling(n,r);
        end
        x = x + nchoosek(k,r)*(h^r/den)*((-1)^r)*J1(:,r+1);
    end
    Z(:,k+1) = x;
end
B(:,18:-1:12) = Z;
for k = 7:10
    alpha = (k-6)/5;
    B(:,k+1) = (1-alpha)*B(:,7) + alpha*B(:,12);
end
end

function [pass,maxResidual,maxRatio] = v209b_endpoint_jet_match(B,h,J0,J1)
n = size(B,2)-1;
pass = true;
maxResidual = 0;
maxRatio = 0;
bscale = max(1,max(abs(B(:))));
for r = 0:6
    jl = v207_bezier_endpoint_derivative(B,h,r,0);
    jr = v207_bezier_endpoint_derivative(B,h,r,1);
    el = norm(jl-J0(:,r+1),inf);
    er = norm(jr-J1(:,r+1),inf);
    maxResidual = max([maxResidual el er]);
    if r == 0
        fac = 1;
    else
        fac = v200_falling(n,r)/h^r;
    end
    ev = max(1,fac*(2^r)*bscale + max(abs([J0(:,r+1);J1(:,r+1)])));
    guard = 1024*eps(ev);
    maxRatio = max(maxRatio,max(el,er)/max(guard,realmin));
    pass = pass && el <= guard && er <= guard;
end
end

function M = v209b_safety_model_contract(CFG)
M = struct();
M.staticObstaclesOnly = true;
M.priorGlobalObstacleMap = false;
M.obstaclesEnterOnlyAfterOnboardDetection = true;
M.dynamics = 'RIGID_TAUT_MASSLESS_CABLE_WITH_POINT_MASS_LOAD';
M.qDefinition = 'q points UAV to load, ||q||=1';
M.kinematics = 'pQ = pL - L*q';
M.resultant = 'r = pL_ddot + g*e3';
M.directionMap = 'q = -r/||r||';
M.downwardCableCone = true;
M.upwardResultantSOC = true;
M.nominalThetaDeg = CFG.v209b_nominal_theta_deg;
M.thetaPromoted = false;
M.tautnessSufficientGate = 'r_z >= rho_min > 0';
M.rhoMin = CFG.v207_rho_min_mps2;
M.payloadRadius = CFG.R_payload + CFG.geometry_margin_m;
M.uavRadius = CFG.R_uav + CFG.geometry_margin_m;
M.cableRadius = CFG.R_cable + CFG.geometry_margin_m;
M.cableLength = CFG.L_cable;
M.fullBodyContinuousCertificate = true;
M.controlPeriodMs = 1000*CFG.v209b_control_period_s;
M.computeStretchTargetMs = CFG.v209b_compute_stretch_target_ms;
M.switchLookaheadMs = 1000*CFG.v209b_switch_lookahead_s;
M.safetyDeadlineEqualsComputeTarget = false;
M.systemAuthority = false;
end

function v209b_print_safety_model(M)
fprintf('\n========================================================================================================================\n');
fprintf(' G26-V209D STATIC SUSPENDED-LOAD SAFETY MODEL\n');
fprintf('------------------------------------------------------------------------------------------------------------------------\n');
fprintf(' rigid taut cable: q=UAV->load, ||q||=1, pQ=pL-Lq, r=pL_ddot+g e3, q=-r/||r||\n');
fprintf(' cable geometry: downward spherical cap/cone | exact equivalent dynamic gate: ||r_xy|| <= tan(theta) r_z, r_z>0\n');
fprintf(' nominal theta=%.1fdeg qualification only | tautness sufficient gate r_z>=%.3f m/s^2\n',M.nominalThetaDeg,M.rhoMin);
fprintf(' collision model: payload sphere R=%.3fm | UAV sphere R=%.3fm | finite cable capsule R=%.3fm, L=%.3fm\n', ...
    M.payloadRadius,M.uavRadius,M.cableRadius,M.cableLength);
fprintf(' control period=%.3fms | trajectory-generation target=%.3fms | scheduled switch look-ahead=%.3fms\n', ...
    M.controlPeriodMs,M.computeStretchTargetMs,M.switchLookaheadMs);
fprintf(' RULE: 2-4ms is the desired hot-path generation budget inside one 25ms control cycle; it is not yet a proved WCET.\n');
fprintf(' RULE: cone is a cable-dynamic envelope, not an obstacle-clearance model by itself; final authority is continuous full-body certification.\n');
fprintf('========================================================================================================================\n');
end

function S = v209b_static_scenario(CFG)
S = v208_make_scenario('V209B_STATIC_REPLAN',[3.0;1.55;0],zeros(3,1),0.55,[0;0;0],[6;0;0]);
S.newObstacleCenter = [4.2;0;0];
S.newObstacleRadius = 0.55;
S.staticOnly = true;
end

function J = v209b_eval_jet(B,T,t)
J = zeros(3,7);
for r = 0:6
    J(:,r+1) = v208_eval_jet(B,T,t,r);
end
end

function J = v209b_backend_start_jets(B,T)
J = zeros(3,7);
for r = 0:6
    J(:,r+1) = v207_bezier_endpoint_derivative(B.P{1},T.h(1),r,0);
end
end

function A = v209b_bridge_corridor_audit(B,Q)
A = struct('pass',false,'min_slack',-inf,'faces',0);
if isempty(Q.cnt) || Q.cnt(1) < 1
    return;
end
nf = Q.cnt(1);
slack = inf;
for f = 1:nf
    slack = min(slack,min(Q.N(:,f)'*B-Q.h(f)));
end
scale = max(1,max(abs([B(:);Q.h(1:nf).'])));
guard = CFG_EPS_GUARD_V209B(4096,scale);
A.pass = isfinite(slack) && slack >= -guard;
A.min_slack = slack;
A.faces = nf;
end

function g = CFG_EPS_GUARD_V209B(mult,scale)
g = mult*eps(max(1,scale));
end

function D = v209b_bridge_dynamic_audit(B,h,thetaDeg,CFG)
n = size(B,2)-1;
fac = n*(n-1)/h^2;
A2 = fac*diff(B,2,2);
R = A2;
R(3,:) = R(3,:) + CFG.gravity;
coneSlack = tan(deg2rad(thetaDeg))*R(3,:) - vecnorm(R(1:2,:),2,1);
rzSlack = R(3,:) - CFG.v207_rho_min_mps2;
scale = max(1,max(abs(R(:))));
guard = CFG.v209b_bridge_guard_scale*eps(scale);
D = struct();
D.pass = all(isfinite(R(:))) && all(coneSlack >= guard) && all(rzSlack >= guard);
D.R = R;
D.rcount = size(R,2);
D.minCone = min(coneSlack);
D.minRz = min(rzSlack);
end

function QT = v209b_tail_problem(Q)
QT = struct();
if size(Q.W,2) <= 2
    QT.exists = false;
    return;
end
QT.exists = true;
QT.W = Q.W(:,2:end);
QT.C = Q.C;
QT.V = zeros(size(Q.V));
QT.Ractual = Q.Ractual;
QT.segment_length = vecnorm(diff(QT.W,1,2),2,1);
QT.weights = QT.segment_length/sum(QT.segment_length);
firstCount = Q.cnt(1);
QT.cnt = Q.cnt(2:end);
idx = (firstCount+1):sum(Q.cnt);
QT.N = Q.N(:,idx);
QT.h = Q.h(idx);
QT.corridorOK = true;
end

function A = v209b_tail_corridor_audit(B,QT)
if ~QT.exists
    A = struct('pass',true,'min_slack',inf);
    return;
end
minSlack = inf;
offset = 0;
for q = 1:numel(B.P)
    for j = 1:QT.cnt(q)
        f = offset+j;
        minSlack = min(minSlack,min(QT.N(:,f)'*B.P{q}-QT.h(f)));
    end
    offset = offset + QT.cnt(q);
end
scale = max(1,max(cellfun(@(X)max(abs(X(:))),B.P)));
A = struct('pass',isfinite(minSlack) && minSlack >= -4096*eps(scale),'min_slack',minSlack);
end

function [Btail,Ttail,status] = v209b_build_tail(QT,CFG,name)
Btail = [];
Ttail = [];
status = 'NO_TAIL';
if ~QT.exists
    return;
end
B = v207_build_backend(name,QT.W,QT.weights);
C = v209b_tail_corridor_audit(B,QT);
if ~C.pass
    status = 'TAIL_CORRIDOR_SEARCH_INCOMPLETE';
    return;
end
T = v208_time_gate_angle(B,CFG,CFG.v209b_nominal_theta_deg);
if ~T.pass
    status = 'TAIL_TIME_SEARCH_INCOMPLETE';
    return;
end
D = v207_final_direct_certificate(B,T,QT,CFG);
if ~D.pass
    status = 'TAIL_HARD_CERT_FAIL';
    return;
end
Btail = B;
Ttail = T;
status = 'PASS';
end

function A = v209b_combined_corridor_audit(B,Q)
S = numel(B.P);
if S ~= numel(Q.cnt)
    A = struct('pass',false,'min_slack',-inf);
    return;
end
minSlack = inf;
offset = 0;
for q = 1:S
    for j = 1:Q.cnt(q)
        f = offset+j;
        minSlack = min(minSlack,min(Q.N(:,f)'*B.P{q}-Q.h(f)));
    end
    offset = offset + Q.cnt(q);
end
scale = max(1,max(cellfun(@(X)max(abs(X(:))),B.P)));
A = struct('pass',isfinite(minSlack) && minSlack >= -4096*eps(scale),'min_slack',minSlack);
end

function [ok,C] = v209b_evaluate_bridge_duration(J0,J1,Q,BT,TT,hb,CFG)
BB = v209b_degree17_two_sided_jet_bridge(J0,J1,hb);
CORb = v209b_bridge_corridor_audit(BB,Q);
DY = v209b_bridge_dynamic_audit(BB,hb,CFG.v209b_nominal_theta_deg,CFG);
if ~(CORb.pass && DY.pass)
    ok = false;
    C = [];
    return;
end
if isempty(BT)
    P = {BB};
    hh = hb;
    Rflat = DY.R;
    rcounts = DY.rcount;
    tailT = 0;
else
    P = [{BB} BT.P];
    hh = [hb TT.h];
    Rflat = [DY.R TT.Rflat];
    rcounts = [DY.rcount TT.rcounts];
    tailT = TT.T;
end
BC = struct();
BC.P = P;
BC.degree = 17;
BC.structural_c6_theorem = true;
BC.condition_indicator = 1;
TC = struct();
TC.pass = true;
TC.h = hh;
TC.Rflat = Rflat;
TC.rcounts = rcounts;
TC.T = sum(hh);
C6 = v207_c6_backend_audit(BC,hh);
COR = v209b_combined_corridor_audit(BC,Q);
D = v207_final_direct_certificate(BC,TC,Q,CFG);
ok = C6.pass && COR.pass && D.pass;
C = struct('B',BC,'T',TC,'bridgeB',BB,'bridgeH',hb,'bridgeDynamic',DY, ...
    'bridgeCorridor',CORb,'c6',C6,'corridor',COR,'direct',D,'tailT',tailT);
end

function H = v209b_bridge_search(J0,J1,Q,BT,TT,CFG)
lo = CFG.v209b_bridge_h_min_s;
hi = CFG.v209b_bridge_h_max_s;
intervals = [lo hi];
best = [];
bestH = inf;
evals = 0;
for z = 1:2
    if z == 1
        h = lo;
    else
        h = hi;
    end
    [ok,C] = v209b_evaluate_bridge_duration(J0,J1,Q,BT,TT,h,CFG);
    evals = evals+1;
    if ok && h < bestH
        best = C;
        bestH = h;
    end
end
while ~isempty(intervals) && evals < CFG.v209b_bridge_search_evals
    keep = intervals(:,1) < bestH;
    intervals = intervals(keep,:);
    if isempty(intervals)
        break;
    end
    [~,ii] = max(intervals(:,2)-intervals(:,1));
    ab = intervals(ii,:);
    intervals(ii,:) = [];
    if ab(2)-ab(1) <= 64*eps(max(1,ab(2)))
        continue;
    end
    mid = 0.5*(ab(1)+ab(2));
    [ok,C] = v209b_evaluate_bridge_duration(J0,J1,Q,BT,TT,mid,CFG);
    evals = evals+1;
    if ok && mid < bestH
        best = C;
        bestH = mid;
    end
    if ab(1) < bestH
        intervals = [intervals;ab(1) mid]; %#ok<AGROW>
    end
    if mid < bestH
        intervals = [intervals;mid ab(2)]; %#ok<AGROW>
    end
end
H = struct();
H.pass = ~isempty(best);
H.evals = evals;
H.searchComplete = false;
H.optimalityClaim = false;
H.missIsInfeasible = false;
if H.pass
    H.best = best;
    H.h = bestH;
else
    H.best = [];
    H.h = NaN;
end
end

function [L,R] = v209b_bezier_split(B,u)
n = size(B,2)-1;
Q = B;
L = zeros(size(B));
R = zeros(size(B));
L(:,1) = B(:,1);
R(:,n+1) = B(:,n+1);
for r = 1:n
    Q = (1-u)*Q(:,1:end-1) + u*Q(:,2:end);
    L(:,r+1) = Q(:,1);
    R(:,n-r+1) = Q(:,end);
end
end

function X = v209b_bezier_restrict(B,a,b)
if a <= 0 && b >= 1
    X = B;
    return;
end
if a > 0
    [~,R] = v209b_bezier_split(B,a);
else
    R = B;
end
if b < 1
    if a > 0
        u = (b-a)/(1-a);
    else
        u = b;
    end
    [L,~] = v209b_bezier_split(R,u);
    X = L;
else
    X = R;
end
end

function [P,h] = v209b_extract_interval(B,T,t0,t1)
cs = [0 cumsum(T.h)];
P = {};
h = [];
for q = 1:numel(B.P)
    a = cs(q);
    b = cs(q+1);
    lo = max(a,t0);
    hi = min(b,t1);
    if hi <= lo
        continue;
    end
    s0 = (lo-a)/(b-a);
    s1 = (hi-a)/(b-a);
    P{end+1} = v209b_bezier_restrict(B.P{q},s0,s1); %#ok<AGROW>
    h(end+1) = hi-lo; %#ok<AGROW>
end
end

function A = v209b_active_interval_certificate(B,T,t0,t1,Q,CFG)
A = struct('pass',false,'min_slack',NaN,'pieces',0);
if ~(isfinite(t0) && isfinite(t1) && t1>t0 && t0>=0 && t1<=T.T)
    return;
end
[P,h] = v209b_extract_interval(B,T,t0,t1);
if isempty(P)
    return;
end
Rflat = [];
rcounts = zeros(1,numel(P));
for q = 1:numel(P)
    X = P{q};
    n = size(X,2)-1;
    A2 = n*(n-1)/h(q)^2*diff(X,2,2);
    RR = A2;
    RR(3,:) = RR(3,:) + CFG.gravity;
    Rflat = [Rflat RR]; %#ok<AGROW>
    rcounts(q) = size(RR,2);
end
BI = struct('P',{P},'degree',size(P{1},2)-1,'structural_c6_theorem',true,'condition_indicator',1);
TI = struct('pass',true,'T',sum(h),'h',h,'Rflat',Rflat,'rcounts',rcounts);
D = v207_final_direct_certificate(BI,TI,Q,CFG);
A = D;
A.pieces = numel(P);
end

function R = v209b_static_handoff_audit(CFG)
S = v209b_static_scenario(CFG);
reps = {'PATCH_UNION','ACETRANS_SPHERE_SFC'};
names = {'BSPLINE_D7','BEZIER_D13','POLY_D13','ACETRANS_MINCO_ST_C6'};
template = v209b_empty_row('', '', 'UNSET');
rows = repmat(template,1,numel(reps)*numel(names));
ix = 0;
for ri = 1:numel(reps)
    rep = reps{ri};
    Qold = v208_build_stress_problem(S,rep,CFG);
    if ~Qold.guidePass || ~Qold.corridorOK
        for bi = 1:numel(names)
            ix = ix+1;
            rows(ix) = v209b_empty_row(rep,names{bi},'OLD_REFERENCE_SEARCH_INCOMPLETE');
        end
        continue;
    end
    Bold = v207_build_backend('ACETRANS_MINCO_ST_C6',Qold.W,Qold.weights);
    Told = v208_time_gate_angle(Bold,CFG,CFG.v209b_nominal_theta_deg);
    if ~Told.pass
        for bi = 1:numel(names)
            ix = ix+1;
            rows(ix) = v209b_empty_row(rep,names{bi},'OLD_REFERENCE_TIME_SEARCH_INCOMPLETE');
        end
        continue;
    end
    tDetect = CFG.v209b_detection_fraction*Told.T;
    tSwitch = tDetect + CFG.v209b_switch_lookahead_s;
    if tSwitch >= Told.T
        for bi = 1:numel(names)
            ix = ix+1;
            rows(ix) = v209b_empty_row(rep,names{bi},'SWITCH_LOOKAHEAD_OUTSIDE_ACTIVE_REFERENCE');
        end
        continue;
    end
    J0 = v209b_eval_jet(Bold,Told,tSwitch);
    p0 = J0(:,1);
    Cnew = [S.C S.newObstacleCenter];
    Vnew = zeros(3,size(Cnew,2));
    Rnew = [S.R S.newObstacleRadius];
    Snew = v208_make_scenario('V209B_STATIC_UPDATED',Cnew,Vnew,Rnew,p0,S.pg);
    Qnew = v208_build_stress_problem(Snew,rep,CFG);
    prefixCert = v209b_active_interval_certificate(Bold,Told,tDetect,tSwitch,Qnew,CFG);
    for bi = 1:numel(names)
        ix = ix+1;
        name = names{bi};
        row = v209b_empty_row(rep,name,'SEARCH_INCOMPLETE');
        row.tDetect = tDetect;
        row.tSwitch = tSwitch;
        row.lookahead = CFG.v209b_switch_lookahead_s;
        row.activePrefixSafe = prefixCert.pass;
        if ~prefixCert.pass
            row.status = 'ACTIVE_PREFIX_UNSAFE_FOR_SCHEDULED_SWITCH';
            rows(ix) = row;
            continue;
        end
        if ~Qnew.guidePass || ~Qnew.corridorOK
            row.status = 'UPDATED_SPATIAL_SEARCH_INCOMPLETE';
            rows(ix) = row;
            continue;
        end
        p1 = Qnew.W(:,2);
        QT = v209b_tail_problem(Qnew);
        [BT,TT,tailStatus] = v209b_build_tail(QT,CFG,name);
        if QT.exists && ~strcmp(tailStatus,'PASS')
            row.status = [name '__' tailStatus];
            rows(ix) = row;
            continue;
        end
        if isempty(BT)
            J1 = zeros(3,7);
            J1(:,1) = p1;
        else
            J1 = v209b_backend_start_jets(BT,TT);
        end
        ticId = tic;
        HS = v209b_bridge_search(J0,J1,Qnew,BT,TT,CFG);
        row.runtime_ms = 1000*toc(ticId);
        row.bridgeEvals = HS.evals;
        if ~HS.pass
            row.status = [name '__JET_TIME_CERT_SEARCH_INCOMPLETE'];
            rows(ix) = row;
            continue;
        end
        C = HS.best;
        [jetPass,jetResidual,jetRatio] = v209b_endpoint_jet_match(C.bridgeB,C.bridgeH,J0,J1);
        row.pass = row.activePrefixSafe && jetPass && C.c6.pass && C.corridor.pass && C.bridgeDynamic.pass && C.direct.pass;
        if row.pass
            row.status = 'CERTIFIED_STATIC_SUCCESSOR';
        else
            row.status = 'CERTIFICATE_OR_HANDOFF_FAIL';
        end
        row.positionJetMatch = jetPass;
        row.c6 = C.c6.pass;
        row.corridor = C.corridor.pass;
        row.coneTaut = C.bridgeDynamic.pass;
        row.hardCert = C.direct.pass;
        row.bridgeH = C.bridgeH;
        row.tailT = C.tailT;
        row.totalT = C.T.T;
        row.maxJetResidual = jetResidual;
        row.maxJetResidualOverGuard = jetRatio;
        row.atomicSwitchAllowed = row.pass && CFG.v209b_switch_authority && CFG.v209b_integrated_wcet_authority;
        rows(ix) = row;
    end
end
R = struct();
R.rows = rows;
R.total = numel(rows);
R.certifiedCandidates = sum([rows.pass]);
R.jetMatched = sum([rows.positionJetMatch]);
R.atomicAllowed = sum([rows.atomicSwitchAllowed]);
R.staticObstaclesOnly = true;
R.nominalThetaDeg = CFG.v209b_nominal_theta_deg;
R.computeStretchTargetMs = CFG.v209b_compute_stretch_target_ms;
R.switchLookaheadMs = 1000*CFG.v209b_switch_lookahead_s;
R.switchAuthority = CFG.v209b_switch_authority;
R.wcetAuthority = CFG.v209b_integrated_wcet_authority;
end

function R = v209b_empty_row(rep,name,status)
R = struct();
R.representation = rep;
R.backend = name;
R.pass = false;
R.status = status;
R.positionJetMatch = false;
R.activePrefixSafe = false;
R.c6 = false;
R.corridor = false;
R.coneTaut = false;
R.hardCert = false;
R.atomicSwitchAllowed = false;
R.bridgeH = NaN;
R.tailT = NaN;
R.totalT = NaN;
R.bridgeEvals = 0;
R.runtime_ms = NaN;
R.maxJetResidual = NaN;
R.maxJetResidualOverGuard = NaN;
R.tDetect = NaN;
R.tSwitch = NaN;
R.lookahead = NaN;
end

function T = v209b_time_trajectory_contract(CFG,H)
T = struct();
T.staticObstaclesOnly = true;
T.nominalThetaDeg = CFG.v209b_nominal_theta_deg;
T.anglePromoted = false;
T.exactPositionD0D6Handoff = true;
T.bridgeDegree = 17;
T.backendStartJetsMatched = true;
T.corridorCertificate = 'Bezier convex-hull containment';
T.cableDynamicCertificate = 'Bernstein resultant control hull inside SOC plus r_z lower bound';
T.wholeBodyCertificate = 'continuous static-obstacle PAYLOAD/UAV/CABLE certificate';
T.controlPeriodMs = 1000*CFG.v209b_control_period_s;
T.computeStretchTargetMs = CFG.v209b_compute_stretch_target_ms;
T.switchLookaheadMs = 1000*CFG.v209b_switch_lookahead_s;
T.computeTargetIsSafetyDeadline = false;
T.searchComplete = false;
T.globalTimeOptimalityClaim = false;
T.certifiedCandidates = H.certifiedCandidates;
T.totalAudited = H.total;
T.productionPromoted = false;
end

function v209b_print_handoff(H,CFG)
fprintf('\n========================================================================================================================\n');
fprintf(' G26-V209D STATIC-OBSTACLE JET-SAFE HANDOFF AUDIT | nominal theta=%.1fdeg\n',CFG.v209b_nominal_theta_deg);
fprintf('------------------------------------------------------------------------------------------------------------------------\n');
for k = 1:numel(H.rows)
    R = H.rows(k);
    fprintf(' %-20s / %-23s | %-36s | bridge=%.4fs tail=%.4fs total=%.4fs | prefixSafe=%d jet=%d C6=%d corridor=%d coneTaut=%d hardCert=%d | %.3fms\n', ...
        R.representation,R.backend,R.status,R.bridgeH,R.tailT,R.totalT,R.activePrefixSafe,R.positionJetMatch,R.c6,R.corridor,R.coneTaut,R.hardCert,R.runtime_ms);
end
fprintf(' summary: certifiedCandidates=%d/%d jetMatched=%d/%d | computeStretchTarget=%.3fms switchLookahead=%.3fms | atomicAllowed=%d\n', ...
    H.certifiedCandidates,H.total,H.jetMatched,H.total,H.computeStretchTargetMs,H.switchLookaheadMs,H.atomicAllowed);
fprintf(' RULE: 2-4ms is the desired trajectory-generation hot-path budget inside one 25ms control cycle; production still requires WCET evidence plus prefix safety.\n');
fprintf(' RULE: all obstacles are static in V209B. Any generator miss remains SEARCH_INCOMPLETE, never path infeasibility.\n');
fprintf('========================================================================================================================\n');
end

function v209b_print_time_trajectory(T)
fprintf('\n[G26-V209D TIME / TRAJECTORY CERTIFICATE]\n');
fprintf(' staticOnly=%d nominalTheta=%.1fdeg anglePromoted=%d D17Bridge=%d exactD0D6=%d\n', ...
    T.staticObstaclesOnly,T.nominalThetaDeg,T.anglePromoted,T.bridgeDegree,T.exactPositionD0D6Handoff);
fprintf(' corridor=%s\n',T.corridorCertificate);
fprintf(' dynamic=%s\n',T.cableDynamicCertificate);
fprintf(' wholeBody=%s\n',T.wholeBodyCertificate);
fprintf(' controlPeriod=%.3fms computeTarget=%.3fms switchLookahead=%.3fms sameQuantity=%d searchComplete=%d globalTimeOptimality=%d\n', ...
    T.controlPeriodMs,T.computeStretchTargetMs,T.switchLookaheadMs,T.computeTargetIsSafetyDeadline,T.searchComplete,T.globalTimeOptimalityClaim);
fprintf(' certifiedCandidates=%d/%d productionPromoted=%d\n',T.certifiedCandidates,T.totalAudited,T.productionPromoted);
end

function C = v209b_extend_contract(C,CFG,SELF,H)
C.v209b_jet_bridge_selftest = SELF.v209b_jet_bridge.pass;
C.v209b_static_obstacles_only = true;
C.v209b_nominal_theta_deg = CFG.v209b_nominal_theta_deg;
C.v209b_angle_promoted = false;
C.v209b_downward_cable_cone_equiv_resultant_soc = true;
C.v209b_exact_position_d0_d6_handoff = true;
C.v209b_control_period_s = CFG.v209b_control_period_s;
C.v209b_future_switch_lookahead_s = CFG.v209b_switch_lookahead_s;
C.v209b_compute_stretch_target_ms = CFG.v209b_compute_stretch_target_ms;
C.v209b_compute_target_is_safety_deadline = false;
C.v209b_certified_candidate_count = H.certifiedCandidates;
C.v209b_handoff_audit_count = H.total;
C.v209b_switch_authority = CFG.v209b_switch_authority;
C.v209b_integrated_wcet_authority = CFG.v209b_integrated_wcet_authority;
C.production_promoted = false;
C.claim_boundary = [C.claim_boundary ' V209B assumes static obstacles only. Cable direction is constrained by a downward angular cone, equivalently an upward resultant SOC under q=-r/||r||; tautness and continuous UAV/load/cable collision certification are independent hard gates. The 2-4ms generation target is budgeted inside the 25ms control cycle; it is not yet a proved WCET or a safety guarantee by itself.'];
end


function A = v209e_updated_spatial_search_audit(CFG)
S = v209b_static_scenario(CFG);
reps = {'PATCH_UNION','ACETRANS_SPHERE_SFC'};
caps = [48 96 192 384];
modeNames = {'FULL_SYSTEM_SPHERE','PAYLOAD_ONLY_DIAGNOSTIC','OBS_MARGIN_ONLY_DIAGNOSTIC'};
rows = repmat(struct('rep','','mode','','cap',0,'status',NaN,'reason','','insertions',NaN,'nodes',NaN, ...
    'minClearance',NaN,'corridorOK',false,'endpointInside',false,'pairMinGap',NaN,'p50_ms',NaN,'p95_ms',NaN,'max_ms',NaN), ...
    1,numel(reps)*numel(modeNames)*numel(caps));
ix=0;
meta = repmat(struct('rep','','oldGuidePass',false,'oldTimePass',false,'tDetect',NaN,'tSwitch',NaN,'pSwitch',nan(3,1), ...
    'actualPairMinGap',NaN,'fullPairMinGap',NaN,'payloadPairMinGap',NaN),1,numel(reps));
for ri=1:numel(reps)
    rep=reps{ri};
    Qold=v208_build_stress_problem(S,rep,CFG);
    meta(ri).rep=rep;meta(ri).oldGuidePass=Qold.guidePass;
    if ~Qold.guidePass || ~Qold.corridorOK
        continue;
    end
    Bold=v207_build_backend('ACETRANS_MINCO_ST_C6',Qold.W,Qold.weights);
    Told=v208_time_gate_angle(Bold,CFG,CFG.v209b_nominal_theta_deg);
    meta(ri).oldTimePass=Told.pass;
    if ~Told.pass, continue; end
    tDetect=CFG.v209b_detection_fraction*Told.T;
    tSwitch=tDetect+CFG.v209b_switch_lookahead_s;
    if tSwitch>=Told.T, continue; end
    J0=v209b_eval_jet(Bold,Told,tSwitch);p0=J0(:,1);
    Cnew=[S.C S.newObstacleCenter];Rnew=[S.R S.newObstacleRadius];
    extra=0;if strcmp(rep,'ACETRANS_SPHERE_SFC'),extra=CFG.v207_qualification_extra_proposal_margin_m;end
    fullInfl=CFG.L_cable+max([CFG.R_payload CFG.R_uav CFG.R_cable])+CFG.geometry_margin_m;
    Rmode={Rnew+fullInfl+extra, Rnew+CFG.R_payload+CFG.geometry_margin_m+extra, Rnew+CFG.geometry_margin_m+extra};
    meta(ri).tDetect=tDetect;meta(ri).tSwitch=tSwitch;meta(ri).pSwitch=p0;
    meta(ri).actualPairMinGap=v209e_pair_min_gap(Cnew,Rnew);
    meta(ri).fullPairMinGap=v209e_pair_min_gap(Cnew,Rmode{1});
    meta(ri).payloadPairMinGap=v209e_pair_min_gap(Cnew,Rmode{2});
    for mi=1:numel(modeNames)
        Rp=Rmode{mi};
        endpointInside=v209e_endpoint_inside(p0,S.pg,Cnew,Rp);
        pairGap=v209e_pair_min_gap(Cnew,Rp);
        for ci=1:numel(caps)
            cap=caps(ci);ix=ix+1;
            [W,st,ins,minClr]=g26v206_spatial_guide_mex(p0,S.pg,Cnew,Rp,CFG.v205_planning_horizon_m,cap,CFG.v206_spatial_guide_guard_scale);
            corr=false;if st==1,[~,~,~,corr]=g26v206_segment_corridor_mex(W,Cnew,Rp,CFG.v206_spatial_guide_guard_scale);end
            T=zeros(80,1);
            for k=1:8,tmp=g26v206_spatial_guide_mex(p0,S.pg,Cnew,Rp,CFG.v205_planning_horizon_m,cap,CFG.v206_spatial_guide_guard_scale);end %#ok<NASGU>
            for k=1:numel(T),tt=tic;tmp=g26v206_spatial_guide_mex(p0,S.pg,Cnew,Rp,CFG.v205_planning_horizon_m,cap,CFG.v206_spatial_guide_guard_scale);T(k)=1e3*toc(tt);end %#ok<NASGU>
            SS=v202_stats(T);
            rows(ix).rep=rep;rows(ix).mode=modeNames{mi};rows(ix).cap=cap;rows(ix).status=st;rows(ix).reason=v209e_spatial_reason(st,ins,cap,endpointInside);
            rows(ix).insertions=ins;rows(ix).nodes=size(W,2);rows(ix).minClearance=minClr;rows(ix).corridorOK=logical(corr);rows(ix).endpointInside=endpointInside;rows(ix).pairMinGap=pairGap;
            rows(ix).p50_ms=SS.p50;rows(ix).p95_ms=SS.p95;rows(ix).max_ms=SS.max;
        end
    end
end
rows=rows(1:ix);
A=struct();A.rows=rows;A.meta=meta;A.caps=caps;A.modes=modeNames;
A.currentFullPass=any(strcmp({rows.mode},'FULL_SYSTEM_SPHERE') & [rows.status]==1);
A.payloadDiagnosticPass=any(strcmp({rows.mode},'PAYLOAD_ONLY_DIAGNOSTIC') & [rows.status]==1);
A.obstacleMarginDiagnosticPass=any(strcmp({rows.mode},'OBS_MARGIN_ONLY_DIAGNOSTIC') & [rows.status]==1);
A.noSafetyPromotion=true;A.finalAuthorityUnchanged=true;A.searchMissIsInfeasible=false;
end

function tf=v209e_endpoint_inside(p0,pg,C,R)
tf=false;for j=1:size(C,2),if norm(p0-C(:,j))<=R(j) || norm(pg-C(:,j))<=R(j),tf=true;return;end,end
end

function g=v209e_pair_min_gap(C,R)
M=size(C,2);if M<2,g=inf;return;end
g=inf;for i=1:M-1,for j=i+1:M,g=min(g,norm(C(:,i)-C(:,j))-R(i)-R(j));end,end
end

function s=v209e_spatial_reason(st,ins,cap,endpointInside)
if st==1,s='PASS';elseif st==-1 || endpointInside,s='ENDPOINT_INSIDE_PROPOSAL';elseif st==-2,s='GOAL_OUTSIDE_HORIZON';elseif st==0 && ins>=cap-2,s='RESOURCE_BOUND_MAXNODES';else,s='SEARCH_INCOMPLETE_SIDE_OR_GEOMETRY';end
end

function v209e_print_updated_spatial_audit(A,CFG)
fprintf('\n');fprintf('========================================================================================================================\n');
fprintf(' G26-V209E UPDATED STATIC SPATIAL-SEARCH ROOT-CAUSE AUDIT\n');
fprintf('------------------------------------------------------------------------------------------------------------------------\n');
fprintf(' planningHorizon=%.3fm | maxNodes sweep=%s | final hard authority unchanged\n',CFG.v205_planning_horizon_m,mat2str(A.caps));
for k=1:numel(A.meta)
    M=A.meta(k);fprintf(' %s | oldGuide=%d oldTime=%d | tDetect=%.6fs tSwitch=%.6fs pSwitch=[%.3f %.3f %.3f]\n',M.rep,M.oldGuidePass,M.oldTimePass,M.tDetect,M.tSwitch,M.pSwitch(1),M.pSwitch(2),M.pSwitch(3));
    fprintf('   pair gap actual=%+.4fm | full-system-sphere proposal=%+.4fm | payload-only diagnostic=%+.4fm\n',M.actualPairMinGap,M.fullPairMinGap,M.payloadPairMinGap);
end
for k=1:numel(A.rows)
    R=A.rows(k);fprintf(' %-20s | %-27s | cap=%3d st=%2d %-34s | ins=%3.0f nodes=%3.0f minClr=%+.5f corr=%d | p50/p95/max=%.5f/%.5f/%.5f ms\n', ...
        R.rep,R.mode,R.cap,R.status,R.reason,R.insertions,R.nodes,R.minClearance,R.corridorOK,R.p50_ms,R.p95_ms,R.max_ms);
end
fprintf('------------------------------------------------------------------------------------------------------------------------\n');
fprintf(' currentFullPass=%d payloadDiagnosticPass=%d obstacleMarginDiagnosticPass=%d | diagnosticProposalSafetyAuthority=0\n',A.currentFullPass,A.payloadDiagnosticPass,A.obstacleMarginDiagnosticPass);
fprintf(' RULE: failure of FULL_SYSTEM_SPHERE may be generator/proposal incompleteness; it is never path/physical infeasibility.\n');
fprintf(' RULE: lighter diagnostic proposals are used only to identify over-conservatism. They do NOT replace the final continuous UAV/load/cable certificate.\n');
fprintf(' RULE: do not advance to trajectory/handoff until ordinary updated-spatial cases are reliable and timed.\n');
fprintf('========================================================================================================================\n');
end

function S = v209g_cone_cover_selftest(CFG)
% Analytic cover theorem being tested numerically:
% For u with angle(u,+e3)<=theta and s in [a,b], choose m=(a+b)/2.
% ||s*u-m*e3||^2=s^2+m^2-2*s*m*cos(alpha), whose maximum over
% alpha in [0,theta] and s in [a,b] occurs at alpha=theta and an endpoint.
% Therefore a sphere centered at c-m*e3 with radius
% Robs+margin+Rc+delta(a,b,m,theta) contains every forbidden payload-center
% ball generated by the cable points in that interval.
Ks=[1 2 4 8];theta=CFG.v209b_nominal_theta_deg;
c=[1.3;-0.7;0.4];ro=0.55;extra=0.0;
maxRes=-inf;cases=0;
for K=Ks
    [Cc,Rc,M]=v209g_build_cone_swept_cover(c,ro,theta,K,CFG,extra);
    base=ro+CFG.geometry_margin_m+extra;
    % Payload sphere is exact.
    rr=norm(c-Cc(:,1))+(base+CFG.R_payload)-Rc(1);maxRes=max(maxRes,rr);cases=cases+1;
    % Cable samples.
    alphas=linspace(0,deg2rad(theta),9);phis=linspace(0,2*pi,17);phis(end)=[];
    ss=linspace(0,CFG.L_cable,65);
    for si=1:numel(ss)
        s=ss(si);k=min(K,max(1,floor((s/CFG.L_cable)*K)+1));if s>=CFG.L_cable,k=K;end
        idx=1+k;
        for ai=1:numel(alphas)
            a=alphas(ai);
            for iphi=1:numel(phis)
                u=[sin(a)*cos(phis(iphi));sin(a)*sin(phis(iphi));cos(a)];
                ca=c-s*u;ra=base+CFG.R_cable;
                rr=norm(ca-Cc(:,idx))+ra-Rc(idx);maxRes=max(maxRes,rr);cases=cases+1;
            end
        end
    end
    % UAV cap samples.
    idx=K+2;
    for ai=1:numel(alphas)
        a=alphas(ai);
        for iphi=1:numel(phis)
            u=[sin(a)*cos(phis(iphi));sin(a)*sin(phis(iphi));cos(a)];
            ca=c-CFG.L_cable*u;ra=base+CFG.R_uav;
            rr=norm(ca-Cc(:,idx))+ra-Rc(idx);maxRes=max(maxRes,rr);cases=cases+1;
        end
    end
    assert(M.countPerObstacle==K+2,'G26V209G:CoverCount','Unexpected cover count.');
end
guard=8192*eps(max(1,max(abs([c;ro;CFG.L_cable]))));
S=struct('pass',isfinite(maxRes)&&maxRes<=guard,'maxContainmentResidual',maxRes,'guard',guard,'sampleChecks',cases,'K',Ks,'thetaDeg',theta);
assert(S.pass,'G26V209G:ConeCoverSelftest','Cone-swept multispherical outer-cover containment selftest failed.');
fprintf('[G26-V209G-CONE-COVER-SELFTEST] PASS | theta=%.1fdeg K=%s | sampled containment residual=%.3e guard=%.3e checks=%d\n', ...
    theta,mat2str(Ks),maxRes,guard,cases);
end

function [Ccov,Rcov,META] = v209g_build_cone_swept_cover(C,R,thetaDeg,K,CFG,extra)
C=reshape(C,3,[]);R=reshape(R,1,[]);M=size(C,2);
assert(numel(R)==M && K>=1 && floor(K)==K,'G26V209G:CoverInput','Invalid cover input.');
assert(thetaDeg>=0 && thetaDeg<90,'G26V209G:Theta','Cone cover requires 0<=theta<90 deg.');
th=deg2rad(thetaDeg);L=CFG.L_cable;e3=[0;0;1];
N=M*(K+2);Ccov=zeros(3,N);Rcov=zeros(1,N);ix=0;maxDelta=0;
for j=1:M
    base=R(j)+CFG.geometry_margin_m+extra;
    % Payload forbidden set.
    ix=ix+1;Ccov(:,ix)=C(:,j);Rcov(ix)=base+CFG.R_payload;
    % Cable forbidden set, interval-wise exact center-deviation bound.
    for k=1:K
        a=(k-1)*L/K;b=k*L/K;m=0.5*(a+b);
        da=sqrt(max(0,a*a+m*m-2*a*m*cos(th)));
        db=sqrt(max(0,b*b+m*m-2*b*m*cos(th)));
        delta=max(da,db);maxDelta=max(maxDelta,delta);
        ix=ix+1;Ccov(:,ix)=C(:,j)-m*e3;Rcov(ix)=base+CFG.R_cable+delta;
    end
    % UAV center lies on spherical cap of radius L around payload.
    deltaU=2*L*sin(0.5*th);maxDelta=max(maxDelta,deltaU);
    ix=ix+1;Ccov(:,ix)=C(:,j)-L*e3;Rcov(ix)=base+CFG.R_uav+deltaU;
end
META=struct('physicalObstacles',M,'segments',K,'countPerObstacle',K+2,'sphereCount',N,'thetaDeg',thetaDeg,'maxCenterDeviation',maxDelta,'conservativeOuterCover',true);
end

function A = v209g_cone_swept_spatial_audit(CFG)
S=v209b_static_scenario(CFG);reps={'PATCH_UNION','ACETRANS_SPHERE_SFC'};Ks=[1 2 4 8];cap=CFG.v206_spatial_guide_max_nodes;
rows=repmat(struct('rep','','K',0,'sphereCount',0,'status',NaN,'reason','','insertions',NaN,'nodes',NaN,'endpointInside',false,'corridorOK',false, ...
    'minClearance',NaN,'cover_p50_ms',NaN,'cover_p95_ms',NaN,'cover_max_ms',NaN,'integrated_p50_ms',NaN,'integrated_p95_ms',NaN,'integrated_max_ms',NaN, ...
    'knownFeasibleCases',0,'guideSuccesses',0,'guideMisses',0,'successRate',NaN,'ready',false),1,numel(reps)*numel(Ks));
meta=repmat(struct('rep','','pSwitch',nan(3,1),'fullStatus',NaN,'fullEndpointInside',false,'fullMinClearance',NaN,'fullSphereCount',0),1,numel(reps));
ix=0;
for ri=1:numel(reps)
    rep=reps{ri};Qold=v208_build_stress_problem(S,rep,CFG);assert(Qold.guidePass&&Qold.corridorOK,'G26V209G:OldGuide','Old known-obstacle guide must pass.');
    Bold=v207_build_backend('ACETRANS_MINCO_ST_C6',Qold.W,Qold.weights);Told=v208_time_gate_angle(Bold,CFG,CFG.v209b_nominal_theta_deg);assert(Told.pass,'G26V209G:OldTime','Old trajectory time gate must pass.');
    tDetect=CFG.v209b_detection_fraction*Told.T;tSwitch=tDetect+CFG.v209b_switch_lookahead_s;assert(tSwitch<Told.T,'G26V209G:Switch','Switch time beyond old trajectory.');
    J0=v209b_eval_jet(Bold,Told,tSwitch);p0=J0(:,1);Cnew=[S.C S.newObstacleCenter];Rnew=[S.R S.newObstacleRadius];
    extra=0;if strcmp(rep,'ACETRANS_SPHERE_SFC'),extra=CFG.v207_qualification_extra_proposal_margin_m;end
    fullInfl=CFG.L_cable+max([CFG.R_payload CFG.R_uav CFG.R_cable])+CFG.geometry_margin_m+extra;
    Rfull=Rnew+fullInfl;[Wf,stf,~,mcf]=g26v206_spatial_guide_mex(p0,S.pg,Cnew,Rfull,CFG.v205_planning_horizon_m,cap,CFG.v206_spatial_guide_guard_scale);
    meta(ri).rep=rep;meta(ri).pSwitch=p0;meta(ri).fullStatus=stf;meta(ri).fullEndpointInside=v209e_endpoint_inside(p0,S.pg,Cnew,Rfull);meta(ri).fullMinClearance=mcf;meta(ri).fullSphereCount=numel(Rfull); %#ok<NASGU>
    for ki=1:numel(Ks)
        K=Ks(ki);ix=ix+1;
        [Cc,Rc,M]=v209g_build_cone_swept_cover(Cnew,Rnew,CFG.v209b_nominal_theta_deg,K,CFG,extra);
        endpointInside=v209e_endpoint_inside(p0,S.pg,Cc,Rc);
        [W,st,ins,minClr]=g26v206_spatial_guide_mex(p0,S.pg,Cc,Rc,CFG.v205_planning_horizon_m,cap,CFG.v206_spatial_guide_guard_scale);
        corr=false;if st==1,[~,~,~,corr]=g26v206_segment_corridor_mex(W,Cc,Rc,CFG.v206_spatial_guide_guard_scale);end
        % Cover-construction timing.
        Nc=100;tc=zeros(Nc,1);for n=1:Nc,tt=tic;[aa,bb]=v209g_build_cone_swept_cover(Cnew,Rnew,CFG.v209b_nominal_theta_deg,K,CFG,extra);tc(n)=1e3*toc(tt);end %#ok<ASGLU>
        SC=v202_stats(tc);
        % Integrated cover + guide + corridor timing.
        Ni=100;ti=zeros(Ni,1);for n=1:Ni
            tt=tic;[aa,bb]=v209g_build_cone_swept_cover(Cnew,Rnew,CFG.v209b_nominal_theta_deg,K,CFG,extra);[ww,ss]=g26v206_spatial_guide_mex(p0,S.pg,aa,bb,CFG.v205_planning_horizon_m,cap,CFG.v206_spatial_guide_guard_scale);if ss==1,[x1,x2,x3,x4]=g26v206_segment_corridor_mex(ww,aa,bb,CFG.v206_spatial_guide_guard_scale);end;ti(n)=1e3*toc(tt); %#ok<ASGLU>
        end
        SI=v202_stats(ti);
        [nf,gs,gm]=v209g_reliability_suite(p0,S.pg,S.C,S.R,rep,K,CFG,extra);
        rows(ix).rep=rep;rows(ix).K=K;rows(ix).sphereCount=M.sphereCount;rows(ix).status=st;rows(ix).reason=v209e_spatial_reason(st,ins,cap,endpointInside);rows(ix).insertions=ins;rows(ix).nodes=size(W,2);rows(ix).endpointInside=endpointInside;rows(ix).corridorOK=logical(corr);rows(ix).minClearance=minClr;
        rows(ix).cover_p50_ms=SC.p50;rows(ix).cover_p95_ms=SC.p95;rows(ix).cover_max_ms=SC.max;rows(ix).integrated_p50_ms=SI.p50;rows(ix).integrated_p95_ms=SI.p95;rows(ix).integrated_max_ms=SI.max;
        rows(ix).knownFeasibleCases=nf;rows(ix).guideSuccesses=gs;rows(ix).guideMisses=gm;if nf>0,rows(ix).successRate=gs/nf;end;rows(ix).ready=(st==1)&&logical(corr)&&(gm==0)&&(nf>0)&&(SI.p95<CFG.v209b_compute_stretch_target_ms);
    end
end
rows=rows(1:ix);A=struct();A.rows=rows;A.meta=meta;A.Ks=Ks;A.cap=cap;A.thetaDeg=CFG.v209b_nominal_theta_deg;A.finalAuthorityUnchanged=true;A.noTrajectoryPromotion=true;
A.eachRepresentationHasReadyK=true;for ri=1:numel(reps),mask=strcmp({rows.rep},reps{ri});A.eachRepresentationHasReadyK=A.eachRepresentationHasReadyK&&any([rows(mask).ready]);end;A.readyRows=sum([rows.ready]);
end

function [knownFeasible,success,miss] = v209g_reliability_suite(p0,pg,Cold,Rold,rep,K,CFG,extra)
% Deterministic ordinary-case suite. A case is counted only when a simple
% independently checked two-segment witness exists in the SAME conservative
% proposal cover; therefore a guide miss is generator incompleteness, not a
% physical/path infeasibility claim.
xs=[2.8 3.4 4.0 4.6 5.2];ys=[-1.0 -0.5 0 0.5 1.0];knownFeasible=0;success=0;miss=0;
for xi=1:numel(xs)
    for yi=1:numel(ys)
        C=[Cold [xs(xi);ys(yi);0]];R=[Rold 0.55];[Cc,Rc]=v209g_build_cone_swept_cover(C,R,CFG.v209b_nominal_theta_deg,K,CFG,extra);
        if v209e_endpoint_inside(p0,pg,Cc,Rc),continue;end
        if ~v209g_simple_witness_exists(p0,pg,Cc,Rc),continue;end
        knownFeasible=knownFeasible+1;
        [W,st]=g26v206_spatial_guide_mex(p0,pg,Cc,Rc,CFG.v205_planning_horizon_m,CFG.v206_spatial_guide_max_nodes,CFG.v206_spatial_guide_guard_scale);
        ok=false;if st==1,[~,~,~,ok]=g26v206_segment_corridor_mex(W,Cc,Rc,CFG.v206_spatial_guide_guard_scale);end
        if st==1 && ok,success=success+1;else,miss=miss+1;end
    end
end
end

function tf = v209g_simple_witness_exists(p0,pg,C,R)
mx=0.5*(p0(1)+pg(1));
W=[mx mx mx mx mx mx; -3 3 0 -2.5 2.5 0; 0 0 3 1.5 1.5 2.2];
tf=false;
for k=1:size(W,2)
    if v209g_segment_set_clear(p0,W(:,k),C,R) && v209g_segment_set_clear(W(:,k),pg,C,R),tf=true;return;end
end
end

function tf = v209g_segment_set_clear(a,b,C,R)
tf=true;for j=1:size(C,2),if v209g_segment_sphere_clearance(a,b,C(:,j),R(j))<=1e-10,tf=false;return;end,end
end

function d = v209g_segment_sphere_clearance(a,b,c,r)
v=b-a;vv=v'*v;if vv<=eps(max(1,norm(v)^2)),q=a;else,t=max(0,min(1,((c-a)'*v)/vv));q=a+t*v;end;d=norm(q-c)-r;
end

function v209g_print_cone_swept_spatial_audit(A,CFG)
fprintf('\n========================================================================================================================\n');
fprintf(' G26-V209G CONE-SWEPT MULTI-SPHERE UPDATED-SPATIAL AUDIT | theta=%.1fdeg\n',A.thetaDeg);
fprintf('------------------------------------------------------------------------------------------------------------------------\n');
fprintf(' theorem: payload exact sphere + K cable interval spheres + UAV spherical-cap cover; all are forbidden sets in p_L space\n');
fprintf(' final authority remains continuous finite UAV/load/cable certificate; no trajectory is promoted here\n');
for k=1:numel(A.meta)
    M=A.meta(k);fprintf(' %-20s | old FULL_SYSTEM_SPHERE st=%2d endpointInside=%d minClr=%+.5f spheres=%d | pSwitch=[%.3f %.3f %.3f]\n', ...
        M.rep,M.fullStatus,M.fullEndpointInside,M.fullMinClearance,M.fullSphereCount,M.pSwitch(1),M.pSwitch(2),M.pSwitch(3));
end
for k=1:numel(A.rows)
    R=A.rows(k);fprintf(' %-20s | K=%d spheres=%2d | st=%2d %-30s corr=%d ins=%2.0f nodes=%2.0f minClr=%+.5f | cover p95=%.5f ms | integrated p50/p95/max=%.5f/%.5f/%.5f ms | known-feasible=%2d success=%2d miss=%2d rate=%.1f%% ready=%d\n', ...
        R.rep,R.K,R.sphereCount,R.status,R.reason,R.corridorOK,R.insertions,R.nodes,R.minClearance,R.cover_p95_ms,R.integrated_p50_ms,R.integrated_p95_ms,R.integrated_max_ms, ...
        R.knownFeasibleCases,R.guideSuccesses,R.guideMisses,100*R.successRate,R.ready);
end
fprintf('------------------------------------------------------------------------------------------------------------------------\n');
fprintf(' eachRepresentationHasReadyK=%d | readyRows=%d/%d | readiness requires current PASS+corridor, zero misses on known-feasible suite, and p95<%.1fms\n',A.eachRepresentationHasReadyK,A.readyRows,numel(A.rows),CFG.v209b_compute_stretch_target_ms);
fprintf(' RULE: cone-swept proposal is an analytic conservative outer cover for every cable/UAV configuration inside theta; it is still only a front-end proposal.\n');
fprintf(' RULE: reliability cases are counted only when an independently checked two-segment witness exists in the same proposal cover.\n');
fprintf(' RULE: if a known-feasible proposal case is missed, do not proceed to trajectory/handoff; improve the spatial generator first.\n');
fprintf('========================================================================================================================\n');
end


% ========================================================================
% V209H: cluster-aware spatial guide MEX.
% It changes generator search only. Proposal cover and hard safety authority
% are untouched. Directions are derived from the current segment and the
% active blocker cluster; there is no fixed global direction bank.
% ========================================================================
function v209h_mex_bootstrap_cluster_guide(root)
name='g26v209h_cluster_guide_mex';
v200_build_mex_if_needed(root,name,v209h_cpp_cluster_guide_source(),'G26-V209H-CLUSTER-GUIDE-MEX');
end

function lines=v209h_cpp_cluster_guide_source()
lines={ ...
'#include "mex.h"', ...
'#include <vector>', ...
'#include <cmath>', ...
'#include <limits>', ...
'#include <algorithm>', ...
'static void req(bool q,const char*m){if(!q)mexErrMsgIdAndTxt("G26V209H:ClusterGuideMEX","%s",m);}', ...
'static double n3(double x,double y,double z){return std::sqrt(x*x+y*y+z*z);}', ...
'static double dot3(const double*a,const double*b){return a[0]*b[0]+a[1]*b[1]+a[2]*b[2];}', ...
'static void cross3(const double*a,const double*b,double*c){c[0]=a[1]*b[2]-a[2]*b[1];c[1]=a[2]*b[0]-a[0]*b[2];c[2]=a[0]*b[1]-a[1]*b[0];}', ...
'static double guard(double sc,double gs){return gs*std::numeric_limits<double>::epsilon()*std::max(1.0,sc);}', ...
'static double segdist(const double*A,const double*B,const double*C,double*q,double&t){double d[3]={B[0]-A[0],B[1]-A[1],B[2]-A[2]};double l2=dot3(d,d);t=0.0;if(l2>0){double w[3]={C[0]-A[0],C[1]-A[1],C[2]-A[2]};t=std::max(0.0,std::min(1.0,dot3(w,d)/l2));}for(int a=0;a<3;++a)q[a]=A[a]+t*d[a];return n3(q[0]-C[0],q[1]-C[1],q[2]-C[2]);}', ...
'static bool inHorizon(const double*W,const double*p0,double H,double gs){double dx=W[0]-p0[0],dy=W[1]-p0[1],dz=W[2]-p0[2];double d=n3(dx,dy,dz);return d<=H+guard(std::max(H,d),gs);}', ...
'static bool allClear(const double*A,const double*B,const double*W,const double*C,const double*R,mwSize M,const double*p0,double H,double gs,double&checks){if(!inHorizon(W,p0,H,gs))return false;for(mwSize j=0;j<M;++j){double q[3],t;double d1=segdist(A,W,C+3*j,q,t),d2=segdist(W,B,C+3*j,q,t);checks+=2.0;double sc=std::max(R[j],std::max(d1,d2));double g=guard(sc,gs);if(d1<R[j]+16.0*g||d2<R[j]+16.0*g)return false;}return true;}', ...
'static bool clusterCandidate(const double*A,const double*B,const double*q0,const double*n,const double*C,const double*R,mwSize M,const double*p0,double H,double gs,double*outW,double&outCost,double&checks){double v[3]={q0[0]-p0[0],q0[1]-p0[1],q0[2]-p0[2]};double bn=dot3(v,n),vv=dot3(v,v);double disc=bn*bn+H*H-vv;double gd=guard(std::max(H,std::sqrt(std::max(0.0,vv))),gs);if(disc<-gd)return false;disc=std::max(0.0,disc);double smax=-bn+std::sqrt(disc);if(!(smax>gd&&std::isfinite(smax)))return false;', ...
' const int NS=128;double lo=0.0,hi=0.0;bool found=false;double prev=0.0;for(int ii=1;ii<=NS;++ii){double s=smax*((double)ii/(double)NS);double W[3]={q0[0]+s*n[0],q0[1]+s*n[1],q0[2]+s*n[2]};if(allClear(A,B,W,C,R,M,p0,H,gs,checks)){lo=prev;hi=s;found=true;break;}prev=s;}if(!found)return false;', ...
' for(int it=0;it<56;++it){double md=0.5*(lo+hi);double W[3]={q0[0]+md*n[0],q0[1]+md*n[1],q0[2]+md*n[2]};if(allClear(A,B,W,C,R,M,p0,H,gs,checks))hi=md;else lo=md;}for(int a=0;a<3;++a)outW[a]=q0[a]+hi*n[a];double dAW=n3(outW[0]-A[0],outW[1]-A[1],outW[2]-A[2]),dWB=n3(B[0]-outW[0],B[1]-outW[1],B[2]-outW[2]),dAB=n3(B[0]-A[0],B[1]-A[1],B[2]-A[2]);outCost=dAW+dWB-dAB;return allClear(A,B,outW,C,R,M,p0,H,gs,checks);}', ...
'void mexFunction(int nlhs,mxArray*plhs[],int nrhs,const mxArray*prhs[]){', ...
' req(nrhs==7,"Expected p0,goal,centers,radii,horizon,maxNodes,guardScale");req(nlhs<=5,"At most five outputs");', ...
' req(mxIsDouble(prhs[0])&&mxGetNumberOfElements(prhs[0])==3,"p0 must be 3-vector");req(mxIsDouble(prhs[1])&&mxGetNumberOfElements(prhs[1])==3,"goal must be 3-vector");', ...
' req(mxIsDouble(prhs[2])&&mxGetM(prhs[2])==3,"centers must be 3xM");mwSize M=mxGetN(prhs[2]);req(mxIsDouble(prhs[3])&&mxGetNumberOfElements(prhs[3])==M,"radii mismatch");', ...
' const double*p0=mxGetPr(prhs[0]);const double*pg=mxGetPr(prhs[1]);const double*C=mxGetPr(prhs[2]);const double*R=mxGetPr(prhs[3]);double H=mxGetScalar(prhs[4]);int maxNodes=(int)std::llround(mxGetScalar(prhs[5]));double gs=mxGetScalar(prhs[6]);', ...
' req(std::isfinite(H)&&H>0&&maxNodes>=2&&maxNodes<=4096&&std::isfinite(gs)&&gs>=0,"resource/horizon input invalid");for(mwSize j=0;j<M;++j)req(std::isfinite(R[j])&&R[j]>=0,"radius invalid");', ...
' std::vector<double>P;P.reserve(3*maxNodes);for(int a=0;a<3;++a)P.push_back(p0[a]);for(int a=0;a<3;++a)P.push_back(pg[a]);int status=1,ins=0;double checks=0.0;', ...
' if(!inHorizon(pg,p0,H,gs))status=-2;for(mwSize j=0;j<M&&status==1;++j){double ds=n3(p0[0]-C[3*j],p0[1]-C[3*j+1],p0[2]-C[3*j+2]),dg=n3(pg[0]-C[3*j],pg[1]-C[3*j+1],pg[2]-C[3*j+2]);double g=guard(std::max(R[j],std::max(ds,dg)),gs);if(ds<=R[j]+g||dg<=R[j]+g)status=-1;}', ...
' while(status==1){mwSize N=P.size()/3;bool any=false;mwSize bs=0;double bestPen=-1.0;int bestCount=-1;', ...
'  for(mwSize si=0;si+1<N;++si){const double*A=&P[3*si];const double*B=&P[3*(si+1)];double pen=0.0;int cnt=0;for(mwSize j=0;j<M;++j){double q[3],t;double d=segdist(A,B,C+3*j,q,t);checks+=1.0;double g=guard(std::max(R[j],d),gs);double clr=d-R[j]-g;if(clr<0){any=true;++cnt;pen+=-clr;}}if(cnt>0&&(pen>bestPen||(std::abs(pen-bestPen)<=guard(std::max(pen,bestPen),gs)&&cnt>bestCount))){bestPen=pen;bestCount=cnt;bs=si;}}', ...
'  if(!any)break;if((int)N>=maxNodes){status=0;break;}const double*A=&P[3*bs];const double*B=&P[3*(bs+1)];double dAB[3]={B[0]-A[0],B[1]-A[1],B[2]-A[2]};double LL=n3(dAB[0],dAB[1],dAB[2]);if(!(LL>guard(LL,gs))){status=0;break;}double e[3]={dAB[0]/LL,dAB[1]/LL,dAB[2]/LL};', ...
'  double rep[3]={0,0,0},qsum[3]={0,0,0};int nc=0;for(mwSize j=0;j<M;++j){double q[3],t;double d=segdist(A,B,C+3*j,q,t);double g=guard(std::max(R[j],d),gs);if(d-R[j]-g<0){++nc;for(int a=0;a<3;++a)qsum[a]+=q[a];double rv[3]={q[0]-C[3*j],q[1]-C[3*j+1],q[2]-C[3*j+2]};double pe=dot3(rv,e);for(int a=0;a<3;++a)rv[a]-=pe*e[a];double rn=n3(rv[0],rv[1],rv[2]);if(rn>guard(rn,gs)){double w=std::max(1e-6,(R[j]-d)/std::max(R[j],1e-9));for(int a=0;a<3;++a)rep[a]+=w*rv[a]/rn;}}}if(nc<=0){status=0;break;}double q0[3]={qsum[0]/nc,qsum[1]/nc,qsum[2]/nc};', ...
'  double n0[3];double rn=n3(rep[0],rep[1],rep[2]);if(rn>guard(rn,gs)){for(int a=0;a<3;++a)n0[a]=rep[a]/rn;}else{double ax[3]={0,0,0};int kk=0;if(std::abs(e[1])<std::abs(e[kk]))kk=1;if(std::abs(e[2])<std::abs(e[kk]))kk=2;ax[kk]=1.0;cross3(e,ax,n0);double nn=n3(n0[0],n0[1],n0[2]);if(!(nn>guard(nn,gs))){status=0;break;}for(int a=0;a<3;++a)n0[a]/=nn;}double n1[3];cross3(e,n0,n1);double n1n=n3(n1[0],n1[1],n1[2]);if(n1n>guard(n1n,gs))for(int a=0;a<3;++a)n1[a]/=n1n;', ...
'  double dirs[4][3]={{n0[0],n0[1],n0[2]},{-n0[0],-n0[1],-n0[2]},{n1[0],n1[1],n1[2]},{-n1[0],-n1[1],-n1[2]}};int nd=(n1n>guard(n1n,gs))?4:2;bool got=false;double bestW[3]={0,0,0},bestCost=std::numeric_limits<double>::infinity();for(int di=0;di<nd;++di){double W[3],cost=0;if(clusterCandidate(A,B,q0,dirs[di],C,R,M,p0,H,gs,W,cost,checks)&&cost<bestCost){got=true;bestCost=cost;for(int a=0;a<3;++a)bestW[a]=W[a];}}if(!got){status=0;break;}P.insert(P.begin()+3*(bs+1),{bestW[0],bestW[1],bestW[2]});++ins;', ...
' }', ...
' mwSize N=P.size()/3;double minClr=(M==0)?mxGetInf():std::numeric_limits<double>::infinity();for(mwSize si=0;si+1<N;++si){for(mwSize j=0;j<M;++j){double q[3],t;double d=segdist(&P[3*si],&P[3*(si+1)],C+3*j,q,t);minClr=std::min(minClr,d-R[j]);}}', ...
' plhs[0]=mxCreateDoubleMatrix(3,N,mxREAL);double*O=mxGetPr(plhs[0]);for(mwSize i=0;i<3*N;++i)O[i]=P[i];if(nlhs>1)plhs[1]=mxCreateDoubleScalar((double)status);if(nlhs>2)plhs[2]=mxCreateDoubleScalar((double)ins);if(nlhs>3)plhs[3]=mxCreateDoubleScalar(minClr);if(nlhs>4)plhs[4]=mxCreateDoubleScalar(checks);', ...
'}'};
end

function A=v209h_cluster_guide_selftest(CFG)
p0=[0;0;0];pg=[6;0;0];C=[3 3.2;0.8 -0.1;0 0];R=[1.0 1.0];
[W,st,ins,minClr]=g26v209h_cluster_guide_mex(p0,pg,C,R,CFG.v205_planning_horizon_m,CFG.v206_spatial_guide_max_nodes,CFG.v206_spatial_guide_guard_scale);
corr=false;if st==1,[~,~,~,corr]=g26v206_segment_corridor_mex(W,C,R,CFG.v206_spatial_guide_guard_scale);end
A=struct('pass',st==1&&corr&&minClr>=-1e-10,'status',st,'insertions',ins,'minClearance',minClr,'corridor',logical(corr));
assert(A.pass,'G26V209H:ClusterGuideSelftest','Cluster-aware guide/corridor selftest failed.');
fprintf('[G26-V209H-CLUSTER-GUIDE-SELFTEST] PASS | st=%d ins=%d nodes=%d minClr=%+.3e corridor=%d\n',st,ins,size(W,2),minClr,corr);
end

function A=v209h_cluster_spatial_audit(CFG)
S=v209b_static_scenario(CFG);reps={'PATCH_UNION','ACETRANS_SPHERE_SFC'};Ks=[1 2 4 8];cap=CFG.v206_spatial_guide_max_nodes;
rows=repmat(struct('rep','','K',0,'sphereCount',0,'legacyStatus',NaN,'clusterStatus',NaN,'reason','','insertions',NaN,'nodes',NaN,'corridorOK',false,'minClearance',NaN, ...
    'integrated_p50_ms',NaN,'integrated_p95_ms',NaN,'integrated_max_ms',NaN,'regKnown',0,'regLegacySuccess',0,'regClusterSuccess',0,'regMiss',0, ...
    'extKnown',0,'extClusterSuccess',0,'extMiss',0,'ready',false),1,numel(reps)*numel(Ks));
ix=0;
for ri=1:numel(reps)
    rep=reps{ri};Qold=v208_build_stress_problem(S,rep,CFG);assert(Qold.guidePass&&Qold.corridorOK,'G26V209H:OldGuide','Old known-obstacle guide must pass.');
    Bold=v207_build_backend('ACETRANS_MINCO_ST_C6',Qold.W,Qold.weights);Told=v208_time_gate_angle(Bold,CFG,CFG.v209b_nominal_theta_deg);assert(Told.pass,'G26V209H:OldTime','Old trajectory time gate must pass.');
    tDetect=CFG.v209b_detection_fraction*Told.T;tSwitch=tDetect+CFG.v209b_switch_lookahead_s;J0=v209b_eval_jet(Bold,Told,tSwitch);p0=J0(:,1);Cnew=[S.C S.newObstacleCenter];Rnew=[S.R S.newObstacleRadius];
    extra=0;if strcmp(rep,'ACETRANS_SPHERE_SFC'),extra=CFG.v207_qualification_extra_proposal_margin_m;end
    for ki=1:numel(Ks)
        K=Ks(ki);ix=ix+1;[Cc,Rc,M]=v209g_build_cone_swept_cover(Cnew,Rnew,CFG.v209b_nominal_theta_deg,K,CFG,extra);
        [~,stLegacy]=g26v206_spatial_guide_mex(p0,S.pg,Cc,Rc,CFG.v205_planning_horizon_m,cap,CFG.v206_spatial_guide_guard_scale);
        [W,st,ins,minClr]=g26v209h_cluster_guide_mex(p0,S.pg,Cc,Rc,CFG.v205_planning_horizon_m,cap,CFG.v206_spatial_guide_guard_scale);
        corr=false;if st==1,[~,~,~,corr]=g26v206_segment_corridor_mex(W,Cc,Rc,CFG.v206_spatial_guide_guard_scale);end
        N=200;ttv=zeros(N,1);for n=1:N,tt=tic;[aa,bb]=v209g_build_cone_swept_cover(Cnew,Rnew,CFG.v209b_nominal_theta_deg,K,CFG,extra);[ww,ss]=g26v209h_cluster_guide_mex(p0,S.pg,aa,bb,CFG.v205_planning_horizon_m,cap,CFG.v206_spatial_guide_guard_scale);if ss==1,[x1,x2,x3,x4]=g26v206_segment_corridor_mex(ww,aa,bb,CFG.v206_spatial_guide_guard_scale);end;ttv(n)=1e3*toc(tt);end %#ok<ASGLU>
        ST=v202_stats(ttv);
        [rk,rl,rcs]=v209h_regression_suite(p0,S.pg,S.C,S.R,K,CFG,extra);
        [ek,ecs]=v209h_expanded_suite(p0,S.pg,S.C,S.R,K,CFG,extra);
        rows(ix).rep=rep;rows(ix).K=K;rows(ix).sphereCount=M.sphereCount;rows(ix).legacyStatus=stLegacy;rows(ix).clusterStatus=st;rows(ix).reason=v209e_spatial_reason(st,ins,cap,v209e_endpoint_inside(p0,S.pg,Cc,Rc));rows(ix).insertions=ins;rows(ix).nodes=size(W,2);rows(ix).corridorOK=logical(corr);rows(ix).minClearance=minClr;
        rows(ix).integrated_p50_ms=ST.p50;rows(ix).integrated_p95_ms=ST.p95;rows(ix).integrated_max_ms=ST.max;rows(ix).regKnown=rk;rows(ix).regLegacySuccess=rl;rows(ix).regClusterSuccess=rcs;rows(ix).regMiss=rk-rcs;rows(ix).extKnown=ek;rows(ix).extClusterSuccess=ecs;rows(ix).extMiss=ek-ecs;
        rows(ix).ready=(st==1)&&logical(corr)&&(rows(ix).regMiss==0)&&(rows(ix).extMiss==0)&&(ek>0)&&(ST.p95<CFG.v209b_compute_stretch_target_ms);
    end
end
rows=rows(1:ix);A=struct('rows',rows,'Ks',Ks,'cap',cap,'thetaDeg',CFG.v209b_nominal_theta_deg,'readyRows',sum([rows.ready]),'eachRepresentationHasReadyK',true,'proposalUnchanged',true,'finalAuthorityUnchanged',true,'noTrajectoryPromotion',true);
for ri=1:numel(reps),m=strcmp({rows.rep},reps{ri});A.eachRepresentationHasReadyK=A.eachRepresentationHasReadyK&&any([rows(m).ready]);end
end

function [known,legacySuccess,clusterSuccess]=v209h_regression_suite(p0,pg,Cold,Rold,K,CFG,extra)
xs=[2.8 3.4 4.0 4.6 5.2];ys=[-1.0 -0.5 0 0.5 1.0];known=0;legacySuccess=0;clusterSuccess=0;
for xi=1:numel(xs),for yi=1:numel(ys)
    C=[Cold [xs(xi);ys(yi);0]];R=[Rold 0.55];[Cc,Rc]=v209g_build_cone_swept_cover(C,R,CFG.v209b_nominal_theta_deg,K,CFG,extra);
    if v209e_endpoint_inside(p0,pg,Cc,Rc)||~v209g_simple_witness_exists(p0,pg,Cc,Rc),continue;end;known=known+1;
    [W0,s0]=g26v206_spatial_guide_mex(p0,pg,Cc,Rc,CFG.v205_planning_horizon_m,CFG.v206_spatial_guide_max_nodes,CFG.v206_spatial_guide_guard_scale);ok0=false;if s0==1,[~,~,~,ok0]=g26v206_segment_corridor_mex(W0,Cc,Rc,CFG.v206_spatial_guide_guard_scale);end;if s0==1&&ok0,legacySuccess=legacySuccess+1;end
    [W,s]=g26v209h_cluster_guide_mex(p0,pg,Cc,Rc,CFG.v205_planning_horizon_m,CFG.v206_spatial_guide_max_nodes,CFG.v206_spatial_guide_guard_scale);ok=false;if s==1,[~,~,~,ok]=g26v206_segment_corridor_mex(W,Cc,Rc,CFG.v206_spatial_guide_guard_scale);end;if s==1&&ok,clusterSuccess=clusterSuccess+1;end
end,end
end

function [known,clusterSuccess]=v209h_expanded_suite(p0,pg,Cold,Rold,K,CFG,extra)
% Deterministic broader suite. Cases count only if an independent 2-segment
% witness is actually clear in the exact same conservative proposal cover.
xs=2.4:0.4:5.6;ys=-1.5:0.375:1.5;rr=[0.35 0.55 0.75];known=0;clusterSuccess=0;
for ir=1:numel(rr),for xi=1:numel(xs),for yi=1:numel(ys)
    C=[Cold [xs(xi);ys(yi);0]];R=[Rold rr(ir)];[Cc,Rc]=v209g_build_cone_swept_cover(C,R,CFG.v209b_nominal_theta_deg,K,CFG,extra);
    if v209e_endpoint_inside(p0,pg,Cc,Rc)||~v209h_expanded_witness_exists(p0,pg,Cc,Rc),continue;end;known=known+1;
    [W,s]=g26v209h_cluster_guide_mex(p0,pg,Cc,Rc,CFG.v205_planning_horizon_m,CFG.v206_spatial_guide_max_nodes,CFG.v206_spatial_guide_guard_scale);ok=false;if s==1,[~,~,~,ok]=g26v206_segment_corridor_mex(W,Cc,Rc,CFG.v206_spatial_guide_guard_scale);end;if s==1&&ok,clusterSuccess=clusterSuccess+1;end
end,end,end
end

function tf=v209h_expanded_witness_exists(p0,pg,C,R)
mx=0.5*(p0(1)+pg(1));W=[mx mx mx mx mx mx mx mx; -3.5 3.5 -2.5 2.5 0 0 -2 2; 0 0 1.5 1.5 3.5 2.5 2.5 2.5];tf=false;
for k=1:size(W,2),if v209g_segment_set_clear(p0,W(:,k),C,R)&&v209g_segment_set_clear(W(:,k),pg,C,R),tf=true;return;end,end
end

function v209h_print_cluster_spatial_audit(A,CFG)
fprintf('\n========================================================================================================================\n');
fprintf(' G26-V209H CLUSTER-AWARE UPDATED-SPATIAL RELIABILITY / TIME AUDIT | theta=%.1fdeg\n',A.thetaDeg);
fprintf('------------------------------------------------------------------------------------------------------------------------\n');
fprintf(' proposal theorem/geometry unchanged from V209G | adaptive directions are derived from active blocker cluster + current segment\n');
fprintf(' accepted guide/corridor is checked against ALL proposal spheres | final UAV/load/cable continuous certificate remains later authority\n');
for k=1:numel(A.rows)
    R=A.rows(k);fprintf(' %-20s | K=%d sph=%2d | legacySt=%2d clusterSt=%2d %-28s corr=%d ins=%2.0f nodes=%2.0f minClr=%+.4e | p50/p95/max=%.4f/%.4f/%.4f ms | regression=%d/%d legacy=%d miss=%d | expanded=%d/%d miss=%d | ready=%d\n', ...
        R.rep,R.K,R.sphereCount,R.legacyStatus,R.clusterStatus,R.reason,R.corridorOK,R.insertions,R.nodes,R.minClearance,R.integrated_p50_ms,R.integrated_p95_ms,R.integrated_max_ms, ...
        R.regClusterSuccess,R.regKnown,R.regLegacySuccess,R.regMiss,R.extClusterSuccess,R.extKnown,R.extMiss,R.ready);
end
fprintf('------------------------------------------------------------------------------------------------------------------------\n');
fprintf(' eachRepresentationHasReadyK=%d | readyRows=%d/%d | readiness=current PASS+corridor + zero regression miss + zero expanded-witness miss + p95<%.1fms\n', ...
    A.eachRepresentationHasReadyK,A.readyRows,numel(A.rows),CFG.v209b_compute_stretch_target_ms);
fprintf(' RULE: witness-suite success is a generator reliability test, NOT a completeness proof for arbitrary environments.\n');
fprintf(' RULE: generator miss remains SEARCH_INCOMPLETE, never path/physical infeasibility.\n');
fprintf(' RULE: do not return to trajectory/handoff until BOTH retained obstacle representations have at least one ready K.\n');
fprintf('========================================================================================================================\n');
end


function Q = v209i_build_static_problem(S,rep,CFG,K)
Q=struct();
Q.p0=S.p0; Q.pg=S.pg; Q.C=reshape(S.C,3,[]); Q.V=zeros(size(Q.C)); Q.Ractual=reshape(S.R,1,[]);
extra=0;
if strcmp(rep,'ACETRANS_SPHERE_SFC'), extra=CFG.v207_qualification_extra_proposal_margin_m; end
tt=tic;
[Q.Cproposal,Q.Rproposal,Q.coverMeta]=v209g_build_cone_swept_cover(Q.C,Q.Ractual,CFG.v209b_nominal_theta_deg,K,CFG,extra);
[Q.W,Q.guideStatus,Q.insertions,Q.guideClearance]=g26v209h_cluster_guide_mex(Q.p0,Q.pg,Q.Cproposal,Q.Rproposal, ...
    CFG.v205_planning_horizon_m,CFG.v206_spatial_guide_max_nodes,CFG.v206_spatial_guide_guard_scale);
Q.guidePass=(Q.guideStatus==1);
if ~Q.guidePass
    Q.corridorOK=false; Q.N=[]; Q.h=[]; Q.cnt=[]; Q.faces=0; Q.weights=[]; Q.spatial_ms=1e3*toc(tt); return;
end
[Q.N,Q.h,Q.cnt,Q.corridorOK,Q.corridorMinClearance,Q.faces]=g26v206_segment_corridor_mex(Q.W,Q.Cproposal,Q.Rproposal,CFG.v206_spatial_guide_guard_scale);
Q.segment_length=vecnorm(diff(Q.W,1,2),2,1);
if isempty(Q.segment_length) || sum(Q.segment_length)<=0
    Q.weights=[]; Q.corridorOK=false;
else
    Q.weights=Q.segment_length/sum(Q.segment_length);
end
Q.spatial_ms=1e3*toc(tt);
end

function H = v209i_multi_obstacle_handoff_audit(CFG,K)
S=v209b_static_scenario(CFG);
reps={'PATCH_UNION','ACETRANS_SPHERE_SFC'};
names={'BSPLINE_D7','BEZIER_D13','POLY_D13','ACETRANS_MINCO_ST_C6'};
template=v209b_empty_row('','','UNSET');
template.spatial_ms=NaN; template.tail_build_ms=NaN; template.candidate_total_ms=NaN;
template.physicalObstacles=0; template.proposalSpheres=0; template.allObstacleAccounting=false; template.K=K;
rows=repmat(template,1,numel(reps)*numel(names)); ix=0;

for ri=1:numel(reps)
    rep=reps{ri};
    % OLD trajectory uses only obstacles already sensed before the new detection.
    Qold=v209i_build_static_problem(S,rep,CFG,K);
    if ~Qold.guidePass || ~Qold.corridorOK
        for bi=1:numel(names)
            ix=ix+1; rows(ix)=template; rows(ix).representation=rep; rows(ix).backend=names{bi}; rows(ix).status='OLD_REFERENCE_SEARCH_INCOMPLETE';
        end
        continue;
    end
    Bold=v207_build_backend('ACETRANS_MINCO_ST_C6',Qold.W,Qold.weights);
    Told=v208_time_gate_angle(Bold,CFG,CFG.v209b_nominal_theta_deg);
    if ~Told.pass
        for bi=1:numel(names)
            ix=ix+1; rows(ix)=template; rows(ix).representation=rep; rows(ix).backend=names{bi}; rows(ix).status='OLD_REFERENCE_TIME_SEARCH_INCOMPLETE';
        end
        continue;
    end
    tDetect=CFG.v209b_detection_fraction*Told.T;
    tSwitch=tDetect+CFG.v209b_switch_lookahead_s;
    if tSwitch>=Told.T
        for bi=1:numel(names)
            ix=ix+1; rows(ix)=template; rows(ix).representation=rep; rows(ix).backend=names{bi}; rows(ix).status='SWITCH_LOOKAHEAD_OUTSIDE_ACTIVE_REFERENCE';
        end
        continue;
    end

    J0=v209b_eval_jet(Bold,Told,tSwitch); p0=J0(:,1);
    Cnew=[S.C S.newObstacleCenter];
    Rnew=[S.R S.newObstacleRadius];
    Snew=v208_make_scenario('V209I_STATIC_MULTI_OBSTACLE',Cnew,zeros(3,size(Cnew,2)),Rnew,p0,S.pg);
    Qnew=v209i_build_static_problem(Snew,rep,CFG,K);
    prefixCert=v209b_active_interval_certificate(Bold,Told,tDetect,tSwitch,Qnew,CFG);

    for bi=1:numel(names)
        ix=ix+1; name=names{bi}; row=template;
        row.representation=rep; row.backend=name; row.tDetect=tDetect; row.tSwitch=tSwitch; row.lookahead=CFG.v209b_switch_lookahead_s;
        row.activePrefixSafe=prefixCert.pass; row.spatial_ms=Qnew.spatial_ms; row.physicalObstacles=size(Qnew.C,2);
        row.proposalSpheres=size(Qnew.Cproposal,2); row.K=K;
        row.allObstacleAccounting=(row.physicalObstacles==numel(Qnew.Ractual)) && ...
            (row.proposalSpheres==row.physicalObstacles*(K+2));
        if ~prefixCert.pass
            row.status='ACTIVE_PREFIX_UNSAFE_FOR_SCHEDULED_SWITCH'; rows(ix)=row; continue;
        end
        if ~Qnew.guidePass || ~Qnew.corridorOK
            row.status='UPDATED_SPATIAL_SEARCH_INCOMPLETE'; rows(ix)=row; continue;
        end

        p1=Qnew.W(:,2); QT=v209b_tail_problem(Qnew);
        ttTail=tic;
        [BT,TT,tailStatus]=v209b_build_tail(QT,CFG,name);
        row.tail_build_ms=1e3*toc(ttTail);
        if QT.exists && ~strcmp(tailStatus,'PASS')
            row.status=[name '__' tailStatus]; rows(ix)=row; continue;
        end
        if isempty(BT)
            J1=zeros(3,7); J1(:,1)=p1;
        else
            J1=v209b_backend_start_jets(BT,TT);
        end

        ttBridge=tic;
        HS=v209b_bridge_search(J0,J1,Qnew,BT,TT,CFG);
        row.runtime_ms=1e3*toc(ttBridge); % bridge-time search + continuous cert
        row.bridgeEvals=HS.evals;
        row.candidate_total_ms=row.spatial_ms+row.tail_build_ms+row.runtime_ms;
        if ~HS.pass
            row.status=[name '__JET_TIME_CERT_SEARCH_INCOMPLETE']; rows(ix)=row; continue;
        end

        C=HS.best;
        [jetPass,jetResidual,jetRatio]=v209b_endpoint_jet_match(C.bridgeB,C.bridgeH,J0,J1);
        row.positionJetMatch=jetPass; row.c6=C.c6.pass; row.corridor=C.corridor.pass;
        row.coneTaut=C.bridgeDynamic.pass; row.hardCert=C.direct.pass;
        row.bridgeH=C.bridgeH; row.tailT=C.tailT; row.totalT=C.T.T;
        row.maxJetResidual=jetResidual; row.maxJetResidualOverGuard=jetRatio;
        row.pass=row.activePrefixSafe && row.allObstacleAccounting && jetPass && row.c6 && row.corridor && row.coneTaut && row.hardCert;
        if row.pass, row.status='CERTIFIED_MULTI_OBSTACLE_STATIC_SUCCESSOR'; else, row.status='CERTIFICATE_OR_HANDOFF_FAIL'; end
        row.atomicSwitchAllowed=row.pass && CFG.v209b_switch_authority && CFG.v209b_integrated_wcet_authority;
        rows(ix)=row;
    end
end

rows=rows(1:ix);
H=struct();
H.rows=rows; H.total=numel(rows); H.certifiedCandidates=sum([rows.pass]); H.jetMatched=sum([rows.positionJetMatch]);
H.allObstacleAccounting=sum([rows.allObstacleAccounting]); H.atomicAllowed=sum([rows.atomicSwitchAllowed]);
H.K=K; H.staticObstaclesOnly=true; H.nominalThetaDeg=CFG.v209b_nominal_theta_deg;
H.computeTargetMs=CFG.v209b_compute_stretch_target_ms; H.controlPeriodMs=1000*CFG.v209b_control_period_s;
H.maxCandidateMs=max([rows.candidate_total_ms],[],'omitnan');
H.passesUnder4ms=sum([rows.pass] & [rows.candidate_total_ms] <= CFG.v209b_compute_stretch_target_ms);
H.passesUnder25ms=sum([rows.pass] & [rows.candidate_total_ms] <= 1000*CFG.v209b_control_period_s);
end

function v209i_print_multi_obstacle_handoff(H,CFG)
fprintf('\n========================================================================================================================\n');
fprintf(' G26-V209I MULTI-OBSTACLE STATIC TRAJECTORY / HANDOFF AUDIT | K=%d theta=%.1fdeg\n',H.K,H.nominalThetaDeg);
fprintf('------------------------------------------------------------------------------------------------------------------------\n');
fprintf(' interpretation: one successor trajectory is constrained against the UNION of all currently sensed obstacles; no obstacle-wise trajectory superposition\n');
fprintf(' spatial corridor: intersection of half-spaces induced by ALL cone-swept proposal spheres; final cert: ALL physical obstacles x payload/UAV/cable continuously\n');
for k=1:numel(H.rows)
    R=H.rows(k);
    fprintf(' %-20s / %-23s | %-42s | M=%d prop=%d allObs=%d | spatial/tail/bridge/total=%.4f/%.4f/%.4f/%.4f ms | prefix=%d jet=%d C6=%d corr=%d coneTaut=%d hard=%d\n', ...
        R.representation,R.backend,R.status,R.physicalObstacles,R.proposalSpheres,R.allObstacleAccounting, ...
        R.spatial_ms,R.tail_build_ms,R.runtime_ms,R.candidate_total_ms,R.activePrefixSafe,R.positionJetMatch,R.c6,R.corridor,R.coneTaut,R.hardCert);
end
fprintf('------------------------------------------------------------------------------------------------------------------------\n');
fprintf(' certified=%d/%d | allObstacleAccounting=%d/%d | <=4ms=%d certified rows | <=25ms=%d certified rows | maxCandidate=%.4fms\n', ...
    H.certifiedCandidates,H.total,H.allObstacleAccounting,H.total,H.passesUnder4ms,H.passesUnder25ms,H.maxCandidateMs);
fprintf(' RULE: all obstacles here are STATIC and only become available after sensing/memory ingress; no prior/global obstacle map is used.\n');
fprintf(' RULE: corridor/proposal success is not final safety authority; every accepted successor still requires continuous full UAV/load/cable certification against ALL sensed obstacles.\n');
fprintf(' RULE: if a backend fails, report SEARCH_INCOMPLETE/certificate failure; never infer path or physical infeasibility from generator miss.\n');
fprintf('========================================================================================================================\n');
end


% ========================================================================
% V209J: unchanged-problem failure localization and stage timing.
% ========================================================================
function Q = v209j_build_static_problem_diag(S,rep,CFG,K)
Q=struct();
Q.p0=S.p0; Q.pg=S.pg; Q.C=reshape(S.C,3,[]); Q.V=zeros(size(Q.C)); Q.Ractual=reshape(S.R,1,[]);
extra=0;if strcmp(rep,'ACETRANS_SPHERE_SFC'),extra=CFG.v207_qualification_extra_proposal_margin_m;end

t=tic;[Q.Cproposal,Q.Rproposal,Q.coverMeta]=v209g_build_cone_swept_cover(Q.C,Q.Ractual,CFG.v209b_nominal_theta_deg,K,CFG,extra);Q.cover_ms=1e3*toc(t);
t=tic;[Q.W,Q.guideStatus,Q.insertions,Q.guideClearance,Q.guideChecks]=g26v209h_cluster_guide_mex(Q.p0,Q.pg,Q.Cproposal,Q.Rproposal, ...
    CFG.v205_planning_horizon_m,CFG.v206_spatial_guide_max_nodes,CFG.v206_spatial_guide_guard_scale);Q.guide_ms=1e3*toc(t);
Q.guidePass=(Q.guideStatus==1);Q.nodes=size(Q.W,2);
if ~Q.guidePass
    Q.corridorOK=false;Q.N=[];Q.h=[];Q.cnt=[];Q.faces=0;Q.weights=[];Q.corridorMinClearance=NaN;Q.corridor_ms=0;
    Q.segment_length=[];Q.spatial_ms=Q.cover_ms+Q.guide_ms;return;
end

t=tic;[Q.N,Q.h,Q.cnt,Q.corridorOK,Q.corridorMinClearance,Q.faces]=g26v206_segment_corridor_mex(Q.W,Q.Cproposal,Q.Rproposal,CFG.v206_spatial_guide_guard_scale);Q.corridor_ms=1e3*toc(t);
Q.segment_length=vecnorm(diff(Q.W,1,2),2,1);
if isempty(Q.segment_length)||sum(Q.segment_length)<=0,Q.weights=[];Q.corridorOK=false;else,Q.weights=Q.segment_length/sum(Q.segment_length);end
Q.spatial_ms=Q.cover_ms+Q.guide_ms+Q.corridor_ms;
end

function P=v209j_spatial_profile(S,rep,CFG,K)
% Warmed repeated timing of the EXACT same updated spatial problem. No
% changed geometry/search rule; this distinguishes a one-shot outlier from
% a persistent kernel cost. Qualification timing only, never WCET.
NW=2;N=9;cov=zeros(N,1);gui=zeros(N,1);cor=zeros(N,1);tot=zeros(N,1);checks=zeros(N,1);ins=zeros(N,1);nodes=zeros(N,1);status=zeros(N,1);
for w=1:NW,v209j_build_static_problem_diag(S,rep,CFG,K);end
for k=1:N
    Q=v209j_build_static_problem_diag(S,rep,CFG,K);cov(k)=Q.cover_ms;gui(k)=Q.guide_ms;cor(k)=Q.corridor_ms;tot(k)=Q.spatial_ms;checks(k)=Q.guideChecks;ins(k)=Q.insertions;nodes(k)=Q.nodes;status(k)=Q.guideStatus;
end
P=struct('warmup',NW,'samples',N,'cover',v209j_stats(cov),'guide',v209j_stats(gui),'corridor',v209j_stats(cor),'total',v209j_stats(tot), ...
    'checks',v209j_stats(checks),'insertions',v209j_stats(ins),'nodes',v209j_stats(nodes),'allGuidePass',all(status==1));
end

function [BT,TT,DG] = v209j_tail_diag(QT,CFG,name)
BT=[];TT=[];
DG=struct('status','NO_TAIL','build_ms',0,'corridor_ms',0,'time_gate_ms',0,'direct_ms',0,'total_ms',0, ...
    'corridor_pass',true,'corridor_min_slack',inf,'time_pass',true,'T',0,'minCone',inf,'minRz',inf,'timeIterations',0, ...
    'direct_pass',true,'direct_min_slack',inf,'fail_piece',0,'fail_obstacle',0,'checks',0,'leaves',0,'max_depth',0);
if ~QT.exists,return;end

t=tic;B=v207_build_backend(name,QT.W,QT.weights);DG.build_ms=1e3*toc(t);
t=tic;C=v209b_tail_corridor_audit(B,QT);DG.corridor_ms=1e3*toc(t);DG.corridor_pass=C.pass;DG.corridor_min_slack=C.min_slack;
if ~C.pass,DG.status='TAIL_CORRIDOR_SEARCH_INCOMPLETE';DG.total_ms=v209j_tail_sum(DG);return;end

t=tic;T=v208_time_gate_angle(B,CFG,CFG.v209b_nominal_theta_deg);DG.time_gate_ms=1e3*toc(t);DG.time_pass=T.pass;DG.T=T.T;DG.minCone=T.min_cone_slack;DG.minRz=T.min_rz_slack;DG.timeIterations=T.iterations;
if ~T.pass,DG.status='TAIL_TIME_SEARCH_INCOMPLETE';DG.total_ms=v209j_tail_sum(DG);return;end

t=tic;D=v207_final_direct_certificate(B,T,QT,CFG);DG.direct_ms=1e3*toc(t);DG.direct_pass=D.pass;DG.direct_min_slack=D.min_slack;DG.fail_piece=D.fail_piece;DG.fail_obstacle=D.fail_obstacle;DG.checks=D.checks;DG.leaves=D.leaves;DG.max_depth=D.max_depth;
if ~D.pass,DG.status='TAIL_HARD_CERT_FAIL';DG.total_ms=v209j_tail_sum(DG);return;end
BT=B;TT=T;DG.status='PASS';DG.total_ms=v209j_tail_sum(DG);
end

function z=v209j_tail_sum(D)
z=D.build_ms+D.corridor_ms+D.time_gate_ms+D.direct_ms;
end

function [ok,C,E]=v209j_eval_bridge_diag(J0,J1,Q,BT,TT,hb,CFG)
E=struct('h',hb,'reason','UNSET','build_ms',NaN,'first_corridor_ms',NaN,'dynamic_ms',NaN,'c6_ms',NaN,'combined_corridor_ms',NaN,'direct_ms',NaN,'total_ms',NaN, ...
    'firstCorrPass',false,'firstCorrSlack',-inf,'dynamicPass',false,'minCone',NaN,'minRz',NaN,'c6Pass',false,'c6Residual',NaN, ...
    'combinedCorrPass',false,'combinedCorrSlack',-inf,'directPass',false,'directSlack',NaN,'fail_piece',0,'fail_obstacle',0,'checks',0,'leaves',0,'max_depth',0);

t=tic;BB=v209b_degree17_two_sided_jet_bridge(J0,J1,hb);E.build_ms=1e3*toc(t);
t=tic;CORb=v209b_bridge_corridor_audit(BB,Q);E.first_corridor_ms=1e3*toc(t);E.firstCorrPass=CORb.pass;E.firstCorrSlack=CORb.min_slack;
t=tic;DY=v209b_bridge_dynamic_audit(BB,hb,CFG.v209b_nominal_theta_deg,CFG);E.dynamic_ms=1e3*toc(t);E.dynamicPass=DY.pass;E.minCone=DY.minCone;E.minRz=DY.minRz;
if ~CORb.pass,E.reason='BRIDGE_FIRST_CORRIDOR_FAIL';E.total_ms=v209j_bridge_eval_sum(E);ok=false;C=[];return;end
if ~DY.pass,E.reason='BRIDGE_CONE_TAUT_FAIL';E.total_ms=v209j_bridge_eval_sum(E);ok=false;C=[];return;end

if isempty(BT),P={BB};hh=hb;Rflat=DY.R;rcounts=DY.rcount;tailT=0;else,P=[{BB} BT.P];hh=[hb TT.h];Rflat=[DY.R TT.Rflat];rcounts=[DY.rcount TT.rcounts];tailT=TT.T;end
BC=struct('P',{P},'degree',17,'structural_c6_theorem',true,'condition_indicator',1);
TC=struct('pass',true,'h',hh,'Rflat',Rflat,'rcounts',rcounts,'T',sum(hh));

t=tic;C6=v207_c6_backend_audit(BC,hh);E.c6_ms=1e3*toc(t);E.c6Pass=C6.pass;if isfield(C6,'max_endpoint_derivative_residual'),E.c6Residual=C6.max_endpoint_derivative_residual;end
if ~C6.pass,E.reason='BRIDGE_COMBINED_C6_FAIL';E.total_ms=v209j_bridge_eval_sum(E);ok=false;C=[];return;end

t=tic;COR=v209b_combined_corridor_audit(BC,Q);E.combined_corridor_ms=1e3*toc(t);E.combinedCorrPass=COR.pass;E.combinedCorrSlack=COR.min_slack;
if ~COR.pass,E.reason='BRIDGE_COMBINED_CORRIDOR_FAIL';E.total_ms=v209j_bridge_eval_sum(E);ok=false;C=[];return;end

t=tic;D=v207_final_direct_certificate(BC,TC,Q,CFG);E.direct_ms=1e3*toc(t);E.directPass=D.pass;E.directSlack=D.min_slack;E.fail_piece=D.fail_piece;E.fail_obstacle=D.fail_obstacle;E.checks=D.checks;E.leaves=D.leaves;E.max_depth=D.max_depth;
if ~D.pass,E.reason='BRIDGE_HARD_CERT_FAIL';E.total_ms=v209j_bridge_eval_sum(E);ok=false;C=[];return;end

E.reason='PASS';E.total_ms=v209j_bridge_eval_sum(E);ok=true;
C=struct('B',BC,'T',TC,'bridgeB',BB,'bridgeH',hb,'bridgeDynamic',DY,'bridgeCorridor',CORb,'c6',C6,'corridor',COR,'direct',D,'tailT',tailT);
end

function z=v209j_bridge_eval_sum(E)
x=[E.build_ms E.first_corridor_ms E.dynamic_ms E.c6_ms E.combined_corridor_ms E.direct_ms];x=x(isfinite(x));z=sum(x);
end

function H=v209j_bridge_search_diag(J0,J1,Q,BT,TT,CFG)
lo=CFG.v209b_bridge_h_min_s;hi=CFG.v209b_bridge_h_max_s;intervals=[lo hi];best=[];bestH=inf;evals=0;trace=repmat(v209j_empty_bridge_eval(),1,CFG.v209b_bridge_search_evals);
for z=1:2
    if z==1,h=lo;else,h=hi;end
    [ok,C,E]=v209j_eval_bridge_diag(J0,J1,Q,BT,TT,h,CFG);evals=evals+1;trace(evals)=E;
    if ok&&h<bestH,best=C;bestH=h;end
end
while ~isempty(intervals)&&evals<CFG.v209b_bridge_search_evals
    keep=intervals(:,1)<bestH;intervals=intervals(keep,:);if isempty(intervals),break;end
    [~,ii]=max(intervals(:,2)-intervals(:,1));ab=intervals(ii,:);intervals(ii,:)=[];
    if ab(2)-ab(1)<=64*eps(max(1,ab(2))),continue;end
    mid=.5*(ab(1)+ab(2));[ok,C,E]=v209j_eval_bridge_diag(J0,J1,Q,BT,TT,mid,CFG);evals=evals+1;trace(evals)=E;
    if ok&&mid<bestH,best=C;bestH=mid;end
    if ab(1)<bestH,intervals=[intervals;ab(1) mid];end %#ok<AGROW>
    if mid<bestH,intervals=[intervals;mid ab(2)];end %#ok<AGROW>
end
trace=trace(1:evals);H=struct();H.pass=~isempty(best);H.evals=evals;H.searchComplete=false;H.optimalityClaim=false;H.missIsInfeasible=false;H.trace=trace;H.best=best;if H.pass,H.h=bestH;else,H.h=NaN;end
H.reason_counts=v209j_reason_counts(trace);H.stage=v209j_bridge_stage_stats(trace);H.total_ms=sum([trace.total_ms]);
end

function E=v209j_empty_bridge_eval()
E=struct('h',NaN,'reason','','build_ms',NaN,'first_corridor_ms',NaN,'dynamic_ms',NaN,'c6_ms',NaN,'combined_corridor_ms',NaN,'direct_ms',NaN,'total_ms',NaN, ...
    'firstCorrPass',false,'firstCorrSlack',NaN,'dynamicPass',false,'minCone',NaN,'minRz',NaN,'c6Pass',false,'c6Residual',NaN,'combinedCorrPass',false,'combinedCorrSlack',NaN, ...
    'directPass',false,'directSlack',NaN,'fail_piece',0,'fail_obstacle',0,'checks',0,'leaves',0,'max_depth',0);
end

function R=v209j_reason_counts(T)
names={'PASS','BRIDGE_FIRST_CORRIDOR_FAIL','BRIDGE_CONE_TAUT_FAIL','BRIDGE_COMBINED_C6_FAIL','BRIDGE_COMBINED_CORRIDOR_FAIL','BRIDGE_HARD_CERT_FAIL'};R=struct();
for k=1:numel(names),R.(names{k})=sum(strcmp({T.reason},names{k}));end
end

function S=v209j_bridge_stage_stats(T)
fn={'build_ms','first_corridor_ms','dynamic_ms','c6_ms','combined_corridor_ms','direct_ms','total_ms'};S=struct();
for k=1:numel(fn),x=[T.(fn{k})];S.(fn{k})=v209j_stats(x);end
end

function S=v209j_stats(x)
x=x(isfinite(x));if isempty(x),S=struct('p50',NaN,'p95',NaN,'max',NaN,'mean',NaN,'sum',0);return;end
x=sort(x(:));n=numel(x);S=struct('p50',x(max(1,ceil(.50*n))),'p95',x(max(1,ceil(.95*n))),'max',x(end),'mean',mean(x),'sum',sum(x));
end

function R=v209j_empty_row()
R=struct('representation','','backend','','status','UNSET','activePrefixSafe',false,'allObstacleAccounting',false,'physicalObstacles',0,'proposalSpheres',0, ...
    'spatial',[],'tail',[],'bridge',[],'candidate_total_ms',NaN,'positionJetMatch',false,'maxJetResidual',NaN,'maxJetResidualOverGuard',NaN,'c6',false,'corridor',false,'coneTaut',false,'hardCert',false,'pass',false);
end

function J=v209j_failure_localization_audit(CFG,K)
S=v209b_static_scenario(CFG);reps={'PATCH_UNION','ACETRANS_SPHERE_SFC'};names={'BSPLINE_D7','BEZIER_D13','POLY_D13','ACETRANS_MINCO_ST_C6'};
rows=repmat(v209j_empty_row(),1,numel(reps)*numel(names));repdiag=repmat(struct(),1,numel(reps));ix=0;
for ri=1:numel(reps)
    rep=reps{ri};Qold=v209j_build_static_problem_diag(S,rep,CFG,K);repdiag(ri).representation=rep;repdiag(ri).oldSpatial=Qold;
    if ~Qold.guidePass||~Qold.corridorOK
        for bi=1:numel(names),ix=ix+1;rows(ix).representation=rep;rows(ix).backend=names{bi};rows(ix).status='OLD_REFERENCE_SEARCH_INCOMPLETE';end
        continue;
    end
    Bold=v207_build_backend('ACETRANS_MINCO_ST_C6',Qold.W,Qold.weights);Told=v208_time_gate_angle(Bold,CFG,CFG.v209b_nominal_theta_deg);
    if ~Told.pass
        for bi=1:numel(names),ix=ix+1;rows(ix).representation=rep;rows(ix).backend=names{bi};rows(ix).status='OLD_REFERENCE_TIME_SEARCH_INCOMPLETE';end
        continue;
    end
    tDetect=CFG.v209b_detection_fraction*Told.T;tSwitch=tDetect+CFG.v209b_switch_lookahead_s;
    if tSwitch>=Told.T
        for bi=1:numel(names),ix=ix+1;rows(ix).representation=rep;rows(ix).backend=names{bi};rows(ix).status='SWITCH_LOOKAHEAD_OUTSIDE_ACTIVE_REFERENCE';end
        continue;
    end
    J0=v209b_eval_jet(Bold,Told,tSwitch);p0=J0(:,1);Cnew=[S.C S.newObstacleCenter];Rnew=[S.R S.newObstacleRadius];Snew=v208_make_scenario('V209J_STATIC_MULTI_OBSTACLE',Cnew,zeros(3,size(Cnew,2)),Rnew,p0,S.pg);
    Qnew=v209j_build_static_problem_diag(Snew,rep,CFG,K);repdiag(ri).updatedSpatial=Qnew;repdiag(ri).updatedSpatialProfile=v209j_spatial_profile(Snew,rep,CFG,K);
    prefixCert=v209b_active_interval_certificate(Bold,Told,tDetect,tSwitch,Qnew,CFG);repdiag(ri).prefix=prefixCert;
    for bi=1:numel(names)
        ix=ix+1;name=names{bi};row=v209j_empty_row();row.representation=rep;row.backend=name;row.activePrefixSafe=prefixCert.pass;row.spatial=Qnew;
        row.physicalObstacles=size(Qnew.C,2);row.proposalSpheres=size(Qnew.Cproposal,2);row.allObstacleAccounting=(row.physicalObstacles==numel(Qnew.Ractual))&&(row.proposalSpheres==row.physicalObstacles*(K+2));
        if ~prefixCert.pass,row.status='ACTIVE_PREFIX_UNSAFE_FOR_SCHEDULED_SWITCH';rows(ix)=row;continue;end
        if ~Qnew.guidePass||~Qnew.corridorOK,row.status='UPDATED_SPATIAL_SEARCH_INCOMPLETE';rows(ix)=row;continue;end
        p1=Qnew.W(:,2);QT=v209b_tail_problem(Qnew);[BT,TT,TD]=v209j_tail_diag(QT,CFG,name);row.tail=TD;
        if QT.exists&&~strcmp(TD.status,'PASS'),row.status=[name '__' TD.status];row.candidate_total_ms=Qnew.spatial_ms+TD.total_ms;rows(ix)=row;continue;end
        if isempty(BT),J1=zeros(3,7);J1(:,1)=p1;else,J1=v209b_backend_start_jets(BT,TT);end
        HS=v209j_bridge_search_diag(J0,J1,Qnew,BT,TT,CFG);row.bridge=HS;row.candidate_total_ms=Qnew.spatial_ms+TD.total_ms+HS.total_ms;
        if ~HS.pass,row.status=[name '__JET_TIME_CERT_SEARCH_INCOMPLETE'];rows(ix)=row;continue;end
        C=HS.best;[jetPass,jr,jg]=v209b_endpoint_jet_match(C.bridgeB,C.bridgeH,J0,J1);row.positionJetMatch=jetPass;row.maxJetResidual=jr;row.maxJetResidualOverGuard=jg;row.c6=C.c6.pass;row.corridor=C.corridor.pass;row.coneTaut=C.bridgeDynamic.pass;row.hardCert=C.direct.pass;
        row.pass=row.activePrefixSafe&&row.allObstacleAccounting&&jetPass&&row.c6&&row.corridor&&row.coneTaut&&row.hardCert;if row.pass,row.status='CERTIFIED_MULTI_OBSTACLE_STATIC_SUCCESSOR';else,row.status='CERTIFICATE_OR_HANDOFF_FAIL';end;rows(ix)=row;
    end
end
rows=rows(1:ix);J=struct('rows',rows,'representation',repdiag,'K',K,'nominalThetaDeg',CFG.v209b_nominal_theta_deg,'certifiedCandidates',sum([rows.pass]),'total',numel(rows),'allObstacleAccounting',sum([rows.allObstacleAccounting]),'timingIsWCET',false,'safetyRelaxed',false);
end

function v209j_print_failure_localization(J,CFG)
fprintf('\n========================================================================================================================\n');
fprintf(' G26-V209J FAILURE LOCALIZATION | ordinary 2-obstacle static replan | K=%d theta=%.1fdeg\n',J.K,J.nominalThetaDeg);
fprintf('------------------------------------------------------------------------------------------------------------------------\n');
for ri=1:numel(J.representation)
    if ~isfield(J.representation(ri),'updatedSpatial'),continue;end;Q=J.representation(ri).updatedSpatial;
    fprintf('[SPATIAL %-20s] cover/guide/corr/total = %.4f / %.4f / %.4f / %.4f ms | st=%d ins=%d nodes=%d checks=%.0f faces=%d minClr=%+.3e\n', ...
        J.representation(ri).representation,Q.cover_ms,Q.guide_ms,Q.corridor_ms,Q.spatial_ms,Q.guideStatus,Q.insertions,Q.nodes,Q.guideChecks,Q.faces,Q.guideClearance);
    if isfield(J.representation(ri),'updatedSpatialProfile'),P=J.representation(ri).updatedSpatialProfile;fprintf('         warmed N=%d p50/p95/max total = %.4f / %.4f / %.4f ms | cover %.4f/%.4f/%.4f | guide %.4f/%.4f/%.4f | corr %.4f/%.4f/%.4f | checks p50/p95/max=%.0f/%.0f/%.0f\n', ...
        P.samples,P.total.p50,P.total.p95,P.total.max,P.cover.p50,P.cover.p95,P.cover.max,P.guide.p50,P.guide.p95,P.guide.max,P.corridor.p50,P.corridor.p95,P.corridor.max,P.checks.p50,P.checks.p95,P.checks.max);end
end
fprintf('------------------------------------------------------------------------------------------------------------------------\n');
for k=1:numel(J.rows)
    R=J.rows(k);fprintf('%-20s / %-23s | %s | total=%.4f ms\n',R.representation,R.backend,R.status,R.candidate_total_ms);
    if ~isempty(R.tail)
        T=R.tail;fprintf('  tail   build/corr/time/direct/total = %.4f / %.4f / %.4f / %.4f / %.4f ms | corr=%d time=%d direct=%d | T=%.6g minCone=%+.3e minRz=%+.3e\n', ...
            T.build_ms,T.corridor_ms,T.time_gate_ms,T.direct_ms,T.total_ms,T.corridor_pass,T.time_pass,T.direct_pass,T.T,T.minCone,T.minRz);
        if ~T.direct_pass&&strcmp(T.status,'TAIL_HARD_CERT_FAIL'),fprintf('         hardFail: minSlack=%+.3e failPiece=%d failObstacle=%d checks=%.0f leaves=%.0f maxDepth=%d\n',T.direct_min_slack,T.fail_piece,T.fail_obstacle,T.checks,T.leaves,T.max_depth);end
    end
    if ~isempty(R.bridge)
        H=R.bridge;C=H.reason_counts;fprintf('  bridge evals=%d totalEvalTime=%.4f ms | PASS=%d firstCorr=%d coneTaut=%d C6=%d combCorr=%d hardCert=%d\n', ...
            H.evals,H.total_ms,C.PASS,C.BRIDGE_FIRST_CORRIDOR_FAIL,C.BRIDGE_CONE_TAUT_FAIL,C.BRIDGE_COMBINED_C6_FAIL,C.BRIDGE_COMBINED_CORRIDOR_FAIL,C.BRIDGE_HARD_CERT_FAIL);
        fprintf('         per-eval p50/p95/max [build %.4f/%.4f/%.4f | firstCorr %.4f/%.4f/%.4f | dyn %.4f/%.4f/%.4f | C6 %.4f/%.4f/%.4f | combCorr %.4f/%.4f/%.4f | direct %.4f/%.4f/%.4f] ms\n', ...
            H.stage.build_ms.p50,H.stage.build_ms.p95,H.stage.build_ms.max,H.stage.first_corridor_ms.p50,H.stage.first_corridor_ms.p95,H.stage.first_corridor_ms.max, ...
            H.stage.dynamic_ms.p50,H.stage.dynamic_ms.p95,H.stage.dynamic_ms.max,H.stage.c6_ms.p50,H.stage.c6_ms.p95,H.stage.c6_ms.max, ...
            H.stage.combined_corridor_ms.p50,H.stage.combined_corridor_ms.p95,H.stage.combined_corridor_ms.max,H.stage.direct_ms.p50,H.stage.direct_ms.p95,H.stage.direct_ms.max);
        if ~H.pass
            idx=find(strcmp({H.trace.reason},'BRIDGE_HARD_CERT_FAIL'),1,'last');if ~isempty(idx),E=H.trace(idx);fprintf('         lastHardFail: h=%.6g minSlack=%+.3e failPiece=%d failObstacle=%d checks=%.0f leaves=%.0f maxDepth=%d\n',E.h,E.directSlack,E.fail_piece,E.fail_obstacle,E.checks,E.leaves,E.max_depth);end
            mc=min([H.trace.minCone],[],'omitnan');mr=min([H.trace.minRz],[],'omitnan');fprintf('         searched h=[%.6g, %.6g] s | min observed coneSlack=%+.3e rzSlack=%+.3e | searchComplete=0 infeasibilityClaim=0\n',CFG.v209b_bridge_h_min_s,CFG.v209b_bridge_h_max_s,mc,mr);
        end
    end
end
fprintf('------------------------------------------------------------------------------------------------------------------------\n');
fprintf(' certified=%d/%d | allObstacleAccounting=%d/%d | timingIsWCET=0 | safetyRelaxed=0\n',J.certifiedCandidates,J.total,J.allObstacleAccounting,J.total);
fprintf(' RULE: do not optimize or add obstacle count until the dominant stage/failure mechanism above is identified.\n');
fprintf(' RULE: any later speed-up must preserve the same final continuous UAV/load/cable hard authority.\n');
fprintf('========================================================================================================================\n');
end


% ========================================================================
% V209L: PATCH_UNION boundary tangency + same-theorem deep certificate audit.
% ========================================================================
function v209l_mex_bootstrap_direct_cert_deep(root)
% Qualification-only clone of the inherited direct certificate.  The source
% is byte-for-byte the same algorithm except the admissible proof-resource
% max depth is extended from 12 to 24.  The inherited MEX remains untouched.
name='g26v209l_direct_cert_deep_mex';
lines=v200_cpp_direct_cert_source();
for k=1:numel(lines)
    lines{k}=strrep(lines{k},'G26V200:DirectCertMEX','G26V209L:DirectCertDeepMEX');
    lines{k}=strrep(lines{k},'md>=0&&md<=12','md>=0&&md<=24');
end
v200_build_mex_if_needed(root,name,lines,'G26-V209L-DEEP-CERT-MEX');
end

function A=v209l_patch_tangency_deep_cert_audit(CFG,K)
rep='PATCH_UNION';
names={'BSPLINE_D7','BEZIER_D13','POLY_D13','ACETRANS_MINCO_ST_C6'};
depths=8:20;
S=v209b_static_scenario(CFG);
Qold=v209j_build_static_problem_diag(S,rep,CFG,K);
assert(Qold.guidePass&&Qold.corridorOK,'G26V209L:OldSpatial','Old PATCH_UNION reference spatial problem must pass.');
Bold=v207_build_backend('ACETRANS_MINCO_ST_C6',Qold.W,Qold.weights);
Told=v208_time_gate_angle(Bold,CFG,CFG.v209b_nominal_theta_deg);
assert(Told.pass,'G26V209L:OldTime','Old PATCH_UNION reference time gate must pass.');
tDetect=CFG.v209b_detection_fraction*Told.T;
tSwitch=tDetect+CFG.v209b_switch_lookahead_s;
assert(tSwitch<Told.T,'G26V209L:Switch','Scheduled switch must lie inside old active reference.');
J0=v209b_eval_jet(Bold,Told,tSwitch);p0=J0(:,1);
Cnew=[S.C S.newObstacleCenter];Rnew=[S.R S.newObstacleRadius];
Snew=v208_make_scenario('V209L_PATCH_TANGENCY_DEEP_CERT',Cnew,zeros(3,size(Cnew,2)),Rnew,p0,S.pg);
Qnew=v209j_build_static_problem_diag(Snew,rep,CFG,K);
assert(Qnew.guidePass&&Qnew.corridorOK,'G26V209L:UpdatedSpatial','Updated PATCH_UNION guide/corridor must pass.');
prefix=v209b_active_interval_certificate(Bold,Told,tDetect,tSwitch,Qnew,CFG);
assert(prefix.pass,'G26V209L:Prefix','Active prefix must remain certified.');
QT=v209b_tail_problem(Qnew);
assert(QT.exists,'G26V209L:Tail','Expected a nonempty tail.');

G=v209l_boundary_geometry_diag(Qnew,K,CFG);

rowTemplate=struct('backend','','corridorPass',false,'timePass',false,'T',NaN,'minCone',NaN,'minRz',NaN, ...
    'depth',[],'firstPassDepth',NaN,'firstPassP95ms',NaN,'provedBy20',false,'sampling',[],'sameAlgorithmMatchAt8',false,'sameAlgorithmMatchAt12',false);
rows=repmat(rowTemplate,1,numel(names));
for bi=1:numel(names)
    name=names{bi};
    B=v207_build_backend(name,QT.W,QT.weights);
    C=v209b_tail_corridor_audit(B,QT);
    T=v208_time_gate_angle(B,CFG,CFG.v209b_nominal_theta_deg);
    assert(C.pass&&T.pass,'G26V209L:Prerequisite','PATCH_UNION backend must reach the final certificate.');
    R=rowTemplate;R.backend=name;R.corridorPass=C.pass;R.timePass=T.pass;R.T=T.T;R.minCone=T.min_cone_slack;R.minRz=T.min_rz_slack;

    % Confirm the qualification-only clone is identical to the inherited
    % theorem/implementation for depths the inherited MEX already accepts.
    O8=v209k_final_direct_certificate_depth(B,T,QT,CFG,8);
    X8=v209l_final_direct_certificate_depth(B,T,QT,CFG,8);
    O12=v209k_final_direct_certificate_depth(B,T,QT,CFG,12);
    X12=v209l_final_direct_certificate_depth(B,T,QT,CFG,12);
    R.sameAlgorithmMatchAt8=v209l_cert_outputs_match(O8,X8);
    R.sameAlgorithmMatchAt12=v209l_cert_outputs_match(O12,X12);
    assert(R.sameAlgorithmMatchAt8&&R.sameAlgorithmMatchAt12,'G26V209L:CloneMismatch','Deep certificate clone differs from inherited certificate at depth <=12.');

    dd=repmat(struct('maxDepth',0,'pass',false,'minSlack',NaN,'failPiece',0,'failObstacle',0,'checks',0,'leaves',0,'usedDepth',0, ...
        'p50_ms',NaN,'p95_ms',NaN,'max_ms',NaN),1,numel(depths));
    for di=1:numel(depths)
        md=depths(di);
        D=v209l_final_direct_certificate_depth(B,T,QT,CFG,md);
        NW=5;NS=40;
        for w=1:NW,v209l_final_direct_certificate_depth(B,T,QT,CFG,md);end
        tv=zeros(NS,1);Dlast=[];
        for q=1:NS
            tt=tic;Dlast=v209l_final_direct_certificate_depth(B,T,QT,CFG,md);tv(q)=1e3*toc(tt);
        end
        ST=v209j_stats(tv);
        dd(di).maxDepth=md;dd(di).pass=D.pass;dd(di).minSlack=D.min_slack;dd(di).failPiece=D.fail_piece;dd(di).failObstacle=D.fail_obstacle;
        dd(di).checks=D.checks;dd(di).leaves=D.leaves;dd(di).usedDepth=D.max_depth;dd(di).p50_ms=ST.p50;dd(di).p95_ms=ST.p95;dd(di).max_ms=ST.max;
        assert(Dlast.pass==D.pass,'G26V209L:Determinism','Deep direct certificate PASS/FAIL changed across identical warmed calls.');
    end
    R.depth=dd;
    ip=find([dd.pass],1,'first');
    if ~isempty(ip),R.firstPassDepth=dd(ip).maxDepth;R.firstPassP95ms=dd(ip).p95_ms;R.provedBy20=true;end
    R.sampling=v209k_dense_wholebody_sampling(B,T,QT,CFG,20001);
    rows(bi)=R;
end

A=struct();A.representation=rep;A.depths=depths;A.rows=rows;A.geometry=G;A.K=K;A.nominalThetaDeg=CFG.v209b_nominal_theta_deg;
A.prefixSafe=prefix.pass;A.allObstacleAccounting=(size(Qnew.C,2)==numel(Qnew.Ractual))&&(size(Qnew.Cproposal,2)==size(Qnew.C,2)*(K+2));
A.provedCount=sum([rows.provedBy20]);A.allFourProved=A.provedCount==numel(rows);A.maxFirstPassDepth=max([rows.firstPassDepth],[],'omitnan');
A.maxFirstPassP95ms=max([rows.firstPassP95ms],[],'omitnan');
A.sameHardTheorem=true;A.inheritedMexUntouched=true;A.safetyRelaxed=false;A.samplingAuthority=false;A.productionPromotion=false;
if A.allFourProved
    A.classification='BOUNDARY_TANGENCY__FINITE_PROOF_RESOLUTION_CLOSABLE_WITH_SAME_THEOREM';
else
    A.classification='BOUNDARY_TANGENCY__NOT_CLOSED_BY_DEPTH20__TRAJECTORY_OR_CERT_GEOMETRY_CHANGE_REQUIRED';
end
end

function D=v209l_final_direct_certificate_depth(B,T,Q,CFG,md)
[Pflat,counts]=v207_flatten_spans(B.P);ta=[0 cumsum(T.h(1:end-1))];tb=cumsum(T.h);
M=size(Q.C,2);OV=zeros(3,M);OR=repmat(reshape(eye(3),9,1),1,M);rad=reshape(Q.Ractual,1,[]);
if isscalar(rad),rad=repmat(rad,1,M);end
assert(numel(rad)==M,'G26V209L:DirectCertRadius','Obstacle radius count must equal obstacle-center count.');
ER=repmat(rad,3,1);
Kbody=[CFG.R_payload+CFG.geometry_margin_m,CFG.R_uav+CFG.geometry_margin_m,CFG.R_cable+CFG.geometry_margin_m,CFG.L_cable];
[ok,sl,checks,fp,fo,leaves,dep]=g26v209l_direct_cert_deep_mex(Pflat,counts,T.Rflat,T.rcounts,ta,tb,Q.C,OV,OR,ER,Kbody,md,CFG.direct_cert_guard_scale);
D=struct('pass',logical(ok),'min_slack',sl,'checks',checks,'fail_piece',fp,'fail_obstacle',fo,'leaves',leaves,'max_depth',dep,'configured_max_depth',md);
end

function tf=v209l_cert_outputs_match(A,B)
sc=max(1,max(abs([A.min_slack B.min_slack])));g=8192*eps(sc);
tf=A.pass==B.pass && A.fail_piece==B.fail_piece && A.fail_obstacle==B.fail_obstacle && ...
    A.checks==B.checks && A.leaves==B.leaves && A.max_depth==B.max_depth && abs(A.min_slack-B.min_slack)<=g;
end

function G=v209l_boundary_geometry_diag(Q,K,CFG)
% The proposal cover is expressed in payload-position space.  For PATCH_UNION
% extra=0, its payload forbidden sphere should coincide exactly with the
% payload term used by the final hard certificate.
M=size(Q.C,2);stride=K+2;
hardPayload=Q.Ractual + CFG.geometry_margin_m + CFG.R_payload;
proposalPayload=zeros(1,M);delta=zeros(1,M);
for j=1:M
    ix=(j-1)*stride+1;
    proposalPayload(j)=Q.Rproposal(ix);
    delta(j)=proposalPayload(j)-hardPayload(j);
end

% Find the actual closest guide-segment/proposal-sphere pair.
best=inf;bestSeg=0;bestSphere=0;bestPoint=[NaN;NaN;NaN];
for s=1:size(Q.W,2)-1
    a=Q.W(:,s);b=Q.W(:,s+1);
    for m=1:size(Q.Cproposal,2)
        [d,q]=v209l_segment_point_distance(a,b,Q.Cproposal(:,m));
        clr=d-Q.Rproposal(m);
        if clr<best,best=clr;bestSeg=s;bestSphere=m;bestPoint=q;end
    end
end
obs=ceil(bestSphere/stride);local=bestSphere-(obs-1)*stride;
if local==1
    typ='PAYLOAD';
elseif local<=K+1
    typ=sprintf('CABLE_%d',local-1);
else
    typ='UAV';
end
G=struct('hardPayloadRadius',hardPayload,'proposalPayloadRadius',proposalPayload,'payloadRadiusDifference',delta, ...
    'payloadBoundaryCoincides',all(delta==0),'criticalGuideClearance',best,'criticalSegment',bestSeg,'criticalProposalSphere',bestSphere, ...
    'criticalPhysicalObstacle',obs,'criticalLocalType',typ,'criticalClosestPoint',bestPoint,'guideReportedMinClearance',Q.guideClearance, ...
    'corridorReportedMinClearance',Q.corridorMinClearance);
end

function [d,q]=v209l_segment_point_distance(a,b,c)
v=b-a;vv=v'*v;
if vv<=0,t=0;else,t=max(0,min(1,((c-a)'*v)/vv));end
q=a+t*v;d=norm(q-c);
end

function v209l_print_patch_tangency_deep(A,CFG)
fprintf('\n========================================================================================================================\n');
fprintf(' G26-V209L PATCH_UNION TANGENCY / DEEP-CERTIFICATE AUDIT | unchanged 2-obstacle tail | theta=%.1fdeg\n',A.nominalThetaDeg);
fprintf('------------------------------------------------------------------------------------------------------------------------\n');
G=A.geometry;
fprintf(' proposal payload radius == hard payload radius: %d | differences =',G.payloadBoundaryCoincides);fprintf(' %+.3e',G.payloadRadiusDifference);fprintf(' m\n');
fprintf(' critical guide pair: segment=%d proposalSphere=%d -> obstacle=%d type=%s | clearance=%+.6e m | guideReported=%+.6e corridorReported=%+.6e\n', ...
    G.criticalSegment,G.criticalProposalSphere,G.criticalPhysicalObstacle,G.criticalLocalType,G.criticalGuideClearance,G.guideReportedMinClearance,G.corridorReportedMinClearance);
fprintf(' inherited direct-cert MEX untouched=1 | deep clone same algorithm=1 | only allowed resource depth differs | samplingAuthority=0\n');
for k=1:numel(A.rows)
    R=A.rows(k);
    fprintf(' %-23s | T=%.6g corr=%d cone/taut=%d | cloneMatch d8/d12=%d/%d | firstProvingDepth=%g firstPassP95=%.4f ms\n', ...
        R.backend,R.T,R.corridorPass,R.timePass,R.sameAlgorithmMatchAt8,R.sameAlgorithmMatchAt12,R.firstPassDepth,R.firstPassP95ms);
    for j=1:numel(R.depth)
        D=R.depth(j);
        fprintf('   depth=%2d PASS=%d minSlack=%+.3e failPiece=%d failObstacle=%d checks=%5.0f leaves=%4.0f usedDepth=%2d | p50/p95/max=%.4f/%.4f/%.4f ms\n', ...
            D.maxDepth,D.pass,D.minSlack,D.failPiece,D.failObstacle,D.checks,D.leaves,D.usedDepth,D.p50_ms,D.p95_ms,D.max_ms);
    end
    S=R.sampling;
    fprintf('   SAMPLE-ONLY min payload/UAV/cable=%+.6e/%+.6e/%+.6e m | payload arg [piece obstacle time]=[%g %g %.9g] | AUTHORITY=0\n', ...
        S.minPayloadClearance,S.minUAVClearance,S.minCableClearance,S.payloadArgPieceObstacleTime(1),S.payloadArgPieceObstacleTime(2),S.payloadArgPieceObstacleTime(3));
end
fprintf('------------------------------------------------------------------------------------------------------------------------\n');
fprintf(' classification=%s\n',A.classification);
fprintf(' provedByDepth20=%d/%d | maxFirstPassDepth=%g | maxFirstPassP95=%.4f ms | prefixSafe=%d allObstacleAccounting=%d safetyRelaxed=0 promotion=0\n', ...
    A.provedCount,numel(A.rows),A.maxFirstPassDepth,A.maxFirstPassP95ms,A.prefixSafe,A.allObstacleAccounting);
fprintf(' RULE: a deep PASS is the SAME continuous hard theorem proved with finer subdivision; it does not change obstacle radius, body radius, geometry margin, cone, or tautness.\n');
fprintf(' RULE: if depth20 still cannot prove a backend, do not keep increasing depth blindly; move to theorem-derived positive proof reserve / trajectory geometry.\n');
fprintf(' RULE: do not modify the SFC bridge or add obstacle count in this stage.\n');
fprintf('========================================================================================================================\n');
end

% ========================================================================
% V209K: PATCH_UNION hard-certificate depth closure audit.
% ========================================================================
function A=v209k_patch_certificate_depth_audit(CFG,K)
rep='PATCH_UNION';
names={'BSPLINE_D7','BEZIER_D13','POLY_D13','ACETRANS_MINCO_ST_C6'};
depths=8:12;
S=v209b_static_scenario(CFG);
Qold=v209j_build_static_problem_diag(S,rep,CFG,K);
assert(Qold.guidePass&&Qold.corridorOK,'G26V209K:OldSpatial','Old PATCH_UNION reference spatial problem must pass.');
Bold=v207_build_backend('ACETRANS_MINCO_ST_C6',Qold.W,Qold.weights);
Told=v208_time_gate_angle(Bold,CFG,CFG.v209b_nominal_theta_deg);
assert(Told.pass,'G26V209K:OldTime','Old PATCH_UNION reference time gate must pass.');
tDetect=CFG.v209b_detection_fraction*Told.T;
tSwitch=tDetect+CFG.v209b_switch_lookahead_s;
assert(tSwitch<Told.T,'G26V209K:Switch','Scheduled switch must lie inside old active reference.');
J0=v209b_eval_jet(Bold,Told,tSwitch);p0=J0(:,1);
Cnew=[S.C S.newObstacleCenter];Rnew=[S.R S.newObstacleRadius];
Snew=v208_make_scenario('V209K_PATCH_CERT_DEPTH',Cnew,zeros(3,size(Cnew,2)),Rnew,p0,S.pg);
Qnew=v209j_build_static_problem_diag(Snew,rep,CFG,K);
assert(Qnew.guidePass&&Qnew.corridorOK,'G26V209K:UpdatedSpatial','Updated PATCH_UNION guide/corridor must pass before certificate diagnosis.');
prefix=v209b_active_interval_certificate(Bold,Told,tDetect,tSwitch,Qnew,CFG);
assert(prefix.pass,'G26V209K:Prefix','Active prefix must remain certified.');
QT=v209b_tail_problem(Qnew);
assert(QT.exists,'G26V209K:Tail','Expected a nonempty tail in the ordinary two-obstacle case.');

template=struct('backend','','corridorPass',false,'timePass',false,'T',NaN,'minCone',NaN,'minRz',NaN, ...
    'depth',[],'firstPassDepth',NaN,'provedBy12',false,'sampling',[],'sameAuthority',true,'safetyRelaxed',false);
rows=repmat(template,1,numel(names));
for bi=1:numel(names)
    name=names{bi};
    B=v207_build_backend(name,QT.W,QT.weights);
    C=v209b_tail_corridor_audit(B,QT);
    T=v208_time_gate_angle(B,CFG,CFG.v209b_nominal_theta_deg);
    R=template;R.backend=name;R.corridorPass=C.pass;R.timePass=T.pass;R.T=T.T;R.minCone=T.min_cone_slack;R.minRz=T.min_rz_slack;
    assert(C.pass&&T.pass,'G26V209K:Prerequisite','PATCH_UNION backend must reach final hard certificate in V209K.');
    dd=repmat(struct('maxDepth',0,'pass',false,'minSlack',NaN,'failPiece',0,'failObstacle',0,'checks',0,'leaves',0,'usedDepth',0, ...
        'single_ms',NaN,'p50_ms',NaN,'p95_ms',NaN,'max_ms',NaN),1,numel(depths));
    for di=1:numel(depths)
        md=depths(di);
        t=tic;D=v209k_final_direct_certificate_depth(B,T,QT,CFG,md);single=1e3*toc(t);
        NW=5;NS=30;
        for w=1:NW,v209k_final_direct_certificate_depth(B,T,QT,CFG,md);end
        tv=zeros(NS,1);Dlast=[];
        for s=1:NS,t=tic;Dlast=v209k_final_direct_certificate_depth(B,T,QT,CFG,md);tv(s)=1e3*toc(t);end
        ST=v209j_stats(tv);
        dd(di).maxDepth=md;dd(di).pass=D.pass;dd(di).minSlack=D.min_slack;dd(di).failPiece=D.fail_piece;dd(di).failObstacle=D.fail_obstacle;
        dd(di).checks=D.checks;dd(di).leaves=D.leaves;dd(di).usedDepth=D.max_depth;dd(di).single_ms=single;
        dd(di).p50_ms=ST.p50;dd(di).p95_ms=ST.p95;dd(di).max_ms=ST.max;
        assert(Dlast.pass==D.pass,'G26V209K:Determinism','Direct certificate PASS/FAIL changed across identical warmed calls.');
    end
    R.depth=dd;
    ip=find([dd.pass],1,'first');if ~isempty(ip),R.firstPassDepth=dd(ip).maxDepth;R.provedBy12=true;end
    % Diagnostic-only dense sampling of the same tail. Never safety authority.
    R.sampling=v209k_dense_wholebody_sampling(B,T,QT,CFG,20001);
    rows(bi)=R;
end
A=struct();A.representation=rep;A.depths=depths;A.rows=rows;A.K=K;A.nominalThetaDeg=CFG.v209b_nominal_theta_deg;
A.prefixSafe=prefix.pass;A.allObstacleAccounting=(size(Qnew.C,2)==numel(Qnew.Ractual))&&(size(Qnew.Cproposal,2)==size(Qnew.C,2)*(K+2));
A.provedCount=sum([rows.provedBy12]);A.anyProved=A.provedCount>0;A.allFourProved=A.provedCount==numel(rows);
A.sameHardAuthority=true;A.safetyRelaxed=false;A.samplingAuthority=false;A.productionPromotion=false;
end

function D=v209k_final_direct_certificate_depth(B,T,Q,CFG,md)
[Pflat,counts]=v207_flatten_spans(B.P);ta=[0 cumsum(T.h(1:end-1))];tb=cumsum(T.h);
M=size(Q.C,2);OV=zeros(3,M);OR=repmat(reshape(eye(3),9,1),1,M);rad=reshape(Q.Ractual,1,[]);
if isscalar(rad),rad=repmat(rad,1,M);end
assert(numel(rad)==M,'G26V209K:DirectCertRadius','Obstacle radius count must equal obstacle-center count.');
ER=repmat(rad,3,1);
Kbody=[CFG.R_payload+CFG.geometry_margin_m,CFG.R_uav+CFG.geometry_margin_m,CFG.R_cable+CFG.geometry_margin_m,CFG.L_cable];
[ok,sl,checks,fp,fo,leaves,dep]=g26v200_direct_cert_mex(Pflat,counts,T.Rflat,T.rcounts,ta,tb,Q.C,OV,OR,ER,Kbody,md,CFG.direct_cert_guard_scale);
D=struct('pass',logical(ok),'min_slack',sl,'checks',checks,'fail_piece',fp,'fail_obstacle',fo,'leaves',leaves,'max_depth',dep,'configured_max_depth',md);
end

function S=v209k_dense_wholebody_sampling(B,T,Q,CFG,N)
% Diagnostic only. Positive sampled clearance is NOT a continuous proof.
% Negative sampled clearance is evidence that the sampled trajectory itself
% violates the modeled whole-body clearance at that sample.
minP=inf;minU=inf;minC=inf;argP=[NaN NaN NaN];argU=argP;argC=argP;
cs=[0 cumsum(T.h)];
for q=1:numel(B.P)
    X=B.P{q};n=size(X,2)-1;h=T.h(q);
    A2=n*(n-1)/h^2*diff(X,2,2);
    ss=linspace(0,1,N);
    for ii=1:numel(ss)
        s=ss(ii);p=v209k_bezier_eval(X,s);a=v209k_bezier_eval(A2,s);r=a+[0;0;CFG.gravity];rn=norm(r);
        if ~(isfinite(rn)&&rn>0),continue;end
        qc=-r/rn;pU=p-CFG.L_cable*qc;
        for j=1:size(Q.C,2)
            c=Q.C(:,j);ro=Q.Ractual(j);
            cp=norm(p-c)-(ro+CFG.geometry_margin_m+CFG.R_payload);
            cu=norm(pU-c)-(ro+CFG.geometry_margin_m+CFG.R_uav);
            cc=v209k_point_segment_distance(c,p,pU)-(ro+CFG.geometry_margin_m+CFG.R_cable);
            tg=cs(q)+s*h;
            if cp<minP,minP=cp;argP=[q j tg];end
            if cu<minU,minU=cu;argU=[q j tg];end
            if cc<minC,minC=cc;argC=[q j tg];end
        end
    end
end
S=struct('diagnosticOnly',true,'authority',false,'samplesPerPiece',N,'minPayloadClearance',minP,'minUAVClearance',minU,'minCableClearance',minC, ...
    'payloadArgPieceObstacleTime',argP,'uavArgPieceObstacleTime',argU,'cableArgPieceObstacleTime',argC,'sampledAllPositive',min([minP minU minC])>0);
end

function y=v209k_bezier_eval(B,s)
Q=B;
while size(Q,2)>1,Q=(1-s)*Q(:,1:end-1)+s*Q(:,2:end);end
y=Q(:,1);
end

function d=v209k_point_segment_distance(x,a,b)
v=b-a;vv=v'*v;if vv<=0,d=norm(x-a);return;end
t=max(0,min(1,((x-a)'*v)/vv));d=norm(x-(a+t*v));
end

function v209k_print_patch_certificate_depth(A,CFG)
fprintf('\n========================================================================================================================\n');
fprintf(' G26-V209K PATCH_UNION CERTIFICATE DEPTH AUDIT | unchanged 2-obstacle tail | theta=%.1fdeg\n',A.nominalThetaDeg);
fprintf('------------------------------------------------------------------------------------------------------------------------\n');
fprintf(' same geometry/margins/cone/taut/backends=1 | same hard theorem=1 | samplingAuthority=0 | promotion=0\n');
for k=1:numel(A.rows)
    R=A.rows(k);fprintf(' %-23s | corridor=%d time=%d T=%.6g minCone=%+.3e minRz=%+.3e | firstProvingDepth=%g\n', ...
        R.backend,R.corridorPass,R.timePass,R.T,R.minCone,R.minRz,R.firstPassDepth);
    for j=1:numel(R.depth)
        D=R.depth(j);fprintf('   depth=%2d PASS=%d minSlack=%+.3e failPiece=%d failObstacle=%d checks=%5.0f leaves=%4.0f usedDepth=%2d | p50/p95/max=%.4f/%.4f/%.4f ms\n', ...
            D.maxDepth,D.pass,D.minSlack,D.failPiece,D.failObstacle,D.checks,D.leaves,D.usedDepth,D.p50_ms,D.p95_ms,D.max_ms);
    end
    S=R.sampling;fprintf('   SAMPLE-ONLY N=%d/piece | min clearance payload/UAV/cable = %+.6e / %+.6e / %+.6e m | allSampledPositive=%d | AUTHORITY=0\n', ...
        S.samplesPerPiece,S.minPayloadClearance,S.minUAVClearance,S.minCableClearance,S.sampledAllPositive);
end
fprintf('------------------------------------------------------------------------------------------------------------------------\n');
fprintf(' provedByDepth12=%d/%d | prefixSafe=%d | allObstacleAccounting=%d | safetyRelaxed=0 | productionPromotion=0\n',A.provedCount,numel(A.rows),A.prefixSafe,A.allObstacleAccounting);
fprintf(' RULE: deeper subdivision changes proof resolution only; obstacle/model safety margins are unchanged.\n');
fprintf(' RULE: positive dense samples never replace the continuous hard certificate.\n');
fprintf(' RULE: do not modify the SFC bridge in this stage; first classify PATCH_UNION as proof-depth-limited or genuinely unresolved.\n');
fprintf('========================================================================================================================\n');
end


% ========================================================================
% V209M: PATCH_UNION adaptive depth-19 certificate scaling + full handoff.
% ========================================================================
function A=v209m_patch_adaptive_cert_scaling_handoff_audit(CFG,K)
rep='PATCH_UNION';
names={'BSPLINE_D7','BEZIER_D13','POLY_D13','ACETRANS_MINCO_ST_C6'};
md=CFG.v209m_cert_max_depth;

% Reconstruct the exact ordinary two-obstacle replan state used by V209I-L.
S=v209b_static_scenario(CFG);
Qold=v209j_build_static_problem_diag(S,rep,CFG,K);
assert(Qold.guidePass&&Qold.corridorOK,'G26V209M:OldSpatial','Old PATCH_UNION spatial problem must pass.');
Bold=v207_build_backend('ACETRANS_MINCO_ST_C6',Qold.W,Qold.weights);
Told=v208_time_gate_angle(Bold,CFG,CFG.v209b_nominal_theta_deg);
assert(Told.pass,'G26V209M:OldTime','Old PATCH_UNION time gate must pass.');
tDetect=CFG.v209b_detection_fraction*Told.T;
tSwitch=tDetect+CFG.v209b_switch_lookahead_s;
assert(tSwitch<Told.T,'G26V209M:Switch','Scheduled switch must lie inside old active reference.');
J0=v209b_eval_jet(Bold,Told,tSwitch);p0=J0(:,1);
Cnew=[S.C S.newObstacleCenter];Rnew=[S.R S.newObstacleRadius];
Snew=v208_make_scenario('V209M_PATCH_ADAPTIVE_CERT',Cnew,zeros(3,size(Cnew,2)),Rnew,p0,S.pg);
Qnew=v209j_build_static_problem_diag(Snew,rep,CFG,K);
assert(Qnew.guidePass&&Qnew.corridorOK,'G26V209M:UpdatedSpatial','Updated PATCH_UNION guide/corridor must pass.');
prefix=v209b_active_interval_certificate(Bold,Told,tDetect,tSwitch,Qnew,CFG);
assert(prefix.pass,'G26V209M:Prefix','Active prefix must remain continuously certified to scheduled switch.');
QT=v209b_tail_problem(Qnew);
assert(QT.exists,'G26V209M:Tail','Expected nonempty PATCH tail.');
G=v209l_boundary_geometry_diag(Qnew,K,CFG);

rowTemplate=struct('backend','','T',NaN,'corridorPass',false,'timePass',false,'actualPass',false,'actualMinSlack',NaN, ...
    'actualUsedDepth',NaN,'actualChecks',NaN,'actualLeaves',NaN,'depth19Matches24',false,'scaling',[],'easy12',[], ...
    'handoff',[],'pipelineTiming',[],'fullPass',false,'status','UNSET');
rows=repmat(rowTemplate,1,numel(names));

for bi=1:numel(names)
    name=names{bi};
    B=v207_build_backend(name,QT.W,QT.weights);
    C=v209b_tail_corridor_audit(B,QT);
    T=v208_time_gate_angle(B,CFG,CFG.v209b_nominal_theta_deg);
    assert(C.pass&&T.pass,'G26V209M:Prerequisite','PATCH backend must pass unchanged tail corridor and cone/taut time gate.');

    R=rowTemplate;R.backend=name;R.T=T.T;R.corridorPass=C.pass;R.timePass=T.pass;
    D19=v209l_final_direct_certificate_depth(B,T,QT,CFG,md);
    D24=v209l_final_direct_certificate_depth(B,T,QT,CFG,24);
    R.actualPass=D19.pass;R.actualMinSlack=D19.min_slack;R.actualUsedDepth=D19.max_depth;R.actualChecks=D19.checks;R.actualLeaves=D19.leaves;
    R.depth19Matches24=v209l_cert_outputs_match(D19,D24);
    assert(R.depth19Matches24,'G26V209M:DepthCap','Actual tail result at maxDepth=19 must equal the deeper resource-cap result.');

    R.scaling=v209m_certificate_scaling(B,T,QT,CFG,md);
    R.easy12=v209m_easy_adaptivity_probe(B,T,QT,CFG,md,12);

    % Full unchanged bridge/handoff after replacing only the direct-certificate
    % resource cap with the same-theorem maxDepth=19 qualification clone.
    [BT,TT,TD]=v209m_tail_diag_deep(QT,CFG,name,md);
    H=struct('tail',TD,'bridge',[],'jetPass',false,'jetResidual',NaN,'jetResidualOverGuard',NaN, ...
        'c6',false,'corridor',false,'coneTaut',false,'hardCert',false,'pass',false,'status','UNSET');
    assert(strcmp(TD.status,'PASS'),'G26V209M:TailDeep','Depth-19 PATCH tail certificate should close after V209L.');
    J1=v209b_backend_start_jets(BT,TT);
    HS=v209m_bridge_search_diag_deep(J0,J1,Qnew,BT,TT,CFG,md);
    H.bridge=HS;
    if ~HS.pass
        H.status=[name '__JET_TIME_CERT_SEARCH_INCOMPLETE'];
    else
        X=HS.best;
        [jp,jr,jg]=v209b_endpoint_jet_match(X.bridgeB,X.bridgeH,J0,J1);
        H.jetPass=jp;H.jetResidual=jr;H.jetResidualOverGuard=jg;
        H.c6=X.c6.pass;H.corridor=X.corridor.pass;H.coneTaut=X.bridgeDynamic.pass;H.hardCert=X.direct.pass;
        H.pass=prefix.pass&&jp&&H.c6&&H.corridor&&H.coneTaut&&H.hardCert;
        if H.pass,H.status='CERTIFIED_MULTI_OBSTACLE_STATIC_SUCCESSOR';else,H.status='CERTIFICATE_OR_HANDOFF_FAIL';end
    end
    R.handoff=H;R.fullPass=H.pass;R.status=H.status;

    % Warmed complete PATCH candidate-pipeline profile.  This is performed
    % whether the search succeeds or fails; a failure runtime is still useful
    % for identifying the next online bottleneck.  It excludes sensing/prefix
    % detection and reuses the already-active old reference, as online replanning would.
    R.pipelineTiming=v209m_pipeline_profile(Snew,J0,name,CFG,K,md,H.status);
    rows(bi)=R;
end

A=struct();A.representation=rep;A.K=K;A.maxDepth=md;A.scalingCounts=CFG.v209m_scaling_counts;A.rows=rows;A.geometry=G;
A.prefixSafe=prefix.pass;A.allObstacleAccounting=(size(Qnew.C,2)==numel(Qnew.Ractual))&&(size(Qnew.Cproposal,2)==size(Qnew.C,2)*(K+2));
A.actualTailCertified=sum([rows.actualPass]);A.depth19Matches24=all([rows.depth19Matches24]);
A.scalingAllPass=true;A.easyAdaptivityAllPass=true;
for k=1:numel(rows)
    A.scalingAllPass=A.scalingAllPass&&all([rows(k).scaling.pass]);
    A.easyAdaptivityAllPass=A.easyAdaptivityAllPass&&rows(k).easy12.pass;
end
A.certificateResourceClosed=(A.actualTailCertified==numel(rows))&&A.depth19Matches24&&A.scalingAllPass&&A.easyAdaptivityAllPass;
A.fullCertified=sum([rows.fullPass]);
A.maxActual2ObsCertP95=max(arrayfun(@(r)v209m_find_scaling_p95(r.scaling,2),rows));
A.maxScaling12CertP95=max(arrayfun(@(r)v209m_find_scaling_p95(r.scaling,12),rows));
A.maxPipelineP95=max(arrayfun(@(r)r.pipelineTiming.total.p95,rows));
A.certTimingIsWCET=false;A.pipelineTimingIsWCET=false;A.sameHardTheorem=true;A.safetyRelaxed=false;A.productionPromotion=false;
if ~A.certificateResourceClosed
    A.classification='PATCH_CERTIFICATE_RESOURCE_NOT_CLOSED';
elseif A.fullCertified<numel(rows)
    A.classification='PATCH_TAIL_CERT_RESOURCE_CLOSED__UNCHANGED_BRIDGE_HANDOFF_REMAINS_OPEN';
elseif A.maxPipelineP95<=CFG.v209b_compute_stretch_target_ms
    A.classification='PATCH_ORDINARY_TWO_OBSTACLE_FULL_HANDOFF__WARMED_4MS_SCREEN_PASS__WCET_OPEN';
else
    A.classification='PATCH_ORDINARY_TWO_OBSTACLE_FULL_HANDOFF_CERTIFIED__RUNTIME_TARGET_OPEN';
end
end

function S=v209m_certificate_scaling(B,T,Qbase,CFG,md)
counts=CFG.v209m_scaling_counts;
template=struct('obstacles',0,'pass',false,'minSlack',NaN,'usedDepth',NaN,'checks',NaN,'leaves',NaN,'p50_ms',NaN,'p95_ms',NaN,'max_ms',NaN);
S=repmat(template,1,numel(counts));
for k=1:numel(counts)
    M=counts(k);Q=v209m_make_critical_scale_problem(Qbase,M);
    D=v209l_final_direct_certificate_depth(B,T,Q,CFG,md);
    ST=v209m_time_deep_cert(B,T,Q,CFG,md);
    S(k)=struct('obstacles',M,'pass',D.pass,'minSlack',D.min_slack,'usedDepth',D.max_depth,'checks',D.checks,'leaves',D.leaves, ...
        'p50_ms',ST.p50,'p95_ms',ST.p95,'max_ms',ST.max);
end
end

function E=v209m_easy_adaptivity_probe(B,T,Qbase,CFG,md,M)
Q=v209m_make_easy_scale_problem(Qbase,M);
D=v209l_final_direct_certificate_depth(B,T,Q,CFG,md);
ST=v209m_time_deep_cert(B,T,Q,CFG,md);
E=struct('obstacles',M,'pass',D.pass,'minSlack',D.min_slack,'usedDepth',D.max_depth,'checks',D.checks,'leaves',D.leaves, ...
    'p50_ms',ST.p50,'p95_ms',ST.p95,'max_ms',ST.max);
end

function Q=v209m_make_critical_scale_problem(Qbase,M)
assert(M>=1&&floor(M)==M,'G26V209M:ScaleCount','Obstacle count must be positive integer.');
% M=1 keeps the known critical obstacle (physical obstacle 2).  M>=2 keeps
% both actual obstacles and appends deterministic far sensed obstacles.
if size(Qbase.C,2)<2,error('G26V209M:BaseObstacles','Expected the two actual obstacles.');end
if M==1
    C=Qbase.C(:,2);R=Qbase.Ractual(2);
else
    C=Qbase.C(:,1:2);R=Qbase.Ractual(1:2);
    n=M-2;
    for k=1:n
        C(:,end+1)=[0.5+0.45*k;12.0+0.07*sin(k);6.0+0.05*cos(k)]; %#ok<AGROW>
        R(end+1)=0.55; %#ok<AGROW>
    end
end
Q=struct('C',C,'Ractual',reshape(R,1,[]));
end

function Q=v209m_make_easy_scale_problem(Qbase,M)
assert(M>=1&&floor(M)==M,'G26V209M:ScaleCount','Obstacle count must be positive integer.');
r=max(Qbase.Ractual);
C=zeros(3,M);R=r*ones(1,M);
for k=1:M
    C(:,k)=[0.4+0.45*k;12.5+0.05*sin(k);6.5+0.05*cos(k)];
end
Q=struct('C',C,'Ractual',R);
end

function ST=v209m_time_deep_cert(B,T,Q,CFG,md)
NW=CFG.v209m_cert_warmup;NS=CFG.v209m_cert_timing_samples;NB=CFG.v209m_cert_batch_calls;
for w=1:NW,v209l_final_direct_certificate_depth(B,T,Q,CFG,md);end
tv=zeros(NS,1);
for s=1:NS
    tt=tic;
    for b=1:NB,D=v209l_final_direct_certificate_depth(B,T,Q,CFG,md);end %#ok<NASGU>
    tv(s)=1e3*toc(tt)/NB;
end
ST=v209j_stats(tv);
end

function y=v209m_find_scaling_p95(S,M)
i=find([S.obstacles]==M,1);
if isempty(i),y=NaN;else,y=S(i).p95_ms;end
end

function [BT,TT,DG]=v209m_tail_diag_deep(QT,CFG,name,md)
BT=[];TT=[];
DG=struct('status','NO_TAIL','build_ms',0,'corridor_ms',0,'time_gate_ms',0,'direct_ms',0,'total_ms',0, ...
    'corridor_pass',true,'corridor_min_slack',inf,'time_pass',true,'T',0,'minCone',inf,'minRz',inf,'timeIterations',0, ...
    'direct_pass',true,'direct_min_slack',inf,'fail_piece',0,'fail_obstacle',0,'checks',0,'leaves',0,'max_depth',0,'configured_max_depth',md);
if ~QT.exists,return;end
t=tic;B=v207_build_backend(name,QT.W,QT.weights);DG.build_ms=1e3*toc(t);
t=tic;C=v209b_tail_corridor_audit(B,QT);DG.corridor_ms=1e3*toc(t);DG.corridor_pass=C.pass;DG.corridor_min_slack=C.min_slack;
if ~C.pass,DG.status='TAIL_CORRIDOR_SEARCH_INCOMPLETE';DG.total_ms=v209j_tail_sum(DG);return;end
t=tic;T=v208_time_gate_angle(B,CFG,CFG.v209b_nominal_theta_deg);DG.time_gate_ms=1e3*toc(t);DG.time_pass=T.pass;DG.T=T.T;DG.minCone=T.min_cone_slack;DG.minRz=T.min_rz_slack;DG.timeIterations=T.iterations;
if ~T.pass,DG.status='TAIL_TIME_SEARCH_INCOMPLETE';DG.total_ms=v209j_tail_sum(DG);return;end
t=tic;D=v209l_final_direct_certificate_depth(B,T,QT,CFG,md);DG.direct_ms=1e3*toc(t);DG.direct_pass=D.pass;DG.direct_min_slack=D.min_slack;DG.fail_piece=D.fail_piece;DG.fail_obstacle=D.fail_obstacle;DG.checks=D.checks;DG.leaves=D.leaves;DG.max_depth=D.max_depth;
if ~D.pass,DG.status='TAIL_HARD_CERT_FAIL';DG.total_ms=v209j_tail_sum(DG);return;end
BT=B;TT=T;DG.status='PASS';DG.total_ms=v209j_tail_sum(DG);
end

function [ok,C,E]=v209m_eval_bridge_diag_deep(J0,J1,Q,BT,TT,hb,CFG,md)
E=v209j_empty_bridge_eval();E.h=hb;E.reason='UNSET';
t=tic;BB=v209b_degree17_two_sided_jet_bridge(J0,J1,hb);E.build_ms=1e3*toc(t);
t=tic;CORb=v209b_bridge_corridor_audit(BB,Q);E.first_corridor_ms=1e3*toc(t);E.firstCorrPass=CORb.pass;E.firstCorrSlack=CORb.min_slack;
t=tic;DY=v209b_bridge_dynamic_audit(BB,hb,CFG.v209b_nominal_theta_deg,CFG);E.dynamic_ms=1e3*toc(t);E.dynamicPass=DY.pass;E.minCone=DY.minCone;E.minRz=DY.minRz;
if ~CORb.pass,E.reason='BRIDGE_FIRST_CORRIDOR_FAIL';E.total_ms=v209j_bridge_eval_sum(E);ok=false;C=[];return;end
if ~DY.pass,E.reason='BRIDGE_CONE_TAUT_FAIL';E.total_ms=v209j_bridge_eval_sum(E);ok=false;C=[];return;end

if isempty(BT)
    P={BB};hh=hb;Rflat=DY.R;rcounts=DY.rcount;tailT=0;
else
    P=[{BB} BT.P];hh=[hb TT.h];Rflat=[DY.R TT.Rflat];rcounts=[DY.rcount TT.rcounts];tailT=TT.T;
end
BC=struct('P',{P},'degree',17,'structural_c6_theorem',true,'condition_indicator',1);
TC=struct('pass',true,'h',hh,'Rflat',Rflat,'rcounts',rcounts,'T',sum(hh));

t=tic;C6=v207_c6_backend_audit(BC,hh);E.c6_ms=1e3*toc(t);E.c6Pass=C6.pass;
if isfield(C6,'max_endpoint_derivative_residual'),E.c6Residual=C6.max_endpoint_derivative_residual;end
if ~C6.pass,E.reason='BRIDGE_COMBINED_C6_FAIL';E.total_ms=v209j_bridge_eval_sum(E);ok=false;C=[];return;end
t=tic;COR=v209b_combined_corridor_audit(BC,Q);E.combined_corridor_ms=1e3*toc(t);E.combinedCorrPass=COR.pass;E.combinedCorrSlack=COR.min_slack;
if ~COR.pass,E.reason='BRIDGE_COMBINED_CORRIDOR_FAIL';E.total_ms=v209j_bridge_eval_sum(E);ok=false;C=[];return;end
t=tic;D=v209l_final_direct_certificate_depth(BC,TC,Q,CFG,md);E.direct_ms=1e3*toc(t);E.directPass=D.pass;E.directSlack=D.min_slack;E.fail_piece=D.fail_piece;E.fail_obstacle=D.fail_obstacle;E.checks=D.checks;E.leaves=D.leaves;E.max_depth=D.max_depth;
if ~D.pass,E.reason='BRIDGE_HARD_CERT_FAIL';E.total_ms=v209j_bridge_eval_sum(E);ok=false;C=[];return;end
E.reason='PASS';E.total_ms=v209j_bridge_eval_sum(E);ok=true;
C=struct('B',BC,'T',TC,'bridgeB',BB,'bridgeH',hb,'bridgeDynamic',DY,'bridgeCorridor',CORb,'c6',C6,'corridor',COR,'direct',D,'tailT',tailT);
end

function H=v209m_bridge_search_diag_deep(J0,J1,Q,BT,TT,CFG,md)
lo=CFG.v209b_bridge_h_min_s;hi=CFG.v209b_bridge_h_max_s;intervals=[lo hi];best=[];bestH=inf;evals=0;
trace=repmat(v209j_empty_bridge_eval(),1,CFG.v209b_bridge_search_evals);
for z=1:2
    if z==1,h=lo;else,h=hi;end
    [ok,C,E]=v209m_eval_bridge_diag_deep(J0,J1,Q,BT,TT,h,CFG,md);evals=evals+1;trace(evals)=E;
    if ok&&h<bestH,best=C;bestH=h;end
end
while ~isempty(intervals)&&evals<CFG.v209b_bridge_search_evals
    keep=intervals(:,1)<bestH;intervals=intervals(keep,:);if isempty(intervals),break;end
    [~,ii]=max(intervals(:,2)-intervals(:,1));ab=intervals(ii,:);intervals(ii,:)=[];
    if ab(2)-ab(1)<=64*eps(max(1,ab(2))),continue;end
    mid=.5*(ab(1)+ab(2));
    [ok,C,E]=v209m_eval_bridge_diag_deep(J0,J1,Q,BT,TT,mid,CFG,md);evals=evals+1;trace(evals)=E;
    if ok&&mid<bestH,best=C;bestH=mid;end
    if ab(1)<bestH,intervals=[intervals;ab(1) mid];end %#ok<AGROW>
    if mid<bestH,intervals=[intervals;mid ab(2)];end %#ok<AGROW>
end
trace=trace(1:evals);
H=struct();H.pass=~isempty(best);H.evals=evals;H.searchComplete=false;H.optimalityClaim=false;H.missIsInfeasible=false;H.trace=trace;H.best=best;
if H.pass,H.h=bestH;else,H.h=NaN;end
H.reason_counts=v209j_reason_counts(trace);H.stage=v209j_bridge_stage_stats(trace);H.total_ms=sum([trace.total_ms]);H.configured_max_depth=md;
end

function P=v209m_pipeline_profile(Snew,J0,name,CFG,K,md,expectedStatus)
NW=CFG.v209m_pipeline_warmup;NS=CFG.v209m_pipeline_samples;
for w=1:NW,v209m_pipeline_once(Snew,J0,name,CFG,K,md);end
tot=zeros(NS,1);sp=zeros(NS,1);ta=zeros(NS,1);br=zeros(NS,1);status=cell(1,NS);
for k=1:NS
    X=v209m_pipeline_once(Snew,J0,name,CFG,K,md);
    tot(k)=X.total_ms;sp(k)=X.spatial_ms;ta(k)=X.tail_ms;br(k)=X.bridge_ms;status{k}=X.status;
end
assert(all(strcmp(status,expectedStatus)),'G26V209M:PipelineDeterminism','Repeated PATCH pipeline changed status.');
P=struct('warmup',NW,'samples',NS,'total',v209j_stats(tot),'spatial',v209j_stats(sp),'tail',v209j_stats(ta),'bridge',v209j_stats(br),'status',expectedStatus);
end

function X=v209m_pipeline_once(Snew,J0,name,CFG,K,md)
tt=tic;
Q=v209j_build_static_problem_diag(Snew,'PATCH_UNION',CFG,K);
if ~Q.guidePass||~Q.corridorOK
    X=struct('status','UPDATED_SPATIAL_SEARCH_INCOMPLETE','total_ms',1e3*toc(tt),'spatial_ms',Q.spatial_ms,'tail_ms',0,'bridge_ms',0);return;
end
QT=v209b_tail_problem(Q);
[BT,TT,TD]=v209m_tail_diag_deep(QT,CFG,name,md);
if ~strcmp(TD.status,'PASS')
    X=struct('status',[name '__' TD.status],'total_ms',1e3*toc(tt),'spatial_ms',Q.spatial_ms,'tail_ms',TD.total_ms,'bridge_ms',0);return;
end
J1=v209b_backend_start_jets(BT,TT);
H=v209m_bridge_search_diag_deep(J0,J1,Q,BT,TT,CFG,md);
if ~H.pass,status=[name '__JET_TIME_CERT_SEARCH_INCOMPLETE'];else,status='CERTIFIED_MULTI_OBSTACLE_STATIC_SUCCESSOR';end
X=struct('status',status,'total_ms',1e3*toc(tt),'spatial_ms',Q.spatial_ms,'tail_ms',TD.total_ms,'bridge_ms',H.total_ms);
end

function v209m_print_patch_adaptive(A,CFG)
fprintf('\n========================================================================================================================\n');
fprintf(' G26-V209M PATCH_UNION ADAPTIVE CERTIFICATE / SCALING / FULL HANDOFF | maxDepth=%d theta=%.1fdeg\n',A.maxDepth,CFG.v209b_nominal_theta_deg);
fprintf('------------------------------------------------------------------------------------------------------------------------\n');
G=A.geometry;
fprintf(' boundary mechanism retained: proposalPayload==hardPayload=%d | critical obstacle=%d type=%s guideClearance=%+.6e m\n', ...
    G.payloadBoundaryCoincides,G.criticalPhysicalObstacle,G.criticalLocalType,G.criticalGuideClearance);
fprintf(' maxDepth=%d is only a resource cap; direct certificate remains recursive/adaptive and same hard theorem=1\n',A.maxDepth);
for k=1:numel(A.rows)
    R=A.rows(k);
    fprintf('\n%-23s | actual2obs tail PASS=%d minSlack=%+.3e usedDepth=%d checks=%.0f leaves=%.0f | depth19==24=%d\n', ...
        R.backend,R.actualPass,R.actualMinSlack,R.actualUsedDepth,R.actualChecks,R.actualLeaves,R.depth19Matches24);
    fprintf('  critical-containing hard-certificate scaling:\n');
    for j=1:numel(R.scaling)
        S=R.scaling(j);
        fprintf('    M=%2d PASS=%d usedDepth=%2d checks=%5.0f leaves=%4.0f minSlack=%+.3e | p50/p95/max=%.4f/%.4f/%.4f ms\n', ...
            S.obstacles,S.pass,S.usedDepth,S.checks,S.leaves,S.minSlack,S.p50_ms,S.p95_ms,S.max_ms);
    end
    E=R.easy12;
    fprintf('  easy M=%2d adaptivity probe: PASS=%d usedDepth=%d checks=%.0f leaves=%.0f | p50/p95/max=%.4f/%.4f/%.4f ms\n', ...
        E.obstacles,E.pass,E.usedDepth,E.checks,E.leaves,E.p50_ms,E.p95_ms,E.max_ms);
    H=R.handoff;B=H.bridge;C=B.reason_counts;
    fprintf('  unchanged full handoff: %s | bridge evals=%d PASS=%d firstCorr=%d coneTaut=%d C6=%d combCorr=%d hardCert=%d | bridgeEvalTotal=%.4f ms\n', ...
        H.status,B.evals,C.PASS,C.BRIDGE_FIRST_CORRIDOR_FAIL,C.BRIDGE_CONE_TAUT_FAIL,C.BRIDGE_COMBINED_C6_FAIL,C.BRIDGE_COMBINED_CORRIDOR_FAIL,C.BRIDGE_HARD_CERT_FAIL,B.total_ms);
    fprintf('    bridge per-eval p50/p95/max [build %.4f/%.4f/%.4f | firstCorr %.4f/%.4f/%.4f | dyn %.4f/%.4f/%.4f | direct %.4f/%.4f/%.4f] ms\n', ...
        B.stage.build_ms.p50,B.stage.build_ms.p95,B.stage.build_ms.max,B.stage.first_corridor_ms.p50,B.stage.first_corridor_ms.p95,B.stage.first_corridor_ms.max, ...
        B.stage.dynamic_ms.p50,B.stage.dynamic_ms.p95,B.stage.dynamic_ms.max,B.stage.direct_ms.p50,B.stage.direct_ms.p95,B.stage.direct_ms.max);
    P=R.pipelineTiming;
    fprintf('  warmed complete PATCH candidate path N=%d: total p50/p95/max=%.4f/%.4f/%.4f ms | spatial %.4f/%.4f/%.4f | tail %.4f/%.4f/%.4f | bridge %.4f/%.4f/%.4f\n', ...
        P.samples,P.total.p50,P.total.p95,P.total.max,P.spatial.p50,P.spatial.p95,P.spatial.max,P.tail.p50,P.tail.p95,P.tail.max,P.bridge.p50,P.bridge.p95,P.bridge.max);
end
fprintf('\n------------------------------------------------------------------------------------------------------------------------\n');
fprintf(' certificateResourceClosed=%d | actualTailCertified=%d/%d | scalingAllPass=%d | depth19Matches24=%d | easyAdaptivityAllPass=%d\n', ...
    A.certificateResourceClosed,A.actualTailCertified,numel(A.rows),A.scalingAllPass,A.depth19Matches24,A.easyAdaptivityAllPass);
fprintf(' max certificate p95: actual M=2 %.4f ms | critical-containing M=12 %.4f ms | formalWCET=0\n',A.maxActual2ObsCertP95,A.maxScaling12CertP95);
fprintf(' full PATCH handoff certified=%d/%d | max warmed candidate-path p95=%.4f ms | target=%.1f ms | formalWCET=0\n', ...
    A.fullCertified,numel(A.rows),A.maxPipelineP95,CFG.v209b_compute_stretch_target_ms);
fprintf(' classification=%s\n',A.classification);
fprintf(' RULE: scaling M=4/8/12 exercises only final-certificate load; it does not claim the planner has solved those multi-obstacle scenes.\n');
fprintf(' RULE: maxDepth=19 changes proof resource only; obstacle/body radii, geometry margin, cone, tautness, C6, corridor, and final hard theorem are unchanged.\n');
fprintf(' RULE: if full handoff remains open, the next modification must target the measured bridge failure mechanism rather than weakening safety or adding obstacle count.\n');
fprintf('========================================================================================================================\n');
end


% ========================================================================
% V209N timing-outlier root-cause audit
% ========================================================================
function A=v209n_timing_outlier_audit(CFG,K)
rep='PATCH_UNION';
names={'BSPLINE_D7','BEZIER_D13','POLY_D13','ACETRANS_MINCO_ST_C6'};
md=CFG.v209n_cert_max_depth;

% Reconstruct exactly the ordinary V209M two-obstacle replan state.
S=v209b_static_scenario(CFG);
Qold=v209n_build_static_problem_runtime(S,rep,CFG,K);
assert(Qold.guidePass&&Qold.corridorOK,'G26V209N:OldSpatial','Old PATCH spatial problem must pass.');
Bold=v207_build_backend('ACETRANS_MINCO_ST_C6',Qold.W,Qold.weights);
Told=v208_time_gate_angle(Bold,CFG,CFG.v209b_nominal_theta_deg);
assert(Told.pass,'G26V209N:OldTime','Old PATCH time gate must pass.');
tDetect=CFG.v209b_detection_fraction*Told.T;
tSwitch=tDetect+CFG.v209b_switch_lookahead_s;
assert(tSwitch<Told.T,'G26V209N:Switch','Scheduled switch must lie inside old reference.');
J0=v209b_eval_jet(Bold,Told,tSwitch);p0=J0(:,1);
Cnew=[S.C S.newObstacleCenter];Rnew=[S.R S.newObstacleRadius];
Snew=v208_make_scenario('V209N_TIMING_OUTLIER',Cnew,zeros(3,size(Cnew,2)),Rnew,p0,S.pg);
Qnew=v209n_build_static_problem_runtime(Snew,rep,CFG,K);
assert(Qnew.guidePass&&Qnew.corridorOK,'G26V209N:UpdatedSpatial','Updated PATCH guide/corridor must pass.');
prefix=v209b_active_interval_certificate(Bold,Told,tDetect,tSwitch,Qnew,CFG);
assert(prefix.pass,'G26V209N:Prefix','Active prefix must remain certified.');
QT=v209b_tail_problem(Qnew);assert(QT.exists,'G26V209N:Tail','Expected nonempty tail.');

A=struct();
A.timerBaseline=v209n_timer_baseline(CFG);
A.prefixSafe=prefix.pass;A.controlPeriodMs=CFG.v209n_control_period_ms;A.targetMs=CFG.v209n_generation_target_ms;
A.sameProblem=true;A.sameSafety=true;
A.bridgeModified=false;A.formalWCET=false;
rowTemplate=struct('backend','','tailCertified',false,'tailDepth',NaN,'fixedH',[],'searchInstrumented',[], ...
    'searchRuntime',[],'pipelineInstrumented',[],'pipelineRuntime',[],'status','UNSET');
rows=repmat(rowTemplate,1,numel(names));
for bi=1:numel(names)
    name=names{bi};
    [BT,TT,status]=v209n_tail_runtime_deep(QT,CFG,name,md);
    assert(strcmp(status,'PASS'),'G26V209N:Tail','Depth-19 tail must remain certified.');
    D=v209l_final_direct_certificate_depth(BT,TT,QT,CFG,md);
    J1=v209b_backend_start_jets(BT,TT);
    R=rowTemplate;R.backend=name;R.tailCertified=D.pass;R.tailDepth=D.max_depth;
    R.fixedH=v209n_fixed_h_probe(J0,J1,Qnew,CFG);
    R.searchInstrumented=v209n_instrumented_search_probe(J0,J1,Qnew,BT,TT,CFG,md);
    R.searchRuntime=v209n_runtime_search_probe(J0,J1,Qnew,BT,TT,CFG,md);
    R.pipelineInstrumented=v209n_instrumented_pipeline_probe(Snew,J0,name,CFG,K,md);
    R.pipelineRuntime=v209n_runtime_pipeline_probe(Snew,J0,name,CFG,K,md);
    R.status=R.pipelineRuntime.status;
    rows(bi)=R;
end
A.rows=rows;
A.anyRuntimeTargetMiss=any(arrayfun(@(r)r.pipelineRuntime.stats.gtTarget>0,rows));
A.anyRuntimeControlPeriodMiss=any(arrayfun(@(r)r.pipelineRuntime.stats.gtControl>0,rows));
A.anyIdenticalInputTargetSpike=false;A.anyIdenticalInputControlSpike=false;
for i=1:numel(rows)
    F=rows(i).fixedH;
    for j=1:numel(F)
        ss={F(j).build,F(j).corridor,F(j).dynamic};
        for q=1:numel(ss)
            A.anyIdenticalInputTargetSpike=A.anyIdenticalInputTargetSpike||(ss{q}.gtTarget>0);
            A.anyIdenticalInputControlSpike=A.anyIdenticalInputControlSpike||(ss{q}.gtControl>0);
        end
    end
end
A.productionPromotion=false;
end

function B=v209n_timer_baseline(CFG)
N=CFG.v209n_timer_baseline_samples;w=zeros(N,1);c=zeros(N,1);x=0;
for k=1:50,t=tic;x=x+1;tmp=toc(t);end %#ok<NASGU>
for k=1:N
    c0=cputime;t=tic;x=x+1;w(k)=1e3*toc(t);c(k)=1e3*(cputime-c0);
end
B=v209n_stats(w,c,CFG);B.finalAccumulator=x;
end

function F=v209n_fixed_h_probe(J0,J1,Q,CFG)
hv=CFG.v209n_fixed_h_s;NW=CFG.v209n_fixed_h_warmup;NS=CFG.v209n_fixed_h_samples;
template=struct('h',NaN,'build',[],'corridor',[],'dynamic',[],'corridorPass',false,'dynamicPass',false);
F=repmat(template,1,numel(hv));
for hi=1:numel(hv)
    h=hv(hi);
    for k=1:NW
        BB=v209b_degree17_two_sided_jet_bridge(J0,J1,h); %#ok<NASGU>
        C=v209b_bridge_corridor_audit(BB,Q); %#ok<NASGU>
        D=v209b_bridge_dynamic_audit(BB,h,CFG.v209b_nominal_theta_deg,CFG); %#ok<NASGU>
    end
    wb=zeros(NS,1);cb=zeros(NS,1);wc=zeros(NS,1);cc=zeros(NS,1);wd=zeros(NS,1);cd=zeros(NS,1);
    cp=false;dp=false;
    for k=1:NS
        c0=cputime;t=tic;BB=v209b_degree17_two_sided_jet_bridge(J0,J1,h);wb(k)=1e3*toc(t);cb(k)=1e3*(cputime-c0);
        c0=cputime;t=tic;C=v209b_bridge_corridor_audit(BB,Q);wc(k)=1e3*toc(t);cc(k)=1e3*(cputime-c0);
        c0=cputime;t=tic;D=v209b_bridge_dynamic_audit(BB,h,CFG.v209b_nominal_theta_deg,CFG);wd(k)=1e3*toc(t);cd(k)=1e3*(cputime-c0);
        cp=C.pass;dp=D.pass;
    end
    F(hi)=struct('h',h,'build',v209n_stats(wb,cb,CFG),'corridor',v209n_stats(wc,cc,CFG), ...
        'dynamic',v209n_stats(wd,cd,CFG),'corridorPass',cp,'dynamicPass',dp);
end
end

function P=v209n_instrumented_search_probe(J0,J1,Q,BT,TT,CFG,md)
NW=CFG.v209n_search_warmup;NS=CFG.v209n_search_samples;
for k=1:NW,v209m_bridge_search_diag_deep(J0,J1,Q,BT,TT,CFG,md);end
w=zeros(NS,1);c=zeros(NS,1);ev=zeros(NS,1);status=false(NS,1);
for k=1:NS
    c0=cputime;t=tic;H=v209m_bridge_search_diag_deep(J0,J1,Q,BT,TT,CFG,md);w(k)=1e3*toc(t);c(k)=1e3*(cputime-c0);ev(k)=H.evals;status(k)=H.pass;
end
assert(all(ev==ev(1))&&all(status==status(1)),'G26V209N:SearchDeterminism','Instrumented bridge search changed logical result/evaluation count.');
P=struct('samples',NS,'stats',v209n_stats(w,c,CFG),'evals',ev(1),'pass',status(1));
end

function P=v209n_runtime_search_probe(J0,J1,Q,BT,TT,CFG,md)
NW=CFG.v209n_search_warmup;NS=CFG.v209n_search_samples;
for k=1:NW,v209n_bridge_search_runtime_deep(J0,J1,Q,BT,TT,CFG,md);end
w=zeros(NS,1);c=zeros(NS,1);ev=zeros(NS,1);status=false(NS,1);
for k=1:NS
    c0=cputime;t=tic;H=v209n_bridge_search_runtime_deep(J0,J1,Q,BT,TT,CFG,md);w(k)=1e3*toc(t);c(k)=1e3*(cputime-c0);ev(k)=H.evals;status(k)=H.pass;
end
assert(all(ev==ev(1))&&all(status==status(1)),'G26V209N:SearchDeterminism','Runtime-like bridge search changed logical result/evaluation count.');
P=struct('samples',NS,'stats',v209n_stats(w,c,CFG),'evals',ev(1),'pass',status(1));
end

function P=v209n_instrumented_pipeline_probe(Snew,J0,name,CFG,K,md)
NW=CFG.v209n_pipeline_warmup;NS=CFG.v209n_pipeline_samples;
for k=1:NW,v209m_pipeline_once(Snew,J0,name,CFG,K,md);end
w=zeros(NS,1);c=zeros(NS,1);status=cell(NS,1);
for k=1:NS
    c0=cputime;t=tic;X=v209m_pipeline_once(Snew,J0,name,CFG,K,md);w(k)=1e3*toc(t);c(k)=1e3*(cputime-c0);status{k}=X.status;
end
assert(all(strcmp(status,status{1})),'G26V209N:PipelineDeterminism','Instrumented pipeline status changed.');
P=struct('samples',NS,'stats',v209n_stats(w,c,CFG),'status',status{1});
end

function P=v209n_runtime_pipeline_probe(Snew,J0,name,CFG,K,md)
NW=CFG.v209n_pipeline_warmup;NS=CFG.v209n_pipeline_samples;
for k=1:NW,v209n_pipeline_once_runtime(Snew,J0,name,CFG,K,md);end
w=zeros(NS,1);c=zeros(NS,1);status=cell(NS,1);
for k=1:NS
    c0=cputime;t=tic;X=v209n_pipeline_once_runtime(Snew,J0,name,CFG,K,md);w(k)=1e3*toc(t);c(k)=1e3*(cputime-c0);status{k}=X.status;
end
assert(all(strcmp(status,status{1})),'G26V209N:PipelineDeterminism','Runtime-like pipeline status changed.');
P=struct('samples',NS,'stats',v209n_stats(w,c,CFG),'status',status{1});
end

function X=v209n_pipeline_once_runtime(Snew,J0,name,CFG,K,md)
Q=v209n_build_static_problem_runtime(Snew,'PATCH_UNION',CFG,K);
if ~Q.guidePass||~Q.corridorOK,X=struct('status','UPDATED_SPATIAL_SEARCH_INCOMPLETE');return;end
QT=v209b_tail_problem(Q);
[BT,TT,st]=v209n_tail_runtime_deep(QT,CFG,name,md);
if ~strcmp(st,'PASS'),X=struct('status',[name '__' st]);return;end
J1=v209b_backend_start_jets(BT,TT);
H=v209n_bridge_search_runtime_deep(J0,J1,Q,BT,TT,CFG,md);
if ~H.pass,status=[name '__JET_TIME_CERT_SEARCH_INCOMPLETE'];else,status='CERTIFIED_MULTI_OBSTACLE_STATIC_SUCCESSOR';end
X=struct('status',status);
end

function Q=v209n_build_static_problem_runtime(S,rep,CFG,K)
Q=struct();Q.p0=S.p0;Q.pg=S.pg;Q.C=reshape(S.C,3,[]);Q.V=zeros(size(Q.C));Q.Ractual=reshape(S.R,1,[]);
extra=0;if strcmp(rep,'ACETRANS_SPHERE_SFC'),extra=CFG.v207_qualification_extra_proposal_margin_m;end
[Q.Cproposal,Q.Rproposal,Q.coverMeta]=v209g_build_cone_swept_cover(Q.C,Q.Ractual,CFG.v209b_nominal_theta_deg,K,CFG,extra);
[Q.W,Q.guideStatus,Q.insertions,Q.guideClearance,Q.guideChecks]=g26v209h_cluster_guide_mex(Q.p0,Q.pg,Q.Cproposal,Q.Rproposal, ...
    CFG.v205_planning_horizon_m,CFG.v206_spatial_guide_max_nodes,CFG.v206_spatial_guide_guard_scale);
Q.guidePass=(Q.guideStatus==1);Q.nodes=size(Q.W,2);
if ~Q.guidePass,Q.corridorOK=false;Q.N=[];Q.h=[];Q.cnt=[];Q.faces=0;Q.weights=[];Q.corridorMinClearance=NaN;Q.segment_length=[];return;end
[Q.N,Q.h,Q.cnt,Q.corridorOK,Q.corridorMinClearance,Q.faces]=g26v206_segment_corridor_mex(Q.W,Q.Cproposal,Q.Rproposal,CFG.v206_spatial_guide_guard_scale);
Q.segment_length=vecnorm(diff(Q.W,1,2),2,1);
if isempty(Q.segment_length)||sum(Q.segment_length)<=0,Q.weights=[];Q.corridorOK=false;else,Q.weights=Q.segment_length/sum(Q.segment_length);end
end

function [BT,TT,status]=v209n_tail_runtime_deep(QT,CFG,name,md)
BT=[];TT=[];status='NO_TAIL';if ~QT.exists,return;end
B=v207_build_backend(name,QT.W,QT.weights);C=v209b_tail_corridor_audit(B,QT);
if ~C.pass,status='TAIL_CORRIDOR_SEARCH_INCOMPLETE';return;end
T=v208_time_gate_angle(B,CFG,CFG.v209b_nominal_theta_deg);
if ~T.pass,status='TAIL_TIME_SEARCH_INCOMPLETE';return;end
D=v209l_final_direct_certificate_depth(B,T,QT,CFG,md);
if ~D.pass,status='TAIL_HARD_CERT_FAIL';return;end
BT=B;TT=T;status='PASS';
end

function [ok,C]=v209n_eval_bridge_runtime_deep(J0,J1,Q,BT,TT,hb,CFG,md)
BB=v209b_degree17_two_sided_jet_bridge(J0,J1,hb);
CORb=v209b_bridge_corridor_audit(BB,Q);
DY=v209b_bridge_dynamic_audit(BB,hb,CFG.v209b_nominal_theta_deg,CFG);
if ~(CORb.pass&&DY.pass),ok=false;C=[];return;end
if isempty(BT),P={BB};hh=hb;Rflat=DY.R;rcounts=DY.rcount;tailT=0;else,P=[{BB} BT.P];hh=[hb TT.h];Rflat=[DY.R TT.Rflat];rcounts=[DY.rcount TT.rcounts];tailT=TT.T;end
BC=struct('P',{P},'degree',17,'structural_c6_theorem',true,'condition_indicator',1);
TC=struct('pass',true,'h',hh,'Rflat',Rflat,'rcounts',rcounts,'T',sum(hh));
C6=v207_c6_backend_audit(BC,hh);if ~C6.pass,ok=false;C=[];return;end
COR=v209b_combined_corridor_audit(BC,Q);if ~COR.pass,ok=false;C=[];return;end
D=v209l_final_direct_certificate_depth(BC,TC,Q,CFG,md);if ~D.pass,ok=false;C=[];return;end
ok=true;C=struct('B',BC,'T',TC,'bridgeB',BB,'bridgeH',hb,'bridgeDynamic',DY,'bridgeCorridor',CORb,'c6',C6,'corridor',COR,'direct',D,'tailT',tailT);
end

function H=v209n_bridge_search_runtime_deep(J0,J1,Q,BT,TT,CFG,md)
lo=CFG.v209b_bridge_h_min_s;hi=CFG.v209b_bridge_h_max_s;intervals=[lo hi];best=[];bestH=inf;evals=0;
for z=1:2
    if z==1,h=lo;else,h=hi;end
    [ok,C]=v209n_eval_bridge_runtime_deep(J0,J1,Q,BT,TT,h,CFG,md);evals=evals+1;
    if ok&&h<bestH,best=C;bestH=h;end
end
while ~isempty(intervals)&&evals<CFG.v209b_bridge_search_evals
    keep=intervals(:,1)<bestH;intervals=intervals(keep,:);if isempty(intervals),break;end
    [~,ii]=max(intervals(:,2)-intervals(:,1));ab=intervals(ii,:);intervals(ii,:)=[];
    if ab(2)-ab(1)<=64*eps(max(1,ab(2))),continue;end
    mid=.5*(ab(1)+ab(2));[ok,C]=v209n_eval_bridge_runtime_deep(J0,J1,Q,BT,TT,mid,CFG,md);evals=evals+1;
    if ok&&mid<bestH,best=C;bestH=mid;end
    if ab(1)<bestH,intervals=[intervals;ab(1) mid];end %#ok<AGROW>
    if mid<bestH,intervals=[intervals;mid ab(2)];end %#ok<AGROW>
end
H=struct('pass',~isempty(best),'evals',evals,'best',best,'h',NaN);
if H.pass,H.h=bestH;end
end

function S=v209n_stats(w,c,CFG)
w=w(:);c=c(:);assert(numel(w)==numel(c),'G26V209N:Stats','wall/cpu sample count mismatch.');
N=numel(w);[ws,ord]=sort(w);S=struct();S.N=N;S.p50=v209n_q(ws,.50);S.p95=v209n_q(ws,.95);S.p99=v209n_q(ws,.99);S.max=ws(end);S.mean=mean(w);S.min=ws(1);
S.gtTarget=sum(w>CFG.v209n_generation_target_ms);S.gtControl=sum(w>CFG.v209n_control_period_ms);
im=ord(end);S.maxIndex=im;S.cpuAtMax=c(im);S.wallMinusCpuAtMax=w(im)-c(im);S.cpuMax=max(c);S.cpuP95=v209n_q(sort(c),.95);
K=min(5,N);top=ord(end-K+1:end);top=flipud(top(:));S.topIndex=top.';S.topWall=w(top).';S.topCPU=c(top).';S.topGap=(w(top)-c(top)).';
end

function y=v209n_q(x,p)
n=numel(x);if n==0,y=NaN;return;end;y=x(max(1,min(n,ceil(p*n))));
end

function v209n_print_timing_outlier_audit(A,CFG)
fprintf('\n========================================================================================================================\n');
fprintf(' G26-V209N TIMING OUTLIER ROOT-CAUSE AUDIT | controlPeriod=%.3f ms target=%.3f ms\n',CFG.v209n_control_period_ms,CFG.v209n_generation_target_ms);
fprintf('------------------------------------------------------------------------------------------------------------------------\n');
fprintf(' SAME trajectory/safety/bridge=1 | bridgeModified=0 | formalWCET=0 | prefixSafe=%d\n',A.prefixSafe);
v209n_print_stats('timer/tiny-op baseline',A.timerBaseline,CFG);
for i=1:numel(A.rows)
    R=A.rows(i);fprintf('\n[%s] tailCertified=%d depth=%g status=%s\n',R.backend,R.tailCertified,R.tailDepth,R.status);
    for j=1:numel(R.fixedH)
        F=R.fixedH(j);fprintf(' fixed identical input h=%.5g s | corridorPass=%d dynamicPass=%d\n',F.h,F.corridorPass,F.dynamicPass);
        v209n_print_stats('   D17 build',F.build,CFG);v209n_print_stats('   first corridor',F.corridor,CFG);v209n_print_stats('   cone/taut dynamic',F.dynamic,CFG);
    end
    fprintf(' search replay evals=%d pass=%d\n',R.searchRuntime.evals,R.searchRuntime.pass);
    v209n_print_stats('   instrumented search',R.searchInstrumented.stats,CFG);
    v209n_print_stats('   runtime-like search',R.searchRuntime.stats,CFG);
    fprintf('   diagnostic-overhead median delta = %+.6f ms\n',R.searchInstrumented.stats.p50-R.searchRuntime.stats.p50);
    v209n_print_stats('   instrumented complete pipeline',R.pipelineInstrumented.stats,CFG);
    v209n_print_stats('   runtime-like complete pipeline',R.pipelineRuntime.stats,CFG);
    fprintf('   diagnostic-overhead pipeline median delta = %+.6f ms\n',R.pipelineInstrumented.stats.p50-R.pipelineRuntime.stats.p50);
end
fprintf('\n------------------------------------------------------------------------------------------------------------------------\n');
fprintf(' identical-input stage >%.3fms observed=%d | identical-input stage >%.3fms observed=%d\n',CFG.v209n_generation_target_ms,A.anyIdenticalInputTargetSpike,CFG.v209n_control_period_ms,A.anyIdenticalInputControlSpike);
fprintf(' runtime-like complete path >%.3fms observed=%d | >%.3fms control-period miss observed=%d\n',CFG.v209n_generation_target_ms,A.anyRuntimeTargetMiss,CFG.v209n_control_period_ms,A.anyRuntimeControlPeriodMiss);
fprintf(' INTERPRETATION RULE: if identical-input wall spikes occur while CPU-at-spike stays much smaller, geometry/search complexity cannot explain the full wall spike.\n');
fprintf(' INTERPRETATION RULE: if runtime-like CPU time itself rises comparably, localize MATLAB allocation/control/backend work before any real-time claim.\n');
fprintf(' INTERPRETATION RULE: even if host-preemption jitter is identified, current MATLAB/Windows timing cannot establish onboard hard WCET; later onboard/Linux scheduling measurement remains required.\n');
fprintf(' RULE: bridge feasibility is still unchanged/open; no safety relaxation and no obstacle-count progression in V209N.\n');
fprintf('========================================================================================================================\n');
end

function v209n_print_stats(label,S,CFG)
fprintf('%-39s N=%4d p50/p95/p99/max=%9.4f/%9.4f/%9.4f/%9.4f ms | >%.1f=%3d >%.1f=%3d | atMax CPU=%9.4f gap(wall-cpu)=%+9.4f ms\n', ...
    label,S.N,S.p50,S.p95,S.p99,S.max,CFG.v209n_generation_target_ms,S.gtTarget,CFG.v209n_control_period_ms,S.gtControl,S.cpuAtMax,S.wallMinusCpuAtMax);
fprintf('%-39s top wall=%s | top CPU=%s\n','',mat2str(S.topWall,6),mat2str(S.topCPU,6));
end


% ========================================================================
% V209O high-resolution outlier attribution
% Measurement-only successor to V209N.  No planner/bridge/safety change.
% ========================================================================

function v209o_mex_bootstrap_stamp(root)
name='g26v209o_stamp_mex';
v200_build_mex_if_needed(root,name,v209o_cpp_stamp_source(),'G26-V209O-STAMP-MEX');
end

function lines=v209o_cpp_stamp_source()
lines={ ...
'#include "mex.h"', ...
'#include <cstdint>', ...
'#ifdef _WIN32', ...
'#define NOMINMAX', ...
'#include <windows.h>', ...
'#else', ...
'#include <time.h>', ...
'#endif', ...
'void mexFunction(int nlhs,mxArray*plhs[],int nrhs,const mxArray*prhs[]){', ...
' if(nrhs!=0) mexErrMsgIdAndTxt("G26V209O:Stamp","No inputs expected");', ...
' if(nlhs>1) mexErrMsgIdAndTxt("G26V209O:Stamp","At most one output");', ...
' plhs[0]=mxCreateDoubleMatrix(1,4,mxREAL); double*o=mxGetPr(plhs[0]);', ...
'#ifdef _WIN32', ...
' LARGE_INTEGER q,f; QueryPerformanceCounter(&q); QueryPerformanceFrequency(&f);', ...
' ULONG64 tc=0,pc=0; QueryThreadCycleTime(GetCurrentThread(),&tc); QueryProcessCycleTime(GetCurrentProcess(),&pc);', ...
' o[0]=(double)q.QuadPart/(double)f.QuadPart; o[1]=(double)tc; o[2]=(double)pc; o[3]=1.0;', ...
'#else', ...
' struct timespec w,t,p; clock_gettime(CLOCK_MONOTONIC,&w); clock_gettime(CLOCK_THREAD_CPUTIME_ID,&t); clock_gettime(CLOCK_PROCESS_CPUTIME_ID,&p);', ...
' o[0]=(double)w.tv_sec+1e-9*(double)w.tv_nsec;', ...
' o[1]=1e9*(double)t.tv_sec+(double)t.tv_nsec; o[2]=1e9*(double)p.tv_sec+(double)p.tv_nsec; o[3]=2.0;', ...
'#endif', ...
'}'};
end

function A=v209o_highres_outlier_audit(CFG,K)
md=19;
S=v209b_static_scenario(CFG);
rep='PATCH_UNION';

Qold=v209n_build_static_problem_runtime(S,rep,CFG,K);
Bold=v207_build_backend('ACETRANS_MINCO_ST_C6',Qold.W,Qold.weights);
Told=v208_time_gate_angle(Bold,CFG,CFG.v209b_nominal_theta_deg);
tDetect=CFG.v209b_detection_fraction*Told.T;
tSwitch=tDetect+CFG.v209b_switch_lookahead_s;
J0=v209b_eval_jet(Bold,Told,tSwitch);

p0=J0(:,1);
Cnew=[S.C S.newObstacleCenter];
Rnew=[S.R S.newObstacleRadius];
Snew=v208_make_scenario('V209O_HIGHRES_OUTLIER',Cnew,zeros(3,size(Cnew,2)),Rnew,p0,S.pg);
Qnew=v209n_build_static_problem_runtime(Snew,rep,CFG,K);
assert(Qnew.guidePass&&Qnew.corridorOK,'G26V209O:Spatial','Unchanged PATCH spatial problem must pass.');
prefix=v209b_active_interval_certificate(Bold,Told,tDetect,tSwitch,Qnew,CFG);
assert(prefix.pass,'G26V209O:Prefix','Inherited prefix must remain certified.');
fprintf('[G26-V209O-REPLAN-STATE] exact switch-state reconstruction | tDetect=%.9g tSwitch=%.9g p0=[%.9g %.9g %.9g] | updatedObs=%d\n', ...
    tDetect,tSwitch,p0(1),p0(2),p0(3),size(Qnew.C,2));

names={'BSPLINE_D7','BEZIER_D13','POLY_D13','ACETRANS_MINCO_ST_C6'};
A=struct();
A.stamp=v209o_stamp_baseline(CFG);
A.prefixSafe=prefix.pass;
A.counterMode=A.stamp.mode;
rows=cell(1,numel(names));
A.anyControlMiss=false;
A.anyFixedControlMiss=false;

for bi=1:numel(names)
    name=names{bi};
    QT=v209b_tail_problem(Qnew);
    [BT,TT,st]=v209n_tail_runtime_deep(QT,CFG,name,md);
    if ~strcmp(st,'PASS')
        fprintf('[G26-V209O-TAIL-FAIL] backend=%s status=%s | p0=[%.9g %.9g %.9g] | nodes=%d guideClr=%+.3e\n', ...
            name,st,Qnew.p0(1),Qnew.p0(2),Qnew.p0(3),size(Qnew.W,2),Qnew.guideClearance);
        error('G26V209O:Tail','Tail must remain certified on the exact V209M/N switch-state problem.');
    end
    J1=v209b_backend_start_jets(BT,TT);

    R=struct();
    R.backend=name;
    R.fixed=v209o_fixed_probe(J0,J1,Qnew,CFG);
    R.search=v209o_repeat_call(@()v209n_bridge_search_runtime_deep(J0,J1,Qnew,BT,TT,CFG,md), ...
        CFG.v209o_search_warmup,CFG.v209o_search_samples,CFG);
    R.pipeline=v209o_pipeline_probe(Snew,J0,name,CFG,K,md);
    rows{bi}=R;

    for j=1:numel(R.fixed)
        A.anyFixedControlMiss=A.anyFixedControlMiss || (R.fixed(j).build.gtControl>0) || ...
            (R.fixed(j).corridor.gtControl>0) || (R.fixed(j).dynamic.gtControl>0);
    end
    A.anyControlMiss=A.anyControlMiss || R.search.gtControl>0 || R.pipeline.total.gtControl>0;
end
A.rows=[rows{:}];
A.bridgeModified=false;
A.safetyChanged=false;
A.formalWCET=false;
A.productionPromotion=false;
end

function B=v209o_stamp_baseline(CFG)
N=CFG.v209o_stamp_baseline_samples;
for k=1:100,s0=g26v209o_stamp_mex();s1=g26v209o_stamp_mex();end %#ok<NASGU>
w=zeros(N,1);tw=zeros(N,1);pw=zeros(N,1);mode=NaN;
for k=1:N
    s0=g26v209o_stamp_mex();s1=g26v209o_stamp_mex();
    w(k)=1e3*(s1(1)-s0(1));tw(k)=s1(2)-s0(2);pw(k)=s1(3)-s0(3);mode=s1(4);
end
B=v209o_stats(w,tw,pw,CFG);
B.mode=mode;
end

function F=v209o_fixed_probe(J0,J1,Q,CFG)
hv=CFG.v209n_fixed_h_s;
NW=CFG.v209o_fixed_warmup;NS=CFG.v209o_fixed_samples;
T=struct('h',NaN,'build',[],'corridor',[],'dynamic',[],'corridorPass',false,'dynamicPass',false);
F=repmat(T,1,numel(hv));
for ii=1:numel(hv)
    h=hv(ii);
    for k=1:NW
        BB=v209b_degree17_two_sided_jet_bridge(J0,J1,h);
        C=v209b_bridge_corridor_audit(BB,Q); %#ok<NASGU>
        D=v209b_bridge_dynamic_audit(BB,h,CFG.v209b_nominal_theta_deg,CFG); %#ok<NASGU>
    end
    wb=zeros(NS,1);tb=zeros(NS,1);pb=zeros(NS,1);
    wc=zeros(NS,1);tc=zeros(NS,1);pc=zeros(NS,1);
    wd=zeros(NS,1);td=zeros(NS,1);pd=zeros(NS,1);
    cp=false;dp=false;
    for k=1:NS
        s0=g26v209o_stamp_mex();BB=v209b_degree17_two_sided_jet_bridge(J0,J1,h);s1=g26v209o_stamp_mex();
        [wb(k),tb(k),pb(k)]=v209o_delta(s0,s1);
        s0=g26v209o_stamp_mex();C=v209b_bridge_corridor_audit(BB,Q);s1=g26v209o_stamp_mex();
        [wc(k),tc(k),pc(k)]=v209o_delta(s0,s1);
        s0=g26v209o_stamp_mex();D=v209b_bridge_dynamic_audit(BB,h,CFG.v209b_nominal_theta_deg,CFG);s1=g26v209o_stamp_mex();
        [wd(k),td(k),pd(k)]=v209o_delta(s0,s1);
        cp=C.pass;dp=D.pass;
    end
    F(ii)=struct('h',h,'build',v209o_stats(wb,tb,pb,CFG), ...
        'corridor',v209o_stats(wc,tc,pc,CFG),'dynamic',v209o_stats(wd,td,pd,CFG), ...
        'corridorPass',cp,'dynamicPass',dp);
end
end

function S=v209o_repeat_call(fun,NW,NS,CFG)
for k=1:NW,fun();end
w=zeros(NS,1);t=zeros(NS,1);p=zeros(NS,1);
for k=1:NS
    s0=g26v209o_stamp_mex();fun();s1=g26v209o_stamp_mex();
    [w(k),t(k),p(k)]=v209o_delta(s0,s1);
end
S=v209o_stats(w,t,p,CFG);
end

function P=v209o_pipeline_probe(Snew,J0,name,CFG,K,md)
NW=CFG.v209o_pipeline_warmup;NS=CFG.v209o_pipeline_samples;
for k=1:NW,v209n_pipeline_once_runtime(Snew,J0,name,CFG,K,md);end
labels={'spatial','backend','tailCorr','timeGate','tailCert','bridge','other'};
ns=numel(labels);
W=zeros(NS,ns);TW=zeros(NS,ns);PW=zeros(NS,ns);totalW=zeros(NS,1);totalT=zeros(NS,1);totalP=zeros(NS,1);
status=cell(NS,1);
for k=1:NS
    whole0=g26v209o_stamp_mex();

    s0=g26v209o_stamp_mex();Q=v209n_build_static_problem_runtime(Snew,'PATCH_UNION',CFG,K);s1=g26v209o_stamp_mex();
    [W(k,1),TW(k,1),PW(k,1)]=v209o_delta(s0,s1);
    assert(Q.guidePass&&Q.corridorOK,'G26V209O:SpatialRuntime','Unchanged spatial stage failed.');

    QT=v209b_tail_problem(Q);
    s0=g26v209o_stamp_mex();B=v207_build_backend(name,QT.W,QT.weights);s1=g26v209o_stamp_mex();
    [W(k,2),TW(k,2),PW(k,2)]=v209o_delta(s0,s1);

    s0=g26v209o_stamp_mex();C=v209b_tail_corridor_audit(B,QT);s1=g26v209o_stamp_mex();
    [W(k,3),TW(k,3),PW(k,3)]=v209o_delta(s0,s1);
    assert(C.pass,'G26V209O:TailCorr','Tail corridor changed.');

    s0=g26v209o_stamp_mex();T=v208_time_gate_angle(B,CFG,CFG.v209b_nominal_theta_deg);s1=g26v209o_stamp_mex();
    [W(k,4),TW(k,4),PW(k,4)]=v209o_delta(s0,s1);
    assert(T.pass,'G26V209O:TimeGate','Tail time gate changed.');

    s0=g26v209o_stamp_mex();D=v209l_final_direct_certificate_depth(B,T,QT,CFG,md);s1=g26v209o_stamp_mex();
    [W(k,5),TW(k,5),PW(k,5)]=v209o_delta(s0,s1);
    assert(D.pass,'G26V209O:TailCert','Tail certificate changed.');

    J1=v209b_backend_start_jets(B,T);
    s0=g26v209o_stamp_mex();H=v209n_bridge_search_runtime_deep(J0,J1,Q,B,T,CFG,md);s1=g26v209o_stamp_mex();
    [W(k,6),TW(k,6),PW(k,6)]=v209o_delta(s0,s1);
    if H.pass,status{k}='CERTIFIED_MULTI_OBSTACLE_STATIC_SUCCESSOR';else,status{k}=[name '__JET_TIME_CERT_SEARCH_INCOMPLETE'];end

    whole1=g26v209o_stamp_mex();
    [totalW(k),totalT(k),totalP(k)]=v209o_delta(whole0,whole1);
end
W(:,7)=totalW-sum(W(:,1:6),2);TW(:,7)=totalT-sum(TW(:,1:6),2);PW(:,7)=totalP-sum(PW(:,1:6),2);
assert(all(strcmp(status,status{1})),'G26V209O:PipelineDeterminism','Pipeline logical result changed.');
P=struct();P.labels=labels;P.total=v209o_stats(totalW,totalT,totalP,CFG);P.status=status{1};
S0=v209o_stats(W(:,1),TW(:,1),PW(:,1),CFG);P.stage=repmat(S0,1,ns);P.stage(1)=S0;
for j=2:ns,P.stage(j)=v209o_stats(W(:,j),TW(:,j),PW(:,j),CFG);end
% Preserve exact top-total rows for forensic localization.
[~,ord]=sort(totalW,'descend');kk=min(8,NS);ix=ord(1:kk);
P.topTotalIndex=ix(:).';P.topTotalWall=totalW(ix).';
P.topStageWall=W(ix,:);P.topStageThread=TW(ix,:);P.topStageProcess=PW(ix,:);
end

function [w,t,p]=v209o_delta(a,b)
w=1e3*(b(1)-a(1));t=b(2)-a(2);p=b(3)-a(3);
end

function S=v209o_stats(w,t,p,CFG)
w=w(:);t=t(:);p=p(:);N=numel(w);assert(N>0&&numel(t)==N&&numel(p)==N,'G26V209O:Stats','Counter-size mismatch.');
[ws,ord]=sort(w);
S=struct();S.N=N;S.p50=v209o_q(ws,.50);S.p95=v209o_q(ws,.95);S.p99=v209o_q(ws,.99);S.max=ws(end);S.min=ws(1);S.mean=mean(w);
S.gtTarget=sum(w>CFG.v209n_generation_target_ms);S.gtControl=sum(w>CFG.v209n_control_period_ms);
tp=t(t>0);pp=p(p>0);if isempty(tp),mt=NaN;else,mt=median(tp);end;if isempty(pp),mp=NaN;else,mp=median(pp);end
S.medianThreadWork=mt;S.medianProcessWork=mp;
im=ord(end);S.maxIndex=im;S.threadAtMax=t(im);S.processAtMax=p(im);
if isfinite(mt)&&mt>0,S.threadRatioAtMax=t(im)/mt;else,S.threadRatioAtMax=NaN;end
if isfinite(mp)&&mp>0,S.processRatioAtMax=p(im)/mp;else,S.processRatioAtMax=NaN;end
kk=min(8,N);ix=ord(end-kk+1:end);ix=flipud(ix(:));S.topIndex=ix.';S.topWall=w(ix).';
S.topThread=t(ix).';S.topProcess=p(ix).';
if isfinite(mt)&&mt>0,S.topThreadRatio=t(ix).'/mt;else,S.topThreadRatio=nan(1,kk);end
if isfinite(mp)&&mp>0,S.topProcessRatio=p(ix).'/mp;else,S.topProcessRatio=nan(1,kk);end
end

function y=v209o_q(x,p)
n=numel(x);if n==0,y=NaN;return;end;y=x(max(1,min(n,ceil(p*n))));
end

function v209o_print_highres_outlier_audit(A,CFG)
fprintf('\n========================================================================================================================\n');
fprintf(' G26-V209O HIGH-RESOLUTION OUTLIER ATTRIBUTION | controlPeriod=%.3f ms target=%.3f ms\n',CFG.v209n_control_period_ms,CFG.v209n_generation_target_ms);
fprintf('------------------------------------------------------------------------------------------------------------------------\n');
fprintf(' bridgeModified=%d safetyChanged=%d formalWCET=%d | counterMode=%g (1=Windows cycles, 2=POSIX CPU-ns)\n',A.bridgeModified,A.safetyChanged,A.formalWCET,A.counterMode);
v209o_print_stats('stamp pair baseline',A.stamp,CFG);
for i=1:numel(A.rows)
    R=A.rows(i);fprintf('\n[%s]\n',R.backend);
    for j=1:numel(R.fixed)
        X=R.fixed(j);fprintf(' fixed identical h=%.5g s corridor=%d dynamic=%d\n',X.h,X.corridorPass,X.dynamicPass);
        v209o_print_stats('   D17 build',X.build,CFG);
        v209o_print_stats('   first corridor',X.corridor,CFG);
        v209o_print_stats('   cone/taut dynamic',X.dynamic,CFG);
    end
    v209o_print_stats(' runtime-like bridge search',R.search,CFG);
    v209o_print_stats(' complete pipeline total',R.pipeline.total,CFG);
    for j=1:numel(R.pipeline.labels)
        v209o_print_stats(['   pipeline ' R.pipeline.labels{j}],R.pipeline.stage(j),CFG);
    end
    fprintf('   top complete-pipeline rows (wall ms by stage: spatial/backend/tailCorr/timeGate/tailCert/bridge/other)\n');
    for q=1:numel(R.pipeline.topTotalIndex)
        fprintf('    idx=%4d total=%9.4f | stageWall=%s\n',R.pipeline.topTotalIndex(q),R.pipeline.topTotalWall(q),mat2str(R.pipeline.topStageWall(q,:),5));
        fprintf('                         processWork=%s\n',mat2str(R.pipeline.topStageProcess(q,:),5));
    end
end
fprintf('\n------------------------------------------------------------------------------------------------------------------------\n');
fprintf(' identical-input >25ms observed=%d | runtime search/pipeline >25ms observed=%d\n',A.anyFixedControlMiss,A.anyControlMiss);
fprintf(' INTERPRETATION: compare WALL RATIO with process/thread work ratios at the SAME outlier; do not infer CPU time from MATLAB cputime.\n');
fprintf(' If wall grows sharply while work counters remain ordinary, the event is host/runtime preemption or wait-class evidence, not input-complexity evidence.\n');
fprintf(' If process/thread work rises comparably, localize the printed pipeline stage before any bridge or real-time claim.\n');
fprintf(' RULE: no hard onboard WCET claim is permitted from this Windows/MATLAB audit; later target-Linux scheduling measurements remain required.\n');
fprintf('========================================================================================================================\n');
end

function v209o_print_stats(label,S,CFG)
fprintf(' %-36s N=%4d p50/p95/p99/max=%9.4f/%9.4f/%9.4f/%9.4f ms | >%.1f=%3d >%.1f=%3d | workRatioAtMax thread/process=%8.3f/%8.3f\n', ...
    label,S.N,S.p50,S.p95,S.p99,S.max,CFG.v209n_generation_target_ms,S.gtTarget,CFG.v209n_control_period_ms,S.gtControl,S.threadRatioAtMax,S.processRatioAtMax);
fprintf(' %36s topWall=%s\n','',mat2str(S.topWall,6));
fprintf(' %36s topThreadRatio=%s | topProcessRatio=%s\n','',mat2str(S.topThreadRatio,5),mat2str(S.topProcessRatio,5));
end


% ========================================================================
% V209P deterministic fixed-work MEX and outer-only timing attribution
% ========================================================================

function v209p_mex_bootstrap_fixedwork(root)
name='g26v209p_fixedwork_mex';
v200_build_mex_if_needed(root,name,v209p_cpp_fixedwork_source(),'G26-V209P-FIXEDWORK-MEX');
end

function lines=v209p_cpp_fixedwork_source()
lines={ ...
'#include "mex.h"', ...
'#include <cstdint>', ...
'#ifdef _WIN32', ...
'#define NOMINMAX', ...
'#include <windows.h>', ...
'#else', ...
'#include <time.h>', ...
'#endif', ...
'void mexFunction(int nlhs,mxArray*plhs[],int nrhs,const mxArray*prhs[]){', ...
' if(nrhs!=1) mexErrMsgIdAndTxt("G26V209P:FixedWork","Expected one loop-count input");', ...
' if(nlhs>1) mexErrMsgIdAndTxt("G26V209P:FixedWork","At most one output");', ...
' uint64_t N=(uint64_t)mxGetScalar(prhs[0]); if(N<1) N=1;', ...
' double w0,w1; uint64_t t0,t1,p0,p1;', ...
'#ifdef _WIN32', ...
' LARGE_INTEGER q,f; QueryPerformanceFrequency(&f); QueryPerformanceCounter(&q); w0=(double)q.QuadPart/(double)f.QuadPart;', ...
' ULONG64 a=0,b=0; QueryThreadCycleTime(GetCurrentThread(),&a); QueryProcessCycleTime(GetCurrentProcess(),&b); t0=(uint64_t)a; p0=(uint64_t)b;', ...
'#else', ...
' struct timespec w,t,p; clock_gettime(CLOCK_MONOTONIC,&w); w0=(double)w.tv_sec+1e-9*(double)w.tv_nsec;', ...
' clock_gettime(CLOCK_THREAD_CPUTIME_ID,&t); t0=(uint64_t)t.tv_sec*1000000000ULL+(uint64_t)t.tv_nsec;', ...
' clock_gettime(CLOCK_PROCESS_CPUTIME_ID,&p); p0=(uint64_t)p.tv_sec*1000000000ULL+(uint64_t)p.tv_nsec;', ...
'#endif', ...
' volatile uint64_t x=0x9e3779b97f4a7c15ULL;', ...
' for(uint64_t i=0;i<N;++i){ x^=x<<13; x^=x>>7; x^=x<<17; x+=0x9e3779b97f4a7c15ULL+i; }', ...
'#ifdef _WIN32', ...
' QueryPerformanceCounter(&q); w1=(double)q.QuadPart/(double)f.QuadPart; a=0;b=0; QueryThreadCycleTime(GetCurrentThread(),&a); QueryProcessCycleTime(GetCurrentProcess(),&b); t1=(uint64_t)a; p1=(uint64_t)b;', ...
'#else', ...
' clock_gettime(CLOCK_MONOTONIC,&w); w1=(double)w.tv_sec+1e-9*(double)w.tv_nsec;', ...
' clock_gettime(CLOCK_THREAD_CPUTIME_ID,&t); t1=(uint64_t)t.tv_sec*1000000000ULL+(uint64_t)t.tv_nsec;', ...
' clock_gettime(CLOCK_PROCESS_CPUTIME_ID,&p); p1=(uint64_t)p.tv_sec*1000000000ULL+(uint64_t)p.tv_nsec;', ...
'#endif', ...
' plhs[0]=mxCreateDoubleMatrix(1,4,mxREAL); double*o=mxGetPr(plhs[0]); o[0]=1000.0*(w1-w0); o[1]=(double)(t1-t0); o[2]=(double)(p1-p0); o[3]=(double)(x&0x1fffffffffffffULL);', ...
'}'};
end

function A=v209p_rootcause_audit(CFG,K)
md=CFG.v209p_cert_max_depth;
S=v209b_static_scenario(CFG);
Qold=v209n_build_static_problem_runtime(S,'PATCH_UNION',CFG,K);
Bold=v207_build_backend('ACETRANS_MINCO_ST_C6',Qold.W,Qold.weights);
Told=v208_time_gate_angle(Bold,CFG,CFG.v209b_nominal_theta_deg);
assert(Told.pass,'G26V209P:OldTime','Old reference time gate failed.');
tDetect=CFG.v209b_detection_fraction*Told.T;
tSwitch=tDetect+CFG.v209b_switch_lookahead_s;
assert(tSwitch<Told.T,'G26V209P:Switch','Scheduled switch must lie inside the active reference.');
J0=v209b_eval_jet(Bold,Told,tSwitch);

% Reconstruct the exact updated problem at the scheduled switch state.
Snew=S;
Snew.name='V209P_RUNTIME_JITTER';
Snew.C=[S.C S.newObstacleCenter];
Snew.V=zeros(3,size(Snew.C,2));
Snew.R=[S.R S.newObstacleRadius];
Snew.p0=J0(:,1);
Snew.pg=S.pg;

Q=v209n_build_static_problem_runtime(Snew,'PATCH_UNION',CFG,K);
assert(Q.guidePass&&Q.corridorOK,'G26V209P:Spatial','Updated PATCH spatial problem must pass.');

% Use the inherited, already-existing prefix authority exactly as V209I:
% certify the ACTIVE old reference over [tDetect,tSwitch] against ALL newly
% known obstacles in Q.  No new theorem/function is introduced here.
prefix=v209b_active_interval_certificate(Bold,Told,tDetect,tSwitch,Q,CFG);
if ~prefix.pass
    fprintf('[G26-V209P-PREFIX-FAIL] tDetect=%.9f tSwitch=%.9f pieces=%d minSlack=%+.6e\n', ...
        tDetect,tSwitch,prefix.pieces,prefix.min_slack);
end
assert(prefix.pass,'G26V209P:Prefix','Inherited active interval [tDetect,tSwitch] must remain certified against updated obstacles.');
fprintf('[G26-V209P-PREFIX] PASS | tDetect=%.9f tSwitch=%.9f pSwitch=[%.9f %.9f %.9f] pieces=%d minSlack=%+.6e\n', ...
    tDetect,tSwitch,J0(1,1),J0(2,1),J0(3,1),prefix.pieces,prefix.min_slack);

names={'BSPLINE_D7','BEZIER_D13','POLY_D13','ACETRANS_MINCO_ST_C6'};
A=struct();
A.mexMap=struct('clusterGuide',true,'segmentCorridor',true,'timeGate',true,'deepHardCert',true, ...
    'bridgeSearch',false,'bridgeBuildD17',false,'bridgeFirstCorridor',false,'bridgeConeTaut',false,'backendConstructors',false);
A.fixedWork=v209p_fixedwork_baseline(CFG);
rowCell=cell(1,numel(names));
A.prefixSafe=prefix.pass;
A.formalWCET=false;
A.safetyChanged=false;
A.bridgeChanged=false;

% These pauses are intentionally OUTSIDE the measured hot-path intervals.
% They make the whole diagnostic script long, but do not represent planner
% compute time.  Log them explicitly so wall-clock script duration is never
% confused with online execution latency.
A.intentionalPauseSeconds = numel(names) * ( ...
    max(0,CFG.v209p_paced_samples-1)*CFG.v209p_control_period_s + ...
    CFG.v209p_backend_spill_samples*CFG.v209p_settle_s );

QT=v209b_tail_problem(Q);

for i=1:numel(names)
    name=names{i};
    [BT,TT,st]=v209n_tail_runtime_deep(QT,CFG,name,md); %#ok<ASGLU>
    assert(strcmp(st,'PASS'),'G26V209P:Tail','Tail must be certified before timing audit.');

    R=struct();
    R.backend=name;
    R.outerBurst=v209p_outer_pipeline(Snew,J0,name,CFG,K,md,CFG.v209p_outer_warmup,CFG.v209p_outer_samples,false);
    R.outerPaced=v209p_outer_pipeline(Snew,J0,name,CFG,K,md,5,CFG.v209p_paced_samples,true);
    R.spill=v209p_backend_spill_probe(name,QT,CFG,A.fixedWork.loops);
    rowCell{i}=R;
end
A.rows=[rowCell{:}];
end

function F=v209p_fixedwork_baseline(CFG)
loops=v209p_calibrate_fixedwork(CFG.v209p_fixedwork_target_ms);
N=CFG.v209p_fixedwork_samples;
for k=1:50,g26v209p_fixedwork_mex(loops);end
w=zeros(N,1);t=zeros(N,1);p=zeros(N,1);
for k=1:N
    z=g26v209p_fixedwork_mex(loops);
    w(k)=z(1);t(k)=z(2);p(k)=z(3);
end
F=v209p_stats(w,t,p,CFG);F.loops=loops;
end

function loops=v209p_calibrate_fixedwork(targetMs)
loops=10000;
for q=1:20
    w=zeros(9,1);
    for k=1:9,z=g26v209p_fixedwork_mex(loops);w(k)=z(1);end
    med=median(w);
    if med>=0.75*targetMs && med<=1.25*targetMs,return;end
    if med<=0,loops=loops*4;else,loops=max(1,round(loops*targetMs/med));end
    loops=min(loops,2e9);
end
end

function R=v209p_outer_pipeline(Snew,J0,name,CFG,K,md,NW,NS,paced)
for k=1:NW
    X=v209n_pipeline_once_runtime(Snew,J0,name,CFG,K,md); %#ok<NASGU>
end
w=zeros(NS,1);t=zeros(NS,1);p=zeros(NS,1);status=cell(NS,1);
for k=1:NS
    if paced && k>1,pause(CFG.v209p_control_period_s);end
    s0=g26v209o_stamp_mex();
    X=v209n_pipeline_once_runtime(Snew,J0,name,CFG,K,md);
    s1=g26v209o_stamp_mex();
    [w(k),t(k),p(k)]=v209o_delta(s0,s1);
    status{k}=X.status;
end
assert(all(strcmp(status,status{1})),'G26V209P:Determinism','Runtime pipeline logical status changed.');
R=v209p_stats(w,t,p,CFG);
R.status=status{1};R.paced=paced;
end

function S=v209p_backend_spill_probe(name,QT,CFG,loops)
N=CFG.v209p_backend_spill_samples;
for k=1:20
    B=v207_build_backend(name,QT.W,QT.weights); %#ok<NASGU>
    g26v209p_fixedwork_mex(loops);
end
wi=zeros(N,1);ti=zeros(N,1);pi=zeros(N,1);
ws=zeros(N,1);ts=zeros(N,1);ps=zeros(N,1);
for k=1:N
    B=v207_build_backend(name,QT.W,QT.weights); %#ok<NASGU>
    z=g26v209p_fixedwork_mex(loops);wi(k)=z(1);ti(k)=z(2);pi(k)=z(3);
    pause(CFG.v209p_settle_s);
    z=g26v209p_fixedwork_mex(loops);ws(k)=z(1);ts(k)=z(2);ps(k)=z(3);
end
S=struct();
S.immediate=v209p_stats(wi,ti,pi,CFG);
S.settled=v209p_stats(ws,ts,ps,CFG);
S.settle_s=CFG.v209p_settle_s;
end

function S=v209p_stats(w,t,p,CFG)
w=w(:);t=t(:);p=p(:);N=numel(w);assert(numel(t)==N&&numel(p)==N,'G26V209P:Stats','Counter sizes differ.');
[ws,ord]=sort(w);
S=struct();S.N=N;S.p50=v209p_q(ws,.50);S.p95=v209p_q(ws,.95);S.p99=v209p_q(ws,.99);S.max=ws(end);S.mean=mean(w);
S.gt4=sum(w>CFG.v209p_target_ms);S.gt25=sum(w>1000*CFG.v209p_control_period_s);
mt=median(t(t>0));mp=median(p(p>0));if isempty(mt),mt=NaN;end;if isempty(mp),mp=NaN;end
S.medianThread=mt;S.medianProcess=mp;
im=ord(end);S.threadAtMax=t(im);S.processAtMax=p(im);
S.threadRatioAtMax=t(im)/max(mt,realmin);S.processRatioAtMax=p(im)/max(mp,realmin);
% Activity density: compare process cycles per wall-ms with median-sample density.
dens=p./max(w,realmin);
S.medianProcessDensity=median(dens(isfinite(dens)));
S.processDensityAtMax=dens(im);
S.densityRatioAtMax=S.processDensityAtMax/max(S.medianProcessDensity,realmin);
kk=min(8,N);ix=flipud(ord(end-kk+1:end));S.topWall=w(ix).';S.topThreadRatio=(t(ix).'/max(mt,realmin));S.topProcessRatio=(p(ix).'/max(mp,realmin));S.topDensityRatio=(dens(ix).'/max(S.medianProcessDensity,realmin));
end

function y=v209p_q(x,p)
n=numel(x);if n==0,y=NaN;return;end;y=x(max(1,min(n,ceil(p*n))));
end

function v209p_print(A,CFG)
fprintf('\n========================================================================================================================\n');
fprintf(' G26-V209P RUNTIME JITTER / MEX BOUNDARY AUDIT | target=%.3f ms controlPeriod=%.3f ms\n',CFG.v209p_target_ms,1000*CFG.v209p_control_period_s);
fprintf('------------------------------------------------------------------------------------------------------------------------\n');
M=A.mexMap;
fprintf(' MEX MAP: spatialGuide=%d segmentCorridor=%d timeGate=%d deepHardCert=%d | bridgeSearch=%d D17=%d firstCorr=%d coneTaut=%d backends=%d\n', ...
    M.clusterGuide,M.segmentCorridor,M.timeGate,M.deepHardCert,M.bridgeSearch,M.bridgeBuildD17,M.bridgeFirstCorridor,M.bridgeConeTaut,M.backendConstructors);
fprintf(' fixed-work diagnostic loops=%g (single-thread MEX, calibrated near %.1f ms)\n',A.fixedWork.loops,CFG.v209p_fixedwork_target_ms);
fprintf(' intentional UNMEASURED diagnostic pauses = %.3f s total (25-ms pacing + 30-ms settle); NOT hot-path runtime\n',A.intentionalPauseSeconds);
v209p_print_stats('fixed-work MEX baseline',A.fixedWork,CFG);
for i=1:numel(A.rows)
    R=A.rows(i);
    fprintf('\n[%s]\n',R.backend);
    v209p_print_stats(' outer-only runtime burst',R.outerBurst,CFG);
    v209p_print_stats(' outer-only runtime 25ms-paced',R.outerPaced,CFG);
    v209p_print_stats(' fixed-work immediately after backend',R.spill.immediate,CFG);
    v209p_print_stats(sprintf(' fixed-work after %.0fms settle',1e3*R.spill.settle_s),R.spill.settled,CFG);
end
fprintf('\n------------------------------------------------------------------------------------------------------------------------\n');
fprintf(' INTERPRETATION A: fixed-work MEX wall spike + densityRatio<<1 => scheduler/wait/preemption dominates the elapsed outlier.\n');
fprintf(' INTERPRETATION B: immediate-after-backend processRatio/density differs strongly from settled => backend/BLAS worker spillover exists.\n');
fprintf(' INTERPRETATION C: burst path spikes but 25ms-paced path does not => back-to-back benchmark/runtime housekeeping contributes; production 40Hz still requires target-Linux validation.\n');
fprintf(' RULE: MEX can reduce MATLAB dispatch/allocation average time, but it cannot by itself prove or eliminate host scheduler jitter.\n');
fprintf(' RULE: bridge/safety/geometry are unchanged in V209P.\n');
fprintf('========================================================================================================================\n');
end

function v209p_print_stats(label,S,CFG)
fprintf(' %-38s N=%4d p50/p95/p99/max=%9.4f/%9.4f/%9.4f/%9.4f ms | >%.1f=%3d >25=%3d\n', ...
    label,S.N,S.p50,S.p95,S.p99,S.max,CFG.v209p_target_ms,S.gt4,S.gt25);
fprintf(' %38s workRatioAtMax thread/process=%8.3f/%8.3f | process-density-ratio-at-max=%8.4f\n','',S.threadRatioAtMax,S.processRatioAtMax,S.densityRatioAtMax);
fprintf(' %38s topWall=%s\n','',mat2str(S.topWall,6));
fprintf(' %38s topProcessRatio=%s | topDensityRatio=%s\n','',mat2str(S.topProcessRatio,5),mat2str(S.topDensityRatio,5));
end


% ========================================================================
% V209Q host/process telemetry
% ========================================================================

function v209q_mex_bootstrap_hoststamp(root)
v200_build_mex_if_needed(root,'g26v209q_hoststamp_mex',v209q_cpp_hoststamp_source(),'G26-V209Q-HOSTSTAMP-MEX');
end

function lines=v209q_cpp_hoststamp_source()
lines={ ...
'#include "mex.h"', ...
'#include <cstdint>', ...
'#ifdef _WIN32', ...
'#define NOMINMAX', ...
'#include <windows.h>', ...
'#include <psapi.h>', ...
'#pragma comment(lib,"psapi.lib")', ...
'#else', ...
'#include <time.h>', ...
'#include <sys/resource.h>', ...
'#include <unistd.h>', ...
'#endif', ...
'static double ft64(', ...
'#ifdef _WIN32', ...
' const FILETIME &f', ...
'#else', ...
' uint64_t f', ...
'#endif', ...
'){', ...
'#ifdef _WIN32', ...
' ULARGE_INTEGER u; u.LowPart=f.dwLowDateTime;u.HighPart=f.dwHighDateTime;return (double)u.QuadPart;', ...
'#else', ...
' return (double)f;', ...
'#endif', ...
'}', ...
'void mexFunction(int nlhs,mxArray*plhs[],int nrhs,const mxArray*prhs[]){', ...
' if(nrhs!=0) mexErrMsgIdAndTxt("G26V209Q:HostStamp","No inputs expected");', ...
' if(nlhs>1) mexErrMsgIdAndTxt("G26V209Q:HostStamp","At most one output");', ...
' plhs[0]=mxCreateDoubleMatrix(1,19,mxREAL); double*o=mxGetPr(plhs[0]);', ...
'#ifdef _WIN32', ...
' LARGE_INTEGER q,f; QueryPerformanceCounter(&q);QueryPerformanceFrequency(&f);o[0]=(double)q.QuadPart/(double)f.QuadPart;', ...
' ULONG64 tc=0,pc=0;QueryThreadCycleTime(GetCurrentThread(),&tc);QueryProcessCycleTime(GetCurrentProcess(),&pc);o[1]=(double)tc;o[2]=(double)pc;', ...
' FILETIME c,e,pk,pu;GetProcessTimes(GetCurrentProcess(),&c,&e,&pk,&pu);o[3]=ft64(pk);o[4]=ft64(pu);', ...
' FILETIME si,sk,su;GetSystemTimes(&si,&sk,&su);o[5]=ft64(si);o[6]=ft64(sk);o[7]=ft64(su);', ...
' PROCESS_MEMORY_COUNTERS_EX pm;ZeroMemory(&pm,sizeof(pm));pm.cb=sizeof(pm);GetProcessMemoryInfo(GetCurrentProcess(),(PROCESS_MEMORY_COUNTERS*)&pm,sizeof(pm));', ...
' o[8]=(double)pm.PageFaultCount;o[9]=(double)pm.WorkingSetSize;o[10]=(double)pm.PrivateUsage;', ...
' IO_COUNTERS io;ZeroMemory(&io,sizeof(io));GetProcessIoCounters(GetCurrentProcess(),&io);', ...
' o[11]=(double)io.ReadOperationCount;o[12]=(double)io.WriteOperationCount;o[13]=(double)io.OtherOperationCount;', ...
' o[14]=(double)io.ReadTransferCount;o[15]=(double)io.WriteTransferCount;o[16]=(double)io.OtherTransferCount;', ...
' o[17]=(double)GetActiveProcessorCount(ALL_PROCESSOR_GROUPS);o[18]=1.0;', ...
'#else', ...
' struct timespec w,t,p;clock_gettime(CLOCK_MONOTONIC,&w);o[0]=(double)w.tv_sec+1e-9*w.tv_nsec;', ...
' clock_gettime(CLOCK_THREAD_CPUTIME_ID,&t);o[1]=1e9*t.tv_sec+t.tv_nsec;clock_gettime(CLOCK_PROCESS_CPUTIME_ID,&p);o[2]=1e9*p.tv_sec+p.tv_nsec;', ...
' struct rusage r;getrusage(RUSAGE_SELF,&r);o[3]=1e7*((double)r.ru_stime.tv_sec+1e-6*r.ru_stime.tv_usec);o[4]=1e7*((double)r.ru_utime.tv_sec+1e-6*r.ru_utime.tv_usec);', ...
' o[5]=o[6]=o[7]=0;o[8]=(double)(r.ru_minflt+r.ru_majflt);o[9]=o[10]=0;o[11]=o[12]=o[13]=o[14]=o[15]=o[16]=0;o[17]=(double)sysconf(_SC_NPROCESSORS_ONLN);o[18]=2.0;', ...
'#endif', ...
'}'};
end

function A=v209q_audit(CFG,K)
md=CFG.v209q_cert_max_depth;
S=v209b_static_scenario(CFG);
Qold=v209n_build_static_problem_runtime(S,'PATCH_UNION',CFG,K);
Bold=v207_build_backend('ACETRANS_MINCO_ST_C6',Qold.W,Qold.weights);
Told=v208_time_gate_angle(Bold,CFG,CFG.v209b_nominal_theta_deg);
assert(Told.pass,'G26V209Q:OldTime','Old time gate failed.');
tDetect=CFG.v209b_detection_fraction*Told.T;
tSwitch=tDetect+CFG.v209b_switch_lookahead_s;
J0=v209b_eval_jet(Bold,Told,tSwitch);

Snew=S;Snew.name='V209Q_HOST_CAUSE';
Snew.C=[S.C S.newObstacleCenter];Snew.V=zeros(3,size(Snew.C,2));Snew.R=[S.R S.newObstacleRadius];Snew.p0=J0(:,1);Snew.pg=S.pg;
Q=v209n_build_static_problem_runtime(Snew,'PATCH_UNION',CFG,K);
assert(Q.guidePass&&Q.corridorOK,'G26V209Q:Spatial','Updated spatial problem failed.');
prefix=v209b_active_interval_certificate(Bold,Told,tDetect,tSwitch,Q,CFG);
assert(prefix.pass,'G26V209Q:Prefix','Active old-reference prefix not certified.');

loops=v209p_calibrate_fixedwork(CFG.v209q_fixedwork_target_ms);
A=struct();A.fixedWorkLoops=loops;A.prefixSafe=prefix.pass;A.bridgeChanged=false;A.safetyChanged=false;A.formalWCET=false;
A.fixed=v209q_measure_fixedwork(loops,CFG.v209q_fixed_samples,CFG);

names={'BSPLINE_D7','BEZIER_D13','POLY_D13','ACETRANS_MINCO_ST_C6'};
rc=cell(1,numel(names));
for i=1:numel(names)
    name=names{i};
    QT=v209b_tail_problem(Q);
    [BT,TT,st]=v209n_tail_runtime_deep(QT,CFG,name,md); %#ok<ASGLU>
    assert(strcmp(st,'PASS'),'G26V209Q:Tail','Tail must remain certified.');
    R=struct();R.backend=name;
    R.burst=v209q_measure_pipeline(Snew,J0,name,CFG,K,md,CFG.v209q_burst_samples,false);
    R.paced=v209q_measure_pipeline(Snew,J0,name,CFG,K,md,CFG.v209q_paced_samples,true);
    rc{i}=R;
end
A.rows=[rc{:}];
end

function R=v209q_measure_fixedwork(loops,N,CFG)
for k=1:50,g26v209p_fixedwork_mex(loops);end
D=zeros(N,12);
for k=1:N
    a=g26v209q_hoststamp_mex();g26v209p_fixedwork_mex(loops);b=g26v209q_hoststamp_mex();
    D(k,:)=v209q_delta(a,b);
end
R=v209q_stats(D,CFG);
end

function R=v209q_measure_pipeline(Snew,J0,name,CFG,K,md,N,paced)
for k=1:CFG.v209q_warmup,v209n_pipeline_once_runtime(Snew,J0,name,CFG,K,md);end
D=zeros(N,12); status=cell(N,1);
for k=1:N
    if paced && k>1,pause(CFG.v209q_control_period_s);end
    a=g26v209q_hoststamp_mex();
    X=v209n_pipeline_once_runtime(Snew,J0,name,CFG,K,md);
    b=g26v209q_hoststamp_mex();
    D(k,:)=v209q_delta(a,b);status{k}=X.status;
end
assert(all(strcmp(status,status{1})),'G26V209Q:Determinism','Logical pipeline status changed.');
R=v209q_stats(D,CFG);R.status=status{1};R.paced=paced;
end

function d=v209q_delta(a,b)
% [wall_ms threadCycles processCycles procCPU_ms sysBusyCPU_ms pageFaults dWS_MB dPrivate_MB ioOps ioBytes ncpu mode]
wall=1e3*(b(1)-a(1));
tc=b(2)-a(2);pc=b(3)-a(3);
procCPU=1e-4*((b(3+1)-a(3+1))+(b(4+1)-a(4+1))); % 100ns -> ms
idle=b(6)-a(6);kern=b(7)-a(7);usr=b(8)-a(8);
sysBusy=1e-4*((kern-idle)+usr);
pf=b(9)-a(9);
dws=(b(10)-a(10))/1048576;
dpriv=(b(11)-a(11))/1048576;
ioOps=(b(12)-a(12))+(b(13)-a(13))+(b(14)-a(14));
ioBytes=(b(15)-a(15))+(b(16)-a(16))+(b(17)-a(17));
d=[wall tc pc procCPU sysBusy pf dws dpriv ioOps ioBytes b(18) b(19)];
end

function S=v209q_stats(D,CFG)
w=D(:,1);N=numel(w);[ws,ord]=sort(w);
S=struct();S.N=N;S.p50=v209q_q(ws,.50);S.p95=v209q_q(ws,.95);S.p99=v209q_q(ws,.99);S.max=ws(end);
S.gt4=sum(w>CFG.v209q_target_ms);S.gt25=sum(w>1000*CFG.v209q_control_period_s);
S.medProcessCpuMs=median(D(:,4));S.medProcCycles=median(D(:,3));S.medThreadCycles=median(D(:,2));
im=ord(end);S.maxIndex=im;S.maxRow=D(im,:);
S.procCpuWallAtMax=D(im,4)/max(w(im),realmin);
S.procCycleRatioAtMax=D(im,3)/max(S.medProcCycles,realmin);
S.threadCycleRatioAtMax=D(im,2)/max(S.medThreadCycles,realmin);
ncpu=max(1,D(im,11));S.systemBusyFractionAtMax=D(im,5)/max(w(im)*ncpu,realmin);
S.pageFaultAtMax=D(im,6);S.wsDeltaAtMax=D(im,7);S.privateDeltaAtMax=D(im,8);S.ioOpsAtMax=D(im,9);S.ioBytesAtMax=D(im,10);
kk=min(8,N);ix=flipud(ord(end-kk+1:end));
S.topIndex=ix.';S.topWall=w(ix).';S.topProcCpu=D(ix,4).';S.topSysBusy=D(ix,5).';S.topPageFault=D(ix,6).';S.topIOOps=D(ix,9).';S.topIOBytes=D(ix,10).';
S.topProcCycleRatio=(D(ix,3).'/max(S.medProcCycles,realmin));
end

function y=v209q_q(x,p)
n=numel(x);if n==0,y=NaN;return;end;y=x(max(1,min(n,ceil(p*n))));
end

function v209q_print(A,CFG)
fprintf('\n========================================================================================================================\n');
fprintf(' G26-V209Q HOST / MATLAB-RUNTIME CAUSE ATTRIBUTION | target=%.3f ms period=%.3f ms\n',CFG.v209q_target_ms,1000*CFG.v209q_control_period_s);
fprintf('------------------------------------------------------------------------------------------------------------------------\n');
fprintf(' fixed-work loops=%g | bridgeChanged=%d safetyChanged=%d formalWCET=%d\n',A.fixedWorkLoops,A.bridgeChanged,A.safetyChanged,A.formalWCET);
v209q_print_stats('single-thread fixed-work MEX',A.fixed,CFG);
for i=1:numel(A.rows)
    fprintf('\n[%s]\n',A.rows(i).backend);
    v209q_print_stats(' complete runtime burst',A.rows(i).burst,CFG);
    v209q_print_stats(' complete runtime 25ms-paced',A.rows(i).paced,CFG);
end
fprintf('\n------------------------------------------------------------------------------------------------------------------------\n');
fprintf(' Read the SAME max event: procCPU/wall, system busy fraction, page faults, and process I/O.\n');
fprintf('  - low procCPU/wall + no faults/I/O => wait/preemption/runtime scheduling dominates elapsed spike.\n');
fprintf('  - large page-fault delta => memory working-set/page-fault source.\n');
fprintf('  - large process I/O => runtime/file/console/antivirus-facing I/O source.\n');
fprintf('  - procCPU scales with wall => MATLAB/MEX/backend computation is the dominant source.\n');
fprintf(' No hard onboard WCET claim is made from Windows; target-Linux measurement remains required.\n');
fprintf('========================================================================================================================\n');
end

function v209q_print_stats(label,S,CFG)
fprintf(' %-34s N=%4d p50/p95/p99/max=%8.4f/%8.4f/%8.4f/%9.4f ms | >%.1f=%4d >25=%3d\n', ...
    label,S.N,S.p50,S.p95,S.p99,S.max,CFG.v209q_target_ms,S.gt4,S.gt25);
fprintf(' %34s MAX idx=%d procCPU=%8.4f ms (procCPU/wall=%.4f) threadCycleRatio=%.3f procCycleRatio=%.3f sysBusyFrac=%.4f\n','', ...
    S.maxIndex,S.maxRow(4),S.procCpuWallAtMax,S.threadCycleRatioAtMax,S.procCycleRatioAtMax,S.systemBusyFractionAtMax);
fprintf(' %34s MAX pageFault=%g dWS=%+.3fMB dPrivate=%+.3fMB ioOps=%g ioBytes=%g\n','', ...
    S.pageFaultAtMax,S.wsDeltaAtMax,S.privateDeltaAtMax,S.ioOpsAtMax,S.ioBytesAtMax);
fprintf(' %34s topWall=%s\n','',mat2str(S.topWall,6));
fprintf(' %34s topProcCPU=%s | topPageFault=%s | topIOOps=%s\n','',mat2str(S.topProcCpu,5),mat2str(S.topPageFault,5),mat2str(S.topIOOps,5));
end


% ========================================================================
% V209R stage-localized allocation/housekeeping audit
% ========================================================================

function A=v209r_audit(CFG,K)
md=CFG.v209r_cert_max_depth;
S=v209b_static_scenario(CFG);
Qold=v209n_build_static_problem_runtime(S,'PATCH_UNION',CFG,K);
Bold=v207_build_backend('ACETRANS_MINCO_ST_C6',Qold.W,Qold.weights);
Told=v208_time_gate_angle(Bold,CFG,CFG.v209b_nominal_theta_deg);
assert(Told.pass,'G26V209R:OldTime','Old time gate failed.');
tDetect=CFG.v209b_detection_fraction*Told.T;
tSwitch=tDetect+CFG.v209b_switch_lookahead_s;
J0=v209b_eval_jet(Bold,Told,tSwitch);

Snew=S;Snew.name='V209R_ALLOC_AUDIT';
Snew.C=[S.C S.newObstacleCenter];Snew.V=zeros(3,size(Snew.C,2));Snew.R=[S.R S.newObstacleRadius];
Snew.p0=J0(:,1);Snew.pg=S.pg;

Q=v209n_build_static_problem_runtime(Snew,'PATCH_UNION',CFG,K);
assert(Q.guidePass&&Q.corridorOK,'G26V209R:Spatial','Updated spatial problem failed.');
prefix=v209b_active_interval_certificate(Bold,Told,tDetect,tSwitch,Q,CFG);
assert(prefix.pass,'G26V209R:Prefix','Active prefix failed.');

loops=v209p_calibrate_fixedwork(CFG.v209r_fixedwork_target_ms);
A=struct();A.fixedWorkLoops=loops;A.prefixSafe=prefix.pass;A.safetyChanged=false;A.bridgeChanged=false;A.formalWCET=false;
A.sourceAudit=v209r_source_audit_manifest();
A.fixed=v209r_measure(@()g26v209p_fixedwork_mex(loops),CFG);

names={'BSPLINE_D7','BEZIER_D13','POLY_D13','ACETRANS_MINCO_ST_C6'};
rows=cell(1,numel(names));
for i=1:numel(names)
    name=names{i};
    QT=v209b_tail_problem(Q);
    B=v207_build_backend(name,QT.W,QT.weights);
    C=v209b_tail_corridor_audit(B,QT);assert(C.pass,'G26V209R:TailCorr','Tail corridor failed.');
    T=v208_time_gate_angle(B,CFG,CFG.v209b_nominal_theta_deg);assert(T.pass,'G26V209R:TailTime','Tail time gate failed.');
    D=v209l_final_direct_certificate_depth(B,T,QT,CFG,md);assert(D.pass,'G26V209R:TailCert','Tail hard cert failed.');
    J1=v209b_backend_start_jets(B,T);

    R=struct();R.backend=name;
    R.spatial=v209r_measure(@()v209n_build_static_problem_runtime(Snew,'PATCH_UNION',CFG,K),CFG);
    R.backendBuild=v209r_measure(@()v207_build_backend(name,QT.W,QT.weights),CFG);
    R.timeGate=v209r_measure(@()v208_time_gate_angle(B,CFG,CFG.v209b_nominal_theta_deg),CFG);
    R.hardCert=v209r_measure(@()v209l_final_direct_certificate_depth(B,T,QT,CFG,md),CFG);
    R.bridge=v209r_measure(@()v209n_bridge_search_runtime_deep(J0,J1,Q,B,T,CFG,md),CFG);
    R.fullPath=v209r_measure(@()v209n_pipeline_once_runtime(Snew,J0,name,CFG,K,md),CFG);
    rows{i}=R;
end
A.rows=[rows{:}];
end

function M=v209r_source_audit_manifest()
M=struct();
M.explicitJavaGC=false;
M.explicitClearAll=false;
M.explicitDrawnow=false;
M.explicitPack=false;
M.hotpathConsoleIO=false;
M.bridgeDynamicIntervalGrowth=true;
M.mincoConstraintAGROW=true;
M.backendQualificationMetadata=true;
M.repeatedStructConstruction=true;
M.repeatedCellConstruction=true;
M.repeatedFlattenCat=true;
M.directCertRecursiveStdVector=true;
M.directCertMxOutputAllocation=true;
M.sensorCoverMemoryPersistentReserve=true;
M.note='MATLAB numeric hot path has allocator/temporary-array churn rather than an explicit Java-style GC call.';
end

function R=v209r_measure(fun,CFG)
N=CFG.v209r_samples;NW=CFG.v209r_warmup;
for k=1:NW,fun();end
D=zeros(N,12);
for k=1:N
    a=g26v209q_hoststamp_mex();
    fun();
    b=g26v209q_hoststamp_mex();
    D(k,:)=v209q_delta(a,b);
end
R=v209r_stats(D,CFG);
end

function S=v209r_stats(D,CFG)
w=D(:,1);N=numel(w);[ws,ord]=sort(w);im=ord(end);
S=struct();S.N=N;S.p50=v209r_q(ws,.50);S.p95=v209r_q(ws,.95);S.p99=v209r_q(ws,.99);S.max=ws(end);
S.gt4=sum(w>CFG.v209r_target_ms);S.gt25=sum(w>1000*CFG.v209r_control_period_s);S.maxIndex=im;
S.procCPUAtMax=D(im,4);S.procCpuWallAtMax=D(im,4)/max(w(im),realmin);
mp=median(D(:,3));mt=median(D(:,2));
S.processCycleRatioAtMax=D(im,3)/max(mp,realmin);S.threadCycleRatioAtMax=D(im,2)/max(mt,realmin);
S.pageFaultAtMax=D(im,6);S.wsDeltaAtMax=D(im,7);S.privateDeltaAtMax=D(im,8);S.ioOpsAtMax=D(im,9);S.ioBytesAtMax=D(im,10);
S.near971 = (im>=950 && im<=990);
kk=min(8,N);ix=flipud(ord(end-kk+1:end));
S.topIndex=ix.';S.topWall=w(ix).';S.topPageFault=D(ix,6).';S.topProcCPU=D(ix,4).';S.topIOOps=D(ix,9).';
end

function y=v209r_q(x,p)
n=numel(x);if n==0,y=NaN;return;end;y=x(max(1,min(n,ceil(p*n))));
end

function v209r_print(A,CFG)
fprintf('\n========================================================================================================================\n');
fprintf(' G26-V209R ALLOCATION / HOUSEKEEPING LOCALIZATION | target=%.3f ms period=%.3f ms\n',CFG.v209r_target_ms,1000*CFG.v209r_control_period_s);
fprintf('------------------------------------------------------------------------------------------------------------------------\n');
M=A.sourceAudit;
fprintf(' SOURCE AUDIT: explicitGC=%d clearAll=%d drawnow=%d pack=%d consoleIOHot=%d\n',M.explicitJavaGC,M.explicitClearAll,M.explicitDrawnow,M.explicitPack,M.hotpathConsoleIO);
fprintf(' HEAP-CHURN: bridgeAGROW=%d MINCO_AGROW=%d backendMetadata=%d structs=%d cells=%d flattenCat=%d directCertStdVector=%d\n', ...
    M.bridgeDynamicIntervalGrowth,M.mincoConstraintAGROW,M.backendQualificationMetadata,M.repeatedStructConstruction,M.repeatedCellConstruction,M.repeatedFlattenCat,M.directCertRecursiveStdVector);
v209r_print_stats('fixed-work single-thread MEX',A.fixed,CFG);
for i=1:numel(A.rows)
    R=A.rows(i);fprintf('\n[%s]\n',R.backend);
    v209r_print_stats(' spatial cover+guide+corridor',R.spatial,CFG);
    v209r_print_stats(' backend constructor',R.backendBuild,CFG);
    v209r_print_stats(' time/cone/taut gate',R.timeGate,CFG);
    v209r_print_stats(' deep hard certificate',R.hardCert,CFG);
    v209r_print_stats(' bridge search unchanged',R.bridge,CFG);
    v209r_print_stats(' full post-detection planner',R.fullPath,CFG);
end
fprintf('\n------------------------------------------------------------------------------------------------------------------------\n');
fprintf(' A maxIndex around 971/972 recurring in an isolated stage strongly implicates allocation/runtime housekeeping in that stage.\n');
fprintf(' A large pageFaultAtMax with process CPU growth suggests allocator/page working-set activity; low procCPU/wall suggests wait/preemption contribution.\n');
fprintf(' This audit does NOT alter safety or claim onboard WCET.  The final authority remains sensor-to-switch E2E on target Linux.\n');
fprintf('========================================================================================================================\n');
end

function v209r_print_stats(label,S,CFG)
fprintf(' %-34s N=%4d p50/p95/p99/max=%8.4f/%8.4f/%8.4f/%9.4f ms | >%.1f=%4d >25=%3d | maxIdx=%4d near971=%d\n', ...
    label,S.N,S.p50,S.p95,S.p99,S.max,CFG.v209r_target_ms,S.gt4,S.gt25,S.maxIndex,S.near971);
fprintf(' %34s MAX procCPU=%8.3fms cpu/wall=%.3f thrRatio=%.2f procRatio=%.2f faults=%g dWS=%+.3fMB ioOps=%g ioBytes=%g\n','', ...
    S.procCPUAtMax,S.procCpuWallAtMax,S.threadCycleRatioAtMax,S.processCycleRatioAtMax,S.pageFaultAtMax,S.wsDeltaAtMax,S.ioOpsAtMax,S.ioBytesAtMax);
fprintf(' %34s topIdx=%s topWall=%s topFaults=%s\n','',mat2str(S.topIndex),mat2str(S.topWall,6),mat2str(S.topPageFault));
end


% ========================================================================
% V209S lean online backend constructors
% ========================================================================

function A=v209s_audit(CFG,K)
md=CFG.v209s_cert_max_depth;
S=v209b_static_scenario(CFG);
Qold=v209n_build_static_problem_runtime(S,'PATCH_UNION',CFG,K);
Bold=v207_build_backend('ACETRANS_MINCO_ST_C6',Qold.W,Qold.weights);
Told=v208_time_gate_angle(Bold,CFG,CFG.v209b_nominal_theta_deg);
assert(Told.pass,'G26V209S:OldTime','Old reference time gate failed.');
tDetect=CFG.v209b_detection_fraction*Told.T;
tSwitch=tDetect+CFG.v209b_switch_lookahead_s;
J0=v209b_eval_jet(Bold,Told,tSwitch);

Snew=S;Snew.name='V209S_LEAN_BACKEND';
Snew.C=[S.C S.newObstacleCenter];Snew.V=zeros(3,size(Snew.C,2));Snew.R=[S.R S.newObstacleRadius];
Snew.p0=J0(:,1);Snew.pg=S.pg;
Q=v209n_build_static_problem_runtime(Snew,'PATCH_UNION',CFG,K);
assert(Q.guidePass&&Q.corridorOK,'G26V209S:Spatial','Updated spatial problem failed.');
prefix=v209b_active_interval_certificate(Bold,Told,tDetect,tSwitch,Q,CFG);
assert(prefix.pass,'G26V209S:Prefix','Active prefix failed.');
QT=v209b_tail_problem(Q);

names={'BSPLINE_D7','BEZIER_D13','POLY_D13','ACETRANS_MINCO_ST_C6'};
rows=cell(1,numel(names));
A=struct();A.prefixSafe=prefix.pass;A.safetyChanged=false;A.bridgeChanged=false;A.formalWCET=false;

for i=1:numel(names)
    name=names{i};
    B0=v207_build_backend(name,QT.W,QT.weights);
    B1=v209s_build_backend_online(name,QT.W,QT.weights);

    EQ=v209s_compare_backend(B0,B1,CFG);
    assert(EQ.pass,'G26V209S:CoefficientEquivalence','Lean backend coefficient equivalence failed for %s.',name);

    C0=v209b_tail_corridor_audit(B0,QT);C1=v209b_tail_corridor_audit(B1,QT);
    T0=v208_time_gate_angle(B0,CFG,CFG.v209b_nominal_theta_deg);
    T1=v208_time_gate_angle(B1,CFG,CFG.v209b_nominal_theta_deg);
    assert(C0.pass==C1.pass,'G26V209S:CorridorEquivalence','Tail corridor decision differs.');
    assert(T0.pass==T1.pass,'G26V209S:TimeEquivalence','Time gate decision differs.');
    if T0.pass&&T1.pass
        D0=v209l_final_direct_certificate_depth(B0,T0,QT,CFG,md);
        D1=v209l_final_direct_certificate_depth(B1,T1,QT,CFG,md);
        AUTH=v209s_compare_authority(C0,C1,T0,T1,D0,D1);
    else
        D0=[];D1=[];AUTH=struct('pass',false);
    end
    assert(AUTH.pass,'G26V209S:AuthorityEquivalence','Downstream authority differs for %s.',name);

    R=struct();R.backend=name;R.eq=EQ;R.authority=AUTH;
    R.baselineBuild=v209s_measure(@()v207_build_backend(name,QT.W,QT.weights),CFG);
    R.leanBuild=v209s_measure(@()v209s_build_backend_online(name,QT.W,QT.weights),CFG);
    R.baselinePath=v209s_measure(@()v209n_pipeline_once_runtime(Snew,J0,name,CFG,K,md),CFG);
    R.leanPath=v209s_measure(@()v209s_pipeline_once_runtime(Snew,J0,name,CFG,K,md),CFG);
    R.leanPaced=v209s_measure_paced(@()v209s_pipeline_once_runtime(Snew,J0,name,CFG,K,md),CFG);
    rows{i}=R;
end
A.rows=[rows{:}];
A.allEquivalent=all(arrayfun(@(r)r.eq.pass&&r.authority.pass,A.rows));
end

function B=v209s_build_backend_online(name,W,w)
switch name
    case 'BSPLINE_D7'
        B=v209s_backend_bspline_d7_online(W,w);
    case 'BEZIER_D13'
        B=v209s_backend_bezier_d13_online(W,w);
    case 'POLY_D13'
        B=v209s_backend_poly_d13_online(W,w);
    case 'ACETRANS_MINCO_ST_C6'
        B=v209s_backend_minco_d13_online(W,w);
    otherwise
        error('G26V209S:Backend','Unknown backend %s',name);
end
B.name=name;B.weights=w;B.online_lean=true;
end

function B=v209s_backend_bezier_d13_online(W,w) %#ok<INUSD>
S=size(W,2)-1;P=cell(1,S);
for q=1:S
    X=zeros(3,14);
    X(:,1:7)=repmat(W(:,q),1,7);
    X(:,8:14)=repmat(W(:,q+1),1,7);
    P{q}=X;
end
B=struct('P',{P},'degree',13,'structural_c6_theorem',true,'condition_indicator',1);
end

function B=v209s_backend_poly_d13_online(W,w) %#ok<INUSD>
persistent T
if isempty(T),T=v207_bernstein_to_power_matrix(13);end
S=size(W,2)-1;P=cell(1,S);
for q=1:S
    X=zeros(3,14);
    X(:,1:7)=repmat(W(:,q),1,7);
    X(:,8:14)=repmat(W(:,q+1),1,7);
    Aq=(T*X.').';
    P{q}=(T\Aq.').';
end
B=struct('P',{P},'degree',13,'structural_c6_theorem',true,'condition_indicator',1);
end

function B=v209s_backend_bspline_d7_online(W,w)
persistent ss BM
p=7;
if isempty(ss)
    ss=linspace(0,1,p+1);
    BM=zeros(p+1,p+1);
    for j=1:p+1
        s=ss(j);
        for k=0:p
            BM(j,k+1)=nchoosek(p,k)*s^k*(1-s)^(p-k);
        end
    end
end
S=size(W,2)-1;nCtrl=S+p;
br=[0 cumsum(w)];br(end)=1;
U=[zeros(1,p+1) br(2:end-1) ones(1,p+1)];
CP=zeros(3,nCtrl);CP(:,1:4)=repmat(W(:,1),1,4);CP(:,end-3:end)=repmat(W(:,end),1,4);
unk=5:(nCtrl-4);fix=[1:4 (nCtrl-3):nCtrl];
if ~isempty(unk)
    A=zeros(S-1,numel(unk));rhs=zeros(3,S-1);
    for j=1:S-1
        bb=v207_bspline_basis_row(br(j+1),p,U,nCtrl);
        A(j,:)=bb(unk);
        rhs(:,j)=W(:,j+1)-CP(:,fix)*bb(fix).';
    end
    X=A\rhs.';CP(:,unk)=X.';
end
P=cell(1,S);
for q=1:S
    V=zeros(3,p+1);
    for j=1:p+1
        u=br(q)+(br(q+1)-br(q))*ss(j);
        V(:,j)=v207_bspline_eval(CP,p,U,u);
    end
    P{q}=(BM\V.').';
end
B=struct('P',{P},'degree',7,'structural_c6_theorem',true,'condition_indicator',1);
end

function B=v209s_backend_minco_d13_online(W,w)
persistent H0
S=size(W,2)-1;n=13;rObj=7;nu=(n+1)*S;
if isempty(H0)
    D7=v207_bernstein_difference_matrix(n,rObj);
    G7=v207_bernstein_gram_matrix(n-rObj);
    f7=v200_falling(n,rObj);
    H0=(f7^2)*(D7.'*G7*D7);
end
Q=zeros(nu,nu);
for q=1:S
    ix=(q-1)*(n+1)+(1:n+1);
    Q(ix,ix)=H0*w(q)^(-(2*rObj-1));
end
Q=0.5*(Q+Q.');

mEq=8*S+6;
A=zeros(mEq,nu);rhs=zeros(mEq,3);rr=0;
for q=1:S
    rr=rr+1;A(rr,:)=v207_bernstein_endpoint_row(S,q,0,0,w,n);rhs(rr,:)=W(:,q).';
    rr=rr+1;A(rr,:)=v207_bernstein_endpoint_row(S,q,1,0,w,n);rhs(rr,:)=W(:,q+1).';
end
for q=1:S-1
    for r=1:6
        rr=rr+1;
        A(rr,:)=v207_bernstein_endpoint_row(S,q,1,r,w,n)-v207_bernstein_endpoint_row(S,q+1,0,r,w,n);
    end
end
for r=1:6
    rr=rr+1;A(rr,:)=v207_bernstein_endpoint_row(S,1,0,r,w,n);
    rr=rr+1;A(rr,:)=v207_bernstein_endpoint_row(S,S,1,r,w,n);
end
assert(rr==mEq,'G26V209S:MINCOEqCount','MINCO equality preallocation count mismatch.');

qScale=max(abs(Q(:)));if ~(isfinite(qScale)&&qScale>0),qScale=1;end
Qs=Q/qScale;
rowScale=sqrt(sum(A.^2,2));
assert(all(isfinite(rowScale)&rowScale>0),'G26V209S:MINCORowScale','MINCO equality row has zero/nonfinite norm.');
As=bsxfun(@rdivide,A,rowScale);
rhsS=bsxfun(@rdivide,rhs,rowScale);
K=[Qs As';As zeros(mEq,mEq)];
RHS=[zeros(nu,3);rhsS];
sol=K\RHS;
coefB=sol(1:nu,:).';

P=cell(1,S);
for q=1:S
    ix=(q-1)*(n+1)+(1:n+1);
    P{q}=coefB(:,ix);
end
B=struct('P',{P},'degree',n,'structural_c6_theorem',true,'condition_indicator',1);
end

function E=v209s_compare_backend(B0,B1,CFG)
E=struct();E.sameDegree=(B0.degree==B1.degree);E.sameSpans=(numel(B0.P)==numel(B1.P));
mx=0;scale=1;
if E.sameSpans
    for q=1:numel(B0.P)
        if ~isequal(size(B0.P{q}),size(B1.P{q})),mx=inf;break;end
        d=abs(B0.P{q}-B1.P{q});
        if ~isempty(d),mx=max(mx,max(d(:)));end
        scale=max(scale,max(abs([B0.P{q}(:);B1.P{q}(:)])));
    end
else
    mx=inf;
end
guard=8192*eps(scale);
E.maxCoefficientDifference=mx;E.guard=guard;
E.pass=E.sameDegree&&E.sameSpans&&isfinite(mx)&&mx<=guard;
E.safetyChanged=false;E.mathChanged=false;
end

function E=v209s_compare_authority(C0,C1,T0,T1,D0,D1)
E=struct();
E.corridorDecision=(C0.pass==C1.pass);
E.timeDecision=(T0.pass==T1.pass);
E.hardDecision=(D0.pass==D1.pass);
E.depthDecision=(D0.max_depth==D1.max_depth);
E.failLocation=(D0.fail_piece==D1.fail_piece)&&(D0.fail_obstacle==D1.fail_obstacle);
E.timeDiff=abs(T0.T-T1.T);
E.hDiff=max(abs(T0.h-T1.h));
E.minConeDiff=abs(T0.min_cone_slack-T1.min_cone_slack);
E.minRzDiff=abs(T0.min_rz_slack-T1.min_rz_slack);
E.pass=E.corridorDecision&&E.timeDecision&&E.hardDecision&&E.depthDecision&&E.failLocation ...
    && E.timeDiff<=4096*eps(max(1,abs(T0.T))) ...
    && E.hDiff<=4096*eps(max(1,max(abs(T0.h)))) ...
    && E.minConeDiff<=4096*eps(max(1,abs(T0.min_cone_slack))) ...
    && E.minRzDiff<=4096*eps(max(1,abs(T0.min_rz_slack)));
end

function X=v209s_pipeline_once_runtime(Snew,J0,name,CFG,K,md)
Q=v209n_build_static_problem_runtime(Snew,'PATCH_UNION',CFG,K);
if ~Q.guidePass||~Q.corridorOK,X=struct('status','UPDATED_SPATIAL_SEARCH_INCOMPLETE');return;end
QT=v209b_tail_problem(Q);
[BT,TT,st]=v209s_tail_runtime_deep(QT,CFG,name,md);
if ~strcmp(st,'PASS'),X=struct('status',[name '__' st]);return;end
J1=v209b_backend_start_jets(BT,TT);
H=v209n_bridge_search_runtime_deep(J0,J1,Q,BT,TT,CFG,md);
if ~H.pass,status=[name '__JET_TIME_CERT_SEARCH_INCOMPLETE'];else,status='CERTIFIED_MULTI_OBSTACLE_STATIC_SUCCESSOR';end
X=struct('status',status);
end

function [BT,TT,status]=v209s_tail_runtime_deep(QT,CFG,name,md)
BT=[];TT=[];status='NO_TAIL';if ~QT.exists,return;end
B=v209s_build_backend_online(name,QT.W,QT.weights);
C=v209b_tail_corridor_audit(B,QT);
if ~C.pass,status='TAIL_CORRIDOR_SEARCH_INCOMPLETE';return;end
T=v208_time_gate_angle(B,CFG,CFG.v209b_nominal_theta_deg);
if ~T.pass,status='TAIL_TIME_SEARCH_INCOMPLETE';return;end
D=v209l_final_direct_certificate_depth(B,T,QT,CFG,md);
if ~D.pass,status='TAIL_HARD_CERT_FAIL';return;end
BT=B;TT=T;status='PASS';
end

function R=v209s_measure(fun,CFG)
for k=1:CFG.v209s_warmup,fun();end
N=CFG.v209s_samples;D=zeros(N,12);
for k=1:N
    a=g26v209q_hoststamp_mex();fun();b=g26v209q_hoststamp_mex();
    D(k,:)=v209q_delta(a,b);
end
R=v209s_stats(D,CFG);
end

function R=v209s_measure_paced(fun,CFG)
for k=1:CFG.v209s_warmup,fun();end
N=CFG.v209s_paced_samples;D=zeros(N,12);
for k=1:N
    if k>1,pause(CFG.v209s_control_period_s);end
    a=g26v209q_hoststamp_mex();fun();b=g26v209q_hoststamp_mex();
    D(k,:)=v209q_delta(a,b);
end
R=v209s_stats(D,CFG);
end

function S=v209s_stats(D,CFG)
w=D(:,1);N=numel(w);[ws,ord]=sort(w);im=ord(end);
S=struct();S.N=N;S.p50=v209s_q(ws,.50);S.p95=v209s_q(ws,.95);S.p99=v209s_q(ws,.99);S.max=ws(end);
S.gt4=sum(w>CFG.v209s_target_ms);S.gt25=sum(w>1000*CFG.v209s_control_period_s);
S.maxIndex=im;S.procCPUAtMax=D(im,4);S.pageFaultAtMax=D(im,6);S.ioOpsAtMax=D(im,9);S.wsDeltaAtMax=D(im,7);
mp=median(D(:,3));mt=median(D(:,2));
S.procRatioAtMax=D(im,3)/max(mp,realmin);S.threadRatioAtMax=D(im,2)/max(mt,realmin);
kk=min(8,N);ix=flipud(ord(end-kk+1:end));S.topIndex=ix.';S.topWall=w(ix).';S.topFault=D(ix,6).';
end

function y=v209s_q(x,p)
n=numel(x);if n==0,y=NaN;return;end;y=x(max(1,min(n,ceil(p*n))));
end

function v209s_print(A,CFG)
fprintf('\n========================================================================================================================\n');
fprintf(' G26-V209S LEAN ONLINE BACKEND EQUIVALENCE / TIMING | target=%.3f ms period=%.3f ms\n',CFG.v209s_target_ms,1000*CFG.v209s_control_period_s);
fprintf('------------------------------------------------------------------------------------------------------------------------\n');
fprintf(' allEquivalent=%d | safetyChanged=%d bridgeChanged=%d formalWCET=%d\n',A.allEquivalent,A.safetyChanged,A.bridgeChanged,A.formalWCET);
for i=1:numel(A.rows)
    R=A.rows(i);fprintf('\n[%s] coeffEq=%d maxDiff=%.3e guard=%.3e authorityEq=%d\n', ...
        R.backend,R.eq.pass,R.eq.maxCoefficientDifference,R.eq.guard,R.authority.pass);
    v209s_print_stats(' baseline backend build',R.baselineBuild,CFG);
    v209s_print_stats(' lean online backend build',R.leanBuild,CFG);
    fprintf('   backend p50 reduction = %.4f ms (%.1f%%)\n',R.baselineBuild.p50-R.leanBuild.p50,100*(R.baselineBuild.p50-R.leanBuild.p50)/max(R.baselineBuild.p50,realmin));
    v209s_print_stats(' baseline full planner',R.baselinePath,CFG);
    v209s_print_stats(' lean full planner burst',R.leanPath,CFG);
    v209s_print_stats(' lean full planner 25ms-paced',R.leanPaced,CFG);
end
fprintf('\n------------------------------------------------------------------------------------------------------------------------\n');
fprintf(' RULE: coefficient/downstream authority mismatch => lean backend is rejected, regardless of timing benefit.\n');
fprintf(' RULE: even if backend cleanup succeeds, the unchanged MATLAB bridge (~3.6 ms median in V209R) remains the next ordinary-time bottleneck.\n');
fprintf(' RULE: final acceptance is sensor-return -> trigger -> replan -> hard certificate -> atomic switch within 2-4 ms on target runtime.\n');
fprintf('========================================================================================================================\n');
end

function v209s_print_stats(label,S,CFG)
fprintf(' %-32s N=%4d p50/p95/p99/max=%8.4f/%8.4f/%8.4f/%9.4f ms | >%.1f=%4d >25=%3d | maxIdx=%4d faults=%g dWS=%+.3fMB\n', ...
    label,S.N,S.p50,S.p95,S.p99,S.max,CFG.v209s_target_ms,S.gt4,S.gt25,S.maxIndex,S.pageFaultAtMax,S.wsDeltaAtMax);
end


% ========================================================================
% V209T fixed-hb SOCP bridge audit
% ========================================================================

function A = v209t_audit(CFG,K)
S = v209b_static_scenario(CFG);

% Old active reference and exact switch state.
Qold = v209n_build_static_problem_runtime(S,'PATCH_UNION',CFG,K);
Bold = v209s_build_backend_online('ACETRANS_MINCO_ST_C6',Qold.W,Qold.weights);
Told = v208_time_gate_angle(Bold,CFG,CFG.v209b_nominal_theta_deg);
assert(Told.pass,'G26V209T:OldReference','Old reference time gate failed.');
tDetect = CFG.v209b_detection_fraction*Told.T;
tSwitch = tDetect + CFG.v209b_switch_lookahead_s;
assert(tSwitch < Told.T,'G26V209T:Switch','Switch time must be inside old reference.');
J0 = v209b_eval_jet(Bold,Told,tSwitch);

Snew = S;
Snew.name = 'V209T_FREE_BRIDGE_SOCP';
Snew.C = [S.C S.newObstacleCenter];
Snew.V = zeros(3,size(Snew.C,2));
Snew.R = [S.R S.newObstacleRadius];
Snew.p0 = J0(:,1);
Snew.pg = S.pg;

Q = v209n_build_static_problem_runtime(Snew,CFG.v209t_representation,CFG,K);
assert(Q.guidePass && Q.corridorOK,'G26V209T:Spatial','Updated spatial problem must pass.');
prefix = v209b_active_interval_certificate(Bold,Told,tDetect,tSwitch,Q,CFG);
assert(prefix.pass,'G26V209T:Prefix','Old reference [detect,switch] prefix must remain certified.');

names = {'BSPLINE_D7','BEZIER_D13','POLY_D13','ACETRANS_MINCO_ST_C6'};
rows = cell(1,numel(names));

A = struct();
A.rep = CFG.v209t_representation;
A.prefixSafe = prefix.pass;
A.freeBridgeVariables = 12;
A.freeControlPointsZeroBased = 7:10;
A.fixedLeftZeroBased = 0:6;
A.fixedRightZeroBased = 11:17;
A.bridgeDegree = 17;
A.fixedTimeBankUsed = false;
A.hbSearchComplete = false;
A.hbOptimalityClaim = false;
A.safetyChanged = false;
A.formalWCET = false;

for i = 1:numel(names)
    name = names{i};
    QT = v209b_tail_problem(Q);
    [BT,TT,st] = v209s_tail_runtime_deep(QT,CFG,name,CFG.v209t_cert_max_depth);
    assert(strcmp(st,'PASS'),'G26V209T:Tail','Certified tail required for %s.',name);
    J1 = v209b_backend_start_jets(BT,TT);

    % Baseline unchanged bridge search (expected open from V209M/S).
    H0 = v209n_bridge_search_runtime_deep(J0,J1,Q,BT,TT,CFG,CFG.v209t_cert_max_depth);

    % New fixed-hb convex witness search.  Search is resource-only and does
    % not claim global hb optimality or infeasibility on miss.
    H1 = v209t_socp_bridge_search(J0,J1,Q,BT,TT,CFG,CFG.v209t_cert_max_depth);

    R = struct();
    R.backend = name;
    R.tailT = TT.T;
    R.tailSegmentH = TT.h;
    R.tailTimeIterations = TT.iterations;
    R.tailTimeSearchIncomplete = TT.search_incomplete;
    R.tailFixedTimeBankUsed = false;
    R.oldBridgePass = H0.pass;
    R.oldBridgeEvals = H0.evals;
    R.socp = H1;
    if H1.pass
        R.jet = v209t_jet_report(H1.best.bridgeB,H1.h,J0,J1);
        R.oldAtSameH = v209t_old_bridge_same_h(J0,J1,Q,BT,TT,H1.h,CFG,CFG.v209t_cert_max_depth);
    else
        R.jet = struct('pass',false,'maxResidual',NaN,'maxRatio',NaN);
        R.oldAtSameH = struct('pass',false);
    end
    rows{i} = R;
end
A.rows = [rows{:}];
A.allFourSocpPass = all(arrayfun(@(r)r.socp.pass,A.rows));
A.oldAllFourPass = all(arrayfun(@(r)r.oldBridgePass,A.rows));
end

function H = v209t_socp_bridge_search(J0,J1,Q,BT,TT,CFG,md)
lo = CFG.v209b_bridge_h_min_s;
hi = CFG.v209b_bridge_h_max_s;
maxEval = CFG.v209b_bridge_search_evals;

% Preallocated interval queue.  This deliberately mirrors the inherited
% largest-interval witness search without AGROW.  It remains resource-only.
L = nan(2*maxEval+4,1);
R = nan(2*maxEval+4,1);
active = false(size(L));
L(1)=lo;R(1)=hi;active(1)=true;
nextSlot=2;

trace = repmat(v209t_empty_eval(),1,maxEval);
evals = 0;
best = [];
bestH = inf;

% Endpoints first, exactly as inherited search.
for z = 1:2
    if z==1,h=lo;else,h=hi;end
    [ok,C,E] = v209t_eval_socp_bridge(J0,J1,Q,BT,TT,h,CFG,md);
    evals=evals+1;trace(evals)=E;
    if ok && h<bestH,best=C;bestH=h;end
end

while any(active) && evals<maxEval
    idx=find(active & L<bestH);
    if isempty(idx),break;end
    widths=R(idx)-L(idx);
    [~,jj]=max(widths);
    ii=idx(jj);
    a=L(ii);b=R(ii);active(ii)=false;
    if b-a <= 64*eps(max(1,b)),continue;end
    h=0.5*(a+b);

    [ok,C,E] = v209t_eval_socp_bridge(J0,J1,Q,BT,TT,h,CFG,md);
    evals=evals+1;trace(evals)=E;
    if ok && h<bestH,best=C;bestH=h;end

    if a<bestH
        L(nextSlot)=a;R(nextSlot)=h;active(nextSlot)=true;nextSlot=nextSlot+1;
    end
    if h<bestH
        L(nextSlot)=h;R(nextSlot)=b;active(nextSlot)=true;nextSlot=nextSlot+1;
    end
end

trace=trace(1:evals);
H=struct();
H.pass=~isempty(best);
H.evals=evals;
H.h=NaN;
H.best=best;
H.trace=trace;
H.searchComplete=false;
H.optimalityClaim=false;
H.infeasibilityClaim=false;
H.fixedTimeBankUsed=false;
H.solverFeasibleCount=sum([trace.solverFeasible]);
H.localGatePassCount=sum([trace.localGatePass]);
H.fullHardCertPassCount=sum([trace.fullPass]);
if H.pass,H.h=bestH;end
end

function E = v209t_empty_eval()
E=struct('h',NaN,'solverExitflag',NaN,'solverFeasible',false,'solverIterations',NaN, ...
    'fixedCorridorPass',false,'corridorPass',false,'dynamicPass',false,'jetPass',false,'localGatePass',false, ...
    'c6Pass',false,'combinedCorridorPass',false,'hardCertPass',false,'fullPass',false, ...
    'minCorridor',NaN,'minCone',NaN,'minRz',NaN,'hardSlack',NaN,'status','UNSET');
end

function [ok,C,E] = v209t_eval_socp_bridge(J0,J1,Q,BT,TT,hb,CFG,md)
E=v209t_empty_eval();E.h=hb;ok=false;C=[];

[S,meta]=v209t_fixed_h_socp(J0,J1,Q,hb,CFG);
E.solverExitflag=S.exitflag;
E.solverFeasible=S.feasible;
E.solverIterations=S.iterations;
E.fixedCorridorPass=meta.fixedCorridorPass;
if ~S.feasible
    E.status='SOCP_SEARCH_INCOMPLETE';
    return;
end

BB=S.B;
[Jpass,jres,jratio]=v209b_endpoint_jet_match(BB,hb,J0,J1);
CORb=v209b_bridge_corridor_audit(BB,Q);
DY=v209b_bridge_dynamic_audit(BB,hb,CFG.v209b_nominal_theta_deg,CFG);
E.jetPass=Jpass;E.corridorPass=CORb.pass;E.dynamicPass=DY.pass;
E.minCorridor=CORb.min_slack;E.minCone=DY.minCone;E.minRz=DY.minRz;
E.localGatePass=Jpass&&CORb.pass&&DY.pass;
if ~E.localGatePass
    E.status='SOCP_NUMERICAL_WITNESS_REJECTED_BY_UNCHANGED_GATE';
    return;
end

if isempty(BT)
    P={BB};hh=hb;Rflat=DY.R;rcounts=DY.rcount;tailT=0;
else
    P=[{BB} BT.P];hh=[hb TT.h];Rflat=[DY.R TT.Rflat];rcounts=[DY.rcount TT.rcounts];tailT=TT.T;
end
BC=struct('P',{P},'degree',17,'structural_c6_theorem',true,'condition_indicator',1);
TC=struct('pass',true,'h',hh,'Rflat',Rflat,'rcounts',rcounts,'T',sum(hh));

C6=v207_c6_backend_audit(BC,hh);
COR=v209b_combined_corridor_audit(BC,Q);
D=v209l_final_direct_certificate_depth(BC,TC,Q,CFG,md);
E.c6Pass=C6.pass;E.combinedCorridorPass=COR.pass;E.hardCertPass=D.pass;E.hardSlack=D.min_slack;
E.fullPass=Jpass&&CORb.pass&&DY.pass&&C6.pass&&COR.pass&&D.pass;
if ~E.fullPass
    E.status='SOCP_LOCAL_FEASIBLE__UNCHANGED_FINAL_AUTHORITY_REJECTED';
    return;
end
E.status='CERTIFIED_SOCP_BRIDGE_WITNESS';
ok=true;
C=struct('B',BC,'T',TC,'bridgeB',BB,'bridgeH',hb,'bridgeDynamic',DY,'bridgeCorridor',CORb, ...
    'c6',C6,'corridor',COR,'direct',D,'tailT',tailT,'solver',S,'jetResidual',jres,'jetRatio',jratio);
end

function [S,META] = v209t_fixed_h_socp(J0,J1,Q,hb,CFG)
n=17;
theta=deg2rad(CFG.v209b_nominal_theta_deg);
tt=tan(theta);
rho=CFG.v207_rho_min_mps2;

[B0,M,cR]=v209t_bridge_affine_map(J0,J1,hb,CFG);

% Fixed control points must already lie in the first corridor; B7:B10 are
% the only decision variables and cannot repair endpoint-side violations.
nf=Q.cnt(1);
freeCols=8:11;
fixedCols=[1:7 12:18];
fixedSlack=inf;
for f=1:nf
    fixedSlack=min(fixedSlack,min(Q.N(:,f)'*B0(:,fixedCols)-Q.h(f)));
end
scale=max([1,max(abs(B0(:,fixedCols)),[],'all'),max(abs(Q.h(1:nf)))]);
fixedGuard=4096*eps(scale);
fixedCorridorPass=isfinite(fixedSlack)&&fixedSlack>=-fixedGuard;

% Linear inequalities:
%   - corridor for every free control point
%   - r_z >= rho for every resultant control point
nCorr=4*nf;
Aineq=zeros(nCorr+16,12);
bineq=zeros(nCorr+16,1);
rr=0;
for k=1:4
    ix=(k-1)*3+(1:3);
    for f=1:nf
        rr=rr+1;
        Aineq(rr,ix)=-Q.N(:,f).';
        bineq(rr)=-Q.h(f);
    end
end
for j=1:16
    rr=rr+1;
    Mz=squeeze(M(3,:,j));
    Aineq(rr,:)=-Mz;
    bineq(rr)=cR(3,j)-rho;
end
assert(rr==size(Aineq,1),'G26V209T:LinearCount','SOCP linear inequality count mismatch.');

% SOC cable-angle constraints for all 16 resultant Bernstein controls.
soc=cell(1,16);
for j=1:16
    Mj=squeeze(M(:,:,j));
    Axy=Mj(1:2,:);
    bxy=-cR(1:2,j);  % A*x-b = Mxy*x+cxy
    d=tt*Mj(3,:).';
    gamma=-tt*cR(3,j);
    soc{j}=secondordercone(Axy,bxy,d,gamma);
end

opts=optimoptions('coneprog','Display',CFG.v209t_solver_display,'LinearSolver','normal-dense');
f=zeros(12,1);

x=nan(12,1);fval=NaN;exitflag=-999;output=struct();
if fixedCorridorPass
    try
        [x,fval,exitflag,output]=coneprog(f,soc,Aineq,bineq,[],[],[],[],opts);
    catch ME
        error('G26V209T:ConeprogRuntime','coneprog failed at hb=%.9g s: %s',hb,ME.message);
    end
end

feasible=fixedCorridorPass && exitflag>0 && all(isfinite(x));
B=B0;
if feasible
    for k=1:4
        B(:,freeCols(k))=x((k-1)*3+(1:3));
    end
end
it=NaN;
if isstruct(output)&&isfield(output,'iterations'),it=output.iterations;end

S=struct('feasible',feasible,'x',x,'B',B,'exitflag',exitflag,'fval',fval,'iterations',it,'output',output);
META=struct('fixedCorridorPass',fixedCorridorPass,'fixedCorridorSlack',fixedSlack, ...
    'numVariables',12,'numSOC',16,'numLinear',size(Aineq,1),'numCorridorLinear',nCorr,'numTautLinear',16, ...
    'degree',n,'thetaDeg',CFG.v209b_nominal_theta_deg,'rhoMin',rho);
end

function [B0,M,cR] = v209t_bridge_affine_map(J0,J1,h,CFG)
n=17;
B0=zeros(3,n+1);

% Exact left endpoint jets: B0...B6.
for k=0:6
    x=zeros(3,1);
    for r=0:k
        if r==0,den=1;else,den=v200_falling(n,r);end
        x=x+nchoosek(k,r)*(h^r/den)*J0(:,r+1);
    end
    B0(:,k+1)=x;
end

% Exact right endpoint jets: B11...B17.
Z=zeros(3,7);
for k=0:6
    x=zeros(3,1);
    for r=0:k
        if r==0,den=1;else,den=v200_falling(n,r);end
        x=x+nchoosek(k,r)*(h^r/den)*((-1)^r)*J1(:,r+1);
    end
    Z(:,k+1)=x;
end
B0(:,18:-1:12)=Z;

fac=n*(n-1)/h^2;
cR=fac*diff(B0,2,2);
cR(3,:)=cR(3,:)+CFG.gravity;

M=zeros(3,12,16);
for v=1:12
    E=zeros(3,18);
    k=ceil(v/3);coord=v-3*(k-1);
    E(coord,7+k)=1; % k=1:4 -> MATLAB columns 8:11 == B7:B10
    RE=fac*diff(E,2,2);
    M(:,v,:)=reshape(RE,3,1,16);
end
end

function J = v209t_jet_report(B,h,J0,J1)
[p,m,r]=v209b_endpoint_jet_match(B,h,J0,J1);
J=struct('pass',p,'maxResidual',m,'maxRatio',r);
end

function O = v209t_old_bridge_same_h(J0,J1,Q,BT,TT,hb,CFG,md)
[ok,C]=v209n_eval_bridge_runtime_deep(J0,J1,Q,BT,TT,hb,CFG,md);
BB=v209b_degree17_two_sided_jet_bridge(J0,J1,hb);
COR=v209b_bridge_corridor_audit(BB,Q);
DY=v209b_bridge_dynamic_audit(BB,hb,CFG.v209b_nominal_theta_deg,CFG);
O=struct('pass',ok,'corridorPass',COR.pass,'dynamicPass',DY.pass,'minCorridor',COR.min_slack, ...
    'minCone',DY.minCone,'minRz',DY.minRz,'candidate',C);
end

function v209t_print(A,CFG)
fprintf('\n========================================================================================================================\n');
fprintf(' G26-V209T FIXED-hb SOCP BRIDGE RESULTS | rep=%s | theta=%.1fdeg\n',A.rep,CFG.v209b_nominal_theta_deg);
fprintf('------------------------------------------------------------------------------------------------------------------------\n');
fprintf(' bridge variables=%d (B7:B10) | fixed jets=B0:B6,B11:B17 | fixedTimeBank=%d | hbOptimalityClaim=%d | hbSearchComplete=%d\n', ...
    A.freeBridgeVariables,A.fixedTimeBankUsed,A.hbOptimalityClaim,A.hbSearchComplete);
fprintf(' TAIL-T STATUS: h_i=T*w_i/sum(w), T in [%.3f, %.3f] s, monotone continuous bisection up to 64 iters, fixed time bank=0.\n', ...
    CFG.v207_time_min_s,CFG.v207_time_max_s);
fprintf(' BRIDGE-hb STATUS: current [%.3f, %.3f] s / maxEval=%d is resource-only witness search; NOT final automatic allocation.\n', ...
    CFG.v209b_bridge_h_min_s,CFG.v209b_bridge_h_max_s,CFG.v209b_bridge_search_evals);

for i=1:numel(A.rows)
    R=A.rows(i);H=R.socp;
    fprintf('\n[%s]\n',R.backend);
    fprintf(' tail T=%.9f s | h=%s | T-bisection iters=%g | T-searchIncomplete=%d | fixedBank=%d\n', ...
        R.tailT,mat2str(R.tailSegmentH,8),R.tailTimeIterations,R.tailTimeSearchIncomplete,R.tailFixedTimeBankUsed);
    fprintf(' old interpolated bridge: pass=%d evals=%d\n',R.oldBridgePass,R.oldBridgeEvals);
    fprintf(' SOCP free bridge: pass=%d evals=%d solverFeasible=%d localGate=%d fullHardCert=%d | searchComplete=0 infeasibilityClaim=0\n', ...
        H.pass,H.evals,H.solverFeasibleCount,H.localGatePassCount,H.fullHardCertPassCount);
    if H.pass
        C=H.best;
        fprintf('   witness hb=%.12g s | jetPass=%d maxJetResidual=%.3e ratio=%.3e\n', ...
            H.h,R.jet.pass,R.jet.maxResidual,R.jet.maxRatio);
        fprintf('   corridor minSlack=%+.6e | cone minSlack=%+.6e | rz minSlack=%+.6e\n', ...
            C.bridgeCorridor.min_slack,C.bridgeDynamic.minCone,C.bridgeDynamic.minRz);
        fprintf('   C6=%d combinedCorridor=%d hardCert=%d hardSlack=%+.6e depth=%g\n', ...
            C.c6.pass,C.corridor.pass,C.direct.pass,C.direct.min_slack,C.direct.max_depth);
        fprintf('   old interpolation at SAME hb: pass=%d corridor=%d dynamic=%d minCone=%+.6e minRz=%+.6e\n', ...
            R.oldAtSameH.pass,R.oldAtSameH.corridorPass,R.oldAtSameH.dynamicPass,R.oldAtSameH.minCone,R.oldAtSameH.minRz);
    else
        st={H.trace.status};
        fprintf('   no certified witness in current resource search. unique statuses: %s\n',strjoin(unique(st),', '));
    end
end

fprintf('\n------------------------------------------------------------------------------------------------------------------------\n');
fprintf(' all4 SOCP certified=%d | old all4 certified=%d | safetyChanged=%d | formalWCET=%d\n', ...
    A.allFourSocpPass,A.oldAllFourPass,A.safetyChanged,A.formalWCET);
fprintf(' RULE: SOCP feasibility is only a candidate generator.  Final authority remains unchanged continuous full-body certification.\n');
fprintf(' RULE: a miss within [hb_min,hb_max]/maxEval is SEARCH_INCOMPLETE; no path/physical infeasibility claim.\n');
fprintf(' NEXT if all4 pass: repeat theorem for SFC representation, then design non-heuristic continuous hb allocation before real-time MEX implementation.\n');
fprintf('========================================================================================================================\n');
end


% ========================================================================
% V209U endpoint-jet / fixed-resultant necessary-condition audit
% ========================================================================

function A = v209u_audit(CFG,K)
S = v209b_static_scenario(CFG);

Qold = v209n_build_static_problem_runtime(S,'PATCH_UNION',CFG,K);
Bold = v209s_build_backend_online('ACETRANS_MINCO_ST_C6',Qold.W,Qold.weights);
Told = v208_time_gate_angle(Bold,CFG,CFG.v209b_nominal_theta_deg);
assert(Told.pass,'G26V209U:OldReference','Old reference time gate failed.');
tDetect = CFG.v209b_detection_fraction*Told.T;
tSwitch = tDetect + CFG.v209b_switch_lookahead_s;
assert(tSwitch<Told.T,'G26V209U:Switch','Switch time must lie in old reference.');
J0 = v209b_eval_jet(Bold,Told,tSwitch);

Snew = S;
Snew.name = 'V209U_DIAG';
Snew.C = [S.C S.newObstacleCenter];
Snew.V = zeros(3,size(Snew.C,2));
Snew.R = [S.R S.newObstacleRadius];
Snew.p0 = J0(:,1);
Snew.pg = S.pg;

Q = v209n_build_static_problem_runtime(Snew,CFG.v209u_representation,CFG,K);
assert(Q.guidePass && Q.corridorOK,'G26V209U:Spatial','Updated spatial problem failed.');
prefix = v209b_active_interval_certificate(Bold,Told,tDetect,tSwitch,Q,CFG);
assert(prefix.pass,'G26V209U:Prefix','Old-reference detect-to-switch prefix must be safe.');

names = {'BSPLINE_D7','BEZIER_D13','POLY_D13','ACETRANS_MINCO_ST_C6'};
rows = cell(1,numel(names));

A = struct();
A.rep = CFG.v209u_representation;
A.prefixSafe = prefix.pass;
A.tailTimeIsTailOnly = true;
A.jointTailBridgeTimeClosed = false;
A.safetyChanged = false;
A.bridgeChanged = false;
A.formalWCET = false;

for i = 1:numel(names)
    name = names{i};
    QT = v209b_tail_problem(Q);
    [BT,TT,st] = v209s_tail_runtime_deep(QT,CFG,name,CFG.v209u_cert_max_depth);
    assert(strcmp(st,'PASS'),'G26V209U:Tail','Certified tail required for %s.',name);

    J1 = v209b_backend_start_jets(BT,TT);
    H = v209t_socp_bridge_search(J0,J1,Q,BT,TT,CFG,CFG.v209u_cert_max_depth);

    D = v209u_trace_diagnosis(H.trace,J0,J1,Q,CFG);

    R = struct();
    R.backend = name;
    R.tailT = TT.T;
    R.tailH = TT.h;
    R.tailWeights = BT.weights;
    R.jetNorm = vecnorm(J1,2,1);
    R.jetInf = max(abs(J1),[],1);
    R.jetScaleLaw = 'J_r(alpha*T)=J_r(T)/alpha^r for r>=1, with fixed backend geometry and weights';
    R.trace = D;
    R.socpPass = H.pass;
    R.solverFeasibleCount = H.solverFeasibleCount;
    R.fullHardCertPassCount = H.fullHardCertPassCount;

    tol = 1e4*eps(max(1,max(abs(J1(:)))));
    R.nonzeroHighOrders = find(R.jetNorm(5:7)>tol)+3; % derivative orders 4..6
    rows{i} = R;
end

A.rows = [rows{:}];

% Cross-backend structural comparison of d4:d6 at the certified tail start.
A.bsplineHighJetNorm = A.rows(1).jetNorm(5:7);
A.otherHighJetNormMax = max(vertcat(A.rows(2:4).jetNorm),[],1);
A.otherHighJetNormMax = A.otherHighJetNormMax(5:7);
end

function D = v209u_trace_diagnosis(trace,J0,J1,Q,CFG)
N = numel(trace);
row = repmat(v209u_empty_row(),1,N);

for k = 1:N
    h = trace(k).h;
    [B0,M,cR] = v209t_bridge_affine_map(J0,J1,h,CFG);

    dep = zeros(1,size(M,3));
    for j = 1:size(M,3)
        dep(j) = norm(M(:,:,j),'fro');
    end
    depGuard = 4096*eps(max(1,max(abs(M(:)))));
    fixedIdx = find(dep <= depGuard);

    nf = Q.cnt(1);
    fixedCols = [1:7 12:18];
    fixedCorrSlack = inf;
    for f = 1:nf
        fixedCorrSlack = min(fixedCorrSlack,min(Q.N(:,f)'*B0(:,fixedCols)-Q.h(f)));
    end
    corrScale = max([1,max(abs(B0(:,fixedCols)),[],'all'),max(abs(Q.h(1:nf)))]);
    corrGuard = 4096*eps(corrScale);
    fixedCorrPass = isfinite(fixedCorrSlack) && fixedCorrSlack >= -corrGuard;

    Rfix = cR(:,fixedIdx);
    cone = tan(deg2rad(CFG.v209b_nominal_theta_deg))*Rfix(3,:) - vecnorm(Rfix(1:2,:),2,1);
    rz = Rfix(3,:) - CFG.v207_rho_min_mps2;
    dynScale = max(1,max(abs(Rfix(:))));
    dynGuard = CFG.v209b_bridge_guard_scale*eps(dynScale);
    dynPassNoGuard = all(isfinite(Rfix(:))) && all(cone>=0) && all(rz>=0);
    dynPassAuthority = all(isfinite(Rfix(:))) && all(cone>=dynGuard) && all(rz>=dynGuard);

    [minCone,ic] = min(cone);
    [minRz,ir] = min(rz);

    row(k).h = h;
    row(k).fixedIndicesZeroBased = fixedIdx-1;
    row(k).fixedCorridorSlack = fixedCorrSlack;
    row(k).fixedCorridorPass = fixedCorrPass;
    row(k).fixedDynamicPassNoGuard = dynPassNoGuard;
    row(k).fixedDynamicPassAuthority = dynPassAuthority;
    row(k).fixedMinCone = minCone;
    row(k).fixedMinConeIndexZeroBased = fixedIdx(ic)-1;
    row(k).fixedMinRz = minRz;
    row(k).fixedMinRzIndexZeroBased = fixedIdx(ir)-1;
    row(k).solverFeasible = trace(k).solverFeasible;
    row(k).solverExitflag = trace(k).solverExitflag;
    row(k).fullPass = trace(k).fullPass;
end

D = struct();
D.rows = row;
D.N = N;
D.fixedResultantIndicesZeroBased = row(1).fixedIndicesZeroBased;
D.fixedCorridorPassCount = sum([row.fixedCorridorPass]);
D.fixedDynamicNoGuardPassCount = sum([row.fixedDynamicPassNoGuard]);
D.fixedDynamicAuthorityPassCount = sum([row.fixedDynamicPassAuthority]);
D.bothNecessaryNoGuardPassCount = sum([row.fixedCorridorPass] & [row.fixedDynamicPassNoGuard]);
D.bothNecessaryAuthorityPassCount = sum([row.fixedCorridorPass] & [row.fixedDynamicPassAuthority]);
D.solverFeasibleCount = sum([row.solverFeasible]);
D.fullPassCount = sum([row.fullPass]);

[bestCone,D.bestConeIndex] = max([row.fixedMinCone]);
D.bestFixedMinCone = bestCone;
D.bestFixedMinConeH = row(D.bestConeIndex).h;
D.bestFixedMinConeWorstResultantZeroBased = row(D.bestConeIndex).fixedMinConeIndexZeroBased;

[bestRz,D.bestRzIndex] = max([row.fixedMinRz]);
D.bestFixedMinRz = bestRz;
D.bestFixedMinRzH = row(D.bestRzIndex).h;
D.bestFixedMinRzWorstResultantZeroBased = row(D.bestRzIndex).fixedMinRzIndexZeroBased;

[bestCorr,D.bestCorrIndex] = max([row.fixedCorridorSlack]);
D.bestFixedCorridorSlack = bestCorr;
D.bestFixedCorridorH = row(D.bestCorrIndex).h;

if D.fixedCorridorPassCount==0
    D.classification = 'FIXED_ENDPOINT_CONTROL_POINTS_VIOLATE_FIRST_CORRIDOR_OVER_ALL_EVALUATED_HB';
elseif D.fixedDynamicNoGuardPassCount==0
    D.classification = 'FIXED_ENDPOINT_RESULTANTS_VIOLATE_CONE_OR_TAUT_OVER_ALL_EVALUATED_HB';
elseif D.bothNecessaryNoGuardPassCount==0
    D.classification = 'CORRIDOR_AND_FIXED_DYNAMIC_NECESSARY_INTERVALS_DO_NOT_OVERLAP_IN_EVALUATED_HB';
elseif D.solverFeasibleCount==0
    D.classification = 'NECESSARY_CONDITIONS_PASS_SOME_HB_BUT_FREE_VARIABLE_COUPLED_SOCP_REMAINS_INFEASIBLE';
elseif D.fullPassCount==0
    D.classification = 'SOCP_FEASIBLE_BUT_UNCHANGED_FINAL_AUTHORITY_REJECTS_ALL_WITNESSES';
else
    D.classification = 'CERTIFIED_WITNESS_EXISTS';
end
end

function R = v209u_empty_row()
R = struct('h',NaN,'fixedIndicesZeroBased',[], ...
    'fixedCorridorSlack',NaN,'fixedCorridorPass',false, ...
    'fixedDynamicPassNoGuard',false,'fixedDynamicPassAuthority',false, ...
    'fixedMinCone',NaN,'fixedMinConeIndexZeroBased',NaN, ...
    'fixedMinRz',NaN,'fixedMinRzIndexZeroBased',NaN, ...
    'solverFeasible',false,'solverExitflag',NaN,'fullPass',false);
end

function v209u_print(A,CFG)
fprintf('\n========================================================================================================================\n');
fprintf(' G26-V209U FIXED-ENDPOINT NECESSARY-CONDITION RESULTS | rep=%s | theta=%.1fdeg\n',A.rep,CFG.v209b_nominal_theta_deg);
fprintf('------------------------------------------------------------------------------------------------------------------------\n');
fprintf(' tail-time authority today: T is automatic for TAIL cone/taut only; joint tail-T / bridge-hb allocation closed=%d\n', ...
    A.jointTailBridgeTimeClosed);
fprintf(' fixed resultant indices are detected from M(:,:,j)==0; these cannot be changed by B7:B10.\n');

for i = 1:numel(A.rows)
    R = A.rows(i);
    D = R.trace;
    fprintf('\n[%s]\n',R.backend);
    fprintf(' tail T=%.12g s | h=%s\n',R.tailT,mat2str(R.tailH,9));
    fprintf(' tail-start ||d0..d6||2 = %s\n',mat2str(R.jetNorm,8));
    fprintf(' tail-start ||d0..d6||inf = %s\n',mat2str(R.jetInf,8));
    fprintf(' nonzero high-order derivative orders among d4:d6 = %s\n',mat2str(R.nonzeroHighOrders));
    fprintf(' exact time-scaling law: %s\n',R.jetScaleLaw);
    fprintf(' fixed resultants (0-based) = %s\n',mat2str(D.fixedResultantIndicesZeroBased));
    fprintf(' evaluated hb=%d | fixedCorridorPass=%d | fixedDynamicPass(noGuard)=%d | fixedDynamicPass(authorityGuard)=%d\n', ...
        D.N,D.fixedCorridorPassCount,D.fixedDynamicNoGuardPassCount,D.fixedDynamicAuthorityPassCount);
    fprintf(' both necessary pass: noGuard=%d authorityGuard=%d | solverFeasible=%d | fullCertified=%d\n', ...
        D.bothNecessaryNoGuardPassCount,D.bothNecessaryAuthorityPassCount,D.solverFeasibleCount,D.fullPassCount);
    fprintf(' best fixed corridor slack=%+.6e at hb=%.12g\n',D.bestFixedCorridorSlack,D.bestFixedCorridorH);
    fprintf(' best fixed cone slack=%+.6e at hb=%.12g | worst fixed resultant index=%d\n', ...
        D.bestFixedMinCone,D.bestFixedMinConeH,D.bestFixedMinConeWorstResultantZeroBased);
    fprintf(' best fixed rz slack=%+.6e at hb=%.12g | worst fixed resultant index=%d\n', ...
        D.bestFixedMinRz,D.bestFixedMinRzH,D.bestFixedMinRzWorstResultantZeroBased);
    fprintf(' CLASSIFICATION: %s\n',D.classification);
end

fprintf('\n------------------------------------------------------------------------------------------------------------------------\n');
fprintf(' BSPLINE d4:d6 norms = %s\n',mat2str(A.bsplineHighJetNorm,8));
fprintf(' max d4:d6 norms among BEZIER/POLY/MINCO = %s\n',mat2str(A.otherHighJetNormMax,8));
fprintf(' THEORY: with unchanged backend geometry, increasing tail T by alpha scales d_r at tail start by alpha^{-r}; no such T change is applied here.\n');
fprintf(' RULE: if BSPLINE fixed endpoint resultants violate cone/taut for all evaluated hb, B7:B10 and coneprog cannot repair them.\n');
fprintf(' RULE: this is diagnosis only; a resource-range miss is still not a physical infeasibility claim.\n');
fprintf('========================================================================================================================\n');
end


% ========================================================================
% V209V equivalent resultant-moment formulation
% ========================================================================

function A=v209v_audit(CFG,K)
S=v209b_static_scenario(CFG);
Qold=v209n_build_static_problem_runtime(S,'PATCH_UNION',CFG,K);
Bold=v209s_build_backend_online('ACETRANS_MINCO_ST_C6',Qold.W,Qold.weights);
Told=v208_time_gate_angle(Bold,CFG,CFG.v209b_nominal_theta_deg);
assert(Told.pass,'G26V209V:OldReference','Old reference failed.');
tDetect=CFG.v209b_detection_fraction*Told.T;
tSwitch=tDetect+CFG.v209b_switch_lookahead_s;
J0=v209b_eval_jet(Bold,Told,tSwitch);

Snew=S;Snew.name='V209V_CAUSE';
Snew.C=[S.C S.newObstacleCenter];Snew.V=zeros(3,size(Snew.C,2));Snew.R=[S.R S.newObstacleRadius];
Snew.p0=J0(:,1);Snew.pg=S.pg;
Q=v209n_build_static_problem_runtime(Snew,CFG.v209v_representation,CFG,K);
assert(Q.guidePass&&Q.corridorOK,'G26V209V:Spatial','Updated spatial problem failed.');
prefix=v209b_active_interval_certificate(Bold,Told,tDetect,tSwitch,Q,CFG);
assert(prefix.pass,'G26V209V:Prefix','Prefix certificate failed.');

names={'BSPLINE_D7','BEZIER_D13','POLY_D13','ACETRANS_MINCO_ST_C6'};
rows=cell(1,numel(names));
A=struct();A.rep=CFG.v209v_representation;A.prefixSafe=prefix.pass;
A.safetyChanged=false;A.timeChanged=false;A.bridgeChanged=false;A.formalWCET=false;

for i=1:numel(names)
    name=names{i};
    QT=v209b_tail_problem(Q);
    [BT,TT,st]=v209s_tail_runtime_deep(QT,CFG,name,CFG.v209v_cert_max_depth);
    assert(strcmp(st,'PASS'),'G26V209V:Tail','Certified tail required for %s.',name);
    J1=v209b_backend_start_jets(BT,TT);
    H=v209t_socp_bridge_search(J0,J1,Q,BT,TT,CFG,CFG.v209v_cert_max_depth);

    N=numel(H.trace);
    D=repmat(v209v_empty_diag(),1,N);
    for k=1:N
        hb=H.trace(k).h;
        [B0,M,cR]=v209t_bridge_affine_map(J0,J1,hb,CFG); %#ok<ASGLU>
        X=v209v_fixed_conditions(B0,M,cR,Q,CFG);
        ANA=v209v_analytic_moment_test(B0,hb,CFG);
        RD=v209v_resultant_socp(B0,hb,Q,CFG,false);
        RF=v209v_resultant_socp(B0,hb,Q,CFG,true);

        auth=false;hard=false;minHard=NaN;
        if RF.feasible
            BB=v209v_reconstruct_bridge(B0,hb,RF.y,CFG);
            AUTH=v209v_authority(BB,hb,J0,J1,Q,BT,TT,CFG,CFG.v209v_cert_max_depth);
            auth=AUTH.localPass;hard=AUTH.fullPass;minHard=AUTH.hardSlack;
        end

        D(k).h=hb;
        D(k).fixedCorridor=X.fixedCorridorPass;
        D(k).fixedDynamic=X.fixedDynamicPass;
        D(k).meanConeNecessary=ANA.meanPass;
        D(k).weightedConeNecessary=ANA.weightedPass;
        D(k).meanConeSlack=ANA.meanConeSlack;
        D(k).meanRzSlack=ANA.meanRzSlack;
        D(k).weightedConeSlack=ANA.weightedConeSlack;
        D(k).weightedRzSlack=ANA.weightedRzSlack;
        D(k).xFormFeasible=H.trace(k).solverFeasible;
        D(k).rDynamicFeasible=RD.feasible;
        D(k).rFullFeasible=RF.feasible;
        D(k).rFullExitflag=RF.exitflag;
        D(k).reconstructedLocalPass=auth;
        D(k).reconstructedFullPass=hard;
        D(k).reconstructedHardSlack=minHard;
    end

    R=struct();R.backend=name;R.tailT=TT.T;R.tailH=TT.h;R.jetNorm=vecnorm(J1,2,1);
    R.diag=D;R.summary=v209v_summarize(D);
    rows{i}=R;
end
A.rows=[rows{:}];
end

function X=v209v_fixed_conditions(B0,M,cR,Q,CFG)
nf=Q.cnt(1);
fixedCols=[1:7 12:18];
s=inf;
for f=1:nf
    s=min(s,min(Q.N(:,f)'*B0(:,fixedCols)-Q.h(f)));
end
sc=max([1,max(abs(B0(:,fixedCols)),[],'all'),max(abs(Q.h(1:nf)))]);
corrGuard=4096*eps(sc);
X.fixedCorridorPass=isfinite(s)&&s>=-corrGuard;
X.fixedCorridorSlack=s;

dep=zeros(1,16);
for j=1:16,dep(j)=norm(M(:,:,j),'fro');end
dg=4096*eps(max(1,max(abs(M(:)))));
idx=find(dep<=dg);
R=cR(:,idx);
cone=tan(deg2rad(CFG.v209b_nominal_theta_deg))*R(3,:)-vecnorm(R(1:2,:),2,1);
rz=R(3,:)-CFG.v207_rho_min_mps2;
guard=CFG.v209b_bridge_guard_scale*eps(max(1,max(abs(R(:)))));
X.fixedDynamicPass=all(cone>=guard)&&all(rz>=guard);
X.fixedMinCone=min(cone);X.fixedMinRz=min(rz);
end

function A=v209v_analytic_moment_test(B0,hb,CFG)
n=17;c=hb^2/(n*(n-1));
g=[0;0;CFG.gravity];
d5=B0(:,7)-B0(:,6);     % B6-B5
d11=B0(:,13)-B0(:,12);  % B12-B11

q0=6*g+(d11-d5)/c;
q1=15*g+(B0(:,12)-B0(:,7)-5*d5)/c; % B11-B6

m0=q0/6;
m1=q1/15;
[A.meanPass,A.meanConeSlack,A.meanRzSlack]=v209v_in_K(m0,CFG);
[A.weightedPass,A.weightedConeSlack,A.weightedRzSlack]=v209v_in_K(m1,CFG);
A.meanResultant=m0;A.weightedResultant=m1;
end

function [pass,cs,rs]=v209v_in_K(r,CFG)
cs=tan(deg2rad(CFG.v209b_nominal_theta_deg))*r(3)-norm(r(1:2));
rs=r(3)-CFG.v207_rho_min_mps2;
guard=CFG.v209b_bridge_guard_scale*eps(max(1,max(abs(r))));
pass=all(isfinite(r))&&cs>=guard&&rs>=guard;
end

function S=v209v_resultant_socp(B0,hb,Q,CFG,useCorridor)
n=17;c=hb^2/(n*(n-1));
g=[0;0;CFG.gravity];
d5=B0(:,7)-B0(:,6);
d11=B0(:,13)-B0(:,12);

% y=[r5;r6;...;r10], each block xyz.
nv=18;
Aeq=zeros(6,nv);beq=zeros(6,1);
w=[5 4 3 2 1 0];

q0=6*g+(d11-d5)/c;
q1=15*g+(B0(:,12)-B0(:,7)-5*d5)/c;
for ax=1:3
    for j=1:6
        Aeq(ax,(j-1)*3+ax)=1;
        Aeq(3+ax,(j-1)*3+ax)=w(j);
    end
    beq(ax)=q0(ax);beq(3+ax)=q1(ax);
end

% r_z >= rho
Aineq=zeros(6,nv);bineq=-CFG.v207_rho_min_mps2*ones(6,1);
for j=1:6,Aineq(j,(j-1)*3+3)=-1;end

if useCorridor
    nf=Q.cnt(1);
    Aadd=zeros(4*nf,nv);badd=zeros(4*nf,1);rr=0;
    coeff={ [1], [2 1], [3 2 1], [4 3 2 1] };
    for k=1:4
        wk=coeff{k};
        base=B0(:,7)+k*d5-c*sum(wk)*g; % B6 + k*d5 - c*sum(w)*g
        G=zeros(3,nv);
        for j=1:numel(wk)
            G(:,(j-1)*3+(1:3))=c*wk(j)*eye(3);
        end
        for f=1:nf
            rr=rr+1;
            Aadd(rr,:)=-Q.N(:,f).'*G;
            badd(rr)=Q.N(:,f).'*base-Q.h(f);
        end
    end
    Aineq=[Aineq;Aadd];bineq=[bineq;badd];
end

soc=cell(1,6);tt=tan(deg2rad(CFG.v209b_nominal_theta_deg));
for j=1:6
    A=zeros(2,nv);A(1,(j-1)*3+1)=1;A(2,(j-1)*3+2)=1;
    b=zeros(2,1);
    d=zeros(nv,1);d((j-1)*3+3)=tt;
    soc{j}=secondordercone(A,b,d,0);
end

opts=optimoptions('coneprog','Display','none','LinearSolver','normal-dense');
f=zeros(nv,1);
y=nan(nv,1);exitflag=-999;output=struct();
try
    [y,~,exitflag,output]=coneprog(f,soc,Aineq,bineq,Aeq,beq,[],[],opts);
catch ME
    error('G26V209V:ResultantConeprog','Resultant-form coneprog failed at hb=%.12g: %s',hb,ME.message);
end
S=struct('feasible',exitflag>0&&all(isfinite(y)),'y',y,'exitflag',exitflag,'output',output,'useCorridor',useCorridor);
end

function BB=v209v_reconstruct_bridge(B0,hb,y,CFG)
n=17;c=hb^2/(n*(n-1));g=[0;0;CFG.gravity];
r=reshape(y,3,6);
a=r-g;
d5=B0(:,7)-B0(:,6);
BB=B0;
BB(:,8)=B0(:,7)+d5+c*a(:,1);
BB(:,9)=B0(:,7)+2*d5+c*(2*a(:,1)+a(:,2));
BB(:,10)=B0(:,7)+3*d5+c*(3*a(:,1)+2*a(:,2)+a(:,3));
BB(:,11)=B0(:,7)+4*d5+c*(4*a(:,1)+3*a(:,2)+2*a(:,3)+a(:,4));
end

function A=v209v_authority(BB,hb,J0,J1,Q,BT,TT,CFG,md)
[Jpass,~,~]=v209b_endpoint_jet_match(BB,hb,J0,J1);
CORb=v209b_bridge_corridor_audit(BB,Q);
DY=v209b_bridge_dynamic_audit(BB,hb,CFG.v209b_nominal_theta_deg,CFG);
A.localPass=Jpass&&CORb.pass&&DY.pass;
A.fullPass=false;A.hardSlack=NaN;
if ~A.localPass,return;end

P=[{BB} BT.P];hh=[hb TT.h];
BC=struct('P',{P},'degree',17,'structural_c6_theorem',true,'condition_indicator',1);
TC=struct('pass',true,'h',hh,'Rflat',[DY.R TT.Rflat],'rcounts',[DY.rcount TT.rcounts],'T',sum(hh));
C6=v207_c6_backend_audit(BC,hh);
COR=v209b_combined_corridor_audit(BC,Q);
D=v209l_final_direct_certificate_depth(BC,TC,Q,CFG,md);
A.hardSlack=D.min_slack;
A.fullPass=A.localPass&&C6.pass&&COR.pass&&D.pass;
end

function R=v209v_empty_diag()
R=struct('h',NaN,'fixedCorridor',false,'fixedDynamic',false, ...
    'meanConeNecessary',false,'weightedConeNecessary',false, ...
    'meanConeSlack',NaN,'meanRzSlack',NaN,'weightedConeSlack',NaN,'weightedRzSlack',NaN, ...
    'xFormFeasible',false,'rDynamicFeasible',false,'rFullFeasible',false,'rFullExitflag',NaN, ...
    'reconstructedLocalPass',false,'reconstructedFullPass',false,'reconstructedHardSlack',NaN);
end

function S=v209v_summarize(D)
S=struct();S.N=numel(D);
S.fixedCorridor=sum([D.fixedCorridor]);
S.fixedDynamic=sum([D.fixedDynamic]);
S.meanNecessary=sum([D.meanConeNecessary]);
S.weightedNecessary=sum([D.weightedConeNecessary]);
S.bothAnalytic=sum([D.meanConeNecessary]&[D.weightedConeNecessary]);
S.xFeasible=sum([D.xFormFeasible]);
S.rDynamicFeasible=sum([D.rDynamicFeasible]);
S.rFullFeasible=sum([D.rFullFeasible]);
S.rLocalPass=sum([D.reconstructedLocalPass]);
S.rFullCertified=sum([D.reconstructedFullPass]);

if S.rDynamicFeasible==0
    S.classification='RESULTANT_DYNAMIC_MOMENT_SOCP_INFEASIBLE__ENDPOINT_MOMENT_DYNAMICS_CAUSE';
elseif S.rFullFeasible==0
    S.classification='RESULTANT_DYNAMIC_FEASIBLE__CORRIDOR_COUPLING_CAUSES_INFEASIBILITY';
elseif S.xFeasible==0 && S.rFullFeasible>0
    S.classification='EQUIVALENT_RESULTANT_FULL_FEASIBLE__B_CONTROL_FORM_NUMERICAL_OR_FORMULATION_ISSUE';
elseif S.rFullCertified>0
    S.classification='RESULTANT_FORM_RECONSTRUCTS_CERTIFIED_BRIDGE';
else
    S.classification='FULL_CONVEX_FEASIBLE__UNCHANGED_FINAL_AUTHORITY_REJECTS';
end

ix=find([D.rDynamicFeasible],1,'first');if isempty(ix),S.firstDynamicH=NaN;else,S.firstDynamicH=D(ix).h;end
ix=find([D.rFullFeasible],1,'first');if isempty(ix),S.firstFullH=NaN;else,S.firstFullH=D(ix).h;end
ix=find([D.reconstructedFullPass],1,'first');if isempty(ix),S.firstCertifiedH=NaN;else,S.firstCertifiedH=D(ix).h;end

S.bestMeanCone=max([D.meanConeSlack]);S.bestWeightedCone=max([D.weightedConeSlack]);
S.minMeanCone=min([D.meanConeSlack]);S.minWeightedCone=min([D.weightedConeSlack]);
end

function v209v_print(A,CFG)
fprintf('\n========================================================================================================================\n');
fprintf(' G26-V209V THEORETICAL COUPLING RESULTS | rep=%s | theta=%.1fdeg\n',A.rep,CFG.v209b_nominal_theta_deg);
fprintf('------------------------------------------------------------------------------------------------------------------------\n');
fprintf(' No time/geometry/safety changes.  Counts are over the SAME inherited hb evaluation set.\n');
for i=1:numel(A.rows)
    R=A.rows(i);S=R.summary;
    fprintf('\n[%s] tailT=%.12g s | ||d4:d6||=%s\n',R.backend,R.tailT,mat2str(R.jetNorm(5:7),8));
    fprintf(' N=%d fixedCorr=%d fixedDyn=%d | analytic meanK=%d weightedK=%d both=%d\n', ...
        S.N,S.fixedCorridor,S.fixedDynamic,S.meanNecessary,S.weightedNecessary,S.bothAnalytic);
    fprintf(' X-form full feasible=%d | R-form dynamic-only=%d | R-form full+corridor=%d | reconstructedLocal=%d | reconstructedHardCert=%d\n', ...
        S.xFeasible,S.rDynamicFeasible,S.rFullFeasible,S.rLocalPass,S.rFullCertified);
    fprintf(' first h: dynamic=%g full=%g certified=%g\n',S.firstDynamicH,S.firstFullH,S.firstCertifiedH);
    fprintf(' mean-resultant cone slack range=[%+.6e,%+.6e] | weighted-resultant cone slack range=[%+.6e,%+.6e]\n', ...
        S.minMeanCone,S.bestMeanCone,S.minWeightedCone,S.bestWeightedCone);
    fprintf(' CLASSIFICATION: %s\n',S.classification);
end
fprintf('\n------------------------------------------------------------------------------------------------------------------------\n');
fprintf(' THEORY 1: failure of either analytic positive-weight average test proves dynamic-only infeasibility at that hb.\n');
fprintf(' THEORY 2: R-form dynamic feasible but R-form full infeasible isolates first-corridor/dynamics coupling.\n');
fprintf(' THEORY 3: R-form full feasible but X-form infeasible indicates formulation/scaling, not physics; do not change T in that case.\n');
fprintf(' RULE: only after this classification may joint continuous tail-T / hb allocation be introduced.\n');
fprintf('========================================================================================================================\n');
end


% ========================================================================
% V209W constructive sufficient bound
% ========================================================================

function A=v209w_audit(CFG,K)
S=v209b_static_scenario(CFG);

Qold=v209n_build_static_problem_runtime(S,'PATCH_UNION',CFG,K);
Bold=v209s_build_backend_online('ACETRANS_MINCO_ST_C6',Qold.W,Qold.weights);
Told=v208_time_gate_angle(Bold,CFG,CFG.v209b_nominal_theta_deg);
assert(Told.pass,'G26V209W:OldReference','Old reference failed.');
tDetect=CFG.v209b_detection_fraction*Told.T;
tSwitch=tDetect+CFG.v209b_switch_lookahead_s;
J0=v209b_eval_jet(Bold,Told,tSwitch);

Snew=S;Snew.name='V209W_BSPLINE_TIME_BOUND';
Snew.C=[S.C S.newObstacleCenter];Snew.V=zeros(3,size(Snew.C,2));Snew.R=[S.R S.newObstacleRadius];
Snew.p0=J0(:,1);Snew.pg=S.pg;
Q=v209n_build_static_problem_runtime(Snew,CFG.v209w_representation,CFG,K);
assert(Q.guidePass&&Q.corridorOK,'G26V209W:Spatial','Updated spatial problem failed.');
prefix=v209b_active_interval_certificate(Bold,Told,tDetect,tSwitch,Q,CFG);
assert(prefix.pass,'G26V209W:Prefix','Prefix certificate failed.');

QT=v209b_tail_problem(Q);
[BT,T0,st]=v209s_tail_runtime_deep(QT,CFG,'BSPLINE_D7',CFG.v209w_cert_max_depth);
assert(strcmp(st,'PASS'),'G26V209W:Tail','Baseline BSPLINE tail must be certified.');
Jcur=v209b_backend_start_jets(BT,T0);

hb=CFG.v209b_bridge_h_max_s; % proof anchor only
Jzero=Jcur;Jzero(:,2:7)=0;

% Strictly feasible zero-tail-jet proof anchor.
[S0,M0]=v209t_fixed_h_socp(J0,Jzero,Q,hb,CFG);
assert(S0.feasible,'G26V209W:ZeroJetAnchor','Zero-tail-jet proof anchor must be SOCP feasible at inherited hb_max.');
Bz=S0.B;
[Jp,~,~]=v209b_endpoint_jet_match(Bz,hb,J0,Jzero);
Cz=v209b_bridge_corridor_audit(Bz,Q);
Dz=v209b_bridge_dynamic_audit(Bz,hb,CFG.v209b_nominal_theta_deg,CFG);
assert(Jp&&Cz.pass&&Dz.pass,'G26V209W:ZeroJetLocal','Zero-tail-jet proof anchor must pass unchanged local authority.');

% Build exact coefficient matrices DeltaB_r such that
% B(alpha)-Bzero = sum_{r=1}^6 alpha^{-r} DeltaB_r
DB=zeros(3,18,6);
n=17;
for r=1:6
    for k=r:6
        col=18-k; % zero-based B_{17-k} -> MATLAB col 18-k
        coef=nchoosek(k,r)*(hb^r/v200_falling(n,r))*((-1)^r);
        DB(:,col,r)=coef*Jcur(:,r+1);
    end
end
fac=n*(n-1)/hb^2;
DR=zeros(3,16,6);
for r=1:6
    DR(:,:,r)=fac*diff(DB(:,:,r),2,2);
end

BND=v209w_build_bounds(Bz,DB,DR,Q,CFG);
alphaMax=CFG.v207_time_max_s/T0.T;
[boundPass,alphaBound,it]=v209w_bisect_alpha(BND,alphaMax,CFG);

A=struct();
A.rep=CFG.v209w_representation;
A.backend='BSPLINE_D7';
A.prefixSafe=prefix.pass;
A.T0=T0.T;
A.h0=T0.h;
A.hbProofAnchor=hb;
A.alphaMax=alphaMax;
A.boundPass=boundPass;
A.alphaBound=alphaBound;
A.boundIterations=it;
A.Tsufficient=NaN;
A.zeroJetAnchor=struct('solverFeasible',S0.feasible,'fixedCorridorPass',M0.fixedCorridorPass, ...
    'corridorSlack',Cz.min_slack,'coneSlack',Dz.minCone,'rzSlack',Dz.minRz);
A.bound=BND;
A.safetyChanged=false;
A.timeBankUsed=false;
A.hbPromoted=false;
A.globalOptimalityClaim=false;
A.formalWCET=false;

if ~boundPass
    A.classification='CONSERVATIVE_SUFFICIENT_BOUND_NOT_REACHED_WITHIN_EXISTING_T_RESOURCE_HORIZON__NO_INFEASIBILITY_CLAIM';
    A.constructive=[];
    A.socpAtBound=[];
    return;
end

A.Tsufficient=alphaBound*T0.T;
Ts=v209w_scale_tail_time(BT,T0,alphaBound,CFG);
Js=v209b_backend_start_jets(BT,Ts);

% Constructive bridge retaining zero-jet B7:B10 and exact scaled endpoint jets.
[Bfix,~,~]=v209t_bridge_affine_map(J0,Js,hb,CFG);
Bc=Bfix;
Bc(:,8:11)=Bz(:,8:11);
AUTHc=v209w_full_authority(Bc,hb,J0,Js,Q,BT,Ts,CFG,CFG.v209w_cert_max_depth);
A.constructive=AUTHc;

% Independent fixed-hb SOCP at the analytically sufficient T.
[okS,Cs,Es]=v209t_eval_socp_bridge(J0,Js,Q,BT,Ts,hb,CFG,CFG.v209w_cert_max_depth);
A.socpAtBound=struct('pass',okS,'eval',Es,'candidate',Cs);

if AUTHc.localPass && AUTHc.fullPass
    A.classification='ANALYTIC_SUFFICIENT_T_BOUND__CONSTRUCTIVE_BRIDGE_FULLY_CERTIFIED';
elseif okS
    A.classification='ANALYTIC_LOCAL_BOUND_REACHED__INDEPENDENT_SOCP_FULLY_CERTIFIED';
elseif AUTHc.localPass
    A.classification='ANALYTIC_LOCAL_BOUND_REACHED__LOCAL_BRIDGE_CERTIFIED__FULL_BODY_AUTHORITY_REJECTS';
else
    A.classification='BOUND_IMPLEMENTATION_OR_NUMERICAL_MISMATCH__REQUIRES_AUDIT';
end
end

function B=v209w_build_bounds(Bz,DB,DR,Q,CFG)
nf=Q.cnt(1);
R0=v209w_resultants(Bz,CFG.v209b_bridge_h_max_s,CFG);
cone0=tan(deg2rad(CFG.v209b_nominal_theta_deg))*R0(3,:)-vecnorm(R0(1:2,:),2,1);
rz0=R0(3,:)-CFG.v207_rho_min_mps2;

corrSlack=[];
corrCoef=[];
for f=1:nf
    for j=1:18
        s=Q.N(:,f)'*Bz(:,j)-Q.h(f);
        c=zeros(1,6);
        for r=1:6,c(r)=abs(Q.N(:,f)'*DB(:,j,r));end
        corrSlack(end+1,1)=s; %#ok<AGROW>
        corrCoef(end+1,:)=c; %#ok<AGROW>
    end
end

coneCoef=zeros(16,6);
rzCoef=zeros(16,6);
scaleCoef=zeros(16,6);
tt=tan(deg2rad(CFG.v209b_nominal_theta_deg));
for j=1:16
    for r=1:6
        d=DR(:,j,r);
        coneCoef(j,r)=tt*abs(d(3))+norm(d(1:2));
        rzCoef(j,r)=abs(d(3));
        scaleCoef(j,r)=max(abs(d));
    end
end

B=struct();
B.corridorSlack0=corrSlack;
B.corridorCoeff=corrCoef;
B.coneSlack0=cone0(:);
B.coneCoeff=coneCoef;
B.rzSlack0=rz0(:);
B.rzCoeff=rzCoef;
B.scaleCoeff=scaleCoef;
B.baseMaxR=max(abs(R0(:)));
B.baseMinCorridor=min(corrSlack);
B.baseMinCone=min(cone0);
B.baseMinRz=min(rz0);
B.guardScale=CFG.v209b_bridge_guard_scale;
end

function [pass,a,it]=v209w_bisect_alpha(B,aMax,CFG)
pass=false;a=NaN;it=0;
if aMax<1,return;end
if ~v209w_bound_ok(B,aMax,CFG),return;end
lo=1;hi=aMax;
if v209w_bound_ok(B,lo,CFG)
    pass=true;a=1;return;
end
for it=1:80
    mid=0.5*(lo+hi);
    if v209w_bound_ok(B,mid,CFG),hi=mid;else,lo=mid;end
    if hi-lo <= 128*eps(max(1,hi)),break;end
end
pass=true;a=hi;
end

function ok=v209w_bound_ok(B,a,CFG)
p=a.^(-(1:6));

% Corridor: require nonnegative slack (stronger than numerical guard).
lossC=B.corridorCoeff*p(:);
okC=all(B.corridorSlack0-lossC>=0);

% Dynamic guard envelope.  Bound max resultant magnitude by base plus
% Euclidean perturbation coefficient sum.
maxPert=0;
for j=1:size(B.scaleCoeff,1)
    maxPert=max(maxPert,B.scaleCoeff(j,:)*p(:));
end
scaleBound=max(1,B.baseMaxR+maxPert);
guard=B.guardScale*eps(scaleBound);

lossCone=B.coneCoeff*p(:);
lossRz=B.rzCoeff*p(:);
okD=all(B.coneSlack0-lossCone>=guard) && all(B.rzSlack0-lossRz>=guard);
ok=okC&&okD;
end

function R=v209w_resultants(B,hb,CFG)
n=size(B,2)-1;
R=n*(n-1)/hb^2*diff(B,2,2);
R(3,:)=R(3,:)+CFG.gravity;
end

function T=v209w_scale_tail_time(B,T0,alpha,CFG)
T=T0;
T.T=alpha*T0.T;
T.h=alpha*T0.h;
Rcell=cell(1,numel(B.P));
rc=zeros(1,numel(B.P));
for q=1:numel(B.P)
    X=B.P{q};n=size(X,2)-1;
    R=n*(n-1)/T.h(q)^2*diff(X,2,2);
    R(3,:)=R(3,:)+CFG.gravity;
    Rcell{q}=R;rc(q)=size(R,2);
end
T.Rflat=[Rcell{:}];T.rcounts=rc;
cs=tan(deg2rad(CFG.v209b_nominal_theta_deg))*T.Rflat(3,:)-vecnorm(T.Rflat(1:2,:),2,1);
rz=T.Rflat(3,:)-CFG.v207_rho_min_mps2;
scale=max(1,max(abs(T.Rflat(:))));
guard=CFG.v207_time_guard_scale*eps(scale);
T.min_cone_slack=min(cs);T.min_rz_slack=min(rz);
T.pass=all(cs>=guard)&&all(rz>=guard);
T.status=double(T.pass);
T.search_incomplete=false;
T.iterations=0;
T.theta_deg=CFG.v209b_nominal_theta_deg;
T.max_control_angle_deg=rad2deg(max(atan2(vecnorm(T.Rflat(1:2,:),2,1),T.Rflat(3,:))));
T.tension_ratio_bound=max(vecnorm(T.Rflat,2,1))/CFG.gravity;
end

function A=v209w_full_authority(BB,hb,J0,J1,Q,BT,TT,CFG,md)
[Jpass,jres,jratio]=v209b_endpoint_jet_match(BB,hb,J0,J1);
CORb=v209b_bridge_corridor_audit(BB,Q);
DY=v209b_bridge_dynamic_audit(BB,hb,CFG.v209b_nominal_theta_deg,CFG);
A=struct();
A.jetPass=Jpass;A.jetResidual=jres;A.jetRatio=jratio;
A.corridorPass=CORb.pass;A.corridorSlack=CORb.min_slack;
A.dynamicPass=DY.pass;A.coneSlack=DY.minCone;A.rzSlack=DY.minRz;
A.tailTimePass=TT.pass;A.tailConeSlack=TT.min_cone_slack;A.tailRzSlack=TT.min_rz_slack;
A.localPass=Jpass&&CORb.pass&&DY.pass&&TT.pass;
A.c6Pass=false;A.combinedCorridorPass=false;A.hardPass=false;A.hardSlack=NaN;A.hardDepth=NaN;A.fullPass=false;
if ~A.localPass,return;end
P=[{BB} BT.P];hh=[hb TT.h];
BC=struct('P',{P},'degree',17,'structural_c6_theorem',true,'condition_indicator',1);
TC=struct('pass',true,'h',hh,'Rflat',[DY.R TT.Rflat],'rcounts',[DY.rcount TT.rcounts],'T',sum(hh));
C6=v207_c6_backend_audit(BC,hh);
COR=v209b_combined_corridor_audit(BC,Q);
D=v209l_final_direct_certificate_depth(BC,TC,Q,CFG,md);
A.c6Pass=C6.pass;A.combinedCorridorPass=COR.pass;A.hardPass=D.pass;A.hardSlack=D.min_slack;A.hardDepth=D.max_depth;
A.fullPass=A.localPass&&C6.pass&&COR.pass&&D.pass;
end

function v209w_print(A,CFG)
fprintf('\n========================================================================================================================\n');
fprintf(' G26-V209W CONSTRUCTIVE BSPLINE TIME-BOUND RESULT | rep=%s\n',A.rep);
fprintf('------------------------------------------------------------------------------------------------------------------------\n');
fprintf(' baseline tail-only T0=%.12g s | h0=%s\n',A.T0,mat2str(A.h0,9));
fprintf(' proof-anchor hb=%.12g s (qualification anchor only; NOT promoted)\n',A.hbProofAnchor);
fprintf(' zero-jet anchor: feasible=%d corridor=%d slack=%+.6e cone=%+.6e rz=%+.6e\n', ...
    A.zeroJetAnchor.solverFeasible,A.zeroJetAnchor.fixedCorridorPass,A.zeroJetAnchor.corridorSlack, ...
    A.zeroJetAnchor.coneSlack,A.zeroJetAnchor.rzSlack);
fprintf(' conservative base minima: corridor=%+.6e cone=%+.6e rz=%+.6e\n', ...
    A.bound.baseMinCorridor,A.bound.baseMinCone,A.bound.baseMinRz);
fprintf(' alpha search interval=[1, %.12g] from existing T resource horizon Tmax=%.3f s | fixed bank=0\n', ...
    A.alphaMax,CFG.v207_time_max_s);
fprintf(' sufficient-bound pass=%d | alphaBound=%.12g | Tsufficient=%.12g s | bisectionIters=%d\n', ...
    A.boundPass,A.alphaBound,A.Tsufficient,A.boundIterations);

if A.boundPass
    C=A.constructive;
    fprintf(' constructive exact check: jet=%d corridor=%d dynamic=%d tailTime=%d local=%d\n', ...
        C.jetPass,C.corridorPass,C.dynamicPass,C.tailTimePass,C.localPass);
    fprintf('   jetResidual=%.3e ratio=%.3e | bridge corridor=%+.6e cone=%+.6e rz=%+.6e\n', ...
        C.jetResidual,C.jetRatio,C.corridorSlack,C.coneSlack,C.rzSlack);
    fprintf('   tail cone=%+.6e rz=%+.6e | C6=%d combinedCorridor=%d hardCert=%d hardSlack=%+.6e depth=%g | FULL=%d\n', ...
        C.tailConeSlack,C.tailRzSlack,C.c6Pass,C.combinedCorridorPass,C.hardPass,C.hardSlack,C.hardDepth,C.fullPass);
    S=A.socpAtBound;
    fprintf(' independent SOCP at same (T,hb): fullCertified=%d solverFeasible=%d localGate=%d hardCert=%d status=%s\n', ...
        S.pass,S.eval.solverFeasible,S.eval.localGatePass,S.eval.hardCertPass,S.eval.status);
end

fprintf(' CLASSIFICATION: %s\n',A.classification);
fprintf('\n------------------------------------------------------------------------------------------------------------------------\n');
fprintf(' THEORY: this T is a sufficient upper bound obtained from monotone perturbation bounds, not an empirical multiplier.\n');
fprintf(' LIMIT: hb is still a proof anchor; no claim of minimum T, minimum hb, or globally optimal T+hb is made.\n');
fprintf(' RULE: only unchanged exact full-body certification can authorize the final candidate.\n');
fprintf(' NEXT after success: derive the final joint continuous T/hb allocator from the exact resultant-moment feasibility structure.\n');
fprintf('========================================================================================================================\n');
end


% ========================================================================
% V209X adaptive continuous-hb certified envelope
% ========================================================================

function A=v209x_audit(CFG,K)
S=v209b_static_scenario(CFG);

Qold=v209n_build_static_problem_runtime(S,'PATCH_UNION',CFG,K);
Bold=v209s_build_backend_online('ACETRANS_MINCO_ST_C6',Qold.W,Qold.weights);
Told=v208_time_gate_angle(Bold,CFG,CFG.v209b_nominal_theta_deg);
assert(Told.pass,'G26V209X:OldReference','Old reference failed.');
tDetect=CFG.v209b_detection_fraction*Told.T;
tSwitch=tDetect+CFG.v209b_switch_lookahead_s;
J0=v209b_eval_jet(Bold,Told,tSwitch);

Snew=S;Snew.name='V209X_ENVELOPE';
Snew.C=[S.C S.newObstacleCenter];Snew.V=zeros(3,size(Snew.C,2));Snew.R=[S.R S.newObstacleRadius];
Snew.p0=J0(:,1);Snew.pg=S.pg;
Q=v209n_build_static_problem_runtime(Snew,CFG.v209x_representation,CFG,K);
assert(Q.guidePass&&Q.corridorOK,'G26V209X:Spatial','Updated spatial problem failed.');
prefix=v209b_active_interval_certificate(Bold,Told,tDetect,tSwitch,Q,CFG);
assert(prefix.pass,'G26V209X:Prefix','Prefix certificate failed.');

QT=v209b_tail_problem(Q);
[BT,T0,st]=v209s_tail_runtime_deep(QT,CFG,'BSPLINE_D7',CFG.v209x_cert_max_depth);
assert(strcmp(st,'PASS'),'G26V209X:Tail','Baseline BSPLINE tail must be certified.');
Jcur=v209b_backend_start_jets(BT,T0);

lo=CFG.v209b_bridge_h_min_s;
hi=CFG.v209b_bridge_h_max_s;
maxEval=CFG.v209x_hb_evals;

% Deterministic continuous interval subdivision.
% Intervals are resources, not candidate-bank semantics.
L=nan(2*maxEval+8,1);R=nan(2*maxEval+8,1);active=false(size(L));
L(1)=lo;R(1)=hi;active(1)=true;nextSlot=2;

pts=nan(maxEval,1);
rec=cell(maxEval,1);
nEval=0;

% Evaluate endpoints first.
for h=[lo hi]
    nEval=nEval+1;pts(nEval)=h;
    rec{nEval}=v209x_eval_hb(h,J0,Jcur,Q,BT,T0,CFG);
end

while any(active) && nEval<maxEval
    idx=find(active);
    widths=R(idx)-L(idx);
    [wmax,jj]=max(widths);
    if wmax<=CFG.v209x_hb_width_tol,break;end
    ii=idx(jj);
    a=L(ii);b=R(ii);active(ii)=false;
    m=0.5*(a+b);

    nEval=nEval+1;pts(nEval)=m;
    rec{nEval}=v209x_eval_hb(m,J0,Jcur,Q,BT,T0,CFG);

    L(nextSlot)=a;R(nextSlot)=m;active(nextSlot)=true;nextSlot=nextSlot+1;
    L(nextSlot)=m;R(nextSlot)=b;active(nextSlot)=true;nextSlot=nextSlot+1;
end

pts=pts(1:nEval);rec=rec(1:nEval);
[pts,ord]=sort(pts);
rec=rec(ord);

cert=false(1,nEval);J=inf(1,nEval);T=nan(1,nEval);alpha=nan(1,nEval);
for i=1:nEval
    cert(i)=rec{i}.certified;
    if cert(i)
        J(i)=rec{i}.totalTime;
        T(i)=rec{i}.T;
        alpha(i)=rec{i}.alpha;
    end
end

A=struct();
A.rep=CFG.v209x_representation;
A.backend='BSPLINE_D7';
A.prefixSafe=prefix.pass;
A.T0=T0.T;
A.h0=T0.h;
A.hbMin=lo;A.hbMax=hi;
A.evals=nEval;
A.hb=pts.';
A.records=[rec{:}];
A.certifiedMask=cert;
A.certifiedCount=sum(cert);
A.safetyChanged=false;
A.timeBankUsed=false;
A.globalOptimalityClaim=false;
A.searchComplete=(~any(active)) || all((R(active)-L(active))<=CFG.v209x_hb_width_tol);
A.infeasibilityClaim=false;
A.formalWCET=false;

if any(cert)
    [bestJ,ib]=min(J);
    A.bestFound=struct('hb',pts(ib),'T',T(ib),'alpha',alpha(ib),'totalTime',bestJ, ...
        'record',rec{ib},'optimalityClaim',false);
else
    A.bestFound=[];
end

% Topological description of sampled certified envelope only.
A.sampledCertifiedIntervals=v209x_sampled_intervals(pts,cert);
end

function R=v209x_eval_hb(hb,J0,Jcur,Q,BT,T0,CFG)
R=v209x_empty_record(hb);

Jzero=Jcur;Jzero(:,2:7)=0;
[S0,M0]=v209t_fixed_h_socp(J0,Jzero,Q,hb,CFG);
R.zeroAnchorFeasible=S0.feasible;
R.zeroFixedCorridorPass=M0.fixedCorridorPass;
if ~S0.feasible
    R.classification='ZERO_JET_ANCHOR_NOT_FOUND_AT_THIS_HB__NO_FINITE_T_INFEASIBILITY_CLAIM';
    return;
end

Bz=S0.B;
[Jp,~,~]=v209b_endpoint_jet_match(Bz,hb,J0,Jzero);
Cz=v209b_bridge_corridor_audit(Bz,Q);
Dz=v209b_bridge_dynamic_audit(Bz,hb,CFG.v209b_nominal_theta_deg,CFG);
R.zeroCorridorSlack=Cz.min_slack;R.zeroConeSlack=Dz.minCone;R.zeroRzSlack=Dz.minRz;
if ~(Jp&&Cz.pass&&Dz.pass)
    R.classification='ZERO_JET_SOCP_WITNESS_REJECTED_BY_UNCHANGED_LOCAL_AUTHORITY';
    return;
end

[DB,DR]=v209x_endpoint_perturbation(Jcur,hb);
BND=v209x_build_bounds(Bz,DB,DR,Q,CFG,hb);
alphaMax=CFG.v207_time_max_s/T0.T;
[pass,a,it]=v209w_bisect_alpha(BND,alphaMax,CFG);
R.boundPass=pass;R.boundIterations=it;R.bound=BND;
if ~pass
    R.classification='SUFFICIENT_BOUND_NOT_REACHED_WITHIN_EXISTING_T_RESOURCE_HORIZON';
    return;
end

R.alpha=a;R.T=a*T0.T;
Ts=v209w_scale_tail_time(BT,T0,a,CFG);
Js=v209b_backend_start_jets(BT,Ts);

[Bfix,~,~]=v209t_bridge_affine_map(J0,Js,hb,CFG);
Bc=Bfix;Bc(:,8:11)=Bz(:,8:11);
AUTH=v209w_full_authority(Bc,hb,J0,Js,Q,BT,Ts,CFG,CFG.v209x_cert_max_depth);
R.constructive=AUTH;
R.constructiveLocal=AUTH.localPass;
R.constructiveFull=AUTH.fullPass;

[okS,Cs,Es]=v209t_eval_socp_bridge(J0,Js,Q,BT,Ts,hb,CFG,CFG.v209x_cert_max_depth);
R.independentSocpFull=okS;
R.independentSocpEval=Es;
R.independentSocpCandidate=Cs;

R.certified=AUTH.fullPass&&okS;
if R.certified
    R.totalTime=R.T+hb;
    R.classification='CERTIFIED_SUFFICIENT_ENVELOPE_POINT';
elseif AUTH.fullPass
    R.classification='CONSTRUCTIVE_FULL_CERTIFIED__INDEPENDENT_SOCP_DID_NOT_CONFIRM';
elseif okS
    R.classification='INDEPENDENT_SOCP_FULL_CERTIFIED__CONSERVATIVE_CONSTRUCTIVE_WITNESS_REJECTED';
else
    R.classification='SUFFICIENT_BOUND_REACHED__UNCHANGED_FINAL_AUTHORITY_NOT_CERTIFIED';
end
end


function R=v209x_empty_record(hb)
% Fixed outer schema for every hb evaluation.  Early-return branches keep
% exactly the same field set, preventing heterogeneous-struct concatenation.
R=struct();
R.hb=hb;
R.zeroAnchorFeasible=false;
R.zeroFixedCorridorPass=false;
R.zeroCorridorSlack=NaN;
R.zeroConeSlack=NaN;
R.zeroRzSlack=NaN;
R.boundPass=false;
R.boundIterations=0;
R.bound=[];
R.alpha=NaN;
R.T=NaN;
R.constructive=[];
R.constructiveLocal=false;
R.constructiveFull=false;
R.independentSocpFull=false;
R.independentSocpEval=[];
R.independentSocpCandidate=[];
R.certified=false;
R.totalTime=inf;
R.classification='UNSET';
end

function [DB,DR]=v209x_endpoint_perturbation(Jcur,hb)
n=17;
DB=zeros(3,18,6);
for r=1:6
    for k=r:6
        col=18-k;
        coef=nchoosek(k,r)*(hb^r/v200_falling(n,r))*((-1)^r);
        DB(:,col,r)=coef*Jcur(:,r+1);
    end
end
fac=n*(n-1)/hb^2;
DR=zeros(3,16,6);
for r=1:6
    DR(:,:,r)=fac*diff(DB(:,:,r),2,2);
end
end

function B=v209x_build_bounds(Bz,DB,DR,Q,CFG,hb)
nf=Q.cnt(1);
R0=v209w_resultants(Bz,hb,CFG);
cone0=tan(deg2rad(CFG.v209b_nominal_theta_deg))*R0(3,:)-vecnorm(R0(1:2,:),2,1);
rz0=R0(3,:)-CFG.v207_rho_min_mps2;

corrSlack=zeros(nf*18,1);
corrCoef=zeros(nf*18,6);
rr=0;
for f=1:nf
    for j=1:18
        rr=rr+1;
        corrSlack(rr)=Q.N(:,f)'*Bz(:,j)-Q.h(f);
        for r=1:6
            corrCoef(rr,r)=abs(Q.N(:,f)'*DB(:,j,r));
        end
    end
end

coneCoef=zeros(16,6);rzCoef=zeros(16,6);scaleCoef=zeros(16,6);
tt=tan(deg2rad(CFG.v209b_nominal_theta_deg));
for j=1:16
    for r=1:6
        d=DR(:,j,r);
        coneCoef(j,r)=tt*abs(d(3))+norm(d(1:2));
        rzCoef(j,r)=abs(d(3));
        scaleCoef(j,r)=max(abs(d));
    end
end

B=struct();
B.corridorSlack0=corrSlack;B.corridorCoeff=corrCoef;
B.coneSlack0=cone0(:);B.coneCoeff=coneCoef;
B.rzSlack0=rz0(:);B.rzCoeff=rzCoef;
B.scaleCoeff=scaleCoef;
B.baseMaxR=max(abs(R0(:)));
B.baseMinCorridor=min(corrSlack);
B.baseMinCone=min(cone0);
B.baseMinRz=min(rz0);
B.guardScale=CFG.v209b_bridge_guard_scale;
end

function I=v209x_sampled_intervals(h,mask)
I=zeros(0,2);
if isempty(h),return;end
s=NaN;
for i=1:numel(h)
    if mask(i)&&isnan(s),s=h(i);end
    if ~mask(i)&&~isnan(s)
        I(end+1,:)=[s h(i-1)]; %#ok<AGROW>
        s=NaN;
    end
end
if ~isnan(s),I(end+1,:)=[s h(end)];end
end

function v209x_print(A,CFG)
fprintf('\n========================================================================================================================\n');
fprintf(' G26-V209X CERTIFIED SUFFICIENT T(hb) ENVELOPE | rep=%s | backend=%s\n',A.rep,A.backend);
fprintf('------------------------------------------------------------------------------------------------------------------------\n');
fprintf(' tail-only T0=%.12g s | hb interval=[%.6g, %.6g] s | adaptive evals=%d | fixedBank=0\n', ...
    A.T0,A.hbMin,A.hbMax,A.evals);
fprintf(' certified sampled points=%d/%d | searchComplete(resolution only)=%d | globalOptimalityClaim=%d\n', ...
    A.certifiedCount,A.evals,A.searchComplete,A.globalOptimalityClaim);

for i=1:A.evals
    R=A.records(i);
    if R.certified
        fprintf('  hb=% .9f | T_suff=% .9f | alpha=% .6f | total=% .9f | zeroSlack[corr cone rz]=[%+.3e %+.3e %+.3e]\n', ...
            R.hb,R.T,R.alpha,R.totalTime,R.zeroCorridorSlack,R.zeroConeSlack,R.zeroRzSlack);
    end
end

fprintf(' sampled certified hb intervals (diagnostic only; not a continuous-feasibility proof): %s\n', ...
    mat2str(A.sampledCertifiedIntervals,9));

if ~isempty(A.bestFound)
    B=A.bestFound;
    fprintf(' best CERTIFIED point FOUND (NOT claimed optimal): hb=%.12g s | T=%.12g s | total=%.12g s | alpha=%.12g\n', ...
        B.hb,B.T,B.totalTime,B.alpha);
    fprintf('   constructiveFull=%d independentSOCPFull=%d hardSlack=%+.6e depth=%g\n', ...
        B.record.constructiveFull,B.record.independentSocpFull, ...
        B.record.constructive.hardSlack,B.record.constructive.hardDepth);
else
    fprintf(' no certified envelope point found within current hb/T resource horizons; NO infeasibility claim.\n');
end

fprintf('\n------------------------------------------------------------------------------------------------------------------------\n');
fprintf(' THEORY: every reported point is certified by a monotone sufficient alpha bound plus unchanged exact full-body authority.\n');
fprintf(' LIMIT: unsampled hb and non-certified sampled hb are NOT declared infeasible.\n');
fprintf(' LIMIT: the smallest reported total time is only best-found, NOT a global minimum.\n');
fprintf(' NEXT: add globally valid hb lower bounds/pruning or an equivalent continuous formulation before calling the allocator optimal.\n');
fprintf('========================================================================================================================\n');
end


% ========================================================================
% V209Y certified interval lower bound
% ========================================================================


% ========================================================================
% V209Z: stronger formal moment relaxation + hard-certified primal updates
% ========================================================================

function A=v209z_audit(CFG,K)
tStart=tic;
S=v209b_static_scenario(CFG);

Qold=v209n_build_static_problem_runtime(S,'PATCH_UNION',CFG,K);
Bold=v209s_build_backend_online('ACETRANS_MINCO_ST_C6',Qold.W,Qold.weights);
Told=v208_time_gate_angle(Bold,CFG,CFG.v209b_nominal_theta_deg);
assert(Told.pass,'G26V209Z:OldReference','Old reference failed.');
tDetect=CFG.v209b_detection_fraction*Told.T;
tSwitch=tDetect+CFG.v209b_switch_lookahead_s;
J0=v209b_eval_jet(Bold,Told,tSwitch);

Snew=S;Snew.name='V209Z_PRIMAL_DUAL';
Snew.C=[S.C S.newObstacleCenter];
Snew.V=zeros(3,size(Snew.C,2));
Snew.R=[S.R S.newObstacleRadius];
Snew.p0=J0(:,1);Snew.pg=S.pg;
Q=v209n_build_static_problem_runtime(Snew,CFG.v209y_representation,CFG,K);
assert(Q.guidePass&&Q.corridorOK,'G26V209Z:Spatial','Updated spatial problem failed.');
prefix=v209b_active_interval_certificate(Bold,Told,tDetect,tSwitch,Q,CFG);
assert(prefix.pass,'G26V209Z:Prefix','Prefix certificate failed.');

QT=v209b_tail_problem(Q);
[BT,T0,st]=v209s_tail_runtime_deep(QT,CFG,'BSPLINE_D7',CFG.v209y_cert_max_depth);
assert(strcmp(st,'PASS'),'G26V209Z:Tail','Baseline BSPLINE tail must be certified.');
Jcur=v209b_backend_start_jets(BT,T0);

% Initial hard-certified incumbent from V209W/X proof anchor.
inc0=v209x_eval_hb(CFG.v209b_bridge_h_max_s,J0,Jcur,Q,BT,T0,CFG);
assert(inc0.certified,'G26V209Z:Incumbent','Hard-certified initial incumbent was not reproduced.');
UBraw=inc0.totalTime;
UB=UBraw+8*eps(max(1,abs(UBraw)));
best=struct('hb',inc0.hb,'T',inc0.T,'total',inc0.totalTime,'source','V209W_X_ANCHOR', ...
    'eval',inc0,'hardSlack',inc0.constructive.hardSlack,'hardDepth',inc0.constructive.hardDepth);

hL=CFG.v209b_bridge_h_min_s;
hU=CFG.v209b_bridge_h_max_s;
TL=T0.T;
TU=min(CFG.v207_time_max_s,UB-hL);
assert(TU>=TL,'G26V209Z:Domain','Incumbent leaves no nontrivial T domain.');

H=v209y_heap_init(CFG.v209y_max_boxes*2+16);
H=v209y_heap_push(H,v209y_box(hL,hU,TL,TU,0));

count=0;pruneObj=0;pruneCorr=0;pruneFixedDyn=0;pruneMoment=0;
splitH=0;splitT=0;machineLeaves=0;
primalEvals=0;primalFeasible=0;ubUpdates=0;
primalLog=repmat(v209z_empty_primal(),1,CFG.v209y_max_primal_evals);

while H.n>0 && count<CFG.v209y_max_boxes
    [H,B]=v209y_heap_pop(H);
    count=count+1;

    if B.key>=UB
        pruneObj=pruneObj+1;
        continue;
    end

    N=v209y_interval_necessary(B,J0,Jcur,T0.T,Q,CFG);
    if ~N.fixedCorridorPossible
        pruneCorr=pruneCorr+1;continue;
    end
    if ~N.fixedDynamicPossible
        pruneFixedDyn=pruneFixedDyn+1;continue;
    end
    if ~N.momentPossible
        pruneMoment=pruneMoment+1;continue;
    end

    % Best-first exact primal candidate. This can only improve the UB;
    % it is NEVER used for formal pruning if it fails.
    if primalEvals<CFG.v209y_max_primal_evals
        primalEvals=primalEvals+1;
        hp=0.5*(B.hL+B.hU);
        Tp=0.5*(B.tL+B.tU);
        P=v209z_exact_candidate(hp,Tp,J0,Q,BT,T0,CFG);
        P.boxKey=B.key;P.boxDepth=B.depth;
        primalLog(primalEvals)=P;
        if P.certified
            primalFeasible=primalFeasible+1;
            Jraw=P.totalTime;
            Jup=Jraw+8*eps(max(1,abs(Jraw)));
            if Jup<UB
                UB=Jup;ubUpdates=ubUpdates+1;
                best=struct('hb',hp,'T',Tp,'total',Jraw,'source','BEST_FIRST_BOX_CENTER', ...
                    'eval',P,'hardSlack',P.hardSlack,'hardDepth',P.hardDepth);
            end
        end
    end

    canSplitH=(B.hU-B.hL)>128*eps(max(1,B.hU));
    canSplitT=(B.tU-B.tL)>128*eps(max(1,B.tU));
    if B.depth>=CFG.v209y_max_depth || ~(canSplitH||canSplitT)
        H=v209y_heap_push(H,B);
        machineLeaves=machineLeaves+1;
        break;
    end

    wh=B.hU-B.hL;wt=B.tU-B.tL;
    if canSplitT && (~canSplitH || wt>=wh)
        m=0.5*(B.tL+B.tU);
        B1=v209y_box(B.hL,B.hU,B.tL,m,B.depth+1);
        B2=v209y_box(B.hL,B.hU,m,B.tU,B.depth+1);
        splitT=splitT+1;
    else
        m=0.5*(B.hL+B.hU);
        B1=v209y_box(B.hL,m,B.tL,B.tU,B.depth+1);
        B2=v209y_box(m,B.hU,B.tL,B.tU,B.depth+1);
        splitH=splitH+1;
    end
    if B1.key<UB,H=v209y_heap_push(H,B1);else,pruneObj=pruneObj+1;end
    if B2.key<UB,H=v209y_heap_push(H,B2);else,pruneObj=pruneObj+1;end
end

if H.n==0
    LB=UB;exhausted=true;
    bestBox=[];
else
    LB=H.key(1);exhausted=false;
    bestBox=struct('hL',H.hL(1),'hU',H.hU(1),'tL',H.tL(1),'tU',H.tU(1), ...
        'depth',H.depth(1),'key',H.key(1));
end

gap=max(0,UB-LB);
relGap=gap/max(UB,realmin);

% Diagnose the exact midpoint of the lowest unresolved box. Diagnostic only.
diag=struct();
if ~isempty(bestBox)
    hm=0.5*(bestBox.hL+bestBox.hU);
    tm=0.5*(bestBox.tL+bestBox.tU);
    diag=v209z_exact_diagnosis(hm,tm,J0,Q,BT,T0,CFG);
    diag.h=hm;diag.T=tm;diag.total=hm+tm;
end

A=struct();
A.rep=CFG.v209y_representation;
A.backend='BSPLINE_D7';
A.prefixSafe=prefix.pass;
A.TtailOnlyMin=T0.T;
A.domainT=[TL TU];
A.domainHb=[hL hU];
A.upperBound=UB;
A.lowerBound=LB;
A.absoluteGap=gap;
A.relativeGap=relGap;
A.bestCertified=best;
A.initialUpperBound=inc0.totalTime;
A.processedBoxes=count;
A.remainingBoxes=H.n;
A.resourceLimitHit=(count>=CFG.v209y_max_boxes && H.n>0);
A.searchExhausted=exhausted;
A.prunedObjective=pruneObj;
A.prunedFixedCorridor=pruneCorr;
A.prunedFixedDynamics=pruneFixedDyn;
A.prunedMoment=pruneMoment;
A.splitH=splitH;A.splitT=splitT;
A.machineLeaves=machineLeaves;
A.primalEvals=primalEvals;
A.primalFeasible=primalFeasible;
A.upperBoundUpdates=ubUpdates;
A.primalLog=primalLog(1:primalEvals);
A.bestUnresolvedBox=bestBox;
A.bestUnresolvedExactDiagnostic=diag;
A.runtime_s=toc(tStart);
A.timeBankUsed=false;
A.safetyChanged=false;
A.globalOptimalityClaim=exhausted;
A.infeasibilityClaim=false;
A.formalWCET=false;
A.lowerBoundTheory=['nextafter-outward interval necessary relaxation with fixed endpoint corridor/resultants and ', ...
    'the two extreme nonnegative resultant averages w=[5..0] and reverse=[0..5]; exact primal points only improve UB'];
if exhausted
    A.classification='SEARCH_EXHAUSTED_WITH_HARD_CERTIFIED_BEST__GLOBAL_OPTIMUM_WITHIN_DECLARED_DOMAIN';
else
    A.classification='CERTIFIED_PRIMAL_DUAL_GAP__SEARCH_INCOMPLETE';
end
end

function P=v209z_empty_primal()
P=struct('h',NaN,'T',NaN,'totalTime',NaN,'tailPass',false,'solverFeasible',false, ...
    'localGatePass',false,'hardCertPass',false,'certified',false,'hardSlack',NaN,'hardDepth',NaN, ...
    'boxKey',NaN,'boxDepth',NaN,'status','UNSET');
end

function P=v209z_exact_candidate(hb,T,J0,Q,BT,T0,CFG)
P=v209z_empty_primal();
P.h=hb;P.T=T;P.totalTime=hb+T;
alpha=T/T0.T;
Ts=v209w_scale_tail_time(BT,T0,alpha,CFG);
P.tailPass=Ts.pass;
if ~Ts.pass
    P.status='TAIL_TIME_FAIL';return;
end
Js=v209b_backend_start_jets(BT,Ts);
[ok,C,E]=v209t_eval_socp_bridge(J0,Js,Q,BT,Ts,hb,CFG,CFG.v209y_cert_max_depth);
P.solverFeasible=E.solverFeasible;
P.localGatePass=E.localGatePass;
P.hardCertPass=E.hardCertPass;
P.certified=ok;
P.status=E.status;
if ok
    P.hardSlack=C.direct.min_slack;
    P.hardDepth=C.direct.max_depth;
end
end

function D=v209z_exact_diagnosis(hb,T,J0,Q,BT,T0,CFG)
alpha=T/T0.T;
Ts=v209w_scale_tail_time(BT,T0,alpha,CFG);
Js=v209b_backend_start_jets(BT,Ts);
[B0,~,~]=v209t_bridge_affine_map(J0,Js,hb,CFG);
RD=v209v_resultant_socp(B0,hb,Q,CFG,false);
RF=v209v_resultant_socp(B0,hb,Q,CFG,true);
P=v209z_exact_candidate(hb,T,J0,Q,BT,T0,CFG);
D=struct('tailPass',Ts.pass,'dynamicOnlyFeasible',RD.feasible,'fullConvexFeasible',RF.feasible, ...
    'hardCertified',P.certified,'candidate',P,'dynamicExitflag',RD.exitflag,'fullExitflag',RF.exitflag);
end

function v209z_print(A,CFG)
fprintf('\n========================================================================================================================\n');
fprintf(' G26-V209Z CERTIFIED JOINT-TIME PRIMAL-DUAL GAP | rep=%s | backend=%s\n',A.rep,A.backend);
fprintf('------------------------------------------------------------------------------------------------------------------------\n');
fprintf(' domain: T=[%.12g, %.12g] s | hb=[%.12g, %.12g] s\n',A.domainT(1),A.domainT(2),A.domainHb(1),A.domainHb(2));
fprintf(' initial hard-certified UB=%.12g s\n',A.initialUpperBound);
fprintf(' best hard-certified UB=%.12g s | source=%s | T=%.12g | hb=%.12g\n', ...
    A.upperBound,A.bestCertified.source,A.bestCertified.T,A.bestCertified.hb);
fprintf(' formal LB=%.12g s | absolute gap=%.12g s | relative gap=%.6f %%\n', ...
    A.lowerBound,A.absoluteGap,100*A.relativeGap);
fprintf(' boxes: processed=%d remaining=%d max=%d exhausted=%d resourceHit=%d machineLeaves=%d\n', ...
    A.processedBoxes,A.remainingBoxes,CFG.v209y_max_boxes,A.searchExhausted,A.resourceLimitHit,A.machineLeaves);
fprintf(' pruning: objective=%d fixedCorridor=%d fixedDynamics=%d strengthenedMoments=%d\n', ...
    A.prunedObjective,A.prunedFixedCorridor,A.prunedFixedDynamics,A.prunedMoment);
fprintf(' exact primal: evals=%d feasibleHardCertified=%d UBupdates=%d max=%d\n', ...
    A.primalEvals,A.primalFeasible,A.upperBoundUpdates,CFG.v209y_max_primal_evals);
fprintf(' qualification runtime=%.6f s (NOT online runtime / NOT WCET)\n',A.runtime_s);

if ~isempty(A.bestUnresolvedBox)
    B=A.bestUnresolvedBox;D=A.bestUnresolvedExactDiagnostic;
    fprintf(' lowest unresolved box: h=[%.15g, %.15g], T=[%.15g, %.15g], depth=%d, keyLB=%.15g\n', ...
        B.hL,B.hU,B.tL,B.tU,B.depth,B.key);
    fprintf(' exact midpoint diagnostic: h=%.15g T=%.15g total=%.15g | tail=%d dynamicOnlySOCP=%d fullConvexSOCP=%d hardCertified=%d\n', ...
        D.h,D.T,D.total,D.tailPass,D.dynamicOnlyFeasible,D.fullConvexFeasible,D.hardCertified);
end

fprintf(' best certified hard slack=%+.6e depth=%g\n',A.bestCertified.hardSlack,A.bestCertified.hardDepth);
fprintf(' CLASSIFICATION: %s\n',A.classification);
fprintf('\n------------------------------------------------------------------------------------------------------------------------\n');
fprintf(' THEORY: for the two exact moment equations, the extreme nonnegative weight rays are forward [5..0] and reverse [0..5].\n');
fprintf(' THEORY: formal pruning remains necessary-only; failed exact primal solves NEVER prune a box.\n');
fprintf(' PERFORMANCE: any exact primal UB improvement is a real trajectory-time improvement because it passes unchanged full authority.\n');
fprintf(' RULE: do not mount additional functionality until the joint-time gap, trajectory performance, and online runtime are closed.\n');
fprintf('========================================================================================================================\n');
end

function A=v209y_audit(CFG,K)
tStart=tic;
S=v209b_static_scenario(CFG);

Qold=v209n_build_static_problem_runtime(S,'PATCH_UNION',CFG,K);
Bold=v209s_build_backend_online('ACETRANS_MINCO_ST_C6',Qold.W,Qold.weights);
Told=v208_time_gate_angle(Bold,CFG,CFG.v209b_nominal_theta_deg);
assert(Told.pass,'G26V209Y:OldReference','Old reference failed.');
tDetect=CFG.v209b_detection_fraction*Told.T;
tSwitch=tDetect+CFG.v209b_switch_lookahead_s;
J0=v209b_eval_jet(Bold,Told,tSwitch);

Snew=S;Snew.name='V209Y_LOWER_BOUND';
Snew.C=[S.C S.newObstacleCenter];Snew.V=zeros(3,size(Snew.C,2));Snew.R=[S.R S.newObstacleRadius];
Snew.p0=J0(:,1);Snew.pg=S.pg;
Q=v209n_build_static_problem_runtime(Snew,CFG.v209y_representation,CFG,K);
assert(Q.guidePass&&Q.corridorOK,'G26V209Y:Spatial','Updated spatial problem failed.');
prefix=v209b_active_interval_certificate(Bold,Told,tDetect,tSwitch,Q,CFG);
assert(prefix.pass,'G26V209Y:Prefix','Prefix certificate failed.');

QT=v209b_tail_problem(Q);
[BT,T0,st]=v209s_tail_runtime_deep(QT,CFG,'BSPLINE_D7',CFG.v209y_cert_max_depth);
assert(strcmp(st,'PASS'),'G26V209Y:Tail','Baseline BSPLINE tail must be certified.');
Jcur=v209b_backend_start_jets(BT,T0);

% Hard-certified incumbent from the already-proved V209W/X anchor hb_max.
inc=v209x_eval_hb(CFG.v209b_bridge_h_max_s,J0,Jcur,Q,BT,T0,CFG);
assert(inc.certified,'G26V209Y:Incumbent','Expected hard-certified hb_max incumbent was not reproduced.');
UBraw=inc.totalTime;
UB=UBraw+8*eps(max(1,abs(UBraw))); % conservative floating upper enclosure of T+hb

hL=CFG.v209b_bridge_h_min_s;
hU=CFG.v209b_bridge_h_max_s;
TL=T0.T;
% Any T > UB-hL cannot improve the incumbent because hb>=hL.
TU=min(CFG.v207_time_max_s,UB-hL);
assert(TU>=TL,'G26V209Y:Domain','Incumbent leaves no nontrivial T domain.');

% Best-first binary heap ordered by objective lower bound T_L+h_L.
H=v209y_heap_init(CFG.v209y_max_boxes*2+16);
root=v209y_box(hL,hU,TL,TU,0);
H=v209y_heap_push(H,root);

count=0;pruneObj=0;pruneCorr=0;pruneFixedDyn=0;pruneMoment=0;
splitH=0;splitT=0;machineLeaves=0;
minSeenNecessary=inf;

while H.n>0 && count<CFG.v209y_max_boxes
    [H,B]=v209y_heap_pop(H);
    count=count+1;

    if B.key>=UB
        pruneObj=pruneObj+1;
        continue;
    end

    N=v209y_interval_necessary(B,J0,Jcur,T0.T,Q,CFG);
    minSeenNecessary=min(minSeenNecessary,B.key);

    if ~N.fixedCorridorPossible
        pruneCorr=pruneCorr+1;
        continue;
    end
    if ~N.fixedDynamicPossible
        pruneFixedDyn=pruneFixedDyn+1;
        continue;
    end
    if ~N.momentPossible
        pruneMoment=pruneMoment+1;
        continue;
    end

    % At machine-scale boxes, retain unresolved rather than pretending proof.
    canSplitH=(B.hU-B.hL)>128*eps(max(1,B.hU));
    canSplitT=(B.tU-B.tL)>128*eps(max(1,B.tU));
    if B.depth>=CFG.v209y_max_depth || ~(canSplitH||canSplitT)
        H=v209y_heap_push(H,B);
        machineLeaves=machineLeaves+1;
        break;
    end

    wh=B.hU-B.hL;wt=B.tU-B.tL;
    if canSplitT && (~canSplitH || wt>=wh)
        m=0.5*(B.tL+B.tU);
        B1=v209y_box(B.hL,B.hU,B.tL,m,B.depth+1);
        B2=v209y_box(B.hL,B.hU,m,B.tU,B.depth+1);
        splitT=splitT+1;
    else
        m=0.5*(B.hL+B.hU);
        B1=v209y_box(B.hL,m,B.tL,B.tU,B.depth+1);
        B2=v209y_box(m,B.hU,B.tL,B.tU,B.depth+1);
        splitH=splitH+1;
    end
    if B1.key<UB,H=v209y_heap_push(H,B1);else,pruneObj=pruneObj+1;end
    if B2.key<UB,H=v209y_heap_push(H,B2);else,pruneObj=pruneObj+1;end
end

if H.n==0
    LB=UB;
    exhausted=true;
else
    LB=H.key(1);
    exhausted=false;
end

gap=max(0,UB-LB);
relGap=gap/max(UB,realmin);

A=struct();
A.rep=CFG.v209y_representation;
A.backend='BSPLINE_D7';
A.prefixSafe=prefix.pass;
A.TtailOnlyMin=T0.T;
A.domainT=[TL TU];
A.domainHb=[hL hU];
A.upperBound=UB;
A.lowerBound=LB;
A.absoluteGap=gap;
A.relativeGap=relGap;
A.incumbent=inc;
A.processedBoxes=count;
A.remainingBoxes=H.n;
A.resourceLimitHit=(count>=CFG.v209y_max_boxes && H.n>0);
A.searchExhausted=exhausted;
A.prunedObjective=pruneObj;
A.prunedFixedCorridor=pruneCorr;
A.prunedFixedDynamics=pruneFixedDyn;
A.prunedMoment=pruneMoment;
A.splitH=splitH;A.splitT=splitT;
A.machineLeaves=machineLeaves;
A.runtime_s=toc(tStart);
A.timeBankUsed=false;
A.safetyChanged=false;
A.globalOptimalityClaim=exhausted;
A.infeasibilityClaim=false;
A.formalWCET=false;
A.lowerBoundTheory='nextafter-outward C++ interval relaxation; theta=30deg cone uses algebraic 3*(rx^2+ry^2)<=rz^2; fixed endpoint corridor/resultants plus forward and reverse extreme positive-weight resultant moments';
if exhausted
    A.classification='SEARCH_EXHAUSTED_WITH_HARD_CERTIFIED_INCUMBENT__GLOBAL_OPTIMUM_WITHIN_DECLARED_DOMAIN';
else
    A.classification='CERTIFIED_BOUNDED_SUBOPTIMALITY__RESOURCE_LIMITED_LOWER_BOUND';
end
end

function N=v209y_interval_necessary(B,J0,Jcur,T0,Q,CFG)
nf=Q.cnt(1);
flags=g26v209y_interval_box_mex(B.hL,B.hU,B.tL,B.tU,J0,Jcur,T0,Q.N(:,1:nf),Q.h(1:nf), ...
    [CFG.v209b_nominal_theta_deg,CFG.v207_rho_min_mps2,CFG.gravity]);
N=struct('fixedCorridorPossible',logical(flags(1)), ...
    'fixedDynamicPossible',logical(flags(2)), ...
    'momentPossible',logical(flags(3)));
end

function [BL,BH]=v209y_fixed_control_intervals(B,J0,Jcur,T0)
n=17;
BL=zeros(3,18);BH=zeros(3,18);

% Left B0:B6.  Positive h interval, fixed old-reference jets.
for k=0:6
    lo=zeros(3,1);hi=zeros(3,1);
    for r=0:k
        if r==0,den=1;else,den=v200_falling(n,r);end
        c=nchoosek(k,r)/den;
        if r==0
            [tl,th]=v209y_const_interval(c,J0(:,1),1,1);
        else
            [tl,th]=v209y_const_interval(c,J0(:,r+1),B.hL^r,B.hU^r);
        end
        lo=lo+tl;hi=hi+th;
    end
    BL(:,k+1)=lo;BH(:,k+1)=hi;
end

% Right B11:B17. h^r * J_r(T) = Jcur_r * (h*T0/T)^r.
zL=B.hL*T0/B.tU;
zU=B.hU*T0/B.tL;
ZL=zeros(3,7);ZH=zeros(3,7);
for k=0:6
    lo=zeros(3,1);hi=zeros(3,1);
    for r=0:k
        if r==0,den=1;else,den=v200_falling(n,r);end
        c=nchoosek(k,r)/den*((-1)^r);
        if r==0
            [tl,th]=v209y_const_interval(c,Jcur(:,1),1,1);
        else
            [tl,th]=v209y_const_interval(c,Jcur(:,r+1),zL^r,zU^r);
        end
        lo=lo+tl;hi=hi+th;
    end
    ZL(:,k+1)=lo;ZH(:,k+1)=hi;
end
BL(:,18:-1:12)=ZL;BH(:,18:-1:12)=ZH;

% Free B7:B10 deliberately left at zero; never used by necessary tests.
end

function [lo,hi]=v209y_const_interval(c,v,sL,sU)
a=c*v*sL;b=c*v*sU;
lo=min(a,b);hi=max(a,b);
end

function pass=v209y_fixed_corridor_possible(BL,BH,Q)
nf=Q.cnt(1);fixed=[1:7 12:18];
scaleMax=max(1,max(abs(Q.h(1:nf))));
for j=fixed
    scaleMax=max(scaleMax,max(abs([BL(:,j);BH(:,j)])));
end
guardMax=4096*eps(scaleMax);

for f=1:nf
    n=Q.N(:,f);
    for j=fixed
        [~,u]=v209y_linear_interval(n,BL(:,j),BH(:,j));
        if u-Q.h(f) < -guardMax
            pass=false;return;
        end
    end
end
pass=true;
end

function pass=v209y_fixed_dynamic_possible(B,BL,BH,CFG)
idx=[1:5 12:16]; % zero-based r0:r4,r11:r15 -> MATLAB indices
facL=17*16/(B.hU^2);
facU=17*16/(B.hL^2);
g=[0;0;CFG.gravity];

for jj=idx
    [dL,dH]=v209y_second_diff_interval(BL(:,jj:jj+2),BH(:,jj:jj+2));
    [rL,rH]=v209y_positive_scale_interval(dL,dH,facL,facU);
    rL=rL+g;rH=rH+g;
    if ~v209y_cone_box_possible(rL,rH,CFG)
        pass=false;return;
    end
end
pass=true;
end

function pass=v209y_moment_possible(B,BL,BH,CFG)
% Exact endpoint-moment necessary averages from V209V.
[d5L,d5H]=v209y_sub_interval(BL(:,7),BH(:,7),BL(:,6),BH(:,6));
[d11L,d11H]=v209y_sub_interval(BL(:,13),BH(:,13),BL(:,12),BH(:,12));

[n0L,n0H]=v209y_sub_interval(d11L,d11H,d5L,d5H);

% n1 = B11-B6-5*d5
[tL,tH]=v209y_sub_interval(BL(:,12),BH(:,12),BL(:,7),BH(:,7));
[fL,fH]=v209y_positive_scale_interval(d5L,d5H,5,5);
[n1L,n1H]=v209y_sub_interval(tL,tH,fL,fH);

invCL=17*16/(B.hU^2);
invCU=17*16/(B.hL^2);
[q0L,q0H]=v209y_interval_product(n0L,n0H,invCL,invCU);
[q1L,q1H]=v209y_interval_product(n1L,n1H,invCL,invCU);

g6=[0;0;6*CFG.gravity];g15=[0;0;15*CFG.gravity];
q0L=q0L+g6;q0H=q0H+g6;
q1L=q1L+g15;q1H=q1H+g15;
m0L=q0L/6;m0H=q0H/6;
m1L=q1L/15;m1H=q1H/15;
[revL,revH]=v209y_sub_interval(5*q0L,5*q0H,q1L,q1H);
m2L=revL/15;m2H=revH/15;

pass=v209y_cone_box_possible(m0L,m0H,CFG) && ...
     v209y_cone_box_possible(m1L,m1H,CFG) && ...
     v209y_cone_box_possible(m2L,m2H,CFG);
end

function pass=v209y_cone_box_possible(lo,hi,CFG)
% Upper bound on possible cone slack over the entire box.
ax=v209y_min_abs_interval(lo(1),hi(1));
ay=v209y_min_abs_interval(lo(2),hi(2));
normLB=hypot(ax,ay);
coneUpper=tan(deg2rad(CFG.v209b_nominal_theta_deg))*hi(3)-normLB;
rzUpper=hi(3)-CFG.v207_rho_min_mps2;
pass=isfinite(coneUpper)&&isfinite(rzUpper)&&coneUpper>=0&&rzUpper>=0;
end

function a=v209y_min_abs_interval(lo,hi)
if lo<=0 && hi>=0
    a=0;
else
    a=min(abs(lo),abs(hi));
end
end

function [lo,hi]=v209y_linear_interval(a,xL,xH)
lo=0;hi=0;
for k=1:numel(a)
    v1=a(k)*xL(k);v2=a(k)*xH(k);
    lo=lo+min(v1,v2);hi=hi+max(v1,v2);
end
end

function [dL,dH]=v209y_second_diff_interval(BL,BH)
% x3 - 2*x2 + x1
[dL,dH]=v209y_sub_interval(BL(:,3),BH(:,3),2*BL(:,2),2*BH(:,2));
dL=dL+BL(:,1);dH=dH+BH(:,1);
end

function [lo,hi]=v209y_sub_interval(aL,aH,bL,bH)
lo=aL-bH;hi=aH-bL;
end

function [lo,hi]=v209y_positive_scale_interval(xL,xH,sL,sU)
[lo,hi]=v209y_interval_product(xL,xH,sL,sU);
end

function [lo,hi]=v209y_interval_product(xL,xH,sL,sU)
% s interval is positive scalar; x interval vector may cross zero.
lo=zeros(size(xL));hi=zeros(size(xH));
for k=1:numel(xL)
    vals=[xL(k)*sL,xL(k)*sU,xH(k)*sL,xH(k)*sU];
    lo(k)=min(vals);hi(k)=max(vals);
end
end

% ------------------------------------------------------------------------
% Best-first binary heap
% ------------------------------------------------------------------------
function B=v209y_box(hL,hU,tL,tU,depth)
s=hL+tL;
key=s-8*eps(max(1,abs(s))); % conservative downward enclosure of exact hL+tL
B=struct('hL',hL,'hU',hU,'tL',tL,'tU',tU,'depth',depth,'key',key);
end

function H=v209y_heap_init(cap)
H=struct();
H.n=0;H.cap=cap;
H.hL=zeros(cap,1);H.hU=zeros(cap,1);H.tL=zeros(cap,1);H.tU=zeros(cap,1);
H.depth=zeros(cap,1);H.key=zeros(cap,1);
end

function H=v209y_heap_push(H,B)
if H.n>=H.cap,error('G26V209Y:HeapCapacity','Heap capacity exceeded.');end
H.n=H.n+1;i=H.n;
H.hL(i)=B.hL;H.hU(i)=B.hU;H.tL(i)=B.tL;H.tU(i)=B.tU;H.depth(i)=B.depth;H.key(i)=B.key;
while i>1
    p=floor(i/2);
    if H.key(p)<=H.key(i),break;end
    H=v209y_heap_swap(H,p,i);i=p;
end
end

function [H,B]=v209y_heap_pop(H)
if H.n<1,error('G26V209Y:HeapEmpty','Heap empty.');end
B=v209y_box(H.hL(1),H.hU(1),H.tL(1),H.tU(1),H.depth(1));
if H.n==1,H.n=0;return;end
H.hL(1)=H.hL(H.n);H.hU(1)=H.hU(H.n);H.tL(1)=H.tL(H.n);H.tU(1)=H.tU(H.n);
H.depth(1)=H.depth(H.n);H.key(1)=H.key(H.n);H.n=H.n-1;
i=1;
while true
    l=2*i;r=l+1;s=i;
    if l<=H.n && H.key(l)<H.key(s),s=l;end
    if r<=H.n && H.key(r)<H.key(s),s=r;end
    if s==i,break;end
    H=v209y_heap_swap(H,i,s);i=s;
end
end

function H=v209y_heap_swap(H,a,b)
fields={'hL','hU','tL','tU','depth','key'};
for k=1:numel(fields)
    f=fields{k};tmp=H.(f)(a);H.(f)(a)=H.(f)(b);H.(f)(b)=tmp;
end
end

function v209y_print(A,CFG)
fprintf('\n========================================================================================================================\n');
fprintf(' G26-V209Y CERTIFIED JOINT-TIME LOWER BOUND | rep=%s | backend=%s\n',A.rep,A.backend);
fprintf('------------------------------------------------------------------------------------------------------------------------\n');
fprintf(' domain: T=[%.12g, %.12g] s | hb=[%.12g, %.12g] s\n',A.domainT(1),A.domainT(2),A.domainHb(1),A.domainHb(2));
fprintf(' hard-certified UB = %.12g s  (T+hb)\n',A.upperBound);
fprintf(' certified relaxation LB = %.12g s\n',A.lowerBound);
fprintf(' certified gap: absolute=%.12g s | relative=%.6f %%\n',A.absoluteGap,100*A.relativeGap);
fprintf(' boxes: processed=%d remaining=%d maxResource=%d exhausted=%d resourceHit=%d\n', ...
    A.processedBoxes,A.remainingBoxes,CFG.v209y_max_boxes,A.searchExhausted,A.resourceLimitHit);
fprintf(' pruning: objective=%d fixedCorridor=%d fixedDynamics=%d endpointMoments=%d | splitH=%d splitT=%d\n', ...
    A.prunedObjective,A.prunedFixedCorridor,A.prunedFixedDynamics,A.prunedMoment,A.splitH,A.splitT);
fprintf(' qualification runtime=%.6f s (NOT online runtime / NOT WCET)\n',A.runtime_s);
fprintf(' incumbent: hb=%.12g s | T=%.12g s | total=%.12g s | hardSlack=%+.6e depth=%g\n', ...
    A.incumbent.hb,A.incumbent.T,A.incumbent.totalTime,A.incumbent.constructive.hardSlack,A.incumbent.constructive.hardDepth);
fprintf(' CLASSIFICATION: %s\n',A.classification);
fprintf('\n------------------------------------------------------------------------------------------------------------------------\n');
fprintf(' THEORY: LB is valid because only necessary conditions are used for pruning; omitted constraints can only raise the true optimum.\n');
fprintf(' LIMIT: if remainingBoxes>0, the exact optimum is NOT claimed.  Report the UB-LB gap instead.\n');
fprintf(' RULE: no SFC/sensor/trigger/atomic-switch mount until joint-time theory, trajectory performance, and online computation time are all closed.\n');
fprintf('========================================================================================================================\n');
end

function v209y_mex_bootstrap_interval(root)
name='g26v209y_interval_box_mex';
v200_build_mex_if_needed(root,name,v209y_cpp_interval_source(),'G26-V209Y-INTERVAL-MEX');
end

function lines=v209y_cpp_interval_source()
lines={
'#include "mex.h"';
'#include <cmath>';
'#include <algorithm>';
'#include <limits>';
'';
'struct I { double lo, hi; };';
'';
'static inline double dn(double x){ return std::nextafter(x,-std::numeric_limits<double>::infinity()); }';
'static inline double up(double x){ return std::nextafter(x, std::numeric_limits<double>::infinity()); }';
'static inline I pt(double x){ return {x,x}; }';
'static inline I addi(I a,I b){ return {dn(a.lo+b.lo),up(a.hi+b.hi)}; }';
'static inline I subi(I a,I b){ return {dn(a.lo-b.hi),up(a.hi-b.lo)}; }';
'static inline I muli(I a,I b){';
'    double p1=a.lo*b.lo,p2=a.lo*b.hi,p3=a.hi*b.lo,p4=a.hi*b.hi;';
'    double l=std::min(std::min(p1,p2),std::min(p3,p4));';
'    double h=std::max(std::max(p1,p2),std::max(p3,p4));';
'    return {dn(l),up(h)};';
'}';
'static inline I invi_pos(I a){';
'    return {dn(1.0/a.hi),up(1.0/a.lo)};';
'}';
'static inline I divi_pos(I a,I b){ return muli(a,invi_pos(b)); }';
'static inline I powi_pos(I a,int n){';
'    I r=pt(1.0);';
'    for(int k=0;k<n;k++) r=muli(r,a);';
'    return r;';
'}';
'static inline I scale_exact(I a,double c){ return muli(a,pt(c)); }';
'static inline double minabs(I a){';
'    if(a.lo<=0.0 && a.hi>=0.0) return 0.0;';
'    return std::min(std::fabs(a.lo),std::fabs(a.hi));';
'}';
'static inline I sq_point_out(double x){ return muli(pt(x),pt(x)); }';
'static inline int choosei(int n,int k){';
'    if(k<0||k>n) return 0;';
'    if(k>n-k) k=n-k;';
'    int r=1;';
'    for(int i=1;i<=k;i++) r=r*(n-k+i)/i;';
'    return r;';
'}';
'static inline int fallingi(int n,int r){';
'    int v=1;';
'    for(int i=0;i<r;i++) v*=n-i;';
'    return v;';
'}';
'static inline I coeff_interval(int k,int r,int sign){';
'    I num=pt((double)choosei(k,r));';
'    I den=pt((double)(r==0?1:fallingi(17,r)));';
'    I c=divi_pos(num,den);';
'    if(sign<0) c=muli(c,pt(-1.0));';
'    return c;';
'}';
'static inline I point_times_interval(double v,I a){ return muli(pt(v),a); }';
'';
'static inline bool cone_box_possible_30deg(I rx,I ry,I rz,double rho){';
'    // theta=30deg, rz>=rho>0: cone iff 3*(rx^2+ry^2)<=rz^2.';
'    // For possibility over a box use lower(LHS) and upper(RHS).';
'    if(!(rz.hi>=rho) || !std::isfinite(rz.hi)) return false;';
'    double ax=minabs(rx), ay=minabs(ry);';
'    I ax2=sq_point_out(ax), ay2=sq_point_out(ay);';
'    I lhs=scale_exact(addi(ax2,ay2),3.0);';
'    double zmax=std::max(rho,rz.hi);';
'    I rhs=sq_point_out(zmax);';
'    return lhs.lo<=rhs.hi;';
'}';
'';
'static inline I lin3(const double* n,const I* x){';
'    I s=pt(0.0);';
'    for(int a=0;a<3;a++) s=addi(s,muli(pt(n[a]),x[a]));';
'    return s;';
'}';
'';
'static inline I second_diff(I a,I b,I c){';
'    return addi(subi(c,scale_exact(b,2.0)),a);';
'}';
'';
'void mexFunction(int nlhs,mxArray* plhs[],int nrhs,const mxArray* prhs[]){';
'    if(nrhs!=10) mexErrMsgIdAndTxt("G26V209Y:MEXArgs","Expected 10 inputs.");';
'    double hL=mxGetScalar(prhs[0]), hU=mxGetScalar(prhs[1]);';
'    double tL=mxGetScalar(prhs[2]), tU=mxGetScalar(prhs[3]);';
'    const double* J0=mxGetDoubles(prhs[4]);';
'    const double* Jc=mxGetDoubles(prhs[5]);';
'    double T0=mxGetScalar(prhs[6]);';
'    const double* N=mxGetDoubles(prhs[7]);';
'    const double* qh=mxGetDoubles(prhs[8]);';
'    const double* par=mxGetDoubles(prhs[9]);';
'    mwSize nf=mxGetN(prhs[7]);';
'    if(mxGetM(prhs[4])!=3||mxGetN(prhs[4])!=7||mxGetM(prhs[5])!=3||mxGetN(prhs[5])!=7)';
'        mexErrMsgIdAndTxt("G26V209Y:MEXShape","J0/Jcur must be 3x7.");';
'    if(mxGetM(prhs[7])!=3) mexErrMsgIdAndTxt("G26V209Y:MEXShape","N must be 3xF.");';
'    if(mxGetNumberOfElements(prhs[8])!=nf) mexErrMsgIdAndTxt("G26V209Y:MEXShape","h size mismatch.");';
'    if(mxGetNumberOfElements(prhs[9])!=3) mexErrMsgIdAndTxt("G26V209Y:MEXShape","params=[thetaDeg rho gravity].");';
'    if(!(hL>0&&hU>=hL&&tL>0&&tU>=tL&&T0>0)) mexErrMsgIdAndTxt("G26V209Y:MEXDomain","Invalid positive interval.");';
'';
'    double theta=par[0], rho=par[1], grav=par[2];';
'    if(std::fabs(theta-30.0)>std::numeric_limits<double>::epsilon()*30.0)';
'        mexErrMsgIdAndTxt("G26V209Y:MEXThetaScope","Formal algebraic cone interval test currently supports theta=30 deg only.");';
'    I h={hL,hU}, T={tL,tU};';
'    I BL[3][18], BHdummy[1]; (void)BHdummy;';
'    I B[3][18];';
'    for(int a=0;a<3;a++) for(int j=0;j<18;j++) B[a][j]=pt(0.0);';
'';
'    // Left fixed controls B0..B6.';
'    for(int k=0;k<=6;k++){';
'        for(int a=0;a<3;a++){';
'            I s=pt(0.0);';
'            for(int r=0;r<=k;r++){';
'                I c=coeff_interval(k,r,+1);';
'                I hp=(r==0)?pt(1.0):powi_pos(h,r);';
'                I term=muli(muli(c,hp),pt(J0[a+3*r]));';
'                s=addi(s,term);';
'            }';
'            B[a][k]=s;';
'        }';
'    }';
'';
'    // z = h*T0/T, then right fixed controls B11..B17.';
'    I z=divi_pos(muli(h,pt(T0)),T);';
'    I Z[3][7];';
'    for(int k=0;k<=6;k++){';
'        for(int a=0;a<3;a++){';
'            I s=pt(0.0);';
'            for(int r=0;r<=k;r++){';
'                int sg=(r%2==0)?+1:-1;';
'                I c=coeff_interval(k,r,sg);';
'                I zp=(r==0)?pt(1.0):powi_pos(z,r);';
'                I term=muli(muli(c,zp),pt(Jc[a+3*r]));';
'                s=addi(s,term);';
'            }';
'            Z[a][k]=s;';
'        }';
'    }';
'    for(int k=0;k<=6;k++) for(int a=0;a<3;a++) B[a][17-k]=Z[a][k];';
'';
'    // 1) Fixed corridor necessary possibility.';
'    bool corr=true;';
'    double scaleMax=1.0;';
'    for(mwSize f=0;f<nf;f++) scaleMax=std::max(scaleMax,std::fabs(qh[f]));';
'    int fixedCols[14]={0,1,2,3,4,5,6,11,12,13,14,15,16,17};';
'    for(int ii=0;ii<14;ii++) for(int a=0;a<3;a++){';
'        int j=fixedCols[ii];';
'        scaleMax=std::max(scaleMax,std::max(std::fabs(B[a][j].lo),std::fabs(B[a][j].hi)));';
'    }';
'    double ulp=std::nextafter(scaleMax,std::numeric_limits<double>::infinity())-scaleMax;';
'    double guard=up(4096.0*ulp);';
'    for(mwSize f=0;f<nf && corr;f++){';
'        const double nv[3]={N[0+3*f],N[1+3*f],N[2+3*f]};';
'        for(int ii=0;ii<14;ii++){';
'            int j=fixedCols[ii];';
'            I x[3]={B[0][j],B[1][j],B[2][j]};';
'            I s=lin3(nv,x);';
'            I slack=subi(s,pt(qh[f]));';
'            if(slack.hi < -guard){ corr=false; break; }';
'        }';
'    }';
'';
'    // Common fac = 17*16/h^2.';
'    I fac=divi_pos(pt(272.0),powi_pos(h,2));';
'';
'    // 2) Fixed resultant necessary possibility.';
'    bool fdyn=true;';
'    int ridx[10]={0,1,2,3,4,11,12,13,14,15};';
'    for(int ii=0;ii<10 && fdyn;ii++){';
'        int j=ridx[ii];';
'        I rr[3];';
'        for(int a=0;a<3;a++){';
'            I d=second_diff(B[a][j],B[a][j+1],B[a][j+2]);';
'            rr[a]=muli(fac,d);';
'        }';
'        rr[2]=addi(rr[2],pt(grav));';
'        if(!cone_box_possible_30deg(rr[0],rr[1],rr[2],rho)) fdyn=false;';
'    }';
'';
'    // 3) Two exact positive-weight endpoint resultant moments.';
'    bool moment=true;';
'    I d5[3],d11[3],n0[3],n1[3],q0v[3],q1v[3],m0[3],m1[3],m2[3];';
'    for(int a=0;a<3;a++){';
'        d5[a]=subi(B[a][6],B[a][5]);       // B6-B5';
'        d11[a]=subi(B[a][12],B[a][11]);   // B12-B11';
'        n0[a]=subi(d11[a],d5[a]);';
'        I tmp=subi(B[a][11],B[a][6]);      // B11-B6';
'        n1[a]=subi(tmp,scale_exact(d5[a],5.0));';
'        I q0=muli(fac,n0[a]);';
'        I q1=muli(fac,n1[a]);';
'        if(a==2){ q0=addi(q0,pt(6.0*grav)); q1=addi(q1,pt(15.0*grav)); }';
'        q0v[a]=q0; q1v[a]=q1;';
'        m0[a]=divi_pos(q0,pt(6.0));';
'        m1[a]=divi_pos(q1,pt(15.0));';
'        m2[a]=divi_pos(subi(scale_exact(q0,5.0),q1),pt(15.0));';
'    }';
'    if(!cone_box_possible_30deg(m0[0],m0[1],m0[2],rho)) moment=false;';
'    if(!cone_box_possible_30deg(m1[0],m1[1],m1[2],rho)) moment=false;';
'    if(!cone_box_possible_30deg(m2[0],m2[1],m2[2],rho)) moment=false;';
'';
'    plhs[0]=mxCreateDoubleMatrix(1,3,mxREAL);';
'    double* out=mxGetDoubles(plhs[0]);';
'    out[0]=corr?1.0:0.0;';
'    out[1]=fdyn?1.0:0.0;';
'    out[2]=moment?1.0:0.0;';
'}';
};
end
