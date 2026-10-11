function OUT = CURRENT_C6_STEP4B3_DEPTH_CAMERA_REPLAN_LIFECYCLE_AUDIT_V64_SCOPE_SAFE_COMPACT_LOG(userCfg)
clc; % V30: clear Command Window before any audit output
b3_event_schema_precheck(); % V35: fail before simulation if canonical event schemas regress
% CURRENT_C6_STEP4B3_DEPTH_CAMERA_REPLAN_LIFECYCLE_AUDIT_V64_SCOPE_SAFE_COMPACT_LOG
%
% STEP 4-B3: DEPTH-CAMERA -> MEMORY -> CURRENT-REFERENCE COLLISION PREDICTION
%             -> CORRIDOR REBUILD -> C6 REPLAN -> ATOMIC REFERENCE SWITCH
%
% This is the continuation of Step 4-B2 after the nonzero start-jet audit.
% It intentionally fixes the information-leakage flaw of the former V3 sensor
% replay: NO obstacle is given to the planner before the forward depth camera
% has actually observed it.
%
% HARD ARCHITECTURE RULES IN THIS VALIDATOR
% -----------------------------------------
%  1) known obstacle memory starts EMPTY.
%  2) This B3 file NEVER calls ENVIRONMENT_OBSTACLE_ELLIPSE_MOVE directly.
%     The only environment ingress is the internal b3_depth_camera_fast() section.
%  3) Only visible_obstacles returned by the sensor may update obstacle memory.
%     The sensor's known_obstacles field is deliberately NOT consumed here,
%     because it is generated from simulator ground truth at the current time.
%  4) Collision prediction is performed on the CURRENT EXECUTING REFERENCE.
%     During avoidance it is never silently replaced by nominal-only prediction.
%  5) A new trajectory starts from the current executing physical-time jet
%     d0..d6 and returns to the nominal physical-time jet d0..d6.
%  6) The corridor is rebuilt from the obstacle memory available at that event.
%  7) A new reference becomes active only after the backend solver succeeds and
%     the original hard halfspace slack is nonnegative. Switching is atomic.
%  8) Sensor acquisition continues during avoidance. Newly visible obstacles can
%     therefore trigger another B3 replan from the already-avoiding reference.
%  9) BEZIER_D13, POLY_D13 and BSPLINE_D7 remain in parallel.
% 10) NO runtime safety filter, NO HOruntime safety filter, NO MPC, NO LiDAR and NO emergency special-case planner.
%
% SENSOR
% ------
% Uses the internal forward depth-camera module:
%   forward pinhole depth camera, 640x480, finite FOV, 0.2--5.0 m depth,
%   pixel skip=2, nominal 30 Hz, nearest-hit occlusion. Finite cylinders in
%   ENVIRONMENT_OBSTACLE_ELLIPSE_MOVE are raycast as raw cylinders by the
%   sensor module. Once visible, the existing oriented-ellipsoid planner model
%   no true obstacle center/radii/orientation/velocity are retained in planner memory; velocity is estimated from successive observed models.
%
% MOVING-OBSTACLE SAFETY MODEL
% ----------------------------
% For an obstacle with fixed orientation/shape and constant remembered velocity
% c(t)=c0+v*t, the support over a cell time interval [ta,tb] is exact:
%
%   h_swept(n;ta,tb) = n'c0 + sqrt(n'Sn)
%                       + max(ta*n'v, tb*n'v).
%
% Every trajectory cell is constrained by one separating halfspace per known
% obstacle against that interval-swept set. Because each Bezier/B-spline piece
% is contained in the convex hull of its constrained control points, the hard
% halfspace certificate is continuous over the whole cell, not a sampled
% collision acceptance test.
%
% CURRENT SUSPENDED-SYSTEM SCOPE
% ------------------------------
% V37 retains the vertical proxy ONLY for initial corridor construction, but
% it is no longer the final safety authority.  The executed / predicted rigid
% cable is massless, inextensible and taut, with q pointing UAV -> payload:
%
%   q = -(pL_ddot + g*e3)/||pL_ddot + g*e3||,   p_Q = p_L - L_cable*q.
%
% After a backend trajectory is solved, every local Bezier proof piece receives
% a continuous p-cone derived from its acceleration control points.  The final
% hard acceptance recomputes UAV/cable support against that cone.  Thus a
% trajectory cannot be accepted merely because the vertical-cable proxy was
% safe.  HLC tracking error, actuator saturation, runtime safety filter and self-occlusion of the
% physical camera mount remain separate later layers.
%
% NOMINAL REFERENCE
% -----------------
% Default standalone mission is a straight C-infinity reference from p0 with
% constant velocity. This mission definition is independent of obstacle ground
% truth, so it cannot leak obstacle locations into the reference. Pass a struct
% userCfg to override p0, nominal_direction, nominal_speed or mission_distance.
%
% OUTPUT / LOGGING
% ----------------
% For every backend, the result stores:
%   - execution time and reference d0..d6 at every control tick,
%   - active reference ID and known-obstacle count,
%   - every sensor event (visible/new IDs, known before/after, sensor runtime),
%   - every predictor/replan event (risk IDs/TTC, start/end jets, T, corridor,
%     solver feasibility, hard slack, C6 residuals, planning runtime, ref ID),
%   - whether a replan occurred while an avoidance reference was already active.
%
% REQUIREMENTS
% ------------
%   Optimization Toolbox (quadprog)
%   embedded b3_environment_obstacle_ellipse_move() [accessed ONLY inside internal sensor section]
%
% Example:
%   R = CURRENT_C6_STEP4B3_DEPTH_CAMERA_REPLAN_LIFECYCLE_AUDIT_V64_SCOPE_SAFE_COMPACT_LOG();
%
% Optional:
%   cfg = struct('nominal_speed',1.2,'mission_distance',50, ...
%       'tracking_position_error_bound_m',0.10, ... % use only if formally justified
%       'obstacle_position_error_bound_m',0.05);   % use only if formally justified
%   R = CURRENT_C6_STEP4B3_DEPTH_CAMERA_REPLAN_LIFECYCLE_AUDIT_V64_SCOPE_SAFE_COMPACT_LOG(cfg);

if nargin < 1 || isempty(userCfg)
    userCfg=struct();
end

P=aud_params();
CFG=b3_default_config(P);
CFG=b3_merge_config(CFG,userCfg);
b3_validate_config(CFG,P);
P=b3_apply_robust_margins(P,CFG);
b3_rigid_cable_model_precheck(P,CFG);
GUARANTEE=b3_guarantee_ledger(CFG,P);
b3_detail_fprintf('__SET__',~CFG.compact_log);
b3_require_dependencies();
b3_sensor_shape_preflight();
b3_local_callgraph_precheck();
b3_source_regression_precheck();
b3_primal_distance_cause_isolation_suite(P);
b3_dynamic_geometry_identity_precheck(P,CFG);

backendNames={'BEZIER_D13','POLY_D13','BSPLINE_D7'};
RUNS_CELL=cell(1,numel(backendNames));

fprintf('\n');
fprintf('========================================================================================================================\n');
fprintf(' STEP 4-B3 / DEPTH-CAMERA CLOSED REPLANNING LIFECYCLE AUDIT V2\n');
fprintf(' unknown=[] -> camera -> memory -> current-ref trigger -> corridor rebuild -> C6 replan -> atomic switch\n');
fprintf(' 3 backends retained | TRAJECTORY MODIFICATION ONLY | moving-obstacle interval-swept support | no runtime safety filter\n');
fprintf('========================================================================================================================\n');
fprintf('[MISSION] p0=%s  e=%s  speed=%.6g m/s  distance=%.6g m  tf=%.6g s\n', ...
    mat2str(CFG.p0.',5),mat2str(CFG.e_nom.',5),CFG.nominal_speed,CFG.mission_distance,CFG.mission_tf);
fprintf('[EXEC] dt=%.6g s | sensor=%.3f Hz | predictor=%.3f s / %d samples\n', ...
    CFG.dt_exec,CFG.sensor_hz,CFG.lookahead_s,CFG.predict_samples);
fprintf('[SCOPE] L=%.6g m massless inextensible rigid cable; vertical proxy is construction-only, final acceptance includes continuous rigid-cable p-cone geometry.\n',P.L_cable);
fprintf(['[ROBUST-SCOPE] geometryMargin=%.6g m | trackingBound=%s | obstacleModelBound=%s | ', ...
    'totalRadiusInflation=%.6g m\n'], ...
    P.d_margin_geometry,b3_bound_or_unspecified(CFG.tracking_position_error_bound_m), ...
    b3_bound_or_unspecified(CFG.obstacle_position_error_bound_m),P.d_margin);
fprintf('[LEAKAGE GUARD] ground truth is accessed only inside the internal depth-camera section of this one-file validator.\n');
fprintf(['[BASELINE] observed-only depth memory | PAYLOAD/UAV/CABLE continuous safety | ', ...
    'position+yaw C6 | recursive replanning | 3 backends.\n']);
fprintf(['[RUNTIME] predictor/dynamic conservative broad-phase enabled | ', ...
    'measurement-only dense audits disabled during mission.\n']);
fprintf('[V64 LOG-SCOPE] compact logging uses only explicit function inputs / certificate summary fields.\n');
b3_detail_fprintf('[HISTORY] detailed V27-V62 change log follows because compact_log=false.\n');
b3_detail_fprintf('[BASELINE V27] observed-only depth memory | PAYLOAD/UAV/CABLE safety | C6 | spatial rejoin | recursive replanning | 3 backends.\n');
b3_detail_fprintf('[BASELINE V28] V27 core retained; failing proof cells may be subdivided exactly for LOCAL safety certificates. Trajectory/C6 are unchanged.\n');
b3_detail_fprintf('[V29] all 3 backends retained. D13 local separator seeds include every exactly fixed restricted C6 control point; QPs use exact constraint generation with final full-row certification.\n');
b3_detail_fprintf('[V30 FIX] scalar fixedCtx + automatic clc at function entry.\n');
b3_detail_fprintf('[V31] scale-aware floating-point guide audit + exact duplicate-halfspace reduction before backend row expansion.\n');
b3_detail_fprintf('[V32 FIX] guide containment is diagnostic only; hard authority remains actual local curve-hull constraints + full original inequality certification.\n');
b3_detail_fprintf('[V33] BSPLINE full-row backward-error certificate + structural C6 knot certificate + physical-time internal residual audit.\n');
b3_detail_fprintf('[V34 FIX] backend-specific hard-certificate routing: D13 uses D13 certificate; BSPLINE uses fp-row + structural-C6 certificate.\n');
b3_detail_fprintf('[V35 FIX] event schemas are canonicalized before every append; replan-event schema includes V34 certificate fields.\n');
b3_detail_fprintf('[V36 AUDIT] no runtime optimization yet: detailed ONLINE/OFFLINE timing split + dense realized v/a diagnostic beside rigorous convex-hull bounds.\n');
b3_detail_fprintf('[V37] massless inextensible rigid-cable dynamics: q from payload acceleration, dynamic UAV/cable geometry in sensor+predictor, and hard continuous p-cone post-certificate.\n');
b3_detail_fprintf('[V38] adaptive exact de-Casteljau subdivision tightens rigid-cable p-cones only where needed; no user-chosen subdivision depth.\n');
b3_detail_fprintf('[V39] stable rigid-cable proof: acceleration Bezier is formed once in physical time and subdivided directly; guarantee ledger is explicit.\n');
b3_detail_fprintf('[V40] trajectory-only robust planning: certified tracking/model position-error bounds inflate all forbidden supports by exact Minkowski support addition.\n');
b3_detail_fprintf('[V41] dynamic separator rebuild: actual p-cone PAYLOAD/UAV/CABLE forbidden sets are separated directly from each local trajectory hull by SOCP.\n');
b3_detail_fprintf('[V42 FIX] dynamic geometry identities enforced: forbidden payload-center shift +L*p/+L*z and SOCP residual pS-pF using MATLAB ||A*x-b|| with b=c0.\n');
b3_detail_fprintf('[V43] Sreenath single-towing model consistency: cable direction is p in S^2, xQ=xL-Lp, pdot=omega x p, and optional Eq.(2.18)/(2.20) thrust reconstruction.\n');
b3_detail_fprintf('[V44 FIX] Sreenath regression guard: explicit pHang precheck symbol + source-level legacy/sign/schema guards before mission execution.\n');
b3_detail_fprintf('[V45 FIX] exact alpha=0 p-cap is reduced analytically (p=-e3); degenerate zero-interior SOC variables are removed rather than tolerance-relaxed.\n');
b3_detail_fprintf('[V46 FIX] full local-callgraph guard + restored rigid-cable/Sreenath dense diagnostics; undefined project helpers are rejected before mission execution.\n');
b3_detail_fprintf('[V47 FIX] SOCP is candidate-generator only; direct continuous support inequality is the hard separator authority. Solver exitflag cannot veto a valid support certificate.\n');
b3_detail_fprintf('[V48 AUDIT] unresolved infinitesimal leaves are classified by direct Sreenath point/segment geometry: true pointwise collision vs convexification/certificate uncertainty.\n');
b3_detail_fprintf('[V49 CERT] floating-point-limit leaves use a continuous Lipschitz clearance certificate derived from global Bezier velocity/jerk/resultant bounds; no sampled-safe shortcut.\n');
b3_detail_fprintf('[V50 FIX] Lipschitz continuous certificate is attempted immediately after support-certificate failure, before any recursive subdivision. Deep recursion is fallback only.\n');
b3_detail_fprintf('[V51 FIX] Lipschitz bounds are leaf-local: V/A/J Bezier polygons are formed once in physical time and restricted together through de Casteljau recursion.\n');
b3_detail_fprintf('[V52 C6 HARD AUDIT] position/yaw d0..d6 continuity is hard-certified with physical-time endpoint backward-error guards; B-spline retains structural C6 as an independent guarantee.\n');
b3_detail_fprintf('[V53 YAW] yaw faces horizontal travel direction atan2(v_y,v_x); degree-27 endpoint control points are constructed analytically from d0..d6 jets, and only the 14 interior control points are least-squares fitted.\n');
b3_detail_fprintf('[V54 RUNTIME AUDIT] no safety logic changed: every dynamic SOCP is logged by cell/span/depth/obstacle-patch/component, with exact prior-normal reuse counterfactual and per-call timing.\n');
b3_detail_fprintf('[V55 FASTPATH] exact previous normal for the same obstacle-patch/component is checked by the SAME direct support certificate before SOCP; if certified, SOCP is skipped with no safety-authority change.\n');
b3_detail_fprintf('[V56 CASCADE] precomputed axis normals + one analytic center-direction witness are checked by exact CURRENT support before cache/SOCP; the same cascade is used in corridor construction.\n');
b3_detail_fprintf('[V56 OFFLINE/ONLINE] fixed separator directions, yaw endpoint matrix, and repeated 0.5 de-Casteljau split maps are cached/precomputed; validation-only dense audits remain outside ONLINE timing.\n');
b3_detail_fprintf('[V57 TIMING LEDGER] no planning/safety logic changed: every replan is closed by a non-overlapping top-level timing ledger, detailed child timings, coverage, closure error, and unaccounted-time alarm.\n');
b3_detail_fprintf('[V58 DEFERRED DIAG] legacy dense exact ellipsoid-distance audit is excluded from ONLINE backend timing and executed after qPlan as MEASUREMENT ONLY; hard acceptance/C6/safety are unchanged.\n');
b3_detail_fprintf('[V59 REAL RUNTIME] measurement-only dense audits are queued during mission and executed after each backend mission; predictor/dynamic conservative sphere broad-phases skip only provably irrelevant patch checks.\n');
b3_detail_fprintf(['[V62 CLEAN STANDALONE] mission runtime uses only local state until final RUN assembly; ', ...
    'measurement-only post-mission audits are OFF by default; no source-file self-read is required.\n']);
fprintf('========================================================================================================================\n');

for ib=1:numel(backendNames)
    fprintf('\n########################################################################################################################\n');
    fprintf(' BACKEND %d/%d : %s\n',ib,numel(backendNames),backendNames{ib});
    fprintf('########################################################################################################################\n');
    RUNS_CELL{ib}=b3_run_backend(backendNames{ib},CFG,P);
end

RUNS=[RUNS_CELL{:}];
pass=[RUNS.pass];
initialInvisible=[RUNS.initial_sensor_clear];
allDetected=[RUNS.n_new_detections];
allReplans=[RUNS.n_successful_replans];
recursive=[RUNS.n_replans_while_active];

fprintf('\n========================================================================================================================\n');
fprintf(' STEP 4-B3 COMPACT SUMMARY\n');
fprintf('========================================================================================================================\n');
for ib=1:numel(RUNS)
    R=RUNS(ib);
    fprintf('%-12s pass=%d | initialClear=%d | detect=%d | trigger=%d | replan=%d | recursive=%d | fail=%s\n', ...
        R.backend,R.pass,R.initial_sensor_clear,R.n_new_detections,R.n_triggers,R.n_successful_replans, ...
        R.n_replans_while_active,R.failure_reason);
    fprintf('             sensor med/p95/max = %.6f / %.6f / %.6f ms\n', ...
        R.sensor_runtime_median_ms,R.sensor_runtime_p95_ms,R.sensor_runtime_max_ms);
    fprintf('             plan   med/p95/max = %.6f / %.6f / %.6f ms\n', ...
        R.plan_runtime_median_ms,R.plan_runtime_p95_ms,R.plan_runtime_max_ms);
    fprintf('             min hard slack     = %+.12e | max physical splice residual d0..d6 = %s\n', ...
        R.min_hard_slack,mat2str(R.max_physical_splice_residual_d0_d6,6));
end
fprintf('------------------------------------------------------------------------------------------------------------------------\n');
fprintf(['[INTERPRETATION] hard acceptance = observed-obstacle PAYLOAD/UAV/CABLE continuous safety + ', ...
    'position/yaw C6; tracking/model bounds remain UNSPECIFIED unless supplied.\n']);
b3_detail_fprintf('[B3 INTERPRETATION]\n');
b3_detail_fprintf(' - The planner begins with zero obstacle knowledge and cannot query the environment directly.\n');
b3_detail_fprintf(' - Detection alone does not switch the reference; future collision on the CURRENT reference is required.\n');
b3_detail_fprintf(' - Sensing remains active during avoidance; NEW/model-changed threats can cause a recursive C6 replan.\n');
b3_detail_fprintf(' - Same-model known obstacles do not retrigger while their currently executing trajectory is already certified.\n');
b3_detail_fprintf(' - yaw = cached degree-27 C6 fit to the executing horizontal tangent (progress direction).\n');
b3_detail_fprintf(' - Every accepted switch preserves the measured current physical-time d0..d6 start jet by construction.\n');
b3_detail_fprintf(' - Hard safety authority is C6 plus component-wise PAYLOAD/UAV/CABLE support; V37 adds a continuous acceleration-induced rigid-cable p-cone post-certificate before switch.\n');
b3_detail_fprintf(' - Guide-cell containment is diagnostic only; actual restricted curve-hull constraints and full-row direct checks are the safety authority.\n');
b3_detail_fprintf(' - Velocity/acceleration and d1..d6 bounds are advisory diagnostics only; hard acceptance remains observed-obstacle safety + C6.\n');
b3_detail_fprintf(' - V42 enforces exact coordinate identities: p_Q=p_L-Lq implies forbidden payload-center shift +Lp, and SOCP distance uses MATLAB ||A*x-b|| with b=c0, giving pS-pF=D*x-c0.\n');
b3_detail_fprintf(' - Analytic PAYLOAD/UAV/CABLE identity tests must pass before mission execution, so adaptive subdivision cannot hide sign/model regressions.\n');
b3_detail_fprintf(' - V40/V41 remain trajectory-modification-only: supplied tracking/model error bounds are absorbed by exact Minkowski support inflation before trajectory acceptance.\n');
b3_detail_fprintf(' - Unspecified error bounds are never silently inferred from RMSE or nominal margins and remain explicitly NOT guaranteed.\n');
b3_detail_fprintf('========================================================================================================================\n');

OUT=struct();
OUT.guarantee_ledger=GUARANTEE;
OUT.mode='STEP4B3_DEPTH_CAMERA_REPLAN_LIFECYCLE_AUDIT_V64_SCOPE_SAFE_COMPACT_LOG';
OUT.config=CFG;
OUT.physical_proxy=P;
OUT.backends=RUNS;
OUT.aggregate=struct();
OUT.aggregate.backend_names=backendNames;
OUT.aggregate.pass=pass;
OUT.aggregate.all_backends_pass=all(pass);
OUT.aggregate.initial_sensor_clear=initialInvisible;
OUT.aggregate.new_detections=allDetected;
OUT.aggregate.successful_replans=allReplans;
OUT.aggregate.replans_while_active=recursive;
OUT.aggregate.environment_access_policy='B3_NO_DIRECT_ENVIRONMENT_CALL_SENSOR_MODULE_ONLY';
OUT.aggregate.sensor_module='INTERNAL_DEPTH_CAMERA_V3_ONEFILE';
OUT.aggregate.controller_closed_loop_simulated=false;
OUT.aggregate.cbf_used=false;
OUT.aggregate.mpc_used=false;
OUT.aggregate.lidar_used=false;
OUT.aggregate.safety_scope='OBSERVED_PATCH_ONLY_COMPONENT_WISE_CLOSED_SAFE_SET_SUPPORT_CERTIFICATE_VERTICAL_CABLE_PROXY';
OUT.aggregate.incremental_map_policy='KEEP_NONREDUNDANT_OBSERVED_ELLIPSOID_PATCHES_ONLY';
OUT.aggregate.unobserved_space_claim='NONE';
OUT.aggregate.system_union_handling='PAYLOAD_UAV_CABLE_CERTIFIED_SEPARATELY_NO_GLOBAL_CONVEX_HULL';
OUT.aggregate.safety_theorem='FOR_EACH_COMPONENT: CONTROL_POINT_HALFSPACE_CONTAINMENT + EXACT_FORBIDDEN_SET_SUPPORT_SEPARATION => CONTINUOUS-TIME COLLISION AVOIDANCE';

assignin('base','c6_step4b3_depth_camera_replan_lifecycle_v64_scope_safe_compact_log_result',OUT);
end



function varargout=b3_detail_fprintf(varargin)
% Display-only gate. This function never changes planner/safety computation.
% Internal mode is set once from CFG.compact_log:
%   compact_log=true  -> detailed messages hidden
%   compact_log=false -> detailed messages printed
persistent DETAIL_ENABLED
if nargin>=1 && ischar(varargin{1}) && strcmp(varargin{1},'__SET__')
    if nargin<2
        error('STEP4B3:DetailLogger','Detail logger setter requires a logical value.');
    end
    DETAIL_ENABLED=logical(varargin{2});
    if nargout>0, varargout{1}=[]; end
    return;
end
if isempty(DETAIL_ENABLED)
    DETAIL_ENABLED=true;
end
if DETAIL_ENABLED
    [varargout{1:nargout}]=fprintf(varargin{:});
else
    for k=1:nargout
        varargout{k}=0;
    end
end
end


function CFG=b3_default_config(P)
CFG=struct();
CFG.p0=[0;0;0];
CFG.nominal_direction=[1;0;0];
CFG.nominal_speed=1.0;
% V19 physical reference limits. NaN means 'not supplied': audit only, no claim.
CFG.max_ref_speed=NaN;
CFG.max_ref_acc=NaN;
% V40 trajectory-only robust planning.
% These are certified POSITION-error upper bounds, not RMSE values.
% NaN means no formal bound was supplied, therefore that robustness layer
% must not be claimed as guaranteed.
CFG.tracking_position_error_bound_m=NaN;
CFG.obstacle_position_error_bound_m=NaN;
% Sreenath single-towing physical parameters.
% Keep NaN until the actual vehicle/payload values used by the experiment are
% supplied. Geometry/p-direction reconstruction does not require the masses.
CFG.mQ_kg=NaN;
CFG.mL_kg=NaN;
CFG.enforce_kinematic_limits=true;
CFG.mission_distance=45.0;
CFG.dt_exec=0.025;
CFG.sensor_hz=30.0;
CFG.lookahead_s=5.0;
CFG.predict_samples=11;
CFG.max_duration_iterations=12;
CFG.max_replans=20;
CFG.separator_direction_count=72;
% V28: adaptive proof-only subdivision. 0 reproduces V27 corridor cells;
% 2 permits at most four exact local proof pieces only when an unsplit
% cell cannot be separated. The trajectory itself is not subdivided/changed.
CFG.corridor_max_subdivision_depth=2;
CFG.cell_direction_count=74;
CFG.min_relative_progress_speed=1e-9;
CFG.duration_convergence_tol=1e-10;
CFG.yaw_mode='C6_DEG27_TANGENT_FIT_FAST';
CFG.yaw_fit_sample_count=65;
% During an already-certified avoidance, do not rebuild the same plan every
% sensor frame. Replanning is reopened only by NEW obstacle knowledge or a
% material change of an observed obstacle model.
CFG.replan_on_model_change=true;
CFG.model_center_change_m=0.05;
CFG.model_velocity_change_mps=0.05;
CFG.model_rotation_change_fro=1e-3;
CFG.model_radii_change_m=1e-3;
CFG.verbose_sensor_frames=false;
CFG.compact_log=true;
CFG.defer_measurement_diagnostics=true;
CFG.run_post_mission_measurement_audits=false;
CFG.predictor_sphere_broadphase=true;
CFG.dynamic_sphere_broadphase=true;
CFG.e3=[0;0;1];
CFG.cable_length_m=P.L_cable;
CFG= b3_finalize_config(CFG);
end



function P=b3_apply_robust_margins(P,CFG)
% V40 trajectory-only robustification.
%
% Let the executed payload-center tracking error satisfy
%     ||e_track(t)||_2 <= eps_track
% and let the obstacle-center/model position error satisfy
%     ||e_obs(t)||_2 <= eps_obs.
%
% For any unit halfspace normal n,
%     h_{B(0,eps)}(n) = eps ||n||_2.
% Therefore Minkowski inflation of the forbidden set by both uncertainty
% balls is EXACTLY equivalent to adding
%     (eps_track + eps_obs)||n||
% to every component support function.
%
% The whole validator already adds P.d_margin to PAYLOAD/UAV/CABLE radii.
% We preserve the original engineering/geometric margin separately, and only
% add uncertainty terms whose formal upper bounds were supplied.

P.d_margin_geometry=P.d_margin;

if isnan(CFG.tracking_position_error_bound_m)
    epsTrack=0;
    P.tracking_error_formally_bounded=false;
else
    epsTrack=CFG.tracking_position_error_bound_m;
    P.tracking_error_formally_bounded=true;
end

if isnan(CFG.obstacle_position_error_bound_m)
    epsObs=0;
    P.obstacle_error_formally_bounded=false;
else
    epsObs=CFG.obstacle_position_error_bound_m;
    P.obstacle_error_formally_bounded=true;
end

P.tracking_position_error_bound_m=epsTrack;
P.obstacle_position_error_bound_m=epsObs;
P.robust_uncertainty_inflation_m=epsTrack+epsObs;
P.d_margin=P.d_margin_geometry+P.robust_uncertainty_inflation_m;

fprintf(['[ROBUST-MARGIN] geometry=%.9g m | tracking=%s | obstacleModel=%s | ', ...
    'effectiveSupportInflation=%.9g m\n'], ...
    P.d_margin_geometry, ...
    b3_bound_or_unspecified(CFG.tracking_position_error_bound_m), ...
    b3_bound_or_unspecified(CFG.obstacle_position_error_bound_m), ...
    P.d_margin);
end

function s=b3_bound_or_unspecified(x)
if isnan(x)
    s='UNSPECIFIED';
else
    s=sprintf('%.9g m',x);
end
end


function CFG=b3_merge_config(CFG,U)
if ~isstruct(U)
    error('STEP4B3:Config','userCfg must be a struct.');
end
f=fieldnames(U);
for i=1:numel(f)
    if ~isfield(CFG,f{i})
        error('STEP4B3:Config','Unknown configuration field: %s',f{i});
    end
    CFG.(f{i})=U.(f{i});
end
CFG=b3_finalize_config(CFG);
end


function CFG=b3_finalize_config(CFG)
CFG.p0=CFG.p0(:);
e=CFG.nominal_direction(:);
if norm(e)<=0
    error('STEP4B3:Config','nominal_direction must be nonzero.');
end
CFG.e_nom=e/norm(e);
CFG.v_nom=CFG.nominal_speed*CFG.e_nom;
CFG.mission_tf=CFG.mission_distance/CFG.nominal_speed;
CFG.nominal_yaw=atan2(CFG.e_nom(2),CFG.e_nom(1));
end


function b3_validate_config(CFG,P)
for fn={'defer_measurement_diagnostics','run_post_mission_measurement_audits', ...
        'predictor_sphere_broadphase','dynamic_sphere_broadphase'}
    v=CFG.(fn{1});
    if ~(islogical(v) && isscalar(v))
        error('STEP4B3:Config','%s must be scalar logical.',fn{1});
    end
end
validateattributes(CFG.p0,{'numeric'},{'real','finite','size',[3 1]});
validateattributes(CFG.e_nom,{'numeric'},{'real','finite','size',[3 1]});
validateattributes(CFG.nominal_speed,{'numeric'},{'real','finite','scalar','positive'});
if ~(isnumeric(CFG.max_ref_speed) && isscalar(CFG.max_ref_speed) && (isnan(CFG.max_ref_speed) || (isfinite(CFG.max_ref_speed) && CFG.max_ref_speed>0)))
    error('STEP4B3:Config','max_ref_speed must be positive finite or NaN.');
end
if ~(isnumeric(CFG.max_ref_acc) && isscalar(CFG.max_ref_acc) && (isnan(CFG.max_ref_acc) || (isfinite(CFG.max_ref_acc) && CFG.max_ref_acc>0)))
    error('STEP4B3:Config','max_ref_acc must be positive finite or NaN.');
end
if ~(islogical(CFG.enforce_kinematic_limits) && isscalar(CFG.enforce_kinematic_limits))
    error('STEP4B3:Config','enforce_kinematic_limits must be scalar logical.');
end
if ~(isnumeric(CFG.tracking_position_error_bound_m) && isscalar(CFG.tracking_position_error_bound_m) && ...
        (isnan(CFG.tracking_position_error_bound_m) || ...
        (isfinite(CFG.tracking_position_error_bound_m) && CFG.tracking_position_error_bound_m>=0)))
    error('STEP4B3:Config', ...
        'tracking_position_error_bound_m must be nonnegative finite or NaN.');
end
if ~(isnumeric(CFG.obstacle_position_error_bound_m) && isscalar(CFG.obstacle_position_error_bound_m) && ...
        (isnan(CFG.obstacle_position_error_bound_m) || ...
        (isfinite(CFG.obstacle_position_error_bound_m) && CFG.obstacle_position_error_bound_m>=0)))
    error('STEP4B3:Config', ...
        'obstacle_position_error_bound_m must be nonnegative finite or NaN.');
end
for fn={'mQ_kg','mL_kg'}
    v=CFG.(fn{1});
    if ~(isnumeric(v) && isscalar(v) && (isnan(v) || (isfinite(v) && v>0)))
        error('STEP4B3:Config','%s must be positive finite or NaN.',fn{1});
    end
end
if xor(isnan(CFG.mQ_kg),isnan(CFG.mL_kg))
    error('STEP4B3:Config','mQ_kg and mL_kg must either both be supplied or both be NaN.');
end
validateattributes(CFG.mission_distance,{'numeric'},{'real','finite','scalar','positive'});
validateattributes(CFG.dt_exec,{'numeric'},{'real','finite','scalar','positive'});
validateattributes(CFG.sensor_hz,{'numeric'},{'real','finite','scalar','positive'});
validateattributes(CFG.lookahead_s,{'numeric'},{'real','finite','scalar','positive'});
validateattributes(CFG.predict_samples,{'numeric'},{'real','finite','scalar','integer','>=',2});
validateattributes(CFG.max_replans,{'numeric'},{'real','finite','scalar','integer','positive'});
validateattributes(CFG.separator_direction_count,{'numeric'},{'real','finite','scalar','integer','>=',8});
validateattributes(CFG.corridor_max_subdivision_depth,{'numeric'},{'real','finite','scalar','integer','>=',0,'<=',4});
validateattributes(CFG.model_center_change_m,{'numeric'},{'real','finite','scalar','nonnegative'});
validateattributes(CFG.model_velocity_change_mps,{'numeric'},{'real','finite','scalar','nonnegative'});
validateattributes(CFG.model_rotation_change_fro,{'numeric'},{'real','finite','scalar','nonnegative'});
validateattributes(CFG.model_radii_change_m,{'numeric'},{'real','finite','scalar','nonnegative'});
assert(P.L_cable>0,'STEP4B3:Config','Cable length must be positive.');
end


function b3_require_dependencies()
if isempty(ver('optim')) || exist('quadprog','file')~=2
    error('STEP4B3:OptimizationToolbox','Optimization Toolbox / quadprog is required.');
end
if exist('coneprog','file')~=2 || exist('secondordercone','file')~=2
    error('STEP4B3:ConeprogRequired', ...
        'V10 theory-based corridor construction requires coneprog and secondordercone (Optimization Toolbox).');
end
end


function RUN=b3_run_backend(backend,CFG,P)
% One complete mission replay for one trajectory representation.
% Each backend receives exactly the same sensor stream semantics, but a fresh
% sensor/memory run is used so there is no cross-backend state leakage.

mem=b3_empty_memory();
active=b3_empty_active();
generation=0;

% V62: no RUN field is touched until final RUN assembly.
deferredValidationJobs=b3_empty_deferred_validation_job_array();
nextSensor=0.0;
terminated=false;
failureReason="NONE";
initialSensorChecked=false;
initialSensorClear=false;

N=floor(CFG.mission_tf/CFG.dt_exec)+1;
tvec=(0:(N-1))*CFG.dt_exec;
if tvec(end)<CFG.mission_tf-1e-12
    tvec=[tvec,CFG.mission_tf]; %#ok<AGROW>
end
N=numel(tvec);
jetLog=zeros(3,7,N);
refLog=zeros(1,N);
knownCount=zeros(1,N);
activeFlag=false(1,N);

sensorEvents=repmat(b3_empty_sensor_event(),1,0);
replanEvents=repmat(b3_empty_replan_event(),1,0);
sensorRuntime=[];
planRuntime=[];
minHard=Inf;
maxSplice=zeros(1,7);
nNew=0;
nTrigger=0;
nReplan=0;
nRecursive=0;
nominalTimeShift=0;

predAgg=struct('calls',0,'tests',0,'rejects',0,'exactPoint',0,'exactSegment',0, ...
    'runtime_ms',0,'max_runtime_ms',0);

fprintf('[START] memory=[] | active reference=NOMINAL | environment direct access=0\n');
fprintf('[RUNTIME-MODE] postMissionMeasurementAudits=%d | denseDiagnosticsDuringMission=%d\n', ...
    CFG.run_post_mission_measurement_audits,~CFG.defer_measurement_diagnostics);

for it=1:N
    t=tvec(it);

    if active.valid && t>active.t_end+1e-12
        if isfield(active,'post_nominal_time_shift') && isfinite(active.post_nominal_time_shift)
            nominalTimeShift=active.post_nominal_time_shift;
        end
        active=b3_empty_active();
        fprintf('  COMPLETE t=%.6f -> TIME-SHIFTED NOMINAL | shift=%.9f s\n',t,nominalTimeShift);
    end

    D=b3_exec_jet(t,active,CFG,nominalTimeShift);
    jetLog(:,:,it)=D;
    currentRefId=0;
    if active.valid, currentRefId=active.reference_id; end
    refLog(it)=currentRefId;
    knownCount(it)=numel(mem);
    activeFlag(it)=active.valid;

    if terminated
        continue;
    end

    % ---------------------------------------------------------------
    % Sensor scheduling. The only environment access occurs INSIDE the
    % external sensor function called below.
    % ---------------------------------------------------------------
    if t+1e-12 < nextSensor
        continue;
    end
    while nextSensor <= t+1e-12
        nextSensor=nextSensor+1/CFG.sensor_hz;
    end

    pL=D(:,1);
    cableNow=b3_sreenath_cable_state_from_jet(D,P,CFG);
    if ~cableNow.valid
        error('STEP4B3:RigidCableStateInvalid', ...
            'Rigid-cable state invalid at sensor time t=%.9f.',t);
    end
    pWC=cableNow.xQ;
    yaw=b3_camera_yaw(t,D,active,CFG);
    R_WC=b3_camera_rotation_from_yaw(yaw);

    idsBefore=b3_memory_ids(mem);
    qSens=tic;
    SOUT=b3_depth_camera_fast(t,pWC,R_WC,idsBefore);
    for iz=1:numel(SOUT.visible_obstacles)
        if ~isfield(SOUT.visible_obstacles(iz),'model_source') || ...
                ~strcmp(SOUT.visible_obstacles(iz).model_source,'DEPTH_HIT_CELLS_ONLY')
            error('STEP4B3:ObservationLeakage','Planner-visible obstacle was not built from depth-hit cells only.');
        end
    end
    sensMs=1e3*toc(qSens);
    sensorRuntime(end+1)=sensMs; %#ok<AGROW>

    % IMPORTANT: update memory only from currently VISIBLE obstacles.
    % Do not use SOUT.known_obstacles because that field is hydrated from
    % the simulator environment at the current time.
    qMem=tic;
    [mem,newIds,changedIds]=b3_update_memory_from_visible(mem,SOUT.visible_obstacles,t,CFG);
    memMs=1e3*toc(qMem);
    idsAfter=b3_memory_ids(mem);
    nNew=nNew+numel(newIds);
    nPatch=numel(mem);
    if ~isempty(newIds) || ~isempty(changedIds)
        b3_detail_fprintf('    [INCREMENTAL-MAP] parentIDs=%s patches=%d newIDs=%s expandedIDs=%s\n', ...
            mat2str(idsAfter),nPatch,mat2str(newIds),mat2str(changedIds));
    end

    if ~initialSensorChecked
        initialSensorChecked=true;
        initialSensorClear=isempty(SOUT.visible_ids);
        fprintf('  SENSOR t=%.6f initial visible=%s -> initialOutsideCamera=%d\n', ...
            t,mat2str(SOUT.visible_ids),initialSensorClear);
        if ~initialSensorClear
            terminated=true;
            failureReason="INITIAL_OBSTACLE_ALREADY_VISIBLE";
            fprintf('  VALIDATION STOP: obstacle is visible in the first frame; B3 requires the scenario to begin outside the new camera.\n');
        end
    end

    if ~isempty(newIds)
        fprintf('  DETECT t=%.6f visible=%s new=%s known %s -> %s | sensor=%.4f ms\n', ...
            t,mat2str(SOUT.visible_ids),mat2str(newIds),mat2str(idsBefore),mat2str(idsAfter),sensMs);
        if isfield(SOUT,'timing')
            b3_detail_fprintf('    [PROFILE-SENSOR] env=%.6f | ROI=%.6f | rayprep=%.6f | raycast=%.6f | pack=%.6f ms | rays=%d/%d\n', ...
                SOUT.timing.environment_ms,SOUT.timing.roi_ms,SOUT.timing.rayprep_ms, ...
                SOUT.timing.raycast_ms,SOUT.timing.pack_ms,SOUT.n_sampled_rays,SOUT.n_fullframe_sampled_rays);
        end
        if SOUT.timing.pack_ms>50
            b3_detail_fprintf('    [PROFILE-NOTE] sensor pack outlier >50 ms: treat as cold/JIT-allocation candidate; compare subsequent backend runs before production conclusion.\n');
        end
        for im=1:numel(SOUT.visible_obstacles)
            zz=SOUT.visible_obstacles(im);
            b3_detail_fprintf('    [OBSERVED-MODEL] id=%d source=%s points=%d center=%s radii=%s\n', ...
                zz.id,zz.model_source,zz.n_observed_points, ...
                mat2str(zz.p_center.',6),mat2str(zz.ellipsoid_radii.',6));
        end
    elseif CFG.verbose_sensor_frames
        fprintf('  SENSOR t=%.6f visible=%s known=%s | sensor=%.4f ms\n', ...
            t,mat2str(SOUT.visible_ids),mat2str(idsAfter),sensMs);
    end

    SE=b3_empty_sensor_event();
    SE.t=t;
    SE.visible_ids=SOUT.visible_ids;
    SE.new_ids=newIds;
    SE.known_before=idsBefore;
    SE.known_after=idsAfter;
    SE.changed_ids=changedIds;
    SE.reference_id=currentRefId;
    SE.active_avoidance=active.valid;
    SE.sensor_runtime_ms=sensMs;
    if isfield(SOUT,'timing')
        SE.sensor_environment_ms=SOUT.timing.environment_ms;
        SE.sensor_roi_ms=SOUT.timing.roi_ms;
        SE.sensor_rayprep_ms=SOUT.timing.rayprep_ms;
        SE.sensor_raycast_ms=SOUT.timing.raycast_ms;
        SE.sensor_pack_ms=SOUT.timing.pack_ms;
    end
    SE.memory_update_ms=memMs;
    SE.cable_swing_deg=cableNow.swing_deg;
    SE.cable_payload_tension_per_mass=cableNow.payload_tension_per_mass;

    if terminated
        SE=b3_enforce_struct_schema(SE,b3_empty_sensor_event(),'sensor event');
    sensorEvents(end+1)=SE; %#ok<AGROW>
        break;
    end

    if isempty(mem)
        SE=b3_enforce_struct_schema(SE,b3_empty_sensor_event(),'sensor event');
    sensorEvents(end+1)=SE; %#ok<AGROW>
        continue;
    end

    % Anti-thrashing rule:
    % while an active trajectory remains certified against the SAME obstacle
    % models, do not rerun predictor/planner at every 30-Hz frame. A new
    % obstacle or a material model update reopens the trigger immediately.
    if active.valid && isempty(newIds) && isempty(changedIds)
        SE.predictor_skipped_certified_active=true;
        SE=b3_enforce_struct_schema(SE,b3_empty_sensor_event(),'sensor event');
    sensorEvents(end+1)=SE; %#ok<AGROW>
        continue;
    end

    % Predict the CURRENT executing reference using only remembered obstacles.
    qPred=tic;
    risk=b3_predict_current_reference(t,active,mem,CFG,P,nominalTimeShift);
    predMs=1e3*toc(qPred);
    SE.predictor_runtime_ms=predMs;
    predAgg.calls=predAgg.calls+1;
    predAgg.runtime_ms=predAgg.runtime_ms+predMs;
    predAgg.max_runtime_ms=max(predAgg.max_runtime_ms,predMs);
    if isfield(risk,'broadphase_tests')
        predAgg.tests=predAgg.tests+risk.broadphase_tests;
        predAgg.rejects=predAgg.rejects+risk.broadphase_rejects;
        predAgg.exactPoint=predAgg.exactPoint+risk.exact_point_tests;
        predAgg.exactSegment=predAgg.exactSegment+risk.exact_segment_tests;
    end
    b3_detail_fprintf(['  [PREDICTOR-BROADPHASE] tests=%d rejects=%d exactPoint=%d ', ...
        'exactSegment=%d rejectRate=%.3f runtime=%.6f ms\n'], ...
        risk.broadphase_tests,risk.broadphase_rejects, ...
        risk.exact_point_tests,risk.exact_segment_tests, ...
        risk.broadphase_rejects/max(1,risk.broadphase_tests),predMs);
    SE.risk_found=risk.found;
    SE.risk_ids=risk.ids;
    SE.risk_ttc_s=risk.TTC;
    SE=b3_enforce_struct_schema(SE,b3_empty_sensor_event(),'sensor event');
    sensorEvents(end+1)=SE; %#ok<AGROW>

    if ~risk.found
        continue;
    end

    nTrigger=nTrigger+1;
    wasActive=active.valid;
    fprintf('  TRIGGER #%d t=%.6f TTC=%.6f riskIDs=%s sourceRef=%d active=%d\n', ...
        nTrigger,t,risk.TTC,mat2str(risk.ids),currentRefId,wasActive);
    fprintf(['    [PREDICTOR] runtime=%.3f ms | tests=%d reject=%.1f%% | ', ...
        'exactPoint=%d exactSegment=%d\n'], ...
        predMs,risk.broadphase_tests, ...
        100*risk.broadphase_rejects/max(1,risk.broadphase_tests), ...
        risk.exact_point_tests,risk.exact_segment_tests);
    fprintf('    [TOWING-RISK] firstBody=%s | payloadClr=%+.6e | uavClr=%+.6e | cableClr=%+.6e | riskPatches=%s\n', ...
        risk.body,risk.min_payload_clearance,risk.min_uav_clearance,risk.min_cable_clearance, ...
        mat2str(risk.risk_patch_ids));

    if nReplan>=CFG.max_replans
        terminated=true;
        failureReason="MAX_REPLAN_COUNT_REACHED";
        fprintf('  VALIDATION STOP: max_replans reached before threat was cleared.\n');
        break;
    end

    Dstart=b3_exec_jet(t,active,CFG,nominalTimeShift);
    Ystart=b3_exec_yaw_jet(t,active,CFG);
    qPlan=tic;
    [cand,PE]=b3_plan_once(backend,t,Dstart,Ystart,mem,risk,CFG,P,generation+1);
    pms=1e3*toc(qPlan);
    PE.plan_runtime_ms=pms;
    PE.profile.external_plan_wall_ms=pms;
    if isfield(PE.profile,'internal_plan_wall_ms') && isfinite(PE.profile.internal_plan_wall_ms)
        PE.profile.internal_external_closure_ms=pms-PE.profile.internal_plan_wall_ms;
    else
        PE.profile.internal_external_closure_ms=NaN;
    end
    if isfield(PE.profile,'measurement_only_ms') && isfinite(PE.profile.measurement_only_ms)
        PE.profile.online_plan_estimate_ms=max(0,pms-PE.profile.measurement_only_ms);
    elseif isfield(PE.profile,'kinematic_dense_diag_ms') && isfinite(PE.profile.kinematic_dense_diag_ms)
        PE.profile.online_plan_estimate_ms=max(0,pms-PE.profile.kinematic_dense_diag_ms);
    else
        PE.profile.online_plan_estimate_ms=pms;
    end
    if isfield(PE.profile,'ledger_ready') && PE.profile.ledger_ready
        b3_detail_fprintf(['    [TIME-LEDGER-EXTERNAL] external=%.6f internal=%.6f ', ...
            'external-internal=%+.6f ms\n'], ...
            pms,PE.profile.internal_plan_wall_ms,PE.profile.internal_external_closure_ms);
    end

    if cand.valid && CFG.defer_measurement_diagnostics && ...
            CFG.run_post_mission_measurement_audits
        JOB=b3_make_deferred_validation_job(cand,mem,t,PE);
        deferredValidationJobs(end+1,1)=JOB; %#ok<AGROW>
        PE.deferred_dense_exact_ellipsoid_diag_ms=NaN;
        PE.deferred_dense_exact_ellipsoid_samples=0;
        PE.profile.deferred_backend_dense_diag_ms=NaN;
        fprintf(['    [MEASUREMENT-QUEUED] ref=%d backend=%s patches=%d ', ...
            'validation deferred until backend mission end\n'], ...
            cand.reference_id,cand.backend,JOB.n_patches);
    elseif cand.valid
        qDeferred=tic;
        [denseSlackDeferred,denseNSamples]= ...
            b3_deferred_backend_dense_ellipsoid_diag(cand,mem,P);
        deferredMs=1e3*toc(qDeferred);
        PE.dense_exact_ellipsoid_slack=denseSlackDeferred;
        PE.deferred_dense_exact_ellipsoid_diag_ms=deferredMs;
        PE.deferred_dense_exact_ellipsoid_samples=denseNSamples;
        PE.profile.deferred_backend_dense_diag_ms=deferredMs;
    else
        PE.deferred_dense_exact_ellipsoid_diag_ms=NaN;
        PE.deferred_dense_exact_ellipsoid_samples=0;
        PE.profile.deferred_backend_dense_diag_ms=NaN;
    end

    planRuntime(end+1)=pms; %#ok<AGROW>
    PE=b3_enforce_struct_schema(PE,b3_empty_replan_event(),'replan event');
    replanEvents(end+1)=PE; %#ok<AGROW>
    if isfinite(PE.min_original_hard_slack)
        minHard=min(minHard,PE.min_original_hard_slack);
    end

    if ~cand.valid
        terminated=true;
        failureReason=string(PE.failure_reason);
        fprintf('  REPLAN FAIL t=%.6f backend=%s reason=%s | old reference NOT replaced\n', ...
            t,backend,PE.failure_reason);
        b3_print_failure_event_detail(PE);
        break;
    end

    active=cand;
    generation=cand.reference_id;
    nReplan=nReplan+1;
    if wasActive, nRecursive=nRecursive+1; end
    maxSplice=max(maxSplice,PE.start_physical_splice_residual_d0_d6);

    fprintf('  SWITCH ref=%d t=%.6f -> %.6f T=%.6f known=%s risk=%s hardSlack=%+.6e plan=%.4f ms\n', ...
        active.reference_id,t,active.t_end,active.T,mat2str(idsAfter),mat2str(risk.ids), ...
        PE.min_original_hard_slack,pms);
    b3_detail_fprintf('         position physical splice residual d0..d6 = %s\n', ...
        mat2str(PE.start_physical_splice_residual_d0_d6,6));
    b3_detail_fprintf('         yaw      physical splice residual d0..d6 = %s | yawFitMax=%.6e rad\n', ...
        mat2str(PE.yaw_start_splice_residual_d0_d6,6),PE.yaw_fit_max_abs_rad);
end

if isempty(sensorRuntime)
    sr=[NaN NaN NaN];
else
    sr=[median(sensorRuntime),b3_percentile(sensorRuntime,95),max(sensorRuntime)];
end
if isempty(planRuntime)
    pr=[NaN NaN NaN];
else
    pr=[median(planRuntime),b3_percentile(planRuntime,95),max(planRuntime)];
end
if isinf(minHard), minHard=NaN; end

% A B3 run is considered exercised only if the first frame is clear, at least
% one obstacle is later learned, at least one future threat causes a successful
% replan, and no safety/replan termination occurred.
pass=initialSensorClear && nNew>0 && nTrigger>0 && nReplan>0 && ~terminated;
if ~pass && failureReason=="NONE"
    if nNew==0
        failureReason="NO_OBSTACLE_DETECTED_IN_MISSION";
    elseif nTrigger==0
        failureReason="DETECTION_OCCURRED_BUT_NO_FUTURE_COLLISION_TRIGGER";
    elseif nReplan==0
        failureReason="TRIGGER_OCCURRED_BUT_NO_CERTIFIED_REPLAN";
    else
        failureReason="B3_INCOMPLETE";
    end
end

fprintf('[END] backend=%s pass=%d detect=%d trigger=%d replans=%d recursive=%d failure=%s\n', ...
    backend,pass,nNew,nTrigger,nReplan,nRecursive,failureReason);
if predAgg.calls>0
    fprintf(['[PREDICTOR-SUMMARY] calls=%d tests=%d reject=%.1f%% exactPoint=%d ', ...
        'exactSegment=%d total=%.1f ms avg=%.2f max=%.2f ms\n'], ...
        predAgg.calls,predAgg.tests,100*predAgg.rejects/max(1,predAgg.tests), ...
        predAgg.exactPoint,predAgg.exactSegment,predAgg.runtime_ms, ...
        predAgg.runtime_ms/predAgg.calls,predAgg.max_runtime_ms);
end

RUN=struct();
RUN.deferred_validation_jobs=deferredValidationJobs;
RUN.backend=backend;
RUN.pass=pass;
RUN.failure_reason=char(failureReason);
RUN.initial_sensor_clear=initialSensorClear;
RUN.n_new_detections=nNew;
RUN.n_triggers=nTrigger;
RUN.n_successful_replans=nReplan;
RUN.n_replans_while_active=nRecursive;
RUN.sensor_events=sensorEvents;
RUN.replan_events=replanEvents;
RUN.execution=struct('t',tvec,'reference_jet_d0_d6',jetLog, ...
    'reference_id',refLog,'known_count',knownCount,'active_avoidance',activeFlag);
RUN.sensor_runtime_ms=sensorRuntime;
RUN.sensor_runtime_median_ms=sr(1);
RUN.sensor_runtime_p95_ms=sr(2);
RUN.sensor_runtime_max_ms=sr(3);
RUN.sensor_stage_runtime_ms=struct( ...
    'environment',[sensorEvents.sensor_environment_ms], ...
    'roi',[sensorEvents.sensor_roi_ms], ...
    'rayprep',[sensorEvents.sensor_rayprep_ms], ...
    'raycast',[sensorEvents.sensor_raycast_ms], ...
    'pack',[sensorEvents.sensor_pack_ms], ...
    'memory_update',[sensorEvents.memory_update_ms], ...
    'predictor',[sensorEvents.predictor_runtime_ms]);
RUN.plan_runtime_ms=planRuntime;
RUN.plan_runtime_median_ms=pr(1);
RUN.plan_runtime_p95_ms=pr(2);
RUN.plan_runtime_max_ms=pr(3);
RUN.min_hard_slack=minHard;
RUN.max_physical_splice_residual_d0_d6=maxSplice;
RUN.final_memory=mem;

% V36 timing taxonomy
RUN.timing_taxonomy=b3_v36_timing_taxonomy(sensorEvents,replanEvents);
TT=RUN.timing_taxonomy;
fprintf(['[TIMING-TAXONOMY] ONLINE sensor med=%.6f | memory med=%.6f | ', ...
    'predictor med=%.6f ms\n'], ...
    TT.sensor_median_ms,TT.memory_median_ms,TT.predictor_median_ms);
fprintf(['[TIMING-TAXONOMY] ONLINE plan core med=%.6f ms | ', ...
    'MEASUREMENT dense-kin diag med=%.6f ms\n'], ...
    TT.online_plan_estimate_median_ms,TT.dense_kin_diag_median_ms);
fprintf(['[TIMING-LEDGER-SUMMARY] internalWall med=%.6f | accounted med=%.6f | ', ...
    'unaccounted med=%+.6f | coverage med=%.3f%% | onlineAccounted med=%.6f | ', ...
    'measurementOnly med=%.6f ms\n'], ...
    TT.ledger_internal_wall_median_ms,TT.ledger_accounted_median_ms, ...
    TT.ledger_unaccounted_median_ms,100*TT.ledger_coverage_median_fraction, ...
    TT.ledger_online_accounted_median_ms,TT.ledger_measurement_only_median_ms);
fprintf(['[TIMING-LEDGER-SUMMARY] dynamic SOCP med=%.6f | ', ...
    'dynamic other-cert med=%.6f | geometry residual med=%.6f ms\n'], ...
    TT.dynamic_socp_median_ms,TT.dynamic_non_socp_overhead_median_ms, ...
    TT.geometry_unaccounted_median_ms);
b3_detail_fprintf(['[TIMING-LEDGER-SUMMARY] deferred backend dense diag med=%.6f ms ', ...
    '(MEASUREMENT ONLY, excluded from plan)\n'], ...
    TT.deferred_backend_dense_diag_median_ms);
if CFG.run_post_mission_measurement_audits
    RUN.post_mission_validation=b3_run_post_mission_validation_queue( ...
        RUN.deferred_validation_jobs,P,CFG);
else
    RUN.post_mission_validation=struct( ...
        'jobs',0,'runtime_ms',0,'disabled',true);
end
end



function T=b3_v36_timing_taxonomy(SE,RE)
T=struct();

sensor=[SE.sensor_runtime_ms];
memory=[SE.memory_update_ms];
predictor=[SE.predictor_runtime_ms];
sensor=sensor(isfinite(sensor));
memory=memory(isfinite(memory));
predictor=predictor(isfinite(predictor));

onlinePlan=[];
denseDiag=[];
geom=[];
build=[];
solve=[];
yaw=[];
ledgerWall=[];
ledgerAccounted=[];
ledgerUnaccounted=[];
ledgerCoverage=[];
ledgerOnline=[];
ledgerMeasurement=[];
dynOther=[];
dynSocp=[];
geomResidual=[];
deferredBackendDense=[];

for k=1:numel(RE)
    if isfield(RE(k),'profile')
        P=RE(k).profile;
        if isfield(P,'online_plan_estimate_ms') && isfinite(P.online_plan_estimate_ms)
            onlinePlan(end+1)=P.online_plan_estimate_ms; %#ok<AGROW>
        end
        if isfield(P,'kinematic_dense_diag_ms') && isfinite(P.kinematic_dense_diag_ms)
            denseDiag(end+1)=P.kinematic_dense_diag_ms; %#ok<AGROW>
        end
        if isfield(P,'geometry_ms') && isfinite(P.geometry_ms)
            geom(end+1)=P.geometry_ms; %#ok<AGROW>
        end
        if isfield(P,'yaw_ms') && isfinite(P.yaw_ms)
            yaw(end+1)=P.yaw_ms; %#ok<AGROW>
        end
        if isfield(P,'internal_plan_wall_ms') && isfinite(P.internal_plan_wall_ms)
            ledgerWall(end+1)=P.internal_plan_wall_ms; %#ok<AGROW>
        end
        if isfield(P,'accounted_top_level_ms') && isfinite(P.accounted_top_level_ms)
            ledgerAccounted(end+1)=P.accounted_top_level_ms; %#ok<AGROW>
        end
        if isfield(P,'other_unaccounted_ms') && isfinite(P.other_unaccounted_ms)
            ledgerUnaccounted(end+1)=P.other_unaccounted_ms; %#ok<AGROW>
        end
        if isfield(P,'coverage_fraction') && isfinite(P.coverage_fraction)
            ledgerCoverage(end+1)=P.coverage_fraction; %#ok<AGROW>
        end
        if isfield(P,'online_required_accounted_ms') && isfinite(P.online_required_accounted_ms)
            ledgerOnline(end+1)=P.online_required_accounted_ms; %#ok<AGROW>
        end
        if isfield(P,'measurement_only_ms') && isfinite(P.measurement_only_ms)
            ledgerMeasurement(end+1)=P.measurement_only_ms; %#ok<AGROW>
        end
        if isfield(P,'dynamic_non_socp_overhead_ms') && isfinite(P.dynamic_non_socp_overhead_ms)
            dynOther(end+1)=P.dynamic_non_socp_overhead_ms; %#ok<AGROW>
        end
        if isfield(P,'dynamic_socp_ms') && isfinite(P.dynamic_socp_ms)
            dynSocp(end+1)=P.dynamic_socp_ms; %#ok<AGROW>
        end
        if isfield(P,'geometry_unaccounted_ms') && isfinite(P.geometry_unaccounted_ms)
            geomResidual(end+1)=P.geometry_unaccounted_ms; %#ok<AGROW>
        end
        if isfield(P,'deferred_backend_dense_diag_ms') && isfinite(P.deferred_backend_dense_diag_ms)
            deferredBackendDense(end+1)=P.deferred_backend_dense_diag_ms; %#ok<AGROW>
        end
    end
    if isfinite(RE(k).build_time_ms), build(end+1)=RE(k).build_time_ms; end %#ok<AGROW>
    if isfinite(RE(k).solve_time_ms), solve(end+1)=RE(k).solve_time_ms; end %#ok<AGROW>
end

T.sensor_median_ms=b3_nanmedian_or_nan(sensor);
T.memory_median_ms=b3_nanmedian_or_nan(memory);
T.predictor_median_ms=b3_nanmedian_or_nan(predictor);
T.geometry_median_ms=b3_nanmedian_or_nan(geom);
T.backend_build_median_ms=b3_nanmedian_or_nan(build);
T.backend_solve_median_ms=b3_nanmedian_or_nan(solve);
T.yaw_median_ms=b3_nanmedian_or_nan(yaw);
T.online_plan_estimate_median_ms=b3_nanmedian_or_nan(onlinePlan);
T.dense_kin_diag_median_ms=b3_nanmedian_or_nan(denseDiag);
T.ledger_internal_wall_median_ms=b3_nanmedian_or_nan(ledgerWall);
T.ledger_accounted_median_ms=b3_nanmedian_or_nan(ledgerAccounted);
T.ledger_unaccounted_median_ms=b3_nanmedian_or_nan(ledgerUnaccounted);
T.ledger_coverage_median_fraction=b3_nanmedian_or_nan(ledgerCoverage);
T.ledger_online_accounted_median_ms=b3_nanmedian_or_nan(ledgerOnline);
T.ledger_measurement_only_median_ms=b3_nanmedian_or_nan(ledgerMeasurement);
T.dynamic_non_socp_overhead_median_ms=b3_nanmedian_or_nan(dynOther);
T.dynamic_socp_median_ms=b3_nanmedian_or_nan(dynSocp);
T.geometry_unaccounted_median_ms=b3_nanmedian_or_nan(geomResidual);
T.deferred_backend_dense_diag_median_ms=b3_nanmedian_or_nan(deferredBackendDense);

T.classification=struct( ...
    'online_required',{{'sensor','memory_update','predictor_when_needed', ...
        'geometry_corridor','backend_dynamic_assembly','solver','hard_certificate','rigid_cable_certificate','yaw_switch'}}, ...
    'precomputable_candidates',{{'static_basis_maps','binomial_tables','restriction_maps', ...
        'fixed_direction_sets','constant_proxy_geometry'}}, ...
    'measurement_only',{{'cause_isolation','dense_kinematic_diagnostic','dense_rigid_cable_diagnostic','dense_sreenath_model_diagnostic','verbose_audit'}});
end

function x=b3_nanmedian_or_nan(v)
if isempty(v), x=NaN; else, x=median(v); end
end


function [safe,lb]=b3_point_ellipsoid_sphere_safe(p,ob,bodyRadius)
rb=max(ob.radii(:));
lb=norm(p(:)-ob.center(:))-rb-bodyRadius;
scale=max([1;abs(p(:));abs(ob.center(:));rb;bodyRadius]);
guard=64*eps(scale);
safe=isfinite(lb) && lb>guard;
end


function [safe,lb]=b3_segment_ellipsoid_sphere_safe(A,B,ob,bodyRadius)
c=ob.center(:); A=A(:); B=B(:); d=B-A; dd=d.'*d;
if dd<=0
    ds=norm(A-c);
else
    u=max(0,min(1,((c-A).'*d)/dd));
    ds=norm(A+u*d-c);
end
rb=max(ob.radii(:));
lb=ds-rb-bodyRadius;
scale=max([1;abs(A);abs(B);abs(c);rb;bodyRadius]);
guard=64*eps(scale);
safe=isfinite(lb) && lb>guard;
end


function reject=b3_dynamic_forbidden_sphere_reject(Ppos,ob,ta,tb,P,component)
cp=mean(Ppos,2);
rp=max(vecnorm(Ppos-cp,2,1));
tm=0.5*(ta+tb);
c=b3_obstacle_center_at_time(ob,tm);
rb=max(ob.radii(:));
rv=0.5*abs(tb-ta)*norm(ob.v_center(:));

switch upper(component)
    case 'PAYLOAD'
        rcomp=P.R_payload+P.d_margin; rshift=0;
    case 'UAV'
        rcomp=P.R_uav+P.d_margin; rshift=P.L_cable;
    case 'CABLE'
        rcomp=P.R_cable+P.d_margin; rshift=P.L_cable;
    otherwise
        reject=false; return;
end

rf=rb+rv+rcomp+rshift;
gap=norm(cp-c)-rp-rf;
scale=max([1;abs(cp);abs(c);rp;rf]);
guard=128*eps(scale);
reject=isfinite(gap) && gap>guard;
end


function J=b3_empty_deferred_validation_job()
J=struct('backend','','ref_id',NaN,'cand',struct(),'obs',[], ...
    'n_patches',0,'event_profile',struct());
end


function A=b3_empty_deferred_validation_job_array()
A=repmat(b3_empty_deferred_validation_job(),0,1);
end


function J=b3_make_deferred_validation_job(cand,mem,t,PE)
required={'valid','backend','reference_id','result','model','T','t_start','t_end'};
for k=1:numel(required)
    if ~isfield(cand,required{k})
        error('STEP4B3:DeferredCandSchema', ...
            'Candidate missing required deferred-validation field: %s',required{k});
    end
end
J=b3_empty_deferred_validation_job();
J.backend=cand.backend;
% Canonical candidate identifier is cand.reference_id.
J.ref_id=cand.reference_id;
J.cand=cand;
J.obs=b3_memory_snapshot(mem,t);
J.n_patches=numel(J.obs);
J.event_profile=PE.profile;
end


function OUT=b3_run_post_mission_validation_queue(JOBS,P,CFG)
OUT=struct('jobs',numel(JOBS),'runtime_ms',0,'job_results',[]);
if isempty(JOBS)
    fprintf('[POST-MISSION-VALIDATION] jobs=0\n');
    return;
end

fprintf('\n[POST-MISSION-VALIDATION] jobs=%d | begin after mission runtime is complete\n',numel(JOBS));
qAll=tic;
RR=repmat(struct('ref_id',NaN,'backend','','patches',0, ...
    'dense_slack',NaN,'dense_samples',0,'dense_ms',NaN, ...
    'kin_dense_ms',NaN,'cable_dense_ms',NaN,'sreenath_dense_ms',NaN),numel(JOBS),1);

requiredJobFields={'backend','ref_id','cand','obs','n_patches','event_profile'};
for jf=1:numel(requiredJobFields)
    if ~isfield(JOBS,requiredJobFields{jf})
        error('STEP4B3:DeferredJobSchema', ...
            'Deferred validation job schema missing field: %s',requiredJobFields{jf});
    end
end

for k=1:numel(JOBS)
    J=JOBS(k); C=J.cand;
    requiredCandFields={'valid','backend','reference_id','result','model','T','t_start','t_end'};
    for cf=1:numel(requiredCandFields)
        if ~isfield(C,requiredCandFields{cf})
            error('STEP4B3:DeferredCandSchema', ...
                'Deferred candidate missing field: %s',requiredCandFields{cf});
        end
    end

    q=tic;
    [denseSlack,nSamples]=b3_deferred_backend_dense_ellipsoid_diag_from_snapshot(C,J.obs,P);
    denseMs=1e3*toc(q);

    q=tic;
    K=b3_kinematic_bezier_certificate_post(C.result.control_points,C.model,C.T,C.backend,CFG); %#ok<NASGU>
    kinMs=1e3*toc(q);

    q=tic;
    CDD=b3_dense_rigid_cable_diagnostic(C.result.control_points,C.model,C.T,P,CFG); %#ok<NASGU>
    cableMs=1e3*toc(q);

    q=tic;
    SDD=b3_dense_sreenath_model_diagnostic(C.result.control_points,C.model,C.T,P,CFG); %#ok<NASGU>
    sreenathMs=1e3*toc(q);

    RR(k)=struct('ref_id',J.ref_id,'backend',J.backend,'patches',J.n_patches, ...
        'dense_slack',denseSlack,'dense_samples',nSamples,'dense_ms',denseMs, ...
        'kin_dense_ms',kinMs,'cable_dense_ms',cableMs,'sreenath_dense_ms',sreenathMs);

    fprintf(['  [POST-VALIDATION] %02d/%02d ref=%d backend=%s patches=%d ', ...
        'ellipsoid=%.3f kin=%.3f cable=%.3f sreenath=%.3f ms slack=%+.9e\n'], ...
        k,numel(JOBS),J.ref_id,J.backend,J.n_patches, ...
        denseMs,kinMs,cableMs,sreenathMs,denseSlack);
end

OUT.runtime_ms=1e3*toc(qAll);
OUT.job_results=RR;
fprintf('[POST-MISSION-VALIDATION] done total=%.3f ms | excluded from mission/runtime timing\n',OUT.runtime_ms);
end


function [slack,nSamples]=b3_deferred_backend_dense_ellipsoid_diag_from_snapshot(cand,obs,P)
switch upper(cand.backend)
    case {'BEZIER_D13','POLY_D13'}
        [~,Pts]=b2_d13_eval_dense(cand.result.control_points,cand.model,301);
    case 'BSPLINE_D7'
        [~,Pts]=bsd7_eval_dense(cand.result.native_control_points,cand.model,301);
    otherwise
        error('STEP4B3:DeferredDenseBackend','Unsupported backend %s',cand.backend);
end
nSamples=size(Pts,2);
slack=bez13_dense_exact_ellipsoid_slack(Pts,obs,P);
end


function K=b3_kinematic_bezier_certificate_post(CP,M,T,backend,CFG)
CFG2=CFG;
CFG2.defer_measurement_diagnostics=false;
K=b3_kinematic_bezier_certificate(CP,M,T,backend,CFG2);
end


function [slack,nSamples]=b3_deferred_backend_dense_ellipsoid_diag(cand,mem,P)
obs=b3_memory_snapshot(mem,cand.t_start);
[slack,nSamples]=b3_deferred_backend_dense_ellipsoid_diag_from_snapshot(cand,obs,P);
end


function [cand,EV]=b3_plan_once(backend,t0,Dstart,Ystart,mem,risk,CFG,P,newRefId)
qPlanInternal=tic;
cand=b3_empty_active();
EV=b3_empty_replan_event();
EV.t=t0;
EV.backend=backend;
EV.source_reference_id=newRefId-1;
EV.proposed_reference_id=newRefId;
EV.known_ids=b3_memory_ids(mem);
EV.risk_ids=risk.ids;
EV.risk_ttc_s=risk.TTC;
EV.start_physical_jet=Dstart;
EV.start_yaw_physical_jet=Ystart;

EV.profile.ledger_ready=false;
EV.profile.bookkeeping_ms=0;
EV.profile.logging_ms=0;
EV.profile.failure_diagnostic_ms=0;
EV.profile.candidate_pack_ms=0;
EV.profile.finalize_ms=0;
EV.profile.other_unaccounted_ms=NaN;
EV.profile.accounted_top_level_ms=NaN;
EV.profile.internal_plan_wall_ms=NaN;
EV.profile.coverage_fraction=NaN;
EV.profile.closure_error_ms=NaN;
EV.profile.measurement_only_ms=0;
EV.profile.online_required_accounted_ms=NaN;
EV.profile.dynamic_non_socp_overhead_ms=NaN;
EV.profile.geometry_child_sum_ms=NaN;
EV.profile.geometry_unaccounted_ms=NaN;
EV.profile.backend_wrapper_overhead_ms=NaN;
EV.profile.kinematic_core_ms=NaN;
EV.profile.yaw_log_ms=0;

qStage=tic;
obs0=b3_memory_snapshot(mem,t0);
EV.profile.memory_snapshot_ms=1e3*toc(qStage);

qStage=tic;

try
    [G,C,T,geomDiag]=b3_build_replan_geometry(backend,t0,Dstart,obs0,risk.ids,CFG,P);
    EV.profile.geometry_ms=1e3*toc(qStage);
    if isfield(geomDiag,'profile')
        gp=geomDiag.profile;
        EV.profile.geometry_progress_ms=b3_getfield_or_nan(gp,'progress_time_ms');
        EV.profile.geometry_direction_ms=b3_getfield_or_nan(gp,'direction_ms');
        EV.profile.geometry_seed_ms=b3_getfield_or_nan(gp,'seed_build_ms');
        EV.profile.geometry_corridor_ms=b3_getfield_or_nan(gp,'corridor_ms');
        gchild=[EV.profile.geometry_progress_ms,EV.profile.geometry_direction_ms, ...
            EV.profile.geometry_seed_ms,EV.profile.geometry_corridor_ms];
        gchild=gchild(isfinite(gchild));
        EV.profile.geometry_child_sum_ms=sum(gchild);
        EV.profile.geometry_unaccounted_ms=max(0,EV.profile.geometry_ms-EV.profile.geometry_child_sum_ms);
    end
catch ME
    EV.profile.geometry_ms=1e3*toc(qStage);
    EV.failure_reason=['GEOMETRY_BUILD_FAILED: ',ME.identifier,' ',ME.message];
    qLog=tic;
    fprintf('    [PROFILE-PLAN] memory=%.6f ms | geometry_until_fail=%.6f ms\n', ...
        EV.profile.memory_snapshot_ms,EV.profile.geometry_ms);
    EV.profile.logging_ms=EV.profile.logging_ms+1e3*toc(qLog);
    EV=b3_finalize_timing_ledger(EV,qPlanInternal,'GEOMETRY_FAIL');
    return;
end

if ~(isfinite(T) && T>0 && t0+T<=CFG.mission_tf+1e-10)
    EV.failure_reason='INVALID_REPLAN_DURATION';
    EV=b3_finalize_timing_ledger(EV,qPlanInternal,'INVALID_DURATION');
    return;
end

qStage=tic;
Dend=G.end_physical_jet;
J0s=b2_physical_to_spatial_jet(Dstart,T);
J1s=b2_physical_to_spatial_jet(Dend,T);
EV.profile.jet_prepare_ms=1e3*toc(qStage);

qStage=tic;
qLog=tic;

b3_detail_fprintf('    [GEOM] T=%+.9f path=%+.9f bypass=%+.9f meanSpeedRatio=%+.9f guideSlack=%+.9e\n', ...
    T,G.path_length,G.bypass_offset,geomDiag.required_mean_speed_ratio,geomDiag.min_guide_slack);
b3_detail_fprintf('    [GEOM] W0=%s\n',mat2str(G.waypoints(:,1).',9));
b3_detail_fprintf('    [GEOM] W1=%s\n',mat2str(G.waypoints(:,2).',9));
b3_detail_fprintf('    [GEOM] W2=%s\n',mat2str(G.waypoints(:,3).',9));
b3_detail_fprintf('    [GEOM] W3=%s\n',mat2str(G.waypoints(:,4).',9));
if isfield(geomDiag,'min_separator_margin')
    b3_detail_fprintf('    [GEOM] seedCompatibleSeparatorMargin=%+.16e method=%s\n', ...
        geomDiag.min_separator_margin,geomDiag.corridor.separator_method);
    if isfield(geomDiag.corridor,'cheap_reject_runtime_ms')
        fprintf('    [CASCADE] cheapReject=%.6f ms | primalDistanceSOCP=%.6f ms | corridorTotal=%.6f ms\n', ...
            geomDiag.corridor.cheap_reject_runtime_ms, ...
            geomDiag.corridor.distance_socp_runtime_ms, ...
            geomDiag.corridor.runtime_ms);
    end
end

EV.profile.logging_ms=EV.profile.logging_ms+1e3*toc(qLog);

% Diagnostic static snapshot for the legacy dense ellipsoid diagnostic only.
% Acceptance authority is the new swept-support corridor hard slack.
obsDiag=obs0;

switch upper(backend)
    case {'BEZIER_D13','POLY_D13'}
        R=b2_solve_d13_fixed_boundary(G,C,J0s,J1s,obsDiag,P,T);
        M=b2_d13_build_model_fixed_boundary(G,C,J0s,J1s);
    case 'BSPLINE_D7'
        R=b2_solve_bspline_fixed_boundary(G,C,J0s,J1s,obsDiag,P,T);
        M=b2_bspline_build_model_fixed_boundary(G,C,J0s,J1s);
    otherwise
        EV.failure_reason='UNKNOWN_BACKEND';
        EV.profile.backend_build_solve_ms=1e3*toc(qStage);
        EV=b3_finalize_timing_ledger(EV,qPlanInternal,'UNKNOWN_BACKEND');
        return;
end
EV.profile.backend_build_solve_ms=1e3*toc(qStage);

EV.T=T;
EV.t_end=t0+T;
EV.end_physical_jet=Dend;
EV.geometry=G;
EV.corridor=C;
EV.geometry_diagnostic=geomDiag;
EV.required_mean_guide_speed=geomDiag.required_mean_guide_speed;
EV.required_mean_speed_ratio=geomDiag.required_mean_speed_ratio;
EV.solver_feasible=R.solver_feasible;
EV.backend_pass=R.pass;
EV.exitflag=R.exitflag;
EV.min_original_hard_slack=R.min_original_hard_slack;
EV.start_spatial_jet_residual_d0_d6=R.start_spatial_jet_residual_d0_d6;
EV.end_spatial_jet_residual_d0_d6=R.end_spatial_jet_residual_d0_d6;
EV.start_physical_splice_residual_d0_d6=R.start_physical_splice_residual_d0_d6;
EV.end_physical_residual_d0_d6=R.end_physical_residual_d0_d6;
EV.c6_internal_residual=R.c6_internal_residual;
EV.c6_internal_physical_residual_d0_d6=R.c6_internal_physical_residual_d0_d6;
EV.c6_internal_guard_d0_d6=R.c6_internal_guard_d0_d6;
EV.start_physical_splice_guard_d0_d6=R.start_physical_splice_guard_d0_d6;
EV.end_physical_guard_d0_d6=R.end_physical_guard_d0_d6;
EV.c6_internal_backward_error_certified=R.c6_internal_backward_error_certified;
EV.start_physical_backward_error_certified=R.start_physical_backward_error_certified;
EV.end_physical_backward_error_certified=R.end_physical_backward_error_certified;
EV.c6_hard_certified=R.c6_hard_certified;
EV.dense_exact_ellipsoid_slack=R.dense_exact_ellipsoid_slack;
EV.build_time_ms=R.build_time_ms;
EV.solve_time_ms=R.solve_time_ms;
EV.profile.backend_post_reconstruct_ms=b3_getfield_or_zero(R,'post_reconstruct_ms');
EV.profile.backend_post_slack_ms=b3_getfield_or_zero(R,'post_slack_ms');
EV.profile.backend_post_c6_audit_ms=b3_getfield_or_zero(R,'post_c6_audit_ms');
EV.row_numeric_certified=R.original_row_numeric_certified;
EV.c6_structural_certified=R.c6_structural_certified;
qBook=tic;
EV.profile.backend_wrapper_overhead_ms=max(0,EV.profile.backend_build_solve_ms-R.build_time_ms-R.solve_time_ms);
EV.profile.bookkeeping_ms=EV.profile.bookkeeping_ms+1e3*toc(qBook);
qLog=tic;
b3_detail_fprintf(['    [PROFILE-BACKEND] build=%.6f | solve=%.6f | reconstruct=%.6f | ', ...
    'slack/rowCert=%.6f | C6audit=%.6f ms\n'], ...
    R.build_time_ms,R.solve_time_ms, ...
    b3_getfield_or_zero(R,'post_reconstruct_ms'), ...
    b3_getfield_or_zero(R,'post_slack_ms'), ...
    b3_getfield_or_zero(R,'post_c6_audit_ms'));
fprintf(['    [C6-CERT] backend=%s hard=%d structural=%d | ', ...
    'startCert=%d internalCert=%d endCert=%d\n'], ...
    backend,R.c6_hard_certified,R.c6_structural_certified, ...
    R.start_physical_backward_error_certified, ...
    R.c6_internal_backward_error_certified, ...
    R.end_physical_backward_error_certified);
b3_detail_fprintf('    [C6-CERT] start residual=%s\n', ...
    mat2str(R.start_physical_splice_residual_d0_d6,9));
b3_detail_fprintf('    [C6-CERT] start guard   =%s\n', ...
    mat2str(R.start_physical_splice_guard_d0_d6,9));
b3_detail_fprintf('    [C6-CERT] internal max residual=%s\n', ...
    mat2str(R.c6_internal_physical_residual_d0_d6,9));
b3_detail_fprintf('    [C6-CERT] internal max guard   =%s\n', ...
    mat2str(R.c6_internal_guard_d0_d6,9));
b3_detail_fprintf('    [C6-CERT] end residual=%s\n', ...
    mat2str(R.end_physical_residual_d0_d6,9));
b3_detail_fprintf('    [C6-CERT] end guard   =%s\n', ...
    mat2str(R.end_physical_guard_d0_d6,9));
EV.profile.logging_ms=EV.profile.logging_ms+1e3*toc(qLog);

if ~R.pass
    EV.failure_reason='BACKEND_HARD_CERTIFICATE_FAILED';

    finiteStartSpatial=all(isfinite(R.start_spatial_jet_residual_d0_d6));
    finiteEndSpatial=all(isfinite(R.end_spatial_jet_residual_d0_d6));
    finiteStartPhysical=all(isfinite(R.start_physical_splice_residual_d0_d6));
    finiteEndPhysical=all(isfinite(R.end_physical_residual_d0_d6));
    finiteC6=R.c6_hard_certified;
    slackPass=isfinite(R.min_original_hard_slack) && (R.min_original_hard_slack>=0);
    if isfield(R,'original_row_numeric_certified') && R.original_row_numeric_certified
        slackPass=true;
    end

    fprintf('    [HARDCERT] feasible=%d exitflag=%g hardSlack=%+.16e slackPass=%d\n', ...
        R.solver_feasible,R.exitflag,R.min_original_hard_slack,slackPass);
    if isfield(R.solver_output,'v22_interior_certificate')
        VI=R.solver_output.v22_interior_certificate;
        fprintf('    [HARDCERT-V22] interiorRho=%+.16e interiorExit=%g directSlack=%+.16e\n', ...
            VI.rho,VI.exitflag,VI.direct_slack);
    end
    fprintf('    [HARDCERT] startSpatial finite=%d residual=%s\n', ...
        finiteStartSpatial,mat2str(R.start_spatial_jet_residual_d0_d6,16));
    fprintf('    [HARDCERT] endSpatial   finite=%d residual=%s\n', ...
        finiteEndSpatial,mat2str(R.end_spatial_jet_residual_d0_d6,16));
    fprintf('    [HARDCERT] startPhysical finite=%d residual=%s\n', ...
        finiteStartPhysical,mat2str(R.start_physical_splice_residual_d0_d6,16));
    fprintf('    [HARDCERT] endPhysical   finite=%d residual=%s\n', ...
        finiteEndPhysical,mat2str(R.end_physical_residual_d0_d6,16));
    fprintf('    [HARDCERT] C6 finite=%d spatialResidual=%+.16e denseEllipsoidSlack=%+.16e\n', ...
        finiteC6,R.c6_internal_residual,R.dense_exact_ellipsoid_slack);
    if isfield(R,'c6_structural_certified')
        fprintf(['    [HARDCERT-C6] structural=%d maxInternalKnotMultiplicity=%g ', ...
            'physicalResidual_d0_d6=%s\n'], ...
            R.c6_structural_certified,R.c6_max_internal_knot_multiplicity, ...
            mat2str(R.c6_internal_physical_residual_d0_d6,16));
    end
    if isfield(R,'original_row_numeric_certified')
        fprintf(['    [HARDCERT-ROW] fpCertified=%d minCertifiedMargin=%+.16e ', ...
            'maxFpGuard=%+.16e\n'], ...
            R.original_row_numeric_certified,R.original_row_min_certified_margin, ...
            R.original_row_max_fp_guard);
    end
    fprintf('    [HARDCERT] FAIL FLAGS: solver=%d slack=%d startS=%d endS=%d startT=%d endT=%d c6=%d\n', ...
        ~R.solver_feasible,~slackPass,~finiteStartSpatial,~finiteEndSpatial, ...
        ~finiteStartPhysical,~finiteEndPhysical,~finiteC6);
    qFail=tic;
    fprintf('    [PROFILE-PLAN] memory=%.6f | geometry=%.6f | jet=%.6f | backendBuildSolve=%.6f ms\n', ...
        EV.profile.memory_snapshot_ms,EV.profile.geometry_ms,EV.profile.jet_prepare_ms,EV.profile.backend_build_solve_ms);
    EV.profile.failure_diagnostic_ms=EV.profile.failure_diagnostic_ms+1e3*toc(qFail);
    EV=b3_finalize_timing_ledger(EV,qPlanInternal,'BACKEND_FAIL');
    return;
end


% -------------------------------------------------------------------------
% V19 MINIMAL KINEMATIC CERTIFICATE
% -------------------------------------------------------------------------
% This does NOT add velocity/acceleration constraints to the trajectory QP.
% It independently certifies the generated continuous-time trajectory using
% the convex-hull property of Bezier derivative control points.
%
% For a degree-n Bezier piece with physical duration dt:
%   V_i = n(P_{i+1}-P_i)/dt
%   A_i = n(n-1)(P_{i+2}-2P_{i+1}+P_i)/dt^2
% and the derivative curves lie in conv{V_i} and conv{A_i}. Therefore
%   ||v(t)|| <= max_i ||V_i||,  ||a(t)|| <= max_i ||A_i||
% continuously, with no temporal sampling.
%
% Orders 3..6 are logged only. This intentionally avoids over-constraining
% the generator at the present stage.
qKin=tic;
K=b3_kinematic_bezier_certificate(R.control_points,M,T,backend,CFG);
EV.profile.kinematic_total_ms=1e3*toc(qKin);
EV.profile.kinematic_dense_diag_ms=K.dense_diagnostic_runtime_ms;
EV.profile.kinematic_core_ms=max(0,EV.profile.kinematic_total_ms-EV.profile.kinematic_dense_diag_ms);
EV.profile.measurement_only_ms=EV.profile.measurement_only_ms+EV.profile.kinematic_dense_diag_ms;
EV.kinematic=K;
qLog=tic;
fprintf(['    [KIN-CERT] vmaxBound=%.9g m/s | amaxBound=%.9g m/s^2 | ', ...
    'limits=[%s,%s] | hardGate=%d | pass=%d\n'], ...
    K.max_speed_bound,K.max_acc_bound,b3_num_or_unspecified(CFG.max_ref_speed), ...
    b3_num_or_unspecified(CFG.max_ref_acc),K.hard_gate_active,K.pass);
if CFG.defer_measurement_diagnostics
    b3_detail_fprintf('    [KIN-DENSE-DIAG] deferred to post-mission validation\n');
else
fprintf(['    [KIN-DENSE-DIAG] vmax=%.9g m/s | amax=%.9g m/s^2 | ', ...
    'bound/dense=[%.6g %.6g] | samples=%d | diag=%.6f ms\n'], ...
    K.dense_realized_max_speed,K.dense_realized_max_acc, ...
    K.speed_bound_to_dense_ratio,K.acc_bound_to_dense_ratio, ...
    K.dense_sample_count,K.dense_diagnostic_runtime_ms);
end
b3_detail_fprintf('    [KIN-D1..D6] upperBounds=%s\n',mat2str(K.derivative_norm_upper_bound,9));
b3_detail_fprintf(['    [SCHEDULE] meanSpeedRatio=%.9f catchupRequired=0 | ', ...
    'spatialRejoinPhase=%.9f s | nominalTimeShiftAfter=%.9f s\n'], ...
    geomDiag.required_mean_speed_ratio,G.rejoin_nominal_phase,(t0+T)-G.rejoin_nominal_phase);
if ~K.pass
    fprintf(['    [KIN-CERT] ADVISORY ONLY: preferred v/a bounds are not met. ', ...
        'Safety and C6 remain the only hard acceptance conditions.\n']);
end
EV.profile.logging_ms=EV.profile.logging_ms+1e3*toc(qLog);

% -------------------------------------------------------------------------
% V37 HARD MASSLESS-RIGID-CABLE CERTIFICATE
% -------------------------------------------------------------------------
% The corridor may have been constructed from the old vertical proxy, but a
% candidate is accepted only if the actual acceleration-induced p(t) can be
% enclosed continuously by local p-cones and every PAYLOAD/UAV/CABLE support
% inequality remains valid with those cones.
qCableCert=tic;
CD=b3_rigid_cable_continuous_certificate(backend,R,M,C,T,obs0,P,CFG);
EV.profile.rigid_cable_certificate_ms=1e3*toc(qCableCert);
EV.profile.dynamic_socp_ms=CD.dynamic_socp_runtime_ms;
EV.profile.dynamic_broadphase_ms=CD.broadphase_runtime_ms;
EV.profile.dynamic_analytic_ms=CD.analytic_witness_check_ms;
EV.profile.dynamic_reuse_ms=CD.normal_reuse_fastpath_check_ms;
EV.profile.dynamic_non_socp_overhead_ms=max(0,EV.profile.rigid_cable_certificate_ms- ...
    EV.profile.dynamic_socp_ms-EV.profile.dynamic_broadphase_ms- ...
    EV.profile.dynamic_analytic_ms-EV.profile.dynamic_reuse_ms);
EV.cable_dynamic=CD;

qLog=tic;
b3_print_dynamic_summary(CD,EV.profile);
b3_detail_fprintf(['    [RIGID-CABLE-CERT] pass=%d | globalSwingBound=%.9g deg | ', ...
    'globalMinTensionPerMassLB=%.9g m/s^2 | minDynamicSlack=%+.9e | ', ...
    'acceptedLeaves=%d splitNodes=%d maxDepth=%d unresolved=%d | ', ...
    'dynSOCP=%d supportCert=%d solverNonPosButCert=%d | ', ...
    'broadphase=%d/%d bpTime=%.6f ms | ', ...
    'analyticCheck=%d analyticHit=%d analyticTime=%.6f ms | ', ...
    'reuseCheck=%d reuseHit=%d reuseCheckTime=%.6f ms | ', ...
    'lipTry=%d lipCert=%d | dynSOCPtime=%.6f ms | total=%.6f ms\n'], ...
    CD.pass,CD.max_swing_bound_deg,CD.min_tension_per_mass_lb, ...
    CD.min_dynamic_corridor_slack,CD.accepted_leaf_count,CD.split_node_count, ...
    CD.max_subdivision_depth,CD.unresolved_leaf_count, ...
    CD.dynamic_socp_calls,CD.support_certified_count, ...
    CD.solver_status_nonpositive_but_support_certified, ...
    CD.broadphase_tests,CD.broadphase_rejects,CD.broadphase_runtime_ms, ...
    CD.analytic_witness_checks,CD.analytic_witness_hits,CD.analytic_witness_check_ms, ...
    CD.normal_reuse_fastpath_checks,CD.normal_reuse_fastpath_hits, ...
    CD.normal_reuse_fastpath_check_ms, ...
    CD.early_lipschitz_attempts,CD.early_lipschitz_certified, ...
    CD.dynamic_socp_runtime_ms,EV.profile.rigid_cable_certificate_ms);
b3_detail_fprintf(['    [SREENATH-CONT-BOUNDS] vL<=%.9g m/s | jerk<=%.9g m/s^3 | ', ...
    'rz>=%.9g m/s^2 | pdot<=%.9g rad/s | vQ<=%.9g m/s | valid=%d\n'], ...
    CD.global_dynamics.max_payload_speed_bound, ...
    CD.global_dynamics.max_payload_jerk_bound, ...
    CD.global_dynamics.min_resultant_vertical_lb, ...
    CD.global_dynamics.max_pdot_bound, ...
    CD.global_dynamics.max_uav_speed_bound, ...
    CD.global_dynamics.valid_lipschitz);
EV.profile.logging_ms=EV.profile.logging_ms+1e3*toc(qLog);
if CFG.defer_measurement_diagnostics && CD.pass
    EV.profile.rigid_cable_dense_diag_ms=0;
    EV.profile.sreenath_dense_diag_ms=0;
    EV.cable_dynamic.dense_diagnostic=struct('deferred',true);
    EV.cable_dynamic.sreenath_dense_diagnostic=struct('deferred',true);
    qLog=tic;
    b3_detail_fprintf('    [DENSE-DIAGNOSTICS] cable/Sreenath dense audits deferred to post-mission validation\n');
    EV.profile.logging_ms=EV.profile.logging_ms+1e3*toc(qLog);
else
    qCableDense=tic;
    CDD=b3_dense_rigid_cable_diagnostic(R.control_points,M,T,P,CFG);
    EV.profile.rigid_cable_dense_diag_ms=1e3*toc(qCableDense);
    EV.cable_dynamic.dense_diagnostic=CDD;
    qSreenathDense=tic;
    SDD=b3_dense_sreenath_model_diagnostic(R.control_points,M,T,P,CFG);
    EV.profile.sreenath_dense_diag_ms=1e3*toc(qSreenathDense);
    EV.profile.measurement_only_ms=EV.profile.measurement_only_ms+ ...
        EV.profile.rigid_cable_dense_diag_ms+EV.profile.sreenath_dense_diag_ms;
    EV.cable_dynamic.sreenath_dense_diagnostic=SDD;
    qLog=tic;
    fprintf(['    [RIGID-CABLE-DENSE-DIAG] maxSwing=%.9g deg | ', ...
        'minTensionPerMass=%.9g m/s^2 | samples=%d | diag=%.6f ms\n'], ...
        CDD.max_swing_deg,CDD.min_tension_per_mass,CDD.sample_count, ...
        EV.profile.rigid_cable_dense_diag_ms);
    fprintf(['    [SREENATH-DENSE-DIAG] maxSwing=%.9g deg | maxOmega=%.9g rad/s | ', ...
        'forceModel=%d | maxThrust=%g N | eq218=%g | eq220=%g | samples=%d | diag=%.6f ms\n'], ...
        SDD.max_swing_deg,SDD.max_omega_rad_s,SDD.force_model_available, ...
        SDD.max_required_thrust_N,SDD.max_eq218_residual,SDD.max_eq220_residual, ...
        SDD.sample_count,EV.profile.sreenath_dense_diag_ms);
    EV.profile.logging_ms=EV.profile.logging_ms+1e3*toc(qLog);
end


if ~CD.pass
    EV.backend_pass=false;
    EV.accepted=false;
    EV.failure_reason='RIGID_CABLE_DYNAMIC_CERTIFICATE_FAILED';
    fprintf(['    [RIGID-CABLE-CERT] FAIL cell=%d subcell=%d span=%d ', ...
        'obs=%g component=%s class=%s reason=%s\n'], ...
        CD.worst_cell,CD.worst_subcell,CD.worst_span,CD.worst_obstacle_id, ...
        CD.worst_component,CD.failure_class,CD.failure_reason);

if isstruct(CD.limit_point_audit) && ~isempty(fieldnames(CD.limit_point_audit))
    PA=CD.limit_point_audit;
    fprintf(['    [LIMIT-POINT-AUDIT] valid=%d t=[%.17g %.17g] u=[%.17g %.17g] ', ...
        'component=%s minClr=%+.12e guard=%.3e collision=%d pointSafe=%d ', ...
        'Vrel=%.9g halfDt=%.3e drop=%.3e contLB=%+.12e lipCert=%d ', ...
        'local=[vL %.6g jerk %.6g rz %.6g pdot %.6g vQ %.6g valid %d]\n'], ...
        PA.valid,CD.failure_ta,CD.failure_tb,CD.failure_u0,CD.failure_u1, ...
        PA.component,PA.min_selected_clearance,PA.clearance_guard, ...
        PA.pointwise_collision_certified,PA.pointwise_strictly_safe, ...
        PA.relative_speed_bound,PA.half_interval,PA.lipschitz_drop_bound, ...
        PA.continuous_lower_bound,PA.continuous_lipschitz_certified, ...
        PA.local_vL_bound,PA.local_jerk_bound,PA.local_rz_lb, ...
        PA.local_pdot_bound,PA.local_vQ_bound,PA.local_bounds_valid);
    fprintf(['    [LIMIT-POINT-AUDIT] clrPayload=%s | clrUAV=%s | clrCable=%s\n'], ...
        mat2str(PA.payload_clearance,9),mat2str(PA.uav_clearance,9), ...
        mat2str(PA.cable_clearance,9));
end
    EV=b3_finalize_timing_ledger(EV,qPlanInternal,'RIGID_CABLE_FAIL');
    return;
end

qBookCandidate=tic;
cand.valid=true;
cand.backend=backend;
cand.t_start=t0;
cand.T=T;
cand.t_end=t0+T;
cand.reference_id=newRefId;
cand.result=R;
cand.model=M;
cand.geometry=G;
cand.corridor=C;
cand.cable_dynamic=CD;
cand.known_ids=EV.known_ids;
cand.risk_ids=EV.risk_ids;
cand.start_jet=Dstart;
cand.end_jet=Dend;
cand.rejoin_nominal_phase=G.rejoin_nominal_phase;
cand.post_nominal_time_shift=(t0+T)-G.rejoin_nominal_phase;
EV.profile.candidate_pack_ms=1e3*toc(qBookCandidate);
qStage=tic;
cand.yaw=b3_build_c6_yaw_for_active(cand,Ystart,CFG);
EV.profile.yaw_ms=1e3*toc(qStage);
EV.end_yaw_physical_jet=cand.yaw.end_physical_jet;
EV.yaw_start_splice_residual_d0_d6=cand.yaw.start_splice_residual_d0_d6;
EV.yaw_end_residual_d0_d6=cand.yaw.end_residual_d0_d6;
EV.yaw_start_splice_guard_d0_d6=cand.yaw.start_splice_guard_d0_d6;
EV.yaw_end_guard_d0_d6=cand.yaw.end_guard_d0_d6;
EV.yaw_c6_hard_certified=cand.yaw.c6_hard_certified;
EV.yaw_constraint_backward_error_certified= ...
    cand.yaw.constraint_backward_error_certified;
EV.yaw_constraint_residual_inf=cand.yaw.constraint_residual_inf;
EV.yaw_constraint_guard_inf=cand.yaw.constraint_guard_inf;
EV.yaw_fit_max_abs_rad=cand.yaw.max_abs_fit_error_rad;

qLog=tic;
fprintf(['    [YAW-C6-CERT] hard=%d structural=%d analyticEndpoints=%d ', ...
    'startCert=%d endCert=%d eqCert=%d fitMax=%.6e rad\n'], ...
    cand.yaw.c6_hard_certified,cand.yaw.structural_c6, ...
    cand.yaw.endpoint_controls_analytic, ...
    cand.yaw.start_backward_error_certified, ...
    cand.yaw.end_backward_error_certified, ...
    cand.yaw.constraint_backward_error_certified, ...
    cand.yaw.max_abs_fit_error_rad);
b3_detail_fprintf(['    [YAW-C6-CERT] endpointEq residualInf=%.6e guardInf=%.6e ', ...
    'target=%s\n'], ...
    cand.yaw.constraint_residual_inf,cand.yaw.constraint_guard_inf, ...
    cand.yaw.target_definition);
b3_detail_fprintf('    [YAW-C6-CERT] start residual=%s\n', ...
    mat2str(cand.yaw.start_splice_residual_d0_d6,9));
b3_detail_fprintf('    [YAW-C6-CERT] start guard   =%s\n', ...
    mat2str(cand.yaw.start_splice_guard_d0_d6,9));
b3_detail_fprintf('    [YAW-C6-CERT] end residual=%s\n', ...
    mat2str(cand.yaw.end_residual_d0_d6,9));
b3_detail_fprintf('    [YAW-C6-CERT] end guard   =%s\n', ...
    mat2str(cand.yaw.end_guard_d0_d6,9));
EV.profile.yaw_log_ms=1e3*toc(qLog);
EV.profile.logging_ms=EV.profile.logging_ms+EV.profile.yaw_log_ms;

if ~cand.yaw.c6_hard_certified
    cand.valid=false;
    EV.accepted=false;
    EV.failure_reason='YAW_C6_HARD_CERTIFICATE_FAILED';
    EV=b3_finalize_timing_ledger(EV,qPlanInternal,'YAW_FAIL');
    return;
end

qFinal=tic;
EV.accepted=true;
EV.failure_reason='NONE';
EV.profile.finalize_ms=1e3*toc(qFinal);
EV=b3_finalize_timing_ledger(EV,qPlanInternal,'SUCCESS');
end


function b3_print_dynamic_summary(CD,profile)
% Compact normal-path summary. Uses only CD/profile inputs; no caller-scope variables.
patches=0; checks=0; socp=0; split=0; depth=0; pass=0; totalMs=0;
if isstruct(CD)
    if isfield(CD,'pass'), pass=double(CD.pass); end
    if isfield(CD,'dynamic_socp_calls'), socp=CD.dynamic_socp_calls; end
    if isfield(CD,'split_node_count'), split=CD.split_node_count; end
    if isfield(CD,'max_subdivision_depth'), depth=CD.max_subdivision_depth; end
    if isfield(CD,'runtime_audit_summary') && isstruct(CD.runtime_audit_summary)
        S=CD.runtime_audit_summary;
        if isfield(S,'n_observed_patches'), patches=S.n_observed_patches; end
        if isfield(S,'n_checks'), checks=S.n_checks; end
    end
end
if isstruct(profile) && isfield(profile,'rigid_cable_certificate_ms')
    totalMs=profile.rigid_cable_certificate_ms;
end
fprintf(['    [DYNAMIC-SUMMARY] pass=%d | patches=%d | checks=%d | SOCP=%d ', ...
    '| split=%d | maxDepth=%d | total=%.3f ms\n'], ...
    pass,patches,checks,socp,split,depth,totalMs);
end



function b3_print_failure_event_detail(EV)
fprintf('    [FAIL-DETAIL] stage=%s reason=%s\n', ...
    b3_getfield_or_text(EV.profile,'ledger_stage','UNKNOWN'), ...
    b3_getfield_or_text(EV,'failure_reason','UNKNOWN'));

fprintf(['    [FAIL-TIMING] wall=%.3f | geometry=%.3f backend=%.3f dynamic=%.3f ', ...
    'yaw=%.3f unaccounted=%+.3f ms\n'], ...
    b3_getfield_or_zero(EV.profile,'internal_plan_wall_ms'), ...
    b3_getfield_or_zero(EV.profile,'geometry_ms'), ...
    b3_getfield_or_zero(EV.profile,'backend_build_solve_ms'), ...
    b3_getfield_or_zero(EV.profile,'rigid_cable_certificate_ms'), ...
    b3_getfield_or_zero(EV.profile,'yaw_ms'), ...
    b3_getfield_or_zero(EV.profile,'other_unaccounted_ms'));

if isfield(EV,'c6_hard_certified') && ~EV.c6_hard_certified
    fprintf('    [FAIL-C6] hard=%d structural=%d\n', ...
        EV.c6_hard_certified,b3_getfield_or_zero(EV,'c6_structural_certified'));
    if isfield(EV,'start_physical_splice_residual_d0_d6')
        fprintf('    [FAIL-C6] startResidual=%s\n',mat2str(EV.start_physical_splice_residual_d0_d6,9));
    end
    if isfield(EV,'start_physical_splice_guard_d0_d6')
        fprintf('    [FAIL-C6] startGuard=%s\n',mat2str(EV.start_physical_splice_guard_d0_d6,9));
    end
end

if isfield(EV,'cable_dynamic') && isstruct(EV.cable_dynamic) && ...
        isfield(EV.cable_dynamic,'runtime_audit_summary')
    b3_print_dynamic_runtime_audit_verbose(EV.cable_dynamic.runtime_audit_summary);
end

if isfield(EV,'cable_dynamic') && isstruct(EV.cable_dynamic) && ...
        isfield(EV.cable_dynamic,'limit_point_audit') && ...
        isstruct(EV.cable_dynamic.limit_point_audit) && ...
        ~isempty(fieldnames(EV.cable_dynamic.limit_point_audit))
    fprintf('    [FAIL-LIMIT] %s\n',b3_struct_one_line(EV.cable_dynamic.limit_point_audit));
end
end


function s=b3_getfield_or_text(S,name,default)
s=default;
if isstruct(S) && isfield(S,name)
    v=S.(name);
    if isstring(v) || ischar(v), s=char(v); end
end
end


function s=b3_struct_one_line(S)
fn=fieldnames(S);
parts=cell(1,numel(fn));
for k=1:numel(fn)
    v=S.(fn{k});
    if isnumeric(v) && isscalar(v)
        parts{k}=sprintf('%s=%g',fn{k},v);
    elseif islogical(v) && isscalar(v)
        parts{k}=sprintf('%s=%d',fn{k},v);
    elseif ischar(v) || (isstring(v) && isscalar(v))
        parts{k}=sprintf('%s=%s',fn{k},char(v));
    else
        parts{k}=sprintf('%s=[%s]',fn{k},class(v));
    end
end
s=strjoin(parts,' | ');
end



function EV=b3_finalize_timing_ledger(EV,qPlanInternal,stageLabel)
EV.profile.internal_plan_wall_ms=1e3*toc(qPlanInternal);

names={'memory_snapshot_ms','geometry_ms','jet_prepare_ms', ...
    'backend_build_solve_ms','kinematic_total_ms', ...
    'rigid_cable_certificate_ms','rigid_cable_dense_diag_ms', ...
    'sreenath_dense_diag_ms','candidate_pack_ms','yaw_ms', ...
    'logging_ms','failure_diagnostic_ms','finalize_ms'};

vals=zeros(size(names));
for k=1:numel(names)
    vals(k)=b3_getfield_or_zero(EV.profile,names{k});
end

EV.profile.accounted_top_level_ms=sum(vals);
EV.profile.other_unaccounted_ms=EV.profile.internal_plan_wall_ms-EV.profile.accounted_top_level_ms;
EV.profile.closure_error_ms=EV.profile.other_unaccounted_ms;
EV.profile.coverage_fraction=EV.profile.accounted_top_level_ms/max(EV.profile.internal_plan_wall_ms,eps);

EV.profile.measurement_only_ms= ...
    b3_getfield_or_zero(EV.profile,'kinematic_dense_diag_ms') + ...
    b3_getfield_or_zero(EV.profile,'rigid_cable_dense_diag_ms') + ...
    b3_getfield_or_zero(EV.profile,'sreenath_dense_diag_ms');

EV.profile.online_required_accounted_ms=max(0, ...
    EV.profile.accounted_top_level_ms-EV.profile.measurement_only_ms);

EV.profile.ledger_stage=stageLabel;
EV.profile.ledger_ready=true;

fprintf(['    [TIME-LEDGER] stage=%s total=%.6f accounted=%.6f ', ...
    'unaccounted=%+.6f ms coverage=%.3f%%\n'], ...
    stageLabel,EV.profile.internal_plan_wall_ms,EV.profile.accounted_top_level_ms, ...
    EV.profile.other_unaccounted_ms,100*EV.profile.coverage_fraction);

b3_detail_fprintf(['    [TIME-LEDGER-TOP] memory=%.6f geometry=%.6f jet=%.6f backend=%.6f ', ...
    'kinTotal=%.6f dynCert=%.6f denseCable=%.6f denseSreenath=%.6f ', ...
    'candPack=%.6f yaw=%.6f logging=%.6f failureDiag=%.6f finalize=%.6f ms\n'], ...
    b3_getfield_or_zero(EV.profile,'memory_snapshot_ms'), ...
    b3_getfield_or_zero(EV.profile,'geometry_ms'), ...
    b3_getfield_or_zero(EV.profile,'jet_prepare_ms'), ...
    b3_getfield_or_zero(EV.profile,'backend_build_solve_ms'), ...
    b3_getfield_or_zero(EV.profile,'kinematic_total_ms'), ...
    b3_getfield_or_zero(EV.profile,'rigid_cable_certificate_ms'), ...
    b3_getfield_or_zero(EV.profile,'rigid_cable_dense_diag_ms'), ...
    b3_getfield_or_zero(EV.profile,'sreenath_dense_diag_ms'), ...
    b3_getfield_or_zero(EV.profile,'candidate_pack_ms'), ...
    b3_getfield_or_zero(EV.profile,'yaw_ms'), ...
    b3_getfield_or_zero(EV.profile,'logging_ms'), ...
    b3_getfield_or_zero(EV.profile,'failure_diagnostic_ms'), ...
    b3_getfield_or_zero(EV.profile,'finalize_ms'));

b3_detail_fprintf(['    [TIME-LEDGER-GEOM] total=%.6f progress=%.6f direction=%.6f ', ...
    'seeds=%.6f corridor=%.6f residual=%.6f ms\n'], ...
    b3_getfield_or_zero(EV.profile,'geometry_ms'), ...
    b3_getfield_or_zero(EV.profile,'geometry_progress_ms'), ...
    b3_getfield_or_zero(EV.profile,'geometry_direction_ms'), ...
    b3_getfield_or_zero(EV.profile,'geometry_seed_ms'), ...
    b3_getfield_or_zero(EV.profile,'geometry_corridor_ms'), ...
    b3_getfield_or_zero(EV.profile,'geometry_unaccounted_ms'));

knownBackend= ...
    b3_getfield_or_zero(EV,'build_time_ms') + ...
    b3_getfield_or_zero(EV,'solve_time_ms') + ...
    b3_getfield_or_zero(EV.profile,'backend_post_reconstruct_ms') + ...
    b3_getfield_or_zero(EV.profile,'backend_post_slack_ms') + ...
    b3_getfield_or_zero(EV.profile,'backend_post_c6_audit_ms');
b3_detail_fprintf(['    [TIME-LEDGER-BACKEND] wrapper=%.6f build=%.6f solve=%.6f ', ...
    'reconstruct=%.6f slack/rowCert=%.6f C6audit=%.6f residualAfterKnown=%.6f ms\n'], ...
    b3_getfield_or_zero(EV.profile,'backend_build_solve_ms'), ...
    b3_getfield_or_zero(EV,'build_time_ms'), ...
    b3_getfield_or_zero(EV,'solve_time_ms'), ...
    b3_getfield_or_zero(EV.profile,'backend_post_reconstruct_ms'), ...
    b3_getfield_or_zero(EV.profile,'backend_post_slack_ms'), ...
    b3_getfield_or_zero(EV.profile,'backend_post_c6_audit_ms'), ...
    max(0,b3_getfield_or_zero(EV.profile,'backend_build_solve_ms')-knownBackend));

b3_detail_fprintf(['    [TIME-LEDGER-KIN] total=%.6f core=%.6f denseMeasurement=%.6f ms\n'], ...
    b3_getfield_or_zero(EV.profile,'kinematic_total_ms'), ...
    b3_getfield_or_zero(EV.profile,'kinematic_core_ms'), ...
    b3_getfield_or_zero(EV.profile,'kinematic_dense_diag_ms'));

b3_detail_fprintf(['    [TIME-LEDGER-DYN] total=%.6f broadphase=%.6f SOCP=%.6f analytic=%.6f reuse=%.6f ', ...
    'otherCertificate=%.6f ms\n'], ...
    b3_getfield_or_zero(EV.profile,'rigid_cable_certificate_ms'), ...
    b3_getfield_or_zero(EV.profile,'dynamic_broadphase_ms'), ...
    b3_getfield_or_zero(EV.profile,'dynamic_socp_ms'), ...
    b3_getfield_or_zero(EV.profile,'dynamic_analytic_ms'), ...
    b3_getfield_or_zero(EV.profile,'dynamic_reuse_ms'), ...
    b3_getfield_or_zero(EV.profile,'dynamic_non_socp_overhead_ms'));

b3_detail_fprintf(['    [TIME-LEDGER-CLASS] onlineAccounted=%.6f measurementOnly=%.6f ', ...
    'internalWall=%.6f ms\n'], ...
    EV.profile.online_required_accounted_ms,EV.profile.measurement_only_ms, ...
    EV.profile.internal_plan_wall_ms);

alarmThreshold=max(0.05*EV.profile.internal_plan_wall_ms,1.0);
if abs(EV.profile.other_unaccounted_ms)>alarmThreshold
    fprintf(['    [TIME-LEDGER-ALARM] unexplained time %.6f ms exceeds ', ...
        'max(5%% wall,1ms)=%.6f ms; optimize NOTHING until isolated.\n'], ...
        EV.profile.other_unaccounted_ms,alarmThreshold);
else
    b3_detail_fprintf('    [TIME-LEDGER-CLOSED] residual within accounting threshold.\n');
end
end


function v=b3_getfield_or_zero(S,name)
if isstruct(S) && isfield(S,name) && isscalar(S.(name)) && isfinite(S.(name))
    v=S.(name);
else
    v=0;
end
end


function v=b3_getfield_or_nan(S,name)
if isstruct(S) && isfield(S,name) && isscalar(S.(name)) && isfinite(S.(name))
    v=S.(name);
else
    v=NaN;
end
end


function [G,C,T,DIAG]=b3_build_replan_geometry(backend,t0,Dstart,obs,riskIds,CFG,P)
% Deterministic B3 geometry construction.
%
% IMPORTANT V6 FIX
% ----------------
% The previous implementation tried to impose
%
%   T = guide_length(T) / nominal_speed
%
% while simultaneously forcing the rejoin point to be the nominal point at
% absolute time t0+T. For any nonzero bypass offset d,
%
%   guide_length(T) > nominal_speed*T,
%
% so that fixed-point iteration cannot converge.
%
% B3 is a geometric/replanning lifecycle audit, not yet the final actuator-
% constrained automatic-time-allocation stage. Therefore:
%
%   1) choose T from the minimum progress-clear condition;
%   2) build the bypass guide to the nominal rejoin point at t0+T;
%   3) record the mean guide-speed ratio as a DIAGNOSTIC;
%   4) do NOT reject merely because the detour guide is longer than v_nom*T.
%
% Later time-allocation / actuator-feasibility stages must decide whether the
% required speed-up is physically admissible.

if isempty(obs)
    error('STEP4B3:NoKnownObstacle','Cannot build an avoidance plan with empty memory.');
end

riskMask=ismember([obs.id],riskIds);
if ~any(riskMask)
    error('STEP4B3:RiskMemoryMismatch','Risk IDs are absent from memory snapshot.');
end

W0=Dstart(:,1);
speed=CFG.nominal_speed;
e=CFG.e_nom(:);
e=e/norm(e);

% V20: spatial rejoin, not absolute-time catch-up.
% Safety/C6 are hard. Time optimality and v/a are advisory.
if any(arrayfun(@(z) norm(z.v_center(:))>0, obs(riskMask)))
    error('STEP4B3:MovingPatchSpatialRejoinNotProven', ...
        'V20 spatial-rejoin proof currently covers static observed patches only.');
end

qt=tic;
H=-Inf;
components={'PAYLOAD','UAV','CABLE'};
for io=find(riskMask)
    for ic=1:numel(components)
        H=max(H,b3_component_forbidden_support(obs(io),e,0,0,P,components{ic}));
    end
end
tProgressMs=1e3*toc(qt);

x0nom=e.'*CFG.p0;
xW0=e.'*W0;
xProv=max(H,xW0);
W3prov=CFG.p0 + e*(xProv-x0nom);

qt=tic;
Tprobe=max(CFG.dt_exec,norm(W3prov-W0)/speed);
[nBy,dreq,dirDiag]=b3_choose_bypass_direction(W0,W3prov,obs(riskMask),Tprobe,CFG,P);
tDirectionMs=1e3*toc(qt);
if ~(isfinite(dreq) && dreq>0)
    error('STEP4B3:NoPositiveBypass', ...
        'Future collision was predicted but no positive certified bypass offset was constructed.');
end

% Endpoint C6 seed clearance. Degree-7 is the dominant retained backend.
alpha=max(6/13,6/7);
xRejoin=max(H + alpha*dreq, xW0);
W3=CFG.p0 + e*(xRejoin-x0nom);

% Re-evaluate the transverse construction at the final spatial rejoin.
qt=tic;
Tprobe=max(CFG.dt_exec,norm(W3-W0)/speed);
[nBy,dreq,dirDiag]=b3_choose_bypass_direction(W0,W3,obs(riskMask),Tprobe,CFG,P);
tDirectionMs=tDirectionMs+1e3*toc(qt);
xRejoin=max(H + alpha*dreq, xW0);
W3=CFG.p0 + e*(xRejoin-x0nom);

W1=W0+dreq*nBy;
W2=W3+dreq*nBy;
W=[W0,W1,W2,W3];

L=vecnorm(diff(W,1,2),2,1);
if any(~isfinite(L)) || any(L<=0)
    error('STEP4B3:BadGuide','All three guide segments must have strictly positive length.');
end
pathLength=sum(L);
frac=L/pathLength;

% Natural detour duration: mean guide speed equals nominal speed exactly.
T=pathLength/speed;
if ~(isfinite(T) && T>0)
    error('STEP4B3:BadDuration','Spatial-rejoin duration is not positive finite.');
end
if t0+T>CFG.mission_tf+1e-10
    error('STEP4B3:NoMissionTime','Spatial-rejoin trajectory extends beyond validator mission horizon.');
end

requiredMeanGuideSpeed=pathLength/T;
requiredMeanSpeedRatio=requiredMeanGuideSpeed/speed;

v2=CFG.v_nom.'*CFG.v_nom;
if ~(v2>0), error('STEP4B3:BadNominalVelocity','Nominal velocity must be nonzero.'); end
rejoinPhase=(CFG.v_nom.'*(W3-CFG.p0))/v2;

DendForCorridor=zeros(3,7);
DendForCorridor(:,1)=W3;
DendForCorridor(:,2)=CFG.v_nom;

tau3=T*frac(3);
forwardGap=(e.'*W3 - alpha*speed*tau3)-H;
spatialGuard=512*eps(max([1,abs(H),abs(e.'*W3),abs(alpha*speed*tau3)]));
if forwardGap < -spatialGuard
    error('STEP4B3:SpatialRejoinAuditFailed', ...
        'Spatial C6 rejoin audit failed (gap=%+.16e).',forwardGap);
end
fprintf(['    [SPATIAL-REJOIN] H=%+.12e xRejoin=%+.12e d=%.12g alpha=%.12g ', ...
    'T=%.12g tau3=%.12g forwardGap=%+.12e phase=%.12g timeShiftAfter=%.12g\\n'], ...
    H,e.'*W3,dreq,alpha,T,tau3,forwardGap,rejoinPhase,(t0+T)-rejoinPhase);

qt=tic;
seedSets=b3_required_boundary_seed_sets(W,frac,T,Dstart,DendForCorridor,backend);
tSeedMs=1e3*toc(qt);
tcorr=tic;
[C,cdiag]=b3_build_time_cell_corridor_theory(backend,W,frac,T,Dstart,DendForCorridor,obs,seedSets,CFG,P,nBy);
cdiag.runtime_ms=1e3*toc(tcorr);
lastCorrMin=cdiag.min_guide_slack;
lastCorrGuideGuard=cdiag.guide_fp_guard;

% V32: GUIDE IS NOT A SAFETY AUTHORITY.
% The guide W is only a construction/rejoin scaffold.  After V28 local
% subdivision, the actual continuous-time safety authority is:
%   (i) local restricted Bezier / extracted-Bezier control-hull constraints,
%   (ii) component-wise PAYLOAD/UAV/CABLE supports, and
%   (iii) final direct certification of every original inequality.
% Therefore guide containment is logged but never used as a hard gate.
if ~isfinite(lastCorrMin)
    error('STEP4B3:GuideDiagnosticNonfinite', ...
        'Guide diagnostic became non-finite; certificate construction is invalid.');
end
if lastCorrMin < -lastCorrGuideGuard
    b3_detail_fprintf(['    [GUIDE-DIAG] minGuideSlack=%+.16e outside fpGuard=%+.16e ', ...
        '(diagnostic only; actual local curve hull remains hard authority)\n'], ...
        lastCorrMin,lastCorrGuideGuard);
elseif lastCorrMin<0
    fprintf('    [GUIDE-FP] minGuideSlack=%+.16e within fpGuard=%+.16e\n', ...
        lastCorrMin,lastCorrGuideGuard);
end

G=struct();
G.waypoints=W;
G.path_length=pathLength;
G.segment_length=L;
G.segment_fraction=frac;
G.bypass_direction=nBy;
G.bypass_offset=dreq;
G.risk_ids=riskIds;

DIAG=struct();
DIAG.duration_iterations=1;
DIAG.duration_policy='SPATIAL_REJOIN_PATH_LENGTH_OVER_NOMINAL_SPEED';
DIAG.corridor_seed_policy='BACKEND_SPECIFIC_MANDATORY_C6_BOUNDARY_SEEDS';
DIAG.direction=dirDiag;
DIAG.corridor=cdiag;
DIAG.min_guide_slack=lastCorrMin;
DIAG.safety_authority='COMPONENT_SUPPORT_CONSTRUCTION_WITNESS_THEN_SOCP_FALLBACK';
DIAG.vertical_cable_proxy=true;
DIAG.T_progress=NaN;
DIAG.spatial_rejoin_support=H;
DIAG.spatial_rejoin_forward_gap=forwardGap;
DIAG.rejoin_nominal_phase=rejoinPhase;
DIAG.T_path_at_nominal_speed=T;
DIAG.required_mean_guide_speed=requiredMeanGuideSpeed;
DIAG.required_mean_speed_ratio=requiredMeanSpeedRatio;
DIAG.backend=backend;
G.rejoin_nominal_phase=rejoinPhase;
G.end_physical_jet=DendForCorridor;
G.post_nominal_time_shift=(t0+T)-rejoinPhase;
DIAG.nominal_speed_is_diagnostic_only=true;
DIAG.profile=struct('progress_time_ms',tProgressMs,'direction_ms',tDirectionMs, ...
    'seed_build_ms',tSeedMs,'corridor_ms',cdiag.runtime_ms);
fprintf('    [PROFILE-GEOM] progressT=%.6f | direction=%.6f | seeds=%.6f | corridor=%.6f ms\n', ...
    tProgressMs,tDirectionMs,tSeedMs,cdiag.runtime_ms);
end

function [nBest,dBest,DIAG]=b3_choose_bypass_direction(W0,W3,riskObs,T,CFG,P)
e=CFG.e_nom;
[b1,b2]=aud_transverse_basis(e);
phi=linspace(0,2*pi,CFG.separator_direction_count+1);
phi(end)=[];
dBest=Inf;
nBest=[NaN;NaN;NaN];
allD=nan(size(phi));

for k=1:numel(phi)
    n=cos(phi(k))*b1+sin(phi(k))*b2;
    n=n/norm(n);
    d=0;
    for i=1:numel(riskObs)
        h=b3_swept_support(riskObs(i),n,0,T);
        req=h+b3_system_offset(n,P)-min(n.'*W0,n.'*W3);
        d=max(d,req);
    end
    d=max(0,d);
    allD(k)=d;
    if d<dBest
        dBest=d;
        nBest=n;
    end
end

DIAG=struct('phi',phi,'required_offset',allD,'chosen_direction',nBest,'chosen_offset',dBest);
end


function seedSets=b3_required_boundary_seed_sets(W,frac,T,Dstart,Dend,backend)
% V23 BACKEND-SPECIFIC mandatory boundary seeds.
%
% IMPORTANT:
% A corridor used by one backend must only contain the fixed boundary
% control points that can actually occur in THAT backend.
%
% Previous versions formed
%   conv(W, D13 boundary seeds, D7 boundary seeds)
% as one common seed hull.  No actual candidate trajectory contains both
% the D13 and D7 boundary-control-point sets simultaneously, so that common
% convex hull was a strict extra conservatism and could cut through an
% obstacle even when each backend's own fixed seed hull was separable.
%
% Safety is unchanged:
%   - every mandatory fixed boundary CP of the selected backend is included;
%   - every guide endpoint is included;
%   - subsequent QP constraints still force all actual backend CPs into the
%     certified halfspaces;
%   - C6 endpoint jets are unchanged.

J0s=b2_physical_to_spatial_jet(Dstart,T);
J1s=b2_physical_to_spatial_jet(Dend,T);

switch upper(backend)
    case {'BEZIER_D13','POLY_D13'}
        B13L=b2_endpoint_bezier_cp_from_jets(J0s,frac(1),13,'left');
        B13R=b2_endpoint_bezier_cp_from_jets(J1s,frac(3),13,'right');
        leftSeeds=B13L;
        rightSeeds=B13R;
        seedFamily='D13_ONLY';

    case 'BSPLINE_D7'
        % Current D7 architecture uses seven simple spans per corridor cell.
        % The boundary extracted-Bezier span width is therefore frac/7.
        B7L=b2_endpoint_bezier_cp_from_jets(J0s,frac(1)/7,7,'left');
        B7R=b2_endpoint_bezier_cp_from_jets(J1s,frac(3)/7,7,'right');
        leftSeeds=B7L;
        rightSeeds=B7R;
        seedFamily='D7_ONLY';

    otherwise
        error('STEP4B3:UnknownBackendSeedFamily', ...
            'Unsupported backend for mandatory corridor seeds: %s',backend);
end

seedSets=cell(1,3);
seedSets{1}=b3_unique_seed_columns([W(:,1),W(:,2),leftSeeds]);
seedSets{2}=b3_unique_seed_columns([W(:,2),W(:,3)]);
seedSets{3}=b3_unique_seed_columns([W(:,3),W(:,4),rightSeeds]);

b3_detail_fprintf(['    [BACKEND-CORRIDOR-SEEDS] backend=%s family=%s ', ...
    'counts=[%d %d %d]\n'], ...
    backend,seedFamily,size(seedSets{1},2),size(seedSets{2},2),size(seedSets{3},2));
end


function X=b3_unique_seed_columns(X)
% Deterministic exact-geometry cleanup only.  No tolerance-based geometry
% decision is made here; repeated columns merely enlarge the same convex hull.
if isempty(X), return; end
[~,ia]=unique(X.','rows','stable');
X=X(:,ia);
end


function [C,DIAG]=b3_build_time_cell_corridor_theory(backend,W,frac,T,Dstart,Dend,obs,seedSets,CFG,P,nBy)
% V28 ADAPTIVE LOCAL CONTINUOUS-TIME SAFETY CERTIFICATE
%
% V27 used one convex proof cell per guide segment.  If that complete seed
% hull could not be separated from one component forbidden set, planning
% stopped before the trajectory optimizer was allowed to exploit the actual
% curved path.
%
% V28 keeps the SAME safety condition and the SAME trajectory/C6 problem,
% but refines only the CERTIFICATE domain:
%
%   unsplit proof cell
%        |
%        +-- separable -> keep one leaf (V27 behavior)
%        |
%        +-- not separable -> split the parameter interval exactly and
%                            certify the children independently.
%
% Downstream, D13 uses exact de Casteljau restriction maps and BSPLINE_D7
% uses exact extracted-Bezier restriction maps.  Therefore every accepted
% local leaf constrains the ACTUAL local curve control hull, not merely a
% sampled trajectory.
%
% No margin reduction, obstacle shrinking, C6 relaxation, or tolerance-based
% acceptance is introduced.

edges=[0,cumsum(frac)];
edges(end)=1;

fixedCtx=b3_backend_fixed_local_seed_context(backend,frac,T,Dstart,Dend);

C=struct();
C.cells=cell(1,3);
meta=cell(1,3);

agg=b3_empty_corridor_diag_accumulator();
allMinSeed=Inf;
allMinGuide=Inf;
guideFpGuard=0;
maxDepthUsed=0;
nLeaves=0;

nBy=nBy(:);
if ~(all(isfinite(nBy)) && abs(norm(nBy)-1)<=1e-10)
    error('STEP4B3:BadConstructionNormal', ...
        'Bypass construction normal must be finite and unit length.');
end

for q=1:3
    taCell=T*edges(q);
    tbCell=T*edges(q+1);

    [leaves,Dq,fail]=b3_build_local_corridor_recursive( ...
        q,0,1,0,W(:,q),W(:,q+1),seedSets{q},fixedCtx, ...
        taCell,tbCell,obs,CFG,P,nBy);

    if ~isempty(fail)
        error('STEP4B3:NoComponentSafeSeparator', ...
            ['Cell %d local interval [%.6f %.6f] obstacle %d component %s: ', ...
             'adaptive exact local certificate failed at maxDepth=%d. ', ...
             'gap=%+.16e guard=%+.16e.'], ...
             q,fail.u0,fail.u1,fail.obs_id,fail.component, ...
             CFG.corridor_max_subdivision_depth,fail.gap,fail.guard);
    end

    C.cells{q}=struct('subcells',{leaves});
    meta{q}=leaves;
    agg=b3_add_corridor_diag(agg,Dq);
    nLeaves=nLeaves+numel(leaves);

    if ~isempty(leaves)
        maxDepthUsed=max(maxDepthUsed,max([leaves.depth]));
        allMinSeed=min(allMinSeed,min([leaves.min_seed_slack]));
        allMinGuide=min(allMinGuide,min([leaves.min_guide_slack]));
        guideFpGuard=max(guideFpGuard,max([leaves.guide_fp_guard]));
    end

    hsBefore=0; hsAfter=0;
    if ~isempty(leaves)
        hsBefore=sum([leaves.halfspace_rows_before]);
        hsAfter=sum([leaves.halfspace_rows_after]);
    end
    b3_detail_fprintf('    [LOCAL-CERT] cell=%d leaves=%d maxDepth=%d halfspaces=%d->%d\n', ...
        q,numel(leaves),isempty(leaves)*0 + (~isempty(leaves))*max([leaves.depth]), ...
        hsBefore,hsAfter);
end

DIAG=struct('min_guide_slack',allMinGuide, ...
    'guide_fp_guard',guideFpGuard, ...
    'min_seed_slack',allMinSeed, ...
    'min_separator_margin',agg.min_separator_distance, ...
    'min_separator_distance',agg.min_separator_distance, ...
    'cell_meta',{meta}, ...
    'time_fraction_edges',edges, ...
    'separator_method','ADAPTIVE_LOCAL_CONSTRUCTION_WITNESS_THEN_COMPONENT_SOCP', ...
    'system_geometry_model','UNION_PAYLOAD_UAV_CABLE_NOT_GLOBAL_CONVEX_HULL', ...
    'safe_set_semantics','ACTUAL_LOCAL_CURVE_HULL_COMPONENT_SUPPORT_CERTIFICATE', ...
    'component_count',3, ...
    'cheap_reject_count',agg.cheap_reject_count, ...
    'construction_witness_accept_count',agg.witness_accept_count, ...
    'analytic_witness_accept_count',agg.analytic_witness_accept_count, ...
    'analytic_witness_checks',agg.analytic_witness_checks, ...
    'socp_call_count',agg.socp_call_count, ...
    'cheap_reject_runtime_ms',agg.cheap_runtime_ms, ...
    'construction_witness_runtime_ms',agg.witness_runtime_ms, ...
    'analytic_witness_runtime_ms',agg.analytic_witness_runtime_ms, ...
    'distance_socp_runtime_ms',agg.socp_runtime_ms, ...
    'local_leaf_count',nLeaves, ...
    'local_max_depth_used',maxDepthUsed);

b3_detail_fprintf(['    [CERT-CASCADE] leaves=%d maxDepth=%d nByWitness=%d analyticWitness=%d ', ...
    'analyticChecks=%d | SOCPcalls=%d | witness/analytic/SOCP=%.6f/%.6f/%.6f ms\n'], ...
    nLeaves,maxDepthUsed,agg.witness_accept_count,agg.analytic_witness_accept_count, ...
    agg.analytic_witness_checks,agg.socp_call_count, ...
    agg.witness_runtime_ms,agg.analytic_witness_runtime_ms,agg.socp_runtime_ms);
end


function [leaves,D,fail]=b3_build_local_corridor_recursive( ...
    cellId,u0,u1,depth,W0,W1,baseSeeds,fixedCtx,taCell,tbCell,obs,CFG,P,nBy)

Seeds=b3_local_construction_seeds(W0,W1,baseSeeds,u0,u1,depth);
fixedSeeds=b3_exact_fixed_local_seeds(fixedCtx,cellId,u0,u1);
if ~isempty(fixedSeeds)
    Seeds=b3_unique_seed_columns([Seeds,fixedSeeds]);
end
ta=taCell+(tbCell-taCell)*u0;
tb=taCell+(tbCell-taCell)*u1;

[ok,leaf,Dtry,fail]=b3_try_local_corridor_leaf( ...
    cellId,u0,u1,depth,W0,W1,Seeds,ta,tb,obs,P,nBy);

if ok
    leaves=leaf;
    D=Dtry;
    fail=[];
    return;
end

if depth>=CFG.corridor_max_subdivision_depth
    leaves=repmat(b3_empty_local_leaf(),1,0);
    D=Dtry;
    return;
end

um=0.5*(u0+u1);
[L,DL,failL]=b3_build_local_corridor_recursive( ...
    cellId,u0,um,depth+1,W0,W1,baseSeeds,fixedCtx,taCell,tbCell,obs,CFG,P,nBy);
if ~isempty(failL)
    leaves=repmat(b3_empty_local_leaf(),1,0);
    D=b3_add_corridor_diag(Dtry,DL);
    fail=failL;
    return;
end

[R,DR,failR]=b3_build_local_corridor_recursive( ...
    cellId,um,u1,depth+1,W0,W1,baseSeeds,fixedCtx,taCell,tbCell,obs,CFG,P,nBy);
if ~isempty(failR)
    leaves=repmat(b3_empty_local_leaf(),1,0);
    D=b3_add_corridor_diag(b3_add_corridor_diag(Dtry,DL),DR);
    fail=failR;
    return;
end

leaves=[L,R];
D=b3_add_corridor_diag(b3_add_corridor_diag(Dtry,DL),DR);
fail=[];
end



function F=b3_backend_fixed_local_seed_context(backend,frac,T,Dstart,Dend)
F=struct('backend',upper(backend),'degree',NaN, ...
    'knownMask',{cell(1,3)},'knownCP',{cell(1,3)});
for q=1:3
    F.knownMask{q}=false(1,14);
    F.knownCP{q}=nan(3,14);
end
switch upper(backend)
    case {'BEZIER_D13','POLY_D13'}
        F.degree=13;
        J0s=b2_physical_to_spatial_jet(Dstart,T);
        J1s=b2_physical_to_spatial_jet(Dend,T);
        BL=b2_endpoint_bezier_cp_from_jets(J0s,frac(1),13,'left');
        BR=b2_endpoint_bezier_cp_from_jets(J1s,frac(3),13,'right');
        F.knownMask{1}(1:7)=true;
        F.knownCP{1}(:,1:7)=BL;
        F.knownMask{3}(8:14)=true;
        F.knownCP{3}(:,8:14)=fliplr(BR);
    case 'BSPLINE_D7'
        F.degree=7;
    otherwise
        error('STEP4B3:UnknownBackendFixedSeedContext','Unsupported backend %s.',backend);
end
end

function S=b3_exact_fixed_local_seeds(F,cellId,u0,u1)
S=zeros(3,0);
if ~ismember(F.backend,{'BEZIER_D13','POLY_D13'}), return; end
mask=F.knownMask{cellId};
if ~any(mask), return; end
P=F.knownCP{cellId};
R=b3_bezier_restrict_map(F.degree,u0,u1);
free=~mask;
zeroTol=64*eps(max(1,max(abs(R(:)))));
for j=1:size(R,1)
    if any(abs(R(j,free))>zeroTol), continue; end
    S(:,end+1)=P(:,mask)*R(j,mask).'; %#ok<AGROW>
end
if ~isempty(S), S=b3_unique_seed_columns(S); end
end


function Seeds=b3_local_construction_seeds(W0,W1,baseSeeds,u0,u1,depth)
% Construction seeds choose the side of each local free-space corridor.
% They are NOT the final safety authority; downstream exact local Bezier
% control-hull constraints are.
P0=(1-u0)*W0+u0*W1;
P1=(1-u1)*W0+u1*W1;

if depth==0
    Seeds=baseSeeds;
    return;
end

d=W1-W0;
dd=d.'*d;
Seeds=[P0,P1];
if dd>0 && ~isempty(baseSeeds)
    tau=(d.'*(baseSeeds-W0))/dd;
    guard=256*eps(max([1;abs(tau(:));abs([u0;u1])]));
    keep=tau>=u0-guard & tau<=u1+guard;
    Seeds=[Seeds,baseSeeds(:,keep)]; %#ok<AGROW>
end
Seeds=b3_unique_seed_columns(Seeds);
end


function [ok,leaf,D,fail]=b3_try_local_corridor_leaf( ...
    cellId,u0,u1,depth,W0,W1,Seeds,ta,tb,obs,P,nBy)

components={'PAYLOAD','UAV','CABLE'};
A=zeros(0,3);
b=zeros(0,1);
M=repmat(struct('obs_id',0,'component','','normal',zeros(3,1),'b',NaN, ...
    'guide_slack',NaN,'seed_slack',NaN,'separator_distance',NaN, ...
    'separator_exitflag',NaN,'separator_source','', ...
    'ta',NaN,'tb',NaN,'cheap_raw_inside',false, ...
    'distance_runtime_ms',NaN,'witness_runtime_ms',NaN, ...
    'support_certified',false,'support_guard',NaN),1,0);

D=b3_empty_corridor_diag_accumulator();
fail=[];
minSlack=Inf;
minGuideSlack=Inf;
maxGuideFpGuard=0;

P0=(1-u0)*W0+u0*W1;
P1=(1-u1)*W0+u1*W1;
Guide=[P0,P1];

for io=1:numel(obs)
    tc=tic;
    rawInside=b3_any_seed_inside_raw_swept_ellipsoid(Seeds,obs(io),ta,tb);
    D.cheap_runtime_ms=D.cheap_runtime_ms+1e3*toc(tc);
    if rawInside
        D.cheap_reject_count=D.cheap_reject_count+1;
        fail=struct('cell',cellId,'u0',u0,'u1',u1,'obs_id',obs(io).id, ...
            'component','RAW_SEED','gap',-Inf,'guard',0);
        ok=false;
        leaf=b3_empty_local_leaf();
        return;
    end

    for ic=1:numel(components)
        comp=components{ic};
        scale=max([1;abs(Seeds(:));abs(obs(io).center(:));obs(io).radii(:)]);
        fpTol=256*eps(scale);

        tw=tic;
        n=nBy;
        bb=b3_component_forbidden_support(obs(io),n,ta,tb,P,comp);
        seedSlack=min(n.'*Seeds-bb);
        guideVals=n.'*Guide-bb;
        guideSlack=min(guideVals);
        guideScale=max([1;abs(n.'*Guide(:,:)).';abs(bb)]);
        guideGuard=512*eps(max(guideScale));
        minGuideSlack=min(minGuideSlack,guideSlack);
        maxGuideFpGuard=max(maxGuideFpGuard,guideGuard);
        witnessMs=1e3*toc(tw);
        D.witness_runtime_ms=D.witness_runtime_ms+witnessMs;

        witnessGuard=fpTol;
        witnessCertified=isfinite(seedSlack) && seedSlack>=-witnessGuard;

        b3_detail_fprintf(['      [WITNESS-CERT] cell=%d local=[%.4f %.4f] obs=%d comp=%-7s ', ...
            'seedSlack=%+.16e guard=%+.16e certified=%d time=%.6f ms\n'], ...
            cellId,u0,u1,obs(io).id,comp,seedSlack,witnessGuard,witnessCertified,witnessMs);

        if witnessCertified
            D.witness_accept_count=D.witness_accept_count+1;
            A(end+1,:)=n.'; %#ok<AGROW>
            b(end+1,1)=bb; %#ok<AGROW>
            M(end+1)=struct('obs_id',obs(io).id,'component',comp, ...
                'normal',n,'b',bb,'guide_slack',guideSlack, ...
                'seed_slack',seedSlack,'separator_distance',NaN, ...
                'separator_exitflag',NaN,'separator_source','CONSTRUCTION_WITNESS', ...
                'ta',ta,'tb',tb,'cheap_raw_inside',false, ...
                'distance_runtime_ms',0,'witness_runtime_ms',witnessMs, ...
                'support_certified',true,'support_guard',witnessGuard); %#ok<AGROW>
            minSlack=min(minSlack,seedSlack);
            continue;
        end

        [aPass,aN,aB,aSlack,aGuard,aIdx,aMs,aTried]= ...
            b3_try_analytic_corridor_witnesses(Seeds,obs(io),ta,tb,P,comp);
        D.analytic_witness_checks=D.analytic_witness_checks+aTried;
        D.analytic_witness_runtime_ms=D.analytic_witness_runtime_ms+aMs;

        if aPass
            D.analytic_witness_accept_count=D.analytic_witness_accept_count+1;
            A(end+1,:)=aN.'; %#ok<AGROW>
            b(end+1,1)=aB; %#ok<AGROW>
            M(end+1)=struct('obs_id',obs(io).id,'component',comp, ...
                'normal',aN,'b',aB,'guide_slack',guideSlack, ...
                'seed_slack',aSlack,'separator_distance',NaN, ...
                'separator_exitflag',NaN,'separator_source', ...
                sprintf('ANALYTIC_WITNESS_%d',aIdx), ...
                'ta',ta,'tb',tb,'cheap_raw_inside',false, ...
                'distance_runtime_ms',0,'witness_runtime_ms',aMs, ...
                'support_certified',true,'support_guard',aGuard); %#ok<AGROW>
            minSlack=min(minSlack,aSlack);
            continue;
        end

        D.socp_call_count=D.socp_call_count+1;
        td=tic;
        [n,bb,dist,ef,distDiag]=b3_primal_component_distance_separator( ...
            obs(io),Seeds,ta,tb,P,comp);
        dtms=1e3*toc(td);
        D.socp_runtime_ms=D.socp_runtime_ms+dtms;

        if ~(all(isfinite(n)) && isfinite(bb))
            fail=struct('cell',cellId,'u0',u0,'u1',u1,'obs_id',obs(io).id, ...
                'component',comp,'gap',-Inf,'guard',fpTol);
            ok=false;
            leaf=b3_empty_local_leaf();
            return;
        end

        seedSlack=min(n.'*Seeds-bb);
        guideVals=n.'*Guide-bb;
        guideSlack=min(guideVals);
        guideScale=max([1;abs(n.'*Guide(:,:)).';abs(bb)]);
        guideGuard=512*eps(max(guideScale));
        minGuideSlack=min(minGuideSlack,guideSlack);
        maxGuideFpGuard=max(maxGuideFpGuard,guideGuard);
        numericalGuard=max(fpTol, ...
            16*max(0,distDiag.primalfeasibility)*max(1,norm(n)));
        certified=isfinite(seedSlack) && seedSlack>=-numericalGuard;

        b3_detail_fprintf(['      [COMP-CERT] cell=%d local=[%.4f %.4f] obs=%d comp=%-7s ', ...
            'source=SOCP exit=%g seedSlack=%+.16e guard=%+.16e certified=%d\n'], ...
            cellId,u0,u1,obs(io).id,comp,ef,seedSlack,numericalGuard,certified);

        if ~certified
            fail=struct('cell',cellId,'u0',u0,'u1',u1,'obs_id',obs(io).id, ...
                'component',comp,'gap',seedSlack,'guard',numericalGuard);
            ok=false;
            leaf=b3_empty_local_leaf();
            return;
        end

        A(end+1,:)=n.'; %#ok<AGROW>
        b(end+1,1)=bb; %#ok<AGROW>
        M(end+1)=struct('obs_id',obs(io).id,'component',comp, ...
            'normal',n,'b',bb,'guide_slack',guideSlack, ...
            'seed_slack',seedSlack,'separator_distance',dist, ...
            'separator_exitflag',ef,'separator_source','SOCP_FALLBACK', ...
            'ta',ta,'tb',tb,'cheap_raw_inside',false, ...
            'distance_runtime_ms',dtms,'witness_runtime_ms',witnessMs, ...
            'support_certified',true,'support_guard',numericalGuard); %#ok<AGROW>

        minSlack=min(minSlack,seedSlack);
        if isfinite(dist)
            D.min_separator_distance=min(D.min_separator_distance,dist);
        end
    end
end

[A,b,M,redDiag]=b3_exact_duplicate_halfspace_reduction(A,b,M);

leaf=b3_empty_local_leaf();
leaf.u0=u0;
leaf.u1=u1;
leaf.depth=depth;
leaf.A=A;
leaf.b=b;
leaf.meta=M;
leaf.ta=ta;
leaf.tb=tb;
leaf.min_seed_slack=minSlack;
leaf.min_guide_slack=minGuideSlack;
leaf.guide_fp_guard=maxGuideFpGuard;
leaf.halfspace_rows_before=redDiag.rows_before;
leaf.halfspace_rows_after=redDiag.rows_after;
ok=true;
end



function [Ar,br,Mr,D]=b3_exact_duplicate_halfspace_reduction(A,b,M)
% EXACT redundancy reduction for halfspaces n'*x >= b.
%
% Only bitwise-identical normal rows are grouped.  If several constraints
% share exactly the same normal n, their conjunction is exactly equivalent to
%
%       n'*x >= max_i b_i.
%
% No tolerance, angular clustering, obstacle shrinking, or safety relaxation
% is used.
D=struct('rows_before',size(A,1),'rows_after',size(A,1),'removed',0);

if isempty(A)
    Ar=A; br=b; Mr=M;
    return;
end

[Au,~,grp]=unique(A,'rows','stable');
ng=size(Au,1);

br=zeros(ng,1);
Mr=repmat(M(1),1,ng);

for g=1:ng
    ids=find(grp==g);
    [bg,k]=max(b(ids));
    keep=ids(k);
    br(g)=bg;
    Mr(g)=M(keep);
end

Ar=Au;
D.rows_after=ng;
D.removed=D.rows_before-D.rows_after;
end


function L=b3_empty_local_leaf()
L=struct('u0',NaN,'u1',NaN,'depth',NaN,'A',zeros(0,3),'b',zeros(0,1), ...
    'meta',[],'ta',NaN,'tb',NaN,'min_seed_slack',Inf, ...
    'min_guide_slack',Inf,'guide_fp_guard',0, ...
    'halfspace_rows_before',0,'halfspace_rows_after',0);
end


function D=b3_empty_corridor_diag_accumulator()
D=struct('cheap_reject_count',0,'witness_accept_count',0,'analytic_witness_accept_count',0, ...
    'analytic_witness_checks',0,'socp_call_count',0, ...
    'cheap_runtime_ms',0,'witness_runtime_ms',0,'analytic_witness_runtime_ms',0, ...
    'socp_runtime_ms',0, ...
    'min_separator_distance',Inf);
end


function A=b3_add_corridor_diag(A,B)
A.cheap_reject_count=A.cheap_reject_count+B.cheap_reject_count;
A.witness_accept_count=A.witness_accept_count+B.witness_accept_count;
A.analytic_witness_accept_count=A.analytic_witness_accept_count+B.analytic_witness_accept_count;
A.analytic_witness_checks=A.analytic_witness_checks+B.analytic_witness_checks;
A.socp_call_count=A.socp_call_count+B.socp_call_count;
A.cheap_runtime_ms=A.cheap_runtime_ms+B.cheap_runtime_ms;
A.witness_runtime_ms=A.witness_runtime_ms+B.witness_runtime_ms;
A.analytic_witness_runtime_ms=A.analytic_witness_runtime_ms+B.analytic_witness_runtime_ms;
A.socp_runtime_ms=A.socp_runtime_ms+B.socp_runtime_ms;
A.min_separator_distance=min(A.min_separator_distance,B.min_separator_distance);
end


function tf=b3_any_seed_inside_raw_swept_ellipsoid(Seeds,ob,ta,tb)
% Rigorous cheap rejection for the RAW swept ellipsoid only.
%
% For constant translational velocity and fixed S, minimize
%   phi(tau)=(s-c-tau*v)' inv(S) (s-c-tau*v)
% over tau in [ta,tb].  If min phi <= 1, the mandatory seed lies in the raw
% obstacle at some time in the cell, hence the full inflated-system problem
% is certainly infeasible for that seed.
Sinv=pinv(0.5*(ob.S+ob.S.'));
v=ob.v_center(:);
tf=false;

for j=1:size(Seeds,2)
    d=Seeds(:,j)-ob.center(:);
    av=v.'*Sinv*v;
    if av>0
        tau=(v.'*Sinv*d)/av;
        tau=min(max(tau,ta),tb);
    else
        tau=ta;
    end
    r=d-tau*v;
    phi=r.'*Sinv*r;
    if phi<=1
        tf=true;
        return;
    end
end
end


function [nBest,bBest,dBest,exitflag,D]=b3_primal_component_distance_separator(ob,Seeds,ta,tb,P,component)
% Exact primal distance between the seed hull and ONE component forbidden set.
%
% Forbidden set in payload-center coordinates:
%
% PAYLOAD:
%   F_L = O_swept (+) B(0,R_L)
%
% UAV, with p_Q = p_L + L e3:
%   F_Q = O_swept (+) (-L e3 + B(0,R_Q))
%
% CABLE:
%   F_C = O_swept (+) ({-lambda L e3 | 0<=lambda<=1} (+) B(0,R_C))
%
% No convex hull between PAYLOAD/UAV/CABLE is introduced.

component=upper(component);
Rp=P.R_payload+P.d_margin;
Ru=P.R_uav+P.d_margin;
Rc=P.R_cable+P.d_margin;
L=P.L_cable;

switch component
    case 'PAYLOAD'
        radius=Rp;
        fixedOffset=zeros(3,1);
        hasCableAlpha=false;
    case 'UAV'
        radius=Ru;
        fixedOffset=[0;0;-L];
        hasCableAlpha=false;
    case 'CABLE'
        radius=Rc;
        fixedOffset=zeros(3,1);
        hasCableAlpha=true;
    otherwise
        error('STEP4B3:UnknownComponent','Unknown component %s.',component);
end

m=size(Seeds,2);
c0=ob.center(:)+ta*ob.v_center(:);
obsDelta=(tb-ta)*ob.v_center(:);
moving=(norm(obsDelta)>0);

% Decision indices.
k=0;
iLamS=(k+1):(k+m); k=k+m;
if moving
    iObsAlpha=k+1; k=k+1;
else
    iObsAlpha=[];
end
iU=(k+1):(k+3); k=k+3;           % obstacle ellipsoid unit-ball variable
iW=(k+1):(k+3); k=k+3;           % component ball unit-ball variable
if hasCableAlpha
    iCableAlpha=k+1; k=k+1;       % segment parameter in forbidden shift
else
    iCableAlpha=[];
end
iT=k+1; k=k+1;
nx=k;

f=zeros(nx,1); f(iT)=1;
A=[]; b=[];
Aeq=zeros(1,nx); Aeq(1,iLamS)=1; beq=1;
lb=-inf(nx,1); ub=inf(nx,1);
lb(iLamS)=0;
if moving
    lb(iObsAlpha)=0; ub(iObsAlpha)=1;
end
if hasCableAlpha
    lb(iCableAlpha)=0; ub(iCableAlpha)=1;
end
lb(iT)=0;

% Obstacle ellipsoid square-root.
S=0.5*(ob.S+ob.S.');
[Ls,pchol]=chol(S,'lower');
if pchol~=0
    [V,E]=eig(S);
    E=max(E,0);
    Ls=V*sqrt(E);
end

% Distance cone:
% pS - [c0 + obsAlpha*obsDelta + Ls*u + fixedOffset
%       - cableAlpha*L*e3 + radius*w]
Dmat=zeros(3,nx);
Dmat(:,iLamS)=Seeds;
if moving, Dmat(:,iObsAlpha)=-obsDelta; end
Dmat(:,iU)=-Ls;
Dmat(:,iW)=-radius*eye(3);
if hasCableAlpha
    % Forbidden cable shift is -lambda L e3, therefore subtracting pF gives
    % +lambda L e3 in pS-pF.
    Dmat(:,iCableAlpha)=[0;0;L];
end
dsoc=zeros(nx,1); dsoc(iT)=1;
soc(1)=secondordercone(Dmat,c0+fixedOffset,dsoc,0);

% ||u|| <= 1
Au=zeros(3,nx); Au(:,iU)=eye(3);
soc(2)=secondordercone(Au,zeros(3,1),zeros(nx,1),-1);

% ||w|| <= 1
Aw=zeros(3,nx); Aw(:,iW)=eye(3);
soc(3)=secondordercone(Aw,zeros(3,1),zeros(nx,1),-1);

solverList={'normal-dense','augmented'};
attempts=repmat(struct('linear_solver','','runtime_ms',NaN,'exitflag',NaN, ...
    'primalfeasibility',NaN,'dualfeasibility',NaN,'dualitygap',NaN, ...
    'iterations',NaN),1,0);

x=[]; exitflag=NaN; out=struct();
for ia=1:numel(solverList)
    opts=optimoptions('coneprog','Display','none', ...
        'ConstraintTolerance',1e-8,'OptimalityTolerance',1e-8, ...
        'MaxIterations',200,'LinearSolver',solverList{ia});
    qs=tic;
    [xt,~,eft,outt]=coneprog(f,soc,A,b,Aeq,beq,lb,ub,opts);
    rt=1e3*toc(qs);

    at=struct('linear_solver',solverList{ia},'runtime_ms',rt,'exitflag',eft, ...
        'primalfeasibility',NaN,'dualfeasibility',NaN,'dualitygap',NaN, ...
        'iterations',NaN);
    if isstruct(outt)
        if isfield(outt,'primalfeasibility'), at.primalfeasibility=outt.primalfeasibility; end
        if isfield(outt,'dualfeasibility'), at.dualfeasibility=outt.dualfeasibility; end
        if isfield(outt,'dualitygap'), at.dualitygap=outt.dualitygap; end
        if isfield(outt,'iterations'), at.iterations=outt.iterations; end
    end
    attempts(end+1)=at; %#ok<AGROW>

    b3_detail_fprintf(['      [COMP-SOCP] comp=%-7s solver=%s exit=%g iter=%g ', ...
        'time=%.6f ms primal=%+.3e dual=%+.3e gap=%+.3e vars=%d cones=%d\\n'], ...
        component,solverList{ia},eft,at.iterations,rt, ...
        at.primalfeasibility,at.dualfeasibility,at.dualitygap,nx,numel(soc));

    x=xt; exitflag=eft; out=outt;
    if eft>0, break; end
    if ~ismember(eft,[0,-7,-10]), break; end
end

D=struct('primalfeasibility',NaN,'dualfeasibility',NaN,'dualitygap',NaN, ...
    'iterations',NaN,'linearsolver','','attempts',attempts, ...
    'nvars',nx,'ncones',numel(soc),'component',component);
if isstruct(out)
    if isfield(out,'primalfeasibility'), D.primalfeasibility=out.primalfeasibility; end
    if isfield(out,'dualfeasibility'), D.dualfeasibility=out.dualfeasibility; end
    if isfield(out,'dualitygap'), D.dualitygap=out.dualitygap; end
    if isfield(out,'iterations'), D.iterations=out.iterations; end
    if isfield(out,'linearsolver'), D.linearsolver=out.linearsolver; end
end

if isempty(x)
    nBest=[NaN;NaN;NaN]; bBest=NaN; dBest=NaN;
    return;
end

pS=Seeds*x(iLamS);
pF=c0+fixedOffset+Ls*x(iU)+radius*x(iW);
if moving, pF=pF+x(iObsAlpha)*obsDelta; end
if hasCableAlpha, pF=pF-x(iCableAlpha)*L*[0;0;1]; end

deltaPF=pS-pF;
dEuclid=norm(deltaPF);
if ~(isfinite(dEuclid) && dEuclid>0)
    deltaPF=mean(Seeds,2)-(c0+fixedOffset);
    dEuclid=norm(deltaPF);
end
if ~(isfinite(dEuclid) && dEuclid>0)
    nBest=[NaN;NaN;NaN]; bBest=NaN; dBest=NaN;
    return;
end

nBest=deltaPF/dEuclid;
bBest=b3_component_forbidden_support(ob,nBest,ta,tb,P,component);
dBest=min(nBest.'*Seeds)-bBest;
D.euclidean_distance=dEuclid;
D.support_gap=dBest;
end


function h=b3_component_forbidden_support(ob,n,ta,tb,P,component)
% Exact support of ONE forbidden set F_k in payload-center coordinates.
n=n(:);
base=b3_swept_support(ob,n,ta,tb);
Rp=P.R_payload+P.d_margin;
Ru=P.R_uav+P.d_margin;
Rc=P.R_cable+P.d_margin;
L=P.L_cable;

switch upper(component)
    case 'PAYLOAD'
        offset=Rp*norm(n);
    case 'UAV'
        offset=-L*n(3)+Ru*norm(n);
    case 'CABLE'
        offset=max(0,-L*n(3))+Rc*norm(n);
    otherwise
        error('STEP4B3:UnknownComponent','Unknown component %s.',component);
end
h=base+offset;
end


function b3_log_component_diagnostics(cellId,ob,Seeds,ta,tb,P,component,n,bb)
% Diagnostic decomposition of a failed component certificate.
base=b3_swept_support(ob,n,ta,tb);
total=b3_component_forbidden_support(ob,n,ta,tb,P,component);
seedProj=min(n.'*Seeds);
b3_detail_fprintf(['      [COMP-DIAG] cell=%d obs=%d comp=%s seedProj=%+.12e ', ...
    'obstacleSupport=%+.12e componentOffset=%+.12e totalSupport=%+.12e gap=%+.12e\\n'], ...
    cellId,ob.id,component,seedProj,base,total-base,total,seedProj-bb);
b3_log_seed_ellipsoid_diagnostics(cellId,ob,Seeds,ta,tb,P);
end


function [nBest,bBest,dBest,exitflag,D]=b3_primal_convex_distance_separator(ob,Seeds,ta,tb,P)
% V12 exact primal SOCP with algebraic redundancy removed.
%
% System proxy support:
% max(Rp||n||, Ru||n||-L*nz, Rc||n||, Rc||n||-L*nz)
% =
% max(R0||n||, RL||n||-L*nz),
% R0=max(Rp,Rc), RL=max(Ru,Rc).
%
% Hence four balls reduce EXACTLY to the convex hull of two balls.
%
% Obstacle translation is represented as
% c(ta)+alpha*(tb-ta)*v, alpha in [0,1].
% For static obstacles delta=(tb-ta)*v=0, alpha is omitted entirely, removing
% a null decision direction.

m=size(Seeds,2);
Rp=P.R_payload+P.d_margin;
Ru=P.R_uav+P.d_margin;
Rc=P.R_cable+P.d_margin;
L=P.L_cable;
R0=max(Rp,Rc);
RL=max(Ru,Rc);
centers=[zeros(3,1),[0;0;-L]];
radii=[R0,RL];

c0=ob.center(:)+ta*ob.v_center(:);
delta=(tb-ta)*ob.v_center(:);
moving=(norm(delta)>0);

% indices
k=0;
iLamS=(k+1):(k+m); k=k+m;
if moving
    iAlpha=k+1; k=k+1;
else
    iAlpha=[];
end
iU=(k+1):(k+3); k=k+3;
iLamK=(k+1):(k+2); k=k+2;
iW=cell(1,2);
for j=1:2
    iW{j}=(k+1):(k+3); k=k+3;
end
iT=k+1; k=k+1;
nx=k;

f=zeros(nx,1); f(iT)=1;
A=[]; b=[];
Aeq=zeros(2,nx); beq=[1;1];
Aeq(1,iLamS)=1;
Aeq(2,iLamK)=1;

lb=-inf(nx,1); ub=inf(nx,1);
lb(iLamS)=0;
lb(iLamK)=0;
if moving
    lb(iAlpha)=0; ub(iAlpha)=1;
end
lb(iT)=0;

S=0.5*(ob.S+ob.S.');
[Ls,pchol]=chol(S,'lower');
if pchol~=0
    [V,E]=eig(S);
    E=max(E,0);
    Ls=V*sqrt(E);
end

% ||pS-pO|| <= t
Dmat=zeros(3,nx);
Dmat(:,iLamS)=Seeds;
if moving, Dmat(:,iAlpha)=-delta; end
Dmat(:,iU)=-Ls;
for j=1:2
    Dmat(:,iLamK(j))=Dmat(:,iLamK(j))-centers(:,j);
    Dmat(:,iW{j})=Dmat(:,iW{j})-radii(j)*eye(3);
end
dsoc=zeros(nx,1); dsoc(iT)=1;
soc(1)=secondordercone(Dmat,c0,dsoc,0);

% ||u||<=1
Au=zeros(3,nx); Au(:,iU)=eye(3);
soc(2)=secondordercone(Au,zeros(3,1),zeros(nx,1),-1);

% ||wj||<=lambda_j
for j=1:2
    Aw=zeros(3,nx); Aw(:,iW{j})=eye(3);
    dw=zeros(nx,1); dw(iLamK(j))=1;
    soc(2+j)=secondordercone(Aw,zeros(3,1),dw,0);
end

% Same convex program, two numerical linear solvers at most.
solverList={'normal-dense','augmented'};
attempts=repmat(struct('linear_solver','','runtime_ms',NaN,'exitflag',NaN, ...
    'primalfeasibility',NaN,'dualfeasibility',NaN,'dualitygap',NaN,'iterations',NaN),1,0);

x=[];
exitflag=NaN;
out=struct();
for ia=1:numel(solverList)
    opts=optimoptions('coneprog','Display','none', ...
        'ConstraintTolerance',1e-8,'OptimalityTolerance',1e-8, ...
        'MaxIterations',200,'LinearSolver',solverList{ia});
    qs=tic;
    [xt,~,eft,outt]=coneprog(f,soc,A,b,Aeq,beq,lb,ub,opts);
    rt=1e3*toc(qs);

    at=struct('linear_solver',solverList{ia},'runtime_ms',rt,'exitflag',eft, ...
        'primalfeasibility',NaN,'dualfeasibility',NaN,'dualitygap',NaN,'iterations',NaN);
    if isstruct(outt)
        if isfield(outt,'primalfeasibility'), at.primalfeasibility=outt.primalfeasibility; end
        if isfield(outt,'dualfeasibility'), at.dualfeasibility=outt.dualfeasibility; end
        if isfield(outt,'dualitygap'), at.dualitygap=outt.dualitygap; end
        if isfield(outt,'iterations'), at.iterations=outt.iterations; end
    end
    attempts(end+1)=at; %#ok<AGROW>
    fprintf('      [SOCP] solver=%s exit=%g iter=%g time=%.6f ms primal=%+.3e dual=%+.3e gap=%+.3e vars=%d cones=%d moving=%d\n', ...
        solverList{ia},eft,at.iterations,rt,at.primalfeasibility,at.dualfeasibility,at.dualitygap,nx,numel(soc),moving);

    x=xt; exitflag=eft; out=outt;
    if eft>0
        break;
    end
    if ~ismember(eft,[0,-7,-10])
        break;
    end
end

D=struct('primalfeasibility',NaN,'dualfeasibility',NaN,'dualitygap',NaN, ...
    'iterations',NaN,'linearsolver','','attempts',attempts,'nvars',nx,'ncones',numel(soc));
if isstruct(out)
    if isfield(out,'primalfeasibility'), D.primalfeasibility=out.primalfeasibility; end
    if isfield(out,'dualfeasibility'), D.dualfeasibility=out.dualfeasibility; end
    if isfield(out,'dualitygap'), D.dualitygap=out.dualitygap; end
    if isfield(out,'iterations'), D.iterations=out.iterations; end
    if isfield(out,'linearsolver'), D.linearsolver=out.linearsolver; end
end

if isempty(x)
    nBest=[NaN;NaN;NaN]; bBest=NaN; dBest=NaN;
    return;
end

% V15: even when coneprog stops with -7, retain its finite primal iterate as
% a candidate direction.  Safety is NOT inferred from this iterate; it is
% independently checked afterward using the exact support-function gap.
pS=Seeds*x(iLamS);
pO=c0+Ls*x(iU);
if moving, pO=pO+x(iAlpha)*delta; end
for j=1:2
    pO=pO+x(iLamK(j))*centers(:,j)+radii(j)*x(iW{j});
end

deltaPO=pS-pO;
dEuclid=norm(deltaPO);
if ~(isfinite(dEuclid) && dEuclid>0)
    % Deterministic geometric fallback direction: obstacle center to seed
    % centroid.  This is only a candidate; exact support certification below
    % remains the authority.
    deltaPO=mean(Seeds,2)-c0;
    dEuclid=norm(deltaPO);
end
if ~(isfinite(dEuclid) && dEuclid>0)
    nBest=[NaN;NaN;NaN]; bBest=NaN; dBest=NaN;
    return;
end

nBest=deltaPO/dEuclid;
bBest=b3_swept_support(ob,nBest,ta,tb)+b3_system_offset(nBest,P);
dBest=min(nBest.'*Seeds)-bBest;
D.euclidean_distance=dEuclid;
D.support_gap=dBest;
end

function b3_log_seed_ellipsoid_diagnostics(cellId,ob,Seeds,ta,tb,P)
% Diagnostic only: expose why a cell cannot be certified.
%
% phi_j is the exact raw-ellipsoid quadratic value for each mandatory seed:
%   phi_j = min_tau (s_j-c-tau v)' S^{-1} (s_j-c-tau v)
% phi<1 raw obstacle interior, phi=1 raw surface, phi>1 raw exterior.
%
% We also report simple towing-inflated clearances for each seed point
% against the observed ellipsoid.  These are diagnostics, not the final
% corridor certificate.

S=0.5*(ob.S+ob.S.');
Sinv=pinv(S);
v=ob.v_center(:);
b3_detail_fprintf('      [SEED-DIAG] cell=%d obs=%d nSeeds=%d center=%s radii=%s\\n', ...
    cellId,ob.id,size(Seeds,2),mat2str(ob.center(:).',9),mat2str(ob.radii(:).',9));

minPhi=Inf;
minRawDist=Inf;
worstIdx=NaN;
for j=1:size(Seeds,2)
    d=Seeds(:,j)-ob.center(:);
    av=v.'*Sinv*v;
    if av>0
        tau=(v.'*Sinv*d)/av;
        tau=min(max(tau,ta),tb);
    else
        tau=ta;
    end
    r=d-tau*v;
    phi=r.'*Sinv*r;
    rawDist=aud_point_ellipsoid_distance(Seeds(:,j),ob);
    if phi<minPhi
        minPhi=phi;
        minRawDist=rawDist;
        worstIdx=j;
    end
    b3_detail_fprintf('        seed=%02d p=%s phi=%+.12e rawDist=%+.12e\\n', ...
        j,mat2str(Seeds(:,j).',9),phi,rawDist);
end

Rp=P.R_payload+P.d_margin;
Ru=P.R_uav+P.d_margin;
Rc=P.R_cable+P.d_margin;
b3_detail_fprintf('      [SEED-DIAG] closestRaw seed=%d minPhi=%+.12e rawDist=%+.12e | towingR payload=%.6g uav=%.6g cable=%.6g L=%.6g\\n', ...
    worstIdx,minPhi,minRawDist,Rp,Ru,Rc,P.L_cable);
end


function h=b3_swept_support(ob,n,ta,tb)
% Exact support for a fixed-shape, fixed-orientation ellipsoid translated at
% constant remembered velocity during [ta,tb] after replan time.
n=n(:);
q=n.'*ob.v_center;
h=n.'*ob.center + sqrt(max(0,n.'*ob.S*n)) + max(ta*q,tb*q);
end


function d=b3_system_offset(n,P)
% Payload-center offset for the temporary vertical-cable full-geometry proxy.
n=n(:);
nz=n(3);
dPayload=P.R_payload+P.d_margin;
dUav=P.R_uav+P.d_margin-P.L_cable*nz;
dCable=P.R_cable+P.d_margin+max(0,-P.L_cable*nz);
d=max([dPayload,dUav,dCable]);
end



function S=b3_sreenath_cable_state_from_jet(D,P,CFG)
% Sreenath single-quadrotor towing model state recovered from payload jet.
%
% Model convention supplied by the project:
%   p in S^2 : unit vector from quadrotor to payload
%   x_Q = x_L - L p
%   pdot = omega x p
%
% Eq.(2.18):
%   (mQ+mL)(a_L+g e3)
%     = ( p'*(f R e3) - mQ L ||pdot||^2 ) p
%
% On the hanging/taut branch containing the nominal equilibrium p=-e3,
% r := a_L + g e3 is antiparallel to p:
%   p = -r/||r||.
%
% Because the accepted reference is C6, jerk and snap are available and give
% pdot, pddot, omega and omegadot analytically.  If mQ,mL are supplied, the
% required thrust vector F=f R e3 is reconstructed so that Eq.(2.18),(2.20)
% can be audited without inventing an attitude controller or actuator limit.

if size(D,2)<5
    error('STEP4B3:SreenathJet','Payload jet must contain d0..d4.');
end

xL=D(:,1);
aL=D(:,3);
jL=D(:,4);
sL=D(:,5);

r=aL+P.g*CFG.e3;
rho=norm(r);

scale=max([1;abs(aL(:));abs(jL(:));abs(sL(:));P.g]);
fpGuard=256*eps(scale);

S=struct( ...
    'valid',false, ...
    'p',nan(3,1), ...
    'pdot',nan(3,1), ...
    'pddot',nan(3,1), ...
    'omega',nan(3,1), ...
    'omegadot',nan(3,1), ...
    'xQ',nan(3,1), ...
    'swing_deg',NaN, ...
    'payload_tension_per_mass',NaN, ...
    'payload_tension_N',NaN, ...
    'thrust_model_available',false, ...
    'thrust_vector',nan(3,1), ...
    'required_thrust_N',NaN, ...
    'required_b3',nan(3,1), ...
    'eq218_residual',NaN, ...
    'eq220_residual',NaN, ...
    'resultant',r, ...
    'fp_guard',fpGuard);

if ~(isfinite(rho) && rho>fpGuard)
    return;
end

u=r/rho;
p=-u;
Proj=eye(3)-u*u.';

rhoDot=u.'*jL;
uDot=(Proj*jL)/rho;
ProjDot=-(uDot*u.' + u*uDot.');
uDDot=(Proj*sL + ProjDot*jL)/rho - (Proj*jL)*(rhoDot/(rho^2));

pdot=-uDot;
pddot=-uDDot;

% With the Sreenath convention p'omega=0, pdot=omega x p gives
% omega = p x pdot and omegadot = p x pddot.
omega=cross(p,pdot);
omegadot=cross(p,pddot);

xQ=xL-P.L_cable*p;
c=max(-1,min(1,(-CFG.e3).'*p));

S.valid=true;
S.p=p;
S.pdot=pdot;
S.pddot=pddot;
S.omega=omega;
S.omegadot=omegadot;
S.xQ=xQ;
S.swing_deg=acosd(c);
S.payload_tension_per_mass=rho;

if isfinite(CFG.mL_kg)
    S.payload_tension_N=CFG.mL_kg*rho;
end

if isfinite(CFG.mQ_kg) && isfinite(CFG.mL_kg)
    mQ=CFG.mQ_kg;
    mL=CFG.mL_kg;
    L=P.L_cable;

    % Reconstruct F := f R e3 from Eq.(2.18),(2.20).
    pDotF = mQ*L*dot(pdot,pdot) - (mQ+mL)*rho;
    F = p*pDotF + mQ*L*cross(p,omegadot);

    fReq=norm(F);
    if isfinite(fReq) && fReq>fpGuard
        b3Req=F/fReq;
    else
        b3Req=nan(3,1);
    end

    lhs218=(mQ+mL)*r;
    rhs218=(dot(p,F)-mQ*L*dot(pdot,pdot))*p;
    lhs220=mQ*L*omegadot;
    rhs220=-cross(p,F);

    S.thrust_model_available=true;
    S.thrust_vector=F;
    S.required_thrust_N=fReq;
    S.required_b3=b3Req;
    S.eq218_residual=norm(lhs218-rhs218);
    S.eq220_residual=norm(lhs220-rhs220);
end
end


function risk=b3_predict_current_reference(t0,active,mem,CFG,P,nominalTimeShift)
risk=struct('found',false,'ids',[],'TTC',NaN,'t_risk',NaN,'body','', ...
    'min_payload_clearance',Inf,'min_uav_clearance',Inf,'min_cable_clearance',Inf, ...
    'risk_patch_ids',[], ...
    'broadphase_tests',0,'broadphase_rejects',0, ...
    'exact_point_tests',0,'exact_segment_tests',0);
if isempty(mem), return; end

H=min(CFG.lookahead_s,CFG.mission_tf-t0);
if H<=0, return; end
tau=linspace(0,H,CFG.predict_samples);
firstT=Inf;
ids=[];
bodyFirst='';

for k=1:numel(tau)
    tt=t0+tau(k);
    D=b3_exec_jet(tt,active,CFG,nominalTimeShift);
    pL=D(:,1);
    cableState=b3_sreenath_cable_state_from_jet(D,P,CFG);
    if ~cableState.valid
        risk.found=true;
        risk.TTC=tau(k);
        risk.t_risk=tt;
        risk.body='CABLE_DYNAMICS_INVALID';
        return;
    end
    pQ=cableState.xQ;
    obs=b3_memory_snapshot(mem,tt);

    for io=1:numel(obs)
        ob=obs(io);

        if CFG.predictor_sphere_broadphase
            risk.broadphase_tests=risk.broadphase_tests+3;

            [safeL,gLlb]=b3_point_ellipsoid_sphere_safe( ...
                pL,ob,P.R_payload+P.d_margin);
            if safeL
                risk.broadphase_rejects=risk.broadphase_rejects+1;
                gL=gLlb;
            else
                risk.exact_point_tests=risk.exact_point_tests+1;
                gL=aud_point_ellipsoid_distance(pL,ob)-(P.R_payload+P.d_margin);
            end

            [safeQ,gQlb]=b3_point_ellipsoid_sphere_safe( ...
                pQ,ob,P.R_uav+P.d_margin);
            if safeQ
                risk.broadphase_rejects=risk.broadphase_rejects+1;
                gQ=gQlb;
            else
                risk.exact_point_tests=risk.exact_point_tests+1;
                gQ=aud_point_ellipsoid_distance(pQ,ob)-(P.R_uav+P.d_margin);
            end

            [safeC,gClb]=b3_segment_ellipsoid_sphere_safe( ...
                pL,pQ,ob,P.R_cable+P.d_margin);
            if safeC
                risk.broadphase_rejects=risk.broadphase_rejects+1;
                gC=gClb;
            else
                risk.exact_segment_tests=risk.exact_segment_tests+1;
                gC=aud_segment_ellipsoid_distance(pL,pQ,ob)-(P.R_cable+P.d_margin);
            end
        else
            risk.exact_point_tests=risk.exact_point_tests+2;
            risk.exact_segment_tests=risk.exact_segment_tests+1;
            gL=aud_point_ellipsoid_distance(pL,ob)-(P.R_payload+P.d_margin);
            gQ=aud_point_ellipsoid_distance(pQ,ob)-(P.R_uav+P.d_margin);
            gC=aud_segment_ellipsoid_distance(pL,pQ,ob)-(P.R_cable+P.d_margin);
        end
        risk.min_payload_clearance=min(risk.min_payload_clearance,gL);
        risk.min_uav_clearance=min(risk.min_uav_clearance,gQ);
        risk.min_cable_clearance=min(risk.min_cable_clearance,gC);
        [gmin,ib]=min([gL,gQ,gC]);
        if gmin<=0
            ids(end+1)=ob.id; %#ok<AGROW>
            if isfield(ob,'patch_id'), risk.risk_patch_ids(end+1)=ob.patch_id; end %#ok<AGROW>
            if tt<firstT
                firstT=tt;
                switch ib
                    case 1, bodyFirst='PAYLOAD';
                    case 2, bodyFirst='UAV';
                    otherwise, bodyFirst='CABLE';
                end
            end
        end
    end
end

ids=unique(ids,'stable');
if ~isempty(ids)
    risk.found=true;
    risk.ids=ids;
    risk.t_risk=firstT;
    risk.TTC=max(0,firstT-t0);
    risk.body=bodyFirst;
    risk.risk_patch_ids=unique(risk.risk_patch_ids,'stable');
end
end


function [mem,newIds,changedIds]=b3_update_memory_from_visible(mem,visible,t,CFG)
% V14 INCREMENTAL OBSERVED-PATCH MEMORY.
%
% Each visible frame contributes an ellipsoid patch constructed ONLY from
% current depth-hit cells.  Patches are NOT blindly unioned into one large
% ellipsoid.  This preserves the observed-only interpretation and avoids
% inventing unobserved free-space/occupied-space between separated views.
%
% Exact redundancy rule:
%   - If the complete observed AABB of a new patch is contained in an
%     existing ellipsoid patch of the same obstacle ID, the new patch adds
%     no new certified occupied volume and is discarded.
%   - If an old patch AABB is completely contained in the new patch
%     ellipsoid, that old patch is removed.
%
% Containment is checked on the 8 AABB vertices.  Because an AABB is the
% convex hull of its vertices and an ellipsoid is convex, vertex containment
% implies containment of the complete AABB.
%
% IMPORTANT MOTION SCOPE:
% Partial-surface centroids move as viewpoint/visibility changes even for a
% static obstacle.  Therefore V14 does NOT infer obstacle translation from
% successive partial-surface centroids.  All current embedded mission
% obstacles are static.  Dynamic-obstacle velocity estimation requires
% surface registration / tracking and remains a later stage.

newIds=[];
changedIds=[];

for i=1:numel(visible)
    z=visible(i);
    item=b3_canonicalize_visible(z,t);
    same=find([mem.id]==item.id);

    if isempty(same)
        item.patch_id=1;
        item.parent_observation_index=1;
        mem(end+1)=item; %#ok<AGROW>
        newIds(end+1)=item.id; %#ok<AGROW>
        continue;
    end

    % If the newly observed AABB is already fully represented by any
    % existing patch, this frame adds no new geometric knowledge.
    contained=false;
    for kk=same
        if b3_box_contained_in_patch(item.obs_box_min,item.obs_box_max,mem(kk))
            contained=true;
            break;
        end
    end
    if contained
        % Refresh last_seen only for bookkeeping; geometry is unchanged.
        latest=max([mem(same).parent_observation_index]);
        for kk=same
            mem(kk).last_seen=max(mem(kk).last_seen,t);
            mem(kk).parent_observation_index=max(mem(kk).parent_observation_index,latest);
        end
        continue;
    end

    % Remove same-ID patches whose complete observed boxes are subsumed by
    % the new patch. This is exact geometric redundancy elimination.
    remove=false(1,numel(mem));
    for kk=same
        if b3_box_contained_in_patch(mem(kk).obs_box_min,mem(kk).obs_box_max,item)
            remove(kk)=true;
        end
    end

    nextPatch=max([mem(same).patch_id])+1;
    nextObs=max([mem(same).parent_observation_index])+1;
    mem(remove)=[];

    item.patch_id=nextPatch;
    item.parent_observation_index=nextObs;
    mem(end+1)=item; %#ok<AGROW>
    changedIds(end+1)=item.id; %#ok<AGROW>
end

newIds=unique(newIds,'stable');
changedIds=unique(changedIds,'stable');
end


function tf=b3_box_contained_in_patch(boxMin,boxMax,patch)
% Exact AABB-in-convex-ellipsoid test via all 8 vertices.
x=[boxMin(1),boxMax(1)];
y=[boxMin(2),boxMax(2)];
z=[boxMin(3),boxMax(3)];
V=zeros(3,8);
c=0;
for ix=1:2
    for iy=1:2
        for iz=1:2
            c=c+1;
            V(:,c)=[x(ix);y(iy);z(iz)];
        end
    end
end

S=0.5*(patch.S+patch.S.');
Sinv=pinv(S);
D=V-patch.center_seen;
phi=sum(D.*(Sinv*D),1);
scale=max(1,max(abs(phi)));
tol=128*eps(scale);
tf=all(phi<=1+tol);
end

function item=b3_canonicalize_visible(z,t)
req={'id','p_center','R_obs','ellipsoid_radii','obs_box_min','obs_box_max','n_observed_points'};
for k=1:numel(req)
    if ~isfield(z,req{k})
        error('STEP4B3:SensorSchema','Observed-only sensor model is missing field %s.',req{k});
    end
end

item=struct();
item.id=z.id;
item.center_seen=z.p_center(:);
item.center=item.center_seen;
item.R=z.R_obs;
item.radii=z.ellipsoid_radii(:);
item.S=item.R*diag(item.radii.^2)*item.R.';
item.v_center=zeros(3,1); % V14: no false motion estimate from changing partial-surface centroid
item.last_seen=t;
item.patch_id=0;
item.parent_observation_index=1;
item.motion_model='STATIC_OBSERVED_PATCH_V14';
item.type='observed_ellipsoid';
item.obs_box_min=z.obs_box_min(:);
item.obs_box_max=z.obs_box_max(:);
item.n_observed_points=z.n_observed_points;
item.model_source='DEPTH_HIT_CELLS_ONLY';
end


function obs=b3_memory_snapshot(mem,t)
if isempty(mem)
    obs=repmat(b3_empty_memory_item(),1,0);
    return;
end
obs=mem;
for i=1:numel(mem)
    obs(i).center=mem(i).center_seen + mem(i).v_center*(t-mem(i).last_seen);
end
end


function ids=b3_memory_ids(mem)
if isempty(mem), ids=[]; else, ids=unique([mem.id],'stable'); end
end


function mem=b3_empty_memory()
mem=repmat(b3_empty_memory_item(),1,0);
end


function z=b3_empty_memory_item()
z=struct('id',0,'center_seen',zeros(3,1),'center',zeros(3,1),'R',eye(3), ...
    'radii',ones(3,1),'S',eye(3),'v_center',zeros(3,1),'last_seen',NaN,'type','', ...
    'obs_box_min',zeros(3,1),'obs_box_max',zeros(3,1),'n_observed_points',0, ...
    'model_source','','patch_id',0,'parent_observation_index',0, ...
    'motion_model','STATIC_OBSERVED_PATCH_V14');
end


function D=b3_exec_jet(t,active,CFG,nominalTimeShift)
if nargin<4 || ~isfinite(nominalTimeShift)
    nominalTimeShift=0;
end
if active.valid && t>=active.t_start-1e-12 && t<=active.t_end+1e-12
    D=b3_eval_active_jet(active,t);
else
    D=b3_nominal_jet(t-nominalTimeShift,CFG);
end
end


function D=b3_nominal_jet(t,CFG)
D=zeros(3,7);
D(:,1)=CFG.p0+CFG.v_nom*t;
D(:,2)=CFG.v_nom;
end


function D=b3_eval_active_jet(A,t)
s=(t-A.t_start)/A.T;
s=min(1,max(0,s));
CP=A.result.control_points;

if any(strcmpi(A.backend,{'BEZIER_D13','POLY_D13'}))
    edges=[0,cumsum(A.model.h)];
    edges(end)=1;
else
    edges=A.model.breaks;
    edges(1)=0;
    edges(end)=1;
end

D=b3_eval_piecewise_bezier_physical(CP,edges,s,A.T);
end


function D=b3_eval_piecewise_bezier_physical(CP,edges,s,T)
nseg=size(CP,3);
if s>=1
    q=nseg;
else
    q=find(s>=edges(1:end-1) & s<edges(2:end),1,'first');
    if isempty(q), q=max(1,min(nseg,find(edges<=s,1,'last'))); end
end
h=edges(q+1)-edges(q);
if ~(h>0), error('STEP4B3:Eval','Nonpositive piece duration.'); end
tau=(s-edges(q))/h;
tau=min(1,max(0,tau));
Pq=CP(:,:,q);
n=size(Pq,2)-1;
D=zeros(3,7);

for r=0:6
    Pr=Pq;
    nr=n;
    for j=1:r
        Pr=nr*diff(Pr,1,2);
        nr=nr-1;
    end
    val=b3_bezier_eval_any(Pr,tau);
    D(:,r+1)=val/((h*T)^r);
end
end


function y=b3_bezier_eval_any(P,tau)
n=size(P,2)-1;
B=zeros(n+1,1);
for k=0:n
    B(k+1)=nchoosek(n,k)*(tau^k)*((1-tau)^(n-k));
end
y=P*B;
end


function yaw=b3_camera_yaw(t,~,active,CFG)
% Camera yaw is a separate C6 reference. It is built only from the currently
% certified trajectory and therefore never uses an unknown obstacle location.
Y=b3_exec_yaw_jet(t,active,CFG);
yaw=Y(1);
end


function Y=b3_exec_yaw_jet(t,active,CFG)
if active.valid && isfield(active,'yaw') && isstruct(active.yaw) && ...
        isfield(active.yaw,'coeff') && t>=active.t_start-1e-12 && t<=active.t_end+1e-12
    s=(t-active.t_start)/active.T;
    s=min(1,max(0,s));
    Y=b3_eval_yaw_poly(active.yaw,s,active.T);
else
    Y=zeros(1,7);
    Y(1)=CFG.nominal_yaw;
end
end


function YAW=b3_build_c6_yaw_for_active(A,Ystart,CFG)
% C6 yaw reference: vehicle FRONT faces the HORIZONTAL travel direction.
%
% Desired geometric heading:
%   psi_dir(t) = atan2(v_y(t),v_x(t)),
% where v is the executing payload-reference velocity.
%
% Degree-27 Bernstein is retained.  C6 endpoint jets are NOT imposed through
% a pseudoinverse particular solution.  Instead, the first 7 and last 7
% Bernstein control points are constructed analytically from the normalized
% endpoint jets d0..d6.  Therefore endpoint C6 conditions are separated from
% the least-squares tangent-direction fit.
%
% Only the middle 14 control points are fitted to the unwrapped horizontal
% travel direction.  This preserves the user's stated yaw objective while
% making start/end C6 constraints structurally explicit.

if ~strcmpi(CFG.yaw_mode,'C6_DEG27_TANGENT_FIT_FAST')
    error('STEP4B3:YawMode','Unsupported B3 yaw mode: %s',CFG.yaw_mode);
end

N=CFG.yaw_fit_sample_count;
n=27;
ss=linspace(0,1,N).';
raw=zeros(N,1);
valid=false(N,1);

for i=1:N
    tt=A.t_start+ss(i)*A.T;
    D=b3_eval_active_jet(A,tt);
    v=D(:,2);

    % Vehicle front follows horizontal velocity direction.
    if hypot(v(1),v(2))>1e-10
        raw(i)=atan2(v(2),v(1));
        valid(i)=true;
    end
end

if ~any(valid)
    raw(:)=Ystart(1);
else
    idx=find(valid);
    raw(1:idx(1)-1)=raw(idx(1));
    raw(idx(end)+1:end)=raw(idx(end));

    for k=1:(numel(idx)-1)
        if idx(k+1)>idx(k)+1
            raw(idx(k):idx(k+1))=linspace( ...
                raw(idx(k)),raw(idx(k+1)),idx(k+1)-idx(k)+1);
        end
    end
end

raw=unwrap(raw);
raw=raw+2*pi*round((Ystart(1)-raw(1))/(2*pi));

% At spatial rejoin the nominal travel direction is +x, hence nominal yaw.
% Keep the equivalent 2*pi branch closest to the fitted heading.
Yend=zeros(1,7);
Yend(1)=CFG.nominal_yaw + ...
    2*pi*round((raw(end)-CFG.nominal_yaw)/(2*pi));

% Convert PHYSICAL-TIME endpoint derivatives to normalized-s derivatives.
J0s=Ystart;
J1s=Yend;
for r=1:6
    J0s(r+1)=Ystart(r+1)*(A.T^r);
    J1s(r+1)=Yend(r+1)*(A.T^r);
end

% Analytically fix the endpoint control points.
[cFixed,fixedIdx,freeIdx]=b3_yaw_construct_endpoint_controls(n,J0s,J1s);

persistent CACHE
if isempty(CACHE) || CACHE.N~=N || CACHE.n~=n
    B=zeros(N,n+1);
    for i=1:N
        B(i,:)=b3_bernstein_basis_row(n,ss(i));
    end

    freeIdx0=8:21; % MATLAB indices for c_7 ... c_20
    CACHE=struct();
    CACHE.N=N;
    CACHE.n=n;
    CACHE.B=B;
    CACHE.freeIdx=freeIdx0;
    CACHE.Bfree=B(:,freeIdx0);
    CACHE.BfreePinv=pinv(CACHE.Bfree);
end

if ~isequal(freeIdx,CACHE.freeIdx)
    error('STEP4B3:YawIndex','Yaw free-control-point index mismatch.');
end

c=cFixed;
rhs=raw-CACHE.B(:,fixedIdx)*cFixed(fixedIdx);
c(freeIdx)=CACHE.BfreePinv*rhs;

fit=CACHE.B*c;
err=atan2(sin(fit-raw),cos(fit-raw));

YAW=struct();
YAW.degree=n;
YAW.coeff=c(:).';
YAW.raw_s=ss.';
YAW.raw_unwrapped_yaw=raw.';
YAW.fit_yaw=fit.';
YAW.start_physical_jet=Ystart;
YAW.end_physical_jet=Yend;
YAW.target='HORIZONTAL_EXECUTING_TRAJECTORY_TANGENT';
YAW.target_definition='psi=atan2(v_y,v_x), vehicle front faces horizontal travel direction';
YAW.fast_cached_linear_map=true;
YAW.endpoint_controls_analytic=true;
YAW.fixed_control_indices=fixedIdx;
YAW.free_control_indices=freeIdx;

% Physical-time endpoint C6 audit.
[Y0,GY0]=b3_bezier_endpoint_jet_with_guard(YAW.coeff,A.T,'left');
[Y1,GY1]=b3_bezier_endpoint_jet_with_guard(YAW.coeff,A.T,'right');

[ysr,ysg,ysc]=b3_jet_target_match_certificate(Y0,GY0,Ystart);
[yer,yeg,yec]=b3_jet_target_match_certificate(Y1,GY1,Yend);

YAW.start_splice_residual_d0_d6=ysr;
YAW.start_splice_guard_d0_d6=ysg;
YAW.start_backward_error_certified=ysc;
YAW.end_residual_d0_d6=yer;
YAW.end_guard_d0_d6=yeg;
YAW.end_backward_error_certified=yec;

% Independent normalized endpoint-constraint residual:
% Aeq*c = beq is exactly the condition used to define C6 endpoint jets.
[Aeq,beq]=b3_yaw_endpoint_constraints(n,J0s,J1s);
YAW.constraint_residual=Aeq*c-beq;
[YAW.constraint_residual_inf,YAW.constraint_guard_inf, ...
    YAW.constraint_backward_error_certified]= ...
    b3_linear_equality_backward_error_certificate(Aeq,c,beq);

YAW.max_abs_fit_error_rad=max(abs(err));
YAW.rms_fit_error_rad=sqrt(mean(err.^2));
YAW.structural_c6=true;
YAW.c6_hard_certified= ...
    YAW.structural_c6 && ...
    YAW.endpoint_controls_analytic && ...
    ysc && yec && ...
    YAW.constraint_backward_error_certified;
end


function [c,fixedIdx,freeIdx]=b3_yaw_construct_endpoint_controls(n,J0s,J1s)
% Construct first/last seven Bernstein control points directly from
% normalized endpoint derivatives d0..d6.
%
% Left:
%   J_r = n!/(n-r)! * Delta^r c_0
%
% Right:
%   J_r = n!/(n-r)! * Delta^r c_{n-r}
%
% This removes the endpoint constraints from the least-squares solve.

if n<13
    error('STEP4B3:YawDegree','Need degree >= 13 for independent C6 endpoints.');
end
if numel(J0s)~=7 || numel(J1s)~=7
    error('STEP4B3:YawJet','Yaw endpoint jets must contain d0..d6.');
end

c=zeros(n+1,1);

% Left endpoint controls c_0 ... c_6.
for r=0:6
    ff=factorial(n)/factorial(n-r);
    if r==0
        c(1)=J0s(1);
    else
        acc=0;
        for k=0:(r-1)
            acc=acc + ((-1)^(r-k))*nchoosek(r,k)*c(k+1);
        end
        c(r+1)=J0s(r+1)/ff - acc;
    end
end

% Right endpoint controls c_n ... c_{n-6}.
for r=0:6
    ff=factorial(n)/factorial(n-r);
    if r==0
        c(n+1)=J1s(1);
    else
        acc=0;
        for k=1:r
            idx=n-r+k+1; % MATLAB index of c_{n-r+k}
            acc=acc + ((-1)^(r-k))*nchoosek(r,k)*c(idx);
        end
        c(n-r+1)=((-1)^r)*(J1s(r+1)/ff - acc);
    end
end

fixedIdx=[1:7, n-5:n+1];
freeIdx=setdiff(1:(n+1),fixedIdx,'stable');

if numel(fixedIdx)~=14 || numel(freeIdx)~=(n+1-14)
    error('STEP4B3:YawIndex','Unexpected fixed/free yaw control-point count.');
end
end


function [resInf,guardInf,cert]=b3_linear_equality_backward_error_certificate(A,x,b)
% Componentwise forward-error envelope for y=A*x-b.
% gamma_k is derived from row dot-product operation count.

if size(A,2)~=numel(x) || size(A,1)~=numel(b)
    error('STEP4B3:LinearEqAuditShape','Linear equality dimensions disagree.');
end

x=x(:);
b=b(:);
y=A*x-b;
u=eps/2;
m=size(A,2);

% m products + (m-1) additions + final subtraction.
k=2*m;
gamma=(k*u)/(1-k*u);

scale=abs(A)*abs(x)+abs(b);
guard=gamma*scale + eps(max(1,abs(y)));

resInf=max(abs(y));
guardInf=max(guard);
cert=all(abs(y)<=guard);
end



function [Aeq,beq]=b3_yaw_endpoint_constraints(n,J0,J1)
% Fixed degree-dependent equality matrix is OFFLINE/cacheable; only the
% endpoint jet right-hand side changes online.
persistent CACHE_N CACHE_A
hit=[];
if ~isempty(CACHE_N)
    hit=find(CACHE_N==n,1,'first');
end

if isempty(hit)
    A=zeros(14,n+1);
    rr=0;
    for r=0:6
        rr=rr+1;
        ff=factorial(n)/factorial(n-r);
        for k=0:r
            A(rr,k+1)=ff*((-1)^(r-k))*nchoosek(r,k);
        end
    end
    for r=0:6
        rr=rr+1;
        ff=factorial(n)/factorial(n-r);
        for k=0:r
            A(rr,n-r+k+1)=ff*((-1)^(r-k))*nchoosek(r,k);
        end
    end

    if isempty(CACHE_N)
        CACHE_N=n;
        CACHE_A={A};
        hit=1;
    else
        CACHE_N(end+1)=n;
        CACHE_A{end+1}=A;
        hit=numel(CACHE_N);
    end
end

Aeq=CACHE_A{hit};
beq=[J0(:);J1(:)];
end




function Y=b3_eval_yaw_poly(YAW,s,T)
n=YAW.degree;
c=YAW.coeff(:).';
Y=zeros(1,7);
for r=0:6
    cr=c;
    nr=n;
    for j=1:r
        cr=nr*diff(cr);
        nr=nr-1;
    end
    Y(r+1)=b3_bernstein_basis_row(nr,s)*cr(:)/(T^r);
end
end


function B=b3_bernstein_basis_row(n,s)
B=zeros(1,n+1);
for k=0:n
    B(k+1)=nchoosek(n,k)*(s^k)*((1-s)^(n-k));
end
end


function R=b3_camera_rotation_from_yaw(yaw)
f=[cos(yaw);sin(yaw);0];
right=[sin(yaw);-cos(yaw);0];
down=[0;0;-1];
R=[right,down,f];
if det(R)<=0 || norm(R.'*R-eye(3),'fro')>1e-12
    error('STEP4B3:CameraRotation','Invalid camera rotation.');
end
end


function q=b3_percentile(x,pct)
% Toolbox-free linear percentile used only for runtime reporting.
x=sort(x(:));
if isempty(x), q=NaN; return; end
if numel(x)==1, q=x; return; end
u=1+(numel(x)-1)*(pct/100);
i=floor(u); j=ceil(u);
if i==j
    q=x(i);
else
    q=x(i)+(u-i)*(x(j)-x(i));
end
end


function K=b3_kinematic_bezier_certificate(CP,M,T,backend,CFG)
% Rigorous continuous-time derivative-norm upper bounds from extracted
% Bezier pieces. Works for D13 and extracted-Bezier BSPLINE_D7 uniformly.
%
% CP: 3 x (degree+1) x npiece.
% M.h for D13, diff(M.breaks) for BSPLINE_D7: normalized piece durations.
if ndims(CP)~=3 || size(CP,1)~=3
    error('STEP4B3:KinematicCPShape','Expected extracted Bezier CP array 3 x (n+1) x npiece.');
end

n=size(CP,2)-1;
np=size(CP,3);
switch upper(backend)
    case {'BEZIER_D13','POLY_D13'}
        frac=M.h(:).';
    case 'BSPLINE_D7'
        frac=diff(M.breaks(:).');
    otherwise
        error('STEP4B3:KinematicBackend','Unknown backend.');
end
if numel(frac)~=np
    error('STEP4B3:KinematicPieceCount','Piece-duration count mismatch.');
end

ub=zeros(1,6);
for q=1:np
    dt=T*frac(q);
    if ~(isfinite(dt) && dt>0)
        error('STEP4B3:KinematicDuration','Nonpositive physical piece duration.');
    end
    D=CP(:,:,q);
    degree=n;
    for r=1:min(6,n)
        D=degree*diff(D,1,2)/dt;
        degree=degree-1;
        nr=vecnorm(D,2,1);
        if ~isempty(nr)
            ub(r)=max(ub(r),max(nr));
        end
    end
end

hasV=isfinite(CFG.max_ref_speed) && CFG.max_ref_speed>0;
hasA=isfinite(CFG.max_ref_acc) && CFG.max_ref_acc>0;
hard=CFG.enforce_kinematic_limits && hasV && hasA;

passV=~hasV || ub(1)<=CFG.max_ref_speed;
passA=~hasA || ub(2)<=CFG.max_ref_acc;

K=struct();
K.derivative_norm_upper_bound=ub;
K.max_speed_bound=ub(1);
K.max_acc_bound=ub(2);
K.max_jerk_bound=ub(3);
K.max_d4_bound=ub(4);
K.max_d5_bound=ub(5);
K.max_d6_bound=ub(6);
K.speed_limit=CFG.max_ref_speed;
K.acc_limit=CFG.max_ref_acc;
K.speed_limit_supplied=hasV;
K.acc_limit_supplied=hasA;
K.preference_limits_supplied=hard;
K.hard_gate_active=false;
K.pass_speed=passV;
K.pass_acc=passA;
K.pass=passV && passA;
K.method='EXTRACTED_BEZIER_DERIVATIVE_CONTROL_POINT_CONVEX_HULL';
K.jerk_to_d6_policy='DIAGNOSTIC_ONLY';

% V59 diagnostic only: defer dense realized kinematics until post-mission.
if CFG.defer_measurement_diagnostics
    K.dense_realized_max_speed=NaN;
    K.dense_realized_max_acc=NaN;
    K.dense_sample_count=0;
    K.dense_diagnostic_runtime_ms=0;
    K.speed_bound_to_dense_ratio=NaN;
    K.acc_bound_to_dense_ratio=NaN;
    K.dense_deferred=true;
else
    qdense=tic;
    [denseV,denseA,denseN]=b3_dense_realized_kinematics(CP,frac,T);
    K.dense_realized_max_speed=denseV;
    K.dense_realized_max_acc=denseA;
    K.dense_sample_count=denseN;
    K.dense_diagnostic_runtime_ms=1e3*toc(qdense);
    K.speed_bound_to_dense_ratio=ub(1)/max(denseV,realmin);
    K.acc_bound_to_dense_ratio=ub(2)/max(denseA,realmin);
    K.dense_deferred=false;
end
end




function D=b3_rigid_cable_continuous_certificate(backend,R,M,C,T,obs,P,CFG)
% V41: DIRECT DYNAMIC-SEPARATOR CERTIFICATE.
%
% The old vertical-proxy separators are used only to obtain an initial
% obstacle-avoiding trajectory.  They are NOT reused as the rigid-cable
% safety authority.
%
% For each local trajectory proof piece:
%   1) form the physical acceleration Bezier once;
%   2) derive a continuous p-cap from r=a+g e3;
%   3) for EVERY observed obstacle and each component PAYLOAD/UAV/CABLE,
%      compute the primal distance between the local trajectory hull and the
%      corresponding DYNAMIC forbidden set;
%   4) build the separating normal from the closest-point pair;
%   5) verify the separator by the exact support function of that dynamic set.
%
% UAV cable-direction p-cap convexification:
%   conv{p : ||p||=1, angle(p,-e3)<=alpha}
%       = {q : ||q||<=1, e3'*q <= -cos(alpha)}.
%
% Cable swept-direction convexification:
%   z=lambda*p, 0<=lambda<=1
%   is represented by
%       ||z||<=lambda,
%       e3'*z <= -cos(alpha)*lambda.
%
% These sets are convex and SOC-representable.  The resulting certificate is
% continuous-time and sufficient.  If the convexified forbidden set overlaps
% the trajectory hull, the result is UNCERTIFIED; it is not automatically
% labelled a true physical collision.

D=struct('pass',true,'piece_count',0,'accepted_leaf_count',0, ...
    'split_node_count',0,'max_subdivision_depth',0,'unresolved_leaf_count',0, ...
    'max_swing_bound_deg',0,'min_tension_per_mass_lb',Inf, ...
    'min_dynamic_corridor_slack',Inf,'max_fp_guard',0, ...
    'dynamic_socp_calls',0,'dynamic_socp_runtime_ms',0, ...
    'support_certified_count',0, ...
    'solver_status_nonpositive_but_support_certified',0, ...
    'early_lipschitz_attempts',0,'early_lipschitz_certified',0, ...
    'runtime_audit_records',b3_empty_dynamic_runtime_record_array(), ...
    'runtime_normal_cache',b3_empty_dynamic_normal_cache(), ...
    'runtime_audit_summary',struct(), ...
    'broadphase_tests',0,'broadphase_rejects',0,'broadphase_runtime_ms',0, ...
    'analytic_witness_checks',0, ...
    'analytic_witness_hits',0, ...
    'analytic_witness_check_ms',0, ...
    'normal_reuse_fastpath_checks',0, ...
    'normal_reuse_fastpath_hits',0, ...
    'normal_reuse_fastpath_check_ms',0, ...
    'worst_cell',NaN,'worst_subcell',NaN,'worst_span',NaN, ...
    'worst_plane',NaN,'worst_component','','worst_obstacle_id',NaN, ...
    'failure_reason','NONE','failure_class','NONE', ...
    'failure_ta',NaN,'failure_tb',NaN,'failure_u0',NaN,'failure_u1',NaN, ...
    'limit_point_audit',struct(), ...
    'guarantee_level','SUFFICIENT_CONTINUOUS_DYNAMIC_SEPARATOR');

GD=b3_global_rigid_cable_dynamics_bound(backend,R,M,T,P,CFG);
D.max_swing_bound_deg=GD.max_swing_bound_deg;
D.min_tension_per_mass_lb=GD.min_tension_per_mass_lb;
D.global_dynamics=GD;

switch upper(backend)
    case {'BEZIER_D13','POLY_D13'}
        for ci=1:3
            leaves=C.cells{ci}.subcells;
            Pseg=R.control_points(:,:,ci);

            for rr=1:numel(leaves)
                CC=leaves(rr);
                Rloc=b3_bezier_restrict_map(size(Pseg,2)-1,CC.u0,CC.u1);
                Ploc=Pseg*Rloc.';
                dt=T*M.h(ci)*(CC.u1-CC.u0);
                [Vloc,Aloc,Jloc]=b3_initial_piece_kinematic_beziers(Ploc,dt);

                beforeSocp=D.dynamic_socp_calls;
                beforeSplit=D.split_node_count;
                beforeLip=D.early_lipschitz_certified;
                D=b3_adaptive_dynamic_separator_piece( ...
                    D,Ploc,Vloc,Aloc,Jloc,CC.ta,CC.tb,obs,ci,rr,ci,P,CFG,0,CC.u0,CC.u1);
                b3_detail_fprintf(['    [DYN-CERT-PROGRESS] cell=%d subcell=%d span=%d ', ...
                    'pass=%d dSOCP=%d dSplit=%d dLipCert=%d totalSOCP=%d\n'], ...
                    ci,rr,ci,D.pass,D.dynamic_socp_calls-beforeSocp, ...
                    D.split_node_count-beforeSplit, ...
                    D.early_lipschitz_certified-beforeLip,D.dynamic_socp_calls);
                if ~D.pass
                    D.runtime_audit_summary=b3_summarize_dynamic_runtime_audit( ...
                        D.runtime_audit_records,obs);
                    b3_print_dynamic_runtime_audit(D.runtime_audit_summary);
                    return;
                end
            end
        end

    case 'BSPLINE_D7'
        for ci=1:3
            leaves=C.cells{ci}.subcells;
            spans=find(M.cellOfSpan==ci);
            ns=numel(spans);

            for kk=1:ns
                qspan=spans(kk);
                spanU0=(kk-1)/ns;
                spanU1=kk/ns;
                Pspan=R.control_points(:,:,qspan);

                for rr=1:numel(leaves)
                    CC=leaves(rr);
                    ov0=max(spanU0,CC.u0);
                    ov1=min(spanU1,CC.u1);
                    if ov1<=ov0, continue; end

                    a=(ov0-spanU0)/(spanU1-spanU0);
                    b=(ov1-spanU0)/(spanU1-spanU0);
                    Rloc=b3_bezier_restrict_map(size(Pspan,2)-1,a,b);
                    Ploc=Pspan*Rloc.';

                    spanFrac=M.breaks(qspan+1)-M.breaks(qspan);
                    dt=T*spanFrac*(b-a);
                    [Vloc,Aloc,Jloc]=b3_initial_piece_kinematic_beziers(Ploc,dt);

                    % Convert overlap position within corridor cell to its
                    % exact relative-time interval.
                    den=max(realmin,CC.u1-CC.u0);
                    ta=CC.ta + (ov0-CC.u0)/den*(CC.tb-CC.ta);
                    tb=CC.ta + (ov1-CC.u0)/den*(CC.tb-CC.ta);

                    beforeSocp=D.dynamic_socp_calls;
                    beforeSplit=D.split_node_count;
                    beforeLip=D.early_lipschitz_certified;
                    D=b3_adaptive_dynamic_separator_piece( ...
                        D,Ploc,Vloc,Aloc,Jloc,ta,tb,obs,ci,rr,qspan,P,CFG,0,ov0,ov1);
                    b3_detail_fprintf(['    [DYN-CERT-PROGRESS] cell=%d subcell=%d span=%d ', ...
                        'pass=%d dSOCP=%d dSplit=%d dLipCert=%d totalSOCP=%d\n'], ...
                        ci,rr,qspan,D.pass,D.dynamic_socp_calls-beforeSocp, ...
                        D.split_node_count-beforeSplit, ...
                        D.early_lipschitz_certified-beforeLip,D.dynamic_socp_calls);
                    if ~D.pass
                    D.runtime_audit_summary=b3_summarize_dynamic_runtime_audit( ...
                        D.runtime_audit_records,obs);
                    b3_print_dynamic_runtime_audit(D.runtime_audit_summary);
                    return;
                end
                end
            end
        end

    otherwise
        error('STEP4B3:RigidCableBackend','Unsupported backend %s.',backend);
end

D.runtime_audit_summary=b3_summarize_dynamic_runtime_audit( ...
    D.runtime_audit_records,obs);
b3_print_dynamic_runtime_audit(D.runtime_audit_summary);

if isinf(D.min_dynamic_corridor_slack)
    D.min_dynamic_corridor_slack=NaN;
end
if isinf(D.min_tension_per_mass_lb)
    D.min_tension_per_mass_lb=NaN;
end
end



function G=b3_global_rigid_cable_dynamics_bound(backend,R,M,T,P,CFG)
% Global continuous bound, independent of obstacle-certificate traversal.
G=struct('max_swing_bound_deg',0,'min_tension_per_mass_lb',Inf, ...
    'max_payload_speed_bound',0,'max_payload_jerk_bound',0, ...
    'min_resultant_vertical_lb',Inf,'max_pdot_bound',NaN, ...
    'max_uav_speed_bound',NaN,'valid_lipschitz',false, ...
    'valid',true,'n_pieces',0);

switch upper(backend)
    case {'BEZIER_D13','POLY_D13'}
        np=size(R.control_points,3);
        for q=1:np
            Pq=R.control_points(:,:,q);
            dt=T*M.h(q);
            [V,A,J]=b3_initial_piece_kinematic_beziers(Pq,dt);
            G=b3_accumulate_global_rigid_dynamics(G,V,A,J,P,CFG);
        end

    case 'BSPLINE_D7'
        np=size(R.control_points,3);
        for q=1:np
            Pq=R.control_points(:,:,q);
            dt=T*(M.breaks(q+1)-M.breaks(q));
            [V,A,J]=b3_initial_piece_kinematic_beziers(Pq,dt);
            G=b3_accumulate_global_rigid_dynamics(G,V,A,J,P,CFG);
        end

    otherwise
        error('STEP4B3:RigidCableBackend','Unsupported backend %s.',backend);
end

if isinf(G.min_tension_per_mass_lb)
    G.min_tension_per_mass_lb=NaN;
    G.valid=false;
end

if isfinite(G.min_resultant_vertical_lb) && G.min_resultant_vertical_lb>0 && ...
        isfinite(G.max_payload_jerk_bound)
    G.max_pdot_bound=G.max_payload_jerk_bound/G.min_resultant_vertical_lb;
    G.max_uav_speed_bound=G.max_payload_speed_bound + ...
        P.L_cable*G.max_pdot_bound;
    G.valid_lipschitz=isfinite(G.max_uav_speed_bound);
else
    G.max_pdot_bound=NaN;
    G.max_uav_speed_bound=NaN;
    G.valid_lipschitz=false;
end
end

function G=b3_accumulate_global_rigid_dynamics(G,V,A,J,P,CFG)
G.max_payload_speed_bound=max(G.max_payload_speed_bound, ...
    max(sqrt(sum(V.^2,1))));
G.max_payload_jerk_bound=max(G.max_payload_jerk_bound, ...
    max(sqrt(sum(J.^2,1))));

Res=A+P.g*CFG.e3;
if any(~isfinite(Res(:)))
    G.valid=false;
    G.max_swing_bound_deg=180;
    G.min_tension_per_mass_lb=NaN;
    return;
end

rz=CFG.e3.'*Res;
G.min_resultant_vertical_lb=min(G.min_resultant_vertical_lb,min(rz));
rxy=sqrt(sum(Res(1:2,:).^2,1));
scale=max([1;abs(Res(:));P.g]);
guard=256*eps(scale);

if min(rz)<=guard
    G.valid=false;
    G.max_swing_bound_deg=180;
    G.min_tension_per_mass_lb=min(G.min_tension_per_mass_lb,min(rz));
    G.n_pieces=G.n_pieces+1;
    return;
end

alpha=atan(max(rxy./rz));
G.max_swing_bound_deg=max(G.max_swing_bound_deg,rad2deg(alpha));
G.min_tension_per_mass_lb=min(G.min_tension_per_mass_lb,min(rz));
G.n_pieces=G.n_pieces+1;
end


function [V,A,J]=b3_initial_piece_kinematic_beziers(Ppos,dt)
% Physical-time derivative Bezier polygons formed ONCE on an initial local
% piece. These are used for global continuous bounds and are never recomputed
% after recursive microscopic subdivision.

n=size(Ppos,2)-1;
if ~(isfinite(dt) && dt>0 && n>=3)
    error('STEP4B3:InitialPieceKinematics', ...
        'Need degree>=3 and positive finite dt; degree=%d dt=%g.',n,dt);
end

V=n*diff(Ppos,1,2)/dt;
A=n*(n-1)*diff(Ppos,2,2)/(dt^2);
J=n*(n-1)*(n-2)*diff(Ppos,3,2)/(dt^3);

if any(~isfinite(V(:))) || any(~isfinite(A(:))) || any(~isfinite(J(:)))
    error('STEP4B3:InitialPieceKinematics', ...
        'Nonfinite initial-piece physical derivative control polygon.');
end
end


function A=b3_physical_acceleration_bezier(Ppos,dt)
n=size(Ppos,2)-1;
if ~(isfinite(dt) && dt>0 && n>=2)
    error('STEP4B3:RigidCableAccelerationPiece', ...
        'Cannot form physical acceleration Bezier: degree=%d dt=%g.',n,dt);
end
A=n*(n-1)*diff(Ppos,2,2)/(dt^2);
if any(~isfinite(A(:)))
    error('STEP4B3:RigidCableAccelerationFinite', ...
        'Initial physical acceleration Bezier is non-finite.');
end
end


function D=b3_adaptive_dynamic_separator_piece( ...
    D,Ppos,Vphys,Aphys,Jphys,ta,tb,obs,ci,rr,spanId,P,CFG,depth,u0Global,u1Global)

D.piece_count=D.piece_count+1;
D.max_subdivision_depth=max(D.max_subdivision_depth,depth);

AUDCTX=struct('cell',ci,'subcell',rr,'span',spanId,'depth',depth, ...
    'u0',u0Global,'u1',u1Global);
[nodePass,node,D.runtime_normal_cache]=b3_dynamic_separator_piece_test( ...
    Ppos,Aphys,ta,tb,obs,P,CFG,AUDCTX,D.runtime_normal_cache);

if ~isempty(node.runtime_records)
    D.runtime_audit_records=[D.runtime_audit_records;node.runtime_records(:)]; %#ok<AGROW>
end

D.dynamic_socp_calls=D.dynamic_socp_calls+node.socp_calls;
D.dynamic_socp_runtime_ms=D.dynamic_socp_runtime_ms+node.socp_runtime_ms;
D.support_certified_count=D.support_certified_count+node.support_certified_count;
D.solver_status_nonpositive_but_support_certified= ...
    D.solver_status_nonpositive_but_support_certified + ...
    node.solver_status_nonpositive_but_support_certified;
D.broadphase_tests=D.broadphase_tests+node.broadphase_tests;
D.broadphase_rejects=D.broadphase_rejects+node.broadphase_rejects;
D.broadphase_runtime_ms=D.broadphase_runtime_ms+node.broadphase_runtime_ms;
D.analytic_witness_checks=D.analytic_witness_checks+node.analytic_witness_checks;
D.analytic_witness_hits=D.analytic_witness_hits+node.analytic_witness_hits;
D.analytic_witness_check_ms=D.analytic_witness_check_ms+node.analytic_witness_check_ms;
D.normal_reuse_fastpath_checks=D.normal_reuse_fastpath_checks+node.normal_reuse_fastpath_checks;
D.normal_reuse_fastpath_hits=D.normal_reuse_fastpath_hits+node.normal_reuse_fastpath_hits;
D.normal_reuse_fastpath_check_ms=D.normal_reuse_fastpath_check_ms+node.normal_reuse_fastpath_check_ms;
D.max_fp_guard=max(D.max_fp_guard,node.max_fp_guard);

% Global swing/tension bounds are computed before obstacle certification.
% Do not replace them with a prefix-only value after an early return.
if isfinite(node.min_dynamic_slack) && node.min_dynamic_slack<D.min_dynamic_corridor_slack
    D.min_dynamic_corridor_slack=node.min_dynamic_slack;
    D.worst_cell=ci;
    D.worst_subcell=rr;
    D.worst_span=spanId;
    D.worst_component=node.worst_component;
    D.worst_obstacle_id=node.worst_obstacle_id;
end

if nodePass
    D.accepted_leaf_count=D.accepted_leaf_count+1;
    return;
end

% V50: BEFORE subdivision, try a continuous-time physical-geometry
% Lipschitz certificate on this entire leaf.  This is not a sampled-safe
% shortcut: acceptance still requires a lower bound valid for every t in
% [ta,tb].
D.early_lipschitz_attempts=D.early_lipschitz_attempts+1;
PA=b3_leaf_physical_lipschitz_audit( ...
    Ppos,Vphys,Aphys,Jphys,ta,tb,obs,P,CFG, ...
    node.worst_obstacle_id,node.worst_component);

if PA.continuous_lipschitz_certified
    D.early_lipschitz_certified=D.early_lipschitz_certified+1;
    D.accepted_leaf_count=D.accepted_leaf_count+1;
    return;
end

uMid=u0Global+0.5*(u1Global-u0Global);
tMid=ta+0.5*(tb-ta);

% Stop only when neither the curve parameter nor time can be represented
% with a distinct midpoint.  This is not a user-selected proof depth.
if ~(uMid>u0Global && uMid<u1Global && tMid>ta && tMid<tb)
    D.pass=false;
    D.unresolved_leaf_count=D.unresolved_leaf_count+1;
    D.worst_cell=ci;
    D.worst_subcell=rr;
    D.worst_span=spanId;
    D.worst_component=node.worst_component;
    D.worst_obstacle_id=node.worst_obstacle_id;
    D.failure_ta=ta;
    D.failure_tb=tb;
    D.failure_u0=u0Global;
    D.failure_u1=u1Global;
    D.limit_point_audit=PA;

    if PA.pointwise_collision_certified
        D.failure_class='TRUE_POINTWISE_DYNAMIC_COLLISION';
        D.failure_reason='POINTWISE_SREENATH_GEOMETRY_COLLISION';
    elseif PA.pointwise_strictly_safe
        D.failure_class='CONVEXIFICATION_OR_SEPARATOR_UNCERTAINTY';
        D.failure_reason='POINTWISE_SAFE_BUT_LIPSCHITZ_CERT_FAILED';
    else
        D.failure_class='BOUNDARY_OR_NUMERICAL_UNCERTAINTY';
        D.failure_reason='POINTWISE_CLEARANCE_WITHIN_BACKWARD_ERROR_GUARD';
    end
    return;
end

D.split_node_count=D.split_node_count+1;

nP=size(Ppos,2)-1;
nV=size(Vphys,2)-1;
nA=size(Aphys,2)-1;
nJ=size(Jphys,2)-1;

RPL=b3_bezier_restrict_map(nP,0,0.5);
RPR=b3_bezier_restrict_map(nP,0.5,1);
RVL=b3_bezier_restrict_map(nV,0,0.5);
RVR=b3_bezier_restrict_map(nV,0.5,1);
RAL=b3_bezier_restrict_map(nA,0,0.5);
RAR=b3_bezier_restrict_map(nA,0.5,1);
RJL=b3_bezier_restrict_map(nJ,0,0.5);
RJR=b3_bezier_restrict_map(nJ,0.5,1);

Pleft=Ppos*RPL.';
Pright=Ppos*RPR.';
Vleft=Vphys*RVL.';
Vright=Vphys*RVR.';
Aleft=Aphys*RAL.';
Aright=Aphys*RAR.';
Jleft=Jphys*RJL.';
Jright=Jphys*RJR.';

D=b3_adaptive_dynamic_separator_piece( ...
    D,Pleft,Vleft,Aleft,Jleft,ta,tMid,obs,ci,rr,spanId,P,CFG, ...
    depth+1,u0Global,uMid);
if ~D.pass, return; end

D=b3_adaptive_dynamic_separator_piece( ...
    D,Pright,Vright,Aright,Jright,tMid,tb,obs,ci,rr,spanId,P,CFG, ...
    depth+1,uMid,u1Global);
end


function A=b3_leaf_physical_lipschitz_audit( ...
    Ppos,Vphys,Aphys,Jphys,ta,tb,obs,P,CFG,obsId,component)
% Continuous physical-geometry audit for any failed support-certificate leaf.
%
% A negative pointwise clearance proves that the generated target trajectory
% is physically unsafe for the Sreenath geometry at that time.
%
% Pointwise clearance alone never accepts a leaf. Acceptance requires the
% continuous Lipschitz lower bound computed below.

A=struct( ...
    'valid',false, ...
    'component',component, ...
    'obstacle_id',obsId, ...
    'sample_tau',[0 0.5 1], ...
    'sample_time',nan(1,3), ...
    'payload_clearance',nan(1,3), ...
    'uav_clearance',nan(1,3), ...
    'cable_clearance',nan(1,3), ...
    'selected_clearance',nan(1,3), ...
    'min_selected_clearance',NaN, ...
    'clearance_guard',NaN, ...
    'pointwise_collision_certified',false, ...
    'pointwise_strictly_safe',false, ...
    'relative_speed_bound',NaN, ...
    'half_interval',NaN, ...
    'lipschitz_drop_bound',NaN, ...
    'continuous_lower_bound',NaN, ...
    'continuous_lipschitz_certified',false, ...
    'local_vL_bound',NaN, ...
    'local_jerk_bound',NaN, ...
    'local_rz_lb',NaN, ...
    'local_pdot_bound',NaN, ...
    'local_vQ_bound',NaN, ...
    'local_bounds_valid',false);

if isempty(obs)
    return;
end

ids=arrayfun(@(o)o.id,obs);
io=find(ids==obsId,1,'first');
if isempty(io)
    return;
end
ob0=obs(io);

tauSet=A.sample_tau;

for k=1:numel(tauSet)
    s=tauSet(k);
    t=ta+s*(tb-ta);
    A.sample_time(k)=t;

    xL=b3_bezier_eval_any(Ppos,s);
    aL=b3_bezier_eval_any(Aphys,s);

    r=aL+P.g*CFG.e3;
    rho=norm(r);
    scaleR=max([1;abs(r(:));P.g]);
    rhoGuard=128*eps(scaleR);
    if ~(isfinite(rho) && rho>rhoGuard)
        continue;
    end

    pCable=-r/rho;
    xQ=xL-P.L_cable*pCable;

    ob=ob0;
    ob.center=ob0.center+t*ob0.v_center;

    gL=aud_point_ellipsoid_distance(xL,ob)-(P.R_payload+P.d_margin);
    gQ=aud_point_ellipsoid_distance(xQ,ob)-(P.R_uav+P.d_margin);
    gC=aud_segment_ellipsoid_distance(xL,xQ,ob)-(P.R_cable+P.d_margin);

    A.payload_clearance(k)=gL;
    A.uav_clearance(k)=gQ;
    A.cable_clearance(k)=gC;

    switch upper(component)
        case 'PAYLOAD'
            A.selected_clearance(k)=gL;
        case 'UAV'
            A.selected_clearance(k)=gQ;
        case 'CABLE'
            A.selected_clearance(k)=gC;
        otherwise
            A.selected_clearance(k)=min([gL,gQ,gC]);
    end
end

vals=A.selected_clearance(isfinite(A.selected_clearance));
if isempty(vals)
    return;
end

A.valid=true;
A.min_selected_clearance=min(vals);

% Backward-error scale for pointwise classification. This is NOT a planning
% margin and is not used to accept a trajectory.
geomScale=max([ ...
    1, ...
    max(abs(Ppos(:))), ...
    max(abs(Aphys(:))), ...
    abs(ta),abs(tb), ...
    P.L_cable,P.g, ...
    P.R_payload+P.d_margin, ...
    P.R_uav+P.d_margin, ...
    P.R_cable+P.d_margin, ...
    max(abs(ob0.center(:))), ...
    max(abs(ob0.v_center(:))) ]);

u=eps/2;
kOps=128;
gamma=(kOps*u)/(1-kOps*u);
A.clearance_guard=gamma*geomScale + eps(geomScale);

A.pointwise_collision_certified=(A.min_selected_clearance < -A.clearance_guard);
A.pointwise_strictly_safe=(A.min_selected_clearance > A.clearance_guard);

% Continuous leaf certificate from 1-Lipschitz distance-to-convex-set property.
% All derivative bounds below are LEAF-LOCAL and come from Bezier control
% polygons that were formed once in physical time and then restricted
% directly through de Casteljau. No microscopic differentiation is used.

A.local_vL_bound=max(sqrt(sum(Vphys.^2,1)));
A.local_jerk_bound=max(sqrt(sum(Jphys.^2,1)));

Res=Aphys+P.g*CFG.e3;
rz=CFG.e3.'*Res;
A.local_rz_lb=min(rz);

scaleDyn=max([1;abs(Res(:));abs(Vphys(:));abs(Jphys(:));P.g]);
dynGuard=256*eps(scaleDyn);

if isfinite(A.local_rz_lb) && A.local_rz_lb>dynGuard && ...
        isfinite(A.local_vL_bound) && isfinite(A.local_jerk_bound)

    A.local_pdot_bound=A.local_jerk_bound/A.local_rz_lb;
    A.local_vQ_bound=A.local_vL_bound + P.L_cable*A.local_pdot_bound;
    A.local_bounds_valid=isfinite(A.local_vQ_bound);

    if A.local_bounds_valid
        vObs=norm(ob0.v_center);

        switch upper(component)
            case 'PAYLOAD'
                vComp=A.local_vL_bound;
            case 'UAV'
                vComp=A.local_vQ_bound;
            case 'CABLE'
                vComp=max(A.local_vL_bound,A.local_vQ_bound);
            otherwise
                vComp=max(A.local_vL_bound,A.local_vQ_bound);
        end

        A.relative_speed_bound=vComp+vObs;
        A.half_interval=0.5*abs(tb-ta);
        A.lipschitz_drop_bound=A.relative_speed_bound*A.half_interval;

        cMid=A.selected_clearance(2);
        A.continuous_lower_bound= ...
            cMid-A.lipschitz_drop_bound-A.clearance_guard;

        if isfinite(A.continuous_lower_bound)
            A.continuous_lipschitz_certified= ...
                (A.continuous_lower_bound>=0);
        end
    end
end
end


function [pass,N,cache]=b3_dynamic_separator_piece_test( ...
    Ppos,Aphys,ta,tb,obs,P,CFG,AUDCTX,cache)
N=struct('swing_bound_deg',NaN,'tension_per_mass_lb',NaN, ...
    'min_dynamic_slack',Inf,'max_fp_guard',0, ...
    'worst_component','','worst_obstacle_id',NaN, ...
    'failure_class','NONE','failure_reason','NONE', ...
    'socp_calls',0,'socp_runtime_ms',0, ...
    'solver_status_nonpositive_but_support_certified',0, ...
    'support_certified_count',0, ...
    'analytic_witness_checks',0, ...
    'analytic_witness_hits',0, ...
    'analytic_witness_check_ms',0, ...
    'normal_reuse_fastpath_checks',0, ...
    'normal_reuse_fastpath_hits',0, ...
    'normal_reuse_fastpath_check_ms',0, ...
    'broadphase_tests',0,'broadphase_rejects',0,'broadphase_runtime_ms',0, ...
    'runtime_records',b3_empty_dynamic_runtime_record_array());

Res=Aphys+P.g*CFG.e3;
if isempty(Res) || any(~isfinite(Res(:)))
    pass=false;
    N.failure_class='NUMERICAL_MODEL';
    N.failure_reason='NONFINITE_RESULTANT_CONTROL_POINT';
    N.worst_component='MODEL';
    return;
end

rz=CFG.e3.'*Res;
rxy=sqrt(sum(Res(1:2,:).^2,1));

scale=max([1;abs(Res(:));P.g]);
guard=256*eps(scale);
N.max_fp_guard=max(N.max_fp_guard,guard);

rzLB=min(rz);
N.tension_per_mass_lb=rzLB;

if rzLB<=guard
    pass=false;
    N.failure_class='DYNAMIC_MODEL_UNCERTIFIED';
    N.failure_reason='RESULTANT_CONE_NOT_POSITIVELY_VERTICAL';
    N.worst_component='TAUT';
    N.swing_bound_deg=180;
    return;
end

alpha=atan(max(rxy./rz));
N.swing_bound_deg=rad2deg(alpha);

components={'PAYLOAD','UAV','CABLE'};
pass=true;

for io=1:numel(obs)
    for ic=1:numel(components)
        comp=components{ic};

        if CFG.dynamic_sphere_broadphase
            N.broadphase_tests=N.broadphase_tests+1;
            qBP=tic;
            bpReject=b3_dynamic_forbidden_sphere_reject(Ppos,obs(io),ta,tb,P,comp);
            N.broadphase_runtime_ms=N.broadphase_runtime_ms+1e3*toc(qBP);
            if bpReject
                N.broadphase_rejects=N.broadphase_rejects+1;
                continue;
            end
        end

        % V56 first-stage certified analytic witness cascade.
        % Candidate generation is cheap; acceptance is still the exact
        % CURRENT direct support certificate.
        [analyticPass,nAnalytic,analyticSlack,analyticGuard, ...
            analyticIndex,analyticCheckMs,nAnalyticTried]= ...
            b3_try_analytic_dynamic_witnesses(Ppos,obs(io),ta,tb,P,comp,alpha);

        N.analytic_witness_checks=N.analytic_witness_checks+nAnalyticTried;
        N.analytic_witness_check_ms=N.analytic_witness_check_ms+analyticCheckMs;

        if analyticPass
            N.analytic_witness_hits=N.analytic_witness_hits+1;
            n=nAnalytic;
            bb=NaN;
            dist=NaN;
            ef=NaN;
            DD=struct();
            qms=0;
            slack=analyticSlack;
            fp=analyticGuard;

            N.max_fp_guard=max(N.max_fp_guard,fp);
            if slack<N.min_dynamic_slack
                N.min_dynamic_slack=slack;
                N.worst_component=comp;
                N.worst_obstacle_id=obs(io).id;
            end

            RREC=b3_make_dynamic_runtime_record( ...
                AUDCTX,obs(io),comp,ta,tb,alpha,size(Ppos,2), ...
                qms,ef,dist,DD,slack,fp,false,NaN,NaN, ...
                n,nan(3,1),false,'ANALYTIC_WITNESS_DIRECT_SUPPORT',false,0, ...
                true,analyticIndex,analyticCheckMs);
            N.runtime_records(end+1,1)=RREC; %#ok<AGROW>
            N.support_certified_count=N.support_certified_count+1;

            % Store this certified direction as the exact-key cache entry.
            cache=b3_dynamic_normal_cache_store(cache,obs(io),comp,n);
            continue;
        end

        % V55 certified fast-path: exact same obstacle/patch/component only.
        [hasPrev,nPrev]=b3_dynamic_normal_cache_lookup(cache,obs(io),comp);
        prevPass=false;
        prevSlack=NaN;
        prevGuard=NaN;
        reuseCheckMs=0;

        if hasPrev
            N.normal_reuse_fastpath_checks=N.normal_reuse_fastpath_checks+1;
            tr=tic;
            [prevSlack,prevGuard]=b3_direct_support_certificate( ...
                nPrev,Ppos,obs(io),ta,tb,P,comp,alpha);
            reuseCheckMs=1e3*toc(tr);
            N.normal_reuse_fastpath_check_ms=N.normal_reuse_fastpath_check_ms+reuseCheckMs;
            prevPass=isfinite(prevSlack) && isfinite(prevGuard) && ...
                (prevSlack>=-prevGuard);
        end

        if prevPass
            N.normal_reuse_fastpath_hits=N.normal_reuse_fastpath_hits+1;
            n=nPrev;
            bb=NaN;
            dist=NaN;
            ef=NaN;
            DD=struct();
            qms=0;
            slack=prevSlack;
            fp=prevGuard;

            N.max_fp_guard=max(N.max_fp_guard,fp);
            if slack<N.min_dynamic_slack
                N.min_dynamic_slack=slack;
                N.worst_component=comp;
                N.worst_obstacle_id=obs(io).id;
            end

            RREC=b3_make_dynamic_runtime_record( ...
                AUDCTX,obs(io),comp,ta,tb,alpha,size(Ppos,2), ...
                qms,ef,dist,DD,slack,fp,prevPass,prevSlack,prevGuard, ...
                n,nPrev,hasPrev,'REUSED_NORMAL_DIRECT_SUPPORT',true,reuseCheckMs, ...
                false,0,analyticCheckMs);
            N.runtime_records(end+1,1)=RREC; %#ok<AGROW>
            N.support_certified_count=N.support_certified_count+1;
            continue;
        end

        N.socp_calls=N.socp_calls+1;
        tq=tic;
        [n,bb,dist,ef,DD]=b3_primal_dynamic_component_separator( ...
            obs(io),Ppos,ta,tb,P,comp,alpha);
        qms=1e3*toc(tq);
        N.socp_runtime_ms=N.socp_runtime_ms+qms;

        if ~(all(isfinite(n)) && isfinite(bb) && isfinite(dist))
            pass=false;
            N.failure_class='DYNAMIC_SEPARATOR_NUMERICAL_FAILURE';
            N.failure_reason='NONFINITE_DYNAMIC_SEPARATOR';
            N.worst_component=comp;
            N.worst_obstacle_id=obs(io).id;
            return;
        end

        [slack,fp]=b3_direct_support_certificate( ...
            n,Ppos,obs(io),ta,tb,P,comp,alpha);
        N.max_fp_guard=max(N.max_fp_guard,fp);

        if slack<N.min_dynamic_slack
            N.min_dynamic_slack=slack;
            N.worst_component=comp;
            N.worst_obstacle_id=obs(io).id;
        end

        RREC=b3_make_dynamic_runtime_record( ...
            AUDCTX,obs(io),comp,ta,tb,alpha,size(Ppos,2), ...
            qms,ef,dist,DD,slack,fp,prevPass,prevSlack,prevGuard, ...
            n,nPrev,hasPrev,'SOCP_THEN_DIRECT_SUPPORT',false,reuseCheckMs, ...
            false,0,analyticCheckMs);
        N.runtime_records(end+1,1)=RREC; %#ok<AGROW>

        % Cache update is diagnostic-only in V54.  The current solved normal
        % remains the normal used by the unchanged hard certificate below.
        if all(isfinite(n))
            cache=b3_dynamic_normal_cache_store(cache,obs(io),comp,n);
        end

        % HARD AUTHORITY:
        % SOCP proposes a direction n only. Safety is certified independently
        % by the exact support inequality of the complete forbidden set.
        if slack>=-fp
            N.support_certified_count=N.support_certified_count+1;
            if ~(ef>0)
                N.solver_status_nonpositive_but_support_certified= ...
                    N.solver_status_nonpositive_but_support_certified+1;
            end
            continue;
        end

        pass=false;
        if ef>0
            N.failure_class='DYNAMIC_CONVEX_SET_NOT_SEPARATED';
            N.failure_reason='DIRECT_SUPPORT_CERTIFICATE_FAILED';
        else
            N.failure_class='DYNAMIC_SEPARATOR_CANDIDATE_OR_GEOMETRY_UNCERTIFIED';
            N.failure_reason='DIRECT_SUPPORT_FAILED_WITH_NONPOSITIVE_SOLVER_STATUS';
        end
        return;
    end
end
end


function R=b3_empty_dynamic_runtime_record()
R=struct( ...
    'cell',0,'subcell',0,'span',0,'depth',0,'u0',NaN,'u1',NaN, ...
    'ta',NaN,'tb',NaN,'dt',NaN, ...
    'obstacle_id',NaN,'patch_id',NaN,'component','', ...
    'alpha_rad',NaN,'n_control_points',0, ...
    'socp_ms',NaN,'exitflag',NaN,'distance',NaN, ...
    'primal_feasibility',NaN,'dual_feasibility',NaN,'relative_gap',NaN, ...
    'support_slack',NaN,'support_guard',NaN,'support_certified',false, ...
    'solver_nonpositive_but_support_certified',false, ...
    'certificate_path','', ...
    'socp_skipped_by_analytic_witness',false, ...
    'analytic_witness_index',0, ...
    'analytic_witness_check_ms',NaN, ...
    'socp_skipped_by_reuse',false, ...
    'reuse_check_ms',NaN, ...
    'prior_normal_available',false,'prior_normal_support_certified',false, ...
    'prior_normal_slack',NaN,'prior_normal_guard',NaN, ...
    'normal_angle_to_prior_deg',NaN);
end


function A=b3_empty_dynamic_runtime_record_array()
A=repmat(b3_empty_dynamic_runtime_record(),0,1);
end


function C=b3_empty_dynamic_normal_cache()
C=repmat(struct('obstacle_id',NaN,'patch_id',NaN,'component','', ...
    'normal',nan(3,1)),0,1);
end


function [tf,n]=b3_dynamic_normal_cache_lookup(C,ob,component)
tf=false;
n=nan(3,1);
pid=NaN;
if isfield(ob,'patch_id'), pid=ob.patch_id; end

for k=numel(C):-1:1
    if isequaln(C(k).obstacle_id,ob.id) && ...
            isequaln(C(k).patch_id,pid) && ...
            strcmpi(C(k).component,component)
        n=C(k).normal;
        tf=all(isfinite(n));
        return;
    end
end
end


function C=b3_dynamic_normal_cache_store(C,ob,component,n)
pid=NaN;
if isfield(ob,'patch_id'), pid=ob.patch_id; end

for k=1:numel(C)
    if isequaln(C(k).obstacle_id,ob.id) && ...
            isequaln(C(k).patch_id,pid) && ...
            strcmpi(C(k).component,component)
        C(k).normal=n(:);
        return;
    end
end

z=struct('obstacle_id',ob.id,'patch_id',pid, ...
    'component',upper(component),'normal',n(:));
C(end+1,1)=z;
end


function R=b3_make_dynamic_runtime_record( ...
    AUDCTX,ob,component,ta,tb,alpha,ncp,qms,ef,dist,DD, ...
    slack,guard,prevPass,prevSlack,prevGuard,n,nPrev,hasPrev, ...
    certificatePath,socpSkipped,reuseCheckMs, ...
    analyticSkipped,analyticIndex,analyticCheckMs)

R=b3_empty_dynamic_runtime_record();
R.cell=AUDCTX.cell;
R.subcell=AUDCTX.subcell;
R.span=AUDCTX.span;
R.depth=AUDCTX.depth;
R.u0=AUDCTX.u0;
R.u1=AUDCTX.u1;
R.ta=ta;
R.tb=tb;
R.dt=tb-ta;
R.obstacle_id=ob.id;
if isfield(ob,'patch_id'), R.patch_id=ob.patch_id; end
R.component=upper(component);
R.alpha_rad=alpha;
R.n_control_points=ncp;
R.socp_ms=qms;
R.exitflag=ef;
R.distance=dist;

if isstruct(DD)
    if isfield(DD,'primalfeasibility'), R.primal_feasibility=DD.primalfeasibility; end
    if isfield(DD,'dualfeasibility'), R.dual_feasibility=DD.dualfeasibility; end
    if isfield(DD,'relativegap'), R.relative_gap=DD.relativegap; end
end

R.support_slack=slack;
R.support_guard=guard;
R.support_certified=isfinite(slack) && isfinite(guard) && (slack>=-guard);
R.solver_nonpositive_but_support_certified= ...
    (~socpSkipped) && isfinite(ef) && (ef<=0) && R.support_certified;
R.certificate_path=certificatePath;
R.socp_skipped_by_analytic_witness=analyticSkipped;
R.analytic_witness_index=analyticIndex;
R.analytic_witness_check_ms=analyticCheckMs;
R.socp_skipped_by_reuse=socpSkipped;
R.reuse_check_ms=reuseCheckMs;

R.prior_normal_available=hasPrev;
R.prior_normal_support_certified=prevPass;
R.prior_normal_slack=prevSlack;
R.prior_normal_guard=prevGuard;

if hasPrev && all(isfinite(nPrev)) && all(isfinite(n))
    a=nPrev(:); b=n(:);
    na=norm(a); nb=norm(b);
    if na>0 && nb>0
        c=max(-1,min(1,(a.'*b)/(na*nb)));
        R.normal_angle_to_prior_deg=acosd(c);
    end
end
end


function S=b3_summarize_dynamic_runtime_audit(records,obs)
S=struct();
S.n_checks=numel(records);
S.n_analytic_hits=sum([records.socp_skipped_by_analytic_witness]);
S.n_reuse_hits=sum([records.socp_skipped_by_reuse]);
S.n_socp_calls=sum(~[records.socp_skipped_by_reuse] & ...
    ~[records.socp_skipped_by_analytic_witness]);
S.n_observed_patches=numel(obs);
if isempty(obs)
    S.n_physical_obstacle_ids=0;
else
    S.n_physical_obstacle_ids=numel(unique([obs.id]));
end

if isempty(records)
    S.total_socp_ms=0;
    S.total_reuse_check_ms=0;
    S.total_analytic_check_ms=0;
    S.med_socp_ms=NaN;
    S.p95_socp_ms=NaN;
    S.max_socp_ms=NaN;
    S.support_certified=0;
    S.solver_nonpos_but_certified=0;
    S.prior_available=0;
    S.prior_reuse_certified=0;
    S.prior_reuse_fraction=NaN;
    z=struct('checks',0,'analytic_hits',0,'socp_calls',0,'reuse_hits',0, ...
        'socp_ms',0,'support_certified',0,'prior_available',0, ...
        'prior_reuse_certified',0);
    S.by_component=struct('payload',z,'uav',z,'cable',z);
    S.top_keys={};
    return;
end

tm=[records.socp_ms];
reuseMs=[records.reuse_check_ms];
analyticMs=[records.analytic_witness_check_ms];
socpMask=~[records.socp_skipped_by_reuse] & ...
    ~[records.socp_skipped_by_analytic_witness];
S.total_socp_ms=sum(tm(socpMask),'omitnan');
S.total_reuse_check_ms=sum(reuseMs,'omitnan');
S.total_analytic_check_ms=sum(analyticMs,'omitnan');
socpTimes=tm(socpMask);
S.med_socp_ms=median(socpTimes,'omitnan');
S.p95_socp_ms=b3_percentile_local(socpTimes,95);
if isempty(socpTimes)
    S.max_socp_ms=NaN;
else
    S.max_socp_ms=max(socpTimes,[],'omitnan');
end
S.support_certified=sum([records.support_certified]);
S.solver_nonpos_but_certified= ...
    sum([records.solver_nonpositive_but_support_certified]);
S.prior_available=sum([records.prior_normal_available]);
S.prior_reuse_certified=sum([records.prior_normal_support_certified]);
if S.prior_available>0
    S.prior_reuse_fraction=S.prior_reuse_certified/S.prior_available;
else
    S.prior_reuse_fraction=NaN;
end

components={'PAYLOAD','UAV','CABLE'};
BC=struct();
for i=1:numel(components)
    c=components{i};
    mask=strcmpi({records.component},c);
    z=struct();
    z.checks=sum(mask);
    z.analytic_hits=sum(mask & [records.socp_skipped_by_analytic_witness]);
    z.socp_calls=sum(mask & socpMask);
    z.reuse_hits=sum(mask & [records.socp_skipped_by_reuse]);
    z.socp_ms=sum(tm(mask & socpMask),'omitnan');
    z.support_certified=sum([records(mask).support_certified]);
    z.prior_available=sum([records(mask).prior_normal_available]);
    z.prior_reuse_certified=sum([records(mask).prior_normal_support_certified]);
    BC.(lower(c))=z;
end
S.by_component=BC;

% Top exact obstacle-patch/component keys by call count/time.
keys=cell(numel(records),1);
for i=1:numel(records)
    keys{i}=sprintf('obs=%g patch=%g comp=%s', ...
        records(i).obstacle_id,records(i).patch_id,records(i).component);
end
[uk,~,ik]=unique(keys,'stable');
count=accumarray(ik,1);
socpCount=accumarray(ik,socpMask(:),[],@sum);
analyticCount=accumarray(ik,[records.socp_skipped_by_analytic_witness].',[],@sum);
reuseCount=accumarray(ik,[records.socp_skipped_by_reuse].',[],@sum);
time=accumarray(ik,(tm(:).*socpMask(:)),[],@sum);
[~,ord]=sortrows([-time(:),-count(:)],[1 2]);
K=min(10,numel(ord));
top=cell(K,1);
for j=1:K
    q=ord(j);
    top{j}=struct('key',uk{q},'checks',count(q), ...
        'analytic_hits',analyticCount(q),'socp_calls',socpCount(q), ...
        'reuse_hits',reuseCount(q),'socp_ms',time(q));
end
S.top_keys=top;
end


function q=b3_percentile_local(x,p)
x=x(isfinite(x));
if isempty(x)
    q=NaN;
    return;
end
x=sort(x(:));
if numel(x)==1
    q=x;
    return;
end
r=1+(p/100)*(numel(x)-1);
i=floor(r);
j=ceil(r);
if i==j
    q=x(i);
else
    q=x(i)+(r-i)*(x(j)-x(i));
end
end


function b3_print_dynamic_runtime_audit(S)
fprintf(['    [RUNTIME-AUDIT] patches=%d physicalObs=%d | checks=%d analyticHits=%d ', ...
    'reuseHits=%d actualSOCP=%d | SOCP total/med/p95/max=%.3f/%.3f/%.3f/%.3f ms ', ...
    '| analytic/reuseCheck=%.3f/%.3f ms\n'], ...
    S.n_observed_patches,S.n_physical_obstacle_ids,S.n_checks,S.n_analytic_hits, ...
    S.n_reuse_hits,S.n_socp_calls,S.total_socp_ms,S.med_socp_ms, ...
    S.p95_socp_ms,S.max_socp_ms,S.total_analytic_check_ms,S.total_reuse_check_ms);

b3_detail_fprintf(['    [RUNTIME-AUDIT] supportCert=%d/%d | solverNonPosButCert=%d | ', ...
    'priorNormal available=%d reuseWouldCertify=%d fraction=%.3f\n'], ...
    S.support_certified,S.n_checks,S.solver_nonpos_but_certified, ...
    S.prior_available,S.prior_reuse_certified,S.prior_reuse_fraction);

BC=S.by_component;
b3_detail_fprintf(['    [RUNTIME-AUDIT] byComponent checks/analytic/reuse/SOCP/time(ms): ', ...
    'PAYLOAD=%d/%d/%d/%d/%.3f UAV=%d/%d/%d/%d/%.3f CABLE=%d/%d/%d/%d/%.3f\n'], ...
    BC.payload.checks,BC.payload.analytic_hits,BC.payload.reuse_hits,BC.payload.socp_calls,BC.payload.socp_ms, ...
    BC.uav.checks,BC.uav.analytic_hits,BC.uav.reuse_hits,BC.uav.socp_calls,BC.uav.socp_ms, ...
    BC.cable.checks,BC.cable.analytic_hits,BC.cable.reuse_hits,BC.cable.socp_calls,BC.cable.socp_ms);

for k=1:numel(S.top_keys)
    z=S.top_keys{k};
    b3_detail_fprintf(['    [RUNTIME-AUDIT-TOP] #%d %s checks=%d analytic=%d reuse=%d ', ...
        'actualSOCP=%d socp=%.3f ms\n'], ...
        k,z.key,z.checks,z.analytic_hits,z.reuse_hits,z.socp_calls,z.socp_ms);
end
end


function b3_print_dynamic_runtime_audit_verbose(S)
if isempty(S) || ~isstruct(S) || ~isfield(S,'n_checks'), return; end

patches=b3_struct_num(S,'n_observed_patches',0);
physicalObs=b3_struct_num(S,'n_physical_obstacle_ids',0);
checks=b3_struct_num(S,'n_checks',0);
socpCalls=b3_struct_num(S,'n_socp_calls',0);
socpTotal=b3_struct_num(S,'total_socp_ms',0);
socpMed=b3_struct_num(S,'med_socp_ms',NaN);
socpP95=b3_struct_num(S,'p95_socp_ms',NaN);
socpMax=b3_struct_num(S,'max_socp_ms',NaN);
analyticHits=b3_struct_num(S,'n_analytic_hits',0);
reuseHits=b3_struct_num(S,'n_reuse_hits',0);
supportCert=b3_struct_num(S,'support_certified',0);
solverNonPos=b3_struct_num(S,'solver_nonpos_but_certified',0);

fprintf(['    [FAIL-DYNAMIC] patches=%d physicalObs=%d checks=%d SOCP=%d ', ...
    'socpTotal=%.3f ms med/p95/max=%.3f/%.3f/%.3f ms\n'], ...
    patches,physicalObs,checks,socpCalls,socpTotal,socpMed,socpP95,socpMax);
fprintf(['    [FAIL-DYNAMIC] analyticHits=%d reuseHits=%d supportCert=%d/%d ', ...
    'solverNonPosButCert=%d\n'], ...
    analyticHits,reuseHits,supportCert,checks,solverNonPos);

z=struct('checks',0,'socp_calls',0,'socp_ms',0);
P=z; U=z; C=z;
if isfield(S,'by_component') && isstruct(S.by_component)
    if isfield(S.by_component,'payload'), P=S.by_component.payload; end
    if isfield(S.by_component,'uav'), U=S.by_component.uav; end
    if isfield(S.by_component,'cable'), C=S.by_component.cable; end
end
fprintf(['    [FAIL-DYNAMIC-COMP] PAYLOAD checks=%d SOCP=%d %.3f ms | ', ...
    'UAV checks=%d SOCP=%d %.3f ms | CABLE checks=%d SOCP=%d %.3f ms\n'], ...
    b3_struct_num(P,'checks',0),b3_struct_num(P,'socp_calls',0),b3_struct_num(P,'socp_ms',0), ...
    b3_struct_num(U,'checks',0),b3_struct_num(U,'socp_calls',0),b3_struct_num(U,'socp_ms',0), ...
    b3_struct_num(C,'checks',0),b3_struct_num(C,'socp_calls',0),b3_struct_num(C,'socp_ms',0));

if isfield(S,'top_keys') && iscell(S.top_keys)
    K=min(10,numel(S.top_keys));
    for k=1:K
        zt=S.top_keys{k};
        if ~isstruct(zt), continue; end
        fprintf(['    [FAIL-DYNAMIC-TOP] #%d %s checks=%d analytic=%d reuse=%d ', ...
            'SOCP=%d socp=%.3f ms\n'], ...
            k,b3_struct_text(zt,'key','UNKNOWN'), ...
            b3_struct_num(zt,'checks',0),b3_struct_num(zt,'analytic_hits',0), ...
            b3_struct_num(zt,'reuse_hits',0),b3_struct_num(zt,'socp_calls',0), ...
            b3_struct_num(zt,'socp_ms',0));
    end
end
end


function v=b3_struct_num(S,name,default)
v=default;
if isstruct(S) && isfield(S,name)
    q=S.(name);
    if isnumeric(q) && isscalar(q)
        v=q;
    elseif islogical(q) && isscalar(q)
        v=double(q);
    end
end
end


function v=b3_struct_text(S,name,default)
v=default;
if isstruct(S) && isfield(S,name)
    q=S.(name);
    if ischar(q)
        v=q;
    elseif isstring(q) && isscalar(q)
        v=char(q);
    end
end
end



function [pass,nBest,slackBest,guardBest,idxBest,elapsedMs,nTried]= ...
    b3_try_analytic_dynamic_witnesses(Ppos,ob,ta,tb,P,component,alpha)
% Cheap candidate directions, exact hard authority.
%
% Fixed ±coordinate axes are precomputed once.  One additional direction
% points from the swept-obstacle center at interval midpoint toward the
% trajectory-hull centroid.  NONE of these directions is trusted by itself:
% every candidate is accepted only if b3_direct_support_certificate passes
% on the CURRENT leaf.

C=b3_precomputed_separator_normals();
cm=b3_obstacle_center_at_time(ob,0.5*(ta+tb));
cp=mean(Ppos,2);
d=cp-cm;
nd=norm(d);
if isfinite(nd) && nd>0
    C=[C,d/nd]; %#ok<AGROW>
end

pass=false;
nBest=nan(3,1);
slackBest=-Inf;
guardBest=Inf;
idxBest=0;
nTried=0;

tt=tic;
for k=1:size(C,2)
    n=C(:,k);
    nTried=nTried+1;
    [slack,guard]=b3_direct_support_certificate( ...
        n,Ppos,ob,ta,tb,P,component,alpha);

    if isfinite(slack) && isfinite(guard) && slack>=-guard
        pass=true;
        nBest=n;
        slackBest=slack;
        guardBest=guard;
        idxBest=k;
        break;
    end

    if slack>slackBest
        nBest=n;
        slackBest=slack;
        guardBest=guard;
        idxBest=k;
    end
end
elapsedMs=1e3*toc(tt);
end


function C=b3_precomputed_separator_normals()
% OFFLINE/FIXED candidate directions.  Computed once per MATLAB function
% load and reused by corridor and dynamic certificate stages.
persistent CACHED
if isempty(CACHED)
    E=eye(3);
    CACHED=[E,-E];
end
C=CACHED;
end


function c=b3_obstacle_center_at_time(ob,t)
c=ob.center(:);
if isfield(ob,'v_center') && ~isempty(ob.v_center)
    c=c+ob.v_center(:)*t;
end
end


function [pass,nBest,bbBest,slackBest,guardBest,idxBest,elapsedMs,nTried]= ...
    b3_try_analytic_corridor_witnesses(Seeds,ob,ta,tb,P,component)
% Corridor construction witness cascade.
% A candidate is accepted only when ALL construction seeds lie in the
% halfspace n'*x >= h_F(n), with a scale-aware floating guard.  This changes
% only separator candidate generation, not the downstream exact curve-hull
% safety authority.

C=b3_precomputed_separator_normals();
cm=b3_obstacle_center_at_time(ob,0.5*(ta+tb));
cs=mean(Seeds,2);
d=cs-cm;
nd=norm(d);
if isfinite(nd) && nd>0
    C=[C,d/nd]; %#ok<AGROW>
end

pass=false;
nBest=nan(3,1);
bbBest=NaN;
slackBest=-Inf;
guardBest=Inf;
idxBest=0;
nTried=0;

tt=tic;
for k=1:size(C,2)
    n=C(:,k);
    nTried=nTried+1;
    bb=b3_component_forbidden_support(ob,n,ta,tb,P,component);
    slack=min(n.'*Seeds-bb);

    scale=max([1;abs(Seeds(:));abs(ob.center(:));ob.radii(:);abs(bb)]);
    guard=256*eps(scale);

    if isfinite(slack) && isfinite(guard) && slack>=-guard
        pass=true;
        nBest=n;
        bbBest=bb;
        slackBest=slack;
        guardBest=guard;
        idxBest=k;
        break;
    end

    if slack>slackBest
        nBest=n;
        bbBest=bb;
        slackBest=slack;
        guardBest=guard;
        idxBest=k;
    end
end
elapsedMs=1e3*toc(tt);
end


function [slack,guard]=b3_direct_support_certificate( ...
    n,Ppos,ob,ta,tb,P,component,alpha)
% Independent continuous separation certificate.
%
% Exact arithmetic:
%   min_j n'*P_j - h_F(n) >= 0
% is sufficient because the Bezier curve is contained in conv{P_j}.
%
% Floating point:
% Use gamma_k = k*u/(1-k*u), u=eps/2, with k assembled from the actual
% elementary-operation counts of the 3D projection/support algebra.
% No solver residual and no user-tuned distance tolerance enters this hard
% certificate.

n=n(:);
proj=n.'*Ppos;
h=b3_dynamic_component_forbidden_support(ob,n,ta,tb,P,component,alpha);
slack=min(proj)-h;

u=eps/2;

% 3D dot product: 3 multiplies + 2 additions.
kDot=5;

% Support algebra:
% center dot 5 + velocity dot 5 + interval scaling/max 4 +
% S*n 15 + n'*(S*n) 5 + sqrt/max 2 +
% component support/norm/trig envelope 12.
kSupport=5+5+4+15+5+2+12;
kTotal=kDot+kSupport+2;

gamma=(kTotal*u)/(1-kTotal*u);

projScale=max(sum(abs(n).*abs(Ppos),1));
supportScale=b3_dynamic_support_scale_bound(ob,n,ta,tb,P,component,alpha);
scale=max([1,projScale,supportScale,abs(h),max(abs(proj))]);

guard=gamma*scale + eps(scale);

if ~(isfinite(slack) && isfinite(guard) && guard>=0)
    slack=-Inf;
    guard=Inf;
end
end


function s=b3_dynamic_support_scale_bound(ob,n,ta,tb,P,component,alpha)
% Absolute scale bound matching b3_dynamic_component_forbidden_support.
n=n(:);

centerTerm=sum(abs(n).*abs(ob.center(:)));
velDot=sum(abs(n).*abs(ob.v_center(:)));
moveTerm=max(abs(ta),abs(tb))*velDot;

Sabs=abs(ob.S);
ellQuad=sum(abs(n).*(Sabs*abs(n)));
ellTerm=sqrt(max(0,ellQuad));

baseScale=centerTerm+moveTerm+ellTerm;

Rp=P.R_payload+P.d_margin;
Ru=P.R_uav+P.d_margin;
Rc=P.R_cable+P.d_margin;
L=P.L_cable;
nn=norm(n);

switch upper(component)
    case 'PAYLOAD'
        offScale=Rp*nn;
    case 'UAV'
        offScale=(L+Ru)*nn;
    case 'CABLE'
        offScale=(L+Rc)*nn;
    otherwise
        error('STEP4B3:UnknownComponent','Unknown component %s.',component);
end

if ~(isfinite(alpha) && alpha>=0 && alpha<=pi)
    error('STEP4B3:DynamicSupportScale','Invalid alpha.');
end

s=baseScale+offScale;
end


function [nBest,bBest,dBest,exitflag,D]=b3_primal_dynamic_component_separator( ...
    ob,Seeds,ta,tb,P,component,alpha)
% Primal Euclidean distance between local payload-trajectory hull and the
% dynamic forbidden payload-center set.
%
% At alpha=0, the Sreenath direction cap is exactly p=-e3. Encoding this as
% ||p||<=1, p_3<=-1 has no strict interior. Because coneprog is an
% interior-point solver, V45 removes that degenerate SOC variable
% analytically. No angular epsilon or user-chosen threshold is introduced.
%
% For alpha>0, the convex p-cap remains SOC representable:
% ||p||<=1, e3'*p <= -cos(alpha).

component=upper(component);
Rp=P.R_payload+P.d_margin;
Ru=P.R_uav+P.d_margin;
Rc=P.R_cable+P.d_margin;
L=P.L_cable;

if ~(isfinite(alpha) && alpha>=0 && alpha<=pi)
    error('STEP4B3:DynamicSeparatorAlpha','alpha must be finite in [0,pi].');
end

ca=cos(alpha);
exactVertical=(alpha==0);
pHang=[0;0;-1];

m=size(Seeds,2);
c0=ob.center(:)+ta*ob.v_center(:);
obsDelta=(tb-ta)*ob.v_center(:);
moving=(norm(obsDelta)>0);

k=0;
iLamS=(k+1):(k+m); k=k+m;
if moving
    iObsAlpha=k+1; k=k+1;
else
    iObsAlpha=[];
end
iU=(k+1):(k+3); k=k+3;
iW=(k+1):(k+3); k=k+3;

iP=[];
iZ=[];
iCableLambda=[];

switch component
    case 'PAYLOAD'
        radius=Rp;

    case 'UAV'
        radius=Ru;
        if ~exactVertical
            iP=(k+1):(k+3); k=k+3;
        end

    case 'CABLE'
        radius=Rc;
        iCableLambda=k+1; k=k+1;
        if ~exactVertical
            iZ=(k+1):(k+3); k=k+3;
        end

    otherwise
        error('STEP4B3:UnknownComponent','Unknown component %s.',component);
end

iT=k+1; k=k+1;
nx=k;

f=zeros(nx,1);
f(iT)=1;

A=[];
b=[];
Aeq=zeros(1,nx);
Aeq(1,iLamS)=1;
beq=1;

lb=-inf(nx,1);
ub=inf(nx,1);
lb(iLamS)=0;
lb(iT)=0;

if moving
    lb(iObsAlpha)=0;
    ub(iObsAlpha)=1;
end
if ~isempty(iCableLambda)
    lb(iCableLambda)=0;
    ub(iCableLambda)=1;
end

if strcmp(component,'UAV') && ~exactVertical
    row=zeros(1,nx);
    row(iP(3))=1;
    A=[A;row]; %#ok<AGROW>
    b=[b;-ca]; %#ok<AGROW>
elseif strcmp(component,'CABLE') && ~exactVertical
    row=zeros(1,nx);
    row(iZ(3))=1;
    row(iCableLambda)=ca;
    A=[A;row]; %#ok<AGROW>
    b=[b;0]; %#ok<AGROW>
end

S=0.5*(ob.S+ob.S.');
[Ls,pchol]=chol(S,'lower');
if pchol~=0
    [V,E]=eig(S);
    E=max(E,0);
    Ls=V*sqrt(E);
end

% pS-pF = Dmat*x - cAffine.
Dmat=zeros(3,nx);
Dmat(:,iLamS)=Seeds;
if moving
    Dmat(:,iObsAlpha)=-obsDelta;
end
Dmat(:,iU)=-Ls;
Dmat(:,iW)=-radius*eye(3);

cAffine=c0;

if strcmp(component,'UAV')
    if exactVertical
        cAffine=c0+L*pHang;
    else
        Dmat(:,iP)=-L*eye(3);
    end
elseif strcmp(component,'CABLE')
    if exactVertical
        Dmat(:,iCableLambda)=-L*pHang;
    else
        Dmat(:,iZ)=-L*eye(3);
    end
end

dsoc=zeros(nx,1);
dsoc(iT)=1;
soc(1)=secondordercone(Dmat,cAffine,dsoc,0);

Au=zeros(3,nx);
Au(:,iU)=eye(3);
soc(end+1)=secondordercone(Au,zeros(3,1),zeros(nx,1),-1);

Aw=zeros(3,nx);
Aw(:,iW)=eye(3);
soc(end+1)=secondordercone(Aw,zeros(3,1),zeros(nx,1),-1);

if strcmp(component,'UAV') && ~exactVertical
    Ap=zeros(3,nx);
    Ap(:,iP)=eye(3);
    soc(end+1)=secondordercone(Ap,zeros(3,1),zeros(nx,1),-1);
elseif strcmp(component,'CABLE') && ~exactVertical
    Az=zeros(3,nx);
    Az(:,iZ)=eye(3);
    cz=zeros(nx,1);
    cz(iCableLambda)=1;
    soc(end+1)=secondordercone(Az,zeros(3,1),cz,0);
end

solverList={'normal-dense','augmented'};
attempts=repmat(struct('linear_solver','','runtime_ms',NaN,'exitflag',NaN, ...
    'primalfeasibility',NaN,'dualfeasibility',NaN,'dualitygap',NaN, ...
    'iterations',NaN),1,0);

x=[];
exitflag=NaN;
out=struct();

for ia=1:numel(solverList)
    opts=optimoptions('coneprog','Display','none', ...
        'ConstraintTolerance',1e-8,'OptimalityTolerance',1e-8, ...
        'MaxIterations',200,'LinearSolver',solverList{ia});
    qs=tic;
    [xt,~,eft,outt]=coneprog(f,soc,A,b,Aeq,beq,lb,ub,opts);
    rt=1e3*toc(qs);

    at=struct('linear_solver',solverList{ia},'runtime_ms',rt,'exitflag',eft, ...
        'primalfeasibility',NaN,'dualfeasibility',NaN,'dualitygap',NaN, ...
        'iterations',NaN);
    if isstruct(outt)
        if isfield(outt,'primalfeasibility'), at.primalfeasibility=outt.primalfeasibility; end
        if isfield(outt,'dualfeasibility'), at.dualfeasibility=outt.dualfeasibility; end
        if isfield(outt,'dualitygap'), at.dualitygap=outt.dualitygap; end
        if isfield(outt,'iterations'), at.iterations=outt.iterations; end
    end
    attempts(end+1)=at; %#ok<AGROW>

    x=xt;
    exitflag=eft;
    out=outt;

    if eft>0
        break;
    end
    if ~ismember(eft,[0,-7,-10])
        break;
    end
end

D=struct('primalfeasibility',NaN,'dualfeasibility',NaN,'dualitygap',NaN, ...
    'iterations',NaN,'attempts',attempts,'nvars',nx,'ncones',numel(soc), ...
    'component',component,'alpha',alpha,'exact_vertical_reduction',exactVertical);

if isstruct(out)
    if isfield(out,'primalfeasibility'), D.primalfeasibility=out.primalfeasibility; end
    if isfield(out,'dualfeasibility'), D.dualfeasibility=out.dualfeasibility; end
    if isfield(out,'dualitygap'), D.dualitygap=out.dualitygap; end
    if isfield(out,'iterations'), D.iterations=out.iterations; end
end

if isempty(x)
    nBest=[NaN;NaN;NaN];
    bBest=NaN;
    dBest=NaN;
    return;
end

pS=Seeds*x(iLamS);
pF=c0+Ls*x(iU)+radius*x(iW);

if moving
    pF=pF+x(iObsAlpha)*obsDelta;
end

if strcmp(component,'UAV')
    if exactVertical
        pF=pF+L*pHang;
    else
        pF=pF+L*x(iP);
    end
elseif strcmp(component,'CABLE')
    if exactVertical
        pF=pF+L*x(iCableLambda)*pHang;
    else
        pF=pF+L*x(iZ);
    end
end

deltaPF=pS-pF;
dEuclid=norm(deltaPF);

if ~(isfinite(dEuclid) && dEuclid>0)
    nBest=[NaN;NaN;NaN];
    bBest=NaN;
    dBest=0;
    D.euclidean_distance=dEuclid;
    D.support_gap=NaN;
    return;
end

nBest=deltaPF/dEuclid;
bBest=b3_dynamic_component_forbidden_support(ob,nBest,ta,tb,P,component,alpha);
dBest=min(nBest.'*Seeds)-bBest;

D.euclidean_distance=dEuclid;
D.support_gap=dBest;
end


function h=b3_dynamic_component_forbidden_support(ob,n,ta,tb,P,component,alpha)
n=n(:);
base=b3_swept_support(ob,n,ta,tb);
offset=b3_qcone_component_offset(n,P,component,alpha);
h=base+offset;
end



function d=b3_vertical_component_offset(n,P,component)
n=n(:);
switch upper(component)
    case 'PAYLOAD'
        d=(P.R_payload+P.d_margin)*norm(n);
    case 'UAV'
        d=-P.L_cable*n(3)+(P.R_uav+P.d_margin)*norm(n);
    case 'CABLE'
        d=max(0,-P.L_cable*n(3))+(P.R_cable+P.d_margin)*norm(n);
    otherwise
        error('STEP4B3:UnknownComponent','Unknown component %s.',component);
end
end


function d=b3_qcone_component_offset(n,P,component,alpha)
% Exact support offset of the FORBIDDEN PAYLOAD-CENTER set.
%
% q points UAV -> payload:
%   p_Q = p_L - L q.
% If p_Q intersects obstacle O, then
%   p_L belongs to O + L q.
% Thus the forbidden payload-center shift is +L*p.
%
% For q on a spherical cap centered at qc=-e3:
%   max_q n'*q =
%       ||n||                      if beta <= alpha
%       ||n|| cos(beta-alpha)      otherwise,
% beta = angle(n,qc).
%
% For a cable point p_C=p_L-lambda L q, lambda in [0,1], the forbidden
% payload-center shift is +lambda L q.

n=n(:);
nn=norm(n);
if nn==0
    d=0;
    return;
end

Rp=P.R_payload+P.d_margin;
Ru=P.R_uav+P.d_margin;
Rc=P.R_cable+P.d_margin;
L=P.L_cable;

switch upper(component)
    case 'PAYLOAD'
        d=Rp*nn;

    case {'UAV','CABLE'}
        qc=[0;0;-1];
        beta=acos(max(-1,min(1,(n.'*qc)/nn)));
        if beta<=alpha
            maxNq=nn;
        else
            maxNq=nn*cos(beta-alpha);
        end

        if strcmpi(component,'UAV')
            d=L*maxNq+Ru*nn;
        else
            d=L*max(0,maxNq)+Rc*nn;
        end

    otherwise
        error('STEP4B3:UnknownComponent','Unknown component %s.',component);
end
end



function D=b3_dense_rigid_cable_diagnostic(CP,M,T,P,CFG)
% Measurement-only diagnostic for the Sreenath cable direction reconstructed
% from payload acceleration. NOT a continuous-time safety authority.
D=struct('max_swing_deg',0,'min_tension_per_mass',Inf,'sample_count',0);
tauGrid=linspace(0,1,101);

np=size(CP,3);
n=size(CP,2)-1;

for iseg=1:np
    Pseg=CP(:,:,iseg);

    if isfield(M,'h') && numel(M.h)>=iseg
        frac=M.h(iseg);
    elseif isfield(M,'breaks') && numel(M.breaks)>=iseg+1
        frac=M.breaks(iseg+1)-M.breaks(iseg);
    else
        error('STEP4B3:DenseCableDiagnostic', ...
            'Cannot determine physical segment fraction for segment %d.',iseg);
    end

    dt=T*frac;
    if ~(isfinite(dt) && dt>0)
        error('STEP4B3:DenseCableDiagnostic','Invalid segment duration.');
    end

    A=n*(n-1)*diff(Pseg,2,2)/(dt^2);

    for k=1:numel(tauGrid)
        a=b3_bezier_eval_any(A,tauGrid(k));
        r=a+P.g*CFG.e3;
        rho=norm(r);

        if isfinite(rho) && rho>0
            pCable=-r/rho;
            c=max(-1,min(1,(-CFG.e3).'*pCable));
            D.max_swing_deg=max(D.max_swing_deg,acosd(c));
            D.min_tension_per_mass=min(D.min_tension_per_mass,rho);
        end

        D.sample_count=D.sample_count+1;
    end
end

if isinf(D.min_tension_per_mass)
    D.min_tension_per_mass=NaN;
end
end


function A=b3_dense_sreenath_model_diagnostic(CP,M,T,P,CFG)
% Measurement-only audit of Sreenath model quantities along the generated
% payload reference. Continuous obstacle safety authority remains the p-cone
% certificate; this diagnostic never replaces it.

A=struct( ...
    'max_swing_deg',0, ...
    'max_omega_rad_s',0, ...
    'max_required_thrust_N',NaN, ...
    'max_eq218_residual',NaN, ...
    'max_eq220_residual',NaN, ...
    'sample_count',0, ...
    'force_model_available',false);

np=size(CP,3);
tauGrid=linspace(0,1,101);

for iseg=1:np
    Pseg=CP(:,:,iseg);
    n=size(Pseg,2)-1;

    if isfield(M,'h') && numel(M.h)>=iseg
        frac=M.h(iseg);
    elseif isfield(M,'breaks') && numel(M.breaks)>=iseg+1
        frac=M.breaks(iseg+1)-M.breaks(iseg);
    else
        error('STEP4B3:SreenathDenseDiagnostic', ...
            'Cannot determine physical segment fraction for segment %d.',iseg);
    end

    dt=T*frac;
    if ~(isfinite(dt) && dt>0)
        error('STEP4B3:SreenathDenseDiagnostic','Invalid segment duration.');
    end

    % Physical-time Bezier control polygons for d0..d4.
    jets=cell(1,5);
    jets{1}=Pseg;
    Pr=Pseg;
    nr=n;
    for r=1:4
        if nr<1
            jets{r+1}=zeros(3,1);
        else
            Pr=nr*diff(Pr,1,2)/dt;
            nr=nr-1;
            jets{r+1}=Pr;
        end
    end

    for it=1:numel(tauGrid)
        tau=tauGrid(it);
        Djet=zeros(3,7);
        for r=0:4
            Djet(:,r+1)=b3_bezier_eval_any(jets{r+1},tau);
        end

        S=b3_sreenath_cable_state_from_jet(Djet,P,CFG);

        if S.valid
            A.max_swing_deg=max(A.max_swing_deg,S.swing_deg);
            A.max_omega_rad_s=max(A.max_omega_rad_s,norm(S.omega));

            if S.thrust_model_available
                A.force_model_available=true;

                if isnan(A.max_required_thrust_N)
                    A.max_required_thrust_N=0;
                    A.max_eq218_residual=0;
                    A.max_eq220_residual=0;
                end

                A.max_required_thrust_N=max( ...
                    A.max_required_thrust_N,S.required_thrust_N);
                A.max_eq218_residual=max( ...
                    A.max_eq218_residual,S.eq218_residual);
                A.max_eq220_residual=max( ...
                    A.max_eq220_residual,S.eq220_residual);
            end
        end

        A.sample_count=A.sample_count+1;
    end
end
end


function [vmax,amax,nEval]=b3_dense_realized_kinematics(CP,frac,T)
% Diagnostic-only dense evaluation. Continuous-time authority remains the
% analytic Bezier derivative-control-point convex-hull certificate.
n=size(CP,2)-1;
np=size(CP,3);
vmax=0;
amax=0;
nEval=0;
tauGrid=linspace(0,1,101);

for q=1:np
    dt=T*frac(q);
    P=CP(:,:,q);
    V=n*diff(P,1,2)/dt;
    if n>=2
        A=n*(n-1)*diff(P,2,2)/(dt^2);
    else
        A=zeros(3,1);
    end
    for it=1:numel(tauGrid)
        tau=tauGrid(it);
        vv=b3_bezier_eval_any(V,tau);
        aa=b3_bezier_eval_any(A,tau);
        vmax=max(vmax,norm(vv));
        amax=max(amax,norm(aa));
        nEval=nEval+1;
    end
end
end


function s=b3_num_or_unspecified(x)
if isnan(x)
    s='UNSPECIFIED';
else
    s=sprintf('%.9g',x);
end
end


function A=b3_empty_active()
A=struct('valid',false,'backend','','t_start',NaN,'T',NaN,'t_end',NaN, ...
    'reference_id',0,'result',struct(),'model',struct(),'geometry',struct(), ...
    'corridor',struct(),'known_ids',[],'risk_ids',[],'start_jet',nan(3,7), ...
    'end_jet',nan(3,7),'rejoin_nominal_phase',NaN,'post_nominal_time_shift',NaN,'yaw',struct());
end






function G=b3_guarantee_ledger(CFG,P)
% Explicit claim boundary for a TRAJECTORY-MODIFICATION-ONLY study.
%
% No runtime barrier/filter/controller safety mechanism is part of this
% validator.  Safety claims must therefore come from:
%   trajectory geometry + robust set inflation + rigid-cable feasibility.

G=struct();

hard={ ...
    'C6 reference continuity hard-certified in physical time with backward-error guards (plus structural C6 for degree-7 B-spline)', ...
    'observed-obstacle component-wise PAYLOAD/UAV/CABLE continuous separator certificate', ...
    'Sreenath single-towing cable direction p(t) on the taut hanging branch, induced by payload acceleration', ...
    'positive continuous lower bound on payload cable tension per payload mass ||xddot_L+g e3||', ...
    'continuous dynamic-separator certificate for Sreenath p-cone UAV center and finite cable segment', ...
    'observed-only planner memory with environment access restricted to sensor simulator', ...
    'recursive trajectory replanning when newly observed/model-changed threats trigger risk'};

if P.tracking_error_formally_bounded
    hard{end+1}=sprintf( ...
        'robust geometric obstacle inflation for certified tracking position error ||e_track||<=%.9g m', ...
        P.tracking_position_error_bound_m);
end

if P.obstacle_error_formally_bounded
    hard{end+1}=sprintf( ...
        'robust geometric obstacle inflation for certified obstacle-model position error ||e_obs||<=%.9g m', ...
        P.obstacle_position_error_bound_m);
end

if isfinite(CFG.mQ_kg) && isfinite(CFG.mL_kg)
    hard{end+1}= ...
        'Sreenath Eq.(2.18)/(2.20) required thrust-vector existence/reconstruction for supplied mQ,mL';
end

G.hard_guaranteed=hard;

G.diagnostic_only={ ...
    'dense realized velocity/acceleration samples', ...
    'dense rigid-cable swing samples', ...
    'advisory velocity/acceleration bounds when no physical limit is specified'};

notYet={ ...
    'camera self-occlusion by UAV/cable/payload using physical camera extrinsics', ...
    'continuous-time unknown-space safety between sensor observations', ...
    'continuous-time trigger between predictor samples', ...
    'vehicle actuation feasibility: thrust/moment/rotor saturation', ...
    'payload rigid-body attitude/oriented shape if payload is not spherical', ...
    'multi-UAV multi-cable coupled rigid-body payload dynamics', ...
    'mission completion beyond fixed validator wall-clock horizon', ...
    'real-time deadline compliance'};

if ~P.tracking_error_formally_bounded
    notYet{end+1}= ...
        'closed-loop tracking robustness: no certified tracking-position error bound supplied';
end
if ~P.obstacle_error_formally_bounded
    notYet{end+1}= ...
        'perception/model position robustness: no certified obstacle-position error bound supplied';
end

if ~(isfinite(CFG.mQ_kg) && isfinite(CFG.mL_kg))
    notYet{end+1}= ...
        'Sreenath Eq.(2.18)/(2.20) dimensional thrust reconstruction: actual mQ,mL not supplied';
end
notYet{end+1}= ...
    'Sreenath Eq.(2.21)/(2.22) full attitude/angular-velocity/moment feasibility and actuator limits';

G.not_yet_guaranteed=notYet;
G.trajectory_only=true;
G.runtime_safety_filter=false;
G.robust_margin_decomposition=struct( ...
    'geometry_margin_m',P.d_margin_geometry, ...
    'tracking_error_bound_m',CFG.tracking_position_error_bound_m, ...
    'obstacle_position_error_bound_m',CFG.obstacle_position_error_bound_m, ...
    'effective_total_margin_m',P.d_margin);

fprintf('[GUARANTEE-LEDGER] trajectoryOnly=1 hard=%d diagnostic=%d notYet=%d\n', ...
    numel(G.hard_guaranteed),numel(G.diagnostic_only),numel(G.not_yet_guaranteed));
end



function b3_rigid_cable_model_precheck(P,CFG)
% Sreenath model identity precheck at nominal hover and at a synthetic
% nonzero-jerk/snap jet. This is an algebraic implementation check, not a
% substitute for mission-time feasibility checks.

D=zeros(3,7);
D(:,1)=[0;0;0];
S=b3_sreenath_cable_state_from_jet(D,P,CFG);

guardP=128*eps(max(1,P.L_cable));
guardG=128*eps(P.g);

if ~(S.valid && norm(S.p+CFG.e3)<=guardP && ...
        norm(S.xQ-(D(:,1)+P.L_cable*CFG.e3))<=guardP && ...
        abs(S.payload_tension_per_mass-P.g)<=guardG && ...
        norm(S.pdot)<=guardP && norm(S.omega)<=guardP)
    error('STEP4B3:SreenathPrecheck','Nominal Sreenath cable identity failed.');
end

% Synthetic smooth jet verifies pdot = omega x p and p'pdot = 0.
D2=zeros(3,7);
D2(:,1)=[1;-2;0.5];
D2(:,3)=[0.8;-0.4;0.2];
D2(:,4)=[0.3;0.1;-0.2];
D2(:,5)=[-0.2;0.15;0.05];
S2=b3_sreenath_cable_state_from_jet(D2,P,CFG);

if ~S2.valid
    error('STEP4B3:SreenathPrecheck','Synthetic Sreenath cable state invalid.');
end

rTang=abs(S2.p.'*S2.pdot);
rKin=norm(S2.pdot-cross(S2.omega,S2.p));
rX=norm(S2.xQ-(D2(:,1)-P.L_cable*S2.p));

scale=max([1;norm(S2.pdot);norm(S2.omega);P.L_cable]);
guard=1024*eps(scale);

if rTang>guard || rKin>guard || rX>guard
    error('STEP4B3:SreenathPrecheck', ...
        'Sreenath kinematic identities failed: tangent=%g kin=%g xQ=%g guard=%g.', ...
        rTang,rKin,rX,guard);
end

if S2.thrust_model_available
    forceScale=max([1;norm(S2.thrust_vector);CFG.mQ_kg*P.L_cable*norm(S2.omegadot)]);
    forceGuard=4096*eps(forceScale);
    if S2.eq218_residual>forceGuard || S2.eq220_residual>forceGuard
        error('STEP4B3:SreenathPrecheck', ...
            'Eq.(2.18)/(2.20) reconstruction failed: r218=%g r220=%g guard=%g.', ...
            S2.eq218_residual,S2.eq220_residual,forceGuard);
    end
end

fprintf(['[PRECHECK] Sreenath model PASS | L=%.6g m | nominal p=[%.3g %.3g %.3g] | ', ...
    'T_payload/mL=%.9g m/s^2 | masses=%s\n'], ...
    P.L_cable,S.p(1),S.p(2),S.p(3),S.payload_tension_per_mass, ...
    b3_sreenath_mass_status(CFG));
end


function s=b3_sreenath_mass_status(CFG)
if isfinite(CFG.mQ_kg) && isfinite(CFG.mL_kg)
    s=sprintf('mQ=%.9g kg,mL=%.9g kg',CFG.mQ_kg,CFG.mL_kg);
else
    s='UNSPECIFIED(force reconstruction disabled)';
end
end


function b3_event_schema_precheck()
% Canonical event-template self-check.
SE=b3_empty_sensor_event();
RE=b3_empty_replan_event();

assert(isstruct(SE)&&isscalar(SE),'STEP4B3:SensorEventTemplateScalar');
assert(isstruct(RE)&&isscalar(RE),'STEP4B3:ReplanEventTemplateScalar');

requiredReplan={ ...
    't','backend','source_reference_id','proposed_reference_id', ...
    'solver_feasible','backend_pass','min_original_hard_slack', ...
    'row_numeric_certified','c6_structural_certified', ...
    'accepted','failure_reason'};

missing=setdiff(requiredReplan,fieldnames(RE),'stable');
if ~isempty(missing)
    error('STEP4B3:ReplanEventTemplateMissing', ...
        'Canonical replan event template missing fields: %s',strjoin(missing,','));
end

fprintf('[PRECHECK] event schemas PASS | sensorFields=%d | replanFields=%d\n', ...
    numel(fieldnames(SE)),numel(fieldnames(RE)));
end


function S=b3_enforce_struct_schema(S,T,label)
% Canonicalize a scalar event struct against a template before struct-array
% append. This prevents MATLAB's "subscripted assignment between dissimilar
% structures" failure and makes a future schema regression fail locally with
% an explicit missing/extra-field report.
if ~(isstruct(S) && isscalar(S) && isstruct(T) && isscalar(T))
    error('STEP4B3:EventSchemaScalar', ...
        '%s and its template must both be scalar structs.',label);
end

fs=fieldnames(S);
ft=fieldnames(T);

missing=setdiff(ft,fs,'stable');
extra=setdiff(fs,ft,'stable');

if ~isempty(missing) || ~isempty(extra)
    error('STEP4B3:EventSchemaMismatch', ...
        '%s schema mismatch. missing=%s extra=%s', ...
        label,strjoin(missing,','),strjoin(extra,','));
end

% Match canonical field order too, so array assignment is deterministic.
S=orderfields(S,T);
end


function S=b3_empty_sensor_event()
S=struct('t',NaN,'visible_ids',[],'new_ids',[],'changed_ids',[], ...
    'known_before',[],'known_after',[], ...
    'reference_id',0,'active_avoidance',false,'sensor_runtime_ms',NaN, ...
    'sensor_environment_ms',NaN,'sensor_roi_ms',NaN,'sensor_rayprep_ms',NaN, ...
    'sensor_raycast_ms',NaN,'sensor_pack_ms',NaN,'memory_update_ms',NaN, ...
    'predictor_runtime_ms',NaN,'predictor_skipped_certified_active',false, ...
    'risk_found',false,'risk_ids',[],'risk_ttc_s',NaN, ...
    'cable_swing_deg',NaN,'cable_payload_tension_per_mass',NaN);
end


function E=b3_empty_replan_event()
E=struct('t',NaN,'backend','','source_reference_id',0,'proposed_reference_id',0, ...
    'known_ids',[],'risk_ids',[],'risk_ttc_s',NaN,'start_physical_jet',nan(3,7), ...
    'end_physical_jet',nan(3,7),'T',NaN,'t_end',NaN,'geometry',struct(), ...
    'corridor',struct(),'geometry_diagnostic',struct(), ...
    'required_mean_guide_speed',NaN,'required_mean_speed_ratio',NaN, ...
    'solver_feasible',false,'backend_pass',false,'exitflag',NaN,'min_original_hard_slack',-Inf, ...
    'start_spatial_jet_residual_d0_d6',nan(1,7),'end_spatial_jet_residual_d0_d6',nan(1,7), ...
    'start_physical_splice_residual_d0_d6',nan(1,7),'end_physical_residual_d0_d6',nan(1,7), ...
    'c6_internal_residual',NaN,'dense_exact_ellipsoid_slack',NaN, ...
    'deferred_dense_exact_ellipsoid_diag_ms',NaN, ...
    'deferred_dense_exact_ellipsoid_samples',0, ...
    'c6_internal_physical_residual_d0_d6',nan(1,7), ...
    'c6_internal_guard_d0_d6',nan(1,7), ...
    'start_physical_splice_guard_d0_d6',nan(1,7), ...
    'end_physical_guard_d0_d6',nan(1,7), ...
    'c6_internal_backward_error_certified',false, ...
    'start_physical_backward_error_certified',false, ...
    'end_physical_backward_error_certified',false, ...
    'c6_hard_certified',false, ...
    'row_numeric_certified',false,'c6_structural_certified',false, ...
    'build_time_ms',NaN,'solve_time_ms',NaN,'plan_runtime_ms',NaN,'profile',struct(), ...
    'start_yaw_physical_jet',nan(1,7),'end_yaw_physical_jet',nan(1,7), ...
    'yaw_start_splice_residual_d0_d6',nan(1,7),'yaw_end_residual_d0_d6',nan(1,7), ...
    'yaw_start_splice_guard_d0_d6',nan(1,7),'yaw_end_guard_d0_d6',nan(1,7), ...
    'yaw_c6_hard_certified',false, ...
    'yaw_constraint_backward_error_certified',false, ...
    'yaw_constraint_residual_inf',NaN,'yaw_constraint_guard_inf',NaN, ...
    'yaw_fit_max_abs_rad',NaN,'kinematic',struct(),'cable_dynamic',struct(), ...
    'schedule',struct(),'accepted',false,'failure_reason','NOT_RUN');
end

function frac=b2_d13_segment_fraction(G)
L=vecnorm(diff(G.waypoints,1,2),2,1);
if any(~isfinite(L)) || any(L<=0)
    error('STEP4B2:SegmentLength','All guide segments must be finite and positive.');
end
frac=L/sum(L);
frac=frac(:).';
end


function J_s=b2_physical_to_spatial_jet(J_t,T)
if ~isequal(size(J_t),[3 7])
    error('STEP4B2:JetShape','Expected 3x7 jet.');
end
J_s=J_t;
Tp=1;
for k=0:6
    if k>0, Tp=Tp*T; end
    J_s(:,k+1)=J_t(:,k+1)*Tp;
end
end


function J_t=b2_spatial_to_physical_jet(J_s,T)
if ~isequal(size(J_s),[3 7])
    error('STEP4B2:JetShape','Expected 3x7 jet.');
end
J_t=J_s;
invTp=1;
for k=0:6
    if k>0, invTp=invTp/T; end
    J_t(:,k+1)=J_s(:,k+1)*invTp;
end
end


function R=b2_solve_d13_fixed_boundary(G,C,J0,J1,obs,P,Tnew)
tBuild=tic;
M=b2_d13_build_model_fixed_boundary(G,C,J0,J1);
buildMs=1e3*toc(tBuild);

qpopt=optimoptions('quadprog', ...
    'Algorithm','interior-point-convex', ...
    'Display','none', ...
    'ConstraintTolerance',1e-10, ...
    'OptimalityTolerance',1e-10, ...
    'StepTolerance',1e-12, ...
    'MaxIterations',500);

tSolve=tic;
[z,ef,out,cgdiag]=b3_qp_constraint_generation( ...
    M.Hscaled,M.fscaled,M.Aineq,M.bineq,qpopt,'D13');
solveMs=1e3*toc(tSolve);
out.constraint_generation=cgdiag;

R=b2_empty_result();
R.build_time_ms=buildMs;
R.solve_time_ms=solveMs;
R.exitflag=ef;
R.solver_output=out;
R.n_decision_variables=M.nvar;
R.n_ineq=size(M.Aineq,1);
R.n_eq=0;

if isempty(z) || ef<=0
    D=b3_infeasibility_audit(M.Aineq,M.bineq,M.safeMeta,'D13');
    R.solver_output.infeasibility_audit=D;
    b3_print_infeasibility_audit(D);
    return;
end

R.solver_feasible=true;

qPost=tic;
CP=b2_d13_reconstruct_cp(z,M);
R.control_points=CP;
R.post_reconstruct_ms=1e3*toc(qPost);

qPost=tic;
sl=M.bineq-M.Aineq*z;
R.min_original_hard_slack=min(sl);
R.post_slack_ms=1e3*toc(qPost);

qPost=tic;
[Jstart,Jend,c6res]=b2_d13_audit_jets(CP,M);
R.start_spatial_jet_residual_d0_d6=b2_jet_residual_by_order(Jstart,J0);
R.end_spatial_jet_residual_d0_d6=b2_jet_residual_by_order(Jend,J1);
R.c6_internal_residual=c6res;

Jstart_t=b2_spatial_to_physical_jet(Jstart,Tnew);
Jend_t=b2_spatial_to_physical_jet(Jend,Tnew);
J0t=b2_spatial_to_physical_jet(J0,Tnew);
J1t=b2_spatial_to_physical_jet(J1,Tnew);

R.start_physical_splice_residual_d0_d6=b2_jet_residual_by_order(Jstart_t,J0t);
R.end_physical_residual_d0_d6=b2_jet_residual_by_order(Jend_t,J1t);

dtSeg=Tnew*M.h;
C6A=b3_piecewise_bezier_c6_hard_audit(CP,dtSeg,J0t,J1t);
R.start_physical_splice_residual_d0_d6=C6A.start_residual_d0_d6;
R.start_physical_splice_guard_d0_d6=C6A.start_guard_d0_d6;
R.start_physical_backward_error_certified=C6A.start_certified;
R.end_physical_residual_d0_d6=C6A.end_residual_d0_d6;
R.end_physical_guard_d0_d6=C6A.end_guard_d0_d6;
R.end_physical_backward_error_certified=C6A.end_certified;
R.c6_internal_physical_residual_d0_d6=C6A.internal_residual_d0_d6;
R.c6_internal_guard_d0_d6=C6A.internal_guard_d0_d6;
R.c6_internal_backward_error_certified=C6A.internal_certified;
R.c6_hard_certified=C6A.all_certified;
R.post_c6_audit_ms=1e3*toc(qPost);

% V58 measurement-only diagnostic is deferred until qPlan has stopped.
R.dense_exact_ellipsoid_slack=NaN;
R.dense_exact_ellipsoid_diag_ms=NaN;
R.deferred_dense_exact_ellipsoid_diag=true;
R.sample_s=[];
R.sample_position=[];

R.pass=R.solver_feasible && ...
       (R.min_original_hard_slack>=0) && ...
       all(isfinite(R.start_spatial_jet_residual_d0_d6)) && ...
       all(isfinite(R.end_spatial_jet_residual_d0_d6)) && ...
       R.start_physical_backward_error_certified && ...
       R.end_physical_backward_error_certified && ...
       R.c6_internal_backward_error_certified && ...
       R.c6_hard_certified;
end


function M=b2_d13_build_model_fixed_boundary(G,C,J0,J1)
n=13;
nseg=3;
ncp=n+1;

W=G.waypoints;
h=b2_d13_segment_fraction(G);

% Two internal knots x 7 derivative orders x 3 axes.
nvar=42;

L=max(1,G.path_length);
jetScale=zeros(2,7);
for kk=1:2
    href=sqrt(h(kk)*h(kk+1));
    for r=0:6
        ff=factorial(n)/factorial(n-r);
        jetScale(kk,r+1)=L*ff/(href^r);
    end
end

ncpall=3*nseg*ncp;
Gcp=zeros(ncpall,nvar);
c0=zeros(ncpall,1);

E=bez13_endpoint_difference_inverse_direct(n);

for q=1:nseg
    for ax=1:3
        D0=zeros(14,1);
        Dz=zeros(14,nvar);

        [D0,Dz]=b2_d13_insert_side(D0,Dz,1,q,h(q),ax,J0,J1,jetScale,n);
        [D0,Dz]=b2_d13_insert_side(D0,Dz,8,q+1,h(q),ax,J0,J1,jetScale,n);

        cp0=E*D0;
        cpG=E*Dz;

        ids=arrayfun(@(j)bez13_jet_cpidx(q,j,ax,ncp),1:ncp);
        c0(ids)=cp0;
        Gcp(ids,:)=cpG;
    end
end

Aineq=zeros(0,nvar);
bineq=zeros(0,1);
safeMeta=struct('segment',{},'subcell',{},'cp',{},'plane',{});

for q=1:nseg
    leaves=C.cells{q}.subcells;
    for rr=1:numel(leaves)
        CC=leaves(rr);
        Rloc=b3_bezier_restrict_map(n,CC.u0,CC.u1);

        for j=1:ncp
            for pl=1:size(CC.A,1)
                a=zeros(1,ncpall);
                for k=1:ncp
                    w=Rloc(j,k);
                    if w==0, continue; end
                    for ax=1:3
                        a(bez13_jet_cpidx(q,k,ax,ncp)) = ...
                            a(bez13_jet_cpidx(q,k,ax,ncp)) - w*CC.A(pl,ax);
                    end
                end
                Aineq(end+1,:)=a*Gcp; %#ok<AGROW>
                bineq(end+1,1)=-CC.b(pl)-a*c0; %#ok<AGROW>
                safeMeta(end+1)=struct('segment',q,'subcell',rr, ...
                    'cp',j,'plane',pl); %#ok<AGROW>
            end
        end
    end
end

target=zeros(ncpall,1);
for q=1:nseg
    for j=1:ncp
        tau=(j-1)/(ncp-1);
        pt=(1-tau)*W(:,q)+tau*W(:,q+1);
        for ax=1:3
            target(bez13_jet_cpidx(q,j,ax,ncp))=pt(ax);
        end
    end
end

H=2*(Gcp.'*Gcp);
H=0.5*(H+H.');
f=2*Gcp.'*(c0-target);

d=max(abs(diag(H)));
if ~(d>0), d=1; end

M=struct();
M.degree=n;
M.nseg=nseg;
M.ncp=ncp;
M.nvar=nvar;
M.h=h;
M.jetScale=jetScale;
M.Gcp=Gcp;
M.c0=c0;
M.Aineq=sparse(Aineq);
M.bineq=bineq;
M.safeMeta=safeMeta;
M.Hscaled=sparse(H/d);
M.fscaled=f/d;
end


function [D0,Dz]=b2_d13_insert_side(D0,Dz,row0,knot,hseg,ax,J0,J1,jetScale,n)
for r=0:6
    ff=factorial(n)/factorial(n-r);
    mult=(hseg^r)/ff;
    row=row0+r;

    if knot==1
        D0(row)=mult*J0(ax,r+1);

    elseif knot==4
        D0(row)=mult*J1(ax,r+1);

    else
        kk=knot-1;
        vidx=(kk-1)*21+r*3+ax;
        Dz(row,vidx)=mult*jetScale(kk,r+1);
    end
end
end


function CP=b2_d13_reconstruct_cp(z,M)
v=M.c0+M.Gcp*z;
CP=zeros(3,M.ncp,M.nseg);
for q=1:M.nseg
    for ax=1:3
        for j=1:M.ncp
            CP(ax,j,q)=v(bez13_jet_cpidx(q,j,ax,M.ncp));
        end
    end
end
end


function A=b3_piecewise_bezier_c6_hard_audit(CP,dtSeg,JstartTarget,JendTarget)
% Hard numerical C6 audit in PHYSICAL TIME for a piecewise Bezier curve.
%
% Each endpoint derivative is a short signed linear combination of control
% points multiplied by n!/(n-r)! / dt^r.  The guard uses a standard
% gamma_k = k*u/(1-k*u), u=eps/2, with k counted from the actual evaluation:
%
%   r+1 coefficient multiplications
%   r   additions
%   1   final scale multiplication
%   r+1 operations for dt^r and the division/scale formation
%
% plus the final subtraction used to form a continuity residual.
%
% No user-chosen C6 tolerance appears here.

if ndims(CP)~=3
    error('STEP4B3:C6AuditShape','CP must be dim x ncp x nseg.');
end
nseg=size(CP,3);
if numel(dtSeg)~=nseg || any(~isfinite(dtSeg)) || any(dtSeg<=0)
    error('STEP4B3:C6AuditTime','Need one positive finite physical duration per segment.');
end

A=struct();
A.start_residual_d0_d6=nan(1,7);
A.start_guard_d0_d6=nan(1,7);
A.start_certified=false;
A.end_residual_d0_d6=nan(1,7);
A.end_guard_d0_d6=nan(1,7);
A.end_certified=false;
A.internal_residual_d0_d6=zeros(1,7);
A.internal_guard_d0_d6=zeros(1,7);
A.internal_certified=true;
A.all_certified=false;

[J0,G0]=b3_bezier_endpoint_jet_with_guard(CP(:,:,1),dtSeg(1),'left');
[J1,G1]=b3_bezier_endpoint_jet_with_guard(CP(:,:,end),dtSeg(end),'right');

[A.start_residual_d0_d6,A.start_guard_d0_d6,A.start_certified]= ...
    b3_jet_target_match_certificate(J0,G0,JstartTarget);
[A.end_residual_d0_d6,A.end_guard_d0_d6,A.end_certified]= ...
    b3_jet_target_match_certificate(J1,G1,JendTarget);

for q=1:(nseg-1)
    [JL,GL]=b3_bezier_endpoint_jet_with_guard(CP(:,:,q),dtSeg(q),'right');
    [JR,GR]=b3_bezier_endpoint_jet_with_guard(CP(:,:,q+1),dtSeg(q+1),'left');

    [r,g,c]=b3_jet_pair_match_certificate(JL,GL,JR,GR);
    A.internal_residual_d0_d6=max(A.internal_residual_d0_d6,r);
    A.internal_guard_d0_d6=max(A.internal_guard_d0_d6,g);
    A.internal_certified=A.internal_certified && c;
end

A.all_certified=A.start_certified && A.end_certified && A.internal_certified;
end


function [J,G]=b3_bezier_endpoint_jet_with_guard(P,dt,side)
% Endpoint physical-time derivatives d0..d6 plus componentwise IEEE-754
% forward-error guards. Works for 1D yaw and 3D position identically.

n=size(P,2)-1;
dim=size(P,1);
if n<6
    error('STEP4B3:C6AuditDegree','C6 audit requires Bezier degree >= 6.');
end
if ~(isfinite(dt) && dt>0)
    error('STEP4B3:C6AuditTime','Physical segment duration must be positive finite.');
end

J=zeros(dim,7);
G=zeros(dim,7);
u=eps/2;

for r=0:6
    ff=factorial(n)/factorial(n-r);
    c=zeros(1,r+1);

    if strcmp(side,'left')
        for k=0:r
            c(k+1)=((-1)^(r-k))*nchoosek(r,k);
        end
        X=P(:,1:r+1);
    elseif strcmp(side,'right')
        for k=0:r
            c(k+1)=((-1)^(r-k))*nchoosek(r,k);
        end
        X=P(:,n-r+1:n+1);
    else
        error('STEP4B3:C6AuditSide','Unknown endpoint side.');
    end

    scale=ff/(dt^r);
    J(:,r+1)=scale*(X*c.');

    % Count the actual floating operations used by this endpoint evaluator.
    % Coefficients and factorial ratios are small exact integers here.
    kLinear=(r+1)+r+1;  % coefficient products + additions + final scale
    kScale=r+1;         % dt^r construction + division/scale formation
    kTotal=kLinear+kScale;
    gamma=(kTotal*u)/(1-kTotal*u);

    absEvalScale=abs(scale)*(abs(X)*abs(c.'));
    G(:,r+1)=gamma*absEvalScale + eps(max(1,abs(J(:,r+1))));
end
end


function [resOrder,guardOrder,cert]=b3_jet_pair_match_certificate(JL,GL,JR,GR)
if ~isequal(size(JL),size(JR),size(GL),size(GR))
    error('STEP4B3:C6AuditShape','Jet/guard sizes must agree.');
end

u=eps/2;
R=abs(JL-JR);
subGuard=u*(abs(JL)+abs(JR));
GC=GL+GR+subGuard;

cert=all(R(:)<=GC(:));
resOrder=sqrt(sum(R.^2,1));
guardOrder=sqrt(sum(GC.^2,1));
end


function [resOrder,guardOrder,cert]=b3_jet_target_match_certificate(J,G,target)
% Certifies the generated endpoint against the supplied physical-time jet
% represented by the target doubles.  The guarantee is relative to that
% supplied jet data, which is exactly the quantity preserved at a replan.

if ~isequal(size(J),size(G),size(target))
    error('STEP4B3:C6AuditShape','Endpoint jet, guard and target sizes must agree.');
end

u=eps/2;
R=abs(J-target);
subGuard=u*(abs(J)+abs(target));
GC=G+subGuard;

cert=all(R(:)<=GC(:));
resOrder=sqrt(sum(R.^2,1));
guardOrder=sqrt(sum(GC.^2,1));
end


function [Jstart,Jend,c6res]=b2_d13_audit_jets(CP,M)
Jstart=b2_bezier_endpoint_jet(CP(:,:,1),M.h(1),'left');
Jend=b2_bezier_endpoint_jet(CP(:,:,end),M.h(end),'right');

c6res=0;
for q=1:(M.nseg-1)
    JL=b2_bezier_endpoint_jet(CP(:,:,q),M.h(q),'right');
    JR=b2_bezier_endpoint_jet(CP(:,:,q+1),M.h(q+1),'left');
    c6res=max(c6res,max(b2_jet_residual_by_order(JL,JR)));
end
end


function J=b2_bezier_endpoint_jet(P,h,side)
n=size(P,2)-1;
J=zeros(3,7);

for r=0:6
    ff=factorial(n)/factorial(n-r);
    D=zeros(3,1);

    if strcmp(side,'left')
        for k=0:r
            c=(-1)^(r-k)*nchoosek(r,k);
            D=D+c*P(:,k+1);
        end
    elseif strcmp(side,'right')
        for k=0:r
            c=(-1)^(r-k)*nchoosek(r,k);
            D=D+c*P(:,n-r+k+1);
        end
    else
        error('STEP4B2:Side','Unknown endpoint side.');
    end

    J(:,r+1)=(ff/(h^r))*D;
end
end


function r=b2_jet_residual_by_order(A,B)
r=zeros(1,7);
for k=1:7
    r(k)=norm(A(:,k)-B(:,k),2);
end
end


function [sAll,pAll]=b2_d13_eval_dense(CP,M,N)
sAll=[];
pAll=[];
edges=[0,cumsum(M.h)];
edges(end)=1;
for q=1:M.nseg
    tau=linspace(0,1,N);
    if q>1, tau=tau(2:end); end
    s=edges(q)+M.h(q)*tau;
    P=CP(:,:,q);
    for ii=1:numel(tau)
        pAll(:,end+1)=b2_bezier_eval(P,tau(ii)); %#ok<AGROW>
    end
    sAll=[sAll,s]; %#ok<AGROW>
end
end


function y=b2_bezier_eval(P,tau)
n=size(P,2)-1;
B=zeros(n+1,1);
for k=0:n
    B(k+1)=nchoosek(n,k)*(tau^k)*((1-tau)^(n-k));
end
y=P*B;
end


function R=b2_solve_bspline_fixed_boundary(G,C,J0,J1,obs,P,Tnew)
tBuild=tic;
M=b2_bspline_build_model_fixed_boundary(G,C,J0,J1);
[M.Aineq,M.bineq,M.safeMeta,M.cert]=bsd7_native_cp_certificate(M,C);
E=b2_bspline_equality_elimination(M);
buildMs=1e3*toc(tBuild);

Ar=M.Aineq*E.T;
br=M.bineq-M.Aineq*E.x0;
Hr=E.T.'*M.Hscaled*E.T;
Hr=0.5*(Hr+Hr.');
fr=E.T.'*(M.Hscaled*E.x0+M.fscaled);
qpopt=optimoptions('quadprog', ...
    'Algorithm','interior-point-convex', ...
    'Display','none', ...
    'ConstraintTolerance',1e-10, ...
    'OptimalityTolerance',1e-10, ...
    'StepTolerance',1e-12, ...
    'MaxIterations',500);

tSolve=tic;
[y,ef,out,cgdiag]=b3_qp_constraint_generation( ...
    sparse(Hr),fr,Ar,br,qpopt,'BSPLINE_D7_REDUCED');
solveMs=1e3*toc(tSolve);
out.constraint_generation=cgdiag;

R=b2_empty_result();
R.build_time_ms=buildMs;
R.solve_time_ms=solveMs;
R.exitflag=ef;
R.solver_output=out;
R.n_decision_variables=E.nred;
R.n_ineq=size(M.Aineq,1);
R.n_eq=size(M.Aeq,1);

if isempty(y) || ef<=0
    D=b3_infeasibility_audit(Ar,br,M.safeMeta,'BSPLINE_D7_REDUCED');
    R.solver_output.infeasibility_audit=D;
    b3_print_infeasibility_audit(D);
    return;
end

x=E.x0+E.T*y;
R.solver_feasible=true;

qPost=tic;
cp=bsd7_native_cp(x,M);
Bcp=bsd7_extract_bezier(cp,M);
R.native_control_points=cp;
R.control_points=Bcp;
R.post_reconstruct_ms=1e3*toc(qPost);

qPost=tic;
sl=M.bineq-M.Aineq*x;
R.min_original_hard_slack=min(sl);

% V33: direct full-original-row backward-error certificate.
%
% For each row, fl(b_i - A_i*x) differs from the exact real arithmetic
% value by at most a standard dot-product forward error bound:
%
%   g_i = gamma_{n+1} * ( |b_i| + sum_j |A_ij*x_j| ),
%   gamma_k = k*eps / (1-k*eps).
%
% A row is numerically certified when measured slack_i + g_i >= 0.
% This is scale-aware floating-point certification, not an empirical safety
% tolerance.
[rowCert,minCertifiedMargin,maxRowGuard,rowGuard]= ...
    b3_full_row_backward_error_certificate(M.Aineq,M.bineq,x);
R.original_row_numeric_certified=rowCert;
R.original_row_min_certified_margin=minCertifiedMargin;
R.original_row_max_fp_guard=maxRowGuard;

if R.min_original_hard_slack < 0 && ~rowCert
    % Only a violation exceeding the derived floating-point error bound
    % is allowed to invoke the old strict-interior recovery.
    [yInterior,rhoInterior,efInterior,diagInterior]= ...
        b3_max_uniform_interior_point(Ar,br);
    R.solver_output.v22_interior_certificate=diagInterior;
    if ~isempty(yInterior) && efInterior>0 && isfinite(rhoInterior) && rhoInterior>0
        xInterior=E.x0+E.T*yInterior;
        slInterior=M.bineq-M.Aineq*xInterior;
        minInterior=min(slInterior);
        fprintf('    [V22-INTERIOR] rho=%+.16e directOriginalSlack=%+.16e exit=%g\n', ...
            rhoInterior,minInterior,efInterior);

        [rowCertI,minCertifiedMarginI,maxRowGuardI,~]= ...
            b3_full_row_backward_error_certificate(M.Aineq,M.bineq,xInterior);

        if rowCertI
            x=xInterior;
            y=yInterior;
            sl=slInterior;
            R.min_original_hard_slack=minInterior;
            R.original_row_numeric_certified=true;
            R.original_row_min_certified_margin=minCertifiedMarginI;
            R.original_row_max_fp_guard=maxRowGuardI;
            cp=bsd7_native_cp(x,M);
            Bcp=bsd7_extract_bezier(cp,M);
            R.native_control_points=cp;
            R.control_points=Bcp;
        end
    else
        fprintf('    [V22-INTERIOR] no strict original-feasible interior certified: rho=%+.16e exit=%g\n', ...
            rhoInterior,efInterior);
    end
elseif R.min_original_hard_slack < 0 && rowCert
    fprintf(['    [ROW-FP-CERT] minDirectSlack=%+.16e maxFpGuard=%+.16e ', ...
        'minCertifiedMargin=%+.16e\n'], ...
        R.min_original_hard_slack,maxRowGuard,minCertifiedMargin);
end

R.post_slack_ms=1e3*toc(qPost);
qPost=tic;
[Jstart,Jend,c6resSpatial,c6resPhysical,c6Structural,maxKnotMult]= ...
    b2_bspline_audit_jets(Bcp,M,Tnew);
R.start_spatial_jet_residual_d0_d6=b2_jet_residual_by_order(Jstart,J0);
R.end_spatial_jet_residual_d0_d6=b2_jet_residual_by_order(Jend,J1);
R.c6_internal_residual=c6resSpatial;
R.c6_internal_physical_residual_d0_d6=c6resPhysical;
R.c6_structural_certified=c6Structural;
R.c6_max_internal_knot_multiplicity=maxKnotMult;

Jstart_t=b2_spatial_to_physical_jet(Jstart,Tnew);
Jend_t=b2_spatial_to_physical_jet(Jend,Tnew);
J0t=b2_spatial_to_physical_jet(J0,Tnew);
J1t=b2_spatial_to_physical_jet(J1,Tnew);

R.start_physical_splice_residual_d0_d6=b2_jet_residual_by_order(Jstart_t,J0t);
R.end_physical_residual_d0_d6=b2_jet_residual_by_order(Jend_t,J1t);

dtSeg=Tnew*diff(M.breaks);
C6A=b3_piecewise_bezier_c6_hard_audit(Bcp,dtSeg,J0t,J1t);
R.start_physical_splice_residual_d0_d6=C6A.start_residual_d0_d6;
R.start_physical_splice_guard_d0_d6=C6A.start_guard_d0_d6;
R.start_physical_backward_error_certified=C6A.start_certified;
R.end_physical_residual_d0_d6=C6A.end_residual_d0_d6;
R.end_physical_guard_d0_d6=C6A.end_guard_d0_d6;
R.end_physical_backward_error_certified=C6A.end_certified;
R.c6_internal_physical_residual_d0_d6=C6A.internal_residual_d0_d6;
R.c6_internal_guard_d0_d6=C6A.internal_guard_d0_d6;
R.c6_internal_backward_error_certified=C6A.internal_certified;
R.c6_hard_certified=R.c6_structural_certified && C6A.all_certified;
R.post_c6_audit_ms=1e3*toc(qPost);

% V58 measurement-only diagnostic is deferred until qPlan has stopped.
R.dense_exact_ellipsoid_slack=NaN;
R.dense_exact_ellipsoid_diag_ms=NaN;
R.deferred_dense_exact_ellipsoid_diag=true;

R.pass=R.solver_feasible && ...
       R.original_row_numeric_certified && ...
       R.c6_structural_certified && ...
       R.start_physical_backward_error_certified && ...
       R.end_physical_backward_error_certified && ...
       R.c6_internal_backward_error_certified && ...
       R.c6_hard_certified;
end




function [cert,minCertifiedMargin,maxGuard,guard]= ...
    b3_full_row_backward_error_certificate(A,b,x)
% Scale-aware IEEE-754 backward/forward error certificate for A*x <= b.
%
% Let s_i = b_i - fl(A_i*x).  With n products in the dot product,
% |error_i| <= gamma_(n+1) * (|b_i| + sum_j |A_ij*x_j|).
% A sufficient numerical certificate is s_i + guard_i >= 0 for every row.
A=full(A);
b=b(:);
x=x(:);

m=size(A,1);
n=size(A,2);

if m==0
    cert=true;
    minCertifiedMargin=Inf;
    maxGuard=0;
    guard=zeros(0,1);
    return;
end

u=eps/2; % IEEE double unit roundoff
k=n+1;
if k*u>=1
    error('STEP4B3:FPBound','Dot-product dimension too large for gamma_k bound.');
end
gamma=(k*u)/(1-k*u);

absDot=sum(abs(A).*abs(x.'),2);
guard=gamma*(abs(b)+absDot);
slack=b-A*x;
certifiedMargin=slack+guard;

cert=all(certifiedMargin>=0);
minCertifiedMargin=min(certifiedMargin);
maxGuard=max(guard);
end


function [y,rho,ef,D]=b3_max_uniform_interior_point(A,b)
% Max-min interior feasibility certificate for linear inequalities.
%
%   rho* = max rho
%          s.t. A*y + rho*1 <= b, rho >= 0.
%
% rho*>0 proves the reduced feasible polyhedron has a strict interior point.
% No empirical safety tolerance is used.
A=full(A);
b=b(:);
n=size(A,2);
m=size(A,1);

y=[];
rho=NaN;
ef=NaN;
D=struct('n_rows',m,'n_vars',n,'rho',NaN,'exitflag',NaN, ...
    'direct_slack',NaN,'method','MAX_MIN_UNIFORM_LINEAR_SLACK');

if m==0
    y=zeros(n,1);
    rho=Inf;
    ef=1;
    D.rho=rho;
    D.exitflag=ef;
    D.direct_slack=Inf;
    return;
end

f=[zeros(n,1);-1];
Aaug=[A,ones(m,1)];
lb=[-inf(n,1);0];
opt=optimoptions('linprog','Display','none','Algorithm','dual-simplex-highs');
[z,~,ef,out]=linprog(f,Aaug,b,[],[],lb,[],opt); %#ok<ASGLU>
D.exitflag=ef;
if ~isempty(z)
    y=z(1:n);
    rho=z(end);
    D.rho=rho;
    D.direct_slack=min(b-A*y);
end
end


function M=b2_bspline_build_model_fixed_boundary(G,C,J0,J1)
p=7;
qcell=3;
rBoundary=6;
ncoef=p*(qcell-1)+2+2*rBoundary;
nspan=ncoef-p;

W=G.waypoints;
segLen=vecnorm(diff(W,1,2),2,1);
Ltot=sum(segLen);
if ~(Ltot>0), error('STEP4B2:GuideLength','Zero guide length.'); end
frac=segLen/Ltot;

spanCount=[p,p,p];
ub=[0,cumsum(frac)];
ub(end)=1;

breaks=0;
cellOfSpan=zeros(1,nspan);
ss=0;
for cidx=1:3
    a=ub(cidx);
    b=ub(cidx+1);
    nc=spanCount(cidx);
    local=a+(0:nc)*(b-a)/nc;
    if cidx==1
        breaks=local;
    else
        breaks=[breaks,local(2:end)]; %#ok<AGROW>
    end
    cellOfSpan(ss+(1:nc))=cidx;
    ss=ss+nc;
end

U=[repmat(breaks(1),1,p+1),breaks(2:end-1),repmat(breaks(end),1,p+1)];
[~,Tbez]=step3a_bs_bezier_decomposition_transform(U,p,ncoef);

pieceMaps=cell(1,nspan);
for q=1:nspan
    ids=(q-1)*p+(1:p+1);
    pieceMaps{q}=Tbez(ids,:);
end

nvar=3*ncoef;
ix=1:ncoef;
iy=ncoef+(1:ncoef);
iz=2*ncoef+(1:ncoef);
axisIds={ix,iy,iz};

h0=breaks(2)-breaks(1);
h1=breaks(end)-breaks(end-1);
E0=pieceMaps{1};
E1=pieceMaps{end};

B0=b2_endpoint_bezier_cp_from_jets(J0,h0,p,'left');
B1=b2_endpoint_bezier_cp_from_jets(J1,h1,p,'right');

Aeq=zeros(0,nvar);
beq=zeros(0,1);

for ax=1:3
    ia=axisIds{ax};

    for j=0:6
        rr=zeros(1,nvar);
        rr(ia)=E0(j+1,:);
        Aeq(end+1,:)=rr; %#ok<AGROW>
        beq(end+1,1)=B0(ax,j+1); %#ok<AGROW>
    end

    for j=0:6
        rr=zeros(1,nvar);
        rr(ia)=E1(p-j+1,:);
        Aeq(end+1,:)=rr; %#ok<AGROW>
        beq(end+1,1)=B1(ax,j+1); %#ok<AGROW>
    end
end

% Temporary extracted-Bezier constraints; replaced by native certificate.
Aineq=zeros(0,nvar);
bineq=zeros(0,1);
safeMeta=struct('span',{},'cell',{},'cp',{},'plane',{});

% Same guide-proximity objective.
[xg,wg]=step3a_gauss8_01();
Cobj=zeros(3*nspan*numel(xg),nvar);
dobj=zeros(3*nspan*numel(xg),1);
rr=0;

for q=1:nspan
    ua=breaks(q);
    uq=breaks(q+1);
    hs=uq-ua;
    cellIdx=cellOfSpan(q);
    ga=ub(cellIdx);
    gb=ub(cellIdx+1);

    for k=1:numel(xg)
        u=ua+hs*xg(k);
        N=step3a_bs_basis_all(u,U,p,ncoef);

        alpha=(u-ga)/(gb-ga);
        alpha=min(1,max(0,alpha));
        pg=(1-alpha)*W(:,cellIdx)+alpha*W(:,cellIdx+1);

        sw=sqrt(wg(k)*hs);

        rr=rr+1; Cobj(rr,ix)=sw*N; dobj(rr)=sw*pg(1);
        rr=rr+1; Cobj(rr,iy)=sw*N; dobj(rr)=sw*pg(2);
        rr=rr+1; Cobj(rr,iz)=sw*N; dobj(rr)=sw*pg(3);
    end
end

H=2*(Cobj.'*Cobj);
H=0.5*(H+H.');
f=-2*(Cobj.'*dobj);
d=max(abs(diag(H)));
if ~(d>0), d=1; end

M=struct();
M.p=p;
M.ncoef=ncoef;
M.nspan=nspan;
M.nvar=nvar;
M.ix=ix;
M.iy=iy;
M.iz=iz;
M.iXi=[]; % retained only so shared helpers see the expected field.
M.frac=frac;
M.spanCount=spanCount;
M.breaks=breaks;
M.U=U;
M.ub=ub;
M.cellOfSpan=cellOfSpan;
M.Tbez=Tbez;
M.pieceMaps=pieceMaps;
M.Aeq=sparse(Aeq);
M.beq=beq;
M.Aineq=sparse(Aineq);
M.bineq=bineq;
M.safeMeta=safeMeta;
M.Hscaled=sparse(H/d);
M.fscaled=f/d;
end


function B=b2_endpoint_bezier_cp_from_jets(J,h,p,side)
% Returns the seven endpoint-near Bezier CPs in the order:
%   left : P0,P1,...,P6
%   right: Pp,P(p-1),...,P(p-6)
d=zeros(3,7);
for r=0:6
    ff=factorial(p)/factorial(p-r);
    d(:,r+1)=(h^r/ff)*J(:,r+1);
end

B=zeros(3,7);
if strcmp(side,'left')
    for j=0:6
        v=zeros(3,1);
        for r=0:j
            v=v+nchoosek(j,r)*d(:,r+1);
        end
        B(:,j+1)=v;
    end

elseif strcmp(side,'right')
    for j=0:6
        v=zeros(3,1);
        for r=0:j
            v=v+((-1)^r)*nchoosek(j,r)*d(:,r+1);
        end
        B(:,j+1)=v;
    end

else
    error('STEP4B2:Side','Unknown side.');
end
end


function E=b2_bspline_equality_elimination(M)
n=M.ncoef;
leftDep=1:7;
rightDep=(n-6):n;
middle=8:(n-7);

dep=[];
free=[];
for ax=0:2
    base=ax*n;
    dep=[dep,base+leftDep,base+rightDep]; %#ok<AGROW>
    free=[free,base+middle]; %#ok<AGROW>
end

Aeq=full(M.Aeq);
Ad=Aeq(:,dep);
Af=Aeq(:,free);

if size(Ad,1)~=size(Ad,2)
    error('STEP4B2:EqElim','Dependent endpoint block is not square.');
end
if rank(Ad)~=size(Ad,1)
    error('STEP4B2:EqElim','Dependent endpoint block is rank deficient.');
end

x0=zeros(M.nvar,1);
T=zeros(M.nvar,numel(free));
T(free,:)=eye(numel(free));
x0(dep)=Ad\M.beq;
T(dep,:)=-Ad\Af;

E=struct();
E.dep=dep;
E.free=free;
E.x0=x0;
E.T=T;
E.nred=numel(free);
E.middleCP=middle;
end


function [Jstart,Jend,c6resSpatial,c6resPhysical,c6Structural,maxKnotMult]= ...
    b2_bspline_audit_jets(Bcp,M,Tphys)
% Degree-p B-spline structural continuity:
% at an internal knot of multiplicity m, continuity is C^(p-m).
% For p=7 and every internal multiplicity <=1, C6 is guaranteed exactly by
% spline construction, independent of the optimized control-point values.
%
% Numerical extracted-Bezier residuals are retained as diagnostics.  They
% are reported both in normalized mission parameter s and in physical time,
% where derivative order r scales by Tphys^(-r).

Jstart=b2_bezier_endpoint_jet(Bcp(:,:,1),M.breaks(2)-M.breaks(1),'left');
Jend=b2_bezier_endpoint_jet(Bcp(:,:,end),M.breaks(end)-M.breaks(end-1),'right');

internal=M.breaks(2:end-1);
maxKnotMult=0;
for k=1:numel(internal)
    maxKnotMult=max(maxKnotMult,sum(M.U==internal(k)));
end
if isempty(internal)
    maxKnotMult=0;
end
c6Structural=(M.p-maxKnotMult)>=6;

c6resSpatial=0;
c6resPhysical=zeros(1,7);

for q=1:(M.nspan-1)
    hL=M.breaks(q+1)-M.breaks(q);
    hR=M.breaks(q+2)-M.breaks(q+1);
    JL=b2_bezier_endpoint_jet(Bcp(:,:,q),hL,'right');
    JR=b2_bezier_endpoint_jet(Bcp(:,:,q+1),hR,'left');

    rs=b2_jet_residual_by_order(JL,JR);
    c6resSpatial=max(c6resSpatial,max(rs));

    for r=0:6
        rp=rs(r+1)/(Tphys^r);
        c6resPhysical(r+1)=max(c6resPhysical(r+1),rp);
    end
end
end


function R=b2_empty_result()
R=struct();
R.solver_feasible=false;
R.pass=false;
R.build_time_ms=NaN;
R.solve_time_ms=NaN;
R.post_reconstruct_ms=NaN;
R.post_slack_ms=NaN;
R.post_c6_audit_ms=NaN;
R.deferred_dense_exact_ellipsoid_diag=true;
R.dense_exact_ellipsoid_diag_ms=NaN;
R.exitflag=NaN;
R.solver_output=struct();
R.n_decision_variables=NaN;
R.n_ineq=NaN;
R.n_eq=NaN;
R.min_original_hard_slack=-Inf;
R.start_spatial_jet_residual_d0_d6=nan(1,7);
R.end_spatial_jet_residual_d0_d6=nan(1,7);
R.start_physical_splice_residual_d0_d6=nan(1,7);
R.end_physical_residual_d0_d6=nan(1,7);
R.c6_internal_residual=NaN;
R.c6_internal_physical_residual_d0_d6=nan(1,7);
R.c6_internal_guard_d0_d6=nan(1,7);
R.c6_internal_backward_error_certified=false;
R.start_physical_splice_guard_d0_d6=nan(1,7);
R.end_physical_guard_d0_d6=nan(1,7);
R.start_physical_backward_error_certified=false;
R.end_physical_backward_error_certified=false;
R.c6_hard_certified=false;
R.c6_structural_certified=false;
R.c6_max_internal_knot_multiplicity=NaN;
R.original_row_numeric_certified=false;
R.original_row_min_certified_margin=-Inf;
R.original_row_max_fp_guard=NaN;
R.dense_exact_ellipsoid_slack=NaN;
R.control_points=[];
R.native_control_points=[];
R.sample_s=[];
R.sample_position=[];
end

function [Aineq,bineq,safeMeta,CERT]=bsd7_native_cp_certificate(M,C)
% V28 exact local extracted-Bezier certificate for the D7 B-spline.
%
% Each simple B-spline span is already represented exactly by M.pieceMaps.
% If a V28 local proof-cell boundary cuts through a span, the span is
% restricted exactly by de Casteljau before the halfspace constraint is
% applied. Therefore every continuous curve point is covered by a local
% Bezier convex hull certificate.

n=M.ncoef;
nvar=M.nvar;
p=M.p;

Aineq=zeros(0,nvar);
bineq=zeros(0,1);
safeMeta=struct('span',{},'cell',{},'subcell',{},'cp',{},'plane',{});

nRestrictedPieces=0;

for ci=1:3
    spans=find(M.cellOfSpan==ci);
    ns=numel(spans);
    if ns<1
        error('BSD7:LocalCert','A corridor cell has no B-spline span.');
    end

    leaves=C.cells{ci}.subcells;

    for kk=1:ns
        q=spans(kk);
        spanU0=(kk-1)/ns;
        spanU1=kk/ns;
        E=M.pieceMaps{q}; % (p+1)-by-ncoef scalar Bezier extraction map

        for rr=1:numel(leaves)
            CC=leaves(rr);
            ov0=max(spanU0,CC.u0);
            ov1=min(spanU1,CC.u1);
            if ov1<=ov0
                continue;
            end

            a=(ov0-spanU0)/(spanU1-spanU0);
            b=(ov1-spanU0)/(spanU1-spanU0);
            Rloc=b3_bezier_restrict_map(p,a,b);
            Eloc=Rloc*E;
            nRestrictedPieces=nRestrictedPieces+1;

            for j=1:p+1
                w=Eloc(j,:);
                for hh=1:size(CC.A,1)
                    nv=CC.A(hh,:).';
                    row=zeros(1,nvar);
                    row(M.ix)=row(M.ix)-nv(1)*w;
                    row(M.iy)=row(M.iy)-nv(2)*w;
                    row(M.iz)=row(M.iz)-nv(3)*w;

                    Aineq(end+1,:)=row; %#ok<AGROW>
                    bineq(end+1,1)=-CC.b(hh); %#ok<AGROW>
                    safeMeta(end+1)=struct('span',q,'cell',ci,'subcell',rr, ...
                        'cp',j,'plane',hh); %#ok<AGROW>
                end
            end
        end
    end
end

CERT=struct();
CERT.method='EXACT_LOCAL_EXTRACTED_BEZIER_RESTRICTION';
CERT.restrictedPieces=nRestrictedPieces;
CERT.constraintRows=size(Aineq,1);
CERT.uniqueMemberships=NaN;
CERT.multiCellCPs=NaN;
CERT.maxCellsPerCP=NaN;
end



function R=b3_bezier_restrict_map(n,a,b)
% Exact affine map from degree-n Bezier control points on [0,1] to control
% points of the SAME curve restricted to [a,b], reparameterized to [0,1].
if ~(isfinite(a) && isfinite(b) && a>=0 && b<=1 && b>a)
    error('STEP4B3:BezierRestriction','Require 0 <= a < b <= 1.');
end

tol=128*eps(max([1,abs(a),abs(b)]));
if abs(a)<=tol && abs(b-1)<=tol
    R=eye(n+1);
    return;
end

if abs(a)<=tol
    [L,~]=b3_bezier_split_maps(n,b);
    R=L;
    return;
end

[~,Ra]=b3_bezier_split_maps(n,a);
t=(b-a)/(1-a);
if abs(t-1)<=tol
    R=Ra;
else
    [Lt,~]=b3_bezier_split_maps(n,t);
    R=Lt*Ra;
end
end


function [Lmap,Rmap]=b3_bezier_split_maps(n,t)
% Exact de Casteljau split matrices.  For scalar original CP vector P:
%   P_left  = Lmap*P
%   P_right = Rmap*P
if ~(isfinite(t) && t>=0 && t<=1)
    error('STEP4B3:BezierSplit','Split parameter must lie in [0,1].');
end

persistent HALF_CACHE_N HALF_CACHE_L HALF_CACHE_R
if t==0.5 && ~isempty(HALF_CACHE_N)
    hit=find(HALF_CACHE_N==n,1,'first');
    if ~isempty(hit)
        Lmap=HALF_CACHE_L{hit};
        Rmap=HALF_CACHE_R{hit};
        return;
    end
end

N=n+1;
Lmap=zeros(N,N);
Rmap=zeros(N,N);

for k=1:N
    q=zeros(N,1);
    q(k)=1;
    left=zeros(N,1);
    right=zeros(N,1);
    left(1)=q(1);
    right(N)=q(N);

    work=q;
    for r=1:n
        work=(1-t)*work(1:end-1)+t*work(2:end);
        left(r+1)=work(1);
        right(N-r)=work(end);
    end

    Lmap(:,k)=left;
    Rmap(:,k)=right;
end

if t==0.5
    if isempty(HALF_CACHE_N)
        HALF_CACHE_N=n;
        HALF_CACHE_L={Lmap};
        HALF_CACHE_R={Rmap};
    elseif ~any(HALF_CACHE_N==n)
        HALF_CACHE_N(end+1)=n;
        HALF_CACHE_L{end+1}=Lmap;
        HALF_CACHE_R{end+1}=Rmap;
    end
end
end




function [Ar,br,D]=b3_exact_duplicate_qp_row_reduction(A,b)
% EXACT redundancy reduction for A*x <= b.
% Bitwise-identical A rows only; retain min(b), the strongest constraint.
D=struct('rows_before',size(A,1),'rows_after',size(A,1),'removed',0);
if isempty(A)
    Ar=A; br=b;
    return;
end

[Au,~,grp]=unique(A,'rows','stable');
ng=size(Au,1);
br=zeros(ng,1);

for g=1:ng
    ids=find(grp==g);
    br(g)=min(b(ids));
end

Ar=Au;
D.rows_after=ng;
D.removed=D.rows_before-D.rows_after;
end


function [x,ef,out,D]=b3_qp_constraint_generation(H,f,A,b,qpopt,label)
% Exact convex-QP constraint generation.
% Final feasibility authority is ALWAYS the exact original feasible set.
% Exact duplicate rows are first replaced by an equivalent strongest row.
[A,b,rowRed]=b3_exact_duplicate_qp_row_reduction(A,b);
m=size(A,1);
n=size(H,1);
D=struct('label',label,'full_rows',m,'rows_before_exact_reduction',rowRed.rows_before, ...
    'exact_duplicate_rows_removed',rowRed.removed,'iterations',0, ...
    'active_rows_final',0,'rows_added',zeros(1,0), ...
    'min_full_slack',NaN,'terminated','NOT_RUN');

if m==0
    [x,~,ef,out]=quadprog(H,f,[],[],[],[],[],[],[],qpopt);
    D.iterations=1;
    D.min_full_slack=Inf;
    D.terminated='NO_INEQUALITIES';
    return;
end

active=false(m,1);
x=[]; ef=NaN; out=struct();

for it=1:(m+1)
    ids=find(active);
    if isempty(ids)
        As=zeros(0,n); bs=zeros(0,1);
    else
        [As,bs]=b3_scale_qp_rows(A(ids,:),b(ids));
    end

    [xc,~,efc,outc]=quadprog(H,f,As,bs,[],[],[],[],[],qpopt);
    D.iterations=it;

    if isempty(xc) || efc<=0
        x=xc; ef=efc; out=outc;
        D.active_rows_final=nnz(active);
        D.terminated='ACTIVE_SUBPROBLEM_INFEASIBLE';
        return;
    end

    fullSlack=b-A*xc;
    D.min_full_slack=min(fullSlack);
    violated=(fullSlack<0) & ~active;

    if ~any(violated)
        x=xc; ef=efc; out=outc;
        D.active_rows_final=nnz(active);
        D.terminated='FULL_ORIGINAL_SET_CERTIFIED';
        b3_detail_fprintf(['    [CG-QP] %-20s rows=%d->%d active=%d iter=%d ', ...
            'minFullSlack=%+.3e\n'], ...
            label,D.rows_before_exact_reduction,m,D.active_rows_final,it,D.min_full_slack);
        return;
    end

    D.rows_added(end+1)=nnz(violated); %#ok<AGROW>
    active=active|violated;
end

error('STEP4B3:ConstraintGenerationLogic', ...
    'Constraint generation exceeded the finite row-addition bound.');
end

function [As,bs]=b3_scale_qp_rows(A,b)
if isempty(A), As=A; bs=b; return; end
sc=max(abs(A),[],2);
sc=max(sc,abs(b));
sc=max(sc,1);
As=A./sc;
bs=b./sc;
end


function [As,bs]=bsd7_scale_rows(A,b)
if isempty(A)
    As=A;
    bs=b;
    return
end
sc=max(abs(A),[],2);
sc=max(sc,abs(b));
sc=max(sc,1);
As=A./sc;
bs=b./sc;
end

function cp=bsd7_native_cp(x,M)
cp=[x(M.ix).';x(M.iy).';x(M.iz).'];
end

function Bcp=bsd7_extract_bezier(cp,M)
Qx=M.Tbez*cp(1,:).';
Qy=M.Tbez*cp(2,:).';
Qz=M.Tbez*cp(3,:).';

Bcp=zeros(3,M.p+1,M.nspan);
for q=1:M.nspan
    ids=(q-1)*M.p+(1:M.p+1);
    Bcp(:,:,q)=[Qx(ids),Qy(ids),Qz(ids)].';
end
end

function [sAll,pAll]=bsd7_eval_dense(cp,M,Ns)
sAll=linspace(0,1,Ns);
pAll=zeros(3,Ns);

for ii=1:Ns
    N=step3a_bs_basis_all(sAll(ii),M.U,M.p,M.ncoef);
    pAll(:,ii)=cp*N.';
end
end

function idx=bez13_jet_cpidx(seg,cp,axis,ncp)
idx=3*((seg-1)*ncp+(cp-1))+axis;
end

function F=bez13_jet_endpoint_difference_matrix(n)
% 14x14 matrix:
% rows 1..7  : left forward differences Delta^r P_0, r=0..6
% rows 8..14 : right forward differences Delta^r P_{n-r}, r=0..6
F=zeros(14,n+1);

for r=0:6
    for k=0:r
        c=(-1)^(r-k)*nchoosek(r,k);
        F(r+1,k+1)=c;
        F(8+r,n-r+k+1)=c;
    end
end
end

function E=bez13_endpoint_difference_inverse_direct(n)
% Closed-form inverse of bez13_jet_endpoint_difference_matrix for n=13.
% Left endpoint differences d_r=Delta^r P_0 give
%   P_j = sum_{r=0}^j C(j,r)d_r, j=0..6.
% Right endpoint differences d_r=Delta^r P_{n-r} give
%   P_{n-j}=sum_{r=0}^j (-1)^r C(j,r)d_r, j=0..6.
% No linear solve is needed.
if n~=13
    error('D13DIRECT:Degree','This exact direct map is defined for degree 13.');
end
E=zeros(14,14);
for j=0:6
    for r=0:j
        E(j+1,r+1)=nchoosek(j,r);
        E(n-j+1,8+r)=(-1)^r*nchoosek(j,r);
    end
end
end

function [As,bs]=bez13_jet_scale_ineq(A,b)
s=max(abs(A),[],2);
s=max(s,abs(b));
s=max(s,1);
As=A./s;
bs=b./s;
end

function mslack=bez13_dense_exact_ellipsoid_slack(Pts,obs,P)
% Diagnostic only.
%
% Exact Euclidean payload-center clearance to each oriented ellipsoid:
%
%   slack = dist(point, ellipsoid) - (R_payload + d_margin)
%
% aud_point_ellipsoid_distance() already uses ob.center / ob.radii / ob.R
% and computes the exact Euclidean point-to-ellipsoid distance.
%
% Continuous-time safety is still certified by Bezier control-point
% containment in the exact-support corridor; this dense check is NOT used
% as the certificate.

Rreq=P.R_payload+P.d_margin;
mslack=Inf;

for k=1:numel(obs)
    for ii=1:size(Pts,2)
        dist=aud_point_ellipsoid_distance(Pts(:,ii),obs(k));
        val=dist-Rreq;
        mslack=min(mslack,val);
    end
end
end

function [Uout,T]=step3a_bs_bezier_decomposition_transform(U,p,ncp)
% Standard repeated knot insertion applied to the identity control polygon.
% This is the same tested construction used in S2-R V6.

Uout=U;
T=eye(ncp);

interior=unique(U(p+2:end-p-1));
interior=interior(interior>U(1) & interior<U(end));

for jj=1:numel(interior)
    u=interior(jj);
    while sum(Uout==u)<p
        [Uout,T]=step3a_bs_insert_knot_once(Uout,T,u,p);
    end
end
end

function [Up,Q]=step3a_bs_insert_knot_once(U,P,u,p)
% One Boehm knot insertion. Rows of P are control points (or transformation
% rows); the operation is a local convex combination.
%
% This indexing is copied from the previously validated S2-R V6 helper.

n=size(P,1)-1; % zero-based last control index
k=find(U<=u,1,'last')-1; % zero-based knot span index
s=sum(U==u);

Q=zeros(size(P,1)+1,size(P,2));

% Q_i = P_i, i=0..k-p
for i0=0:(k-p)
    Q(i0+1,:)=P(i0+1,:);
end

% Q_{i+1} = P_i, i=k-s..n
for i0=(k-s):n
    Q(i0+2,:)=P(i0+1,:);
end

% Blended control points.
for i0=(k-p+1):(k-s)
    den=U(i0+p+1)-U(i0+1);
    if den==0
        alpha=0;
    else
        alpha=(u-U(i0+1))/den;
    end
    Q(i0+1,:)=alpha*P(i0+1,:)+(1-alpha)*P(i0,:);
end

Up=[U(1:k+1),u,U(k+2:end)];
end

function N=step3a_bs_basis_all(u,U,p,ncoef)
% Cox-de Boor, endpoint-safe.
N0=zeros(1,ncoef+p);

if abs(u-U(end))<=16*eps(max(1,abs(U(end))))
    N=zeros(1,ncoef);
    N(end)=1;
    return
end

for i=1:(numel(U)-1)
    if U(i)<=u && u<U(i+1)
        N0(i)=1;
    end
end

for d=1:p
    N1=zeros(size(N0));
    for i=1:(numel(U)-d-1)
        a=0; b=0;
        den1=U(i+d)-U(i);
        den2=U(i+d+1)-U(i+1);
        if den1>0
            a=(u-U(i))/den1*N0(i);
        end
        if den2>0
            b=(U(i+d+1)-u)/den2*N0(i+1);
        end
        N1(i)=a+b;
    end
    N0=N1;
end
N=N0(1:ncoef);
end

function [x,w]=step3a_gauss8_01()
% 8-point Gauss-Legendre mapped from [-1,1] to [0,1].
z=[ ...
 -0.9602898564975363
 -0.7966664774136267
 -0.5255324099163290
 -0.1834346424956498
  0.1834346424956498
  0.5255324099163290
  0.7966664774136267
  0.9602898564975363];
ww=[ ...
 0.1012285362903763
 0.2223810344533745
 0.3137066458778873
 0.3626837833783620
 0.3626837833783620
 0.3137066458778873
 0.2223810344533745
 0.1012285362903763];
x=(z+1)/2;
w=ww/2;
end

function P=aud_params()
P=struct();
P.d_margin=0.15; % geometry/engineering margin only; NOT a tracking-error bound
P.R_payload=0.24;
P.R_uav=0.30;
P.R_cable=0.045;
P.L_cable=2.0;
P.g=9.80665;
P.sensor_R=15.0;
P.dt_sensor=0.05;
P.lookahead=5.0;
P.pred_dt=0.10;
end

function [dmin,tmin,pmin]=aud_segment_ellipsoid_distance(A,B,ob)
A=A(:); B=B(:); dir=B-A;

if norm(dir)==0
    dmin=aud_point_ellipsoid_distance(A,ob);
    tmin=0; pmin=A; return
end

fun=@(t) aud_point_ellipsoid_distance(A+t*dir,ob);
opts=optimset('Display','off','TolX',1e-13);
[tmin,dmin]=fminbnd(fun,0,1,opts);

d0=fun(0); d1=fun(1);
if d0<dmin, dmin=d0; tmin=0; end
if d1<dmin, dmin=d1; tmin=1; end

pmin=A+tmin*dir;
end

function dist=aud_point_ellipsoid_distance(p,ob)
q=ob.R.'*(p(:)-ob.center);
a=ob.radii(:);
a2=a.^2;

if sum((q./a).^2)<=1
    dist=0;
    return
end

F=@(lam) sum((a2.*(q.^2))./((lam+a2).^2))-1;
lo=0;
hi=max([1;norm(q)^2;max(a2)]);
while F(hi)>0
    hi=2*hi;
end

for k=1:100
    mid=0.5*(lo+hi);
    if F(mid)>0
        lo=mid;
    else
        hi=mid;
    end
end
lam=0.5*(lo+hi);
x=(a2.*q)./(lam+a2);
dist=norm(q-x);
end

function [b1,b2]=aud_transverse_basis(e)
e=aud_unit(e);

ref=[0;0;1];
if abs(e.'*ref)>0.9
    ref=[0;1;0];
end

b1=cross(e,ref);
b1=b1-e*(e.'*b1);
b1=b1/norm(b1);

b2=cross(e,b1);
b2=b2-e*(e.'*b2)-b1*(b1.'*b2);
b2=b2/norm(b2);
end

function v=aud_unit(v)
v=v(:);
v=v/norm(v);
end

function b3_sensor_shape_preflight()
fprintf('\n[PRECHECK] internal depth-camera / obstacle-shape interface\n');

obs=b3_environment_obstacle_ellipse_move(0);
req={'id','type','raw_param','p_center','R_obs','ellipsoid_radii','Q_obs','v_center'};
for i=1:numel(obs)
    for k=1:numel(req)
        if ~isfield(obs(i),req{k})
            error('STEP4B3:ObstacleSchema', ...
                'Environment obstacle %d (%s) is missing field %s.',i,obs(i).type,req{k});
        end
    end
    if ~isequal(size(obs(i).p_center),[3 1])
        error('STEP4B3:ObstacleCenter','Obstacle id=%d p_center must be 3x1.',obs(i).id);
    end
    if ~isequal(size(obs(i).R_obs),[3 3])
        error('STEP4B3:ObstacleRotation','Obstacle id=%d R_obs must be 3x3.',obs(i).id);
    end
    if numel(obs(i).ellipsoid_radii)~=3 || any(obs(i).ellipsoid_radii(:)<=0)
        error('STEP4B3:ObstacleRadii','Obstacle id=%d ellipsoid_radii must contain 3 positive values.',obs(i).id);
    end
    b3_detail_fprintf('  ENV id=%d type=%-10s schema=PASS\n',obs(i).id,lower(obs(i).type));
end

% Kernel/fallback dispatch test. This does not alter the mission/environment.
types={'sphere','cylinder','box','prism','cone','pyramid','ellipsoid','custom'};
for k=1:numel(types)
    z=b3cam_synthetic_obstacle(types{k},k);
    o=[0;0;0];
    d=[0;0;1];
    [lam,mode]=b3cam_ray_obstacle(o,d,z);
    if ~(isfinite(lam) && lam>=0)
        error('STEP4B3:SensorKernel','Shape %s did not return a finite forward hit.',types{k});
    end
    b3_detail_fprintf('  %-10s visibility-kernel=PASS | %s\n',types{k},mode);
end
fprintf('[PRECHECK] shape interface PASS\n');
end


function z=b3cam_synthetic_obstacle(type,id)
z=struct();
z.id=id;
z.type=type;
z.p_center=[0;0;3];
z.R_obs=eye(3);
z.v_center=zeros(3,1);
z.d_margin=0.5;

switch type
    case 'sphere'
        z.raw_param=1.0;
        r=[1;1;1];
    case 'cylinder'
        z.raw_param=[1.0,2.0];
        r=[sqrt(2);sqrt(2);sqrt(2)];
    case 'box'
        z.raw_param=[2.0,2.0,2.0];
        r=[1;1;1]*sqrt(3);
    case 'prism'
        z.raw_param=[2.0,2.0];
        rt=2/sqrt(3);
        r=[rt*sqrt(2);rt*sqrt(2);sqrt(2)];
    case 'cone'
        z.raw_param=[1.0,2.0];
        rb=max(1,sqrt(1+(1)^2));
        r=[rb;rb;rb];
    case 'pyramid'
        z.raw_param=[2.0,2.0];
        rb=max(1,sqrt(2));
        r=[rb;rb;rb];
    case 'ellipsoid'
        z.raw_param=[1.0;0.8;1.2];
        r=z.raw_param(:);
    case 'custom'
        V=[-1 1 1 -1 -1 1 1 -1; ...
           -1 -1 1 1 -1 -1 1 1; ...
           -1 -1 -1 -1 1 1 1 1] + [0;0;3];
        z.raw_param=V;
        z.p_center=mean(V,2);
        r=[sqrt(3);sqrt(3);sqrt(3)];
    otherwise
        error('STEP4B3:SelfTestType','Unknown synthetic type.');
end
z.ellipsoid_radii=r(:);
z.Q_obs=diag(1./(r(:).^2));
z.p_obs=z.p_center;
z.r_obs=max(r);
z.r_obs_margin=z.r_obs+z.d_margin;
end

function OUT = b3_depth_camera_fast(t,p_WC,R_WC,known_ids_prev)
% b3_depth_camera_fast
%
% Production-oriented simulated forward depth camera for the C6 replanner.
%
% IMPORTANT:
%   * ENVIRONMENT_OBSTACLE_ELLIPSE_MOVE is accessed ONLY here.
%   * No spherical sensing gate is used.
%   * Raw cylinder/sphere/box/ellipsoid geometry is raycast for visibility.
%   * Camera visibility may raycast the simulator raw shape ONLY inside the sensor.
%   Planner safety geometry is reconstructed solely from returned depth-hit cells
%   as observed ellipsoid patches; raw center/size/orientation are not exported.
%   * Large/wide obstacles are supported without a special-case radius limit:
%     their conservative image ROI simply grows, up to the full FOV.
%   * Full-frame rays are NOT generated unless the projected obstacle ROIs
%     actually cover the full image.
%
% Camera convention:
%   +z optical forward, +x image right, +y image down.
%
% The returned schema is compatible with CURRENT_C6_SENSOR_DEPTH_CAMERA_V1.
%
% This is still a simulator front-end. A real depth-camera driver can replace
% this module while keeping the planner-side interface unchanged.

if nargin < 1 || isempty(t), t=0.0; end
if nargin < 4 || isempty(known_ids_prev), known_ids_prev=[]; end

tStart=tic;
validateattributes(t,{'numeric'},{'real','finite','scalar'});
validateattributes(p_WC,{'numeric'},{'real','finite','size',[3 1]'});
validateattributes(R_WC,{'numeric'},{'real','finite','size',[3 3]'});
if norm(R_WC.'*R_WC-eye(3),'fro') > 1e3*eps(max(1,norm(R_WC,'fro'))) || det(R_WC)<=0
    error('b3_depth_camera_fast:Rotation','R_WC must be a proper rotation matrix.');
end

S=b3cam_sensor_cfg();
qEnv=tic;
obs=b3_environment_obstacle_ellipse_move(t);
tEnv=1e3*toc(qEnv);
qROI=tic;
nObs=numel(obs);
if nObs==0
    OUT=b3cam_empty_out(S,known_ids_prev,toc(tStart)*1e3);
    return
end

% -------------------------------------------------------------------------
% Stage 1: conservative projected ROI generation.
% The enclosing ellipsoid radius is used ONLY to avoid testing image pixels
% that cannot possibly intersect an object. Final visibility uses raw shape.
% -------------------------------------------------------------------------
pixU=zeros(1,0);
pixV=zeros(1,0);
candidateObs=false(1,nObs);

for io=1:nObs
    cC=R_WC.'*(obs(io).p_center(:)-p_WC);
    rb=b3cam_outer_radius(obs(io));

    % Conservative depth rejection. Surface overlap with [min,max] is enough.
    if cC(3)+rb < S.min_depth_m || cC(3)-rb > S.max_depth_m || cC(3)+rb <= 0
        continue
    end

    candidateObs(io)=true;

    % Camera center inside/very near conservative bound -> full sampled FOV.
    if norm(cC)<=rb || cC(3)<=rb
        uu=0:S.pixel_skip:(S.width-1);
        vv=0:S.pixel_skip:(S.height-1);
    else
        zNear=max(S.min_depth_m,cC(3)-rb);
        u0=S.fx*cC(1)/cC(3)+S.cx;
        v0=S.fy*cC(2)/cC(3)+S.cy;
        du=abs(S.fx*rb/zNear);
        dv=abs(S.fy*rb/zNear);

        umin=max(0,floor((u0-du)/S.pixel_skip)*S.pixel_skip);
        umax=min(S.width-1,ceil((u0+du)/S.pixel_skip)*S.pixel_skip);
        vmin=max(0,floor((v0-dv)/S.pixel_skip)*S.pixel_skip);
        vmax=min(S.height-1,ceil((v0+dv)/S.pixel_skip)*S.pixel_skip);

        if umin>umax || vmin>vmax
            continue
        end
        uu=umin:S.pixel_skip:umax;
        vv=vmin:S.pixel_skip:vmax;
    end

    [UU,VV]=meshgrid(uu,vv);
    pixU=[pixU,UU(:).']; %#ok<AGROW>
    pixV=[pixV,VV(:).']; %#ok<AGROW>
end

tROI=1e3*toc(qROI);
known_ids_prev=unique(known_ids_prev(:).','stable');

if isempty(pixU)
    OUT=b3cam_empty_out(S,known_ids_prev,toc(tStart)*1e3);
    OUT.time=t;
    OUT.camera_position_W=p_WC;
    OUT.R_WC=R_WC;
    OUT.geometry_mode=cell(1,nObs);
    OUT.candidate_obstacle_ids=[obs(candidateObs).id];
    OUT.timing=struct('environment_ms',tEnv,'roi_ms',tROI,'rayprep_ms',0,'raycast_ms',0,'pack_ms',0);
    return
end

qPrep=tic;
UV=unique([pixU(:),pixV(:)],'rows','stable');
u=UV(:,1).';
v=UV(:,2).';

x=(u-S.cx)/S.fx;
y=(v-S.cy)/S.fy;
D_C=[x;y;ones(1,numel(u))];
D_C=D_C./vecnorm(D_C,2,1);
D_W=R_WC*D_C;
tPrep=1e3*toc(qPrep);
qRay=tic;

depth=inf(1,size(D_W,2));
owner=zeros(1,size(D_W,2));
geomMode=cell(1,nObs);

% -------------------------------------------------------------------------
% Stage 2: exact/raw-shape nearest-hit raycast only on candidate ROI rays.
% All obstacles are tested on this union so cross-object occlusion is kept.
% -------------------------------------------------------------------------
for io=1:nObs
    if ~candidateObs(io), continue; end
    [lam,mode]=b3cam_ray_obstacle(p_WC,D_W,obs(io));
    geomMode{io}=mode;
    valid=isfinite(lam) & lam>=S.min_depth_m & lam<=S.max_depth_m;
    nearer=valid & lam<depth;
    depth(nearer)=lam(nearer);
    owner(nearer)=obs(io).id;
end

tRay=1e3*toc(qRay);
qPack=tic;
visible_ids=unique(owner(owner~=0),'stable');
new_ids=setdiff(visible_ids,known_ids_prev,'stable');
known_ids=unique([known_ids_prev,visible_ids],'stable');

% Build planner models ONLY from depth-hit points.  The full environment
% obstacle structs are deliberately not exported from the sensor.
visibleModels=b3cam_observed_ellipsoids_from_hits(p_WC,D_W,depth,owner,visible_ids,S);
newModels=visibleModels(arrayfun(@(z)ismember(z.id,new_ids),visibleModels));

% Keep V1-compatible image fields, but only ROI pixels are populated.
uGrid=0:S.pixel_skip:(S.width-1);
vGrid=0:S.pixel_skip:(S.height-1);
depthImage=nan(numel(vGrid),numel(uGrid));
ownerImage=zeros(numel(vGrid),numel(uGrid));
iu=round(u/S.pixel_skip)+1;
iv=round(v/S.pixel_skip)+1;
lin=sub2ind(size(depthImage),iv,iu);
hit=isfinite(depth);
depthImage(lin(hit))=depth(hit);
ownerImage(lin)=owner;

OUT=struct();
OUT.time=t;
OUT.visible_ids=visible_ids;
OUT.new_ids=new_ids;
OUT.known_ids=known_ids;
OUT.visible_obstacles=visibleModels;
OUT.new_obstacles=newModels;
OUT.known_obstacles=visibleModels;
OUT.depth_m=depthImage;
OUT.owner_id=ownerImage;
OUT.sampled_u=uGrid;
OUT.sampled_v=vGrid;
OUT.sensor=S;
OUT.camera_position_W=p_WC;
OUT.R_WC=R_WC;
OUT.geometry_mode=geomMode;
OUT.n_sampled_rays=numel(u);
OUT.n_fullframe_sampled_rays=numel(uGrid)*numel(vGrid);
OUT.n_visible_pixels=sum(owner~=0);
OUT.candidate_obstacle_ids=[obs(candidateObs).id];
tPack=1e3*toc(qPack);
OUT.runtime_ms=toc(tStart)*1e3;
OUT.timing=struct('environment_ms',tEnv,'roi_ms',tROI,'rayprep_ms',tPrep,'raycast_ms',tRay,'pack_ms',tPack);
end


function models=b3cam_observed_ellipsoids_from_hits(p_WC,D_W,depth,owner,visible_ids,S)
% Construct one planner ellipsoid per visible connected object ID using ONLY
% depth-hit locations.
%
% The simulator uses owner ID only to separate occluded objects.  For each ID,
% the planner receives no true center, orientation, raw dimensions, or true
% enclosing ellipsoid.
%
% Pixel-cell inflation:
% sampled rays are spaced by pixel_skip pixels.  At maximum observed range d,
% a half sampling cell subtends
%   ax=atan(pixel_skip/(2*fx)), ay=atan(pixel_skip/(2*fy)).
% The Euclidean lateral half-diagonal
%   rho = d*sqrt(tan(ax)^2+tan(ay)^2)
% inflates the observed point AABB before ellipsoid conversion.
%
% The final ellipsoid contains the complete inflated AABB by the sqrt(3)
% construction.

models=repmat(struct('id',0,'p_center',zeros(3,1),'R_obs',eye(3), ...
    'ellipsoid_radii',ones(3,1),'obs_box_min',zeros(3,1),'obs_box_max',zeros(3,1), ...
    'n_observed_points',0,'type','observed_ellipsoid','model_source','DEPTH_HIT_CELLS_ONLY'),1,0);

ax=atan(S.pixel_skip/(2*S.fx));
ay=atan(S.pixel_skip/(2*S.fy));

for ii=1:numel(visible_ids)
    id=visible_ids(ii);
    mask=(owner==id) & isfinite(depth);
    if ~any(mask), continue; end

    PW=p_WC + D_W(:,mask).*depth(mask);
    dmax=max(depth(mask));
    rho=dmax*sqrt(tan(ax)^2+tan(ay)^2);

    boxMin=min(PW,[],2)-rho;
    boxMax=max(PW,[],2)+rho;
    c=0.5*(boxMin+boxMax);
    h=0.5*(boxMax-boxMin);
    r=sqrt(3)*h;

    z=struct();
    z.id=id;
    z.p_center=c;
    z.R_obs=eye(3);
    z.ellipsoid_radii=r;
    z.obs_box_min=boxMin;
    z.obs_box_max=boxMax;
    z.n_observed_points=size(PW,2);
    z.type='observed_ellipsoid';
    z.model_source='DEPTH_HIT_CELLS_ONLY';
    models(end+1)=z; %#ok<AGROW>
end
end


function S=b3cam_sensor_cfg()
S=struct();
S.width=640;
S.height=480;
S.fx=387.229248046875;
S.fy=387.229248046875;
S.cx=321.04638671875;
S.cy=243.44969177246094;
S.min_depth_m=0.2;
S.max_depth_m=5.0;
S.pixel_skip=2;
S.nominal_rate_hz=30.0;
S.optical_frame='+z forward, +x image-right, +y image-down';
S.visibility='conservative projected ROI + sampled nearest-hit raw-shape raycast';
S.post_detection_model='observed depth-hit cells -> guaranteed enclosing ellipsoid; no ground-truth size/center/orientation exported';
end


function OUT=b3cam_empty_out(S,known_ids_prev,runtimeMs)
u=0:S.pixel_skip:(S.width-1);
v=0:S.pixel_skip:(S.height-1);
OUT=struct();
OUT.time=NaN;
OUT.visible_ids=[];
OUT.new_ids=[];
OUT.known_ids=unique(known_ids_prev(:).','stable');
OUT.visible_obstacles=[];
OUT.new_obstacles=[];
OUT.known_obstacles=[];
OUT.depth_m=nan(numel(v),numel(u));
OUT.owner_id=zeros(numel(v),numel(u));
OUT.sampled_u=u;
OUT.sampled_v=v;
OUT.sensor=S;
OUT.camera_position_W=nan(3,1);
OUT.R_WC=nan(3,3);
OUT.geometry_mode={};
OUT.n_sampled_rays=0;
OUT.n_fullframe_sampled_rays=numel(u)*numel(v);
OUT.n_visible_pixels=0;
OUT.candidate_obstacle_ids=[];
OUT.runtime_ms=runtimeMs;
OUT.timing=struct('environment_ms',NaN,'roi_ms',NaN,'rayprep_ms',NaN,'raycast_ms',NaN,'pack_ms',NaN);
end


function rb=b3cam_outer_radius(item)
if isfield(item,'ellipsoid_radii') && ~isempty(item.ellipsoid_radii)
    rb=max(item.ellipsoid_radii(:));
    return
end
type=lower(item.type);
switch type
    case 'sphere'
        rb=item.raw_param(1);
    case 'cylinder'
        r=item.raw_param(1); h=item.raw_param(2);
        rb=sqrt(r^2+(h/2)^2);
    case 'box'
        rb=0.5*norm(item.raw_param(:));
    otherwise
        error('DEPTHV2:OuterRadius','Obstacle %d has no conservative size information.',item.id);
end
end


function [lambda,mode]=b3cam_ray_obstacle(oW,dW,item)
type=lower(item.type);
switch type
    case 'cylinder'
        lambda=b3cam_ray_cylinder(oW,dW,item);
        mode='raw finite cylinder';
    case 'sphere'
        lambda=b3cam_ray_sphere(oW,dW,item);
        mode='raw sphere';
    case 'ellipsoid'
        lambda=b3cam_ray_ellipsoid(oW,dW,item.p_center,item.R_obs,item.ellipsoid_radii);
        mode='raw ellipsoid';
    case 'box'
        lambda=b3cam_ray_box(oW,dW,item);
        mode='raw oriented box';
    case 'cone'
        lambda=b3cam_ray_cone(oW,dW,item);
        mode='raw finite cone';
    case {'prism','pyramid','custom'}
        % Conservative visibility fallback: never drop an obstacle merely
        % because a dedicated raw-shape ray kernel is not yet available.
        lambda=b3cam_ray_ellipsoid(oW,dW,item.p_center,item.R_obs,item.ellipsoid_radii);
        mode=['conservative enclosing ellipsoid for ',type];
    otherwise
        lambda=b3cam_ray_ellipsoid(oW,dW,item.p_center,item.R_obs,item.ellipsoid_radii);
        mode=['conservative enclosing ellipsoid for unknown type ',type];
end
end


function lambda=b3cam_ray_cylinder(oW,dW,item)
r=item.raw_param(1);
h=item.raw_param(2);
R=item.R_obs;
c=item.p_center(:);
o=R.'*(oW-c);
d=R.'*dW;
N=size(d,2);
lambda=inf(1,N);

A=d(1,:).^2+d(2,:).^2;
B=2*(o(1)*d(1,:)+o(2)*d(2,:));
C=o(1)^2+o(2)^2-r^2;
disc=B.^2-4*A*C;
mask=A>eps & disc>=0;
sd=zeros(1,N); sd(mask)=sqrt(disc(mask));

for sg=[-1,1]
    root=inf(1,N);
    root(mask)=(-B(mask)+sg*sd(mask))./(2*A(mask));
    z=o(3)+root.*d(3,:);
    good=root>=0 & abs(z)<=h/2;
    lambda(good)=min(lambda(good),root(good));
end

for zcap=[-h/2,h/2]
    maskCap=abs(d(3,:))>eps;
    root=inf(1,N);
    root(maskCap)=(zcap-o(3))./d(3,maskCap);
    x=o(1)+root.*d(1,:);
    y=o(2)+root.*d(2,:);
    good=root>=0 & (x.^2+y.^2)<=r^2;
    lambda(good)=min(lambda(good),root(good));
end
end


function lambda=b3cam_ray_sphere(oW,dW,item)
r=item.raw_param(1);
c=item.p_center(:);
oc=oW-c;
B=2*(oc.'*dW);
C=dot(oc,oc)-r^2;
disc=B.^2-4*C;
lambda=inf(1,size(dW,2));
mask=disc>=0;
sd=zeros(size(disc)); sd(mask)=sqrt(disc(mask));
r1=(-B-sd)/2;
r2=(-B+sd)/2;
g1=mask&r1>=0; g2=mask&r2>=0;
lambda(g1)=r1(g1);
lambda(g2)=min(lambda(g2),r2(g2));
end


function lambda=b3cam_ray_ellipsoid(oW,dW,c,R,radii)
o=R.'*(oW-c(:));
d=R.'*dW;
ir2=1./(radii(:).^2);
A=ir2.'*(d.^2);
B=2*((ir2.*o).'*d);
C=sum(ir2.*o.^2)-1;
disc=B.^2-4*A*C;
lambda=inf(1,size(dW,2));
mask=A>eps & disc>=0;
sd=zeros(size(disc)); sd(mask)=sqrt(disc(mask));
r1=inf(size(disc)); r2=inf(size(disc));
r1(mask)=(-B(mask)-sd(mask))./(2*A(mask));
r2(mask)=(-B(mask)+sd(mask))./(2*A(mask));
g1=mask&r1>=0; g2=mask&r2>=0;
lambda(g1)=r1(g1);
lambda(g2)=min(lambda(g2),r2(g2));
end



function lambda=b3cam_ray_cone(oW,dW,item)
% Finite right circular cone centered at item.p_center, local +z apex side.
% raw_param = [base radius r, height h]. Local cone spans z in [-h/2,+h/2],
% base at z=-h/2 and apex at z=+h/2.
r=item.raw_param(1);
h=item.raw_param(2);
R=item.R_obs;
c=item.p_center(:);
o=R.'*(oW-c);
d=R.'*dW;
N=size(d,2);
lambda=inf(1,N);

% Side: x^2+y^2 = k^2*(h/2-z)^2, k=r/h.
k=r/h;
q0=h/2-o(3);
A=d(1,:).^2+d(2,:).^2-(k^2)*d(3,:).^2;
B=2*(o(1)*d(1,:)+o(2)*d(2,:) + (k^2)*q0*d(3,:));
C=o(1)^2+o(2)^2-(k^2)*q0^2;
disc=B.^2-4*A*C;
mask=abs(A)>eps & disc>=0;
sd=zeros(1,N);
sd(mask)=sqrt(disc(mask));
for sg=[-1,1]
    root=inf(1,N);
    root(mask)=(-B(mask)+sg*sd(mask))./(2*A(mask));
    z=o(3)+root.*d(3,:);
    good=root>=0 & z>=-h/2 & z<=h/2;
    lambda(good)=min(lambda(good),root(good));
end

% Degenerate linear side equation.
mlin=abs(A)<=eps & abs(B)>eps;
root=inf(1,N);
root(mlin)=-C./B(mlin);
z=o(3)+root.*d(3,:);
good=mlin & root>=0 & z>=-h/2 & z<=h/2;
lambda(good)=min(lambda(good),root(good));

% Base disk z=-h/2.
mcap=abs(d(3,:))>eps;
root=inf(1,N);
root(mcap)=(-h/2-o(3))./d(3,mcap);
x=o(1)+root.*d(1,:);
y=o(2)+root.*d(2,:);
good=root>=0 & (x.^2+y.^2)<=r^2;
lambda(good)=min(lambda(good),root(good));
end

function lambda=b3cam_ray_box(oW,dW,item)
half=item.raw_param(:)/2;
R=item.R_obs;
c=item.p_center(:);
o=R.'*(oW-c);
d=R.'*dW;
N=size(d,2);
tmin=-inf(1,N);
tmax=inf(1,N);
ok=true(1,N);
for ax=1:3
    da=d(ax,:);
    oa=o(ax);
    h=half(ax);
    par=abs(da)<=eps;
    ok(par & (oa<-h | oa>h))=false;
    idx=find(~par);
    t1=(-h-oa)./da(idx);
    t2=( h-oa)./da(idx);
    tmin(idx)=max(tmin(idx),min(t1,t2));
    tmax(idx)=min(tmax(idx),max(t1,t2));
end
ok=ok & tmax>=max(tmin,0);
lambda=inf(1,N);
outside=ok&tmin>=0;
inside=ok&tmin<0&tmax>=0;
lambda(outside)=tmin(outside);
lambda(inside)=tmax(inside);
end

function b3_primal_distance_cause_isolation_suite(P)
% DIAGNOSTIC ONLY. Excluded from mission/runtime statistics.
fprintf('\n[CAUSE-ISOLATION] primal-distance kernel sanity tests (excluded from production timing)\n');
ob=b3_empty_memory_item();
ob.id=9001;
ob.center=[0;0;0];
ob.center_seen=ob.center;
ob.radii=[1;1;1];
ob.S=eye(3);
ob.v_center=zeros(3,1);

cases={ ...
    struct('name','FAR_STATIC','Seeds',[8;0;0]), ...
    struct('name','NEAR_STATIC','Seeds',[4;0;0]) ...
    };

for k=1:numel(cases)
    q=tic;
    [~,~,d,ef,D]=b3_primal_convex_distance_separator(ob,cases{k}.Seeds,0,1,P);
    ms=1e3*toc(q);
    fprintf('  [CAUSE] %-12s exit=%g certifiedGap=%+.9e total=%.6f ms nvars=%d ncones=%d\n', ...
        cases{k}.name,ef,d,ms,D.nvars,D.ncones);
end

inside=b3_any_seed_inside_raw_swept_ellipsoid([0;0;0],ob,0,1);
fprintf('  [CAUSE] RAW_INSIDE cheapNecessaryReject=%d (expected 1)\n',inside);
fprintf('[CAUSE-ISOLATION] end\n');
end





function b3_local_callgraph_precheck()
% V46 full-file local helper callgraph regression guard.
%
% Audits all project-local helper families used by this one-file validator:
%   b2_*, b3_*, aud_*, bez13_*, bsd7_*.
%
% This specifically catches the failure class where a large function-block
% replacement removes a helper definition while leaving its call site intact.

fn=[mfilename('fullpath') '.m'];
if ~isfile(fn)
    error('STEP4B3:CallgraphGuard', ...
        'Cannot locate validator source file: %s',fn);
end
src=fileread(fn);

% Function definitions may use multiline output lists and MATLAB "..." line
% continuation before the function name.
defTok=regexp(src, ...
    ['(?ms)^\s*function\s+(?:\[[^\]]*\]\s*=\s*(?:\.\.\.\s*)*|' ...
     '[A-Za-z]\w*\s*=\s*(?:\.\.\.\s*)*)?([A-Za-z]\w*)\s*\('], ...
    'tokens');
defs=cellfun(@(c)c{1},defTok,'UniformOutput',false);

families={'b2_','b3_','aud_','bez13_','bsd7_'};
calls={};
for k=1:numel(families)
    % Do not use \b here. V46 showed that the assumed Python-style word
    % boundary caused MATLAB regexp to extract zero project calls.
    pat=['(' regexptranslate('escape',families{k}) '[A-Za-z]\w*)\s*\('];
    tok=regexp(src,pat,'tokens');
    calls=[calls,cellfun(@(c)c{1},tok,'UniformOutput',false)]; %#ok<AGROW>
end

defs=unique(defs);
calls=unique(calls);

if numel(calls)<20
    error('STEP4B3:CallgraphGuard', ...
        'Project-call extraction is implausibly small (%d); callgraph guard itself is invalid.', ...
        numel(calls));
end

missing=setdiff(calls,defs);

if ~isempty(missing)
    error('STEP4B3:CallgraphGuard', ...
        'Undefined local project helper(s): %s',strjoin(missing,', '));
end

% Duplicate function definitions are forbidden.
defAll=cellfun(@(c)c{1},defTok,'UniformOutput',false);
[un,~,idx]=unique(defAll);
cnt=accumarray(idx(:),1);
dup=un(cnt>1);
if ~isempty(dup)
    error('STEP4B3:CallgraphGuard', ...
        'Duplicate local function definition(s): %s',strjoin(dup,', '));
end

fprintf('[PRECHECK] local callgraph PASS | definitions=%d | projectCalls=%d | undefined=0\n', ...
    numel(defs),numel(calls));
end


function b3_source_regression_precheck()
% Targeted source regression guard for previously observed symbol/sign/schema
% failures. Patterns are assembled by concatenation so the guard does not
% self-match its own forbidden literals.

fn=[mfilename('fullpath') '.m'];
if ~isfile(fn)
    error('STEP4B3:SourceRegressionGuard', ...
        'Cannot locate validator source file: %s',fn);
end
src=fileread(fn);

forbidden={ ...
    ['centerUav=ob.center+L*' 'p0;'], ...
    ['centerCableMid=ob.center+0.5*L*' 'q0;'], ...
    ['function S=b3_' 'rigid_cable_state_from_jet'], ...
    ['S.' 'q='], ...
    ['.' 'pQ'], ...
    ['secondordercone(Dmat,' '-c0,dsoc,0)'], ...
    ['Dmat(:,iQ)=' 'L*eye(3);'], ...
    ['Dmat(:,iZ)=' 'L*eye(3);'], ...
    ['pF=pF-' 'L*x(iQ);'], ...
    ['pF=pF-' 'L*x(iZ);'], ...
    ['Dmat(:,iQ)=-' 'L*eye(3);'], ...
    ['pF=pF+' 'L*x(iQ);']};

for k=1:numel(forbidden)
    if contains(src,forbidden{k})
        error('STEP4B3:SourceRegressionGuard', ...
            'Forbidden legacy/regression pattern remains: %s',forbidden{k});
    end
end

required={ ...
    ['pHang=' '-CFG.e3;'], ...
    ['centerUav=ob.center+L*' 'pHang;'], ...
    ['centerCableMid=ob.center+0.5*L*' 'pHang;'], ...
    ['function S=b3_' 'sreenath_cable_state_from_jet'], ...
    ['xQ=xL-' 'P.L_cable*p;'], ...
    ['omega=cross(' 'p,pdot);'], ...
    ['omegadot=cross(' 'p,pddot);'], ...
    ['secondordercone(Dmat,' 'c0,dsoc,0)'], ...
    ['Dmat(:,iP)=-' 'L*eye(3);'], ...
    ['Dmat(:,iZ)=-' 'L*eye(3);'], ...
    ['pF=pF+' 'L*x(iP);'], ...
    ['pF=pF+' 'L*x(iZ);'], ...
    ['exactVertical=(' 'alpha==0);'], ...
    ['cAffine=c0+' 'L*pHang;'], ...
    ['Dmat(:,iCableLambda)=-' 'L*pHang;'], ...
    ['function D=b3_' 'dense_rigid_cable_diagnostic'], ...
    ['function A=b3_' 'dense_sreenath_model_diagnostic']};

for k=1:numel(required)
    if ~contains(src,required{k})
        error('STEP4B3:SourceRegressionGuard', ...
            'Required Sreenath/dynamic-geometry invariant missing: %s',required{k});
    end
end

tok=regexp(src,'(?m)^\s*function\s+(?:\[[^\]]*\]\s*=|[^=\n]*=\s*)?([A-Za-z]\w*)\s*\(','tokens');
names=cellfun(@(c)c{1},tok,'UniformOutput',false);
[un,~,idx]=unique(names);
cnt=accumarray(idx(:),1);
dup=un(cnt>1);
if ~isempty(dup)
    error('STEP4B3:SourceRegressionGuard', ...
        'Duplicate local function name(s): %s',strjoin(dup,','));
end

fprintf('[PRECHECK] source regression guard PASS | functions=%d | legacyPatterns=0\n',numel(names));
end


function b3_dynamic_geometry_identity_precheck(P,CFG)
% Hard precheck for coordinate/sign consistency.
fprintf('\n[DYNAMIC-GEOMETRY-PRECHECK] analytic identity tests\n');

ob=b3_empty_memory_item();
ob.id=9101;
ob.center=[3;-1;2];
ob.center_seen=ob.center;
ob.radii=[1;1;1];
ob.S=eye(3);
ob.v_center=zeros(3,1);

dirs=[1 0 0; 0 0 1; 0 0 -1; 1 2 3; -2 1 0.5].';
components={'PAYLOAD','UAV','CABLE'};
maxIdentityResidual=0;

for ic=1:numel(components)
    comp=components{ic};
    for j=1:size(dirs,2)
        n=dirs(:,j);
        n=n/norm(n);

        hLegacy=b3_component_forbidden_support(ob,n,0,1,P,comp);
        hDynamic=b3_dynamic_component_forbidden_support(ob,n,0,1,P,comp,0);
        res=abs(hLegacy-hDynamic);
        maxIdentityResidual=max(maxIdentityResidual,res);

        guard=64*eps(max([1,abs(hLegacy),abs(hDynamic)]));
        if res>guard
            error('STEP4B3:DynamicSupportIdentity', ...
                'alpha=0 support identity failed for %s dir=%s residual=%g guard=%g.', ...
                comp,mat2str(n.',6),res,guard);
        end
    end
end

Rp=P.R_payload+P.d_margin;
Ru=P.R_uav+P.d_margin;
Rc=P.R_cable+P.d_margin;
L=P.L_cable;

% Nominal hanging Sreenath cable direction.
% Explicit name avoids collision with CFG.p0 (mission origin).
pHang=-CFG.e3;

% Direct Sreenath geometry identity before any SOCP call.
xLtest=[1.25;-0.75;0.4];
xQtest=xLtest-L*pHang;
guardGeom=128*eps(max([1;abs(xLtest);L]));
if norm((xLtest-xQtest)-L*pHang)>guardGeom
    error('STEP4B3:SreenathGeometryIdentity', ...
        'xQ=xL-Lp identity failed before dynamic separator audit.');
end

% Translated spherical obstacle: analytic point-to-forbidden-set distances.
seedPayload=ob.center+[7;0;0];
b3_dynamic_distance_analytic_case('PAYLOAD_TRANSLATED_SPHERE', ...
    ob,seedPayload,P,'PAYLOAD',0,7-(1+Rp));

centerUav=ob.center+L*pHang;
seedUav=centerUav+[7;0;0];
b3_dynamic_distance_analytic_case('UAV_VERTICAL_IDENTITY', ...
    ob,seedUav,P,'UAV',0,7-(1+Ru));

centerCableMid=ob.center+0.5*L*pHang;
seedCable=centerCableMid+[7;0;0];
b3_dynamic_distance_analytic_case('CABLE_VERTICAL_CAPSULE', ...
    ob,seedCable,P,'CABLE',0,7-(1+Rc));

fprintf(['[DYNAMIC-GEOMETRY-PRECHECK] PASS | alpha0SupportResidual=%.3e | ', ...
    'translated-center/+Lp identities verified\n'],maxIdentityResidual);
end

function b3_dynamic_distance_analytic_case(name,ob,seed,P,component,alpha,expected)
[n,b,d,ef,D]=b3_primal_dynamic_component_separator(ob,seed,0,1,P,component,alpha);

if ~(ef>0 && all(isfinite(n)) && isfinite(b) && isfinite(d))
    error('STEP4B3:DynamicDistancePrecheckSolver', ...
        '%s did not return a finite certified solution.',name);
end

solverGuard=32*max([0,D.primalfeasibility,D.dualfeasibility,D.dualitygap]);
fpGuard=256*eps(max([1,abs(expected),abs(d)]));
guard=max(solverGuard,fpGuard);

if abs(d-expected)>guard
    error('STEP4B3:DynamicDistanceAnalyticMismatch', ...
        ['%s computed=%+.16e expected=%+.16e residual=%+.3e guard=%+.3e ' ...
         'primal=%+.3e dual=%+.3e gap=%+.3e.'], ...
        name,d,expected,abs(d-expected),guard, ...
        D.primalfeasibility,D.dualfeasibility,D.dualitygap);
end

fprintf('  [DYN-ID] %-28s gap=%+.9e expected=%+.9e guard=%.3e exactVertical=%d\n', ...
    name,d,expected,guard,D.exact_vertical_reduction);
end


function obs = b3_environment_obstacle_ellipse_move(t)
% Embedded environment provider.
% Based directly on the user's current ENVIRONMENT_OBSTACLE_ELLIPSE_MOVE.
% No external ENVIRONMENT_OBSTACLE_ELLIPSE_MOVE.m is required.
if nargin < 1
    t = 0.0;
end

obs = [];

% Obstacle 1
p0_1  = [10.0; 0.0; 2.0]; % aligned with the current +x mission / camera height
v_1   = [0.0; 0.0; 0.0];
p_center1 = p0_1 + v_1*t;
cyl_param1 = [2.5, 1.0];
R_obs1 = eye(3);
d_margin1 = 0.5;
item1 = b3_build_obstacle_data('cylinder',p_center1,cyl_param1,R_obs1,d_margin1);
item1.v_center = v_1;
item1.id = 1;
obs = [obs,item1];

% Obstacle 2
p_center2 = [22.0; 2.0; 2.0]; % wide obstacle, offset but still intersects current UAV/cable corridor
v0_2 = [0;0;0];
cyl_param2 = [3.0,1.0];
R_obs2 = eye(3);
d_margin2 = 0.5;
item2 = b3_build_obstacle_data('cylinder',p_center2,cyl_param2,R_obs2,d_margin2);
item2.v_center = v0_2;
item2.id = 2;
obs = [obs,item2];

% Obstacle 3
p_center3 = [32.0; -1.5; 2.0]; % later obstacle for repeated-sensing/replanning audit
v_3 = [0;0;0];
cyl_param3 = [1.0,1.0];
R_obs3 = eye(3);
d_margin3 = 0.5;
item3 = b3_build_obstacle_data('cylinder',p_center3,cyl_param3,R_obs3,d_margin3);
item3.v_center = v_3;
item3.id = 3;
obs = [obs,item3];
end


function item = b3_build_obstacle_data(type,p_center,raw_param,R_obs,d_margin)
item.type = type;
item.raw_param = raw_param;
item.p_center = p_center;
item.R_obs = R_obs;
item.d_margin = d_margin;
item.v_center = [0;0;0];

switch lower(type)
    case 'sphere'
        r = raw_param;
        radii = [r;r;r];

    case 'cylinder'
        r = raw_param(1);
        h = raw_param(2);
        radii = [r*sqrt(2);r*sqrt(2);(h/2)*sqrt(2)];

    case 'box'
        dx = raw_param(1);
        dy = raw_param(2);
        dz = raw_param(3);
        radii = [dx/2;dy/2;dz/2]*sqrt(3);

    case 'prism'
        r_tri = raw_param(1)/sqrt(3);
        h = raw_param(2);
        radii = [r_tri*sqrt(2);r_tri*sqrt(2);(h/2)*sqrt(2)];

    case 'cone'
        r = raw_param(1);
        h = raw_param(2);
        r_bound = max(h/2,sqrt(r^2+(h/2)^2));
        radii = [r_bound;r_bound;r_bound];

    case 'pyramid'
        r_bot = sqrt(2)*(raw_param(1)/2);
        h = raw_param(2);
        r_bound = max(h/2,sqrt(r_bot^2+(h/2)^2));
        radii = [r_bound;r_bound;r_bound];

    case 'ellipsoid'
        radii = raw_param(:);

    case 'custom'
        vertices = raw_param;
        p_c = mean(vertices,2);
        item.p_center = p_c;
        diffs = vertices-p_c;
        a = max(abs(diffs(1,:)))*sqrt(3);
        b = max(abs(diffs(2,:)))*sqrt(3);
        c = max(abs(diffs(3,:)))*sqrt(3);
        radii = [max(a,0.1);max(b,0.1);max(c,0.1)];

    otherwise
        error('STEP4B3:ObstacleType','Unsupported obstacle type: %s',type);
end

item.ellipsoid_radii = radii;
item.Q_obs = diag(1./(radii.^2));
item.p_obs = item.p_center;
item.r_obs = max(radii);
item.r_obs_margin = item.r_obs+d_margin;
end

function D=b3_infeasibility_audit(A,b,meta,label)
% Diagnostic only. Acceptance constraints are NOT changed.
A=full(A);
b=b(:);
m=size(A,1);
n=size(A,2);

rowNorm=sqrt(sum(A.^2,2));
scale=max(1,max(rowNorm));
fixedTol=1e-13*scale;
fixed=(rowNorm<=fixedTol);
fixedViolation=max(0,-b(fixed));

D=struct();
D.label=label;
D.n_rows=m;
D.n_vars=n;
D.n_fixed_rows=sum(fixed);
D.n_fixed_violated=sum(fixed & (b<0));
D.worst_fixed_violation=0;
D.worst_fixed_row=NaN;
D.worst_fixed_meta=struct();
D.phase1_success=false;
D.phase1_exitflag=NaN;
D.min_uniform_relaxation=NaN;
D.worst_phase1_row=NaN;
D.worst_phase1_violation=NaN;
D.worst_phase1_meta=struct();

if any(fixed & (b<0))
    ids=find(fixed & (b<0));
    [vv,kk]=max(-b(ids));
    rid=ids(kk);
    D.worst_fixed_violation=vv;
    D.worst_fixed_row=rid;
    if ~isempty(meta) && rid<=numel(meta)
        D.worst_fixed_meta=meta(rid);
    end
end

% Phase-I LP:
% min rho
% s.t. A*x - rho <= b, rho >= 0.
% This measures the smallest common relaxation needed to make all rows feasible.
if exist('linprog','file')==2 && m>0
    f=[zeros(n,1);1];
    Aaug=[A,-ones(m,1)];
    lb=[-inf(n,1);0];
    opts=optimoptions('linprog','Display','none');
    [xr,~,ef]=linprog(f,Aaug,b,[],[],lb,[],opts);
    D.phase1_exitflag=ef;
    if ~isempty(xr) && ef>0
        x=xr(1:n);
        rho=xr(end);
        viol=A*x-b;
        [vmax,rid]=max(viol);
        D.phase1_success=true;
        D.min_uniform_relaxation=rho;
        D.worst_phase1_row=rid;
        D.worst_phase1_violation=vmax;
        if ~isempty(meta) && rid<=numel(meta)
            D.worst_phase1_meta=meta(rid);
        end
    end
end
end


function b3_print_infeasibility_audit(D)
fprintf('    [INFEAS] label=%s rows=%d vars=%d fixedRows=%d fixedViolated=%d\\n', ...
    D.label,D.n_rows,D.n_vars,D.n_fixed_rows,D.n_fixed_violated);
fprintf('    [INFEAS] worstFixedViolation=%+.16e row=%g meta=%s\\n', ...
    D.worst_fixed_violation,D.worst_fixed_row,b3_meta_to_string(D.worst_fixed_meta));
fprintf('    [INFEAS] phase1Success=%d exitflag=%g minUniformRelaxation=%+.16e\\n', ...
    D.phase1_success,D.phase1_exitflag,D.min_uniform_relaxation);
fprintf('    [INFEAS] worstPhase1Violation=%+.16e row=%g meta=%s\\n', ...
    D.worst_phase1_violation,D.worst_phase1_row,b3_meta_to_string(D.worst_phase1_meta));
end


function s=b3_meta_to_string(M)
if isempty(M) || ~isstruct(M) || isempty(fieldnames(M))
    s='{}';
    return;
end
F=fieldnames(M);
parts=cell(1,numel(F));
for k=1:numel(F)
    v=M.(F{k});
    if isnumeric(v) && isscalar(v)
        parts{k}=sprintf('%s=%g',F{k},v);
    elseif ischar(v)
        parts{k}=sprintf('%s=%s',F{k},v);
    else
        parts{k}=sprintf('%s=<...>',F{k});
    end
end
s=['{',strjoin(parts,','),'}'];
end
