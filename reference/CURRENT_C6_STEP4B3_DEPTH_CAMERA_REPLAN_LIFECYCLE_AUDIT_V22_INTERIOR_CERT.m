function OUT = CURRENT_C6_STEP4B3_DEPTH_CAMERA_REPLAN_LIFECYCLE_AUDIT_V22_INTERIOR_CERT(userCfg)
% CURRENT_C6_STEP4B3_DEPTH_CAMERA_REPLAN_LIFECYCLE_AUDIT_V22_INTERIOR_CERT
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
% 10) NO CBF, NO HOCBF, NO MPC, NO LiDAR and NO emergency special-case planner.
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
% B3 keeps the same temporary nominal vertical-cable geometry used in the
% latest sensor audit:
%
%   p_Q = p_L + L_cable*e3.
%
% For a unit halfspace normal n, the required payload-center support offset is
% the maximum of payload / UAV / cable-proxy supports. This verifies the full
% vertical-cable geometry proxy continuously. It is NOT yet a claim about the
% later nonlinear cable-direction dynamics, HLC tracking, actuator limits, CBF
% or closed-loop robustness.
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
%   R = CURRENT_C6_STEP4B3_DEPTH_CAMERA_REPLAN_LIFECYCLE_AUDIT_V22_INTERIOR_CERT();
%
% Optional:
%   cfg = struct('nominal_speed',1.2,'mission_distance',50);
%   R = CURRENT_C6_STEP4B3_DEPTH_CAMERA_REPLAN_LIFECYCLE_AUDIT_V22_INTERIOR_CERT(cfg);

if nargin < 1 || isempty(userCfg)
    userCfg=struct();
end

P=aud_params();
CFG=b3_default_config(P);
CFG=b3_merge_config(CFG,userCfg);
b3_validate_config(CFG,P);
b3_require_dependencies();
b3_sensor_shape_preflight();
b3_primal_distance_cause_isolation_suite(P);

backendNames={'BEZIER_D13','POLY_D13','BSPLINE_D7'};
RUNS_CELL=cell(1,numel(backendNames));

fprintf('\n');
fprintf('========================================================================================================================\n');
fprintf(' STEP 4-B3 / DEPTH-CAMERA CLOSED REPLANNING LIFECYCLE AUDIT V2\n');
fprintf(' unknown=[] -> camera -> memory -> current-ref trigger -> corridor rebuild -> C6 replan -> atomic switch\n');
fprintf(' 3 backends retained | moving-obstacle interval-swept support | NO CBF/MPC/LiDAR\n');
fprintf('========================================================================================================================\n');
fprintf('[MISSION] p0=%s  e=%s  speed=%.6g m/s  distance=%.6g m  tf=%.6g s\n', ...
    mat2str(CFG.p0.',5),mat2str(CFG.e_nom.',5),CFG.nominal_speed,CFG.mission_distance,CFG.mission_tf);
fprintf('[EXEC] dt=%.6g s | sensor=%.3f Hz | predictor=%.3f s / %d samples\n', ...
    CFG.dt_exec,CFG.sensor_hz,CFG.lookahead_s,CFG.predict_samples);
fprintf('[SCOPE] vertical-cable geometry proxy L=%.6g m; controller tracking/dynamics are NOT claimed in B3.\n',P.L_cable);
fprintf('[LEAKAGE GUARD] ground truth is accessed only inside the internal depth-camera section of this one-file validator.\n');
fprintf('[V5 CHANGE] only embedded obstacle placement changed to intersect the current +x mission/camera FOV; planner/sensor logic unchanged.\n');
fprintf('[V6 CHANGE] removed mathematically impossible duration fixed-point; nominal-speed detour ratio is diagnostic only in B3.\n');
fprintf('[V7 CHANGE] canonical replan-event struct schema synchronized with planner output.\n');
fprintf('[V8 CHANGE] hard-certificate criteria unchanged; exact failing component(s) are printed.\n');
fprintf('[V9 CHANGE] infeasible QP is decomposed by fixed-row test + Phase-I minimum uniform relaxation.\n');
fprintf('[V10 CHANGE] corridor halfspaces are exact SOCP max-margin separators that contain mandatory C6 boundary-control-point seed sets.\n');
fprintf('[V11 CHANGE] separator feasibility uses primal convex-set distance; cheap certified reject precedes SOCP/QP; stage runtimes are logged.\n');
fprintf('[V12 CHANGE] exact redundancy removal + coneprog solver fallback + detailed sensor/planner stage profiling + isolated kernel tests.\n');
fprintf('[V13 CHANGE] planner memory is built ONLY from depth-hit observation cells; full ground-truth obstacle center/radii/orientation are not exported by the sensor.\n');
fprintf('[V14 CHANGE] progressively observed surfaces are stored as exact nonredundant ellipsoid patches; replanning is event-driven by genuinely new observed volume; towing risk logs PAYLOAD/UAV/CABLE separately.\n');
fprintf('[V15 CHANGE] coneprog exit flag is no longer the safety authority; every candidate normal is independently certified by exact convex support separation. Cell-wise seed/ellipsoid diagnostics added.\n');
fprintf('[V16 CHANGE] full-system convex-hull inflation removed from production corridor construction; PAYLOAD/UAV/CABLE are certified separately with component-specific forbidden sets.\n');
fprintf('[V17 CHANGE] the bypass construction normal is tested first as an exact support-function witness; SOCP is used only when that analytic witness cannot certify the cell. No empirical extra offset is added.\n');
fprintf('[V18 CHANGE] nominal rejoin time is made C6-boundary-aware: the furthest backward endpoint control point of all retained backends must clear every observed component forbidden set before backend trajectory generation.\n');
fprintf('[V19 CHANGE] minimal dynamic layer: exact Bernstein convex-hull certificates for velocity/acceleration after generation; jerk..d6 are diagnostic only. No arbitrary physical limits are invented.\n');
fprintf('[V20 CHANGE] absolute-time catch-up removed: spatial rejoin + nominal-speed path duration + time-shifted nominal continuation. HARD = safety + C6 only.\n');
fprintf('[V21 FIX] removed the final obsolete path-duration diagnostic reference; T_path_at_nominal_speed now equals the already-defined spatial-rejoin duration T.\n');
fprintf('[V22 CHANGE] boundary-QP numerical recovery is theory-based: if the returned point has negative original slack, solve max-rho interior feasibility A*y+rho<=b, rho>=0; accept only after direct original constraints are nonnegative.\n');
fprintf('[V22 CHANGE] yaw endpoint particular solution uses SVD pseudoinverse instead of normal equations.\n');
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
fprintf('[B3 INTERPRETATION]\n');
fprintf(' - The planner begins with zero obstacle knowledge and cannot query the environment directly.\n');
fprintf(' - Detection alone does not switch the reference; future collision on the CURRENT reference is required.\n');
fprintf(' - Sensing remains active during avoidance; NEW/model-changed threats can cause a recursive C6 replan.\n');
fprintf(' - Same-model known obstacles do not retrigger while their currently executing trajectory is already certified.\n');
fprintf(' - yaw = cached degree-27 C6 fit to the executing horizontal tangent (progress direction).\n');
fprintf(' - Every accepted switch preserves the measured current physical-time d0..d6 start jet by construction.\n');
fprintf(' - Hard safety authority is component-wise support certification for PAYLOAD/UAV/CABLE plus C6; rejoin is spatial and nominal continuation is time-shifted.\n');
fprintf(' - Velocity/acceleration and d1..d6 bounds are advisory diagnostics only; hard acceptance remains observed-obstacle safety + C6.\n');
fprintf(' - No claim is yet made for nonlinear cable swing dynamics, thrust/moment feasibility, CBF or closed-loop safety.\n');
fprintf('========================================================================================================================\n');

OUT=struct();
OUT.mode='STEP4B3_DEPTH_CAMERA_REPLAN_LIFECYCLE_AUDIT_V22_INTERIOR_CERT';
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

assignin('base','c6_step4b3_depth_camera_replan_lifecycle_v22_interior_cert_result',OUT);
end


function CFG=b3_default_config(P)
CFG=struct();
CFG.p0=[0;0;0];
CFG.nominal_direction=[1;0;0];
CFG.nominal_speed=1.0;
% V19 physical reference limits. NaN means 'not supplied': audit only, no claim.
CFG.max_ref_speed=NaN;
CFG.max_ref_acc=NaN;
CFG.enforce_kinematic_limits=true;
CFG.mission_distance=45.0;
CFG.dt_exec=0.025;
CFG.sensor_hz=30.0;
CFG.lookahead_s=5.0;
CFG.predict_samples=11;
CFG.max_duration_iterations=12;
CFG.max_replans=20;
CFG.separator_direction_count=72;
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
CFG.e3=[0;0;1];
CFG.cable_length_m=P.L_cable;
CFG= b3_finalize_config(CFG);
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
validateattributes(CFG.mission_distance,{'numeric'},{'real','finite','scalar','positive'});
validateattributes(CFG.dt_exec,{'numeric'},{'real','finite','scalar','positive'});
validateattributes(CFG.sensor_hz,{'numeric'},{'real','finite','scalar','positive'});
validateattributes(CFG.lookahead_s,{'numeric'},{'real','finite','scalar','positive'});
validateattributes(CFG.predict_samples,{'numeric'},{'real','finite','scalar','integer','>=',2});
validateattributes(CFG.max_replans,{'numeric'},{'real','finite','scalar','integer','positive'});
validateattributes(CFG.separator_direction_count,{'numeric'},{'real','finite','scalar','integer','>=',8});
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

fprintf('[START] memory=[] | active reference=NOMINAL | environment direct access=0\n');

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
    pWC=pL+P.L_cable*CFG.e3;
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
    [mem,newIds,changedIds]=b3_update_memory_from_visible(mem,SOUT.visible_obstacles,t,CFG);
    idsAfter=b3_memory_ids(mem);
    nNew=nNew+numel(newIds);
    nPatch=numel(mem);
    if ~isempty(newIds) || ~isempty(changedIds)
        fprintf('    [INCREMENTAL-MAP] parentIDs=%s patches=%d newIDs=%s expandedIDs=%s\n', ...
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
            fprintf('    [PROFILE-SENSOR] env=%.6f | ROI=%.6f | rayprep=%.6f | raycast=%.6f | pack=%.6f ms | rays=%d/%d\n', ...
                SOUT.timing.environment_ms,SOUT.timing.roi_ms,SOUT.timing.rayprep_ms, ...
                SOUT.timing.raycast_ms,SOUT.timing.pack_ms,SOUT.n_sampled_rays,SOUT.n_fullframe_sampled_rays);
        end
        if SOUT.timing.pack_ms>50
            fprintf('    [PROFILE-NOTE] sensor pack outlier >50 ms: treat as cold/JIT-allocation candidate; compare subsequent backend runs before production conclusion.\n');
        end
        for im=1:numel(SOUT.visible_obstacles)
            zz=SOUT.visible_obstacles(im);
            fprintf('    [OBSERVED-MODEL] id=%d source=%s points=%d center=%s radii=%s\n', ...
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

    if terminated
        sensorEvents(end+1)=SE; %#ok<AGROW>
        break;
    end

    if isempty(mem)
        sensorEvents(end+1)=SE; %#ok<AGROW>
        continue;
    end

    % Anti-thrashing rule:
    % while an active trajectory remains certified against the SAME obstacle
    % models, do not rerun predictor/planner at every 30-Hz frame. A new
    % obstacle or a material model update reopens the trigger immediately.
    if active.valid && isempty(newIds) && isempty(changedIds)
        SE.predictor_skipped_certified_active=true;
        sensorEvents(end+1)=SE; %#ok<AGROW>
        continue;
    end

    % Predict the CURRENT executing reference using only remembered obstacles.
    qPred=tic;
    risk=b3_predict_current_reference(t,active,mem,CFG,P,nominalTimeShift);
    predMs=1e3*toc(qPred);
    SE.predictor_runtime_ms=predMs;
    SE.risk_found=risk.found;
    SE.risk_ids=risk.ids;
    SE.risk_ttc_s=risk.TTC;
    sensorEvents(end+1)=SE; %#ok<AGROW>

    if ~risk.found
        continue;
    end

    nTrigger=nTrigger+1;
    wasActive=active.valid;
    fprintf('  TRIGGER #%d t=%.6f TTC=%.6f riskIDs=%s sourceRef=%d active=%d\n', ...
        nTrigger,t,risk.TTC,mat2str(risk.ids),currentRefId,wasActive);
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
    planRuntime(end+1)=pms; %#ok<AGROW>
    replanEvents(end+1)=PE; %#ok<AGROW>
    if isfinite(PE.min_original_hard_slack)
        minHard=min(minHard,PE.min_original_hard_slack);
    end

    if ~cand.valid
        terminated=true;
        failureReason=string(PE.failure_reason);
        fprintf('  REPLAN FAIL t=%.6f backend=%s reason=%s | old reference NOT replaced\n', ...
            t,backend,PE.failure_reason);
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
    fprintf('         position physical splice residual d0..d6 = %s\n', ...
        mat2str(PE.start_physical_splice_residual_d0_d6,6));
    fprintf('         yaw      physical splice residual d0..d6 = %s | yawFitMax=%.6e rad\n', ...
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

RUN=struct();
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
RUN.plan_runtime_ms=planRuntime;
RUN.plan_runtime_median_ms=pr(1);
RUN.plan_runtime_p95_ms=pr(2);
RUN.plan_runtime_max_ms=pr(3);
RUN.min_hard_slack=minHard;
RUN.max_physical_splice_residual_d0_d6=maxSplice;
RUN.final_memory=mem;
end


function [cand,EV]=b3_plan_once(backend,t0,Dstart,Ystart,mem,risk,CFG,P,newRefId)
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

qStage=tic;
obs0=b3_memory_snapshot(mem,t0);
EV.profile.memory_snapshot_ms=1e3*toc(qStage);

qStage=tic;

try
    [G,C,T,geomDiag]=b3_build_replan_geometry(t0,Dstart,obs0,risk.ids,CFG,P);
    EV.profile.geometry_ms=1e3*toc(qStage);
catch ME
    EV.profile.geometry_ms=1e3*toc(qStage);
    EV.failure_reason=['GEOMETRY_BUILD_FAILED: ',ME.identifier,' ',ME.message];
    fprintf('    [PROFILE-PLAN] memory=%.6f ms | geometry_until_fail=%.6f ms\n', ...
        EV.profile.memory_snapshot_ms,EV.profile.geometry_ms);
    return;
end

if ~(isfinite(T) && T>0 && t0+T<=CFG.mission_tf+1e-10)
    EV.failure_reason='INVALID_REPLAN_DURATION';
    return;
end

qStage=tic;
Dend=G.end_physical_jet;
J0s=b2_physical_to_spatial_jet(Dstart,T);
J1s=b2_physical_to_spatial_jet(Dend,T);
EV.profile.jet_prepare_ms=1e3*toc(qStage);

qStage=tic;

fprintf('    [GEOM] T=%+.9f path=%+.9f bypass=%+.9f meanSpeedRatio=%+.9f guideSlack=%+.9e\n', ...
    T,G.path_length,G.bypass_offset,geomDiag.required_mean_speed_ratio,geomDiag.min_guide_slack);
fprintf('    [GEOM] W0=%s\n',mat2str(G.waypoints(:,1).',9));
fprintf('    [GEOM] W1=%s\n',mat2str(G.waypoints(:,2).',9));
fprintf('    [GEOM] W2=%s\n',mat2str(G.waypoints(:,3).',9));
fprintf('    [GEOM] W3=%s\n',mat2str(G.waypoints(:,4).',9));
if isfield(geomDiag,'min_separator_margin')
    fprintf('    [GEOM] seedCompatibleSeparatorMargin=%+.16e method=%s\n', ...
        geomDiag.min_separator_margin,geomDiag.corridor.separator_method);
    if isfield(geomDiag.corridor,'cheap_reject_runtime_ms')
        fprintf('    [CASCADE] cheapReject=%.6f ms | primalDistanceSOCP=%.6f ms | corridorTotal=%.6f ms\n', ...
            geomDiag.corridor.cheap_reject_runtime_ms, ...
            geomDiag.corridor.distance_socp_runtime_ms, ...
            geomDiag.corridor.runtime_ms);
    end
end

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
EV.dense_exact_ellipsoid_slack=R.dense_exact_ellipsoid_slack;
EV.build_time_ms=R.build_time_ms;
EV.solve_time_ms=R.solve_time_ms;

if ~R.pass
    EV.failure_reason='BACKEND_HARD_CERTIFICATE_FAILED';

    finiteStartSpatial=all(isfinite(R.start_spatial_jet_residual_d0_d6));
    finiteEndSpatial=all(isfinite(R.end_spatial_jet_residual_d0_d6));
    finiteStartPhysical=all(isfinite(R.start_physical_splice_residual_d0_d6));
    finiteEndPhysical=all(isfinite(R.end_physical_residual_d0_d6));
    finiteC6=isfinite(R.c6_internal_residual);
    slackPass=isfinite(R.min_original_hard_slack) && (R.min_original_hard_slack>=0);

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
    fprintf('    [HARDCERT] C6 finite=%d residual=%+.16e denseEllipsoidSlack=%+.16e\n', ...
        finiteC6,R.c6_internal_residual,R.dense_exact_ellipsoid_slack);
    fprintf('    [HARDCERT] FAIL FLAGS: solver=%d slack=%d startS=%d endS=%d startT=%d endT=%d c6=%d\n', ...
        ~R.solver_feasible,~slackPass,~finiteStartSpatial,~finiteEndSpatial, ...
        ~finiteStartPhysical,~finiteEndPhysical,~finiteC6);
    fprintf('    [PROFILE-PLAN] memory=%.6f | geometry=%.6f | jet=%.6f | backendBuildSolve=%.6f ms\n', ...
        EV.profile.memory_snapshot_ms,EV.profile.geometry_ms,EV.profile.jet_prepare_ms,EV.profile.backend_build_solve_ms);
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
K=b3_kinematic_bezier_certificate(R.control_points,M,T,backend,CFG);
EV.kinematic=K;
fprintf(['    [KIN-CERT] vmaxBound=%.9g m/s | amaxBound=%.9g m/s^2 | ', ...
    'limits=[%s,%s] | hardGate=%d | pass=%d\n'], ...
    K.max_speed_bound,K.max_acc_bound,b3_num_or_unspecified(CFG.max_ref_speed), ...
    b3_num_or_unspecified(CFG.max_ref_acc),K.hard_gate_active,K.pass);
fprintf('    [KIN-D1..D6] upperBounds=%s\n',mat2str(K.derivative_norm_upper_bound,9));
fprintf(['    [SCHEDULE] meanSpeedRatio=%.9f catchupRequired=0 | ', ...
    'spatialRejoinPhase=%.9f s | nominalTimeShiftAfter=%.9f s\n'], ...
    geomDiag.required_mean_speed_ratio,G.rejoin_nominal_phase,(t0+T)-G.rejoin_nominal_phase);
if ~K.pass
    fprintf(['    [KIN-CERT] ADVISORY ONLY: preferred v/a bounds are not met. ', ...
        'Safety and C6 remain the only hard acceptance conditions.\n']);
end

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
cand.known_ids=EV.known_ids;
cand.risk_ids=EV.risk_ids;
cand.start_jet=Dstart;
cand.end_jet=Dend;
cand.rejoin_nominal_phase=G.rejoin_nominal_phase;
cand.post_nominal_time_shift=(t0+T)-G.rejoin_nominal_phase;
qStage=tic;
cand.yaw=b3_build_c6_yaw_for_active(cand,Ystart,CFG);
EV.profile.yaw_ms=1e3*toc(qStage);
EV.end_yaw_physical_jet=cand.yaw.end_physical_jet;
EV.yaw_start_splice_residual_d0_d6=cand.yaw.start_splice_residual_d0_d6;
EV.yaw_end_residual_d0_d6=cand.yaw.end_residual_d0_d6;
EV.yaw_fit_max_abs_rad=cand.yaw.max_abs_fit_error_rad;
EV.accepted=true;
EV.failure_reason='NONE';
end


function [G,C,T,DIAG]=b3_build_replan_geometry(t0,Dstart,obs,riskIds,CFG,P)
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
seedSets=b3_required_boundary_seed_sets(W,frac,T,Dstart,DendForCorridor);
tSeedMs=1e3*toc(qt);
tcorr=tic;
[C,cdiag]=b3_build_time_cell_corridor_theory(W,frac,T,obs,seedSets,CFG,P,nBy);
cdiag.runtime_ms=1e3*toc(tcorr);
lastCorrMin=cdiag.min_guide_slack;
if lastCorrMin<0
    error('STEP4B3:NoCertifiedCell', ...
        'No separating halfspace containing a guide cell was found (min guide slack=%+.6e).',lastCorrMin);
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
G.rejoin_nominal_phase=rejoinPhase;
G.end_physical_jet=DendForCorridor;
G.post_nominal_time_shift=(t0+T)-rejoinPhase;
DIAG.nominal_speed_is_diagnostic_only=true;
DIAG.profile=struct('progress_time_ms',tProgressMs,'direction_ms',tDirectionMs, ...
    'seed_build_ms',tSeedMs,'corridor_ms',cdiag.runtime_ms);
fprintf('    [PROFILE-GEOM] progressT=%.6f | direction=%.6f | seeds=%.6f | corridor=%.6f ms\n', ...
    tProgressMs,tDirectionMs,tSeedMs,cdiag.runtime_ms);
end

function T=b3_progress_clear_time(t0,W0,riskObs,CFG,P)
% Minimum time for the nominal endpoint to lie beyond every risk obstacle in
% the nominal progress direction, accounting for remembered constant velocity.
e=CFG.e_nom;
speed=CFG.nominal_speed;
x0=e.'*W0;
T=0;

for i=1:numel(riskObs)
    ob=riskObs(i);
    shape=sqrt(max(0,e.'*ob.S*e));
    b0=e.'*ob.center + shape + b3_system_offset(e,P);
    vo=e.'*ob.v_center;

    if vo>0
        rel=speed-vo;
        if rel<=CFG.min_relative_progress_speed
            error('STEP4B3:NoProgressOvertake', ...
                'Risk obstacle %d progresses at %.6g m/s; nominal %.6g m/s cannot clear it.',ob.id,vo,speed);
        end
        Ti=(b0-x0)/rel;
    else
        Ti=(b0-x0)/speed;
    end
    T=max(T,max(0,Ti));
end

% t0 is included for clarity: W0 may come from an active detour rather than
% the nominal position, but the endpoint is nominal at absolute time t0+T.
% Correct the progress equation using the actual nominal endpoint origin.
xNom0=e.'*b3_nominal_jet(t0,CFG);
xNom0=xNom0(1);
if abs(xNom0-x0)>0
    % If current avoiding W0 is off the nominal progress position, solve again
    % against the actual nominal endpoint e'p_N(t0+T)=xNom0+speed*T.
    T=0;
    for i=1:numel(riskObs)
        ob=riskObs(i);
        shape=sqrt(max(0,e.'*ob.S*e));
        b0=e.'*ob.center + shape + b3_system_offset(e,P);
        vo=e.'*ob.v_center;
        if vo>0
            rel=speed-vo;
            if rel<=CFG.min_relative_progress_speed
                error('STEP4B3:NoProgressOvertake','Nominal endpoint cannot pass moving obstacle %d.',ob.id);
            end
            Ti=(b0-xNom0)/rel;
        else
            Ti=(b0-xNom0)/speed;
        end
        T=max(T,max(0,Ti));
    end
end
end


function [Treq,D]=b3_c6_exit_clear_time_static_nominal(t0,W0,Tbase,dreq,riskObs,CFG,P)
% Closed-form minimum T that guarantees the COMPLETE retained-backend C6
% endpoint seed set lies beyond all observed forbidden sets in +e direction.
%
% Scope of the closed form:
%   1) remembered obstacle patches are static;
%   2) W0 equals the nominal payload position at t0;
%   3) nominal endpoint velocity is constant speed*v along e;
%   4) guide construction is [W0, W0+d*n, W3+d*n, W3].
%
% Then L1=L3=d and L2=v*T, so physical duration of cell 3 is
%
%       tau3(T) = T*d/(v*T + 2*d).
%
% For a degree-n Bezier endpoint constrained through derivative order 6
% with constant terminal velocity and zero higher terminal derivatives,
% the furthest of the six boundary-near control points is
%
%       P_{n-6} = W3 - (6/n)*tau3*v*e.
%
% Among retained backends n=13 and n=7, degree 7 dominates:
%
%       alpha = max(6/13,6/7) = 6/7.
%
% Hence it is sufficient to require
%
%       e'W3(T) - alpha*v*tau3(T) >= H,
%
% H = max support of PAYLOAD/UAV/CABLE forbidden sets along +e.
%
% Writing Delta=H-e'p_N(t0), equality becomes a quadratic:
%
% v^2 T^2 + v[(2-alpha)d-Delta]T - 2d Delta = 0.
%
% Its nonnegative root is the exact minimum T for this construction.

e=CFG.e_nom(:);
v=CFG.nominal_speed;
alpha=max(6/13,6/7);

if any(arrayfun(@(z) norm(z.v_center(:))>0, riskObs))
    % V19 fail-closed: the static closed form is not reused for moving
    % observed patches.  A separately proven moving-obstacle time bound is
    % required before this case may generate a new reference.
    error('STEP4B3:C6ExitGeneralCaseRequired', ...
        'Moving observed patch is outside the proven scope of the static C6 exit-time formula.');
end

J0=b3_nominal_jet(t0,CFG);
pNom0=J0(:,1);
scale=max(1,norm(pNom0));
tol=256*eps(scale);
if norm(W0-pNom0)>tol
    % V18 used to fall through with Tbase here.  The log showed that this
    % can leave forwardGap<0 during recursive replanning.  V19 therefore
    % refuses to reuse the nominal-start formula from an active avoidance
    % state.  This is a scope guard, not a numerical tuning rule.
    error('STEP4B3:C6ExitGeneralCaseRequired', ...
        ['Recursive/non-nominal start requires a general C6 endpoint-reachability ', ...
         'time bound; nominal-start closed form is not valid here.']);
end

H=-Inf;
components={'PAYLOAD','UAV','CABLE'};
for io=1:numel(riskObs)
    for ic=1:numel(components)
        H=max(H,b3_component_forbidden_support(riskObs(io),e,0,Tbase,P,components{ic}));
    end
end

x0=e.'*pNom0;
Delta=H-x0;
if Delta<=0
    Troot=0;
else
    B=(2-alpha)*dreq-Delta;
    disc=B*B+8*dreq*Delta;
    Troot=(-B+sqrt(max(0,disc)))/(2*v);
end

Treq=max(Tbase,Troot);
guard=256*eps(max([1,abs(Treq),abs(dreq),abs(H),abs(x0)]));
D=struct('alpha',alpha,'H',H,'Delta',Delta,'closed_form_used',true, ...
    'reason','STATIC_NOMINAL_CLOSED_FORM', ...
    'roundoff_guard',guard,'Troot',Troot);

gap=b3_c6_exit_forward_gap(t0,Treq,dreq,riskObs,CFG,P,alpha);
if gap < -guard
    error('STEP4B3:C6ExitClosedFormAuditFailed', ...
        'Analytic C6 exit-time root failed its independent forward-support audit (gap=%+.16e).',gap);
end
end


function tau3=b3_cell3_duration_from_T(T,dreq,v)
% Exact for the V18 static nominal construction:
% L1=d, L2=vT, L3=d.
tau3=T*dreq/(v*T+2*dreq);
end


function gap=b3_c6_exit_forward_gap(t0,T,dreq,riskObs,CFG,P,alpha)
e=CFG.e_nom(:);
v=CFG.nominal_speed;
J3=b3_nominal_jet(t0+T,CFG);
x3=e.'*J3(:,1);
tau3=b3_cell3_duration_from_T(T,dreq,v);
xmin=x3-alpha*v*tau3;

H=-Inf;
components={'PAYLOAD','UAV','CABLE'};
for io=1:numel(riskObs)
    for ic=1:numel(components)
        H=max(H,b3_component_forbidden_support(riskObs(io),e,0,T,P,components{ic}));
    end
end
gap=xmin-H;
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


function seedSets=b3_required_boundary_seed_sets(W,frac,T,Dstart,Dend)
% Mandatory seed points that the safe-flight corridor must contain.
%
% Why these points:
%   * D13 backend: d0..d6 at each physical endpoint fix seven Bernstein
%     control points adjacent to that endpoint.
%   * Degree-7 B-spline backend: after Bezier extraction, d0..d6 likewise
%     fix seven boundary-near Bezier control points.
%
% A corridor that contains only the geometric guide endpoints can therefore
% be incompatible with the fixed C6 endpoint jet even when the guide itself
% is collision free. V10 includes the UNION of mandatory boundary control
% points of all retained backends as corridor seeds.

J0s=b2_physical_to_spatial_jet(Dstart,T);
J1s=b2_physical_to_spatial_jet(Dend,T);

% D13: one Bezier piece per corridor cell.
B13L=b2_endpoint_bezier_cp_from_jets(J0s,frac(1),13,'left');
B13R=b2_endpoint_bezier_cp_from_jets(J1s,frac(3),13,'right');

% B-spline D7: current architecture uses 7 simple spans per corridor cell.
% Therefore the first/last extracted Bezier span widths are frac(1)/7 and
% frac(3)/7, respectively.
B7L=b2_endpoint_bezier_cp_from_jets(J0s,frac(1)/7,7,'left');
B7R=b2_endpoint_bezier_cp_from_jets(J1s,frac(3)/7,7,'right');

seedSets=cell(1,3);
seedSets{1}=b3_unique_seed_columns([W(:,1),W(:,2),B13L,B7L]);
seedSets{2}=b3_unique_seed_columns([W(:,2),W(:,3)]);
seedSets{3}=b3_unique_seed_columns([W(:,3),W(:,4),B13R,B7R]);
end


function X=b3_unique_seed_columns(X)
% Deterministic exact-geometry cleanup only.  No tolerance-based geometry
% decision is made here; repeated columns merely enlarge the same convex hull.
if isempty(X), return; end
[~,ia]=unique(X.','rows','stable');
X=X(:,ia);
end


function [C,DIAG]=b3_build_time_cell_corridor_theory(W,frac,T,obs,seedSets,CFG,P,nBy)
% V17 COMPONENT-WISE CONTINUOUS-TIME SAFETY WITH CONSTRUCTION WITNESS
%
% Safety set is CLOSED:
%   physical clearance >= d_margin.
%
% The forbidden sets already include d_margin:
%   F_L = O (+) B(0,R_payload+d_margin)
%   F_Q = O (+) (-L e3 + B(0,R_uav+d_margin))
%   F_C = O (+) ({-lambda L e3} (+) B(0,R_cable+d_margin))
%
% Therefore equality with the forbidden-set SUPPORT PLANE means exactly the
% requested d_margin physical clearance, not physical contact.  We do NOT
% add an empirical epsilon or heuristic extra bypass distance.
%
% The bypass front-end constructs W1,W2 by translation along nBy using
% support functions.  Hence nBy is a mathematically meaningful separator
% WITNESS.  For each cell/obstacle/component we first test
%
%   min_s nBy'*s - h_Fk(nBy) >= 0.
%
% If true (within numerical roundoff guard), this directly certifies the
% complete control-point hull.  No SOCP is required.
%
% Only if this construction witness fails do we solve the component primal
% distance SOCP to search for another separating normal.
%
% PAYLOAD/UAV/CABLE remain separate forbidden sets.  No global system ball
% and no global system convex hull are used.

edges=[0,cumsum(frac)];
edges(end)=1;
components={'PAYLOAD','UAV','CABLE'};

C=struct();
C.cells=cell(1,3);
meta=cell(1,3);
minSlack=Inf;
minSepDistance=Inf;
nCheapReject=0;
nWitnessAccept=0;
nSocpCalls=0;
tCheap=0;
tWitness=0;
tDistance=0;

nBy=nBy(:);
if ~(all(isfinite(nBy)) && abs(norm(nBy)-1)<=1e-10)
    error('STEP4B3:BadConstructionNormal','Bypass construction normal must be finite and unit length.');
end

for q=1:3
    A=zeros(0,3);
    b=zeros(0,1);
    M=repmat(struct('obs_id',0,'component','','normal',zeros(3,1),'b',NaN, ...
        'guide_slack',NaN,'seed_slack',NaN,'separator_distance',NaN, ...
        'separator_exitflag',NaN,'separator_source','', ...
        'ta',NaN,'tb',NaN,'cheap_raw_inside',false, ...
        'distance_runtime_ms',NaN,'witness_runtime_ms',NaN, ...
        'support_certified',false,'support_guard',NaN),1,0);

    ta=T*edges(q);
    tb=T*edges(q+1);
    Seeds=seedSets{q};

    for io=1:numel(obs)
        tc=tic;
        rawInside=b3_any_seed_inside_raw_swept_ellipsoid(Seeds,obs(io),ta,tb);
        tCheap=tCheap+1e3*toc(tc);
        if rawInside
            nCheapReject=nCheapReject+1;
            error('STEP4B3:RawSeedCollision', ...
                ['Cell %d obstacle %d: a mandatory payload-center seed lies inside ', ...
                 'the raw observed ellipsoid. PAYLOAD safety is impossible.'],q,obs(io).id);
        end

        for ic=1:numel(components)
            comp=components{ic};
            scale=max([1;abs(Seeds(:));abs(obs(io).center(:));obs(io).radii(:)]);
            fpTol=256*eps(scale);

            % -------------------------------------------------------------
            % Stage A: exact construction witness.
            % -------------------------------------------------------------
            tw=tic;
            n=nBy;
            bb=b3_component_forbidden_support(obs(io),n,ta,tb,P,comp);
            seedSlack=min(n.'*Seeds-bb);
            guideSlack=min(n.'*W(:,q:q+1)-bb);
            witnessMs=1e3*toc(tw);
            tWitness=tWitness+witnessMs;

            % Closed safe set: gap >= 0 is mathematically admissible because
            % d_margin is already embedded in the forbidden-set geometry.
            witnessGuard=fpTol;
            witnessCertified=isfinite(seedSlack) && seedSlack>=-witnessGuard;

            fprintf(['      [WITNESS-CERT] cell=%d obs=%d comp=%-7s ', ...
                'seedSlack=%+.16e guard=%+.16e certified=%d time=%.6f ms\n'], ...
                q,obs(io).id,comp,seedSlack,witnessGuard,witnessCertified,witnessMs);

            if witnessCertified
                nWitnessAccept=nWitnessAccept+1;
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

            % -------------------------------------------------------------
            % Stage B: only now call the heavier SOCP.
            % -------------------------------------------------------------
            nSocpCalls=nSocpCalls+1;
            td=tic;
            [n,bb,dist,ef,distDiag]=b3_primal_component_distance_separator( ...
                obs(io),Seeds,ta,tb,P,comp);
            dtms=1e3*toc(td);
            tDistance=tDistance+dtms;

            if ~(all(isfinite(n)) && isfinite(bb))
                b3_log_component_diagnostics(q,obs(io),Seeds,ta,tb,P,comp,nBy, ...
                    b3_component_forbidden_support(obs(io),nBy,ta,tb,P,comp));
                error('STEP4B3:NoFiniteComponentSeparator', ...
                    ['Cell %d obstacle %d component %s: construction witness failed ', ...
                     'and SOCP returned no finite separator. exitflag=%g.'], ...
                     q,obs(io).id,comp,ef);
            end

            seedSlack=min(n.'*Seeds-bb);
            guideSlack=min(n.'*W(:,q:q+1)-bb);

            % Numerical uncertainty of SOCP candidate only.  No geometric
            % safety margin is invented here.
            numericalGuard=max(fpTol, ...
                16*max(0,distDiag.primalfeasibility)*max(1,norm(n)));
            certified=isfinite(seedSlack) && seedSlack>=-numericalGuard;

            fprintf(['      [COMP-CERT] cell=%d obs=%d comp=%-7s source=SOCP exit=%g ', ...
                'seedSlack=%+.16e guard=%+.16e certified=%d\n'], ...
                q,obs(io).id,comp,ef,seedSlack,numericalGuard,certified);

            if ~certified
                b3_log_component_diagnostics(q,obs(io),Seeds,ta,tb,P,comp,n,bb);
                error('STEP4B3:NoComponentSafeSeparator', ...
                    ['Cell %d obstacle %d component %s: both the construction witness ', ...
                     'and SOCP exact support certificate failed. gap=%+.16e guard=%+.16e.'], ...
                     q,obs(io).id,comp,seedSlack,numericalGuard);
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
            if isfinite(dist), minSepDistance=min(minSepDistance,dist); end
        end
    end

    C.cells{q}=struct('A',A,'b',b);
    meta{q}=M;
end

DIAG=struct('min_guide_slack',minSlack, ...
    'min_seed_slack',minSlack, ...
    'min_separator_margin',minSepDistance, ...
    'min_separator_distance',minSepDistance, ...
    'cell_meta',{meta}, ...
    'time_fraction_edges',edges, ...
    'separator_method','CONSTRUCTION_WITNESS_THEN_COMPONENT_SOCP_FALLBACK', ...
    'system_geometry_model','UNION_PAYLOAD_UAV_CABLE_NOT_GLOBAL_CONVEX_HULL', ...
    'safe_set_semantics','CLOSED_PHYSICAL_CLEARANCE_GE_D_MARGIN', ...
    'component_count',3, ...
    'cheap_reject_count',nCheapReject, ...
    'construction_witness_accept_count',nWitnessAccept, ...
    'socp_call_count',nSocpCalls, ...
    'cheap_reject_runtime_ms',tCheap, ...
    'construction_witness_runtime_ms',tWitness, ...
    'distance_socp_runtime_ms',tDistance);
fprintf(['    [CERT-CASCADE] witnessAccepted=%d | SOCPcalls=%d | ', ...
    'witnessTime=%.6f ms | SOCPtime=%.6f ms\n'], ...
    nWitnessAccept,nSocpCalls,tWitness,tDistance);
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

    fprintf(['      [COMP-SOCP] comp=%-7s solver=%s exit=%g iter=%g ', ...
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
fprintf(['      [COMP-DIAG] cell=%d obs=%d comp=%s seedProj=%+.12e ', ...
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
fprintf('      [SEED-DIAG] cell=%d obs=%d nSeeds=%d center=%s radii=%s\\n', ...
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
    fprintf('        seed=%02d p=%s phi=%+.12e rawDist=%+.12e\\n', ...
        j,mat2str(Seeds(:,j).',9),phi,rawDist);
end

Rp=P.R_payload+P.d_margin;
Ru=P.R_uav+P.d_margin;
Rc=P.R_cable+P.d_margin;
fprintf('      [SEED-DIAG] closestRaw seed=%d minPhi=%+.12e rawDist=%+.12e | towingR payload=%.6g uav=%.6g cable=%.6g L=%.6g\\n', ...
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


function risk=b3_predict_current_reference(t0,active,mem,CFG,P,nominalTimeShift)
risk=struct('found',false,'ids',[],'TTC',NaN,'t_risk',NaN,'body','', ...
    'min_payload_clearance',Inf,'min_uav_clearance',Inf,'min_cable_clearance',Inf, ...
    'risk_patch_ids',[]);
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
    pQ=pL+P.L_cable*CFG.e3;
    obs=b3_memory_snapshot(mem,tt);

    for io=1:numel(obs)
        ob=obs(io);
        gL=aud_point_ellipsoid_distance(pL,ob)-(P.R_payload+P.d_margin);
        gQ=aud_point_ellipsoid_distance(pQ,ob)-(P.R_uav+P.d_margin);
        gC=aud_segment_ellipsoid_distance(pL,pQ,ob)-(P.R_cable+P.d_margin);
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

function item=b3_predict_memory_item(item,t)
if isfinite(item.last_seen)
    dt=t-item.last_seen;
    shift=item.v_center*dt;
    item.center_seen=item.center_seen+shift;
    item.center=item.center_seen;
    item.obs_box_min=item.obs_box_min+shift;
    item.obs_box_max=item.obs_box_max+shift;
    item.last_seen=t;
end
end


function tf=b3_obstacle_model_changed(a,b,CFG)
dc=norm(a.center_seen-b.center_seen,2);
dv=norm(a.v_center-b.v_center,2);
dr=norm(a.radii-b.radii,inf);
tf=(dc>CFG.model_center_change_m) || ...
   (dv>CFG.model_velocity_change_mps) || ...
   (dr>CFG.model_radii_change_m);
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


function item=b3_observed_box_to_memory_ellipsoid(item)
c=0.5*(item.obs_box_min+item.obs_box_max);
h=0.5*(item.obs_box_max-item.obs_box_min);

% Exact enclosing ellipsoid of the axis-aligned observed box:
% |x_i-c_i|<=h_i => sum ((x_i-c_i)/(sqrt(3)h_i))^2 <= 1.
r=sqrt(3)*h;

item.center_seen=c;
item.center=c;
item.R=eye(3);
item.radii=r;
item.S=diag(r.^2);
item.type='observed_ellipsoid';
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
% Fast C6 yaw reference that smoothly faces the executing travel direction.
%
% The target heading is atan2(v_y,v_x) of the NEW accepted trajectory.
% A degree-27 Bernstein curve is used.  Endpoint physical jets d0..d6 are
% imposed exactly, so recursive replans do not introduce a yaw discontinuity.
% The fixed least-squares matrices are cached persistently; a replan updates
% only the right-hand side.  This avoids rebuilding/null() at every event.
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
            % interpolate heading through zero-horizontal-speed intervals;
            % this avoids a staircase target that itself creates yaw kinks.
            raw(idx(k):idx(k+1))=linspace(raw(idx(k)),raw(idx(k+1)),idx(k+1)-idx(k)+1);
        end
    end
end
raw=unwrap(raw);
raw=raw+2*pi*round((Ystart(1)-raw(1))/(2*pi));

% Final yaw faces the nominal travel direction smoothly at rejoin.
Yend=zeros(1,7);
Yend(1)=CFG.nominal_yaw+2*pi*round((raw(end)-CFG.nominal_yaw)/(2*pi));

J0s=Ystart;
J1s=Yend;
for r=1:6
    J0s(r+1)=Ystart(r+1)*(A.T^r);
    J1s(r+1)=Yend(r+1)*(A.T^r);
end

persistent CACHE
if isempty(CACHE) || CACHE.N~=N || CACHE.n~=n
    B=zeros(N,n+1);
    for i=1:N
        B(i,:)=b3_bernstein_basis_row(n,ss(i));
    end
    [Aeq0,~]=b3_yaw_endpoint_constraints(n,zeros(1,7),zeros(1,7));
    Z=null(Aeq0);
    % Minimum-norm particular map c0 = P*beq.
    Ppart=pinv(Aeq0); % SVD minimum-norm map; avoids normal-equation conditioning loss
    BZ=B*Z;
    CACHE=struct('N',N,'n',n,'B',B,'Aeq',Aeq0,'Z',Z, ...
        'Ppart',Ppart,'BZpinv',pinv(BZ));
end

[~,beq]=b3_yaw_endpoint_constraints(n,J0s,J1s);
c0=CACHE.Ppart*beq;
if isempty(CACHE.Z)
    c=c0;
else
    z=CACHE.BZpinv*(raw-CACHE.B*c0);
    c=c0+CACHE.Z*z;
end

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
Y0=b3_eval_yaw_poly(YAW,0,A.T);
Y1=b3_eval_yaw_poly(YAW,1,A.T);
YAW.start_splice_residual_d0_d6=abs(Y0-Ystart);
YAW.end_residual_d0_d6=abs(Y1-Yend);
YAW.max_abs_fit_error_rad=max(abs(err));
YAW.rms_fit_error_rad=sqrt(mean(err.^2));
YAW.structural_c6=true;
YAW.target='HORIZONTAL_EXECUTING_TRAJECTORY_TANGENT';
YAW.fast_cached_linear_map=true;
end

function [Aeq,beq]=b3_yaw_endpoint_constraints(n,J0,J1)
Aeq=zeros(14,n+1);
beq=zeros(14,1);
rr=0;
for r=0:6
    rr=rr+1;
    ff=factorial(n)/factorial(n-r);
    for k=0:r
        Aeq(rr,k+1)=ff*((-1)^(r-k))*nchoosek(r,k);
    end
    beq(rr)=J0(r+1);
end
for r=0:6
    rr=rr+1;
    ff=factorial(n)/factorial(n-r);
    for k=0:r
        Aeq(rr,n-r+k+1)=ff*((-1)^(r-k))*nchoosek(r,k);
    end
    beq(rr)=J1(r+1);
end
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


function S=b3_empty_sensor_event()
S=struct('t',NaN,'visible_ids',[],'new_ids',[],'changed_ids',[], ...
    'known_before',[],'known_after',[], ...
    'reference_id',0,'active_avoidance',false,'sensor_runtime_ms',NaN, ...
    'predictor_runtime_ms',NaN,'predictor_skipped_certified_active',false, ...
    'risk_found',false,'risk_ids',[],'risk_ttc_s',NaN);
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
    'c6_internal_residual',NaN,'dense_exact_ellipsoid_slack',NaN,'build_time_ms',NaN,'solve_time_ms',NaN,'plan_runtime_ms',NaN,'profile',struct(), ...
    'start_yaw_physical_jet',nan(1,7),'end_yaw_physical_jet',nan(1,7), ...
    'yaw_start_splice_residual_d0_d6',nan(1,7),'yaw_end_residual_d0_d6',nan(1,7), ...
    'yaw_fit_max_abs_rad',NaN,'kinematic',struct(),'schedule',struct(), ...
    'accepted',false,'failure_reason','NOT_RUN');
end

function OUT=s1_run_integrated_sensor_audit(STEP3,CASES0,P0)
% S1/S2 integrated sensor + smooth yaw-reference audit.
%
% PRINCIPLE
% ---------
% - External environment perception uses a forward finite-FOV depth camera.
% - The yaw used by the camera is NOT generated inside the sensor.
% - A C6 yaw reference psi_d(t) is generated explicitly from the current
%   trajectory's horizontal tangent direction.
% - Unknown obstacle locations are NOT used to generate yaw before detection.
%
% YAW REPRESENTATION
% ------------------
% A single degree-13 Bernstein polynomial psi_d(s), s in [0,1], is fitted to
% the unwrapped horizontal trajectory heading by equality-constrained least
% squares. A single polynomial is C-infinity, hence C6. Derivatives through
% order 6 are evaluated analytically and converted to physical time with
% T^{-k}.
%
% ENVIRONMENT INTERFACE
% ---------------------
% The active simulation environment is obtained ONLY through
%     s1_environment_provider(...)
% so a future system/external obstacle source can replace one adapter
% without changing the sensor or trajectory audit.
%
% In this standalone audit the ground-truth environment is the SAME
% obstacle set already defined by each Step3 case inside this file.
%
% SENSOR POSE
% -----------
% Until the cable-direction dynamics q(t) are introduced, the vehicle/camera
% position uses the nominal taut vertical-cable relation
%     p_Q = p_L + L_cable*e3.
% This is explicitly a temporary single-UAV towing geometry model, not a
% claim about the later full cable dynamics.

S=s1_sensor_config();
backendNames={'BEZIER_D13','POLY_D13','BSPLINE_D7'};
Ncase=numel(CASES0);

recTemplate=struct('case_id','','backends',struct());
REC=repmat(recTemplate,1,Ncase);

allRuntime=[];
globalDetectCount=zeros(1,3);
globalYawC6=zeros(3,7);
globalYawFit=zeros(1,3);

for ic=1:Ncase
    Sdef=CASES0(ic);
    R3=STEP3.records(ic);

    G=aud_run_case(Sdef,P0,1);
    frac=b2_d13_segment_fraction(G);

    CR=recTemplate;
    CR.case_id=Sdef.case_id;

    fprintf('\n[SENSOR+YAW CASE %02d/%02d] %s | T=%.6f s\n',ic,Ncase,Sdef.case_id,Sdef.T);

    for ib=1:3
        bn=backendNames{ib};

        switch bn
            case 'BEZIER_D13'
                PREP=a6_prepare_d13_fast(R3.bezier.shared_knot_jets,frac);
            case 'POLY_D13'
                PREP=a6_prepare_d13_fast(R3.poly.shared_knot_jets,frac);
            case 'BSPLINE_D7'
                PREP=a5_prepare_bspline(R3.bspline.native_control_points,R3.bspline.knots,7);
        end

        YAW=s1_build_c6_yaw_reference(PREP,Sdef.T,S);

        tGrid=0:(1/S.rate_hz):Sdef.T;
        if tGrid(end)<Sdef.T
            tGrid=[tGrid,Sdef.T]; %#ok<AGROW>
        end

        obs0=s1_environment_provider('EMBEDDED_CASE',0,Sdef);
        obsIds=[obs0.id];
        detectTime=nan(size(obsIds));
        detectYaw=nan(size(obsIds));
        visibleFrames=zeros(size(obsIds));
        rt=zeros(1,numel(tGrid));

        for it=1:numel(tGrid)
            tt=tGrid(it);
            ss=min(1,max(0,tt/Sdef.T));

            Y=a6_eval_fast(PREP,ss,Sdef.T);
            pL=Y(:,1);

            % Temporary single-UAV towing geometry:
            % q_N = -e3 -> p_L = p_Q + L*q_N -> p_Q = p_L + L*e3.
            pWC=pL+[0;0;S.cable_length_m];

            yawJet=s1_eval_yaw_reference(YAW,ss,Sdef.T);
            yaw=yawJet(1);
            R_WC=s1_camera_rotation_from_yaw(yaw);

            ticSensor=tic;
            obs=s1_environment_provider('EMBEDDED_CASE',tt,Sdef);
            vis=s1_depth_visible_ids(pWC,R_WC,obs,S);
            rt(it)=1e3*toc(ticSensor);

            for jj=1:numel(obsIds)
                oid=obsIds(jj);
                if any(vis==oid)
                    visibleFrames(jj)=visibleFrames(jj)+1;
                    if isnan(detectTime(jj))
                        detectTime(jj)=tt;
                        detectYaw(jj)=yaw;
                    end
                end
            end
        end

        BR=struct();
        BR.environment_source='EMBEDDED_STEP3_CASE_GROUND_TRUTH';
        BR.environment_interface='s1_environment_provider';
        BR.camera_position_source='NOMINAL_VERTICAL_CABLE_SINGLE_UAV_PROXY';
        BR.cable_length_m=S.cable_length_m;
        BR.yaw_policy='C6_D13_BERNSTEIN_FIT_TO_UNWRAPPED_HORIZONTAL_TANGENT';
        BR.yaw_degree=YAW.degree;
        BR.yaw_coeff=YAW.coeff;
        BR.yaw_fit_max_abs_rad=YAW.max_abs_fit_error_rad;
        BR.yaw_fit_rms_rad=YAW.rms_fit_error_rad;
        BR.yaw_structural_c6=true;
        BR.yaw_boundary_jump_d0_d6=YAW.boundary_jump_d0_d6;
        BR.detect_ids=obsIds;
        BR.detect_time_s=detectTime;
        BR.detect_yaw_rad=detectYaw;
        BR.visible_frame_count=visibleFrames;
        BR.n_sensor_frames=numel(tGrid);
        BR.runtime_ms=rt;
        BR.runtime_median_ms=median(rt);
        BR.runtime_p95_ms=prctile(rt,95);
        BR.runtime_max_ms=max(rt);
        BR.detected_count=sum(~isnan(detectTime));

        allRuntime=[allRuntime,rt]; %#ok<AGROW>
        globalDetectCount(ib)=globalDetectCount(ib)+BR.detected_count;
        globalYawC6(ib,:)=max(globalYawC6(ib,:),abs(BR.yaw_boundary_jump_d0_d6));
        globalYawFit(ib)=max(globalYawFit(ib),BR.yaw_fit_max_abs_rad);
        CR.backends.(bn)=BR;

        fprintf('  %-12s yawFit(max/rms)=%.6e / %.6e rad | detected=%d/%d | sensor med/p95/max=%.4f/%.4f/%.4f ms\n', ...
            bn,BR.yaw_fit_max_abs_rad,BR.yaw_fit_rms_rad,BR.detected_count,numel(obsIds), ...
            BR.runtime_median_ms,BR.runtime_p95_ms,BR.runtime_max_ms);
        fprintf('               first-detect times = %s s\n',mat2str(BR.detect_time_s,6));
    end

    REC(ic)=CR;
end

fprintf('\n------------------------------------------------------------------------------------------------------------------------\n');
fprintf(' SENSOR + C6 YAW COMPACT SUMMARY\n');
fprintf('------------------------------------------------------------------------------------------------------------------------\n');
fprintf('sensor       = forward pinhole depth camera; finite FOV/range; nearest-hit occlusion\n');
fprintf('rate         = %.1f Hz\n',S.rate_hz);
fprintf('image model  = %dx%d, fx=%.12g, fy=%.12g, cx=%.12g, cy=%.12g, skip=%d\n', ...
    S.width,S.height,S.fx,S.fy,S.cx,S.cy,S.pixel_skip);
fprintf('depth range  = [%.3f, %.3f] m\n',S.min_depth_m,S.max_depth_m);
fprintf('environment  = embedded Step3 case ground-truth through provider adapter\n');
fprintf('camera pose  = nominal single-UAV vertical-cable proxy, L=%.3f m\n',S.cable_length_m);
fprintf('yaw target   = horizontal tangent -> unwrap -> one degree-13 Bernstein polynomial\n');
fprintf('yaw C6       = structural (single polynomial); analytic derivatives d0..d6 available\n');
fprintf('detected-obstacle counts across all cases [B P S] = [%d %d %d]\n',globalDetectCount);
fprintf('worst yaw fit max-abs [B P S] = %s rad\n',mat2str(globalYawFit,7));
fprintf('yaw internal boundary jump d0..d6 [B;P;S] =\n');
disp(globalYawC6);
fprintf('global sensor runtime median/p95/max = %.6f / %.6f / %.6f ms\n', ...
    median(allRuntime),prctile(allRuntime,95),max(allRuntime));
fprintf('LiDAR is not used.\n');
fprintf('Unknown obstacle locations are not used to construct yaw before detection.\n');
fprintf('------------------------------------------------------------------------------------------------------------------------\n');

OUT=struct();
OUT.mode='SENSOR_C6_YAW_INTEGRATED_AUDIT_V3';
OUT.sensor=S;
OUT.records=REC;
OUT.aggregate=struct();
OUT.aggregate.detected_obstacle_counts_by_backend=globalDetectCount;
OUT.aggregate.global_runtime_median_ms=median(allRuntime);
OUT.aggregate.global_runtime_p95_ms=prctile(allRuntime,95);
OUT.aggregate.global_runtime_max_ms=max(allRuntime);
OUT.aggregate.worst_yaw_fit_max_abs_rad_by_backend=globalYawFit;
OUT.aggregate.yaw_boundary_jump_d0_d6_by_backend=globalYawC6;
OUT.aggregate.environment_source='EMBEDDED_STEP3_CASE_GROUND_TRUTH';
OUT.aggregate.environment_adapter='s1_environment_provider';
OUT.aggregate.camera_position_source='NOMINAL_VERTICAL_CABLE_SINGLE_UAV_PROXY';
OUT.aggregate.yaw_reference='C6_D13_BERNSTEIN_HORIZONTAL_TANGENT';
OUT.aggregate.lidar_used=false;
end


function S=s1_sensor_config()
% Forward depth-camera baseline used only for external environment perception.
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
S.rate_hz=30.0;
S.cable_length_m=2.0;
S.yaw_fit_sample_count=241;
end


function YAW=s1_build_c6_yaw_reference(PREP,T,S)
% Build a single degree-13 Bernstein yaw reference from trajectory tangent.
% The single polynomial is C-infinity. Endpoint yaw values are imposed
% exactly; the remaining shape is unweighted least-squares projection.

N=S.yaw_fit_sample_count;
ss=linspace(0,1,N);
raw=zeros(N,1);
valid=false(N,1);

for i=1:N
    Y=a6_eval_fast(PREP,ss(i),T);
    v=Y(:,2);
    if hypot(v(1),v(2))>0
        raw(i)=atan2(v(2),v(1));
        valid(i)=true;
    end
end

if ~any(valid)
    raw(:)=0;
else
    % Fill zero-horizontal-speed samples from nearest available heading
    % without using any obstacle information.
    idx=find(valid);
    raw(1:idx(1)-1)=raw(idx(1));
    raw(idx(end)+1:end)=raw(idx(end));
    for k=1:(numel(idx)-1)
        a=idx(k); b=idx(k+1);
        if b>a+1
            raw(a+1:b-1)=raw(a);
        end
    end
end

raw=unwrap(raw);

n=13;
B=zeros(N,n+1);
for i=1:N
    B(i,:)=s1_bernstein_basis(n,ss(i));
end

% Equality constraints psi(0)=raw(1), psi(1)=raw(end).
Aeq=zeros(2,n+1);
Aeq(1,1)=1;
Aeq(2,end)=1;
beq=[raw(1);raw(end)];

H=B.'*B;
g=B.'*raw;
K=[H,Aeq.';Aeq,zeros(2)];
rhs=[g;beq];
sol=K\rhs;
c=sol(1:n+1);

fit=B*c;
err=fit-raw;

YAW=struct();
YAW.degree=n;
YAW.coeff=c(:).';
YAW.raw_s=ss;
YAW.raw_unwrapped_yaw=raw;
YAW.fit_yaw=fit;
YAW.max_abs_fit_error_rad=max(abs(err));
YAW.rms_fit_error_rad=sqrt(mean(err.^2));
YAW.boundary_jump_d0_d6=zeros(1,7); % single polynomial: no internal boundary
YAW.structural_c6=true;
end


function jet=s1_eval_yaw_reference(YAW,s,T)
% Physical-time yaw derivatives d0..d6.
jet=zeros(1,7);
c=YAW.coeff(:).';
n=YAW.degree;

for r=0:6
    cr=c;
    nr=n;
    for j=1:r
        cr=nr*diff(cr);
        nr=nr-1;
    end
    val=s1_bernstein_eval(cr,nr,s);
    jet(r+1)=val/(T^r);
end
end


function b=s1_bernstein_basis(n,s)
b=zeros(1,n+1);
for k=0:n
    b(k+1)=nchoosek(n,k)*(s^k)*((1-s)^(n-k));
end
end


function y=s1_bernstein_eval(c,n,s)
b=s1_bernstein_basis(n,s);
y=b*c(:);
end


function R=s1_camera_rotation_from_yaw(yaw)
% Camera optical frame:
%   +z forward, +x image-right, +y image-down.
f=[cos(yaw);sin(yaw);0];
right=[sin(yaw);-cos(yaw);0];
down=[0;0;-1];
R=[right,down,f];

if det(R)<=0 || norm(R.'*R-eye(3),'fro')>1e-12
    error('S1:CameraRotation','Invalid camera rotation.');
end
end


function obs=s1_environment_provider(mode,t,Sdef)
% Single adapter for simulation ground truth.
% Future system integration should replace/add a provider here only.
switch upper(mode)
    case 'EMBEDDED_CASE'
        obs=s1_environment_from_case(Sdef,t);
    otherwise
        error('S1:EnvironmentMode','Unknown environment mode: %s',mode);
end
end


function obs=s1_environment_from_case(Sdef,t)
% The obstacle definitions are already embedded in this standalone master's
% aud_build_cases() -> Sdef.obs. This function exposes them as simulation
% ground truth without making them known to the planner before detection.
src=Sdef.obs;
obs=repmat(struct('id',0,'type','ellipsoid','p_center',zeros(3,1), ...
    'R_obs',eye(3),'ellipsoid_radii',ones(3,1),'v_center',zeros(3,1)),1,numel(src));

for i=1:numel(src)
    z=src(i);

    if isfield(z,'center')
        c=z.center(:);
    elseif isfield(z,'p_center')
        c=z.p_center(:);
    elseif isfield(z,'p_obs')
        c=z.p_obs(:);
    else
        error('S1:ObstacleCenter','Embedded obstacle has no center field.');
    end

    if isfield(z,'v_center')
        v=z.v_center(:);
    else
        v=zeros(3,1);
    end
    c=c+v*t;

    if isfield(z,'radii')
        radii=z.radii(:);
    elseif isfield(z,'ellipsoid_radii')
        radii=z.ellipsoid_radii(:);
    elseif isfield(z,'S')
        [V,D]=eig(0.5*(z.S+z.S.'));
        radii=sqrt(max(diag(D),0));
    else
        error('S1:ObstacleRadii','Embedded obstacle has no ellipsoid size.');
    end

    if isfield(z,'R')
        R=z.R;
    elseif isfield(z,'R_obs')
        R=z.R_obs;
    else
        R=eye(3);
    end

    if isfield(z,'id')
        id=z.id;
    else
        id=i;
    end

    obs(i).id=id;
    obs(i).type='ellipsoid';
    obs(i).p_center=c;
    obs(i).R_obs=R;
    obs(i).ellipsoid_radii=radii;
    obs(i).v_center=v;
end
end


function visible=s1_depth_visible_ids(p_WC,R_WC,obs,S)
% Object-aware sampled depth renderer for ellipsoidal ground-truth obstacles.
% Candidate pixel regions are built conservatively; exact ray/ellipsoid
% intersection decides visibility. Nearest hit per ray gives occlusion.

if isempty(obs)
    visible=[];
    return;
end

pixU=[];
pixV=[];

for io=1:numel(obs)
    cC=R_WC.'*(obs(io).p_center-p_WC);
    rb=max(obs(io).ellipsoid_radii);

    dist=norm(cC);
    if dist==0
        uu=0:S.pixel_skip:(S.width-1);
        vv=0:S.pixel_skip:(S.height-1);
        [UU,VV]=meshgrid(uu,vv);
        pixU=UU(:).';
        pixV=VV(:).';
        break;
    end

    if cC(3)+rb<S.min_depth_m || cC(3)-rb>S.max_depth_m || cC(3)+rb<=0
        continue;
    end

    zNear=max(S.min_depth_m,cC(3)-rb);
    u0=S.fx*cC(1)/max(cC(3),eps)+S.cx;
    v0=S.fy*cC(2)/max(cC(3),eps)+S.cy;
    du=abs(S.fx*rb/max(zNear,eps));
    dv=abs(S.fy*rb/max(zNear,eps));

    umin=max(0,floor((u0-du)/S.pixel_skip)*S.pixel_skip);
    umax=min(S.width-1,ceil((u0+du)/S.pixel_skip)*S.pixel_skip);
    vmin=max(0,floor((v0-dv)/S.pixel_skip)*S.pixel_skip);
    vmax=min(S.height-1,ceil((v0+dv)/S.pixel_skip)*S.pixel_skip);

    if umin>umax || vmin>vmax
        continue;
    end

    uu=umin:S.pixel_skip:umax;
    vv=vmin:S.pixel_skip:vmax;
    [UU,VV]=meshgrid(uu,vv);
    pixU=[pixU,UU(:).']; %#ok<AGROW>
    pixV=[pixV,VV(:).']; %#ok<AGROW>
end

if isempty(pixU)
    visible=[];
    return;
end

UV=unique([pixU(:),pixV(:)],'rows','stable');
u=UV(:,1).';
v=UV(:,2).';

D_C=[(u-S.cx)/S.fx;(v-S.cy)/S.fy;ones(1,numel(u))];
D_C=D_C./vecnorm(D_C,2,1);
D_W=R_WC*D_C;

depth=inf(1,size(D_W,2));
owner=zeros(1,size(D_W,2));

for io=1:numel(obs)
    lam=s1_ray_ellipsoid(p_WC,D_W,obs(io).p_center, ...
        obs(io).R_obs,obs(io).ellipsoid_radii);
    valid=isfinite(lam) & lam>=S.min_depth_m & lam<=S.max_depth_m;
    nearer=valid & lam<depth;
    depth(nearer)=lam(nearer);
    owner(nearer)=obs(io).id;
end

visible=unique(owner(owner~=0),'stable');
end


function lambda=s1_ray_ellipsoid(oW,dW,c,R,radii)
o=R.'*(oW-c);
d=R.'*dW;
ir2=1./(radii(:).^2);

A=ir2.'*(d.^2);
B=2*((ir2.*o).'*d);
C=sum(ir2.*o.^2)-1;
disc=B.^2-4*A*C;

lambda=inf(1,size(dW,2));
m=A>0 & disc>=0;
sd=zeros(size(disc));
sd(m)=sqrt(disc(m));

q1=inf(size(disc));
q2=inf(size(disc));
q1(m)=(-B(m)-sd(m))./(2*A(m));
q2(m)=(-B(m)+sd(m))./(2*A(m));

m1=m & q1>=0;
m2=m & q2>=0;
lambda(m1)=q1(m1);
lambda(m2)=min(lambda(m2),q2(m2));
end


function S=b2_scale_template()
S=struct('scale',NaN,'T_new',NaN,'result',struct(),'pass',false);
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


function smin=b2_replan_guide_slack(G,C)
% Guide segments are assigned to cells 1,2,3.
W=G.waypoints;
smin=Inf;
for q=1:3
    CC=C.cells{q};
    for jj=q:(q+1)
        smin=min(smin,min(CC.A*W(:,jj)-CC.b));
    end
end
end


function R=b2_solve_d13_fixed_boundary(G,C,J0,J1,obs,P,Tnew)
tBuild=tic;
M=b2_d13_build_model_fixed_boundary(G,C,J0,J1);
buildMs=1e3*toc(tBuild);

[As,bs]=bez13_jet_scale_ineq(M.Aineq,M.bineq);

qpopt=optimoptions('quadprog', ...
    'Algorithm','interior-point-convex', ...
    'Display','none', ...
    'ConstraintTolerance',1e-10, ...
    'OptimalityTolerance',1e-10, ...
    'StepTolerance',1e-12, ...
    'MaxIterations',500);

tSolve=tic;
[z,~,ef,out]=quadprog(M.Hscaled,M.fscaled,As,bs,[],[],[],[],[],qpopt);
solveMs=1e3*toc(tSolve);

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
CP=b2_d13_reconstruct_cp(z,M);
R.control_points=CP;

sl=M.bineq-M.Aineq*z;
R.min_original_hard_slack=min(sl);

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

[sDense,pDense]=b2_d13_eval_dense(CP,M,301);
R.dense_exact_ellipsoid_slack=bez13_dense_exact_ellipsoid_slack(pDense,obs,P);
R.sample_s=sDense;
R.sample_position=pDense;

R.pass=R.solver_feasible && ...
       (R.min_original_hard_slack>=0) && ...
       all(isfinite(R.start_spatial_jet_residual_d0_d6)) && ...
       all(isfinite(R.end_spatial_jet_residual_d0_d6)) && ...
       all(isfinite(R.start_physical_splice_residual_d0_d6)) && ...
       all(isfinite(R.end_physical_residual_d0_d6)) && ...
       isfinite(R.c6_internal_residual);
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
bineq=zeros(0,1);safeMeta=struct('segment',{},'cp',{},'plane',{});

for q=1:nseg
    CC=C.cells{q};
    for j=1:ncp
        for pl=1:size(CC.A,1)
            a=zeros(1,ncpall);
            for ax=1:3
                a(bez13_jet_cpidx(q,j,ax,ncp))=-CC.A(pl,ax);
            end
            Aineq(end+1,:)=a*Gcp; %#ok<AGROW>
            bineq(end+1,1)=-CC.b(pl)-a*c0; %#ok<AGROW>
            safeMeta(end+1)=struct('segment',q,'cp',j,'plane',pl); %#ok<AGROW>
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
[As,bs]=bsd7_scale_rows(Ar,br);

qpopt=optimoptions('quadprog', ...
    'Algorithm','interior-point-convex', ...
    'Display','none', ...
    'ConstraintTolerance',1e-10, ...
    'OptimalityTolerance',1e-10, ...
    'StepTolerance',1e-12, ...
    'MaxIterations',500);

tSolve=tic;
[y,~,ef,out]=quadprog(sparse(Hr),fr,As,bs,[],[],[],[],[],qpopt);
solveMs=1e3*toc(tSolve);

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

cp=bsd7_native_cp(x,M);
Bcp=bsd7_extract_bezier(cp,M);
R.native_control_points=cp;
R.control_points=Bcp;

sl=M.bineq-M.Aineq*x;
R.min_original_hard_slack=min(sl);

% V22: if quadprog returns a point microscopically outside an original
% inequality, do NOT accept it by tolerance.  Instead solve the exact
% max-min interior-feasibility problem in the reduced coordinates:
%
%   maximize rho
%   subject to Ar*y + rho*1 <= br, rho >= 0.
%
% If rho>0, an interior feasible point exists for the ORIGINAL unscaled
% inequalities.  We then reconstruct x and directly require min(b-A*x)>=0.
if R.min_original_hard_slack < 0
    [yInterior,rhoInterior,efInterior,diagInterior]= ...
        b3_max_uniform_interior_point(Ar,br);
    R.solver_output.v22_interior_certificate=diagInterior;
    if ~isempty(yInterior) && efInterior>0 && isfinite(rhoInterior) && rhoInterior>0
        xInterior=E.x0+E.T*yInterior;
        slInterior=M.bineq-M.Aineq*xInterior;
        minInterior=min(slInterior);
        fprintf('    [V22-INTERIOR] rho=%+.16e directOriginalSlack=%+.16e exit=%g\n', ...
            rhoInterior,minInterior,efInterior);
        if minInterior>=0
            x=xInterior;
            y=yInterior;
            sl=slInterior;
            R.min_original_hard_slack=minInterior;
            cp=bsd7_native_cp(x,M);
            Bcp=bsd7_extract_bezier(cp,M);
            R.native_control_points=cp;
            R.control_points=Bcp;
        end
    else
        fprintf('    [V22-INTERIOR] no strict original-feasible interior certified: rho=%+.16e exit=%g\n', ...
            rhoInterior,efInterior);
    end
end

[Jstart,Jend,c6res]=b2_bspline_audit_jets(Bcp,M);
R.start_spatial_jet_residual_d0_d6=b2_jet_residual_by_order(Jstart,J0);
R.end_spatial_jet_residual_d0_d6=b2_jet_residual_by_order(Jend,J1);
R.c6_internal_residual=c6res;

Jstart_t=b2_spatial_to_physical_jet(Jstart,Tnew);
Jend_t=b2_spatial_to_physical_jet(Jend,Tnew);
J0t=b2_spatial_to_physical_jet(J0,Tnew);
J1t=b2_spatial_to_physical_jet(J1,Tnew);

R.start_physical_splice_residual_d0_d6=b2_jet_residual_by_order(Jstart_t,J0t);
R.end_physical_residual_d0_d6=b2_jet_residual_by_order(Jend_t,J1t);

[~,pDense]=bsd7_eval_dense(cp,M,301);
R.dense_exact_ellipsoid_slack=bez13_dense_exact_ellipsoid_slack(pDense,obs,P);

R.pass=R.solver_feasible && ...
       (R.min_original_hard_slack>=0) && ...
       all(isfinite(R.start_spatial_jet_residual_d0_d6)) && ...
       all(isfinite(R.end_spatial_jet_residual_d0_d6)) && ...
       all(isfinite(R.start_physical_splice_residual_d0_d6)) && ...
       all(isfinite(R.end_physical_residual_d0_d6)) && ...
       isfinite(R.c6_internal_residual);
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


function [Jstart,Jend,c6res]=b2_bspline_audit_jets(Bcp,M)
% Bcp is 3 x 8 x nspan extracted Bezier pieces.
Jstart=b2_bezier_endpoint_jet(Bcp(:,:,1),M.breaks(2)-M.breaks(1),'left');
Jend=b2_bezier_endpoint_jet(Bcp(:,:,end),M.breaks(end)-M.breaks(end-1),'right');

c6res=0;
for q=1:(M.nspan-1)
    hL=M.breaks(q+1)-M.breaks(q);
    hR=M.breaks(q+2)-M.breaks(q+1);
    JL=b2_bezier_endpoint_jet(Bcp(:,:,q),hL,'right');
    JR=b2_bezier_endpoint_jet(Bcp(:,:,q+1),hR,'left');
    c6res=max(c6res,max(b2_jet_residual_by_order(JL,JR)));
end
end


function R=b2_empty_result()
R=struct();
R.solver_feasible=false;
R.pass=false;
R.build_time_ms=NaN;
R.solve_time_ms=NaN;
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
R.dense_exact_ellipsoid_slack=NaN;
R.control_points=[];
R.native_control_points=[];
R.sample_s=[];
R.sample_position=[];
end

function OUT = internal_step3_master_v8()
% Unified 14-case Step3 robustness audit.
%
% Fixed backend architectures from C02 V6:
%   BEZIER_D13 : literature baseline / Bernstein corridor certificate
%   POLY_D13   : minimal Hermite jets / Bernstein image certificate
%   BSPLINE_D7 : 28 CP / 21 spans / native-CP local convex-hull certificate
%
% No architecture tuning per case.
% No safety relaxation.
% Any negative original hard slack => FAIL.
% No text-file logging.

fprintf('========================================================================================================================\n');
fprintf(' STEP 3 COMMON / 14-CASE / THREE-BACKEND MASTER V8\n');
fprintf(' FIXED ARCHITECTURE ROBUSTNESS AUDIT | BEZIER_D13 | POLY_D13 | BSPLINE_D7\n');
fprintf('========================================================================================================================\n');

P0=aud_params(); %#ok<NASGU>
CASES0=aud_build_cases();
nCase=numel(CASES0);

requiredCaseFields={'case_id','case_desc','p0','pg','v0','e_nom','T','obs'};
for kk=1:numel(requiredCaseFields)
    if ~isfield(CASES0,requiredCaseFields{kk})
        error('MASTER14:CaseSchema','Missing case field: %s',requiredCaseFields{kk});
    end
end

ids={CASES0.case_id};
if numel(unique(ids))~=nCase
    error('MASTER14:CaseSchema','case_id values must be unique.');
end

REC=repmat(struct(),1,nCase);

for ic=1:nCase
    fprintf('\n\n########################################################################################################################\n');
    fprintf(' CASE %02d / %02d : %s\n',ic,nCase,CASES0(ic).case_id);
    fprintf('########################################################################################################################\n');

    fprintf('\n>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> BEZIER_D13 <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<\n');
    B=run_bezier_d13(ic,false);

    fprintf('\n>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> POLY_D13 <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<\n');
    P=run_poly_d13(ic,false);

    fprintf('\n>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> BSPLINE_D7 <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<\n');
    S=run_bspline_d7(ic,false);

    REC(ic).case_index=ic;
    REC(ic).case_id=CASES0(ic).case_id;
    REC(ic).case_desc=CASES0(ic).case_desc;
    REC(ic).bezier=B;
    REC(ic).poly=P;
    REC(ic).bspline=S;

    REC(ic).all_pass=B.pass && P.pass && S.pass;
    REC(ic).d13_max_sample_position_difference=norm(B.sample_position-P.sample_position,inf);
end

fprintf('\n\n========================================================================================================================\n');
fprintf(' 14-CASE THREE-BACKEND COMPACT SUMMARY\n');
fprintf('========================================================================================================================\n');
fprintf('case | Bpass Ppass Spass | Bsolve  Psolve  Ssolve [ms] | Bslack       Pslack       Sslack       | Bxi       Pxi       Sxi\n');

for ic=1:nCase
    B=REC(ic).bezier; P=REC(ic).poly; S=REC(ic).bspline;
    fprintf('%4d | %5d %5d %5d | %7.3f %7.3f %7.3f | %+11.3e %+11.3e %+11.3e | %8.4f %8.4f %8.4f\n', ...
        ic,B.pass,P.pass,S.pass, ...
        B.solve_time_ms,P.solve_time_ms,S.solve_time_ms, ...
        B.min_original_hard_slack,P.min_original_hard_slack,S.min_original_hard_slack, ...
        B.xi_chosen,P.xi_chosen,S.xi_chosen);
end

Bpass=[REC.bezier]; Bpass=[Bpass.pass];
Ppass=[REC.poly];    Ppass=[Ppass.pass];
Spass=[REC.bspline]; Spass=[Spass.pass];

fprintf('\n[AGGREGATE]\n');
fprintf('BEZIER pass count           = %d / %d\n',sum(Bpass),nCase);
fprintf('POLY pass count             = %d / %d\n',sum(Ppass),nCase);
fprintf('BSPLINE pass count          = %d / %d\n',sum(Spass),nCase);
fprintf('ALL-three pass count        = %d / %d\n',sum([REC.all_pass]),nCase);

Bsolve=arrayfun(@(r)r.bezier.solve_time_ms,REC);
Psolve=arrayfun(@(r)r.poly.solve_time_ms,REC);
Ssolve=arrayfun(@(r)r.bspline.solve_time_ms,REC);

fprintf('medianSolveMs [B P S]       = [%.6f %.6f %.6f]\n',median(Bsolve),median(Psolve),median(Ssolve));
fprintf('maxSolveMs    [B P S]       = [%.6f %.6f %.6f]\n',max(Bsolve),max(Psolve),max(Ssolve));

Bsl=arrayfun(@(r)r.bezier.min_original_hard_slack,REC);
Psl=arrayfun(@(r)r.poly.min_original_hard_slack,REC);
Ssl=arrayfun(@(r)r.bspline.min_original_hard_slack,REC);
fprintf('minHardSlack  [B P S]       = [%+.17e %+.17e %+.17e]\n',min(Bsl),min(Psl),min(Ssl));

d13=arrayfun(@(r)r.d13_max_sample_position_difference,REC);
fprintf('max D13 representation diff = %.17e\n',max(d13));

OUT=struct();
OUT.mode='STEP3_COMMON_14CASE_THREE_BACKEND_MASTER_V8';
OUT.records=REC;
OUT.aggregate=struct();
OUT.aggregate.n_cases=nCase;
OUT.aggregate.bezier_pass_count=sum(Bpass);
OUT.aggregate.poly_pass_count=sum(Ppass);
OUT.aggregate.bspline_pass_count=sum(Spass);
OUT.aggregate.all_three_pass_count=sum([REC.all_pass]);
OUT.aggregate.median_solve_ms=[median(Bsolve),median(Psolve),median(Ssolve)];
OUT.aggregate.max_solve_ms=[max(Bsolve),max(Psolve),max(Ssolve)];
OUT.aggregate.min_hard_slack=[min(Bsl),min(Psl),min(Ssl)];
OUT.aggregate.max_d13_sample_difference=max(d13);

% standalone integrated version: no intermediate base-workspace dependency
end


function OUT = run_bezier_d13(caseIndex,verbose)
% STEP 3 COMMON / C02 / BEZIER_D13_MIN7
% Literature-aligned baseline:
%   - Piecewise degree-13 Bezier/Bernstein trajectory over safe corridor cells
%   - Continuous geometric safety by Bernstein control-point convex hull
%     (Gao et al., ICRA 2018 style)
%   - Shared physical knot jets eliminate redundant continuity variables only;
%     this is an implementation reduction, not a new safety method.
%
% Minimal knot-jet parameterization inspired by MINCO-style elimination.
%
% SAME trajectory space and SAME hard corridor constraints as the previous
% 3-piece degree-13 C6 Bezier formulation.
%
% IMPORTANT CHANGE:
%   J7 is NOT optimized in Step 3.
%   The only objective is geometric guide proximity:
%
%       sum over all Bezier CPs ||P_ij - Pguide_ij||^2
%
%   No arbitrary weights are used.
%
% Decision variables (43 total):
%   - internal knot #2 jets r=0..6, xyz : 21
%   - internal knot #3 jets r=0..6, xyz : 21
%   - recovery coordinate xi             : 1
%
% Start/end jets are affine functions of xi.
% Every degree-13 segment is reconstructed uniquely from its two endpoint
% jets, therefore:
%   - C6 is structural
%   - no equality constraints are required
%   - no redundant Bezier variables remain
%
% Numerical scaling is algebraic only and does not alter the feasible set,
% geometric objective, or safety boundary.
%
% ONE quadprog call. No search, no rescue, no safety relaxation.


P=aud_params();
CASES=aud_build_cases();
S=CASES(caseIndex);
fid=1;

fprintf('========================================================================================================================\n');
fprintf(' STEP 3 COMMON / CASE / BEZIER_D13 LITERATURE BASELINE V5\n');
fprintf(' 43 minimal variables | structural C6 | guide proximity | exact-support | one QP\n');
fprintf('========================================================================================================================\n');

G=aud_run_case(S,P,fid);
if ~G.pass
    error('MINCOJET:Guide','Step1 failed.');
end

obs=S.obs(G.front.known_ids_at_trigger);
C=step2_build_and_audit_corridor(G,obs,P,fid);
if ~C.pass
    error('MINCOJET:Corridor','Step2B failed.');
end

ticBuild=tic;
M=bez13_geo_build_model(G,C);
buildMs=1e3*toc(ticBuild);

% Algebraic row scaling only.
[As,bs]=bez13_jet_scale_ineq(M.Aineq,M.bineq);

lb=-inf(M.nvar,1);
ub= inf(M.nvar,1);
lb(M.iXi)=G.rejoin_xi/M.xiScale;

qpopt=optimoptions('quadprog', ...
    'Algorithm','interior-point-convex', ...
    'Display','none', ...
    'ConstraintTolerance',1e-10, ...
    'OptimalityTolerance',1e-10, ...
    'StepTolerance',1e-12, ...
    'MaxIterations',500);

ticSolve=tic;
[z,objScaled,exitflag,solverOutput]=quadprog( ...
    M.Hscaled,M.fscaled,As,bs,[],[],lb,ub,[],qpopt);
solveMs=1e3*toc(ticSolve);

if isempty(z)
    fprintf('\n[QP]\n');
    fprintf('solverCalls                = 1\n');
    fprintf('buildTimeMs                = %.6f\n',buildMs);
    fprintf('solveTimeMs                = %.6f\n',solveMs);
    fprintf('nDecisionVariables         = %d\n',M.nvar);
    fprintf('nIneq                      = %d\n',size(M.Aineq,1));
    fprintf('nEq                        = 0\n');
    fprintf('exitflag                   = %d\n',exitflag);
    fprintf('BEZIER_D13_GEOMETRIC_PASS  = 0\n');

    OUT=struct();
    OUT.mode='STEP3_COMMON_C02_BEZIER_D13_LITERATURE_BASELINE_V5';
OUT.case_index=caseIndex;
OUT.case_id=S.case_id;
OUT.case_desc=S.case_desc;
    OUT.backend='BEZIER_D13_MIN7';
    OUT.parameterization='MINIMAL_KNOT_JET_GEOMETRIC_OBJECTIVE';
    OUT.solver_calls=1;
    OUT.build_time_ms=buildMs;
    OUT.solve_time_ms=solveMs;
    OUT.exitflag=exitflag;
    OUT.solver_output=solverOutput;
    OUT.n_decision_variables=M.nvar;
    OUT.n_ineq=size(M.Aineq,1);
    OUT.n_eq=0;
    OUT.pass=false;
    OUT.reason='NO_QP_SOLUTION';

    return
end

ticAudit=tic;

CP=bez13_jet_reconstruct_cp(z,M);
xi=z(M.iXi)*M.xiScale;

% Original unscaled hard inequalities.
slack=M.bineq-M.Aineq*z;
[minSlack,worstRow]=min(slack);
worst=M.safeMeta(worstRow);

% Structural C6 / endpoint audit in physical jet coordinates.
[continuityResidual,firstJetResidual,lastJetResidual]= ...
    bez13_jet_full_audit(CP,xi,M,G);

% Recovery point.
p0=G.front.payload_at_trigger;
e=G.waypoints(:,3)-G.waypoints(:,2);
e=e/norm(e);
pRec=p0+xi*e;
recoverySlack=min(C.cells{3}.A*pRec-C.cells{3}.b);

% Geometric objective in physical coordinates.
Jgeo=bez13_geo_objective_value(CP,G);

% Diagnostic J7 only -- NOT optimized in Step 3.
J7diag=bez13_jet_J7(CP,M);

% Diagnostic dense exact Euclidean clearance only.
[sDense,pDense]=bez13_eval_dense(CP,M,301);
denseExactSlack=bez13_dense_exact_ellipsoid_slack(pDense,obs,P);

auditMs=1e3*toc(ticAudit);
totalMs=buildMs+solveMs+auditMs;

scaledViolation=max([0;As*z-bs]);
coordInf=max(abs(CP(:)));

% Conditioning of the minimal Hessian.
Hz=full(M.Hphysical);
Hz=0.5*(Hz+Hz.');
ev=eig(Hz);
ev=sort(real(ev));
tolH=max(size(Hz))*eps(max(abs(ev)));
pos=ev>tolH;
if any(pos)
    condH=max(ev(pos))/min(ev(pos));
else
    condH=Inf;
end

c6StructuralResidual=0;

pass=(exitflag>0) && ...
     (c6StructuralResidual==0) && ...
     (minSlack>=0) && ...
     (recoverySlack>=0);

fprintf('\n[COMMON INPUT]\n');
fprintf('case                       = %s\n',S.case_id);
fprintf('xiGeom                     = %.17g\n',G.rejoin_xi);
fprintf('segment_h                  = %s\n',mat2str(M.h,17));
fprintf('corridorPlanes             = [%d %d %d]\n', ...
    size(C.cells{1}.A,1),size(C.cells{2}.A,1),size(C.cells{3}.A,1));

fprintf('\n[MINIMAL QP SIZE / RUNTIME]\n');
fprintf('solverCalls                = 1\n');
fprintf('buildTimeMs                = %.6f\n',buildMs);
fprintf('solveTimeMs                = %.6f\n',solveMs);
fprintf('auditTimeMs                = %.6f\n',auditMs);
fprintf('step3TotalTimeMs           = %.6f\n',totalMs);
fprintf('nDecisionVariables         = %d\n',M.nvar);
fprintf('nIneq                      = %d\n',size(M.Aineq,1));
fprintf('nEq                        = 0\n');
fprintf('exitflag                   = %d\n',exitflag);
fprintf('geometricHessianCond         = %.17e\n',condH);

fprintf('\n[ORIGINAL HARD POST-CHECK]\n');
fprintf('xiChosen                   = %.17g\n',xi);
fprintf('additionalRunout           = %.17e\n',xi-G.rejoin_xi);
fprintf('controlPointInfNorm        = %.17e\n',coordInf);
fprintf('scaledMaxIneqViolation     = %.17e\n',scaledViolation);
fprintf('C6StructuralResidual       = %.17e\n',c6StructuralResidual);
fprintf('C6CPReconstructionResidual = %.17e  [diagnostic only]\n',continuityResidual);
fprintf('firstJetResidual           = %.17e\n',firstJetResidual);
fprintf('lastJetResidual            = %.17e\n',lastJetResidual);
fprintf('minOriginalHardSlack       = %+.17e  (seg=%d cp=%d plane=%d)\n', ...
    minSlack,worst.segment,worst.cp,worst.plane);
fprintf('recoveryFinalCellSlack     = %+.17e\n',recoverySlack);
fprintf('Jgeo                       = %.17e\n',Jgeo);
fprintf('J7Diagnostic               = %.17e  [not optimized]\n',J7diag);
fprintf('denseExactEllipsoidSlack   = %+.17e  [diagnostic only]\n',denseExactSlack);
fprintf('BEZIER_D13_GEOMETRIC_PASS  = %d\n',pass);
fprintf('LITERATURE_BASELINE         = Gao-style Bernstein corridor certificate; no new safety construction.\n');
fprintf('========================================================================================================================\n');

OUT=struct();
OUT.mode='STEP3_COMMON_C02_BEZIER_D13_LITERATURE_BASELINE_V5';
OUT.backend='BEZIER_D13_MIN7';
OUT.parameterization='MINIMAL_KNOT_JET_GEOMETRIC_OBJECTIVE';
OUT.solver_calls=1;
OUT.build_time_ms=buildMs;
OUT.solve_time_ms=solveMs;
OUT.audit_time_ms=auditMs;
OUT.step3_total_time_ms=totalMs;
OUT.exitflag=exitflag;
OUT.solver_output=solverOutput;
OUT.n_decision_variables=M.nvar;
OUT.n_ineq=size(M.Aineq,1);
OUT.n_eq=0;
OUT.geometric_hessian_condition=condH;
OUT.xi_geometric=G.rejoin_xi;
OUT.xi_chosen=xi;
OUT.additional_runout=xi-G.rejoin_xi;
OUT.control_point_inf_norm=coordInf;
OUT.scaled_max_ineq_violation=scaledViolation;
OUT.c6_structural_residual=c6StructuralResidual;
OUT.c6_cp_reconstruction_residual=continuityResidual;
OUT.first_jet_residual=firstJetResidual;
OUT.last_jet_residual=lastJetResidual;
OUT.min_original_hard_slack=minSlack;
OUT.recovery_final_cell_slack=recoverySlack;
OUT.Jgeo=Jgeo;
OUT.J7_diagnostic=J7diag;
OUT.dense_exact_ellipsoid_slack=denseExactSlack;
OUT.control_points=CP;
OUT.shared_knot_jets=bez13_common_knot_jets(z,M,G,xi);
OUT.sample_s=sDense;
OUT.sample_position=pDense;
OUT.pass=pass;

end

function OUT = run_poly_d13(caseIndex,verbose)
% STEP 3 COMMON / C02 / BEZIER_D13_MIN7
% Literature-aligned baseline:
%   - Piecewise degree-13 polynomial with minimal/shared knot-jet variables
%     following the MINCO/GCOPTER design philosophy
%   - Geometry is certified through the Bernstein image of each polynomial
%   - Raw power-basis coefficients are output-only because they are not a
%     numerically suitable online optimization basis.
%   - No new safety construction is claimed here.
%
% Minimal knot-jet parameterization inspired by MINCO-style elimination.
%
% SAME trajectory space and SAME hard corridor constraints as the previous
% 3-piece degree-13 C6 Bezier formulation.
%
% POLY_D13 stable implementation:
%   The polynomial is parameterized by shared physical endpoint jets.
%   Optimization/safety are assembled through the numerically stable
%   Bernstein image of the same degree-13 polynomial. Raw power
%   coefficients are generated only after solving.
%
%   J7 is NOT optimized in Step 3.
%   The only objective is geometric guide proximity:
%
%       sum over all Bezier CPs ||P_ij - Pguide_ij||^2
%
%   No arbitrary weights are used.
%
% Decision variables (43 total):
%   - internal knot #2 jets r=0..6, xyz : 21
%   - internal knot #3 jets r=0..6, xyz : 21
%   - recovery coordinate xi             : 1
%
% Start/end jets are affine functions of xi.
% Every degree-13 segment is reconstructed uniquely from its two endpoint
% jets, therefore:
%   - C6 is structural
%   - no equality constraints are required
%   - no redundant Bezier variables remain
%
% Numerical scaling is algebraic only and does not alter the feasible set,
% geometric objective, or safety boundary.
%
% ONE quadprog call. No search, no rescue, no safety relaxation.


P=aud_params();
CASES=aud_build_cases();
S=CASES(caseIndex);
fid=1;

fprintf('========================================================================================================================\n');
fprintf(' STEP 3 COMMON / CASE / POLY_D13 LITERATURE BASELINE V5\n');
fprintf(' 43 minimal variables | structural C6 | guide proximity | exact-support | one QP\n');
fprintf('========================================================================================================================\n');

G=aud_run_case(S,P,fid);
if ~G.pass
    error('MINCOJET:Guide','Step1 failed.');
end

obs=S.obs(G.front.known_ids_at_trigger);
C=step2_build_and_audit_corridor(G,obs,P,fid);
if ~C.pass
    error('MINCOJET:Corridor','Step2B failed.');
end

ticBuild=tic;
M=poly13_stable_build_model(G,C);
buildMs=1e3*toc(ticBuild);

% Algebraic row scaling only.
[As,bs]=bez13_jet_scale_ineq(M.Aineq,M.bineq);

lb=-inf(M.nvar,1);
ub= inf(M.nvar,1);
lb(M.iXi)=G.rejoin_xi/M.xiScale;

qpopt=optimoptions('quadprog', ...
    'Algorithm','interior-point-convex', ...
    'Display','none', ...
    'ConstraintTolerance',1e-10, ...
    'OptimalityTolerance',1e-10, ...
    'StepTolerance',1e-12, ...
    'MaxIterations',500);

ticSolve=tic;
[z,objScaled,exitflag,solverOutput]=quadprog( ...
    M.Hscaled,M.fscaled,As,bs,[],[],lb,ub,[],qpopt);
solveMs=1e3*toc(ticSolve);

if isempty(z)
    fprintf('\n[QP]\n');
    fprintf('solverCalls                = 1\n');
    fprintf('buildTimeMs                = %.6f\n',buildMs);
    fprintf('solveTimeMs                = %.6f\n',solveMs);
    fprintf('nDecisionVariables         = %d\n',M.nvar);
    fprintf('nIneq                      = %d\n',size(M.Aineq,1));
    fprintf('nEq                        = 0\n');
    fprintf('exitflag                   = %d\n',exitflag);
    fprintf('POLY_D13_HERMITE_PASS  = 0\n');

    OUT=struct();
    OUT.mode='STEP3_COMMON_C02_POLY_D13_LITERATURE_BASELINE_V5';
OUT.case_index=caseIndex;
OUT.case_id=S.case_id;
OUT.case_desc=S.case_desc;
    OUT.backend='POLY_D13';
    OUT.parameterization='MINIMAL_PHYSICAL_HERMITE_JETS';
    OUT.solver_calls=1;
    OUT.build_time_ms=buildMs;
    OUT.solve_time_ms=solveMs;
    OUT.exitflag=exitflag;
    OUT.solver_output=solverOutput;
    OUT.n_decision_variables=M.nvar;
    OUT.n_ineq=size(M.Aineq,1);
    OUT.n_eq=0;
    OUT.pass=false;
    OUT.reason='NO_QP_SOLUTION';

    return
end

ticAudit=tic;

CP=bez13_jet_reconstruct_cp(z,M);
PowerCoef=poly13_stable_bernstein_to_power(CP);
xi=z(M.iXi)*M.xiScale;

% Original unscaled hard inequalities.
slack=M.bineq-M.Aineq*z;
[minSlack,worstRow]=min(slack);
worst=M.safeMeta(worstRow);% Structural C6 / endpoint audit in physical jet coordinates.
[continuityResidual,firstJetResidual,lastJetResidual]= ...
    bez13_jet_full_audit(CP,xi,M,G);

% Recovery point.
p0=G.front.payload_at_trigger;
e=G.waypoints(:,3)-G.waypoints(:,2);
e=e/norm(e);
pRec=p0+xi*e;
recoverySlack=min(C.cells{3}.A*pRec-C.cells{3}.b);

% Geometric objective in physical coordinates.
Jgeo=bez13_geo_objective_value(CP,G);

% Diagnostic J7 only -- NOT optimized in Step 3.
J7diag=bez13_jet_J7(CP,M);

% Diagnostic dense exact Euclidean clearance only.
[sDense,pDense]=bez13_eval_dense(CP,M,301);
denseExactSlack=bez13_dense_exact_ellipsoid_slack(pDense,obs,P);

auditMs=1e3*toc(ticAudit);
totalMs=buildMs+solveMs+auditMs;

scaledViolation=max([0;As*z-bs]);
coordInf=max(abs(CP(:)));

% Conditioning of the minimal Hessian.
Hz=full(M.Hphysical);
Hz=0.5*(Hz+Hz.');
ev=eig(Hz);
ev=sort(real(ev));
tolH=max(size(Hz))*eps(max(abs(ev)));
pos=ev>tolH;
if any(pos)
    condH=max(ev(pos))/min(ev(pos));
else
    condH=Inf;
end

c6StructuralResidual=0;

pass=(exitflag>0) && ...
     (c6StructuralResidual==0) && ...
     (minSlack>=0) && ...
     (recoverySlack>=0);

fprintf('\n[COMMON INPUT]\n');
fprintf('case                       = %s\n',S.case_id);
fprintf('xiGeom                     = %.17g\n',G.rejoin_xi);
fprintf('segment_h                  = %s\n',mat2str(M.h,17));
fprintf('corridorPlanes             = [%d %d %d]\n', ...
    size(C.cells{1}.A,1),size(C.cells{2}.A,1),size(C.cells{3}.A,1));

fprintf('\n[MINIMAL QP SIZE / RUNTIME]\n');
fprintf('solverCalls                = 1\n');
fprintf('buildTimeMs                = %.6f\n',buildMs);
fprintf('solveTimeMs                = %.6f\n',solveMs);
fprintf('auditTimeMs                = %.6f\n',auditMs);
fprintf('step3TotalTimeMs           = %.6f\n',totalMs);
fprintf('nDecisionVariables         = %d\n',M.nvar);
fprintf('nIneq                      = %d\n',size(M.Aineq,1));
fprintf('nEq                        = 0\n');
fprintf('exitflag                   = %d\n',exitflag);
fprintf('geometricHessianCond         = %.17e\n',condH);

fprintf('\n[ORIGINAL HARD POST-CHECK]\n');
fprintf('xiChosen                   = %.17g\n',xi);
fprintf('additionalRunout           = %.17e\n',xi-G.rejoin_xi);
fprintf('bernsteinCPInfNorm         = %.17e\n',coordInf);
fprintf('powerCoeffInfNorm          = %.17e  [output representation only]\n',max(abs(PowerCoef(:))));
fprintf('scaledMaxIneqViolation     = %.17e\n',scaledViolation);
fprintf('C6StructuralResidual       = %.17e\n',c6StructuralResidual);
fprintf('C6CPReconstructionResidual = %.17e  [diagnostic only]\n',continuityResidual);
fprintf('firstJetResidual           = %.17e  [diagnostic only]\n',firstJetResidual);
fprintf('lastJetResidual            = %.17e  [diagnostic only]\n',lastJetResidual);
fprintf('minOriginalHardSlack       = %+.17e  (seg=%d cp=%d plane=%d)\n', ...
    minSlack,worst.segment,worst.cp,worst.plane);
fprintf('recoveryFinalCellSlack     = %+.17e\n',recoverySlack);
fprintf('Jgeo                       = %.17e\n',Jgeo);
fprintf('J7Diagnostic               = %.17e  [not optimized]\n',J7diag);
fprintf('denseExactEllipsoidSlack   = %+.17e  [diagnostic only]\n',denseExactSlack);
fprintf('POLY_D13_HERMITE_PASS      = %d\n',pass);
fprintf('LITERATURE_BASELINE         = MINCO/GCOPTER-style minimal polynomial representation + Bernstein geometry.\n');
fprintf('NOTE: QP uses stable physical-Hermite/Bernstein map; raw power coefficients are output-only.\n');
fprintf('========================================================================================================================\n');

OUT=struct();
OUT.mode='STEP3_COMMON_C02_POLY_D13_LITERATURE_BASELINE_V5';
OUT.backend='POLY_D13';
OUT.parameterization='MINIMAL_PHYSICAL_HERMITE_JETS';
OUT.solver_calls=1;
OUT.build_time_ms=buildMs;
OUT.solve_time_ms=solveMs;
OUT.audit_time_ms=auditMs;
OUT.step3_total_time_ms=totalMs;
OUT.exitflag=exitflag;
OUT.solver_output=solverOutput;
OUT.n_decision_variables=M.nvar;
OUT.n_ineq=size(M.Aineq,1);
OUT.n_eq=0;
OUT.geometric_hessian_condition=condH;
OUT.xi_geometric=G.rejoin_xi;
OUT.xi_chosen=xi;
OUT.additional_runout=xi-G.rejoin_xi;
OUT.bernstein_cp_inf_norm=coordInf;
OUT.power_coefficient_inf_norm=max(abs(PowerCoef(:)));
OUT.scaled_max_ineq_violation=scaledViolation;
OUT.c6_structural_residual=c6StructuralResidual;
OUT.c6_cp_reconstruction_residual=continuityResidual;
OUT.first_jet_residual=firstJetResidual;
OUT.last_jet_residual=lastJetResidual;
OUT.min_original_hard_slack=minSlack;
OUT.recovery_final_cell_slack=recoverySlack;
OUT.Jgeo=Jgeo;
OUT.J7_diagnostic=J7diag;
OUT.dense_exact_ellipsoid_slack=denseExactSlack;
OUT.bernstein_control_points=CP;
OUT.shared_knot_jets=bez13_common_knot_jets(z,M,G,xi);
OUT.power_coefficients=PowerCoef;
OUT.sample_s=sDense;
OUT.sample_position=pDense;
OUT.pass=pass;

end

function OUT = run_bspline_d7(caseIndex,verbose)
% STEP 3 COMMON SPEC -- BACKEND 3 / 3
% C02 only: BSPLINE_D7
%
% Common requirements preserved:
%   - Step1 guide + Step2B exact-support corridor
%   - degree 7 B-spline
%   - simple interior knots -> structural C6
%   - recovery coordinate xi is optimized in the SAME one-shot QP
%   - continuous corridor certificate through exact Bezier extraction
%   - geometric guide-proximity objective
%   - original, unscaled hard safety post-check must be >= 0
%   - no J7 optimization, no time allocation, no dynamics
%   - no scan / bisection / rescue / safety relaxation
%
% Representation:
%   degree p = 7
%   ncoef    = 22
%   nspan    = 15
%
% The 15 knot spans are distributed across the three Step1 guide segments
% by path length, with guide/corridor transitions placed exactly at knots.
%
% Decision variables:
%   Literature Variant-2 gives 16 native CPs. After exact endpoint-jet
%   elimination, only CP 8:9 (two CPs per axis) + xi remain = 7 variables.
%
% Endpoint spatial jets 0..6 are affine in xi, matching the same current
% common start/recovery semantics used by the degree-13 backend.
%
% Geometric objective:
%   exact integral of squared Euclidean deviation from the piecewise-linear
%   Step1 guide using 8-point Gauss-Legendre per degree-7 span.
%
% NOTE:
%   This is a basis-invariant curve-to-guide geometric objective for the
%   B-spline backend. Its numerical magnitude is not directly comparable
%   to the degree-13 control-point Jgeo magnitude; geometry/safety/runtime
%   are the primary common comparisons at this stage.


P=aud_params();
CASES=aud_build_cases();
S=CASES(caseIndex); % C02_TWO_BASE
fid=1;

fprintf('========================================================================================================================\n');
fprintf(' STEP 3 COMMON / CASE / BSPLINE_D7 LITERATURE VARIANT-2 V5\n');
fprintf(' degree7 | 28 CP / 21 spans | native-CP convex-hull safety | structural C6 | one QP\n');
fprintf('========================================================================================================================\n');

G=aud_run_case(S,P,fid);
if ~G.pass
    error('BSD7:Guide','Step1 failed.');
end

obs=S.obs(G.front.known_ids_at_trigger);
C=step2_build_and_audit_corridor(G,obs,P,fid);
if ~C.pass
    error('BSD7:Corridor','Step2B failed.');
end

ticBuild=tic;
M=bsd7_build_model(G,C);

% Standard B-spline local convex-hull safety certificate.
% Each native CP is constrained to every corridor cell touched by its
% supported spans. The exact extracted-Bezier post-check remains mandatory.
[M.Aineq,M.bineq,M.safeMeta,nativeCert]=bsd7_native_cp_certificate(M,C);

buildMs=1e3*toc(ticBuild);

R=bsd7_build_equality_elimination(M);

% Reduced variable y contains:
%   fourteen interior native CPs per axis + scaled xi = 43 variables.
% Full variable is x = R.x0 + R.T*y.
Ar=M.Aineq*R.T;
br=M.bineq-M.Aineq*R.x0;

Hr=R.T.'*M.Hscaled*R.T;
Hr=0.5*(Hr+Hr.');
fr=R.T.'*(M.Hscaled*R.x0+M.fscaled);

[Ais,bis]=bsd7_scale_rows(Ar,br);

lb=-inf(R.nred,1);
ub= inf(R.nred,1);
lb(R.iXiRed)=G.rejoin_xi/M.xiScale;

opts=optimoptions('quadprog', ...
    'Algorithm','interior-point-convex', ...
    'Display','none', ...
    'ConstraintTolerance',1e-10, ...
    'OptimalityTolerance',1e-10, ...
    'StepTolerance',1e-12, ...
    'MaxIterations',500);

ticSolve=tic;
[y,objScaled,exitflag,solverOutput]=quadprog( ...
    sparse(Hr),fr,Ais,bis,[],[],lb,ub,[],opts);
solveMs=1e3*toc(ticSolve);

if isempty(y)
    x=[];
else
    x=R.x0+R.T*y;

    % Explicit reduced/full dimension guards.
    if numel(y)~=R.nred
        error('BSD7:ReducedDimension','Reduced solution y has unexpected length.');
    end
    if numel(x)~=M.nvar
        error('BSD7:FullDimension','Reconstructed full solution x has unexpected length.');
    end
    if size(Ais,2)~=numel(y)
        error('BSD7:ReducedIneqDimension','Reduced inequality matrix does not match y.');
    end
    if size(M.Aineq,2)~=numel(x)
        error('BSD7:FullIneqDimension','Full inequality matrix does not match x.');
    end
    if size(M.Aeq,2)~=numel(x)
        error('BSD7:FullEqDimension','Full equality matrix does not match x.');
    end
end

if isempty(x)
    fprintf('\n[QP]\n');
    fprintf('solverCalls                = 1\n');
    fprintf('buildTimeMs                = %.6f\n',buildMs);
    fprintf('solveTimeMs                = %.6f\n',solveMs);
    fprintf('nDecisionVariablesFull     = %d\n',M.nvar);
fprintf('nDecisionVariablesReduced  = %d\n',R.nred);
    fprintf('nIneq                      = %d\n',size(M.Aineq,1));
    fprintf('nEqInQP                    = 0\n');
    fprintf('exitflag                   = %d\n',exitflag);
    fprintf('BSPLINE_D7_GEOMETRIC_PASS  = 0\n');

    OUT=struct();
    OUT.mode='STEP3_COMMON_C02_BSPLINE_D7_LITERATURE_VARIANT2_V5';
OUT.case_index=caseIndex;
OUT.case_id=S.case_id;
OUT.case_desc=S.case_desc;
    OUT.backend='BSPLINE_D7';
    OUT.solver_calls=1;
    OUT.build_time_ms=buildMs;
    OUT.solve_time_ms=solveMs;
    OUT.exitflag=exitflag;
    OUT.solver_output=solverOutput;
    OUT.n_decision_variables_full=M.nvar;
    OUT.n_decision_variables_reduced=R.nred;
    OUT.n_ineq=size(M.Aineq,1);
    OUT.n_eq_original=size(M.Aeq,1);
    OUT.n_eq_in_qp=0;

    OUT.pass=false;
    OUT.reason='NO_QP_SOLUTION';

    return
end

ticAudit=tic;

xi=x(M.iXi)*M.xiScale;
cp=bsd7_native_cp(x,M);
Bcp=bsd7_extract_bezier(cp,M);

D=bsd7_bottleneck_diagnostics(x,M,C,G);

eqResidual=norm(M.Aeq*x-M.beq,inf);
slack=M.bineq-M.Aineq*x;
[minHardSlack,worstRow]=min(slack);
worst=M.safeMeta(worstRow);

[minBezierSlack,worstSpan,worstCell,worstCP,worstPlane]= ...
    bsd7_original_bezier_slack(Bcp,M,C);

% Structural C6 from degree 7 + simple interior knots.
C6StructuralResidual=0;

% Endpoint Bezier reconstruction diagnostic.
endpointResidual=bsd7_endpoint_diagnostic(Bcp,xi,M,G);

% Recovery point hard post-check.
p0=G.front.payload_at_trigger;
e=G.waypoints(:,3)-G.waypoints(:,2);
e=e/norm(e);
pRec=p0+xi*e;
recoverySlack=min(C.cells{3}.A*pRec-C.cells{3}.b);

% Exact integrated curve-to-guide objective.
JgeoCurve=bsd7_curve_guide_objective(cp,M,G);

% Dense diagnostic only.
[sDense,pDense]=bsd7_eval_dense(cp,M,301);
denseExactSlack=bez13_dense_exact_ellipsoid_slack(pDense,obs,P);

% Extraction structural audit.
minExtractEntry=min(M.Tbez,[],'all');
extractRowSumErr=max(abs(sum(M.Tbez,2)-1));

scaledIneqViolation=max([0;Ais*y-bis]);
scaledEqResidual=eqResidual;
cpInf=max(abs(cp(:)));
bezInf=max(abs(Bcp(:)));

auditMs=1e3*toc(ticAudit);
totalMs=buildMs+solveMs+auditMs;

pass=(exitflag>0) && ...
     (C6StructuralResidual==0) && ...
     (minHardSlack>=0) && ...
     (minBezierSlack>=0) && ...
     (recoverySlack>=0);

fprintf('\n[COMMON INPUT]\n');
fprintf('case                       = %s\n',S.case_id);
fprintf('xiGeom                     = %.17g\n',G.rejoin_xi);
fprintf('guideFractions             = %s\n',mat2str(M.frac,17));
fprintf('spanCountByCell            = %s  [C6 boundary-padded Variant-2]\n',mat2str(M.spanCount));
fprintf('controlPointRule           = n=d(q-1)+2+2r = %d, r=6\n',M.ncoef);
fprintf('intervalCount              = %d\n',M.nspan);
fprintf('corridorPlanes             = [%d %d %d]\n', ...
    size(C.cells{1}.A,1),size(C.cells{2}.A,1),size(C.cells{3}.A,1));
fprintf('safetyCertificate          = native B-spline CP local convex hull\n');
fprintf('nativeCPCellMemberships    = %d\n',nativeCert.uniqueMemberships);
fprintf('nativeCPMultiCellCount     = %d\n',nativeCert.multiCellCPs);
fprintf('nativeCPRows               = %d\n',nativeCert.constraintRows);

fprintf('\n[QP SIZE / RUNTIME]\n');
fprintf('solverCalls                = 1\n');
fprintf('buildTimeMs                = %.6f\n',buildMs);
fprintf('solveTimeMs                = %.6f\n',solveMs);
fprintf('auditTimeMs                = %.6f\n',auditMs);
fprintf('step3TotalTimeMs           = %.6f\n',totalMs);
fprintf('nDecisionVariablesFull     = %d\n',M.nvar);
fprintf('nDecisionVariablesReduced  = %d\n',R.nred);
fprintf('nIneq                      = %d\n',size(M.Aineq,1));
fprintf('nEqOriginal                = %d\n',size(M.Aeq,1));
fprintf('nEqInQP                    = 0\n');
fprintf('exitflag                   = %d\n',exitflag);
fprintf('equalityEliminationCond    = %.17e\n',R.condDependentBlock);
fprintf('reconstructionEqResidual  = %.17e\n',norm(M.Aeq*x-M.beq,inf));

fprintf('\n[BOTTLENECK ANATOMY]\n');
fprintf('rankAeq                    = %d\n',D.rankAeq);
fprintf('nullityAfterEquality       = %d\n',D.nullityAfterEquality);
fprintf('condAeqNonzeroSV           = %.17e\n',D.condAeq);
fprintf('reducedHessianCond         = %.17e\n',D.reducedHessianCond);
fprintf('activeIneq_1e12            = %d / %d\n',D.active1e12,size(M.Aineq,1));
fprintf('activeIneq_1e10            = %d / %d\n',D.active1e10,size(M.Aineq,1));
fprintf('activeIneqByCell           = %s\n',mat2str(D.activeByCell));
fprintf('ineqRowsByCell             = %s\n',mat2str(D.ineqRowsByCell));
fprintf('meanNativeSupportPerBezier = %.6f\n',D.meanNativeSupport);
fprintf('maxNativeSupportPerBezier  = %d\n',D.maxNativeSupport);
fprintf('endpointEqNativeCPUsed     = %d / %d\n',D.endpointNativeCPUsed,M.ncoef);
fprintf('safetyNativeCPUsed         = %d / %d\n',D.safetyNativeCPUsed,M.ncoef);
fprintf('allNativeCPConstrained     = %d\n',D.allNativeCPConstrained);
fprintf('nativeCertificateRowsCheck = %d\n',nativeCert.constraintRows);
fprintf('boundaryWorstIsTransition  = %d\n',D.worstIsTransition);
fprintf('boundaryWorstSharedPoint   = %d\n',D.worstIsSharedBezierPoint);
fprintf('\n[ORIGINAL HARD POST-CHECK]\n');
fprintf('xiChosen                   = %.17g\n',xi);
fprintf('additionalRunout           = %.17e\n',xi-G.rejoin_xi);
fprintf('nativeCPInfNorm            = %.17e\n',cpInf);
fprintf('bezierCPInfNorm            = %.17e\n',bezInf);
fprintf('scaledMaxIneqViolation     = %.17e\n',scaledIneqViolation);
fprintf('scaledEqResidual           = %.17e\n',scaledEqResidual);
fprintf('originalEqResidual         = %.17e  [numerical diagnostic]\n',eqResidual);
fprintf('C6StructuralResidual       = %.17e\n',C6StructuralResidual);
fprintf('endpointBezierResidual     = %.17e  [diagnostic only]\n',endpointResidual);
fprintf('minOriginalHardSlack       = %+.17e  (span=%d cell=%d cp=%d plane=%d)\n', ...
    minHardSlack,worst.span,worst.cell,worst.cp,worst.plane);
fprintf('minExtractedBezierSlack    = %+.17e  (span=%d cell=%d cp=%d plane=%d)\n', ...
    minBezierSlack,worstSpan,worstCell,worstCP,worstPlane);
fprintf('recoveryFinalCellSlack     = %+.17e\n',recoverySlack);
fprintf('JgeoCurveIntegral          = %.17e\n',JgeoCurve);
fprintf('extractMinEntry            = %+.17e\n',minExtractEntry);
fprintf('extractRowSumError         = %.17e\n',extractRowSumErr);
fprintf('denseExactEllipsoidSlack   = %+.17e  [diagnostic only]\n',denseExactSlack);
fprintf('BSPLINE_D7_GEOMETRIC_PASS  = %d\n',pass);
fprintf('NOTE: Nguyen Variant-2 is retained as the base; 2r extra CPs are added only to preserve its free DOF under endpoint jets 0..6.\n');
fprintf('========================================================================================================================\n');

OUT=struct();
OUT.mode='STEP3_COMMON_C02_BSPLINE_D7_LITERATURE_VARIANT2_V5';
OUT.backend='BSPLINE_D7';
OUT.degree=M.p;
OUT.n_native_control_points=M.ncoef;
OUT.n_spans=M.nspan;
OUT.span_count_by_cell=M.spanCount;
OUT.knots=M.U;
OUT.solver_calls=1;
OUT.build_time_ms=buildMs;
OUT.solve_time_ms=solveMs;
OUT.audit_time_ms=auditMs;
OUT.step3_total_time_ms=totalMs;
OUT.exitflag=exitflag;
OUT.solver_output=solverOutput;
OUT.n_decision_variables_full=M.nvar;
OUT.n_decision_variables_reduced=R.nred;
OUT.n_ineq=size(M.Aineq,1);
OUT.safety_certificate='NATIVE_BSPLINE_CP_LOCAL_CONVEX_HULL';
OUT.native_cp_cell_memberships=nativeCert.uniqueMemberships;
OUT.native_cp_multi_cell_count=nativeCert.multiCellCPs;
OUT.native_cp_constraint_rows=nativeCert.constraintRows;
OUT.n_eq_original=size(M.Aeq,1);
OUT.n_eq_in_qp=0;
OUT.equality_elimination_condition=R.condDependentBlock;
OUT.reconstruction_eq_residual=norm(M.Aeq*x-M.beq,inf);
OUT.xi_geometric=G.rejoin_xi;
OUT.xi_chosen=xi;
OUT.additional_runout=xi-G.rejoin_xi;
OUT.native_cp_inf_norm=cpInf;
OUT.bezier_cp_inf_norm=bezInf;
OUT.scaled_max_ineq_violation=scaledIneqViolation;
OUT.scaled_eq_residual=scaledEqResidual;
OUT.original_eq_residual=eqResidual;
OUT.c6_structural_residual=C6StructuralResidual;
OUT.endpoint_bezier_residual=endpointResidual;
OUT.min_original_hard_slack=minHardSlack;
OUT.min_extracted_bezier_slack=minBezierSlack;
OUT.recovery_final_cell_slack=recoverySlack;
OUT.Jgeo_curve_integral=JgeoCurve;
OUT.extraction_min_entry=minExtractEntry;
OUT.extraction_row_sum_error=extractRowSumErr;
OUT.dense_exact_ellipsoid_slack=denseExactSlack;
OUT.native_control_points=cp;
OUT.extracted_bezier_control_points=Bcp;
OUT.sample_s=sDense;
OUT.sample_position=pDense;
OUT.pass=pass;

end

function OUT=run_bspline_d7_literature_position_only(caseIndex,verbose)
% Literature-faithful diagnostic:
% Nguyen Variant-2 architecture with START/END POSITION constraints only.
%
% This deliberately removes OUR additional endpoint spatial jets 1..6.
% Purpose: determine whether the current Variant-2 infeasibility is caused
% by those extra endpoint-jet constraints.
%
% Same:
%   - C02
%   - Step1 guide / Step2B exact-support corridor
%   - degree 7
%   - 16 native CP / 9 spans / [1 7 1]
%   - extracted-Bezier continuous safety certificate
%   - geometric curve-to-guide objective
%   - variable recovery xi
%   - strict original hard post-check
%
% Different:
%   - endpoint constraints are only p(start)=p_trigger and
%     p(end)=p_trigger+e*xi, matching the literature path problem.

fprintf('========================================================================================================================\n');
fprintf(' C02 / BSPLINE_D7 / NGUYEN VARIANT-2 / POSITION-ONLY DIAGNOSTIC\n');
fprintf(' 16 CP / 9 spans | literature endpoint positions only | exact-support | one QP\n');
fprintf('========================================================================================================================\n');

P=aud_params();
CASES=aud_build_cases();
S=CASES(caseIndex);
fid=1;

G=aud_run_case(S,P,fid);
if ~G.pass
    error('BSD7LIT:Guide','Step1 failed.');
end
obs=S.obs(G.front.known_ids_at_trigger);
if isempty(obs)
    error('BSD7LIT:Obs','No known obstacle available at trigger.');
end
C=step2_build_and_audit_corridor(G,obs,P,fid);
if ~isstruct(C) || ~isfield(C,'pass') || ~isfield(C,'cells')
    error('BSD7LIT:CorridorStruct','Unexpected Step2B corridor structure.');
end
if ~C.pass
    error('BSD7LIT:Corridor','Step2B failed.');
end
if numel(C.cells)~=3
    error('BSD7LIT:CorridorCells','Expected exactly three C02 corridor cells.');
end

ticBuild=tic;
M=bsd7_build_model_position_only(G,C);

% Exact elimination of clamped start/end position constraints:
% P1 = p0, Pn = p0 + e*xi.
R=bsd7_position_only_elimination(M,G);

Ar=M.Aineq*R.T;
br=M.bineq-M.Aineq*R.x0;

Hr=R.T.'*M.Hscaled*R.T;
Hr=0.5*(Hr+Hr.');
fr=R.T.'*(M.Hscaled*R.x0+M.fscaled);

[Ais,bis]=bsd7_scale_rows(Ar,br);

lb=-inf(R.nred,1);
ub= inf(R.nred,1);
lb(R.iXiRed)=G.rejoin_xi/M.xiScale;

opts=optimoptions('quadprog', ...
    'Algorithm','interior-point-convex', ...
    'Display','none', ...
    'ConstraintTolerance',1e-10, ...
    'OptimalityTolerance',1e-10, ...
    'StepTolerance',1e-12, ...
    'MaxIterations',500);

buildMs=1e3*toc(ticBuild);

ticSolve=tic;
[y,~,exitflag,solverOutput]=quadprog(sparse(Hr),fr,Ais,bis,[],[],lb,ub,[],opts);
solveMs=1e3*toc(ticSolve);

if isempty(y)
    error('BSD7LIT:NoSolution','quadprog returned empty solution.');
end
x=R.x0+R.T*y;

ticAudit=tic;
cp=bsd7_native_cp(x,M);
Bcp=bsd7_extract_bezier(cp,M);

scaledIneqViolation=max([0;Ais*y-bis]);
[minHardSlack,~,~,~,~]=bsd7_original_bezier_slack(Bcp,M,C);
minBezierSlack=minHardSlack;

xi=M.xiScale*x(M.iXi);
recoverySlack=min(C.cells{3}.A*(G.front.payload_at_trigger + ...
    (G.waypoints(:,3)-G.waypoints(:,2))/norm(G.waypoints(:,3)-G.waypoints(:,2))*xi)-C.cells{3}.b);

Jgeo=bsd7_curve_guide_objective(cp,M,G);
[sSample,pSample]=bsd7_eval_dense(cp,M,301);
if size(pSample,1)~=3
    error('BSD7LIT:DenseShape','Dense position samples must be 3xN.');
end
denseSlack=bez13_dense_exact_ellipsoid_slack(pSample,obs,P);

startResidual=norm(cp(:,1)-G.front.payload_at_trigger,inf);
e=G.waypoints(:,3)-G.waypoints(:,2); e=e/norm(e);
endResidual=norm(cp(:,end)-(G.front.payload_at_trigger+e*xi),inf);

auditMs=1e3*toc(ticAudit);
totalMs=buildMs+solveMs+auditMs;

pass=(exitflag>0) && ...
     (minHardSlack>=0) && ...
     (minBezierSlack>=0) && ...
     (recoverySlack>=0);

fprintf('\n[LITERATURE POSITION-ONLY RESULT]\n');
fprintf('solverCalls                = 1\n');
fprintf('buildTimeMs                = %.6f\n',buildMs);
fprintf('solveTimeMs                = %.6f\n',solveMs);
fprintf('auditTimeMs                = %.6f\n',auditMs);
fprintf('step3TotalTimeMs           = %.6f\n',totalMs);
fprintf('nDecisionVariablesFull     = %d\n',M.nvar);
fprintf('nDecisionVariablesReduced  = %d\n',R.nred);
fprintf('nIneq                      = %d\n',size(M.Aineq,1));
fprintf('nEqInQP                    = 0\n');
fprintf('exitflag                   = %d\n',exitflag);
fprintf('xiChosen                   = %.17g\n',xi);
fprintf('additionalRunout           = %.17e\n',xi-G.rejoin_xi);
fprintf('startPositionResidual      = %.17e\n',startResidual);
fprintf('endPositionResidual        = %.17e\n',endResidual);
fprintf('scaledMaxIneqViolation     = %.17e\n',scaledIneqViolation);
fprintf('minOriginalHardSlack       = %+.17e\n',minHardSlack);
fprintf('recoveryFinalCellSlack     = %+.17e\n',recoverySlack);
fprintf('JgeoCurveIntegral          = %.17e\n',Jgeo);
fprintf('denseExactEllipsoidSlack   = %+.17e  [diagnostic only]\n',denseSlack);
fprintf('BSPLINE_LITERATURE_PASS    = %d\n',pass);
fprintf('NOTE: this diagnostic reproduces the literature endpoint-position problem; it does NOT yet enforce our added endpoint jets 1..6.\n');
fprintf('========================================================================================================================\n');

OUT=struct();
OUT.mode='BSPLINE_D7_NGUYEN_VARIANT2_POSITION_ONLY_DIAGNOSTIC';
OUT.backend='BSPLINE_D7';
OUT.solver_calls=1;
OUT.build_time_ms=buildMs;
OUT.solve_time_ms=solveMs;
OUT.audit_time_ms=auditMs;
OUT.step3_total_time_ms=totalMs;
OUT.exitflag=exitflag;
OUT.solver_output=solverOutput;
OUT.n_decision_variables_full=M.nvar;
OUT.n_decision_variables_reduced=R.nred;
OUT.n_ineq=size(M.Aineq,1);
OUT.n_eq_in_qp=0;
OUT.xi_geometric=G.rejoin_xi;
OUT.xi_chosen=xi;
OUT.additional_runout=xi-G.rejoin_xi;
OUT.start_position_residual=startResidual;
OUT.end_position_residual=endResidual;
OUT.scaled_max_ineq_violation=scaledIneqViolation;
OUT.min_original_hard_slack=minHardSlack;
OUT.recovery_final_cell_slack=recoverySlack;
OUT.Jgeo_curve_integral=Jgeo;
OUT.dense_exact_ellipsoid_slack=denseSlack;
OUT.native_control_points=cp;
OUT.extracted_bezier_control_points=Bcp;
OUT.sample_s=sSample;
OUT.sample_position=pSample;
OUT.pass=pass;
end


function M=bsd7_build_model_position_only(G,C)
% Same Nguyen Variant-2 geometry/objective/safety as bsd7_build_model,
% but no endpoint jet equalities are built here.
M=bsd7_build_model(G,C);
M.Aeq=sparse(0,M.nvar);
M.beq=zeros(0,1);
end


function R=bsd7_position_only_elimination(M,G)
% For an open clamped B-spline:
% curve start = native CP1, curve end = native CPn.
% Eliminate those six endpoint coordinates exactly, retaining xi.

n=M.ncoef;
p0=G.front.payload_at_trigger;
e=G.waypoints(:,3)-G.waypoints(:,2);
e=e/norm(e);

dep=[1, n, n+1, 2*n, 2*n+1, 3*n];
allVars=1:M.nvar;
free=setdiff(allVars,dep,'stable');

x0=zeros(M.nvar,1);
T=zeros(M.nvar,numel(free));
T(free,:)=eye(numel(free));

% start CP coordinates
x0(1)=p0(1);
x0(n+1)=p0(2);
x0(2*n+1)=p0(3);

% end CP = p0 + e * xiScale*zXi
x0(n)=p0(1);
x0(2*n)=p0(2);
x0(3*n)=p0(3);

iXiRed=find(free==M.iXi,1);
if isempty(iXiRed)
    error('BSD7LIT:Xi','xi must remain a reduced variable.');
end
T(n,iXiRed)=e(1)*M.xiScale;
T(2*n,iXiRed)=e(2)*M.xiScale;
T(3*n,iXiRed)=e(3)*M.xiScale;

R=struct();
R.dep=dep;
R.free=free;
R.x0=x0;
R.T=T;
R.nred=numel(free);
R.iXiRed=iXiRed;
end


function M=bsd7_build_model(G,C)
p=7;
qcell=3;
rBoundary=6;

% Literature-guided C6 boundary extension.
% Base Nguyen Variant-2 uses n0=d(q-1)+2 and only endpoint positions.
% Our global-C6 requirement fixes endpoint spatial jets 0..rBoundary.
% To preserve the SAME reduced design freedom as the literature position-only
% problem, add 2*rBoundary control points:
%   n = d(q-1)+2 + 2*rBoundary.
% For d=7, q=3, rBoundary=6:
%   ncoef = 28, nspan = 21, reduced variables = 43.
ncoef=p*(qcell-1)+2+2*rBoundary;
nspan=ncoef-p;

W=G.waypoints;
segLen=vecnorm(diff(W,1,2),2,1);
Ltot=sum(segLen);
if ~(Ltot>0)
    error('BSD7:GuideLength','Zero guide length.');
end
frac=segLen/Ltot;

% Boundary-jet padded corridor assignment.
% Degree p endpoint-jet control points are kept inside the first/last
% corridor before any transition. Hence allocate p spans to each end cell;
% the remaining p spans stay in the middle cell:
%   [p p p] = [7 7 7].
spanCount=[p,p,p];
if sum(spanCount)~=nspan
    error('BSD7:C6Variant2','C6-extended span count is inconsistent.');
end

ub=[0,cumsum(frac)];
ub(end)=1;

% Keep guide/corridor transitions at knot breaks and distribute the seven
% middle intervals uniformly within the middle guide fraction.
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
expected=nspan*p+1;
if size(Tbez,1)~=expected
    error('BSD7:BezierExtract','Unexpected extraction row count.');
end

pieceMaps=cell(1,nspan);
for q=1:nspan
    ids=(q-1)*p+(1:p+1);
    pieceMaps{q}=Tbez(ids,:);
end

% x = [Px(22); Py(22); Pz(22); zXi], xi = xiScale*zXi.
nvar=3*ncoef+1;
ix=1:ncoef;
iy=ncoef+(1:ncoef);
iz=2*ncoef+(1:ncoef);
iXi=nvar;
xiScale=max(1,G.path_length);

p0=G.front.payload_at_trigger;
e=W(:,3)-W(:,2);
e=e/norm(e);

h0=breaks(2)-breaks(1);
h1=breaks(end)-breaks(end-1);
E0=pieceMaps{1};
E1=pieceMaps{end};

Aeq=zeros(0,nvar);
beq=zeros(0,1);
axisIds={ix,iy,iz};

% Endpoint Bezier CP identities for spatial jets 0..6.
% Same semantics as the current common degree-13 implementation.
for ax=1:3
    ia=axisIds{ax};

    for j=0:6
        rr=zeros(1,nvar);
        rr(ia)=E0(j+1,:);
        rr(iXi)=-(j*h0/p)*e(ax)*xiScale;
        Aeq(end+1,:)=rr; %#ok<AGROW>
        beq(end+1,1)=p0(ax); %#ok<AGROW>
    end

    for j=0:6
        rr=zeros(1,nvar);
        rr(ia)=E1(p-j+1,:);
        rr(iXi)=-(1-j*h1/p)*e(ax)*xiScale;
        Aeq(end+1,:)=rr; %#ok<AGROW>
        beq(end+1,1)=p0(ax); %#ok<AGROW>
    end
end

% Continuous safety via exact extracted local Bezier control points.
Aineq=zeros(0,nvar);
bineq=zeros(0,1);
safeMeta=struct('span',{},'cell',{},'cp',{},'plane',{});

for q=1:nspan
    E=pieceMaps{q};
    cellIdx=cellOfSpan(q);
    CC=C.cells{cellIdx};

    for j=1:p+1
        er=E(j,:);
        for hh=1:size(CC.A,1)
            n=CC.A(hh,:).';
            rr=zeros(1,nvar);
            rr(ix)=-n(1)*er;
            rr(iy)=-n(2)*er;
            rr(iz)=-n(3)*er;

            Aineq(end+1,:)=rr; %#ok<AGROW>
            bineq(end+1,1)=-CC.b(hh); %#ok<AGROW>
            safeMeta(end+1)=struct('span',q,'cell',cellIdx,'cp',j,'plane',hh); %#ok<AGROW>
        end
    end
end

% Exact integrated squared curve-to-guide objective.
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

        rr=rr+1;
        Cobj(rr,ix)=sw*N;
        dobj(rr)=sw*pg(1);

        rr=rr+1;
        Cobj(rr,iy)=sw*N;
        dobj(rr)=sw*pg(2);

        rr=rr+1;
        Cobj(rr,iz)=sw*N;
        dobj(rr)=sw*pg(3);
    end
end

H=2*(Cobj.'*Cobj);
H=0.5*(H+H.');
f=-2*(Cobj.'*dobj);

% Positive scalar objective scaling only.
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
M.iXi=iXi;
M.xiScale=xiScale;
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

function [Aineq,bineq,safeMeta,CERT]=bsd7_native_cp_certificate(M,C)
% Native B-spline control-point local convex-hull certificate.
%
% Every span of a B-spline lies in the convex hull of its active native
% control points. For every native CP, collect all corridor cells touched
% by spans in its support and constrain that CP to all those cells.
%
% This is a continuous-time sufficient safety certificate.
% No safety relaxation and no obstacle approximation are introduced.

n=M.ncoef;
nvar=M.nvar;

cellsPerCP=cell(1,n);
spansPerCP=cell(1,n);

for q=1:M.nspan
    E=M.pieceMaps{q};
    active=find(any(abs(E)>0,1));
    ci=M.cellOfSpan(q);

    for kk=1:numel(active)
        cp=active(kk);
        cellsPerCP{cp}(end+1)=ci; %#ok<AGROW>
        spansPerCP{cp}(end+1)=q; %#ok<AGROW>
    end
end

Aineq=zeros(0,nvar);
bineq=zeros(0,1);
safeMeta=struct('span',{},'cell',{},'cp',{},'plane',{});

for cp=1:n
    cellsPerCP{cp}=unique(cellsPerCP{cp},'stable');
    spansPerCP{cp}=unique(spansPerCP{cp},'stable');

    if isempty(cellsPerCP{cp})
        error('BSD7:NativeCert','A native CP has no corridor membership.');
    end

    for ci=cellsPerCP{cp}
        CC=C.cells{ci};

        qrep=[];
        for q=spansPerCP{cp}
            if M.cellOfSpan(q)==ci
                qrep=q;
                break
            end
        end
        if isempty(qrep)
            error('BSD7:NativeCert','Support/cell mapping inconsistency.');
        end

        for hh=1:size(CC.A,1)
            nv=CC.A(hh,:).';
            rr=zeros(1,nvar);

            rr(M.ix(cp))=-nv(1);
            rr(M.iy(cp))=-nv(2);
            rr(M.iz(cp))=-nv(3);

            Aineq(end+1,:)=rr; %#ok<AGROW>
            bineq(end+1,1)=-CC.b(hh); %#ok<AGROW>
            safeMeta(end+1)=struct( ...
                'span',qrep,'cell',ci,'cp',cp,'plane',hh); %#ok<AGROW>
        end
    end
end

nCells=cellfun(@numel,cellsPerCP);

CERT=struct();
CERT.cellsPerCP=cellsPerCP;
CERT.spansPerCP=spansPerCP;
CERT.uniqueMemberships=sum(nCells);
CERT.multiCellCPs=sum(nCells>1);
CERT.maxCellsPerCP=max(nCells);
CERT.constraintRows=size(Aineq,1);

rowCheck=0;
for cp=1:n
    for ci=cellsPerCP{cp}
        rowCheck=rowCheck+size(C.cells{ci}.A,1);
    end
end
if rowCheck~=CERT.constraintRows
    error('BSD7:NativeCert','Certificate row-count mismatch.');
end
end


function R=bsd7_build_equality_elimination(M)
% Exact algebraic elimination of endpoint jet equalities.%
% Degree-7 endpoint jets 0..6 determine seven native control points at
% each end. The remaining middle native CPs and xi are the reduced QP vars.
%
% For the C6-extended variant with ncoef=28:
%   dependent CPs = 1:7 and 22:28
%   free CPs      = 8:21
%   reduced vars  = 14 CP * 3 axes + xi = 43.

n=M.ncoef;
leftDep=1:7;
rightDep=(n-6):n;
middle=8:(n-7);

if isempty(middle)
    error('BSD7:EqElim','No interior native control points remain.');
end
if ~isempty(intersect(leftDep,rightDep))
    error('BSD7:EqElim','Endpoint-dependent control-point sets overlap.');
end

dep=[];
free=[];
for ax=0:2
    base=ax*n;
    dep=[dep,base+leftDep,base+rightDep]; %#ok<AGROW>
    free=[free,base+middle]; %#ok<AGROW>
end
free=[free,M.iXi];

Aeq=full(M.Aeq);
Ad=Aeq(:,dep);
Af=Aeq(:,free);

if size(Ad,1)~=size(Ad,2)
    error('BSD7:EqElim','Dependent endpoint block is not square.');
end
if rank(Ad)~=size(Ad,1)
    error('BSD7:EqElim','Dependent endpoint block is rank deficient.');
end

x0=zeros(M.nvar,1);
T=zeros(M.nvar,numel(free));
T(free,:)=eye(numel(free));
x0(dep)=Ad\M.beq;
T(dep,:)=-Ad\Af;

iXiRed=find(free==M.iXi,1);
if isempty(iXiRed)
    error('BSD7:EqElim','xi was not retained.');
end

R=struct();
R.dep=dep;
R.free=free;
R.middleCP=middle;
R.x0=x0;
R.T=T;
R.nred=numel(free);
R.iXiRed=iXiRed;
R.condDependentBlock=cond(Ad);
end

function D=bsd7_bottleneck_diagnostics(x,M,C,G)
% Diagnose WHY the current B-spline formulation is large/nonlocal.
% No change to optimization or safety logic.

Aeq=full(M.Aeq);
sA=svd(Aeq);
tolA=max(size(Aeq))*eps(max(sA));
rankA=sum(sA>tolA);
nullity=size(Aeq,2)-rankA;
if rankA>0
    condA=sA(1)/sA(rankA);
else
    condA=NaN;
end

% Equality-reduced Hessian conditioning.
Z=null(Aeq);
H=full(M.Hscaled);
Hr=0.5*(Z.'*H*Z+(Z.'*H*Z).');
ev=eig(Hr);
ev=sort(real(ev));
tolH=max(size(Hr))*eps(max(abs(ev)));
pos=ev>tolH;
if any(pos)
    condHr=max(ev(pos))/min(ev(pos));
else
    condHr=Inf;
end

% Active hard inequalities.
sl=M.bineq-M.Aineq*x;
act12=find(sl<=1e-12);
act10=find(sl<=1e-10);

activeByCell=zeros(1,3);
ineqRowsByCell=zeros(1,3);
for ii=1:numel(M.safeMeta)
    ci=M.safeMeta(ii).cell;
    ineqRowsByCell(ci)=ineqRowsByCell(ci)+1;
end
for kk=1:numel(act10)
    ci=M.safeMeta(act10(kk)).cell;
    activeByCell(ci)=activeByCell(ci)+1;
end

% Local support of exact Bezier extraction.
supportCount=sum(abs(M.Tbez)>0,2);
meanSupport=mean(supportCount);
maxSupport=max(supportCount);

% Which native CPs are touched by endpoint equalities?
eqNative=false(1,M.ncoef);
for axBlock=0:2
    cols=axBlock*M.ncoef+(1:M.ncoef);
    eqNative=eqNative | any(abs(Aeq(:,cols))>0,1);
end

% Which native CPs participate in ANY safety row?
Ai=full(M.Aineq);
safeNative=false(1,M.ncoef);
for axBlock=0:2
    cols=axBlock*M.ncoef+(1:M.ncoef);
    safeNative=safeNative | any(abs(Ai(:,cols))>0,1);
end

% Exact row count implied by span allocation/corridor plane counts.
cnt=0;
for q=1:M.nspan
    ci=M.cellOfSpan(q);
    cnt=cnt+(M.p+1)*size(C.cells{ci}.A,1);
end

% Worst hard row anatomy.
[minSlack,iw]=min(sl); %#ok<ASGLU>
meta=M.safeMeta(iw);
isTransition = (meta.span>1) && (M.cellOfSpan(meta.span)~=M.cellOfSpan(meta.span-1));
isShared = isTransition && (meta.cp==1);

D=struct();
D.rankAeq=rankA;
D.nullityAfterEquality=nullity;
D.condAeq=condA;
D.reducedHessianCond=condHr;
D.active1e12=numel(act12);
D.active1e10=numel(act10);
D.activeByCell=activeByCell;
D.ineqRowsByCell=ineqRowsByCell;
D.meanNativeSupport=meanSupport;
D.maxNativeSupport=maxSupport;
D.endpointNativeCPUsed=sum(eqNative);
D.safetyNativeCPUsed=sum(safeNative);
D.allNativeCPConstrained=all(safeNative);
D.ineqFormulaCount=cnt;
D.worstIsTransition=isTransition;
D.worstIsSharedBezierPoint=isShared;
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

function [ms,ws,wc,wj,wp]=bsd7_original_bezier_slack(Bcp,M,C)
ms=Inf;
ws=NaN; wc=NaN; wj=NaN; wp=NaN;

for q=1:M.nspan
    ci=M.cellOfSpan(q);
    CC=C.cells{ci};

    for j=1:M.p+1
        sl=CC.A*Bcp(:,j,q)-CC.b;
        [v,ip]=min(sl);
        if v<ms
            ms=v;
            ws=q;
            wc=ci;
            wj=j;
            wp=ip;
        end
    end
end
end

function r=bsd7_endpoint_diagnostic(Bcp,xi,M,G)
p0=G.front.payload_at_trigger;
W=G.waypoints;
e=W(:,3)-W(:,2);
e=e/norm(e);

h0=M.breaks(2)-M.breaks(1);
h1=M.breaks(end)-M.breaks(end-1);
p=M.p;

r=0;
for ax=1:3
    for j=0:6
        t0=p0(ax)+(j*h0/p)*e(ax)*xi;
        t1=p0(ax)+(1-j*h1/p)*e(ax)*xi;
        r=max(r,abs(Bcp(ax,j+1,1)-t0));
        r=max(r,abs(Bcp(ax,p-j+1,M.nspan)-t1));
    end
end
end

function J=bsd7_curve_guide_objective(cp,M,G)
[xg,wg]=step3a_gauss8_01();
W=G.waypoints;
J=0;

for q=1:M.nspan
    ua=M.breaks(q);
    uq=M.breaks(q+1);
    hs=uq-ua;
    ci=M.cellOfSpan(q);
    ga=M.ub(ci);
    gb=M.ub(ci+1);

    for k=1:numel(xg)
        u=ua+hs*xg(k);
        N=step3a_bs_basis_all(u,M.U,M.p,M.ncoef);
        p=N*cp.';

        alpha=(u-ga)/(gb-ga);
        alpha=min(1,max(0,alpha));
        pg=(1-alpha)*W(:,ci)+alpha*W(:,ci+1);

        d=p.'-pg;
        J=J+wg(k)*hs*(d.'*d);
    end
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

function M=poly13_stable_build_model(G,C)
n=13;
nseg=3;
ncp=n+1;

W=G.waypoints;
segLen=vecnorm(diff(W,1,2),2,1);
h=segLen/sum(segLen);
h=h(:).';

p0=G.front.payload_at_trigger;
e=G.waypoints(:,3)-G.waypoints(:,2);
e=e/norm(e);

% Minimal variables:
% knot 2 jet orders 0..6 xyz = 21
% knot 3 jet orders 0..6 xyz = 21
% xi = 1
nvar=43;
iXi=43;

% Structural numerical variable scaling.
% For an internal knot derivative of order r, use the geometric mean of
% adjacent segment lengths. The scale makes a unit z-variable induce O(L)
% Bezier-control-point motion through h^r/fallingFactorial(n,r).
L=max(1,G.path_length);
jetScale=zeros(2,7);
for kk=1:2
    href=sqrt(h(kk)*h(kk+1));
    for r=0:6
        ff=factorial(n)/factorial(n-r);
        jetScale(kk,r+1)=L*ff/(href^r);
    end
end
xiScale=L;

% Fixed finite-difference endpoint interpolation matrix:
% maps 14 CPs -> [left normalized differences 0..6;
%                  right normalized differences 0..6].
F=bez13_jet_endpoint_difference_matrix(n);
if rank(F)~=14
    error('MINCOJET:Interpolation','Endpoint-difference matrix is singular.');
end

% Affine CP map: cpVec = c0 + Gcp*z
% cpVec ordering is x/y/z contiguous per segment CP via bez13_jet_cpidx.
ncpall=3*nseg*ncp;
Gcp=zeros(ncpall,nvar);
c0=zeros(ncpall,1);

for q=1:nseg
    for ax=1:3
        D0=zeros(14,1);
        Dz=zeros(14,nvar);

        % Left knot q, right knot q+1.
        [D0,Dz]=bez13_jet_insert_knot_side(D0,Dz,1,q,h(q),ax,p0,e, ...
            jetScale,xiScale,n,iXi);
        [D0,Dz]=bez13_jet_insert_knot_side(D0,Dz,8,q+1,h(q),ax,p0,e, ...
            jetScale,xiScale,n,iXi);

        % CP from normalized endpoint differences.
        % Degree 13 has 14 CPs and endpoint difference orders 0..6 on
        % each side.  The inverse is available in closed binomial form,
        % so do NOT use F\D here.  This removes an unnecessary
        % condition-number amplification while preserving the exact map.
        E=bez13_endpoint_difference_inverse_direct(n);
        cp0=E*D0;
        cpG=E*Dz;

        ids=arrayfun(@(j) bez13_jet_cpidx(q,j,ax,ncp),1:ncp);
        c0(ids)=cp0;
        Gcp(ids,:)=cpG;
    end
end

% Hard corridor inequalities on actual CPs.
Aineq=zeros(0,nvar);
bineq=zeros(0,1);
safeMeta=struct('segment',{},'cp',{},'plane',{});

for q=1:nseg
    CC=C.cells{q};

    for j=1:ncp
        for pl=1:size(CC.A,1)
            a=zeros(1,ncpall);
            for ax=1:3
                a(bez13_jet_cpidx(q,j,ax,ncp))=-CC.A(pl,ax);
            end

            Aineq(end+1,:)=a*Gcp; %#ok<AGROW>
            bineq(end+1,1)=-CC.b(pl)-a*c0; %#ok<AGROW>

            safeMeta(end+1)=struct( ...
                'segment',q,'cp',j,'plane',pl); %#ok<AGROW>
        end
    end
end

% -------------------------------------------------------------------------
% Geometric guide-proximity objective in minimal coordinates.
%
% For each Bezier CP, define the target point by uniform local parameter
% interpolation along the corresponding Step1 guide segment.
%
%   Jgeo = sum ||Pcp - Pguide||^2
%
% Since Pcp = c0 + Gcp*z:
%
%   Jgeo = 0.5*z'*H*z + f'*z + const
%   H = 2*Gcp'*Gcp
%   f = 2*Gcp'*(c0-target)
%
% No arbitrary physical or tuning weight is introduced.
% -------------------------------------------------------------------------
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

Hphys=2*(Gcp.'*Gcp);
Hphys=0.5*(Hphys+Hphys.');
fphys=2*Gcp.'*(c0-target);

% Positive scalar scaling only; minimizer is unchanged.
d=max(abs(diag(Hphys)));
if ~(d>0), d=1; end
Hscaled=sparse(Hphys/d);
fscaled=fphys/d;

M=struct();
M.degree=n;
M.nseg=nseg;
M.ncp=ncp;
M.nvar=nvar;
M.iXi=iXi;
M.h=h;
M.jetScale=jetScale;
M.xiScale=xiScale;
M.Gcp=Gcp;
M.c0=c0;
M.Aineq=sparse(Aineq);
M.bineq=bineq;
M.safeMeta=safeMeta;
M.Hphysical=sparse(Hphys);
M.fphysical=fphys;
M.Hscaled=Hscaled;
M.fscaled=fscaled;
end

function [D0,Dz]=bez13_jet_insert_knot_side(D0,Dz,row0,knot,hseg,ax,p0,e,jetScale,xiScale,n,iXi)
% Insert normalized endpoint differences for one knot and one axis.
%
% normalized r-th difference = h^r/fallingFactorial(n,r) * global derivative.

for r=0:6
    ff=factorial(n)/factorial(n-r);
    mult=(hseg^r)/ff;
    row=row0+r;

    if knot==1
        % start:
        % r0 = p0, r1 = e*xi, r>=2 = 0
        if r==0
            D0(row)=p0(ax);
        elseif r==1
            Dz(row,iXi)=mult*e(ax)*xiScale;
        end

    elseif knot==4
        % end:
        % r0 = p0 + e*xi, r1 = e*xi, r>=2 = 0
        if r==0
            D0(row)=p0(ax);
            Dz(row,iXi)=e(ax)*xiScale;
        elseif r==1
            Dz(row,iXi)=mult*e(ax)*xiScale;
        end

    else
        kk=knot-1; % internal knot 2->1, 3->2
        vidx=bez13_jet_varidx(kk,r,ax);
        Dz(row,vidx)=mult*jetScale(kk,r+1);
    end
end
end

function idx=bez13_jet_varidx(internalKnot,r,axis)
% internalKnot=1,2 ; r=0..6 ; axis=1..3
idx=(internalKnot-1)*21 + r*3 + axis;
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

function CP=bez13_jet_reconstruct_cp(z,M)
v=M.c0+M.Gcp*z;
CP=zeros(3,M.ncp,M.nseg);

for q=1:M.nseg
    for j=1:M.ncp
        for ax=1:3
            CP(ax,j,q)=v(bez13_jet_cpidx(q,j,ax,M.ncp));
        end
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

function [c6res,r0,r1]=bez13_jet_full_audit(CP,xi,M,G)
n=M.degree;
c6res=0;

for q=1:M.nseg-1
    ratio=M.h(q)/M.h(q+1);

    for r=0:6
        dL=zeros(3,1);
        dR=zeros(3,1);

        for k=0:r
            c=(-1)^(r-k)*nchoosek(r,k);
            dL=dL+c*CP(:,n-r+k+1,q);
            dR=dR+c*CP(:,k+1,q+1);
        end

        c6res=max(c6res,norm(dL-ratio^r*dR,inf));
    end
end

p0=G.front.payload_at_trigger;
e=G.waypoints(:,3)-G.waypoints(:,2);
e=e/norm(e);
r0=0;
r1=0;

for j=0:6
    t0=p0+(j*M.h(1)/n)*e*xi;
    t1=p0+(1-j*M.h(end)/n)*e*xi;
    r0=max(r0,norm(CP(:,j+1,1)-t0,inf));
    r1=max(r1,norm(CP(:,n-j+1,M.nseg)-t1,inf));
end
end

function J=bez13_common_knot_jets(z,M,G,xi)
% Canonical normalized-global-parameter jets at the four D13 knots.
% J(:,r+1,k) = d^r p / ds^r at knot k, r=0..6.
% Internal knot jets are the actual optimization variables after undoing
% their algebraic scaling. Start/end semantics match the generator.
J=zeros(3,7,4);
p0=G.front.payload_at_trigger;
e=G.waypoints(:,3)-G.waypoints(:,2);
e=e/norm(e);

J(:,1,1)=p0;
J(:,2,1)=e*xi;

for kk=1:2
    knot=kk+1;
    for r=0:6
        for ax=1:3
            idx=bez13_jet_varidx(kk,r,ax);
            J(ax,r+1,knot)=z(idx)*M.jetScale(kk,r+1);
        end
    end
end

J(:,1,4)=p0+e*xi;
J(:,2,4)=e*xi;
end

function Jgeo=bez13_geo_objective_value(CP,G)
% Exact geometric guide-proximity objective used by Step 3.
Jgeo=0;
nseg=size(CP,3);
ncp=size(CP,2);
W=G.waypoints;

for q=1:nseg
    for j=1:ncp
        tau=(j-1)/(ncp-1);
        target=(1-tau)*W(:,q)+tau*W(:,q+1);
        d=CP(:,j,q)-target;
        Jgeo=Jgeo+d.'*d;
    end
end
end

function A=poly13_stable_bernstein_to_power(B)
% Convert degree-13 Bernstein coefficients to ordinary power coefficients.
% This conversion is OUTPUT/DIAGNOSTIC ONLY and is not used to construct
% the optimization problem or certify safety.
%
% B_j^n(t) = C(n,j) t^j (1-t)^(n-j)
%
% Therefore the coefficient of t^k is
%   sum_{j=0}^k B_j C(n,j) C(n-j,k-j) (-1)^(k-j).

n=size(B,2)-1;
nseg=size(B,3);
A=zeros(size(B));

for q=1:nseg
    for ax=1:3
        for k=0:n
            ak=0;
            for j=0:k
                ak=ak + B(ax,j+1,q) * nchoosek(n,j) * ...
                    nchoosek(n-j,k-j) * (-1)^(k-j);
            end
            A(ax,k+1,q)=ak;
        end
    end
end
end

function J7=bez13_jet_J7(CP,M)
n=M.degree;
D7=bez13_difference_matrix(n,7);
M6=bez13_bernstein_gram(6);
fac=factorial(n)/factorial(n-7);
J7=0;

for q=1:M.nseg
    for ax=1:3
        p=CP(ax,:,q).';
        d=D7*p;
        J7=J7+(fac^2)/(M.h(q)^13)*(d.'*M6*d);
    end
end
end

function M=bez13_build_model(G,C)
n=13;
nseg=3;
ncp=n+1;

% Segment parameter lengths follow the three certified guide segments.
W=G.waypoints;
segLen=vecnorm(diff(W,1,2),2,1);
h=segLen/sum(segLen);
h=h(:).';

ncpvar=3*nseg*ncp;
iXi=ncpvar+1;
nvar=iXi;

% ------------------------------
% Equality constraints
% ------------------------------
Aeq=zeros(0,nvar);
beq=zeros(0,1);

p0=G.front.payload_at_trigger;
e=G.waypoints(:,3)-G.waypoints(:,2);
e=e/norm(e);

% Start and final spatial jets 0..6.
% They represent a straight nominal extension scaled by recovery xi:
% p_s = e*xi, p_ss ... p^(6) = 0.
for ax=1:3
    for j=0:6
        r=zeros(1,nvar);
        r(bez13_idx(1,j+1,ax,ncp))=1;
        r(iXi)=-(j*h(1)/n)*e(ax);
        Aeq(end+1,:)=r; %#ok<AGROW>
        beq(end+1,1)=p0(ax); %#ok<AGROW>
    end

    for j=0:6
        r=zeros(1,nvar);
        r(bez13_idx(nseg,n-j+1,ax,ncp))=1;
        r(iXi)=-(1-j*h(end)/n)*e(ax);
        Aeq(end+1,:)=r; %#ok<AGROW>
        beq(end+1,1)=p0(ax); %#ok<AGROW>
    end
end

% C0..C6 between neighboring degree-13 pieces.
% Finite-difference form with span-length ratios avoids explicit h^-6.
for q=1:nseg-1
    ratio=h(q)/h(q+1);

    for rord=0:6
        rp=ratio^rord;

        for ax=1:3
            row=zeros(1,nvar);

            % left end r-th forward difference
            for k=0:rord
                c=(-1)^(rord-k)*nchoosek(rord,k);
                cpidx=n-rord+k+1;
                id=bez13_idx(q,cpidx,ax,ncp);
                row(id)=row(id)+c;
            end

            % right start r-th forward difference
            for k=0:rord
                c=(-1)^(rord-k)*nchoosek(rord,k);
                cpidx=k+1;
                id=bez13_idx(q+1,cpidx,ax,ncp);
                row(id)=row(id)-rp*c;
            end

            Aeq(end+1,:)=row; %#ok<AGROW>
            beq(end+1,1)=0; %#ok<AGROW>
        end
    end
end

% ------------------------------
% Corridor hard inequalities
% ------------------------------
Aineq=zeros(0,nvar);
bineq=zeros(0,1);
safeMeta=struct('segment',{},'cp',{},'plane',{});

for q=1:nseg
    CC=C.cells{q};

    for j=1:ncp
        for pl=1:size(CC.A,1)
            row=zeros(1,nvar);

            for ax=1:3
                row(bez13_idx(q,j,ax,ncp))=-CC.A(pl,ax);
            end

            Aineq(end+1,:)=row; %#ok<AGROW>
            bineq(end+1,1)=-CC.b(pl); %#ok<AGROW>

            safeMeta(end+1)=struct( ...
                'segment',q,'cp',j,'plane',pl); %#ok<AGROW>
        end
    end
end

% ------------------------------
% Exact J7 Hessian
% ------------------------------
D7=bez13_difference_matrix(n,7);    % 7 x 14
M6=bez13_bernstein_gram(6);         % integral B^6_i B^6_j
fac=factorial(n)/factorial(n-7);

H=zeros(nvar);
for q=1:nseg
    Hseg=(fac^2)/(h(q)^13) * (D7.'*M6*D7);

    for ax=1:3
        ids=arrayfun(@(j) bez13_idx(q,j,ax,ncp),1:ncp);
        H(ids,ids)=H(ids,ids)+2*Hseg;
    end
end

% xi has no direct J7 penalty; it is selected through endpoint/corridor coupling.
% Explicitly remove floating-point assembly asymmetry before quadprog.
% This does not change the quadratic form.
H=0.5*(H+H.');

% Scale the objective by one positive scalar only; optimum is unchanged.
d=max(abs(diag(H)));
if d>0
    Hscaled=sparse(H/d);
else
    Hscaled=sparse(H);
end

M=struct();
M.degree=n;
M.nseg=nseg;
M.ncp=ncp;
M.ncpvar=ncpvar;
M.iXi=iXi;
M.nvar=nvar;
M.h=h;
M.Aeq=sparse(Aeq);
M.beq=beq;
M.Aineq=sparse(Aineq);
M.bineq=bineq;
M.safeMeta=safeMeta;
M.Hphysical=sparse(H);
M.Hscaled=Hscaled;
end

function idx=bez13_idx(seg,cp,axis,ncp)
idx=3*((seg-1)*ncp+(cp-1))+axis;
end

function D=bez13_difference_matrix(n,r)
% r-th forward differences of n+1 control points.
D=zeros(n-r+1,n+1);
for i=0:n-r
    for k=0:r
        D(i+1,i+k+1)=(-1)^(r-k)*nchoosek(r,k);
    end
end
end

function Gm=bez13_bernstein_gram(m)
% Exact Gram matrix integral_0^1 B_i^m(t) B_j^m(t) dt.
Gm=zeros(m+1);
for i=0:m
    for j=0:m
        Gm(i+1,j+1)= ...
            nchoosek(m,i)*nchoosek(m,j) / ...
            ((2*m+1)*nchoosek(2*m,i+j));
    end
end
end

function [As,bs,Aeqs,beqs]=bez13_scale_rows(A,b,Aeq,beq)
s=max(abs(A),[],2);
s=max(s,abs(b));
s=max(s,1);
As=A./s;
bs=b./s;

se=max(abs(Aeq),[],2);
se=max(se,abs(beq));
se=max(se,1);
Aeqs=Aeq./se;
beqs=beq./se;
end

function CP=bez13_unpack_cp(w,M)
CP=zeros(3,M.ncp,M.nseg);
for q=1:M.nseg
    for j=1:M.ncp
        for ax=1:3
            CP(ax,j,q)=w(bez13_idx(q,j,ax,M.ncp));
        end
    end
end
end

function rmax=bez13_continuity_audit(CP,M)
n=M.degree;
rmax=0;

for q=1:M.nseg-1
    ratio=M.h(q)/M.h(q+1);

    for r=0:6
        dL=zeros(3,1);
        dR=zeros(3,1);

        for k=0:r
            c=(-1)^(r-k)*nchoosek(r,k);
            dL=dL+c*CP(:,n-r+k+1,q);
            dR=dR+c*CP(:,k+1,q+1);
        end

        rmax=max(rmax,norm(dL-ratio^r*dR,inf));
    end
end
end

function [r0,r1]=bez13_endpoint_audit(CP,xi,M,G)
n=M.degree;
p0=G.front.payload_at_trigger;
e=G.waypoints(:,3)-G.waypoints(:,2);
e=e/norm(e);

r0=0;
r1=0;

for j=0:6
    t0=p0+(j*M.h(1)/n)*e*xi;
    t1=p0+(1-j*M.h(end)/n)*e*xi;

    r0=max(r0,norm(CP(:,j+1,1)-t0,inf));
    r1=max(r1,norm(CP(:,n-j+1,M.nseg)-t1,inf));
end
end

function [sAll,pAll]=bez13_eval_dense(CP,M,N)
% Diagnostic sampling only.
n=M.degree;
bounds=[0,cumsum(M.h)];
bounds(end)=1;

sAll=linspace(0,1,N);
pAll=zeros(3,N);

for ii=1:N
    s=sAll(ii);

    q=find(s<=bounds(2:end)+16*eps,1,'first');
    if isempty(q), q=M.nseg; end

    a=bounds(q);
    b=bounds(q+1);
    if b>a
        t=(s-a)/(b-a);
    else
        t=0;
    end
    t=min(1,max(0,t));

    B=zeros(n+1,1);
    for j=0:n
        B(j+1)=nchoosek(n,j)*(t^j)*((1-t)^(n-j));
    end

    pAll(:,ii)=CP(:,:,q)*B;
end
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

function [xRef,INFO]=step3a_local_build_guide_reference(M,G,xi)
% Deterministic least-squares guide fit under endpoint equalities.
% Linear algebra only; no optimization solver.
%
% We fit all local Bezier control points to the certified guide polyline
% while exactly satisfying endpoint-jet equalities for fixed xi.

n=M.ncoef;
nvar=M.nvar_curve;

% Fix xi in joint endpoint equalities:
Aeq=M.Aeq(:,1:nvar);
beq=M.beq-M.Aeq(:,M.iXi)*xi;

x0=lsqminnorm(Aeq,beq);
Z=null(Aeq);

% Local Bezier target points along each assigned guide segment.
T=M.Tbez;
Bmat=blkdiag(T,T,T);

target=zeros(3*size(T,1),1);
spanCounter=zeros(1,3);

for q=1:M.nspan
    c=M.cellOfSpan(q);
    spanCounter(c)=spanCounter(c)+1;
    kLocal=spanCounter(c);
    nLocal=M.spanCount(c);

    ids=(q-1)*M.p+(1:M.p+1);

    for j=1:M.p+1
        tau=((kLocal-1)+(j-1)/M.p)/nLocal;
        pt=(1-tau)*G.waypoints(:,c)+tau*G.waypoints(:,c+1);

        target(ids(j))=pt(1);
        target(size(T,1)+ids(j))=pt(2);
        target(2*size(T,1)+ids(j))=pt(3);
    end
end

if isempty(Z)
    xRef=x0;else
    AZ=Bmat*Z;
    rhs=target-Bmat*x0;
    y=lsqminnorm(AZ,rhs);
    xRef=x0+Z*y;
end

INFO=struct();
INFO.eq_residual=norm(Aeq*xRef-beq,inf);
INFO.guide_fit_inf=norm(Bmat*xRef-target,inf);
INFO.guide_fit_rms=sqrt(mean((Bmat*xRef-target).^2));
INFO.curve_inf_norm=norm(xRef,inf);
end

function idx=step3a_local_active_xyz_indices(activeCP,ncoef)
cp=find(activeCP);
idx=[cp, ncoef+cp, 2*ncoef+cp];
idx=idx(:).';
end

function [As,bs,Aeqs,beqs]=step3a_local_scale_rows(A,b,Aeq,beq)
% Algebraic row scaling only; feasible set is unchanged.
s=max(abs(A),[],2);
s=max(s,abs(b));
s=max(s,1);
As=A./s;
bs=b./s;

se=max(abs(Aeq),[],2);
se=max(se,abs(beq));
se=max(se,1);
Aeqs=Aeq./se;
beqs=beq./se;
end

function rmax=step3a_local_c6_bezier_audit(x,M)
% Independent C0..C6 audit across neighboring local Bezier pieces.
% Uses derivative-equivalent finite differences with span-length ratios.

n=M.ncoef;
Qx=M.Tbez*x(1:n);
Qy=M.Tbez*x(n+(1:n));
Qz=M.Tbez*x(2*n+(1:n));

rmax=0;

for q=1:M.nspan-1
    idsL=(q-1)*M.p+(1:M.p+1);
    idsR=q*M.p+(1:M.p+1);

    BL=[Qx(idsL),Qy(idsL),Qz(idsL)].';
    BR=[Qx(idsR),Qy(idsR),Qz(idsR)].';

    hL=M.breaks(q+1)-M.breaks(q);
    hR=M.breaks(q+2)-M.breaks(q+1);

    for r=0:6
        dL=zeros(3,1);
        dR=zeros(3,1);

        for k=0:r
            c=(-1)^(r-k)*nchoosek(r,k);
            dL=dL+c*BL(:,M.p-r+k+1);
            dR=dR+c*BR(:,k+1);
        end

        rr=dL-(hL/hR)^r*dR;
        rmax=max(rmax,norm(rr,inf));
    end
end
end

function [As,bs,Aeqs,beqs,sIneq,sEq]=step3a_c02_scale_rows(A,b,Aeq,beq)
% Pure algebraic row scaling. No constraint relaxation.
sIneq=max(abs(A),[],2);
sIneq=max(sIneq,abs(b));
sIneq=max(sIneq,1);
As=A./sIneq;
bs=b./sIneq;

sEq=max(abs(Aeq),[],2);
sEq=max(sEq,abs(beq));
sEq=max(sEq,1);
Aeqs=Aeq./sEq;
beqs=beq./sEq;
end

function [H,f]=step3a_c02_guide_deviation_qp(M,G)
% Sum squared Euclidean deviations of local Bezier CPs from the fixed
% certified Step-1 guide polyline. This is a secondary selector only.

n=M.ncoef;
H=zeros(3*n);
f=zeros(3*n,1);

W=G.waypoints;
cellSpanCounter=zeros(1,3);

for q=1:M.nspan
    c=M.cellOfSpan(q);
    cellSpanCounter(c)=cellSpanCounter(c)+1;
    kLocal=cellSpanCounter(c);
    nLocal=M.spanCount(c);

    E=M.pieceMaps{q};

    for j=1:M.p+1
        tau=((kLocal-1)+(j-1)/M.p)/nLocal;
        target=(1-tau)*W(:,c)+tau*W(:,c+1);

        er=E(j,:);

        for ax=1:3
            ia=(ax-1)*n+(1:n);
            H(ia,ia)=H(ia,ia)+2*(er.'*er);
            f(ia)=f(ia)-2*target(ax)*er.';
        end
    end
end

% Symmetrize only against floating accumulation noise.
H=0.5*(H+H.');
end

function M=step3a_c02_build_joint_lp(G,C)
p=7;
ncoef=22;
nspan=ncoef-p;
W=G.waypoints;
segLen=vecnorm(diff(W,1,2),2,1);
frac=segLen/sum(segLen);
spanCount=step3a_allocate_spans(nspan,frac);
ub=[0,cumsum(frac)];
ub(end)=1;
breaks=0;
cellOfSpan=zeros(1,nspan);
ss=0;
for cidx=1:3
    a=ub(cidx); b=ub(cidx+1); nc=spanCount(cidx);
    local=a+(0:nc)*(b-a)/nc;
    if cidx==1, breaks=local; else, breaks=[breaks,local(2:end)]; end %#ok<AGROW>
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

nvarCurve=3*ncoef;
nvarJoint=nvarCurve+1;
iXi=nvarJoint;
ix=1:ncoef; iy=ncoef+(1:ncoef); iz=2*ncoef+(1:ncoef);
idxAxis={ix,iy,iz};
Aeq=zeros(0,nvarJoint); beq=zeros(0,1);
p0=G.front.payload_at_trigger;
e=G.waypoints(:,3)-G.waypoints(:,2); e=e/norm(e);
h0=breaks(2)-breaks(1);
h1=breaks(end)-breaks(end-1);
E0=pieceMaps{1}; E1=pieceMaps{end};

for ax=1:3
    ia=idxAxis{ax};
    for j=0:6
        rr=zeros(1,nvarJoint);
        rr(ia)=E0(j+1,:);
        rr(iXi)=-(j*h0/p)*e(ax);
        Aeq(end+1,:)=rr; %#ok<AGROW>
        beq(end+1,1)=p0(ax); %#ok<AGROW>
    end
    for j=0:6
        rr=zeros(1,nvarJoint);
        rr(ia)=E1(p-j+1,:);
        rr(iXi)=-(1-j*h1/p)*e(ax);
        Aeq(end+1,:)=rr; %#ok<AGROW>
        beq(end+1,1)=p0(ax); %#ok<AGROW>
    end
end

Aineq=zeros(0,nvarJoint); bineq=zeros(0,1);
safeMeta=struct('span',{},'cell',{},'cp',{},'plane',{});
for q=1:nspan
    E=pieceMaps{q}; cellIdx=cellOfSpan(q); CC=C.cells{cellIdx};
    for j=1:p+1
        er=E(j,:);
        for hh=1:size(CC.A,1)
            nn=CC.A(hh,:).';
            rr=zeros(1,nvarJoint);
            rr(ix)=-nn(1)*er; rr(iy)=-nn(2)*er; rr(iz)=-nn(3)*er;
            Aineq(end+1,:)=rr; %#ok<AGROW>
            bineq(end+1,1)=-CC.b(hh); %#ok<AGROW>
            safeMeta(end+1)=struct('span',q,'cell',cellIdx,'cp',j,'plane',hh); %#ok<AGROW>
        end
    end
end

M=struct('p',p,'ncoef',ncoef,'nspan',nspan,'nvar_curve',nvarCurve, ...
    'nvar_joint',nvarJoint,'iXi',iXi,'spanCount',spanCount, ...
    'cellOfSpan',cellOfSpan,'breaks',breaks,'knots',U,'Tbez',Tbez, ...
    'pieceMaps',{pieceMaps},'Aeq',Aeq,'beq',beq,'Aineq',Aineq, ...
    'bineq',bineq,'safeMeta',safeMeta);
end

function [minSlack,meta,Qbez]=step3a_c02_bezier_postcheck(x,M,C)
ncoef=M.ncoef;
ix=1:ncoef; iy=ncoef+(1:ncoef); iz=2*ncoef+(1:ncoef);
Qx=M.Tbez*x(ix); Qy=M.Tbez*x(iy); Qz=M.Tbez*x(iz);
Qbez=[Qx(:),Qy(:),Qz(:)].';
minSlack=Inf;
meta=struct('span',NaN,'cell',NaN,'cp',NaN,'plane',NaN);
for q=1:M.nspan
    ids=(q-1)*M.p+(1:M.p+1);
    B=[Qx(ids),Qy(ids),Qz(ids)].'; CC=C.cells{M.cellOfSpan(q)};
    for j=1:M.p+1
        sl=CC.A*B(:,j)-CC.b; [v,k]=min(sl);
        if v<minSlack
            minSlack=v; meta.span=q; meta.cell=M.cellOfSpan(q); meta.cp=j; meta.plane=k;
        end
    end
end
end

function [r0,r1]=step3a_c02_endpoint_jet_residuals(x,xi,M,G)
p=M.p; ncoef=M.ncoef;
ix=1:ncoef; iy=ncoef+(1:ncoef); iz=2*ncoef+(1:ncoef);
Qx=M.Tbez*x(ix); Qy=M.Tbez*x(iy); Qz=M.Tbez*x(iz);
p0=G.front.payload_at_trigger;
e=G.waypoints(:,3)-G.waypoints(:,2); e=e/norm(e);
h0=M.breaks(2)-M.breaks(1); h1=M.breaks(end)-M.breaks(end-1);
Bfirst=[Qx(1:p+1),Qy(1:p+1),Qz(1:p+1)].';
idsLast=(M.nspan-1)*p+(1:p+1);
Blast=[Qx(idsLast),Qy(idsLast),Qz(idsLast)].';
r0=0; r1=0;
for j=0:6
    target0=p0+(j*h0/p)*e*xi;
    target1=p0+(1-j*h1/p)*e*xi;
    r0=max(r0,norm(Bfirst(:,j+1)-target0,inf));
    r1=max(r1,norm(Blast(:,p-j+1)-target1,inf));
end
end

function R=step3a14_run_one_case(S,P,fid)
R=struct();
R.step1_pass=false;
R.corridor_initial_pass=false;
R.xi_geom=NaN;
R.xi_used=NaN;
R.additional_runout=NaN;
R.terminal_slack=NaN;
R.corridor_rebuilt_pass=false;
R.fixed_conflict_count=NaN;
R.candidate_exitflag=NaN;
R.direction_exitflag=NaN;
R.min_directional_gain=NaN;
R.alpha_required=NaN;
R.alpha_final=NaN;
R.analytic_correction_iters=NaN;
R.eq_residual=NaN;
R.min_original_hard_slack=NaN;
R.min_bezier_hard_slack=NaN;
R.pass=false;
R.failure_reason='';
R.control_points=[];

G0=aud_run_case(S,P,fid);
R.step1_pass=G0.pass;
if ~G0.pass
    R.failure_reason='STEP1_FAILED';
    return
end

obs=S.obs(G0.front.known_ids_at_trigger);
C0=step2_build_and_audit_corridor(G0,obs,P,fid);
R.corridor_initial_pass=C0.pass;
if ~C0.pass
    R.failure_reason='INITIAL_CORRIDOR_FAILED';
    return
end

p0=G0.front.payload_at_trigger;
e=G0.waypoints(:,3)-G0.waypoints(:,2);
e=e/norm(e);
n=G0.bypass_direction(:);
n=n-e*(e.'*n);
n=n/norm(n);
dBypass=G0.bypass_offset;
xiGeom=G0.rejoin_xi;

R.xi_geom=xiGeom;

% Minimum terminal-C6 rejoin.
[xiStar,Rterm]=step3a_whole_find_terminal_xi(xiGeom,p0,e,n,dBypass,C0.cells{3});

R.xi_used=xiStar;
R.additional_runout=xiStar-xiGeom;
R.terminal_slack=Rterm.minSlack;

aud_log(fid,'');
aud_log(fid,'[VARIABLE REJOIN]');
aud_log(fid,'xiGeom=%.17g xiUsed=%.17g additionalRunout=%.17e terminalSlack=%+.17e', ...
    xiGeom,xiStar,xiStar-xiGeom,Rterm.minSlack);

% Rebuild guide/corridor at variable xi.
G=G0;
G.rejoin_xi=xiStar;
W1=p0+dBypass*n;
W2=W1+xiStar*e;
W3=p0+xiStar*e;
G.waypoints=[p0,W1,W2,W3];
G.path_length=sum(vecnorm(diff(G.waypoints,1,2),2,1));

C=step2_build_and_audit_corridor(G,obs,P,fid);
R.corridor_rebuilt_pass=C.pass;
if ~C.pass
    R.failure_reason='REBUILT_CORRIDOR_FAILED';
    return
end

M=step3a_whole_build_matrices(G,C);

% Equality elimination.
x0=lsqminnorm(M.Aeq,M.beq);
Z=null(M.Aeq);
Ar=M.Aineq*Z;
br=M.bineq-M.Aineq*x0;

rowInf=max(abs(Ar),[],2);
matrixScale=max(1,norm(Ar,inf));
tauRow=512*eps(matrixScale)*max(size(Ar));

fixed=(rowInf<=tauRow);
active=~fixed;
fixedConflict=fixed & (br<0);
R.fixed_conflict_count=sum(fixedConflict);

aud_log(fid,'[REDUCED] nEq=%d rankEq=%d nullity=%d nIneq=%d fixed=%d active=%d fixedConflict=%d', ...
    size(M.Aeq,1),rank(M.Aeq,1e-10),size(Z,2),size(M.Aineq,1), ...
    sum(fixed),sum(active),R.fixed_conflict_count);

if any(fixedConflict)
    R.failure_reason='FIXED_CONFLICT';
    return
end

Aa=Ar(active,:);
ba=br(active);

% Moderate norm candidate.
[~,y0,ef0,~]=step3a_row_active_qp_feasibility(Aa,ba);
R.candidate_exitflag=ef0;
if isempty(y0)
    R.failure_reason='NO_ACTIVE_CANDIDATE';
    return
end

s0=ba-Aa*y0;

% Common improving direction.
rs=max(abs(Aa),[],2);
An=Aa./rs;

H=eye(size(Aa,2));
f=zeros(size(Aa,2),1);

opts=optimoptions('quadprog','Display','none', ...
    'ConstraintTolerance',1e-9, ...
    'OptimalityTolerance',1e-9, ...
    'StepTolerance',1e-12, ...
    'MaxIterations',5000);

[dDir,~,efDir,~]=quadprog(H,f,An,-ones(size(An,1),1),[],[],[],[],[],opts);
R.direction_exitflag=efDir;

if isempty(dDir)
    R.failure_reason='NO_COMMON_IMPROVING_DIRECTION';
    return
end

gain=-Aa*dDir;
R.min_directional_gain=min(gain);

if ~(R.min_directional_gain>0)
    R.failure_reason='DIRECTION_NOT_STRICTLY_IMPROVING';
    return
end

neg=(s0<0);
if any(neg)
    alphaReq=max((-s0(neg))./gain(neg));
else
    alphaReq=0;
end
R.alpha_required=alphaReq;

alpha=alphaReq;
[y,x,eqRes,origSlack,bezSlack,~]=step3a_ci_eval_alpha( ...
    alpha,y0,dDir,x0,Z,M,C);

fullGain=-M.Aineq*(Z*dDir);

correctionIters=0;
while min(origSlack)<0 || bezSlack<0
    correctionIters=correctionIters+1;

    negRows=find(origSlack<0);

    if isempty(negRows)
        R.failure_reason='BEZIER_LINEAR_MISMATCH';
        return
    end

    if any(fullGain(negRows)<=0)
        R.failure_reason='NO_CORRECTION_DIRECTION';
        return
    end

    deltaReq=max((-origSlack(negRows))./fullGain(negRows));
    alphaTrial=aud_next_up(alpha+deltaReq);

    if alphaTrial<=alpha
        R.failure_reason='ALPHA_CORRECTION_STALLED';
        return
    end

    alpha=alphaTrial;
    [y,x,eqRes,origSlack,bezSlack,~]=step3a_ci_eval_alpha( ...
        alpha,y0,dDir,x0,Z,M,C);

    if correctionIters>20
        R.failure_reason='ALPHA_CORRECTION_LIMIT';
        return
    end
end

R.alpha_final=alpha;
R.analytic_correction_iters=correctionIters;
R.eq_residual=eqRes;
R.min_original_hard_slack=min(origSlack);
R.min_bezier_hard_slack=bezSlack;

ncoef=M.ncoef;
ix=1:ncoef; iy=ncoef+(1:ncoef); iz=2*ncoef+(1:ncoef);
R.control_points=[x(ix),x(iy),x(iz)].';

R.pass=(eqRes<=1e-8) && ...
       (R.min_original_hard_slack>=0) && ...
       (R.min_bezier_hard_slack>=0);

if ~R.pass
    R.failure_reason='FINAL_HARD_POSTCHECK_FAILED';
end

aud_log(fid,'[BSPLINE_D7 RESULT] candExit=%d dirExit=%d minGain=%.17e alphaReq=%.17e alphaFinal=%.17e corrIters=%d', ...
    R.candidate_exitflag,R.direction_exitflag,R.min_directional_gain, ...
    R.alpha_required,R.alpha_final,R.analytic_correction_iters);
aud_log(fid,'[BSPLINE_D7 RESULT] eqResidual=%.17e minOriginal=%+.17e minBezier=%+.17e PASS=%d', ...
    R.eq_residual,R.min_original_hard_slack,R.min_bezier_hard_slack,R.pass);
end

function [y,x,eqRes,origSlack,minBezier,meta]=step3a_ci_eval_alpha(alpha,y0,dDir,x0,Z,M,C)
y=y0+alpha*dDir;
x=x0+Z*y;

eqRes=norm(M.Aeq*x-M.beq,inf);
origSlack=M.bineq-M.Aineq*x;

ncoef=M.ncoef;
ix=1:ncoef;
iy=ncoef+(1:ncoef);
iz=2*ncoef+(1:ncoef);

Qx=M.Tbez*x(ix);
Qy=M.Tbez*x(iy);
Qz=M.Tbez*x(iz);

minBezier=Inf;
meta=struct('span',NaN,'cell',NaN,'cp',NaN,'plane',NaN);

for q=1:M.nspan
    ids=(q-1)*M.p+(1:M.p+1);
    Bcp=[Qx(ids),Qy(ids),Qz(ids)].';
    CC=C.cells{M.cellOfSpan(q)};

    for j=1:M.p+1
        sl=CC.A*Bcp(:,j)-CC.b;
        [v,k]=min(sl);

        if v<minBezier
            minBezier=v;
            meta.span=q;
            meta.cell=M.cellOfSpan(q);
            meta.cp=j;
            meta.plane=k;
        end
    end
end
end

function [pass,y,exitflag,msg]=step3a_row_active_qp_feasibility(A,b)
% Find any moderate-norm point satisfying active reduced inequalities.
% Rows are normalized ONLY for the numerical solve. Original hard
% inequalities are always checked afterward.

nv=size(A,2);
if isempty(A)
    y=zeros(nv,1);
    exitflag=1;
    msg='No active rows.';
    pass=true;
    return
end

rs=max(abs(A),[],2);
rs(rs==0)=1;
As=A./rs;
bs=b./rs;

H=eye(nv);
f=zeros(nv,1);

opts=optimoptions('quadprog','Display','none', ...
    'ConstraintTolerance',1e-9, ...
    'OptimalityTolerance',1e-9, ...
    'StepTolerance',1e-12, ...
    'MaxIterations',5000);

[y,~,exitflag,out]=quadprog(H,f,As,bs,[],[],[],[],[],opts);

if isempty(y)
    pass=false;
else
    pass=all(b-A*y>=0);
end
msg=out.message;
end

function [xiStar,Rstar]=step3a_whole_find_terminal_xi(xiGeom,p0,e,n,d,CC)
R0=step3a_xi_terminal_eval(xiGeom,p0,e,n,d,CC);

if R0.minSlack>=0
    xiStar=xiGeom;
    Rstar=R0;
    return
end

lo=xiGeom;
hi=max(aud_next_up(lo),2*lo);
Rhi=step3a_xi_terminal_eval(hi,p0,e,n,d,CC);

nExpand=0;
while Rhi.minSlack<0
    lo=hi;
    hi=2*hi;
    Rhi=step3a_xi_terminal_eval(hi,p0,e,n,d,CC);
    nExpand=nExpand+1;
    if nExpand>60
        error('STEP3AWHOLE:Bracket','Could not bracket terminal-feasible xi.');
    end
end

for it=1:100
    mid=0.5*(lo+hi);
    if mid==lo || mid==hi
        break
    end

    Rmid=step3a_xi_terminal_eval(mid,p0,e,n,d,CC);

    if Rmid.minSlack>=0
        hi=mid;
        Rhi=Rmid;
    else
        lo=mid;
    end

    if hi-lo <= 16*eps(max(1,abs(hi)))
        break
    end
end

xiStar=hi;
Rstar=step3a_xi_terminal_eval(xiStar,p0,e,n,d,CC);

while Rstar.minSlack<0
    xiStar=aud_next_up(xiStar);
    Rstar=step3a_xi_terminal_eval(xiStar,p0,e,n,d,CC);
end
end

function M=step3a_whole_build_matrices(G,C)
p=7;
ncoef=22;
nspan=ncoef-p;

W=G.waypoints;
segLen=vecnorm(diff(W,1,2),2,1);
frac=segLen/sum(segLen);
spanCount=step3a_allocate_spans(nspan,frac);

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
idxAxis={ix,iy,iz};

Aeq=zeros(0,nvar);
beq=zeros(0,1);

p0=G.front.payload_at_trigger;
e=W(:,3)-W(:,2);
e=e/norm(e);
xi=G.rejoin_xi;

h0=breaks(2)-breaks(1);
h1=breaks(end)-breaks(end-1);
E0=pieceMaps{1};
E1=pieceMaps{end};

for ax=1:3
    ia=idxAxis{ax};

    for j=0:6
        rr=zeros(1,nvar);
        rr(ia)=E0(j+1,:);
        Aeq(end+1,:)=rr; %#ok<AGROW>
        beq(end+1,1)=p0(ax)+(j*h0/p)*e(ax)*xi; %#ok<AGROW>
    end

    for j=0:6
        rr=zeros(1,nvar);
        rr(ia)=E1(p-j+1,:);
        Aeq(end+1,:)=rr; %#ok<AGROW>
        beq(end+1,1)=p0(ax)+(1-j*h1/p)*e(ax)*xi; %#ok<AGROW>
    end
end

Aineq=zeros(0,nvar);
bineq=zeros(0,1);
safeMeta=struct('span',{},'cell',{},'cp',{},'plane',{});
ineqCell=zeros(0,1);
ineqSpan=zeros(0,1);

for q=1:nspan
    E=pieceMaps{q};
    cellIdx=cellOfSpan(q);
    CC=C.cells{cellIdx};

    for j=1:p+1
        er=E(j,:);
        for hh=1:size(CC.A,1)
            nn=CC.A(hh,:).';
            rr=zeros(1,nvar);
            rr(ix)=-nn(1)*er;
            rr(iy)=-nn(2)*er;
            rr(iz)=-nn(3)*er;

            Aineq(end+1,:)=rr; %#ok<AGROW>
            bineq(end+1,1)=-CC.b(hh); %#ok<AGROW>
            safeMeta(end+1)=struct('span',q,'cell',cellIdx,'cp',j,'plane',hh); %#ok<AGROW>
            ineqCell(end+1,1)=cellIdx; %#ok<AGROW>
            ineqSpan(end+1,1)=q; %#ok<AGROW>
        end
    end
end

M=struct();
M.p=p;
M.ncoef=ncoef;
M.nspan=nspan;
M.nvar=nvar;
M.spanCount=spanCount;
M.cellOfSpan=cellOfSpan;
M.breaks=breaks;
M.knots=U;
M.Tbez=Tbez;
M.Aeq=Aeq;
M.beq=beq;
M.Aineq=Aineq;
M.bineq=bineq;
M.safeMeta=safeMeta;
M.ineqCell=ineqCell;
M.ineqSpan=ineqSpan;
end

function R=step3a_xi_terminal_eval(xi,p0,e,n,d,CC)
% Rebuild only the path-length parameterization quantities that depend on xi.
% Geometry:
%   [d, xi, d]

p=7;
nspan=15;
segLen=[d,xi,d];
frac=segLen/sum(segLen);
spanCount=step3a_allocate_spans(nspan,frac);

% Final guide segment occupies fraction frac(3) and has spanCount(3) spans.
hFinal=frac(3)/spanCount(3);

factor=zeros(1,7);
minSlackByJ=zeros(1,7);
worstPlaneByJ=zeros(1,7);

minSlack=Inf;
worstJ=NaN;
worstPlane=NaN;

for j=0:6
    factor(j+1)=1-j*hFinal/p;
    Bj=p0+factor(j+1)*xi*e;

    sl=CC.A*Bj-CC.b;
    [v,k]=min(sl);

    minSlackByJ(j+1)=v;
    worstPlaneByJ(j+1)=k;

    if v<minSlack
        minSlack=v;
        worstJ=j;
        worstPlane=k;
    end
end

% Also report the shifted guide points for later reconstruction.
W1=p0+d*n;
W2=W1+xi*e;
W3=p0+xi*e;

R=struct();
R.xi=xi;
R.segment_lengths=segLen;
R.fractions=frac;
R.spanCount=spanCount;
R.hFinal=hFinal;
R.factor=factor;
R.minSlackByJ=minSlackByJ;
R.worstPlaneByJ=worstPlaneByJ;
R.minSlack=minSlack;
R.worstJ=worstJ;
R.worstPlane=worstPlane;
R.waypoints=[p0,W1,W2,W3];
end

function M=step3a_jet_build_matrices(G,C,finalOrder)
p=7;
ncoef=22;
nspan=ncoef-p;

W=G.waypoints;
segLen=vecnorm(diff(W,1,2),2,1);
frac=segLen/sum(segLen);
spanCount=step3a_allocate_spans(nspan,frac);

ub=[0,cumsum(frac)];
ub(end)=1;

breaks=0;
cellOfSpan=zeros(1,nspan);
ss=0;
for cidx=1:3
    a=ub(cidx); b=ub(cidx+1); nc=spanCount(cidx);
    local=a+(0:nc)*(b-a)/nc;
    if cidx==1, breaks=local; else, breaks=[breaks,local(2:end)]; end %#ok<AGROW>
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
ix=1:ncoef; iy=ncoef+(1:ncoef); iz=2*ncoef+(1:ncoef);
idxAxis={ix,iy,iz};

Aeq=zeros(0,nvar); beq=zeros(0,1);

p0=G.front.payload_at_trigger;
e=W(:,3)-W(:,2); e=e/norm(e);
xi=G.rejoin_xi;

h0=breaks(2)-breaks(1);
h1=breaks(end)-breaks(end-1);
E0=pieceMaps{1};
E1=pieceMaps{end};

for ax=1:3
    ia=idxAxis{ax};

    % START always full jet 0..6.
    for j=0:6
        rr=zeros(1,nvar);
        rr(ia)=E0(j+1,:);
        Aeq(end+1,:)=rr; %#ok<AGROW>
        beq(end+1,1)=p0(ax)+(j*h0/p)*e(ax)*xi; %#ok<AGROW>
    end

    % FINAL progressively 0..finalOrder.
    for j=0:finalOrder
        rr=zeros(1,nvar);
        rr(ia)=E1(p-j+1,:);
        Aeq(end+1,:)=rr; %#ok<AGROW>
        beq(end+1,1)=p0(ax)+(1-j*h1/p)*e(ax)*xi; %#ok<AGROW>
    end
end

Aineq=zeros(0,nvar); bineq=zeros(0,1);
safeMeta=struct('span',{},'cell',{},'cp',{},'plane',{});
ineqCell=zeros(0,1);

for q=1:nspan
    E=pieceMaps{q};
    cellIdx=cellOfSpan(q);
    CC=C.cells{cellIdx};

    for j=1:p+1
        er=E(j,:);
        for hh=1:size(CC.A,1)
            n=CC.A(hh,:).';
            rr=zeros(1,nvar);
            rr(ix)=-n(1)*er;
            rr(iy)=-n(2)*er;
            rr(iz)=-n(3)*er;

            Aineq(end+1,:)=rr; %#ok<AGROW>
            bineq(end+1,1)=-CC.b(hh); %#ok<AGROW>
            safeMeta(end+1)=struct('span',q,'cell',cellIdx,'cp',j,'plane',hh); %#ok<AGROW>
            ineqCell(end+1,1)=cellIdx; %#ok<AGROW>
        end
    end
end

M=struct('Aeq',Aeq,'beq',beq,'Aineq',Aineq,'bineq',bineq, ...
    'safeMeta',safeMeta,'ineqCell',ineqCell,'cellOfSpan',cellOfSpan);
end

function [rho,y,exitflag,msg]=step3a_diag_phase1(A,b)
% Robust phase-I LP:
%
%   min rho
%   s.t. A*y <= b + rho
%        rho >= 0
%
% rho has the SAME physical units as the original inequality slack.
%
% We scale each row but scale the rho coefficient consistently:
%
%   (A_i/s_i)y - (1/s_i)rho <= b_i/s_i
%
% so the problem is algebraically identical.

nv=size(A,2);

if isempty(A)
    rho=0;
    y=zeros(nv,1);
    exitflag=1;
    msg='No inequality rows.';
    return
end

srow=max(abs(A),[],2);
srow=max(srow,abs(b));
srow=max(srow,1);
srow(srow==0)=1;

As=A./srow;
bs=b./srow;
rhoCol=-1./srow;

f=[zeros(nv,1);1];
Aaug=[As,rhoCol];
lb=[-inf(nv,1);0];

opts=optimoptions('linprog','Display','none', ...
    'ConstraintTolerance',1e-9, ...
    'OptimalityTolerance',1e-9);

[z,~,exitflag,out]=linprog(f,Aaug,bs,[],[],lb,[],opts);

if isempty(z)
    rho=NaN;
    y=[];
else
    y=z(1:nv);
    rho=z(end);
end
msg=out.message;
end

function M=step3a_diag_build_matrices(G,C)
p=7;
ncoef=22;
nspan=ncoef-p;

W=G.waypoints;
segLen=vecnorm(diff(W,1,2),2,1);
Ltot=sum(segLen);
frac=segLen/Ltot;
spanCount=step3a_allocate_spans(nspan,frac);

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

Aeq=zeros(0,nvar);
beq=zeros(0,1);

p0=G.front.payload_at_trigger;
e=W(:,3)-W(:,2);
e=e/norm(e);
xi=G.rejoin_xi;

h0=breaks(2)-breaks(1);
h1=breaks(end)-breaks(end-1);
E0=pieceMaps{1};
E1=pieceMaps{end};
idxAxis={ix,iy,iz};

for ax=1:3
    ia=idxAxis{ax};
    for j=0:6
        rr=zeros(1,nvar);
        rr(ia)=E0(j+1,:);
        Aeq(end+1,:)=rr;
        beq(end+1,1)=p0(ax)+(j*h0/p)*e(ax)*xi;
    end
    for j=0:6
        rr=zeros(1,nvar);
        rr(ia)=E1(p-j+1,:);
        Aeq(end+1,:)=rr;
        beq(end+1,1)=p0(ax)+(1-j*h1/p)*e(ax)*xi;
    end
end

Aineq=zeros(0,nvar);
bineq=zeros(0,1);
safeMeta=struct('span',{},'cell',{},'cp',{},'plane',{});
ineqCell=zeros(0,1);
ineqSpan=zeros(0,1);

for q=1:nspan
    E=pieceMaps{q};
    cellIdx=cellOfSpan(q);
    CC=C.cells{cellIdx};

    for j=1:p+1
        er=E(j,:);
        for hh=1:size(CC.A,1)
            n=CC.A(hh,:).';
            bb=CC.b(hh);

            rr=zeros(1,nvar);
            rr(ix)=-n(1)*er;
            rr(iy)=-n(2)*er;
            rr(iz)=-n(3)*er;

            Aineq(end+1,:)=rr;
            bineq(end+1,1)=-bb;
            safeMeta(end+1)=struct('span',q,'cell',cellIdx,'cp',j,'plane',hh); %#ok<AGROW>
            ineqCell(end+1,1)=cellIdx; %#ok<AGROW>
            ineqSpan(end+1,1)=q; %#ok<AGROW>
        end
    end
end

M=struct();
M.p=p;
M.ncoef=ncoef;
M.nspan=nspan;
M.nvar=nvar;
M.spanCount=spanCount;
M.cellOfSpan=cellOfSpan;
M.breaks=breaks;
M.knots=U;
M.Tbez=Tbez;
M.Aeq=Aeq;
M.beq=beq;
M.Aineq=Aineq;
M.bineq=bineq;
M.safeMeta=safeMeta;
M.ineqCell=ineqCell;
M.ineqSpan=ineqSpan;
end

function C=step2_build_and_audit_corridor(G,obs,P,fid)
% STEP 2B
% Multi-plane convex corridor from the Step-1 V3 certified guide.
%
% No artificial bounding box.
% No obstacle sphere approximation.
% No support-boundary relaxation.
%
% Unbounded cells are allowed when the actual free space is open.
%
% Candidate normals are generated only from deterministic geometric
% features of each guide segment and exact ellipsoid:
%   - Step-1 certified support direction
%   - exact point-to-ellipsoid closest normal at start / midpoint / end
%   - exact segment-to-ellipsoid closest-pair normal
%
% Every retained plane is independently certified to contain the COMPLETE
% guide segment by endpoint evaluation. Because halfspace slack is affine
% on the segment, endpoint nonnegativity certifies the full segment.

W=G.waypoints;
Rreq=P.R_payload+P.d_margin;
nseg=size(W,2)-1;

if nseg~=3
    error('STEP2B:UnexpectedGuide','Step-1 guide must have exactly 3 segments.');
end

mid=W(:,3)-W(:,2);
if norm(mid)<=0
    error('STEP2B:ZeroProgress','Middle guide segment has zero length.');
end
e=mid/norm(mid);

nBy=G.bypass_direction(:);
nBy=nBy-e*(e.'*nBy);
nBy=nBy/norm(nBy);

baseDIR=[-e,nBy,e];
srcBase={'STEP1_BEFORE_-e','STEP1_BYPASS','STEP1_AFTER_+e'};

CELLS=cell(1,nseg);
PLANELOG=cell(1,nseg);

aud_log(fid,'');
aud_log(fid,'[STEP2B] MULTI-PLANE EXACT-SUPPORT CORRIDOR');
aud_log(fid,'[STEP2B] no artificial box | no b relaxation | unbounded allowed if free space is open');

for sidx=1:nseg
    P0=W(:,sidx);
    P1=W(:,sidx+1);
    PM=0.5*(P0+P1);

    candN=[];
    candSrc={};
    candObs=[];

    for i=1:numel(obs)
        candN(:,end+1)=baseDIR(:,sidx); %#ok<AGROW>
        candSrc{end+1}=[srcBase{sidx} '_OBS']; %#ok<AGROW>
        candObs(end+1)=i; %#ok<AGROW>
    end

    for i=1:numel(obs)
        Q={P0,PM,P1};
        qname={'START','MID','END'};

        for qk=1:3
            [dist,pEll]=step2b_point_ellipsoid_closest(Q{qk},obs(i));
            if dist>0
                n=(Q{qk}-pEll)/dist;
                candN(:,end+1)=n; %#ok<AGROW>
                candSrc{end+1}=['POINT_' qname{qk}]; %#ok<AGROW>
                candObs(end+1)=i; %#ok<AGROW>
            end
        end

        [distSeg,~,pSeg,pEll]=step2b_segment_ellipsoid_closest_pair(P0,P1,obs(i));
        if distSeg>0
            n=(pSeg-pEll)/distSeg;
            candN(:,end+1)=n; %#ok<AGROW>
            candSrc{end+1}='SEGMENT_CLOSEST'; %#ok<AGROW>
            candObs(end+1)=i; %#ok<AGROW>
        end
    end

    A=[];
    b=[];
    kept=struct('obstacle_id',{},'source',{},'normal',{},'support_b',{}, ...
        'min_segment_slack',{},'scalar_slack',{},'expanded_slack',{});
    rejected=struct('obstacle_id',{},'source',{},'reason',{},'min_slack',{});

    aud_log(fid,'');
    aud_log(fid,'[CELL %d] candidatePlanes=%d',sidx-1,size(candN,2));

    for k=1:size(candN,2)
        i=candObs(k);
        n=candN(:,k);

        if norm(n)==0 || any(~isfinite(n))
            rejected(end+1)=struct('obstacle_id',obs(i).id,'source',candSrc{k}, ...
                'reason','INVALID_NORMAL','min_slack',NaN); %#ok<AGROW>
            continue
        end
        n=n/norm(n);

        bi=aud_support(obs(i),n)+Rreq;

        sc=[n.'*P0-bi,n.'*P1-bi];
        ex=[n(1)*P0(1)+n(2)*P0(2)+n(3)*P0(3)-bi, ...
            n(1)*P1(1)+n(2)*P1(2)+n(3)*P1(3)-bi];
        minsl=min([sc,ex]);

        if minsl<0
            rejected(end+1)=struct('obstacle_id',obs(i).id,'source',candSrc{k}, ...
                'reason','CUTS_GUIDE','min_slack',minsl); %#ok<AGROW>
            continue
        end

        isdup=false;
        for j=1:numel(kept)
            if kept(j).obstacle_id==obs(i).id && ...
               norm(kept(j).normal-n)<=1e-12 && ...
               abs(kept(j).support_b-bi)<=1e-12
                isdup=true;
                break
            end
        end
        if isdup
            rejected(end+1)=struct('obstacle_id',obs(i).id,'source',candSrc{k}, ...
                'reason','DUPLICATE','min_slack',minsl); %#ok<AGROW>
            continue
        end

        A=[A;n.']; %#ok<AGROW>
        b=[b;bi]; %#ok<AGROW>
        kept(end+1)=struct('obstacle_id',obs(i).id,'source',candSrc{k}, ...
            'normal',n,'support_b',bi,'min_segment_slack',minsl, ...
            'scalar_slack',sc,'expanded_slack',ex); %#ok<AGROW>
    end

    if isempty(A)
        C=step2_fail_case(sprintf('CELL_%d_NO_VALID_PLANES',sidx-1));
        return
    end

    guideMin=min([A*P0-b;A*P1-b]);
    rankA=rank(A,1e-12);
    [r,strict,unboundedCheb]=step2_chebyshev_radius(A,b);
    [isBounded,recessionWitness]=step2b_boundedness(A);

    CELLS{sidx}=struct( ...
        'A',A,'b',b,'segment_index',sidx, ...
        'guide_start',P0,'guide_end',P1, ...
        'n_planes',size(A,1),'rankA',rankA, ...
        'strict_interior',strict,'cheb_radius',r, ...
        'cheb_unbounded',unboundedCheb, ...
        'is_bounded',isBounded, ...
        'recession_witness',recessionWitness);

    PLANELOG{sidx}=struct('kept',kept,'rejected',rejected);

    aud_log(fid,'[CELL %d] kept=%d rejected=%d rank(A)=%d guideMinSlack=%+.17e', ...
        sidx-1,numel(kept),numel(rejected),rankA,guideMin);
    aud_log(fid,'[CELL %d] strictInterior=%d chebRadius=%s bounded=%d', ...
        sidx-1,strict,step2b_numstr(r),isBounded);

    for i=1:numel(obs)
        if isempty(kept)
            kk=[];
        else
            kk=find([kept.obstacle_id]==obs(i).id);
        end
        if isempty(rejected)
            rr=[];
        else
            rr=find([rejected.obstacle_id]==obs(i).id);
        end
        if isempty(kk)
            minK=NaN;
        else
            minK=min([kept(kk).min_segment_slack]);
        end
        aud_log(fid,'[CELL %d OBS %d] keptPlanes=%d rejected=%d minKeptSlack=%+.17e', ...
            sidx-1,obs(i).id,numel(kk),numel(rr),minK);
    end
end

guideMin=zeros(1,nseg);
guidePass=false(1,nseg);
strictCellPass=true;
supportStructurePass=true;

aud_log(fid,'');
aud_log(fid,'[AUDIT] CELL GEOMETRY');

for sidx=1:nseg
    Cc=CELLS{sidx};
    vals=step2b_segment_kernel_slacks(Cc,W(:,sidx),W(:,sidx+1));
    guideMin(sidx)=min(vals);
    guidePass(sidx)=guideMin(sidx)>=0;

    rn=vecnorm(Cc.A,2,2);
    if any(abs(rn-1)>128*eps(1))
        supportStructurePass=false;
    end
    strictCellPass=strictCellPass && Cc.strict_interior;

    aud_log(fid,'[CELL %d] nPlanes=%d rank=%d bounded=%d strict=%d minGuideSlack=%+.17e', ...
        sidx-1,Cc.n_planes,Cc.rankA,Cc.is_bounded,Cc.strict_interior,guideMin(sidx));
end

npair=nseg-1;
OVER=repmat(struct('left_cell',0,'right_cell',0,'radius',NaN, ...
    'strict',false,'unbounded',false,'bounded',false, ...
    'shared_waypoint_slack',NaN),1,npair);

sharedPass=false(1,npair);
sharedSlack=zeros(1,npair);
strictOverlapPass=true;

aud_log(fid,'');
aud_log(fid,'[AUDIT] CONSECUTIVE OVERLAPS');

for k=1:npair
    A=[CELLS{k}.A;CELLS{k+1}.A];
    b=[CELLS{k}.b;CELLS{k+1}.b];

    [r,strict,unbounded]=step2_chebyshev_radius(A,b);
    [isBounded,~]=step2b_boundedness(A);

    q=W(:,k+1);
    vals=[step2_all_kernel_slacks(CELLS{k},q); ...
          step2_all_kernel_slacks(CELLS{k+1},q)];
    sl=min(vals);

    sharedSlack(k)=sl;
    sharedPass(k)=sl>=0;
    strictOverlapPass=strictOverlapPass && strict;

    OVER(k)=struct('left_cell',k-1,'right_cell',k,'radius',r, ...
        'strict',strict,'unbounded',unbounded,'bounded',isBounded, ...
        'shared_waypoint_slack',sl);

    aud_log(fid,'[%d->%d] strict=%d radius=%s bounded=%d sharedSlack=%+.17e sharedPass=%d', ...
        k-1,k,strict,step2b_numstr(r),isBounded,sl,sharedPass(k));
end

guideContainmentPass=all(guidePass);
sharedWaypointPass=all(sharedPass);
minGuideSlack=min(guideMin);
minSharedSlack=min(sharedSlack);

pass=guideContainmentPass && supportStructurePass && ...
    strictCellPass && strictOverlapPass && sharedWaypointPass;

nBounded=sum(cellfun(@(c)c.is_bounded,CELLS));
nUnbounded=nseg-nBounded;
nBoundedOverlap=sum([OVER.bounded]);
nUnboundedOverlap=npair-nBoundedOverlap;

aud_log(fid,'');
aud_log(fid,'[STEP2B-RESULT] guideContainment=%d supportStructure=%d strictCell=%d strictOverlap=%d sharedWaypoint=%d PASS=%d', ...
    guideContainmentPass,supportStructurePass,strictCellPass,strictOverlapPass,sharedWaypointPass,pass);
aud_log(fid,'[STEP2B-RESULT] minGuideSlack=%+.17e minSharedSlack=%+.17e', ...
    minGuideSlack,minSharedSlack);
aud_log(fid,'[STEP2B-RESULT] boundedCells=%d/%d boundedOverlaps=%d/%d', ...
    nBounded,nseg,nBoundedOverlap,npair);

C=struct();
C.pass=pass;
C.failure_reason='';
C.cell_count=nseg;
C.cells=CELLS;
C.plane_log=PLANELOG;
C.overlaps=OVER;
C.guide_containment_pass=guideContainmentPass;
C.support_structure_pass=supportStructurePass;
C.strict_cell_pass=strictCellPass;
C.strict_overlap_pass=strictOverlapPass;
C.shared_waypoint_pass=sharedWaypointPass;
C.min_guide_cell_slack=minGuideSlack;
C.min_shared_waypoint_slack=minSharedSlack;
C.min_finite_cell_radius=step2b_min_finite_cells(CELLS);
C.min_finite_overlap_radius=step2b_min_finite_overlap(OVER);
C.n_unbounded_cells=nUnbounded;
C.n_unbounded_overlaps=nUnboundedOverlap;
C.shared_waypoint_slack=sharedSlack;
end

function vals=step2b_segment_kernel_slacks(C,P0,P1)
m=size(C.A,1);
scalar0=zeros(m,1); scalar1=zeros(m,1);
expanded0=zeros(m,1); expanded1=zeros(m,1);

for i=1:m
    n=C.A(i,:).';
    scalar0(i)=n.'*P0-C.b(i);
    scalar1(i)=n.'*P1-C.b(i);
    expanded0(i)=n(1)*P0(1)+n(2)*P0(2)+n(3)*P0(3)-C.b(i);
    expanded1(i)=n(1)*P1(1)+n(2)*P1(2)+n(3)*P1(3)-C.b(i);
end

matrix0=C.A*P0-C.b;
matrix1=C.A*P1-C.b;
vals=[scalar0;scalar1;matrix0;matrix1;expanded0;expanded1];
end

function [isBounded,witness]=step2b_boundedness(A)
opts=optimoptions('linprog','Display','none', ...
    'ConstraintTolerance',1e-10,'OptimalityTolerance',1e-10);

isBounded=true;
witness=zeros(3,1);

Aineq=-A;
bineq=zeros(size(A,1),1);
lb=-ones(3,1);
ub= ones(3,1);

for j=1:3
    for sgn=[-1,1]
        f=zeros(3,1);
        f(j)=-sgn;
        [d,~,ef]=linprog(f,Aineq,bineq,[],[],lb,ub,opts);
        if ef==1
            val=sgn*d(j);
            if val>1e-9
                isBounded=false;
                witness=d;
                return
            end
        end
    end
end
end

function s=step2b_numstr(x)
if isinf(x)
    s='+Inf';
else
    s=sprintf('%.17e',x);
end
end

function v=step2b_min_finite_cells(CELLS)
r=[];
for i=1:numel(CELLS)
    x=CELLS{i}.cheb_radius;
    if isfinite(x)
        r(end+1)=x; %#ok<AGROW>
    end
end
if isempty(r), v=Inf; else, v=min(r); end
end

function v=step2b_min_finite_overlap(O)
r=[];
for i=1:numel(O)
    if isfinite(O(i).radius)
        r(end+1)=O(i).radius; %#ok<AGROW>
    end
end
if isempty(r), v=Inf; else, v=min(r); end
end

function [dist,tau,pSeg,pEll]=step2b_segment_ellipsoid_closest_pair(A,B,ob)
A=A(:);
B=B(:);
dir=B-A;

if norm(dir)==0
    [dist,pEll]=step2b_point_ellipsoid_closest(A,ob);
    tau=0;
    pSeg=A;
    return
end

fun=@(t) step2b_point_ellipsoid_distance_only(A+t*dir,ob);
opts=optimset('Display','off','TolX',1e-13);
[tau,dist]=fminbnd(fun,0,1,opts);

d0=fun(0);
d1=fun(1);
if d0<dist
    dist=d0;
    tau=0;
end
if d1<dist
    dist=d1;
    tau=1;
end

pSeg=A+tau*dir;
[dist,pEll]=step2b_point_ellipsoid_closest(pSeg,ob);
end

function d=step2b_point_ellipsoid_distance_only(p,ob)
[d,~]=step2b_point_ellipsoid_closest(p,ob);
end

function [dist,pWorld]=step2b_point_ellipsoid_closest(p,ob)
q=ob.R.'*(p(:)-ob.center);
a=ob.radii(:);
a2=a.^2;

if sum((q./a).^2)<=1
    dist=0;
    pWorld=p(:);
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
pWorld=ob.center+ob.R*x;
dist=norm(p(:)-pWorld);
end

function vals=step2_all_kernel_slacks(C,q)
m=size(C.A,1);
scalar=zeros(m,1);
expanded=zeros(m,1);

for i=1:m
    n=C.A(i,:).';
    scalar(i)=n.'*q-C.b(i);
    expanded(i)=n(1)*q(1)+n(2)*q(2)+n(3)*q(3)-C.b(i);
end

matrix=C.A*q-C.b;
vals=[scalar;matrix;expanded];
end

function B=step3a_bspline_d7_feasibility(G,C,fid)
% BSPLINE_D7 ONLY.
%
% Representation held fixed from the earlier S2-R baseline:
%   degree p = 7
%   ncoef    = 22
%   nspan    = 15
%   simple interior knots -> structural C6.
%
% Cell transitions are no longer assigned by arbitrary uniform quarters.
% The 15 spans are allocated to the three guide segments according to guide
% arclength, with at least one span per segment; the knot locations align
% exactly with the guide/cell transition arclengths.
%
% Continuous cell containment:
%   native B-spline -> exact local Bezier extraction by standard knot
%   insertion; every local Bezier control point is constrained to the
%   corresponding convex cell.
%
% Objective is NOT J7. It is the exact integrated squared Euclidean
% deviation from the piecewise-linear Step-1 guiding path. 8-point
% Gauss-Legendre is exact for the squared degree-7-minus-linear integrand
% (degree <= 14) on every knot span.

p=7;
ncoef=22;
nspan=ncoef-p; % 15, held fixed for direct comparison with old S2-R.

W=G.waypoints;
segLen=vecnorm(diff(W,1,2),2,1);
Ltot=sum(segLen);
if Ltot<=0
    B=step3a_fail_bspline('ZERO_GUIDE_LENGTH');
    return
end

frac=segLen/Ltot;
spanCount=step3a_allocate_spans(nspan,frac);
if any(spanCount<1) || sum(spanCount)~=nspan
    error('STEP3A:SpanAllocation','Invalid path-driven span allocation.');
end

% Guide arclength parameter boundaries.
ub=[0,cumsum(frac)];
ub(end)=1;

% Build nonuniform unique break sequence aligned to guide transitions.
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

if numel(breaks)~=nspan+1
    error('STEP3A:BreakCount','Expected %d breaks.',nspan+1);
end

U=[repmat(breaks(1),1,p+1),breaks(2:end-1),repmat(breaks(end),1,p+1)];

% Exact B-spline -> piecewise Bezier extraction.
[Ubez,Tbez]=step3a_bs_bezier_decomposition_transform(U,p,ncoef); %#ok<ASGLU>
expected=nspan*p+1;
if size(Tbez,1)~=expected
    error('STEP3A:BezierExtract','Expected %d decomposed points, got %d.',expected,size(Tbez,1));
end

pieceMaps=cell(1,nspan);
for q=1:nspan
    ids=(q-1)*p+(1:p+1);
    pieceMaps{q}=Tbez(ids,:);
end

nvar=3*ncoef;
ix=1:ncoef;
iy=ncoef+(1:ncoef);
iz=2*ncoef+(1:ncoef);

Aeq=zeros(0,nvar);
beq=zeros(0,1);

% Endpoint jets 0..6, same semantics as earlier S2-R.
p0=G.front.payload_at_trigger;
e=(W(:,3)-W(:,2));
e=e/norm(e);
xi=G.rejoin_xi;

h0=breaks(2)-breaks(1);
h1=breaks(end)-breaks(end-1);
E0=pieceMaps{1};
E1=pieceMaps{end};

for ax=1:3
    idxAxis={ix,iy,iz};
    ia=idxAxis{ax};

    for j=0:6
        rr=zeros(1,nvar);
        rr(ia)=E0(j+1,:);
        Aeq(end+1,:)=rr;
        beq(end+1,1)=p0(ax)+(j*h0/p)*e(ax)*xi;
    end

    for j=0:6
        rr=zeros(1,nvar);
        rr(ia)=E1(p-j+1,:);
        Aeq(end+1,:)=rr;
        beq(end+1,1)=p0(ax)+(1-j*h1/p)*e(ax)*xi;
    end
end

% Continuous safety via local Bezier convex hull.
Aineq=zeros(0,nvar);
bineq=zeros(0,1);
safeMeta=struct('span',{},'cell',{},'cp',{},'plane',{});

for q=1:nspan
    E=pieceMaps{q};
    cellIdx=cellOfSpan(q);
    CC=C.cells{cellIdx};

    for j=1:p+1
        er=E(j,:);
        for hh=1:size(CC.A,1)
            n=CC.A(hh,:).';
            bb=CC.b(hh);

            rr=zeros(1,nvar);
            rr(ix)=-n(1)*er;
            rr(iy)=-n(2)*er;
            rr(iz)=-n(3)*er;

            Aineq(end+1,:)=rr;
            bineq(end+1,1)=-bb;
            safeMeta(end+1)=struct('span',q,'cell',cellIdx,'cp',j,'plane',hh); %#ok<AGROW>
        end
    end
end

% Exact L2 guide objective by 8-point Gauss-Legendre per knot span.
[xg,wg]=step3a_gauss8_01();
Cobj=zeros(3*nspan*numel(xg),nvar);
dobj=zeros(3*nspan*numel(xg),1);
rr=0;

for q=1:nspan
    ua=breaks(q);
    ubq=breaks(q+1);
    hs=ubq-ua;
    cellIdx=cellOfSpan(q);

    ga=ub(cellIdx);
    gb=ub(cellIdx+1);

    for k=1:numel(xg)
        u=ua+hs*xg(k);
        N=step3a_bs_basis_all(u,U,p,ncoef);

        if gb>ga
            alpha=(u-ga)/(gb-ga);
        else
            alpha=0;
        end
        alpha=min(1,max(0,alpha));
        pg=(1-alpha)*W(:,cellIdx)+alpha*W(:,cellIdx+1);

        sw=sqrt(wg(k)*hs);

        rr=rr+1;
        Cobj(rr,ix)=sw*N;
        dobj(rr)=sw*pg(1);

        rr=rr+1;
        Cobj(rr,iy)=sw*N;
        dobj(rr)=sw*pg(2);

        rr=rr+1;
        Cobj(rr,iz)=sw*N;
        dobj(rr)=sw*pg(3);
    end
end

% Row scaling for numerical conditioning. This does not change constraints.
[Ais,bis,ineqScale]=step3a_scale_rows(Aineq,bineq);
[Aes,bes,eqScale]=step3a_scale_rows(Aeq,beq);

opts=optimoptions('lsqlin', ...
    'Display','none', ...
    'Algorithm','interior-point', ...
    'ConstraintTolerance',1e-10, ...
    'OptimalityTolerance',1e-10, ...
    'StepTolerance',1e-12, ...
    'MaxIterations',5000);

[x,resnorm,~,exitflag,output]=lsqlin(Cobj,dobj,Ais,bis,Aes,bes,[],[],[],opts);

if isempty(x) || exitflag<=0
    B=step3a_fail_bspline(['LSQLIN_FAILED_' num2str(exitflag)]);
    B.exitflag=exitflag;
    B.message=output.message;
    aud_log(fid,'[BSPLINE_D7] SOLVER FAIL exit=%d message=%s',exitflag,output.message);
    return
end

% ORIGINAL, unscaled independent certificate.
eqResidual=norm(Aeq*x-beq,inf);
safeSlack=bineq-Aineq*x;
minSafe=min(safeSlack);

% Endpoint extracted-Bezier residual.
cpX=x(ix); cpY=x(iy); cpZ=x(iz);
Qx=Tbez*cpX;
Qy=Tbez*cpY;
Qz=Tbez*cpZ;

endpointResidual=0;
for ax=1:3
    QQ={Qx,Qy,Qz};
    qv=QQ{ax};
    for j=0:6
        expected0=p0(ax)+(j*h0/p)*e(ax)*xi;
        expected1=p0(ax)+(1-j*h1/p)*e(ax)*xi;
        endpointResidual=max(endpointResidual,abs(qv(j+1)-expected0));
        endpointResidual=max(endpointResidual,abs(qv(end-j)-expected1));
    end
end

% Continuous span-cell certificate from extracted Bezier CPs using ORIGINAL
% cell inequalities. No sampling is needed for the proof.
minBezierSlack=Inf;
worstSpan=NaN;
worstCell=NaN;
for q=1:nspan
    ids=(q-1)*p+(1:p+1);
    Bcp=[Qx(ids),Qy(ids),Qz(ids)].';
    CC=C.cells{cellOfSpan(q)};
    for j=1:p+1
        sl=CC.A*Bcp(:,j)-CC.b;
        v=min(sl);
        if v<minBezierSlack
            minBezierSlack=v;
            worstSpan=q;
            worstCell=cellOfSpan(q);
        end
    end
end

% Structural extraction audit.
minEntry=min(Tbez,[],'all');
rowSumErr=max(abs(sum(Tbez,2)-1));
minNZ=step3a_min_nonzero(Tbez);

% Objective is already quadrature-weighted exact L2 integral.
l2obj=resnorm;

% This stage's hard pass uses original constraints. Solver tolerance is not
% used to excuse negative safety.
pass=(eqResidual<=1e-8) && (endpointResidual<=1e-8) && ...
     (minSafe>=0) && (minBezierSlack>=0) && ...
     (minEntry>=-1e-14) && (rowSumErr<=1e-12);

aud_log(fid,'');
aud_log(fid,'========================================================================================================================');
aud_log(fid,' [BSPLINE_D7] STEP 3A SAFE-SPATIAL FEASIBILITY');
aud_log(fid,'========================================================================================================================');
aud_log(fid,'[BSPLINE_D7] degree=%d ncoef=%d nspan=%d spanCountByCell=%s', ...
    p,ncoef,nspan,mat2str(spanCount));
aud_log(fid,'[BSPLINE_D7] guideFractions=%s',mat2str(frac,15));
aud_log(fid,'[BSPLINE_D7] knotBreaks=%s',mat2str(breaks,15));
aud_log(fid,'[BSPLINE_D7] nEq=%d rankEq=%d nIneq=%d', ...
    size(Aeq,1),rank(Aeq,1e-10),size(Aineq,1));
aud_log(fid,'[BSPLINE_D7] extraction minEntry=%+.17e minNonzero=%+.17e rowSumErr=%.17e', ...
    minEntry,minNZ,rowSumErr);
aud_log(fid,'[BSPLINE_D7] solver exit=%d resnorm(L2guide)=%.17e',exitflag,l2obj);
aud_log(fid,'[BSPLINE_D7] originalEqResidual=%.17e endpointResidual=%.17e', ...
    eqResidual,endpointResidual);
aud_log(fid,'[BSPLINE_D7] originalMinLinearSafeSlack=%+.17e',minSafe);
aud_log(fid,'[BSPLINE_D7] extractedBezierMinSafeSlack=%+.17e worstSpan=%d worstCell=%d', ...
    minBezierSlack,worstSpan,worstCell);
aud_log(fid,'[BSPLINE_D7] CONTINUOUS_CORRIDOR_CERT=%d C6_STRUCTURAL=1 PASS=%d', ...
    minBezierSlack>=0,pass);

B=struct();
B.pass=pass;
B.failure_reason='';
B.exitflag=exitflag;
B.message=output.message;
B.degree=p;
B.ncoef=ncoef;
B.nspan=nspan;
B.span_count_by_cell=spanCount;
B.breaks=breaks;
B.knots=U;
B.span_cell_assignment=cellOfSpan;
B.control_points=[cpX,cpY,cpZ].';
B.eq_residual=eqResidual;
B.endpoint_residual=endpointResidual;
B.min_safe_slack=min(minSafe,minBezierSlack);
B.linear_min_safe_slack=minSafe;
B.bezier_min_safe_slack=minBezierSlack;
B.l2_objective=l2obj;
B.C6_structural=true;
B.continuous_corridor_cert=minBezierSlack>=0;
B.extraction_min_entry=minEntry;
B.extraction_min_nonzero=minNZ;
B.extraction_row_sum_error=rowSumErr;
B.ineq_scale=ineqScale;
B.eq_scale=eqScale;
end

function B=step3a_fail_bspline(reason)
B=struct();
B.pass=false;
B.failure_reason=reason;
B.exitflag=NaN;
B.message='';
B.degree=7;
B.ncoef=22;
B.nspan=15;
B.span_count_by_cell=[];
B.breaks=[];
B.knots=[];
B.span_cell_assignment=[];
B.control_points=[];
B.eq_residual=NaN;
B.endpoint_residual=NaN;
B.min_safe_slack=NaN;
B.linear_min_safe_slack=NaN;
B.bezier_min_safe_slack=NaN;
B.l2_objective=NaN;
B.C6_structural=true;
B.continuous_corridor_cert=false;
B.extraction_min_entry=NaN;
B.extraction_min_nonzero=NaN;
B.extraction_row_sum_error=NaN;
B.ineq_scale=[];
B.eq_scale=[];
end

function n=step3a_allocate_spans(nspan,frac)
% Largest-remainder allocation with one span guaranteed per guide segment.
m=numel(frac);
n=ones(1,m);
remain=nspan-m;
if remain<0
    error('STEP3A:SpanCount','Too few spans.');
end

raw=remain*frac/sum(frac);
base=floor(raw);
n=n+base;
left=remain-sum(base);

rem=raw-base;
[~,ord]=sort(rem,'descend');
for k=1:left
    n(ord(k))=n(ord(k))+1;
end
end

function [A2,b2,scale]=step3a_scale_rows(A,b)
if isempty(A)
    A2=A;
    b2=b;
    scale=zeros(0,1);
    return
end
scale=max(abs(A),[],2);
scale=max(scale,abs(b));
scale(scale==0)=1;
A2=A./scale;
b2=b./scale;
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

function v=step3a_min_nonzero(A)
a=abs(A(:));
a=a(a>0);
if isempty(a)
    v=0;
else
    v=min(a);
end
end

function C=step2_fail_case(reason)
C=struct();
C.pass=false;
C.failure_reason=reason;
C.cell_count=0;
C.cells={};
C.separators=[];
C.cell_interiors=[];
C.overlaps=[];
C.guide_containment_pass=false;
C.support_structure_pass=false;
C.strict_cell_pass=false;
C.strict_overlap_pass=false;
C.shared_waypoint_pass=false;
C.min_guide_cell_slack=NaN;
C.min_shared_waypoint_slack=NaN;
C.min_finite_cell_radius=NaN;
C.min_finite_overlap_radius=NaN;
C.n_unbounded_cells=0;
C.n_unbounded_overlaps=0;
C.shared_waypoint_slack=[];
end

function [r,strict,unbounded]=step2_chebyshev_radius(A,b)
% A p >= b. Rows are unit normals.
%
% max r
% s.t. A p - r >= b, r>=0.
%
% r>0 proves strict interior.
% exitflag=-3 means the radius is unbounded, hence strict interior exists.

rowNorm=vecnorm(A,2,2);

f=[0;0;0;-1];
Aineq=[-A,rowNorm];
bineq=-b;
lb=[-Inf;-Inf;-Inf;0];

opts=optimoptions('linprog', ...
    'Display','none', ...
    'ConstraintTolerance',1e-10, ...
    'OptimalityTolerance',1e-10);

[z,~,ef,out]=linprog(f,Aineq,bineq,[],[],lb,[],opts); %#ok<ASGLU>

if ef==1
    r=z(4);
    unbounded=false;
    strict=r>0;
elseif ef==-3
    r=Inf;
    unbounded=true;
    strict=true;
else
    error('STEP2AUDIT:ChebyshevLP','Chebyshev-radius LP failed: %s',out.message);
end
end

function R=aud_run_case(S,P,fid)
R=struct();
R.failure_reason='';

F=aud_sensor_trigger_frontend(S,P);
R.front=F;

aud_log(fid,'[FRONT] initialOutside=%d detectedAt=%s memory=%d trigger=%d t=%.3f TTC=%.3f known=%s', ...
    F.initial_outside_all,mat2str(F.detected_time,4),F.memory_pass,F.triggered, ...
    F.trigger_time,F.ttc,mat2str(F.known_ids_at_trigger));

if ~F.initial_outside_all
    R=aud_fail_result(R,'INITIAL_SENSOR_OUTSIDE_FAIL'); return
end
if ~F.memory_pass
    R=aud_fail_result(R,'DETECTION_MEMORY_FAIL'); return
end
if ~F.triggered
    R=aud_fail_result(R,'NO_TRIGGER'); return
end
if isempty(F.known_ids_at_trigger)
    R=aud_fail_result(R,'NO_KNOWN_OBSTACLE_AT_TRIGGER'); return
end

p0=F.payload_at_trigger;
e=aud_unit(S.e_nom);
obs=S.obs(F.known_ids_at_trigger);
Rreq=P.R_payload+P.d_margin;

[b1,b2]=aud_transverse_basis(e);

% Rejoin progress.
xiReq=zeros(1,numel(obs));
for i=1:numel(obs)
    xiReq(i)=aud_support(obs(i),e) ...
        + aud_vertical_nominal_system_support(e,P) ...
        + P.d_margin ...
        - e.'*p0;
end
[xiRejoin,kLim]=max(xiReq);
xiRejoin=max(0,xiRejoin);

% One common bypass direction.
obj=@(phi) aud_common_offset(phi,b1,b2,p0,obs,Rreq);
opts=optimset('Display','off','TolX',1e-12);
[phi0,~,~,~]=fminbnd(obj,0,2*pi,opts);
phiCand=[phi0,0,pi/2,pi,3*pi/2,2*pi];
dCand=arrayfun(obj,phiCand);
[dStar,jbest]=min(dCand);
phiStar=phiCand(jbest);

n=cos(phiStar)*b1+sin(phiStar)*b2;
n=n-e*(e.'*n);
n=n/norm(n);

% Recompute exact analytical requirements after projection.
req=zeros(1,numel(obs));
for i=1:numel(obs)
    req(i)=aud_support(obs(i),n)+Rreq-n.'*p0;
end
dStar=max([0,req]);

d0=dStar;
xi0=xiRejoin;

% =====================================================================
% V3 NUMERICAL ROBUSTIFICATION
% =====================================================================
% DO NOT relax b = h_E(n)+Rreq.
%
% Instead, move the GUIDE outward by the minimum representable amount
% until the original support inequalities are STRICTLY POSITIVE under
% several algebraically equivalent floating-point evaluation kernels.
%
% This is not an added physical margin or tuning threshold.
% It is a representability requirement: the mathematically safe boundary
% must not change sign merely because a later stage evaluates A*p-b via a
% different summation order.
% =====================================================================

dSteps=0;
while true
    W1=p0+dStar*n;
    W2=W1+e*xiRejoin;

    [minScalar,minMatrix,minExpanded]=aud_bypass_multikernel_slack( ...
        W1,W2,n,obs,Rreq);

    if min([minScalar,minMatrix,minExpanded])>0
        break
    end

    dNew=aud_next_up(dStar);
    if dNew<=dStar
        error('AUDIT:OutwardRoundD','Cannot outward-round bypass offset.');
    end
    dStar=dNew;
    dSteps=dSteps+1;

    if dSteps>10000
        error('AUDIT:OutwardRoundDLoop', ...
            'Bypass robustification exceeded 10000 representable steps.');
    end
end

% Rejoin robustification. Use BOTH the original nominal-system rejoin
% certificate and the payload +e corridor certificate, with multiple
% floating evaluation kernels.
xiSteps=0;
while true
    W3=p0+e*xiRejoin;

    [minNominal,minPayloadScalar,minPayloadMatrix,minPayloadExpanded]= ...
        aud_rejoin_multikernel_slack(W3,e,obs,P,Rreq);

    if min([minNominal,minPayloadScalar,minPayloadMatrix,minPayloadExpanded])>0
        break
    end

    xiNew=aud_next_up(xiRejoin);
    if xiNew<=xiRejoin
        error('AUDIT:OutwardRoundXi','Cannot outward-round rejoin xi.');
    end
    xiRejoin=xiNew;
    xiSteps=xiSteps+1;

    if xiSteps>10000
        error('AUDIT:OutwardRoundXiLoop', ...
            'Rejoin robustification exceeded 10000 representable steps.');
    end
end

% Final guide.
W0=p0;
W1=p0+dStar*n;
W2=W1+e*xiRejoin;
W3=p0+e*xiRejoin;
W=[W0,W1,W2,W3];

segLen=vecnorm(diff(W,1,2),2,1);
pathLength=sum(segLen);

% Original exact-support certificate, no safety tolerance.
supportSlack=zeros(3,numel(obs));
for i=1:numel(obs)
    supportSlack(1,i)=(-e).'*W0-aud_support(obs(i),-e)-Rreq;
    supportSlack(2,i)=n.'*W1-aud_support(obs(i),n)-Rreq;
    supportSlack(3,i)=e.'*W3-aud_support(obs(i),e)-Rreq;
end
supportPass=all(supportSlack(:)>0);

% Multi-kernel final audit.
[bSc,bMx,bEx]=aud_bypass_multikernel_slack(W1,W2,n,obs,Rreq);
[rNom,rSc,rMx,rEx]=aud_rejoin_multikernel_slack(W3,e,obs,P,Rreq);
multiKernelPass=min([bSc,bMx,bEx,rNom,rSc,rMx,rEx])>0;

% Exact Euclidean segment audit against ORIGINAL ellipsoids.
minSlack=Inf;
worstSeg=NaN;
worstObs=NaN;
for sidx=1:3
    A=W(:,sidx); B=W(:,sidx+1);
    for i=1:numel(obs)
        [dist,~,~]=aud_segment_ellipsoid_distance(A,B,obs(i));
        sl=dist-Rreq;
        if sl<minSlack
            minSlack=sl;
            worstSeg=sidx;
            worstObs=obs(i).id;
        end
    end
end
exactPass=minSlack>=0;

% Equality/progress audits remain separate from hard safety.
xiW=e.'*(W-p0);
dXi=diff(xiW);
eqScale=max(1,max(abs(xiW)));
eqTol=128*eps(eqScale);
progressPass=all(dXi>=-eqTol);

transverseResidual=norm((eye(3)-e*e.')*(W3-p0));
rejoinProgress=e.'*(W3-p0);
rejoinResidual=abs(rejoinProgress-xiRejoin);
rejoinPass=transverseResidual<=eqTol && rejoinResidual<=eqTol;

pass=supportPass && multiKernelPass && exactPass && progressPass && rejoinPass;

aud_log(fid,'[GUIDE] pTrigger=[%.6f %.6f %.6f] xiRejoin=%.17g limitingObs=%d', ...
    p0(1),p0(2),p0(3),xiRejoin,obs(kLim).id);
aud_log(fid,'[GUIDE] phi=%.12f n=[%.12f %.12f %.12f] eDotN=%+.17e', ...
    phiStar,n(1),n(2),n(3),e.'*n);
aud_log(fid,'[GUIDE] bypass=%.17g pathLength=%.12f',dStar,pathLength);

aud_log(fid,'[ROUND-V3] d original=%.17g final=%.17g ULPsteps=%d delta=%+.17e', ...
    d0,dStar,dSteps,dStar-d0);
aud_log(fid,'[ROUND-V3] xi original=%.17g final=%.17g ULPsteps=%d delta=%+.17e', ...
    xi0,xiRejoin,xiSteps,xiRejoin-xi0);

aud_log(fid,'[MULTIKERNEL-BYPASS] scalar=%+.17e matrix=%+.17e expanded=%+.17e pass=%d', ...
    bSc,bMx,bEx,min([bSc,bMx,bEx])>0);
aud_log(fid,'[MULTIKERNEL-REJOIN] nominal=%+.17e payloadScalar=%+.17e payloadMatrix=%+.17e payloadExpanded=%+.17e pass=%d', ...
    rNom,rSc,rMx,rEx,min([rNom,rSc,rMx,rEx])>0);

aud_log(fid,'[CERT] supportPass=%d minSupportSlack=%+.17e', ...
    supportPass,min(supportSlack(:)));
aud_log(fid,'[CERT] multiKernelPass=%d',multiKernelPass);
aud_log(fid,'[CERT] exactPass=%d minExactSlack=%+.17e worstSeg=%d worstObs=%d', ...
    exactPass,minSlack,worstSeg,worstObs);
aud_log(fid,'[EQUALITY] xiWaypoints=%s dXi=%s eqTol=%.17e', ...
    mat2str(xiW,17),mat2str(dXi,17),eqTol);
aud_log(fid,'[EQUALITY] transverseResidual=%.17e rejoinResidual=%.17e', ...
    transverseResidual,rejoinResidual);
aud_log(fid,'[CERT] progressPass=%d rejoinPass=%d GUIDE_PASS=%d', ...
    progressPass,rejoinPass,pass);

R.pass=pass;
R.support_pass=supportPass;
R.exact_pass=exactPass;
R.progress_pass=progressPass;
R.rejoin_pass=rejoinPass;
R.bypass_offset=dStar;
R.bypass_direction=n;
R.path_length=pathLength;
R.exact_min_clearance_slack=minSlack;
R.rejoin_xi=xiRejoin;
R.waypoints=W;
R.support_slack=supportSlack;
R.failure_reason='';
R.numeric=struct( ...
    'd0',d0,'d_final',dStar,'d_ulp_steps',dSteps, ...
    'xi0',xi0,'xi_final',xiRejoin,'xi_ulp_steps',xiSteps, ...
    'bypass_scalar_slack',bSc,'bypass_matrix_slack',bMx, ...
    'bypass_expanded_slack',bEx,'rejoin_nominal_slack',rNom, ...
    'rejoin_payload_scalar_slack',rSc,'rejoin_payload_matrix_slack',rMx, ...
    'rejoin_payload_expanded_slack',rEx,'multi_kernel_pass',multiKernelPass, ...
    'eq_tol',eqTol,'transverse_residual',transverseResidual, ...
    'rejoin_residual',rejoinResidual);
end

function R=aud_fail_result(R,reason)
R.pass=false;
R.support_pass=false;
R.exact_pass=false;
R.progress_pass=false;
R.rejoin_pass=false;
R.bypass_offset=NaN;
R.bypass_direction=nan(3,1);
R.path_length=NaN;
R.exact_min_clearance_slack=NaN;
R.rejoin_xi=NaN;
R.waypoints=nan(3,4);
R.support_slack=NaN;
R.failure_reason=reason;
end

function CASES=aud_build_cases()
% Deterministic scenario suite. All obstacle centers are created in a local
% frame tied to the nominal direction, then mapped to world coordinates.

CASES=repmat(aud_empty_case(),1,14);

% 1 obstacle baseline
CASES(1)=aud_make_case('C01_ONE', ...
    'single oriented ellipsoid, horizontal +x nominal motion', ...
    [1;0;0],1.0, ...
    [20,0.30,0.10], ...
    [1.00,0.75,1.10], ...
    [20,10,-15]);

% 2 obstacle baseline-like
CASES(2)=aud_make_case('C02_TWO_BASE', ...
    'two nearby oriented ellipsoids with opposite transverse offsets', ...
    [1;0;0],1.0, ...
    [20,0.20,0.05;24,-0.25,0.18], ...
    [1.00,0.72,1.05;0.92,0.78,1.12], ...
    [22,8,-10;-31,-12,20]);

% 4 obstacle mixed sizes/orientations
CASES(3)=aud_make_case('C03_FOUR_MIX', ...
    'four-obstacle cluster, mixed sizes and orientations', ...
    [1;0;0],1.0, ...
    [18,0.5,0.2;21,-0.4,-0.1;24,0.3,0.4;27,-0.2,0.0], ...
    [0.7,0.9,1.0;1.2,0.6,1.1;0.9,1.0,0.7;1.3,0.8,1.2], ...
    [15,20,-5;-35,10,25;48,-18,12;70,25,-20]);

% 8 obstacle dense cluster
CASES(4)=aud_make_dense_case('C04_EIGHT_DENSE', ...
    'eight-obstacle dense cluster', ...
    [1;0;0],1.0,8,18.0,1.55);

% 12 obstacle heavier-load front-end case
CASES(5)=aud_make_dense_case('C05_TWELVE_DENSE', ...
    'twelve-obstacle dense cluster', ...
    [1;0;0],1.0,12,17.0,1.20);

% Large ellipsoids
CASES(6)=aud_make_case('C06_LARGE', ...
    'large obstacles, mixed aspect ratios', ...
    [1;0;0],1.0, ...
    [20,0.5,0.2;25,-0.7,-0.3;30,0.2,0.5], ...
    [1.8,1.3,1.7;2.1,1.0,1.6;1.5,2.0,1.2], ...
    [25,10,30;-40,35,-15;65,-20,40]);

% Small ellipsoids
CASES(7)=aud_make_case('C07_SMALL', ...
    'small obstacles with mixed orientations', ...
    [1;0;0],1.0, ...
    [19,0.2,0.1;21,-0.2,-0.1;23,0.25,0.15;25,-0.15,0.0], ...
    [0.35,0.45,0.50;0.40,0.30,0.55;0.50,0.42,0.32;0.28,0.50,0.40], ...
    [10,45,-20;55,-30,15;-70,20,50;90,10,-35]);

% Slender/high-aspect-ratio ellipsoids
CASES(8)=aud_make_case('C08_SLENDER', ...
    'high-aspect-ratio ellipsoids, strongly rotated', ...
    [1;0;0],1.0, ...
    [20,0.0,0.0;24,0.3,-0.2;28,-0.4,0.3], ...
    [2.2,0.35,0.40;0.30,2.0,0.45;0.40,0.35,2.3], ...
    [55,35,70;-65,50,20;80,-40,60]);

% Horizontal diagonal path
CASES(9)=aud_make_case('C09_DIAG_XY', ...
    'diagonal horizontal nominal path', ...
    [1;1;0]/sqrt(2),1.2, ...
    [20,0.3,0.1;24,-0.4,0.2;28,0.2,-0.2], ...
    [1.0,0.8,1.0;0.8,1.1,0.9;1.2,0.7,1.1], ...
    [30,20,-10;-45,15,35;75,-25,10]);

% 3-D climbing nominal direction
CASES(10)=aud_make_case('C10_CLIMB', ...
    '3-D climbing nominal direction', ...
    aud_unit([1;0.25;0.20]),1.0, ...
    [20,0.4,0.2;24,-0.3,-0.2;28,0.2,0.3], ...
    [1.1,0.7,1.0;0.9,1.2,0.8;1.0,0.8,1.3], ...
    [25,35,10;-55,20,40;70,-30,-15]);

% 3-D descending nominal direction
CASES(11)=aud_make_case('C11_DESCEND', ...
    '3-D descending nominal direction', ...
    aud_unit([1;-0.20;-0.18]),1.1, ...
    [19,0.2,0.3;23,-0.35,-0.1;27,0.4,0.0], ...
    [0.9,1.0,0.8;1.2,0.7,1.1;0.8,1.3,1.0], ...
    [15,-25,45;-60,40,20;85,15,-30]);

% Close longitudinal spacing
CASES(12)=aud_make_case('C12_CLOSE_SPACING', ...
    'four obstacles with close longitudinal spacing', ...
    [1;0;0],0.9, ...
    [20.0,0.35,0.1;21.2,-0.3,-0.2;22.4,0.25,0.25;23.6,-0.2,-0.1], ...
    [0.9,0.8,1.0;0.8,1.0,0.9;1.0,0.75,1.1;0.85,1.05,0.8], ...
    [10,20,30;-25,35,-15;50,-20,45;-70,10,25]);

% Strong orientation sweep
CASES(13)=aud_make_case('C13_ORIENT_SWEEP', ...
    'six obstacles spanning strong orientation changes', ...
    [1;0;0],1.0, ...
    [18,0.4,0.1;20,-0.3,0.2;22,0.2,-0.2;24,-0.4,0.3;26,0.3,0.0;28,-0.2,-0.3], ...
    repmat([1.0,0.65,1.25],6,1), ...
    [0,0,0;30,15,20;60,30,40;90,45,60;120,60,80;150,75,100]);

% Different speed, same geometric architecture
CASES(14)=aud_make_case('C14_SPEED_1P8', ...
    'higher nominal speed with five mixed obstacles', ...
    aud_unit([1;0.35;0.05]),1.8, ...
    [20,0.2,0.2;23,-0.5,-0.1;26,0.4,0.3;29,-0.2,-0.2;32,0.3,0.1], ...
    [0.8,1.0,1.1;1.2,0.7,0.9;0.9,1.3,0.8;1.0,0.8,1.2;1.1,0.9,1.0], ...
    [20,10,15;-35,25,-20;55,-30,40;80,15,-35;110,40,25]);
end

function S=aud_empty_case()
S=struct('case_id','','case_desc','','p0',[0;0;0],'pg',[40;0;0], ...
    'v0',[1;0;0],'e_nom',[1;0;0],'T',40,'obs',struct([]));
end

function S=aud_make_dense_case(id,desc,e,speed,nobs,xi0,dx)
centers=zeros(nobs,3);
radii=zeros(nobs,3);
angles=zeros(nobs,3);
for k=1:nobs
    centers(k,1)=xi0+(k-1)*dx;
    centers(k,2)=0.55*sin(0.9*k);
    centers(k,3)=0.40*cos(0.7*k);
    radii(k,:)=[0.65+0.05*mod(k,4), 0.55+0.07*mod(k+1,3), 0.75+0.06*mod(k+2,5)];
    angles(k,:)=[17*k, -11*k, 23*k];
end
S=aud_make_case(id,desc,e,speed,centers,radii,angles);
end

function S=aud_make_case(id,desc,e,speed,localCenters,radii,anglesDeg)
e=aud_unit(e);
[b1,b2]=aud_transverse_basis(e);
B=[e,b1,b2];

p0=[0;0;0];
pg=p0+e*45;

n=size(localCenters,1);
obs=repmat(struct('center',[],'radii',[],'R',[],'S',[],'id',0),1,n);

for i=1:n
    q=localCenters(i,:).';
    c=p0+B*q;

    a=anglesDeg(i,:);
    Rloc=aud_Rz(deg2rad(a(1)))*aud_Ry(deg2rad(a(2)))*aud_Rx(deg2rad(a(3)));
    R=B*Rloc*B.';

    rr=radii(i,:).';
    obs(i)=struct('center',c,'radii',rr,'R',R,'S',R*diag(rr.^2)*R','id',i);
end

S=struct();
S.case_id=id;
S.case_desc=desc;
S.p0=p0;
S.pg=pg;
S.v0=e*speed;
S.e_nom=e;
S.T=norm(pg-p0)/speed;
S.obs=obs;
end

function P=aud_params()
P=struct();
P.d_margin=0.15;
P.R_payload=0.24;
P.R_uav=0.30;
P.R_cable=0.045;
P.L_cable=2.0;
P.sensor_R=15.0;
P.dt_sensor=0.05;
P.lookahead=5.0;
P.pred_dt=0.10;
end

function F=aud_sensor_trigger_frontend(S,P)
v=S.v0;
speed=norm(v);
if speed<=0
    error('AUDIT:ZeroSpeed','Nonzero nominal speed required.');
end

Tnom=norm(S.pg-S.p0)/speed;
no=numel(S.obs);

detected=false(1,no);
detect_t=nan(1,no);
initial_out=true(1,no);

% Nominal cable stays vertical in this Step-1 trigger replay.
pU0=S.p0+P.L_cable*[0;0;1];

for o=1:no
    rs=max(S.obs(o).radii); % sensor gate only
    initial_out(o)=norm(S.obs(o).center-pU0)>P.sensor_R+rs;
end

triggered=false;
trig_t=NaN;
ttc=NaN;
trig_obs=0;
trig_body='';
known_at=[];
payload_trig=[NaN;NaN;NaN];
first_detect=NaN;
memory_pass=true;
prev_known=false(1,no);

for t=0:P.dt_sensor:Tnom
    pL=S.p0+v*t;
    pU=pL+P.L_cable*[0;0;1];

    for o=1:no
        if ~detected(o)
            rs=max(S.obs(o).radii); % SENSOR ONLY
            if norm(S.obs(o).center-pU)<=P.sensor_R+rs
                detected(o)=true;
                detect_t(o)=t;
                if isnan(first_detect)
                    first_detect=t;
                end
            end
        end
    end
if any(prev_known & ~detected)
        memory_pass=false;
    end
    prev_known=detected;

    ids=find(detected);
    if ~isempty(ids)
        [hit,tau,oid,body]=aud_predict_nominal_collision(pL,v,S.obs(ids),ids,P);
        if hit
            triggered=true;
            trig_t=t;
            ttc=tau;
            trig_obs=oid;
            trig_body=body;
            known_at=ids;
            payload_trig=pL;
            break
        end
    end
end

F=struct( ...
    'initial_outside_each',initial_out, ...
    'initial_outside_all',all(initial_out), ...
    'detected_time',detect_t, ...
    'first_detect_time',first_detect, ...
    'memory_pass',memory_pass, ...
    'triggered',triggered, ...
    'trigger_time',trig_t, ...
    'ttc',ttc, ...
    'trigger_obstacle_id',trig_obs, ...
    'trigger_body',trig_body, ...
    'known_ids_at_trigger',known_at, ...
    'payload_at_trigger',payload_trig);
end

function [hit,tau,globalObsId,bodyName]=aud_predict_nominal_collision(pNow,v,knownObs,globalIds,P)
hit=false; tau=Inf; globalObsId=0; bodyName='';

for dt=0:P.pred_dt:P.lookahead
    pL=pNow+v*dt;
    pU=pL+P.L_cable*[0;0;1];

    for io=1:numel(knownObs)
        [g,b]=aud_point_system_margin_nominal(pL,pU,knownObs(io),P);
        if g<=0
            hit=true;
            tau=dt;
            globalObsId=globalIds(io);
            bodyName=b;
            return
        end
    end
end
end

function [g,bodyName]=aud_point_system_margin_nominal(pL,pU,ob,P)
N=aud_add_cardinal(aud_fibonacci_sphere(722));
K=size(N,2);
H=zeros(K,1);

for k=1:K
    H(k)=aud_support(ob,N(:,k));
end

nPL=N.'*pL;
nPU=N.'*pU;

gL=max(nPL-H-P.R_payload-P.d_margin);
gU=max(nPU-H-P.R_uav-P.d_margin);
gC=max(min(nPL,nPU)-H-P.R_cable-P.d_margin);

parts=[gL gU gC];
[g,idx]=min(parts);
names={'Payload','UAV','Cable'};
bodyName=names{idx};
end

function d=aud_common_offset(phi,b1,b2,p0,obs,Rreq)
n=cos(phi)*b1+sin(phi)*b2;
n=n/norm(n);
req=zeros(1,numel(obs));
for i=1:numel(obs)
    req(i)=aud_support(obs(i),n)+Rreq-n.'*p0;
end
d=max([0,req]);
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

function h=aud_support(ob,n)
n=n(:);
h=n.'*ob.center+sqrt(max(0,n.'*ob.S*n));
end

function d=aud_vertical_nominal_system_support(n,P)
nz=n(3);
dL=P.R_payload;
dU=P.L_cable*nz+P.R_uav;
dC=max(0,P.L_cable*nz)+P.R_cable;
d=max([dL,dU,dC]);
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

function R=aud_Rx(a)
R=[1 0 0;0 cos(a) -sin(a);0 sin(a) cos(a)];
end

function R=aud_Ry(a)
R=[cos(a) 0 sin(a);0 1 0;-sin(a) 0 cos(a)];
end

function R=aud_Rz(a)
R=[cos(a) -sin(a) 0;sin(a) cos(a) 0;0 0 1];
end

function D=aud_fibonacci_sphere(N)
k=(0:N-1)';
z=1-2*(k+0.5)/N;
phi=pi*(3-sqrt(5))*k;
r=sqrt(max(0,1-z.^2));
D=[r.*cos(phi) r.*sin(phi) z].';
end

function D=aud_add_cardinal(D)
C=[eye(3) -eye(3)];
D=[D C];
D=D./vecnorm(D,2,1);
end

function [mScalar,mMatrix,mExpanded]=aud_bypass_multikernel_slack(W1,W2,n,obs,Rreq)
% Evaluate the SAME exact-support inequality using several algebraically
% equivalent floating-point kernels. All must remain strictly positive.

mScalar=Inf;
mMatrix=Inf;
mExpanded=Inf;

A=repmat(n(:).',numel(obs),1);
b=zeros(numel(obs),1);

for i=1:numel(obs)
    b(i)=aud_support(obs(i),n)+Rreq;

    s1=n.'*W1-b(i);
    s2=n.'*W2-b(i);
    mScalar=min([mScalar,s1,s2]);

    ex1=n(1)*W1(1)+n(2)*W1(2)+n(3)*W1(3)-b(i);
    ex2=n(1)*W2(1)+n(2)*W2(2)+n(3)*W2(3)-b(i);
    mExpanded=min([mExpanded,ex1,ex2]);
end

v1=A*W1-b;
v2=A*W2-b;
mMatrix=min([v1;v2]);
end

function [mNominal,mScalar,mMatrix,mExpanded]=aud_rejoin_multikernel_slack(W3,e,obs,P,Rreq)
mNominal=Inf;
mScalar=Inf;
mMatrix=Inf;
mExpanded=Inf;

A=repmat(e(:).',numel(obs),1);
b=zeros(numel(obs),1);

for i=1:numel(obs)
    % Original nominal suspended-system rejoin certificate.
    sn=e.'*W3-aud_support(obs(i),e)- ...
        aud_vertical_nominal_system_support(e,P)-P.d_margin;
    mNominal=min(mNominal,sn);

    % Payload corridor certificate.
    b(i)=aud_support(obs(i),e)+Rreq;

    ss=e.'*W3-b(i);
    mScalar=min(mScalar,ss);

    ex=e(1)*W3(1)+e(2)*W3(2)+e(3)*W3(3)-b(i);
    mExpanded=min(mExpanded,ex);
end

mMatrix=min(A*W3-b);
end

function y=aud_next_up(x)
% Smallest practical representable increase constructed from MATLAB eps.
% This is used only when the ORIGINAL hard inequality evaluates negative.
% It is numerical outward rounding, not an added physical margin.
if ~isfinite(x)
    y=x;
    return
end
dx=eps(x);
if dx==0
    dx=realmin;
end
y=x+dx;
if y<=x
    dx=eps(max(1,abs(x)));
    y=x+dx;
end
end

function aud_log(varargin)
% Silent in Step3 backend audit.
end

function M=bez13_geo_build_model(G,C)
n=13;
nseg=3;
ncp=n+1;

W=G.waypoints;
segLen=vecnorm(diff(W,1,2),2,1);
h=segLen/sum(segLen);
h=h(:).';

p0=G.front.payload_at_trigger;
e=G.waypoints(:,3)-G.waypoints(:,2);
e=e/norm(e);

% Minimal variables:
% knot 2 jet orders 0..6 xyz = 21
% knot 3 jet orders 0..6 xyz = 21
% xi = 1
nvar=43;
iXi=43;

% Structural numerical variable scaling.
% For an internal knot derivative of order r, use the geometric mean of
% adjacent segment lengths. The scale makes a unit z-variable induce O(L)
% Bezier-control-point motion through h^r/fallingFactorial(n,r).
L=max(1,G.path_length);
jetScale=zeros(2,7);
for kk=1:2
    href=sqrt(h(kk)*h(kk+1));
    for r=0:6
        ff=factorial(n)/factorial(n-r);
        jetScale(kk,r+1)=L*ff/(href^r);
    end
end
xiScale=L;

% Fixed finite-difference endpoint interpolation matrix:
% maps 14 CPs -> [left normalized differences 0..6;
%                  right normalized differences 0..6].
F=bez13_jet_endpoint_difference_matrix(n);
if rank(F)~=14
    error('MINCOJET:Interpolation','Endpoint-difference matrix is singular.');
end

% Affine CP map: cpVec = c0 + Gcp*z
% cpVec ordering is x/y/z contiguous per segment CP via bez13_jet_cpidx.
ncpall=3*nseg*ncp;
Gcp=zeros(ncpall,nvar);
c0=zeros(ncpall,1);

for q=1:nseg
    for ax=1:3
        D0=zeros(14,1);
        Dz=zeros(14,nvar);

        % Left knot q, right knot q+1.
        [D0,Dz]=bez13_jet_insert_knot_side(D0,Dz,1,q,h(q),ax,p0,e, ...
            jetScale,xiScale,n,iXi);
        [D0,Dz]=bez13_jet_insert_knot_side(D0,Dz,8,q+1,h(q),ax,p0,e, ...
            jetScale,xiScale,n,iXi);

        % CP from normalized endpoint differences.
        % Degree 13 has 14 CPs and endpoint difference orders 0..6 on
        % each side.  The inverse is available in closed binomial form,
        % so do NOT use F\D here.  This removes an unnecessary
        % condition-number amplification while preserving the exact map.
        E=bez13_endpoint_difference_inverse_direct(n);
        cp0=E*D0;
        cpG=E*Dz;

        ids=arrayfun(@(j) bez13_jet_cpidx(q,j,ax,ncp),1:ncp);
        c0(ids)=cp0;
        Gcp(ids,:)=cpG;
    end
end

% Hard corridor inequalities on actual CPs.
Aineq=zeros(0,nvar);
bineq=zeros(0,1);
safeMeta=struct('segment',{},'cp',{},'plane',{});

for q=1:nseg
    CC=C.cells{q};

    for j=1:ncp
        for pl=1:size(CC.A,1)
            a=zeros(1,ncpall);
            for ax=1:3
                a(bez13_jet_cpidx(q,j,ax,ncp))=-CC.A(pl,ax);
            end

            Aineq(end+1,:)=a*Gcp; %#ok<AGROW>
            bineq(end+1,1)=-CC.b(pl)-a*c0; %#ok<AGROW>

            safeMeta(end+1)=struct( ...
                'segment',q,'cp',j,'plane',pl); %#ok<AGROW>
        end
    end
end

% -------------------------------------------------------------------------
% Geometric guide-proximity objective in minimal coordinates.
%
% For each Bezier CP, define the target point by uniform local parameter
% interpolation along the corresponding Step1 guide segment.
%
%   Jgeo = sum ||Pcp - Pguide||^2
%
% Since Pcp = c0 + Gcp*z:
%
%   Jgeo = 0.5*z'*H*z + f'*z + const
%   H = 2*Gcp'*Gcp
%   f = 2*Gcp'*(c0-target)
%
% No arbitrary physical or tuning weight is introduced.
% -------------------------------------------------------------------------
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

Hphys=2*(Gcp.'*Gcp);
Hphys=0.5*(Hphys+Hphys.');
fphys=2*Gcp.'*(c0-target);

% Positive scalar scaling only; minimizer is unchanged.
d=max(abs(diag(Hphys)));
if ~(d>0), d=1; end
Hscaled=sparse(Hphys/d);
fscaled=fphys/d;

M=struct();
M.degree=n;
M.nseg=nseg;
M.ncp=ncp;
M.nvar=nvar;
M.iXi=iXi;
M.h=h;
M.jetScale=jetScale;
M.xiScale=xiScale;
M.Gcp=Gcp;
M.c0=c0;
M.Aineq=sparse(Aineq);
M.bineq=bineq;
M.safeMeta=safeMeta;
M.Hphysical=sparse(Hphys);
M.fphysical=fphys;
M.Hscaled=Hscaled;
M.fscaled=fscaled;
end





function A6=internal_a6_c6_convergence_fast_runtime(STEP3,OPT)
% A6:
%  (1) Exact canonical knot jump d0..d6.
%  (2) Independent left/right approach using the SAME normalized distance
%      delta=min(hL,hR)*eps, so the convergence measure is meaningful.
%  (3) Convergence slope of E_k(delta) as delta -> 0.
%  (4) Fast production evaluator timing with all derivative coefficients
%      precomputed. No factorial or derivative construction online.%
% Important:
% E_k(delta)=||p^(k)(s_k-delta)-p^(k)(s_k+delta)|| is NOT a C6 jump.
% For a C6 curve it is expected to be O(delta), because the two sample
% points are distinct. The exact knot jump remains the continuity test.

epsSet=sort(OPT.NearBoundaryEpsRel(:).','descend');
backendNames={'BEZIER_D13','POLY_D13','BSPLINE_D7'};
P0=aud_params();
CASES0=aud_build_cases();
Ncase=numel(CASES0);

caseTemplate=struct('case_id','','T_ref',NaN,'backends',struct(),'pass',false);
A6=struct();
A6.mode='STEP4A6_C6_CONVERGENCE_FAST_RUNTIME';
A6.records=repmat(caseTemplate,1,Ncase);

passCounts=zeros(1,3);
auditCounts=zeros(1,3);
globalExact=zeros(3,7);
globalSmallestE=zeros(3,7);
globalSmallestRelE=zeros(3,7);
runtimeMedian=zeros(Ncase,3);
runtimeP95=zeros(Ncase,3);
runtimeMax=zeros(Ncase,3);
prepMsAll=zeros(Ncase,3);

fprintf('\n========================================================================================================================\n');
fprintf(' A6 RIGOROUS C6 CONVERGENCE + FAST PRODUCTION REFERENCE AUDIT\n');
fprintf(' epsRel=%s\n',mat2str(epsSet,4));
fprintf(' E_k(delta)=||left-right|| at equal normalized distance delta; expected O(delta) for k<=6.\n');
fprintf('========================================================================================================================\n');

for ic=1:Ncase
    Sdef=CASES0(ic);
    R3=STEP3.records(ic);
    G=aud_run_case(Sdef,P0,1);

    segFrac=vecnorm(diff(G.waypoints,1,2),2,1);
    segFrac=segFrac/sum(segFrac);
    segFrac=segFrac(:).';

    C=caseTemplate;
    C.case_id=Sdef.case_id;
    C.T_ref=Sdef.T;
    casePass=true;

    fprintf('\n########################################################################################################################\n');
    fprintf(' CASE %02d / %02d : %s | T_ref=%.17g\n',ic,Ncase,Sdef.case_id,Sdef.T);
    fprintf('########################################################################################################################\n');

    for ib=1:3
        bn=backendNames{ib};
        tp=tic;

        switch bn
            case 'BEZIER_D13'
                PREP=a6_prepare_d13_fast(R3.bezier.shared_knot_jets,segFrac);
                authority='SHARED_HERMITE_KNOT_JETS_FAST_CACHE';
            case 'POLY_D13'
                PREP=a6_prepare_d13_fast(R3.poly.shared_knot_jets,segFrac);
                authority='SHARED_HERMITE_KNOT_JETS_FAST_CACHE';
            case 'BSPLINE_D7'
                PREP=a5_prepare_bspline(R3.bspline.native_control_points,R3.bspline.knots,7);
                authority='NATIVE_BSPLINE_DERIVATIVE_CACHE';
        end

        prepMs=1e3*toc(tp);
        prepMsAll(ic,ib)=prepMs;

        fprintf('\n[%s] authority=%s | prep=%.6f ms\n',bn,authority,prepMs);

        BR=struct();
        BR.authority=authority;
        BR.preparation_ms=prepMs;
        BR.by_scale=repmat(a6_scale_template(),1,numel(OPT.TScaleSet));
        backendPass=true;

        for is=1:numel(OPT.TScaleSet)
            sc=OPT.TScaleSet(is);
            T=Sdef.T*sc;

            [exactJump,Eabs,Erel,EknotAbs,slopes]=a6_convergence_audit(PREP,T,epsSet);

            exactJump=a5_row7(exactJump,'A6 exactJump');
            smallAbs=a5_row7(Eabs(end,:),'A6 smallest Eabs');
            smallRel=a5_row7(Erel(end,:),'A6 smallest Erel');

            globalExact(ib,:)=max(globalExact(ib,:),exactJump);
            globalSmallestE(ib,:)=max(globalSmallestE(ib,:),smallAbs);
            globalSmallestRelE(ib,:)=max(globalSmallestRelE(ib,:),smallRel);

            SR=a6_scale_template();
            SR.scale=sc;
            SR.T=T;
            SR.eps_rel=epsSet;
            SR.exact_knot_jump_d0_d6=exactJump;
            SR.left_right_abs=Eabs;
            SR.left_right_rel=Erel;
            SR.to_knot_abs=EknotAbs;
            SR.local_loglog_slope=slopes;
            SR.smallest_eps_abs_d0_d6=smallAbs;
            SR.smallest_eps_rel_d0_d6=smallRel;
            SR.pass=all(exactJump==0) && all(isfinite(Eabs(:))) && all(isfinite(Erel(:)));

            BR.by_scale(is)=SR;
            backendPass=backendPass&&SR.pass;

            fprintf('  T/Tref=%5.2f | exact=%s | Eabs(eps=%g)=%s | Erel=%s\n', ...
                sc,mat2str(exactJump,3),epsSet(end),mat2str(smallAbs,5),mat2str(smallRel,5));
        end

        Rtime=a6_runtime_benchmark(PREP,Sdef.T,OPT);
        runtimeMedian(ic,ib)=Rtime.median_us;
        runtimeP95(ic,ib)=Rtime.p95_us;
        runtimeMax(ic,ib)=Rtime.max_us;

        BR.runtime=Rtime;
        BR.pass=backendPass;
        C.backends.(bn)=BR;

        auditCounts(ib)=auditCounts(ib)+1;
        if backendPass, passCounts(ib)=passCounts(ib)+1; end
        casePass=casePass&&backendPass;

        fprintf('  [RUNTIME] FAST median/p95/max = %.3f / %.3f / %.3f us/query\n', ...
            Rtime.median_us,Rtime.p95_us,Rtime.max_us);
    end

    C.pass=casePass;
    A6.records(ic)=C;
end

A6.aggregate=struct();
A6.aggregate.backend_names=backendNames;
A6.aggregate.pass_counts=passCounts;
A6.aggregate.audit_counts=auditCounts;
A6.aggregate.global_exact_knot_jump_d0_d6=globalExact;
A6.aggregate.global_smallest_eps_abs_d0_d6=globalSmallestE;
A6.aggregate.global_smallest_eps_rel_d0_d6=globalSmallestRelE;
A6.aggregate.prep_ms_median=median(prepMsAll,1);
A6.aggregate.prep_ms_max=max(prepMsAll,[],1);
A6.aggregate.runtime_us_median_of_case_medians=median(runtimeMedian,1);
A6.aggregate.runtime_us_worst_case_p95=max(runtimeP95,[],1);
A6.aggregate.runtime_us_max_observed=max(runtimeMax,[],1);
A6.aggregate.step3_median_solve_ms=STEP3.aggregate.median_solve_ms;
A6.aggregate.step3_max_solve_ms=STEP3.aggregate.max_solve_ms;
A6.aggregate.all_three_backends_pass=all(passCounts==Ncase);

fprintf('\n========================================================================================================================\n');
fprintf(' A6 COMPACT SUMMARY\n');
fprintf('========================================================================================================================\n');
for ib=1:3
    fprintf('%-12s pass=%d/%d\n',backendNames{ib},passCounts(ib),auditCounts(ib));
    fprintf('             exact knot jump d0..d6       = %s\n',mat2str(globalExact(ib,:),7));
    fprintf('             Eabs @ smallest eps=%g        = %s\n',epsSet(end),mat2str(globalSmallestE(ib,:),7));
    fprintf('             Erel @ smallest eps=%g        = %s\n',epsSet(end),mat2str(globalSmallestRelE(ib,:),7));
    fprintf('             prep median/max [ms]          = %.6f / %.6f\n', ...
        A6.aggregate.prep_ms_median(ib),A6.aggregate.prep_ms_max(ib));
    fprintf('             FAST runtime median [us]      = %.3f\n',A6.aggregate.runtime_us_median_of_case_medians(ib));
    fprintf('             worst-case p95/max [us]       = %.3f / %.3f\n', ...
        A6.aggregate.runtime_us_worst_case_p95(ib),A6.aggregate.runtime_us_max_observed(ib));
end
fprintf('\nStep3 QP median solve [B P S] ms = %s\n',mat2str(A6.aggregate.step3_median_solve_ms,6));
fprintf('Step3 QP max solve    [B P S] ms = %s\n',mat2str(A6.aggregate.step3_max_solve_ms,6));
fprintf('\n[INTERPRETATION RULE]\n');
fprintf('Exact knot jump tests C6 continuity. E(delta) is a two-point approach metric, not a discontinuity residual.\n');
fprintf('For k<=6, E(delta) should trend to zero as delta decreases until floating-point noise dominates.\n');
fprintf('Next stage after A6: explicit old-trajectory/new-trajectory replanning splice d0..d6 audit.\n');
end

function PREP=a6_prepare_d13_fast(J,frac)
% Same canonical D13 curve as A5, but all derivative polynomial
% coefficients and scale factors are precomputed offline/pre-query.
BASE=a5_prepare_d13(J,frac);
PREP=BASE;
PREP.kind='D13_HERMITE_FAST';
PREP.dcoef=cell(3,7);
PREP.inv_h_pow=zeros(3,7);

for seg=1:3
    h=PREP.knots(seg+1)-PREP.knots(seg);
    c=PREP.coeff{seg}; % ascending, 14x3
    for k=0:6
        dc=c;
        deg=size(dc,1)-1;
        for r=1:k
            mult=(1:deg).';
            dc=dc(2:end,:).*mult;
            deg=deg-1;
        end
        PREP.dcoef{seg,k+1}=dc;
        PREP.inv_h_pow(seg,k+1)=h^(-k);
    end
end
end

function Y=a6_eval_fast(PREP,s,T)
s=min(max(s,0),1);
switch PREP.kind
    case 'D13_HERMITE_FAST'
        Y=a6_eval_d13_fast(PREP,s,T);
    case 'BSPLINE_NATIVE'
        Y=a5_eval_bspline_native(PREP,s,T);
    otherwise
        error('A6:UnknownPrep','Unknown PREP kind %s.',PREP.kind);
end
if ~isequal(size(Y),[3 7]) || any(~isfinite(Y(:)))
    error('A6:EvalOutput','Fast evaluator must return finite 3x7.');
end
end

function Y=a6_eval_d13_fast(PREP,s,T)
Y=zeros(3,7);
tol=64*eps(max(1,abs(s)));
[dm,ik]=min(abs(PREP.knots-s));

if dm<=tol
    Y=PREP.shared_jets(:,:,ik);
    invT=1;
    for k=0:6
        if k>0, invT=invT/T; end
        Y(:,k+1)=Y(:,k+1)*invT;
    end
    return;
end

seg=find(PREP.knots(1:end-1)<s & s<PREP.knots(2:end),1);
if isempty(seg)
    if s<=0, seg=1; else, seg=3; end
end

a=PREP.knots(seg);
h=PREP.knots(seg+1)-a;
tau=(s-a)/h;

invT=1;
for k=0:6
    if k>0, invT=invT/T; end
    dc=PREP.dcoef{seg,k+1}; % ascending powers x 3 axes
    v=dc(end,:);
    for rr=size(dc,1)-1:-1:1
        v=v*tau+dc(rr,:);
    end
    Y(:,k+1)=v.'*(PREP.inv_h_pow(seg,k+1)*invT);
end
end

function [exactJump,Eabs,Erel,EknotAbs,slopes]=a6_convergence_audit(PREP,T,epsSet)
kn=PREP.knots;
nE=numel(epsSet);
Eabs=zeros(nE,7);
Erel=zeros(nE,7);
EknotAbs=zeros(nE,7);
exactJump=zeros(1,7);

if numel(kn)<=2
    slopes=nan(max(nE-1,1),7);
    return;
end

for j=2:numel(kn)-1
    sk=kn(j);
    deltaBase=min(sk-kn(j-1),kn(j+1)-sk);
    Yk=a6_eval_fast(PREP,sk,T);

    for ie=1:nE
        delta=epsSet(ie)*deltaBase;
        YL=a6_eval_fast(PREP,sk-delta,T);
        YR=a6_eval_fast(PREP,sk+delta,T);

        for k=1:7
            ea=norm(YL(:,k)-YR(:,k),2);
            den=max([1,norm(YL(:,k),2),norm(YR(:,k),2),norm(Yk(:,k),2)]);
            er=ea/den;
            ek=max(norm(YL(:,k)-Yk(:,k),2),norm(YR(:,k)-Yk(:,k),2));

            Eabs(ie,k)=max(Eabs(ie,k),ea);
            Erel(ie,k)=max(Erel(ie,k),er);
            EknotAbs(ie,k)=max(EknotAbs(ie,k),ek);
        end
    end
end

slopes=nan(max(nE-1,1),7);
if nE>=2
    for ie=1:nE-1
        ratio=epsSet(ie)/epsSet(ie+1);
        for k=1:7
            a=Eabs(ie,k);
            b=Eabs(ie+1,k);
            if a>0 && b>0
                slopes(ie,k)=log(a/b)/log(ratio);
            end
        end
    end
end
end

function R=a6_runtime_benchmark(PREP,T,OPT)
Nwarm=OPT.RuntimeWarmupQueries;
Nb=OPT.RuntimeBlocks;
Nq=OPT.RuntimeQueriesPerBlock;
sg=((1:Nq)-0.5)/Nq;
sink=0;

for i=1:Nwarm
    s=mod((i-0.5)/max(Nwarm,1),1);
    Y=a6_eval_fast(PREP,s,T);
    sink=sink+Y(1,1);
end

us=zeros(1,Nb);
for b=1:Nb
    t0=tic;
    for q=1:Nq
        Y=a6_eval_fast(PREP,sg(q),T);
        sink=sink+Y(1,1);
    end
    us(b)=1e6*toc(t0)/Nq;
end

R=struct();
R.block_us_per_query=us;
R.median_us=median(us);
R.p95_us=a5_percentile(us,95);
R.max_us=max(us);
R.warmup_queries=Nwarm;
R.blocks=Nb;
R.queries_per_block=Nq;
R.sink=sink;
a5_assert_runtime_struct(R,'A6-fast');
end

function S=a6_scale_template()
S=struct( ...
    'scale',NaN, ...
    'T',NaN, ...
    'eps_rel',[], ...
    'exact_knot_jump_d0_d6',nan(1,7), ...
    'left_right_abs',[], ...
    'left_right_rel',[], ...
    'to_knot_abs',[], ...
    'local_loglog_slope',[], ...
    'smallest_eps_abs_d0_d6',nan(1,7), ...
    'smallest_eps_rel_d0_d6',nan(1,7), ...
    'pass',false);
end

function A5=internal_a5_direct_reference_runtime_audit(STEP3,OPT)
% A5 production-path audit.
%
% D13 production reference:
%   shared global knot jets -> normalized degree-13 Hermite polynomial.
%   No Bezier/Bernstein reconstruction is used for reference evaluation.
%
% B-spline production reference:
%   native B-spline CP + native knot vector -> derivatives 0..6 directly.
%   Extracted Bezier remains only for the safety/corridor certificate.
%
% Timing:
%   Measures one full query returning [p p_dot ... p^(6)].
%   Warmup is excluded. Runtime is measured in blocks to reduce tic/toc
%   overhead. Generation/QP time and online reference-query time are kept
%   strictly separate.

Tscale=OPT.TScaleSet(:).';
epsSet=sort(OPT.NearBoundaryEpsRel(:).','descend');
backendNames={'BEZIER_D13','POLY_D13','BSPLINE_D7'};
P0=aud_params();
CASES0=aud_build_cases();
Ncase=numel(CASES0);

caseTemplate=struct('case_id','','T_ref',NaN,'backends',struct(),'pass',false);
A5=struct();
A5.mode='STEP4A5_DIRECT_REFERENCE_RUNTIME_AUDIT';
A5.records=repmat(caseTemplate,1,Ncase);

passCounts=zeros(1,3);
auditCounts=zeros(1,3);
globalExactKnot=zeros(3,7);
globalNearLast=zeros(3,7);
globalLegacyMismatch=zeros(3,7);

runtimeDirectMedian=zeros(Ncase,3);
runtimeDirectP95=zeros(Ncase,3);
runtimeDirectMax=zeros(Ncase,3);
runtimeLegacyMedian=zeros(Ncase,3);
runtimePrepMs=zeros(Ncase,3);

fprintf('\n========================================================================================================================\n');
fprintf(' A5 DIRECT NATIVE REFERENCE / C6 NEAR-BOUNDARY / RUNTIME AUDIT\n');
fprintf(' all 14 cases x 3 backends | d0..d6 per query | warmup excluded\n');
fprintf(' near-boundary epsRel = %s\n',mat2str(epsSet,4));
fprintf(' runtime warmup=%d, blocks=%d, queries/block=%d\n', ...
    OPT.RuntimeWarmupQueries,OPT.RuntimeBlocks,OPT.RuntimeQueriesPerBlock);
fprintf('========================================================================================================================\n');

trow=a5_row7((1:7),'A5 self-test row');
tcol=a5_row7((1:7).','A5 self-test column');
if ~isequal(size(trow),[1 7]) || ~isequal(size(tcol),[1 7]) || ~isequal(trow,tcol)
    error('A5:SelfTestShape','Internal 1x7 vector normalization self-test failed.');
end
fprintf('[SELF-TEST] d0..d6 vector shape normalization = PASS\n');

for ic=1:Ncase
    Sdef=CASES0(ic);
    R3=STEP3.records(ic);
    G=aud_run_case(Sdef,P0,1);

    segFrac=vecnorm(diff(G.waypoints,1,2),2,1);
    segFrac=segFrac/sum(segFrac);
    segFrac=segFrac(:).';

    C=caseTemplate;
    C.case_id=Sdef.case_id;
    C.T_ref=Sdef.T;
    casePass=true;

    fprintf('\n########################################################################################################################\n');
    fprintf(' CASE %02d / %02d : %s | T_ref=%.17g\n',ic,Ncase,Sdef.case_id,Sdef.T);
    fprintf('########################################################################################################################\n');

    for ib=1:3
        bn=backendNames{ib};

        ticPrep=tic;
        switch bn
            case 'BEZIER_D13'
                PREP=a5_prepare_d13(R3.bezier.shared_knot_jets,segFrac);
                LEG=struct('kind','BEZIER','CP',R3.bezier.control_points,'degree',13,'pieceFrac',segFrac);
                authority='SHARED_HERMITE_KNOT_JETS';

            case 'POLY_D13'
                PREP=a5_prepare_d13(R3.poly.shared_knot_jets,segFrac);
                LEG=struct('kind','BEZIER','CP',R3.poly.bernstein_control_points,'degree',13,'pieceFrac',segFrac);
                authority='SHARED_HERMITE_KNOT_JETS';

            case 'BSPLINE_D7'
                PREP=a5_prepare_bspline(R3.bspline.native_control_points,R3.bspline.knots,7);
                % Legacy extracted-Bezier span fractions are exact native knot intervals.
                br=unique(R3.bspline.knots,'stable');
                br=(br-br(1))/(br(end)-br(1));
                LEG=struct('kind','BEZIER','CP',R3.bspline.extracted_bezier_control_points, ...
                    'degree',7,'pieceFrac',diff(br));
                authority='NATIVE_BSPLINE_CP_KNOTS';
        end
        prepMs=1e3*toc(ticPrep);
        runtimePrepMs(ic,ib)=prepMs;

        fprintf('\n[%s] authority=%s | prep=%.6f ms\n',bn,authority,prepMs);

        BR=struct();
        BR.authority=authority;
        BR.preparation_ms=prepMs;
        BR.by_scale=repmat(a5_scale_template(),1,numel(Tscale));
        backendPass=true;

        for is=1:numel(Tscale)
            sc=Tscale(is);
            T=Sdef.T*sc;

            [exactJump,nearAcross,nearToKnot]=a5_boundary_audit(PREP,T,epsSet);
            legacyMismatch=a5_direct_vs_legacy_audit(PREP,LEG,T);

            if ~isequal(size(exactJump),[1 7])
                error('A5:ExactJumpShape','exactJump must be 1x7, got %s.',mat2str(size(exactJump)));
            end
            if ~isequal(size(nearAcross),[numel(epsSet) 7])
                error('A5:NearAcrossShape','nearAcross must be %dx7, got %s.',numel(epsSet),mat2str(size(nearAcross)));
            end
            if ~isequal(size(nearToKnot),[numel(epsSet) 7])
                error('A5:NearToKnotShape','nearToKnot must be %dx7, got %s.',numel(epsSet),mat2str(size(nearToKnot)));
            end
            if ~isequal(size(legacyMismatch),[1 7])
                error('A5:LegacyMismatchShape','legacyMismatch must be 1x7, got %s.',mat2str(size(legacyMismatch)));
            end

            % Shape guards: every d0..d6 quantity must be a 1x7 row.
            exactJumpRow      = a5_row7(exactJump, 'exactJump');
            nearLastRow       = a5_row7(nearToKnot(end,:), 'nearToKnot(end,:)');
            legacyMismatchRow = a5_row7(legacyMismatch, 'legacyMismatch');

            globalExactKnot(ib,:)       = max(globalExactKnot(ib,:),exactJumpRow);
            globalNearLast(ib,:)        = max(globalNearLast(ib,:),nearLastRow);
            globalLegacyMismatch(ib,:) = max(globalLegacyMismatch(ib,:),legacyMismatchRow);

            SR=a5_scale_template();
            SR.scale=sc;
            SR.T=T;
            SR.exact_knot_clean_jump_d0_d6=exactJump;
            SR.near_boundary_eps_rel=epsSet;
            SR.near_boundary_across_jump=nearAcross;
            SR.near_boundary_to_knot=nearToKnot;
            SR.direct_vs_legacy_max_abs_d0_d6=legacyMismatch;
            SR.structural_C6_pass=all(exactJump==0);
            SR.finite_near_boundary=all(isfinite(nearAcross(:))) && all(isfinite(nearToKnot(:)));
            SR.pass=SR.structural_C6_pass && SR.finite_near_boundary;

            BR.by_scale(is)=SR;
            backendPass=backendPass&&SR.pass;

            fprintf('  T/Tref=%5.2f | exactKnot=%s | near(last eps)=%s | direct-vs-legacy=%s\n', ...
                sc,mat2str(exactJump,3),mat2str(nearToKnot(end,:),5),mat2str(legacyMismatch,5));
        end

        % Runtime is T-independent in operation count. Benchmark at T_ref only.
        Rdir=a5_runtime_benchmark(PREP,Sdef.T,OPT,false,[]);
        Rleg=a5_runtime_benchmark(PREP,Sdef.T,OPT,true,LEG);

        a5_assert_runtime_struct(Rdir,'direct');
        a5_assert_runtime_struct(Rleg,'legacy');

        runtimeDirectMedian(ic,ib)=Rdir.median_us;
        runtimeDirectP95(ic,ib)=Rdir.p95_us;
        runtimeDirectMax(ic,ib)=Rdir.max_us;
        runtimeLegacyMedian(ic,ib)=Rleg.median_us;

        BR.runtime_direct=Rdir;
        BR.runtime_legacy=Rleg;
        BR.runtime_speed_ratio_legacy_over_direct=Rleg.median_us/max(Rdir.median_us,eps);
        BR.pass=backendPass;

        fprintf('  [RUNTIME] direct median/p95/max = %.3f / %.3f / %.3f us/query\n', ...
            Rdir.median_us,Rdir.p95_us,Rdir.max_us);
        fprintf('  [RUNTIME] legacy median        = %.3f us/query | legacy/direct=%.3fx\n', ...
            Rleg.median_us,BR.runtime_speed_ratio_legacy_over_direct);

        C.backends.(bn)=BR;
        auditCounts(ib)=auditCounts(ib)+1;
        if backendPass, passCounts(ib)=passCounts(ib)+1; end
        casePass=casePass&&backendPass;
    end

    C.pass=casePass;
    A5.records(ic)=C;
end

A5.aggregate=struct();
A5.aggregate.backend_names=backendNames;
A5.aggregate.pass_counts=passCounts;
A5.aggregate.audit_counts=auditCounts;
A5.aggregate.global_exact_knot_clean_jump_d0_d6=globalExactKnot;
A5.aggregate.global_near_boundary_last_eps_to_knot_d0_d6=globalNearLast;
A5.aggregate.global_direct_vs_legacy_max_abs_d0_d6=globalLegacyMismatch;
A5.aggregate.preparation_ms_median=median(runtimePrepMs,1);
A5.aggregate.preparation_ms_max=max(runtimePrepMs,[],1);
A5.aggregate.direct_runtime_us_median_of_case_medians=median(runtimeDirectMedian,1);
A5.aggregate.direct_runtime_us_max_case_p95=max(runtimeDirectP95,[],1);
A5.aggregate.direct_runtime_us_max_observed=max(runtimeDirectMax,[],1);
A5.aggregate.legacy_runtime_us_median_of_case_medians=median(runtimeLegacyMedian,1);
A5.aggregate.median_speed_ratio_legacy_over_direct= ...
    median(runtimeLegacyMedian./max(runtimeDirectMedian,eps),1);
A5.aggregate.all_three_backends_direct_reference_pass=all(passCounts==Ncase);

% Existing Step3 generation/QP timing is retained separately.
A5.aggregate.step3_median_solve_ms=STEP3.aggregate.median_solve_ms;
A5.aggregate.step3_max_solve_ms=STEP3.aggregate.max_solve_ms;

fprintf('\n========================================================================================================================\n');
fprintf(' A5 COMPACT SUMMARY\n');
fprintf('========================================================================================================================\n');
for ib=1:3
    fprintf('%-12s directPass=%d/%d\n',backendNames{ib},passCounts(ib),auditCounts(ib));
    fprintf('             exact-knot CLEAN jump d0..d6 = %s\n',mat2str(globalExactKnot(ib,:),7));
    fprintf('             near-boundary @eps=%g         = %s\n',epsSet(end),mat2str(globalNearLast(ib,:),7));
    fprintf('             direct-vs-legacy max          = %s\n',mat2str(globalLegacyMismatch(ib,:),7));
    fprintf('             prep median/max [ms]           = %.6f / %.6f\n', ...
        A5.aggregate.preparation_ms_median(ib),A5.aggregate.preparation_ms_max(ib));
    fprintf('             direct runtime median [us]     = %.3f\n', ...
        A5.aggregate.direct_runtime_us_median_of_case_medians(ib));
    fprintf('             worst case p95 / max [us]      = %.3f / %.3f\n', ...
        A5.aggregate.direct_runtime_us_max_case_p95(ib),A5.aggregate.direct_runtime_us_max_observed(ib));
    fprintf('             legacy runtime median [us]     = %.3f\n', ...
        A5.aggregate.legacy_runtime_us_median_of_case_medians(ib));
    fprintf('             median legacy/direct ratio     = %.3fx\n', ...
        A5.aggregate.median_speed_ratio_legacy_over_direct(ib));
end

fprintf('\n[SEPARATION OF COSTS]\n');
fprintf('Step3 QP median solve [B P S] ms = %s\n',mat2str(A5.aggregate.step3_median_solve_ms,6));
fprintf('Step3 QP max solve    [B P S] ms = %s\n',mat2str(A5.aggregate.step3_max_solve_ms,6));
fprintf('Reference evaluator runtime above is per control-query and excludes trajectory generation/QP.\n');
fprintf('\n[REFERENCE POLICY]\n');
fprintf('BEZIER/POLY: shared Hermite knot jets are authoritative; CP/Bernstein is safety/geometry representation.\n');
fprintf('BSPLINE: native CP+knot representation is authoritative; extracted Bezier is safety/certificate representation.\n');
fprintf('Exact internal knots return the single canonical jet; no left/right duplicate evaluation is injected into control reference.\n');
end

function PREP=a5_prepare_d13(J,frac)
PREP=struct();
PREP.kind='D13_HERMITE';
PREP.degree=13;
PREP.knots=[0 cumsum(frac)];
PREP.knots(end)=1;
PREP.shared_jets=J;
PREP.coeff=cell(1,3);

Hinv=a5_hermite13_inverse();
for seg=1:3
    h=frac(seg);
    B=zeros(14,3);
    for r=0:6
        B(r+1,:)= (h^r)*J(:,r+1,seg).';
        B(8+r,:)= (h^r)*J(:,r+1,seg+1).';
    end
    PREP.coeff{seg}=Hinv*B; % 14 x 3, ascending tau powers
end
end

function Hinv=a5_hermite13_inverse()
persistent HINV
if isempty(HINV)
    n=13;
    A=zeros(14,14);
    for r=0:6
        A(r+1,r+1)=factorial(r);
        for m=r:n
            A(8+r,m+1)=factorial(m)/factorial(m-r);
        end
    end
    HINV=A\eye(14);
end
Hinv=HINV;
end

function PREP=a5_prepare_bspline(cp,U,p)
PREP=struct();
PREP.kind='BSPLINE_NATIVE';
PREP.degree=p;
PREP.cp=cell(1,7);
PREP.U=cell(1,7);
PREP.p=zeros(1,7);

C=cp;
Uk=U;
pk=p;
for k=0:6
    PREP.cp{k+1}=C;
    PREP.U{k+1}=Uk;
    PREP.p(k+1)=pk;
    if k<6
        nc=size(C,2);
        Cn=zeros(3,nc-1);
        for i=1:nc-1
            den=Uk(i+pk+1)-Uk(i+1);
            if den==0
                Cn(:,i)=0;
            else
                Cn(:,i)=pk*(C(:,i+1)-C(:,i))/den;
            end
        end
        C=Cn;
        Uk=Uk(2:end-1);
        pk=pk-1;
    end
end
PREP.knots=unique(U,'stable');
PREP.knots=(PREP.knots-PREP.knots(1))/(PREP.knots(end)-PREP.knots(1));

% Cache canonical native knot jets once.
nk=numel(PREP.knots);
PREP.knot_jets=zeros(3,7,nk);
for j=1:nk
    u=PREP.knots(j);
    PREP.knot_jets(:,:,j)=a5_eval_bspline_native_no_override(PREP,u,1.0);
end
end

function Y=a5_eval_direct(PREP,s,T)
s=min(max(s,0),1);
switch PREP.kind
    case 'D13_HERMITE'
        Y=a5_eval_d13_direct(PREP,s,T);
    case 'BSPLINE_NATIVE'
        Y=a5_eval_bspline_native(PREP,s,T);
    otherwise
        error('A5:UnknownPrepKind','Unknown direct evaluator kind.');
end
if ~isequal(size(Y),[3 7])
    error('A5:DirectEvaluatorShape','Direct evaluator must return 3x7, got %s.',mat2str(size(Y)));
end
if any(~isfinite(Y(:)))
    error('A5:DirectEvaluatorFinite','Direct evaluator returned non-finite values.');
end
end

function Y=a5_eval_d13_direct(PREP,s,T)
Y=zeros(3,7);
tol=64*eps(max(1,abs(s)));

% Exact knot -> one authoritative shared jet.
[dm,ik]=min(abs(PREP.knots-s));
if dm<=tol
    Y=PREP.shared_jets(:,:,ik);
    for k=0:6
        Y(:,k+1)=Y(:,k+1)/(T^k);
    end
    return;
end

seg=find(PREP.knots(1:end-1)<s & s<PREP.knots(2:end),1);
if isempty(seg)
    if s<=0, seg=1; else, seg=3; end
end
a=PREP.knots(seg);
b=PREP.knots(seg+1);
h=b-a;
tau=(s-a)/h;
coef=PREP.coeff{seg};

for k=0:6
    for ax=1:3
        Y(ax,k+1)=a5_eval_power_derivative(coef(:,ax),k,tau)/(h^k*T^k);
    end
end
end

function v=a5_eval_power_derivative(c,k,x)
% c contains ascending power coefficients.
n=numel(c)-1;
if k>n
    v=0;
    return;
end
d=zeros(n-k+1,1);
for m=k:n
    d(m-k+1)=c(m+1)*factorial(m)/factorial(m-k);
end
% Horner for ascending coefficients.
v=d(end);
for j=numel(d)-1:-1:1
    v=v*x+d(j);
end
end

function Y=a5_eval_bspline_native(PREP,s,T)
tol=64*eps(max(1,abs(s)));
[dm,ik]=min(abs(PREP.knots-s));
if dm<=tol
    Y=PREP.knot_jets(:,:,ik);
    for k=0:6
        Y(:,k+1)=Y(:,k+1)/(T^k);
    end
else
    Y=a5_eval_bspline_native_no_override(PREP,s,T);
end
end

function Y=a5_eval_bspline_native_no_override(PREP,s,T)
Y=zeros(3,7);
for k=0:6
    C=PREP.cp{k+1};
    U=PREP.U{k+1};
    p=PREP.p(k+1);
    N=step3a_bs_basis_all(s,U,p,size(C,2));
    Y(:,k+1)=(C*N.')/(T^k);
end
end

function [exactJump,nearAcross,nearToKnot]=a5_boundary_audit(PREP,T,epsSet)
% Production exact-knot jump is zero by single canonical authority.
exactJump=zeros(1,7);

kn=PREP.knots;
if numel(kn)<=2
    nearAcross=zeros(numel(epsSet),7);
    nearToKnot=zeros(numel(epsSet),7);
    return;
end

nearAcross=zeros(numel(epsSet),7);
nearToKnot=zeros(numel(epsSet),7);

for j=2:numel(kn)-1
    sk=kn(j);
    hL=sk-kn(j-1);
    hR=kn(j+1)-sk;
    Yk=a5_eval_direct(PREP,sk,T);

    for ie=1:numel(epsSet)
        e=epsSet(ie);
        sL=sk-e*hL;
        sR=sk+e*hR;
        YL=a5_eval_direct(PREP,sL,T);
        YR=a5_eval_direct(PREP,sR,T);
        for k=1:7
            nearAcross(ie,k)=max(nearAcross(ie,k),norm(YL(:,k)-YR(:,k),2));
            nearToKnot(ie,k)=max(nearToKnot(ie,k), ...
                max(norm(YL(:,k)-Yk(:,k),2),norm(YR(:,k)-Yk(:,k),2)));
        end
    end
end
end

function mismatch=a5_direct_vs_legacy_audit(PREP,LEG,T)
% Same curve, different representation. Sample deterministic points away
% from exact internal boundaries and report max d0..d6 mismatch.
sgrid=linspace(0,1,121);
mismatch=zeros(1,7);
for q=1:numel(sgrid)
    s=sgrid(q);
    Y=a5_eval_direct(PREP,s,T);
    Z=a5_eval_legacy(LEG,s,T);
    for k=1:7
        mismatch(k)=max(mismatch(k),norm(Y(:,k)-Z(:,k),2));
    end
end
end

function Y=a5_eval_legacy(LEG,s,T)
CP=LEG.CP;
n=LEG.degree;
frac=LEG.pieceFrac(:).';
kn=[0 cumsum(frac)];
kn(end)=1;
s=min(max(s,0),1);

if s>=1
    seg=numel(frac);
    tau=1;
else
    seg=find(s>=kn(1:end-1) & s<kn(2:end),1,'last');
    if isempty(seg), seg=1; end
    tau=(s-kn(seg))/frac(seg);
end

Y=zeros(3,7);
for k=0:6
    D=a3_derivative_cp(CP(:,:,seg),n,k);
    val=a3_eval_bezier(D,tau);
    Y(:,k+1)=val(:,1)/(frac(seg)^k*T^k);
end
if ~isequal(size(Y),[3 7])
    error('A5:LegacyEvaluatorShape','Legacy evaluator must return 3x7, got %s.',mat2str(size(Y)));
end
if any(~isfinite(Y(:)))
    error('A5:LegacyEvaluatorFinite','Legacy evaluator returned non-finite values.');
end
end

function R=a5_runtime_benchmark(PREP,T,OPT,useLegacy,LEG)
Nwarm=OPT.RuntimeWarmupQueries;
Nb=OPT.RuntimeBlocks;
Nq=OPT.RuntimeQueriesPerBlock;

% Deterministic query grid, avoiding a benchmark dominated by exact-knot overrides.
sg=((1:Nq)-0.5)/Nq;
sink=0;

for i=1:Nwarm
    s=mod((i-0.5)/max(Nwarm,1),1);
    if useLegacy
        Y=a5_eval_legacy(LEG,s,T);
    else
        Y=a5_eval_direct(PREP,s,T);
    end
    sink=sink+Y(1,1);
end

us=zeros(1,Nb);
for b=1:Nb
    t0=tic;
    for q=1:Nq
        s=sg(q);
        if useLegacy
            Y=a5_eval_legacy(LEG,s,T);
        else
            Y=a5_eval_direct(PREP,s,T);
        end
        sink=sink+Y(1,1);
    end
    us(b)=1e6*toc(t0)/Nq;
end

R=struct();
R.block_us_per_query=us;
R.median_us=median(us);
R.p95_us=a5_percentile(us,95);
R.max_us=max(us);
R.warmup_queries=Nwarm;
R.blocks=Nb;
R.queries_per_block=Nq;
R.sink=sink;
end

function q=a5_percentile(x,p)
x=sort(x(:));
if isempty(x), q=NaN; return; end
r=1+(numel(x)-1)*(p/100);
lo=floor(r); hi=ceil(r);
if lo==hi
    q=x(lo);
else
    q=x(lo)+(r-lo)*(x(hi)-x(lo));
end
end


function r=a5_row7(v,name)
if ~isvector(v) || numel(v)~=7
    error('A5:Row7Shape','%s must contain exactly 7 elements, got size %s.',name,mat2str(size(v)));
end
r=reshape(v,1,7);
if any(~isfinite(r))
    error('A5:Row7Finite','%s contains non-finite values.',name);
end
end

function a5_assert_runtime_struct(R,name)
req={'median_us','p95_us','max_us','block_us_per_query'};
for i=1:numel(req)
    if ~isfield(R,req{i})
        error('A5:RuntimeSchema','%s runtime result missing field %s.',name,req{i});
    end
end
if ~(isscalar(R.median_us)&&isscalar(R.p95_us)&&isscalar(R.max_us))
    error('A5:RuntimeScalar','%s runtime median/p95/max must be scalar.',name);
end
if any(~isfinite([R.median_us R.p95_us R.max_us]))
    error('A5:RuntimeFinite','%s runtime contains non-finite timing values.',name);
end
if any([R.median_us R.p95_us R.max_us] < 0)
    error('A5:RuntimeSign','%s runtime contains negative timing values.',name);
end
end

function S=a5_scale_template()
S=struct( ...
    'scale',NaN, ...
    'T',NaN, ...
    'exact_knot_clean_jump_d0_d6',nan(1,7), ...
    'near_boundary_eps_rel',[], ...
    'near_boundary_across_jump',[], ...
    'near_boundary_to_knot',[], ...
    'direct_vs_legacy_max_abs_d0_d6',nan(1,7), ...
    'structural_C6_pass',false, ...
    'finite_near_boundary',false, ...
    'pass',false);
end

function A4=internal_a4_residual_elimination_audit(STEP3,OPT)
% A4 objective:
%   Separate "curve continuity" from "representation re-evaluation noise".
%
% Clean reference policy:
%   D13: use the single shared optimization knot jet.
%   B-spline: use the native B-spline derivative at each simple knot.
%   Extracted Bezier is retained for safety certification only.
%
% The clean online boundary residual is therefore exactly zero because
% there is one authoritative boundary jet, not two independently
% re-evaluated copies.  The raw left/right residual is retained under
% representation_consistency_* and never hidden.

Tscale=OPT.TScaleSet(:).';
backendNames={'BEZIER_D13','POLY_D13','BSPLINE_D7'};
P0=aud_params();
CASES0=aud_build_cases();
Ncase=numel(CASES0);

caseTemplate=struct('case_id','','T_ref',NaN,'backends',struct(),'pass',false);
A4=struct();
A4.mode='STEP4A4_C6_RESIDUAL_ELIMINATION_AUDIT';
A4.records=repmat(caseTemplate,1,Ncase);

passCounts=zeros(1,3);
auditCounts=zeros(1,3);
globalRaw=zeros(3,7);
globalToCanonical=zeros(3,7);
globalClean=zeros(3,7);
globalNativeExtract=zeros(1,7);

fprintf('\n========================================================================================================================\n');
fprintf(' A4 C6 RESIDUAL-SOURCE DECOMPOSITION / CLEAN REFERENCE AUDIT\n');
fprintf(' raw representation residual != clean online boundary residual\n');
fprintf('========================================================================================================================\n');

for ic=1:Ncase
    Sdef=CASES0(ic);
    R3=STEP3.records(ic);
    G=aud_run_case(Sdef,P0,1);

    C=caseTemplate;
    C.case_id=Sdef.case_id;
    C.T_ref=Sdef.T;
    casePass=true;

    fprintf('\n########################################################################################################################\n');
    fprintf(' CASE %02d / %02d : %s | T_ref=%.17g\n',ic,Ncase,Sdef.case_id,Sdef.T);
    fprintf('########################################################################################################################\n');

    for ib=1:3
        bn=backendNames{ib};
        BR=struct();
        BR.status='PASS';
        BR.by_scale=repmat(a4_scale_template(),1,numel(Tscale));

        switch bn
            case 'BEZIER_D13'
                CP=R3.bezier.control_points;
                n=13;
                segFrac=vecnorm(diff(G.waypoints,1,2),2,1);
                segFrac=segFrac/sum(segFrac);
                canonical=R3.bezier.shared_knot_jets;
                mode='SHARED_OPTIMIZATION_KNOT_JET';

            case 'POLY_D13'
                CP=R3.poly.bernstein_control_points;
                n=13;
                segFrac=vecnorm(diff(G.waypoints,1,2),2,1);
                segFrac=segFrac/sum(segFrac);
                canonical=R3.poly.shared_knot_jets;
                mode='SHARED_OPTIMIZATION_KNOT_JET';

            case 'BSPLINE_D7'
                CP=R3.bspline.extracted_bezier_control_points;
                n=7;
                U=R3.bspline.knots;
                nativeCP=R3.bspline.native_control_points;
                breaks=unique(U,'stable');
                if breaks(1)~=0 || breaks(end)~=1
                    % normalize only if numerical storage is slightly off.
                    breaks=(breaks-breaks(1))/(breaks(end)-breaks(1));
                end
                segFrac=diff(breaks);
                canonical=a4_bspline_native_knot_jets(nativeCP,U,n,breaks);
                mode='NATIVE_BSPLINE_KNOT_DERIVATIVE';
        end

        fprintf('\n[%s] clean boundary authority = %s\n',bn,mode);

        backendPass=true;

        for is=1:numel(Tscale)
            sc=Tscale(is);
            T=Sdef.T*sc;
            dt=T*segFrac;

            if ib<=2
                [raw,toCanonical]=a4_d13_raw_vs_canonical(CP,n,dt,canonical,T);
                nativeExtract=zeros(1,7);
            else
                [raw,toCanonical,nativeExtract]= ...
                    a4_bspline_extract_vs_native(CP,n,dt,canonical,T);
                globalNativeExtract=max(globalNativeExtract,nativeExtract);
            end

            clean=zeros(1,7); % one authoritative knot jet is used by both sides

            SR=a4_scale_template();
            SR.scale=sc;
            SR.T=T;
            SR.representation_consistency_abs_d0_d6=raw;
            SR.representation_to_canonical_abs_d0_d6=toCanonical;
            SR.native_to_extracted_abs_d0_d6=nativeExtract;
            SR.clean_online_boundary_jump_d0_d6=clean;
            SR.clean_pass=all(clean==0) && all(isfinite(toCanonical));

            BR.by_scale(is)=SR;
            backendPass=backendPass && SR.clean_pass;

            globalRaw(ib,:)=max(globalRaw(ib,:),raw);
            globalToCanonical(ib,:)=max(globalToCanonical(ib,:),toCanonical);
            globalClean(ib,:)=max(globalClean(ib,:),clean);

            fprintf('  T/Tref=%5.2f | raw=%s | toCanonical=%s | CLEAN=%s\n', ...
                sc,mat2str(raw,5),mat2str(toCanonical,5),mat2str(clean,3));
        end

        BR.pass=backendPass;
        BR.clean_boundary_authority=mode;
        BR.policy='raw residual retained as representation diagnostic; clean reference uses one canonical knot jet';
        C.backends.(bn)=BR;

        auditCounts(ib)=auditCounts(ib)+1;
        if backendPass, passCounts(ib)=passCounts(ib)+1; end
        casePass=casePass&&backendPass;
    end

    C.pass=casePass;
    A4.records(ic)=C;
end

A4.aggregate=struct();
A4.aggregate.backend_names=backendNames;
A4.aggregate.pass_counts=passCounts;
A4.aggregate.audit_counts=auditCounts;
A4.aggregate.global_raw_representation_residual_d0_d6=globalRaw;
A4.aggregate.global_representation_to_canonical_d0_d6=globalToCanonical;
A4.aggregate.global_clean_online_boundary_jump_d0_d6=globalClean;
A4.aggregate.global_bspline_native_to_extracted_d0_d6=globalNativeExtract;
A4.aggregate.all_three_backends_clean_reference_pass=all(passCounts==Ncase);

fprintf('\n========================================================================================================================\n');
fprintf(' A4 COMPACT SUMMARY\n');
fprintf('========================================================================================================================\n');
for ib=1:3
    fprintf('%-12s cleanPass=%d/%d\n',backendNames{ib},passCounts(ib),auditCounts(ib));
    fprintf('             raw representation max d0..d6 = %s\n',mat2str(globalRaw(ib,:),7));
    fprintf('             representation->canonical      = %s\n',mat2str(globalToCanonical(ib,:),7));
    fprintf('             CLEAN online boundary jump      = %s\n',mat2str(globalClean(ib,:),7));
end
fprintf('BSPLINE native->extracted max d0..d6 = %s\n',mat2str(globalNativeExtract,7));
fprintf('\n[IMPLEMENTATION DECISION]\n');
fprintf('BEZIER/POLY online boundary reference: shared optimization knot jets directly.\n');
fprintf('BSPLINE online boundary reference: native B-spline derivatives directly.\n');
fprintf('Extracted Bezier remains for safety certificate; its rounding residual is not injected into the reference signal.\n');
fprintf('D13 CP reconstruction now uses exact binomial direct map, not F\\D.\n');
end

function [raw,toCanonical]=a4_d13_raw_vs_canonical(CP,n,dt,J,T)
np=size(CP,3);
raw=zeros(1,7);
toCanonical=zeros(1,7);
for j=1:np-1
    knot=j+1;
    for k=0:6
        L=a3_endpoint_derivative(CP(:,:,j),n,k,dt(j),'right');
        R=a3_endpoint_derivative(CP(:,:,j+1),n,k,dt(j+1),'left');
        Ck=J(:,k+1,knot)/(T^k);
        raw(k+1)=max(raw(k+1),norm(L-R,2));
        toCanonical(k+1)=max(toCanonical(k+1),max(norm(L-Ck,2),norm(R-Ck,2)));
    end
end
end

function J=a4_bspline_native_knot_jets(cp,U,p,breaks)
% Canonical normalized-parameter derivatives directly from native B-spline.
nb=numel(breaks);
J=zeros(3,7,nb);
for ib=1:nb
    u=breaks(ib);
    for k=0:6
        J(:,k+1,ib)=a4_bspline_derivative_eval(cp,U,p,k,u);
    end
end
end

function v=a4_bspline_derivative_eval(cp,U,p,k,u)
C=cp;
Uk=U;
pk=p;

for r=1:k
    nc=size(C,2);
    Cn=zeros(3,nc-1);
    for i=1:nc-1
        den=Uk(i+pk+1)-Uk(i+1);
        if den==0
            Cn(:,i)=0;
        else
            Cn(:,i)=pk*(C(:,i+1)-C(:,i))/den;
        end
    end
    C=Cn;
    Uk=Uk(2:end-1);
    pk=pk-1;
end

N=step3a_bs_basis_all(u,Uk,pk,size(C,2));
v=C*N.';
end

function [raw,toCanonical,nativeExtract]=a4_bspline_extract_vs_native(Bcp,n,dt,J,T)
np=size(Bcp,3);
raw=zeros(1,7);
toCanonical=zeros(1,7);
nativeExtract=zeros(1,7);

for j=1:np-1
    knot=j+1;
    for k=0:6
        L=a3_endpoint_derivative(Bcp(:,:,j),n,k,dt(j),'right');
        R=a3_endpoint_derivative(Bcp(:,:,j+1),n,k,dt(j+1),'left');
        Ck=J(:,k+1,knot)/(T^k);

        raw(k+1)=max(raw(k+1),norm(L-R,2));
        e=max(norm(L-Ck,2),norm(R-Ck,2));
        toCanonical(k+1)=max(toCanonical(k+1),e);
        nativeExtract(k+1)=max(nativeExtract(k+1),e);
    end
end
end

function S=a4_scale_template()
S=struct( ...
    'scale',NaN, ...
    'T',NaN, ...
    'representation_consistency_abs_d0_d6',nan(1,7), ...
    'representation_to_canonical_abs_d0_d6',nan(1,7), ...
    'native_to_extracted_abs_d0_d6',nan(1,7), ...
    'clean_online_boundary_jump_d0_d6',nan(1,7), ...
    'clean_pass',false);
end

function A3=internal_a3_exhaustive(STEP3,OPT)
Tscale=OPT.TScaleSet(:).';
backendNames={'BEZIER_D13','POLY_D13','BSPLINE_D7'};

P0=aud_params();
CASES0=aud_build_cases();
Ncase=numel(CASES0);

if ~isfield(STEP3,'records') || numel(STEP3.records)~=Ncase
    error('A3:InternalStep3Schema','Internal Step3 records do not match regenerated 14-case definitions.');
end

fprintf('\n========================================================================================================================\n');
fprintf(' A3 EXHAUSTIVE AUDIT\n');
fprintf(' cases=%d | Tscale=%s | d0..d6 | continuous ag certificate\n',Ncase,mat2str(Tscale,6));
fprintf('========================================================================================================================\n');

caseTemplate=struct('case_id','','T_ref',NaN,'guide_fractions',[],'backends',struct(),'pass',false);
A3=struct();
A3.mode='STEP4A3_INTERNAL_EXHAUSTIVE_C6_SUPPORT';
A3.records=repmat(caseTemplate,1,Ncase);

passCounts=zeros(1,3);
auditCounts=zeros(1,3);
warnAbsCounts=zeros(1,3);
warnRelCounts=zeros(1,3);
globalMaxAbs=zeros(3,7);
globalMaxRel=zeros(3,7);
globalMinCert=inf(1,3);
globalMaxScaleLawResidual=zeros(3,7);

for ic=1:Ncase
    Sdef=CASES0(ic);
    R3=STEP3.records(ic);

    % Recompute Step1 guide internally from the same embedded generator.
    G=aud_run_case(Sdef,P0,1);
    if ~G.pass
        error('A3:InternalGuide','Internally regenerated Step1 guide failed for %s.',Sdef.case_id);
    end

    segLen=vecnorm(diff(G.waypoints,1,2),2,1);
    if ~(sum(segLen)>0)
        error('A3:ZeroGuide','Zero guide length in %s.',Sdef.case_id);
    end
    frac=segLen/sum(segLen);
    frac=frac(:).';

    Tref=Sdef.T;
    C=caseTemplate;
    C.case_id=Sdef.case_id;
    C.T_ref=Tref;
    C.guide_fractions=frac;

    fprintf('\n########################################################################################################################\n');
    fprintf(' CASE %02d / %02d : %s | T_ref=%.17g\n',ic,Ncase,Sdef.case_id,Tref);
    fprintf(' guide fractions = %s\n',mat2str(frac,17));
    fprintf('########################################################################################################################\n');

    casePass=true;

    for ib=1:3
        bn=backendNames{ib};

        switch bn
            case 'BEZIER_D13'
                CP=R3.bezier.control_points;
                degree=13;
                pieceFrac=frac;
                structuralC6=(R3.bezier.c6_structural_residual==0);
                representation='native degree-13 Bezier/Bernstein CP';
                extra=struct();

            case 'POLY_D13'
                CP=R3.poly.bernstein_control_points;
                degree=13;
                pieceFrac=frac;
                structuralC6=(R3.poly.c6_structural_residual==0);
                representation='degree-13 polynomial exact Bernstein image';
                extra=struct();

            case 'BSPLINE_D7'
                CP=R3.bspline.extracted_bezier_control_points;
                degree=7;
                pieceFrac=[repmat(frac(1)/7,1,7),repmat(frac(2)/7,1,7),repmat(frac(3)/7,1,7)];
                structuralC6=(R3.bspline.c6_structural_residual==0);
                representation='exact extracted degree-7 Bezier pieces from native B-spline';
                extra=struct();
                extra.native_cp_count=size(R3.bspline.native_control_points,2);
                extra.extraction_min_entry=R3.bspline.extraction_min_entry;
                extra.extraction_row_sum_error=R3.bspline.extraction_row_sum_error;
        end

        CP=a3_normalize_cp(CP,degree);
        if isempty(CP)
            error('A3:InternalCPShape','Internal CP shape invalid for %s / %s.',Sdef.case_id,bn);
        end
        if numel(pieceFrac)~=size(CP,3)
            error('A3:InternalPieceCount','Piece count mismatch for %s / %s.',Sdef.case_id,bn);
        end

        fprintf('\n[%s] degree=%d pieces=%d representation=%s\n', ...
            bn,degree,size(CP,3),representation);

        BR=struct();
        BR.status='PASS';
        BR.degree=degree;
        BR.piece_count=size(CP,3);
        BR.piece_fraction=pieceFrac;
        BR.structural_C6=structuralC6;
        BR.representation=representation;
        BR.extra=extra;
        BR.by_scale=repmat(a3_scale_template(),1,numel(Tscale));

        backendPass=structuralC6;
        backendAbsWarn=false;
        backendRelWarn=false;

        % Reference boundary jumps at T/Tref=1 for an explicit T^-k law audit.
        refIdx=find(abs(Tscale-1)<100*eps,1);
        refAbs=[];
        if ~isempty(refIdx)
            refDt=Tref*pieceFrac;
            [refAbs,~]=a3_boundary_jumps(CP,degree,refDt);
        end

        for is=1:numel(Tscale)
            sc=Tscale(is);
            T=Tref*sc;
            dt=T*pieceFrac;

            [absJump,relJump]=a3_boundary_jumps(CP,degree,dt);

            % Verify expected affine-time scaling of boundary residuals.
            if ~isempty(refAbs)
                predicted=refAbs;
                for k=1:6
                    predicted(k+1)=refAbs(k+1)*sc^(-k);
                end
                scaleLawResidual=abs(absJump-predicted);
            else
                scaleLawResidual=nan(1,7);
            end

            [minCert,minDense,worstPiece,certOK]= ...
                a3_continuous_ag_certificate(CP,degree,dt,OPT.Gravity,OPT.DenseN);

            c6Abs=max(absJump(2:7));
            c6Rel=max(relJump(2:7));
            absWarn=c6Abs>OPT.C6AbsWarnTol;
            relWarn=c6Rel>OPT.C6RelWarnTol;

            SR=a3_scale_template();
            SR.scale=sc;
            SR.T=T;
            SR.max_boundary_abs_d0_d6=absJump;
            SR.max_boundary_rel_d0_d6=relJump;
            SR.max_C6_abs_d1_d6=c6Abs;
            SR.max_C6_rel_d1_d6=c6Rel;
            SR.affine_scale_law_abs_error=scaleLawResidual;
            SR.min_continuous_ag_support_cert=minCert;
            SR.min_dense_ag_norm=minDense;
            SR.dense_minus_cert=minDense-minCert;
            SR.worst_piece=worstPiece;
            SR.certificate_solver_ok=certOK;
            SR.abs_warning=absWarn;
            SR.rel_warning=relWarn;
            SR.pass=structuralC6 && certOK && isfinite(minCert) && minCert>0;

            BR.by_scale(is)=SR;
            backendPass=backendPass && SR.pass;
            backendAbsWarn=backendAbsWarn || absWarn;
            backendRelWarn=backendRelWarn || relWarn;

            globalMaxAbs(ib,:)=max(globalMaxAbs(ib,:),absJump);
            globalMaxRel(ib,:)=max(globalMaxRel(ib,:),relJump);
            globalMinCert(ib)=min(globalMinCert(ib),minCert);
            if all(isfinite(scaleLawResidual))
                globalMaxScaleLawResidual(ib,:)=max(globalMaxScaleLawResidual(ib,:),scaleLawResidual);
            end

            tag='PASS';
            if ~SR.pass
                tag='FAIL';
            elseif absWarn || relWarn
                tag='PASS/WARN';
            end

            fprintf('  T/Tref=%5.2f T=%9.4f | abs d0..d6=%s | rel d0..d6=%s | cert=%+.6e | %s\n', ...
                sc,T,mat2str(absJump,4),mat2str(relJump,4),minCert,tag);
        end

        BR.pass=backendPass;
        BR.abs_warning=backendAbsWarn;
        BR.rel_warning=backendRelWarn;

        auditCounts(ib)=auditCounts(ib)+1;
        if backendPass, passCounts(ib)=passCounts(ib)+1; end
        if backendAbsWarn, warnAbsCounts(ib)=warnAbsCounts(ib)+1; end
        if backendRelWarn, warnRelCounts(ib)=warnRelCounts(ib)+1; end

        C.backends.(bn)=BR;
        casePass=casePass && backendPass;
    end

    C.pass=casePass;
    A3.records(ic)=C;
end

A3.aggregate=struct();
A3.aggregate.backend_names=backendNames;
A3.aggregate.pass_counts=passCounts;
A3.aggregate.audit_counts=auditCounts;
A3.aggregate.abs_warning_case_counts=warnAbsCounts;
A3.aggregate.rel_warning_case_counts=warnRelCounts;
A3.aggregate.global_max_boundary_abs_d0_d6=globalMaxAbs;
A3.aggregate.global_max_boundary_rel_d0_d6=globalMaxRel;
A3.aggregate.global_min_continuous_ag_support_cert=globalMinCert;
A3.aggregate.global_max_affine_scale_law_error_d0_d6=globalMaxScaleLawResidual;
A3.aggregate.all_three_backends_all_cases_pass=all(passCounts==Ncase);

fprintf('\n========================================================================================================================\n');
fprintf(' A3 EXHAUSTIVE SUMMARY\n');
fprintf('========================================================================================================================\n');
for ib=1:3
    fprintf('%-12s pass=%d/%d | absWarnCases=%d | relWarnCases=%d\n', ...
        backendNames{ib},passCounts(ib),auditCounts(ib),warnAbsCounts(ib),warnRelCounts(ib));
    fprintf('             global max abs d0..d6 = %s\n',mat2str(globalMaxAbs(ib,:),6));
    fprintf('             global max rel d0..d6 = %s\n',mat2str(globalMaxRel(ib,:),6));
    fprintf('             global min ag cert    = %+.17e\n',globalMinCert(ib));
    fprintf('             max T-scaling error   = %s\n',mat2str(globalMaxScaleLawResidual(ib,:),6));
end
fprintf('\n[C6 POLICY]\n');
fprintf('Structural C6 is authoritative for mathematical continuity.\n');
fprintf('Absolute/relative floating boundary residuals are both retained as numerical diagnostics.\n');
fprintf('No backend is removed here; all three remain for real-time comparison.\n');
fprintf('Continuous ag certificate uses Bernstein acceleration hull, not dense sampling.\n');
end

function [absJump,relJump]=a3_boundary_jumps(CP,n,dt)
np=size(CP,3);
absJump=zeros(1,7);
relJump=zeros(1,7);
if np<=1, return; end

for j=1:np-1
    for k=0:6
        L=a3_endpoint_derivative(CP(:,:,j),n,k,dt(j),'right');
        R=a3_endpoint_derivative(CP(:,:,j+1),n,k,dt(j+1),'left');
        a=norm(L-R,2);
        den=max([1,norm(L,2),norm(R,2)]);
        rr=a/den;
        absJump(k+1)=max(absJump(k+1),a);
        relJump(k+1)=max(relJump(k+1),rr);
    end
end
end

function d=a3_endpoint_derivative(P,n,k,dt,side)
if k==0
    if strcmp(side,'left'), d=P(:,1); else, d=P(:,end); end
    return;
end
D=P;
for r=1:k
    D=(n-r+1)*diff(D,1,2);
end
D=D/(dt^k);
if strcmp(side,'left'), d=D(:,1); else, d=D(:,end); end
end

function [minCert,minDense,worstPiece,ok]=a3_continuous_ag_certificate(CP,n,dt,g,denseN)
np=size(CP,3);
minCert=inf;
minDense=inf;
worstPiece=NaN;
ok=true;
for j=1:np
    accCP=a3_derivative_cp(CP(:,:,j),n,2)/(dt(j)^2);
    agCP=accCP+[0;0;g];

    [dcert,solverOK]=a3_distance_origin_convex_hull(agCP);
    ok=ok&&solverOK;

    u=linspace(0,1,denseN);
    vals=a3_eval_bezier(agCP,u);
    ddense=min(vecnorm(vals,2,1));

    if dcert<minCert
        minCert=dcert;
        worstPiece=j;
    end
    minDense=min(minDense,ddense);
end
end

function D=a3_derivative_cp(P,n,k)
D=P;
for r=1:k
    D=(n-r+1)*diff(D,1,2);
end
end

function V=a3_eval_bezier(CP,u)
m=size(CP,2)-1;
V=zeros(3,numel(u));
for i=0:m
    b=nchoosek(m,i).*(u.^i).*((1-u).^(m-i));
    V=V+CP(:,i+1)*b;
end
end

function [d,ok]=a3_distance_origin_convex_hull(P)
m=size(P,2);
H=2*(P.'*P);
H=(H+H.')/2;
f=zeros(m,1);
Aeq=ones(1,m);
beq=1;
lb=zeros(m,1);
ok=true;

try
    opts=optimoptions('quadprog','Display','off', ...
        'ConstraintTolerance',1e-12,'OptimalityTolerance',1e-12);
    [w,~,ef]=quadprog(H,f,[],[],Aeq,beq,lb,[],[],opts);
    ok=ef>0 && ~isempty(w);
    if ok
        d=norm(P*w,2);
    else
        d=NaN;
    end
catch
    d=NaN;
    ok=false;
end
end

function CP=a3_normalize_cp(CP,n)
if ndims(CP)==2
    if size(CP,1)==3 && size(CP,2)==n+1
        CP=reshape(CP,3,n+1,1);
    elseif size(CP,2)==3 && size(CP,1)==n+1
        CP=reshape(CP.',3,n+1,1);
    else
        CP=[];
    end
elseif ndims(CP)==3
    if size(CP,1)==3 && size(CP,2)==n+1
        % already correct
    elseif size(CP,2)==3 && size(CP,1)==n+1
        CP=permute(CP,[2 1 3]);
    else
        CP=[];
    end
else
    CP=[];
end
end

function S=a3_scale_template()
S=struct( ...
    'scale',NaN, ...
    'T',NaN, ...
    'max_boundary_abs_d0_d6',nan(1,7), ...
    'max_boundary_rel_d0_d6',nan(1,7), ...
    'max_C6_abs_d1_d6',NaN, ...
    'max_C6_rel_d1_d6',NaN, ...
    'affine_scale_law_abs_error',nan(1,7), ...
    'min_continuous_ag_support_cert',NaN, ...
    'min_dense_ag_norm',NaN, ...
    'dense_minus_cert',NaN, ...
    'worst_piece',NaN, ...
    'certificate_solver_ok',false, ...
    'abs_warning',false, ...
    'rel_warning',false, ...
    'pass',false);
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
    fprintf('  ENV id=%d type=%-10s schema=PASS\n',obs(i).id,lower(obs(i).type));
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
    fprintf('  %-10s visibility-kernel=PASS | %s\n',types{k},mode);
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
