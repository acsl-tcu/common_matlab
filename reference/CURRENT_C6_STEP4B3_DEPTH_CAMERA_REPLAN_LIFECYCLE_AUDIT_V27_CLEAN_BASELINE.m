function OUT = CURRENT_C6_STEP4B3_DEPTH_CAMERA_REPLAN_LIFECYCLE_AUDIT_V27_CLEAN_BASELINE(userCfg)
% CURRENT_C6_STEP4B3_DEPTH_CAMERA_REPLAN_LIFECYCLE_AUDIT_V27_CLEAN_BASELINE
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
%   R = CURRENT_C6_STEP4B3_DEPTH_CAMERA_REPLAN_LIFECYCLE_AUDIT_V27_CLEAN_BASELINE();
%
% Optional:
%   cfg = struct('nominal_speed',1.2,'mission_distance',50);
%   R = CURRENT_C6_STEP4B3_DEPTH_CAMERA_REPLAN_LIFECYCLE_AUDIT_V27_CLEAN_BASELINE(cfg);

if nargin < 1 || isempty(userCfg)
    userCfg=struct();
end

P=aud_params();
CFG=b3_default_config(P);
CFG=b3_merge_config(CFG,userCfg);
b3_validate_config(CFG,P);
b3_detail_fprintf('__SET__',~CFG.compact_log);
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
fprintf('[BASELINE V27] observed-only depth memory | PAYLOAD/UAV/CABLE safety | C6 | spatial rejoin | recursive replanning | 3 backends.\n');
fprintf('[BASELINE V27] retained validated mechanisms: support-certified corridors, strict-interior recovery, backend-specific C6 seeds, compact display gate.\n');
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
OUT.mode='STEP4B3_DEPTH_CAMERA_REPLAN_LIFECYCLE_AUDIT_V27_CLEAN_BASELINE';
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

assignin('base','c6_step4b3_depth_camera_replan_lifecycle_v27_clean_baseline_result',OUT);
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
CFG.compact_log=true;
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
    [G,C,T,geomDiag]=b3_build_replan_geometry(backend,t0,Dstart,obs0,risk.ids,CFG,P);
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
b3_detail_fprintf('    [GEOM] W0=%s\n',mat2str(G.waypoints(:,1).',9));
b3_detail_fprintf('    [GEOM] W1=%s\n',mat2str(G.waypoints(:,2).',9));
b3_detail_fprintf('    [GEOM] W2=%s\n',mat2str(G.waypoints(:,3).',9));
b3_detail_fprintf('    [GEOM] W3=%s\n',mat2str(G.waypoints(:,4).',9));
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
b3_detail_fprintf('    [KIN-D1..D6] upperBounds=%s\n',mat2str(K.derivative_norm_upper_bound,9));
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

            b3_detail_fprintf(['      [WITNESS-CERT] cell=%d obs=%d comp=%-7s ', ...
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

            b3_detail_fprintf(['      [COMP-CERT] cell=%d obs=%d comp=%-7s source=SOCP exit=%g ', ...
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
