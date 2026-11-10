function REPORT = run_VERTICAL_CBF_DUAL_INPUT_COMPARISON(cfg)
% Equal-condition comparison: FOUR system geometries x TWO input spaces.
% Default: 8 independent runs (S1..S4 x virtual/physical), straight line.
% Optional: cfg.referenceModes=[1 2 3] for 24 runs including C6 references.
% NO generator execution, NO reference/HLC/EKF/plant modification.
% Figure 1: 2-by-2 3D system geometry comparisons; display only, no images.
% One fixed results ZIP in common_matlab; temporary experiment files removed.
% No numerical outcome by itself proves sampled-data CBF safety.
if nargin<1 || isempty(cfg),cfg=struct();end
assert(isstruct(cfg) && isscalar(cfg),'CBF:InvalidStudyConfiguration', ...
    'Optional input must be a scalar struct.');
if ~isfield(cfg,'geometryMethods') || isempty(cfg.geometryMethods)
    cfg.geometryMethods={'S1','S2','S3','S4'};
end
if ischar(cfg.geometryMethods) || isstring(cfg.geometryMethods)
    cfg.geometryMethods=cellstr(cfg.geometryMethods);
end
methods=cellfun(@(v)upper(char(v)),cfg.geometryMethods(:).','UniformOutput',false);
assert(numel(unique(methods))==numel(methods) && ...
    all(ismember(methods,{'S1','S2','S3','S4'})), ...
    'CBF:InvalidGeometries','Expected unique methods in S1,S2,S3,S4.');
% Offline experimental backend. 'zoh': input-hold consistent prediction;
% 'legacy': original continuous-time high-order CBF implementation.
if ~isfield(cfg,'safetyBackend') || isempty(cfg.safetyBackend)
    cfg.safetyBackend='zoh';
end
backend=lower(char(cfg.safetyBackend));
assert(any(strcmp(backend,{'zoh','legacy','baseline'})), ...
    'CBF:UnknownBackend','cfg.safetyBackend must be zoh, legacy or baseline.');
if ~isfield(cfg,'referenceModes') || isempty(cfg.referenceModes)
    cfg.referenceModes=1;
end
refs=cfg.referenceModes(:).';
assert(isnumeric(refs) && all(ismember(refs,[1 2 3])) && ...
    numel(unique(refs))==numel(refs), ...
    'CBF:InvalidReferences','referenceModes must contain unique values from [1 2 3].');
originalDirectory=pwd;
scriptDirectory=fileparts(mfilename('fullpath'));
cleaner=onCleanup(@()cd(originalDirectory)); %#ok<NASGU>
addpath(genpath(scriptDirectory));
required={'HLC_SUSPENDED_CBF_PHYSICAL_FLIGHT','CBF_PREFLIGHT_WARMUP', ...
    'Generate_CBF_ActuatorAware_6Methods','ENVIRONMENT_OBSTACLE_ELLIPSE_MOVE'};
for k=1:numel(required)
    assert(exist(required{k},'file')==2,'CBF:MissingDependency', ...
        'Missing MATLAB dependency: %s',required{k});
end
if strcmp(backend,'zoh')
    assert(exist('CBF_ZOH_FILTER','file')==2, ...
        'CBF:ZOHFilterMissing','Expected controller/CBF_ZOH_FILTER.m.');
    sanity=CBF_ZOH_FILTER('selftest');
    assert(sanity.passed,'CBF:ZOHCalibrationFailed');
    fprintf('[CBF-ZOH] Analytical hover/geometry preflight PASSED; not a flight proof.\n');
end
% S4 is evaluated online using six PRE-GENERATED standalone kernels.
% Never regenerate during flight or preflight; users have already generated.
if any(strcmp(methods,'S4')) && strcmp(backend,'legacy')
    genSource=which('Generate_CBF_ActuatorAware_6Methods');
    generatedRoot=fullfile(fileparts(genSource),'generated_cbf_dual_input');
    assert(isfolder(generatedRoot),'CBF:S4NotGenerated', ...
        'S4 kernel folder absent. Run the generator ONCE from model/substance.');
    addpath(genpath(generatedRoot));
    for mm={'V','P'}
        for part={'uav','capsule','load'}
            kernel=sprintf('CBF_%s_S4_%s',mm{1},part{1});
            kernelPath=which(kernel);
            assert(~isempty(kernelPath) && isfile(kernelPath),'CBF:KernelNotGenerated', ...
                'Missing %s. Generate S4 once, then re-run the study.',kernel);
            % A stale kernel with the same name elsewhere is not acceptable.
            assert(startsWith(lower(strrep(kernelPath,'/','\')), ...
                lower(strrep(generatedRoot,'/','\'))),'CBF:StaleKernelPath', ...
                'S4 kernel %s resolves outside expected generated folder: %s', ...
                kernel,kernelPath);
        end
    end
end
requested={'virtual','physical'}; % default: never silently omit either input mode
if strcmp(backend,'baseline')
    % DIAGNOSTIC ONLY: isolate unchanged HLC + original actuator saturation.
    % One run is sufficient, since Virtual/Physical CBF is deliberately OFF.
    requested={'physical'};
    fprintf('[CBF-BASELINE] Unmodified HLC + actuator saturation; no safety guarantee.\n');
end
root=tempname(tempdir);mkdir(root);
tmpCleaner=onCleanup(@()localRemoveTemporaryRun(root,scriptDirectory)); %#ok<NASGU>
nCase=numel(refs)*numel(methods)*numel(requested);
res=cell(nCase,1);caseNo=0;
fprintf('[CBF-STUDY] %d independent trials: %d geometries x %d controller modes x %d references.\n', ...
    nCase,numel(methods),numel(requested),numel(refs));
fprintf('[CBF-STUDY] Generator is NOT run. MATLAB plots DISPLAY ONLY.\n');
for ir=1:numel(refs)
    refMode=refs(ir);
    for ig=1:numel(methods)
        method=methods{ig};scenario=sprintf('ref%d_%s',refMode,method);
        for im=1:numel(requested)
            inputMode=requested{im};caseNo=caseNo+1;
            wd=fullfile(root,scenario,inputMode);
            mkdir(wd);
            fprintf('\n[CBF-STUDY] %d/%d: %s / %s / reference=%d\n', ...
                caseNo,nCase,method,inputMode,refMode);
            % Identical noise seed for each independent comparison trial.
            rng(20261016,'twister');
            try
                one=localRunOneFlight(inputMode,wd,method,refMode,backend);
            catch ME
                err=struct('mode',inputMode,'geometryMethod',method, ...
                    'referenceMode',refMode,'identifier',ME.identifier, ...
                    'message',ME.message,'stack',ME.stack);
                save(fullfile(wd,'CBF_compare_unexpected_failure.mat'),'err');
                fprintf(2,'[CBF-STUDY] %s/%s unexpected failure: %s: %s\n', ...
                    method,inputMode,ME.identifier,ME.message);
                one=struct('mode',inputMode,'folder',wd,'stopped',true, ...
                    'reason',[ME.identifier ': ' ME.message], ...
                    'lastPathSample_s',NaN,'firstCBFEngage_s',NaN,'maxDeltaU_inf',NaN);
            end
            one.geometryMethod=method;
            one.referenceMode=refMode;
            res{caseNo}=one;
            cd(scriptDirectory);
        end
    end
end
R=vertcat(res{:});
for i=1:numel(R)
    R(i).folder=sprintf('ZIP/ref%d_%s/%s/', ...
        R(i).referenceMode,R(i).geometryMethod,R(i).mode);
end
summary=table(string({R.geometryMethod}).', [R.referenceMode].', ...
    string({R.mode}).',string({R.folder}).',[R.stopped].', ...
    [R.lastPathSample_s].',[R.firstCBFEngage_s].', ...
    [R.maxDeltaU_inf].',string({R.reason}).', ...
    'VariableNames',{'geometry_method','reference_mode','input_mode', ...
    'folder','stopped','last_integrated_sample_s', ...
    'first_cbf_active_s','max_base_safe_delta_inf','reason'});
if strcmp(backend,'baseline')
    % Do not mislabel a CBF-OFF baseline as a Physical-CBF result.
    summary.input_mode(:)="hlc_only";
end
writetable(summary,fullfile(root,'CBF_dual_input_summary.csv'));
% The two plots remain in MATLAB windows. No PNG/FIG images are saved.
for ir=1:numel(refs)
    fig3d=1;figZ=10;
    if ir>1,fig3d=20+refs(ir);figZ=30+refs(ir);end
    try
        localPlotFourGeometries3D(root,methods,refs(ir),fig3d);
    catch ME
        warning('CBF:Study3DPlot','Figure 3D unavailable for reference %d: %s',refs(ir),ME.message);
    end
    try
        localPlotFourGeometriesZ(root,methods,refs(ir),figZ);
    catch ME
        warning('CBF:StudyZPlot','Altitude plots unavailable for reference %d: %s',refs(ir),ME.message);
    end
end
archivePath=fullfile(scriptDirectory,'CBF_DUAL_RESULTS_UPLOAD.zip');
REPORT=struct('outputFolder','ZIP/ref<N>_<method>/<mode>/', ...
    'zip',archivePath,'geometryMethods',{methods}, ...
    'referenceModes',refs,'safetyBackend',backend,'summary',summary,'runs',R, ...
    'note','Baseline disables CBF; zoh/legacy compare two input modes. Neither is a safety proof.');
save(fullfile(root,'CBF_dual_input_report.mat'),'REPORT','-v7');
% Backward compatibility ONLY for the old S4 failure auditor: 2 MAT copies.
% Full originals always remain under ref1_S4/{virtual,physical}/.
if any(refs==1) && any(strcmp(methods,'S4'))
    for im=1:numel(requested)
        mode=requested{im};srcMat=fullfile(root,'ref1_S4',mode,'CBF_physical_QP_failure.mat');
        if isfile(srcMat)
            copyfile(srcMat,fullfile(root,['CBF_compare_' mode '_QP_failure.mat']));
        end
    end
end
% Read-only diagnosis based on the saved traces. No extra project sources.
try
    localWriteStudyDiagnostics(root,R);
catch ME
    warning('CBF:StudyDiagnostics','Study diagnostics incomplete: %s',ME.message);
end
try
    localCopyProjectSourceSnapshot(scriptDirectory,root);
catch ME
    warning('CBF:SourceCapture','Source collection incomplete: %s',ME.message);
end
cd(scriptDirectory);
stageZip=[tempname(tempdir) '.zip'];
zip(stageZip,'*',root);
[ok,msg]=movefile(stageZip,archivePath,'f');
assert(ok,'CBF:ResultZipMove','Cannot finalize result ZIP: %s',msg);
assert(isfile(archivePath),'CBF:ResultZip','Result ZIP missing.');
fprintf('[CBF-STUDY] FINISHED. Figure 1: four 3D trajectories; Figure 10: altitude.\n');
fprintf('[CBF-STUDY] ONE RESULT FILE (overwrites prior): %s\n',archivePath);
disp(summary);
end

function localRemoveTemporaryRun(root,scriptDirectory)
try
    cd(scriptDirectory);
    if exist(root,'dir')==7,rmdir(root,'s');end
catch ME
    warning('CBF:TemporaryCleanup','Could not remove %s: %s',root,ME.message);
end
end

function localCopyProjectSourceSnapshot(scriptDirectory,root)
% Capture only the experiment's static/runtime-discovered project sources.
% DO NOT recursively copy the entire common_matlab tree (previously >1000
% files including unrelated trajectory programs). Preserve real subfolders.
sourceRoot=fullfile(root,'PROJECT_SOURCES');
mkdir(sourceRoot);
entry=fullfile(scriptDirectory,'run_VERTICAL_CBF_DUAL_INPUT_COMPARISON.m');
files={entry};unresolved={};
% Explicit dynamic dependencies known to the flight/HLC/CBF integration.
critical={'HLC_SUSPENDED_CBF_PHYSICAL_FLIGHT', ...
    'Generate_CBF_ActuatorAware_6Methods','CBF_PREFLIGHT_WARMUP', ...
    'ENVIRONMENT_OBSTACLE_ELLIPSE_MOVE', 'Model_Suspended_Load', ...
    'Estimator_EKF_SuspendedLoad','Controller_HL_Suspended_Load'};
for ii=1:numel(critical)
    pp=which(critical{ii});
    if ~isempty(pp)
        files{end+1,1}=pp; %#ok<AGROW>
    else
        unresolved{end+1,1}=['NOT FOUND: ' critical{ii}]; %#ok<AGROW>
    end
end
try
    [deps,~]=matlab.codetools.requiredFilesAndProducts(entry);
    files=[files(:);deps(:)];
catch ME
    unresolved{end+1,1}=['Static dependency scan: ' ME.message]; %#ok<AGROW>
end
try
    live=inmem('-completenames');
    files=[files(:);live(:)];
catch ME
    unresolved{end+1,1}=['Runtime loaded-function scan: ' ME.message]; %#ok<AGROW>
end
% Include class method files adjacent to every discovered @class member,
% because obj.method dispatch can be missed by static analysis.
paths=unique(cellfun(@char,files(:),'UniformOutput',false),'stable');
for ii=1:numel(paths)
    pp=paths{ii};
    parent=fileparts(pp);
    [~,folderName]=fileparts(parent);
    if startsWith(folderName,'@') && isfolder(parent)
        siblings=dir(fullfile(parent,'*.m'));
        for jj=1:numel(siblings)
            paths{end+1,1}=fullfile(siblings(jj).folder,siblings(jj).name); %#ok<AGROW>
        end
    end
end
paths=unique(paths,'stable');
rootPrefix=[lower(strrep(scriptDirectory,'/','\')) filesep];
count=0;
for ii=1:numel(paths)
    pp=char(paths{ii});
    normalized=lower(strrep(pp,'/','\'));
    if ~startsWith(normalized,rootPrefix)
        continue; % MATLAB/toolbox/external code is not recopied.
    end
    if ~isfile(pp),continue;end
    relative=pp(numel(scriptDirectory)+2:end);
    normRel=strrep(relative,'\','/');
    if startsWith(normRel,'.git/') || contains(normRel,'/.git/') || ...
       contains(normRel,'/generated_cbf_dual_input/') || ...
       startsWith(normRel,'model/substance/generated_cbf_dual_input/')
        continue;
    end
    destination=fullfile(sourceRoot,relative);
    targetDir=fileparts(destination);
    if ~exist(targetDir,'dir'),mkdir(targetDir);end
    [ok,msg]=copyfile(pp,destination);
    if ok,count=count+1;else,unresolved{end+1,1}=[pp ': ' msg];end %#ok<AGROW>
end
fid=fopen(fullfile(root,'SOURCE_PROVENANCE.txt'),'w');
if fid>=0
    fprintf(fid,'Source collection: static and in-memory dependencies only.\n');
    fprintf(fid,'Project source files copied: %d\n',count);
    fprintf(fid,'No recursive whole-project copy; source files keep original relative paths.\n');
    fprintf(fid,'Dynamic function names not loaded or statically resolved may be omitted.\n');
    fprintf(fid,'Generator is not executed by this RUN; recorded dependency source is authoritative.\n');
    for ii=1:numel(unresolved),fprintf(fid,'%s\n',unresolved{ii});end
    fclose(fid);
end
end

function runResult=localRunOneFlight(inputMode,workFolder,geometryMethod,referenceMode,safetyBackend)
% The following is the current physical flight script with the requested
% input-mode switch and read-only logging. Do not reset or reuse another
% mode's plant, EKF, CBF states, warmup context or integration memory.
flightCBFTrial=3;
% Each function call constructs a NEW DRONE, plant, EKF, reference, HLC and
% CBF supervisor. Nothing is inherited from the previous A/B flight.
if ~exist(workFolder,'dir'),mkdir(workFolder);end
cd(workFolder);
%% 1. INITIALIZATION -- same plant and mass-estimator configuration as HLC-only baseline
ts = 0;
dt = 0.025;
te = 50;
time = TIME(ts,dt,te);
logger = LOGGER(1, size(ts:dt:te, 2), 0, [],[]);

agent = DRONE;
agent.parameter = DRONE_PARAM_SUSPENDED_LOAD("DIATONE");
agent.parameter.set("loadmass",0.14);
agent.parameter.set("cableL",2.0);
initial_state.q  = [0; 0; 0];
initial_state.w  = [0; 0; 0];
initial_state.vL = [0; 0; 0];
initial_state.p = [0; 0; 0];
initial_state.pT = [0; 0; -1];
initial_state.wL = [0; 0; 0];
initial_state.pL = initial_state.p + initial_state.pT*agent.parameter.cableL;
initial_state.v = [0; 0; 0];

agent.plant = MODEL_CLASS(agent,Model_Suspended_Load(dt, initial_state,1,agent));
% INTENTIONAL: plant true mass fixed at construction (0.14 kg),
% estimator starts at 0.01 kg and estimates mass toward its true value.
agent.parameter.set("loadmass",0.01);
motive = Connector_Natnet_sim(dt, {{1,"p","q"},{1,"pL","pT"}});
motive.getData(agent);

agent.sensor.set_function_class("motive", MOTIVE(agent, motive,"output_func",@motive_output,"rigid_id",[1,2],"state_list",{["p","q"],"p"}));
% ORIGINAL common EKF; do not change its update or mass-estimation settings.
agent.estimator.set_function_class("ekf", EKF(agent, Estimator_EKF_SuspendedLoad(agent,dt,...
    MODEL_CLASS(agent,Model_Suspended_Load(dt, initial_state, 1,agent,"Load_mL_HL")),...
    ["p", "q","pL","pT"])));
agent.estimator.set_function_class("loadstate", SUSPENDED_LOAD_STATE_MANAGER(agent));
L = agent.parameter.cableL;

%% 1A. REFERENCE COMPARISON: one unchanged TIME reference + two separate C6 spatial paths
% Change ONLY this numeric flag (like run_VERTICAL_HLC_FLIGHT_READY.m):
%   1 = original symbolic / constant-speed vertical line
%   2 = independent 13th-degree C6 polynomial through vertical waypoints
%   3 = independent degree-7 C6 B-spline with vertical control points
% referenceMode supplied by the comparison runner; never changed within a trial.
% FLIGHT-ONLY experiment selector. Takeoff/landing, EKF implementation, and original HLC
% inverse remain unchanged. 1=old yaw-locked S3, 2=yaw-free S3,
% 3=yaw-free S3 + FLIGHT-ONLY Euler measurement chart alignment.
% Trial 3 is a controlled diagnosis of the observed +2*pi sensor roll jump
% at t=13.000 s. It changes only flight EKF input measurements, never the
% original EKF class, tuning, HLC law, physical CBF inequalities, or plant.
% This is NOT a global SO(3) estimator redesign or safety certificate.
% flightCBFTrial=3 comes from caller for both variants.
assert(any(flightCBFTrial==[1,2,3]),'CBF:FlightTrialFlag');

% Same 3D STRAIGHT LINE for all methods: x=y=0, start [0 0 3], end [0 0 66].
% Flight runs from t=5 to t=47, i.e. 42 s nominal at 1.5 m/s (63 m).
% Unlike the original line, methods 2/3 smoothly start/stop along that line.
lineP0        = [0 0 3];
lineVelocity  = 1.5;  % original baseline [m/s] -- DO NOT CHANGE for method 1
lineDirection = [0 0 1];

% Method 2: polynomial WAYPOINTS. These are mandatory pass-through points.
polyWaypoints = [0 0 3; 0 0 7.5; 0 0 16.5; ...
    0 0 25.5; 0 0 34.5; 0 0 43.5; 0 0 52.5; ...
    0 0 61.5; 0 0 66];

% Method 3: B-spline CONTROL POINTS. The middle points are not generally
% passed through. For this straight comparison, all are collinear and
% sampled at the spline's Greville abscissae to reproduce z=3+63*s.
bsplineControlPoints = [0 0 3; 0 0 7.5; 0 0 16.5; ...
    0 0 25.5; 0 0 34.5; 0 0 43.5; 0 0 52.5; ...
    0 0 61.5; 0 0 66];

% No intermediary arrival times; computed once from spatial path and speed.
% This is NOT online state-dependent path following. A 3-s endpoint ramp
% and ~1.65 m/s cruise are roughly comparable to the 1.5-m/s baseline.
referenceMaxSpeed = 1.65;
referenceRampTime = 3;
referenceYaw = 0;

switch referenceMode
    case 1
        referenceLabel='original_line';
        referenceKey="timevarying";
        nominal_ref=TIME_VARYING_REFERENCE(agent, ...
            {"gen_ref_line",{"p0",lineP0,"velocity",lineVelocity, ...
             "direction",lineDirection},6});
        refDuration=NaN;
        fprintf('[CBF-REF] 1 original_line: vertical speed=%.3f m/s (unchanged)\n',lineVelocity);
    case 2
        referenceLabel='poly_c6';
        referenceKey="polynomial_path";
        nominal_ref=PATH_REFERENCE_POLYNOMIAL(agent, ...
            {"gen_ref_c6_polynomial",{"waypoints",polyWaypoints, ...
             "maxSpeed",referenceMaxSpeed,"rampTime",referenceRampTime, ...
             "yaw",referenceYaw},6});
        refDuration=nominal_ref.ref.info.duration;
        fprintf('[CBF-REF] 2 poly_c6: vertical %d waypoints, auto-duration=%.3f s\n', ...
            size(polyWaypoints,1),refDuration);
    case 3
        referenceLabel='bspline_c6';
        referenceKey="bspline_path";
        nominal_ref=PATH_REFERENCE_BSPLINE(agent, ...
            {"gen_ref_c6_bspline",{"controlPoints",bsplineControlPoints, ...
             "maxSpeed",referenceMaxSpeed,"rampTime",referenceRampTime, ...
             "yaw",referenceYaw},6});
        refDuration=nominal_ref.ref.info.duration;
        fprintf('[CBF-REF] 3 bspline_c6: vertical %d control points, auto-duration=%.3f s\n', ...
            size(bsplineControlPoints,1),refDuration);
    otherwise
        error('CBF:UnknownReferenceMode','referenceMode must be 1, 2, or 3.');
end
% Check the new references BEFORE takeoff, not during the flight.
ref0=nominal_ref.func(0);
assert(isequal(size(ref0),[28 1]) && isnumeric(ref0) && ...
       isreal(ref0) && all(isfinite(ref0)) && ...
       norm(ref0(1:3)-lineP0(:))<1e-6, ...
       'CBF:ReferenceJet','Method %d: invalid initial 28x1 reference.',referenceMode);
if referenceMode~=1
    refEnd=nominal_ref.func(refDuration);
    assert(norm(refEnd(1:3)-[0;0;66])<1e-5 && ...
           all(isfinite(refEnd)) && norm(refEnd(5:end))<1e-5, ...
           'CBF:ReferenceEndpoint','C6 vertical reference endpoint mismatch.');
    % Verify the WHOLE sampled flight path is vertical, with real finite
    % 0..6 derivatives. This avoids accidentally testing an old curved plan.
    zPrev=-Inf;
    worstXY=0;
    for tau=linspace(0,refDuration,25)
        jet=nominal_ref.func(tau);
        assert(isequal(size(jet),[28 1]) && all(isfinite(jet)), ...
            'CBF:ReferenceJet','Nonfinite reference during preflight sampling.');
        horizontal=reshape(jet,4,7);
        worstXY=max(worstXY,max(abs(horizontal(1:2,:)),[],'all'));
        assert(jet(3)>=zPrev-1e-6, ...
            'CBF:ReferenceDirection','C6 path is not monotonically ascending.');
        zPrev=jet(3);
    end
    assert(worstXY<1e-5,'CBF:ReferenceNotVertical', ...
        'C6 reference must have zero x,y and horizontal derivatives.');
    fprintf('[CBF-REF] Verified straight C6 reference: max|XY jet|=%.3g, zEnd=%.3f m\n', ...
        worstXY,refEnd(3));
end
fprintf('[CBF-REF] method=%d (%s), order=6, referenceKey=%s, CBF=ON flight only\n', ...
    referenceMode,referenceLabel,char(referenceKey));
fprintf('[CBF-REF] Straight geometry x=y=0; C6 methods have smooth endpoint ramps.\n');
% A/B output folders are isolated; historical trial logs are never overwritten.
agent.reference.set_function_class(referenceKey,nominal_ref);
agent.reference.set_function_class("sload", SUSPENDED_LOAD_REF_ADJUST(agent));
agent.reference.set_function_class("takeoff", TAKEOFF_REFERENCE(agent,"zd",3.0,"te",5));
agent.reference.set_function_class("landing", LANDING_REFERENCE(agent,"dt",dt,"zd",-L,"te",3));

%% *** CBF CHANGE A: create/register HLC_SUSPENDED_CBF ***
hlc = HLC_SUSPENDED_CBF_PHYSICAL_FLIGHT(agent,Controller_HL_Suspended_Load(dt,agent));
hlc.cbfMode = inputMode; % assigned once from A/B runner; do not switch
hlc.cbfBackend = safetyBackend; % legacy / zoh, same backend for all eight trials
hlc.cbfMethod = geometryMethod; % fixed for both trials: S1/S2/S3/S4
hlc.cbfEnabled = false; % NEVER enable outside flight phase

% Geometry values below are PLACEHOLDERS, not verified physical sizes.
% Measure/enclose real UAV body, cable, and payload before safety claims.
rU = 0.45;    % outer sphere radius of UAV [m] (example)
rL = 0.30;    % outer sphere radius of payload [m] (example)
rC = 0.015;   % cable cross-section radius [m] (example)
Rmax = max([rU,rL,rC]);
shapes = struct();
shapes.single = sqrt(2)*[L/2 + Rmax; Rmax];

% Model 2: series of overlapping balls, end balls cover payload and UAV.
lambda = (0:0.1:1)';
ds = L*max(diff(lambda));
rInterior = sqrt(rC^2+(ds/2)^2);
rSphere = rInterior*ones(numel(lambda),1);
rSphere(1) = max(rSphere(1),rL);  % lambda=0: payload
rSphere(end) = max(rSphere(end),rU); % lambda=1: UAV
shapes.spheres = [lambda,rSphere];

% Model 3: UAV sphere (as ellipsoid), cable prolate ellipsoid, payload sphere.
shapes.uav = rU*ones(3,1);
aC = L/2 + 0.10;
bC = rC/sqrt(1-(L/(2*aC))^2);
shapes.cable = [aC;bC];
% S4: exact capsule of cable radius rC, enclosing the segment between ends.
% Conservative smoothing error max: L/2 * epsilon = 0.005 m.
capsuleSupportTolerance_m=0.005;
shapes.capsule=[rC; 2*capsuleSupportTolerance_m/L];
shapes.load = rL*ones(3,1);

hlc.cbfShapes = shapes;
hlc.cbfKappa = 5*ones(6,1); % diagnostic candidate, NOT globally tuned
hlc.cbfOpts = struct( ...
    'requireInitialChain',true, ...
    'uLimits',[0;20;1;1;1], ...
    'lockNominalYaw',(flightCBFTrial==1), ... % 1: original yaw lock; 2: QP may use Mz
    'onMargin',0.75, ... % current normalized CBF margin threshold
    'offMargin',0.85, ... % hysteresis; must remain above onMargin
    'preemptivePush',0.03, ... % EXPERIMENTAL: soft extra CBF margin before sigma reaches zero
    'preemptiveWeight',100, ... % weight on soft-target slack; hard constraints unchanged
    'preemptiveMaxRows',3, ... % obstacle/order-independent critical CBF rows
    'navigationLeadSeconds',4.0, ... % comparative trial only; NOT geometric TTC, viability or safety guarantee
    'navigationMinClosingRate',0.002, ... % normalized margin / second noise floor
    'navigationRateBlend',0.65, ... % signed filtering of barrier-margin rate
    'navigationConfirmSteps',4, ... % require 4 consecutive samples (~0.1s) to preactivate
    'navigationMaxPush',0.05, ... % reduced upper bound for SOFT CBF margin encouragement
    'smoothInputWeight',0.20, ... % secondary smoothing on [fDDot Mx My Mz]
    'failOnGeometricCertificateLoss',true, ... % stop if S3 geometry separation certificate lost
    'experimentalContinueAfterPsiViolation',false, ... % FAIL CLOSED when HOCBF initial chain is invalid
    'experimentalStopAtGeometricBoundary',true, ... % stop if S3 separating-plane h<=0 (conservative)
    'experimentalTraceFile',['CBF_chain_recovery_trial_' referenceLabel '.csv'], ... % per-method ACTIVE-step CSV
    'releaseHoldSteps',8, ...
    'releaseForceTol',0.10, ...
    'releaseMomentTol',0.02, ...
    'releaseAuxTol',0.05, ...
    'allowPhysicalFallback',false, ... % forbid hidden controller-mode change
    'monitorOnly',false, ... % ENABLE intervention; first test 'true' for diagnostics only
    'emitEvents',true);
% RUN version/config print: generator & warmup versions alone do NOT prove this RUN is current.
% Read-only verification; no estimator/controller/safety model is changed.
fprintf('[CBF-RUN] revision=FLIGHT_ONLY_YAWFREE_EULER_CHART_20261016 file=%s\n',mfilename('fullpath'));
fprintf('[CBF-TRIAL] mode=%s reference=%s, flightCBFTrial=%d, yawLocked=%d, chart correction trial3\n', ...
    inputMode,referenceLabel,flightCBFTrial,hlc.cbfOpts.lockNominalYaw);
fprintf(['[CBF-RUN] navigationLeadSeconds=%.2f; navigationMaxPush=%.3f; ' ...
    'preemptiveMaxRows=%d; onMargin=%.3f; offMargin=%.3f\n'], ...
    hlc.cbfOpts.navigationLeadSeconds, hlc.cbfOpts.navigationMaxPush, ...
    hlc.cbfOpts.preemptiveMaxRows, hlc.cbfOpts.onMargin, hlc.cbfOpts.offMargin);
assert(hlc.cbfOpts.navigationLeadSeconds==4.0 && ...
       hlc.cbfOpts.navigationMaxPush==0.05 && ...
       hlc.cbfOpts.preemptiveMaxRows==3, ...
       'CBF:RunOptionMismatch', ...
       'Flight run does not use expected lead4 / maxPush0.05 / maxRows3 configuration.');
hlc.cbfAux = [];
hlc.cbfAuxMode = '';
hlc.cbfSupervisor = struct();
hlc.cbfTime = 0;

agent.controller.set_function_class("hlc_suspended", hlc);

agent.set_cha_allocation_for_all("sensor","motive");
agent.set_cha_allocation_for_all("estimator",["ekf","loadstate"]);
agent.cha_allocation.a.reference = ["takeoff","sload"];
agent.cha_allocation.t.reference = ["takeoff","sload"];
agent.cha_allocation.f.reference = [referenceKey,"sload"];
agent.cha_allocation.l.reference = ["landing","sload"];
env = [];

%% READ-ONLY FORENSICS: a ring of in-memory per-step observations, saved at end.
% No controller, estimator, sensor or plant object is modified. The capture
% covers the period leading up to NAV4 engagement, NOT just the failure.
% Sensor output and pre/post EKF states are captured in the existing order.
% 'ekfBefore' is the PREVIOUS POSTERIOR, not the EKF internal xh_pre.
% Plant truth at t is captured BEFORE plant.do(t); plantAfter is at t+dt.
diagEnabled = true;
diagStart_s = 9.0;
diagEnd_s = 15.5;
diagRecords = cell(0,1);
fprintf('[CBF-DIAG] READ-ONLY stage trace enabled t=[%.2f, %.2f] s; MAT+CSV saved on stop/end.\n', ...
    diagStart_s,diagEnd_s);

%% 2. WARMUP
fprintf('[CBF-COMPARE] requested mode=%s / method=%s / flight-phase supervision\n',hlc.cbfMode,hlc.cbfMethod);
fprintf('[VERTICAL/CBF] %s/%s | reference=%s | identical EKF + mass estimation\n',inputMode,geometryMethod,referenceLabel);
fprintf('[CBF-TRIAL] SOFT ANTICIPATION: onMargin=%.3f, offMargin=%.3f; NOT safety-certified\n', ...
    hlc.cbfOpts.onMargin,hlc.cbfOpts.offMargin);
if hlc.cbfOpts.lockNominalYaw
    fprintf('[CBF-YAW] trial 1: yaw locked to saturated original HLC Mz.\n');
else
    fprintf('[CBF-YAW] trial 2: yaw QP free within unchanged +/-1 Nm actuator bounds.\n');
end
disp("Initializing simulation states...");
for cha_init = ['a', 't', 'f', 'l']
    agent.cha = cha_init;
    do_prop_local(time, cha_init, "0", logger, env, agent, "sensor", 1, false);
    do_prop_local(time, cha_init, "0", logger, env, agent, "estimator", 1, false);
    do_prop_local(time, cha_init, "0", logger, env, agent, "reference", 1, false);
    do_prop_local(time, cha_init, "0", logger, env, agent, "controller", 1, false);
    do_prop_local(time, cha_init, "0", logger, env, agent, "input_transform", 1, false);
end

%% 3. ARMING
 disp("Arming...");
time.t = 0;
cha = 'a';
if ~isempty(motive), motive.getData(agent); end
tStartTotal = tic;
do_prop_local(time, cha, cha, logger, env, agent, "sensor", 1, true);
do_prop_local(time, cha, cha, logger, env, agent, "estimator", 1, true);
do_prop_local(time, cha, cha, logger, env, agent, "reference", 1, true);
do_prop_local(time, cha, cha, logger, env, agent, "controller", 1, true);
do_prop_local(time, cha, cha, logger, env, agent, "input_transform", 1, true);
do_prop_local(time, cha, cha, logger, env, agent, "plant", 1, true);
agent.controller.result.controllertime_total = toc(tStartTotal);
logger.logging(time, cha, agent, []);
time.k = logger.k;

%% PREFLIGHT: key CBF and QP paths before takeoff and before flight.
% This intentionally adds initialization wall time, never a plant command.
% If warmup fails, do NOT silently start a cold CBF during flight.
fprintf('[CBF-SOURCE] preflight=%s\n',which('CBF_PREFLIGHT_WARMUP'));
fprintf('[CBF-SOURCE] generator=%s\n',which('Generate_CBF_ActuatorAware_6Methods'));
fprintf('[CBF-SOURCE] hlc=%s\n',which('HLC_SUSPENDED_CBF_PHYSICAL_FLIGHT'));
% Display the ACTUAL obstacle source and evaluated obstacle configuration.
% Read-only: this does not change any obstacle or controller state.
fprintf('[CBF-OBS] source=%s\n',which('ENVIRONMENT_OBSTACLE_ELLIPSE_MOVE'));
obsProbeTime = 5.0; % at the beginning of the only CBF-supervised phase
obsProbe = ENVIRONMENT_OBSTACLE_ELLIPSE_MOVE(obsProbeTime);
fprintf('[CBF-OBS] at t=%.3f s, obstacles=%d (geometry=%s)\n', ...
    obsProbeTime,numel(obsProbe),geometryMethod);
for obsIdx = 1:numel(obsProbe)
    oneObs = obsProbe(obsIdx);
    obsId = obsIdx;
    if isfield(oneObs,'id'), obsId = oneObs.id; end
    obsMargin = 0;
    if isfield(oneObs,'d_margin'), obsMargin = oneObs.d_margin; end
    obsVelocity = zeros(3,1);
    if isfield(oneObs,'v_center'), obsVelocity = oneObs.v_center(:); end
    fprintf(['[CBF-OBS] id=%g type=%s center=[%.3f %.3f %.3f] ' ...
        'ellipsoid_radii=[%.3f %.3f %.3f] margin=%.3f velocity=[%.3f %.3f %.3f]\n'], ...
        double(obsId),char(oneObs.type),oneObs.p_center(:), ...
        oneObs.ellipsoid_radii(:),obsMargin,obsVelocity);
end
if strcmp(safetyBackend,'baseline')
    % Never exercise the CBF codepath in a nominal-only diagnostic.
    fprintf('[CBF-BASELINE] No CBF preflight; original HLC remains active with its original saturation.\n');
else
    preflight=hlc.prepareFlightCBF();
    assert(preflight.passed && hlc.cbfPreflightCompleted, ...
        'CBF:PreflightMissing','Preflight incomplete. Flight CBF not enabled.');
    fprintf('[CBF-PREFLIGHT] completed at t=%.3f before takeoff. monitor=%d rows\n', ...
        time.t, preflight.monitorRows);
end

%% 4. SIMULATION LOOP
disp("Starting Simulation...");
fprintf('[CBF-NAV] generic CBF margin-trend lead=%.1f s, softPush=[%.3g,%.3g], smoothWeight=%.3g\n', ...
    hlc.cbfOpts.navigationLeadSeconds,hlc.cbfOpts.preemptivePush, ...
    hlc.cbfOpts.navigationMaxPush,hlc.cbfOpts.smoothInputWeight);
if strcmp(safetyBackend,'zoh')
    fprintf('[CBF-ZOH] Actual input held over dt. Predictive filter; no recursive safety proof.\n');
elseif strcmp(safetyBackend,'legacy')
    fprintf('[CBF-SAFE] Strict psi-chain check ON, original hard CBF and actuator inequalities retained.\n');
else
    fprintf('[CBF-BASELINE] No CBF intervention at any time; HLC reference and saturation unchanged.\n');
end
trialStopped=false;trialStopReason='';
cha = 't';
verticalTrace = zeros(0,12);  % includes yaw CBF correction = 0
trajectoryTrace = zeros(0,17); % true/estimated UAV/load position and reference
refJetTrace = zeros(0,29); % [time, 0..6 time derivatives of x,y,z,yaw]
runtimeTrace = zeros(0,6); % t, phase_f, controller_s, total_step_s, cbf_s, cbf_active
flightEulerTrace = zeros(0,13); % flight sensor chart audit: time/prior/raw/adjusted/max gaps/changed

flightHlcTrace=zeros(0,28); % read-only per-call flight HLC inversion audit
inputCompareTrace=zeros(0,33); % 33 columns: inputs, virtual/physical mode, safety, timing
constraintsTrace=zeros(0,14); % monitor/active, 3 shapes x obstacles, psi 0..5
firstFlightCostLogged=false;
firstActiveCostLogged=false;
firstBudgetWarning=false;
while time.t <= time.te
    if time.t < 5.0
        if cha ~= 't'
            disp("Phase: Takeoff");
            cha = 't';
        end
    elseif time.t < 47.0
        if cha ~= 'f'
            disp("Phase: Flight (CBF enabled)");
            cha = 'f';
            % Key evaluator and sampled/soft QP paths were exercised beforehand.
            % Real supervisor states have not been touched by dry-run.
            if ~strcmp(safetyBackend,'baseline')
                assert(hlc.cbfPreflightCompleted,'CBF:PreflightMissing', ...
                    'Flight entered without completed preflight.');
            end
            hlc.cbfSupervisor = struct();
            hlc.cbfAux = [];
            hlc.cbfAuxMode = '';
        end
    else
        if cha ~= 'l'
            % This experiment promises FLIGHT ONLY supervision.
            % If still active, do not quietly drop CBF or redesign landing.
            if isfield(hlc.cbfSupervisor,'active') && hlc.cbfSupervisor.active
                trialStopped=true;
                trialStopReason=['CBF:FlightExitActive: selected CBF still active ' ...
                    'at the flight->landing boundary; no uncertified shutdown'];
                fprintf(2,'[CBF-EXP] CONTROLLED STOP at t=%.5f: %s\n', ...
                    time.t,trialStopReason);
                break;
            end
            disp("Phase: Landing (CBF disabled: HLC unchanged)");
            cha = 'l';
        end
    end

    traceThisStep = diagEnabled && cha=='f' && ...
        time.t>=diagStart_s-1e-10 && time.t<=diagEnd_s+1e-10;
    if traceThisStep
        diagStep=localDiagStart(time.t,dt,agent);
    end
    if ~isempty(motive), motive.getData(agent); end
    tStartTotal = tic;
    do_prop_local(time, cha, cha, logger, env, agent, "sensor", 1, true);
    if traceThisStep
        % Capture ORIGINAL sensor signal (unmodified) for causal forensics.
        diagStep.sensor=localDiagSensor(agent);
    end
    if cha=='f'
        if flightCBFTrial==3
            chart=localFlightEulerChartAlign(agent);
        else
            chart=localFlightEulerChartObserve(agent);
        end
        flightEulerTrace(end+1,:)=[time.t,chart.prior(:).', ...
            chart.raw(:).',chart.adjusted(:).',chart.rawGapMax, ...
            chart.wrappedGapMax,double(chart.changed)]; %#ok<AGROW>
        if traceThisStep
            diagStep.sensorEulerForEKF=chart.adjusted(:);
            diagStep.sensorEulerChartChanged=chart.changed;
        end
        if chart.changed && flightCBFTrial==3
            fprintf(['[CBF-EKF-CHART] t=%.3f flight-only Euler sensor ' ...
                'branch align: raw=[%.5f %.5f %.5f], forEKF=[%.5f %.5f %.5f], ' ...
                'maxRawGap=%.5f maxWrappedGap=%.5f rad\n'], ...
                time.t,chart.raw,chart.adjusted,chart.rawGapMax,chart.wrappedGapMax);
        end
    end
    do_prop_local(time, cha, cha, logger, env, agent, "estimator", 1, true);
    if traceThisStep
        diagStep.ekfAfter=localDiagState(localDiagEKFState(agent));
        diagStep.estimatorMerged=localDiagState(localDiagMergedState(agent));
        diagStep.kalmanGainFro=localDiagKalmanGain(agent);
    end
    do_prop_local(time, cha, cha, logger, env, agent, "reference", 1, true);

    %% *** CBF CHANGE B: set phase and CURRENT time before controller ***
    hlc.cbfEnabled = (cha == 'f') && ~strcmp(safetyBackend,'baseline'); % nominal diagnostic keeps it OFF
    hlc.cbfTime = time.t;

    try
        tController=tic;
        do_prop_local(time, cha, cha, logger, env, agent, "controller", 1, true);
        controllerElapsed=toc(tController);
        if cha=='f'
            flightHlcTrace(end+1,:)=localFlightHlcAuditRow(time.t,hlc,0); %#ok<AGROW>
            inputCompareTrace(end+1,:)=localInputCompareRow(time.t,hlc,0); %#ok<AGROW>
        end
        if traceThisStep
            diagStep=localDiagController(diagStep,agent,hlc);
        end
        do_prop_local(time, cha, cha, logger, env, agent, "input_transform", 1, true);
        do_prop_local(time, cha, cha, logger, env, agent, "plant", 1, true);
        if traceThisStep
            diagStep.plantAfter=localDiagState(agent.plant.state);
            diagStep.complete=true;
        end
    catch ME
        if cha=='f'
            flightHlcTrace(end+1,:)=localFlightHlcAuditRow(time.t,hlc,1); %#ok<AGROW>
            inputCompareTrace(end+1,:)=localInputCompareRow(time.t,hlc,1); %#ok<AGROW>
        end
        if traceThisStep
            % The HLC may have written the RAW command before failing in CBF.
            % agent.controller.result may still belong to the previous step.
            diagStep.failed=true;
            diagStep.failureID=char(ME.identifier);
            diagStep.failureMessage=char(ME.message);
            diagStep.rawOnFailedCall=localDiagNum(localDiagGet(hlc.result,'tmp'),4);
            diagRecords{end+1,1}=diagStep; %#ok<AGROW>
        end
        % A diagnosed CBF limitation terminates the experimental run
        % cleanly, without silently applying uncertified nominal commands.
        % Programming/model errors elsewhere still throw normally.
        if strncmp(ME.identifier,'CBF:',4)
            trialStopped=true;
            trialStopReason=sprintf('%s: %s',ME.identifier,ME.message);
            fprintf(2,'\n[CBF-EXP] CONTROLLED STOP at t=%.5f: %s\n',...
                time.t,trialStopReason);
            break;
        else
            rethrow(ME);
        end
    end
    totalElapsed=toc(tStartTotal);
    agent.controller.result.controllertime_total = totalElapsed;
    logger.logging(time, cha, agent, []);
    if traceThisStep
        diagRecords{end+1,1}=diagStep; %#ok<AGROW>
    end
    verticalTrace(end+1,:) = localVerticalDiagnostic(time.t,agent); %#ok<AGROW>
    trajectoryTrace(end+1,:) = localFlightPathSnapshot(time.t,cha,agent); %#ok<AGROW>
    refJetTrace(end+1,:) = localReferenceJetSnapshot(time.t,agent); %#ok<AGROW>
    cbfElapsed=0;activeNow=false;
    if hlc.cbfEnabled && ~isempty(hlc.result.cbf)
        cbfElapsed=hlc.result.cbf.totalSeconds;
        activeNow=isfield(hlc.result.cbf.supervisor,'active') && ...
            hlc.result.cbf.supervisor.active;
        % Research configuration requirement: QP must not change yaw.
        if activeNow && hlc.cbfOpts.lockNominalYaw && ...
                abs(hlc.result.cbf.uSafe(4)-hlc.result.cbf.uBase(4))>1e-6
            error('CBF:YawLockViolated', ...
                'CBF altered yaw torque despite lockNominalYaw=true.');
        end
    end
    runtimeTrace(end+1,:)=[time.t,double(cha=='f'),controllerElapsed, ...
        totalElapsed,cbfElapsed,double(activeNow)]; %#ok<AGROW>
    if cha=='f'
        constraintsTrace=[constraintsTrace;localConstraintAuditRows(time.t,hlc)]; %#ok<AGROW>
    end
    if cha=='f' && ~firstFlightCostLogged
        fprintf(['[CBF-TIME] first flight step t=%.3f: controller=%.1f ms, ' ...
            'CBF=%.1f ms, total=%.1f ms (budget=%.1f ms)\n'], ...
            time.t,1000*controllerElapsed,1000*cbfElapsed, ...
            1000*totalElapsed,1000*dt);
        firstFlightCostLogged=true;
    end
    if activeNow && ~firstActiveCostLogged
        fprintf(['[CBF-TIME] first ACTIVE step t=%.3f: controller=%.1f ms, ' ...
            'CBF=%.1f ms, total=%.1f ms. QP is now active.\n'], ...
            time.t,1000*controllerElapsed,1000*cbfElapsed,1000*totalElapsed);
        firstActiveCostLogged=true;
    end
    if cha=='f' && totalElapsed>dt && ~firstBudgetWarning
        fprintf(2,['[CBF-TIME] WARNING: measured step elapsed %.1f ms > ' ...
            'dt %.1f ms. Preflight can reduce first-call overhead but ' ...
            'CANNOT guarantee real-time performance.\n'], ...
            1000*totalElapsed,1000*dt);
        firstBudgetWarning=true;
    end

    %% ACTUATOR-AWARE MONITOR (not a proof of future safety)
    % Print at 0.25 s while ACTIVE, 1 s while inactive. Avoid missing
    % intermediate force/fDot evolution immediately before QP failure.
    if hlc.cbfEnabled && ~isempty(hlc.result.cbf)
        cbf = hlc.result.cbf;
        isActive = isfield(cbf.supervisor,'active') && cbf.supervisor.active;
        if isActive, interval = 10; else, interval = 40; end
        if mod(time.k,interval)==0
            if isActive && numel(cbf.supervisor.aux)>=2
                fCBF=cbf.supervisor.aux(1);
                fDotCBF=cbf.supervisor.aux(2);
            else
                fCBF=cbf.uBase(1);
                fDotCBF=0;
            end
            fprintf(['[CBF-AA] t=%.3f status=%s hMin=%.4g ' ...
                'sigma=%.4g bad=%d dU=%.4g f=%.5g fDot=%.5g CPU=%.1f ms\n'], ...
                time.t,cbf.status,cbf.monitor.minH, ...
                cbf.monitor.minNormalizedMargin,cbf.monitor.violations, ...
                norm(cbf.uSafe-cbf.uBase),fCBF,fDotCBF, ...
                1000*cbf.totalSeconds);
            if isfield(cbf,'navigation') && cbf.navigation.enabled
                fprintf('  [CBF-NAV] leadMin=%+.4g minTTC=%.3g s trendRisk=%d\n', ...
                    cbf.navigation.minPredicted,cbf.navigation.minTimeToTrigger, ...
                    cbf.navigation.risk);
            end
            if isfield(cbf,'experimental')
                fprintf('  [CBF-EXP] badChains=%d psiMin=%+.6g psiLastMin=%+.6g recoveries=%d (UNCERTIFIED)\n',...
                    cbf.experimental.badChains,cbf.experimental.minPsi,...
                    cbf.experimental.minPsiLast,cbf.experimental.recoveryCount);
            end
            if isfield(cbf,'preemptive') && cbf.preemptive.enabled
                fprintf('  [CBF-PUSH] rows=%s dynamicPush=%.3g maxSlack=%.3g\n', ...
                    mat2str(cbf.preemptive.rowIndices.'), ...
                    cbf.preemptive.push,cbf.preemptive.maxSlack);
            end
        end
    end

    time.k = logger.k;
    time.t = time.t + dt;
    if mod(time.t, 5.0) < dt
        fprintf('Time: %.1f / %.1f sec (Phase: %s)\n',time.t,time.te,cha);
    end
end

% Save the READ-ONLY forensic log even if CBF fails mid-controller.
% Heavy numerical diagnostics are done ONLY AFTER the simulation loop.
if diagEnabled
    try
        localDiagWrite(diagRecords,diagStart_s,diagEnd_s,dt,trialStopped,trialStopReason);
        if ~isempty(diagRecords)
            for f={'.mat','.csv'}
                base='CBF_flight_state_diagnostic';
                if exist([base f{1}],'file')==2
                    copyfile([base f{1}],[base '_' referenceLabel f{1}]);
                end
            end
        end
    catch ME
        warning('CBF:ReadOnlyDiagnosticFailure', ...
            'Read-only diagnostic failed after simulation: %s',ME.message);
    end
end

% Timing CSV, including partial trajectory on controlled stop.
if ~isempty(runtimeTrace)
    timingNames={'time_s','in_flight','controller_s','total_step_s', ...
        'cbf_s','cbf_active'};
    writetable(array2table(runtimeTrace,'VariableNames',timingNames), ...
        'VERTICAL_CBF_timing.csv');
    flightTiming=runtimeTrace(runtimeTrace(:,2)==1,:);
    if ~isempty(flightTiming)
        fprintf(['[CBF-TIME] flight controller elapsed median=%.1f ms, ' ...
            'max=%.1f ms; CBF median=%.1f ms, max=%.1f ms; ' ...
            'budget overruns=%d/%d (dt=%.1f ms).\n'], ...
            1000*median(flightTiming(:,3)),1000*max(flightTiming(:,3)), ...
            1000*median(flightTiming(:,5)),1000*max(flightTiming(:,5)), ...
            nnz(flightTiming(:,4)>dt),size(flightTiming,1),1000*dt);
    end
end

% [FLIGHT-ONLY HLC INVERSE AUDIT] Record even when CBF fails mid-call.
% Only flight-controller invocations are included. These diagnostics do not
% change the nominal HLC/QP, and cannot certify physical safety.
if ~isempty(flightHlcTrace)
    trialKey='yaw_locked';
    if flightCBFTrial==2,trialKey='yaw_free';end
    if flightCBFTrial==3,trialKey='yaw_free_chart_aligned';end
    auditColumns={'time_s','controller_failed','beta2_rcond','beta2_sigma_min', ...
        'beta2_sigma_max','beta2_rhs_norm2','beta2_rhs_norm_inf', ...
        'load_pT3','z_ref_d0','z_ref_d1','z_ref_d2','z_ref_d3', ...
        'z_ref_d4','z_ref_d5','z_ref_d6','raw_thrust_N', ...
        'raw_moment_inf_Nm','raw_moment_norm2_Nm', ...
        'raw_minus_nominal_input_inf','nominal_minus_safe_input_inf', ...
        'nominal_v1','clip_equivalent_v1','clip_equivalent_delta_v1', ...
        'thrust_clip_delta_N','torque_clip_delta_inf_Nm', ...
        'raw_second_layer_reconstruction_residual_rel', ...
        'clipped_torque_virtual_proxy_rel', ...
        'nominal_hierarchy_unchanged_after_clip'};
    auditName=sprintf('CBF_flight_HLC_inverse_%s_%s.csv',referenceLabel,trialKey);
    writetable(array2table(flightHlcTrace,'VariableNames',auditColumns),auditName);
    spike=find(flightHlcTrace(:,17)>1,1,'first');
    severe=find(flightHlcTrace(:,17)>100,1,'first');
    badRcond=find(flightHlcTrace(:,3)<1e-7,1,'first');
    fprintf('[HLC-FLIGHT-AUDIT] %s: %d controller calls; trial=%d\n', ...
        auditName,size(flightHlcTrace,1),flightCBFTrial);
    if ~isempty(spike)
        fprintf('[HLC-FLIGHT-AUDIT] FIRST raw torque >1 Nm at t=%.5f\n',flightHlcTrace(spike,1));
    end
    if ~isempty(severe)
        fprintf('[HLC-FLIGHT-AUDIT] FIRST raw torque >100 Nm at t=%.5f\n',flightHlcTrace(severe,1));
    end
    if ~isempty(badRcond)
        fprintf('[HLC-FLIGHT-AUDIT] FIRST beta2 rcond <1e-7 at t=%.5f\n',flightHlcTrace(badRcond,1));
    end
end

% NEW: full flight input/certificate audit. Includes failed controller invocation.
if ~isempty(inputCompareTrace)
    fieldNames=localInputCompareColumnNames();
    assert(numel(fieldNames)==size(inputCompareTrace,2),'CBF:CompareTraceDimension');
    writetable(array2table(inputCompareTrace,'VariableNames',fieldNames), ...
        'CBF_input_comparison_full_flight.csv');
    fprintf('[CBF-COMPARE] mode=%s %d flight calls saved to CBF_input_comparison_full_flight.csv\n', ...
        inputMode,size(inputCompareTrace,1));
end

% Mode-agnostic per-obstacle per-component physical-monitor/HOCBF chains.
% source_type=0: saturated baseline frozen-thrust PHYSICAL monitor proxy;
% source_type=1: active filter own HOCBF chain (virtual or physical).
if ~isempty(constraintsTrace)
    constraintNames={'time_s','source_type','obstacle_id','component_code', ...
        'h_m','normalized_monitor_margin','psi0','psi1','psi2','psi3', ...
        'psi4','psi5','initial_chain_ok','monitor_raw_margin'};
    assert(numel(constraintNames)==size(constraintsTrace,2), ...
        'CBF:ConstraintTraceDimension');
    writetable(array2table(constraintsTrace,'VariableNames',constraintNames), ...
        'CBF_constraints_all_rows.csv');
end

% Flight-only Euler chart audit is always exported (including partial stop).
% Trials 1/2 are observation-only; trial 3 uses the adjusted Euler readings.
if ~isempty(flightEulerTrace)
    angleNames={'time_s','prior_roll_rad','prior_pitch_rad','prior_yaw_rad', ...
        'raw_sensor_roll_rad','raw_sensor_pitch_rad','raw_sensor_yaw_rad', ...
        'ekf_input_roll_rad','ekf_input_pitch_rad','ekf_input_yaw_rad', ...
        'max_raw_gap_rad','max_wrapped_gap_rad','chart_changed'};
    angleFilename=sprintf('CBF_flight_EKF_Euler_chart_%s_trial%d.csv', ...
        referenceLabel,flightCBFTrial);
    writetable(array2table(flightEulerTrace,'VariableNames',angleNames),angleFilename);
    fprintf('[CBF-EKF-CHART] saved %s; detected chart wraps=%d/%d (applied only in trial3)\n', ...
        angleFilename,nnz(flightEulerTrace(:,13)),size(flightEulerTrace,1));
end

% Saved at end, including partial trajectory if physical CBF stops.
if ~isempty(verticalTrace)
    colNames = {'time_s','zLoad_m','zDesired_m','zError_m','mLhat_kg', ...
        'thrust_N','Mx_Nm','My_Nm','Mz_Nm','CBF_min_h_m','CBF_deltaU', ...
        'CBF_deltaMz_Nm'};
    writetable(array2table(verticalTrace,'VariableNames',colNames),'VERTICAL_CBF_timeseries.csv');
    fprintf('[VERTICAL] telemetry saved: VERTICAL_CBF_timeseries.csv (%d rows)\n',size(verticalTrace,1));
    copyfile('VERTICAL_CBF_timeseries.csv', ['VERTICAL_CBF_timeseries_' referenceLabel '.csv']);
end

if ~isempty(trajectoryTrace)
    pathNames={'time_s','phase_code','uav_x_m','uav_y_m','uav_z_m', ...
        'load_x_m','load_y_m','load_z_m','ekf_uav_x_m','ekf_uav_y_m', ...
        'ekf_uav_z_m','ekf_load_x_m','ekf_load_y_m','ekf_load_z_m', ...
        'ref_load_x_m','ref_load_y_m','ref_load_z_m'};
    writetable(array2table(trajectoryTrace,'VariableNames',pathNames), ...
        'VERTICAL_CBF_path.csv');
    fprintf('[VERTICAL-PATH] saved VERTICAL_CBF_path.csv (%d rows)\n',size(trajectoryTrace,1));
    % Keep original CSV names for existing workflows, and save a tagged copy
    % so the next comparison run does not overwrite this run's trajectory.
    copyfile('VERTICAL_CBF_path.csv', ['VERTICAL_CBF_path_' referenceLabel '.csv']);
    fprintf('[CBF-REF] comparison path saved: VERTICAL_CBF_path_%s.csv\n',referenceLabel);
end

if ~isempty(runtimeTrace)
    % A per-method timing copy for comparing the same run with three inputs.
    copyfile('VERTICAL_CBF_timing.csv',['VERTICAL_CBF_timing_' referenceLabel '.csv']);
end
if ~isempty(refJetTrace)
    % Read-only, common 28-entry jet; plotted only after the flight stops/ends.
    jetNames=cell(1,29);jetNames{1}='time_s';
    axisNames={'x','y','z','yaw'};
    for ord=0:6
        for a=1:4
            jetNames{2+ord*4+a-1}=sprintf('%s_d%d',axisNames{a},ord);
        end
    end
    jetFile=['VERTICAL_CBF_reference_jet_' referenceLabel '.csv'];
    writetable(array2table(refJetTrace,'VariableNames',jetNames),jetFile);
    fprintf('[CBF-REF] reference derivative log saved: %s\n',jetFile);
end

if trialStopped
    fprintf(2,'[CBF-EXP] Trial stopped, not safety-certified. %s\n',trialStopReason);
    fprintf('[CBF-EXP] Active-step CSV: %s\n',hlc.cbfOpts.experimentalTraceFile);
else
    disp('[CBF-EXP] Simulation end reached; this alone is NOT a safety proof.');
end
% Mode-local results were already written into the isolated scenario folder.
% Plotting and animation are deferred until both independent runs finish.
fprintf('[CBF-COMPARE] %s: lastIntegrated=%.3f stopped=%d reason=%s\n', ...
    inputMode,time.t-dt,trialStopped,trialStopReason);
runResult=struct('mode',inputMode,'folder',workFolder, ...
    'stopped',logical(trialStopped),'reason',trialStopReason, ...
    'lastPathSample_s',NaN,'firstCBFEngage_s',NaN,'maxDeltaU_inf',NaN);
if ~isempty(trajectoryTrace),runResult.lastPathSample_s=trajectoryTrace(end,1);end
if ~isempty(inputCompareTrace)
    jj=find(inputCompareTrace(:,3)>0.5,1,'first');
    if ~isempty(jj),runResult.firstCBFEngage_s=inputCompareTrace(jj,1);end
    d=inputCompareTrace(:,22);d=d(isfinite(d));
    if ~isempty(d),runResult.maxDeltaU_inf=max(d);end
end

end % localRunOneFlight

%% LOCAL VERTICAL DIAGNOSTICS (READ-ONLY)
function row = localVerticalDiagnostic(t,agent)
% [time,z_load,z_reference,z_error,estimated_load_mass,thrust,Mx,My,Mz,CBF_hmin,CBF_input_delta]
row = nan(1,12);
row(1) = double(t);
try
    est = agent.estimator.result.state;
    ref = agent.reference.result.state;
    u = agent.controller.result.input(:);
    z = est.pL(3);
    if isprop(ref,'xd')
        zd = ref.xd(3);
    else
        xdtmp = ref.get();
        zd = xdtmp(3);
    end
    row(2:4) = [double(z),double(zd),double(z-zd)];
    if isprop(est,'mL')
        row(5) = double(est.mL);
    end
    if numel(u)>=4
        row(6:9) = double(u(1:4)).';
    end
    c = agent.controller.result;
    if isfield(c,'cbf') && isstruct(c.cbf) && isfield(c.cbf,'monitor')
        row(10) = c.cbf.monitor.minH;
        if isfield(c.cbf,'uSafe') && isfield(c.cbf,'uBase')
            row(11) = norm(c.cbf.uSafe(:)-c.cbf.uBase(:));
            row(12) = c.cbf.uSafe(4)-c.cbf.uBase(4);
        end
    end
catch
    % Diagnostic failure must not affect the plant/controller.
end
end

%% LOCAL FUNCTIONS
function y = motive_output(obj,data)
    p = data.rigid(obj.rigid_id(1)).p;
    pT = data.rigid(obj.rigid_id(2)).p - p;
    pT = pT/norm(pT);
    y = [p;Quat2Eul(data.rigid(obj.rigid_id(1)).q);data.rigid(obj.rigid_id(2)).p;pT];
end

function do_prop_local(time, cha, fCha, logger, env, agent, prop, i, isReady)
    if ~isReady && strcmp(prop, "plant")
        return;
    end
    if isfield(agent(i).cha_allocation, cha) && isfield(agent(i).cha_allocation.(cha), prop)
        list = agent(i).cha_allocation.(cha).(prop);
    else
        list = [];
    end
    if isempty(list)
        agent(i).(prop).result = agent(i).(prop).do(time, fCha, logger, env, agent, i);
    else
        agent(i).(prop).result = agent(i).(prop).(list(1)).do(time, fCha, logger, env, agent, i);
        for j = 2:length(list)
            agent(i).(prop).result = merge_result(agent(i).(prop).result, agent(i).(prop).(list(j)).do(time, fCha, logger, env, agent, i));
        end
    end
end


%% READ-ONLY FLIGHT PATH ACQUISITION AND PLOTTING
function v=localFlightPathSnapshot(t,phase,agent)
% t, phase, true UAV xyz, true load xyz, EKF UAV xyz, EKF load xyz,
% reference load xyz. This is independent of LOGGER plotting APIs.
persistent plantWarned estimatorWarned referenceWarned
v=nan(1,17);v(1)=double(t);
switch phase
    case 't',v(2)=1;
    case 'f',v(2)=2;
    case 'l',v(2)=3;
    otherwise,v(2)=0;
end
try
    v(3:5)=double(agent.plant.state.p(:)).';
    v(6:8)=double(agent.plant.state.pL(:)).';
catch ME
    if isempty(plantWarned)
        warning('VERTICAL:FlightPathPlant', ...
            'Cannot read plant position for flight plots: %s',ME.message);
        plantWarned=true;
    end
end
try
    v(9:11)=double(agent.estimator.result.state.p(:)).';
    v(12:14)=double(agent.estimator.result.state.pL(:)).';
catch ME
    if isempty(estimatorWarned)
        warning('VERTICAL:FlightPathEstimator', ...
            'Cannot read EKF position for flight plots: %s',ME.message);
        estimatorWarned=true;
    end
end
try
    ref=agent.reference.result.state;
    if isprop(ref,'xd')
        xd=ref.xd;
    else
        xd=ref.get();
    end
    v(15:17)=double(xd(1:3)).';
catch ME
    if isempty(referenceWarned)
        warning('VERTICAL:FlightPathReference', ...
            'Cannot read reference position for flight plots: %s',ME.message);
        referenceWarned=true;
    end
end
end

function localPlotFlightTrace(path,verticalTrace,runtimeTrace,obstacles,wasStopped,stopReason,label,cableLength,referenceLabel)
% Uses only the saved trajectory arrays. Works on a partial QP-stop run.
if isempty(path)
    warning('VERTICAL:NoFlightPath','No path data to draw.');
    return;
end
T=path(:,1); fMask=path(:,2)==2;
tr=path(:,3:5); tl=path(:,6:8);
er=path(:,9:11);el=path(:,12:14);rr=path(:,15:17);
if ~any(all(isfinite(tr),2))
    warning('VERTICAL:NoPlantTrajectory', ...
        'Plant position unavailable: check agent.plant.state.p and .pL.');
end
% Overlay the NO-CBF run only when its reference really matches this run.
% Never overlay old curved-path data under the same method label.
other=[];
if strcmp(label,'CBF')
    hlcFile=['VERTICAL_HLC_path_' referenceLabel '.csv'];
    if exist(hlcFile,'file')==2
        try
            candidate=readmatrix(hlcFile);
            if size(candidate,2)==17 && size(candidate,1)>3
                candidate=candidate(candidate(:,1)<=T(end)+1e-9,:);
                common=candidate(:,1)>=T(1) & candidate(:,1)<=T(end);
                otherRef=interp1(T,rr,candidate(common,1),'linear',NaN);
                match=any(common) && all(all(isfinite(otherRef))) && ...
                    max(abs(otherRef-candidate(common,15:17)),[],'all')<1e-2;
                if match,other=candidate;end
            end
        catch
            other=[];
        end
    end
end

% 1. 3D spatial trajectory with original obstacle bounding ellipsoids.
figure(1);clf; ax=gca;hold(ax,'on');grid(ax,'on');
localDrawObstacleEllipsoids(ax,obstacles);
plot3(ax,rr(:,1),rr(:,2),rr(:,3),'k--','LineWidth',1.3,'DisplayName','吊り荷目標');
plot3(ax,tr(:,1),tr(:,2),tr(:,3),'-','Color',[0 .40 .85], ...
    'LineWidth',1.7,'DisplayName','機体・実位置');
plot3(ax,tl(:,1),tl(:,2),tl(:,3),'-','Color',[.85 .20 .13], ...
    'LineWidth',1.7,'DisplayName','吊り荷・実位置');
if ~isempty(other)
    plot3(ax,other(:,3),other(:,4),other(:,5),'--', ...
        'Color',[0 .63 .85],'LineWidth',1,'DisplayName','CBFなし機体（比較）');
    plot3(ax,other(:,6),other(:,7),other(:,8),'--', ...
        'Color',[.8 .50 .28],'LineWidth',1,'DisplayName','CBFなし荷物（比較）');
end
last=find(all(isfinite(tr),2),1,'last');
if ~isempty(last)
    plot3(ax,tr(last,1),tr(last,2),tr(last,3),'o', ...
        'MarkerSize',8,'MarkerFaceColor',[0 .4 .85],'MarkerEdgeColor','k', ...
        'DisplayName','機体の最終記録点');
    plot3(ax,tl(last,1),tl(last,2),tl(last,3),'o', ...
        'MarkerSize',7,'MarkerFaceColor',[.85 .2 .13],'MarkerEdgeColor','k', ...
        'DisplayName','荷物の最終記録点');
end
xlabel(ax,'x [m]');ylabel(ax,'y [m]');zlabel(ax,'z [m]');
title(ax,sprintf('%s | 3D軌跡（障害物は外接楕円体）%s',label,localStopTag(wasStopped)));
view(ax,35,23);legend(ax,'Location','best');

% 2. Top view is essential: long vertical flight hides lateral avoidance.
figure(2);clf;ax=gca;hold(ax,'on');grid(ax,'on');
localDrawObstacleFootprints(ax,obstacles);
plot(ax,rr(:,1),rr(:,2),'k--','LineWidth',1.4,'DisplayName','吊り荷目標');
plot(ax,tr(:,1),tr(:,2),'-','Color',[0 .4 .85], ...
    'LineWidth',1.7,'DisplayName','機体・実位置');
plot(ax,tl(:,1),tl(:,2),'-','Color',[.85 .20 .13], ...
    'LineWidth',1.7,'DisplayName','吊り荷・実位置');
plot(ax,er(:,1),er(:,2),':','Color',[0 .65 .7], ...
    'LineWidth',1,'DisplayName','機体EKF');
plot(ax,el(:,1),el(:,2),':','Color',[.6 0 .5], ...
    'LineWidth',1,'DisplayName','吊り荷EKF');
if ~isempty(other)
    plot(ax,other(:,3),other(:,4),'--','Color',[.25 .65 .95], ...
        'LineWidth',1,'DisplayName','CBFなし機体');
    plot(ax,other(:,6),other(:,7),'--','Color',[.95 .55 .4], ...
        'LineWidth',1,'DisplayName','CBFなし荷物');
end
if ~isempty(last)
    plot(ax,tr(last,1),tr(last,2),'bo','MarkerFaceColor','b', ...
        'HandleVisibility','off');
    plot(ax,tl(last,1),tl(last,2),'ro','MarkerFaceColor','r', ...
        'HandleVisibility','off');
end
axis(ax,'equal');xlabel(ax,'x [m]');ylabel(ax,'y [m]');
title(ax,sprintf('%s | 上面図：横方向への回避量と障害物の水平投影',label));
legend(ax,'Location','best');

% 3. Vertical tracking and lateral motion over simulation time.
figure(3);clf;
ax1=subplot(2,1,1);hold(ax1,'on');grid(ax1,'on');
plot(ax1,T,rr(:,3),'k--','LineWidth',1.3,'DisplayName','吊り荷z目標');
plot(ax1,T,tl(:,3),'Color',[.85 .2 .13],'LineWidth',1.4,'DisplayName','吊り荷z実位置');
plot(ax1,T,el(:,3),'Color',[.6 0 .5],'LineStyle',':', ...
    'DisplayName','吊り荷z推定');
plot(ax1,T,tr(:,3),'Color',[0 .4 .85],'LineWidth',1.3,'DisplayName','機体z実位置');
plot(ax1,T,rr(:,3)+cableLength,'Color',[.2 .4 .2],'LineStyle','--', ...
    'DisplayName','機体z目標（荷物目標+紐長）');
ylabel(ax1,'高度 [m]');title(ax1,sprintf('%s | 高度追従',label));
legend(ax1,'Location','best');
ax2=subplot(2,1,2);hold(ax2,'on');grid(ax2,'on');
plot(ax2,T,tl(:,1),'LineWidth',1.3,'DisplayName','荷物 x');
plot(ax2,T,tl(:,2),'LineWidth',1.3,'DisplayName','荷物 y');
plot(ax2,T,tr(:,1),'--','LineWidth',1.1,'DisplayName','機体 x');
plot(ax2,T,tr(:,2),'--','LineWidth',1.1,'DisplayName','機体 y');
xlabel(ax2,'時刻 [s]');ylabel(ax2,'横方向位置 [m]');
legend(ax2,'Location','best');

% 4. Existing useful time plot, now computed from the same run data.
figure(4);clf;ax=gca;hold(ax,'on');grid(ax,'on');
if ~isempty(runtimeTrace)
    plot(ax,runtimeTrace(:,1),1000*runtimeTrace(:,3), ...
        'LineWidth',1.2,'DisplayName','HLC + CBF controller [ms]');
    plot(ax,runtimeTrace(:,1),1000*runtimeTrace(:,4), ...
        'LineWidth',1.0,'DisplayName','全step [ms]');
    if strcmp(label,'CBF')
        plot(ax,runtimeTrace(:,1),1000*runtimeTrace(:,5), ...
            'LineWidth',1.1,'DisplayName','CBF単独 [ms]');
    end
end
xlabel(ax,'時刻 [s]');ylabel(ax,'計算時間 [ms]');
title(ax,sprintf('%s | 計算時間（参考・ノートPC）',label));
legend(ax,'Location','best');

% 5. Safety margin and yaw correction, without forcing any intervention.
if strcmp(label,'CBF') && ~isempty(verticalTrace)
    figure(5);clf;
    ax1=subplot(3,1,1);hold(ax1,'on');grid(ax1,'on');
    plot(ax1,verticalTrace(:,1),verticalTrace(:,10),'LineWidth',1.2);
    plot(ax1,[T(1),T(end)],[0,0],'k--');
    ylabel(ax1,'CBF hMin');title(ax1,'CBF幾何学的分離余裕（距離[m]ではない）');
    ax2=subplot(3,1,2);hold(ax2,'on');grid(ax2,'on');
    plot(ax2,verticalTrace(:,1),verticalTrace(:,11),'LineWidth',1.2);
    ylabel(ax2,'||uCBF-uHLC||');title(ax2,'CBF補正量');
    ax3=subplot(3,1,3);hold(ax3,'on');grid(ax3,'on');
    if size(verticalTrace,2)>=12
        plot(ax3,verticalTrace(:,1),verticalTrace(:,12),'LineWidth',1.2);
    end
    plot(ax3,[T(1),T(end)],[0,0],'k--');
    xlabel(ax3,'時刻 [s]');ylabel(ax3,'ΔMz [Nm]');
    title(ax3,'CBFによるyawトルク補正（ゼロのはず）');
end

fprintf('[VERTICAL-PLOT] 3D, XY, altitude, timing%s plotted (%d samples, t<=%.3f s).\n', ...
    localCBFSuffix(label),size(path,1),T(end));
if wasStopped
    fprintf('[VERTICAL-PLOT] Stopped trial: trajectory is PARTIAL; last integrated sample at %.3f s.\n',T(end));
    fprintf('[VERTICAL-PLOT] Stop reason: %s\n',stopReason);
end
end

function s=localStopTag(stopped)
if stopped,s='・途中停止まで';else,s='';end
end
function s=localCBFSuffix(label)
if strcmp(label,'CBF'),s=', CBF margins and yaw';else,s='';end
end

function localDrawObstacleEllipsoids(ax,obstacles)
if isempty(obstacles),return;end
[xx,yy,zz]=sphere(16);
for i=1:numel(obstacles)
    ob=obstacles(i);r=double(ob.ellipsoid_radii(:));
    center=double(ob.p_center(:));R=double(ob.R_obs);
    xyz=R*diag(r)*[xx(:).';yy(:).';zz(:).'];
    xyz=xyz+center;
    surf(ax,reshape(xyz(1,:),size(xx)),reshape(xyz(2,:),size(xx)), ...
        reshape(xyz(3,:),size(xx)), ...
        'FaceColor',[.5 .5 .5],'EdgeColor','none','FaceAlpha',0.17, ...
        'HandleVisibility','off');
    plot3(ax,center(1),center(2),center(3),'kx','MarkerSize',7, ...
        'LineWidth',1.5,'HandleVisibility','off');
    text(ax,center(1),center(2),center(3),sprintf('  obs%d',i), ...
        'FontSize',9,'HandleVisibility','off');
end
end

function localDrawObstacleFootprints(ax,obstacles)
if isempty(obstacles),return;end
th=linspace(0,2*pi,121);
for i=1:numel(obstacles)
    ob=obstacles(i); c=double(ob.p_center(:));
    r=double(ob.ellipsoid_radii(:));
    margin=0;
    if isfield(ob,'d_margin'),margin=double(ob.d_margin);end
    % Horizontal projection of axis-aligned enclosing ellipsoid, with an
    % APPROXIMATE additive margin for visual orientation only (not S3 h).
    plot(ax,c(1)+(r(1)+margin)*cos(th),c(2)+(r(2)+margin)*sin(th), ...
        '-','Color',[.6 .6 .6],'LineWidth',1.1,'HandleVisibility','off');
    text(ax,c(1),c(2),sprintf('obs%d',i),'FontSize',9, ...
        'HandleVisibility','off');
end
end


%% READ-ONLY STAGE-AWARE FLIGHT FORENSICS (NO CONTROL/ESTIMATOR MUTATION)
function d=localDiagStart(t,dt,agent)
% At this point the sensor has NOT updated and the estimator has NOT run.
d=struct();
d.t=double(t);d.dt=double(dt);d.complete=false;d.failed=false;
d.failureID='';d.failureMessage='';
d.plantBefore=localDiagState(localDiagGet(agent.plant,'state'));
d.ekfBefore=localDiagState(localDiagEKFState(agent));
d.previousInput=localDiagNum(localDiagGet(agent.controller.result,'input'),4);
d.sensor=struct('y',nan(12,1),'qRaw',nan(4,1));
d.sensorEulerForEKF=nan(3,1);
d.sensorEulerChartChanged=false;
d.ekfAfter=localDiagState([]);
d.estimatorMerged=localDiagState([]);
d.kalmanGainFro=nan;
d.plantAfter=localDiagState([]);
d.rawOnFailedCall=nan(4,1);
d.rawHLC=nan(4,1);d.nominalHLC=nan(4,1);
d.safeCBF=nan(4,1);d.commandToPlant=nan(4,1);
d.cbfActive=nan;d.cbfHMin=nan;d.cbfMargin=nan;
d.hlcX=[];d.hlcXd=[];d.hlcP=[];d.hlcF1=[];
d.beta2Rcond=nan;d.beta2Cond=nan;
end

function st=localDiagEKFState(agent)
st=[];
try
    ekf=localDiagGet(agent.estimator,'ekf');
    result=localDiagGet(ekf,'result');
    st=localDiagGet(result,'state');
catch
end
end

function st=localDiagMergedState(agent)
st=[];
try
    st=localDiagGet(localDiagGet(agent.estimator,'result'),'state');
catch
end
end

function g=localDiagKalmanGain(agent)
g=nan;
try
    ekf=localDiagGet(agent.estimator,'ekf');
    r=localDiagGet(ekf,'result');
    G=localDiagGet(r,'G');
    if isnumeric(G) && all(isfinite(G(:)))
        g=norm(double(G),'fro');
    end
catch
end
end

function s=localDiagState(state)
% Copy values by value. Never retain live STATE_CLASS handle references.
s=struct('euler',nan(3,1),'quat',nan(4,1),'w',nan(3,1), ...
    'p',nan(3,1),'v',nan(3,1),'pL',nan(3,1), ...
    'vL',nan(3,1),'pT',nan(3,1),'wL',nan(3,1),'mL',nan);
if isempty(state),return;end
s.euler=localDiagNum(localDiagGet(state,'q'),3);
s.w=localDiagNum(localDiagGet(state,'w'),3);
s.p=localDiagNum(localDiagGet(state,'p'),3);
s.v=localDiagNum(localDiagGet(state,'v'),3);
s.pL=localDiagNum(localDiagGet(state,'pL'),3);
s.vL=localDiagNum(localDiagGet(state,'vL'),3);
s.pT=localDiagNum(localDiagGet(state,'pT'),3);
s.wL=localDiagNum(localDiagGet(state,'wL'),3);
s.mL=localDiagNum(localDiagGet(state,'mL'),1);
try
    % The HLC and prior diagnostics both use scalar-first compact quat.
    s.quat=localDiagNum(state.getq('compact'),4);
catch
    if all(isfinite(s.euler))
        s.quat=localDiagNum(Eul2Quat(s.euler),4);
    end
end
end

function s=localDiagSensor(agent)
s=struct('y',nan(12,1),'qRaw',nan(4,1));
try
    res=localDiagGet(agent.sensor,'result');
    y=localDiagGet(res,'output');
    if isempty(y)
        m=localDiagGet(agent.sensor,'motive');
        y=localDiagGet(localDiagGet(m,'result'),'output');
    end
    s.y=localDiagNum(y,12);
    m=localDiagGet(agent.sensor,'motive');
    ms=localDiagGet(localDiagGet(m,'result'),'state');
    if ~isempty(ms)
        s.qRaw=localDiagNum(localDiagGet(ms(1),'q'),4);
    end
catch
end
end

function d=localDiagController(d,agent,hlc)
% Do NOT use old controller results on a failed controller call: this is
% only invoked after do_prop_local(controller) has returned successfully.
try
    r=agent.controller.result;
    d.rawHLC=localDiagNum(localDiagGet(r,'tmp'),4);
    d.commandToPlant=localDiagNum(localDiagGet(r,'input'),4);
    d.hlcX=localDiagGet(r,'x');
    d.hlcXd=localDiagGet(r,'xd');
    d.hlcF1=localDiagGet(hlc.param,'F1');
    d.hlcP=[agent.parameter.get(["mass","jx","jy","jz", ...
        "gravity","loadmass","cableL"]),0,0];
    cbf=localDiagGet(r,'cbf');
    if isstruct(cbf) && ~isempty(cbf)
        d.nominalHLC=localDiagNum(localDiagGet(cbf,'uBase'),4);
        d.safeCBF=localDiagNum(localDiagGet(cbf,'uSafe'),4);
        sup=localDiagGet(cbf,'supervisor');
        active=localDiagGet(sup,'active');
        if islogical(active) || (isnumeric(active) && isscalar(active))
            d.cbfActive=double(active);
        end
        mon=localDiagGet(cbf,'monitor');
        h=localDiagGet(mon,'minH');
        mar=localDiagGet(mon,'minNormalizedMargin');
        if isnumeric(h) && isscalar(h),d.cbfHMin=double(h);end
        if isnumeric(mar) && isscalar(mar),d.cbfMargin=double(mar);end
    end
catch ME
    warning('CBF:ReadOnlyDiagnosticCapture', ...
        'Could not read one controller diagnostic: %s',ME.message);
end
end

function v=localDiagGet(node,field)
v=[];
try
    if isstruct(node)
        if isfield(node,field),v=node.(field);end
    elseif isobject(node)
        if isprop(node,field),v=node.(field);end
    end
catch
end
end

function v=localDiagNum(input,n)
v=nan(n,1);
if isnumeric(input) && numel(input)==n
    v=double(input(:));
end
end

function angleDeg=localDiagQuatAngle(a,b)
angleDeg=nan;
a=a(:);b=b(:);
if numel(a)~=4 || numel(b)~=4 || any(~isfinite([a;b]))
    return
end
den=norm(a)*norm(b);
if den<eps,return;end
c=abs(dot(a,b))/den;
angleDeg=(360/pi)*acos(min(1,max(0,c)));
end

function val=localDiagCableOrthogonality(s)
val=nan;
if all(isfinite([s.pT(:);s.wL(:)]))
    val=dot(s.pT,s.wL);
end
end

function localDiagWrite(records,windowStart,windowEnd,dt,trialStopped,stopReason)
% Records are SAVED even if the last sample ended with MonitorInvalid/QP fail.
if isempty(records)
    fprintf('[CBF-DIAG] No samples in trace window [%.2f %.2f] s.\n', ...
        windowStart,windowEnd);
    return;
end
names={'time_s','complete_step','failed_step', ...
    'plant_roll_rad','plant_pitch_rad','plant_yaw_rad', ...
    'ekf_prior_roll_rad','ekf_prior_pitch_rad','ekf_prior_yaw_rad', ...
    'sensor_roll_rad','sensor_pitch_rad','sensor_yaw_rad', ...
    'ekf_post_roll_rad','ekf_post_pitch_rad','ekf_post_yaw_rad', ...
    'plant_wx','plant_wy','plant_wz', ...
    'ekf_prior_wx','ekf_prior_wy','ekf_prior_wz', ...
    'ekf_post_wx','ekf_post_wy','ekf_post_wz', ...
    'plant_pT_dot_wL','ekf_post_pT_dot_wL', ...
    'ekf_plant_attitude_error_deg','ekf_prior_to_post_angle_deg', ...
    'ekf_plant_w_error_norm_rad_s','ekf_prior_to_post_w_delta_rad_s', ...
    'raw_euler_prior_sensor_maxdiff_deg','wrapped_euler_prior_sensor_maxdiff_deg', ...
    'ekf_gain_fro','cbf_active','cbf_hMin','cbf_sigma', ...
    'raw_hlc_thrust','raw_hlc_mx','raw_hlc_my','raw_hlc_mz', ...
    'raw_failed_call_thrust','raw_failed_call_mx','raw_failed_call_my','raw_failed_call_mz', ...
    'nominal_thrust','nominal_mx','nominal_my','nominal_mz', ...
    'safe_thrust','safe_mx','safe_my','safe_mz', ...
    'applied_thrust','applied_mx','applied_my','applied_mz', ...
    'cbf_delta_input_norm','beta2_rcond','beta2_cond', ...
    'plant_pT_norm','ekf_post_pT_norm','ekf_post_mass_kg'};
rows=nan(numel(records),numel(names));
for k=1:numel(records)
    d=records{k};
    % HLC inverse-matrix conditioning is recomputed POST-FLIGHT from the
    % exact successful HLC x/xd and current parameters; no control changes.
    if d.complete && ~isempty(d.hlcX) && ~isempty(d.hlcXd) ...
            && ~isempty(d.hlcP) && ~isempty(d.hlcF1)
        try
            vf=Vfd_SuspendedLoadxyDst([],d.dt,d.hlcX(:), ...
                d.hlcXd(:).',d.hlcF1(:));
            B=Beta2_SuspendedLoadxyDst([],d.hlcX(:), ...
                d.hlcXd(:).',vf,d.hlcP);
            if isnumeric(B) && size(B,1)==size(B,2) && all(isfinite(B(:)))
                d.beta2Rcond=rcond(B);
                d.beta2Cond=cond(B);
            end
        catch
            % Unavailable data is NaN, not manufactured evidence.
        end
    end
    records{k}=d;
    sen=d.sensor.y(4:6);
    prior=d.ekfBefore.euler;
    raw=nan;wrapped=nan;
    if all(isfinite(sen)) && all(isfinite(prior))
        delta=sen-prior;
        raw=max(abs(delta))*180/pi;
        wrapped=max(abs(atan2(sin(delta),cos(delta))))*180/pi;
    end
    attErr=localDiagQuatAngle(d.ekfAfter.quat,d.plantBefore.quat);
    postChange=localDiagQuatAngle(d.ekfAfter.quat,d.ekfBefore.quat);
    wErr=nan;wChange=nan;
    if all(isfinite([d.ekfAfter.w;d.plantBefore.w]))
        wErr=norm(d.ekfAfter.w-d.plantBefore.w);
    end
    if all(isfinite([d.ekfAfter.w;d.ekfBefore.w]))
        wChange=norm(d.ekfAfter.w-d.ekfBefore.w);
    end
    deltaU=nan;
    if all(isfinite([d.safeCBF;d.nominalHLC]))
        deltaU=norm(d.safeCBF-d.nominalHLC);
    end
    rows(k,:)=[d.t,double(d.complete),double(d.failed), ...
        d.plantBefore.euler.',d.ekfBefore.euler.',sen.',d.ekfAfter.euler.', ...
        d.plantBefore.w.',d.ekfBefore.w.',d.ekfAfter.w.', ...
        localDiagCableOrthogonality(d.plantBefore), ...
        localDiagCableOrthogonality(d.ekfAfter), ...
        attErr,postChange,wErr,wChange,raw,wrapped, ...
        d.kalmanGainFro,d.cbfActive,d.cbfHMin,d.cbfMargin, ...
        d.rawHLC.',d.rawOnFailedCall.',d.nominalHLC.',d.safeCBF.',d.commandToPlant.', ...
        deltaU,d.beta2Rcond,d.beta2Cond, ...
        norm(d.plantBefore.pT),norm(d.ekfAfter.pT),d.ekfAfter.mL];
end
% Document the sample phases and what this file does NOT contain.
forensic=struct();
forensic.revision='CBF_READONLY_STAGE_TRACE_20261015';
forensic.window_s=[windowStart,windowEnd];
forensic.dt=dt;
forensic.records=records;
forensic.completedFlight=~trialStopped;
forensic.trialStopReason=stopReason;
forensic.explanation=['plantBefore is physical state at t, plantAfter is at t+dt. ' ...
    'ekfBefore is previous posterior BEFORE estimator.do; ekfAfter is ' ...
    'posterior AFTER estimator.do + loadstate. EKF internal xh_pre and ' ...
    'its exact measurement innovation are NOT accessible without modifying EKF. ' ...
    'rawOnFailedCall is an HLC internal partial write, NOT an applied command. ' ...
    'beta2 condition numbers are computed offline only on successful cycles.'];
try
    save('CBF_flight_state_diagnostic.mat','forensic','-v7');
    writetable(array2table(rows,'VariableNames',names), ...
        'CBF_flight_state_diagnostic.csv');
    fprintf('[CBF-DIAG] Saved CBF_flight_state_diagnostic.mat and .csv (%d samples, %.3f to %.3f s).\n', ...
        size(rows,1),rows(1,1),rows(end,1));
catch ME
    warning('CBF:ReadOnlyDiagnosticSave', ...
        'Cannot save stage-aware CBF diagnostics: %s',ME.message);
end
% Factual timeline summaries, not assumptions about causality.
localDiagFirstThreshold(rows,names,'ekf_plant_attitude_error_deg',30, ...
    'EKF-vs-plant attitude mismatch >30 deg');
localDiagFirstThreshold(rows,names,'ekf_prior_to_post_angle_deg',20, ...
    'prior-vs-post EKF quaternion change >20 deg');
localDiagFirstThreshold(rows,names,'ekf_post_pT_dot_wL',0.001, ...
    '|EKF pT-dot-wL| > 0.001',true);
localDiagFirstThreshold(rows,names,'raw_hlc_thrust',20, ...
    'raw HLC thrust >20 N');
try
    t=rows(:,1);
    figure(6);clf;
    ax1=subplot(3,1,1);hold(ax1,'on');grid(ax1,'on');
    plot(ax1,t,rows(:,strcmp(names,'ekf_plant_attitude_error_deg')),'LineWidth',1.2);
    plot(ax1,t,rows(:,strcmp(names,'ekf_prior_to_post_angle_deg')),'LineWidth',1.2);
    ylabel(ax1,'角度差 [deg]');legend(ax1,'EKF対プラント','EKF更新前対更新後');
    title(ax1,'CBF診断：姿勢推定の急変と真値との差');
    ax2=subplot(3,1,2);hold(ax2,'on');grid(ax2,'on');
    plot(ax2,t,rows(:,strcmp(names,'ekf_plant_w_error_norm_rad_s')),'LineWidth',1.2);
    ylabel(ax2,'角速度差 [rad/s]');
    ax3=subplot(3,1,3);hold(ax3,'on');grid(ax3,'on');
    plot(ax3,t,rows(:,strcmp(names,'ekf_post_pT_dot_wL')),'LineWidth',1.2);
    plot(ax3,t,rows(:,strcmp(names,'plant_pT_dot_wL')),'LineWidth',1.2);
    plot(ax3,[t(1),t(end)],[.001,.001],'k--');
    plot(ax3,[t(1),t(end)],[-.001,-.001],'k--');
    ylabel(ax3,'p_T^T\omega_L');xlabel(ax3,'時刻 [s]');
    legend(ax3,'EKF','プラント','閾値','Location','best');
    figure(7);clf;
    b1=subplot(2,1,1);hold(b1,'on');grid(b1,'on');
    plot(b1,t,rows(:,strcmp(names,'raw_hlc_thrust')),'LineWidth',1.1);
    plot(b1,t,rows(:,strcmp(names,'applied_thrust')),'LineWidth',1.1);
    ylabel(b1,'推力 [N]');legend(b1,'HLC飽和前','CBF後・プラント入力');
    title(b1,'CBF診断：制御入力と逆変換行列の条件数');
    b2=subplot(2,1,2);hold(b2,'on');grid(b2,'on');
    plot(b2,t,rows(:,strcmp(names,'beta2_rcond')),'LineWidth',1.1);
    xlabel(b2,'時刻 [s]');ylabel(b2,'rcond(\beta_2)');
    drawnow;
catch ME
    warning('CBF:ReadOnlyDiagnosticPlot', ...
        'Diagnostic CSV/MAT saved but figures unavailable: %s',ME.message);
end
end

function localDiagFirstThreshold(rows,names,key,threshold,description,absolute)
if nargin<6,absolute=false;end
col=rows(:,strcmp(names,key));
if absolute,col=abs(col);end
idx=find(isfinite(col) & col>threshold,1,'first');
if ~isempty(idx)
    fprintf('[CBF-DIAG-FIRST] first observed in window t=%.3f s: %s (value=%.6g)\n', ...
        rows(idx,1),description,rows(idx,strcmp(names,key)));
else
    fprintf('[CBF-DIAG-FIRST] %s: not observed in recorded samples.\n',description);
end
end

%% READ-ONLY 28x1 REFERENCE TRACE AND MODE COMPARISON (PLOTS ONLY)
function row=localReferenceJetSnapshot(t,agent)
row=nan(1,29);row(1)=double(t);
try
    ref=agent.reference.result.state;
    if isprop(ref,'xd'), xd=ref.xd; else, xd=ref.get(); end
    xd=double(xd(:));
    if numel(xd)==28, row(2:end)=xd.'; end
catch
    % A plotting failure must not affect the controller or QP.
end
end

function localPlotCBFThreeModeComparison()
% Each mode writes its own CSV; any CBF-controlled stop leaves a partial path.
labels={'original_line','poly_c6','bspline_c6'};
displayNames={'1: 等速直線','2: 13次多項式','3: 7次B-spline'};
figure(8);clf;
a1=subplot(2,1,1);hold(a1,'on');grid(a1,'on');
a2=subplot(2,1,2);hold(a2,'on');grid(a2,'on');
found=0;
for m=1:3
    f=['VERTICAL_CBF_path_' labels{m} '.csv'];
    if exist(f,'file')~=2,continue;end
    try
        P=readtable(f);
        if ~all(ismember({'time_s','uav_x_m','uav_y_m','load_x_m', ...
                'load_y_m','load_z_m','ref_load_z_m'},P.Properties.VariableNames))
            continue;
        end
        good=isfinite(P.time_s) & P.time_s>=5 & ...
            isfinite(P.uav_x_m) & isfinite(P.uav_y_m) & ...
            isfinite(P.load_x_m) & isfinite(P.load_y_m);
        if ~any(good),continue;end
        if max(abs(P.ref_load_x_m(good)))>1e-4 || ...
                max(abs(P.ref_load_y_m(good)))>1e-4
            fprintf('[CBF-REF] skip old nonvertical comparison file: %s\n',f);
            continue;
        end
        plot(a1,P.uav_x_m(good),P.uav_y_m(good),'-', ...
            'LineWidth',1.5,'DisplayName',[displayNames{m} ' 機体']);
        plot(a1,P.load_x_m(good),P.load_y_m(good),'--', ...
            'LineWidth',1.1,'DisplayName',[displayNames{m} ' 吊り荷']);
        plot(a2,P.time_s(good),P.load_z_m(good)-P.ref_load_z_m(good), ...
            'LineWidth',1.3,'DisplayName',displayNames{m});
        found=found+1;
    catch ME
        warning('CBF:ModePlotRead','Cannot read %s: %s',f,ME.message);
    end
end
if found==0
    close(8);return;
end
axis(a1,'equal');xlabel(a1,'x [m]');ylabel(a1,'y [m]');
title(a1,'CBFあり：3方式のXY実軌跡（途中停止は記録範囲のみ）');
legend(a1,'Location','best');
xlabel(a2,'時刻 [s]');ylabel(a2,'吊り荷z追従誤差 [m]');
title(a2,'同一障害物・CBF設定での高度追従');
legend(a2,'Location','best');
fprintf('[CBF-REF] Figure 8: %d available method paths overlaid.\n',found);
end

function localPlotCBFReferenceJet(R,label)
if isempty(R),return;end
valid=all(isfinite(R),2) & R(:,1)>=5 & R(:,1)<47;
if ~any(valid),return;end
T=R(valid,1);X=R(valid,2:end);
figure(9);clf;
a1=subplot(2,1,1);hold(a1,'on');grid(a1,'on');
plot(a1,T,X(:,3),'LineWidth',1.4,'DisplayName','z目標');
plot(a1,T,X(:,7),'LineWidth',1.2,'DisplayName','z目標速度');
title(a1,['CBF / ' label '：目標高さ・速度']);
ylabel(a1,'高さ[m] / 速度[m/s]');legend(a1,'Location','best');
a2=subplot(2,1,2);hold(a2,'on');grid(a2,'on');
for k=2:6
    c=4*k+3; % z derivative index in 28-vector
    plot(a2,T,X(:,c),'DisplayName',sprintf('z %d階',k));
end
xlabel(a2,'時刻 [s]');ylabel(a2,'目標z高階時間微分');
title(a2,'2～6階微分（単位は次数別）');
legend(a2,'Location','best');
end


%% 2026-10-16: READ-ONLY FLIGHT-ONLY HLC/QP INPUT AUDIT
function row=localFlightHlcAuditRow(t,hlc,failed)
% 28-column scalar row; NaN means unavailable, NOT zero or pass.
row=nan(1,28);row(1:2)=[double(t),double(failed)];
try
    if ~isstruct(hlc.result) || ~isfield(hlc.result,'flightHlcAudit')
        return;
    end
    a=hlc.result.flightHlcAudit;
    names={'beta2Rcond','beta2SigmaMin','beta2SigmaMax', ...
        'beta2RHSNorm','beta2RHSInf','loadCableDirectionZ', ...
        'zRef0','zRef1','zRef2','zRef3','zRef4','zRef5','zRef6', ...
        'rawThrust','rawMomentInf','rawMoment2'};
    for j=1:numel(names)
        if isfield(a,names{j}) && isnumeric(a.(names{j})) && ...
                isscalar(a.(names{j}))
            row(j+2)=double(a.(names{j}));
        end
    end
    extraNames={'nominalV1','clipEquivalentV1','clipEquivalentDeltaV1', ...
        'thrustClippingDelta_N','torqueClippingDeltaInf_Nm', ...
        'rawSecondLayerResidualRel','clippedTorqueVirtualProxyRel', ...
        'nominalHierarchyUnchangedAfterClip'};
    for j=1:numel(extraNames)
        if isfield(a,extraNames{j}) && isnumeric(a.(extraNames{j})) && ...
                isscalar(a.(extraNames{j}))
            row(j+20)=double(a.(extraNames{j}));
        end
    end
    if ~failed && isfield(hlc.result,'tmp') && isnumeric(hlc.result.tmp) && ...
            numel(hlc.result.tmp)==4 && ...
            isfield(hlc.result,'cbf') && isstruct(hlc.result.cbf) && ...
            isfield(hlc.result.cbf,'uBase') && ...
            numel(hlc.result.cbf.uBase)==4
        row(19)=norm(hlc.result.tmp(:)-hlc.result.cbf.uBase(:),inf);
        if isfield(hlc.result.cbf,'uSafe') && numel(hlc.result.cbf.uSafe)==4
            row(20)=norm(hlc.result.cbf.uBase(:)-hlc.result.cbf.uSafe(:),inf);
        end
    end
catch
    % Never let a read-only trace change the control/plant outcome.
end
end


%% FLIGHT-ONLY Euler measurement chart check / alignment
% EKF.m takes Euclidean y-yh, while Quat2Eul wraps at +/-pi.
% Re-choose equivalent sensor Euler representatives relative to the last
% posterior's Euler chart BEFORE calling the ORIGINAL EKF.do.
% Only a 2*pi chart branch is fixed. This does NOT address gimbal lock,
% SO(3) covariance, HLC beta2 singularity, or QP recursive feasibility.
function d=localFlightEulerChartObserve(agent)
    prev=localDiagEKFState(agent);
    if isempty(prev)
        error('CBF:FlightEulerEKFStateMissing', ...
            'Cannot read previous EKF posterior for flight sensor chart.');
    end
    prior=localDiagNum(localDiagGet(prev,'q'),3);
    sensorResult=localDiagGet(agent.sensor,'result');
    y=localDiagGet(sensorResult,'output');
    if ~isnumeric(y) || numel(y)~=12 || ~isreal(y) || ...
            ~all(isfinite(y(:))) || ~all(isfinite(prior))
        error('CBF:FlightEulerMeasurementInvalid', ...
            'Flight angle chart requires finite real 12x1 sensor output and 3x1 EKF Euler.');
    end
    raw=double(y(4:6)); raw=raw(:);
    rawDelta=raw-prior;
    wrappedDelta=atan2(sin(rawDelta),cos(rawDelta));
    d=struct('prior',prior(:),'raw',raw, ...
        'adjusted',prior(:)+wrappedDelta, ...
        'rawGapMax',max(abs(rawDelta)), ...
        'wrappedGapMax',max(abs(wrappedDelta)), ...
        'changed',any(abs(rawDelta-wrappedDelta)>1e-8));
end

function d=localFlightEulerChartAlign(agent)
    d=localFlightEulerChartObserve(agent);
    if ~d.changed
        return;
    end
    % Write only the flight-phase COMBINED sensor result used by EKF.do.
    % The raw sensor/MOTIVE source remains unchanged and is logged BEFORE
    % this call. The estimator and its gains remain byte-identical.
    sensorResult=agent.sensor.result;
    originalShape=size(sensorResult.output);
    tmp=double(sensorResult.output(:));
    tmp(4:6)=d.adjusted(:);
    sensorResult.output=reshape(tmp,originalShape);
    agent.sensor.result=sensorResult;
end


%% Full-flight input/CBF/HLC audit (both selected modes; scalar rows, NaN if absent)
function names=localInputCompareColumnNames()
names={'time_s','failed','cbf_active','requested_virtual','effective_mode', ...
    'engaged','u_raw_f_N','u_raw_Mx_Nm','u_raw_My_Nm','u_raw_Mz_Nm', ...
    'u_base_f_N','u_base_Mx_Nm','u_base_My_Nm','u_base_Mz_Nm', ...
    'u_safe_f_N','u_safe_Mx_Nm','u_safe_My_Nm','u_safe_Mz_Nm', ...
    'raw_safe_diff_inf','raw_base_diff_inf','base_safe_diff_inf', ...
    'deltaU_inf','aux_1','aux_2','aux_3','aux_4', ...
    'hmin_m','sigma','min_psi','beta2_rcond','raw_moment_inf_Nm', ...
    'qp_exitflag','cbf_cpu_s'};
end

function row=localInputCompareRow(t,hlc,failed)
row=nan(1,33);row(1)=double(t);row(2)=double(failed);
row(4)=double(strcmpi(hlc.cbfMode,'virtual'));
if failed,row(3)=0;else,row(3)=0;end
try
    if isstruct(hlc.result) && isfield(hlc.result,'tmp') && numel(hlc.result.tmp)==4
        u=double(hlc.result.tmp(:));row(7:10)=u.';
    end
    if isstruct(hlc.result) && isfield(hlc.result,'flightHlcAudit') && ...
            isstruct(hlc.result.flightHlcAudit)
        a=hlc.result.flightHlcAudit;
        if isfield(a,'beta2Rcond'),row(30)=double(a.beta2Rcond);end
        if isfield(a,'rawMomentInf'),row(31)=double(a.rawMomentInf);end
    end
    if isstruct(hlc.result) && isfield(hlc.result,'cbf') && ...
            isstruct(hlc.result.cbf) && ~isempty(fieldnames(hlc.result.cbf))
        c=hlc.result.cbf;
        if isfield(c,'uBase') && numel(c.uBase)==4,row(11:14)=double(c.uBase(:)).';end
        if isfield(c,'uSafe') && numel(c.uSafe)==4,row(15:18)=double(c.uSafe(:)).';end
        if all(isfinite(row([7:10 15:18]))),row(19)=norm(row(7:10)-row(15:18),inf);end
        if all(isfinite(row([7:10 11:14]))),row(20)=norm(row(7:10)-row(11:14),inf);end
        if all(isfinite(row(11:18))),row(21)=norm(row(11:14)-row(15:18),inf);end
        row(22)=row(21);
        if isfield(c,'supervisor') && isstruct(c.supervisor)
            st=c.supervisor;
            if isfield(st,'active'),row(3)=double(st.active);end
            if isfield(st,'aux') && isnumeric(st.aux)
                n=min(4,numel(st.aux));row(22+(1:n))=double(st.aux(1:n));
            end
            if isfield(st,'mode')
                if strcmpi(st.mode,'virtual'),row(5)=1;end
                if strcmpi(st.mode,'physical'),row(5)=2;end
            end
        end
        if isfield(c,'engaged'),row(6)=double(c.engaged);end
        if isfield(c,'monitor') && isstruct(c.monitor)
            if isfield(c.monitor,'minH'),row(27)=double(c.monitor.minH);end
            if isfield(c.monitor,'minNormalizedMargin'),row(28)=double(c.monitor.minNormalizedMargin);end
        end
        if isfield(c,'activeRows') && ~isempty(c.activeRows)
            ar=c.activeRows;
            psi=cellfun(@(v)min(v(:)),{ar.psiChain});row(29)=min(psi);
        end
        if isfield(c,'activeQP') && isstruct(c.activeQP) && isfield(c.activeQP,'exitflag')
            row(32)=double(c.activeQP.exitflag);
        end
        if isfield(c,'totalSeconds'),row(33)=double(c.totalSeconds);end
    end
catch
    % Read-only logging cannot affect either control law or numerical solution.
end
end

function rows=localConstraintAuditRows(t,hlc)
rows=zeros(0,14);
try
    if ~isstruct(hlc.result) || ~isfield(hlc.result,'cbf') || ...
            ~isstruct(hlc.result.cbf),return;end
    c=hlc.result.cbf;
    for src=0:1
        if src==0
            if ~isfield(c,'monitor') || ~isstruct(c.monitor) || ...
                    ~isfield(c.monitor,'rows'),continue;end
            rr=c.monitor.rows;
        else
            if ~isfield(c,'activeRows') || isempty(c.activeRows),continue;end
            rr=c.activeRows;
        end
        for i=1:numel(rr)
            one=rr(i);
            v=nan(1,14);v(1:2)=[double(t),double(src)];
            v(3)=double(one.obstacleID);
            component=lower(char(one.component));
            if strcmp(component,'uav'),v(4)=1;end
            if strcmp(component,'cable') || strcmp(component,'capsule'),v(4)=2;end
            if strcmp(component,'load'),v(4)=3;end
            v(5)=double(one.h);
            if isfield(one,'normalizedMargin'),v(6)=double(one.normalizedMargin);end
            if isfield(one,'psiChain')
                ps=double(one.psiChain(:));n=min(6,numel(ps));v(6+(1:n))=ps(1:n).';
            end
            if isfield(one,'initialChainSatisfied')
                v(13)=double(one.initialChainSatisfied);
            end
            if isfield(one,'margin'),v(14)=double(one.margin);end
            rows(end+1,:)=v; %#ok<AGROW>
        end
    end
catch
    % Optional read-only row audit cannot change plant/controller behavior.
end
end

function localPlotFourGeometriesZ(root,methods,refMode,figureNumber)
% Four spatial models x two control spaces: altitude, DISPLAY ONLY.
f=figure(figureNumber);clf(f);
set(f,'Name',sprintf('CBF study | reference %d | four geometries: altitude',refMode));
colors=[0.10 0.43 0.82;0.87 0.27 0.14];
for ig=1:numel(methods)
    method=methods{ig};figure(f);ax=subplot(2,2,ig);
    hold(ax,'on');grid(ax,'on');hasAny=false;
    for im=1:2
        if im==1,mode='virtual';else,mode='physical';end
        file=fullfile(root,sprintf('ref%d_%s',refMode,method),mode,'VERTICAL_CBF_path.csv');
        if ~isfile(file),continue;end
        d=readmatrix(file);
        if size(d,2)~=17 || size(d,1)<2,continue;end
        ids=d(:,2)==2 & all(isfinite(d(:,[1 8 17])),2);
        if ~any(ids),continue;end
        plot(ax,d(ids,1),d(ids,8),'Color',colors(im,:), ...
            'LineWidth',1.3,'DisplayName',[mode ' load']);
        if ~hasAny
            plot(ax,d(ids,1),d(ids,17),'k:','LineWidth',1.2,'DisplayName','reference');
        end
        hasAny=true;
    end
    title(ax,sprintf('%s | reference %d (stopped trials end early)',method,refMode));
    xlabel(ax,'t [s]');ylabel(ax,'load z [m]');
    if hasAny,legend(ax,'Location','best');end
end
end

function localPlotFourGeometries3D(root,methods,refMode,figureNumber)
% Each subplot is a DIFFERENT system-body geometry. Obstacles are identical.
% Plot only recorded plant truth and reference up to the stop time.
% No file output. Oriented ellipsoids use R_obs (not axis-aligned radii).
f=figure(figureNumber);clf(f);
set(f,'Name',sprintf('CBF study | reference %d | 3D four geometries',refMode));
obs=ENVIRONMENT_OBSTACLE_ELLIPSE_MOVE(5.0);
colors=[0.10 0.43 0.82;0.88 0.27 0.15];
for ig=1:numel(methods)
    method=methods{ig};figure(f);ax=subplot(2,2,ig);hold(ax,'on');grid(ax,'on');
    localDrawObstacleEllipsoids(ax,obs);
    refDone=false;hasAny=false;
    for im=1:2
        if im==1,mode='virtual';else,mode='physical';end
        file=fullfile(root,sprintf('ref%d_%s',refMode,method),mode,'VERTICAL_CBF_path.csv');
        if ~isfile(file),continue;end
        d=readmatrix(file);
        if size(d,2)~=17 || size(d,1)<3,continue;end
        valid=d(:,2)==2 & all(isfinite(d(:,3:8)),2);
        d=d(valid,:);if isempty(d),continue;end
        plot3(ax,d(:,3),d(:,4),d(:,5),'-','Color',colors(im,:), ...
            'LineWidth',1.5,'DisplayName',[mode ' UAV']);
        plot3(ax,d(:,6),d(:,7),d(:,8),'--','Color',colors(im,:), ...
            'LineWidth',1.5,'DisplayName',[mode ' load']);
        marks=unique(round(linspace(1,size(d,1),min(4,size(d,1)))));
        for k=marks
            plot3(ax,[d(k,3),d(k,6)],[d(k,4),d(k,7)], ...
                [d(k,5),d(k,8)],'-','Color',0.55*colors(im,:)+0.45, ...
                'LineWidth',0.6,'HandleVisibility','off');
        end
        if ~refDone
            idx=all(isfinite(d(:,15:17)),2);
            plot3(ax,d(idx,15),d(idx,16),d(idx,17),'k:', ...
                'LineWidth',1.2,'DisplayName','load reference');
            refDone=true;
        end
        plot3(ax,d(end,3),d(end,4),d(end,5),'o', ...
            'Color',colors(im,:),'HandleVisibility','off');
        plot3(ax,d(end,6),d(end,7),d(end,8),'s', ...
            'Color',colors(im,:),'HandleVisibility','off');
        hasAny=true;
    end
    title(ax,sprintf('%s | Virtual / Physical | reference %d',method,refMode));
    xlabel(ax,'x [m]');ylabel(ax,'y [m]');zlabel(ax,'z [m]');
    axis(ax,'vis3d');view(ax,34,21);
    if hasAny,legend(ax,'Location','best');end
end
end

function localWriteStudyDiagnostics(root,R)
% A compact per-trial source-derived audit for safety margin, actual inputs,
% controller elapsed time, and HLC conditioning. These are OBSERVATIONS,
% not feasibility, forward invariance, or avoidance certificates.
n=numel(R);
minH=nan(n,1);maxF=nan(n,1);minF=nan(n,1);maxMoment=nan(n,1);
medianCBFms=nan(n,1);minRcond=nan(n,1);activeSteps=zeros(n,1);
qpTime=nan(n,1);qpRows=nan(n,1);qpLPslack=nan(n,1);
qpLPFlag=nan(n,1);qpSnapshotFound=false(n,1);
for k=1:n
    scenario=sprintf('ref%d_%s',R(k).referenceMode,R(k).geometryMethod);
    wd=fullfile(root,scenario,R(k).mode);
    inputFile=fullfile(wd,'CBF_input_comparison_full_flight.csv');
    if isfile(inputFile)
        T=readtable(inputFile);
        if all(ismember({'cbf_active','failed','u_safe_f_N','u_safe_Mx_Nm', ...
                'u_safe_My_Nm','u_safe_Mz_Nm','hmin_m','beta2_rcond','cbf_cpu_s'}, ...
                T.Properties.VariableNames))
            ix=T.cbf_active>0.5 & T.failed<0.5 & ...
                isfinite(T.u_safe_f_N) & isfinite(T.hmin_m);
            activeSteps(k)=nnz(ix);
            if any(ix)
                minH(k)=min(T.hmin_m(ix));
                minF(k)=min(T.u_safe_f_N(ix));maxF(k)=max(T.u_safe_f_N(ix));
                maxMoment(k)=max(max(abs([T.u_safe_Mx_Nm(ix), ...
                    T.u_safe_My_Nm(ix),T.u_safe_Mz_Nm(ix)])));
                medianCBFms(k)=1000*median(T.cbf_cpu_s(ix));
                cond=T.beta2_rcond(ix);cond=cond(isfinite(cond));
                if ~isempty(cond),minRcond(k)=min(cond);end
            end
        end
    end
    snapFile=fullfile(wd,'CBF_physical_QP_failure.mat');
    if isfile(snapFile)
        snap=load(snapFile);
        if isfield(snap,'D') && isfield(snap.D,'Aineq') && ...
                isfield(snap.D,'bineq')
            D=snap.D;A=double(D.Aineq);b=double(D.bineq(:));
            qpSnapshotFound(k)=true;qpRows(k)=size(A,1);
            if isfield(D,'time'),qpTime(k)=double(D.time);end
            if size(A,2)==4 && numel(b)==size(A,1) && ...
                    all(isfinite(A(:))) && all(isfinite(b))
                % Minimize s >= 0, subject to A*w <= b+s.
                % A zero result means the SAVED LINEAR inequalities are feasible.
                % It does not imply safety, future feasibility, or bounded input.
                try
                    lpOpt=optimoptions('linprog','Display','none');
                    [~,slack,ef]=linprog([zeros(4,1);1], ...
                        [A,-ones(size(A,1),1)],b,[],[], ...
                        [-inf(4,1);0],[],lpOpt);
                    qpLPFlag(k)=ef;
                    if ef>0 && isfinite(slack),qpLPslack(k)=max(0,slack);end
                catch ME
                    warning('CBF:LPStudyAudit','Case %d LP audit unavailable: %s',k,ME.message);
                end
            end
        end
    end
end
T=table(string({R.geometryMethod}).',[R.referenceMode].',string({R.mode}).', ...
    [R.stopped].',activeSteps,minH,minF,maxF,maxMoment,medianCBFms,minRcond, ...
    qpSnapshotFound,qpTime,qpRows,qpLPFlag,qpLPslack, ...
    'VariableNames',{'geometry_method','reference_mode','input_mode','stopped', ...
    'active_samples','min_recorded_monitor_h_m','min_applied_thrust_N', ...
    'max_applied_thrust_N','max_abs_applied_torque_Nm', ...
    'median_active_cbf_ms','min_hlc_beta2_rcond', ...
    'qp_failure_snapshot_found','qp_failure_time_s', ...
    'qp_inequality_rows','independent_lp_exitflag', ...
    'minimum_common_hard_constraint_slack'});
writetable(T,fullfile(root,'CBF_EIGHT_CASE_DIAGNOSTICS.csv'));
end
