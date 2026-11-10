function REPORT = run_flight_CBF_dual_compare(projectRoot, cbfProfile)
% run_flight_CBF_dual_compare
% Self-contained sequential experiment derived from user's run_flight_CBF.m.
% SOURCE run_flight_CBF.m IS NEVER EDITED OR OVERWRITTEN.
% Simulates Virtual then Physical, each from a NEW DRONE / EKF / HLC instance.
% Motion dynamics and core control/phase loop copied from source; only cbfMode
% becomes an input. Extra logging is read-only. No forced fallback on QP errors.
% This research prototype is NOT a safety or 25ms real-time certificate.
%
% Installation: place this .m in C:\...\common_matlab\ (same folder as original)
% Usage: REPORT = run_flight_CBF_dual_compare();
%        REPORT = run_flight_CBF_dual_compare([], 'legacy_strict');
% Default 'auto' chooses installed flight-only supervisory wrapper when present;
% 'legacy_strict' reproduces the original immediate initial-chain enforcement.
% CBF controller choice changes ONLY the CBF wrapper, never the plant/estimator/
% reference/HL mathematical functions/phase schedule. Never certify safety.
%        REPORT = run_flight_CBF_dual_compare('C:\...\common_matlab');

if nargin<1 || isempty(projectRoot)
    source=which('run_flight_CBF');
    if ~isempty(source)
        projectRoot=fileparts(source);
    else
        projectRoot=fileparts(mfilename('fullpath'));
    end
end
if nargin<2 || isempty(cbfProfile), cbfProfile='auto'; end
cbfProfile=lower(char(cbfProfile));
if ~any(strcmp(cbfProfile,{'auto','supervised','legacy_strict'}))
    error('CBF:Profile','cbfProfile must be auto, supervised or legacy_strict');
end
projectRoot=char(projectRoot);
assert(exist(fullfile(projectRoot,'run_flight_CBF.m'),'file')==2, ...
    'CBF:OriginalRunMissing', 'Original run_flight_CBF.m missing under %s.',projectRoot);
oldDir=pwd;
restoreDir=onCleanup(@() cd(oldDir)); %#ok<NASGU>
addpath(genpath(projectRoot));
% The legacy wrapper enforces all high-order initial conditions at t=5 even
% if the initial geometric h is positive. Previous working comparison used a
% flight-only supervisor instead. Do not silently disable chain constraints.
legacyPath=which('HLC_SUSPENDED_CBF');
supervisedPath=which('HLC_SUSPENDED_CBF_PHYSICAL_FLIGHT');
if strcmp(cbfProfile,'supervised') && isempty(supervisedPath)
    error('CBF:SupervisedWrapperMissing', ...
        'HLC_SUSPENDED_CBF_PHYSICAL_FLIGHT.m is not on the MATLAB path.');
end
if strcmp(cbfProfile,'legacy_strict')
    if isempty(legacyPath),error('CBF:LegacyWrapperMissing', ...
        'Original HLC_SUSPENDED_CBF is not on MATLAB path.');end
    selectedProfile='legacy_strict';
elseif ~isempty(supervisedPath)
    selectedProfile='supervised';
else
    if isempty(legacyPath),error('CBF:WrapperMissing', ...
        'Neither CBF wrapper is on the MATLAB path.');end
    selectedProfile='legacy_strict';
    warning('CBF:NoSupervisor', ...
        ['The flight-only supervisory wrapper is missing. Running legacy strict' ...
         ' mode: initial HOCBF chain may fail at t=5. No constraint is relaxed.']);
end
seed=42;
% Only one persistent output ZIP, overwritten atomically on each successful run.
% Per-mode folders exist solely inside a temporary staging directory.
rootOut=tempname(tempdir);
mkdir(rootOut);
stageCleanup=onCleanup(@()localRemoveStage(rootOut)); %#ok<NASGU>
modeNames={'virtual','physical'};
REPORT=struct();REPORT.created=datestr(now,31);
REPORT.projectRoot=projectRoot;REPORT.outputRoot='(temporary; removed after ZIP creation)';
REPORT.cbfProfile=selectedProfile;
REPORT.sourceRun=fullfile(projectRoot,'run_flight_CBF.m');
REPORT.generator=which('Generate_CBF_ActuatorAware_6Methods');
REPORT.legacyGenerator=which('Generate_CBF_DualInput_6Methods');
REPORT.legacyHLC=legacyPath;REPORT.supervisedHLC=supervisedPath;
REPORT.rngSeed=seed;
REPORT.note=['Two independent executions from an unchanged copy of the original plant, ' ...
    'EKF, reference and phase loop. CBF wrapper profile is explicitly recorded. ' ...
    'The optional supervised wrapper differs from the legacy original wrapper.'];
REPORT.cases=repmat(struct('mode','','status','','stopTime',NaN, ...
    'message','','completedSamples',0,'outputFolder',''),1,2);
for k=1:2
    modeName=modeNames{k};
    resultDir=fullfile(rootOut,modeName);
    mkdir(resultDir);
    fprintf('\n[CBF-DUAL-ORIGINAL] START %d/2 mode=%s (source untouched)\n',k,modeName);
    rng(seed,'twister');
    try
        REPORT.cases(k)=localRunOriginalOne(projectRoot,resultDir,modeName,selectedProfile);
    catch ME
        REPORT.cases(k)=struct('mode',modeName,'status','RUNNER_ERROR', ...
            'stopTime',NaN,'message',ME.message,'completedSamples',0,'outputFolder',resultDir);
        localSaveError(resultDir,ME);
        fprintf(2,'[CBF-DUAL-ORIGINAL] %s runner error: %s\n',modeName,ME.message);
    end
    fprintf('[CBF-DUAL-ORIGINAL] END mode=%s status=%s time=%.3f\n', ...
        modeName,REPORT.cases(k).status,REPORT.cases(k).stopTime);
    REPORT.cases(k).outputFolder=[modeName '/ (inside ZIP)'];
end
% Summary is written even if either mode stopped. No accidental 'success' claim.
mode=string({REPORT.cases.mode}).';
status=string({REPORT.cases.status}).';
stop_time_s=[REPORT.cases.stopTime].';
completed_samples=[REPORT.cases.completedSamples].';
message=string({REPORT.cases.message}).';
T=table(mode,status,stop_time_s,completed_samples,message);
writetable(T,fullfile(rootOut,'COMPARISON_SUMMARY.csv'));
save(fullfile(rootOut,'COMPARISON_REPORT.mat'),'REPORT');
fid=fopen(fullfile(rootOut,'README_RESULTS.txt'),'w');
if fid>0
    fprintf(fid,['Virtual and Physical each started from the same initial configuration and RNG seed.\n' ...
        'Original run_flight_CBF.m was NOT altered. Requested virtual/physical is fixed by allowPhysicalFallback=false.\n' ...
        'These are simulations, not continuous-time safety or realtime certification.\n' ...
        'Inspect comparison traces, error logs and QP snapshots for both modes.\n']);
    fclose(fid);
end
% Read-only comparative plots; plot errors never stop or invalidate simulation logs.
try
    localDualPlots(rootOut);
catch ME
    warning('CBF:DualPlots','Comparison plots not created: %s',ME.message);
end
% Include the audit file if available (audit uses an overwrite-only fixed name).
auditFile=fullfile(projectRoot,'CBF_AUDIT_LATEST.mat');
if exist(auditFile,'file')==2
    copyfile(auditFile,fullfile(rootOut,'CBF_AUDIT_LATEST.mat'));
end
% Gather source files including indirect/dynamic MATLAB path dependencies.
% Source snapshot is nested inside the ONE result ZIP. Failure to collect does
% not erase the experiment results.
try
    if exist('collect_CBF_run_handoff','file')==2
        sourceZip=fullfile(rootOut,'CBF_ALL_LOADED_PROJECT_SOURCES.zip');
        sourceReport=collect_CBF_run_handoff(projectRoot,sourceZip);
        save(fullfile(rootOut,'SOURCE_COLLECTION_REPORT.mat'),'sourceReport');
    else
        fid=fopen(fullfile(rootOut,'SOURCE_COLLECTION_UNAVAILABLE.txt'),'w');
        if fid>0
            fprintf(fid,'collect_CBF_run_handoff.m missing from MATLAB path.\n');fclose(fid);
        end
    end
catch ME
    fid=fopen(fullfile(rootOut,'SOURCE_COLLECTION_ERROR.txt'),'w');
    if fid>0,fprintf(fid,'%s\n',getReport(ME,'extended','hyperlinks','off'));fclose(fid);end
end
REPORT.zip=fullfile(projectRoot,'CBF_DUAL_RESULTS_UPLOAD.zip');
REPORT.sourceWrapperProfile=selectedProfile;
REPORT.note=['Comparison of legacy/supervised wrapper is not a controller proof. ' ...
    'Inspect FIRST_FLIGHT_FAILURE.mat if either t=5 initial-chain stop occurs.'];
save(fullfile(rootOut,'COMPARISON_REPORT.mat'),'REPORT');
try
    include={'virtual','physical','COMPARISON_SUMMARY.csv', ...
        'COMPARISON_REPORT.mat','README_RESULTS.txt'};
    optionalFiles={'CBF_ALL_LOADED_PROJECT_SOURCES.zip', ...
        'SOURCE_COLLECTION_REPORT.mat','SOURCE_COLLECTION_ERROR.txt', ...
        'SOURCE_COLLECTION_UNAVAILABLE.txt','CBF_AUDIT_LATEST.mat'};
    for ii=1:numel(optionalFiles)
        if exist(fullfile(rootOut,optionalFiles{ii}),'file')==2
            include{end+1}=optionalFiles{ii}; %#ok<AGROW>
        end
    end
    if exist(fullfile(rootOut,'COMPARISON_ALTITUDE.png'),'file')==2
        include{end+1}='COMPARISON_ALTITUDE.png';
    end
    if exist(fullfile(rootOut,'COMPARISON_TOPVIEW.png'),'file')==2
        include{end+1}='COMPARISON_TOPVIEW.png';
    end
    tmpZip=fullfile(projectRoot,'CBF_DUAL_RESULTS_UPLOAD.pending.zip');
    if exist(tmpZip,'file')==2,delete(tmpZip);end
    zip(tmpZip,include,rootOut);
    [ok,msg]=movefile(tmpZip,REPORT.zip,'f');
    if ~ok,error('CBF:FinalZip','Cannot publish result ZIP: %s',msg);end
    fprintf('[CBF-DUAL-ORIGINAL] RESULT ZIP: %s\n',REPORT.zip);
catch ME
    warning('CBF:DualZip','Failed to zip outputs: %s',ME.message);
    REPORT.zip='';
end
% The source/report variables are already stored before packing; the only
% persistent artifact is REPORT.zip (plus optional audit fixed-name MAT).
end

function R=localRunOriginalOne(projectRoot,resultDir,modeName,selectedProfile)
% BEGIN source-derived run body. Changes limited to mode selection and logging.
old=pwd;
cleanup=onCleanup(@() cd(old)); %#ok<NASGU>
cd(projectRoot);
addpath(genpath(projectRoot));
% Working folder isolated to prevent duplicate CBF diagnostic MAT overwrites.
cd(resultDir);
close all hidden;
compareTrace=zeros(0,31);
compareRefJets=zeros(0,29);
compareCBF=cell(0,1);
compareTiming=zeros(0,3);
compareStepCount=0;
modeHistory=cell(0,4);
status='COMPLETED';stopMsg=''; stopTime=NaN;
try
%% 1. INITIALIZATION -- original plant and estimator, no changes
ts = 0;
dt = 0.025;
te = 50;
time = TIME(ts,dt,te);
logger = LOGGER(1, size(ts:dt:te, 2), 0, [],[]);
logger.display_on = false;

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
% NOTE: Retained as in user's original script. It creates a possible
% plant/model mass mismatch (0.14 -> 0.01); check whether intentional.
agent.parameter.set("loadmass",0.01);
motive = Connector_Natnet_sim(dt, {{1,"p","q"},{1,"pL","pT"}});
motive.getData(agent);

agent.sensor.set_function_class("motive", MOTIVE(agent, motive,"output_func",@motive_output,"rigid_id",[1,2],"state_list",{["p"],"p"}));
agent.estimator.set_function_class("ekf", EKF(agent, Estimator_EKF_SuspendedLoad(agent,dt,...
    MODEL_CLASS(agent,Model_Suspended_Load(dt, initial_state, 1,agent,"Load_mL_HL")),...
    ["p", "q","pL","pT"])));
agent.estimator.set_function_class("loadstate", SUSPENDED_LOAD_STATE_MANAGER(agent));
L = agent.parameter.cableL;

nominal_ref = TIME_VARYING_REFERENCE(agent, ...
    {"gen_ref_line", {"p0", [0, 0, 3], "velocity", 1.5, "direction", [0, 0, 1]}, 6});
replanner_instance = nominal_ref;
agent.reference.set_function_class("timevarying", replanner_instance);
agent.reference.set_function_class("sload", SUSPENDED_LOAD_REF_ADJUST(agent));
agent.reference.set_function_class("takeoff", TAKEOFF_REFERENCE(agent,"zd",3.0,"te",5));
agent.reference.set_function_class("landing", LANDING_REFERENCE(agent,"dt",dt,"zd",-L,"te",3));

%% *** CBF CHANGE A: create/register HLC_SUSPENDED_CBF ***
if strcmp(selectedProfile,'supervised')
    hlc = HLC_SUSPENDED_CBF_PHYSICAL_FLIGHT(agent,Controller_HL_Suspended_Load(dt,agent));
else
    hlc = HLC_SUSPENDED_CBF(agent,Controller_HL_Suspended_Load(dt,agent));
end
fprintf('[CBF-DUAL] actual wrapper=%s source=%s mode=%s\n', ...
    class(hlc),which(class(hlc)),modeName);
hlc.cbfMode = modeName; % dual experiment selection ONLY; original was 'virtual'
hlc.cbfMethod = 'S3';   % 'S1','S2','S3','K1','K2','K3'
hlc.cbfEnabled = false; % keep off during warmup, arming, takeoff

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
shapes.load = rL*ones(3,1);

hlc.cbfShapes = shapes;
hlc.cbfKappa = 2*ones(6,1);
hlc.cbfOpts = struct('requireInitialChain',true,'uLimits',[0;20;1;1;1], ...
    'allowPhysicalFallback',false); % never mix Virtual and Physical in one run
% No initial-chain override. When using the supervisor, its MONITOR stage
% determines engagement; the legacy strict version can still stop at t=5.
hlc.cbfAux = [];
hlc.cbfAuxMode = '';
hlc.cbfTime = 0;

agent.controller.set_function_class("hlc_suspended", hlc);

agent.set_cha_allocation_for_all("sensor","motive");
agent.set_cha_allocation_for_all("estimator",["ekf","loadstate"]);
agent.cha_allocation.a.reference = ["takeoff","sload"];
agent.cha_allocation.t.reference = ["takeoff","sload"];
agent.cha_allocation.f.reference = ["timevarying","sload"];
agent.cha_allocation.l.reference = ["landing","sload"];
env = [];

%% 2. WARMUP
fprintf('CBF mode=%s / method=%s / enabled during flight phase only\n',hlc.cbfMode,hlc.cbfMethod);
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

%% Dual-run READ-ONLY logging buffers (no change to controller or plant)
compareTrace=zeros(0,31);
compareRefJets=zeros(0,29);
compareCBF=cell(0,1);
compareTiming=zeros(0,3);
compareStepCount=0;

%% 4. SIMULATION LOOP
disp("Starting Simulation...");
cha = 't';
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
            hlc.cbfAux = []; % initialize only when entering flight phase
            hlc.cbfAuxMode = '';
        end
    else
        if cha ~= 'l'
            disp("Phase: Landing (CBF disabled)");
            cha = 'l';
        end
    end

    if ~isempty(motive), motive.getData(agent); end
    tStartTotal = tic;
    do_prop_local(time, cha, cha, logger, env, agent, "sensor", 1, true);
    do_prop_local(time, cha, cha, logger, env, agent, "estimator", 1, true);
    do_prop_local(time, cha, cha, logger, env, agent, "reference", 1, true);

    %% *** CBF CHANGE B: set phase and CURRENT time before controller ***
    hlc.cbfEnabled = (cha == 'f');
    hlc.cbfTime = time.t;

    do_prop_local(time, cha, cha, logger, env, agent, "controller", 1, true);
    do_prop_local(time, cha, cha, logger, env, agent, "input_transform", 1, true);
    do_prop_local(time, cha, cha, logger, env, agent, "plant", 1, true);
    agent.controller.result.controllertime_total = toc(tStartTotal);
    logger.logging(time, cha, agent, []);
    % Additional read-only capture: full plant / EKF / reference / CBF inputs.
    compareStepCount=compareStepCount+1;
    compareTrace(end+1,:)=localDualSnapshot(time.t,cha,agent,hlc); %#ok<AGROW>
    compareRefJets(end+1,:)=localDualReferenceJet(time.t,agent); %#ok<AGROW>
    try
        compareCBF{end+1,1}=hlc.result.cbf; %#ok<AGROW>
    catch
        compareCBF{end+1,1}=[]; %#ok<AGROW>
    end
    compareTiming(end+1,:)=[time.t,toc(tStartTotal),double(cha=='f')]; %#ok<AGROW>
    modeHistory(end+1,:)={double(time.t),char(modeName), ...
        localGetAppliedMode(hlc),char(selectedProfile)}; %#ok<AGROW>

    %% *** CBF CHANGE C: diagnostics; does not alter control ***
    % Read-only diagnostics must never cause a controller failure. The
    % supervisory wrapper returns monitor rows before it has active QP rows.
    if hlc.cbfEnabled && mod(time.k,40)==0
        try
            cbf = hlc.result.cbf;
            hmin=NaN; qpok=NaN; cpu=NaN;
            if isstruct(cbf)
                if isfield(cbf,'rows') && ~isempty(cbf.rows)
                    hmin=min(double([cbf.rows.h]));
                elseif isfield(cbf,'monitor') && isstruct(cbf.monitor)
                    if isfield(cbf.monitor,'minH')
                        hmin=double(cbf.monitor.minH);
                    end
                end
                if isfield(cbf,'success'),qpok=double(cbf.success);end
                if isfield(cbf,'totalSeconds'),cpu=double(cbf.totalSeconds);end
            end
            fprintf('[CBF] t=%.3f requested=%s method=%s reportedOK=%g min(h)=%.5g CPU=%.1f ms\n', ...
                time.t,hlc.cbfMode,hlc.cbfMethod,qpok,hmin,1000*cpu);
        catch MEdiag
            warning('CBF:ReadOnlyTrace','t=%.3f CBF print skipped: %s',time.t,MEdiag.message);
        end
    end

    time.k = logger.k;
    time.t = time.t + dt;
    if mod(time.t, 5.0) < dt
        fprintf('Time: %.1f / %.1f sec (Phase: %s)\n',time.t,time.te,cha);
    end
end


catch ME
    status='STOPPED';stopMsg=sprintf('%s: %s',ME.identifier,ME.message);
    localSaveError(resultDir,ME);
    if exist('agent','var') && exist('time','var') && exist('hlc','var')
        try
            localSaveFailureState(resultDir,ME,modeName,selectedProfile,time,agent,hlc);
        catch Ediag
            warning('CBF:StateSnapshot','Failure-state snapshot unavailable: %s',Ediag.message);
        end
    end
    if exist('time','var') && isprop(time,'t'),stopTime=double(time.t);end
    % Capture the failing controller call even if it did not reach plant update.
    if exist('agent','var') && exist('time','var')
        try
            compareTrace(end+1,:)=localDualSnapshot(time.t,cha,agent,hlc); %#ok<AGROW>
            compareRefJets(end+1,:)=localDualReferenceJet(time.t,agent); %#ok<AGROW>
        catch
        end
    end
    fprintf(2,'[CBF-DUAL-ORIGINAL] STOP mode=%s t=%g: %s\n',modeName,stopTime,stopMsg);
end
if strcmp(status,'COMPLETED') && exist('time','var')
    stopTime=double(time.t);
end
% Results are always archived regardless of success or QP failure.
if ~isempty(compareTrace)
    writetable(array2table(compareTrace,'VariableNames',localDualTraceNames()), ...
        fullfile(resultDir,'FLIGHT_TRACE.csv'));
end
if ~isempty(compareRefJets)
    writetable(array2table(compareRefJets,'VariableNames',localDualJetNames()), ...
        fullfile(resultDir,'REFERENCE_JET_0_TO_6.csv'));
end
if ~isempty(modeHistory)
    try
        time_s=cell2mat(modeHistory(:,1));
        requested_mode=string(modeHistory(:,2));
        reported_applied_mode=string(modeHistory(:,3));
        CBF_profile=string(modeHistory(:,4));
        mh=table(time_s,requested_mode,reported_applied_mode,CBF_profile);
        writetable(mh,fullfile(resultDir,'APPLIED_MODE_HISTORY.csv'));
    catch ME
        warning('CBF:ModeHistory','Cannot write mode history: %s',ME.message);
    end
end
if ~isempty(compareTiming)
    writetable(array2table(compareTiming,'VariableNames',{'time_s','total_step_s','flight_phase'}), ...
        fullfile(resultDir,'STEP_TIMING.csv'));
end
% Only plain numerical/struct traces, not cyclic DRONE handles.
try
    save(fullfile(resultDir,'FLIGHT_DETAIL.mat'),'compareTrace', ...
        'compareRefJets','compareCBF','compareTiming','modeHistory','status','stopMsg', ...
        'stopTime','modeName','compareStepCount','-v7.3');
catch ME
    warning('CBF:FlightDetailSave','Could not save detailed CBF trace: %s',ME.message);
    save(fullfile(resultDir,'FLIGHT_DETAIL_LITE.mat'),'compareTrace', ...
        'compareRefJets','compareTiming','modeHistory','status','stopMsg', ...
        'stopTime','modeName','compareStepCount');
end
R=struct('mode',modeName,'status',status,'stopTime',stopTime, ...
    'message',stopMsg,'completedSamples',compareStepCount,'outputFolder',resultDir);
end

function r=localDualSnapshot(t,cha,agent,hlc)
% Read only; record the same numerically meaningful channels for both modes.
r=nan(1,31);r(1)=double(t);
switch char(cha),case 't',r(2)=1;case 'f',r(2)=2;case 'l',r(2)=3;otherwise,r(2)=0;end
try,r(3:5)=double(agent.plant.state.p(:)).'; catch, end
try,r(6:8)=double(agent.plant.state.pL(:)).'; catch, end
try,r(9:11)=double(agent.estimator.result.state.p(:)).'; catch, end
try,r(12:14)=double(agent.estimator.result.state.pL(:)).'; catch, end
try
    xd=double(agent.reference.result.state.xd(:));
    r(15:17)=xd(1:3).';
catch
end
try
    u=double(agent.controller.result.input(:));
    if numel(u)>=4,r(18:21)=u(1:4).';end
catch
end
try
    st=agent.estimator.result.state;
    if isprop(st,'mL'),r(27)=double(st.mL);end
catch
end
try
    cbf=hlc.result.cbf;
    if isstruct(cbf)
        if isfield(cbf,'rows') && ~isempty(cbf.rows)
            r(22)=min(double([cbf.rows.h]));
        elseif isfield(cbf,'monitor') && isstruct(cbf.monitor) && ...
                isfield(cbf.monitor,'minH')
            r(22)=double(cbf.monitor.minH);
        end
        if isfield(cbf,'totalSeconds'),r(23)=double(cbf.totalSeconds);end
        if isfield(cbf,'success'),r(24)=double(cbf.success);end
        if isfield(cbf,'uSafe') && isfield(cbf,'uBase')
            r(25)=norm(double(cbf.uSafe(:)-cbf.uBase(:)));
        end
    end
catch
end
try,r(26)=double(hlc.cbfEnabled); catch, end
try,r(28:30)=double(agent.plant.state.q(:)).'; catch, end
try
    if isprop(agent.estimator.result.state,'q')
        q=double(agent.estimator.result.state.q(:));
        r(31)=norm(q);
    end
catch
end
end

function r=localDualReferenceJet(t,agent)
r=nan(1,29);r(1)=double(t);
try
    xd=double(agent.reference.result.state.xd(:));
    if numel(xd)==28,r(2:29)=xd.';end
catch
end
end

function names=localDualTraceNames()
names={'time_s','phase_code','plant_uav_x_m','plant_uav_y_m','plant_uav_z_m', ...
    'plant_load_x_m','plant_load_y_m','plant_load_z_m', ...
    'ekf_uav_x_m','ekf_uav_y_m','ekf_uav_z_m', ...
    'ekf_load_x_m','ekf_load_y_m','ekf_load_z_m', ...
    'ref_load_x_m','ref_load_y_m','ref_load_z_m', ...
    'controller_thrust_N','controller_Mx_Nm','controller_My_Nm','controller_Mz_Nm', ...
    'CBF_min_h','CBF_compute_s','CBF_success','CBF_deltaU', ...
    'CBF_enabled','estimated_loadmass_kg','plant_attitude_q1', ...
    'plant_attitude_q2','plant_attitude_q3','ekf_q_norm'};
end

function names=localDualJetNames()
names=cell(1,29);names{1}='time_s';
ax={'x','y','z','yaw'};
for k=0:6,for j=1:4,names{2+4*k+j-1}=sprintf('d%d_%s',k,ax{j});end,end
end

function localDualPlots(rootOut)
modeNames={'virtual','physical'};
C={'b',[0.86 0.30 0.15]};
f=figure('Visible','off');hold on;grid on;
refDrawn=false;
for i=1:2
    fn=fullfile(rootOut,modeNames{i},'FLIGHT_TRACE.csv');
    if exist(fn,'file')~=2,continue;end
    T=readtable(fn);
    good=isfinite(T.time_s) & isfinite(T.plant_load_z_m);
    if any(good)
        plot(T.time_s(good),T.plant_load_z_m(good),'Color',C{i}, ...
            'DisplayName',[modeNames{i} ' load z']);
    end
    if ~refDrawn
        good=isfinite(T.time_s) & isfinite(T.ref_load_z_m);
        if any(good)
            plot(T.time_s(good),T.ref_load_z_m(good),'k--', ...
                'DisplayName','reference load z');
            refDrawn=true;
        end
    end
end
xlabel('time [s]');ylabel('load z [m]');legend('Location','best');
title('Independent Virtual / Physical CBF flight comparison');
saveas(f,fullfile(rootOut,'COMPARISON_ALTITUDE.png'));close(f);
f=figure('Visible','off');hold on;grid on;
for i=1:2
    fn=fullfile(rootOut,modeNames{i},'FLIGHT_TRACE.csv');
    if exist(fn,'file')~=2,continue;end
    T=readtable(fn);
    good=isfinite(T.plant_load_x_m) & isfinite(T.plant_load_y_m);
    if any(good)
        plot(T.plant_load_x_m(good),T.plant_load_y_m(good),'Color',C{i}, ...
            'DisplayName',[modeNames{i} ' load']);
    end
end
axis equal;xlabel('load x [m]');ylabel('load y [m]');
legend('Location','best');title('Load XY trajectories (partial if stopped)');
saveas(f,fullfile(rootOut,'COMPARISON_TOPVIEW.png'));close(f);
end

function localSaveError(folder,ME)
fid=fopen(fullfile(folder,'STOP_REASON.txt'),'w');
if fid<0,return;end
fprintf(fid,'ID: %s\nMessage: %s\n',ME.identifier,ME.message);
for j=1:numel(ME.stack)
    fprintf(fid,'  %s line %d\n',ME.stack(j).name,ME.stack(j).line);
end
fclose(fid);
end

%% LOCAL FUNCTIONS -- unchanged
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


function localSaveFailureState(resultDir,ME,mode,profile,time,agent,hlc)
% Only scalars, vectors and value structs; no cyclic agent or HLC handles.
F=struct();F.mode=mode;F.profile=profile;
F.time=double(time.t);F.message=ME.message;F.identifier=ME.identifier;
F.estimator=localReadState(agent.estimator.result.state);
F.plant=localReadState(agent.plant.state);
F.reference=localReadState(agent.reference.result.state);
F.hlcClass=class(hlc);F.hlcSource=which(class(hlc));
F.generatorLegacy=which('Generate_CBF_DualInput_6Methods');
F.generatorSupervised=which('Generate_CBF_ActuatorAware_6Methods');
try,F.shapes=hlc.cbfShapes;catch,end
try,F.kappa=hlc.cbfKappa;catch,end
try,F.opts=hlc.cbfOpts;catch,end
try,F.cbfTime=hlc.cbfTime;catch,end
try
    r=hlc.result;
    if isstruct(r) && isfield(r,'cbf'),F.cbfResult=r.cbf;end
catch
end
try,F.obstacles=ENVIRONMENT_OBSTACLE_ELLIPSE_MOVE(time.t);catch,end
save(fullfile(resultDir,'FIRST_FLIGHT_FAILURE.mat'),'F');
end

function V=localReadState(st)
V=struct();
fields={'p','q','pL','pT','v','vL','w','wL','mL','xd','x','state','input'};
for i=1:numel(fields)
    nm=fields{i};
    try
        if isstruct(st)
            if isfield(st,nm),val=st.(nm);else,continue;end
        else
            if isprop(st,nm),val=st.(nm);else,continue;end
        end
        if isnumeric(val) || islogical(val)
            V.(nm)=double(val);
        end
    catch
    end
end
end

function localRemoveStage(stage)
try
    if exist(stage,'dir')==7,rmdir(stage,'s');end
catch ME
    warning('CBF:StageCleanup','Temporary stage cleanup failed: %s',ME.message);
end
end


function mode=localGetAppliedMode(hlc)
% Do not infer an applied mode merely from requested hlc.cbfMode.
mode='unknown';
try
    o=hlc.result.cbf;
    if ~isstruct(o),return;end
    fields={'appliedMode','actualMode','mode'};
    for i=1:numel(fields)
        if isfield(o,fields{i}) && ~isempty(o.(fields{i}))
            mode=char(string(o.(fields{i})));return;
        end
    end
    if isfield(o,'supervisor') && isstruct(o.supervisor) && ...
            isfield(o.supervisor,'mode')
        mode=char(string(o.supervisor.mode));return;
    end
catch
end
end
