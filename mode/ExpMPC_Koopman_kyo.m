clc
ts = 0; % initial time
dt = 0.025; % sampling period
te = 100; % termina time
time = TIME(ts,dt,te);

in_prog_func = @(app) in_prog(app);
post_func = @(app) post(app);
logger = LOGGER(1, size(ts:dt:te, 2), 1, [],[]);
mmatflag = 0;
filename = '2025-03-31_Exp_Kyomo_code00_saddle';
matfile_info = dir(fullfile(pwd, '**', [filename, '.mat']));
mfile_info = dir(fullfile(pwd, '**', [filename, '.m']));

if ~isempty(matfile_info)
    mmatflag = 1;
    model_file = fullfile(matfile_info(1).folder, matfile_info(1).name);
elseif ~isempty(mfile_info)
    mmatflag = 2;
    model_file = fullfile(mfile_info(1).folder, mfile_info(1).name);
    est = import_vars_from_mfile(model_file);
else
    disp('no files');
end

% if exist(fullfile(pwd, [filename, '.mat']), 'file') == 2
%     mmatflag =1;
%     model_file = '2025-03-31_Exp_Kyomo_code00_saddle.mat';
% elseif exist(fullfile(pwd, [filename, '.m']), 'file') == 2
%     mmatflag =2;
%    est =import_vars_from_mfile('2025-03-31_Exp_Kyomo_code00_saddle.m'); 
%    model_file = '2025-03-31_Exp_Kyomo_code00_saddle.m';
% else
%     disp('no files');
% end
motive = Connector_Natnet('192.168.100.4'); % connect to Motive　
motive.getData([], []); % get data from Motive
rigid_ids = 1; % rigid-body number on Motive
sstate = motive.result.rigid(rigid_ids);
initial_state.p = sstate.p;
initial_state.q = sstate.q;
initial_state.v = [0; 0; 0];
initial_state.w = [0; 0; 0];


agent = DRONE;
%agent.plant = DRONE_EXP_MODEL(agent,Model_Drone_Exp(dt, initial_state, "udp", [1, 252])); %プロポ無線
agent.plant = DRONE_EXP_MODEL(agent,Model_Drone_Exp(dt, initial_state, "serial", "COM9")); %プロポ有線
agent.parameter = DRONE_PARAM("DIATONE");
agent.estimator = EKF(agent, Estimator_EKF(agent,dt,MODEL_CLASS(agent,Model_EulerAngle(dt, initial_state, 1)), ["p", "q"]));
agent.sensor = MOTIVE(agent, Sensor_Motive(1,0, motive));
agent.input_transform = THRUST2THROTTLE_DRONE(agent,InputTransform_Thrust2Throttle_drone_KMPC()); % 推力からスロットルに変換
% 
agent.reference.bezier = BEZIER_REFERENCE(agent,{[0,0,0.6]},time);

%agent.reference = TIME_VARYING_REFERENCE(agent,{"Case_study_trajectory",{[0,0,0.6]},"HL"});

%2つのコントローラの設定---------------------------------------------------------------------------------------------------
agent.controller.hlc = HLC(agent,Controller_HL(dt));
 agent.controller.kmpc = MPC_CONTROLLER_KMC_kyo_guiexperiment(agent,Controller_MPC_KMC_kyo(dt,model_file,agent,mmatflag,est)); %最適化手法：QP
% %agent.controller.kmpc =  MPC_CONTROLLER_KMC_kyo(agent, Controller_MPC_KMC_kyo(dt, model_file, agent));
agent.controller.result.input = [0;0;0;0];
% agent.controller.do = @controller_do;
% %-run("ExpBase");-----------------------------------------------------------------------------------------------------------------------
run("ExpBase");
agent.reference.takeoff = TAKEOFF_REFERENCE(agent,[]);
agent.reference.landing = LANDING_REFERENCE(agent,dt,0.1);
 agent.reference.arming = ARMING_REFERENCE(agent,[]);
 agent.cha_allocation.a.reference = "arming";
agent.cha_allocation.f.reference = "bezier";
agent.cha_allocation.t.reference = "takeoff";
agent.cha_allocation.l.reference = "landing";
 agent.cha_allocation.a.controller = "hlc";
agent.cha_allocation.f.controller =  "kmpc";
agent.cha_allocation.t.controller = "hlc";
agent.cha_allocation.l.controller = "hlc";
function post(app)
app.logger.plot({1, "p", "er"},"ax",app.UIAxes,"xrange",[app.time.ts,app.time.te]);
app.logger.plot({1, "inner_input", ""},"ax",app.UIAxes2,"xrange",[app.time.ts,app.time.te]);
% app.logger.plot({1, "v", "e"},"ax",app.UIAxes3,"xrange",[app.time.ts,app.time.te]);
app.logger.plot({1, "input", ""},"ax",app.UIAxes3,"xrange",[app.time.ts,app.time.te]);
% app.logger.plot({1, "input", ""},"ax",app.UIAxes5,"xrange",[app.time.ts,app.time.te]);
% app.logger.plot({1, "inner_input", ""},"ax",app.UIAxes6,"xrange",[app.time.ts,app.time.te]);
end


function est =import_vars_from_mfile(mfile)
   
    fid = fopen(mfile, 'r');
    if fid == -1
        error('Cannot open file: %s', mfile);
    end

    while ~feof(fid)
        line = fgetl(fid);
        if ischar(line) && ~isempty(strtrim(line)) && ~startsWith(strtrim(line), '%')
            try
                evalin('base', line); 
            catch ME
                warning('Skipped line: %s\nReason: %s\n', line, ME.message);
            end
        end
    end

    fclose(fid);
    fprintf('Imported variables from %s into workspace.\n', mfile);

     try
        est = evalin('base', 'est');
    catch
        error('Variable ''est'' was not defined in the file.');
    end
end