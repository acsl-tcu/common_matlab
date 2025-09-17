clc
ts = 0; % initial time
dt = 0.025; % sampling period
te = 100; % termina time
time = TIME(ts,dt,te);
in_prog_func = @(app) in_prog(app);
post_func = @(app) post(app);
logger = LOGGER(1, size(ts:dt:te, 2), 1, [],[]);
mmatflag = 0;
model_file = '2025-07-30_exp_koseki_code00_randompp';
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
agent.plant = DRONE_EXP_MODEL(agent,Model_Drone_Exp(dt, initial_state, "serial", "COM4")); %プロポ有線
agent.parameter = DRONE_PARAM("DIATONE");
agent.estimator = EKF(agent, Estimator_EKF(agent,dt,MODEL_CLASS(agent,Model_EulerAngle(dt, initial_state, 1)), ["p", "q"]));
agent.sensor = MOTIVE(agent, Sensor_Motive(1,0, motive));
agent.input_transform = THRUST2THROTTLE_DRONE(agent,InputTransform_Thrust2Throttle_drone_KMPC()); % 推力からスロットルに変換
% agent.input_transform.param.pitch_offset = 510;
% agent.input_transform.param.roll_offset = 490;
% agent.reference.bezier = BEZIER_REFERENCE(agent,{[0,0,0.6]},time);
 agent.reference.time_var= TIME_VARYING_REFERENCE(agent,{"gen_ref_saddle",{"freq",20,"orig",[0;0;0.8],"size",[0,0,0]},"HL"});%{"Case_study_trajectory",{[0,0,0.6]},"HL"});

%2つのコントローラの設定---------------------------------------------------------------------------------------------------
agent.controller.hlc = HLC(agent,Controller_HL(dt));
agent.controller.kmpc = MPC_CONTROLLER_KMC_kyo_guiexperiment(agent,Controller_MPC_KMC_kyo(dt,model_file,agent)); %最適化手法：QP
agent.controller.result.input = [0;0;0;0];
run("ExpBase");
agent.cha_allocation.reference = "time_var";
agent.cha_allocation.controller = "hlc";
agent.cha_allocation.f.controller = ["kmpc","hlc"];
% agent.cha_allocation.f.controller = ["kmpc"];
function post(app)
app.logger.plot({1, "p", "er"},"ax",app.UIAxes,"phase","tfl");
% app.logger.plot({1, "inner_input", ""}, "fig_num", 1,"xrange",[app.time.ts,app.time.te]);
 app.logger.plot({1, "q", "e"},"fig_num", 4,"phase","tfl");
app.logger.plot({1, "input", ""},"fig_num", 1,"phase","tfl");
app.logger.plot({1, "v", "er"}, "fig_num", 2,"phase","tfl");
% app.logger.plot({1, "input", ""},"ax",app.UIAxes5,"xrange",[app.time.ts,app.time.te]);
 app.logger.plot({1, "inner_input", ""},"fig_num",3,"phase","tfl");
% app.logger.plot({1, "controller.result.input_kmpc", ""}, "fig_num", 4);
   app.logger.plot({{1, "controller.result.hlc", ""},{1, "controller.result.kmpc", ""}},"fig_num", 5,"phase","f");
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