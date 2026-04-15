clc
ts = 0; % initial time
dt = 0.025; % sampling period
te = 150; % termina time
time = TIME(ts,dt,te);
in_prog_func = @(app) in_prog(app);
post_func = @(app) post(app);
logger = LOGGER(1, size(ts:dt:te, 2), 1, [],[]);
motive = Connector_Natnet('192.168.100.43'); % connect to Motive　
motive.getData([], []); % get data from Motive
rigid_ids = 1; % rigid-body number on Motive
sstate = motive.result.rigid(rigid_ids);
initial_state.p = sstate.p;
initial_state.q = sstate.q;
initial_state.v = [0; 0; 0];
initial_state.w = [0; 0; 0];
agent = DRONE;
%agent.plant = DRONE_EXP_MODEL(agent,Model_Drone_Exp(dt, initial_state, "udp", [1, 252])); %プロポ無線
agent.plant = DRONE_EXP_MODEL(agent,Model_Drone_Exp(dt, initial_state, "serial", "COM3")); %プロポ有線
agent.parameter = DRONE_PARAM("DIATONE");
agent.estimator = EKF(agent, Estimator_EKF(agent,dt,MODEL_CLASS(agent,Model_EulerAngle(dt, initial_state, 1)), ["p", "q"]));
agent.sensor = MOTIVE(agent, Sensor_Motive(1,0, motive));
agent.input_transform = THRUST2THROTTLE_DRONE(agent,InputTransform_Thrust2Throttle_drone_KMPC()); % 推力からスロットルに変換
agent.reference.time_var= TIME_VARYING_REFERENCE(agent,{"gen_ref_saddle",{"freq",8,"orig",[0;0;1.0],"size",[0,0,0]},"HL"});%{"Case_study_trajectory",{[0,0,0.6]},"HL"});
% agent.reference.time_var= TIME_VARYING_REFERENCE(agent,{"gen_ref_spline",{"point",10,"order",9,"point_dt",5,"ManualSetting",0,"check",1}});
% agent.reference.time_var = MY_POINT_REFERENCE(agent, {struct("f", center, "g", [1;0;takeoff_zd], "h",center, "j",[0;1;takeoff_zd], "k",center, "z",[0;0;takeoff_zd-0.5], "x",center...
                                                                % , "c",[-1;-1;takeoff_zd], "v",center, "b",[1;-1;takeoff_zd+0.5], "n",center), 7.5}); % P2P
% agent.reference.time_var = MY_POINT_REFERENCE(agent, {struct("f", [0;0;0.6], "g", [0;0;0.5], "h",[0;0;0.8],"j",[0;0;1.0],"k",[0;0;1.2],"z",[0;0;0.4],"x",[0;0;0.5],"c",[0;0;0.7],"v",[0;0;0.6]), 7.5}); % P2P
%2つのコントローラの設定---------------------------------------------------------------------------------------------------
agent.controller.hlc = HLC(agent,Controller_HL(dt));
% agent.controller.kqlmpc = KQ_LMPC_CONTROLLER(agent,Controller_KQ_LMPC(dt,agent)); %最適化手法：QP
agent.controller.kqlmpc = KQ_LMPC_EDMD_CONTROLLER(agent,Controller_KQ_LMPC_EDMD(dt,agent)); 
agent.controller.result.input = [0;0;0;0];
run("ExpBase");
agent.cha_allocation.reference = "time_var";
agent.cha_allocation.controller = "hlc";
agent.cha_allocation.f.controller = ["kqlmpc"];
% agent.cha_allocation.f.controller = ["kqlmpc","hlc"];
function post(app)
app.logger.plot({1, "p", "er"},"ax",app.UIAxes,"phase","tfl");
% app.logger.plot({1, "inner_input", ""}, "fig_num", 1,"xrange",[app.time.ts,app.time.te]);
 app.logger.plot({1, "q", "e"},"fig_num", 4,"phase","tfl");
app.logger.plot({1, "input", "e"},"fig_num", 1,"phase","tfl");
app.logger.plot({1, "v", "er"}, "fig_num", 2,"phase","tfl");
% app.logger.plot({1, "input", ""},"ax",app.UIAxes5,"xrange",[app.time.ts,app.time.te]);
 app.logger.plot({1, "inner_input", ""},"fig_num",3,"phase","tfl");
 app.logger.plot({1, "p1-p2-p3", "er"},"fig_num", 6,"phase",'tfl', "color",0);
  % app.agent.animation(app.logger, "target",1, "fig_num",999, "mp4",1, "phase",'tfl');

   % app.logger.plot({{1, "controller.result.hlc", ""},{1, "controller.result.kqlmpc", ""}},"fig_num", 5,"phase","f");
    app.logger.plot({1, "reference.result.state.p", ""},"fig_num", 5,"phase","f");
    app.logger.plot({1, "controller.result.ref1", ""},"fig_num", 5,"phase","f");
%    fig = figure(100); clf
% tiledlayout(2,2,"TileSpacing","compact","Padding","compact")
% 
% %% 1 — p error
% nexttile
% app.logger.plot({1, "p", "er"}, ...
%     "ax", gca, ...
%     "xrange",[app.time.ts,app.time.te], ...
%     "linewidth", 2.5, ...
%     "fontsize", 14);
% title("Position Error")
% 
% %% 2 — input
% nexttile
% app.logger.plot({1, "input", ""}, ...
%     "ax", gca, ...
%     "xrange",[app.time.ts,app.time.te], ...
%     "linewidth", 2.5, ...
%     "fontsize", 14);
% title("Control Input")
% 
% %% 3 — velocity error
% nexttile
% app.logger.plot({1, "v", "er"}, ...
%     "ax", gca, ...
%     "xrange",[app.time.ts,app.time.te], ...
%     "linewidth", 2.5, ...
%     "fontsize", 14);
% title("Velocity Error")
% 
% %% 4 — phase plot
% nexttile
% app.logger.plot({1, "p1-p2-p3", "er"}, ...
%     "ax", gca, ...
%     "phase",'tfl', ...
%     "color",0, ...
%     "linewidth", 2.5, ...
%     "fontsize", 14);
% title("Phase Plot")
   % app.agent.animation(app.logger, "target",1, "fig_num",999, "mp4",1, "phase",'tfl');
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