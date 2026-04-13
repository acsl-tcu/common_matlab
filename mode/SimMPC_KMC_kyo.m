ts = 0; % initial time
dt = 0.025; % sampling period
te = 50; % terminal time
time = TIME(ts,dt,te); % instance of time class
post_func = @(app) post(app); % function working at the "draw button" pushed.
motive = Connector_Natnet_sim(dt); % imitation of Motive camera (motion capture system)
logger = LOGGER(1, size(ts:dt:te, 2), 0, [],[]); % instance of LOOGER class for data logging
initial_state.p = arranged_position([0, 0], 1, 1, 0); % [x, y], 1, 1, z
initial_state.q = [1; 0; 0; 0];
initial_state.v = [0; 0; 0];
initial_state.w = [0; 0; 0];
mmatflag = 0;

 model_file = '2026-04-01_exp_ob26_code00_random';%%%%%observer 1 なし
 % model_file = '2026-04-01_exp_ob71_code00_random';

%%
agent = DRONE;
agent.plant = MODEL_CLASS(agent,Model_EulerAngle(dt, initial_state, 1));
agent.parameter = DRONE_PARAM("DIATONE");
agent.plant.param(1) = 0.7875; % ５％増->0.7875, ５％減->0.7125
% agent.plant.param(1) = 1.0;

% agent.plant.param(6) = 0.24; % (1;1;1)P2PでのNNMECを入れたときの限界値
% agent.plant.param(7) = 0.24;
agent.plant.param(6) = 0.18; % x3
agent.plant.param(7) = 0.18; % (1;1;1)P2Pでの限界値
% agent.plant.param(6) = 0.15;
% agent.plant.param(7) = 0.15;

% agent.plant.param(6) = 0.12; % x2
% agent.plant.param(6) = 0.15; % x2.5

% agent.plant.param(10:13) = [0.003, 0.003, 0.003, 0.003];
% agent.plant.param(4) = 0.07;
% agent.plant.param(10) = 0.003;
agent.estimator = EKF(agent, Estimator_EKF(agent,dt,MODEL_CLASS(agent,Model_EulerAngle(dt, initial_state, 1)),["p", "q"]));
agent.sensor = MOTIVE(agent, Sensor_Motive(1,0, motive));
% agent.reference.bezier = BEZIER_REFERENCE(agent,{[0,0,0.6]},time);

 % agent.reference.time_var= TIME_VARYING_REFERENCE(agent,{"gen_ref_saddle",{"freq",8,"orig",[0;0;0.6],"size",[0,0,0]},"HL"});%{"Case_study_trajectory",{[0,0,0.6]},"HL"});
% agent.reference.time_var= TIME_VARYING_REFERENCE(agent,{"gen_ref_spline",{"point",12,"order",9,"point_dt",5,"ManualSetting",0,"check",1}});
% agent.reference.time_varying = MY_POINT_REFERENCE(agent, {struct("f", center, "g", [1;0;takeoff_zd], "h",center, "j",[0;1;takeoff_zd], "k",center, "z",[0;0;takeoff_zd-0.5], "x",center...
                                                                % , "c",[-1;-1;takeoff_zd], "v",center, "b",[1;-1;takeoff_zd+0.5], "n",center), 7.5}); % P2P
% agent.reference.time_var = MY_POINT_REFERENCE(agent, {struct("f", [0;0;0.6], "g", [0;-1;0.6], "h",[0;1;0.6],"j",[0;0;0.6],"k",[1;0;0.6],"z",[0;0;0.6]), 8}); % P2P

 agent.reference.time_var= TIME_VARYING_REFERENCE(agent,{"gen_ref_saddle",{"freq",12,"orig",[0;0;0.6],"size",[1,1,0]},"HL"});%{"Case_study_trajectory",{[0,0,0.6]},"HL"});
% agent.reference.time_var= TIME_VARYING_REFERENCE(agent,{"gen_ref_spline",{"point",6,"order",6,"point_dt",8,"ManualSetting",0,"check",1}});
% agent.reference.time_var = TIME_VARYING_REFERENCE(agent,{"gen_ref_heart", {"freq",20,"orig",[0 0 0.6],"size",[1 1 0],"phase",-pi/2}});

% 8字
% agent.reference.time_var = TIME_VARYING_REFERENCE(agent,{"gen_ref_figure8", {"freq",20,"orig",[0 0 0.6],"size",[1 1 0],"phase",0}});
% agent.reference.time_varying = MY_POINT_REFERENCE(agent, {struct("f", center, "g", [1;0;takeoff_zd], "h",center, "j",[0;1;takeoff_zd], "k",center, "z",[0;0;takeoff_zd-0.5], "x",center...
                                                                % , "c",[-1;-1;takeoff_zd], "v",center, "b",[1;-1;takeoff_zd+0.5], "n",center), 7.5}); % P2P
% agent.reference.time_var = MY_POINT_REFERENCE(agent, {struct("f", [0;0;0.6], "g", [0;-1;0.6], "h",[0;1;0.6],"j",[0;0;0.6],"k",[1;0;0.6],"z",[0;0;0.6]), 6}); % P2P

%2つのコントローラの設定---------------------------------------------------------------------------------------------------
agent.controller.hlc = HLC(agent,Controller_HL(dt));
 % agent.controller.kmpc = MPC_CONTROLLER_KMC_kyo_guiexperiment(agent,Controller_MPC_KMC_kyo(dt,model_file,agent)); %最適化手法：QP
agent.controller.kmpc = LPV_EDMD_CONTROLLER(agent,Controller_LPV_EDMD(dt,model_file,agent)); %最適化手法：QP
run("SimBase");
agent.cha_allocation.reference = "time_var";
agent.cha_allocation.controller = "hlc";
agent.cha_allocation.f.controller = ["hlc"];
function post(app)
% app.logger.plot({1, "p", "er"},"ax",app.UIAxes,"xrange",[app.time.ts,app.time.te],"linewidth", 2.5, ...
%     "fontsize", 14);
% % app.logger.plot({1, "inner_input", ""}, "fig_num", 1,"xrange",[app.time.ts,app.time.te]);
% % app.logger.plot({1, "v", "e"},"ax",app.UIAxes3,"xrange",[app.time.ts,app.time.te]);
% app.logger.plot({1, "input", ""},"fig_num", 2,"xrange",[app.time.ts,app.time.te], "linewidth", 2.5, ...
%     "fontsize", 14);
% app.logger.plot({1, "v", "er"}, "fig_num", 3,"xrange",[app.time.ts,app.time.te], "linewidth", 2.5, ...
%     "fontsize", 14);
%  app.logger.plot({1, "p1-p2-p3", "er"},"fig_num", 4,"phase",'tfl', "color",0, "linewidth", 2.5, ...
%     "fontsize", 14);
fig = figure(100); clf
tiledlayout(2,2,"TileSpacing","compact","Padding","compact")

%% 1 — p error
nexttile
app.logger.plot({1, "p", "er"}, ...
    "ax", gca, ...
    "xrange",[app.time.ts,app.time.te], ...
    "linewidth", 2.5, ...
    "fontsize", 14);
title("Position Error")

%% 2 — input
nexttile
app.logger.plot({1, "input", ""}, ...
    "ax", gca, ...
    "xrange",[app.time.ts,app.time.te], ...
    "linewidth", 2.5, ...
    "fontsize", 14);
title("Control Input")

%% 3 — velocity error
nexttile
app.logger.plot({1, "v", "er"}, ...
    "ax", gca, ...
    "xrange",[app.time.ts,app.time.te], ...
    "linewidth", 2.5, ...
    "fontsize", 14);
title("Velocity Error")

%% 4 — phase plot
nexttile
app.logger.plot({1, "p1-p2-p3", "er"}, ...
    "ax", gca, ...
    "phase",'tfl', ...
    "color",0, ...
    "linewidth", 2.5, ...
    "fontsize", 14);
title("Phase Plot")
% app.logger.plot({1, "inner_input", ""},"ax",app.UIAxes6,"xrange",[app.time.ts,app.time.te]);
% app.logger.plot({1, "controller.result.input_kmpc", ""}, "fig_num", 4);
% app.logger.plot({1, "controller.result.hlc", ""},{1, "controller.result.kmpc", ""},"fig_num", 4,"phase","f");
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
