tmp = matlab.desktop.editor.getActive;
dir = fileparts(tmp.Filename);
if ~contains(path,dir)
    cd(erase(dir,'\mode'));
[~, tmp] = regexp(genpath('.'), '\.\\\.git.*?;', 'match', 'split');
cellfun(@(xx) addpath(xx), tmp, 'UniformOutput', false);
close all hidden; clear ; clc;
userpath('clear');
end
%%
N = 1; % the number of agents
ts = 0; % initial time
dt = 0.025; % sampling period
te = 10000; % termina time
time = TIME(ts, dt, te, N);
in_prog_func = @(app) in_prog(app);
post_func = @(app) post(app);
logger = LOGGER(1, size(ts:dt:te, 2), 1, [], []);
logger.display_func = @(agent, time) build_display_vector(agent, time);
logger.display_on = true;
logger.set_time_handler(time);
fprintf("表示物\nref:[px, py, pz]  est:[px, py, pz]  delta_U:[T, tx, ty, tz]\n\n");

motive = Connector_Natnet('192.168.100.59'); % connect to Motive 405
motive.getData([], []); % get data from Motive
rigid_ids = [1]; % rigid-body number on Motive
sstate = motive.result.rigid(rigid_ids);
initial_state.p = sstate.p;
initial_state.q = sstate.q;
initial_state.v = [0; 0; 0];
initial_state.w = [0; 0; 0];

agent = DRONE;
agent.plant = DRONE_EXP_MODEL(agent, Model_Drone_Exp(dt, initial_state, "serial", "COM3"));
agent.parameter = DRONE_PARAM("DIATONE");
agent.estimator.set_function_class("ekf", EKF(agent, Estimator_EKF(agent,dt,MODEL_CLASS(agent,Model_EulerAngle(dt, initial_state, 1)))));
agent.sensor.set_function_class("motive", MOTIVE(agent,motive, dt));


% リファレンス設定 ~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-
% ~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-
takeoff_zd = 1.0; % だいたい1m
center = [base';takeoff_zd]; % base基準
center = [0;0;takeoff_zd]; % 原点

% ref = TIME_VARYING_REFERENCE(agent,{"gen_ref_saddle",{"freq",10, "center",center, "radius",[0,0,0]}, 4}); % center_hover
% ref = TIME_VARYING_REFERENCE(agent,{"gen_ref_saddle",{"freq",10, "center",[base'+1;takeoff_zd+1], "radius",[0,0,0]}, 4}); % point_hover
% ref = TIME_VARYING_REFERENCE(agent,{"gen_ref_circle",{"freq",10, "center",center, "radius",1.0}, 4}); %                     circle
% ref = TIME_VARYING_REFERENCE(agent,{"gen_ref_saddle",{"freq",5, "center",center, "radius",[1,1,0.25]}, 4}); %               saddle
ref = TIME_VARYING_REFERENCE(agent,{"gen_ref_lemniscate",{"freq",10, "center",center, "radius",1, "x",1}, 4}); %            lemniscate
% ref = TIME_VARYING_REFERENCE(agent,{"gen_ref_triangle",{"freq",10, "center",center, "radius",[1,1,0]}, 4}); %               triangle
% ref = TIME_VARYING_REFERENCE(agent,{"gen_ref_flower",{"freq",10, "center",center, "radius",1.0}, 4}); %                     flower
% ref = TIME_VARYING_REFERENCE(agent,{"gen_ref_heart",{"freq",10, "center",center, "radius",1.0}, 4}); %                      heart
% ref = TIME_VARYING_REFERENCE(agent,{"gen_ref_star",{"freq",15, "center",center, "radius",1.0}, 4}); %                       star
% ref = TIME_VARYING_REFERENCE(agent,{"gen_ref_spline",{"point",20, "order",9, "point_dt",4.5, ...
%                                     "start_end",center', "xlim",[-1,1], "ylim",[0,0], "zlim",[takeoff_zd,takeoff_zd]}}); %  spline9th 20 points
% ref = TIME_VARYING_REFERENCE(agent,{"gen_ref_spline",{"point",60, "order",9, "point_dt",2.5, ...
%                                     "start_end",center', "xlim",[-1,1], "ylim",[0,0], "zlim",[takeoff_zd,takeoff_zd]}}); %  spline9th 60 points
% 
% refpoints = {struct("f",center, "g",[1;0;takeoff_zd], "h",center, "j",[0;1;takeoff_zd], "k",center, "z",[0;0;takeoff_zd+1], "x",center, "c",[-1;-1;takeoff_zd], "v",center, "b",[1;-1;takeoff_zd+1], "n",center), 7.5};
% refpoints = {struct("f",center, "g",[1;0;takeoff_zd], "h",[1;1;takeoff_zd], "j",center, "k",[0;0;takeoff_zd+0.5]), 7.5};
% ref = MULTI_POINT_REFERENCE(agent,refpoints); % multiP2P


agent.reference.set_function_class("time_varying", ref) % 最終的なset
agent.reference.set_function_class("takeoff", TAKEOFF_REFERENCE(agent,"zd",takeoff_zd));
agent.reference.set_function_class("landing", LANDING_REFERENCE(agent,"dt",dt,"vd",0));
%~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-
%~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-


agent.controller.set_function_class("nominal", HLC(agent,Controller_HL(dt)));
% agent.controller.set_function_class("nominal", FUNCTIONAL_HLC_SERVO(agent, Controller_FHL_Servo(dt))); % 位置偏差に対するサーボ系HL

% Learn Exp
ptName = "2026-7-23_11_28_41__DNN12__Exp_random__Euler__Activation=ReLU__100000epoch_model.pt";

agent.controller.set_function_class("nnmec", PythonNNMEC(agent, ptName));

% agent.controller.set_function_class("nnmec", PythonRNNMEC(agent, RNNptName, "len",4));


% cha_allocation ==============================================
agent.cha_allocation.f.reference = "time_varying";
agent.cha_allocation.a.reference = "takeoff";
agent.cha_allocation.t.reference = "takeoff";
agent.cha_allocation.l.reference = "landing";

agent.set_cha_allocation_for_all("controller", ["nominal","nnmec"])


function post(app)
phase = "tfl";
FS = 16; %FontSize
LW = 1.5;%LineWidth
app.logger.plot({1, "p", "esr"},"ax",app.UIAxes,"phase",phase);

% app.logger.plot({1, "p", "esr"},"fig_num",10, "phase",phase, "Fontsize",FS, "Linewidth",LW);
app.logger.plot({1, "q", "es"},"fig_num",20, "phase",phase, "Fontsize",FS, "Linewidth",LW);
% app.logger.plot({1, "v", "er"},"fig_num",30, "phase",phase, "Fontsize",FS, "Linewidth",LW);
% app.logger.plot({1, "w", "e"},"fig_num",40, "phase",phase, "Fontsize",FS, "Linewidth",LW);
% app.logger.plot({1, "input", ""},"fig_num",50, "phase",phase, "Fontsize",FS, "Linewidth",LW);
% app.logger.plot({1, "input2:4", ""},"fig_num",51, "phase",phase, "Fontsize",FS, "Linewidth",LW);
% app.logger.plot({1, "inner_input", ""},"fig_num",60, "phase",phase, "Fontsize",FS, "Linewidth",LW);
% app.logger.plot({1, "p1-p2", "er"},"fig_num",70, "color",0, "phase",phase, "Fontsize",FS, "Linewidth",LW);
% app.logger.plot({1, "p1-p2-p3", "er"},"fig_num",71, "color",0, "phase",phase, "Fontsize",FS, "Linewidth",LW);

% show_cooperative_animation(app);

plot_calc_time(app.logger, app.time);
end

function dfunc(app)
app.logger.plot({1, "p", "er"},"ax",app.UIAxes,"phase","tfl");
end



function show_animation(app)
% 協調吊り下げ（複数ドローン＋牽引物）のアニメーションを生成する。
% Nは機体数、牽引物はN+1
if app.logger.k <= 1
    return
end
  mov = DRAW_DRONE_MOTION(app.logger, "self", app.agent, "target", 1);
  mov.animation(app.logger, "self", app.agent, "target", 1, "Motive_ref", 1);
end



function v = build_display_vector(agent, time)
% コンソール表示用の文字列を作る。
% 参照位置、推定位置、入力を並べる。
idx = 1;
if ~isprop(agent(idx).reference.result.state, "xd")
    v = [];
    return
end
xd = agent(idx).reference.result.state.xd;
if isfield(agent(idx).estimator.result, "state")
    p = agent(idx).estimator.result.state.p;
    q = agent(idx).estimator.result.state.q;
else
    p = [NaN; NaN; NaN];
    q = [NaN; NaN; NaN];
end
% u = agent(idx).controller.result.input;
% v = sprintf("%c %.3f : R [%7.3f,%7.3f,%7.3f] : P [%7.3f,%7.3f,%7.3f] : Q [%7.3f,%7.3f,%7.3f] : U [%7.3f,%7.3f,%7.3f,%7.3f]",agent(idx).cha, time.t, xd(1:3)', p', q', u');

du = agent(idx).controller.result.delta_input;
v = sprintf("%c %.3f : R [%7.3f,%7.3f,%7.3f] : P [%7.3f,%7.3f,%7.3f] : Q [%7.3f,%7.3f,%7.3f] : dU [%7.3f,%7.3f,%7.3f,%7.3f]",agent(idx).cha, time.t, xd(1:3)', p', q', du');
end