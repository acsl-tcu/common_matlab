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
ts = 0; % initial time
dt = 0.025; % sampling period
te = 50; % terminal time
time = TIME(ts,dt,te); % instance of time class
in_prog_func = @(app) dfunc(app); % in progress plot
post_func = @(app) post(app); % function working at the "draw button" pushed.
motive = Connector_Natnet_sim(dt); % imitation of Motive camera (motion capture system)
logger = LOGGER(1, size(ts:dt:te,2), 0, [],[]); % instance of LOOGER class for data logging
logger.display_func = @(agent, time) build_display_vector(agent, time);
logger.display_on = true;
fprintf("表示物\nref:[px, py, pz]  est:[px, py, pz]  U:[T, tx, ty, tz]\n\n");

base = [0,0]; % center
initial_state.p = arranged_position(base, 1, 1, 0);
initial_state.q = [1; 0; 0; 0];
initial_state.v = [0; 0; 0];
initial_state.w = [0; 0; 0];

agent = DRONE;
agent.parameter = DRONE_PARAM("DIATONE"); % プラントでModel_EulerAngleを使うときはノミナルモデル

% プラントモデル定義 ================================================================================================================================
plant = Model_EulerAngle(dt, initial_state, 1);
agent.plant = MODEL_CLASS(agent, plant);

% デフォルト物理パラメータ(DRONE_PARAM.m準拠: 2025/07/07時点)
% 1:mass=0.75  |  2,3:Lx,y=0.16  |  4,5: lx,y=0.08  |  6,7,8: jx,y,z=0.06  |  9: gravity=9.81
% 10,11,12,13: km(各ロータ定数)=0.0301  |  14,15,16,17: k(推力定数)=8.0e-6  |  18: rotor_r=0.0392

% ↓パラメータの上書き モデル誤差をプラントに与える
agent.plant.param(1) = 0.7875; % ５％増->0.7875, ５％減->0.7125
% agent.plant.param(1) = 1.0;

% agent.plant.param(6) = 0.2; % モデル誤差を陽に入れたSimデータ取得時(Spline)の値（センサーノイズ無し）
% agent.plant.param(7) = 0.2;
% agent.plant.param(6) = 0.19; % モデル誤差を陽に入れたSimデータ取得時(Spline)の値（センサーノイズ分散=10^-3）
% agent.plant.param(7) = 0.19;
% agent.plant.param(6) = 0.18; % x3
% agent.plant.param(7) = 0.18; % (1;1;1)P2Pでの限界値
% agent.plant.param(6) = 0.24;
% agent.plant.param(7) = 0.24;

agent.plant.param(6) = 0.12;
agent.plant.param(7) = 0.12;
%===================================================================================================================================================
agent.estimator.set_function_class("ekf", EKF(agent, Estimator_EKF(agent,dt,MODEL_CLASS(agent,Model_EulerAngle(dt, initial_state, 1)))));


% agent.sensor.set_function_class("motive", MOTIVE(agent,motive));
agent.sensor.set_function_class("direct", DIRECT_SENSOR(agent, 0.001, struct("output_list",["p","q"]))); % 分散 10^-3
% agent.sensor.set_function_class("direct", DIRECT_SENSOR(agent, 0.0, struct("output_list",["p","q"]))); % 真値を使う


% リファレンス設定 ~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-
% ~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-
takeoff_zd = 0.5; % だいたい1m
center = [base';takeoff_zd]; % base基準
center = [0;0;takeoff_zd]; % 原点

center_hover    = TIME_VARYING_REFERENCE(agent,{"gen_ref_saddle",{"freq",10, "center",center, "radius",[0,0,0]}, 4});
% point_hover     = TIME_VARYING_REFERENCE(agent,{"gen_ref_saddle",{"freq",10, "center",[base'+1;takeoff_zd+1], "radius",[0,0,0]}, 4});
% circle          = TIME_VARYING_REFERENCE(agent,{"gen_ref_circle",{"freq",10, "center",center, "radius",1.0}, 4});
saddle          = TIME_VARYING_REFERENCE(agent,{"gen_ref_saddle",{"freq",5, "center",center, "radius",[1,1,0.25]}, 4});
% lemniscate      = TIME_VARYING_REFERENCE(agent,{"gen_ref_lemniscate",{"freq",5, "center",center, "radius",1, "x",1}, 4});
% triangle        = TIME_VARYING_REFERENCE(agent,{"gen_ref_triangle",{"freq",10, "center",center, "radius",[1,1,0]}, 4});
% flower          = TIME_VARYING_REFERENCE(agent,{"gen_ref_flower",{"freq",10, "center",center, "radius",1.0}, 4});
% heart           = TIME_VARYING_REFERENCE(agent,{"gen_ref_heart",{"freq",10, "center",center, "radius",1.0}, 4});
% star            = TIME_VARYING_REFERENCE(agent,{"gen_ref_star",{"freq",15, "center",center, "radius",1.0}, 4});
% spline9th       = TIME_VARYING_REFERENCE(agent,{"gen_ref_spline",{"point",20, "order",9, "point_dt",4.5, "xylim",[-1,1], "zlim",[0.5,1.5]}}); % 20 points
% spline9th       = TIME_VARYING_REFERENCE(agent,{"gen_ref_spline",{"point",60, "order",9, "point_dt",4.5, "xylim",[-1,1], "zlim",[0.5,1.5]}}); % 60 points

% refpoints = {struct("f",center, "g",[1;0;takeoff_zd], "h",center, "j",[0;1;takeoff_zd], "k",center, "z",[0;0;takeoff_zd+1], "x",center, "c",[-1;-1;takeoff_zd], "v",center, "b",[1;-1;takeoff_zd+1], "n",center), 7.5};
% refpoints = {struct("f",center, "g",[1;0;takeoff_zd], "h",[1;1;takeoff_zd], "j",center, "k",[0;0;takeoff_zd+0.5]), 7.5};
% multiP2P        = MULTI_POINT_REFERENCE(agent,refpoints);


agent.reference.set_function_class("time_varying", saddle) % 最終的なset．２つ目の引数の名前を適宜変更
agent.reference.set_function_class("takeoff", TAKEOFF_REFERENCE(agent,"zd",takeoff_zd));
agent.reference.set_function_class("landing", LANDING_REFERENCE(agent,"dt",dt,"vd",0));
%~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-
%~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-


agent.controller.set_function_class("nominal", HLC(agent,Controller_HL(dt)));
% agent.controller.set_function_class("nominal", FUNCTIONAL_HLC_SERVO(agent, Controller_FHL_Servo(dt))); % 位置偏差に対するサーボ系HL


onnxName = "2026-5-18_15_0_32__DNN12__Plant_data_Sim_60ptsSpline__m0.7875_jxjy0.19__Euler__Activation=SiLU__100000epoch.onnx";

% onnxName = "2026-5-15_16_37_24__DNN12__Plant_data_Exp_random__Euler__Activation=ReLU__100000epoch.onnx";
% onnxName = "2026-2-3_9_53_19__DNN12__Plant_data_Exp_random__Euler__Activation=SiLU__100000epoch.onnx"; % 2025年度卒論で使用

agent.controller.set_function_class("nnmec", NNMEC(agent, onnxName));


% cha_allocation ==============================================
agent.cha_allocation.f.reference = "time_varying";
agent.cha_allocation.a.reference = "takeoff";
agent.cha_allocation.t.reference = "takeoff";
agent.cha_allocation.l.reference = "landing";

agent.set_cha_allocation_for_all("controller", ["nominal","nnmec"])


function post(app)
LW = 1.5; % LineWidth
FS = 18; % FontSize
phase = "tfl";
phase = "tf";
calc_rmse(app.logger);
app.logger.plot({1, "p", "er"},"ax",app.UIAxes, "phase",phase, "fig_num",1, "Linewidth",LW, "Fontsize",FS);
% app.logger.plot({1, "p", "er"}, "phase",phase, "fig_num",1, "Linewidth",LW, "Fontsize",FS, "color",fcolor);
% app.logger.plot({1, "p1:2", "er"}, "phase",phase, "fig_num",1, "Linewidth",LW, "Fontsize",FS, "color",fcolor);
% app.logger.plot({1, "q", "e"}, "phase",phase, "fig_num",2, "Linewidth",LW, "Fontsize",FS, "color",fcolor);
% app.logger.plot({1, "v", "er"}, "phase",phase, "fig_num",3, "Linewidth",LW, "Fontsize",FS, "color",fcolor);
% app.logger.plot({1, "w", "e"}, "phase",phase, "fig_num",4, "Linewidth",LW, "Fontsize",FS, "color",fcolor);
% app.logger.plot({{1, "input", ""}, {1, "controller.result.nominal_input", ""},...
%     {1, "controller.result.delta_input", ""}}, "phase",phase,"fig_num",5); % inputをまとめて見る
show_animation(app);
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
else
    p = [NaN; NaN; NaN];
end
u = agent(idx).controller.result.input;
v = sprintf("%c %.3f : R [%7.3f,%7.3f,%7.3f] : P [%7.3f,%7.3f,%7.3f] : U [%7.3f,%7.3f,%7.3f,%7.3f]",agent(idx).cha, time.t, xd(1:3)', p', u');
end
