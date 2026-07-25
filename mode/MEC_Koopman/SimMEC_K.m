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
post_func = @(app) dfunc(app); % function working at the "draw button" pushed.
% motive = Connector_Natnet_sim(1, dt); % imitation of Motive camera (motion capture system)
logger = LOGGER(1, size(ts:dt:te, 2), 0, [],[]); % instance of LOOGER class for data logging
logger.display_func = @(agent, time) build_display_vector(agent, time);
logger.display_on = true;
fprintf("表示物\nref:[px, py, pz]  est:[px, py, pz]  U:[T, tx, ty, tz]\n\n");


initial_state.p = arranged_position([0, 0], 1, 1, 0);
initial_state.q = [1; 0; 0; 0];
initial_state.v = [0; 0; 0];
initial_state.w = [0; 0; 0];

agent = DRONE;
agent.parameter = DRONE_PARAM("DIATONE"); %ノミナルモデル．DRONE_PARAMのパラメータを上書きしている．
% プラントモデル定義 ================================================================================================================================
plant_model = Model_EulerAngle(dt, initial_state, 1);
% plant_model = MODEL_CLASS(agent,Model_EulerAngle(dt, initial_state, 1));
% デフォルト物理パラメータ(DRONE_PARAM準拠: 2025/07/07時点)
% 1:mass=0.75  |  2,3:Lx,y=0.16  |  4,5: lx,y=0.08  |  6,7,8: jx,y,z=0.06  |  9: gravity=9.81
% 10,11,12,13: km(各ロータ定数)=0.0301  |  14,15,16,17: k(推力定数)=8.0e-6  |  18: rotor_r=0.0392

% ↓パラメータの上書き モデル誤差をプラントに与える
% plant_model.param.param(1) = 0.7875; % ５％減->0.7125 ５％増->0.7875
% plant_model.param.param(1) = 0.8;
% plant_model.param.param(1) = 0.6; % ５％減->0.7125 ５％増->0.7875

plant_model.param.param(6) = 0.185; % 0.18<jx,jy<0.22ぐらいが良き
plant_model.param.param(7) = 0.185; % 

% agent.parameter = DRONE_PARAM("DIATONE", "jx", 0.185);
% agent.parameter = DRONE_PARAM("DIATONE", "jy", 0.185);
% plant_model.param.param(8) = 0.6; % 0.18 < jzぐらいが良き
% plant_model.param.param(10) = 0.6; % ５％減->0.028595
% plant_model.param.param(13) = 0.3;
% plant_model.param.param(14) = 0.008;
agent.plant = MODEL_CLASS(agent,plant_model);
% agent.plant = MODEL_CLASS(agent,Model_EulerAngle(dt, initial_state, 1));
% agent.plant = MODEL_CLASS(agent,Model_Quat13(dt, initial_state, 1)); % Model_Quat13
%===================================================================================================================================================
agent.sensor.set_function_class("direct", DIRECT_SENSOR(agent, 0.0,struct("output_list",["p","q"])));
% agent.sensor = DIRECT_SENSOR(agent, 0.0); % modeファイル内で回すとき
% agent.estimator.set_function_class("ekf", EKF(agent, Estimator_EKF(agent,dt,MODEL_CLASS(agent,Model_EulerAngle(dt, initial_state, 1)))));

agent.estimator.set_function_class("ekf", EKF(agent, Estimator_EKF(agent,dt,MODEL_CLASS(agent,Model_EulerAngle(dt, initial_state, 1)),["p", "q"])));


% agent.reference.time_varying = TIME_VARYING_REFERENCE(agent,{"gen_ref_saddle",{"freq",10,"orig",[0;0;1.0],"size",[1,1,0.3]},"HL"});
% agent.reference.time_varying = TIME_VARYING_REFERENCE(agent,{"gen_ref_circle",{"freq",5,"orig",[0;0;1],"radius",1.0},"HL"});
% agent.reference.time_varying = MY_POINT_REFERENCE(agent, {struct("f", [0;0;1], "g", [1;1;1], "h",[0;0;1]), 15}); % P2P
% agent.reference.time_varying = TIME_VARYING_REFERENCE(agent,{"gen_ref_lemniscate",{"freq",10,"orig",[0;0;1],"radius",1.0},"HL"});

% agent.reference.set_function_class("time_varying", TIME_VARYING_REFERENCE(agent,{"gen_ref_circle",{"freq",10,"center",[0;0;1],"radius",1.0},4}));

agent.reference.set_function_class("time_varying", TIME_VARYING_REFERENCE(agent,{"gen_ref_lemniscate",{"freq",10,"orig",[0;0;1],"radius",1.0,"phase",0.0,"x",1},4}));%最後は微分回数(ドローンだけの時は4，他は他に合わせる)

agent.reference.set_function_class("takeoff", TAKEOFF_REFERENCE(agent,"zd",1));
agent.reference.set_function_class("landing", LANDING_REFERENCE(agent,"dt",dt,"vd",0));

% agent.reference.time_varying = TIME_VARYING_REFERENCE(agent,{"gen_ref_flower",{"freq",15,"orig",[0;0;1],"radius",1.0},"HL"});
% agent.reference.time_varying = TIME_VARYING_REFERENCE(agent,{"gen_ref_lemniscate_3D",{"freq",10,"orig",[0;0;1],"size",[1,0.5],"phase",[0,0]},"HL"});
% agent.reference.time_varying = TIME_VARYING_REFERENCE(agent,{"gen_ref_star",{"freq",15,"orig",[0;0;1],"radius",1.0},"HL"});
% agent.reference.time_varying = TIME_VARYING_REFERENCE(agent,{"gen_ref_triangle",{"freq",10,"orig",[0;0;1],"size",1.0},"HL"});
% agent.reference.time_varying = TIME_VARYING_REFERENCE(agent,{"gen_ref_heart",{"freq",10,"orig",[0;0;1],"size",1.0},"HL"});



agent.cha_allocation.a.reference="takeoff";
agent.cha_allocation.t.reference="takeoff";
agent.cha_allocation.f.reference = "time_varying";
agent.cha_allocation.l.reference="landing";


agent.controller.set_function_class("nominal", HLC(agent,Controller_HL(dt)));
agent.controller.set_function_class("mec", MECKC(agent,Controller_HL(dt)));

%　agent.controller.mec=MECKC(agent,Controller_HL(dt));
% agent.controller.nominal = FUNCTIONAL_HLC_SERVO(agent,Controller_FHL_Servo(dt));
% agent.controller.mec=MECKC(agent,Controller_FHL_Servo(dt));
agent.cha_allocation.controller=["nominal","mec"];%cha_allocationにコントローラー登録

function dfunc(app)
app.logger.plot({1, "p", "er"},"ax",app.UIAxes,"xrange",[app.time.ts,app.time.te],"fig_num",1);
app.logger.plot({1, "q", "e"},"xrange",[app.time.ts,app.time.te],"fig_num",2);
app.logger.plot({1, "v", "er"},"xrange",[app.time.ts,app.time.te],"fig_num",3);
% app.logger.plot({1, "input1", ""}, "xrange",[app.time.ts,app.time.te],"fig_num",4);%スラスト
% app.logger.plot({1, "input2:4", ""}, "xrange",[app.time.ts,app.time.te],"fig_num",5);
app.logger.plot({1, "controller.result.input", ""}, "xrange",[app.time.ts,app.time.te],"fig_num",6);
app.logger.plot({1, "controller.result.delta_u", ""}, "xrange",[app.time.ts,app.time.te],"fig_num",7);
app.logger.plot({{1, "controller.result.z_p_forward", ""},{1, "controller.result.z_n_forward", "s"}}, "xrange",[app.time.ts,app.time.te], "fig_num", 13);
app.logger.plot({{1, "controller.result.z_p_back", ""},{1, "controller.result.z_n_back", "s"}}, "xrange",[app.time.ts,app.time.te], "fig_num", 14);

% app.logger.plot({1, "p1-p2", "er"},"color", 0,"fig_num",8);
app.logger.plot({1, "p1-p2-p3", "er"},"color",0,"fig_num",9);
% % % 刻み時間描画
%  t0id = find(app.logger.Data.phase==97,1,'last')+1;
% teid = find(app.logger.Data.phase==0,1,'first')-1;
% dt = diff(app.logger.Data.t(t0id:teid));
% t = app.logger.Data.t(t0id:teid-1);
% figure(100)
% ax = gca;
% [t,dt];
% plot(t,dt);
% hold on
% yline(0.025)
% hold off
% grid on
% legend("dt","25 ms")

% % set(ax.XAxis, fontsize=FS-2)
% % set(ax.YAxis, fontsize=FS-2)
% % set(ax.Legend, 'FontSize',FS-4);
end


function v = build_display_vector(agent, time)
% コンソール表示用の文字列を作る。
% 参照位置、推定位置、入力、推定質量を並べる。
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
