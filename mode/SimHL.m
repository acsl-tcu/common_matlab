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
motive = Connector_Natnet_sim(dt); % imitation of Motive camera (motion capture system)
logger = LOGGER(1, size(ts:dt:te, 2), 0, [],[]); % instance of LOOGER class for data logging
initial_state.p = arranged_position([0, 0], 1, 1, 0);
initial_state.q = [1; 0; 0; 0];
initial_state.v = [0; 0; 0];
initial_state.w = [0; 0; 0];


agent = DRONE;
agent.parameter = DRONE_PARAM("DIATONE");
% プラントモデル定義 ================================================================================================================================
plant_model = Model_EulerAngle(dt, initial_state, 1);
% デフォルト物理パラメータ(DRONE_PARAM準拠: 2025/07/07時点)
% 1:mass=0.75  |  2,3:Lx,y=0.16  |  4,5: lx,y=0.08  |  6,7,8: jx,y,z=0.06  |  9: gravity=9.81
% 10,11,12,13: km(各ロータ定数)=0.0301  |  14,15,16,17: k(推力定数)=8.0e-6  |  18: rotor_r=0.0392

% ↓パラメータの上書き モデル誤差をプラントに与える
% plant_model.param.param(1) = 0.7875; % ５％減->0.7125 ５％増->0.7875
plant_model.param.param(1) = 0.4; % ５％減->0.7125 ５％増->0.7875
% plant_model.param.param(6) = 0.2; % 0.18<jx,jy<0.22ぐらいが良き
% plant_model.param.param(7) = 0.2; % 
% plant_model.param.param(8) = 0.6; % 0.18 < jzぐらいが良き
% plant_model.param.param(10) = 0.6; % ５％減->0.028595
% plant_model.param.param(13) = 0.3;
% plant_model.param.param(14) = 0.008;
agent.plant = MODEL_CLASS(agent,plant_model);
% agent.plant = MODEL_CLASS(agent,Model_Quat13(dt, initial_state, 1)); % Model_Quat13
%===================================================================================================================================================
agent.estimator = EKF(agent, Estimator_EKF(agent,dt,MODEL_CLASS(agent,Model_EulerAngle(dt, initial_state, 1)),["p", "q"]));
agent.sensor = MOTIVE(agent, Sensor_Motive(1,0, motive));
% agent.reference.time_varying = TIME_VARYING_REFERENCE(agent,{"gen_ref_circle",{"freq",10,"init",[0;0;1],"radius",1.0},"HL"});
agent.reference.time_varying = TIME_VARYING_REFERENCE(agent,{"gen_ref_spline",{"point",15,"order",9,"point_dt",5,"check",1,"ManualSetting",0}});%引数としてHLをいれると軌道が微分される
agent.controller.hl = HLC(agent,Controller_HL(dt));
agent.controller.delta = HLC_delta_u(agent,Controller_HL(dt));
%run("ExpBase");
agent.reference.takeoff = TAKEOFF_REFERENCE(agent,[]);
agent.reference.landing = LANDING_REFERENCE(agent,dt,0.1);
agent.cha_allocation = struct("reference","time_varying", ...
    "a",struct("reference","takeoff"), "t",struct("reference","takeoff"),"l",struct("reference","landing"));
motive.getData(agent);
agent.cha_allocation.controller = "hl";
agent.cha_allocation.f.controller = "delta";

function dfunc(app)
app.logger.plot({1, "p", "er"},"ax",app.UIAxes);
% app.logger.plot({1, "p", "per"},"xrange",[app.time.ts,app.time.te]);
app.logger.plot({1, "q", "e"},"fig_num",3);
app.logger.plot({1, "v", "er"},"fig_num",4);
% app.logger.plot({1, "input", ""},"fig_num",5);
app.logger.plot({1, "p1-p2-p3", "er"},"color",0,"fig_num",6);
app.logger.plot({1, "controller.result.nominal", ""}, "fig_num",7);
app.logger.plot({1, "controller.result.delta_u", ""}, "fig_num",8);

end