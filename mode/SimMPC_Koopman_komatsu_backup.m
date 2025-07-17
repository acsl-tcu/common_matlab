%%
%% Initialize
tmp = matlab.desktop.editor.getActive;
dir = fileparts(tmp.Filename);
if ~contains(path,dir)
    cd(erase(dir,'\mode'));
[~, tmp] = regexp(genpath('.'), '\.\\\.git.*?;', 'match', 'split');
cellfun(@(xx) addpath(xx), tmp, 'UniformOutput', false);
close all hidden; clear ; clc;
userpath('clear');
end

clear gui
%%
clc; close all;
ts = 0; % initial timefghj
dt = 0.025; % sampling period
te = 100; % terminal time
time = TIME(ts,dt,te); % instance of time class
in_prog_func = @(app) dfunc(app); % in progress plot
post_func = @(app) dfunc(app); % function working at the "draw button" pushed.
motive = Connector_Natnet_sim(1, dt, 0); % imitation of Motive camera (motion capture system)
logger = LOGGER(1, size(ts:dt:te, 2), 0, [],[]); % instance of LOOGER class for data logging
initial_state.p = arranged_position([0, 0], 1, 1, 0.6); % [x, y], 機数，1, z (初期位置)
initial_state.q = [0; 0; 0];
initial_state.v = [0; 0; 0];
initial_state.w = [0; 0; 0];

%% クープマンモデルの設定
model_file = "2025-01-12_Exp_Kiyama_code00_saddle_increased.mat";
% model_file = '2024-12-23_Exp_Kiyama_code23_saddle_increased_weight10.mat';
load(model_file,'est'); % main
% [A,B,C] = AB_transfer(est.A, est.B, est.C, dt, 0.08);
A=est.A; B=est.B; C=est.C;
agent = DRONE;

%% 非線形モデルをプラントに設定する場合
agent.plant = MODEL_CLASS(agent,Model_EulerAngle(dt, initial_state, 1));
agent.parameter = DRONE_PARAM("DIATONE");
% agent.parameter.mass = 0.5884;
% agent.parameter.mass = 0.730;
agent.estimator = EKF(agent, Estimator_EKF(agent,dt,MODEL_CLASS(agent,Model_EulerAngle(dt, initial_state, 1)),["p", "q"]));

%% クープマンモデルをプラントに設定する場合
% % model_discrete: クープマンモデルを使用するうえでA,B行列の設定をする、discrete_linear_modelの観測量
% agent.parameter = POINT_MASS_PARAM("rigid","row","A",A,"B",B,"C",C,"D",0,"mass",0.730);
% agent.plant = MODEL_CLASS(agent,Model_Discrete(dt,initial_state,1,"FREE",agent)); 
% agent.estimator = EKF(agent, Estimator_EKF(agent,dt,MODEL_CLASS(agent,Model_EulerAngle(dt, initial_state, 1)),["p", "q"]));

%% controller and reference and sensor (common)
% agent.sensor = MOTIVE(agent, Sensor_Motive(1,0, motive)); % GUIで回すとき
agent.sensor = DIRECT_SENSOR(agent, 0.0); % modeファイル内で回すとき

% agent.reference = TIME_VARYING_REFERENCE(agent,{"gen_ref_saddle",{"freq",5,"orig",[0;0;1],"size",[2,2,0.5]},"HL"});
agent.reference = TIME_VARYING_REFERENCE(agent,{"Case_study_trajectory",{[0,0,0.6]},"HL"});
% agent.reference = MY_POINT_REFERENCE(agent,{struct("f",[1;0;1],"g",[-1.5;0;1],"h",[0;0;1],"j",[-1;0;1]),7});
% agent.reference = MY_REFERENCE_KOMA2(agent,{"",2,te}); % 1:from mat, 2:9-order polynomial

% KMPC & HLC
agent.controller = MPC_CONTROLLER_KOOPMAN_quadprog_experiment_HL(agent,Controller_MPC_Koopman_komatsu(dt, model_file, agent)); %最適化手法：QP

%%
% run("ExpBase");
run("SimBase");

%% modeファイル内でプログラムを回す
phase = 'f'
for i = 1:te/dt
    % if i < 20 || rem(i, 10) == 0 end
    tic
    pre_est = agent.estimator.result;
    agent(1).sensor.do(time, phase);
    agent(1).estimator.do(time, phase);
    agent(1).reference.do(time, phase);
    agent(1).controller.do(time, phase);
    agent(1).plant.do(time, phase);
    logger.logging(time, phase, agent);
    time.t = time.t + time.dt;
    %pause(1)
    toc

    agent.controller.show(agent.controller.result.mpc.fval, agent.controller.result.mpc.exitflag);
    est = agent(1).estimator.result.state.p;
    if est(3) < 0 || est(3) > 2 %終了判定
        break
    end
end
%%
experiment_figure_case_study;
%%
% app.logger = logger;
% result_plot(app, model_file);
% 
% function result_plot(app, model)
%     app.fExp = 0;
%     flg.figtype = 0; % 0:subplot
%     flg.savefig = 0;
%     flg.animation_save = 0;
%     flg.animation = 0;
%     flg.timerange = 0;
%     flg.plotmode = 1; % 1:inner_input, 2:xy, 3:xyz
%     filename = string(datetime('now'), 'yyyy-MM-dd');
%     fig = FIGURE_EXP(app,struct('flg',flg,'phase',1,'filename',filename,'time_idx',[],'yrange',[],'fignum',[2, 3]), struct('model', model));
%     fig.main_figure();
%     % fig.main_animation();
% end

function dfunc(app)
    experiment_figure_case_study;
    % result_plot(app, '');
end