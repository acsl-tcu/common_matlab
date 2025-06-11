%% Initialize
tmp = matlab.desktop.editor.getActive;
dir = fileparts(tmp.Filename);
if ~contains(path,dir)
    cd(erase(dir,'\mode'));
[~, tmp] = regexp(genpath('.'), '\.\\\.git.*?;', 'match', 'split');
cellfun(@(xx) addpath(xx), tmp, 'UniformOutput', false);
close all hidden; 
clear;
clc;
userpath('clear');
end
%% 20回まとめてシミュレーションする
clear; close all; clc;
    % fprintf('Initializing... N:%d \n', j);
    clear data
    ts = 0; % initial time
    dt = 0.025; % sampling period
    te = 10; % terminal time
    time = TIME(ts,dt,te); % instance of time class
    in_prog_func = @(app) dfunc(app); % in progress plot
    post_func = @(app) dfunc(app); % function working at the "draw button" pushed.
    motive = Connector_Natnet_sim(1, dt); % imitation of Motive camera (motion capture system)
    logger = LOGGER(1, size(ts:dt:te, 2), 0, [],[]); % instance of LOOGER class for data logging
    % logger2 = LOGGER(1, size(ts:dt:te, 2), 0, [],[]); % instance of LOOGER class for data logging
    initial_state.p = arranged_position([0, 0], 1, 1, 0);
    initial_state.q = [1; 0; 0; 0];
    initial_state.v = [0; 0; 0];
    initial_state.w = [0; 0; 0];

    %----------------------------
    % agent(1) = ノミナルモデル → 通常のSimHLでの定義と同じ
    % agent(2) = プラントモデル → あえてモデル誤差を与えたモデル
    %----------------------------
    
    agent = DRONE;
    
    % agent.parameter.nominal = DRONE_PARAM("DIATONE");
    agent.parameter = DRONE_PARAM("DIATONE","mass",0.4); % プラントモデルにモデル誤差を与える．DRONE_PARAMのパラメータを上書きしている．
    agent.plant = MODEL_CLASS(agent,Model_EulerAngle(dt, initial_state, 1)); % Model_Quat13
    
    agent.estimator = EKF(agent, Estimator_EKF(agent,dt,MODEL_CLASS(agent,Model_EulerAngle(dt, initial_state, 1)),["p", "q"]));
    
    agent.sensor = DIRECT_SENSOR(agent, 0.0); % modeファイル内で回すとき

    % agent.reference = TIME_VARYING_REFERENCE(agent,{"gen_ref_circle",{"freq",5,"init",[0;0;1],"radius",1.0},"HL"});
    agent.reference = TIME_VARYING_REFERENCE(agent,{"gen_ref_saddle",{"freq",5,"orig",[0;0;1],"size",[1,1,0.5]},"HL"});
    % agent.reference = TIME_VARYING_REFERENCE(agent,{"gen_ref_p2p",{"freq",5,"init",[0;0;1],"radius",1.0},"HL"});
    
    % agent.controller.nominal=FUNCTIONAL_HLC(agent,Controller_FHL(dt));
    % agent.controller.mec=FUNCTIONAL_MECKC(agent,Controller_FHLMECK(dt));
    % agent.cha_allocation.controller=["nominal","mec"];

    agent.controller=FUNCTIONAL_MEC(FUNCTIONAL_HLC(agent,Controller_FHL(dt)),FUNCTIONAL_MECKC(agent,Controller_FHLMECK(dt)));
 
    Pa_estimator.state = initial_state;
    run("ExpBase");

    for i = 1:te/dt
    if i < 20 || rem(i, 10) == 0 end
        tic

        agent.sensor.do(time,'f');
        agent.estimator.do(time,'f');
        agent.reference.do(time,'f');
        agent.controller.do(time,'f');
        agent.input_transform.do(time,'f');
        agent.plant.do(time,'f');

     
        % agent.controller.mec.Pa_p_pre = Pa_estimator.state.p;    % 1. 現時刻のプラントの推定位置を「mecのコントローラ」内に格納
        % agent.plant.do(time, 'f');%  xn[k] % 2.状態更新
        % agent.controller.mec.Pn_p_cur = [agent.plant.state.p;agent.plant.state.q;agent.plant.state.v;agent.plant.state.w];
        % agent.sensor.do(time, 'f'); % 2 hxa[k] % 4.センサ情報取得
        % Pa_estimator = agent(2).estimator.do(time, 'f'); % 5. プラントの推定器を回し，情報を取得
        % Pa_estimator = agent.estimator.do(time, 'f'); % 5.推定器を回し，情報を取得
        % agent.controller.mec.Pa_p_cur = [Pa_estimator.state.p;Pa_estimator.state.q;Pa_estimator.state.v;Pa_estimator.state.w];
        % agent.reference.do(time, 'f'); % 7. プラントのリファレンス更新
        % Pn_controller = agent.controller.nominal.do(time, 'f'); % 10. ノミナルのコントローラを回し，情報を取得
        % agent.controller.mec.Pn_u = Pn_controller.input; % 11. ノミナルのコントローラから得られた制御入力を，プラントの制御入力にコピー
        % agent.controller.do(time, 'f');% 4, 5 du[k], u[k] % 12. プラントのコントローラ計算
        % agent.plant.do(time, 'f');% 6 xa[k+1] % 13. プラントの状態更新
        logger.logging(time, 'f', agent);
        time.t = time.t + time.dt;
        all = toc;
        disp([num2str(time.t)])
    end
    
    save("Data\test","logger")



%%
set(0,'defaultAxesFontSize', 10);
set(0, 'DefaultLineLineWidth', 1.5);
logger.plot({1, "p", "er"}, {1, "input", ""},"xrange",[time.ts,time.t],"fig_num",1,"row_col",[1 2]);
% logger.save('HL_sim_test_1008_sigmoid');
app.logger = logger;
result_plot(app)

function result_plot(app)
    app.fExp = 1;
    flg.figtype = 1; % 0:subplot
    flg.savefig = 0;
    flg.animation_save = 0;
    flg.animation = 0;
    flg.timerange = 0;
    flg.plotmode = 3; % 1:inner_input, 2:xy, 3:xyz
    filename = string(datetime('now'), 'yyyy-MM-dd');
    fig = FIGURE_EXP(app,struct('flg',flg,'phase',1,'filename',filename,'time_idx',[],'yrange',[],'fignum',[3, 3]), struct('model', ""));
    fig.main_figure();
end

function in_prog(app)
p = round(app.agent.estimator.result.state.get("p"), 3);
pr = round(app.agent.estimator.result.state.get("p"),3);
input = round(app.agent.controller.result.input,3);
% app.Label_2.Text = ["estimator p : " + app.agent(1).estimator.result.state.get("p")];
app.Label_2.Text = ["estimator: x="+p(1)+", y="+p(2)+", z="+p(3)];
app.Label_3.Text = ["reference: x="+pr(1)+", y="+pr(2)+", z="+pr(3)];
app.Label_4.Text = ["input: "+input(1)+", "+input(2)+", "+input(3)+", "+input(4)];
end