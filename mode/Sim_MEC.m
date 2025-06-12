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
    te = 25; % terminal time
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
    agent.reference.time_varying = TIME_VARYING_REFERENCE(agent,{"gen_ref_saddle",{"freq",5,"orig",[0;0;1],"size",[1,1,0.5]},"HL"});
    % agent.reference = TIME_VARYING_REFERENCE(agent,{"gen_ref_p2p",{"freq",5,"init",[0;0;1],"radius",1.0},"HL"});
    
    agent.cha_allocation.controller=["nominal","mec"];
    agent.controller.nominal=FUNCTIONAL_HLC(agent,Controller_FHL(dt));%mainGUIに合わせるならこの3行必要
    agent.controller.mec=FUNCTIONAL_MECKC(agent,Controller_FHLMECK(dt));
    

    Pa_estimator.state = initial_state;
    % run("ExpBase");
agent.reference.takeoff = TAKEOFF_REFERENCE(agent,[]);
agent.reference.landing = LANDING_REFERENCE(agent,dt,0.1);
agent.cha_allocation = struct("reference",["time_varying"], ...
    "t",struct("reference",["takeoff"]),"l",struct("reference","landing"));
motive.getData(agent);

function dfunc(app)
app.logger.plot({1, "p", "per"},"ax",app.UIAxes,"xrange",[app.time.ts,app.time.te]);
app.logger.plot({1, "q", "s"},"ax",app.UIAxes2,"xrange",[app.time.ts,app.time.te]);
app.logger.plot({1, "v", "er"},"ax",app.UIAxes3,"xrange",[app.time.ts,app.time.te]);
app.logger.plot({1, "input", ""},"ax",app.UIAxes4,"xrange",[app.time.ts,app.time.t]);
end