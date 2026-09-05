tmp = matlab.desktop.editor.getActive;
dir = fileparts(tmp.Filename);
if ~contains(path,dir)
    root_dir = fileparts(fileparts(dir));
    cd(root_dir);
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

initial_state.p = arranged_position([0, 0], 1, 1, 0);
initial_state.q = [1; 0; 0; 0];
initial_state.v = [0; 0; 0];
initial_state.w = [0; 0; 0];


agent = DRONE;
agent.parameter = DRONE_PARAM("DIATONE");
% agent.parameter = DRONE_PARAM("DIATONE", "mass", 0.7);
agent.plant = MODEL_CLASS(agent,Model_Quat13(dt, initial_state, 1));
%agent.parameter.set("mass",struct("mass",0.5))

agent.sensor.set_function_class("motive", MOTIVE(agent,motive));

agent.estimator.set_function_class("ekf", EKF(agent, Estimator_EKF(agent,dt,MODEL_CLASS(agent,Model_EulerAngle(dt, initial_state, 1)))));

% agent.reference.set_function_class("time_varying", TIME_VARYING_REFERENCE(agent,{"gen_ref_circle",{"freq",10,"center",[0;0;1],"radius",1.0},4}));
% agent.reference.set_function_class("time_varying", TIME_VARYING_REFERENCE(agent, {"gen_ref_p2p_line", {"p0", [0;0;1.0], "p1", [0;15.0;1.0], "t_go", 15.0}, 4}));
agent.reference.set_function_class("time_varying", TIME_VARYING_REFERENCE(agent, {"gen_ref_p2p_line", {"p0", [0;0;1.0], "p1", [0;0.0;18.0], "t_go", 18.0}, 4}));
agent.reference.set_function_class("takeoff", TAKEOFF_REFERENCE(agent,"zd",1));
agent.reference.set_function_class("landing", LANDING_REFERENCE(agent,"dt",dt,"vd",0));

% agent.controller.set_function_class("hlc", HLC_CBF(agent,Controller_HL(dt)));
% agent.controller.set_function_class("hlc", HLC_CBF_DynamicExtension(agent,Controller_HL(dt)));
agent.controller.set_function_class("hlc", HLC_CBF_RealDynExt(agent,Controller_HL(dt)));

agent.cha_allocation.f.reference = "time_varying";
agent.cha_allocation.a.reference="takeoff";
agent.cha_allocation.t.reference="takeoff";
agent.cha_allocation.l.reference="landing";
motive.getData(agent);
%% utility functions
function post(app)
app.logger.plot({1, "p", "er"},"ax",app.UIAxes,"phase","tfl");
try
    app.logger.plot({1, "controller.result.min_clearance", ""}, ...
        "phase", "f", ...
        "fig_num", 3);
catch
    disp('⚠️ 距離データ controller.result.min_clearance が見つかりませんでした。');
end
% app.logger.plot({1, "p", "per"},"xrange",[app.time.ts,app.time.te]);
% app.logger.plot({1, "q", "s"},"ax",app.UIAxes2,"xrange",[app.time.ts,app.time.te]);
% app.logger.plot({1, "v", "er"},"ax",app.UIAxes3,"xrange",[app.time.ts,app.time.te]);
%app.logger.plot({1, "input", ""},"ax",app.UIAxes4,"xrange",[app.time.ts,app.time.t]);
show_animation(app);
end
function dfunc(app)
app.logger.plot({1, "p", "er"},"ax",app.UIAxes,"phase","tfl");
% 2. 衝突確認用クリアランス（距離）プロット
% app.logger.plot({1, "p", "per"},"xrange",[app.time.ts,app.time.te]);
% app.logger.plot({1, "q", "s"},"ax",app.UIAxes2,"xrange",[app.time.ts,app.time.te]);
% app.logger.plot({1, "v", "er"},"ax",app.UIAxes3,"xrange",[app.time.ts,app.time.te]);
%app.logger.plot({1, "input", ""},"ax",app.UIAxes4,"xrange",[app.time.ts,app.time.t]);
end
function show_animation(app)
% 協調吊り下げ（複数ドローン＋牽引物）のアニメーションを生成する。
% Nは機体数、牽引物はN+1
if app.logger.k <= 1
    return
end
  % mov = DRAW_DRONE_MOTION_CBF(app.logger, "self", app.agent, "target", 1,...
  %         "lims", [ -5 5;  -5 15;  -3 5 ]);
  mov = DRAW_DRONE_MOTION_CBF(app.logger, "self", app.agent, "target", 1,...
      "lims", [ -5 5;  -5 15;  -3 5 ]);
  mov.animation(app.logger, "self", app.agent, "target", 1, "Motive_ref", 1);
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
