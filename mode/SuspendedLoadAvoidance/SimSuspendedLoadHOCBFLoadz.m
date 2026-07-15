ts = 0; % initial time
dt = 0.025; % sampling period
te = 50; % termina time
time = TIME(ts,dt,te);
in_prog_func = @(app) in_prog(app);
post_func = @(app) post(app);
logger = LOGGER(1, size(ts:dt:te, 2), 0, [],[]); % target, number, fExp, items, agent_items, option
logger.display_func = @(agent, time) build_display_vector(agent, time);
logger.display_on = true;
% fprintf("表示物\nref:[px, py, pz]  est:[px, py, pz]  U:[T, tx, ty, tz]  mL\n\n");
fprintf("表示物\nref:[px, py, pz]  est:[px, py, pz]  Euler:[roll, pitch, yaw]  U:[T, tx, ty, tz]  mL\n\n");

% drone plant setting
agent = DRONE;
agent.parameter = DRONE_PARAM_SUSPENDED_LOAD("DIATONE");
agent.parameter.set("loadmass",0.14);%0.0968);%0.968
agent.parameter.set("cableL",2.0);%0.0968);%0.968
initial_state.q  = [0; 0; 0];
initial_state.w  = [0; 0; 0];
initial_state.vL = [0; 0; 0];
initial_state.p = [0; 0; 0];
initial_state.pT = [0; 0; -1];
initial_state.wL = [0; 0; 0];
initial_state.pL = initial_state.p + initial_state.pT*agent.parameter.cableL;
initial_state.v = [0; 0; 0];
% Note: set the model error after setting "plant"
agent.plant = MODEL_CLASS(agent,Model_Suspended_Load(dt, initial_state,1,agent));%dt,initial,id,agent,modelName
agent.parameter.set("loadmass",0.01);%0.0968);%0.968

% Sim only: getData works after setting "plant"
motive = Connector_Natnet_sim(dt, {{1,"p","q"},{1,"pL","pT"}}); % imitation of Motive camera (motion capture system)
motive.getData(agent);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% drone setting  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
agent.sensor.set_function_class("motive", MOTIVE(agent, motive,"output_func",@motive_output,"rigid_id",[1,2],"state_list",{["p","q"],"p"}));
function y = motive_output(obj,data)
% Motiveの生データから、機体姿勢と吊り荷位置・ケーブル方向を整理する。
% 出力は推定器の入力として [p; euler; pL; pT] の形に整形する。
p = data.rigid(obj.rigid_id(1)).p;
pT = data.rigid(obj.rigid_id(2)).p - p;
pT = pT/norm(pT);
y = [p;Quat2Eul(data.rigid(obj.rigid_id(1)).q);data.rigid(obj.rigid_id(2)).p;pT];
end

agent.estimator.set_function_class("ekf", EKF(agent, Estimator_EKF_SuspendedLoad(agent,dt,...
    MODEL_CLASS(agent,Model_Suspended_Load(dt, initial_state, 1,agent,"Load_mL_HL")),...%"Load_mL_HL"
    ["p", "q","pL","pT"])));%expの流用 質量推定有

agent.estimator.set_function_class("loadstate", SUSPENDED_LOAD_STATE_MANAGER(agent));
L = agent.parameter.cableL;
% agent.reference.set_function_class("timevarying", TIME_VARYING_REFERENCE(agent,{"gen_ref_saddle",{"freq",30,"center",[0;0;2.5],"radius",[15,15,0]},4})); %円系軌道
% agent.reference.set_function_class("timevarying", TIME_VARYING_REFERENCE(agent,{"gen_ref_p2p_back_and_forth",{"p0",[0;0;3.0], "p1",[2;2;3.0], "t_go",3.0, "t_hold",3.0, "t_back",3.0},4})); %P2P
% agent.reference.set_function_class("timevarying", TIME_VARYING_REFERENCE(agent,{"gen_ref_triangle",{"freq",9,"center",[0;0;1.5],"radius",[1,1,0]},4})); % triangle
agent.reference.set_function_class("timevarying", TIME_VARYING_REFERENCE(agent, {"gen_ref_p2p_line", {"p0", [0;0;3.0], "p1", [0;15.0;3.0], "t_go", 15.0}, 4}));
agent.reference.set_function_class("sload", SUSPENDED_LOAD_REF_ADJUST(agent));
agent.reference.set_function_class("takeoff", TAKEOFF_REFERENCE(agent,"zd",3.0,"te",5));
agent.reference.set_function_class("landing", LANDING_REFERENCE(agent,"dt",dt,"zd",-L,"te",3)); % zd = -Lとするのがミソ：l移行時のrefは牽引物用なので

% try
%     % 配列や空の状態に左右されず、xdが存在する実行ステップのみをピンポイントで狙い撃ち
%     if isfield(agent.reference.result, 'state') && isprop(agent.reference.result.state, 'xd') && ~isempty(agent.reference.result.state.xd)
%         agent.reference.result.state.xd = cbf_reference_filter(agent, agent.reference.result.state.xd);
%     end
% catch
%     % 初期化時の startupFcn での空アクセスは完全に無害化してスルー
% end
% agent.controller.set_function_class("hlc_suspended", HLC_SUSPENDED_LOAD(agent,Controller_HL_Suspended_Load(dt,agent)));
agent.controller.set_function_class("hlc_suspended", HLC_SUSPENDED_LOAD_HOCBF_LOAD_Z(agent, Controller_HL_Suspended_Load(dt, agent)));
% % =================================================================
% % 🔍 [SimScript Debug] コントローラの登録状態を完全可視化
% % =================================================================
% agent.controller.set_function_class("hlc_suspended", HLC_SUSPENDED_LOAD_AVOIDANCE(agent, Controller_HL_Suspended_Load(dt, agent)));
% 
% disp('=== 🚀 CONTROLLER REGISTRATION CHECK ===');
% disp(['Registered class name: ', class(agent.controller.hlc_suspended)]);
% if isprop(agent.controller.hlc_suspended, 'self')
%     disp('  -> "self" property exists.');
% end
% disp('========================================');
% % =================================================================
agent.set_cha_allocation_for_all("sensor","motive");
agent.set_cha_allocation_for_all("estimator",["ekf","loadstate"]);
agent.cha_allocation.a.reference =["takeoff","sload"]; % aも忘れずにセットする
agent.cha_allocation.t.reference =["takeoff","sload"];
agent.cha_allocation.f.reference =["timevarying","sload"];
agent.cha_allocation.l.reference =["landing","sload"];

%%

function post(app)
% シミュレーション終了後の結果表示とアニメーション作成。
app.logger.plot({{1, "p", "er"},{1, "estimator.result.state.pL", "e"}},"ax",app.UIAxes,"phase","tfl");
app.logger.plot({1, "state.mL", "e"},"phase","tfl");
app.logger.plot({1, "controller.result.controllertime", ""}, "phase", "f", "fig_num", 2);
%app.logger.plot({1, "estimator.result.ekf_mL", ""},"phase","tfl", "fig_num",2);
% app.logger.plot({1, "p", "er"},"phase","tf", "fig_num",1); % 位置: p_x,p_y,p_z
% app.logger.plot({1, "q", "e"}, "phase","tfl", "fig_num",2 ); % 角度: θ_roll, θ_pitch, θ_yaw
% app.logger.plot({{1, "p", "er"},{1, "estimator.result.state.pL", "e"}},"phase","f","fig_num",2);
% app.logger.plot({{1, "v1:2", "er"},{1,"estimator.result.state.vL1:2",""}},"fig_num",3);% 速度: v_x, v_y, v_z
% app.logger.plot({1, "w", "e"}, "phase","tf", "fig_num",4); % 角速度: ω_roll, ω_ptich, ω_yaw
% app.logger.plot({1, "input", ""}, "phase","f", "fig_num",5); % 制御入力: Thrust, roll, pitch, yaw
% app.logger.plot({{1, "reference.result.state.xd1:3", ""},{1,"reference.result.state.xd5:7",""}},"phase","f","fig_num",4);
% app.logger.plot({{1, "reference.result.state.xd9:11", ""},{1,"reference.result.state.xd13:15",""}},"phase","f","fig_num",5);

show_suspended_load_animation(app); % アニメーション描画
end

% function show_suspended_load_animation(app)
% % 単機の吊り下げモデルをアニメーション表示する。
% if app.logger.k <= 1
%     return
% end
% 
% mov = DRAW_SUSPENDED_LOAD(app.logger, ...
%     "target", 1, ...
%     "self", app.agent(1));
% mov.animation(app.logger,"target", 1,"self", app.agent(1));%表示だけ用
% 
% % mov.animation(app.logger,"target", 1,"self", app.agent(1),"mp4", true, "pause", 0);%mp4保存用
% 
% % mov.animation(app.logger, "target", 1,"self", app.agent(1), "gif", "Data/suspended_load.gif", ...
% %     "fps", 20,"gif_delay", 0.05,"skip", 2, "pause", 0);%gif保存用
% end
function show_suspended_load_animation(app)
% 単機の吊り下げモデルを障害物回避マージン付きでアニメーション表示する。
if app.logger.k <= 1
    return
end

actual_rl = app.agent(1).controller.result.rl;
% 新しい回避対応版クラスをインスタンス化
mov = DRAW_SUSPENDED_LOAD_HOCBF_Z(app.logger, ...
    "target", 1, ...
    "self", app.agent(1), ...
    "rl", actual_rl);
mov.animation(app.logger,"target", 1,"self", app.agent(1), "rl", actual_rl); % 表示開始
% mov.animation(app.logger,"target", 1,"self", app.agent(1), "rl", actual_rl,"mp4", true, "pause", 0); % 表示開始
end

function in_prog(app)
% 実行中に推定状態をUIに表示する。
app.TextArea.Text = "estimator : " + app.agent.estimator.result.state.get();
end



% function v = build_display_vector(agent, time)
% % コンソール表示用の文字列を作る。
% % 参照位置、推定位置、入力、推定質量を並べる。
% idx = 1;
% if ~isprop(agent(idx).reference.result.state, "xd")
%     v = [];
%     return
% end
% xd = agent(idx).reference.result.state.xd;
% if isfield(agent(idx).estimator.result, "state")
%     p = agent(idx).estimator.result.state.p;
%     if isprop(agent(idx).estimator.result.state, "mL")
%         mL = agent(idx).estimator.result.state.mL;
%     else
%         mL = NaN;
%     end
% else
%     p = [NaN; NaN; NaN];
%     mL = NaN;
% end
% u = agent(idx).controller.result.input;
% v = sprintf("%c %.3f : R [%7.3f,%7.3f,%7.3f] : P [%7.3f,%7.3f,%7.3f] : U [%7.3f,%7.3f,%7.3f,%7.3f] : mL %7.3f",agent(idx).cha, time.t, xd(1:3)', p', u', mL);
% end
function v = build_display_vector(agent, time)
% コンソール表示用の文字列を作る。
% 参照位置、推定位置、機体姿勢(新規追加)、入力、推定質量を並べる。
idx = 1;
if ~isprop(agent(idx).reference.result.state, "xd")
    v = [];
    return
end
xd = agent(idx).reference.result.state.xd;

% 推定状態の取得
if isfield(agent(idx).estimator.result, "state")
    p = agent(idx).estimator.result.state.p;
    
    % --- 機体姿勢(オイラー角)の抽出ロジック ---
    if isprop(agent(idx).estimator.result.state, "q")
        q_state = agent(idx).estimator.result.state.q;
        % クオータニオン（4成分）か Euler（3成分）かを自動判別して[Roll; Pitch; Yaw]に変換
        if numel(q_state) == 4
            euler = Quat2Eul(q_state); 
        else
            euler = q_state; % 既に3成分（オイラー角）として保持されている場合
        end
    else
        euler = [NaN; NaN; NaN];
    end
    
    % 質量推定の抽出
    if isprop(agent(idx).estimator.result.state, "mL")
        mL = agent(idx).estimator.result.state.mL;
    else
        mL = NaN;
    end
else
    p = [NaN; NaN; NaN];
    euler = [NaN; NaN; NaN];
    mL = NaN;
end

u = agent(idx).controller.result.input;

% コンソール表示フォーマットの生成
% R:目標位置, P:現在の推定位置, E:現在の機体姿勢(Roll, Pitch, Yaw), U:制御入力, mL:推定質量
v = sprintf("%c %.3f : R [%7.3f,%7.3f,%7.3f] : P [%7.3f,%7.3f,%7.3f] : E [%7.3f,%7.3f,%7.3f] : U [%7.3f,%7.3f,%7.3f,%7.3f] : mL %7.3f", ...
    agent(idx).cha, time.t, xd(1:3)', p', euler', u', mL);
end
