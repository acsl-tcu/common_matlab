ts = 0;    % initial time
dt = 0.025; % sampling period
te = 50;   % terminal time
time = TIME(ts,dt,te);
in_prog_func = @(app) in_prog(app);
post_func = @(app) post(app);
logger = LOGGER(1, size(ts:dt:te, 2), 0, [],[]); % target, number, fExp, items, agent_items, option
logger.display_func = @(agent, time) build_display_vector(agent, time);
logger.display_on = true;
fprintf("表示物\nref:[px,py,pz]  est:[px,py,pz]  T:%5.2f  F:[f1,f2,f3,f4]  mL\n\n");

% drone plant setting
agent = DRONE;
agent.parameter = DRONE_PARAM_SUSPENDED_LOAD("DIATONE");
agent.parameter.set("loadmass",0.14);
agent.parameter.set("cableL",2.0);
initial_state.q  = [0; 0; 0];
initial_state.w  = [0; 0; 0];
initial_state.vL = [0; 0; 0];
initial_state.p = [0; 0; 0];
initial_state.pT = [0; 0; -1];
initial_state.wL = [0; 0; 0];
initial_state.pL = initial_state.p + initial_state.pT*agent.parameter.cableL;
initial_state.v = [0; 0; 0];
% Note: set the model error after setting "plant"
agent.plant = MODEL_CLASS(agent, Model_SuspendedLoad_RotorLevel(dt, initial_state, 1, agent));
agent.parameter.set("loadmass",0.01);

% ロータ推力の変換・飽和クリッピングは SuspendedLoad_RotorLevel_Dynamics 内部で行う。
% input_transform は不要 (EKF との互換性を保つため [T;tau] のまま渡す)


% Sim only: getData works after setting "plant"
motive = Connector_Natnet_sim(dt, {{1,"p","q"},{1,"pL","pT"}});
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
    MODEL_CLASS(agent, Model_SuspendedLoad_RotorLevel(dt, initial_state, 1, agent)),...
    ["p", "q","pL","pT"])));

agent.estimator.set_function_class("loadstate", SUSPENDED_LOAD_STATE_MANAGER(agent));
L = agent.parameter.cableL;
agent.reference.set_function_class("timevarying", TIME_VARYING_REFERENCE(agent,{"gen_ref_saddle",{"freq",5,"center",[0;0;1.5],"radius",[1,1,0]},6}));
agent.reference.set_function_class("sload", SUSPENDED_LOAD_REF_ADJUST(agent));
agent.reference.set_function_class("takeoff", TAKEOFF_REFERENCE(agent,"zd",3.0,"te",5));
agent.reference.set_function_class("landing", LANDING_REFERENCE(agent,"dt",dt,"zd",-L,"te",3));

agent.controller.set_function_class("hlc_suspended", HLC_SUSPENDED_LOAD(agent,Controller_HL_Suspended_Load(dt,agent)));

agent.set_cha_allocation_for_all("sensor","motive");
agent.set_cha_allocation_for_all("estimator",["ekf","loadstate"]);
agent.cha_allocation.a.reference =["takeoff","sload"];
agent.cha_allocation.t.reference =["takeoff","sload"];
agent.cha_allocation.f.reference =["timevarying","sload"];
agent.cha_allocation.l.reference =["landing","sload"];

%%

function post(app)
% シミュレーション終了後の結果表示とアニメーション作成。
app.logger.plot({{1, "p", "er"},{1, "estimator.result.state.pL", "e"}},"ax",app.UIAxes,"phase","tfl");
app.logger.plot({{1, "p", "er"},{1, "estimator.result.state.pL", "e"}},"phase","f","fig_num",2);
app.logger.plot({{1, "v1:2", "er"},{1,"estimator.result.state.vL1:2",""}},"fig_num",3);

% figure 20: ロータ推力の手動プロット (inv(B) * controller.result.input から計算)
try
    figure(20); clf;
    k_end = app.logger.k - 1;
    if k_end > 1
        t_vec = app.logger.Data.t(1:k_end);

        % アロケーション行列の構築
        P_param = app.agent(1).parameter.get(["Lx","Ly","lx","ly","km1","km2","km3","km4"]);
        Lx_p=P_param(1); Ly_p=P_param(2); lx_p=P_param(3); ly_p=P_param(4);
        km1_p=P_param(5); km2_p=P_param(6); km3_p=P_param(7); km4_p=P_param(8);
        B_p = [1,     1,          1,        1;
              -ly_p, -ly_p,      (Ly_p-ly_p), (Ly_p-ly_p);
               lx_p, -(Lx_p-lx_p), lx_p,   -(Lx_p-lx_p);
               km1_p,-km2_p,     -km3_p,    km4_p];
        IIT_p = inv(B_p);

        % ロータ推力時系列を計算
        f_log = zeros(4, k_end);
        if iscell(app.logger.Data.agent)
            agent_log = app.logger.Data.agent{1};
        else
            agent_log = app.logger.Data.agent(1);
        end
        for kk = 1:k_end
            try
                if iscell(agent_log.controller.result)
                    u_k = agent_log.controller.result{kk}.input(1:4);
                else
                    u_k = agent_log.controller.result(kk).input(1:4);
                end
                f_log(:,kk) = max(0, min(5.0, IIT_p * u_k));
            catch
                f_log(:,kk) = NaN;
            end
        end

        labels = {"f1 [N]","f2 [N]","f3 [N]","f4 [N]"};
        colors = {[0.8 0.1 0.1],[0.1 0.6 0.1],[0.1 0.1 0.8],[0.8 0.5 0.0]};
        for ri = 1:4
            subplot(4,1,ri);
            plot(t_vec, f_log(ri,:), 'Color', colors{ri}, 'LineWidth', 1.2); hold on;
            yline(5.0, '--k', 'LineWidth', 0.8);  % 飽和ライン
            ylabel(labels{ri}); grid on; ylim([-0.2 5.5]);
        end
        xlabel("time [s]"); sgtitle("Rotor Thrusts [N]  (-- = saturation limit)");
    end
catch ME
    warning("ロータ推力プロット失敗: %s", ME.message);
end


show_suspended_load_animation(app);
end

function show_suspended_load_animation(app)
% 単機の吊り下げモデルをアニメーション表示する。
if app.logger.k <= 1
    return
end
mov = DRAW_SUSPENDED_LOAD(app.logger, ...
    "target", 1, ...
    "self", app.agent(1));
mov.animation(app.logger,"target", 1,"self", app.agent(1));
end

function in_prog(app)
% 実行中に推定状態をUIに表示する。
app.TextArea.Text = "estimator : " + app.agent.estimator.result.state.get();
end

function v = build_display_vector(agent, time)
% コンソール表示用の文字列を作る。
% 参照位置、推定位置、合力T、ロータ推力[f1..f4]、推定質量を並べる。
idx = 1;
if ~isprop(agent(idx).reference.result.state, "xd")
    v = [];
    return
end
xd = agent(idx).reference.result.state.xd;
if isfield(agent(idx).estimator.result, "state")
    p = agent(idx).estimator.result.state.p;
    if isprop(agent(idx).estimator.result.state, "mL")
        mL = agent(idx).estimator.result.state.mL;
    else
        mL = NaN;
    end
else
    p = [NaN; NaN; NaN];
    mL = NaN;
end
T_val = agent(idx).controller.result.input(1);

% ロータ推力を inv(B) * [T;tau] で計算（表示用・クリッピング前の要求値）
try
    P = agent(idx).parameter.get(["Lx","Ly","lx","ly","km1","km2","km3","km4"]);
    Lx=P(1); Ly=P(2); lx_=P(3); ly_=P(4);
    km1_=P(5); km2_=P(6); km3_=P(7); km4_=P(8);
    B_mat = [1,    1,         1,       1;
            -ly_,  -ly_,     (Ly-ly_), (Ly-ly_);
             lx_,  -(Lx-lx_), lx_,    -(Lx-lx_);
             km1_, -km2_,    -km3_,    km4_];
    u_ctrl = agent(idx).controller.result.input(1:4);
    f_rotor = inv(B_mat) * u_ctrl;  % 要求ロータ推力（クリップ前）
    f_sat   = max(0, min(5.0, f_rotor));  % 飽和後（実際に適用される値）
catch
    f_rotor = [NaN;NaN;NaN;NaN];
    f_sat   = f_rotor;
end

% 飽和しているロータは '*' で強調表示
sat_flag = abs(f_rotor - f_sat) > 0.01;
sat_str = arrayfun(@(f,s) sprintf("%4.2f%s", f, repmat("*",1,s)), f_sat, double(sat_flag), 'UniformOutput', false);
v = sprintf("%c %.3f : P [%5.2f,%5.2f,%5.2f] : T %5.2f : F [%s,%s,%s,%s] : mL %.3f", ...
    agent(idx).cha, time.t, p(1), p(2), p(3), T_val, ...
    sat_str{1}, sat_str{2}, sat_str{3}, sat_str{4}, mL);
end

%%
% ローカルファクトリ関数: SuspendedLoad_RotorLevel_Dynamics を使う
% プラントモデルを構築する。Model_Suspended_Load.m の構造に準拠。
function Model = Model_SuspendedLoad_RotorLevel(dt, initial, id, agent)
% Model_SuspendedLoad_RotorLevel  ロータレベル入力対応の吊り荷プラントモデル設定
%   SuspendedLoad_RotorLevel_Dynamics を動力学実体として使う。
%   入力次元 = 4 (各ロータ推力 [N])。状態次元・パラメータ次元は
%   with_load_model_mL_euler_for_HL.m と同じ (25 states, 4 inputs, 21 params)。
arguments
    dt
    initial
    id
    agent = ""
end
Model.id                = id;
Model.name              = "load_rotor_level";
Model.type              = "Suspended_Load_RotorLevel_Model";

% 状態方程式の実体: 直接関数名で指定（get_model_name には未登録のため）
Setting.method          = "SuspendedLoad_RotorLevel_Dynamics";
Setting.dim             = [25, 4, 21];           % [状態数, 入力数, パラメータ数]
Setting.num_list        = [3,3,3,3,3,3,3,3,1];  % 各状態変数の次元 (p,q,v,w,pL,vL,pT,wL,mL)
Setting.state_list      = ["p","q","v","w","pL","vL","pT","wL","mL"];
Setting.initial         = initial;
if ~isfield(Setting.initial,"p")
    Setting.initial.p   = Setting.initial.pL - agent.parameter.cableL*Setting.initial.pT;
    Setting.initial.v   = [0; 0; 0];
end
Setting.initial.vL      = [0;0;0];
Setting.initial.wL      = [0;0;0];
Setting.initial.mL      = agent.parameter.loadmass;
Setting.dt              = dt;
Setting.param           = agent.parameter.get;   % 21要素の行ベクトル

% 質量は非負に射影
Setting.projection = @(x) [x(1:end-1); max(0, x(end))];

Model.param = Setting;
Model.parameter_order = resolve_parameter_order(Setting.method, Setting.dim(3));
end
