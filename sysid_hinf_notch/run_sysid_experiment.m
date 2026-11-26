%% run_sysid_experiment.m   (acsl-tcu/common_matlab 対応版)
%  共通プログラム（common_matlab）の吊り下げ負荷プラントを使い，
%  「ピッチトルク τy → 牽引物の揺れ角 θ」の同定用データを生成する.
%
%  やっていること:
%   - SimSuspendedLoad.m と同じ手順で agent(=DRONE) と負荷プラントを構築
%     （sensor/estimator/reference/controller の function class は使わず，
%       プラントを直接ステップして開ループ励振する）
%   - 各ステップで plant の入力  u=[thrust; τx; τy; τz]  を直接与える
%       * thrust : 高度保持の簡易P-D（機体を落とさない/暴れさせないため）
%       * τy     : ピッチ角速度ダンパ（暴走防止）＋ 励振信号（チャープ）
%   - 負荷のケーブル方向 pT から揺れ角 θ = atan2(pT_x, -pT_z) を計算して記録
%   - (t, τy, θ) を sysid_data.mat / sysid_data.csv に保存
%
%  ※ common_matlab のルート（DRONE.m がある場所）で実行する単独スクリプト.
%    AGENTS.md の experiment/test_*.m と同じ扱い（mainGUI/SimExp とは独立に動く）.
%    下の addpath でリポジトリのクラス群にパスを通す.
%

%% Initialize settings
% set path
clear all
cf = pwd;

if contains(mfilename('fullpath'), "mainGUI")
    cd(fileparts(mfilename('fullpath')));
else
    tmp = matlab.desktop.editor.getActive;
    cd(fileparts(tmp.Filename));
end

[~, tmp] = regexp(genpath('.'), '\.\\\.git.*?;', 'match', 'split');
cellfun(@(xx) addpath(xx), tmp, 'UniformOutput', false);
close all hidden; clear; clc;
userpath('clear');
%%
clc; clear;
% --- common_matlab の場所 ---
%  既定は「今このスクリプトを common_matlab ルートで実行している」前提.
%  別の場所から実行するなら repoRoot に common_matlab の絶対パスを入れること.
repoRoot = pwd;
% repoRoot = "C:\path\to\common_matlab";   % ← 別フォルダから回す場合はここを指定
addpath(genpath(repoRoot));

% --- リポジトリのクラスが本当に見えているか（見えなければ明示的に失敗させる） ---
need = ["DRONE","DRONE_PARAM_SUSPENDED_LOAD","MODEL_CLASS","Model_Suspended_Load","TIME"];
missing = need(arrayfun(@(s) exist(s,'class')==0 && exist(s,'file')==0, need));
if ~isempty(missing)
    error("common_matlab が参照できていません（未定義: %s）。repoRoot を common_matlab のルートに設定してください。", ...
        strjoin(missing, ", "));
end

%% --- シミュレーション設定 ---
N  = 1;
ts = 0;  dt = 0.025;  te = 50;         % 同定用は dt を細かめ（0.01s）に
time = TIME(ts, dt, te,N);
tvec = ts:dt:te;  Nk = numel(tvec);
notchy=[];

%% --- agent / plant 構築（SimSuspendedLoad と同じ流儀） ---
agent = DRONE;
agent.parameter = DRONE_PARAM_SUSPENDED_LOAD("DIATONE");
% ▼ 対象に合わせて設定（例: sim既定）. 実機なら実測値に置き換える.
agent.parameter.set("cableL",  3);
agent.parameter.set("loadmass",0.0556);
% agent.parameter.set("mass",   0.762);

initial_state.q  = [0; 0; 0];
initial_state.w  = [0; 0; 0];
initial_state.v  = [0; 0; 0];
initial_state.vL = [0; 0; 0];
initial_state.wL = [0; 0; 0];
initial_state.p  = [0; 0; 1.5];
initial_state.pT = [0; 0; -1];                                   % ケーブルは初期に真下
initial_state.pL = initial_state.p + initial_state.pT*agent.parameter.cableL;


agent.plant = MODEL_CLASS(agent, Model_Suspended_Load(dt, initial_state, 1, agent));

% --- 実際に積分される非線形ダイナミクスを実行時に表示（自己申告） ---
%  ここに with_load_model_euler_for_HL と出れば、リポジトリの非線形モデルを使っている.
fprintf("使用プラント dynamics = %s  (状態 %d次元)\n", ...
    func2str(agent.plant.method), agent.plant.dim(1));

M  = agent.parameter.mass;
mp = agent.parameter.loadmass;
g  = agent.parameter.gravity;
hover_thrust = (M + mp) * g;

%% --- 励振信号（チャープ）: ω_n を挟む 0.05〜3 Hz ---
f_lo = 0.05;  f_hi = 3.0;             % [Hz]
tau_amp = 0.02;                       % 励振トルク振幅 [N*m]（小さめ・線形域）
probe = tau_amp * chirp(tvec, f_lo, te, f_hi, "logarithmic");
% 別案: 多正弦(multisine)にしたい場合は下を有効化
% freqs = logspace(log10(f_lo), log10(f_hi), 15);
% probe = zeros(1,Nk);
% for ff = freqs, probe = probe + (tau_amp/sqrt(numel(freqs)))*sin(2*pi*ff*tvec + 2*pi*rand); end

%% --- 簡易安定化ゲイン（暴走・墜落を防ぐだけ．同定対象ではない） ---
kdw  = 0.100;   % ピッチ角速度ダンパ
kpz  = 2.0;    % 高度 P
kdz  = 1.5;    % 高度 D
zref = initial_state.p(3);

%% --- ログ用 ---
T = zeros(Nk,1);  U = zeros(Nk,1);  Y = zeros(Nk,1); pL=zeros(Nk,3);
WY_raw = zeros(Nk,1);   % ノッチフィルタ適用前のwy
WY_filt = zeros(Nk,1);  % ノッチフィルタ適用後のwy

%% --- メインループ（プラントを直接ステップ） ---
for k = 1:Nk
    % 状態読み出し
    w = agent.plant.get("w");   wy = w(2);         % ピッチ角速度
    p = agent.plant.get("p");   z  = p(3);
    v = agent.plant.get("v");   vz = v(3);

    % WY_raw(k) = wy;                                 % フィルタ前を記録
    % [wy, notchy] = notchFilter(wy, dt, notchy);      % wyをノッチフィルタで更新
    % WY_filt(k) = wy;                                 % フィルタ後を記録

    % 入力生成
    thrust = hover_thrust + kpz*(zref - z) - kdz*vz;
    tau_y  = -kdw*wy + probe(k);                    % ダンパ + 励振
    % [tau_y, notchy] = notchFilter(tau_y, dt, notchy);    % wyをノッチフィルタで更新
    u = [thrust; 0; tau_y; 0];

    % プラントに入力を渡して 1 ステップ更新
    agent.controller.result.input = u;              % MODEL_CLASS.do が参照する場所
    agent.plant.do(time, 'f');

    % 出力（揺れ角）算出： pT は負荷側ケーブル方向の単位ベクトル
    pT = agent.plant.get("pT");
    theta = atan2(pT(1), -pT(3));                   % ピッチ面内の揺れ角 [rad]

    pL(k,:)=agent.plant.get("pL");

    % 記録（入力は「プラントに実際に与えた τy 総量」）
    T(k) = time.t;  U(k) = tau_y;  Y(k) = theta;

    time.t = time.t + dt;
end

%% --- 保存 ---
sysid.t = T;  sysid.tau_y = U;  sysid.theta = Y;  sysid.dt = dt; sysid.pL=pL;
% sysid.wy_raw = WY_raw;  sysid.wy_filt = WY_filt;   % 追加
save("sysid_data.mat", "-struct", "sysid");
writematrix([T U Y], "sysid_data.csv");   % 列: t, tau_y[N*m], theta[rad]

%% --- 簡単な確認プロット ---
figure("Name","sysid excitation");
subplot(2,1,1); plot(T, U, "LineWidth",1); grid on;
ylabel("\tau_y [N·m]");xlabel("time [s]");
title("励振入力（総トルク）");
subplot(2,1,2); plot(T, rad2deg(Y), "LineWidth",1); grid on;
ylabel("\theta [deg]"); xlabel("time [s]");
title("牽引物の揺れ角");

figure(2);
plot(T, pL, "LineWidth",1);
grid on;
ylabel("position [m]");
xlabel("time [s]");
legend("x", "y", "z");
title("牽引物の位置");

figure("Name","notch filter design");
sys_notch = tf(notchy.b, notchy.a, dt);   % 離散時間の伝達関数として構築
w_plot = logspace(-1, log10(pi/dt), 500);  % 0.1 rad/s ~ ナイキスト周波数まで
[mag, ph, w_out] = bode(sys_notch, w_plot);
mag_db = 20*log10(squeeze(mag));
ph_deg = squeeze(ph);

subplot(2,1,1);
semilogx(w_out, mag_db, "LineWidth", 1.5); grid on;
ylabel("Gain [dB]");
title("ノッチフィルタ特性");

subplot(2,1,2);
semilogx(w_out, ph_deg, "LineWidth", 1.5); grid on;
ylabel("Phase [deg]");
xlabel("\omega [rad/s]");
fprintf("保存しました: sysid_data.mat / sysid_data.csv  (Nk=%d, dt=%.3f s)\n", Nk, dt);
fprintf("次は identify_sway_tf.m を実行してください.\n");


%%
function [wy, notchy] = notchFilter(wy, dt, notchy)
if isempty(notchy)
    wn = 3.25;
    zz = [1, 0.1];    % 分子側（小さいほど深い谷）
    zp = [0.3, 0.5];    % 分母側（谷の幅）
    % pitch用フィルタ設計
    dy = c2d(tf([1 2*zz(2)*wn wn^2], [1 2*zp(2)*wn wn^2])^2, dt, 'tustin');
    [by, ay] = tfdata(dy, 'v');
    zy0 = zeros(max(length(ay), length(by)) - 1, 1);
    notchy = struct('b', by, 'a', ay, 'z', zy0);
end
[wy, notchy.z] = filter(notchy.b, notchy.a, wy, notchy.z);
end

% rmse_theta_deg = sqrt(mean(rad2deg(sysid.theta).^2)); fprintf('θ の RMSE (角度) = %.3f deg\n', rmse_theta_deg);