function [K, info] = design_lqg_sway(P, opt)
% design_lqg_sway  H2/LQG（LQR＋カルマン観測器）で牽引物の揺れを能動制振する.
%
%   [K, info] = design_lqg_sway(P)
%
%   mixsyn がうまく共振を抑えられない場合の、確実で頑健な代替設計.
%   負帰還 u = K*(r - θ)（レギュレーションでは r=0）で用いる.
%   同定モデル Pid（tf）でも解析モデルでも動くよう、状態重みは出力（揺れ角）
%   ベース Q = qy*C'C とし、観測器は入力点プロセス雑音 Qn=B*B'*rho（LTR的）で
%   実現に依らず良い減衰が出るようにしている.
%
%   入力:
%     P   : ss/tf  トルク→揺れ角 モデル（susp_load_linear_model か identify_sway_tf の Pid）
%     opt : name-value
%           "qy"    : 揺れ角(出力)への重み（既定 200. 大きいほど強く抑える）
%           "R"     : 制御入力への重み（既定 1. 大きいほど穏やか）
%           "rho"   : 観測器の速応性(LTR, 既定 1e3. 大きいほど速い観測器)
%           "rmeas" : 観測雑音（既定 1e-2. 大きいほど観測器は穏やか）
%           "x0deg" : 初期揺れ角の評価値（既定 10 deg, 4状態物理モデル時のみ）
%
%   出力:
%     K    : 制御器 ss（e=r-y -> u）
%     info : 閉ループ極, 減衰比, 余裕, S,T など

arguments
  P
  opt.qy   (1,1) double = 200
  opt.R    (1,1) double = 1
  opt.rho  (1,1) double = 1e3
  opt.rmeas(1,1) double = 1e-2
  opt.x0deg(1,1) double = 10
end
P = ss(P);
A = P.A;  B = P.B;  C = P.C;  n = size(A,1);

% --- LQR（出力=揺れ角を主に重み付け → 実現に依らない） ---
Q  = opt.qy*(C.'*C) + 1e-3*eye(n);
Kr = lqr(A, B, Q, opt.R);

% --- カルマン観測器（LTR的: プロセス雑音を入力点に） ---
Qn = B*B.' * opt.rho + 1e-6*eye(n);
Lk = lqr(A.', C.', Qn, opt.rmeas).';    % 双対で観測器ゲイン

% --- 制御器（e=r-y を入力, u を出力） ---
Ak = A - B*Kr - Lk*C;
K  = ss(Ak, -Lk, -Kr, 0);
K.InputName = "e"; K.OutputName = "u";

% --- 閉ループ量 ---
L  = P*K;  Sf = feedback(1,L);  Tf = feedback(L,1);
CLfb = feedback(P,K);                    % 入力外乱→θ（＝P*S）
[Gm,Pm] = margin(L);
zeta_cl = osc_zeta(CLfb);

info = struct("K",K,"S",Sf,"T",Tf,"L",L,"cl_poles",pole(CLfb), ...
              "GainMargin_dB",20*log10(Gm),"PhaseMargin_deg",Pm,"zeta_cl_min",zeta_cl, ...
              "Kr",Kr,"Lk",Lk);

fprintf("[LQG] Gain margin = %.1f dB, Phase margin = %.1f deg\n", 20*log10(Gm), Pm);
fprintf("[LQG] 閉ループ最小振動減衰比 zeta = %.3f （開ループ ~0 から改善）\n", zeta_cl);
if any(real(pole(CLfb)) > 1e-9)
  warning("閉ループが不安定です. rho/rmeas/R を調整してください.");
end

%% ===== 評価プロット =====
w = logspace(-2,3,800);

figure("Name","LQG: S/T");
sigma(Sf,"b",Tf,"r",w); grid on; legend("S","T","Location","southwest");
title(sprintf("LQG 感度  (\\zeta_{cl}=%.2f)", zeta_cl));

figure("Name","LQG: disturbance -> theta");
bode(P,"b", CLfb,"r--", w); grid on;
legend("開ループ","閉ループ（LQG）","Location","southwest");
title("入力トルク外乱 \rightarrow 揺れ角 \theta");

% インパルス外乱（突風）応答: 開ループ vs 閉ループ
figure("Name","LQG: gust impulse");
t = 0:0.01:15;
impulse(P,"b", CLfb,"r", t); grid on;
legend("開ループ（ふらつく）","閉ループ（LQG）","Location","northeast");
title("入力トルクにインパルス外乱を与えたときの \theta");

% 4状態の物理モデルのときは 10deg リリースの初期値応答も
if n==4 && isequal(C,[0 0 1 0])
  try
    x0 = [0;0;deg2rad(opt.x0deg);0];
    Pol = ss(A, zeros(4,0), C, zeros(1,0));
    yo  = initial(Pol, x0, t);
    yc  = initial(build_closed_loop_ic(P,K), [x0; zeros(order(K),1)], t);
    figure("Name","LQG: initial response");
    plot(t,rad2deg(yo),"b","LineWidth",1.5); hold on;
    plot(t,rad2deg(yc),"r","LineWidth",1.5); grid on;
    xlabel("time [s]"); ylabel("\theta [deg]");
    legend("開ループ (\zeta\approx0)","LQG 閉ループ","Location","northeast");
    title(sprintf("負荷を %d° 振らして解放", opt.x0deg));
  catch ME
    warning("初期値応答をスキップ: %s", ME.message);
  end
end

save("lqg_controller.mat","K","info");
end
