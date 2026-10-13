function [K, CL, gam, info] = design_hinf_sway(P, opt)
% design_hinf_sway  混合感度 H∞（mixsyn）で牽引物の揺れを抑える制御器を設計する.
%
%   [K, CL, gam, info] = design_hinf_sway(P)
%
%   Robust Control Toolbox の mixsyn を用いて，plant P（τy→θ）に対し
%   S/KS/T の混合感度最小化により，振り子共振を減衰させる制御器 K を得る.
%   負のフィードバック u = K*(r - θ) を想定（レギュレーションでは r=0）.
%
%   入力:
%     P   : ss/tf  トルク→揺れ角 モデル
%           （susp_load_linear_model の解析モデル，または identify_sway_tf の Pid）
%     opt : name-value（重みの調整）
%           "Ms"  : 感度 S のピーク上限に対応（既定 2.0）
%           "wb"  : 目標帯域（既定 = 2*wn 相当を自動．手動指定可）[rad/s]
%           "Mu"  : 制御感度 KS の高域ゲイン上限（既定 1e2）
%           "x0deg" : 初期揺れ角（評価用，既定 10 [deg]）
%
%   出力:
%     K    : 設計した制御器（ss）
%     CL   : 重み付き閉ループ（mixsyn 出力）
%     gam  : 達成 H∞ ノルム（1 前後なら重み仕様をほぼ満たす）
%     info : S,T,KS, 余裕, 閉ループ極 等

arguments
  P
  opt.Ms (1,1) double = 2.0
  opt.wb (1,1) double = NaN
  opt.Mu (1,1) double = 1e2
  opt.x0deg (1,1) double = 10
end
P = ss(P);

% --- 共振周波数の推定（帯域設定の目安） ---
pOL   = pole(P);
pOLc  = pOL(imag(pOL) > 1e-6);            % 振動極（複素共役の上側）
if ~isempty(pOLc)
  wn = abs(pOLc(1));
else
  wn = 5;
end
if isnan(opt.wb), wb = 2*wn; else, wb = opt.wb; end

% --- 重みの設計 -----------------------------------------------------------
%  W1 (性能/S): 低域で高ゲイン→積分作用と外乱抑制, 帯域 wb, 高域は緩める.
%       |S| < 1/|W1| を要求 → 低周波で S を小さく，共振も帯域内に入れて減衰.
A_lf = 1e-4;                                   % S の低域下限（外乱抑制の強さ）
W1 = makeweight(1/A_lf, wb, 1/opt.Ms);         % dcgain, crossover, hfgain
%  W3 (ロバスト性/T): 高域で T を絞る（未モデル化・雑音対策）.
W3 = makeweight(1e-2, 8*wn, opt.Mu);           % 低域緩, 高域で締める
%  W2 (制御感度/KS): 高域の制御入力を抑える.
W2 = makeweight(1e-2, 12*wn, opt.Mu) * tf(1,1);

% --- H∞ 混合感度合成 -----------------------------------------------------
[K, CL, gam] = mixsyn(P, W1, W2, W3);
K = ss(K);
fprintf("mixsyn achieved gamma = %.3f  (<=~1 が目安)\n", gam);

% --- 閉ループ量 ----------------------------------------------------------
L  = P*K;                      % 開ループ伝達
Sf = feedback(1, L);           % 感度 S = 1/(1+L)
Tf = feedback(L, 1);           % 相補感度 T = L/(1+L)
KS = K*Sf;                     % 制御感度
[Gm, Pm, Wcg, Wcp] = margin(L);

info = struct();
info.S = Sf; info.T = Tf; info.KS = KS; info.L = L;
info.GainMargin_dB = 20*log10(Gm);
info.PhaseMargin_deg = Pm;
info.cl_poles = pole(feedback(P,K));
info.wn = wn; info.wb = wb; info.gamma = gam;

fprintf("Gain margin = %.1f dB,  Phase margin = %.1f deg\n", 20*log10(Gm), Pm);
pCL   = info.cl_poles;
pCLc  = pCL(imag(pCL) > 1e-6);
zeta_cl_min = NaN;
if ~isempty(pCLc)
  zeta_cl_min = min(-real(pCLc)./abs(pCLc));
  fprintf("閉ループの最小振動減衰比 zeta = %.3f （開ループの軽減衰から改善）\n", zeta_cl_min);
end
info.zeta_cl_min = zeta_cl_min;

%% ===================== 評価プロット =====================
w = logspace(-2, 3, 800);

% (1) 重みと S/T/KS
figure("Name","H∞ mixed sensitivity");
sigma(Sf,"b", Tf,"r", KS,"g", 1/W1,"b--", 1/W3,"r--", w); grid on;
legend("S","T","KS","1/W1","1/W3","Location","southwest");
title(sprintf("混合感度  (\\gamma=%.2f)", gam));

% (2) 開ループ Bode（余裕）
figure("Name","Loop L = P*K"); margin(L); grid on;

% (3) プラント Bode: 開ループ vs 閉ループ(τ外乱→θ)
figure("Name","Bode: disturbance -> theta");
Gd_ol = P;                     % 入力外乱→θ（開ループ）
Gd_cl = feedback(P, K);        % 入力外乱→θ（閉ループ）= P*S
bode(Gd_ol,"b", Gd_cl,"r--", w); grid on;
legend("開ループ","閉ループ（H∞）","Location","southwest");
title("入力トルク外乱 \rightarrow 揺れ角 \theta");

% (4) 初期揺れ角からの時間応答（x0deg リリース）
%     解析モデル（susp_load_linear_model）で使うときのみ有効.
%     状態順が [beta; beta'; theta; theta'] であることを利用する.
if size(P.A,1) == 4 && isequal(P.C, [0 0 1 0])
  try
    x0  = [0; 0; deg2rad(opt.x0deg); 0];
    t   = 0:0.01:15;
    Pol = ss(P.A, zeros(4,0), P.C, zeros(1,0));      % 入力0の自律系（開ループ）
    yo  = initial(Pol, x0, t);
    CLic = build_closed_loop_ic(P, K);               % 状態を保った H∞ 閉ループ
    yc  = initial(CLic, [x0; zeros(order(K),1)], t);
    z_ol = minosc(P);
    figure("Name","initial response");
    plot(t, rad2deg(yo),"b","LineWidth",1.5); hold on;
    plot(t, rad2deg(yc),"r","LineWidth",1.5); grid on;
    xlabel("time [s]"); ylabel("\theta [deg]");
    legend(sprintf("開ループ (\\zeta\\approx%.2f)", z_ol), "H∞ 閉ループ", ...
           "Location","northeast");
    title(sprintf("負荷を %d° 振らして解放", opt.x0deg));
  catch ME
    warning("初期値応答の作図をスキップ: %s", ME.message);
  end
end

save("hinf_controller.mat", "K", "gam", "info");
end

% ---- 補助: 状態を保った負帰還閉ループ（初期値応答用） ----
function CLic = build_closed_loop_ic(P, K)
Pss = ss(P);  Kss = ss(K);
Ap=Pss.A; Bp=Pss.B; Cp=Pss.C;             % Dp=0 前提
Ak=Kss.A; Bk=Kss.B; Ck=Kss.C; Dk=Kss.D;   % u = Ck xk + Dk e,  e = r - y, r=0 → e=-y
% xp' = Ap xp + Bp u = (Ap - Bp Dk Cp) xp + Bp Ck xk
% xk' = Ak xk + Bk e = -Bk Cp xp + Ak xk
Acl = [Ap - Bp*Dk*Cp,  Bp*Ck;
       -Bk*Cp,         Ak    ];
Ccl = [Cp, zeros(1, size(Ak,1))];
CLic = ss(Acl, zeros(size(Acl,1),0), Ccl, zeros(1,0));
end

% ---- 補助: 開ループの最小振動減衰比 ----
function z = minosc(P)
pp = pole(ss(P)); pc = pp(imag(pp) > 1e-6);
if isempty(pc), z = NaN; else, z = min(-real(pc)./abs(pc)); end
end
