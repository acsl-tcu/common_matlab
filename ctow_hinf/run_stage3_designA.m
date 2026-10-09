%RUN_STAGE3_DESIGNA  段階3：設計A（静的重みの連続時間 状態フィードバック H∞）
P = ctow_params();
opt = struct('alpha',0.01, 'q_th',0, 'b_int',0.3, 'rho_v',0.003);   % 事前検討の初期値
G = ctow_genplant_A(P, opt);
gopt = ctow_hinf_gamma_opt(G.A, G.B1, G.B2, G.C1, G.rho_v);
% gamma の選び方: gopt < 1 なら gamma = min(1, 1.5*gopt)（r 保証を残しつつ最適近傍の過大ゲインを避ける）
if gopt < 1, gamma = max(1.05*gopt, min(1, 1.5*gopt)); else, gamma = 1.5*gopt; end
[K, ~, ok] = ctow_hinf_sf(G.A, G.B1, G.B2, G.C1, G.rho_v, gamma);
assert(ok, 'H∞ が解けない');
fprintf('gamma_opt = %.4f, gamma = %.4f\nK = %s\n', gopt, gamma, mat2str(K, 6));
mA  = ctow_eval(K, 'Hinf A', P);
mG1 = ctow_eval(P.K.G1, 'G1 (LQR R=1e-2)', P);
mG2 = ctow_eval(P.K.G2, 'G2 (optimized)', P);
save('stage3_designA.mat', 'K', 'gamma', 'gopt', 'opt', 'mA', 'mG1', 'mG2');

% ---- 判定（手順書4章） ----
c1 = mA.TD_inf < 1;                 % r in [0.5,3.5] のロバスト安定保証　‖T_Δ‖∞
c2 = mA.sigma >= 0.3;               % 速さσ
c3 = mA.stable_impl;                % 実装モデルで安定（tau 10/30/50 ms）τ_r
fprintf('\n判定: r保証(||TD||<1)=%d, 速さ(sigma>=0.3)=%d, 実装安定=%d\n', c1, c2, c3);
if c1 && c2 && c3
    fprintf('判定: OK\n次のアクション: 段階4（周波数重み）と段階6（シミュレーション比較）へ進む。zeta_min は G1 より悪い見込み（手順書1.3）なので、値を記録しておくこと。\n');
elseif ~c3
    fprintf('判定: NG（実装モデルで不安定）\n次のアクション: rho_v を上げる（速い極を約10 rad/s 以下に）か gamma を大きくして再実行。改善しなければ段階5（離散時間設計）へ。\n');
else
    fprintf('判定: NG（r保証と速さが両立しない）\n次のアクション: alpha を 0.003〜0.03、r の範囲を [0.7,3] などで振り、(sigma, ||TD||) のトレードオフ図を作って PI に相談。\n');
end

% ---- 時間応答（S1: 位置ステップ, S2: 外力ステップ） ----
sc1 = struct('T',40, 'p_ref',@(t) double(t>=1));
sc2 = struct('T',40, 'w',@(t) 0.5*double(t>=1));
figure; tl = tiledlayout(2,2);
Ks = {P.K.G1, P.K.G2, K}; lab = {'G1','G2','Hinf A'};
for i = 1:3
    s1 = ctow_sim_impl(Ks{i}, P, sc1); s2 = ctow_sim_impl(Ks{i}, P, sc2);
    nexttile(1); hold on; plot(s1.t, s1.p);
    nexttile(2); hold on; plot(s1.t, rad2deg(s1.theta));
    nexttile(3); hold on; plot(s2.t, s2.p);
    nexttile(4); hold on; plot(s2.t, rad2deg(s2.theta));
end
ttl = {'S1 position','S1 swing angle [deg]','S2 position','S2 swing angle [deg]'};
for i = 1:4, nexttile(i); title(ttl{i}); grid on; legend(lab); xlabel('t [s]'); end
