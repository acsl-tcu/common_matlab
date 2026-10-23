%RUN_STAGE4_DESIGNB  段階4：設計B（周波数重み付き H∞、拡張状態 x_a=[x; x_W] の状態フィードバック）
P = ctow_params();
opt = struct('alpha',0.02, 'q_th',0, 'b_int',0.3, 'rho_v',0.003);   % 事前検討の初期値
% opt.kW = 0;                                                       % 追加: 0 にすると設計 A と同じ結果になる（再現確認用。確認後は % に戻す）
G = ctow_genplant_Bmy(P, opt);                                        % 変更: A → B（重み W_theta を加えた拡張プラント）
gopt = ctow_hinf_gamma_opt(G.A, G.B1, G.B2, G.C1, G.rho_v);
% gamma の選び方: gopt < 1 なら gamma = min(1, 1.5*gopt)（r 保証を残しつつ最適近傍の過大ゲインを避ける）
if gopt < 1, gamma = max(1.05*gopt, min(1, 1.5*gopt)); else, gamma =1.5*gopt; end
[K, ~, ok] = ctow_hinf_sf(G.A, G.B1, G.B2, G.C1, G.rho_v, gamma);
assert(ok, 'H∞ が解けない');
fprintf('gamma_opt = %.4f, gamma = %.4f\nK = %s\n', gopt, gamma, mat2str(K, 6));
% Kx = K(:, 1:G.nx);  Kw = K(:, G.nx+1:end);                          % 追加: K は 1x8 になるので [Kx(1x6), Kw(1x2)] に分ける

%定常偏差を追加した際のK
% Kx  = K(1:G.nx); 
% Kw  = K(G.nx+1 : G.nx+G.nw);
% Kxi = K(end);

mA  = ctow_eval(K, 'Hinf A', P);                                  
% A_cl = G.A - G.B2*K;  ev = eig(A_cl);
% mA.sigma    = -max(real(ev));
% mA.zeta_min = min(-real(ev)./abs(ev));
% mA.TD_inf   = norm(ss(A_cl, G.B1(:,2), G.C1(G.idx_zD,:), 0), inf);
% nc = 1;
% Ac = [G.Aw,           zeros(G.nw,1);
%       zeros(1,G.nw),  0];
% Bc = [G.Bw*G.e3/P.g;
%       G.e1];
% Wd = c2d(ss(Ac, Bc, eye(G.nw+1), 0), P.dt, 'zoh');
% Kdyn = ss(Wd.A, [Wd.B, zeros(G.nw+1, nc)], -[Kw, Kxi], [-Kx, zeros(1, nc)], P.dt);
% mA.rho = arrayfun(@(t) ctow_impl_rho_dyn(Kdyn, P, t), P.tau_list);
% mA.stable_impl = all(mA.rho < 1);
% mA.sigma_cont = mA.sigma;
% mA.sigma = -log(mA.rho(2))/P.dt;      % G1・G2（ctow_eval）と同じ定義                                % 変更ここまで
mG1 = ctow_eval(P.K.G1, 'G1 (LQR R=1e-2)', P);
mG2 = ctow_eval(P.K.G2, 'G2 (optimized)', P);
save('stage4_designB.mat', 'K', 'gamma', 'gopt', 'opt', 'mA', 'mG1', 'mG2');   % 変更: 保存先のみ（stage3_designA.mat を上書きしないため）
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

% ---- 設計B の時間応答（ctow_sim_impl と同じ離散化・遅れ。ctow_sim_impl は変更しない）----
[Ap, Bp, Ep] = ctow_chain(1);
Af = Ap; Af(6,6) = Af(6,6) - 1/P.tau_r; Bf = Bp/P.tau_r;
M = expm([Af [Bf Ep]; zeros(2,8)]*P.dt);
Phi = M(1:6,1:6); Gc = M(1:6,7); Gw = M(1:6,8);
N = round(40/P.dt);
scs = {sc1, sc2};
sB = cell(1,2);
for j = 1:2
    pref = @(t) 0;  wf = @(t) 0;
    if isfield(scs{j}, 'p_ref'), pref = scs{j}.p_ref; end
    if isfield(scs{j}, 'w'),     wf   = scs{j}.w;     end
    x = zeros(6,1); c = 0; z = zeros(3,1);          % z = [x_W; xi]
    t = (0:N-1)*P.dt; p = zeros(1,N); th = zeros(1,N);
    for k = 1:N
        xe = x - [pref(t(k)); zeros(5,1)];
        v  = -Kx*xe - Kw*z(1:2) - Kxi*z(3);
        p(k) = x(1); th(k) = x(3)/P.g;
        x = Phi*x + Gc*c + Gw*wf(t(k));
        c = c + P.dt*v;                              % 1 サンプル遅れ
        z = Wd.A*z + Wd.B*xe;                        % 次の周期の x_W, xi
    end
    sB{j} = struct('t',t, 'p',p, 'theta',th);
end

% ---- 図 ----
figure; tl = tiledlayout(2,2);
lab = {'G1','G2','Hinf B'};
for i = 1:3
    if i < 3
        Kg = P.K.(sprintf('G%d',i));
        s1 = ctow_sim_impl(Kg, P, sc1);  s2 = ctow_sim_impl(Kg, P, sc2);
    else
        s1 = sB{1};  s2 = sB{2};
    end
    nexttile(1); hold on; plot(s1.t, s1.p);
    nexttile(2); hold on; plot(s1.t, rad2deg(s1.theta));
    nexttile(3); hold on; plot(s2.t, s2.p);
    nexttile(4); hold on; plot(s2.t, rad2deg(s2.theta));
end
ttl = {'S1 position','S1 swing angle [deg]','S2 position','S2 swing angle [deg]'};
for i = 1:4, nexttile(i); title(ttl{i}); grid on; legend(lab); xlabel('t [s]'); end