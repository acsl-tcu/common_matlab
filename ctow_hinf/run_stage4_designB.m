%RUN_STAGE4_DESIGNB  段階4（模範解答）：振れ角の周波数重み W_theta を加えた状態フィードバック H∞
%   設計 A と同じ重み（alpha, b_int, rho_v, r の範囲）に W_theta を足し、k_th を振って比較する。
P = ctow_params();
k_list = [0 1 3 10 30];
res = struct([]);
fprintf('%5s %7s %7s %8s %9s %22s %7s %9s\n', 'k_th','gopt','gamma','||TD||','||Twth||','rho(10/30/50ms)','sigma','S2 maxθ');
for i = 1:numel(k_list)
    G = ctow_genplant_B(P, struct('k_th', k_list(i)));
    gopt = ctow_hinf_gamma_opt(G.A, G.B1, G.B2, G.C1, G.rho_v);
    gamma=1.05*gopt;
    % if gopt < 1, gamma = max(1.05*gopt, min(1, 1.5*gopt)); else, gamma = 1.5*gopt; end
    [K, ~, ok] = ctow_hinf_sf(G.A, G.B1, G.B2, G.C1, G.rho_v, gamma); assert(ok);
    % 連続時間の指標（閉ループ 8 次）
    % e3 = [0 0 1 0 0 0];
    % [A1, B, E] = ctow_chain(0.75);   Aa1 = [A1 zeros(6,2); G.Bw*e3/P.g G.Aw];  %質量比 r の変変更
    % [Ar0, ~, ~] = ctow_chain(P.r0); Aa0 = [Ar0 zeros(6,2); G.Bw*e3/P.g G.Aw]; %質量比 r の変更大本は3行目
    % Ba = [B; 0; 0]; Ea = [E; 0; 0];
    % TD   = norm(ss(Aa0 - Ba*K, Ea, [P.r0*P.Wr*e3 0 0], 0), Inf);
    % Twth = norm(ss(Aa1 - Ba*K, Ea, [e3/P.g 0 0], 0), Inf);
    % rho  = arrayfun(@(t) ctow_impl_rho_B(K, P, G, t), P.tau_list);
    % sigma = -log(rho(2))/P.dt;
    % s2 = ctow_sim_impl_B(K, P, G, @(t) 0.5*double(t>=1));  %外乱の入力設計
    % res(i).k_th = k_list(i); res(i).K = K; res(i).gopt = gopt; res(i).gamma = gamma;
    % res(i).TD = TD; res(i).Twth = Twth; res(i).rho = rho; res(i).sigma = sigma;
    % res(i).S2max = max(abs(s2.theta)); res(i).s2 = s2;
    % fprintf('%5g %7.3f %7.3f %8.3f %9.4f   [%.4f %.4f %.4f] %7.3f %8.3f°\n', ...
    %     k_list(i), gopt, gamma, TD, Twth, rho, sigma, rad2deg(res(i).S2max));
    
    %連続時間の指標(閉ループ 9次元)
       e3 = [0 0 1 0 0 0];
    e1 = [1 0 0 0 0 0];                                                          % p（ξ' = p）
    [A1, B, E] = ctow_chain(0.75);
    Aa1 = [A1 zeros(6,2) zeros(6,1); G.Bw*e3/P.g G.Aw zeros(2,1); e1 zeros(1,2) 0];   %質量比 r の変更
    [Ar0, ~, ~] = ctow_chain(P.r0);
    Aa0 = [Ar0 zeros(6,2) zeros(6,1); G.Bw*e3/P.g G.Aw zeros(2,1); e1 zeros(1,2) 0]; %質量比 r の変更大本は3行目
    Ba = [B; 0; 0; 0]; Ea = [E; 0; 0; 0];
    TD   = norm(ss(Aa0 - Ba*K, Ea, [P.r0*P.Wr*e3 0 0 0], 0), Inf);
    Twth = norm(ss(Aa1 - Ba*K, Ea, [e3/P.g 0 0 0], 0), Inf);
    rho  = arrayfun(@(t) ctow_impl_rho_B(K, P, G, t), P.tau_list);
    sigma = -log(rho(2))/P.dt;
    s2 = ctow_sim_impl_B(K, P, G, @(t) 0.5*double(t>=1));  %外乱の入力設計
    res(i).k_th = k_list(i); res(i).K = K; res(i).gopt = gopt; res(i).gamma = gamma;
    res(i).TD = TD; res(i).Twth = Twth; res(i).rho = rho; res(i).sigma = sigma;
    res(i).S2max = max(abs(s2.theta)); res(i).s2 = s2;
    fprintf('%5g %7.3f %7.3f %8.3f %9.4f   [%.4f %.4f %.4f] %7.3f %8.3f°\n', ...
        k_list(i), gopt, gamma, TD, Twth, rho, sigma, rad2deg(res(i).S2max));

end
% 基準 G1
sG1 = ctow_sim_impl(P.K.G1, P, struct('T',40, 'w',@(t) 0.5*double(t>=1)));  %外乱の入力設計
[A1, B, E] = ctow_chain(1);
Twth_G1 = norm(ss(A1 - B*P.K.G1, E, [0 0 1 0 0 0]/P.g, 0), Inf);
fprintf('%5s %31s %9.4f %40s %8.3f°\n', 'G1', '', Twth_G1, '', rad2deg(max(abs(sG1.theta))));
save('stage4_designB.mat', 'res');

% ---- 判定（手順書 5 章） ----
i0 = find([res.k_th] == 0);
fprintf('\n設計点 r0 = %.2f（r の範囲 [%.2f, %.2f] の中央）。評価・シミュレーションは r = 1（公称）で行っている。\n', P.r0, P.r_lo, P.r_hi);
okc = [res.TD] < 1 & arrayfun(@(r) all(r.rho < 1), res) & [res.sigma] >= 0.3;
if ~any(okc)
    fprintf(['判定: NG（r保証・実装安定・sigma>=0.3 をすべて満たす k_th がない）\n' ...
             '  各 k_th の sigma: %s\n' ...
             '次のアクション: r の範囲を変えると設計点 r0 も動き、r=1 での速さ sigma が変わる（手順書 5.4）。\n' ...
             '  alpha（性能の重み）を上げて sigma を 0.3 以上に戻すか、sigma の基準を G1 との相対比較に変えて PI に相談する。\n'], ...
             mat2str([res.sigma], 3));
else
    best = res(okc);
    [~, j] = min([best.Twth]);
    gain_pct = 100*(res(i0).Twth - best(j).Twth)/res(i0).Twth;
    fprintf('条件(r保証・実装安定・sigma>=0.3)を満たす中で ||Twth|| 最小: k_th=%g, 設計Aからの改善 %.1f%%\n', best(j).k_th, gain_pct);
    if gain_pct >= 5
        fprintf('判定: OK（W_theta で振れ角が設計 A より改善）\n次のアクション: 段階5（離散時間 H∞）へ進む。\n');
    else
        fprintf(['判定: W_theta の効果は小さい（%.1f%%）。期待どおりの結果（手順書 5.3 参照）\n' ...
                 '次のアクション: 理由（z_Delta = r0*Wr*x3 が既に x3 を強く重み付けしている）を確認し、\n' ...
                 '  任意課題として r の範囲を狭めて W_theta の効きが増すかを調べる。その後 段階5 へ。\n'], gain_pct);
    end
end
% figure; hold on;
% plot(sG1.t, rad2deg(sG1.theta), 'k--', 'DisplayName', 'G1');
% for i = 1:numel(res)
%     plot(res(i).s2.t, rad2deg(res(i).s2.theta), 'DisplayName', sprintf('Hinf B k_{th}=%g', res(i).k_th));
% end
% grid on; legend; xlabel('t [s]'); ylabel('swing angle [deg]'); 
% % title('S2: step force 0.5 @1s');
% 
% 
% figure; hold on;
% plot(sG1.t, sG1.p, 'k--', 'DisplayName', 'G1');
% for i = 1:numel(res)
%     plot(res(i).s2.t, res(i).s2.p, 'DisplayName', sprintf('Hinf B k_{th}=%g', res(i).k_th));
% end
% grid on; legend; xlabel('t [s]'); ylabel('position [m]');
% % title('S2: step force 0.5 @1s');

figure; hold on;
plot(sG1.t, rad2deg(sG1.theta), 'k--', ...
    'LineWidth', 2, 'DisplayName', 'G1');

for i = 2:numel(res)
    plot(res(i).s2.t, rad2deg(res(i).s2.theta), ...
        'LineWidth', 2, ...
        'DisplayName', sprintf('Hinf B k_{th}=%g', res(i).k_th));
end

grid on;
legend('FontSize', 14);
xlabel('t [s]', 'FontSize', 18);
ylabel('swing angle [deg]', 'FontSize', 18);
set(gca, 'FontSize', 18);
% title('S2: step force 0.5 @1s');


figure; hold on;
plot(sG1.t, sG1.p, 'k--', 'LineWidth', 2);

for i = 2:numel(res)
    plot(res(i).s2.t, res(i).s2.p, 'LineWidth', 2);
end

grid on;
xlabel('t [s]', 'FontSize', 18);
ylabel('position [m]', 'FontSize', 18);
set(gca, 'FontSize', 18);
% title('S2: step force 0.5 @1s');