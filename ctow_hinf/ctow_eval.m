function m = ctow_eval(K, name, P, kc, do_zeta)
%CTOW_EVAL  静的ゲイン v = -K x + kc*c の評価指標をまとめて計算・表示
if nargin < 4 || isempty(kc), kc = 0; end
if nargin < 5, do_zeta = true; end
dt = P.dt;
m.name = name; m.K = K(:)'; m.kc = kc;
m.rho = arrayfun(@(t) ctow_impl_rho(K, dt, t, 1, kc), P.tau_list);
m.stable_impl = all(m.rho < 1);
rho_nom = ctow_impl_rho(K, dt, P.tau_r, 1, kc);
m.sigma = -log(rho_nom)/dt;                          % 最も遅い減衰率 [1/s]
rs = 0.05:0.05:8;
okr = arrayfun(@(r) ctow_impl_rho(K, dt, P.tau_r, r, kc) < 1, rs);
if any(okr), m.r_max = rs(find(okr, 1, 'last')); else, m.r_max = NaN; end
% 連続時間の指標（r = 1）
[A1, B, E] = ctow_chain(1);
e1 = [1 0 0 0 0 0]; e2 = [0 1 0 0 0 0]; e3 = [0 0 1 0 0 0];
Acl = A1 - B*K(:)';
m.Twp_inf  = norm(ss(Acl, E, e1, 0), Inf);           % 外力 -> 位置
m.Twth_inf = norm(ss(Acl, E, e3/P.g, 0), Inf);       % 外力 -> 振れ角
[Ar0, ~, ~] = ctow_chain(P.r0);
m.TD_inf   = norm(ss(Ar0 - B*K(:)', E, P.r0*P.Wr*e3, 0), Inf);  % < 1 で r のロバスト安定保証
w = logspace(-2, 3, 4000);
h = squeeze(freqresp(ss(Acl, E, e2, 0), w));
m.minReGv = min(real(h));                            % 相互作用ポートの受動性（>=0 が受動）
if do_zeta && m.stable_impl
    m.zeta_min = ctow_zeta_min(K, dt, P.tau_r, kc);
else
    m.zeta_min = NaN;
end
fprintf('%-14s rho(10/30/50ms)=[%.4f %.4f %.4f] sigma=%.3f r_max=%.2f ||Twp||=%.3f ||Twth||=%.4f ||TD||=%.3f minReGv=%.5f zeta_min=%.4f\n', ...
    name, m.rho, m.sigma, m.r_max, m.Twp_inf, m.Twth_inf, m.TD_inf, m.minReGv, m.zeta_min);
end
