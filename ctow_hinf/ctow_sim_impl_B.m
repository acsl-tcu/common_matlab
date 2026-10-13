function s = ctow_sim_impl_B(K, P, Wd, w_fun, T, tau)
%CTOW_SIM_IMPL_B  設計 B の実装モデルの時間応答（外力 w_fun(t) のみ、初期値 0）
if nargin < 5, T = 40; end
if nargin < 6, tau = P.tau_r; end
dt = P.dt; N = round(T/dt);
[A, B, E] = ctow_chain(1);
Af = A; Af(6,6) = Af(6,6) - 1/tau; Bc = B/tau;
M = expm([Af [Bc E]; zeros(2,8)]*dt); Gw = M(1:6,8);
[~, Mcl] = ctow_impl_rho_B(K, P, Wd, tau, 1);
x = zeros(9,1); s.t = (0:N-1)*dt; s.theta = zeros(1,N); s.p = zeros(1,N);
for k = 1:N
    s.p(k) = x(1); s.theta(k) = x(3)/P.g;
    x = Mcl*x; x(1:6) = x(1:6) + Gw*w_fun(s.t(k));
end
end
