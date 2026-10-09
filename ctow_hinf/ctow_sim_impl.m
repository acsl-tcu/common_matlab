function s = ctow_sim_impl(K, P, scen, kc)
%CTOW_SIM_IMPL  実装準拠（25 ms, v一定積分, 1サンプル遅れ, FC一次遅れ）の時間応答
%   scen のフィールド: T, p_ref(t関数), w(t関数), r, tau, noise(1x6 の標準偏差)
if nargin < 4, kc = 0; end
d = struct('T',40, 'p_ref',@(t) 0*t, 'w',@(t) 0*t, 'r',1, 'tau',P.tau_r, 'noise',zeros(1,6));
f = fieldnames(d);
for i = 1:numel(f), if ~isfield(scen, f{i}), scen.(f{i}) = d.(f{i}); end, end
dt = P.dt; N = round(scen.T/dt);
[A, B, E] = ctow_chain(scen.r);
Af = A; Af(6,6) = Af(6,6) - 1/scen.tau; Bc = B/scen.tau;
M = expm([Af [Bc E]; zeros(2, 8)] * dt);
Phi = M(1:6,1:6); Gc = M(1:6,7); Gw = M(1:6,8);
x = zeros(6,1); c = 0;
s.t = (0:N-1)*dt; s.p = zeros(1,N); s.theta = zeros(1,N); s.c = zeros(1,N); s.v = zeros(1,N);
for k = 1:N
    t = s.t(k);
    xr = [scen.p_ref(t); zeros(5,1)];
    xm = x + scen.noise(:).*randn(6,1);
    v  = -K(:)'*(xm - xr) + kc*c;
    s.p(k) = x(1); s.theta(k) = x(3)/P.g; s.c(k) = c; s.v(k) = v;
    x = Phi*x + Gc*c + Gw*scen.w(t);
    c = c + dt*v;                         % 次周期に効く（1 サンプル遅れ）
end
end
