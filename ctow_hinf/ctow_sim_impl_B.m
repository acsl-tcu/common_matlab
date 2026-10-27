function s = ctow_sim_impl_B(K, P, Wd, w_fun, T, tau)
%CTOW_SIM_IMPL_B  設計 B の実装モデルの時間応答（外力 w_fun(t) のみ、初期値 0）
%   制御器 : 25 ms ごとに v = -Kx*x - Kw*xw を計算し、角速度指令 c を c + dt*v に更新。
%            重みフィルタの状態 xw も制御器の中で更新する（入力は x3/g）。
%   制御対象: FC（一次遅れ tau）＋6次チェーン。周期 dt の間は c 一定（ZOH）。
%            今周期に計算した v は次周期の c として効く（1 サンプル遅れ）。
%   ctow_impl_rho_B の閉ループ行列 Mcl を繰り返し掛けるのと同じ計算を、手順を分けて書いたもの。
if nargin < 5, T = 40; end
if nargin < 6, tau = P.tau_r; end
dt = P.dt; N = round(T/dt);
K = K(:)'; Kx = K(1:6); Kw = K(7:8);

% ---- 制御対象（FC 一次遅れ＋チェーン）の ZOH 離散化 ----
[A, B, E] = ctow_chain(1);
Af = A; Af(6,6) = Af(6,6) - 1/tau; Bc = B/tau;
M = expm([Af [Bc E]; zeros(2,8)]*dt);
Phi = M(1:6,1:6); Gc = M(1:6,7); Gw = M(1:6,8);

% ---- 制御器の中の重みフィルタの離散化 ----
Md = expm([Wd.Aw Wd.Bw; zeros(1,3)]*dt);
Awd = Md(1:2,1:2); Bwd = Md(1:2,3);

x = zeros(6,1);    % 制御対象の状態
c = 0;             % 角速度指令（制御器が持つ積分器）
xw = zeros(2,1);   % 重みフィルタの状態（制御器内）
s.t = (0:N-1)*dt; s.p = zeros(1,N); s.theta = zeros(1,N); s.v = zeros(1,N); s.c = zeros(1,N);
for k = 1:N
    t = s.t(k);
    % --- 制御器: 入力を計算 ---
    v = -Kx*x - Kw*xw;
    s.p(k) = x(1); s.theta(k) = x(3)/P.g; s.v(k) = v; s.c(k) = c;
    % --- 制御対象: 現在の指令 c と外力 w で 1 周期進める ---
    x_next = Phi*x + Gc*c + Gw*w_fun(t);
    % --- 制御器: 内部状態を更新（v は次周期の c になる＝1 サンプル遅れ）---
    c  = c + dt*v;
    xw = Awd*xw + Bwd*(x(3)/P.g);
    x  = x_next;
end
end
