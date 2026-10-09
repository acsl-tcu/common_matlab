function rho = ctow_impl_rho_dyn(Kd, P, tau, r)
%CTOW_IMPL_RHO_DYN  離散動的制御器 Kd（入力 y=[x; c] 7次元, 出力 v, Ts=dt）の実装モデル rho
%   設計 B（重み状態を持つ制御器）や設計 C（hinfsyn の結果）の評価用。
%   静的ゲインの場合 Kd = ss([],[],[],[-K kc],dt) で ctow_impl_rho と一致する（ctow_selftest で確認）。
%   注意: hinfsyn の制御器は u = K y（正帰還）の約束で返る。
if nargin < 3 || isempty(tau), tau = P.tau_r; end
if nargin < 4 || isempty(r), r = 1; end
dt = P.dt;
[A, B] = ctow_chain(r);
Af = A; Af(6,6) = Af(6,6) - 1/tau; Bc = B/tau;
M = expm([Af Bc; zeros(1, 7)] * dt);
Phi = M(1:6,1:6); Gc = M(1:6,7);
Ad = [Phi Gc; zeros(1,6) 1];
Bd = [zeros(6,1); dt];
Pd = ss(Ad, Bd, eye(7), zeros(7,1), dt);
CL = feedback(Pd, Kd, +1);
rho = max(abs(pole(CL)));
end
