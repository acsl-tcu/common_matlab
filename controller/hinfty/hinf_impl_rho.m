function [rho, Mcl] = hinf_impl_rho(K, P, Wd, tau, r)
%HINF_IMPL_RHO  H∞ 制御器（8次: v = -Kx*x - Kw*xw）の実装モデルの rho（旧 ctow_impl_rho_B）
%   実装: 重みフィルタは制御器内で周期 dt の ZOH 離散化 xw+ = Awd*xw + Bwd*(x3/g)
%         c+ = c + dt*v、FC 一次遅れ tau、1 サンプル遅れ
%   K  : 1x8 [Kx(1:6) Kw(7:8)]（制御器は 8 次。下の Mcl の 9 次は c = 制御対象側の FC 指令の状態）
%   Wd : hinf_genplant の出力 G（G.Aw, G.Bw を使う）
if nargin < 4 || isempty(tau), tau = P.tau_r; end
if nargin < 5 || isempty(r), r = 1; end
dt = P.dt; K = K(:)'; Kx = K(1:6); Kw = K(7:8);
[A, B] = hinf_chain(r);
Af = A; Af(6,6) = Af(6,6) - 1/tau; Bc = B/tau;
M = expm([Af Bc; zeros(1,7)]*dt); Phi = M(1:6,1:6); Gc = M(1:6,7);
Md = expm([Wd.Aw Wd.Bw; zeros(1,3)]*dt); Awd = Md(1:2,1:2); Bwd = Md(1:2,3);
e3 = [0 0 1 0 0 0];
% 状態 [x(6); c; xw(2)]
Mcl = zeros(9);
Mcl(1:6,1:6) = Phi;  Mcl(1:6,7) = Gc;
Mcl(7,1:6) = -dt*Kx; Mcl(7,7) = 1; Mcl(7,8:9) = -dt*Kw;
Mcl(8:9,1:6) = Bwd*e3/P.g; Mcl(8:9,8:9) = Awd;
rho = max(abs(eig(Mcl)));
end