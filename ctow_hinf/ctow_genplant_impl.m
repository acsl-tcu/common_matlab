function Pg = ctow_genplant_impl(P, opt)
%CTOW_GENPLANT_IMPL  設計 C 用：実装準拠の離散時間一般化プラント（hinfsyn 用の雛形）
%   状態 s = [x(6, x6 は FC 一次遅れ); c]、制御入力 v、周期 dt、1 サンプル遅れは構造上含まれる
%   入力: [w_int; w_Delta; n(7, 計測ノイズ eps); v]
%   出力: [z; y],  z = [alpha*sqrt(Q)*x ; q_th*x3/g ; r0*Wr*x3 ; rho_v*v],  y = s + eps*n
%   使い方の例:  Pg = ctow_genplant_impl(P);  [Kd,CL,gam] = hinfsyn(Pg, 7, 1);
%               rho = ctow_impl_rho_dyn(Kd, P);
%   ※ y にノイズ eps を入れるのは hinfsyn の正則性条件（D21 フルランク）を満たすため
if nargin < 2, opt = struct(); end
d = struct('alpha',0.01, 'Qshape',[100 800 600 1000 1 1], 'q_th',0, 'b_int',0.3, 'rho_v',0.003, 'eps',1e-3, 'tau',P.tau_r);
f = fieldnames(d);
for i = 1:numel(f), if ~isfield(opt, f{i}), opt.(f{i}) = d.(f{i}); end, end
dt = P.dt;
[A, B, E] = ctow_chain(P.r0);
Af = A; Af(6,6) = Af(6,6) - 1/opt.tau; Bc = B/opt.tau;
M = expm([Af [Bc E]; zeros(2, 8)] * dt);
Phi = M(1:6,1:6); Gc = M(1:6,7); Gw = M(1:6,8);
Ad = [Phi Gc; zeros(1,6) 1];
Bw = [Gw*opt.b_int, Gw; 0 0];          % w_int, w_Delta
Bn = zeros(7,7);                        % 計測ノイズは状態に入らない
Bv = [zeros(6,1); dt];
e3 = [0 0 1 0 0 0];
Cz = [opt.alpha*diag(sqrt(opt.Qshape)) zeros(6,1);
      opt.q_th*e3/P.g 0;
      P.r0*P.Wr*e3 0;
      zeros(1,7)];
Dzv = [zeros(size(Cz,1)-1,1); opt.rho_v];
Cy = eye(7);
Dyn = opt.eps*eye(7);
Bg = [Bw Bn Bv];
Cg = [Cz; Cy];
Dg = [zeros(size(Cz,1),2) zeros(size(Cz,1),7) Dzv;
      zeros(7,2)          Dyn                 zeros(7,1)];
Pg = ss(Ad, Bg, Cg, Dg, dt);
end
