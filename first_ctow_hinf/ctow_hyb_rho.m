function [rho, Mcl] = ctow_hyb_rho(A, Bm, Kx, kc, dt, tau)
%CTOW_HYB_RHO  実装準拠の離散閉ループのスペクトル半径
%   連続プラント x' = A x + Bm c（Bm の非零行 = 角速度レベルの状態）
%   実装: 周期 dt ごとに v = Kx*x + kc*c を計算し、c <- c + dt*v（v 一定の積分）
%         FC は c を一次遅れ tau で追従（該当行に -1/tau を加える）
%         c_k はプラントに 1 サンプル遅れて効く
%   Kx は v = Kx*x の係数（v = -K x なら Kx = -K）。rho < 1 で安定。
n = size(A,1); m = size(Bm,2);
Af = A; Bn = Bm / tau;
for j = 1:m
    i = find(Bm(:,j) ~= 0, 1);
    Af(i,i) = Af(i,i) - 1/tau;
end
M   = expm([Af Bn; zeros(m, n+m)] * dt);
Phi = M(1:n, 1:n);
Gam = M(1:n, n+1:end);
Mcl = [Phi Gam; dt*Kx (1 + dt*kc)*eye(m)];
rho = max(abs(eig(Mcl)));
end
