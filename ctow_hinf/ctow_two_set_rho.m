function rho = ctow_two_set_rho(K, ws, zs, dt, tau, kc)
%CTOW_TWO_SET_RHO  2セットを弾性（相対モード固有角周波数 ws, 減衰比 zs）で結合した系の rho
%   w_1 = k(p2-p1) + c(p2'-p1'), w_2 = -w_1,  k = ws^2/2, c = zs*ws（質量で正規化）
if nargin < 6, kc = 0; end
[A, ~, E] = ctow_chain(1); n = 6;
e1 = zeros(1,n); e1(1) = 1;
e2 = zeros(1,n); e2(2) = 1;
S  = E * ((ws^2/2)*e1 + zs*ws*e2);
A2 = blkdiag(A, A);
i1 = 1:n; i2 = n+1:2*n;
A2(i1,i1) = A2(i1,i1) - S;  A2(i1,i2) = A2(i1,i2) + S;
A2(i2,i2) = A2(i2,i2) - S;  A2(i2,i1) = A2(i2,i1) + S;
B2 = zeros(2*n, 2); B2(n,1) = 1; B2(2*n,2) = 1;
Kxx = zeros(2, 2*n); Kxx(1,i1) = -K(:)'; Kxx(2,i2) = -K(:)';
rho = ctow_hyb_rho(A2, B2, Kxx, kc, dt, tau);
end
