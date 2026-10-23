function [K, X, ok] = ctow_hinf_sf(A, B1, B2, C1, rho_v, gamma)
%CTOW_HINF_SF  連続時間の状態フィードバック H∞（DGKF 型）
%   一般化プラント  x' = A x + B1 w + B2 v,  z = [C1 x ; rho_v v]
%   ||T_zw||_inf < gamma を満たす v = -K x を返す（解なしなら ok = false）
%   リカッチ: A'X + XA + X(gamma^-2 B1 B1' - B2n B2n')X + C1'C1 = 0,  B2n = B2/rho_v
B2n = B2 / rho_v;
X = ctow_are_ham(A, gamma^-2*(B1*B1') - B2n*B2n', C1'*C1);
if isempty(X)
    K = []; ok = false; return;
end
K  = (B2n' * X) / rho_v;
ok = true;
end
