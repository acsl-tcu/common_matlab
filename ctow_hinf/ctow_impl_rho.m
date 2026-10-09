function rho = ctow_impl_rho(K, dt, tau, r, kc)
%CTOW_IMPL_RHO  単機の局所ループ（静的ゲイン v = -K x + kc*c）の実装モデル rho
if nargin < 4 || isempty(r),  r = 1;  end
if nargin < 5 || isempty(kc), kc = 0; end
[A, B] = ctow_chain(r);
rho = ctow_hyb_rho(A, B, -K(:)', kc, dt, tau);
end
