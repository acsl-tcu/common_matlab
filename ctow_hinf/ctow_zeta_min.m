function z = ctow_zeta_min(K, dt, tau, kc)
%CTOW_ZETA_MIN  全ての構造共振 ws で 2セット結合系が安定になる最小の構造減衰比
%   協調搬送への橋渡しの指標。参照値: G1 = 0.0347, G2 = 0.0096（dt=25ms, tau=30ms）
if nargin < 4, kc = 0; end
WS = logspace(log10(0.3), log10(pi/dt*0.9), 45);
stab = @(zs) all(arrayfun(@(w) ctow_two_set_rho(K, w, zs, dt, tau, kc) < 1, WS));
lo = 1e-4; hi = 0.3;
if stab(lo), z = lo; return; end
if ~stab(hi), z = hi; return; end
for it = 1:9
    mid = sqrt(lo*hi);
    if stab(mid), hi = mid; else, lo = mid; end
end
z = hi;
end
