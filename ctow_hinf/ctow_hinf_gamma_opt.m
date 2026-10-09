function gopt = ctow_hinf_gamma_opt(A, B1, B2, C1, rho_v)
%CTOW_HINF_GAMMA_OPT  状態フィードバック H∞ の最適 gamma を二分探索
lo = 1e-3; hi = 1e4;
[~, ~, ok] = ctow_hinf_sf(A, B1, B2, C1, rho_v, hi);
if ~ok, error('gamma = %g でも解けない。重みを見直すこと。', hi); end
for it = 1:80
    mid = sqrt(lo*hi);
    [~, ~, ok] = ctow_hinf_sf(A, B1, B2, C1, rho_v, mid);
    if ok, hi = mid; else, lo = mid; end
end
gopt = hi;
end
