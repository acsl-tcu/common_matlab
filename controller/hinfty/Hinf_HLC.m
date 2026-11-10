function v = Hinf_HLC(xi, dt, k_th,gravity)
% x,y 方向の仮想入力 v を H∞ 制御器で求める（階層型線形化の vs(1:2) を上書きする用）
% ctow_sim_impl_B の「制御器」部分（v を計算し、重みフィルタの状態 xw を更新する）を毎周期1回分行う。
%
%   xi   : 2x6  [x; y] 行ごとの ξ = 積荷位置の誤差の鎖（実際 - 目標、位置〜5階微分。ξ3 が加速度）
%   dt   : 制御周期 [s]
%   k_th : W_theta の重み（hinf_genplant に渡す）。最初の呼び出しだけで必要
%   v    : 1x2  [vx, vy]
%
%   ctow_sim_impl_B と同じ（8次の拡張状態 xa = [ξ(6); xw(2)]）:
%       v  = -Kx*ξ - Kw*xw                 （K = [Kx(1:6) Kw(7:8)]）
%       xw = Awd*xw + Bwd*(ξ3/g)           （v を出した後に更新。Awd,Bwd は expm による ZOH 離散化）
%   K は最初の呼び出しで設計（hinf_genplant → hinf_gamma_opt → hlinf_hinf_sf, γ=1.05*γopt）。
%   使う関数は下にローカル関数として列挙（元の ctow_* に対応）。
%   リセット:  Hinf_HLC([])   または   clear Hinf_HLC
persistent Kx Kw Awd Bwd g xw dtd Aw Bw
if isempty(xi)       % リセット
    Kx = []; return
end
% --- 設計（最初の1回） ---
if isempty(Kx)
    if nargin < 3, error('Hinf_HLC: 最初の呼び出しでは k_th を指定してください'); end
    P = hinf_params(gravity,dt);
    G = hinf_genplant(P, struct('k_th', k_th));
    gopt  = hinf_gamma_opt(G.A, G.B1, G.B2, G.C1, G.rho_v);
    gamma = 1.05*gopt;
    [K, ~, ok] = hinf_hinf_sf(G.A, G.B1, G.B2, G.C1, G.rho_v, gamma);
    assert(ok, 'Hinf_HLC: hlinf_hinf_sf が解を返しませんでした');
    K = K(:)';
    Kx = K(1:6); Kw = K(7:8);          % 8次: [チェーン6, 重みxw 2]
    Aw = G.Aw; Bw = G.Bw; g = P.g;
    xw = zeros(2,2);                   % 列: x, y
    dtd = [];
end
% --- 重みフィルタの離散化（dt が変わったときも再計算） ---
if isempty(dtd) || abs(dtd - dt) > 1e-12
    Md  = expm([Aw Bw; zeros(1,3)]*dt);
    Awd = Md(1:2,1:2); Bwd = Md(1:2,3);
    dtd = dt;
end
% --- v を求める → xw を更新（ctow_sim_impl_B のループ1回分） ---
v = zeros(1,2);
for ax = 1:2    % x, y
    v(ax)    = -Kx*xi(ax,:)' - Kw*xw(:,ax);
    xw(:,ax) = Awd*xw(:,ax) + Bwd*(xi(ax,3)/g);
end
end