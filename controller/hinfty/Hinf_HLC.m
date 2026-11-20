function v = Hinf_HLC(xi, dt, k_th,gravity)
% x,y 方向の仮想入力 v を H∞ 制御器で求める（階層型線形化の vs(1:2) を上書きする用）
% ctow_sim_impl_B の「制御器」部分（v を計算し、重みフィルタの状態 xw を更新する）を毎周期1回分行う。
%
%   xi   : 2x6  [x; y] 行ごとの ξ = 積荷位置の誤差の鎖（実際 - 目標、位置〜5階微分。ξ3 が加速度）
%   dt   : 制御周期 [s]
%   k_th : W_theta の重み（hinf_genplant に渡す）。最初の呼び出しだけで必要 ctowで確認したものを採用する
%   v    : 1x2  [vx, vy]
%
%   ctow_sim_impl_B と同じ（8次の拡張状態 xa = [ξ(6); xw(2)]）:
%       v  = -Kx*ξ - Kw*xw                 （K = [Kx(1:6) Kw(7:8)]）
%       xw = Awd*xw + Bwd*(ξ3/g)           （v を出した後に更新。Awd,Bwd は expm による ZOH 離散化）
%   K は最初の呼び出しで設計（hinf_genplant → hinf_gamma_opt → hlinf_hinf_sf, γ=1.05*γopt）。
%   リセット:  Hinf_HLC([])   または   clear Hinf_HLC
persistent Kx Kw Awd Bwd g xw dtd Aw Bw
if isempty(xi)       % リセット
    Kx = []; return
end
% --- 設計（最初の1回） ---
if isempty(Kx)
    if nargin < 3, error('Hinf_HLC: 最初の呼び出しでは k_th を指定してください'); end
    P = hinf_params(gravity,dt);  %パラメータを設定
    G = hinf_genplant(P, struct('k_th', k_th)); %プラントを設計
    gopt  = hinf_gamma_opt(G.A, G.B1, G.B2, G.C1, G.rho_v); %plantから作られる最小のγを算出
    % gamma = 1.05*gopt;
    if gopt < 1 %gammaの設定
        gamma = max(1.05*gopt, min(1, 1.5*gopt));
    else
        gamma = 1.5*gopt;
    end
        [K, ~, ok] = hinf_hinf_sf(G.A, G.B1, G.B2, G.C1, G.rho_v, gamma); %算出したγでH∞でのゲインを設計
    assert(ok, 'Hinf_HLC: hlinf_hinf_sf が解を返しませんでした');
    K = K(:)';
   %  % ---- 安定判断（実装モデル）----　設計とはあまり関係なし
   %  rho   = arrayfun(@(t) hinf_impl_rho(K, P, G, t), P.tau_list);   % 遅れ 10/30/50 ms
   %  sigma = -log(rho(2))/P.dt;
   %  fprintf('[Hinf_HLC] k_th=%g gopt=%.3f gamma=%.3f rho=[%.4f %.4f %.4f] sigma=%.3f\n', ...
   %      k_th, gopt, gamma, rho, sigma);
   %  if any(rho >= 1)
   %      error('Hinf_HLC: 実装モデルで不安定 (rho>=1)。gamma や重みを見直すこと。');
   %  end
   %  if sigma < 0.3
   %      warning('Hinf_HLC: sigma=%.3f < 0.3（応答が遅い）', sigma);
   %  end
   % 
   % % ---- TD, Twth（質量比の保証と振れ角） ----設計とはあまり関係なし
   %  e3 = [0 0 1 0 0 0];
   %  [A1,  B, E] = hinf_chain(1);
   %  [Ar0, ~, ~] = hinf_chain(P.r0);
   %  Aa1 = [A1  zeros(6,2); G.Bw*e3/P.g  G.Aw];
   %  Aa0 = [Ar0 zeros(6,2); G.Bw*e3/P.g  G.Aw];
   %  Ba = [B; 0; 0];  Ea = [E; 0; 0];
   %  TD   = norm(ss(Aa0 - Ba*K, Ea, [P.r0*P.Wr*e3 0 0], 0), Inf);
   %  Twth = norm(ss(Aa1 - Ba*K, Ea, [e3/P.g 0 0], 0), Inf);
   %  if TD >= 1
   %      error('Hinf_HLC: TD >= 1（質量比の保証が成り立たない）');
   %  end
   %  fprintf('[Hinf_HLC] k_th=%g gopt=%.3f gamma=%.3f TD=%.3f Twth=%.4f rho=[%.4f %.4f %.4f] sigma=%.3f\n', ...
   %      k_th, gopt, gamma, TD, Twth, rho, sigma);

    
    Kx = K(1:6); Kw = K(7:8);          % 8次: [チェーン6, 重みxw 2] ゲインk
    Aw = G.Aw; Bw = G.Bw; g = P.g;
    xw = zeros(2,2);                   % 列: x, yの角度
    dtd = [];
end
% ---制御器の中の重みフィルタの離散化  ---
if isempty(dtd) || abs(dtd - dt) > 1e-12
    Md  = expm([Aw Bw; zeros(1,3)]*dt);
    Awd = Md(1:2,1:2); Bwd = Md(1:2,3);
    dtd = dt;
end
% --- v を求める → xw を更新（ctow_sim_impl_B のループ1回分） ---
v = zeros(1,2); %初期の入力は0
for ax = 1:2    % x, y
    v(ax)    = -Kx*xi(ax,:)' - Kw*xw(:,ax); % --- 制御器: 入力を計算 ---
    xw(:,ax) = Awd*xw(:,ax) + Bwd*(xi(ax,3)/g); %揺れやすい成分を保管
end
end