function [A_cbf, b_cbf] = CBF_Constraints_MRD(obj, x, xd, vf, obs_params, sphere_params, gamma_params, P, alpha2, beta2)
% CBF_Constraints_MRD
% 代数的バックステッピングを用いて、推力(u1)とトルク(u2,u3,u4)に対する
% 混合相対次数(MRD)のCBF制約 A_cbf * [u1; u2; u3; u4] >= b_cbf を生成する。

    % 1. 状態のパース (x = [q(4); w(3); pL(3); vL(3); pT(3); wL(3)])
    % q  = x(1:4);
    % w  = x(5:7);
    pL = x(8:10);
    vL = x(11:13);
    pT = x(14:16);
    wL = x(17:19);

    % パラメータのパース (P = [mass, jx, jy, jz, gravity, loadmass, cableL, 0, 0])
    m  = P(1);
    mL = P(6);
    L  = P(7);
    g  = P(5);

    % 障害物パラメータ
    xo = obs_params(1);
    yo = obs_params(2);
    zo = obs_params(3);
    ro = obs_params(4);
    p_obs = [xo; yo; zo];

    % 被覆球体パラメータ
    lambda = sphere_params(1);
    r_l    = sphere_params(2);

    % ECBF ゲイン
    k1 = gamma_params(1);
    k2 = gamma_params(2);

    % 2. Symbolic Math で生成した関数の呼び出し
    % A_f = [A_fx, A_fy, A_fz], b_core, h, h_dot
    [A_f, b_core, h, h_dot] = CBF_Constraints_MRD_Core(pL, vL, pT, wL, p_obs, ro, r_l, lambda, m, mL, L, g, k1, k2);

    % 3. 代数的バックステッピング (HLCマッピング)
    % A_f は機体推力ベクトル f = [fx; fy; fz] に対する係数。
    % HLCの構造において:
    % fz ≒ u1
    % [fx; fy] ≒ m * [vs_x; vs_y]
    % ここで、vs = beta2 * [u2; u3; u4] + alpha2
    
    A_fz = A_f(3);
    A_fxy = [m * A_f(1), m * A_f(2), 0]; % yaw(vsz)成分への影響は0とする
    
    % u_tau = [u2; u3; u4] に対する係数
    A_tau = A_fxy * beta2;
    
    % 統合係数行列 A_cbf = [A_u1, A_u2, A_u3, A_u4]
    A_cbf = [A_fz, A_tau];
    
    % b_cbf の計算
    % b_core から alpha2 の影響を引く
    b_cbf = b_core - A_fxy * alpha2(:);

end
