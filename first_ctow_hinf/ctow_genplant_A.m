function G = ctow_genplant_A(P, opt)
%CTOW_GENPLANT_A  設計 A（静的重み）の一般化プラント（連続時間, 協調搬送を見越した設定）
%   外生入力 w = [w_int ; w_Delta]  … どちらも p'' の段（E）から入る
%       w_int  : 吊り点の外力（単機: 風・模型誤差 / 協調: 他機からの相互作用力）
%       w_Delta: 質量比 r の不確かさの入口（w_Delta = delta * z_Delta, |delta|<=1）
%   評価出力 z = [alpha*sqrt(Qshape)*x ; q_th*x3/g ; r0*Wr*x3 ; rho_v*v]
%       最後の r0*Wr*x3 が z_Delta（||T_{wDelta->zDelta}|| < 1 で r in [r_lo, r_hi] のロバスト安定）
%   opt のフィールド（既定値は事前検討で r 保証・実装安定・σ≈0.34 が得られた値）
if nargin < 2, opt = struct(); end
d = struct('alpha',0.01, 'Qshape',[100 800 600 1000 1 1], 'q_th',0, 'b_int',0.3, 'rho_v',0.003);
f = fieldnames(d);
for i = 1:numel(f), if ~isfield(opt, f{i}), opt.(f{i}) = d.(f{i}); end, end
[A, B, E] = ctow_chain(P.r0);
e3 = zeros(1,6); e3(3) = 1;
C_perf = opt.alpha * diag(sqrt(opt.Qshape));
C_th   = opt.q_th * e3 / P.g;
C_D    = P.r0 * P.Wr * e3;
G.A  = A;
G.B1 = [E*opt.b_int, E];
G.B2 = B;
G.C1 = [C_perf; C_th; C_D];
G.idx_zD  = size(G.C1, 1);          % z_Delta の行
G.rho_v   = opt.rho_v;
G.opt     = opt;
end
