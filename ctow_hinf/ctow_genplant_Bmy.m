function G = ctow_genplant_Bmy(P, opt)
%CTOW_GENPLANT_A  設計 A（静的重み）の一般化プラント（連続時間, 協調搬送を見越した設定）
%   外生入力 w = [w_int ; w_Delta]  … どちらも p'' の段（E）から入る
%       w_int  : 吊り点の外力（単機: 風・模型誤差 / 協調: 他機からの相互作用力）
%       w_Delta: 質量比 r の不確かさの入口（w_Delta = delta * z_Delta, |delta|<=1）
%   評価出力 z = [alpha*sqrt(Qshape)*x ; q_th*x3/g ; r0*Wr*x3 ; rho_v*v]
%       最後の r0*Wr*x3 が z_Delta（||T_{wDelta->zDelta}|| < 1 で r in [r_lo, r_hi] のロバスト安定）
%   opt のフィールド（既定値は事前検討で r 保証・実装安定・σ≈0.34 が得られた値）
if nargin < 2, opt = struct(); end
d = struct('alpha',0.01, 'Qshape',[100 800 600 1000 1 1], 'b_int',0.3, 'rho_v',0.003, ...
           'kW',3.0, 'w0',2.5, 'zeta',1.0, 'kI',0.7);
f = fieldnames(d);
for i = 1:numel(f), if ~isfield(opt, f{i}), opt.(f{i}) = d.(f{i}); end, end

[A, B, E] = ctow_chain(P.r0);
nx = 6;
e3 = zeros(1,6); e3(3) = 1;

% W_theta（2 次バンドパス）。kW は Cw に掛ける y=cxの方に設定する
Wth0 = tf([2*opt.zeta*opt.w0 0], [1 2*opt.zeta*opt.w0 opt.w0^2]);
[Aw, Bw, Cw, ~] = ssdata(ss(Wth0));
Cw = opt.kW * Cw;
nw = size(Aw,1);


C_perf = opt.alpha * diag(sqrt(opt.Qshape));
C_D    = P.r0 * P.Wr * e3;
B1     = [E*opt.b_int, E];

% W_theatの追加 　元のplantの式とx_aをまとめる
G.A  = [A,            zeros(nx,nw); 
        Bw*e3/P.g,    Aw];
G.B1 = [B1; zeros(nw, size(B1,2))];
G.B2 = [B;  zeros(nw, size(2,2))];
G.C1 = [C_perf,         zeros(nx,nw);
        zeros(1,nx),    Cw;
        C_D,            zeros(1,nx*0+nw)];
G.idx_zD = size(G.C1, 1);

%W_theatと定常偏差の追加 定常偏差を位置に追加
% G.A  = [A,            zeros(nx,nw),   zeros(nx,1);
%         Bw*e3/P.g,    Aw,             zeros(nw,1);
%         [1 0 0 0 0 0], zeros(1,nw),   0];
% 
% G.B1 = [B1; 0 0; 0 0; 0 0];
% G.B2 = [B;  0;   0;   0];
% 
% G.C1 = [C_perf,        zeros(nx,nw),   zeros(nx,1);
%         zeros(1,nx),   Cw,             0;
%         zeros(1,nx),   zeros(1,nw),    opt.kI;
%         C_D,           zeros(1,nw),    0];
% G.idx_zD = size(G.C1, 1);

G.rho_v  = opt.rho_v;
G.nx = nx;  G.nw = nw;
G.Aw = Aw;  G.Bw = Bw;  G.Cw = Cw;  G.e3 = e3;
G.e1 = [1 0 0 0 0 0];   
G.opt = opt;

end
