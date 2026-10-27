function G = ctow_genplant_B(P, opt)
%CTOW_GENPLANT_B  設計 B（振れ角の周波数重み W_theta 付き）の一般化プラント
%   設計 A の一般化プラントに、振れ角 x3/g を入力とする 2 次バンドパス
%       W_theta(s) = k_th * 2*z_w*w_w*s / (s^2 + 2*z_w*w_w*s + w_w^2)
%   を加える。重みの状態 xw は計測できる信号（x3）を通すので制御器が内部で計算でき、
%   拡張状態 xa = [x; xw]（8 次）に対する状態フィードバック v = -K*xa で設計できる。
%   z = [alpha*sqrt(Qshape)*x ; W_theta(x3/g) ; r0*Wr*x3 ; rho_v*v]
if nargin < 2, opt = struct(); end
d = struct('alpha',0.01, 'Qshape',[100 800 600 1000 1 1], 'b_int',0.3, 'rho_v',0.003, ...
           'k_th',3, 'w_w',2.5, 'z_w',0.7,"k_I",0.01);
f = fieldnames(d);
for i = 1:numel(f), if ~isfield(opt, f{i}), opt.(f{i}) = d.(f{i}); end, end
% 8次の拡張系
[A, B, E] = ctow_chain(P.r0);
% e3 = [0 0 1 0 0 0];
% [Aw, Bw, Cw] = ctow_wtheta(opt);
% G.A  = [A zeros(6,2); Bw*e3/P.g Aw];
% G.B1 = [E*opt.b_int, E; zeros(2,2)];
% G.B2 = [B; zeros(2,1)];
% G.C1 = [opt.alpha*diag(sqrt(opt.Qshape)) zeros(6,2);
%         zeros(1,6) opt.k_th*Cw;
%         P.r0*P.Wr*e3 zeros(1,2)];
% G.rho_v = opt.rho_v;
% G.opt   = opt;
% G.Aw = Aw; G.Bw = Bw; G.Cw = Cw;　

%積分器を追加した9次のもの
[Aw, Bw, Cw] = ctow_wtheta(opt);
e1 = [1 0 0 0 0 0];                     % p（x_I' = p）
e3 = [0 0 1 0 0 0];                     % x3（θ = x3/g、zΔ）

G.A  = [A         zeros(6,2) zeros(6,1);    % 元のプラント（6 次）
        Bw*e3/P.g Aw         zeros(2,1);    % x3/g を x_W の入力に
        e1        zeros(1,2) 0          ];  % x_I' = p
G.B1 = [E*opt.b_int, E; zeros(3,2)];
G.B2 = [B; zeros(3,1)];
G.C1 = [opt.alpha*diag(sqrt(opt.Qshape)) zeros(6,3);   % zperf（6 行）
        zeros(1,6) opt.k_th*Cw           0;            % zθ
        zeros(1,8)                       opt.k_I;      % zI（積分の重み）
        P.r0*P.Wr*e3 zeros(1,3)];                      % zΔ（最後の行）
G.rho_v = opt.rho_v;                                   % zv = rho_v*v
G.opt = opt;  G.Aw = Aw;  G.Bw = Bw;  G.Cw = Cw;
end
