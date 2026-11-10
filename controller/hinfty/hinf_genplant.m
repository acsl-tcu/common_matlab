function G = hinf_genplant(P, opt)
% 元: ctow_genplant_B   設計 B（振れ角の周波数重み W_theta 付き、8 次）の一般化プラント
%   W_theta(s) = k_th * 2*z_w*w_w*s / (s^2 + 2*z_w*w_w*s + w_w^2)  を振れ角 x3/g に加える。
%   拡張状態 xa = [x(6); xw(2)]（8 次）に対する状態フィードバック v = -K*xa で設計する。
%   z = [alpha*sqrt(Qshape)*x ; W_theta(x3/g) ; r0*Wr*x3 ; rho_v*v]
if nargin < 2, opt = struct(); end
d = struct('alpha',0.01, 'Qshape',[100 800 600 1000 1 1], 'b_int',0.3, 'rho_v',0.003, ...
    'k_th',3, 'w_w',2.5, 'z_w',0.7);
f = fieldnames(d);
for i = 1:numel(f), if ~isfield(opt, f{i}), opt.(f{i}) = d.(f{i}); end, end

[A, B, E] = hinf_chain(P.r0);
[Aw, Bw, Cw] = hinf_wtheta(opt);
e3 = [0 0 1 0 0 0];                     % x3（θ = x3/g、zΔ）
G.A  = [A         zeros(6,2);
        Bw*e3/P.g Aw        ];          % x3/g を xw の入力に
G.B1 = [E*opt.b_int, E; zeros(2,2)];
G.B2 = [B; zeros(2,1)];
G.C1 = [opt.alpha*diag(sqrt(opt.Qshape)) zeros(6,2);   % zperf（6 行）
        zeros(1,6) opt.k_th*Cw;                        % zθ
        P.r0*P.Wr*e3 zeros(1,2)];                      % zΔ（最後の行）
G.rho_v = opt.rho_v;                                   % zv = rho_v*v
G.opt = opt;  G.Aw = Aw;  G.Bw = Bw;  G.Cw = Cw;
end