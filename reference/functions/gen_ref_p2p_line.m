function xdt = gen_ref_p2p_line(varargin)
% Smooth (piecewise-free) P2P single-direction line reference.
% Works with gen_ref_for_rigid_body: symbolic diff + matlabFunction.

p = inputParser;
p.addParameter("p0", [0;0;3], @(x)isnumeric(x)&&numel(x)==3); % 初期位置
p.addParameter("p1", [2;2;3], @(x)isnumeric(x)&&numel(x)==3); % 最終行
p.addParameter("t_go",   5.0, @(x)isnumeric(x)&&x>0); % 移動時間 (デフォルト5秒)
p.addParameter("t_start",0.0, @(x)isnumeric(x)&&x>=0);
p.addParameter("k", 10.0, @(x)isnumeric(x)&&x>0); % スイッチの鋭さ
p.parse(varargin{:});
prm = p.Results;

p0 = prm.p0(:);
p1 = prm.p1(:);

t0 = prm.t_start;
t1 = t0 + prm.t_go;

k  = prm.k;

syms tau real

% 既存の滑らかなヘビサイド近似スイッチ
H = @(x) (1 + tanh(k*x))/2;

% 区間ゲート（往復ではなく、移動してホールドするためのゲートに修正）
g1 = H(tau - t0) - H(tau - t1); % A→B移動区間
g0 = 1 - H(tau - t0);           % スタート前区間
g2 = H(tau - t1);               % B到達後のホールド区間

% 5次 time-scaling（端点で速度・加速度が滑らかに0になる既存の関数）
s5 = @(u) (10*u^3 - 15*u^4 + 6*u^5);

u1 = (tau - t0)/(t1 - t0);
s1 = s5(u1);

% 移動中の直線軌道
pAB = (1-s1)*p0 + s1*p1;

% 位置：ゲートで合成（matlabFunction に通せる通常の組み込み関数のみで構成）
gs = g0 + g1 + g2;
pos = (g0*p0 + g1*pAB + g2*p1) / gs;

expr = [pos; sym(0)]; % [x;y;z;yaw]

% sym入力→symを返す / 数値→double の既存の仕組みに完全準拠
xdt = @(t) local_eval(expr, tau, t);
end

function y = local_eval(expr, tau, t)
v = subs(expr, tau, t);
if isa(t, "sym")
    y = v;
else
    y = double(v);
end
end