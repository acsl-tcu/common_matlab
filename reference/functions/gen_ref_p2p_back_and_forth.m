function xdt = gen_ref_p2p_back_and_forth(varargin)
% Smooth (piecewise-free) P2P back-and-forth reference.
% Works with gen_ref_for_rigid_body: symbolic diff + matlabFunction.

p = inputParser;
p.addParameter("p0", [0;0;3], @(x)isnumeric(x)&&numel(x)==3);
p.addParameter("p1", [2;2;3], @(x)isnumeric(x)&&numel(x)==3);
p.addParameter("t_go",   2.0, @(x)isnumeric(x)&&x>0);
p.addParameter("t_hold", 0.5, @(x)isnumeric(x)&&x>=0);
p.addParameter("t_back", 2.0, @(x)isnumeric(x)&&x>0);
p.addParameter("t_start",0.0, @(x)isnumeric(x)&&x>=0);
p.addParameter("k", 20.0, @(x)isnumeric(x)&&x>0); % スイッチの鋭さ（大きいほど区間っぽくなる）
p.parse(varargin{:});
prm = p.Results;

p0 = prm.p0(:);
p1 = prm.p1(:);

t0 = prm.t_start;
t1 = t0 + prm.t_go;
t2 = t1 + prm.t_hold;
t3 = t2 + prm.t_back;

k  = prm.k;

syms tau real

% 0→1に滑らかに切り替えるスイッチ（piecewise無し）
H = @(x) (1 + tanh(k*x))/2;  % ≈ heaviside

% 区間ゲート（滑らか）
g1 = H(tau - t0) - H(tau - t1); % A→B
g2 = H(tau - t1) - H(tau - t2); % hold at B
g3 = H(tau - t2) - H(tau - t3); % B→A
g0 = 1 - H(tau - t0);           % before start
g4 = H(tau - t3);               % after back

% 5次 time-scaling（0→1、端で速度・加速度0）
s5 = @(u) (10*u^3 - 15*u^4 + 6*u^5);

u1 = (tau - t0)/(t1 - t0);
u2 = (tau - t2)/(t3 - t2);

s1 = s5(u1);
s2 = s5(u2);

pAB = (1-s1)*p0 + s1*p1;
pBA = (1-s2)*p1 + s2*p0;

% 位置：ゲートで合成（全部通常関数なので matlabFunction が通る）
pos = g0*p0 + g1*pAB + g2*p1 + g3*pBA + g4*p0;

expr = [pos; sym(0)]; % [x;y;z;yaw]

% sym入力→symを返す / 数値→double
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
