function ref = gen_ref_star(param)
% gen_ref_star  星形（5芒星）軌跡の参照関数
%
% デフォルトは gen_ref_heart と同一スケール。
% 範囲・周期はすべて外部から引数で制御する。
%
% 使用例（外部から範囲を指定）:
%   agent.reference.time_var = TIME_VARYING_REFERENCE(agent, ...
%       {"gen_ref_star", {"freq",10,"orig",[0 0 1],"size",[1 1 0],"phase",pi/2}});
%
%   → 範囲を縮小したい場合は size=[0.5 0.5 0] に変更する
%   → 周期を遅くしたい場合は freq=20 に変更する
%   → 星の頂点を真上に向けたい場合は phase=pi/2 に設定すると美しいです
%
% 返り値フォーマット（generate_reference 準拠, 15次元列ベクトル）:
%   ref(1:3)   = pos  [x; y; z]
%   ref(4)     = yaw  (= 0)
%   ref(5:7)   = vel  [vx; vy; vz]
%   ref(8)     = dyaw (= 0)
%   ref(9:11)  = acc  [ax; ay; az]
%   ref(12)    = 0
%   ref(13:15) = jerk [jx; jy; jz]
%
% 星形（ハイポサイクロイド）パラメトリック式（θ = w*t + phase）:
%   x(θ) = lx * (4*cos(θ) + cos(4*θ)) / 5    → 振幅 ±lx
%   y(θ) = ly * (4*sin(θ) - sin(4*θ)) / 5    → 振幅 ±ly
%   ※ 最大値が5となるため、5で割ることで振幅を ±lx, ±ly に正規化
%   各軸に warmup = 1 - exp(-(t/tau)^2) を乗じて滑らかに立ち上げる

arguments
    param.freq  = 10         % 周期 [s]
    param.orig  = [0 0 1]    % 中心座標 [x0 y0 z0]
    param.size  = [1 1 0]    % 各軸の振幅 [lx ly lz]
    param.phase = pi/2       % 位相オフセット [rad]（デフォルトpi/2で頂点が上を向く）
end

T      = param.freq;
origin = param.orig;
scale  = param.size;
phase  = param.phase;

lx        = scale(1);
lx_offset = origin(1);
ly        = scale(2);
ly_offset = origin(2);
lz        = scale(3);
lz_offset = origin(3);

w   = 2*pi / T;
tau = T / 2;

% ── syms で pos を定義し vel/acc/jerk を解析的に導出 ────────────
syms t_s real
wu      = 1 - exp(-(t_s/tau)^2);
theta_s = w*t_s + phase;

% 星形（5頂点ハイポサイクロイド）の軌跡
px_s = lx * wu * (4*cos(theta_s) + cos(4*theta_s)) / 5 + lx_offset;
py_s = ly * wu * (4*sin(theta_s) - sin(4*theta_s)) / 5 + ly_offset;
pz_s = lz * wu * sin(2*w*t_s + phase/2) + lz_offset;

vx_s = diff(px_s, t_s);  vy_s = diff(py_s, t_s);  vz_s = diff(pz_s, t_s);
ax_s = diff(vx_s, t_s);  ay_s = diff(vy_s, t_s);  az_s = diff(vz_s, t_s);
jx_s = diff(ax_s, t_s);  jy_s = diff(ay_s, t_s);  jz_s = diff(az_s, t_s);

px_f = matlabFunction(px_s, 'Vars', t_s);
py_f = matlabFunction(py_s, 'Vars', t_s);
pz_f = matlabFunction(pz_s, 'Vars', t_s);

vx_f = matlabFunction(vx_s, 'Vars', t_s);
vy_f = matlabFunction(vy_s, 'Vars', t_s);
vz_f = matlabFunction(vz_s, 'Vars', t_s);

ax_f = matlabFunction(ax_s, 'Vars', t_s);
ay_f = matlabFunction(ay_s, 'Vars', t_s);
az_f = matlabFunction(az_s, 'Vars', t_s);

jx_f = matlabFunction(jx_s, 'Vars', t_s);
jy_f = matlabFunction(jy_s, 'Vars', t_s);
jz_f = matlabFunction(jz_s, 'Vars', t_s);

% ── 参照関数（15次元列ベクトル） ───────────────────────────────
ref = @(t) [ ...
    px_f(t); py_f(t); pz_f(t); ...   % 1:3   pos
    0; ...                            % 4     yaw
    vx_f(t); vy_f(t); vz_f(t); ...   % 5:7   vel
    0; ...                            % 8     dyaw
    ax_f(t); ay_f(t); az_f(t); ...   % 9:11  acc
    0; ...                            % 12
    jx_f(t); jy_f(t); jz_f(t)];      % 13:15 jerk

fprintf("gen_ref_star: T=%.1fs  size=[%.2f %.2f %.2f]  orig=[%.2f %.2f %.2f]\n", ...
    T, lx, ly, lz, lx_offset, ly_offset, lz_offset);
fprintf("              max z-acc(t=T/4) = %.4f [m/s^2]\n", az_f(T/4));
end