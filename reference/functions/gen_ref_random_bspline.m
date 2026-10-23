function ref = gen_ref_random_bspline(param)
% 進行方向へのyaw追従に対応したランダム軌道（一様Bスプライン）
%
% 制御点を「旋回角と速度を制限したランダムウォーク」で生成し，一様Bスプラインで補間する．
%   - 凸包性により，制御点が範囲内なら軌道全体も範囲内に収まる
%   - 始点・終点は制御点を重ねて静止させる（1～degree-1階微分が0）
%   - 始点直後・終点直前は直線上で加減速するため，速度0付近でもyaw角が定まる
%   - 旋回角を制限しているので途中で速度が0にならず，yaw角を解析的に進行方向へ向けられる
%   - 傾き角・yaw角速度・各軸の利用幅を検証し，満たさなければ再生成する
%
% 戻り値はTIME_VARYING_REFERENCEで使える構造体
%   pYaw : t -> [x;y;z;yaw]の0～4階微分 (20×1)
%   q    : t -> [roll;pitch;yaw]の0～4階微分 (15×1)
%   info : 再現用のseed，制御点，検証結果など
% t > T では終点で静止し続ける．
%
% 使い方（第3引数のorderは付けない．微分は内部で解析的に計算する）:
%   ref = TIME_VARYING_REFERENCE(agent,{"gen_ref_random_bspline",{"T",60,"yaw_follow",1,"seed",1}});
arguments
    param.T = 60 % 飛行時間 [s]（序盤の加速と終盤の減速・中心への復帰があるので，動き回れるのは(T-12)秒ぐらい．knot_dtの整数倍に丸める）
    param.start = [0;0;1] % 始点 [m]
    param.start_yaw = 0 % 始点の進行方向・yaw角 [rad]（離陸時のyaw角に合わせる）
    param.xlim = [-2 2] % x方向の範囲 [m]
    param.ylim = [-2 2] % y方向の範囲 [m]
    param.zlim = [0.5 1.7] % z方向の範囲 [m]
    param.speed = [0.1 1.25] % 水平速度の大きさ(速さ)の範囲 [min max] [m/s]（向きは旋回で決まる．min > 0 が必要）
    param.vz_max = 1 % 鉛直速度の上限 [m/s]
    param.turn_max = pi/3 % 制御点1つあたりの最大旋回角 [rad]（degree*turn_max < pi で速度が0にならない）
    param.knot_dt = 0.5 % 制御点の時間間隔 [s]
    param.degree = 6 % Bスプラインの次数（yaw_followでyawの4階微分まで連続にするには6以上）
    param.yaw_follow = 0 % 1: yaw角を進行方向に向ける, 0: yaw角一定(start_yaw)
    param.max_tilt = pi/4 % 傾き角の上限 [rad]（加速度から求める）
    param.max_yaw_rate = pi % yaw角速度の上限 [rad/s]（yaw_follow時）
    param.yaw_wind_max = 2*pi % yaw角の累積回転量 |yaw - start_yaw| の上限の目安 [rad]（超えたら始点のyaw角付近まで逆回りに旋回する．infで制限なし）
    param.coverage = 0.7 % 各軸の (軌道の幅)/(指定範囲の幅) の下限
    param.seed = [] % []: 毎回異なる軌道（使ったseedを表示），整数: 同じ軌道を再現
    param.max_attempts = 1000 % 再生成の上限回数
    param.check = 1 % 1: 軌道の確認図を表示
end
k = param.degree;
h = param.knot_dt;
lb = [param.xlim(1); param.ylim(1); param.zlim(1)];
ub = [param.xlim(2); param.ylim(2); param.zlim(2)];
start = param.start(:);
if any(start < lb | start > ub)
    error("gen_ref_random_bspline:StartOutOfRange", "始点が範囲外です。");
end
if ~(0 < param.speed(1) && param.speed(1) <= param.speed(2))
    error("gen_ref_random_bspline:InvalidSpeed", "speed は 0 < min <= max の [min max] で指定してください。");
end
if param.yaw_follow && k < 6
    error("gen_ref_random_bspline:DegreeTooLow", "yaw_follow には degree >= 6 が必要です。");
end

nSegment = round(param.T/h); % セグメント数
nRamp = k; % 加減速の直線区間の制御点数
nWalk = nSegment - k + 1 - 2*nRamp; % ランダムウォークの制御点数
if nWalk < 1
    error("gen_ref_random_bspline:TooShort", "T が短すぎます（%.1f s 以上にしてください）。", (3*k)*h);
end

if isempty(param.seed)
    seed = RandStream("mt19937ar", "Seed", "shuffle").randi(2^31-1);
else
    seed = param.seed;
end
stream = RandStream("mt19937ar", "Seed", seed);

% 棄却理由の内訳（生成できなかったときに原因を表示するため）
reject = struct("step_xy", 0, "step_z", 0, "ramp", 0, "tilt", 0, "yaw_rate", 0, "coverage", zeros(1,3));
best = struct("tilt", inf, "yaw_rate", inf, "coverage", zeros(1,3)); % 検証まで進んだ軌道での最良値
fFound = false;
for attempt = 1:param.max_attempts
    [waypoints, failReason] = gen_waypoints(stream, start, lb, ub, nRamp, nWalk, param);
    if isempty(waypoints)
        reject.(failReason) = reject.(failReason) + 1;
        continue
    end
    controlPoints = [repmat(waypoints(:,1),1,k-1), waypoints, repmat(waypoints(:,end),1,k-1)];
    traj = build_traj(controlPoints, k, h, param.start_yaw, param.yaw_follow);
    [fFound, stats, fail] = check_traj(traj, lb, ub, param);
    if fFound
        break
    end
    reject.tilt = reject.tilt + fail.tilt;
    reject.yaw_rate = reject.yaw_rate + fail.yaw_rate;
    reject.coverage = reject.coverage + fail.coverage;
    best.tilt = min(best.tilt, stats.max_tilt);
    best.yaw_rate = min(best.yaw_rate, stats.max_yaw_rate);
    best.coverage = max(best.coverage, stats.coverage);
end
if ~fFound
    error("gen_ref_random_bspline:NotFound", "%s", not_found_message(reject, best, param, lb, ub, nWalk, nRamp));
end

fprintf("gen_ref_random_bspline: seed = %d, T = %.1f s, 試行 %d 回, 最大速度 %.2f m/s, 最大傾き %.3f rad, yaw範囲 [%.2f %.2f] rad, 最大yaw角速度 %.2f rad/s, 利用幅 [%.2f %.2f %.2f]\n", ...
    seed, traj.T, attempt, stats.max_speed, stats.max_tilt, stats.yaw_range, stats.max_yaw_rate, stats.coverage);

ref.pYaw = @(t) eval_pYaw(traj, t);
ref.q = @(t) eval_q(traj, t);
ref.order = 4;
ref.info = struct("seed", seed, "T", traj.T, "knot_dt", h, "degree", k, ...
    "control_points", controlPoints, "param", param, "stats", stats);

if param.check
    plot_traj(traj, lb, ub, stats);
end
end

function msg = not_found_message(reject, best, param, lb, ub, nWalk, nRamp)
% 生成できなかったときのエラーメッセージ：棄却理由の内訳と，設定を見直す目安
h = param.knot_dt;
axisName = ["x", "y", "z"];
width = (ub - lb)';
lines = string(sprintf("%d 回試行しても条件を満たす軌道が見つかりませんでした。棄却理由の内訳（1回の試行で複数該当することがあります）:", param.max_attempts));
if reject.step_z > 0
    lines(end+1) = sprintf("  - z方向に範囲外 (%d 回): 1ステップの上下移動 vz_max*knot_dt = %.2f m が z の範囲幅 %.2f m に対して大きすぎる可能性があります。vz_max を下げてください。", ...
        reject.step_z, param.vz_max*h, width(3));
end
if reject.step_xy > 0
    turnRadius = param.speed(2)*h/param.turn_max;
    if turnRadius > 0.5*min(width(1:2))/2
        hint = "範囲に対して大きく，端で曲がり切れないことがあります。speed の上限を下げるか，範囲を広げてください。";
    else
        hint = "範囲に対しては小さいので，試行回数（max_attempts）を増やすと見つかる可能性があります。";
    end
    lines(end+1) = sprintf("  - 水平方向に範囲外 (%d 回): 最高速度での旋回半径の目安 speed(2)*knot_dt/turn_max = %.2f m（範囲の半幅 %.2f m）は%s", ...
        reject.step_xy, turnRadius, min(width(1:2))/2, hint);
end
if reject.ramp > 0
    lines(end+1) = sprintf("  - 加減速区間の直線が範囲外 (%d 回): 直線の長さの目安 %.2f m。speed の下限を下げるか，始点を範囲の中央寄りにしてください。", ...
        reject.ramp, nRamp/2*param.speed(1)*h);
end
if reject.tilt > 0
    lines(end+1) = sprintf("  - 傾き角が上限超過 (%d 回): 最も小さい軌道でも %.3f rad（上限 max_tilt = %.3f rad）。speed の上限か turn_max を下げるか，max_tilt を上げてください。", ...
        reject.tilt, best.tilt, param.max_tilt);
end
if reject.yaw_rate > 0
    lines(end+1) = sprintf("  - yaw角速度が上限超過 (%d 回): 最も小さい軌道でも %.2f rad/s（上限 max_yaw_rate = %.2f rad/s）。旋回の速さの目安 turn_max/knot_dt = %.2f rad/s より十分大きい上限にするか，turn_max を下げてください。", ...
        reject.yaw_rate, best.yaw_rate, param.max_yaw_rate, param.turn_max/h);
end
for ax = 1:3
    if reject.coverage(ax) > 0
        lines(end+1) = sprintf("  - %s方向の利用幅が不足 (%d 回): 最も大きい軌道でも %.2f（下限 coverage = %.2f）。", ...
            axisName(ax), reject.coverage(ax), best.coverage(ax), param.coverage); %#ok<AGROW>
    end
end
if any(reject.coverage > 0)
    lines(end+1) = sprintf("    自由に動ける時間の目安は %.1f s です（加速区間と終盤の減速・中心への復帰を除く）。T を延ばすか，coverage を下げてください。", ...
        (nWalk - 2*nRamp)*h);
end
lines(end+1) = "  max_attempts を増やすと見つかる場合もあります。";
msg = strjoin(lines, newline);
end

function [waypoints, failReason] = gen_waypoints(stream, start, lb, ub, nRamp, nWalk, param)
% 重複を除いた制御点列 [始点, 加速区間, ランダムウォーク, 減速区間] を生成する
% 範囲内に収まらなかった場合は空と理由（"step_xy", "step_z", "ramp"）を返す
failReason = "";
h = param.knot_dt;
center = (lb + ub)/2;
halfWidth = (ub - lb)/2;
theta = param.start_yaw;
v = param.speed(1); % 加減速区間は最低速度で行い，直線区間を短くする
vz = 0;
p = start;
waypoints = zeros(3, 1 + 2*nRamp + nWalk);
waypoints(:,1) = p;
col = 1;

% 加速区間：始点のyaw方向へ直線上で速度を上げる
for j = 1:nRamp
    p = p + j/(nRamp+1)*v*h*[cos(theta); sin(theta); 0];
    col = col + 1;
    waypoints(:,col) = p;
end

% ランダムウォーク：旋回角と速度の変化を制限し，範囲の端に近いほど中心方向へ曲げる
nEnd = 2*nRamp; % 終盤：減速しながら中心へ向かい，減速区間の直線が範囲内に収まるようにする
fUnwind = false;
vTarget = param.speed(1) + diff(param.speed)*stream.rand;
for j = 1:nWalk
    fEnd = j > nWalk - nEnd;
    % yaw_follow時は同じ向きに回り続けてyaw角が巻き込まれないよう，上限を超えたら
    % 始点のyaw角付近（±pi/2）に戻るまで巻き戻す向きに旋回する（必要なら減速して旋回半径を小さくする）
    wind = abs(theta - param.start_yaw);
    fUnwind = param.yaw_follow && ~fEnd && (wind > param.yaw_wind_max || (fUnwind && wind > pi/2));
    % 速度：ときどき目標速度を範囲内で一様に選び直し，加減速を制限しながら近づける
    if fEnd
        vTarget = param.speed(1);
    elseif fUnwind
        % 巻き戻す向きに半周旋回しても範囲内に収まる最大の速度
        vTarget = param.speed(1);
        for vCandidate = linspace(param.speed(2), param.speed(1), 5)
            if is_arc_inside(p, theta, -sign(theta - param.start_yaw)*0.8*param.turn_max, vCandidate*h, lb, ub)
                vTarget = vCandidate;
                break
            end
        end
    elseif stream.rand < 0.25
        vTarget = param.speed(1) + diff(param.speed)*stream.rand;
    end
    v = v + min(max(vTarget - v, -diff(param.speed)/nRamp), diff(param.speed)/nRamp);
    vz = min(max(vz + 0.5*param.vz_max*(2*stream.rand-1), -param.vz_max), param.vz_max);
    if p(3) + vz*h < lb(3) || p(3) + vz*h > ub(3)
        vz = -vz;
    end
    fStep = false;
    for trial = 1:20
        proximity = max(abs(p(1:2) - center(1:2))./halfWidth(1:2)); % 0:中心, 1:境界
        weight = min(max((proximity - 0.4)/0.4, 0), 1);
        if fEnd || trial > 10 % 終盤と，ランダムな旋回で範囲外に出続ける場合は中心へ向ける
            weight = 1;
        end
        steer = angle(exp(1i*(atan2(center(2)-p(2), center(1)-p(1)) - theta)));
        dtheta = param.turn_max*(2*stream.rand-1) + weight*steer;
        % 巻き戻す向きに半周旋回しても範囲内に収まる場合のみ，その向きに旋回する
        if fUnwind && trial <= 10
            turnDir = -sign(theta - param.start_yaw);
            if is_arc_inside(p, theta, turnDir*0.8*param.turn_max, v*h, lb, ub)
                dtheta = turnDir*param.turn_max*(0.8 + 0.2*stream.rand);
            end
        end
        dtheta = min(max(dtheta, -param.turn_max), param.turn_max);
        pNext = p + [v*h*[cos(theta+dtheta); sin(theta+dtheta)]; vz*h];
        if all(pNext >= lb & pNext <= ub)
            fStep = true;
            break
        end
    end
    if ~fStep
        waypoints = [];
        if any(pNext(1:2) < lb(1:2) | pNext(1:2) > ub(1:2))
            failReason = "step_xy";
        else
            failReason = "step_z";
        end
        return
    end
    theta = theta + dtheta;
    p = pNext;
    col = col + 1;
    waypoints(:,col) = p;
end

% 減速区間：最後の進行方向へ直線上で速度を下げる
for j = nRamp:-1:1
    p = p + j/(nRamp+1)*v*h*[cos(theta); sin(theta); 0];
    col = col + 1;
    waypoints(:,col) = p;
end
if any(waypoints < lb | waypoints > ub, "all")
    waypoints = [];
    failReason = "ramp";
end
end

function fInside = is_arc_inside(p, theta, dtheta, stepLength, lb, ub)
% 一定の旋回角で半周したときに水平方向の範囲内に収まるか
fInside = true;
for j = 1:ceil(pi/abs(dtheta))
    theta = theta + dtheta;
    p(1:2) = p(1:2) + stepLength*[cos(theta); sin(theta)];
    if any(p(1:2) < lb(1:2) | p(1:2) > ub(1:2))
        fInside = false;
        return
    end
end
end

function traj = build_traj(controlPoints, k, h, startYaw, fYawFollow)
% 各セグメントの多項式係数と，yaw計算用の情報を求める
nSegment = size(controlPoints,2) - k;
M = bspline_matrix(k);
coef = zeros(k+1, 3, nSegment); % p(u) = [1 u ... u^k] * coef, u = t/h - (セグメント番号)
for j = 1:nSegment
    coef(:,:,j) = M * controlPoints(:, j:j+k)';
end
traj.coef = coef;
traj.k = k;
traj.h = h;
traj.nSegment = nSegment;
traj.T = nSegment*h;
traj.fYawFollow = fYawFollow;
traj.startYaw = startYaw;

% 微分用の係数：d^r/du^r u^i = i!/(i-r)! u^(i-r)
nDeriv = 6; % yawの4階微分に位置の5階微分まで必要
traj.dfactor = zeros(nDeriv, k+1);
traj.dpower = zeros(nDeriv, k+1);
for r = 0:nDeriv-1
    for i = r:k
        traj.dfactor(r+1, i+1) = prod(i-r+1:i);
        traj.dpower(r+1, i+1) = i - r;
    end
end

% 直線セグメント（水平速度の制御点がすべて同じ向き）ではyaw角を一定にする
% それ以外のセグメントの始点でyaw角をunwrapしておき，セグメント内は連続な分枝を選ぶ
% （1セグメント内の進行方向の変化は (k-1)*turn_max < pi）
dP = diff(controlPoints(1:2,:), 1, 2);
traj.fStraight = false(1, nSegment);
traj.yawBase = zeros(1, nSegment);
yawPrev = startYaw;
for j = 1:nSegment
    dPSegment = dP(:, j:j+k-1);
    dPSegment = dPSegment(:, vecnorm(dPSegment) > 1e-9);
    direction = atan2(dPSegment(2,:), dPSegment(1,:));
    traj.fStraight(j) = all(abs(angle(exp(1i*(direction - direction(1))))) < 1e-9);
    if traj.fStraight(j)
        yawRaw = direction(1);
    else
        pd = eval_derivatives(traj, (j-1)*h, 2);
        yawRaw = atan2(pd(2,2), pd(1,2));
    end
    traj.yawBase(j) = yawPrev + angle(exp(1i*(yawRaw - yawPrev)));
    yawPrev = traj.yawBase(j);
end
end

function M = bspline_matrix(k)
% 一様Bスプライン（k次）のセグメント行列：p(u) = [1 u ... u^k] * M * [C_0; ...; C_k]
% C_i の重み = 1/k! * sum_{j=0}^{k-i} (-1)^j nchoosek(k+1,j) (u + k-i-j)^k を u について展開
M = zeros(k+1);
for i = 0:k
    for j = 0:k-i
        c = k - i - j;
        for pw = 0:k
            M(pw+1, i+1) = M(pw+1, i+1) + (-1)^j*nchoosek(k+1, j)*nchoosek(k, pw)*c^(k-pw);
        end
    end
end
M = M/factorial(k);
end

function [pd, segment] = eval_derivatives(traj, t, nDeriv)
% 位置の0～(nDeriv-1)階微分 (3×nDeriv)．範囲外の時刻は始点・終点で静止
t = min(max(t, 0), traj.T);
segment = min(floor(t/traj.h), traj.nSegment-1) + 1;
u = t/traj.h - (segment-1);
pd = zeros(3, nDeriv);
for r = 0:nDeriv-1
    pd(:, r+1) = ((traj.dfactor(r+1,:).*u.^traj.dpower(r+1,:)) * traj.coef(:,:,segment))'/traj.h^r;
end
end

function [pd, yawd] = eval_all(traj, t)
% 位置とyaw角の0～4階微分
if isa(t, "sym")
    error("gen_ref_random_bspline:Symbolic", ...
        "TIME_VARYING_REFERENCE の第3引数(order)は付けないでください。微分は内部で計算しています。");
end
[pd, segment] = eval_derivatives(traj, t, 6);
if ~traj.fYawFollow
    yawd = [traj.startYaw, 0, 0, 0, 0];
elseif traj.fStraight(segment)
    yawd = [traj.yawBase(segment), 0, 0, 0, 0];
else
    % 速度を複素数 z = vx + i*vy とおくと yaw = Im(log z)
    % h = log z の微分は z*h' = z' をLeibniz則で微分した漸化式から求める
    % h^(m+1) = ( z^(m+1) - sum_{j=1}^{m} C(m,j) z^(j) h^(m+1-j) ) / z
    z = pd(1,2:6) + 1i*pd(2,2:6); % z(n+1) = z^(n)
    logd = zeros(1,4); % logd(n) = h^(n)
    for m = 0:3
        s = z(m+2);
        for j = 1:m
            s = s - nchoosek(m,j)*z(j+1)*logd(m+1-j);
        end
        logd(m+1) = s/z(1);
    end
    yaw = traj.yawBase(segment) + angle(exp(1i*(angle(z(1)) - traj.yawBase(segment))));
    yawd = [yaw, imag(logd)];
end
pd = pd(:,1:5);
end

function x = eval_pYaw(traj, t)
[pd, yawd] = eval_all(traj, t);
x = reshape([pd; yawd], [], 1); % [x;y;z;yaw]の0～4階微分
end

function q = eval_q(traj, t)
[~, yawd] = eval_all(traj, t);
q = reshape([zeros(2,5); yawd], [], 1); % [roll;pitch;yaw]の0～4階微分
end

function [fOk, stats, fail] = check_traj(traj, lb, ub, param)
% 傾き角・yaw角速度・利用幅を検証する
g = 9.81;
tSample = 0:0.02:traj.T;
n = length(tSample);
P = zeros(3,n); V = zeros(3,n); A = zeros(3,n); yaw = zeros(1,n); dyaw = zeros(1,n);
for j = 1:n
    [pd, yawd] = eval_all(traj, tSample(j));
    P(:,j) = pd(:,1); V(:,j) = pd(:,2); A(:,j) = pd(:,3);
    yaw(j) = yawd(1); dyaw(j) = yawd(2);
end
stats.t = tSample;
stats.p = P;
stats.v = V;
stats.max_speed = max(vecnorm(V));
stats.tilt = atan2(vecnorm(A(1:2,:)), g + A(3,:));
stats.max_tilt = max(stats.tilt);
stats.yaw = yaw;
stats.yaw_range = [min(yaw) max(yaw)];
stats.dyaw = dyaw;
stats.max_yaw_rate = max(abs(dyaw));
stats.coverage = ((max(P,[],2) - min(P,[],2))./(ub - lb))';
fail.tilt = stats.max_tilt > param.max_tilt;
fail.yaw_rate = param.yaw_follow && stats.max_yaw_rate > param.max_yaw_rate;
fail.coverage = stats.coverage < param.coverage;
fOk = ~fail.tilt && ~fail.yaw_rate && ~any(fail.coverage);
end

function plot_traj(traj, lb, ub, stats)
fig= figure(103);
fig.Name = "B-spline ref check";
clf
subplot(3,2,[1 3 5])
plot3(stats.p(1,:), stats.p(2,:), stats.p(3,:), 'LineWidth', 1.5)
hold on
idx = 1:round(1/(stats.t(2)-stats.t(1))):length(stats.t); % 1秒ごとに機首方向を表示
yaw = zeros(1,length(idx));
for j = 1:length(idx)
    [~, yawd] = eval_all(traj, stats.t(idx(j)));
    yaw(j) = yawd(1);
end
quiver3(stats.p(1,idx), stats.p(2,idx), stats.p(3,idx), 0.15*cos(yaw), 0.15*sin(yaw), zeros(size(yaw)), 0, 'r')
plot3(stats.p(1,1), stats.p(2,1), stats.p(3,1), 'gp', 'MarkerSize', 12, 'MarkerFaceColor', 'g')
[X,Y,Z] = ndgrid([lb(1) ub(1)], [lb(2) ub(2)], [lb(3) ub(3)]);
plot3(X(:), Y(:), Z(:), 'k.')
grid on
axis equal
xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]')
legend('path', 'yaw', 'start')
% title('Random B-spline reference')
subplot(3,2,2)
plot(stats.t, stats.p, 'LineWidth', 1.2)
grid on
ylabel('p [m]')
legend('x', 'y', 'z')
subplot(3,2,4)
yyaxis left
hSpeed = plot(stats.t, vecnorm(stats.v(1:2,:)), '-', stats.t, stats.v(3,:), '--', 'LineWidth', 1.2);
ylabel('speed [m/s]')
yyaxis right
hTilt = plot(stats.t, stats.tilt, 'LineWidth', 1.2);
ylabel('tilt [rad]')
legend([hSpeed; hTilt], {'xy speed', 'z velocity', 'tilt'})
grid on
subplot(3,2,6)
yyaxis left
plot(stats.t, stats.yaw, 'LineWidth', 1.2)
ylabel('yaw [rad]')
yyaxis right
plot(stats.t, stats.dyaw, 'LineWidth', 1.2)
ylabel('d yaw [rad/s]')
xlabel('time [s]')
grid on
end
