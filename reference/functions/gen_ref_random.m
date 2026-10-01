function [ref, traj] = gen_ref_random(param)
%GEN_REF_RANDOM パラメータを指定してランダム軌道の関数ref(t)を生成。
%
%   ref(t) は20行N列：
%   1:4     [position; yaw]
%   5:8     [velocity; yaw_dot]
%   9:12    [acceleration; yaw_ddot]
%   13:16   [jerk; yaw_jerk]
%   17:20   [snap; yaw_snap]
%
%   例：
%   ref = gen_ref_random(T=40, seed=42, plot=false);
%   [ref, traj] = gen_ref_random(T=32);
%
%   ref(1.5)       -> 20行1列
%   ref([0 1 2])   -> 20行3列

arguments
    % ----- 主に調整するパラメータ -----
    param.T (1,1) double ...
        {mustBeReal,mustBeFinite,mustBePositive} = 32

    param.n_waypoint (1,1) double ...
        {mustBeInteger,mustBeFinite,...
         mustBeGreaterThanOrEqual(param.n_waypoint,4)} = 17

    param.start (3,1) double ...
        {mustBeReal,mustBeFinite} = [0; 0; 0.6]

    param.x_range (1,2) double ...
        {mustBeReal,mustBeFinite} = [-1 1]

    param.y_range (1,2) double ...
        {mustBeReal,mustBeFinite} = [-1 1]

    param.z_range (1,2) double ...
        {mustBeReal,mustBeFinite} = [0.6 1.3]

    param.max_step (1,3) double ...
        {mustBeReal,mustBeFinite,mustBeNonnegative} = [0.5 0.5 0.2]

    param.max_angle (1,1) double ...
        {mustBeReal,mustBeFinite,mustBePositive} = 0.5

    % ----- 出力・乱数・表示の設定 -----
    param.dt (1,1) double ...
        {mustBeReal,mustBeFinite,mustBePositive} = 0.01

    param.seed = []

    param.max_attempts (1,1) double ...
        {mustBeInteger,mustBeFinite,mustBePositive} = 10000

    param.plot (1,1) logical = true
    param.verbose (1,1) logical = true
end


%% waypoint設定
waypoint_interval = param.T / (param.n_waypoint - 1);

lower_bound = [ ...
    param.x_range(1), ...
    param.y_range(1), ...
    param.z_range(1)];

upper_bound = [ ...
    param.x_range(2), ...
    param.y_range(2), ...
    param.z_range(2)];


%% ランダム軌道生成
traj = generate_random_trajectory( ...
    'WaypointInterval', waypoint_interval, ...
    'NumWaypoints', param.n_waypoint, ...
    'StartPoint', param.start.', ...
    'LowerBound', lower_bound, ...
    'UpperBound', upper_bound, ...
    'MaxStep', param.max_step, ...
    'AngleLimit', param.max_angle, ...
    'SampleTime', param.dt, ...
    'Seed', param.seed, ...
    'MaxAttempts', param.max_attempts, ...
    'Plot', param.plot, ...
    'Verbose', param.verbose);

traj.param = param;


%% 位置・速度・加速度のpiecewise polynomial
pp_pos = traj.ppPosition;
pp_vel = traj.ppVelocity;
pp_acc = traj.ppAcceleration;


%% jerk・snapを追加で作成
pp_jerk = cell(1,3);
pp_snap = cell(1,3);

for axis_idx = 1:3
    pp_jerk{axis_idx} = differentiate_pp(pp_acc{axis_idx});
    pp_snap{axis_idx} = differentiate_pp(pp_jerk{axis_idx});
end

% trajからも確認できるよう保存
traj.ppJerk = pp_jerk;
traj.ppSnap = pp_snap;


%% 20状態を返すreference関数
final_time = traj.t(end);

ref = @(t) evaluate_reference( ...
    pp_pos, ...
    pp_vel, ...
    pp_acc, ...
    pp_jerk, ...
    pp_snap, ...
    final_time, ...
    t);

end


%% ========================================================================
function value = evaluate_reference( ...
    pp_pos, pp_vel, pp_acc, pp_jerk, pp_snap, final_time, t)

% ref(t)を20行N列にそろえる。
validateattributes( ...
    t, ...
    {'numeric'}, ...
    {'real','finite','vector'}, ...
    mfilename, ...
    't');

t = double(t(:).');


%% 時刻範囲確認
time_tolerance = 8 * eps(max(1,final_time));

if any(t < -time_tolerance | t > final_time + time_tolerance)
    error('gen_ref_random:TimeOutOfRange', ...
        'ref(t)の時刻は0から%.16g秒の範囲で指定してください。', ...
        final_time);
end

t = min(max(t,0),final_time);


%% position
pos = [ ...
    ppval(pp_pos{1},t); ...
    ppval(pp_pos{2},t); ...
    ppval(pp_pos{3},t)];


%% velocity
vel = [ ...
    ppval(pp_vel{1},t); ...
    ppval(pp_vel{2},t); ...
    ppval(pp_vel{3},t)];


%% acceleration
acc = [ ...
    ppval(pp_acc{1},t); ...
    ppval(pp_acc{2},t); ...
    ppval(pp_acc{3},t)];


%% jerk
jerk = [ ...
    ppval(pp_jerk{1},t); ...
    ppval(pp_jerk{2},t); ...
    ppval(pp_jerk{3},t)];


%% snap
snap = [ ...
    ppval(pp_snap{1},t); ...
    ppval(pp_snap{2},t); ...
    ppval(pp_snap{3},t)];


%% yawおよびyawの各階微分
zero = zeros(1,numel(t));


%% 20状態
value = [ ...
    pos;   zero; ...
    vel;   zero; ...
    acc;   zero; ...
    jerk;  zero; ...
    snap;  zero];

end


%% ========================================================================
function derivative = differentiate_pp(pp)
% piecewise polynomialを解析的に1階微分する。

[breaks, coefficients, pieces, order] = unmkpp(pp);

if order == 1
    % 定数を微分すると0
    derivative = mkpp(breaks, zeros(pieces,1));
else
    derivative_coefficients = ...
        coefficients(:,1:end-1) .* (order-1:-1:1);

    derivative = mkpp(breaks, derivative_coefficients);
end

end