function [ref, traj] = gen_ref_random(param)
%GEN_REF_RANDOM パラメータを指定してランダム軌道の関数ref(t)を生成。
%   ref = gen_ref_random;
%   ref = gen_ref_random(T=40, seed=42, plot=false);
%   [ref, traj] = gen_ref_random(T=32);
%   ref(1.5) は [x; y; z; 0] の4行1列。末尾の0はyaw [rad]。
%   ref([0 1 2]) は4行3列。時刻を列ベクトルで渡しても同じ形式。
%   軌道はこの関数の呼び出し時に一度生成し、ref(t)で繰り返し評価。
%   評価範囲は0 <= t <= T。T秒後の周期的な繰り返しは行わない。
%   roll/pitch等の詳細は2番目の出力trajに格納（従来と同じ形式）。
%   generate_random_trajectory.m を同じフォルダーに置いて使用する。

arguments
    % ----- 主に調整するパラメータ -----
    param.T (1,1) double {mustBeReal,mustBeFinite,mustBePositive} = 32 % 全体の時間 [s]
    param.n_waypoint (1,1) double {mustBeInteger,mustBeFinite,mustBeGreaterThanOrEqual(param.n_waypoint,4)} = 17 % 点数
    param.start (3,1) double {mustBeReal,mustBeFinite} = [0; 0; 0.6] % 固定する始点 [m]
    param.x_range (1,2) double {mustBeReal,mustBeFinite} = [-1 1]   % xの範囲 [m]
    param.y_range (1,2) double {mustBeReal,mustBeFinite} = [-1 1]   % yの範囲 [m]
    param.z_range (1,2) double {mustBeReal,mustBeFinite} = [0.6 1.3] % 始点を含むzの範囲 [m]
    param.max_step (1,3) double {mustBeReal,mustBeFinite,mustBeNonnegative} = [0.5 0.5 0.2] % 各座標の最大変化量 [m]
    param.max_angle (1,1) double {mustBeReal,mustBeFinite,mustBePositive} = 0.5 % roll/pitch上限 [rad]、pi/2未満

    % ----- 出力・乱数・表示の設定 -----
    param.dt (1,1) double {mustBeReal,mustBeFinite,mustBePositive} = 0.01 % 出力周期 [s]
    param.seed = []                 % []: 現在の乱数列、整数: 同じ軌道を再現
    param.max_attempts (1,1) double {mustBeInteger,mustBeFinite,mustBePositive} = 10000 % 最大試行回数
    param.plot (1,1) logical = true % 3次元図・位置と姿勢の確認図
    param.verbose (1,1) logical = true % 検証結果の表示
end

% waypointを等間隔の時刻に配置。既定値は32/(17-1)=2秒間隔。
waypoint_interval = param.T / (param.n_waypoint - 1);
lower_bound = [param.x_range(1), param.y_range(1), param.z_range(1)];
upper_bound = [param.x_range(2), param.y_range(2), param.z_range(2)];

% 三次スプラインの位置・姿勢制約を検証し、違反時は全点を再生成する。
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

% symsは不要。生成済みのスプラインを保存した関数ハンドルを返す。
pp = traj.ppPosition;
final_time = traj.t(end);
ref = @(t) evaluate_reference(pp, final_time, t);
end

function value = evaluate_reference(pp, final_time, t)
% スカラー・行ベクトル・列ベクトルのいずれも4行N列にそろえる。
validateattributes(t, {'numeric'}, {'real','finite','vector'}, mfilename, 't');
t = double(t(:).');
time_tolerance = 8*eps(max(1,final_time));
if any(t < -time_tolerance | t > final_time + time_tolerance)
    error('gen_ref_random:TimeOutOfRange', ...
        'ref(t)の時刻は0から%.16g秒の範囲で指定してください。', final_time);
end
% 時間の丸め誤差だけを端点にそろえ、制約検証範囲外へ外挿しない。
t = min(max(t,0),final_time);
value = [ppval(pp{1},t); ... % x [m]
         ppval(pp{2},t); ... % y [m]
         ppval(pp{3},t); ... % z [m]
         zeros(1,numel(t))]; % yaw [rad]
end
