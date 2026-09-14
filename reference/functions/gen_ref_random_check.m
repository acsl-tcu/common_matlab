% このスクリプトを実行すると、ref(t)、軌道データ、2つの図を生成します。
% 必要な設定だけを指定できます。省略したものはarguments内の既定値。
[ref, traj] = gen_ref_random( ...
    T=32, ...                        % 軌道全体の時間 [s]
    n_waypoint=17, ...               % waypoint数
    start=[0; 0; 0.6], ...           % 1点目の固定座標 [m]
    x_range=[-1 1], ...              % xの範囲 [m]
    y_range=[-1 1], ...              % yの範囲 [m]
    z_range=[0.6 1.3], ...           % 始点を含むzの範囲 [m]
    max_step=[0.5 0.5 0.2], ...      % 各座標の最大変化量 [m]
    max_angle=0.5, ...               % roll/pitchの絶対値上限 [rad]
    dt=0.01, ...                     % データの出力周期 [s]
    plot=true);

ref_at_midpoint = ref(traj.t(end)/2); % 4-by-1: [x; y; z; 0]
ref_at_start = ref(0);               % [0; 0; 0.6; 0]
ref_values = ref(traj.t);            % 4-by-N: 各列が1時刻

% 制御・シミュレーションで使う変数。
waypoints = traj.waypoint;       % 17-by-3: [x y z] [m]
t_ref = traj.t;                  % N-by-1: 時刻 [s]
pos_ref = traj.position;         % N-by-3: [x y z] [m]
vel_ref = traj.velocity;         % N-by-3: [vx vy vz] [m/s]
acc_ref = traj.acceleration;     % N-by-3: [ax ay az] [m/s^2]
roll_ref = traj.roll;            % N-by-1 [rad]
pitch_ref = traj.pitch;          % N-by-1 [rad]
yaw_ref = traj.yaw;              % N-by-1 [rad]（0固定）
reference = traj.reference;      % N-by-7: [t x y z roll pitch yaw]

% 同じ軌道を再現する場合: ref = gen_ref_random(seed=42)
% 保存する場合: save('random_trajectory.mat','traj','reference','waypoints')
