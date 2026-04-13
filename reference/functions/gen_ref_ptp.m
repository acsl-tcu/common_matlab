function ref = gen_ref_ptp(param)
% gen_ref_ptp  点到点 (Point-to-Point) 轨迹生成函数
%
% 按照输入的路径点 (waypoints) 依次飞行，并在每个点处平滑过渡（Minimum Jerk 轨迹）。
% 可以用来画正方形、矩形、或者任何多边形。
%
% 使用例（画边长为2的正方形，总耗时20秒）:
%   sq_pts = [0 0 1; 2 0 1; 2 2 1; 0 2 1; 0 0 1]; % 5个点（注意最后要回到起点闭合）
%   agent.reference.time_var = TIME_VARYING_REFERENCE(agent, ...
%       {"gen_ref_ptp", {"freq", 20, "waypoints", sq_pts}});
%
% 返り値フォーマット（generate_reference 準拠, 15次元列ベクトル）

arguments
    param.freq      = 20         % 总飞行周期 [s]
    % 默认给出的是一个边长为1的正方形 (位于 z=1 的高度)
    % 顺序: 起点 -> 右下角 -> 右上角 -> 左上角 -> 回到起点
    param.waypoints = [0 0 0.6; 1 0 0.6; 1 1 0.6; 0 1 0.6; 0 0 0.6] 
end

T_total = param.freq;
pts = param.waypoints;
N = size(pts, 1);
num_segments = N - 1;

% 容错：如果只输入了一个点，则保持在该点悬停
if num_segments < 1
    ref = @(t) [pts(1,:)'; 0; zeros(3,1); 0; zeros(3,1); 0; zeros(3,1)];
    return;
end

% 计算每条边的飞行时间（这里假设每段路程平均分配时间）
T_seg = T_total / num_segments;

% ── 参照関数（15次元列ベクトル） ───────────────────────────────
% 这里不再使用 syms 求导，而是用局部函数实时计算五次多项式状态
ref = @(t) ptp_state(t, pts, T_seg, num_segments);

fprintf("gen_ref_ptp: T_total=%.1fs, 共有 %d 个点，构成 %d 条线段\n", T_total, N, num_segments);
fprintf("              每条线段分配时间: %.2f [s]\n", T_seg);
end

%% 局部函数：计算任意时刻 t 的无人机状态
function state = ptp_state(t, pts, T_seg, num_segments)
    % 1. 限制 t 的范围，防止超出总时间
    t = max(0, min(t, T_seg * num_segments));

    % 2. 判断当前时间 t 属于哪一条线段
    idx = floor(t / T_seg) + 1;
    if idx > num_segments
        idx = num_segments;
        t_local = T_seg; % 保持在最后一条线段的终点
    else
        t_local = t - (idx - 1) * T_seg; % 当前线段内经历的时间
    end

    % 获取当前线段的起点 p0 和终点 p1
    p0 = pts(idx, :)';
    p1 = pts(idx+1, :)';

    % 3. Minimum Jerk (5次多项式) 轨迹计算
    % 归一化时间 tau (0 到 1)
    tau = t_local / T_seg;

    if tau >= 1
        pos  = p1;
        vel  = zeros(3,1);
        acc  = zeros(3,1);
        jerk = zeros(3,1);
    else
        % 5次多项式系数运算
        c_p = 10*tau^3 - 15*tau^4 + 6*tau^5;
        c_v = (30*tau^2 - 60*tau^3 + 30*tau^4) / T_seg;
        c_a = (60*tau - 180*tau^2 + 120*tau^3) / (T_seg^2);
        c_j = (60 - 360*tau + 360*tau^2) / (T_seg^3);

        dp = p1 - p0;
        
        pos  = p0 + dp * c_p;
        vel  = dp * c_v;
        acc  = dp * c_a;
        jerk = dp * c_j;
    end

    % 4. 组装为 15 次元列ベクトル
    state = [
        pos(1); pos(2); pos(3); ... % 1:3   pos
        0; ...                      % 4     yaw
        vel(1); vel(2); vel(3); ... % 5:7   vel
        0; ...                      % 8     dyaw
        acc(1); acc(2); acc(3); ... % 9:11  acc
        0; ...                      % 12
        jerk(1); jerk(2); jerk(3)   % 13:15 jerk
    ];
end