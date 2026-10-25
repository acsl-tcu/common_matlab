function ref = gen_ref_line(param)
arguments
    param.p0 = [0 0 1.5]         % 開始位置
    param.velocity = 0.3         % 速度 [m/s]
    param.direction = [0 1 0]    % 進行方向
    param.t_start = 0.0          % 移動開始時刻 (ここを追加!)
end

p0 = param.p0(:);
v = param.velocity;
dir = param.direction(:);
dir = dir / norm(dir);

syms t real

% 切り替えなしの純粋な等速直線運動 (t_startから開始するように修正)
pos = p0 + (v * (t - param.t_start)) * dir;

yaw = atan2(dir(2), dir(1));

ref = @(t) [pos(1);
            pos(2);
            pos(3);
            yaw];
end
