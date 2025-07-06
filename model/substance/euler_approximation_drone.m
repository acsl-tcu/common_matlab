function x_plus = euler_approximation_drone(x_pre, input, params, dt)
%EULER_APPROXIMATION_DRONE
%   1次オイラー近似による状態更新
%   [Inputs]
%    x_pre=[p; q; v; w]: 現時刻の状態
%    input=[Thrust, roll, pitch, yaw]: 制御入力
%    params : ドローンの物理パラメータ
%    =["mass", "Lx", "Ly", "lx", "ly", "jx", "jy", "jz", "gravity", "km1", "km2", "km3", "km4", "k1", "k2", "k3", "k4"]
%    dt: 刻み時間
%   [Output]
%    x_plus: 次時刻の状態

%   2025/07 作成者:小関      学番:2212044

dx = call_dx(x_pre, input, params);
x_plus = x_pre + dt*dx;
end

function dx = call_dx(x_pre, input, params)
% 状態微分 dx を計算するローカル関数

    dp1 = x_pre(7);
    dp2 = x_pre(8);
    dp3 = x_pre(9); % 速度項

    gravity = params(9);
    jx = params(6);
    jy = params(7);
    jz = params(8);
    m = params(1);

    o1 = x_pre(10);
    o2 = x_pre(11);
    o3 = x_pre(12); % 角速度項

    pitch = x_pre(5);
    roll = x_pre(4);
    yaw = x_pre(6);

    u1 = input(1);
    u2 = input(2);
    u3 = input(3);
    u4 = input(4);

    t2 = cos(pitch);
    t3 = cos(roll);
    t4 = sin(pitch);
    t5 = sin(roll);
    t6 = 1.0 / jx;
    t7 = 1.0 / jy;
    t8 = 1.0 / jz;
    t9 = 1.0 / m;
    t10 = 1.0 ./ t2;

    t11 = pitch / 2.0;
    t12 = roll / 2.0;
    t13 = yaw / 2.0;

    t14 = cos(t11);
    t15 = cos(t12);
    t16 = cos(t13);
    t17 = sin(t11);
    t18 = sin(t12);
    t19 = sin(t13);

    t20 = t14 .* t15 .* t16;
    t21 = t14 .* t15 .* t19;
    t22 = t14 .* t16 .* t18;
    t23 = t15 .* t16 .* t17;
    t24 = t14 .* t18 .* t19;
    t25 = t15 .* t17 .* t19;
    t26 = t16 .* t17 .* t18;
    t27 = t17 .* t18 .* t19;

    t28 = -t25;
    t29 = -t26;
    t30 = t20 + t27;
    t31 = t23 + t24;
    t32 = t21 + t29;
    t33 = t22 + t28;

    dx = [
        dp1;
        dp2;
        dp3;
        t10 .* (o1 .* t2 + o3 .* t3 .* t4 + o2 .* t4 .* t5);
        o2 .* t3 - o3 .* t5;
        t10 .* (o3 .* t3 + o2 .* t5);
        t9 .* u1 .* (t30 .* t31 * 2.0 + t32 .* t33 * 2.0);
        -t9 .* u1 .* (t30 .* t33 * 2.0 - t31 .* t32 * 2.0);
        -gravity + t9 .* u1 .* (t30.^2 - t31.^2 + t32.^2 - t33.^2);
        t6 .* u2 + t6 .* (jy .* o2 .* o3 - jz .* o2 .* o3);
        t7 .* u3 - t7 .* (jx .* o1 .* o3 - jz .* o1 .* o3);
        t8 .* u4 + t8 .* (jx .* o1 .* o2 - jy .* o1 .* o2)
    ];
end
