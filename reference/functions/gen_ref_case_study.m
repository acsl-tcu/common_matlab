function ref = gen_ref_case_study(param)
arguments
    param.freq = 10 % 周期
    param.orig = [0 0 1]
end

x_0 = param.orig(1);
y_0 = param.orig(2);
z_0 = param.orig(3);
T = param.freq; % ★追加

ref = @trajectory;

    function r = trajectory(t)

        %% ===== 軌道の式をそのまま貼り付け =====
        x = 1.0*sin(2*pi*t/T);
        y = 1.0*sin(4*pi*t/T);

        %% ===================================

        r = [x_0 + x;
            y_0 + y;
            z_0;
            0];
    end
end