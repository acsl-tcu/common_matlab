function ref = gen_ref_custom(param)
arguments
    param.orig = [0 0 1]
end

x_0 = param.orig(1);
y_0 = param.orig(2);
z_0 = param.orig(3);

ref = @trajectory;

    function r = trajectory(t)

        %% ===== 軌道の式をそのまま貼り付け =====

        x = cos(2 * 2*pi*t/10) .* cos(2*pi*t/10);
        y = cos(2 * 2*pi*t/10) .* sin(2*pi*t/10);

        %% ===================================

        r = [x_0 + x;
             y_0 + y;
             z_0;
             0];
    end
end