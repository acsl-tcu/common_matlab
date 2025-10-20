function ref = gen_ref_flower(param)
arguments
    param.freq = 15 % 周期は最低15s
    param.orig = [0 0 1]% 中心
    param.radius = 1.0 % 原点(orig)から大体の半径.少しだけ大きくなる
    param.phase = 0.0 % 位相    
    param.i
end

T = param.freq;
x_0=param.orig(1);
y_0=param.orig(2);
z_0=param.orig(3);

size = param.radius;
phase = param.phase;

ref=@(t) [  x_0 + size*(cos(2*pi*t/T)-cos(6*2*pi*t/T)/6); %x
            y_0 + size*(sin(2*pi*t/T)-sin(6*2*pi*t/T)/6); %y
            z_0; %z
            0];
end
