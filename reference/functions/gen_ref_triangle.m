function ref = gen_ref_triangle(param)
arguments
    param.freq = 5% 周期
    param.orig = [0 0 1]% 三角形もどきの中心
    param.size = 1.0 % 原点からx正への最大値
    param.phase = 0.0 % 位相    
    param.i
end

T = param.freq;
x_0=param.orig(1);
y_0=param.orig(2);
z_0=param.orig(3);

size = param.size;
phase = param.phase;
ref=@(t) [x_0+size*(2*cos(2*pi*t/T) + cos(2*2*pi*t/T))/3; %x
            y_0 + +size*(2*sin(2*pi*t/T) - sin(2*2*pi*t/T))/3; %y
            z_0; %z
            0];
end
