function ref = gen_ref_circle(param)
arguments
    param.freq = 20% 周期
    param.orig = [0 0 0]% 円の中心
    param.radius = 1.0 % 半径
    param.phase = 0.0 % 位相    
    param.i
end
x_0=param.orig(1);
y_0=param.orig(2);
z_0=param.orig(3);
T = param.freq;
r=param.radius;
% origin = param.orig;
phase = param.phase;
% syms t real
ref=@(t) [x_0+r*sin(2*pi*t/T); %x
    y_0 + r*cos(2*pi*t/T); %y
    z_0; %z
    0];
end
