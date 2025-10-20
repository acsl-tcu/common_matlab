function ref = gen_ref_heart(param)
arguments
    param.freq = 10% 周期
    param.orig = [0 0 1]% 中心
    param.size = 1.0 % x正負, y負方向の最大サイズ
    param.phase = 0.0 % 位相    
    param.i
end

T = param.freq;
x_0=param.orig(1);
y_0=param.orig(2);
z_0=param.orig(3);

size = param.size;
phase = param.phase;
ref=@(t) [  x_0 + size*((16*(sin(2*pi*t/T)).^3)/16); %x
            y_0 + size*((13*cos(2*pi*t/T) - 5*cos(2*2*pi*t/T) - 2*cos(3*2*pi*t/T) - cos(4*2*pi*t/T))/17); %y
            z_0; %z
            0];
end
