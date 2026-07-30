function ref = gen_ref_star(param)
arguments
    param.freq = 15% 周期は最低15s
    param.center = [0 0 1]% 中心
    param.radius = 1.0 % 大体の半径サイズ.少し大きくなる
    param.phase = 0.0 % 位相    
    param.i
end

T = param.freq;
x_0=param.center(1);
y_0=param.center(2);
z_0=param.center(3);

size = param.radius;
phase = param.phase;
ref=@(t) [  x_0 + size*(3*(0.3 + 0.1*cos(5*2*pi*t/T)).*cos(2*pi*t/T)); %x
            y_0 + size*(3*(0.3 + 0.1*cos(5*2*pi*t/T)).*sin(2*pi*t/T)); %y
            z_0; %z
            0];
end