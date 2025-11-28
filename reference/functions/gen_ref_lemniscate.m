function ref = gen_ref_lemniscate(param)
arguments
    param.freq = 10% 周期
    param.orig = [0 0 1]% 円の中心
    param.radius = 1.0 % 長軸方向の大きさ
    param.phase = 0.0 % 位相
    param.x = 1 % x方向がレム二スケート∞の長軸
end
x_0=param.orig(1);
y_0=param.orig(2);
z_0=param.orig(3);
T = param.freq;
a = param.radius;
phase = param.phase;

if param.x == 1 % x方向に∞
    ref=@(t) [  x_0+a*sin(2*pi*t/T+phase)/(1+cos(2*pi*t/T+phase).^2); %x
                y_0 + a*sin(2*pi*t/T+phase)*cos(2*pi*t/T+phase)/(1+cos(2*pi*t/T+phase).^2); %y
                z_0; % z
                0]; % yaw
else            % y方向に∞
    ref=@(t) [  x_0 + a*sin(2*pi*t/T+phase)*cos(2*pi*t/T+phase)/(1+cos(2*pi*t/T+phase).^2); %x
                y_0 + a*sin(2*pi*t/T+phase)/(1+cos(2*pi*t/T+phase).^2); %y
                z_0; %z
                0]; % yaw
end
end
