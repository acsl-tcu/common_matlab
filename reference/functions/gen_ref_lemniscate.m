function ref = gen_ref_lemniscate(param)
arguments
    param.freq = 10% 周期
    param.center = [0 0 1]% 円の中心
    param.radius = 1.0 % 長軸方向の大きさ
    param.phase = 0.0 % 位相
    param.x = 1 % x方向がレム二スケート∞の長軸
    param.yaw_follow = 0 % 1: yaw角を進行方向に向ける, 0: yaw角一定(0)
end
x_0=param.center(1);
y_0=param.center(2);
z_0=param.center(3);
T = param.freq;
a = param.radius;
phase = param.phase;
th = @(t) 2*pi*t/T+phase;

% 進行方向yaw角
% 速度ベクトルは a/(1+cos^2)^2 * [cos(3-cos^2); 3cos^2-1] に比例（x方向の場合）
% 進行方向は[-5pi/4, pi/4]の範囲を往復するため，atan2の不連続点(±pi)を
% 使用範囲外に回転させて連続なyaw角にしている（a>0を仮定）
if param.yaw_follow
    if param.x == 1
        yaw = @(t) atan2(cos(th(t))*(3-cos(th(t))^2), 1-3*cos(th(t))^2) - pi/2;
    else
        yaw = @(t) atan2(-cos(th(t))*(3-cos(th(t))^2), 1-3*cos(th(t))^2) + pi;
    end
else
    yaw = @(t) 0;
end

if param.x == 1 % x方向に∞
    ref=@(t) [  x_0 + a*sin(th(t))/(1+cos(th(t)).^2); %x
                y_0 + a*sin(th(t))*cos(th(t))/(1+cos(th(t)).^2); %y
                z_0; % z
                yaw(t)]; % yaw

else            % y方向に∞
    ref=@(t) [  x_0 + a*sin(th(t))*cos(th(t))/(1+cos(th(t)).^2); %x
                y_0 + a*sin(th(t))/(1+cos(th(t)).^2); %y
                z_0; %z
                yaw(t)]; % yaw
end
end
