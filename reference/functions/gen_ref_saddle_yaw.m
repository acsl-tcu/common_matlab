function ref = gen_ref_saddle_yaw(param)
arguments
    param.freq = 10% 周期
    param.center = [0 0 1]% サドルの中心
    param.radius = [1 1 0.25] % 各軸の振幅
    param.yaw_rad= 1; % yaw角の振幅
    param.phase = -pi % 位相   
end

T = param.freq;
origin = param.center;
scale = [param.radius, param.yaw_rad];
phase = param.phase;
syms t real
syms lx ly real
lx = scale(1); % 4
lx_offset = origin(1); %4;
ly = scale(2); %3.5;
ly_offset = origin(2);% 3.5;
lz = scale(3);% 1;
lz_offset=origin(3);% 1;
lyaw = scale(4);
w = 2*pi/T; % T秒で一周

ref=@(t) [lx*cos(w*t + phase)+lx_offset; % x
ly*sin(w*t + phase)+ly_offset; % y
lz*sin(2*w*t + phase/2)+lz_offset; % z
lyaw*sin(w*t + phase)];% yaw
ddx = diff(ref,t,2);
fprintf("max ref acceleration = %f\n",subs(ddx(3),t,T/4));

% 圧倒的に遅いので以下のような書き方はしないこと
% xdf =@(t) [xd1(t),xd2(t),xd3(t),xd4(t)];
% dxdf =@(tt) subs(diff(xdf(t),t),t,tt);
% ddxdf =@(tt) subs( diff(dxdf(t),t),t,tt);
% dddxdf =@(tt)  subs(diff(ddxdf(t),t),t,tt);
% ddddxdf =@(tt)  subs(diff(dddxdf(t),t),t,tt);
% tXd=@(t) double([xdf(t),dxdf(t),ddxdf(t),dddxdf(t),ddddxdf(t)]);
end
