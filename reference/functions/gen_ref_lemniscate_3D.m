function ref = gen_ref_lemniscate_3D(param)
arguments
    param.freq = 20% 周期
    param.orig = [0 0 1]% 中心
    param.size = [1,0.5] % 長軸方向のサイズ, z方向のサイズ
    param.phase = [0, 0] %  % x&y,z方向の位相
    param.x = 1 % x方向がレム二スケート∞の長軸
end
x_0=param.orig(1);
y_0=param.orig(2);
z_0=param.orig(3);
T = param.freq;
s_xy = param.size(1);
s_z = param.size(2)*2;
phase_xy = param.phase(1);
phase_z = param.phase(2);

if param.x == 1 % x方向に∞
    ref=@(t) [  x_0 + s_xy*sin(2*pi*t/T+phase_xy)/(1+cos(2*pi*t/T+phase_xy).^2); %x
                y_0 + s_xy*sin(2*pi*t/T+phase_xy)*cos(2*pi*t/T+phase_xy)/(1+cos(2*pi*t/T+phase_xy).^2); %y
                z_0 + s_z*sin(2*pi*t/T+phase_z)*cos(2*2*pi*t/T+phase_z)/(1+cos(2*2*pi*t/T+phase_z).^2); % z
                % % z_0 + s_z*sin(4*2*pi*t/T+phase_z); % z
                0]; % yaw
    
else            % y方向に∞
    ref=@(t) [  x_0 + s_xy*sin(2*pi*t/T+phase_xy)*cos(2*pi*t/T+phase_xy)/(1+cos(2*pi*t/T+phase_xy).^2); %x
                y_0 + s_xy*sin(2*pi*t/T+phase_xy)/(1+cos(2*pi*t/T+phase_xy).^2); %y
                z_0 + s_z*sin(2*2*pi*t/T+phase_z)*cos(2*2*pi*t/T+phase_z)/(1+cos(2*2*pi*t/T+phase_z).^2); % z
                0]; % yaw
end
end
