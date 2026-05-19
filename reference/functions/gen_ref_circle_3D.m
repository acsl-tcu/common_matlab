function ref = gen_ref_circle_3D(param)
arguments
    param.freq = 20% 周期
    param.orig = [0 0 1]% 円の中心
    param.size = [1,1] % x&y,z方向のサイズ
    param.phase = [0,0] % x&y,z方向の位相    
    param.i
end
x_0=param.orig(1);
y_0=param.orig(2);
z_0=param.orig(3);
T = param.freq;
r_xy=param.size(1);
r_z=param.size(2);
phase_xy = param.phase(1);
phase_z = param.phase(2);
% syms t real
ref=@(t) [  x_0 + r_xy*sin(2*pi*t/T+phase_xy); %x
            y_0 + r_xy*cos(2*pi*t/T+phase_xy); %y
            z_0 + r_z*sin(2*pi*t/T+phase_z); %z
            0]; % yaw
end
