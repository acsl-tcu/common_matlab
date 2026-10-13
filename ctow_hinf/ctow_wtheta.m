function [Aw, Bw, Cw] = ctow_wtheta(opt)
%CTOW_WTHETA  振れ角重みの状態空間（k_th を除いた形状部分）
%   Wshape(s) = 2*z_w*w_w*s / (s^2 + 2*z_w*w_w*s + w_w^2)（中心 w_w でゲイン 1）
w = opt.w_w; z = opt.z_w;
Aw = [0 1; -w^2 -2*z*w];
Bw = [0; 1];
Cw = [0 2*z*w];
end
