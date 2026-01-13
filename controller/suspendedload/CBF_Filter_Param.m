function p = CBF_Filter_Param()
p.theta_max = deg2rad(15);  % 許容揺れ角
p.h_margin  = 0.02;         % 境界付近のマージン（m^2）
p.k_shrink  = 0.8;          % どれだけ縮めるか（0..1）
end
