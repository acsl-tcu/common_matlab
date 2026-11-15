function xyz=motion_demo_figure8_position(t)
% 個別障害物用の数式軌道。world座標[m]。
w=2*pi/9;
xyz=[4+1.0*sin(w*t),-2+0.9*sin(2*w*t),2.3+0.25*sin(w*t)];
end
