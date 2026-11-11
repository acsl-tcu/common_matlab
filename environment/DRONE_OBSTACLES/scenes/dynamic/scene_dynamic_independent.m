function objects=scene_dynamic_independent()
% 動的障害物の見本：1障害物ごとに別の形状・位置・姿勢・軌道を設定。
% まだHLC/CBF/Waypointには接続しない（表示・構成確認用）。
objects(1)=obs_box('moving_box',[0 0 2],[1.3 0.8 1.1],[10 0 20]);
% 円軌道の中心 [x y z], 半径[rx ry], 一周秒数, 初期位相[deg], yaw速度[deg/s]
objects(1)=obs_attach_motion(objects(1), ...
    obs_motion_circle([0 1.5 2.5],[1.6 0.9],9,0,15));
objects(2)=obs_ellipsoid('moving_ellipsoid',[0 -3 1.8],[0.55 0.35 1.0],[0 30 0]);
% [times], 位置行列 [x y z] (1行＝1時刻), 姿勢 [roll pitch yaw]
objects(2)=obs_attach_motion(objects(2), ...
    obs_motion_waypoints([0 3 6 10], ...
        [-2 -3 1.8; -1 -2.4 2.1; 0 -3.3 1.6; 2 -3 2.0], ...
        [0 30 0; 0 10 30; 10 -10 70; 0 30 90],false));
objects(3)=obs_cone('moving_formula',[4 -2 2.3],0.65,1.3,[0 0 0]);
objects(3)=obs_attach_motion(objects(3), ...
    obs_motion_formula(@motion_demo_figure8_position,@motion_demo_figure8_rpy));
end
