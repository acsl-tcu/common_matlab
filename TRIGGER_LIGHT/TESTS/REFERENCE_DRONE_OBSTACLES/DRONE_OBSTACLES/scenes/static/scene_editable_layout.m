function objects=scene_editable_layout()
% 日本語の設定例：形状・位置・大きさ・姿勢を項目別に変更できる。
% 単位: 全てm、角度はdeg。1つのsceneの全オブジェクトは静止する。
%
% 1) 直方体の壁
wallName='wall_A';
wallCenterXYZ=[2.0 4.0 2.5];   % 中心 [x, y, z] [m]
wallSizeXYZ=[5.0 0.6 3.2];    % 長さ [x方向, y方向, z方向] [m]
wallRPYDeg=[0 0 25];           % [roll, pitch, yaw] [deg]
objects(1)=obs_box(wallName,wallCenterXYZ,wallSizeXYZ,wallRPYDeg);

% 2) 傾いた円柱
columnName='column_B';
columnCenterXYZ=[-3 2 2];     % 中心 [m]
columnRadius=0.6;             % 半径 [m]
columnHeight=3.3;             % 高さ [m] (円柱のローカルZ方向)
columnRPYDeg=[15 -20 0];      % 任意の傾き [deg]
objects(2)=obs_cylinder(columnName,columnCenterXYZ, ...
    columnRadius,columnHeight,columnRPYDeg);

% 3) 中央に四角い貫通穴があるゲート
name='square_gate_C';
center=[0 8 3.0];             % ゲートの中心[m]
outerXY=[4.0 3.6];           % 外枠の幅と奥行き[m]
holeXY=[2.2 2.0];            % 四角穴の幅と奥行き[m]
height=0.5;                 % ゲートのローカルZ方向厚さ[m]
rpy=[90 0 0];                % XY面の枠を立てるためroll=90deg
objects(3)=obs_hollow_square_prism(name,center,outerXY,holeXY,height,rpy);
end
