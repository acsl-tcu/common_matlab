function obs = ENVIRONMENT_OBSTACLE_ELLIPSOID_STATIC(varargin)
% 静止・任意姿勢の楕円体障害物（元のELLIPSE_MOVE.mの配置を維持）
% R_obs: 楕円体主軸座標 -> ワールド座標。時間変化させないこと。
% raw_param/ellipsoid_radii: 楕円体の主半径 [a;b;c] [m]
% d_margin: CBF側で紐カプセル半径に加算する安全マージン [m]。現在0.10m
% ソフトQPのスラックではなく、幾何学的な余裕（HOCBFはハード制約）。
% 注意: 元の例の中心z=15～20mは、z=3mの飛行経路から遠い。

% 元の円柱に対する外接楕円体半径を、そのまま楕円体そのものの半径に採用。
% yaw/pitch/rollをそれぞれ変更すれば任意の固定姿勢を設定できる。
obs(1) = make_ellipsoid(1, [0.3;0.3;20.0], ...
    [2.5*sqrt(2); 2.5*sqrt(2); 0.5*sqrt(2)], [25;15;-10], 0.1);
obs(2) = make_ellipsoid(2, [5;5;18.0], ...
    [3.0*sqrt(2); 3.0*sqrt(2); 0.5*sqrt(2)], [-20;18;10], 0.1);
obs(3) = make_ellipsoid(3, [5;-5;15.0], ...
    [sqrt(2);sqrt(2);0.5*sqrt(2)], [15;-25;20], 0.1);
end

function item = make_ellipsoid(id, center, radii, ypr_deg, margin)
assert(numel(center)==3 && numel(radii)==3 && all(isfinite(radii)) ...
    && all(radii>0) && margin>=0, '楕円体の中心・半径・マージンが不正です');
% ypr_deg = [yaw; pitch; roll] [deg], R = Rz*Ry*Rx
ang = ypr_deg(:)*pi/180;
y=ang(1); p=ang(2); r=ang(3);
Rz=[cos(y),-sin(y),0;sin(y),cos(y),0;0,0,1];
Ry=[cos(p),0,sin(p);0,1,0;-sin(p),0,cos(p)];
Rx=[1,0,0;0,cos(r),-sin(r);0,sin(r),cos(r)];
R=Rz*Ry*Rx;
item.id=id;
item.type='ellipsoid';
item.raw_param=radii(:);
item.p_center=center(:);
item.p_obs=center(:);
item.ellipsoid_radii=radii(:);
item.R_obs=R;
item.d_margin=margin;
item.v_center=[0;0;0];
% ワールド座標系の形状行列（従来の非回転diag表現では不十分）
item.S_obs=R*diag(radii(:).^2)*R.';
item.Q_obs=R*diag(1./(radii(:).^2))*R.';
% 旧描画コード向け互換値。CBFには使わない。
item.r_obs=max(radii);
item.r_obs_margin=item.r_obs+margin;
end
