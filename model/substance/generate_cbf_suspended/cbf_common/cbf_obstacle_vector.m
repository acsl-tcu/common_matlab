function O = cbf_obstacle_vector(obstacle,obstacleType)
%CBF_OBSTACLE_VECTOR  任意姿勢の球/楕円体を15要素で内部表現する。
% 外部構造体: .center [3x1], .radius (sphere), .axes [3x1] (ellipsoid),
%            .R [3x3] 右手系正規直交回転行列（ワールド座標）
% .type は確認用、指定しない場合は生成関数側の型を優先。

if ~isfield(obstacle,'center')
    error('CBF:ObstacleCenter','obstacle.center が必要です。');
end
center = obstacle.center(:);
if strcmp(obstacleType,'sphere')
    if ~isfield(obstacle,'radius')
        error('CBF:ObstacleRadius','球障害物には obstacle.radius が必要です。');
    end
    r = obstacle.radius;
    axesLength = [r;r;r];
    R = eye(3);
elseif strcmp(obstacleType,'ellipsoid')
    if ~isfield(obstacle,'axes') || ~isfield(obstacle,'R')
        error('CBF:ObstacleEllipsoid','楕円体には obstacle.axes と obstacle.R が必要です。');
    end
    axesLength = obstacle.axes(:);
    R = obstacle.R;
else
    error('CBF:ObstacleType','型はsphereまたはellipsoidです。');
end
if numel(center)~=3 || numel(axesLength)~=3 || any(~isfinite(center)) || ...
   any(~isfinite(axesLength)) || any(axesLength<=0) || ...
   ~isequal(size(R),[3 3]) || any(~isfinite(R(:))) || ...
   norm(R.'*R-eye(3),'fro')>1e-7 || abs(det(R)-1)>1e-7
    error('CBF:InvalidObstacle','障害物の中心・半軸・SO(3)回転行列を確認してください。');
end
O = [center;axesLength;R(:)];
end
