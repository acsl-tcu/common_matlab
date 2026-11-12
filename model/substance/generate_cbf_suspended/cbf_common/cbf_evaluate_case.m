function [h, info] = cbf_evaluate_case(mode,shape,obstacleType,x,settings,obstacle)
%CBF_EVALUATE_CASE 外部設定を受け取ってCBFの幾何学的安全関数を評価する。
% 使用例:
%  [h,info]=cbf_evaluate_case('virtual','C','ellipsoid',x,settings,obs);

P = cbf_parameter_vector(settings);
O = cbf_obstacle_vector(obstacle,obstacleType);
plan = cbf_choose_cable_spheres(settings);
if strcmp(mode,'real')
    if numel(x) ~= 29,error('CBF:StateLength','実入力拡張状態は29要素です。');end
    if isnumeric(x) && abs(norm(x(1:4))-1)>1e-6
        error('CBF:QuaternionNorm','実入力系のクォータニオンは単位ノルムが必要です。');
    end
    if isnumeric(x) && abs(norm(x(14:16))-1)>1e-6
        error('CBF:CableUnit','ケーブル方向 x(14:16) は単位ベクトルでなければなりません。');
    end
elseif strcmp(mode,'virtual')
    if numel(x) ~= 20,error('CBF:StateLength','仮想入力理想状態は20要素です。');end
    if isnumeric(x)
        a=x(7:9)+P(4)*[0;0;1]-[P(14);P(15);0];
        if norm(a)<1e-6,error('CBF:FlatMapSingularity','正規化する吊り荷加速度が小さすぎます。');end
    end
else
    error('CBF:UnknownMode','mode はreal/virtualです。');
end
[h,info] = cbf_geometry_core(mode,shape,obstacleType,x(:),P,O,plan);
info.cablePlan = plan;
end
