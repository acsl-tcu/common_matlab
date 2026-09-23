% -------------------------------------------------------------------------
% Case 3: 緩旋回 (初期値: [0, 0, 3.0])
% -------------------------------------------------------------------------
function ref = gen_ref_case3_slow_circle(param)
arguments
    param.freq   = 12.0
    param.center = [1.5 0 3.0] % 円の中心を X=+1.5 にずらす
    param.radius = [1.5 1.5 0] % 半径 1.5m
    param.phase  = -pi         % cos(-pi)=-1 より x = -1.5 + 1.5 = 0 からスタート
end
T = param.freq; origin = param.center; scale = param.radius; phase = param.phase;
w = 2*pi/T; syms t real
ref = @(t) [ scale(1)*cos(w*t + phase) + origin(1); ...
             scale(2)*sin(w*t + phase) + origin(2); ...
             origin(3); ...
             0 ];
end
