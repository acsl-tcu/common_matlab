% -------------------------------------------------------------------------
% Case 5: 急旋回＋3次元加速 (初期値: [0, 0, 3.0])
% -------------------------------------------------------------------------
function ref = gen_ref_case5_fast_saddle(param)
arguments
    param.freq   = 4.5
    param.center = [1.5 0 3.0] % X中心を +1.5 にオフセット
    param.radius = [1.5 1.5 0.5]
    param.phase  = -pi         % t=0 で [0, 0, 3.0] から開始
end
T = param.freq; origin = param.center; scale = param.radius; phase = param.phase;
w = 2*pi/T; syms t real
ref = @(t) [ scale(1)*cos(w*t + phase) + origin(1); ...
             scale(2)*sin(w*t + phase) + origin(2); ...
             scale(3)*sin(2*w*t) + origin(3); ... % sin(0)=0 で Z も 3.0
             0 ];
end
