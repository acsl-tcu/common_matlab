% -------------------------------------------------------------------------
% Case 2: 垂直・縦方向急加減速 (初期値: [0, 0, 3.0])
% -------------------------------------------------------------------------
function ref = gen_ref_case2_vertical(param)
arguments
    param.freq   = 8.0
    param.center = [0 0 3.0]  % 高度 3.0
    param.radius = [1.0 0 0.8]
    param.phase  = 0
end
T = param.freq; origin = param.center; scale = param.radius; phase = param.phase;
w = 2*pi/T; syms t real
% sin(w*t) にすることで t=0 で [0, 0, 3.0] からスムーズに立ち上がり
ref = @(t) [ scale(1)*sin(w*t + phase) + origin(1); ...
             origin(2); ...
             scale(3)*sin(2*w*t + phase) + origin(3); ...
             0 ];
end
