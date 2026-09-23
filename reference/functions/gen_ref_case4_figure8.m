% -------------------------------------------------------------------------
% Case 4: S字・8の字旋回 (初期値: [0, 0, 3.0])
% -------------------------------------------------------------------------
function ref = gen_ref_case4_figure8(param)
arguments
    param.freq   = 8.0
    param.center = [0 0 3.0]  % 高度 3.0
    param.radius = [2.0 1.2 0]
    param.phase  = 0          % sin(0)=0 より [0, 0, 3.0] からスタート
end
T = param.freq; origin = param.center; scale = param.radius; phase = param.phase;
w = 2*pi/T; syms t real
ref = @(t) [ scale(1)*sin(w*t + phase) + origin(1); ...
             scale(2)*sin(2*w*t + phase) + origin(2); ...
             origin(3); ...
             0 ];
end
