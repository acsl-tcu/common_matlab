% -------------------------------------------------------------------------
% Case 1: Y軸正弦往復 (初期値: [0, 0, 3.0])
% -------------------------------------------------------------------------
function ref = gen_ref_case1_line(param)
arguments
    param.freq   = 10.0
    param.center = [0 0 3.0]  % 高度を 3.0 に統一
    param.radius = [0 2.0 0]
    param.phase  = 0          % sin(0) = 0 より [0, 0, 3.0] からスタート
end
T = param.freq; origin = param.center; scale = param.radius; phase = param.phase;
w = 2*pi/T; syms t real
ref = @(t) [ origin(1); scale(2)*sin(w*t + phase) + origin(2); origin(3); 0 ];
end
