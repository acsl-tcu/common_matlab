function motion = obs_motion_formula(positionFunction,rpyFunction)
% ユーザー作成の絶対位置・姿勢関数を障害物ごとに登録。
% 例: obs_motion_formula(@(t)[cos(t),sin(t),2],@(t)[0 0 15*t])
% 各関数の戻り値は [x y z] / [roll pitch yaw] の1x3。
% CBF向けの導関数は将来別途設計する (現版は表示・位置評価専用)。
assert(isa(positionFunction,'function_handle'),'positionFunction must be a function handle');
if nargin<2 || isempty(rpyFunction),rpyFunction=@(t)[0 0 0];end
assert(isa(rpyFunction,'function_handle'),'rpyFunction must be a function handle');
motion=struct('type','formula','positionFunction',positionFunction, ...
    'rpyFunction',rpyFunction);
end
