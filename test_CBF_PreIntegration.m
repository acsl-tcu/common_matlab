function test_CBF_PreIntegration()
%TEST_CBF_PREINTEGRATION HLCに接続せずCBF単体を検証する追加試験。
%
% 使い方: common_matlabを現在のフォルダにして
%   test_CBF_All();             % 前回の35ファイルの回帰試験
%   test_CBF_PreIntegration();  % 今回の追加試験
%
% ・現行HLCを呼び出さない（名目入力はテスト専用）
% ・失敗やQP実行不能は隠さずエラーにする
% ・MATLAB Optimization Toolbox (quadprog) が必要
% ・PASSしても任意軌道・任意初期条件の安全性を証明しない
fprintf('\n=== HLC接続前: CBF単体の追加試験開始 ===\n');
test_CBF_SetupPaths();
if exist('quadprog','file')~=2
    error('CBF:MissingQuadprog', ...
       'Optimization Toolboxのquadprogがありません。QP試験は必須なのでスキップしません。');
end
%% 1. 実入力・仮想入力の幾何学的な一致
fprintf('[1/8] 実入力/仮想入力の幾何一致\n');
test_CBF_ModelParity();
%% 2. 独立運動方程式との照合
fprintf('[2/8] 実入力拡張モデルの独立導関数検証\n');
test_CBF_IndependentDynamics();
%% 3. 障害物の回転および2個同時制約
fprintf('[3/8] 任意姿勢楕円体と複数障害物\n');
test_CBF_ObstacleStress();
%% 4. QPの可行・不可行・非緩和
fprintf('[4/8] QPの可行/不可行判定と制約余裕\n');
test_CBF_QPContract();
%% 5. 支持関数の十分性を独立距離で検証
fprintf('[5/8] 幾何CBFと楕円体までの独立距離計算\n');
test_CBF_GeometricClearance();
%% 6. 初期高次集合の必要性
fprintf('[6/8] h>=0と高次初期安全条件の違い\n');
test_CBF_InitialSet();
%% 7. 高階入力係数のアフィン性（基底差分での見落としを防ぐ）
fprintf('[7/8] 入力合成時の高階係数アフィン性\n');
test_CBF_InputAffinity();
%% 8. HLCを使わない時間積分つき閉ループ
fprintf('[8/8] 代替名目入力による実/仮想QP閉ループ\n');
test_CBF_StandaloneRollout();
fprintf('\n=== 接続前の追加試験 PASS ===\n');
fprintf(['※ 有限条件の数値試験です。将来可行性、HLC逆変換、\n' ...
         '　任意軌道の安全性保証は検証していません。\n']);
end
