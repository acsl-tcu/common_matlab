function results=test_CBF_IndependentAudit20261112(options)
% TEST_CBF_INDEPENDENTAUDIT20261112
% V7の生成器・CBF本体・ゲイン設計・QP本体には一切変更せず実施する監査。
% 配置: common_matlab直下。既存のtest_CBF_SetupPathsとtest_CBF_Fixtureを使用。
%
%   results=test_CBF_IndependentAudit20261112();
%   results=test_CBF_IndependentAudit20261112(struct('runTiming',true));
%
% 実モデルでは、推定質量は吊り荷だけ（settings.physical.massLoad）。
% 機体質量massDroneには推定値を代入しない。真値との差は自動導入しない。
if nargin<1, options=struct();end
paths=test_CBF_SetupPaths();
s=test_CBF_Fixture();
if isfield(options,'settings')
    s=options.settings;
end
% 入力として渡すsが既存生成器で有効かの確認。
cbf_parameter_vector(s);
origDroneMass=s.physical.massDrone;
% この監査は質量を内部で変更しない。
assert(origDroneMass>0 && s.physical.massLoad>0);
options.settings=s;

fprintf('\n=== CBF V7：本体無変更・独立監査（2026-11-12） ===\n');
fprintf('機体質量（固定）=%g kg / 吊り荷の推定質量=%g kg\n', ...
    s.physical.massDrone,s.physical.massLoad);
% 厳密なホバリング平衡で元運動方程式の右辺が0か検算。
xH=CBF_Audit_State('real',s,1.5,0,0,0,0,0,0);
fH=test_CBF_IndependentPlantRHS(xH,zeros(4,1),s);
assert(norm(fH,Inf)<1e-10,'CBF:HoverNotEquilibrium', ...
    '開始状態が物理ホバリング平衡にありません。');
hoverDiagnostics=cbf_model_diagnostics('real',xH,s);
assert(hoverDiagnostics.tension>0, ...
    'CBF:NonpositiveHoverTension','ホバリング張力が正ではありません。');
fprintf('[PASS] 元の運動方程式でホバリング平衡（導関数0、正張力）\n');

fprintf('\n[1/5] 地面だけ＋実入力ハード制約の高さ/速度/姿勢/ゲイン監査\n');
results.ground=test_CBF_AuditGroundGrid(options);
fprintf('\n[2/5] 実/仮想の低次入力作用と相対次数監査\n');
results.degree=test_CBF_AuditRelativeDegree(options);
fprintf('\n[3/5] 仮想Dの実→仮想整合性・退化・QP監査\n');
results.virtualD=test_CBF_AuditVirtualD(options);
fprintf('\n[4/5] HLC未接続の接近・下降・多姿勢短時間閉ループ\n');
results.rollout=test_CBF_AuditRollouts(options);
fprintf('\n[5/5] 仮想入力の参照軌道/初期位置・速度ずれ監査\n');
results.references=test_CBF_AuditReferenceInitial(options);
if isfield(options,'runTiming') && options.runTiming
    fprintf('\n[追加] 距離別・個数別の実時間計測\n');
    results.timing=test_CBF_AuditTimingNearFar(options);
end
assert(s.physical.massDrone==origDroneMass,'機体質量が変更されました。');
results.outputFolder=fullfile(paths.generator,'CBF','test_results');
fprintf('\n=== 監査終了：CSVをCBF/test_resultsに保存しました ===\n');
fprintf('注意：QP可行とHOCBF初期条件成立は別。失敗件数も結果です。\n');
fprintf('既存HLC、実入力逆変換、離散時間安全性はまだ保証していません。\n');
end
