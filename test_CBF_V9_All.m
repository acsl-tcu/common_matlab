function results=test_CBF_V9_All(options)
% TEST_CBF_V9_ALL 既存V7/V8本体の無変更・6テーマ一括監査。
% 実行例:
%    results = test_CBF_V9_All();
%    results = test_CBF_V9_All(struct('full',true));
%    results = test_CBF_V9_All(struct('settings',mySettings,'full',true));
% 成立しない制約を隠さない。スラック、入力クリップ、遠方足切りは行わない。
if nargin<1 || isempty(options),options=struct();end
if ~isstruct(options),error('CBF:V9Options','optionsは構造体');end
options.full=CBF_V9_Util('options',options,'full',false);
if ~isfield(options,'settings') || isempty(options.settings)
 options.settings=test_CBF_Fixture();
end
s=options.settings;
test_CBF_SetupPaths();
folder=CBF_V9_Util('folder');
assert(s.physical.massDrone>0 && s.physical.massLoad>0);
fprintf('\n=== V9 独立監査: 既存CBF・QP・生成器は無変更 ===\n');
fprintf('機体の既知質量=%g kg、吊り荷の推定質量=%g kg\n',...
 s.physical.massDrone,s.physical.massLoad);
labels={...
 '開始前の元モデル・参照高次導関数の自己検算',...
 '100m問題: 六階Taylor導関数の項分解とQPの原因行',...
 '100m問題: 物理入力T/tauの感度と相対次数の数値診断',...
 '低高度ホバリング: 初期集合・地面/入力ハード制約',...
 '安全な下降: 元の29状態モデルで滑らかな推力変化',...
 '地面設計比較: 同一ゲイン・段別ゲイン・制動参考条件',...
 '低高度軌道: 仮想CBF追従（実HLCではない）',...
 '低高度軌道: 実29状態鉛直入力指定（水平HLCなし）',...
 '低高度軌道: 実29状態地面+入力ハードCBF閉ループ',...
 '姿勢とケーブル: ロール/ピッチ/揺れと可行性・停止'};
fun={@test_CBF_V9_Sanity,@test_CBF_V9_FarDerivatives,@test_CBF_V9_PhysicalChannels,...
     @test_CBF_V9_LowHover,@test_CBF_V9_Braking,@test_CBF_V9_GroundAlternatives,...
     @test_CBF_V9_LowAltitudeVirtual,@test_CBF_V9_LowAltitudeReal,...
     @test_CBF_V9_LowAltitudeRealCBF,@test_CBF_V9_AttitudeCable};
fields={'sanity','far','physical','lowHover','braking','groundAlternative',...
        'virtualTrajectory','realTrajectory','realCBFTrajectory','attitudeCable'};
rows=cell(numel(fun),5);results=struct();
for i=1:numel(fun)
 timer=tic;
 try
  results.(fields{i})=fun{i}(options);
  rows(i,:)={i,labels{i},'TEST_COMPLETED_NOT_SAFETY_PASS',toc(timer),''};
  fprintf('[完了 %d/%d] %s\n',i,numel(fun),labels{i});
 catch ME
  rows(i,:)={i,labels{i},'MATLAB_ERROR',toc(timer),[ME.identifier ': ' ME.message]};
  results.([fields{i} '_error'])=getReport(ME,'extended','hyperlinks','off');
  fprintf(2,'[MATLAB ERROR %d/%d] %s: %s\n',i,numel(fun),labels{i},ME.message);
 end
end
results.execution=cell2table(rows,'VariableNames',...
 {'testId','testName','executionStatus','seconds','errorText'});
writetable(results.execution,fullfile(folder,'v9_execution_summary.csv'));
disp(results.execution);
fprintf('\n保存先：%s\n',folder);
fprintf('注意：完走＝安全保証ではありません。非線形系/仮想系と実HLC接続を区別。\n');
end
