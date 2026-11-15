function results=test_CBF_V10_All(options)
%TEST_CBF_V10_ALL 既存CBFは無変更。V9の誤り修正後、実HLC模擬と100mを比較。
% options.hlcFolder: controller/@HLC_SUSPENDED_LOAD の親フォルダ。
% options.full=true: 低高度全区間・姿勢差試験も実行。
% 例: opts=struct('hlcFolder','C:\\research\\PROJECT_FULL\\controller','full',false);
%     results=test_CBF_V10_All(opts);
if nargin<1,options=struct();end
if ~isfield(options,'settings') || isempty(options.settings)
 options.settings=test_CBF_Fixture();
end
test_CBF_SetupPaths();
fprintf('\n===== V10 TEST ONLY：実HLC生成関数・実入力系・CBF =====\n');
fprintf('既知機体質量=%g kg / 使用する吊り荷の推定質量=%g kg\n',...
  options.settings.physical.massDrone,options.settings.physical.massLoad);
results=struct();
labels={'V9行抽出修正とホバリング',...
 'V9低高度ホバリング再監査','V9独立実モデル下降再監査',...
 'V9地面代替案再監査','V9姿勢・ケーブル再監査',...
 'V9実モデル入力指定軌道再監査','V9実モデルCBF閉ループ再監査',...
 '100m旧バリアと距離正規化案の比較',...
 '実HLC raw実入力→19状態物理モデル',...
 '実HLC raw入力→テスト用動的拡張→CBFハードQP'};
fields={'smoke','lowHover','brake','groundAlternatives','attitude',...
 'realFeedforward','realCBF','farNormalized','hlcPhysical','hlcDynamicCBF'};
functions={@test_CBF_V10_Smoke,@test_CBF_V9_LowHover,...
  @test_CBF_V9_Braking,@test_CBF_V9_GroundAlternatives,...
  @test_CBF_V9_AttitudeCable,@test_CBF_V9_LowAltitudeReal,...
  @test_CBF_V9_LowAltitudeRealCBF,...
  @test_CBF_V10_FarNormalization,@test_CBF_V10_HLCPhysical,...
  @test_CBF_V10_HLCDynamicCBF};
entries=cell(numel(functions),5);
for i=1:numel(functions)
 t0=tic;
 try
  if exist(func2str(functions{i}),'file')~=2
   error('CBF:V10MissingTest','依存テストが見つかりません: %s',func2str(functions{i}));
  end
  if i>=numel(functions)-1
    [res,details]=functions{i}(options);
    results.(fields{i})=res;
    results.([fields{i} '_timeline'])=details;
  else
    results.(fields{i})=functions{i}(options);
  end
  status='TEST_EXECUTED_RESULTS_REQUIRE_REVIEW';errorText='';
 catch ME
  status='MATLAB_ERROR_OR_HLC_MISSING';
  errorText=[ME.identifier ': ' ME.message];
  results.([fields{i} '_error'])=errorText;
  fprintf(2,'[V10 %d/%d] %s\n',i,numel(functions),errorText);
 end
 entries(i,:)={i,labels{i},status,toc(t0),errorText};
end
results.execution=cell2table(entries,'VariableNames',...
 {'testId','testName','executionStatus','seconds','errorText'});
writetable(results.execution,fullfile(CBF_V9_Util('folder'),'v10_execution_summary.csv'));
disp(results.execution);
fprintf('保存先: %s\n',CBF_V9_Util('folder'));
fprintf('注意: テスト完了・HLC生入力・フィルタ付きCBFは別物。安全保証済みとはしません。\n');
end
