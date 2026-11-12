function test_CBF_GenerateFiles()
%TEST_CBF_GENERATEFILES 実入力6 + 仮想入力6の生成先・配置を検証。
paths = test_CBF_SetupPaths();

%% 1. 数値パラメータを渡さず生成する
realResult = generateCBF_RealInput();
virtualResult = generateCBF_VirtualInput();

%% 2. 保存場所とファイル数を確認
assert(numel(realResult.generatedFiles)==6);
assert(numel(virtualResult.generatedFiles)==6);
assert(strcmp(realResult.outputDirectory,paths.real));
assert(strcmp(virtualResult.outputDirectory,paths.virtual));
for i=1:6
    assert(exist(realResult.generatedFiles{i},'file')==2);
    assert(exist(virtualResult.generatedFiles{i},'file')==2);
end
fprintf('[PASS] CBF/real に6件、CBF/virtual に6件\n');
end
