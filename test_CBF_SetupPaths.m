function paths = test_CBF_SetupPaths()
%TEST_CBF_SETUPPATHS テスト側から生成器の所在を自動的に検出します。
% 「common_matlab」または「commonmatlab」のどちらのフォルダ名でもOK。
% CBF_initialize をcommon_matlab直下へコピーする必要はありません。

%% 1. テストのあるcommon_matlabを起点にする
commonmatlab = fileparts(mfilename('fullpath'));
generator = fullfile(commonmatlab,'model','substance','generate_cbf_suspended');

%% 2. ディレクトリが正しいか確認
if exist(fullfile(generator,'CBF_initialize.m'),'file')~=2
    error('CBF:InstallPath', ...
        ['generate_cbf_suspendedフォルダが見つかりません。配置先を確認してください: ' generator]);
end

%% 3. 初期化関数のある場所へ先にパスを通す
addpath(generator);
paths = CBF_initialize();
end
