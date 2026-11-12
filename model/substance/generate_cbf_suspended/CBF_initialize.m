function paths = CBF_initialize()
%CBF_INITIALIZE  CBF生成ツールのパスを登録します（配置場所は自動検出）。
%
%  生成器のフォルダ構成（配置先の名前は変えないでください）
%    common_matlab/model/substance/generate_cbf_suspended/
%      generateCBF_RealInput.m
%      generateCBF_VirtualInput.m
%      cbf_common/
%      CBF/real/    <-- 実行時に作成
%      CBF/virtual/ <-- 実行時に作成
%
%  注意: このファイルはcommon_matlab直下ではありません。
%        テストからは test_CBF_SetupPaths() が自動で呼び出します。

%% 1. このファイルの置き場所から生成器の場所を特定する
root = fileparts(mfilename('fullpath'));
paths.generator = root;
paths.common = fullfile(root,'cbf_common');
paths.output = fullfile(root,'CBF');
paths.real = fullfile(paths.output,'real');
paths.virtual = fullfile(paths.output,'virtual');

%% 2. MATLAB検索パスを登録する
addpath(paths.generator);
addpath(paths.common);
if exist(paths.real,'dir') == 7, addpath(paths.real); end
if exist(paths.virtual,'dir') == 7, addpath(paths.virtual); end
end
