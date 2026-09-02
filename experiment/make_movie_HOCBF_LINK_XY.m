%% make_movie_HOCBF_LINK_XY.m
% 保存された .mat ログファイルから HOCBF_LINK_XY シミュレーションの
% MP4 動画を作成するスクリプト
%
% 使い方:
%   1. MATLAB でこのスクリプトを開く
%   2. 必要に応じて下部の「設定」セクションを編集する
%   3. 実行する（Run ボタン or コマンドウィンドウで run("experiment/make_movie_HOCBF_LINK_XY.m")）
%
% 出力: Data/ フォルダに Movie_<タイムスタンプ>.mp4 が保存される

%% パスの設定
if contains(mfilename('fullpath'), 'experiment')
    rootDir = fileparts(fileparts(mfilename('fullpath')));
else
    tmp = matlab.desktop.editor.getActive;
    rootDir = fileparts(fileparts(tmp.Filename));
end
[~, pathList] = regexp(genpath(rootDir), '\\.\\\.git.*?;', 'match', 'split');
cellfun(@(p) addpath(p), pathList, 'UniformOutput', false);

%% =====================================================================
%% 設定（必要に応じて変更してください）
%% =====================================================================

% --- MAT ファイルのパス ---
matFilePath = fullfile(rootDir, ...
    'Data', 'Sim_data', ...
    'HOCBF_LINK_XY_normal_Log(13-Aug-2026_16_30_08).mat');

% --- 動画の出力先フォルダ ---
outdir = fullfile(rootDir, 'Data');

% --- 動画の FPS ---
fps = 30;

% --- フレームのスキップ数（1=全フレーム、2=1フレームおきなど） ---
skip = 1;

%% =====================================================================

%% ログファイルの読み込み
fprintf('=== [make_movie_HOCBF_LINK_XY] ログファイルを読み込み中...\n');
fprintf('  Path: %s\n', matFilePath);
if ~isfile(matFilePath)
    error('make_movie_HOCBF_LINK_XY:FileNotFound', ...
        'MAT ファイルが見つかりません:\n  %s', matFilePath);
end
logger = LOGGER(matFilePath);
fprintf('  => 読み込み完了 (ステップ数: %d)\n', logger.k);

%% actual_rl をログデータから復元
% コントローラが毎ステップ controller.result.rl に書き込んでいる値を使用する
actual_rl = [];
for k_idx = 1:logger.k
    cr = logger.Data.agent(1).controller.result{k_idx};
    if isstruct(cr) && isfield(cr, 'rl') && ~isempty(cr.rl)
        actual_rl = cr.rl;
        break;
    end
end
if isempty(actual_rl)
    actual_rl = 1.2; % フォールバック（コントローラの固定値と同じ）
    fprintf('  [警告] rl がログに見つからなかったためデフォルト値 %.3f を使用します\n', actual_rl);
else
    fprintf('  => rl = %.3f (ログから復元)\n', actual_rl);
end

%% agent オブジェクトの再構築
% DRAW クラスのコンストラクタが参照するパラメータを SimSuspendedLoadHOCBFlinkxy.m と同じ設定で準備
fprintf('=== [make_movie_HOCBF_LINK_XY] agent を再構築中...\n');
agent = DRONE;
agent.parameter = DRONE_PARAM_SUSPENDED_LOAD("DIATONE");
agent.parameter.set("loadmass", 0.14);
agent.parameter.set("cableL", 2.0);

%% MP4 動画の作成
fprintf('=== [make_movie_HOCBF_LINK_XY] アニメーションを生成中...\n');
fprintf('  actual_rl = %.3f\n', actual_rl);
fprintf('  outdir    = %s\n', outdir);
fprintf('  fps       = %d, skip = %d\n', fps, skip);

% DRAW クラスをインスタンス化（この時点で figure が生成される）
mov = DRAW_SUSPENDED_LOAD_HOCBF_LINK_XY(logger, ...
    "target", 1, ...
    "self",   agent, ...
    "rl",     actual_rl);

% --- figure をスクリーン中央に移動（ウィンドウサイズは変えない）---
fig = ancestor(mov.ax, 'figure');
fig.Units = 'pixels';
screenSz = get(0, 'ScreenSize'); % [left bottom width height]
figPos   = fig.Position;         % 現在のサイズを取得
figW = figPos(3);
figH = figPos(4);
fig.Position = [ ...
    (screenSz(3) - figW) / 2, ...   % 水平中央
    (screenSz(4) - figH) / 2, ...   % 垂直中央
    figW, figH];

% MP4 を保存しながらアニメーション再生
mov.animation(logger, ...
    "target",  1, ...
    "self",    agent, ...
    "rl",      actual_rl, ...
    "mp4",     true, ...
    "outdir",  outdir, ...
    "fps",     fps, ...
    "skip",    skip, ...
    "pause",   0);

fprintf('=== [make_movie_HOCBF_LINK_XY] 完了!\n');
