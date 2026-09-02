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
% このスクリプトのディレクトリからリポジトリルートに移動してパスを通す
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
% リポジトリルートからの相対パス、または絶対パスで指定
matFilePath = fullfile(rootDir, ...
    'Data', 'Sim_data', ...
    'HOCBF_LINK_XY_normal_Log(13-Aug-2026_16_30_08).mat');

% --- 保護球半径 (rl) ---
% HLC_SUSPENDED_LOAD_HOCBF_LINK_XY コントローラ内で rl_sys = 1.2 に固定されています
% コントローラを変更した場合はここも合わせて変更してください
actual_rl = 1.2;

% --- 動画の出力先フォルダ ---
outdir = fullfile(rootDir, 'Data');

% --- 動画の FPS ---
fps = 30;

% --- フレームのスキップ数（1=全フレーム、2=1フレームおきなど、高速化したい場合に増やす）---
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

%% agent オブジェクトの再構築
% DRAW_SUSPENDED_LOAD_HOCBF_LINK_XY のコンストラクタは
%   - agent.parameter.Lx / Ly（フレームサイズ）
%   - agent.parameter.rotor_r（ロータ半径）
%   - agent.parameter.Length（荷物半径）
%   - agent.parameter.cableL（ケーブル長）
% を参照します。
% SimSuspendedLoadHOCBFlinkxy.m と同じ設定を使用します。
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

% DRAW クラスをインスタンス化
mov = DRAW_SUSPENDED_LOAD_HOCBF_LINK_XY(logger, ...
    "target", 1, ...
    "self",   agent, ...
    "rl",     actual_rl);

% MP4 を保存しながらアニメーション再生
mov.animation(logger, ...
    "target",  1, ...
    "self",    agent, ...
    "rl",      actual_rl, ...
    "mp4",     true, ...   % true: 自動ファイル名 (Movie_<timestamp>.mp4)
    "outdir",  outdir, ...
    "fps",     fps, ...
    "skip",    skip, ...
    "pause",   0);          % 0: ウェイトなし（最速で書き出し）

fprintf('=== [make_movie_HOCBF_LINK_XY] 完了!\n');
