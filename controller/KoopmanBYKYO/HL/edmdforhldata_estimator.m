%% edmdforhldata_estimator.m
% HLC_EDMD_MEC.m 用のEDMD残差モデルを学習する実行スクリプト
% estimator状態を使い，HL系ログから results_edmd を作成する

clear; clc;
% ワークスペースとコマンドウィンドウを初期化する

opts = struct();
% 学習条件をopts構造体にまとめる

opts.data_dir = 'C:\Users\student\Documents\GitHub\common_matlab\Data\Sim_data';
% 学習に使用するloggerデータの保存フォルダ

opts.file_pattern = 'HL_*.mat';
% 読み込むログファイル名のパターン

opts.save_path = 'C:\Users\student\Documents\GitHub\common_matlab\mode\KMPC\KQLMPC\edmd_residual_model_hl_estimator.mat';
% 学習したEDMD残差モデルの保存先

opts.state_source = 'estimator';
% estimator状態を学習データとして使用する

train_edmd_residual_from_logs(opts);
% A_nom, B_nom, A_err, B_errを学習して保存する


