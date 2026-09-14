%% edmdforhldata_plant.m
% HLC_EDMD_MEC.m 用のEDMD残差モデルを学習する実行スクリプト
% plant状態を使い，ノイズ環境でも滑らかな補償を得るための設定を行う
% 残差EDMD学習（ノイズ環境向けに正則化を強化した版）
% 変更点：lambda_err を大きくしてノイズ過学習を抑制し、補償を滑らかにする
clear; clc;
% ワークスペースとコマンドウィンドウを初期化する

opts = struct();
% 学習条件をopts構造体にまとめる
% 学習に使用するloggerデータの保存フォルダ
opts.data_dir   = 'C:\Users\student\Documents\GitHub\common_matlab\Data\Sim_data';
% 読み込むdegradedログファイル名のパターン
opts.file_pattern = 'HL_degraded_*.mat';
% 学習したEDMD残差モデルの保存先
opts.save_path  = 'C:\Users\student\Documents\GitHub\common_matlab\mode\KMPC\KQLMPC\edmd_residual_model_hl_plant.mat';
% plant状態を学習データとして使用する
opts.state_source = 'plant';

% ===== ノイズ環境で「滑らかな補償」を得るための正則化設定 =====
% lambda_err を 1e-3 -> 1e-1 に大きくするのが平滑化の本丸。
% 残差モデルがノイズの高周波成分を追わなくなり、系統的偏差だけを学ぶ。
opts.lambda_nom = 1e-4;        % 標称モデルも僅かに平滑化 (既定 1e-6)
opts.lambda_err = 1e-1;        % 残差モデルを強く正則化 (既定 1e-3) ★最重要
opts.res_norm_thresh = 20.0;   % 残差外れ値をより厳しく除去 (既定 50)
opts.mad_scale = 4.0;          % ジャンプ外れ値除去を強める (既定 6)

results_edmd = train_edmd_residual_from_logs(opts);
% A_nom, B_nom, A_err, B_errを学習して保存する
% 学習結果の簡易診断
fprintf('\n===== 学習結果サマリ =====\n');
fprintf('observable dim : %d\n', results_edmd.n_z);
fprintf('nominal RMSE   : %.4e\n', results_edmd.rmse_nom);
fprintf('residual RMSE  : %.4e\n', results_edmd.rmse_err);
fprintf('combined RMSE  : %.4e\n', results_edmd.rmse_total);
fprintf('rho(A_err)     : %.4f  (1未満で安定)\n', results_edmd.rho_A_err);
fprintf('||B_err||_F    : %.4e\n', norm(results_edmd.B_err, 'fro'));
fprintf('B_err col1(thrust) norm : %.4e\n', norm(results_edmd.B_err(:,1)));
fprintf('B_err col2(roll)   norm : %.4e\n', norm(results_edmd.B_err(:,2)));
fprintf('B_err col3(pitch)  norm : %.4e\n', norm(results_edmd.B_err(:,3)));