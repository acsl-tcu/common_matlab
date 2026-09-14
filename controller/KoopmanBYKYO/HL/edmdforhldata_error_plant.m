%% edmdforhldata_error_plant.m
% HLC_EDMD_ERROR.m 用の閉ループ誤差モデルを学習する実行スクリプト
% plant状態を使い，HL系ログから results_hl_edmd_error を作成する

opts = struct();
% 学習条件をopts構造体にまとめる

opts.data_dir = 'C:\Users\student\Documents\GitHub\common_matlab\Data\Sim_data';
% 学習に使用するloggerデータの保存フォルダ

opts.file_pattern = 'HL_*.mat';
% 読み込むログファイル名のパターン

opts.save_path = 'C:\Users\student\Documents\GitHub\common_matlab\mode\KMPC\KQLMPC\hl_edmd_error_model_plant.mat';
% 学習したEDMD誤差モデルの保存先

opts.state_source = 'plant';
% plant状態を教師データとして使用する

train_hl_edmd_error_from_logs(opts);
% 閉ループ誤差モデルを学習し，Az, Bz, Aa, Ba, Kz, Kaを保存する

