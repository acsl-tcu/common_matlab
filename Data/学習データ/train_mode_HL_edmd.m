% 設定用の構造体を作る
opts = struct();

% 読み込むログ
opts.data_dir = 'C:\Users\yuriy\github\common_matlab\Data\学習データ';
opts.file_pattern = 'HL_*.mat';

% 学習結果の保存先
opts.save_path = 'C:\Users\yuriy\github\common_matlab\Data\学習データ\hl_edmd_error_model.mat';

% opts を渡して関数を実行する
results = train_hl_edmd_error_from_logs(opts);