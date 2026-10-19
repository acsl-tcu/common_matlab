function filename = save_mec_dataset(logger,metadata)
% 正常な1試行から、学習用のE・dU・E_nextと再現条件を保存する。
Dataset = build_mec_error_dataset(logger,metadata);
folder = metadata.setting.output_dir;
if ~exist(folder,'dir')
    mkdir(folder);
end
stamp = char(datetime('now','Format','yyyyMMdd_HHmmss_SSS'));
filename = fullfile(folder,sprintf('MEC_seed_%d_%s.mat',metadata.trajectory_seed,stamp));
save(filename,'Dataset');
fprintf('学習用データを保存：%s\n',filename);
fprintf('サンプル数=%d、入力ランク=%d/4、回帰ランク=%d/%d\n', ...
    Dataset.N,Dataset.diagnostics.input_rank,Dataset.diagnostics.regressor_rank, ...
    Dataset.diagnostics.regressor_rows);
if Dataset.diagnostics.regressor_rank<Dataset.diagnostics.regressor_rows
    warning('MEC:RegressorRank','回帰行列がランク不足です。別シードのデータも収集してください。');
end
end
