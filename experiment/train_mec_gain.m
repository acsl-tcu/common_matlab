function [gain_file, model_file, report] = train_mec_gain(setting)
% データの読み込みからゲイン検証・保存までを実行する。
% 例：[gain_file, model_file, report] = train_mec_gain();
% 設定を変える場合は、引数にmec_gain_settings()の戻り値を渡す。

%% 設定と学習データを準備する
if nargin == 0
    setting = mec_gain_settings();
end

trials = load_mec_training_trials(setting.dataset_dir);

%% 誤差モデルをクープマン線形化で同定
[est, report] = fit_mec_error_model(trials, setting);
gain = design_mec_error_gain(est, setting);

% 最終評価用の試行で、現在のゲインが出す補償量の最大値も記録する。
% これは補償を適用した閉ループ応答の性能を示す値ではない。
test_errors = [];
for trial_id = report.test_ids
    test_errors = [test_errors, trials{trial_id}.E]; %#ok<AGROW>
end
report.test.compensation_peak = max(abs(gain.K_full * test_errors), [], 2);
gain.report = report;

%% 検証前の候補を、比較用の時刻付きファイルとして保存する
if ~exist(setting.output_dir, 'dir')
    mkdir(setting.output_dir);
end

timestamp = char(datetime('now', 'Format', 'yyyyMMdd_HHmmss_SSS'));
model_file = fullfile(setting.output_dir, ['mec_model_' timestamp '.mat']);
gain_file = fullfile(setting.output_dir, ['mec_gain_' timestamp '.mat']);

save(model_file, 'est', 'report', 'setting');
save(gain_file, '-struct', 'gain');

%% 非線形シミュレーションで検証し、結果を記録する
% 検証を通過した場合だけ、既定ゲインを更新する。
report.nonlinear_validation = validate_mec_gain(gain_file);
gain.report = report;
save(gain_file, '-struct', 'gain');

%% 検証済みゲインを既定ファイルとして保存する
repo_root = fileparts(fileparts(mfilename('fullpath')));
gain_source = fullfile(repo_root, 'controller', 'mec_gain_regularized.m');
write_gain_source(gain_source, gain);

save(fullfile(setting.output_dir, 'mec_model_regularized.mat'), ...
    'est', 'report', 'setting');
save(fullfile(setting.output_dir, 'mec_gain_regularized.mat'), ...
    '-struct', 'gain');

fprintf('同定結果：%s\nゲイン：%s\n', model_file, gain_file);
end

function write_gain_source(file, gain)
% MATファイルが共有されない環境でも使えるよう、ゲインをMATLABコードで保存する。
file_id = fopen(file, 'w');
assert(file_id >= 0, 'MEC:GainSource', ...
    'ゲイン係数のMATLABソースを書き込めません：%s', file);

close_file = onCleanup(@() fclose(file_id)); %#ok<NASGU>

fprintf(file_id, 'function gain = mec_gain_regularized()\n');
fprintf(file_id, '%% 試行分割検証と非線形シミュレーションを通過したゲイン。\n');
fprintf(file_id, 'gain.K_full = %s;\n', mat2str(gain.K_full, 17));
fprintf(file_id, 'gain.error_definition = ''%s'';\n', gain.error_definition);
fprintf(file_id, 'gain.dt = %.17g;\n', gain.dt);
fprintf(file_id, 'gain.design_method = ''%s'';\n', gain.design_method);
fprintf(file_id, 'end\n');
end
