function [est, report] = fit_mec_error_model(trials, setting)
% 誤差系 e[k+1] ≈ A*e[k] + B*dU[k] を試行単位で同定する。
% 状態・入力の尺度をそろえ、弱い状態方向と過剰適合を抑える。

%% 試行を学習・調整・最終評価に分ける
trial_count = numel(trials);
train_count = max(1, floor(trial_count * setting.train_fraction));
tune_count = max(1, floor(trial_count * setting.tune_fraction));

assert(train_count + tune_count < trial_count, ...
    'MEC:TrialSplit', '最終評価に使う試行が必要です。');

train_ids = 1:train_count;
tune_ids = train_count + (1:tune_count);
test_ids = train_count + tune_count + 1:trial_count;

% 学習用試行の誤差・入力・次の誤差を、それぞれ横方向に連結する。
error_data = [];
input_data = [];
next_error_data = [];
for trial_id = train_ids
    error_data = [error_data, trials{trial_id}.E]; %#ok<AGROW>
    input_data = [input_data, trials{trial_id}.dU]; %#ok<AGROW>
    next_error_data = [next_error_data, trials{trial_id}.E_next]; %#ok<AGROW>
end

%% 常にゼロの成分を除き、誤差状態の尺度をそろえる
% 26番目までのうち、16番目は定数観測量の差で常にゼロ。
active_state_ids = [1:15, 17:26];
state_scale = sqrt(mean(error_data(active_state_ids, :).^2, 2));
state_scale = max(state_scale, 1e-12);

input_scale = sqrt(mean(input_data.^2, 2));
assert(all(input_scale > 1e-12), ...
    'MEC:InputExcitation', '4軸すべてに入力励振が必要です。');

%% SVDで情報の弱い状態方向を除く
scaled_error = error_data(active_state_ids, :) ./ state_scale;
[state_basis, state_singular_matrix, ~] = svd(scaled_error, 'econ');
state_singular_values = diag(state_singular_matrix);

effective_rank = sum(state_singular_values >= ...
    setting.state_svd_ratio * state_singular_values(1));
state_basis = state_basis(:, 1:effective_rank);

% encode：26次元の誤差を、低次元の状態へ写す。
% decode：低次元の状態を、26次元の誤差へ戻す。
encode = zeros(effective_rank, 26);
encode(:, active_state_ids) = state_basis' ./ state_scale';

decode = zeros(26, effective_rank);
decode(active_state_ids, :) = state_scale .* state_basis;

%% 回帰行列を作り、特異値と入力ランクを確認する
% z[k] = [低次元の誤差状態; 正規化した入力] として回帰する。
regressor = [encode * error_data; input_data ./ input_scale];
[regressor_left, regressor_singular_matrix, regressor_right] = ...
    svd(regressor, 'econ');
regressor_singular_values = diag(regressor_singular_matrix);

assert(rank(input_data ./ input_scale) == 4, ...
    'MEC:InputRank', '4入力が独立に励振されていません。');

scaled_next_error = encode * next_error_data;
best_score = inf;
candidate_models = struct([]);

%% 正則化係数ごとにモデルを作り、調整用試行で比較する
for candidate_id = 1:numel(setting.ridge_candidates)
    ridge_lambda = setting.ridge_candidates(candidate_id);

    % SVDを使ってridge回帰を解き、正規方程式による条件数の悪化を避ける。
    ridge_weights = regressor_singular_values ./ ...
        (regressor_singular_values.^2 + size(regressor, 2) * ridge_lambda);
    numerical_zero = max(size(regressor)) * eps(regressor_singular_values(1));
    ridge_weights(regressor_singular_values < numerical_zero) = 0;

    coefficient = ((scaled_next_error * regressor_right) .* ridge_weights') ...
        * regressor_left';

    model = struct( ...
        'A', coefficient(:, 1:effective_rank), ...
        'B', coefficient(:, effective_rank+1:end) ./ input_scale', ...
        'encode', encode, ...
        'decode', decode);

    % 同定結果を、MECKCが使う元の26次元の誤差座標に戻す。
    model.A_original = decode * model.A * encode;
    model.B_original = decode * model.B;

    scores = evaluate_model( ...
        model, trials(tune_ids), state_scale, setting.validation_horizon);

    candidate_models(candidate_id).lambda = ridge_lambda;
    candidate_models(candidate_id).scores = scores;
    candidate_models(candidate_id).rho = max(abs(eig(model.A)));

    % 誤差モデルは毎時刻の誤差を状態として使う。
    % ここでは、予測誤差が小さく、無入力応答も発散しない候補を選ぶ。
    score = scores.one_step_scaled + scores.rollout_scaled;
    if candidate_models(candidate_id).rho < 1 && score < best_score
        est = model;
        best_score = score;
        best_candidate_id = candidate_id;
    end
end

assert(isfinite(best_score), ...
    'MEC:NoValidatedModel', ...
    '安定な候補がありません。励振条件またはデータ範囲を見直してください。');

%% 選択したモデルと、調整・最終評価の結果をまとめる
est.error_definition = trials{1}.metadata.error_definition;
est.dt = trials{1}.metadata.dt;
est.scale_e = state_scale;
est.scale_u = input_scale;
est.state_singular = state_singular_values;
est.lambda = candidate_models(best_candidate_id).lambda;
est.C = decode;

report = struct( ...
    'train_ids', train_ids, ...
    'tune_ids', tune_ids, ...
    'test_ids', test_ids, ...
    'source_files', {cellfun(@(data)data.source_file, trials, 'UniformOutput', false)}, ...
    'raw_condition', cond([error_data(active_state_ids, :); input_data]), ...
    'scaled_condition', cond([scaled_error; input_data ./ input_scale]), ...
    'reduced_condition', regressor_singular_values(1) / regressor_singular_values(end), ...
    'effective_rank', effective_rank, ...
    'candidates', candidate_models, ...
    'selected_index', best_candidate_id, ...
    'tune', candidate_models(best_candidate_id).scores, ...
    'test', evaluate_model( ...
        est, trials(test_ids), state_scale, setting.validation_horizon));

fprintf('誤差状態：25 -> %d、条件数：%.3g -> %.3g、lambda=%.3g\n', ...
    effective_rank, report.raw_condition, report.reduced_condition, est.lambda);
fprintf(['未学習試行：1-step RMSE=%.4g、正規化RMSE=%.4g、' ...
    '%d-step正規化RMSE=%.4g\n'], ...
    report.test.rmse, sqrt(report.test.one_step_scaled), ...
    setting.validation_horizon, sqrt(report.test.rollout_scaled));
end

function scores = evaluate_model(model, trials, state_scale, horizon)
% 1ステップ予測と、実測誤差に戻さない連続予測を評価する。
active_state_ids = [1:15, 17:26];
one_step_errors = [];
rollout_errors = [];
physical_errors = [];

for trial_id = 1:numel(trials)
    data = trials{trial_id};

    % 各時刻の実測誤差を使った1ステップ予測。
    predicted_next_error = ...
        model.A_original * data.E + model.B_original * data.dU;
    residual = predicted_next_error - data.E_next;
    one_step_errors = [one_step_errors, ...
        residual(active_state_ids, :) ./ state_scale]; %#ok<AGROW>
    physical_errors = [physical_errors, residual]; %#ok<AGROW>

    % horizon区間の先頭だけ実測値で初期化し、以降は推定値を引き継ぐ。
    for first_sample = 1:horizon:size(data.E, 2)-horizon+1
        last_sample = first_sample + horizon - 1;

        % サンプル間隔が崩れた場所をまたいだ予測は行わない。
        if isfield(data, 'T') && any(abs( ...
                diff(data.T(first_sample:last_sample)) - data.metadata.dt) > 1e-8)
            continue;
        end

        predicted_state = model.encode * data.E(:, first_sample);
        for sample_id = first_sample:last_sample
            predicted_state = model.A * predicted_state ...
                + model.B * data.dU(:, sample_id);
            predicted_error = model.decode * predicted_state;
            rollout_errors(:, end+1) = ...
                (predicted_error(active_state_ids) ...
                - data.E_next(active_state_ids, sample_id)) ./ state_scale; %#ok<AGROW>
        end
    end
end

assert(~isempty(rollout_errors), ...
    'MEC:ShortValidation', '連続した検証データが不足しています。');

scores = struct( ...
    'one_step_scaled', mean(one_step_errors(:).^2), ...
    'rollout_scaled', mean(rollout_errors(:).^2), ...
    'rmse', sqrt(mean(physical_errors(:).^2)));
end
