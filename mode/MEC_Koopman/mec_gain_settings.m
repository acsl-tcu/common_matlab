function setting = mec_gain_settings()
% 同定とゲイン設計の共通条件。変更後はtrain_mec_gainを実行して再計算する。
repo_root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
setting.dataset_dir = fullfile(repo_root,'datasets','MEC_4input');
if isempty(dir(fullfile(setting.dataset_dir,'*.mat')))
    setting.dataset_dir = fullfile(repo_root,'datasets'); % 新形式の情報を持つ通常ログも読める
end
setting.output_dir = fullfile(repo_root,'Data','MEC_design');
% 同一試行を分割せず、学習60%、調整20%、最終評価20%に分ける。
setting.train_fraction = 0.6;
setting.tune_fraction = 0.2;
% 正規化した誤差の特異値が最大値のこの比率未満なら、設計対象から除く。
% 微小な差から巨大なゲインを作らないための実効ランク判定。
setting.state_svd_ratio = 1e-3;
setting.ridge_candidates = [0,logspace(-6,0,7)]; % 正則化係数を調整用の試行で選ぶ
setting.validation_horizon = 20; % 1ステップに加え、20ステップの自由応答予測を確認
% 最初の12誤差の許容スケール：[位置m、姿勢rad、速度m/s、角速度rad/s]。
% Q=diag(1./scale.^2)。派生観測量を重複して罰しない。
setting.error_scale = [0.05*ones(3,1);0.05*ones(3,1);0.1*ones(6,1)];
% 補償入力の設計スケール：[推力N; roll; pitch; yaw のトルクN m]。
% 小さくすると入力コストRが大きくなり、補償を穏やかにする。飽和制限ではない。
setting.input_scale = [0.5;0.03;0.03;0.02];
setting.input_cost_multiplier = 1;
end
