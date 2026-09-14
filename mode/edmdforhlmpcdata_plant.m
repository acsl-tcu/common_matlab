%% edmdforhlmpcdata_plant.m
% HL-MPC 劣化データから残差EDMDモデルを学習（ノイズ環境向け強正則化）
clear; clc;

opts = struct();
opts.data_dir     = 'C:\Users\student\Documents\GitHub\common_matlab\Data\Sim_data';
opts.file_pattern = 'HLMPC_degraded_*.mat';   % degradedのみ学習に使う
opts.save_path    = 'C:\Users\student\Documents\GitHub\common_matlab\mode\KMPC\KQLMPC\edmd_residual_model_hlmpc_plant.mat';
opts.state_source = 'plant';

% ノイズ環境で滑らかな補償を得るための正則化（HLで検証済み）
opts.lambda_nom      = 1e-4;
opts.lambda_err      = 1e-1;
opts.res_norm_thresh = 20.0;
opts.mad_scale       = 4.0;

results_edmd = train_edmd_residual_from_logs(opts);

fprintf('\n===== HL-MPC 残差学習結果 =====\n');
fprintf('observable dim : %d\n', results_edmd.n_z);
fprintf('||B_err||_F    : %.4e\n', norm(results_edmd.B_err,'fro'));
fprintf('B_err col1(thrust) : %.4e\n', norm(results_edmd.B_err(:,1)));
fprintf('B_err col2(roll)   : %.4e\n', norm(results_edmd.B_err(:,2)));
fprintf('B_err col3(pitch)  : %.4e\n', norm(results_edmd.B_err(:,3)));
if norm(results_edmd.B_err(:,1)) < 0.05
    warning('B_err推力列が小さい(<0.05)。質量残差が十分学習されていない可能性。');
end
