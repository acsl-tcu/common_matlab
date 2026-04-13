%% edmdforlpvdata.m
% ================================================================
% EDMD residual learning for quadrotor logs
%
% Observable:
%   z = [common_z; kyo_z]
%   common_z = [P; Q; V; W; R13; R23; R33; 1]  -> 16
%   kyo_z    = user nonlinear terms             -> 10
%   dim(z) = 26
%
% Learning flow:
%   1) z(k+1) ~= A_nom * z(k) + B_nom * u_nom(k)
%   2) z_bar(k+1) = z(k+1) - (A_nom*z(k) + B_nom*u_nom(k))
%      z_bar(k+1) ~= A_err * z(k) + B_err * delta_u(k)
%      delta_u(k) = u_raw(k) - u_nom(k)
% ================================================================
clear; clc;

%% Settings
data_dir = 'C:\Users\student\Documents\GitHub\common_matlab\Data\Sim_data';
file_pattern = 'KQLMPC_*.mat';
save_name = 'edmd_residual_model_kyo.mat';

lambda_nom = 1e-6;
lambda_err = 1e-3;

pos_err_thresh = 1.5;
vel_thresh = 5.0;
ang_thresh = pi / 3;
obs_norm_thresh = 200.0;
res_norm_thresh = 50.0;
cos_guard = 1e-3;
du_ratio_thresh = 0.35;
du_abs_thresh = 3.0;
jump_norm_hard_thresh = 80.0;
mad_scale = 6.0;

use_gpu = (gpuDeviceCount > 0);
if use_gpu
    fprintf('[GPU] %s\n', gpuDevice().Name);
else
    fprintf('[CPU] GPU not found, running on CPU.\n');
end

fprintf('\n=== Load dataset ===\n');
files = dir(fullfile(data_dir, file_pattern));
if isempty(files)
    error('No files matched: %s', fullfile(data_dir, file_pattern));
end
fprintf('Files found: %d\n', numel(files));

Z = [];
ZN = [];
U_NOM = [];
DU = [];

stats = struct();
stats.file_used = 0;
stats.n_raw = 0;
stats.n_clean = 0;
stats.n_missing_nominal = 0;

for fi = 1:numel(files)
    fpath = fullfile(data_dir, files(fi).name);
    fprintf('[%3d/%d] %s ... ', fi, numel(files), files(fi).name);

    try
        tmp = load(fpath);
        log_data = tmp.log;
    catch ME
        fprintf('load failed: %s\n', ME.message);
        continue;
    end

    try
        results_est = log_data.Data.agent.estimator.result;
        results_ref = log_data.Data.agent.reference.result;
        results_ctl = log_data.Data.agent.controller.result;
        results_inp = log_data.Data.agent.input;
        N_steps = numel(results_est);
    catch ME
        fprintf('invalid log structure: %s\n', ME.message);
        continue;
    end

    if N_steps < 2
        fprintf('too short\n');
        continue;
    end

    stats.file_used = stats.file_used + 1;

    traj = nan(N_steps, 12);
    ref_seq = nan(N_steps, 9);
    u_raw_seq = nan(N_steps, 4);
    u_nom_seq = nan(N_steps, 4);
    has_nom_seq = false(N_steps, 1);

    for k = 1:N_steps
        try
            s = results_est{1, k}.state;
            traj(k, :) = [double(s.p(:))', double(s.q(:))', ...
                          double(s.v(:))', double(s.w(:))'];

            sr = results_ref{1, k}.state;
            ref_seq(k, :) = [double(sr.p(:))', double(sr.q(:))', ...
                             double(sr.v(:))'];

            u_raw_seq(k, :) = reshape(double(results_inp{1, k}(1:4)), 1, []);

            [u_nom_k, has_nom_k] = extract_nominal_input(results_ctl{1, k});
            if has_nom_k
                u_nom_seq(k, :) = u_nom_k(:)';
                has_nom_seq(k) = true;
            end
        catch
        end
    end

    n_clean_file = 0;
    for k = 1:(N_steps - 1)
        if any(isnan(traj(k, :))) || any(isnan(traj(k + 1, :))) || ...
           any(isnan(u_raw_seq(k, :))) || any(isnan(u_raw_seq(k + 1, :))) || ...
           any(isnan(ref_seq(k, :)))
            continue;
        end

        stats.n_raw = stats.n_raw + 1;

        x12_t = traj(k, :)';
        x12_tp1 = traj(k + 1, :)';
        u_raw_t = u_raw_seq(k, :)';
        u_raw_tp1 = u_raw_seq(k + 1, :)';

        if has_nom_seq(k)
            u_nom_t = u_nom_seq(k, :)';
        else
            u_nom_t = u_raw_t;
            stats.n_missing_nominal = stats.n_missing_nominal + 1;
        end
        delta_u = u_raw_t - u_nom_t;

        x16_t = [x12_t; u_raw_t];
        x16_tp1 = [x12_tp1; u_raw_tp1];
        z_t = build_kyo_observable(x16_t, cos_guard);
        z_tp1 = build_kyo_observable(x16_tp1, cos_guard);

        if norm(x12_t(1:3) - ref_seq(k, 1:3)') > pos_err_thresh
            continue;
        end
        if norm(x12_t(7:9)) > vel_thresh
            continue;
        end
        if any(abs(x12_t(4:6)) > ang_thresh)
            continue;
        end
        if norm(z_t) > obs_norm_thresh || norm(z_tp1) > obs_norm_thresh
            continue;
        end

        Z = [Z, z_t]; %#ok<AGROW>
        ZN = [ZN, z_tp1]; %#ok<AGROW>
        U_NOM = [U_NOM, u_nom_t]; %#ok<AGROW>
        DU = [DU, delta_u]; %#ok<AGROW>

        n_clean_file = n_clean_file + 1;
        stats.n_clean = stats.n_clean + 1;
    end

    fprintf('raw=%d clean=%d\n', N_steps - 1, n_clean_file);
end

fprintf('\n=== Data stats ===\n');
fprintf('used files            : %d\n', stats.file_used);
fprintf('raw transitions       : %d\n', stats.n_raw);
fprintf('clean transitions     : %d\n', stats.n_clean);
fprintf('missing nominal input : %d\n', stats.n_missing_nominal);

if isempty(Z)
    error('No valid samples remained after filtering.');
end

du_norm_all = vecnorm(DU, 2, 1);
stats.du_nonzero_count = nnz(du_norm_all > 1e-8);
stats.du_nonzero_ratio = stats.du_nonzero_count / numel(du_norm_all);
stats.du_mean_norm = mean(du_norm_all);
stats.du_max_norm = max(du_norm_all);
fprintf('delta_u nonzero count  : %d / %d\n', stats.du_nonzero_count, numel(du_norm_all));
fprintf('delta_u nonzero ratio  : %.4f\n', stats.du_nonzero_ratio);
fprintf('delta_u mean norm      : %.6e\n', stats.du_mean_norm);
fprintf('delta_u max norm       : %.6e\n', stats.du_max_norm);

%% Robust outlier pruning for manually injected bias/error data
fprintf('\n=== Robust outlier pruning ===\n');
du_norm = vecnorm(DU, 2, 1);
u_nom_norm = vecnorm(U_NOM, 2, 1);
du_ratio = du_norm ./ max(u_nom_norm, 1e-6);
z_jump = vecnorm(ZN - Z, 2, 1);

mask_du_abs = du_norm <= du_abs_thresh;
mask_du_ratio = du_ratio <= du_ratio_thresh;
mask_jump_hard = z_jump <= jump_norm_hard_thresh;
mask_du_mad = robust_upper_mask(du_norm, mad_scale);
mask_jump_mad = robust_upper_mask(z_jump, mad_scale);

mask_keep = mask_du_abs & mask_du_ratio & mask_jump_hard & ...
            mask_du_mad & mask_jump_mad;

fprintf('kept after robust pruning: %d / %d\n', nnz(mask_keep), numel(mask_keep));
fprintf('removed by |du|          : %d\n', nnz(~mask_du_abs));
fprintf('removed by du ratio      : %d\n', nnz(~mask_du_ratio));
fprintf('removed by jump hard     : %d\n', nnz(~mask_jump_hard));
fprintf('removed by du MAD        : %d\n', nnz(~mask_du_mad));
fprintf('removed by jump MAD      : %d\n', nnz(~mask_jump_mad));

Z = Z(:, mask_keep);
ZN = ZN(:, mask_keep);
U_NOM = U_NOM(:, mask_keep);
DU = DU(:, mask_keep);
stats.n_clean_after_prune = size(Z, 2);

if isempty(Z)
    error('All samples were removed by robust pruning. Relax thresholds.');
end

n_z = size(Z, 1);
fprintf('observable dimension  : %d\n', n_z);

%% Step 1: nominal EDMD
fprintf('\n=== Train nominal EDMD ===\n');
[A_nom, B_nom] = solve_ridge(Z, U_NOM, ZN, lambda_nom, use_gpu);
ZN_nom = A_nom * Z + B_nom * U_NOM;
rmse_nom = sqrt(mean((ZN - ZN_nom) .^ 2, 'all'));
fprintf('nominal RMSE: %.6e\n', rmse_nom);

%% Step 2: residual EDMD
fprintf('\n=== Train residual EDMD ===\n');
ZBAR = ZN - ZN_nom;
keep_idx = vecnorm(ZBAR, 2, 1) <= res_norm_thresh;

Z2 = Z(:, keep_idx);
DU2 = DU(:, keep_idx);
ZBAR2 = ZBAR(:, keep_idx);

fprintf('residual samples kept: %d / %d\n', size(Z2, 2), size(Z, 2));
if isempty(Z2)
    error('No residual samples remained after residual filtering.');
end

[A_err, B_err] = solve_ridge(Z2, DU2, ZBAR2, lambda_err, use_gpu);
ZBAR_hat = A_err * Z2 + B_err * DU2;
rmse_err = sqrt(mean((ZBAR2 - ZBAR_hat) .^ 2, 'all'));
rho_err = max(abs(eig(A_err)));
fprintf('residual RMSE: %.6e\n', rmse_err);
fprintf('rho(A_err): %.6f\n', rho_err);
fprintf('rank(B_err): %d / %d\n', rank(B_err), size(B_err, 2));
fprintf('||B_err||_F: %.6e\n', norm(B_err, 'fro'));
fprintf('||B_err(:,1)||: %.6e\n', norm(B_err(:,1)));
fprintf('||B_err(:,2)||: %.6e\n', norm(B_err(:,2)));
fprintf('||B_err(:,3)||: %.6e\n', norm(B_err(:,3)));
fprintf('||B_err(:,4)||: %.6e\n', norm(B_err(:,4)));

A_total = A_nom + A_err;
B_delta = B_err;
ZN_total = A_nom * Z + B_nom * U_NOM + A_err * Z + B_err * DU;
rmse_total = sqrt(mean((ZN - ZN_total) .^ 2, 'all'));
fprintf('\ncombined RMSE: %.6e\n', rmse_total);

results_edmd = struct();
results_edmd.A_nom = A_nom;
results_edmd.B_nom = B_nom;
results_edmd.A_err = A_err;
results_edmd.B_err = B_err;
results_edmd.A_total = A_total;
results_edmd.B_delta = B_delta;
results_edmd.rmse_nom = rmse_nom;
results_edmd.rmse_err = rmse_err;
results_edmd.rmse_total = rmse_total;
results_edmd.rho_A_err = rho_err;
results_edmd.n_z = n_z;
results_edmd.N = size(Z, 2);
results_edmd.observable_name = 'common_z_plus_kyo_z';
results_edmd.stats = stats;
results_edmd.keep_idx_residual = keep_idx;
results_edmd.du_nonzero_count = stats.du_nonzero_count;
results_edmd.du_nonzero_ratio = stats.du_nonzero_ratio;
results_edmd.du_mean_norm = stats.du_mean_norm;
results_edmd.du_max_norm = stats.du_max_norm;

save_path = fullfile(data_dir, save_name);
save(save_path, 'results_edmd', 'lambda_nom', 'lambda_err', ...
    'pos_err_thresh', 'vel_thresh', 'ang_thresh', ...
    'obs_norm_thresh', 'res_norm_thresh', 'cos_guard', ...
    'du_ratio_thresh', 'du_abs_thresh', 'jump_norm_hard_thresh', ...
    'mad_scale');
fprintf('\nSaved: %s\n', save_path);

function [A_mat, B_mat] = solve_ridge(Z_cur, U_cur, Z_next, lambda, use_gpu)
Phi = [Z_cur; U_cur];
if use_gpu
    Phi_g = gpuArray(single(Phi));
    Z_g = gpuArray(single(Z_next));
    gram = Phi_g * Phi_g' + lambda * eye(size(Phi_g, 1), 'like', Phi_g);
    AB_g = Z_g * Phi_g' / gram;
    AB = double(gather(AB_g));
else
    gram = Phi * Phi' + lambda * eye(size(Phi, 1));
    AB = Z_next * Phi' / gram;
end
nz = size(Z_cur, 1);
A_mat = AB(:, 1:nz);
B_mat = AB(:, nz + 1:end);
end

function [u_nom, ok] = extract_nominal_input(ctrl_result)
u_nom = nan(4, 1);
ok = false;

if isempty(ctrl_result)
    return;
end

if isstruct(ctrl_result) && isfield(ctrl_result, 'input') && isfield(ctrl_result, 'deltau')
    val_input = ctrl_result.input;
    val_du = ctrl_result.deltau;
    if isnumeric(val_input) && isnumeric(val_du) && numel(val_input) >= 4 && numel(val_du) >= 4
        u_nom = reshape(double(val_input(1:4)), [], 1) - reshape(double(val_du(1:4)), [], 1);
        ok = true;
        return;
    end
end

candidates = {'u_nom', 'unom', 'input_nom', 'nominal_input', 'u'};
for i = 1:numel(candidates)
    name = candidates{i};
    if isstruct(ctrl_result) && isfield(ctrl_result, name)
        val = ctrl_result.(name);
        if isnumeric(val) && numel(val) >= 4
            u_nom = reshape(double(val(1:4)), [], 1);
            ok = true;
            return;
        end
    end
end

if isstruct(ctrl_result) && isfield(ctrl_result, 'pre_u')
    val = ctrl_result.pre_u;
    if isnumeric(val) && numel(val) >= 4
        u_nom = reshape(double(val(1:4)), [], 1);
        ok = true;
        return;
    end
end

if isstruct(ctrl_result) && isfield(ctrl_result, 'var')
    val = ctrl_result.var;
    if isnumeric(val) && numel(val) >= 4
        u_nom = reshape(double(val(1:4)), [], 1);
        ok = true;
        return;
    end
end

end

function z = build_kyo_observable(x, cos_guard)
P1 = x(1);
P2 = x(2);
P3 = x(3);
Q1 = x(4);
Q2 = x(5);
Q3 = x(6);
V1 = x(7);
V2 = x(8);
V3 = x(9);
W1 = x(10);
W2 = x(11);
W3 = x(12);
U1 = x(13);
U2 = x(14);
U3 = x(15);
U4 = x(16);

c1 = cos(Q1);
s1 = sin(Q1);
c2 = cos(Q2);
s2 = sin(Q2);
c3 = cos(Q3);
s3 = sin(Q3);

c1_safe = guard_abs(c1, cos_guard);
c2_safe = guard_abs(c2, cos_guard);

R13 = c3 * s2 * c1 + s3 * s1;
R23 = s3 * s2 * c1 - c3 * s1;
R33 = c2 * c1;

common_z = [P1; P2; P3; ...
            Q1; Q2; Q3; ...
            V1; V2; V3; ...
            W1; W2; W3; ...
            R13; R23; R33; ...
            1];

kyo_z = [W1 * W2; ...
         W2 * W3; ...
         W3 * W1; ...
         W2 * c1; ...
         W3 * s1; ...
         W1 * c2 / c1_safe; ...
         W2 * s1 / c2_safe; ...
         W3 * c1 / c2_safe; ...
         W2 * s1 * s2 / c2_safe; ...
         W3 * c1 * s2 / c2_safe];

dummy_u = [U1; U2; U3; U4]; %#ok<NASGU>
z = [common_z; kyo_z];
end

function y = guard_abs(x, eps_val)
if abs(x) < eps_val
    y = sign_nonzero(x) * eps_val;
else
    y = x;
end
end

function s = sign_nonzero(x)
if x >= 0
    s = 1;
else
    s = -1;
end
end

function mask = robust_upper_mask(v, scale)
med_v = median(v);
mad_v = median(abs(v - med_v));
sigma_v = 1.4826 * mad_v;
if sigma_v < 1e-12
    mask = true(size(v));
else
    mask = v <= (med_v + scale * sigma_v);
end
end
