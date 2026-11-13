% Collect four SimHL conditions and train an HLC_EDMD_ERROR model.
repoRoot = fileparts(fileparts(mfilename('fullpath')));
cd(repoRoot);
addpath(genpath(pwd));
addpath(pwd, '-begin');
outputDir = fullfile(pwd, 'Data', 'Sim_data', ...
    ['hl_edmd_error_' datestr(now, 'yyyymmdd_HHMMSS')]);
setting = struct('fExp', 0, 'fDebug', 1, 'PInterval', 0.6, ...
    'mode', "SimHL", 'trajectory_seed', 1);
app = SimExp(setting);
cleanupApp = onCleanup(@() delete(app));
app.collect_hl_error_data(outputDir);
opts = struct('data_dir', outputDir, 'file_pattern', 'hl_error_me*_exc*.mat', ...
    'save_path', fullfile(outputDir, 'hl_edmd_error_model.mat'), ...
    'state_source', 'estimator', 'use_gpu', false, 'training_phase', 'f');
results = train_hl_edmd_error_from_logs(opts);
fprintf('LQR: vertical=%d, attitude=%d\n', results.vertical_dlqr_ok, results.attitude_dlqr_ok);
fprintf('Vertical: %s\nAttitude: %s\n', results.vertical_dlqr_message, results.attitude_dlqr_message);
fprintf('Output folder: %s\n', outputDir);
