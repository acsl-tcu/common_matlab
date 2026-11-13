% Train HLC_EDMD_MEC from the latest 12 SimHL Dataset logs (three trajectories x four cases).
repoRoot = fileparts(fileparts(mfilename('fullpath')));
cd(repoRoot);
addpath(genpath(repoRoot));
addpath(repoRoot, '-begin');
sourceDir = fullfile(repoRoot, 'Data', 'Sim_data');
outputDir = fullfile(sourceDir, ['edmd_mec_12_' datestr(now, 'yyyymmdd_HHMMSS')]);
mkdir(outputDir);
trajectories = ["circle", "saddle", "figure8"];
for trajectory = trajectories
    for modelError = 0:1
        for excitation = 0:1
            pattern = sprintf('%s*Dataset_ME%d_reiki%d_*.mat', trajectory, modelError, excitation);
            matches = dir(fullfile(sourceDir, pattern));
            assert(~isempty(matches), 'Missing dataset: %s', pattern);
            [~, latest] = max([matches.datenum]);
            copyfile(fullfile(matches(latest).folder, matches(latest).name), ...
                fullfile(outputDir, sprintf('%s_ME%d_reiki%d.mat', trajectory, modelError, excitation)));
        end
    end
end
opts = struct('data_dir', outputDir, ...
    'nominal_file_pattern', '*_ME0_reiki*.mat', ...
    'degraded_file_pattern', '*_ME1_reiki*.mat', ...
    'save_path', fullfile(outputDir, 'hl_edmd_mec_model.mat'), ...
    'state_source', 'estimator', 'use_gpu', false, ...
    'nominal_use_actual_input', true, 'training_phase', 'f');
results = train_edmd_residual_paired_from_logs(opts);
assert(results.stats.nominal.file_used == 6 && results.stats.degraded.file_used == 6);
fprintf('HLC_EDMD_MEC model: %s\n', opts.save_path);
