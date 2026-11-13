% Train the HLC_EDMD_MEC model from the no-payload flight logs.
repoRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(genpath(repoRoot));
dataDir = fullfile(repoRoot, 'Data', '重り無し');
nominalPattern = 'HL_*_nopayload_Log*.mat';
excitedPattern = fullfile('Reiki', 'HL_Reiki_*_Nopayload_*_Log*.mat');

nominalFiles = dir(fullfile(dataDir, nominalPattern));
excitedFiles = dir(fullfile(dataDir, excitedPattern));
assert(numel(nominalFiles) == 12, 'Expected 12 no-payload nominal logs.');
assert(numel(excitedFiles) == 12, 'Expected 12 no-payload excited logs.');

modelFile = fullfile(dataDir, 'hl_edmd_mec_nopayload.mat');
opts = struct('data_dir', dataDir, ...
    'nominal_file_pattern', nominalPattern, ...
    'degraded_file_pattern', excitedPattern, ...
    'save_path', modelFile, ...
    'state_source', 'estimator', ...
    'use_gpu', false, ...
    'nominal_use_actual_input', true, ...
    'training_phase', 'f');

results = train_edmd_residual_paired_from_logs(opts);
assert(results.stats.nominal.file_used == 12, 'Not all nominal logs were used.');
assert(results.stats.degraded.file_used == 12, 'Not all excited logs were used.');
assert(results.n_z == 26 && all(size(results.B_err) == [26, 4]), ...
    'Unexpected EDMD model dimensions.');
fprintf('HLC_EDMD_MEC no-payload model: %s\n', modelFile);
