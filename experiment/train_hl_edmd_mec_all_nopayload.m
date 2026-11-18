% Train from the no-payload flight logs collected in September and November.
repoRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(genpath(repoRoot));
sourceDir = '\\192.168.100.209\ws2026\Work2026\YuriMiyazaki\kyosa_jikken\data_HL_for_edmd\重り無し\dataset';
workDir = fullfile(repoRoot, 'Data', 'hl_edmd_mec_nopayload_training');
nominalDir = fullfile(workDir, 'nominal');
excitedDir = fullfile(workDir, 'excited');
if ~isfolder(nominalDir), mkdir(nominalDir); end
if ~isfolder(excitedDir), mkdir(excitedDir); end
files = dir(fullfile(sourceDir, '*.mat'));
names = string({files.name});
nominal = startsWith(names, 'Dataset_HL_mE0_reiki0_') | ...
    (~startsWith(names, 'Dataset_') & startsWith(names, 'HL_') & contains(lower(names), '_nopayload_'));
excited = startsWith(names, 'Dataset_HL_mE0_reiki1_') | startsWith(names, 'HL_Reiki_');
nominal = nominal & ~excited;
assert(nnz(nominal) == 25 && nnz(excited) == 25, 'Unexpected source file selection.');
for index = 1:numel(files)
    if nominal(index)
        destination = nominalDir;
    elseif excited(index)
        destination = excitedDir;
    else
        continue;
    end
    copyfile(fullfile(files(index).folder, files(index).name), fullfile(destination, files(index).name));
end
opts = struct('data_dir', workDir, ...
    'nominal_file_pattern', fullfile('nominal', '*.mat'), ...
    'degraded_file_pattern', fullfile('excited', '*.mat'), ...
    'save_path', fullfile(workDir, 'hl_edmd_mec_nopayload.mat'), ...
    'state_source', 'estimator', 'use_gpu', false, ...
    'nominal_use_actual_input', true, 'training_phase', 'f');
results = train_edmd_residual_paired_from_logs(opts);
assert(results.stats.nominal.file_used == 25 && results.stats.degraded.file_used == 25);
