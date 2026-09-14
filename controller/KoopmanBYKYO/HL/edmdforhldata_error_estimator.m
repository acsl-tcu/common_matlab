opts = struct();
opts.data_dir = 'C:\Users\student\Documents\GitHub\common_matlab\Data\Sim_data';
opts.file_pattern = 'HL_*.mat';
opts.save_path = 'C:\Users\student\Documents\GitHub\common_matlab\mode\KMPC\KQLMPC\hl_edmd_error_model_estimator.mat';
opts.state_source = 'estimator';

train_hl_edmd_error_from_logs(opts);