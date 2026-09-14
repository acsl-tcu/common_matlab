%% edmdforlpvdata_plant.m
clear; clc;

opts = struct();
opts.data_dir = 'C:\Users\student\Documents\GitHub\common_matlab\Data\Sim_data';
opts.file_pattern = 'KQLMPC_*.mat';
opts.save_path = 'C:\Users\student\Documents\GitHub\common_matlab\mode\KMPC\KQLMPC\edmd_residual_model_lpv_plant.mat';
opts.state_source = 'plant';

train_edmd_residual_from_logs(opts);