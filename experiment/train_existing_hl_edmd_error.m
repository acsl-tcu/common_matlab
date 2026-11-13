% Train using flight-only samples from a previously collected dataset.
repoRoot = fileparts(fileparts(mfilename('fullpath')));
cd(repoRoot);
addpath(genpath(pwd));
addpath(pwd, '-begin');
dataDir = fullfile(pwd, 'Data', 'Sim_data', 'hl_edmd_error_20261112_204555');
opts = struct('data_dir', dataDir, 'file_pattern', 'hl_error_me*_exc*.mat', ...
    'save_path', fullfile(dataDir, 'hl_edmd_error_model_flight.mat'), ...
    'state_source', 'estimator', 'use_gpu', false, 'training_phase', 'f');
results = train_hl_edmd_error_from_logs(opts);
assert(results.stats.file_used == 4 && results.vertical_dlqr_ok && results.attitude_dlqr_ok);
assert(norm(results.Bz) > 0 && rank(results.Ba) == 3);
setting = struct('fExp', 0, 'fDebug', 1, 'PInterval', 0.6, 'mode', "SimHL");
app = SimExp(setting);
cleanupApp = onCleanup(@() delete(app));
param = Controller_HL_EDMD_ERROR(app.time.dt, opts.save_path, 'estimator');
controller = HLC_EDMD_ERROR(app.agent(1), param);
assert(controller.comp.loaded, 'Model was not loaded.');
app.agent(1).controller.set_function_class("hlc", controller);
for phase = ['a', 't', 'f', 'l']
    app.cha = phase;
    app.motive.getData(app.agent);
    evalc('app.do_calculation();');
    assert(all(isfinite(controller.result.input)));
end
fprintf('HL_ERROR_MODEL_SMOKE_OK Bz=%g Ba_rank=%d Kz=%g Ka=%g\n', ...
    norm(results.Bz), rank(results.Ba), norm(results.Kz), norm(results.Ka));
