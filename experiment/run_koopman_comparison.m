function results = run_koopman_comparison()
% Compare predictor candidates on one fixed nonlinear Euler-angle plant.
root = fileparts(fileparts(mfilename('fullpath'))); cd(root); addpath(genpath(root));
modelFiles = ["KL_26_MPC_fixed.mat"; "KL_candidate_trig.mat"];
observableNames = ["quaternions_all"; "observables_candidate_trig"];
for k = 1:numel(modelFiles)
    output = fullfile(root,'Koopman_Linearization','Koopman_Linear_by_Data','refult_of_KL_Miya',modelFiles(k));
    if ~exist(output,'file')
        train_koopman_candidate(erase(modelFiles(k),'.mat'),observableNames(k));
    end
end
results = struct([]);
for k = 1:numel(modelFiles)
    Setting = struct('fDebug',1,'PInterval',0.6,'mode',"SimMPC_Koopman",'fExp',0, ...
        'trajectory_seed',1,'prediction_model',modelFiles(k),'plant_model',"EulerAngle");
    app = SimExp(Setting);
    capture = evalc('app.run_auto_trials(1,"KoopmanCompare"+k);');
    results(k).model = modelFiles(k);
    results(k).observable = observableNames(k);
    results(k).complete = app.time.t >= 58.99;
    results(k).samples = app.logger.k;
    fid=fopen(fullfile(tempdir,sprintf('koopman_compare_%d.log',k)),'w'); fprintf(fid,'%s',capture); fclose(fid);
    delete(app);
end
save(fullfile(root,'Data','Sim_data','koopman_observable_comparison_runs.mat'),'results');
assert(all([results.complete]));
end
