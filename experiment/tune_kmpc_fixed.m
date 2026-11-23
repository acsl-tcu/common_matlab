% Full GUI simulation trials using the corrected physical-coordinate model.
cd(fileparts(fileparts(mfilename('fullpath'))));
addpath(genpath(pwd));
fprintf('MATLAB %s; quadprog implementations:\n',version);
disp(which('quadprog','-all'));
base = Controller_MPC_Koopman([]);
weights = repmat(base.param.weight,1,3);
weights(2).P = 2*weights(1).P;
weights(3).P = 4*weights(1).P;
for trialIndex = 1:3
    weights(trialIndex).Pf = weights(trialIndex).P;
end
metrics = struct([]);
for trialIndex = 1:3
    rng(1);
    Setting = struct('fDebug',1,'PInterval',0.6,'mode',"SimMPC_Koopman", ...
        'fExp',0,'trajectory_seed',1,'mpc_weight',weights(trialIndex));
    app = SimExp(Setting);
    transcript = evalc('app.run_auto_trials(1,"KMPC_weight"+trialIndex);');
    fid=fopen(fullfile(tempdir,sprintf('kmpc_trial_%d.log',trialIndex)),'w');
    fprintf(fid,'%s',transcript); fclose(fid);
    n=app.logger.k; d=app.logger.Data; a=d.agent(1);
    flight=find(d.phase(1:n)==double('f'));
    pos=cell2mat(cellfun(@(r) r.state.p(:),a.plant.result(flight),'UniformOutput',false));
    ref=cell2mat(cellfun(@(r) r.state.p(:),a.reference.result(flight),'UniformOutput',false));
    u=cell2mat(a.input(1:n));
    flags=cellfun(@(r) r.exitflag,a.controller.result(flight));
    metrics(trialIndex).end_time=app.time.t;
    metrics(trialIndex).samples=n;
    metrics(trialIndex).rmse=sqrt(mean(sum((pos-ref).^2,1)));
    metrics(trialIndex).max_input=max(abs(u),[],2);
    metrics(trialIndex).min_exitflag=min(flags);
    metrics(trialIndex).complete=app.time.t>=58.99 && all(isfinite([pos(:);u(:)])) && all(flags>0);
    fprintf('TRIAL %d complete=%d time=%.3f n=%d RMSE=%.6g maxU=%s minFlag=%d\n', ...
        trialIndex,metrics(trialIndex).complete,app.time.t,n,metrics(trialIndex).rmse,mat2str(metrics(trialIndex).max_input',5),min(flags));
    save(fullfile('Data','Sim_data','KMPC_tuning_metrics.mat'),'metrics','weights');
    delete(app);
end
assert(all([metrics.complete]),'At least one full simulation failed.');
