% Summarize saved comparison logs without rerunning the plant.
root=fileparts(fileparts(mfilename('fullpath'))); cd(root); addpath(genpath(root));
patterns=["KoopmanCompare1*";"KoopmanCompare2*"];
summary=struct([]);
for k=1:numel(patterns)
    files=dir(fullfile('Data','Sim_data',patterns(k)+".mat"));
    assert(~isempty(files),'Missing log for candidate %d',k);
    [~,order]=sort([files.datenum]); files=files(order); s=load(fullfile(files(end).folder,files(end).name));
    d=s.log.Data; n=find(d.t,1,'last'); idx=find(d.phase(1:n)==double('f'));
    a=d.agent(1); p=cell2mat(cellfun(@(r)r.state.p(:),a.plant.result(idx),'UniformOutput',false));
    r=cell2mat(cellfun(@(q)q.state.p(:),a.reference.result(idx),'UniformOutput',false));
    u=cell2mat(a.input(1:n)); c=a.controller.result(idx);
    flags=cellfun(@(q)q.exitflag,c); times=cellfun(@(q)q.calculation_time,c);
    summary(k).log=string(files(end).name); summary(k).time=d.t(n); summary(k).samples=n;
    summary(k).position_rmse=sqrt(mean(sum((p-r).^2,1))); summary(k).position_max_error=max(sqrt(sum((p-r).^2,1)));
    summary(k).max_input=max(abs(u),[],2); summary(k).mean_qp_time=mean(times); summary(k).max_qp_time=max(times); summary(k).min_exitflag=min(flags);
    fprintf('%s time=%.3f RMSE=%.6g maxErr=%.6g meanQP=%.6g maxU=%s flag=%d\n',summary(k).log,summary(k).time,summary(k).position_rmse,summary(k).position_max_error,summary(k).mean_qp_time,mat2str(summary(k).max_input',5),summary(k).min_exitflag);
end
save(fullfile('Data','Sim_data','koopman_observable_comparison.mat'),'summary');
