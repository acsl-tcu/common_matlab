% Validate the actual mainGUI entry point and its draw-graph callback.
cd(fileparts(fileparts(mfilename('fullpath'))));
if exist('verifiedLogFile','var')
    % Resume only the checks after a completed, saved mainGUI simulation.
    addpath(genpath(pwd));
    Setting=struct('fDebug',1,'PInterval',0.6,'mode',"SimMPC_Koopman",'fExp',0);
    app=SimExp(Setting);
    app.logger=LOGGER(verifiedLogFile);
    app.time.t=max(app.logger.Data.t)+app.time.dt;
else
    mainGUI;
end
n=app.logger.k;
assert(abs(app.time.t-59)<1e-6 && n==2484,'Simulation did not finish.');
results=app.logger.Data.agent(1).controller.result(1:n);
hasQP=cellfun(@(r) isfield(r,'exitflag'),results);
flags=cellfun(@(r) r.exitflag,results(hasQP));
assert(~isempty(flags) && all(flags>0),'QP failed.');
u=cell2mat(app.logger.Data.agent(1).input(1:n));
assert(all(isfinite(u(:))));
c=app.agent(1).controller.mpc;
p=c.param; p.current=zeros(12,1); p.ref=zeros(16,p.H);
[H,f]=change_equation(p);
minimumEigenvalue=min(eig(H));
assert(minimumEigenvalue>0 && all(isfinite(f)));
app.DrawgraphButton.ButtonPushedFcn(app.DrawgraphButton,[]);
drawnow;
names={'Koopman position','Koopman trajectory','Koopman input'};
for figureIndex=1:numel(names)
    fig=findall(groot,'Type','figure','Name',names{figureIndex});
    assert(~isempty(fig) && ~isempty(findall(fig(1),'Type','line')));
    exportgraphics(fig(1),fullfile('Data','Sim_data',sprintf('KMPC_verified_%d.png',figureIndex)));
end
fprintf('MAIN_GUI_VERIFIED time=%.3f samples=%d QP=%d flags=%s minEigH=%.9g figures=3\n', ...
    app.time.t,n,numel(flags),mat2str(unique(flags)),minimumEigenvalue);
