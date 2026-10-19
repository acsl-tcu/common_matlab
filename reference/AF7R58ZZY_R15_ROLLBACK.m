function AF7R58ZZY_R15_ROLLBACK
runnerPath=which('run_flight_headless'); assert(~isempty(runnerPath)&&isfile(runnerPath),'[R15] runner not found.');
root=fileparts(runnerPath); backup=fullfile(root,'run_flight_headless.R15_PREINSTALL_BACKUP.m');
assert(isfile(backup),'[R15] rollback backup not found.');
copyfile(backup,runnerPath,'f');
files={ 'REPLANNING_SYSTEM_R15_FAST.m','af7r33_flat_jet_batch_model_R15FAST.m','AF7R58ZZY_R15_INSTALL_MANIFEST.txt'};
for k=1:numel(files), p=fullfile(root,files{k}); if isfile(p), delete(p); end, end
mx={fullfile(root,['af7r70_observer_mex.' mexext]),fullfile(root,['af7r71_decasteljau_batch_mex.' mexext])};
clear REPLANNING_SYSTEM_R15_FAST af7r70_observer_mex af7r71_decasteljau_batch_mex af7r33_flat_jet_batch_model_R15FAST;
for k=1:numel(mx), if isfile(mx{k}), delete(mx{k}); end, end
rehash;
fprintf('[R15] rollback complete. REPLANNING_SYSTEM.m was never modified.\n');
end
