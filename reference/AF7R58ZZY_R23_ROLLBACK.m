function AF7R58ZZY_R23_ROLLBACK
runner=which('run_flight_headless');
assert(~isempty(runner)&&isfile(runner),'[R23 rollback] runner not found.');
root=fileparts(runner);
bak=fullfile(root,'run_flight_headless.R23_PREINSTALL_BACKUP.m');
assert(isfile(bak),'[R23 rollback] exact preinstall runner backup not found.');
copyfile(bak,runner,'f');
rm={ ...
    'REPLANNING_SYSTEM_R23_COMPILED.m', ...
    ['af7r76_compiled_eval_mex.' mexext], ...
    'af7r76_compile_correction_packet_R23.m', ...
    'AF7R58ZZY_R23_INSTALL_MANIFEST.txt'};
for k=1:numel(rm)
    p=fullfile(root,rm{k}); if isfile(p), delete(p); end
end
rehash;
fprintf('[R23] rollback restored the exact pre-R23 runner.\n');
end
