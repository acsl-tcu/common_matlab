function AF7R58ZZY_R18_ROLLBACK
runner=which('run_flight_headless');
assert(~isempty(runner)&&isfile(runner),'[R18] runner missing.');
root=fileparts(runner); backup=fullfile(root,'run_flight_headless.R18_PREINSTALL_BACKUP.m');
assert(isfile(backup),'[R18] R18 backup missing.');
copyfile(backup,runner,'f'); delete(backup); rehash;
fprintf('[R18] runner restored to the pre-R18 R17 implementation.\n');
end
