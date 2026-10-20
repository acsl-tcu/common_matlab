function AF7R58ZZY_R22_ROLLBACK
runner=which('run_flight_headless');
assert(~isempty(runner)&&isfile(runner),'[R22] runner not found.');
root=fileparts(runner);
backup=fullfile(root,'run_flight_headless.R22_PREINSTALL_BACKUP.m');
assert(isfile(backup),'[R22] backup not found.');
copyfile(backup,runner,'f');
delete(backup);
q=fullfile(root,'REPLANNING_SYSTEM_R22_ACCOUNTING_DIAG.m'); if isfile(q), delete(q); end
q=fullfile(root,'AF7R58ZZY_R22_REPORT.m'); if isfile(q), delete(q); end
q=fullfile(root,'AF7R58ZZY_R22_INSTALL_MANIFEST.txt'); if isfile(q), delete(q); end
clear REPLANNING_SYSTEM_R22_ACCOUNTING_DIAG AF7R58ZZY_R22_REPORT;
rehash;
fprintf('[R22] Rolled back runner to exact pre-R22 R21 state.\n');
end
