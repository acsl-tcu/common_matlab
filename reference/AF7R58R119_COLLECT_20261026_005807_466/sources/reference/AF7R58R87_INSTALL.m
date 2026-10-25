function I=AF7R58R87_INSTALL()
% One-time offline R86->R87 string-scalar key bug fix.
% Stop SimHL and restart MATLAB after install. No unsafe live class reload.
root=fileparts(mfilename('fullpath'));
S=AF7R58R87_SOURCE_CHECK();
assert(S.before_install,'R87:AlreadyInstalledOrChanged_NoWrite');
target=fullfile(root,'REPLANNING_SYSTEM.m');
patch=fullfile(root,'REPLANNING_SYSTEM_R87_PATCHED_SOURCE.txt');
backup=fullfile(root,['REPLANNING_SYSTEM_pre_AF7R58R87_' ...
    datestr(now,'yyyymmdd_HHMMSSFFF') '.m']);
[ok,msg]=copyfile(target,backup);
assert(ok,'R87:BackupFailed:%s',msg);
assert(strcmp(AF7R58R87_SHA256_FILE(backup),S.source_sha256), ...
    'R87:BackupDoesNotMatchLiveSource');
try
    [ok,msg]=copyfile(patch,target,'f');
    if ~ok,error('R87:CopyError','%s',msg);end
    V=AF7R58R87_SOURCE_CHECK();
    assert(V.already_installed,'R87:AfterCopyHashMismatch');
catch ME
    [restored,reason]=copyfile(backup,target,'f');
    assert(restored,'R87:RollbackFailed:%s',reason);
    error('R87:InstallRolledBack','Original restored: %s',ME.message);
end
I=struct('status','R87_PARAMETER_STRING_KEY_HOTFIX_INSTALLED__RESTART_MATLAB', ...
    'backup_file',backup, ...
    'original_sha256',S.source_sha256, ...
    'patched_sha256',V.source_sha256, ...
    'cableL_callsite_fix_count',S.cableL_calls_repaired, ...
    'gravity_callsite_fix_count',S.gravity_calls_repaired, ...
    'flight_loop_changed',false,'controller_plant_ekf_changed',false, ...
    'trajectory_generation_changed',false, ...
    'production_execution_certified',false);
end
