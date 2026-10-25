function I=AF7R58R106_INSTALL()
% Explicit fail-closed audited source install after ending Flight.
% Make a backup, atomically validate hashes, and roll back on failure.
S=AF7R58R106_SOURCE_CHECK();
assert(startsWith(S.status,'PASS_R106_'), ...
    'R106:SourceCheckFailed','%s',S.status);
I=struct('status','NOT_INSTALLED','source_modified',false, ...
    'backup_path','','old_sha256',S.class_current_sha256, ...
    'new_sha256',S.patched_sha256, ...
    'trigger_switched',false,'generator_switched',false, ...
    'closed_loop_certified',false);
if S.already_installed
    I.status='R106_ALREADY_INSTALLED__NO_WRITE';return;
end
assert(S.installable,'R106:InstallationDenied', ...
    'Only exact pinned R88R2 source is accepted.');
T=AF7R58R106_SELFTEST();
assert(startsWith(T.status,'PASS_R106_'),'R106:SelftestMustPass', ...
    'Event tests must pass before the production class changes.');
root=fileparts(mfilename('fullpath'));
target=fullfile(root,'REPLANNING_SYSTEM.m');
source=fullfile(root,'REPLANNING_SYSTEM_R106_PATCHED_SOURCE.txt');
backup=fullfile(root,['REPLANNING_SYSTEM_pre_R106_' ...
    datestr(now,'yyyymmdd_HHMMSSFFF') '.m']);
[ok,msg]=copyfile(target,backup);
assert(ok,'R106:BackupFailed','%s',msg);
assert(strcmp(AF7R58R106_SHA256_FILE(backup),S.baseline_sha256), ...
    'R106:BackupInvalid','Backup SHA mismatch; no writes to production.');
try
    [ok,msg]=copyfile(source,target,'f');
    if ~ok,error('R106:WriteFailed','%s',msg);end
    actual=AF7R58R106_SHA256_FILE(target);
    assert(strcmp(actual,S.patched_sha256),'R106:NewHashMismatch', ...
        'Production class checksum differs from pinned source.');
catch ME
    [rolled,msg]=copyfile(backup,target,'f');
    assert(rolled,'R106:RollbackCopyFailed','%s',msg);
    assert(strcmp(AF7R58R106_SHA256_FILE(target),S.baseline_sha256), ...
        'R106:RollbackChecksum','Cannot confirm restored baseline.');
    error('R106:RolledBack','Original restored: %s',ME.message);
end
I.status='R106_SAME_TICK_EKF_CAPTURE_INSTALLED__RESTART_MATLAB';
I.source_modified=true;
I.backup_path=backup;
end
