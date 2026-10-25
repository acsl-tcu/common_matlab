function I=AF7R58R111_INSTALL()
% Back up the exact user source and replace ONLY a known R110 variant.
% Install with Flight stopped, then restart MATLAB for class reload.
S=AF7R58R111_SOURCE_CHECK();
I=struct('status','R111_NOT_INSTALLED','modified',false,...
    'backup_path','','source_variant',S.source_variant,...
    'user_flag_preserved',false);
if S.already_installed
    I.status='R111_ALREADY_INSTALLED';I.user_flag_preserved=true;return;
end
assert(S.installable,'R111:SourceDrift',...
    'Unexpected user source: no overwrite. Preserve your current REPLANNING_SYSTEM.m.');
T=AF7R58R111_SELFTEST();
assert(startsWith(T.status,'PASS_R111_'),'R111:SelftestFailed');
root=fileparts(mfilename('fullpath'));
current=fullfile(root,'REPLANNING_SYSTEM.m');
backup=fullfile(root,['REPLANNING_SYSTEM_pre_R111_' ...
    datestr(now,'yyyymmdd_HHMMSSFFF') '.m']);
[ok,msg]=copyfile(current,backup);
assert(ok,'R111:BackupFailed','%s',msg);
assert(strcmp(AF7R58R111_HASH_FILE(backup),S.active_sha256),...
    'R111:BackupHash','Backup differs from active source; aborted.');
try
    [ok,msg]=copyfile(S.patch_path,current,'f');
    if ~ok,error('R111:CopyFailed','%s',msg);end
    assert(strcmp(AF7R58R111_HASH_FILE(current),S.expected_patch_sha256),...
        'R111:PatchHash','Patched file hash mismatch.');
catch ME
    [ok,msg]=copyfile(backup,current,'f');
    assert(ok && strcmp(AF7R58R111_HASH_FILE(current),S.active_sha256),...
        'R111:RollbackFailed','Original production class could not be restored.');
    error('R111:RolledBack','The source was restored: %s',ME.message);
end
I.status='R111_TIME_ONLY_CUTOFF_REMOVED__RESTART_MATLAB';
I.modified=true;I.backup_path=backup;I.user_flag_preserved=true;
end
