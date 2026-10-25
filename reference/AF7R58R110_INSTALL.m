function I=AF7R58R110_INSTALL()
% Install only over the exact R109 production class; reversible.
S=AF7R58R110_SOURCE_CHECK();
I=struct('status','NOT_INSTALLED','modified',false, ...
    'backup_path','','flight_code_changed',false);
if S.already_installed
    I.status='R110_ALREADY_INSTALLED';return;
end
assert(S.installable,'R110:SourceDrift', ...
    'Active class differs from R109; no overwrite performed.');
AF7R58R110_SELFTEST();
root=fileparts(mfilename('fullpath'));
a=fullfile(root,'REPLANNING_SYSTEM.m');
b=fullfile(root,'REPLANNING_SYSTEM_R110_PATCHED_SOURCE.txt');
backup=fullfile(root,['REPLANNING_SYSTEM_pre_R110_' ...
    datestr(now,'yyyymmdd_HHMMSSFFF') '.m']);
[ok,msg]=copyfile(a,backup);
assert(ok,'R110:BackupFailed','%s',msg);
assert(strcmp(AF7R58R110_HASH_FILE(backup),S.r109_sha256), ...
    'R110:BackupMismatch');
try
    [ok,msg]=copyfile(b,a,'f');
    if ~ok,error('R110:CopyFailed','%s',msg);end
    assert(strcmp(AF7R58R110_HASH_FILE(a),S.r110_sha256), ...
        'R110:PatchedHashMismatch');
catch ME
    [ok,msg]=copyfile(backup,a,'f');
    assert(ok&&strcmp(AF7R58R110_HASH_FILE(a),S.r109_sha256), ...
        'R110:RollbackFailed', ...
        'Original R109 class could not be restored.');
    error('R110:SourceRestored','R110 rolled back: %s',ME.message);
end
I.status='R110_POSTFLIGHT_PUBLISH_FIXED__RESTART_MATLAB';
I.modified=true;
I.backup_path=backup;
end
