function I=AF7R58R109_INSTALL()
% Install the verified R109 class only from the exact R108 class bytes.
S=AF7R58R109_SOURCE_CHECK();
I=struct('status','NOT_INSTALLED','modified',false, ...
    'backup_path','','new_engines_armed',false);
if S.already_installed
    I.status='R109_ALREADY_INSTALLED';return;
end
assert(S.installable,'R109:SourceDrift', ...
    'Current main class differs from the installed R108 source. No overwrite.');
T=AF7R58R109_SELFTEST();
assert(startsWith(T.status,'PASS_R109_'),'R109:StaticCheckFailed');
root=fileparts(mfilename('fullpath'));
a=fullfile(root,'REPLANNING_SYSTEM.m');
b=fullfile(root,'REPLANNING_SYSTEM_R109_PATCHED_SOURCE.txt');
backup=fullfile(root,['REPLANNING_SYSTEM_pre_R109_' ...
    datestr(now,'yyyymmdd_HHMMSSFFF') '.m']);
[ok,msg]=copyfile(a,backup);assert(ok,'R109:BackupFailed','%s',msg);
assert(strcmp(AF7R58R109_HASH_FILE(backup),S.r108_sha256), ...
    'R109:BackupMismatch');
try
    [ok,msg]=copyfile(b,a,'f');
    if ~ok,error('R109:CopyFailed','%s',msg);end
    assert(strcmp(AF7R58R109_HASH_FILE(a),S.r109_sha256), ...
        'R109:PatchedHashMismatch');
catch ME
    [ok,msg]=copyfile(backup,a,'f');
    assert(ok&&strcmp(AF7R58R109_HASH_FILE(a),S.r108_sha256), ...
        'R109:RollbackFailed','Original R108 source could not be restored.');
    error('R109:SourceRestored','R109 rolled back: %s',ME.message);
end
I.status='R109_MAIN_STAGE1_EKF_BOUND__RESTART_MATLAB';
I.modified=true;I.backup_path=backup;
end
