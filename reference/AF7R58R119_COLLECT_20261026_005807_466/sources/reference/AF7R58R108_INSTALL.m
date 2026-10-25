function I=AF7R58R108_INSTALL()
% Explicit atomic-ish in-place source upgrade. Run after stopping Flight.
S=AF7R58R108_SOURCE_CHECK();
I=struct('status','NOT_INSTALLED','source_modified',false, ...
    'backup_path','','dual_engines_enabled',false, ...
    'new_generator_active',false,'physical_future_active',false);
if S.already_installed
    I.status='R108_ALREADY_INSTALLED';return;
end
assert(S.installable,'R108:SourceDrift', ...
    'The current REPLANNING_SYSTEM.m does not match the R106 pinned source.');
T=AF7R58R108_SELFTEST();
assert(startsWith(T.status,'PASS_R108_'),'R108:StaticRegressionFailed');
root=fileparts(mfilename('fullpath'));
target=fullfile(root,'REPLANNING_SYSTEM.m');
source=fullfile(root,'REPLANNING_SYSTEM_R108_PATCHED_SOURCE.txt');
backup=fullfile(root,['REPLANNING_SYSTEM_pre_R108_' ...
    datestr(now,'yyyymmdd_HHMMSSFFF') '.m']);
[ok,msg]=copyfile(target,backup);
assert(ok,'R108:BackupFailed','%s',msg);
assert(strcmp(AF7R58R108_HASH_FILE(backup),S.expected_R106_sha256), ...
    'R108:BackupInvalid','Baseline backup hash mismatch.');
try
    [ok,msg]=copyfile(source,target,'f');
    if ~ok,error('R108:CopyFailed','%s',msg);end
    assert(strcmp(AF7R58R108_HASH_FILE(target),S.patched_sha256), ...
        'R108:NewHashMismatch','Patched main class differs from audited source.');
catch ME
    [ok,msg]=copyfile(backup,target,'f');
    assert(ok,'R108:RollbackFailed','%s',msg);
    assert(strcmp(AF7R58R108_HASH_FILE(target),S.expected_R106_sha256), ...
        'R108:RollbackIntegrity','Cannot verify baseline restored.');
    error('R108:SourceRestored','Installation rolled back: %s',ME.message);
end
I.status='R108_PRODUCTION_MAIN_DISPATCH_INSTALLED__RESTART_MATLAB';
I.source_modified=true;I.backup_path=backup;
end
