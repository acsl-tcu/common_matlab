function I=AF7R58R88_INSTALL()
% MATLAB must be restarted after installation. Never patch a live class.
S=AF7R58R88_SOURCE_CHECK();
assert(S.before_install,'R88:AlreadyInstalled_NoWrite');
root=fileparts(mfilename('fullpath'));
target=fullfile(root,'REPLANNING_SYSTEM.m');
patch=fullfile(root,'REPLANNING_SYSTEM_R88_PATCHED_SOURCE.txt');
backup=fullfile(root,['REPLANNING_SYSTEM_pre_AF7R58R88_' ...
    datestr(now,'yyyymmdd_HHMMSSFFF') '.m']);
[ok,msg]=copyfile(target,backup);assert(ok,'R88:BackupFailed:%s',msg);
assert(strcmp(AF7R58R88_SHA256_FILE(backup),S.source_sha256), ...
    'R88:BackupChecksumMismatch');
try
    [ok,msg]=copyfile(patch,target,'f');
    if ~ok,error('R88:CopyFailed','%s',msg);end
    V=AF7R58R88_SOURCE_CHECK();
    assert(V.already_installed,'R88:AfterInstallChecksumMismatch');
catch ME
    [ok,msg]=copyfile(backup,target,'f');
    assert(ok,'R88:RollbackFailed:%s',msg);
    error('R88:RollbackComplete','Restored original: %s',ME.message);
end
I=struct('status','R88_TIME_BOUND_EKF_MASS_CAPTURE_INSTALLED__RESTART_MATLAB', ...
    'backup_file',backup,'original_sha256',S.source_sha256, ...
    'patched_sha256',V.source_sha256, ...
    'production_trigger_changed',false, ...
    'trajectory_generation_changed',false, ...
    'controller_plant_ekf_changed',false, ...
    'flight_diagnostic_capture_added',true, ...
    'production_execution_certified',false);
end
