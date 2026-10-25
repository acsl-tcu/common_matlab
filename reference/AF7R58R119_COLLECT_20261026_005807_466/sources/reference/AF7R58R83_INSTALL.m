function I=AF7R58R83_INSTALL()
% Install ONE source-bound read-only R81-live-authority entry point.
% Requires captured R82 EXACT production source. Back up before any write.
% Does not edit triggers, Stage1/2/3 decisions, HLC, plant, EKF, C++ or R81.
root=fileparts(mfilename('fullpath'));
target=fullfile(root,'REPLANNING_SYSTEM.m');
patch=fullfile(root,'REPLANNING_SYSTEM_R83_PATCHED_SOURCE.txt');
assert(isfile(target)&&isfile(patch),'R83:LiveOrPatchSourceMissing');
oldHash='5ad7a1d242fbbc418b6b6743c41ededd6ec4192138c3d317661f3d6780781b33';
newHash='57e60163d054c3c102573496cb9457c5ece155c4f59a6c47acd1dc6c9a4bbff5';
assert(strcmp(AF7R58R83_SHA256_FILE(target),oldHash), ...
    'R83:ProductionSourceChanged_FailWithoutModifying');
assert(strcmp(AF7R58R83_SHA256_FILE(patch),newHash), ...
    'R83:PackageIntegrityFailed');
W=fileread(target); Z=fileread(patch);
needle='        function R = r71_report_live_flight_frame(obj,tglobal)';
assert(numel(strfind(W,needle))==1 && numel(strfind(Z,needle))==1, ...
    'R83:OriginalMethodAnchorMismatch');
assert(isempty(strfind(W,'r83_live_r81_natural_return_reference_only')) && ...
    numel(strfind(Z,'function R = r83_live_r81_natural_return_reference_only'))==1, ...
    'R83:UnexpectedReadOnlyMethodCount');
% Strip exactly the newly introduced method and establish BYTE-EXACT
% equivalence with the captured, verified preinstallation source.
a='        function R = r83_live_r81_natural_return_reference_only(obj,tglobal,opts)';
ka=strfind(Z,a); kb=strfind(Z,needle);
assert(numel(ka)==1 && numel(kb)==1 && kb>ka, ...
    'R83:SourceRegionNotUnique');
restored=[Z(1:ka-1) Z(kb:end)];
assert(isequal(restored,W), ...
    'R83:NonAdditiveModificationDetected');
T83=AF7R58R83_SELFTEST();
assert(strcmp(T83.status,'PASS_R83_LIVE_FRAME_KNOWN_MASK_AND_VERSION_BINDING'), ...
    'R83:StandaloneRegressionFailed');
% Preserve original as a dated file IN THE SAME SCRIPT DIRECTORY.
backup=fullfile(root, ...
    ['REPLANNING_SYSTEM_pre_AF7R58R83_' datestr(now,'yyyymmdd_HHMMSSFFF') '.m']);
[ok,msg]=copyfile(target,backup);
assert(ok,'R83:OriginalBackupFailed:%s',msg);
try
    [ok,msg]=copyfile(patch,target,'f');
    if ~ok,error('R83:CopyFailed','%s',msg);end
    if ~strcmp(AF7R58R83_SHA256_FILE(target),newHash)
        error('R83:PostWriteHashMismatch','Patched source differs from package.');
    end
catch ME
    copyfile(backup,target,'f');
    error('R83:InstallRolledBack', ...
        'Patch not installed, original restored. Root cause: %s',ME.message);
end
I=struct('status','R83_ADDITIVE_LIVE_R81_ENTRY_INSTALLED__RESTART_MATLAB', ...
    'original_sha256',oldHash,'patched_sha256',newHash, ...
    'backup_file',backup,'reference_only',true, ...
    'trigger_changed',false,'stage_commit_changed',false, ...
    'hlc_plant_ekf_changed',false,'production_execution_certified',false);
% Do not clear classes automatically while SimHL/agent may hold live handles.
% Before invoking the new method, restart MATLAB, or clear agent/classes
% SAFELY with the Flight system stopped.
end
