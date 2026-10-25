function I=AF7R58R93_INSTALL()
% Exact source-bound replacement of R92 RUNNER ONLY. No Flight class edits.
base=fileparts(mfilename('fullpath'));
dst=fullfile(base,'AF7R58R92_RUN_SAVED_ROUTE.m');
patch=fullfile(base,'AF7R58R92_RUN_SAVED_ROUTE_R93_SOURCE.txt');
assert(isfile(dst)&&isfile(patch),'R93:RequiredSourceMissing');
assert(strcmp(fileparts(which('AF7R58R92_RUN_SAVED_ROUTE')),base), ...
 'R93:ShadowedRunnerRefuseToInstall');
original='bc0fa1772a3830ba9cd0ddd43646361fa71de973adeae82e354600f4c08f2e18';
new='5aa99988805093fff9e82111c08321062e6f85a5919c634af6e8419863746e8c';
assert(strcmpi(AF7R58R93_SHA256_FILE(patch),new),'R93:PatchSourceCorrupt');
old=AF7R58R93_SHA256_FILE(dst);
if strcmpi(old,new)
 T=AF7R58R93_SELFTEST();
 I=struct('status','R93_ALREADY_INSTALLED_AND_TESTED', ...
     'source_sha256',old,'selftest',T, ...
     'production_class_changed',false,'trigger_switched',false);
 return;
end
assert(strcmpi(old,original),'R93:InstalledR92NotTheCapturedExactVersion');
stamp=char(datetime('now','Format','yyyyMMdd_HHmmssSSS'));
bak=fullfile(base,['AF7R58R92_RUN_SAVED_ROUTE_pre_R93_' stamp '.m']);
[ok,msg]=copyfile(dst,bak);
assert(ok,'R93:BackupFailed:%s',msg);
try
    [ok,msg]=copyfile(patch,dst,'f');
    assert(ok,'R93:SourceReplacementFailed:%s',msg);
    clear('AF7R58R92_RUN_SAVED_ROUTE');
    rehash;
    assert(strcmpi(AF7R58R93_SHA256_FILE(dst),new),'R93:PostCopyHashMismatch');
    T=AF7R58R93_SELFTEST();
catch ME
    copyfile(bak,dst,'f');
    clear('AF7R58R92_RUN_SAVED_ROUTE');
    rehash;
    rethrow(ME);
end
I=struct('status','R93_R92_RUNNER_REPLACED_WITH_PER_SEED_TRACE', ...
    'backup_file',bak,'original_sha256',old, ...
    'patched_sha256',new,'selftest',T, ...
    'flight_class_changed',false,'flight_trigger_switched',false, ...
    'initial_production_generator_replaced',false, ...
    'physical_closed_loop_certified',false);
end
