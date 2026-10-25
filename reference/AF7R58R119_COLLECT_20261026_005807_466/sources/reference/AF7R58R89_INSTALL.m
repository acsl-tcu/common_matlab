function I=AF7R58R89_INSTALL()
% Direct replace reference-only R81 generator, NEVER Flight class or trigger.
root=fileparts(mfilename('fullpath'));
target=fullfile(root,'AF7R58R81_ADAPTIVE_DELAYED_RETURN.m');
patch=fullfile(root,'AF7R58R81_ADAPTIVE_DELAYED_RETURN_R89_SOURCE.txt');
base='4c5b12210992ac3b8e31f343edaba837a22f6145138e6b6c4a7fc640f83d8c4a';
after='bd922558261e8369c9eaae94ff7281c38837afe530559cbdf0f3bec194f7c1e9';
assert(isfile(target)&&isfile(patch), 'R89:MissingSource');
assert(strcmp(AF7R58R89_SHA256_FILE(patch),after), 'R89:PatchChecksum');
assert(isfile(fullfile(root,'AF7R58R89_LOCAL_KINEMATIC_CERT.m')), ...
    'R89:CertifierNotInstalled');
h=AF7R58R89_SHA256_FILE(target);
if strcmp(h,after)
    I=struct('status','R89_ALREADY_INSTALLED__NO_WRITE', ...
        'reference_only',true,'production_modified',false);
    return;
end
assert(strcmp(h,base),'R89:UnexpectedLiveR81SourceRefuseOverwrite');
T=AF7R58R89_SELFTEST();
assert(contains(T.status,'PASS_R89'),'R89:SelftestFailure');
backup=fullfile(root,['AF7R58R81_pre_R89_' datestr(now,'yyyymmdd_HHMMSSFFF') '.m']);
[ok,msg]=copyfile(target,backup);assert(ok,'R89:BackupFailed:%s',msg);
assert(strcmp(AF7R58R89_SHA256_FILE(backup),base),'R89:BackupChecksum');
try
    [ok,msg]=copyfile(patch,target,'f');
    assert(ok,'R89:CopyFailed:%s',msg);
    assert(strcmp(AF7R58R89_SHA256_FILE(target),after),'R89:PostChecksum');
catch ME
    [ok,msg]=copyfile(backup,target,'f');
    assert(ok&&strcmp(AF7R58R89_SHA256_FILE(target),base), ...
        'R89:RollbackFailed');
    rethrow(ME);
end
clear AF7R58R81_ADAPTIVE_DELAYED_RETURN
rehash;
I=struct('status','R89_LOCAL_CERT_DIRECT_R81_REPLACED', ...
    'original_sha256',base,'patched_sha256',after, ...
    'backup_file',backup,'reference_only',true, ...
    'production_flight_class_changed',false,'trigger_switched',false, ...
    'input_tension_and_closed_loop_proved',false);
end
