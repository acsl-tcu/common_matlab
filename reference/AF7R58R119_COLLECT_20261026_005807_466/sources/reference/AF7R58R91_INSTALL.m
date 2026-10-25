function I=AF7R58R91_INSTALL()
% R91: Replace ONLY the R89 reference-only kinematic helper.
% No HLC/plant/EKF/Flight/trigger/native/candidate generation modifications.
root=fileparts(mfilename('fullpath'));
target=fullfile(root,'AF7R58R89_LOCAL_KINEMATIC_CERT.m');
patch=fullfile(root,'AF7R58R89_LOCAL_KINEMATIC_CERT_R91_SOURCE.txt');
base='ba04b98bba93da8c6515dc935a733e22dc1be4bb64ed7125f396dea9f44e98ed';
after='d558812b8dc95bb420ddbc6db52af7063352b352e60cfd6ffd6fac671c30d657';
assert(isfile(target)&&isfile(patch),'R91:MissingSource');
assert(strcmp(AF7R58R91_SHA256_FILE(patch),after),'R91:PackageIntegrity');
resolved=which('AF7R58R89_LOCAL_KINEMATIC_CERT');
assert(strcmpi(resolved,target),'R91:ActiveHelperShadowed');
h=AF7R58R91_SHA256_FILE(target);
if strcmp(h,after)
 I=struct('status','R91_ALREADY_INSTALLED__NO_WRITE', ...
  'patch_sha256',after,'production_changed',false);return;
end
assert(strcmp(h,base),'R91:UnexpectedCurrentHelperRefuseOverwrite');
% Validate the exact prospective algorithm before any write.
S=AF7R58R91_SELFTEST();
assert(strcmp(S.status,'PASS_R91_STABLE_NORMALIZED_DERIVATIVE_CP'), ...
    'R91:PreflightFailure');
backup=fullfile(root,['AF7R58R89_LOCAL_KINEMATIC_CERT_pre_R91_' ...
    datestr(now,'yyyymmdd_HHMMSSFFF') '.m']);
[ok,msg]=copyfile(target,backup);
assert(ok,'R91:BackupFailed:%s',msg);
assert(strcmp(AF7R58R91_SHA256_FILE(backup),base),'R91:BackupHash');
try
 [ok,msg]=copyfile(patch,target,'f');
 assert(ok,'R91:FileWriteFailed:%s',msg);
 assert(strcmp(AF7R58R91_SHA256_FILE(target),after), ...
    'R91:PatchedFileChecksumMismatch');
 clear AF7R58R89_LOCAL_KINEMATIC_CERT
 rehash;
 % Run three live-entrypoint tests AFTER the overwrite, rollback on error.
 T=1;g=9.81;L=2;lim=3.10559006211;
 P=zeros(3,16);P(2,:)=linspace(0,T,16);
 C=struct('payload_cp',P,'t0',19.75,'t1',19.75+T,'duration_s',T);
 K=AF7R58R89_LOCAL_KINEMATIC_CERT(C,g,L,lim,512);
 assert(K.certified && K.ieee754_outward_closed==false, ...
  'R91:LiveEntrypointTestFail');
 P(2,:)=linspace(0,5,16); C.payload_cp=P;
 K2=AF7R58R89_LOCAL_KINEMATIC_CERT(C,g,L,lim,512);
 assert(~K2.certified,'R91:LiveEntrypointUnsafeAcceptance');
 P=zeros(3,16);P(1,7:10)=[0 6 6 0];C.payload_cp=P;
 K3=AF7R58R89_LOCAL_KINEMATIC_CERT(C,g,L,lim,1);
 assert(~K3.certified,'R91:LiveEntrypointBudgetBypass');
catch ME
 [ok,msg]=copyfile(backup,target,'f');
 assert(ok&&strcmp(AF7R58R91_SHA256_FILE(target),base), ...
   'R91:RollbackFailed:%s',msg);
 clear AF7R58R89_LOCAL_KINEMATIC_CERT
 rehash;
 rethrow(ME);
end
I=struct('status','R91_DERIVATIVE_CP_REFERENCE_CERTIFIER_REPLACED', ...
 'original_sha256',base,'patched_sha256',after,'backup_file',backup, ...
 'matlab_patch_selftest',S.status,'production_changed',false, ...
 'flight_trigger_switched',false,'initial_generator_changed',false, ...
 'torque_tracking_and_IEEE_safety_proved',false);
end
