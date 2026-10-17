function OUT = AF7R49_INSTALL_PRODUCTION_CLOSEOUT()
clc;
fprintf('\n====================================================================================================\n');
fprintf(' AF7R49 | PRODUCTION CLOSEOUT INSTALL\n');
fprintf('====================================================================================================\n');
fprintf('[AUTHORITY] shadowRequired=1 | exactR46SourceBound=1 | safetyAcceptanceUnchanged=1\n');

src=which('REPLANNING_SYSTEM'); assert(~isempty(src)&&isfile(src));
root=fileparts(src); here=fileparts(mfilename('fullpath'));
old='890876116b4782725d8c337e53923a14892a9b5f4f00a0e7de030b9d4433c446'; new='a18b8cd4953f5e67cc5ddcf9efe1b11c284cce674a941f2dbbba0f68c40a04be';
h0=sha256_file(src);
if strcmp(h0,new)
    fprintf(' exact AF7R49 already installed          = 1\n');
    OUT=struct('installed',true,'already_installed',true,'source',src,'hash',h0);
    assignin('base','AF7R49_INSTALL_OUT',OUT); return;
end
assert(strcmp(h0,old),'[AF7R49] source is not exact AF7R46; no write performed.');

tokPath=fullfile(root,'AF7R49_SHADOW_TOKEN.mat');
assert(isfile(tokPath),'[AF7R49] shadow token missing; run AF7R49A first.');
Q=load(tokPath,'token'); token=Q.token;
assert(isstruct(token)&&isfield(token,'closed')&&logical(token.closed), ...
    '[AF7R49] shadow did not close; production install forbidden.');
assert(strcmp(char(token.source_hash),old),'[AF7R49] shadow token source mismatch.');
mexPath=which('af7r49_single_core_mex'); assert(~isempty(mexPath)&&isfile(mexPath));
assert(strcmp(sha256_file(mexPath),char(token.mex_hash)), ...
    '[AF7R49] shadow-tested MEX changed after audit.');

payload=fullfile(here,'REPLANNING_SYSTEM_AF7R49_EXACT_SOURCE.txt'); assert(isfile(payload));
assert(strcmp(sha256_file(payload),new),'[AF7R49] payload hash mismatch.');
% Ensure runtime helper files are in the production class folder.
for f={'af7r49_prime_core_reference_runtime.m','af7r49_replay_context.m','af7r49_core.hpp','af7r49_single_core_mex.cpp'}
    a=fullfile(here,f{1}); b=fullfile(root,f{1});
    if ~strcmpi(a,b), [ok,msg]=copyfile(a,b,'f'); assert(ok,'%s',msg); end
end
stamp=datestr(now,'yyyymmdd_HHMMSS');
bak=fullfile(root,['REPLANNING_SYSTEM_pre_AF7R49_' stamp '.m']);
[ok,msg]=copyfile(src,bak,'f'); assert(ok,'%s',msg);
[ok,msg]=copyfile(payload,src,'f'); assert(ok,'%s',msg);
assert(strcmp(sha256_file(src),new),'[AF7R49] written source hash mismatch.');
rehash;

fprintf(' source                                  = %s\n',src);
fprintf(' backup                                  = %s\n',bak);
fprintf(' synthesis primary objective             = integral ||correction||^2 dt\n');
fprintf(' minimum-snap recoverability             = retained, separately identified\n');
fprintf(' continuous safety certificate           = unchanged\n');
fprintf(' light runtime audit in production       = OFF\n');
fprintf(' real trigger forensic capture           = OFF\n');
fprintf(' proposal diagnostic mirror              = OFF\n');
fprintf(' observer equations/support              = UNCHANGED\n');
fprintf(' observer runtime-stat bookkeeping        = OFF\n');
fprintf('====================================================================================================\n');
OUT=struct('schema','AF7R49_INSTALL_PRODUCTION_V1','installed',true, ...
    'already_installed',false,'source',src,'backup',bak,'old_hash',old,'new_hash',new);
assignin('base','AF7R49_INSTALL_OUT',OUT);
end
function h=sha256_file(fp)
fid=fopen(fp,'rb');assert(fid>=0);c=onCleanup(@()fclose(fid));b=fread(fid,Inf,'*uint8');clear c;
md=java.security.MessageDigest.getInstance('SHA-256');md.update(b);d=typecast(md.digest(),'uint8');h=lower(reshape(dec2hex(d,2).',1,[]));
end
