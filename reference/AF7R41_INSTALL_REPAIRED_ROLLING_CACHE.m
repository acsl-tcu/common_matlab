function OUT = AF7R41_INSTALL_REPAIRED_ROLLING_CACHE()
clc;
fprintf('\n====================================================================================================\n');
fprintf(' AF7R41 | EXACT REPAIRED ROLLING-CACHE INSTALL\n');
fprintf('====================================================================================================\n');
fprintf('[AUTHORITY] exactR40SourceBound=1 | shadowRequired=1 | noRegexPatching=1 | safetyMathUnchanged=1\n');
assert(evalin('base','exist(''AF7R41A_REPAIRED_ROLLING_CACHE_SHADOW_OUT'',''var'')')==1, ...
    '[AF7R41] run AF7R41A_REPAIRED_ROLLING_CACHE_SHADOW first.');
S=evalin('base','AF7R41A_REPAIRED_ROLLING_CACHE_SHADOW_OUT');
assert(isstruct(S)&&isfield(S,'closed')&&logical(S.closed), ...
    '[AF7R41] shadow equivalence is not closed. No write performed.');
src=which('REPLANNING_SYSTEM'); assert(~isempty(src)&&isfile(src));
here=fileparts(mfilename('fullpath'));
payload=fullfile(here,'REPLANNING_SYSTEM_AF7R41_EXACT_SOURCE.txt'); assert(isfile(payload));
expectedOld='721fdc7cb5b771299215ebb69ec76fdbe6c1ec65554eaf1576c788e303491c71'; expectedNew='ffce61f125c0d2a8f1f216396344817e32ddfd0d24794842adeee47742d7f762';
h0=sha256_file(src); hp=sha256_file(payload); assert(strcmp(hp,expectedNew));
if strcmp(h0,expectedNew)
    fprintf(' exact AF7R41 source already installed    = 1\n');
    OUT=struct('installed',true,'already_installed',true,'source',src,'hash',h0);
    assignin('base','AF7R41_INSTALL_OUT',OUT); fprintf('====================================================================================================\n'); return;
end
assert(strcmp(h0,expectedOld), ...
    '[AF7R41] current source is not the exact AF7R40 source used by the closed shadow. No write performed.');
stamp=datestr(now,'yyyymmdd_HHMMSS');
bak=fullfile(fileparts(src),['REPLANNING_SYSTEM_pre_AF7R41_' stamp '.m']);
[ok,msg]=copyfile(src,bak,'f'); assert(ok,'%s',msg);
[ok,msg]=copyfile(payload,src,'f'); assert(ok,'%s',msg);
assert(strcmp(sha256_file(src),expectedNew)); rehash;
fprintf(' source                                  = %s\n',src);
fprintf(' backup                                  = %s\n',bak);
fprintf(' AF7R40 source SHA-256                   = %s\n',expectedOld);
fprintf(' AF7R41 source SHA-256                   = %s\n',expectedNew);
fprintf(' steady repaired horizon                 = exact shift + one new tail point\n');
fprintf(' repaired full/shift path pre-Flight     = 1\n');
fprintf(' safety/deadline mathematics modified    = 0\n');
fprintf('====================================================================================================\n');
OUT=struct('installed',true,'already_installed',false,'source',src,'backup',bak,'old_hash',expectedOld,'new_hash',expectedNew);
assignin('base','AF7R41_INSTALL_OUT',OUT);
end
function h=sha256_file(fp)
fid=fopen(fp,'rb'); assert(fid>=0); c=onCleanup(@()fclose(fid)); b=fread(fid,Inf,'*uint8'); clear c;
md=java.security.MessageDigest.getInstance('SHA-256'); md.update(b); d=typecast(md.digest(),'uint8');
h=lower(reshape(dec2hex(d,2).',1,[]));
end
