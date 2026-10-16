function OUT = AF7R42_INSTALL_REQUEST_HARDPATH_ATTRIBUTION()
clc;
fprintf('\n====================================================================================================\n');
fprintf(' AF7R42 | EXACT <=4 ms REQUEST / COMMIT ATTRIBUTION INSTALL\n');
fprintf('====================================================================================================\n');
fprintf('[AUTHORITY] exactR41SourceBound=1 | diagnosticOnly=1 | noRegexPatching=1 | safetyMathUnchanged=1\n');

src=which('REPLANNING_SYSTEM');
assert(~isempty(src)&&isfile(src),'[AF7R42] REPLANNING_SYSTEM.m unresolved.');
here=fileparts(mfilename('fullpath'));
payload=fullfile(here,'REPLANNING_SYSTEM_AF7R42_EXACT_SOURCE.txt');
assert(isfile(payload),'[AF7R42] exact source payload missing.');

expectedOld='ffce61f125c0d2a8f1f216396344817e32ddfd0d24794842adeee47742d7f762';
expectedNew='794bd628763d620a0d0cb4be0fb497aed9c0b4d9e1fb05e629c7e83a7e456aef';
h0=sha256_file(src);
hp=sha256_file(payload);
assert(strcmp(hp,expectedNew),'[AF7R42] packaged source hash mismatch.');

if strcmp(h0,expectedNew)
    fprintf(' exact AF7R42 source already installed    = 1\n');
    OUT=struct('installed',true,'already_installed',true,'source',src,'hash',h0);
    assignin('base','AF7R42_INSTALL_OUT',OUT);
    fprintf('====================================================================================================\n');
    return;
end

assert(strcmp(h0,expectedOld), ...
    ['[AF7R42] current source is not the exact AF7R41 source used to build this diagnostic. ' ...
     'No write performed.']);

stamp=datestr(now,'yyyymmdd_HHMMSS');
bak=fullfile(fileparts(src),['REPLANNING_SYSTEM_pre_AF7R42_' stamp '.m']);
[ok,msg]=copyfile(src,bak,'f');
assert(ok,'[AF7R42] backup failed: %s',msg);
[ok,msg]=copyfile(payload,src,'f');
assert(ok,'[AF7R42] exact source replacement failed: %s',msg);
assert(strcmp(sha256_file(src),expectedNew),'[AF7R42] written hash mismatch.');
rehash;

fprintf(' source                                  = %s\n',src);
fprintf(' backup                                  = %s\n',bak);
fprintf(' AF7R41 source SHA-256                   = %s\n',expectedOld);
fprintf(' AF7R42 source SHA-256                   = %s\n',expectedNew);
fprintf(' safety/commit logic modified            = 0\n');
fprintf(' timing checkpoints added only on request= 1\n');
fprintf('====================================================================================================\n');

OUT=struct('schema','AF7R42_INSTALL_V1','installed',true, ...
    'already_installed',false,'source',src,'backup',bak, ...
    'old_hash',expectedOld,'new_hash',expectedNew);
assignin('base','AF7R42_INSTALL_OUT',OUT);
end

function h=sha256_file(fp)
fid=fopen(fp,'rb'); assert(fid>=0);
c=onCleanup(@()fclose(fid));
b=fread(fid,Inf,'*uint8'); clear c;
md=java.security.MessageDigest.getInstance('SHA-256');
md.update(b);
d=typecast(md.digest(),'uint8');
h=lower(reshape(dec2hex(d,2).',1,[]));
end
