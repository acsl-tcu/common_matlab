function OUT = AF7R43_INSTALL_HARDPATH_CLEANUP()
clc;
fprintf('\n====================================================================================================\n');
fprintf(' AF7R43 | HARD-PATH DIAGNOSTIC DEFERRAL + NOMINAL-MODEL CACHE INSTALL\n');
fprintf('====================================================================================================\n');
fprintf('[AUTHORITY] exactSourceBound=1 | safetyMathUnchanged=1 | diagnosticWorkDeferred=1\n');

src=which('REPLANNING_SYSTEM');
assert(~isempty(src)&&isfile(src),'[AF7R43] REPLANNING_SYSTEM.m unresolved.');
here=fileparts(mfilename('fullpath'));
payload=fullfile(here,'REPLANNING_SYSTEM_AF7R43_EXACT_SOURCE.txt');
assert(isfile(payload));

h0=sha256_file(src);
expectedR41='ffce61f125c0d2a8f1f216396344817e32ddfd0d24794842adeee47742d7f762';
expectedR42='794bd628763d620a0d0cb4be0fb497aed9c0b4d9e1fb05e629c7e83a7e456aef';
expectedNew='52f2e46a73c5e35aec5e436a4484dd7663564e867374a017869e604ae04842c1';
assert(strcmp(sha256_file(payload),expectedNew),'[AF7R43] payload hash mismatch.');

if strcmp(h0,expectedNew)
    fprintf(' exact AF7R43 source already installed    = 1\n');
    OUT=struct('installed',true,'already_installed',true,'source',src,'hash',h0);
    assignin('base','AF7R43_INSTALL_OUT',OUT);
    fprintf('====================================================================================================\n');
    return;
end

assert(strcmp(h0,expectedR41)||strcmp(h0,expectedR42), ...
    ['[AF7R43] current source is neither exact AF7R41 nor exact AF7R42 diagnostic source. ' ...
     'No write performed.']);

stamp=datestr(now,'yyyymmdd_HHMMSS');
bak=fullfile(fileparts(src),['REPLANNING_SYSTEM_pre_AF7R43_' stamp '.m']);
[ok,msg]=copyfile(src,bak,'f'); assert(ok,'%s',msg);
[ok,msg]=copyfile(payload,src,'f'); assert(ok,'%s',msg);
assert(strcmp(sha256_file(src),expectedNew),'[AF7R43] written hash mismatch.');
rehash;

fprintf(' source                                  = %s\n',src);
fprintf(' backup                                  = %s\n',bak);
fprintf(' input hash                              = %s\n',h0);
fprintf(' AF7R43 hash                             = %s\n',expectedNew);
fprintf(' AF7R42 request instrumentation removed  = %d\n',strcmp(h0,expectedR42));
fprintf(' trigger replay capture deferred         = 1\n');
fprintf(' proposal diagnostic record deferred     = 1\n');
fprintf(' nominal AF7R33 batch model cached       = 1\n');
fprintf(' safety/deadline mathematics modified    = 0\n');
fprintf('====================================================================================================\n');

OUT=struct('schema','AF7R43_INSTALL_V1','installed',true,'already_installed',false, ...
    'source',src,'backup',bak,'input_hash',h0,'new_hash',expectedNew);
assignin('base','AF7R43_INSTALL_OUT',OUT);
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
