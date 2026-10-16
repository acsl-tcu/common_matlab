function OUT = AF7R44_INSTALL_SENSOR_MEMORY()
clc;
fprintf('\n====================================================================================================\n');
fprintf(' AF7R44 | SIMULATED SENSOR MEMORY / UNKNOWN-UNTIL-DETECTED INSTALL\n');
fprintf('====================================================================================================\n');
fprintf('[AUTHORITY] exactR43SourceBound=1 | simulationOnly=1 | noArbitraryHoldTime=1 | safetyMathUnchanged=1\n');

src=which('REPLANNING_SYSTEM');
assert(~isempty(src)&&isfile(src),'[AF7R44] REPLANNING_SYSTEM.m unresolved.');
here=fileparts(mfilename('fullpath'));
payload=fullfile(here,'REPLANNING_SYSTEM_AF7R44_EXACT_SOURCE.txt');
assert(isfile(payload),'[AF7R44] exact source payload missing.');

expectedOld='52f2e46a73c5e35aec5e436a4484dd7663564e867374a017869e604ae04842c1';
expectedNew='67c026dd021e26f7fcd7eefb9c1ffa9aa9023b77dd64525ae674fdaae83011d9';
h0=sha256_file(src);
assert(strcmp(sha256_file(payload),expectedNew),'[AF7R44] payload hash mismatch.');

if strcmp(h0,expectedNew)
    fprintf(' exact AF7R44 source already installed    = 1\n');
    OUT=struct('installed',true,'already_installed',true,'source',src,'hash',h0);
    assignin('base','AF7R44_INSTALL_OUT',OUT);
    fprintf('====================================================================================================\n');
    return;
end

assert(strcmp(h0,expectedOld), ...
    ['[AF7R44] current source is not the exact AF7R43 source audited for this replacement. ' ...
     'No write performed.']);

stamp=datestr(now,'yyyymmdd_HHMMSS');
bak=fullfile(fileparts(src),['REPLANNING_SYSTEM_pre_AF7R44_' stamp '.m']);
[ok,msg]=copyfile(src,bak,'f'); assert(ok,'[AF7R44] backup failed: %s',msg);
[ok,msg]=copyfile(payload,src,'f'); assert(ok,'[AF7R44] exact source replacement failed: %s',msg);
assert(strcmp(sha256_file(src),expectedNew),'[AF7R44] written hash mismatch.');
rehash;

fprintf(' source                                  = %s\n',src);
fprintf(' backup                                  = %s\n',bak);
fprintf(' AF7R43 source SHA-256                   = %s\n',expectedOld);
fprintf(' AF7R44 source SHA-256                   = %s\n',expectedNew);
fprintf(' initial planner obstacle knowledge      = EMPTY\n');
fprintf(' admission                               = current simulated sensor only\n');
fprintf(' retention                               = through certified rejoin\n');
fprintf(' arbitrary time retention                = 0\n');
fprintf(' safety/deadline mathematics modified    = 0\n');
fprintf('====================================================================================================\n');

OUT=struct('schema','AF7R44_INSTALL_SENSOR_MEMORY_V1','installed',true, ...
    'already_installed',false,'source',src,'backup',bak, ...
    'old_hash',expectedOld,'new_hash',expectedNew);
assignin('base','AF7R44_INSTALL_OUT',OUT);
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
