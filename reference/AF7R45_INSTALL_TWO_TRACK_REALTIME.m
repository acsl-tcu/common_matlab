function OUT = AF7R45_INSTALL_TWO_TRACK_REALTIME()
clc;
fprintf('\n====================================================================================================\n');
fprintf(' AF7R45 | TWO-TRACK REALTIME CLEANUP INSTALL\n');
fprintf('====================================================================================================\n');
fprintf('[AUTHORITY] exactR44SourceBound=1 | preArmingPrimeOnly=1 | safetyMathUnchanged=1\n');

src=which('REPLANNING_SYSTEM');
assert(~isempty(src)&&isfile(src),'[AF7R45] REPLANNING_SYSTEM.m unresolved.');
here=fileparts(mfilename('fullpath'));
payload=fullfile(here,'REPLANNING_SYSTEM_AF7R45_EXACT_SOURCE.txt');
assert(isfile(payload),'[AF7R45] source payload missing.');

expectedOld='67c026dd021e26f7fcd7eefb9c1ffa9aa9023b77dd64525ae674fdaae83011d9';
expectedNew='ded3e68fb1b7da5187985e1f171dc970afeb82d9ba09fecacbc4f07f553936d9';
h0=sha256_file(src);
assert(strcmp(sha256_file(payload),expectedNew),'[AF7R45] payload hash mismatch.');

if strcmp(h0,expectedNew)
    fprintf(' exact AF7R45 source already installed    = 1\n');
    OUT=struct('installed',true,'already_installed',true,'source',src,'hash',h0);
    assignin('base','AF7R45_INSTALL_OUT',OUT);
    fprintf('====================================================================================================\n');
    return;
end

assert(strcmp(h0,expectedOld), ...
    ['[AF7R45] current source is not exact AF7R44. No write was performed.']);

stamp=datestr(now,'yyyymmdd_HHMMSS');
bak=fullfile(fileparts(src),['REPLANNING_SYSTEM_pre_AF7R45_' stamp '.m']);
[ok,msg]=copyfile(src,bak,'f'); assert(ok,'[AF7R45] backup failed: %s',msg);
[ok,msg]=copyfile(payload,src,'f'); assert(ok,'[AF7R45] replacement failed: %s',msg);
assert(strcmp(sha256_file(src),expectedNew),'[AF7R45] written hash mismatch.');
rehash;

fprintf(' source                                  = %s\n',src);
fprintf(' backup                                  = %s\n',bak);
fprintf(' AF7R44 SHA-256                          = %s\n',expectedOld);
fprintf(' AF7R45 SHA-256                          = %s\n',expectedNew);
fprintf(' REFERENCE_ONLY exec-only dep fast path  = 1\n');
fprintf(' exact equal-speed commit packet reuse   = 1\n');
fprintf(' reuse helper prime timing               = PRE-MOTION / BEFORE ARMING\n');
fprintf(' Takeoff extra heavy prime               = 0\n');
fprintf(' safety/deadline mathematics modified    = 0\n');
fprintf('====================================================================================================\n');

OUT=struct('schema','AF7R45_INSTALL_V1','installed',true, ...
    'already_installed',false,'source',src,'backup',bak, ...
    'old_hash',expectedOld,'new_hash',expectedNew);
assignin('base','AF7R45_INSTALL_OUT',OUT);
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
