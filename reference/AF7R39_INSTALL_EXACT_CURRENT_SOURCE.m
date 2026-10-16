function OUT = AF7R39_INSTALL_EXACT_CURRENT_SOURCE()
clc;
fprintf('\n====================================================================================================\n');
fprintf(' AF7R39 | EXACT CURRENT-SOURCE R38 PROPERTY REPAIR\n');
fprintf('====================================================================================================\n');
fprintf('[AUTHORITY] exactUploadedSourceBound=1 | noAnchorPatching=1 | safetyLogicUnchanged=1\n');

src=which('REPLANNING_SYSTEM');
assert(~isempty(src)&&isfile(src), ...
    '[AF7R39] REPLANNING_SYSTEM.m is not resolvable on the current MATLAB path.');

here=fileparts(mfilename('fullpath'));
payload=fullfile(here,'REPLANNING_SYSTEM_AF7R39_EXACT_SOURCE.txt');
assert(isfile(payload),'[AF7R39] exact replacement payload is missing.');

expectedOld='c9e6157cb7d1243a600042640dbd5b927ce07cdd54ace34557eafe7d001de3c5';
expectedNew='880cd6a3f9ec0c6d7a54445091becf776e0d766e8327aff7f2e899ac5ff41682';

h0=sha256_file(src);
hp=sha256_file(payload);
assert(strcmp(hp,expectedNew), ...
    '[AF7R39] packaged replacement hash mismatch.');

if strcmp(h0,expectedNew)
    fprintf(' exact repaired source is already installed = 1\n');
    OUT=struct('schema','AF7R39_EXACT_INSTALL_V1','installed',true, ...
        'already_installed',true,'source',src,'source_hash',h0);
    assignin('base','AF7R39_INSTALL_EXACT_CURRENT_SOURCE_OUT',OUT);
    fprintf('====================================================================================================\n');
    return;
end

assert(strcmp(h0,expectedOld), ...
    ['[AF7R39] CURRENT source does not match the uploaded file that was fully audited. ' ...
     'No write was performed. Upload the current REPLANNING_SYSTEM.m again instead of patching a different source.']);

stamp=datestr(now,'yyyymmdd_HHMMSS');
bak=fullfile(fileparts(src),['REPLANNING_SYSTEM_pre_AF7R39_' stamp '.m']);
[ok,msg]=copyfile(src,bak,'f');
assert(ok,'[AF7R39] backup failed: %s',msg);

[ok,msg]=copyfile(payload,src,'f');
assert(ok,'[AF7R39] exact-source replacement failed: %s',msg);

h1=sha256_file(src);
assert(strcmp(h1,expectedNew), ...
    '[AF7R39] written source hash does not match the audited replacement.');

rehash;

fprintf(' source                                  = %s\n',src);
fprintf(' backup                                  = %s\n',bak);
fprintf(' uploaded source SHA-256                 = %s\n',expectedOld);
fprintf(' repaired source SHA-256                 = %s\n',expectedNew);
fprintf(' exact full-file replacement             = 1\n');
fprintf(' string/regex source patching             = 0\n');
fprintf(' safety/deadline mathematics modified     = 0\n');
fprintf('====================================================================================================\n');

OUT=struct('schema','AF7R39_EXACT_INSTALL_V1','installed',true, ...
    'already_installed',false,'source',src,'backup',bak, ...
    'old_hash',expectedOld,'new_hash',expectedNew);
assignin('base','AF7R39_INSTALL_EXACT_CURRENT_SOURCE_OUT',OUT);
end

function h=sha256_file(fp)
fid=fopen(fp,'rb');
assert(fid>=0,'[AF7R39] cannot open file for SHA-256: %s',fp);
c=onCleanup(@()fclose(fid));
b=fread(fid,Inf,'*uint8');
clear c;
md=java.security.MessageDigest.getInstance('SHA-256');
md.update(b);
d=typecast(md.digest(),'uint8');
h=lower(reshape(dec2hex(d,2).',1,[]));
end
