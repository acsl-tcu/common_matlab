function OUT = AF7R50C_INSTALL_CLEAN_TIMING()
clc;
fprintf('\n====================================================================================================\n');
fprintf(' AF7R50C | CLEAN TIMING A/B INSTALL\n');
fprintf('====================================================================================================\n');
fprintf('[AUTHORITY] exactR46SourceBound=1 | diagnosticsOnlyChanged=1 | safetyMathUnchanged=1\n');

src=which('REPLANNING_SYSTEM');
assert(~isempty(src)&&isfile(src),'[AF7R50C] REPLANNING_SYSTEM unresolved.');
here=fileparts(mfilename('fullpath'));
payload=fullfile(here,'REPLANNING_SYSTEM_AF7R50C_EXACT_SOURCE.txt');
assert(isfile(payload),'[AF7R50C] payload missing.');

old='890876116b4782725d8c337e53923a14892a9b5f4f00a0e7de030b9d4433c446'; new='223d34c49d7148b8ea14ff4ab9c420d409560c0650ac19e9b0f99318887090c9';
h0=sha256_file(src);
assert(strcmp(sha256_file(payload),new),'[AF7R50C] payload hash mismatch.');
if strcmp(h0,new)
    fprintf(' exact AF7R50C source already installed = 1\n');
    OUT=struct('installed',true,'already_installed',true,'source',src,'hash',h0);
    assignin('base','AF7R50C_INSTALL_OUT',OUT);
    fprintf('====================================================================================================\n');
    return;
end
assert(strcmp(h0,old), ...
    '[AF7R50C] current source is not exact AF7R46. No write performed.');

stamp=datestr(now,'yyyymmdd_HHMMSS');
bak=fullfile(fileparts(src),['REPLANNING_SYSTEM_pre_AF7R50C_' stamp '.m']);
[ok,msg]=copyfile(src,bak,'f'); assert(ok,'[AF7R50C] backup failed: %s',msg);
[ok,msg]=copyfile(payload,src,'f'); assert(ok,'[AF7R50C] replacement failed: %s',msg);
assert(strcmp(sha256_file(src),new),'[AF7R50C] written hash mismatch.');
rehash;

fprintf(' source                                  = %s\n',src);
fprintf(' backup                                  = %s\n',bak);
fprintf(' AF7R46 hash                             = %s\n',old);
fprintf(' AF7R50C hash                            = %s\n',new);
fprintf(' light runtime audit                     = OFF\n');
fprintf(' true-trigger replay capture             = OFF\n');
fprintf(' proposal diagnostic mirror              = OFF\n');
fprintf(' high-res callback QPC stamps            = ON (kept for independent callback evidence)\n');
fprintf(' observer hard-support mathematics       = UNCHANGED\n');
fprintf(' R9 / finite-sensing / rejoin / geometry = UNCHANGED\n');
fprintf(' hard 4 ms deadline                      = UNCHANGED\n');
fprintf('====================================================================================================\n');

OUT=struct('schema','AF7R50C_INSTALL_V1','installed',true,'already_installed',false, ...
    'source',src,'backup',bak,'old_hash',old,'new_hash',new);
assignin('base','AF7R50C_INSTALL_OUT',OUT);
end

function h=sha256_file(fp)
fid=fopen(fp,'rb'); assert(fid>=0); c=onCleanup(@()fclose(fid));
b=fread(fid,Inf,'*uint8'); clear c;
md=java.security.MessageDigest.getInstance('SHA-256'); md.update(b);
d=typecast(md.digest(),'uint8'); h=lower(reshape(dec2hex(d,2).',1,[]));
end
