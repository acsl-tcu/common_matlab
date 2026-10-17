function OUT = AF7R53_INSTALL_PREPARED_COMMIT_SWITCH()
clc;
fprintf('\n====================================================================================================\n');
fprintf(' AF7R53A | PREPARED COMMIT SWITCH INSTALL\n');
fprintf('====================================================================================================\n');
fprintf('[AUTHORITY] exactR50CSourceBound=1 | switchDefaultOff=1 | safetyPredicatesUnchanged=1\n');

src=which('REPLANNING_SYSTEM');
assert(~isempty(src)&&isfile(src),'[AF7R53A] REPLANNING_SYSTEM unresolved.');
here=fileparts(mfilename('fullpath'));
payload=fullfile(here,'REPLANNING_SYSTEM_AF7R53_EXACT_SOURCE.txt');
assert(isfile(payload),'[AF7R53A] exact payload missing.');

old='223d34c49d7148b8ea14ff4ab9c420d409560c0650ac19e9b0f99318887090c9';
new='c16b244794fc7bc6f2d7794f81fd83d2f62944def60fd606454fe472fc012383';
h0=sha256_file(src);
assert(strcmp(sha256_file(payload),new),'[AF7R53A] payload hash mismatch.');

if strcmp(h0,new)
    fprintf(' exact AF7R53 source already installed   = 1\n');
    OUT=struct('schema','AF7R53A_INSTALL_V1','closed',true,'already_installed',true,'hash',h0);
    assignin('base','AF7R53A_INSTALL_OUT',OUT);
    fprintf('====================================================================================================\n');
    return;
end

assert(strcmp(h0,old), ...
    '[AF7R53A] current source is not exact AF7R50C. No write performed.');
assert(exist('af7r53_prepared_commit_mex','file')==3, ...
    '[AF7R53A] build/add AF7R53 prepared MEX before install.');

stamp=datestr(now,'yyyymmdd_HHMMSS');
bak=fullfile(fileparts(src),['REPLANNING_SYSTEM_pre_AF7R53_' stamp '.m']);
[ok,msg]=copyfile(src,bak,'f');assert(ok,'[AF7R53A] backup failed: %s',msg);
[ok,msg]=copyfile(payload,src,'f');assert(ok,'[AF7R53A] write failed: %s',msg);
assert(strcmp(sha256_file(src),new),'[AF7R53A] post-write hash mismatch.');
rehash;

fprintf(' source                                  = %s\n',src);
fprintf(' backup                                  = %s\n',bak);
fprintf(' AF7R50C hash                            = %s\n',old);
fprintf(' AF7R53 hash                             = %s\n',new);
fprintf(' switch default                          = OFF\n');
fprintf(' OFF path                                = legacy R50C commit\n');
fprintf(' ON path                                 = R52-equivalent prepared packet\n');
fprintf(' final currentness asserts               = RETAINED\n');
fprintf(' final hard 4 ms check                   = RETAINED\n');
fprintf(' sampled acceptance                      = 0\n');
fprintf('====================================================================================================\n');

OUT=struct('schema','AF7R53A_INSTALL_V1','closed',true,'already_installed',false, ...
    'source',src,'backup',bak,'old_hash',old,'new_hash',new);
assignin('base','AF7R53A_INSTALL_OUT',OUT);
end

function h=sha256_file(fp)
fid=fopen(fp,'rb');assert(fid>=0);c=onCleanup(@()fclose(fid));
b=fread(fid,Inf,'*uint8');clear c;
md=java.security.MessageDigest.getInstance('SHA-256');md.update(b);
d=typecast(md.digest(),'uint8');h=lower(reshape(dec2hex(d,2).',1,[]));
end
