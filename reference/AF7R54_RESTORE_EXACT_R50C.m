function OUT = AF7R54_RESTORE_EXACT_R50C()
clc;
fprintf('\n====================================================================================================\n');
fprintf(' AF7R54 | RESTORE EXACT AF7R50C SOURCE\n');
fprintf('====================================================================================================\n');
src=which('REPLANNING_SYSTEM');
assert(~isempty(src)&&isfile(src));
here=fileparts(mfilename('fullpath'));
payload=fullfile(here,'REPLANNING_SYSTEM_AF7R50C_EXACT_SOURCE.txt');
expected='223d34c49d7148b8ea14ff4ab9c420d409560c0650ac19e9b0f99318887090c9';
assert(strcmp(sha256_file(payload),expected),'[AF7R54 restore] R50C payload hash mismatch.');

stamp=datestr(now,'yyyymmdd_HHMMSS');
bak=fullfile(fileparts(src),['REPLANNING_SYSTEM_pre_AF7R53_restore_' stamp '.m']);
[ok,msg]=copyfile(src,bak,'f');assert(ok,'[AF7R54 restore] backup failed: %s',msg);
[ok,msg]=copyfile(payload,src,'f');assert(ok,'[AF7R54 restore] restore failed: %s',msg);
assert(strcmp(sha256_file(src),expected),'[AF7R54 restore] restored hash mismatch.');
rehash;

fprintf(' restored source                         = %s\n',src);
fprintf(' backup of prior source                  = %s\n',bak);
fprintf(' AF7R50C SHA-256                         = %s\n',expected);
fprintf(' run-script switch/dashboard touched     = 0\n');
fprintf('====================================================================================================\n');
OUT=struct('schema','AF7R54_RESTORE_R50C_V1','closed',true,'source',src,'backup',bak);
assignin('base','AF7R54_RESTORE_R50C_OUT',OUT);
end

function h=sha256_file(fp)
fid=fopen(fp,'rb');assert(fid>=0);c=onCleanup(@()fclose(fid));
b=fread(fid,Inf,'*uint8');clear c;
md=java.security.MessageDigest.getInstance('SHA-256');md.update(b);
d=typecast(md.digest(),'uint8');h=lower(reshape(dec2hex(d,2).',1,[]));
end
