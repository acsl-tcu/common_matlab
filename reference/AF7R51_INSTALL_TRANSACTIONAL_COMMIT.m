function OUT = AF7R51_INSTALL_TRANSACTIONAL_COMMIT()
clc;
fprintf('\n====================================================================================================\n');
fprintf(' AF7R51 | TRANSACTIONAL REFERENCE-ONLY COMMIT INSTALL\n');
fprintf('====================================================================================================\n');
fprintf('[AUTHORITY] exactR50CSourceBound=1 | safetyPredicatesUnchanged=1 | hard4msUnchanged=1\n');

src=which('REPLANNING_SYSTEM');
assert(~isempty(src)&&isfile(src),'[AF7R51] REPLANNING_SYSTEM unresolved.');
here=fileparts(mfilename('fullpath'));
payload=fullfile(here,'REPLANNING_SYSTEM_AF7R51_EXACT_SOURCE.txt');
old='223d34c49d7148b8ea14ff4ab9c420d409560c0650ac19e9b0f99318887090c9'; new='8f9aa73bdb2780f1041eac24e69251f501914326dae62705e1fd14154f2d82a7';
h0=sha256_file(src);
assert(strcmp(sha256_file(payload),new),'[AF7R51] payload hash mismatch.');
if strcmp(h0,new)
    fprintf(' exact AF7R51 already installed            = 1\n');
    OUT=struct('installed',true,'already_installed',true,'source',src,'hash',h0);
    assignin('base','AF7R51_INSTALL_OUT',OUT);
    fprintf('====================================================================================================\n');
    return;
end
assert(strcmp(h0,old), ...
    '[AF7R51] current source is not exact AF7R50C. No write performed.');

stamp=datestr(now,'yyyymmdd_HHMMSS');
bak=fullfile(fileparts(src),['REPLANNING_SYSTEM_pre_AF7R51_' stamp '.m']);
[ok,msg]=copyfile(src,bak,'f'); assert(ok,'[AF7R51] backup failed: %s',msg);
[ok,msg]=copyfile(payload,src,'f'); assert(ok,'[AF7R51] replacement failed: %s',msg);
assert(strcmp(sha256_file(src),new),'[AF7R51] written hash mismatch.');
rehash;

fprintf(' source                                  = %s\n',src);
fprintf(' backup                                  = %s\n',bak);
fprintf(' AF7R50C hash                            = %s\n',old);
fprintf(' AF7R51 hash                             = %s\n',new);
fprintf(' native AF7R36 mathematics               = UNCHANGED\n');
fprintf(' continuous geometry / R9 / sensing      = UNCHANGED\n');
fprintf(' commit construction                     = LOCAL PREVALIDATE THEN SWAP\n');
fprintf(' flat validation                         = EXACT AF7R33 BATCH JET\n');
fprintf(' rollback on preparation deadline miss   = REMOVED (authority never mutated)\n');
fprintf(' final post-swap hard deadline check     = RETAINED\n');
fprintf(' sampled acceptance                      = 0\n');
fprintf('====================================================================================================\n');

OUT=struct('schema','AF7R51_INSTALL_V1','installed',true,'already_installed',false, ...
    'source',src,'backup',bak,'old_hash',old,'new_hash',new);
assignin('base','AF7R51_INSTALL_OUT',OUT);
end
function h=sha256_file(fp)
fid=fopen(fp,'rb'); assert(fid>=0); c=onCleanup(@()fclose(fid));
b=fread(fid,Inf,'*uint8'); clear c;
md=java.security.MessageDigest.getInstance('SHA-256'); md.update(b);
d=typecast(md.digest(),'uint8'); h=lower(reshape(dec2hex(d,2).',1,[]));
end
