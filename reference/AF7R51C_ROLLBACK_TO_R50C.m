function OUT = AF7R51C_ROLLBACK_TO_R50C()
% Restore only REPLANNING_SYSTEM from the exact backup created immediately
% before AF7R51.  The repaired AF7R51P V2 post-run dashboard is intentionally
% kept; run_flight_headless.m is not modified here.

clc;
fprintf('\n====================================================================================================\n');
fprintf(' AF7R51C | ROLLBACK AF7R51 COMMIT EXPERIMENT -> EXACT AF7R50C\n');
fprintf('====================================================================================================\n');
fprintf('[AUTHORITY] exactBackupOnly=1 | plotDashboardKept=1 | noSafetyRedesign=1\n');

src=which('REPLANNING_SYSTEM');
assert(~isempty(src)&&isfile(src),'[AF7R51C] REPLANNING_SYSTEM unresolved.');
folder=fileparts(src);

expectedR50C='223d34c49d7148b8ea14ff4ab9c420d409560c0650ac19e9b0f99318887090c9';
expectedR51 ='8f9aa73bdb2780f1041eac24e69251f501914326dae62705e1fd14154f2d82a7';

h0=sha256_file(src);
if strcmp(h0,expectedR50C)
    fprintf(' exact AF7R50C already active             = 1\n');
    OUT=struct('schema','AF7R51C_ROLLBACK_V1','closed',true, ...
        'already_restored',true,'source',src,'sha256',h0);
    assignin('base','AF7R51C_ROLLBACK_OUT',OUT);
    fprintf('====================================================================================================\n');
    return;
end
assert(strcmp(h0,expectedR51), ...
    '[AF7R51C] current source is neither exact R51 nor exact R50C. No write performed.');

D=dir(fullfile(folder,'REPLANNING_SYSTEM_pre_AF7R51_*.m'));
assert(~isempty(D),'[AF7R51C] pre-AF7R51 backup not found.');
[~,ord]=sort([D.datenum],'descend');

bak='';
for ii=ord
    q=fullfile(D(ii).folder,D(ii).name);
    if strcmp(sha256_file(q),expectedR50C)
        bak=q;
        break;
    end
end
assert(~isempty(bak),'[AF7R51C] no exact AF7R50C backup found.');

stamp=datestr(now,'yyyymmdd_HHMMSS');
r51copy=fullfile(folder,['REPLANNING_SYSTEM_AF7R51_regression_' stamp '.m']);
[ok,msg]=copyfile(src,r51copy,'f');
assert(ok,'[AF7R51C] R51 preservation failed: %s',msg);

[ok,msg]=copyfile(bak,src,'f');
assert(ok,'[AF7R51C] restore failed: %s',msg);
assert(strcmp(sha256_file(src),expectedR50C),'[AF7R51C] restored hash mismatch.');
rehash;

fprintf(' R51 preserved                            = %s\n',r51copy);
fprintf(' restored from                            = %s\n',bak);
fprintf(' restored SHA-256                         = %s\n',expectedR50C);
fprintf(' run_flight_headless plot V2 touched      = 0\n');
fprintf(' AF7R51C rollback closed                  = 1\n');
fprintf('====================================================================================================\n');

OUT=struct('schema','AF7R51C_ROLLBACK_V1','closed',true, ...
    'already_restored',false,'source',src,'backup',bak, ...
    'preserved_r51',r51copy,'sha256',expectedR50C);
assignin('base','AF7R51C_ROLLBACK_OUT',OUT);
end

function h=sha256_file(fp)
fid=fopen(fp,'rb'); assert(fid>=0); c=onCleanup(@()fclose(fid));
b=fread(fid,Inf,'*uint8'); clear c;
md=java.security.MessageDigest.getInstance('SHA-256'); md.update(b);
d=typecast(md.digest(),'uint8'); h=lower(reshape(dec2hex(d,2).',1,[]));
end
