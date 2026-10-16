function OUT = AF7R40_INSTALL_REPAIRED_PACKET_BATCH()
clc;
fprintf('\n====================================================================================================\n');
fprintf(' AF7R40 | EXACT REPAIRED FUTURE-PACKET BATCH INSTALL\n');
fprintf('====================================================================================================\n');
fprintf('[AUTHORITY] exactR39SourceBound=1 | noRegexPatching=1 | safetyMathUnchanged=1 | batchAuthorityReuse=1\n');

src=which('REPLANNING_SYSTEM');
assert(~isempty(src)&&isfile(src),'[AF7R40] REPLANNING_SYSTEM.m unresolved.');
here=fileparts(mfilename('fullpath'));
payload=fullfile(here,'REPLANNING_SYSTEM_AF7R40_EXACT_SOURCE.txt');
assert(isfile(payload),'[AF7R40] exact source payload missing.');

expectedOld='880cd6a3f9ec0c6d7a54445091becf776e0d766e8327aff7f2e899ac5ff41682';
expectedNew='721fdc7cb5b771299215ebb69ec76fdbe6c1ec65554eaf1576c788e303491c71';
h0=sha256_file(src);
hp=sha256_file(payload);
assert(strcmp(hp,expectedNew),'[AF7R40] packaged source hash mismatch.');

if strcmp(h0,expectedNew)
    fprintf(' exact AF7R40 source already installed    = 1\n');
    OUT=struct('installed',true,'already_installed',true,'source',src,'hash',h0);
    assignin('base','AF7R40_INSTALL_OUT',OUT);
    fprintf('====================================================================================================\n');
    return;
end

assert(strcmp(h0,expectedOld), ...
    ['[AF7R40] current source is not the exact AF7R39 source audited for this replacement. ' ...
     'No write performed.']);

assert(exist('af7r33_flat_jet_batch_model','file')==2, ...
    '[AF7R40] required AF7R33 batch evaluator is not on path.');

stamp=datestr(now,'yyyymmdd_HHMMSS');
bak=fullfile(fileparts(src),['REPLANNING_SYSTEM_pre_AF7R40_' stamp '.m']);
[ok,msg]=copyfile(src,bak,'f');
assert(ok,'[AF7R40] backup failed: %s',msg);
[ok,msg]=copyfile(payload,src,'f');
assert(ok,'[AF7R40] exact source replacement failed: %s',msg);
assert(strcmp(sha256_file(src),expectedNew),'[AF7R40] written hash mismatch.');
rehash;

fprintf(' source                                  = %s\n',src);
fprintf(' backup                                  = %s\n',bak);
fprintf(' AF7R39 source SHA-256                   = %s\n',expectedOld);
fprintf(' AF7R40 source SHA-256                   = %s\n',expectedNew);
fprintf(' old scalar horizon eval count/cycle     = 2*N function-handle calls\n');
fprintf(' new repaired future packet              = one AF7R33 batch-flat evaluation\n');
fprintf(' safety/deadline mathematics modified    = 0\n');
fprintf('====================================================================================================\n');

OUT=struct('installed',true,'already_installed',false,'source',src,'backup',bak, ...
    'old_hash',expectedOld,'new_hash',expectedNew);
assignin('base','AF7R40_INSTALL_OUT',OUT);
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
