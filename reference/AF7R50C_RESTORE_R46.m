function OUT = AF7R50C_RESTORE_R46()
clc;
fprintf('\n====================================================================================================\n');
fprintf(' AF7R50C | RESTORE EXACT AF7R46 SOURCE\n');
fprintf('====================================================================================================\n');

src=which('REPLANNING_SYSTEM');
assert(~isempty(src)&&isfile(src),'[AF7R50C restore] REPLANNING_SYSTEM unresolved.');
folder=fileparts(src);
D=dir(fullfile(folder,'REPLANNING_SYSTEM_pre_AF7R50C_*.m'));
assert(~isempty(D),'[AF7R50C restore] no AF7R50C backup found.');

[~,ix]=max([D.datenum]);
bak=fullfile(D(ix).folder,D(ix).name);
expected='890876116b4782725d8c337e53923a14892a9b5f4f00a0e7de030b9d4433c446';
assert(strcmp(sha256_file(bak),expected), ...
    '[AF7R50C restore] newest backup is not exact AF7R46.');

[ok,msg]=copyfile(bak,src,'f'); assert(ok,'[AF7R50C restore] restore failed: %s',msg);
assert(strcmp(sha256_file(src),expected),'[AF7R50C restore] restored hash mismatch.');
rehash;

fprintf(' restored from                           = %s\n',bak);
fprintf(' restored AF7R46 SHA-256                 = %s\n',expected);
fprintf('====================================================================================================\n');
OUT=struct('schema','AF7R50C_RESTORE_R46_V1','restored',true,'source',src,'backup',bak);
assignin('base','AF7R50C_RESTORE_OUT',OUT);
end

function h=sha256_file(fp)
fid=fopen(fp,'rb'); assert(fid>=0); c=onCleanup(@()fclose(fid));
b=fread(fid,Inf,'*uint8'); clear c;
md=java.security.MessageDigest.getInstance('SHA-256'); md.update(b);
d=typecast(md.digest(),'uint8'); h=lower(reshape(dec2hex(d,2).',1,[]));
end
