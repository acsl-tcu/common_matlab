function OUT = AF7R46_INSTALL_LAST_TWO_REALTIME()
clc;
fprintf('\n====================================================================================================\n');
fprintf(' AF7R46 | LAST-TWO REALTIME CLEANUP INSTALL\n');
fprintf('====================================================================================================\n');
fprintf('[AUTHORITY] exactR45SourceBound=1 | sameNativeCore=1 | safetyMathUnchanged=1\n');
src=which('REPLANNING_SYSTEM'); assert(~isempty(src)&&isfile(src));
here=fileparts(mfilename('fullpath')); payload=fullfile(here,'REPLANNING_SYSTEM_AF7R46_EXACT_SOURCE.txt');
old='ded3e68fb1b7da5187985e1f171dc970afeb82d9ba09fecacbc4f07f553936d9'; new='890876116b4782725d8c337e53923a14892a9b5f4f00a0e7de030b9d4433c446'; h0=sha256_file(src); assert(strcmp(sha256_file(payload),new));
if strcmp(h0,new), OUT=struct('installed',true,'already_installed',true,'source',src); assignin('base','AF7R46_INSTALL_OUT',OUT); return; end
assert(strcmp(h0,old),'[AF7R46] current source is not exact AF7R45; no write performed.');
stamp=datestr(now,'yyyymmdd_HHMMSS'); bak=fullfile(fileparts(src),['REPLANNING_SYSTEM_pre_AF7R46_' stamp '.m']);
[ok,msg]=copyfile(src,bak,'f'); assert(ok,'%s',msg); [ok,msg]=copyfile(payload,src,'f'); assert(ok,'%s',msg);
assert(strcmp(sha256_file(src),new)); rehash;
fprintf(' source                                  = %s\n',src);
fprintf(' backup                                  = %s\n',bak);
fprintf(' AF7R45 hash                             = %s\n',old);
fprintf(' AF7R46 hash                             = %s\n',new);
fprintf(' native mathematical core changed        = 0\n');
fprintf(' MATLAB proposal materialization         = COMPACT\n');
fprintf(' changed-speed packet rebuild            = PURE_FAST_EXACT\n');
fprintf(' observer equations changed              = 0\n');
fprintf(' observer static setup cached            = PRE-MOTION\n');
fprintf(' Takeoff/Arming heavy work added         = 0\n');
fprintf('====================================================================================================\n');
OUT=struct('schema','AF7R46_INSTALL_V1','installed',true,'already_installed',false,'source',src,'backup',bak,'old_hash',old,'new_hash',new); assignin('base','AF7R46_INSTALL_OUT',OUT);
end
function h=sha256_file(fp)
fid=fopen(fp,'rb'); assert(fid>=0); c=onCleanup(@()fclose(fid)); b=fread(fid,Inf,'*uint8'); clear c; md=java.security.MessageDigest.getInstance('SHA-256'); md.update(b); d=typecast(md.digest(),'uint8'); h=lower(reshape(dec2hex(d,2).',1,[]));
end
