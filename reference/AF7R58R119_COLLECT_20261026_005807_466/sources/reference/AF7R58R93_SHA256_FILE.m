function out=AF7R58R93_SHA256_FILE(path)
assert(ischar(path)||isstring(path),'R93:FilePathRequired');
path=char(path);
assert(isfile(path),'R93:SourceFileMissing');
fid=fopen(path,'rb');assert(fid>0,'R93:CannotReadSource');
clean=onCleanup(@()fclose(fid)); %#ok<NASGU>
bytes=fread(fid,Inf,'*uint8');
md=java.security.MessageDigest.getInstance('SHA-256');
md.update(bytes);
b=typecast(md.digest(),'uint8');
out=lower(reshape(dec2hex(b,2).',1,[]));
end
