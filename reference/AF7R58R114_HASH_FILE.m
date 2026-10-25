function h=AF7R58R114_HASH_FILE(path)
fid=fopen(path,'rb');
assert(fid>=0,'R114:CannotRead','Cannot read file: %s',path);
cleanup=onCleanup(@()fclose(fid)); %#ok<NASGU>
bytes=fread(fid,Inf,'*uint8');
md=java.security.MessageDigest.getInstance('SHA-256');
md.update(typecast(bytes,'int8'));
h=lower(reshape(dec2hex(typecast(md.digest(),'uint8'),2).',1,[]));
end
