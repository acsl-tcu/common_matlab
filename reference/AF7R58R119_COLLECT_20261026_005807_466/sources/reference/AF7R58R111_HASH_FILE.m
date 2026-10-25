function h=AF7R58R111_HASH_FILE(path)
% Exact source-byte SHA256. No dependency on an offline audit workspace.
fid=fopen(path,'rb');
assert(fid~=-1,'R111:CannotRead','Cannot read file: %s',path);
c=onCleanup(@()fclose(fid)); %#ok<NASGU>
b=fread(fid,Inf,'*uint8');
md=java.security.MessageDigest.getInstance('SHA-256');
md.update(typecast(b,'int8'));
v=typecast(md.digest(),'uint8');
h=lower(reshape(dec2hex(v,2).',1,[]));
end
