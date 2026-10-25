function h=AF7R58R83_SHA256_FILE(path)
% Exact raw-byte SHA256. Used only during one-time offline installation.
assert(ischar(path)||isstring(path),'R83:InvalidPath');
fid=fopen(char(path),'rb');
assert(fid>=0,'R83:HashOpenFailed');
c=onCleanup(@()fclose(fid));
b=fread(fid,Inf,'*uint8');
md=java.security.MessageDigest.getInstance('SHA-256');
md.update(b);
bytes=typecast(int8(md.digest()),'uint8');
h=lower(reshape(dec2hex(bytes,2).',1,[]));
end
