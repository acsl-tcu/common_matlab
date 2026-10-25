function h=AF7R58R87_SHA256_FILE(path)
% Read-only exact source-byte SHA-256. No production mutation.
assert(ischar(path)||isstring(path),'R87:InvalidPath');
fid=fopen(char(path),'rb');assert(fid>=0,'R87:HashOpenFailed');
c=onCleanup(@()fclose(fid)); %#ok<NASGU>
b=fread(fid,Inf,'*uint8');
md=java.security.MessageDigest.getInstance('SHA-256');
md.update(b);
h=lower(reshape(dec2hex(typecast(int8(md.digest()),'uint8'),2).',1,[]));
end
