function h=AF7R58R86_SHA256_FILE(path)
% Exact raw file SHA-256 for the one-time source-bound installer.
assert(ischar(path)||isstring(path),'R86:InvalidPath');
fid=fopen(char(path),'rb');
assert(fid>=0,'R86:HashOpenFailed');
c=onCleanup(@()fclose(fid)); %#ok<NASGU>
b=fread(fid,Inf,'*uint8');
md=java.security.MessageDigest.getInstance('SHA-256');
md.update(b);
h=lower(reshape(dec2hex(typecast(int8(md.digest()),'uint8'),2).',1,[]));
end
