function h=AF7R58R88_SHA256_FILE(file)
assert(isfile(file),'R88:FileMissing');
d=java.security.MessageDigest.getInstance('SHA-256');
fid=fopen(file,'rb');assert(fid>=0,'R88:FileOpenFailed');
c=onCleanup(@()fclose(fid));
while true
    b=fread(fid,2^16,'uint8=>uint8');
    if isempty(b),break;end
    d.update(b);
end
raw=typecast(d.digest(),'uint8');
h=lower(reshape(dec2hex(raw,2).',1,[]));
end
