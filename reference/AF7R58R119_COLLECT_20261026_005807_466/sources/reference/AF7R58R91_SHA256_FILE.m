function h=AF7R58R91_SHA256_FILE(p)
% SHA-256 over original bytes, matching the existing R88 installer approach.
assert(isfile(p),'R89:FileMissing');
d=java.security.MessageDigest.getInstance('SHA-256');
fid=fopen(p,'rb');assert(fid>=0,'R89:FileOpen');
c=onCleanup(@()fclose(fid));
while true
    b=fread(fid,2^16,'uint8=>uint8');
    if isempty(b),break;end
    d.update(b);
end
raw=typecast(d.digest(),'uint8');
h=lower(reshape(dec2hex(raw,2).',1,[]));
end
