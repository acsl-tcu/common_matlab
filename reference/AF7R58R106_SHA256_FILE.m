function h=AF7R58R106_SHA256_FILE(path)
% SHA256 of unchanged source bytes. Does not depend on R94/R88 utilities.
assert(ischar(path)||isstring(path),'R106:HashPath','File path required.');
assert(isfile(path),'R106:HashMissing','File must exist.');
fid=fopen(path,'rb');
assert(fid>0,'R106:HashRead','Cannot open source.');
c=onCleanup(@()fclose(fid));
raw=fread(fid,Inf,'*uint8');
d=java.security.MessageDigest.getInstance('SHA-256');
d.update(raw);
h=lower(reshape(dec2hex(typecast(d.digest(),'uint8'),2).',1,[]));
end
