function h=AF7R58R115_SHA256(path)
assert(isfile(path),'R115:HashFileMissing','Source file is missing.');
fid=fopen(path,'rb');
assert(fid>=0,'R115:HashOpen','Cannot open source file.');
cleanup=onCleanup(@()fclose(fid));
b=fread(fid,Inf,'*uint8');
d=java.security.MessageDigest.getInstance('SHA-256');
d.update(b);
h=lower(reshape(dec2hex(typecast(d.digest(),'uint8'),2).',1,[]));
end
