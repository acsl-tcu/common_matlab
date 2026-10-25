function h=AF7R58R108_HASH_FILE(path)
% Hash exact bytes; no assumptions about line endings or MATLAB paths.
assert(isfile(path),'R108:HashMissing','File does not exist.');
fid=fopen(path,'rb');assert(fid>=0,'R108:HashOpen');
c=onCleanup(@()fclose(fid));raw=fread(fid,Inf,'*uint8');
d=java.security.MessageDigest.getInstance('SHA-256');d.update(raw);
h=lower(reshape(dec2hex(typecast(d.digest(),'uint8'),2).',1,[]));
end
