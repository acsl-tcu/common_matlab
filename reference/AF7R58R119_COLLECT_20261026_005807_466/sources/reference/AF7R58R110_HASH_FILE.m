function h=AF7R58R110_HASH_FILE(path)
% SHA256 of exact file bytes; never trust a class source name alone.
fid=fopen(path,'rb');
assert(fid~=-1,'R110:CannotRead','Cannot read source: %s',path);
c=onCleanup(@()fclose(fid));
b=fread(fid,Inf,'*uint8');
md=java.security.MessageDigest.getInstance('SHA-256');
md.update(typecast(b,'int8'));
v=typecast(md.digest(),'uint8');
h=lower(reshape(dec2hex(v,2).',1,[]));
end
