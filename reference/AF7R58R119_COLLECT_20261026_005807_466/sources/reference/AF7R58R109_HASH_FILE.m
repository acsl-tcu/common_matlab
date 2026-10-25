function h=AF7R58R109_HASH_FILE(path)
% SHA256 over the exact bytes, independent of MATLAB newline handling.
fid=fopen(path,'rb');
assert(fid~=-1,'R109:OpenFile','Cannot open %s',path);
c=onCleanup(@()fclose(fid));
b=fread(fid,Inf,'*uint8');
md=java.security.MessageDigest.getInstance('SHA-256');
md.update(typecast(b,'int8'));
v=typecast(md.digest(),'uint8');
h=lower(reshape(dec2hex(v,2).',1,[]));
end
