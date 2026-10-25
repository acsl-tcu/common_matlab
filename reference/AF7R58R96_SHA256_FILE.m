function hash=AF7R58R96_SHA256_FILE(path)
% Dependency-local SHA256, no calls to any R93/R94 helper.
assert(ischar(path)||isstring(path),'R96:SourcePathRequired', ...
    'Provide the source filename.');
path=char(path);
assert(isfile(path),'R96:SourceMissing','Source file does not exist: %s',path);
f=fopen(path,'rb');
assert(f>0,'R96:ReadFailed','Cannot read source %s',path);
c=onCleanup(@()fclose(f));
raw=fread(f,Inf,'*uint8');
md=java.security.MessageDigest.getInstance('SHA-256');
md.update(raw);
bytes=typecast(md.digest(),'uint8');
hash=lower(reshape(dec2hex(bytes,2).',1,[]));
end
