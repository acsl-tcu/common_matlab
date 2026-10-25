function hash=AF7R58R103_SHA256_FILE(path)
% Exact bytes of the local MATLAB source, independent of older helper names.
assert(ischar(path)||isstring(path),'R103:SourcePathRequired', ...
    'Provide an existing source file path.');
path=char(path);
assert(isfile(path),'R103:SourceMissing','Missing source: %s',path);
f=fopen(path,'rb');
assert(f>0,'R103:ReadFailed','Cannot read source %s',path);
c=onCleanup(@()fclose(f)); %#ok<NASGU>
data=fread(f,Inf,'*uint8');
h=java.security.MessageDigest.getInstance('SHA-256');
h.update(data);bytes=typecast(h.digest(),'uint8');
hash=lower(reshape(dec2hex(bytes,2).',1,[]));
end
