function R=TRIGGER_NATIVE_PE_CODE_IDENTITY(binary_path)
% REPRODUCIBLE CODE identity for a Windows x64 MEX PE binary.
% Not a numerical-parity or continuous-safety theorem.
% Hashes executable and loaded data sections, excluding ONLY PE build
% timestamp (COFF header + its unique copy in .rdata), and excludes
% .rsrc containing compiler/MATLAB manifests. Unknown PE layouts FAIL.
% This is OFFLINE / PREFLIGHT ONLY: no call from Flight update.
R=struct('valid',false,'status','UNVERIFIED_PE','fingerprint','',...
    'whole_binary_sha256','','path',char(binary_path));
if ~isfile(binary_path), R.status='MISSING_BINARY';return;end
f=fopen(binary_path,'rb','ieee-le');
if f<0,R.status='UNREADABLE_BINARY';return;end
cleaner=onCleanup(@()fclose(f));
b=fread(f,inf,'*uint8');
R.whole_binary_sha256=local_sha(b);
try
    assert(numel(b)>=512 && isequal(char(b(1:2).'),'MZ'));
    pe=double(local_u32(b,61))+1;
    assert(pe>=65 && pe+23<=numel(b));
    assert(isequal(b(pe:pe+3).',uint8([80 69 0 0])));
    assert(local_u16(b,pe+4)==hex2dec('8664')); % AMD64
    count=double(local_u16(b,pe+6));
    assert(count>=5 && count<=32);
    stamp=b(pe+8:pe+11); % COFF TimeDateStamp
    optional=double(local_u16(b,pe+20));
    section_start=pe+24+optional;
    required={'.text','.rdata','.data','.pdata','.reloc'};
    sections=cell(1,numel(required));
    found=false(1,numel(required));
    for k=1:count
        h=section_start+(k-1)*40;
        assert(h+39<=numel(b));
        chars=char(b(h:h+7).');
        name=chars(1:find(chars==0,1,'first')-1);
        if isempty(name),name=chars;end
        idx=find(strcmp(required,name),1);
        if isempty(idx)
            assert(strcmp(name,'.rsrc')); % no extra executable/data sections
            continue
        end
        assert(~found(idx));
        raw_size=double(local_u32(b,h+16));
        raw_offset=double(local_u32(b,h+20))+1;
        assert(raw_size>0 && raw_offset>0 && ...
            raw_offset+raw_size-1<=numel(b));
        sections{idx}=b(raw_offset:raw_offset+raw_size-1);
        found(idx)=true;
    end
    assert(all(found));
    r=sections{2};
    % Do NOT globally zero all 4-byte occurrences: only one exact
    % build-time timestamp is allowed in this verified format.
    locations=find(r(1:end-3)==stamp(1) & r(2:end-2)==stamp(2) & ...
        r(3:end-1)==stamp(3) & r(4:end)==stamp(4));
    assert(numel(locations)==1);
    r(locations(1):locations(1)+3)=uint8(0);
    sections{2}=r;
    digest_bytes=uint8([]);
    for j=1:numel(required)
        payload=sections{j};
        digest_bytes=[digest_bytes;uint8(required{j}(:));uint8(0);...
            typecast(uint32(numel(payload)),'uint8').';payload]; %#ok<AGROW>
    end
    R.fingerprint=local_sha(digest_bytes);
    R.valid=true;
    R.status='VERIFIED_PE_SECTION_LAYOUT__NO_RUNTIME_PARITY_CLAIM';
catch
    R.valid=false;
    R.status='UNKNOWN_PE_LAYOUT_OR_NONREPRODUCIBLE_DATA';
end
end
function v=local_u16(b,i)
assert(i>=1 && i+1<=numel(b));
v=double(b(i))+256*double(b(i+1));
end
function v=local_u32(b,i)
assert(i>=1 && i+3<=numel(b));
v=double(b(i))+256*double(b(i+1))+65536*double(b(i+2))+16777216*double(b(i+3));
end
function h=local_sha(bytes)
md=java.security.MessageDigest.getInstance('SHA-256');
md.update(typecast(uint8(bytes(:)),'int8'));
raw=typecast(int8(md.digest()),'uint8');
h=lower(reshape(dec2hex(raw,2).',1,[]));
end
