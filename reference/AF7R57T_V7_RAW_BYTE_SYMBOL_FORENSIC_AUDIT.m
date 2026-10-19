function OUT = AF7R57T_V7_RAW_BYTE_SYMBOL_FORENSIC_AUDIT()
% AF7R57T_V7_RAW_BYTE_SYMBOL_FORENSIC_AUDIT
% Read-only forensic for the V6 false-negative on af7r36_single_core_proposal.
% No production modification, no theorem upgrade, no backend reclassification.

clc;
fprintf('\n====================================================================================================\n');
fprintf(' AF7R57T V7 | RAW-BYTE / FILEREAD SYMBOL FORENSIC\n');
fprintf('====================================================================================================\n');
fprintf('[AUTHORITY] readOnly=1 | noParser=1 | rawBytes=1 | noClassificationUpgrade=1 | productionModified=0\n');

root=fileparts(mfilename('fullpath'));
self=[mfilename('fullpath') '.m'];
rps=fullfile(root,'REPLANNING_SYSTEM.m');
prime=fullfile(root,'af7r36_runtime_prime.m');
prop=fullfile(root,'af7r36_single_core_proposal.m');

assert(isfile(rps),'[AF7R57T V7] REPLANNING_SYSTEM.m missing.');
assert(isfile(prime),'[AF7R57T V7] af7r36_runtime_prime.m missing.');
assert(isfile(prop),'[AF7R57T V7] af7r36_single_core_proposal.m missing.');

fprintf(' immutable entrypoint                    = %s\n',self);
fprintf(' immutable entrypoint SHA-256             = %s\n',sha256_file(self));

expR='223d34c49d7148b8ea14ff4ab9c420d409560c0650ac19e9b0f99318887090c9';
expP='300adebef4140d68cfc393f43472f378bfe0ebf6fa00fcea6f0e386f76ae9cb4';
expG='d7bbe05efd268743e7307bcf3291f8f0e7777ab126432abc190ac60c999cc7ac';
shaR=sha256_file(rps); shaP=sha256_file(prime); shaG=sha256_file(prop);

fprintf('\n[A. EXACT SOURCE BINDING]\n');
fprintf(' REPLANNING_SYSTEM SHA-256               = %s\n',shaR);
fprintf(' bound REPLANNING_SYSTEM current         = %d\n',strcmp(shaR,expR));
fprintf(' af7r36_runtime_prime current            = %d\n',strcmp(shaP,expP));
fprintf(' af7r36_single_core_proposal current     = %d\n',strcmp(shaG,expG));

% Read the exact raw bytes and fileread text independently.
fid=fopen(rps,'rb'); assert(fid>=0,'[AF7R57T V7] cannot open source as raw bytes.');
c=onCleanup(@()fclose(fid));
raw=fread(fid,Inf,'*uint8').';
clear c;
src=fileread(rps);

needle='af7r36_single_core_proposal';
needle8=uint8(needle);
rawPos=find_bytes(raw,needle8);
charPos=strfind(src,needle);

fprintf('\n[B. RAW-BYTE VS FILEREAD TOKEN TEST]\n');
fprintf(' raw byte count                         = %d\n',numel(raw));
fprintf(' fileread char count                    = %d\n',numel(src));
fprintf(' raw exact ASCII token count             = %d\n',numel(rawPos));
fprintf(' fileread exact char token count         = %d\n',numel(charPos));
if ~isempty(rawPos), fprintf(' raw first exact token byte position      = %d\n',rawPos(1)); end
if ~isempty(charPos), fprintf(' fileread first exact token char position = %d\n',charPos(1)); end

% Broader raw/text probes; no assertion.
probes={'af7r36','single_core','single_core_proposal','proposal','af7r36_single_core'};
fprintf('\n[C. BROAD TOKEN COUNTS]\n');
for k=1:numel(probes)
    pr=probes{k};
    rp=find_bytes(raw,uint8(pr));
    cp=strfind(src,pr);
    fprintf(' %-28s raw=%d | text=%d\n',pr,numel(rp),numel(cp));
end

% Print every source line containing a relevant broad fragment.
lines=regexp(src,'\r\n|\n|\r','split');
sel=false(size(lines));
for i=1:numel(lines)
    li=lines{i};
    sel(i)=contains(li,'af7r36','IgnoreCase',true) || ...
           contains(li,'single_core','IgnoreCase',true) || ...
           contains(li,'generator_backend_locked_callback','IgnoreCase',true);
end
idx=find(sel);
fprintf('\n[D. EXACT SOURCE LINES WITH RELEVANT SYMBOLS]\n');
fprintf(' relevant line count                    = %d\n',numel(idx));
for q=1:numel(idx)
    i=idx(q);
    fprintf(' L%-6d %s\n',i,lines{i});
end

% For lines mentioning af7r36/single_core, print UTF-16 code units. This exposes
% zero-width/non-ASCII/homoglyph characters without relying on display rendering.
fprintf('\n[E. CHARACTER-CODE FORENSICS]\n');
for q=1:numel(idx)
    i=idx(q); li=lines{i};
    if contains(li,'af7r36','IgnoreCase',true) || contains(li,'single_core','IgnoreCase',true)
        codes=double(li);
        fprintf(' L%-6d codes =',i);
        fprintf(' %d',codes);
        fprintf('\n');
    end
end

% Also locate the code==1, prime, proposal broad token, and callback-store in
% full text, independently. No chain theorem is asserted here.
tIf='obj.generator_backend_locked_code)==1';
tPrime='af7r36_runtime_prime';
tPropBroad='single_core_proposal';
tStore='obj.generator_backend_locked_callback=fn';
P1=strfind(src,tIf); P2=strfind(src,tPrime); P3=strfind(src,tPropBroad); P4=strfind(src,tStore);

fprintf('\n[F. ABSOLUTE BROAD POSITIONS - DIAGNOSTIC ONLY]\n');
fprintf(' code==1 positions                     = %s\n',mat2str(P1));
fprintf(' runtime_prime positions                = %s\n',mat2str(P2));
fprintf(' single_core_proposal positions         = %s\n',mat2str(P3));
fprintf(' locked_callback store positions        = %s\n',mat2str(P4));

exactAgreement=(isempty(rawPos)&&isempty(charPos)) || (~isempty(rawPos)&&~isempty(charPos));
sourceHasExact=~isempty(rawPos);

fprintf('\n[G. FORENSIC GATE]\n');
fprintf(' raw/text exact-token agreement           = %d\n',exactAgreement);
fprintf(' exact proposal symbol physically present = %d\n',sourceHasExact);
fprintf(' backend classification upgraded          = 0\n');
fprintf(' production source modified               = 0\n');
fprintf(' timing claim upgraded                    = 0\n');
if sourceHasExact
    fprintf(' next action                              = FIX_AUDIT_ONLY__THEN_CLOSE_OPERATIONAL_BINDING\n');
else
    fprintf(' next action                              = USE_PRINTED_RAW_SOURCE_LINE_TO_BIND_ACTUAL_PROPOSAL_SYMBOL__DO_NOT_ASSUME_NAME\n');
end
fprintf('====================================================================================================\n');

OUT=struct();
OUT.schema='AF7R57T_V7_RAW_BYTE_SYMBOL_FORENSIC_V1';
OUT.source_sha256=shaR;
OUT.raw_exact_count=numel(rawPos);
OUT.text_exact_count=numel(charPos);
OUT.raw_text_agree=exactAgreement;
OUT.exact_symbol_present=sourceHasExact;
OUT.relevant_lines=idx;
OUT.production_modified=false;
OUT.entrypoint=self;
OUT.entrypoint_sha256=sha256_file(self);
assignin('base','AF7R57T_V7_RAW_BYTE_SYMBOL_FORENSIC_AUDIT_OUT',OUT);
save(fullfile(root,'AF7R57T_V7_RAW_BYTE_SYMBOL_FORENSIC_AUDIT_OUT.mat'),'OUT');
end

function pos=find_bytes(hay,needle)
pos=[];
nh=numel(hay); nn=numel(needle);
if nn==0 || nh<nn, return; end
first=find(hay(1:nh-nn+1)==needle(1));
for k=1:numel(first)
    i=first(k);
    if all(hay(i:i+nn-1)==needle)
        pos(end+1)=i; %#ok<AGROW>
    end
end
end

function h=sha256_file(f)
fid=fopen(f,'rb'); assert(fid>=0,'cannot open %s',f);
c=onCleanup(@()fclose(fid));
b=fread(fid,Inf,'*uint8');
md=java.security.MessageDigest.getInstance('SHA-256');
md.update(b);
d=typecast(md.digest(),'uint8');
h=lower(reshape(dec2hex(d,2).',1,[]));
end
