function OUT = AF7R57U_AF7R46_PACKET_PROVENANCE_AUDIT()
% AF7R57U_AF7R46_PACKET_PROVENANCE_AUDIT
% Read-only current-source audit of the actual default production proposal
% method identified by AF7R57T V7:
%   REPLANNING_SYSTEM.af7r46_single_core_proposal_compact
%
% Goal: expose the exact pkt construction path before any new guard or
% serialization claim is installed. No production modifications.

clc;
fprintf('\n====================================================================================================\n');
fprintf(' AF7R57U | AF7R46 COMPACT PROPOSAL -> NATIVE PACKET PROVENANCE AUDIT\n');
fprintf('====================================================================================================\n');
fprintf('[AUTHORITY] readOnly=1 | currentSourceOnly=1 | actualV7Symbol=1 | noGuardInstall=1 | productionModified=0\n');

root=fileparts(mfilename('fullpath'));
self=[mfilename('fullpath') '.m'];
fprintf(' immutable entrypoint                    = %s\n',self);
fprintf(' immutable entrypoint SHA-256             = %s\n',sha256_file(self));

rps=fullfile(root,'REPLANNING_SYSTEM.m');
a34=fullfile(root,'af7r34_make_core_packet.m');
prime=fullfile(root,'af7r36_runtime_prime.m');
oldprop=fullfile(root,'af7r36_single_core_proposal.m');
coremex=fullfile(root,'af7r36_single_core_mex.cpp');

assert(isfile(rps),'[AF7R57U] REPLANNING_SYSTEM.m missing.');
assert(isfile(a34),'[AF7R57U] af7r34_make_core_packet.m missing.');
assert(isfile(prime),'[AF7R57U] af7r36_runtime_prime.m missing.');

expR='223d34c49d7148b8ea14ff4ab9c420d409560c0650ac19e9b0f99318887090c9';
shaR=sha256_file(rps);

fprintf('\n[A. EXACT CURRENT SOURCE AUTHORITY]\n');
fprintf(' REPLANNING_SYSTEM SHA-256               = %s\n',shaR);
fprintf(' bound production source current         = %d\n',strcmp(shaR,expR));
fprintf(' af7r34_make_core_packet SHA-256         = %s\n',sha256_file(a34));
fprintf(' af7r36_runtime_prime SHA-256            = %s\n',sha256_file(prime));
fprintf(' old af7r36_single_core_proposal present = %d\n',isfile(oldprop));
if isfile(oldprop)
    fprintf(' old proposal SHA-256                    = %s\n',sha256_file(oldprop));
end
fprintf(' af7r36_single_core_mex.cpp present      = %d\n',isfile(coremex));
if isfile(coremex)
    fprintf(' af7r36_single_core_mex.cpp SHA-256      = %s\n',sha256_file(coremex));
end
assert(strcmp(shaR,expR),'[AF7R57U] bound production source changed.');

src=fileread(rps);
L=splitlines(string(src));
trimmed=strtrim(L);

decl='function P = af7r46_single_core_proposal_compact(obj,CTX)';
idx=find(trimmed==string(decl),1,'first');
fprintf('\n[B. V7-BOUND ACTUAL DEFAULT PROPOSAL METHOD]\n');
fprintf(' exact declaration found                = %d\n',~isempty(idx));
assert(~isempty(idx),'[AF7R57U] V7-bound af7r46 compact method declaration missing.');

next=[];
for k=idx+1:numel(trimmed)
    q=char(trimmed(k));
    if startsWith(q,'function ')
        next=k; break;
    end
end
if isempty(next), next=numel(L)+1; end
bodyLines=idx:(next-1);
B=strjoin(L(bodyLines),newline);
fprintf(' method lines                            = [%d %d]\n',idx,next-1);
fprintf('\n[C. EXACT af7r46_single_core_proposal_compact BODY]\n');
for k=bodyLines
    fprintf(' L%-6d %s\n',k,char(L(k)));
end

% Token-level provenance. These are diagnostics, not theorem upgrades.
toks={ ...
    'af7r34_make_core_packet', ...
    'nominal_batch_model_cache', ...
    'CTX', ...
    'pkt=', ...
    'pkt =', ...
    'af7r36_single_core_mex(1,pkt)', ...
    'repair_t0', ...
    'repair_t1', ...
    'repair_offsets', ...
    'repair_seg0', ...
    'repair_seg1', ...
    'repair_cp', ...
    'BEZIER_D15_C7', ...
    'reference_version', ...
    'backend'};

fprintf('\n[D. COMPACT-METHOD PACKET / IDENTITY TOKENS]\n');
for k=1:numel(toks)
    fprintf(' %-36s = %d\n',toks{k},contains(B,toks{k}));
end

% Extract only lines mentioning pkt/RAW/model/cache/repair/backend/version.
keys={'pkt','RAW','model','cache','repair_','backend','reference_version','af7r34','af7r36'};
sel=false(numel(bodyLines),1);
for ii=1:numel(bodyLines)
    q=char(L(bodyLines(ii)));
    for jj=1:numel(keys)
        if contains(q,keys{jj})
            sel(ii)=true; break;
        end
    end
end
fprintf('\n[E. PACKET-PROVENANCE SOURCE LINES]\n');
ids=find(sel);
fprintf(' relevant line count                    = %d\n',numel(ids));
for jj=1:numel(ids)
    k=bodyLines(ids(jj));
    fprintf(' L%-6d %s\n',k,char(L(k)));
end

% af7r34 exact producer body is printed in full because R57P/Q established
% that this is the only native repair-field producer candidate in the chain.
A=fileread(a34);
AL=splitlines(string(A));
fprintf('\n[F. EXACT af7r34_make_core_packet SOURCE]\n');
for k=1:numel(AL)
    fprintf(' A34-L%-5d %s\n',k,char(AL(k)));
end

repairFields={'repair_t0','repair_t1','repair_offsets','repair_seg0','repair_seg1','repair_cp'};
fprintf('\n[G. af7r34 REPAIR-FIELD PRODUCER STATUS]\n');
allRepair=true;
for k=1:numel(repairFields)
    hit=contains(A,repairFields{k});
    allRepair=allRepair&&hit;
    fprintf(' %-20s = %d\n',repairFields{k},hit);
end
fprintf(' complete native repair field set        = %d\n',allRepair);

% Current production path classification is deliberately narrow.
compactCallsA34=contains(B,'af7r34_make_core_packet');
compactCallsMex=contains(B,'af7r36_single_core_mex(1,pkt)');
compactMentionsRepair=false;
for k=1:numel(repairFields)
    compactMentionsRepair=compactMentionsRepair||contains(B,repairFields{k});
end

fprintf('\n[H. CURRENT SERIALIZATION-PATH GATE]\n');
fprintf(' compact method directly calls af7r34    = %d\n',compactCallsA34);
fprintf(' compact method calls native MEX packet  = %d\n',compactCallsMex);
fprintf(' compact method directly names repairs   = %d\n',compactMentionsRepair);
fprintf(' exact packet provenance body captured   = 1\n');
fprintf(' canonical serialization theorem closed  = 0\n');
fprintf(' G/S guard installed                     = 0\n');
fprintf(' production source modified              = 0\n');
fprintf(' timing claim upgraded                   = 0\n');
fprintf(' next action                              = DERIVE_AF7R46_PKT_CONSTRUCTION_IDENTITY_FROM_PRINTED_BODY__THEN_TEST_NONZERO_REPAIR_PATH__THEN_G_S_PREPARSE_GUARD\n');
fprintf('====================================================================================================\n');

OUT=struct();
OUT.schema='AF7R57U_AF7R46_PACKET_PROVENANCE_V1';
OUT.method_lines=[idx next-1];
OUT.compact_calls_af7r34=compactCallsA34;
OUT.compact_calls_native_mex=compactCallsMex;
OUT.compact_mentions_repair_fields=compactMentionsRepair;
OUT.af7r34_complete_repair_fields=allRepair;
OUT.canonical_serialization_closed=false;
OUT.production_modified=false;
OUT.entrypoint=self;
OUT.entrypoint_sha256=sha256_file(self);
assignin('base','AF7R57U_AF7R46_PACKET_PROVENANCE_AUDIT_OUT',OUT);
save(fullfile(root,'AF7R57U_AF7R46_PACKET_PROVENANCE_AUDIT_OUT.mat'),'OUT');
end

function h=sha256_file(f)
fid=fopen(f,'r'); assert(fid>=0,'cannot open %s',f);
c=onCleanup(@()fclose(fid));
b=fread(fid,Inf,'*uint8');
md=java.security.MessageDigest.getInstance('SHA-256');
md.update(b);
d=typecast(md.digest(),'uint8');
h=lower(reshape(dec2hex(d,2).',1,[]));
end
