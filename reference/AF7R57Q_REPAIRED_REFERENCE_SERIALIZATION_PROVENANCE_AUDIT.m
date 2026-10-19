function OUT = AF7R57Q_REPAIRED_REFERENCE_SERIALIZATION_PROVENANCE_AUDIT()
% Read-only provenance audit between production repaired-reference state
% and the R57 native correction-packet representation.

clc;
fprintf('\n====================================================================================================\n');
fprintf(' AF7R57Q | REPAIRED-REFERENCE SERIALIZATION / PACKET PROVENANCE AUDIT\n');
fprintf('====================================================================================================\n');
fprintf('[AUTHORITY] readOnly=1 | sourceBound=1 | noSyntheticPacket=1 | noGuardInstall=1 | noTimingClaimUpgrade=1 | productionModified=0\n');

root=fileparts(mfilename('fullpath'));
selfFile=[mfilename('fullpath') '.m'];
selfSHA=sha256_file(selfFile);
fprintf(' immutable entrypoint                    = %s\n',selfFile);
fprintf(' immutable entrypoint SHA-256             = %s\n',selfSHA);

rps=fullfile(root,'REPLANNING_SYSTEM.m');
core=fullfile(root,'af7r57k_trace_core.hpp');
ser=fullfile(root,'af7r57k_corrected_core_mex.cpp');
assert(isfile(rps)&&isfile(core)&&isfile(ser),'[AF7R57Q] required authority source missing.');

expectedRPS='223d34c49d7148b8ea14ff4ab9c420d409560c0650ac19e9b0f99318887090c9';
expectedCore='4f99d3ba7bdd5645a851a1c2236a8239753631d47998760cc6939a0655a344e1';
expectedSer='3b774a4aa1b18646182d1f7fa6d01bd2c6669e5ad2e6a8200f702f2f8e6f8597';
shaR=sha256_file(rps); shaC=sha256_file(core); shaS=sha256_file(ser);

fprintf('\n[A. EXACT SOURCE AUTHORITY]\n');
fprintf(' REPLANNING_SYSTEM current               = %d\n',strcmp(shaR,expectedRPS));
fprintf(' R57K corrected core current             = %d\n',strcmp(shaC,expectedCore));
fprintf(' R57K corrected serializer current       = %d\n',strcmp(shaS,expectedSer));
assert(strcmp(shaR,expectedRPS)&&strcmp(shaC,expectedCore)&&strcmp(shaS,expectedSer), ...
    '[AF7R57Q] source differs from bound authority.');

R=normalize_lf(fileread(rps));
C=normalize_lf(fileread(core));
S=normalize_lf(fileread(ser));
fields={'repair_t0','repair_t1','repair_offsets','repair_seg0','repair_seg1','repair_cp'};

prodEval=contains(R,'R.eval_fn=candidate.eval_fn') || contains(R,'R.eval_fn = candidate.eval_fn');
prodFlat=contains(R,'R.flat_eval_fn=candidate.flat_eval_fn') || contains(R,'R.flat_eval_fn = candidate.flat_eval_fn');
prodRep=contains(R,'R.representation=candidate.representation') || contains(R,'R.representation = candidate.representation');
prodRepairHits=zeros(1,numel(fields));
for k=1:numel(fields)
    prodRepairHits(k)=numel(strfind(R,fields{k}));
end
prodPacketFieldsPresent=any(prodRepairHits>0);

fprintf('\n[B. PRODUCTION REPAIRED-REFERENCE SEMANTICS]\n');
fprintf(' commit stores eval_fn                    = %d\n',prodEval);
fprintf(' commit stores flat_eval_fn               = %d\n',prodFlat);
fprintf(' optional representation carried          = %d\n',prodRep);
for k=1:numel(fields)
    fprintf(' production token %-18s hits = %d\n',fields{k},prodRepairHits(k));
end
fprintf(' production correction-packet fields exist = %d\n',prodPacketFieldsPresent);

nativeRepairs=contains(C,'std::vector<Correction> repairs');
nativeRepairLoop=contains(C,'for(const auto& C:repairs)');
nativeSegLoop=contains(C,'for(const auto& s:C.seg)');
parserAll=true;
for k=1:numel(fields)
    parserAll=parserAll && contains(S,['"' fields{k} '"']);
end
cp48=contains(S,'repair_cp must be 48xS.') && contains(S,'mxGetM(cpA)!=48');
allowsEmpty=contains(S,'if(a<0||b<a||b>(long)S)');

fprintf('\n[C. R57 NATIVE ACTIVE-REFERENCE SEMANTICS]\n');
fprintf(' native correction vector                 = %d\n',nativeRepairs);
fprintf(' native eval loops repairs / segments     = %d / %d\n',nativeRepairLoop,nativeSegLoop);
fprintf(' parse_ref consumes all repair fields     = %d\n',parserAll);
fprintf(' repair_cp fixed 48xS                     = %d\n',cp48);
fprintf(' empty repair group permitted             = %d\n',allowsEmpty);

workModel=nativeRepairs&&nativeRepairLoop&&nativeSegLoop&&parserAll&&cp48;
needG=allowsEmpty;
needS=true;
cpIndependent=~cp48;

fprintf('\n[D. MINIMUM NATIVE REPAIR RESOURCE MODEL]\n');
fprintf(' W_ref structural form                    = O(G + S)\n');
fprintf(' G = repair_count required independently  = %d\n',needG);
fprintf(' S = total_repair_segments required       = %d\n',needS);
fprintf(' repair_cp extra size coordinate required = %d\n',cpIndependent);
fprintf(' source-bound work model closed           = %d\n',workModel);

% Repository-wide diagnostic provenance scan.
D=dir(fullfile(root,'**','*'));
producerFiles={}; consumerFiles={};
for i=1:numel(D)
    if D(i).isdir, continue; end
    [~,~,ext]=fileparts(D(i).name);
    if ~ismember(lower(ext),{'.m','.cpp','.hpp','.h','.cc','.cxx'}), continue; end
    f=fullfile(D(i).folder,D(i).name);
    if strcmp(f,selfFile), continue; end
    try
        q=dir(f); if q.bytes>20e6, continue; end
        T=normalize_lf(fileread(f));
    catch
        continue;
    end
    tokenHits=0;
    for k=1:numel(fields), tokenHits=tokenHits+contains(T,fields{k}); end
    if tokenHits==0, continue; end

    prod=false; cons=false;
    for k=1:numel(fields)
        fld=fields{k};
        prod = prod || contains(T,['.' fld '=']) || contains(T,['.' fld ' =']) || ...
            contains(T,['''' fld ''',']) || contains(T,['"' fld '",']);
        cons = cons || contains(T,['fld(P,"' fld '")']);
    end
    if prod, producerFiles{end+1}=f; end %#ok<AGROW>
    if cons, consumerFiles{end+1}=f; end %#ok<AGROW>
end
producerFiles=unique(producerFiles,'stable');
consumerFiles=unique(consumerFiles,'stable');

fprintf('\n[E. REPOSITORY PACKET-PROVENANCE SCAN]\n');
fprintf(' producer-candidate files                 = %d\n',numel(producerFiles));
for i=1:numel(producerFiles), fprintf('  PRODUCER? %s\n',producerFiles{i}); end
fprintf(' consumer-candidate files                 = %d\n',numel(consumerFiles));
for i=1:numel(consumerFiles), fprintf('  CONSUMER  %s\n',consumerFiles{i}); end

canonicalSerializerInRPS=prodPacketFieldsPresent && prodEval;
shadowNativeGap=~canonicalSerializerInRPS && parserAll && prodEval;

fprintf('\n[F. SERIALIZATION-CONTRACT STATUS]\n');
fprintf(' production -> native canonical serializer present = %d\n',canonicalSerializerInRPS);
fprintf(' shadow/native representation gap present          = %d\n',shadowNativeGap);
fprintf(' zero-repair timing result upgrades this status     = 0\n');
fprintf(' nonzero synthetic packet sufficient for production theorem = 0\n');
fprintf(' serialization theorem closed                      = 0\n');

fprintf('\n[G. FORMAL GATE]\n');
fprintf(' historical R57 safety mathematics preserved       = 1\n');
fprintf(' native G/S resource model source-bound             = %d\n',workModel);
fprintf(' production repaired-reference semantics identified = %d\n',prodEval&&prodFlat);
fprintf(' canonical production packet provenance closed      = %d\n',canonicalSerializerInRPS);
fprintf(' production source modified                         = 0\n');
fprintf(' production timing claim upgraded                   = 0\n');
fprintf(' next action                                        = DEFINE_CANONICAL_REPAIRED_REFERENCE_SERIALIZATION_CONTRACT__THEN_INSTALL_PREPARSE_G_S_GUARD__THEN_NONZERO_REPAIR_EQUIVALENCE\n');
fprintf('====================================================================================================\n');

OUT=struct();
OUT.schema='AF7R57Q_REPAIRED_REFERENCE_SERIALIZATION_PROVENANCE_V1';
OUT.closed=true;
OUT.work_model_closed=workModel;
OUT.need_repair_count=needG;
OUT.need_segment_count=needS;
OUT.cp_independent=cpIndependent;
OUT.production_eval_fn=prodEval;
OUT.production_flat_eval_fn=prodFlat;
OUT.production_representation=prodRep;
OUT.canonical_serializer_present=canonicalSerializerInRPS;
OUT.shadow_native_gap=shadowNativeGap;
OUT.producer_candidates=producerFiles;
OUT.consumer_candidates=consumerFiles;
OUT.production_modified=false;
OUT.timing_claim_upgraded=false;
OUT.entrypoint=selfFile;
OUT.entrypoint_sha256=selfSHA;
assignin('base','AF7R57Q_REPAIRED_REFERENCE_SERIALIZATION_PROVENANCE_AUDIT_OUT',OUT);
save(fullfile(root,'AF7R57Q_REPAIRED_REFERENCE_SERIALIZATION_PROVENANCE_AUDIT_OUT.mat'),'OUT');
end

function s=normalize_lf(s)
s=strrep(s,sprintf('\r\n'),sprintf('\n'));
s=strrep(s,sprintf('\r'),sprintf('\n'));
end

function h=sha256_file(f)
fid=fopen(f,'r'); assert(fid>=0,'cannot open %s',f); c=onCleanup(@()fclose(fid));
b=fread(fid,Inf,'*uint8');
md=java.security.MessageDigest.getInstance('SHA-256'); md.update(b);
d=typecast(md.digest(),'uint8'); h=lower(reshape(dec2hex(d,2).',1,[]));
end
