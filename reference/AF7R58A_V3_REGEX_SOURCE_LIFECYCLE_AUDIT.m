function OUT = AF7R58A_V3_REGEX_SOURCE_LIFECYCLE_AUDIT()
% AF7R58A_V3_REGEX_SOURCE_LIFECYCLE_AUDIT
% =========================================================================
% Immutable read-only correction of AF7R58A V1/V2 audit implementation.
%
% PURPOSE
%   Close only source/theorem facts already established by the current
%   AF7R36 / REPLANNING_SYSTEM / AF7R34 / AF7R33 sources:
%
%   (A) op=1 already transports PlanResult through result_struct.
%   (B) PlanResult has ordinary success=false / reason state.
%   (C) committed repaired reference stores an AF7R33 batch_flat_model.
%   (D) native ActiveReference correction support is exactly open (t0,t1).
%   (E) therefore every correction with t1 <= t_now is EXACTLY removable
%       for every future evaluation t >= t_now.
%
% IMPORTANT
%   - Regex is whitespace/newline independent.
%   - Source windows are printed so semantic matches are directly auditable.
%   - No production source or MEX is modified.
%   - No production Gmax/Smax is guessed.
%   - No sampled safety acceptance is used.
%   - No terminal assert hides the forensic result; OUT.closed is authority.
% =========================================================================

clc;
fprintf('\n====================================================================================================\n');
fprintf(' AF7R58A V3 | REGEX SOURCE-BOUND ORDINARY RETURN + REPAIR-LIFECYCLE AUDIT\n');
fprintf('====================================================================================================\n');
fprintf('[AUTHORITY] readOnly=1 | regexSourceBinding=1 | whitespaceIndependent=1 | noCapGuess=1 | productionModified=0\n');

root=fileparts(mfilename('fullpath'));
self=[mfilename('fullpath') '.m'];

mexSrc=fullfile(root,'af7r36_single_core_mex.cpp');
rps=fullfile(root,'REPLANNING_SYSTEM.m');
a34=fullfile(root,'af7r34_make_core_packet.m');
a33=fullfile(root,'af7r33_make_batch_model_from_proposal.m');

assert(isfile(mexSrc),'[AF7R58A V3] af7r36_single_core_mex.cpp missing.');
assert(isfile(rps),'[AF7R58A V3] REPLANNING_SYSTEM.m missing.');
assert(isfile(a34),'[AF7R58A V3] af7r34_make_core_packet.m missing.');
assert(isfile(a33),'[AF7R58A V3] af7r33_make_batch_model_from_proposal.m missing.');

fprintf(' entrypoint                               = %s\n',self);
fprintf(' entrypoint SHA-256                        = %s\n',sha256_file(self));

shaM=sha256_file(mexSrc);
shaR=sha256_file(rps);
sha34=sha256_file(a34);
sha33=sha256_file(a33);

expM='4f9778723aa3aa339d5167b90845fcfcaa8564cae68326ff1e4ed504c7e6daea';
expR='223d34c49d7148b8ea14ff4ab9c420d409560c0650ac19e9b0f99318887090c9';
exp34='ce1f98dac7b49ec8510c09d5c89b7edd4aea64a6c6320e7d2752431528cf22f8';
exp33='f0e9f487cd270430d191871366c84595be586b6da0fb7b93eb98f4dfc25b46fd';

current=[strcmp(shaM,expM),strcmp(shaR,expR),strcmp(sha34,exp34),strcmp(sha33,exp33)];

fprintf('\n[A. EXACT CURRENT SOURCE BINDING]\n');
fprintf(' AF7R36 current                          = %d | %s\n',current(1),shaM);
fprintf(' REPLANNING_SYSTEM current               = %d | %s\n',current(2),shaR);
fprintf(' AF7R34 current                          = %d | %s\n',current(3),sha34);
fprintf(' AF7R33 current                          = %d | %s\n',current(4),sha33);
fprintf(' all current                              = %d\n',all(current));
fprintf(' production source modified               = 0\n');

M=fileread(mexSrc);
R=fileread(rps);
A34=fileread(a34);
A33=fileread(a33);

% -------------------------------------------------------------------------
% B. PlanResult / ordinary op=1 transport.
% -------------------------------------------------------------------------
patPlan='struct\s+PlanResult\s*\{[\s\S]*?\};';
planBlock=regex_extract(M,patPlan);
hasPlan=~isempty(planBlock);
hasSuccess=regex_hit(planBlock,'bool\s+success\s*=\s*false\s*;');
hasReason=regex_hit(planBlock,'int\s+reason\s*=\s*0\s*;');
hasResultStruct=regex_hit(M,'result_struct\s*\(\s*const\s+PlanResult\s*&');
hasOp1=regex_hit(M,'if\s*\(\s*op\s*==\s*1\s*\)\s*\{');
hasOp1Serializer=regex_hit(M,'result_struct\s*\(\s*Q\s*,\s*t0\s*,\s*tm\s*,\s*t1\s*,\s*O\s*\)');
transportBasis=hasPlan&&hasSuccess&&hasReason&&hasResultStruct&&hasOp1&&hasOp1Serializer;

fprintf('\n[B. OP=1 ORDINARY STRUCTURED-RETURN BASIS]\n');
fprintf(' PlanResult block found                  = %d\n',hasPlan);
fprintf(' PlanResult success=false                = %d\n',hasSuccess);
fprintf(' PlanResult reason=0                     = %d\n',hasReason);
fprintf(' result_struct(const PlanResult&,...)     = %d\n',hasResultStruct);
fprintf(' op=1 branch                             = %d\n',hasOp1);
fprintf(' op=1 serializes Q through result_struct = %d\n',hasOp1Serializer);
fprintf(' ordinary-failure transport basis closed = %d\n',transportBasis);
fprintf(' ordinary G/S return installed production = 0\n');
fprintf(' reject-only transport theorem            = %d\n',transportBasis);
fprintf(' safety inequality modified               = 0\n');

fprintf('\n[B2. SOURCE WINDOWS]\n');
print_regex_context(M,'struct\s+PlanResult\s*\{',3,8,'PlanResult');
print_regex_context(M,'if\s*\(\s*op\s*==\s*1\s*\)\s*\{',2,9,'op==1');

% -------------------------------------------------------------------------
% C. Current repaired-reference lifecycle.
% -------------------------------------------------------------------------
commitReplaces=regex_hit(R,'obj\s*\.\s*active_repaired_reference\s*=\s*R\s*;');
versionAdvances=regex_hit(R,'(?:newVersion|targetVersion)\s*=\s*double\s*\(\s*obj\s*\.\s*active_reference_version\s*\)\s*\+\s*1\s*;');
storesBatch=regex_hit(R,'R\s*\.\s*batch_flat_model\s*=\s*af7r33_make_batch_model_from_proposal\s*\(');
a34ReadsBatch=regex_hit(A34,'R\s*=\s*replanner\s*\.\s*active_repaired_reference\s*;') && ...
              regex_hit(A34,'model\s*=\s*R\s*\.\s*batch_flat_model\s*;');
a34CountsG=regex_hit(A34,'G\s*=\s*numel\s*\(\s*model\s*\.\s*corrections\s*\)\s*;');
a34CountsS=regex_hit(A34,'S\s*=\s*S\s*\+\s*numel\s*\(\s*model\s*\.\s*corrections\s*\(\s*g\s*\)\s*\.\s*control_points\s*\)\s*;');
lifecycleClosed=commitReplaces&&versionAdvances&&storesBatch&&a34ReadsBatch&&a34CountsG&&a34CountsS;

fprintf('\n[C. CURRENT REPAIRED-REFERENCE LIFECYCLE]\n');
fprintf(' active repaired object replaced at commit= %d\n',commitReplaces);
fprintf(' reference version advances               = %d\n',versionAdvances);
fprintf(' committed reference stores batch model   = %d\n',storesBatch);
fprintf(' AF7R34 retrieves active batch model       = %d\n',a34ReadsBatch);
fprintf(' AF7R34 G=#corrections                     = %d\n',a34CountsG);
fprintf(' AF7R34 S=sum(segment counts)              = %d\n',a34CountsS);
fprintf(' lifecycle relation closed                = %d\n',lifecycleClosed);
fprintf(' object replacement implies G=1           = 0\n');

fprintf('\n[C2. SOURCE WINDOWS]\n');
print_regex_context(R,'R\s*\.\s*batch_flat_model\s*=\s*af7r33_make_batch_model_from_proposal',3,5,'batch_flat_model commit');
print_regex_context(A34,'G\s*=\s*numel\s*\(\s*model\s*\.\s*corrections',2,12,'AF7R34 G/S serialization');

% -------------------------------------------------------------------------
% D. Exact expired correction pruning theorem.
% -------------------------------------------------------------------------
patSupport='if\s*\(\s*!\s*\(\s*t\s*>\s*C\s*\.\s*t0\s*&&\s*t\s*<\s*C\s*\.\s*t1\s*\)\s*\)\s*continue\s*;';
strictSupport=regex_hit(M,patSupport);

% Mathematical authority (no sampled acceptance):
%   C.t1 <= t_now and t >= t_now => t >= C.t1.
%   Native activation requires simultaneously t < C.t1.
%   Therefore activation is false for every future t >= t_now.
orderProof=true;
pruneTheorem=strictSupport&&orderProof;

fprintf('\n[D. EXACT EXPIRED-CORRECTION PRUNING THEOREM]\n');
fprintf(' native strict support (t0,t1) found     = %d\n',strictSupport);
fprintf(' order-transitivity implication           = %d\n',orderProof);
fprintf(' boundary t=t1 excluded                   = %d\n',strictSupport);
fprintf(' exact prune for ALL t>=t_now             = %d\n',pruneTheorem);
fprintf(' sampled theorem acceptance               = 0\n');
fprintf(' approximation/resampling                 = 0\n');

fprintf('\n[D2. SOURCE WINDOW]\n');
print_regex_context(M,patSupport,3,8,'ActiveReference strict repair window');

% -------------------------------------------------------------------------
% E. Current AF7R33 future-chain resource contract.
% -------------------------------------------------------------------------
L33=lower(A33);
tokCorrections=contains(L33,'corrections');
tokAppend=regex_hit(A33,'corrections\s*\(\s*end\s*\+\s*1') || ...
          regex_hit(A33,'\[\s*[^\]]*corrections[^\]]*\]');
tokPrune=contains(L33,'prune') || contains(L33,'expired');
tokCompact=contains(L33,'compact') || contains(L33,'coalesce') || contains(L33,'merge_correction');
tokCapG=contains(L33,'max_repair_groups') || contains(A33,'Gmax') || contains(A33,'MAX_REPAIR_GROUP');
tokCapS=contains(L33,'max_repair_segments') || contains(A33,'Smax') || contains(A33,'MAX_REPAIR_SEGMENT');
capDerivable=tokCapG&&tokCapS;

fprintf('\n[E. AF7R33 CORRECTION-CHAIN RESOURCE CONTRACT]\n');
fprintf(' correction-chain source present         = %d\n',tokCorrections);
fprintf(' append/carry syntax detected             = %d\n',tokAppend);
fprintf(' explicit expired-prune policy            = %d\n',tokPrune);
fprintf(' explicit exact compaction policy         = %d\n',tokCompact);
fprintf(' explicit finite G cap                    = %d\n',tokCapG);
fprintf(' explicit finite S cap                    = %d\n',tokCapS);
fprintf(' production Gmax/Smax derivable now       = %d\n',capDerivable);
fprintf(' current observed G/S promoted to cap     = 0\n');
fprintf(' audit-only 4/8 promoted to cap           = 0\n');

fprintf('\n[E2. AF7R33 LIFECYCLE/RESOURCE SOURCE LINES]\n');
print_lines_with_tokens(A33,{'correction','control_points','prune','expired','compact','coalesce','max_repair','Gmax','Smax'});

% Exact pruning removes dead history but does not bound simultaneous future
% active corrections.  Therefore finite cap theorem remains open unless the
% future chain itself has an explicit finite contract.
fprintf('\n[F. FORMAL RESOURCE STATUS]\n');
fprintf(' exact dead-history removal theorem       = %d\n',pruneTheorem);
fprintf(' simultaneous future-chain finite bound   = %d\n',capDerivable);
fprintf(' pruning alone proves finite G/S cap      = 0\n');
fprintf(' production finite-chain theorem closed   = %d\n',capDerivable);

basisClosed=all(current)&&transportBasis&&lifecycleClosed&&pruneTheorem;

fprintf('\n[G. FINAL GATE]\n');
fprintf(' exact current source binding             = %d\n',all(current));
fprintf(' ordinary op1 transport basis             = %d\n',transportBasis);
fprintf(' repaired lifecycle relation              = %d\n',lifecycleClosed);
fprintf(' expired-correction pruning theorem       = %d\n',pruneTheorem);
fprintf(' production finite G/S cap theorem        = %d\n',capDerivable);
fprintf(' production caps selected                 = 0\n');
fprintf(' production source modified               = 0\n');
fprintf(' production 4 ms theorem claimed          = 0\n');
fprintf(' AF7R58A V3 SOURCE/THEOREM BASIS GATE     = %d\n',basisClosed);

if capDerivable
    next='INSTALL_ORDINARY_OP1_GS_RETURN_WITH_SOURCE_DERIVED_CAPS__THEN_END_TO_END_QPC';
else
    next=['INSTALL_EXACT_EXPIRED_PRUNE__DEFINE_EXPLICIT_FUTURE_ACTIVE_CHAIN_CONTRACT__' ...
          'DERIVE_GMAX_SMAX__INSTALL_ORDINARY_OP1_RETURN__THEN_END_TO_END_QPC'];
end
fprintf(' next action                               = %s\n',next);
fprintf('====================================================================================================\n');

OUT=struct();
OUT.schema='AF7R58A_V3_REGEX_SOURCE_LIFECYCLE_AUDIT';
OUT.source_current=all(current);
OUT.transport_basis_closed=transportBasis;
OUT.lifecycle_relation_closed=lifecycleClosed;
OUT.expired_pruning_theorem_closed=pruneTheorem;
OUT.production_caps_derivable=capDerivable;
OUT.production_caps_selected=false;
OUT.production_modified=false;
OUT.production_4ms_claimed=false;
OUT.closed=basisClosed;
OUT.next_action=next;
OUT.entrypoint=self;
OUT.entrypoint_sha256=sha256_file(self);
assignin('base','AF7R58A_V3_REGEX_SOURCE_LIFECYCLE_AUDIT_OUT',OUT);
save(fullfile(root,'AF7R58A_V3_REGEX_SOURCE_LIFECYCLE_AUDIT_OUT.mat'),'OUT');
end

function tf=regex_hit(txt,pat)
tf=~isempty(regexp(txt,pat,'once'));
end

function s=regex_extract(txt,pat)
[a,b]=regexp(txt,pat,'start','end','once');
if isempty(a), s=''; else, s=txt(a:b); end
end

function print_regex_context(txt,pat,nBefore,nAfter,label)
[a,~]=regexp(txt,pat,'start','end','once');
fprintf(' ---- %s ----\n',label);
if isempty(a)
    fprintf(' NO MATCH\n');
    return;
end
starts=[1,regexp(txt,'\n','end')+1];
line=find(starts<=a,1,'last');
L=regexp(txt,'\r\n|\n|\r','split');
i0=max(1,line-nBefore); i1=min(numel(L),line+nAfter);
for i=i0:i1
    fprintf(' L%-5d %s\n',i,L{i});
end
end

function print_lines_with_tokens(txt,toks)
L=regexp(txt,'\r\n|\n|\r','split');
n=0;
for i=1:numel(L)
    hit=false;
    for k=1:numel(toks)
        if contains(L{i},toks{k},'IgnoreCase',true)
            hit=true; break;
        end
    end
    if hit
        n=n+1;
        fprintf(' L%-5d %s\n',i,strtrim(L{i}));
    end
end
fprintf(' matched lines                            = %d\n',n);
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
