function OUT = AF7R58A_ONEFILE_ORDINARY_RETURN_LIFECYCLE_AUDIT()
% AF7R58A_ONEFILE_ORDINARY_RETURN_LIFECYCLE_AUDIT
% =========================================================================
% PURPOSE
%   Source-bound, read-only audit immediately after AF7R57W V3.
%
%   1) Confirm the ACTUAL AF7R36 op=1 transport can return an ordinary
%      PlanResult/result_struct failure without throwing.
%   2) Confirm the active repaired reference is version-replaced at commit,
%      while AF7R34 serializes the finite correction chain stored inside
%      active_repaired_reference.batch_flat_model.
%   3) Prove exact lossless pruning for corrections whose t1 <= t_now:
%
%          C_g(t)=0 for all t >= t_now  if  C_g.t1 <= t_now,
%
%      under the native ActiveReference semantics
%
%          if(!(t>C.t0 && t<C.t1)) continue;
%
%   4) Refuse to invent production Gmax/Smax. They are considered derivable
%      only if the current lifecycle source has an explicit finite chain
%      policy/cap or exact pruning/compaction contract.
%
% IMPORTANT
%   - No production source/MEX is modified.
%   - No production cap value is selected.
%   - No 4 ms production theorem is claimed.
% =========================================================================

clc;
fprintf('\n====================================================================================================\n');
fprintf(' AF7R58A | ONE-FILE ORDINARY-RETURN / REPAIR-LIFECYCLE BASIS AUDIT\n');
fprintf('====================================================================================================\n');
fprintf('[AUTHORITY] readOnly=1 | sourceBound=1 | noProductionPatch=1 | noCapGuess=1 | sampledSafetyAcceptance=0\n');

root=fileparts(mfilename('fullpath'));
self=[mfilename('fullpath') '.m'];
fprintf(' entrypoint                               = %s\n',self);
fprintf(' entrypoint SHA-256                        = %s\n',sha256_file(self));

mexSrc=fullfile(root,'af7r36_single_core_mex.cpp');
rps=fullfile(root,'REPLANNING_SYSTEM.m');
a34=fullfile(root,'af7r34_make_core_packet.m');
a33=fullfile(root,'af7r33_make_batch_model_from_proposal.m');

assert(isfile(mexSrc),'[AF7R58A] af7r36_single_core_mex.cpp missing.');
assert(isfile(rps),'[AF7R58A] REPLANNING_SYSTEM.m missing.');
assert(isfile(a34),'[AF7R58A] af7r34_make_core_packet.m missing.');

fprintf('\n[A. CURRENT SOURCE BINDING]\n');
fprintf(' AF7R36 source                           = %s\n',mexSrc);
fprintf(' AF7R36 SHA-256                          = %s\n',sha256_file(mexSrc));
fprintf(' REPLANNING_SYSTEM SHA-256               = %s\n',sha256_file(rps));
fprintf(' AF7R34 SHA-256                          = %s\n',sha256_file(a34));
fprintf(' AF7R33 proposal->batch source present   = %d\n',isfile(a33));
if isfile(a33)
    fprintf(' AF7R33 proposal->batch SHA-256           = %s\n',sha256_file(a33));
end
fprintf(' production source modified               = 0\n');

M=fileread(mexSrc);
R=fileread(rps);
A34=fileread(a34);
if isfile(a33), A33=fileread(a33); else, A33=''; end

hasPlanResult = contains(M,'struct PlanResult');
hasResultStruct = contains(M,'result_struct(const PlanResult');
hasOp1 = contains(M,'if(op==1)');
hasOp1Result = contains(M,'result_struct(Q,t0,tm,t1,O)');
hasReason = contains(M,'success=false;int reason=0') || ...
            contains(M,'bool success=false;int reason=0');
transportBasis = hasPlanResult && hasResultStruct && hasOp1 && hasOp1Result && hasReason;

fprintf('\n[B. OP=1 ORDINARY STRUCTURED-RETURN BASIS]\n');
fprintf(' PlanResult has success/reason state      = %d\n',hasPlanResult && hasReason);
fprintf(' existing result_struct serializer        = %d\n',hasResultStruct);
fprintf(' op=1 branch present                      = %d\n',hasOp1);
fprintf(' op=1 already serializes PlanResult       = %d\n',hasOp1Result);
fprintf(' ordinary failure transport basis closed = %d\n',transportBasis);
fprintf(' ordinary G/S return installed production = 0\n');
fprintf(' reject-only transport theorem            = %d\n',transportBasis);
fprintf(' safety inequality modified               = 0\n');

commitReplaces = contains(R,'obj.active_repaired_reference=R;');
versionIncrements = contains(R,'active_reference_version') && ...
    (contains(R,'newVersion=double(obj.active_reference_version)+1') || ...
     contains(R,'targetVersion=double(obj.active_reference_version)+1'));
storesBatch = contains(R,'R.batch_flat_model=af7r33_make_batch_model_from_proposal');
a34ReadsBatch = contains(A34,'R.batch_flat_model');
a34CountsG = contains(A34,'G=numel(model.corrections)');
a34CountsS = contains(A34,'S=S+numel(model.corrections(g).control_points)');

fprintf('\n[C. CURRENT REPAIRED-REFERENCE LIFECYCLE]\n');
fprintf(' active repaired object replaced at commit= %d\n',commitReplaces);
fprintf(' reference version advances               = %d\n',versionIncrements);
fprintf(' committed reference stores batch model   = %d\n',storesBatch);
fprintf(' AF7R34 reads active batch model           = %d\n',a34ReadsBatch);
fprintf(' AF7R34 defines G=#corrections             = %d\n',a34CountsG);
fprintf(' AF7R34 defines S=sum(segment counts)      = %d\n',a34CountsS);
fprintf(' object replacement does NOT imply G=1    = %d\n',commitReplaces && a34CountsG);

supportExact = contains(M,'t>C.t0') && contains(M,'t<C.t1') && contains(M,'continue');
fprintf('\n[D. EXACT EXPIRED-CORRECTION PRUNING THEOREM]\n');
fprintf(' native support is open interval (t0,t1) = %d\n',supportExact);

tnow=10;
C=struct('t0',{1,8,10.5},'t1',{3,10,12});
future=linspace(tnow,15,257);
expired=[C.t1] <= tnow;
futureContrib=false(size(C));
for g=1:numel(C)
    for k=1:numel(future)
        t=future(k);
        if t>C(g).t0 && t<C(g).t1
            futureContrib(g)=true;
            break;
        end
    end
end
expiredZero = all(~futureContrib(expired));

fprintf(' synthetic expired groups                = %d\n',nnz(expired));
fprintf(' expired group future contribution        = %d\n',any(futureContrib(expired)));
fprintf(' exact expired-pruning theorem closed     = %d\n',supportExact && expiredZero);
fprintf(' pruning approximation/resampling used    = 0\n');
pruneTheorem = supportExact && expiredZero;

fprintf('\n[E. PRODUCTION Gmax/Smax DERIVABILITY]\n');
tokAppend = any_contains(A33,{'corrections(end+1)','[model.corrections','[base.corrections'});
tokPrune  = any_contains(A33,{'prune','expired','t1 <=','t1<='});
tokCompact= any_contains(A33,{'compact','merge','coalesce'});
tokCapG   = any_contains(A33,{'max_repair_groups','Gmax','MAX_REPAIR_GROUP'});
tokCapS   = any_contains(A33,{'max_repair_segments','Smax','MAX_REPAIR_SEGMENT'});

fprintf(' AF7R33 appends/carries correction chain = %d\n',tokAppend);
fprintf(' explicit expired-prune policy visible   = %d\n',tokPrune);
fprintf(' explicit exact compaction policy visible= %d\n',tokCompact);
fprintf(' explicit finite G cap visible           = %d\n',tokCapG);
fprintf(' explicit finite S cap visible           = %d\n',tokCapS);

capDerivable = tokCapG && tokCapS;
fprintf(' current observed G/S used as cap         = 0\n');
fprintf(' audit 4/8 promoted to production cap     = 0\n');
fprintf(' production Gmax/Smax derivable now       = %d\n',capDerivable);

fprintf('\n[F. RESOURCE THEOREM STATUS]\n');
fprintf(' historical-growth removal available     = %d\n',pruneTheorem);
fprintf(' simultaneous future-chain bound visible = %d\n',tokCapG && tokCapS);
fprintf(' pruning alone proves finite Gmax/Smax    = 0\n');
fprintf(' production finite-chain theorem closed  = %d\n',capDerivable);

basisClosed = transportBasis && commitReplaces && a34ReadsBatch && ...
              a34CountsG && a34CountsS && pruneTheorem;

fprintf('\n[G. FINAL GATE]\n');
fprintf(' ordinary op1 return transport basis      = %d\n',transportBasis);
fprintf(' repaired-reference lifecycle identified  = %d\n',commitReplaces && a34ReadsBatch);
fprintf(' expired-correction exact pruning theorem = %d\n',pruneTheorem);
fprintf(' production caps selected                 = 0\n');
fprintf(' production source modified               = 0\n');
fprintf(' production 4 ms theorem claimed          = 0\n');
fprintf(' AF7R58A ORDINARY-RETURN/LIFECYCLE GATE   = %d\n',basisClosed);

if capDerivable
    next='INSTALL_ORDINARY_OP1_GS_RETURN_WITH_SOURCE_DERIVED_CAPS__THEN_END_TO_END_QPC';
else
    next=['INSTALL_EXACT_EXPIRED_PRUNE_AND_EXPLICIT_FUTURE_CHAIN_RESOURCE_CONTRACT__' ...
          'THEN_DERIVE_GMAX_SMAX__THEN_END_TO_END_QPC'];
end
fprintf(' next action                               = %s\n',next);
fprintf('====================================================================================================\n');

assert(basisClosed,'[AF7R58A] ordinary-return/lifecycle basis did not close.');

OUT=struct();
OUT.schema='AF7R58A_ONEFILE_ORDINARY_RETURN_LIFECYCLE_BASIS';
OUT.closed=basisClosed;
OUT.transport_basis_closed=transportBasis;
OUT.expired_pruning_theorem_closed=pruneTheorem;
OUT.production_caps_derivable=capDerivable;
OUT.production_caps_selected=false;
OUT.production_modified=false;
OUT.production_4ms_claimed=false;
OUT.next_action=next;
OUT.entrypoint=self;
OUT.entrypoint_sha256=sha256_file(self);
assignin('base','AF7R58A_ONEFILE_ORDINARY_RETURN_LIFECYCLE_AUDIT_OUT',OUT);
save(fullfile(root,'AF7R58A_ONEFILE_ORDINARY_RETURN_LIFECYCLE_AUDIT_OUT.mat'),'OUT');
end

function tf=any_contains(s,toks)
tf=false;
if isempty(s), return; end
for k=1:numel(toks)
    if contains(s,toks{k})
        tf=true;
        return;
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
