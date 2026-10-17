function OUT = AF7R54A_COMMIT_THEOREM_AUDIT(replanner)
% Read-only theorem gate for the next timing architecture.
%
% It does NOT install another commit implementation.
% The purpose is to make explicit what must remain invariant before the
% MATLAB handle/batch-model commit path may be replaced.
%
% Current AF7R53 result showed that merely packing candidate data did not
% remove the hard-path tail.  The next admissible optimization is therefore
% an immutable packed correction-chain authority whose evaluator is used both
% by HLC reference output and repaired future-reference packets.
%
% This audit closes only the mathematical/architectural preconditions.

clc;
fprintf('\n====================================================================================================\n');
fprintf(' AF7R54A | COMMIT SAFETY THEOREM / ARCHITECTURE AUDIT\n');
fprintf('====================================================================================================\n');
fprintf('[AUTHORITY] readOnly=1 | productionModified=0 | noSafetyPredicateRemoved=1\n');

assert(isa(replanner,'REPLANNING_SYSTEM'));
src=which('REPLANNING_SYSTEM');
expected='223d34c49d7148b8ea14ff4ab9c420d409560c0650ac19e9b0f99318887090c9';
h=sha256_file(src);
assert(strcmp(h,expected), ...
    '[AF7R54A] exact AF7R50C required before theorem audit.');

raw=fileread(src);

% Source-level architectural facts that matter to a safe redesign.
C=struct();
C.active_eval_uses_repaired_eval_handle= ...
    contains(raw,'xd=R.eval_fn(double(t_now));');
C.repaired_packet_uses_batch_model= ...
    contains(raw,'af7r33_flat_jet_batch_model(obj,tq,R.batch_flat_model)');
C.repaired_packet_requires_batch_model= ...
    contains(raw,'repaired batch flat model unavailable');
C.reference_currentness_requires_eval_handle= ...
    contains(raw,'repaired reference eval_fn is unavailable');
C.reference_only_requires_flat_handle= ...
    contains(raw,'reference-only flat-jet authority unavailable');
C.trigger_currentness_speed= ...
    contains(raw,'obj.validate_active_speed_certificate');
C.trigger_currentness_sweep_version= ...
    contains(raw,'trigger sweep certificate is stale');
C.trigger_currentness_packet_version= ...
    contains(raw,'obstacle packet sweep certificate is stale');
C.strict_hard_deadline=contains(raw,'hard_modify_deadline_s<=4.0e-3');
C.continuous_wholebody=contains(raw,'MIDPOINT_STATIC_SUPPORT_MINUS_SPEED_TIMES_HALFWIDTH');
C.r9_stageB=contains(raw,'R9_LOCAL_VECTOR_COUPLED_ADAPTIVE_BERNSTEIN_FLATNESS_SPEED');

% Formal replacement contract.
% A future packed-chain implementation is admissible only if it preserves:
%
% (T1) For every derivative order r=0..6 and all t in the certified domain,
%      x_packed^(r)(t) = x_legacy^(r)(t)
%      as the same Bernstein polynomial / additive correction chain.
%
% (T2) The speed, sweep, sensor-horizon and obstacle-inflation certificate
%      values are version/dependency-bound to the same active reference.
%
% (T3) The active authority switch is atomic:
%      before switch -> old complete bundle;
%      after switch  -> new complete bundle;
%      no observable mixed-version state.
%
% (T4) Any packed future-reference evaluator used by sensing/replanning is
%      the SAME mathematical correction chain used by the HLC reference.
%
% (T5) The final hard deadline remains a reject condition, not a diagnostic.
%
% (T6) Existing continuous C7/R9/rejoin/whole-body certificates remain the
%      acceptance authority. Packed representation cannot create acceptance.
%
% Current AF7R52 already established zero numerical difference for one
% correction packet on the tested authority.  Multi-replan chain equivalence
% and atomic version switching remain open and must close before production.

fprintf('\n[A. CURRENT R50C EXECUTION DEPENDENCIES]\n');
F=fieldnames(C);
for k=1:numel(F)
    fprintf(' %-48s = %d\n',strrep(F{k},'_',' '),logical(C.(F{k})));
end

sourceClosed=all(structfun(@logical,C));

fprintf('\n[B. REQUIRED THEOREM CONTRACT]\n');
fprintf(' T1 same correction polynomial chain 0..6      = REQUIRED\n');
fprintf(' T2 same version-bound speed/sweep/sensor      = REQUIRED\n');
fprintf(' T3 atomic old-bundle/new-bundle visibility    = REQUIRED\n');
fprintf(' T4 HLC and future sensing share same chain    = REQUIRED\n');
fprintf(' T5 <=4 ms remains reject condition            = REQUIRED\n');
fprintf(' T6 existing continuous certificates authority = REQUIRED\n');

fprintf('\n[C. PRODUCTION GATE]\n');
fprintf(' current-source obligations identified          = %d\n',sourceClosed);
fprintf(' multi-replan packed-chain theorem closed        = 0\n');
fprintf(' atomic version-switch theorem closed            = 0\n');
fprintf(' production packed-chain install allowed         = 0\n');
fprintf(' next implementation                             = IMMUTABLE_NATIVE_CORRECTION_CHAIN_SHADOW\n');
fprintf('====================================================================================================\n');

OUT=struct();
OUT.schema='AF7R54A_COMMIT_THEOREM_AUDIT_V1';
OUT.source_obligations_closed=sourceClosed;
OUT.multi_replan_chain_closed=false;
OUT.atomic_switch_closed=false;
OUT.production_install_allowed=false;
OUT.next='IMMUTABLE_NATIVE_CORRECTION_CHAIN_SHADOW';
assignin('base','AF7R54A_COMMIT_THEOREM_AUDIT_OUT',OUT);
end

function h=sha256_file(fp)
fid=fopen(fp,'rb');assert(fid>=0);c=onCleanup(@()fclose(fid));
b=fread(fid,Inf,'*uint8');clear c;
md=java.security.MessageDigest.getInstance('SHA-256');md.update(b);
d=typecast(md.digest(),'uint8');h=lower(reshape(dec2hex(d,2).',1,[]));
end
