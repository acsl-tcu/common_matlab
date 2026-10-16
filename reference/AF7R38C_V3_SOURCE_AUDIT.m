function OUT = AF7R38C_V3_SOURCE_AUDIT()
clc;
fprintf('\n====================================================================================================\n');
fprintf(' AF7R38C V3 | R38 + ROBUST 10 s ATTRIBUTION SOURCE AUDIT\n');
fprintf('====================================================================================================\n');

src=which('REPLANNING_SYSTEM');
assert(~isempty(src)&&isfile(src), ...
    '[AF7R38C] REPLANNING_SYSTEM unresolved. Clear/reload class first.');
raw=fileread(src);
compact=regexprep(raw,'\s+','');
needle=[char(39) 'sampled_acceptance' char(39) ',true'];

C=struct();
C.r14d=contains(raw,'AF7R37-R14D-PRODUCTION');
C.r38=contains(raw,'AF7R38-PREFLIGHT-POLICY-HOTPATH');
C.attr_v3=contains(raw,'AF7R38V3-10S-ATTRIBUTION');
C.observer_prime=contains(raw,'observer_preflight_prime_done');
C.observer_restore=contains(raw,'obj.observer_hard_support_internal=observer_prime_saved;');
C.qpc_prime=contains(raw,'obj.qpc_stamp_prime_done=true;');
C.postcallback_candidate=contains(raw,'obj.postcallback_prime_candidate_found=~isempty(primeCandidate);');
C.postcallback_done=contains(raw,'obj.postcallback_prime_done=true;');
C.postcallback_restore=contains(raw,'obj.active_reference_mode=sp.mode;');
C.reference_only_defer=contains(raw, ...
    'EXECUTION_AWARE_DEFERRED_IN_REFERENCE_ONLY_HOTPATH');
C.execution_aware_retained=contains(raw, ...
    'Xexec=obj.build_execution_aware_proposal_certificate(candidate,CTX);');
C.ref_eval_timing=contains(raw,'obj.r38_ref_eval_ms=1e3*toc(tr38_ref);');
C.commit_prepare_timing=contains(raw,'obj.r38_commit_prepare_ms=1e3*toc(tr38_prepare);');
C.commit_rebuild_timing=contains(raw,'obj.r38_commit_rebuild_ms=1e3*toc(tr38_rebuild);');
C.commit_total_timing=contains(raw,'obj.r38_commit_total_ms=1e3*toc(tr38_total);');
C.native_callback=contains(raw,'af7r36_single_core_proposal(obj,CTX)');
C.hard4ms=contains(raw,'hard_modify_deadline_s<=4.0e-3');
C.r9=contains(raw,'R9_LOCAL_VECTOR_COUPLED_ADAPTIVE_BERNSTEIN_FLATNESS_SPEED');
C.wholebody=contains(raw,'MIDPOINT_STATIC_SUPPORT_MINUS_SPEED_TIMES_HALFWIDTH');
C.sampled_acceptance_true=contains(compact,needle);

closed=C.r14d&&C.r38&&C.attr_v3&&C.observer_prime&&C.observer_restore&& ...
    C.qpc_prime&&C.postcallback_candidate&&C.postcallback_done&& ...
    C.postcallback_restore&&C.reference_only_defer&&C.execution_aware_retained&& ...
    C.ref_eval_timing&&C.commit_prepare_timing&&C.commit_rebuild_timing&& ...
    C.commit_total_timing&&C.native_callback&&C.hard4ms&&C.r9&& ...
    C.wholebody&&~C.sampled_acceptance_true;

F=fieldnames(C);
for k=1:numel(F)
    fprintf(' %-46s = %d\n',strrep(F{k},'_',' '),logical(C.(F{k})));
end
fprintf(' R38 V3 source closed                       = %d\n',closed);
fprintf('====================================================================================================\n');

OUT=struct('schema','AF7R38C_V3_SOURCE_AUDIT','closed',closed,'checks',C);
assignin('base','AF7R38C_V3_SOURCE_AUDIT_OUT',OUT);
end
