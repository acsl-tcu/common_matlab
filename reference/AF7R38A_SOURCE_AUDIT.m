function OUT = AF7R38A_SOURCE_AUDIT()
clc;
fprintf('\n====================================================================================================\n');
fprintf(' AF7R38A | PREFLIGHT / POLICY-HOTPATH SOURCE AUDIT\n');
fprintf('====================================================================================================\n');

src=which('REPLANNING_SYSTEM');
assert(~isempty(src)&&isfile(src));
raw=fileread(src);
compact=regexprep(raw,'\s+','');
needle=[char(39) 'sampled_acceptance' char(39) ',true'];

C=struct();
C.r14d=contains(raw,'AF7R37-R14D-PRODUCTION');
C.r38=contains(raw,'AF7R38-PREFLIGHT-POLICY-HOTPATH');
C.observer_prime_property=contains(raw,'observer_preflight_prime_done');
C.observer_prime_state_restore=contains(raw, ...
    'obj.observer_hard_support_internal=observer_prime_saved;');
C.observer_prime_required=contains(raw, ...
    'obj.observer_preflight_prime_done && ...');
C.qpc_prime=contains(raw,'obj.qpc_stamp_prime_done=true;');
C.postcallback_prime=contains(raw,'obj.postcallback_prime_done=true;');
C.postcallback_restore=contains(raw,'obj.active_reference_mode=sp.mode;');
C.reference_only_defer=contains(raw, ...
    'EXECUTION_AWARE_DEFERRED_IN_REFERENCE_ONLY_HOTPATH');
C.execution_aware_retained=contains(raw, ...
    'Xexec=obj.build_execution_aware_proposal_certificate(candidate,CTX);');
C.native_callback=contains(raw,'af7r36_single_core_proposal(obj,CTX)');
C.hard4ms=contains(raw,'hard_modify_deadline_s<=4.0e-3');
C.r9=contains(raw,'R9_LOCAL_VECTOR_COUPLED_ADAPTIVE_BERNSTEIN_FLATNESS_SPEED');
C.wholebody=contains(raw,'MIDPOINT_STATIC_SUPPORT_MINUS_SPEED_TIMES_HALFWIDTH');
C.sampled_acceptance_true=contains(compact,needle);

closed=C.r14d&&C.r38&&C.observer_prime_property&& ...
    C.observer_prime_state_restore&&C.observer_prime_required&& ...
    C.qpc_prime&&C.postcallback_prime&&C.postcallback_restore&& ...
    C.reference_only_defer&&C.execution_aware_retained&&C.native_callback&& ...
    C.hard4ms&&C.r9&&C.wholebody&&~C.sampled_acceptance_true;

F=fieldnames(C);
for k=1:numel(F)
    fprintf(' %-44s = %d\n',strrep(F{k},'_',' '),logical(C.(F{k})));
end
fprintf(' R38 source closed                         = %d\n',closed);
fprintf('====================================================================================================\n');

OUT=struct('schema','AF7R38A_SOURCE_AUDIT_V1','closed',closed,'checks',C);
assignin('base','AF7R38A_SOURCE_AUDIT_OUT',OUT);
end
