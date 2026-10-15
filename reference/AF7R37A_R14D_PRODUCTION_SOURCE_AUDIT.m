function OUT = AF7R37A_R14D_PRODUCTION_SOURCE_AUDIT()
clc;
fprintf('\n====================================================================================================\n');
fprintf(' AF7R37A | R14D PRODUCTION SOURCE AUDIT\n');
fprintf('====================================================================================================\n');

src=which('REPLANNING_SYSTEM');
assert(~isempty(src)&&isfile(src),'[AF7R37A] source unresolved.');
raw=fileread(src);
compact=regexprep(raw,'\s+','');
needle=[char(39) 'sampled_acceptance' char(39) ',true'];

C=struct();
C.r13c=contains(raw,'AF7R33-R13C');
C.r14d_marker=contains(raw,'AF7R37-R14D-PRODUCTION');
C.native_default_callback=contains(raw, ...
    'fn=@(CTX)af7r36_single_core_proposal(obj,CTX);');
C.runtime_prime=contains(raw,'af7r36_runtime_prime(obj);');
C.canonical_separator=contains(raw, ...
    '[n,~]=af7r36_canonical_separator_normal(R,a,progress);');
C.legacy_builtin_retained=contains(raw, ...
    'function OUT = builtin_bezier_d15_c7_proposal(obj,CTX)');
C.kernel_prime_retained=contains(raw,'prime_locked_backend_kernel_variants');
C.hard4ms=contains(raw,'hard_modify_deadline_s<=4.0e-3');
C.r9=contains(raw,'R9_LOCAL_VECTOR_COUPLED_ADAPTIVE_BERNSTEIN_FLATNESS_SPEED');
C.wholebody=contains(raw,'MIDPOINT_STATIC_SUPPORT_MINUS_SPEED_TIMES_HALFWIDTH');
C.repeat_model=contains(raw,'batch_flat_model');
C.sampled_acceptance_true=contains(compact,needle);

closed=C.r13c&&C.r14d_marker&&C.native_default_callback&& ...
    C.runtime_prime&&C.canonical_separator&&C.legacy_builtin_retained&& ...
    C.kernel_prime_retained&&C.hard4ms&&C.r9&&C.wholebody&& ...
    C.repeat_model&&~C.sampled_acceptance_true;

F=fieldnames(C);
for k=1:numel(F)
    fprintf(' %-42s = %d\n',strrep(F{k},'_',' '),logical(C.(F{k})));
end
fprintf(' R14D production source closed            = %d\n',closed);
fprintf('====================================================================================================\n');

OUT=struct('schema','AF7R37A_R14D_PRODUCTION_SOURCE_AUDIT_V1', ...
    'closed',closed,'checks',C);
assignin('base','AF7R37A_R14D_PRODUCTION_SOURCE_AUDIT_OUT',OUT);
end
