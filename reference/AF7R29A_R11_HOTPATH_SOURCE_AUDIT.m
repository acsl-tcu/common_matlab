function OUT = AF7R29A_R11_HOTPATH_SOURCE_AUDIT()
% Read-only structural audit for R11 hot-path diagnostic deferral.

fprintf('\n');
fprintf('====================================================================================================\n');
fprintf(' AF7R29A | R11 NONCOMMIT HOT-PATH SOURCE AUDIT\n');
fprintf('====================================================================================================\n');
fprintf('[AUTHORITY] readOnly=1 | commitSetUnchanged=1 | safetyAcceptanceUnchanged=1\n');

src=which('REPLANNING_SYSTEM');
assert(~isempty(src)&&isfile(src),'[AF7R29A] source unresolved.');
raw=fileread(src);
compact=regexprep(raw,'\s+','');

C=struct();
C.r11_marker=contains(raw,'AF7R29-R11');
C.default_off=contains(raw,'ENABLE_NONCOMMIT_DIAGNOSTIC_EVAL = 0');
C.failed_fast_return=contains(raw,'SKIPPED_FAILED_PROPOSAL_PRODUCTION_HOTPATH');
C.deadline_fast_return=contains(raw,'SKIPPED_DEADLINE_MISS_PRODUCTION_HOTPATH');
C.failed_status_21_retained=contains(raw,'obj.generator_last_status_code=21;');
C.deadline_status_minus20_retained=contains(raw,'obj.generator_last_status_code=-20;');
C.original_diagnostics_retained=contains(raw,'evaluate_reference_only_proposal_certificate(candidate)') && ...
    contains(raw,'build_execution_aware_proposal_certificate(candidate,CTX)');
C.sampled_acceptance_true=contains(compact,'''sampled_acceptance'',true');

closed=C.r11_marker&&C.default_off&&C.failed_fast_return&& ...
    C.deadline_fast_return&&C.failed_status_21_retained&& ...
    C.deadline_status_minus20_retained&&C.original_diagnostics_retained&& ...
    ~C.sampled_acceptance_true;

F=fieldnames(C);
for k=1:numel(F)
    fprintf(' %-38s = %d\n',strrep(F{k},'_',' '),logical(C.(F{k})));
end
fprintf(' R11 source closed                       = %d\n',closed);
fprintf('====================================================================================================\n');

OUT=struct('schema','AF7R29A_R11_HOTPATH_SOURCE_AUDIT_V1', ...
    'checks',C,'closed',closed,'source',src);
assignin('base','AF7R29A_R11_HOTPATH_SOURCE_AUDIT_OUT',OUT);
end
