function OUT = AF7R27A_R10_SOURCE_THEOREM_AUDIT()
clc;
fprintf('\n');
fprintf('====================================================================================================\n');
fprintf(' AF7R27A | R10 SOURCE / THEOREM AUDIT\n');
fprintf('====================================================================================================\n');
fprintf('[AUTHORITY] readOnly=1 | sampledAcceptance=0 | caseIndependent=1\n');

src=which('REPLANNING_SYSTEM');
assert(~isempty(src)&&isfile(src),'[AF7R27A] source unresolved.');
raw=fileread(src);
compact=regexprep(raw,'\s+','');

A=struct();
A.r10_header=contains(raw,'AF7R27-R10');
A.adaptive_cover=contains(raw,'AF7R27-R10A ADAPTIVE CONTINUOUS SUPPORT COVER');
A.safe_only_by_continuous_support=contains(raw,'r10_nominal_interval_continuously_safe_one_obstacle');
A.no_fixed_depth=contains(raw,'There is no fixed depth authority');
A.unresolved_fail_closed=contains(raw,'conservatively UNSAFE');
A.live_dt_terminal_width=contains(raw,'width<=dt+tolMerge');
A.projected_corridor=contains(raw,'AF7R27-R10B CERTIFIED PROJECTED CORRIDOR COMPRESSION');
A.upper_envelope=contains(raw,'r10_upper_envelope_row_indices');
A.ambiguous_rows_retained=contains(raw,'Ambiguous rows are retained');
A.active_set_qp=contains(raw,'''Algorithm'',''active-set''');
A.codegen_solver_attempted=contains(raw,'''UseCodegenSolver'',true');
A.post_qp_constraint_recheck=contains(raw,'slack=Az-double(b(:));');
A.sampled_acceptance_true=contains(compact,'''sampled_acceptance'',true');
A.python_raw_marker=contains(raw,'r''''''');

forbidden={'215.681','152.235','36.4065','3.105590062','13.963699','2.932184'};
A.current_case_constants_hardcoded=any(cellfun(@(s)contains(raw,s),forbidden));

closed=A.r10_header&&A.adaptive_cover&&A.safe_only_by_continuous_support&& ...
    A.no_fixed_depth&&A.unresolved_fail_closed&&A.live_dt_terminal_width&& ...
    A.projected_corridor&&A.upper_envelope&&A.ambiguous_rows_retained&& ...
    A.active_set_qp&&A.post_qp_constraint_recheck&& ...
    ~A.sampled_acceptance_true&&~A.python_raw_marker&& ...
    ~A.current_case_constants_hardcoded;

F=fieldnames(A);
for k=1:numel(F)
    fprintf(' %-39s = %d\n',strrep(F{k},'_',' '),logical(A.(F{k})));
end
fprintf(' R10 source closed                       = %d\n',closed);
fprintf('====================================================================================================\n');

OUT=struct('schema','AF7R27A_R10_SOURCE_THEOREM_AUDIT_V2', ...
    'source',src,'checks',A,'closed',closed);
assignin('base','AF7R27A_R10_SOURCE_THEOREM_AUDIT_OUT',OUT);
end
