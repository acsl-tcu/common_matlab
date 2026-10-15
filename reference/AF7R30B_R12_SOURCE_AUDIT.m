function OUT = AF7R30B_R12_SOURCE_AUDIT()
clc;
fprintf('\n====================================================================================================\n');
fprintf(' AF7R30B | R12 SOURCE AUDIT\n');
fprintf('====================================================================================================\n');
fprintf('[AUTHORITY] readOnly=1 | caseIndependent=1 | safetyLogicUnchanged=1\n');
src=which('REPLANNING_SYSTEM');
raw=fileread(src); compact=regexprep(raw,'\s+','');
C=struct();
C.r12_marker=contains(raw,'AF7R30-R12');
C.native_point=contains(raw,'af7r30_support_hull_mex')&&contains(raw,'best_point_support_margin');
C.native_segment=contains(raw,'best_segment_support_margin');
C.native_hull=contains(raw,'r10_upper_envelope_row_indices');
C.matlab_fallback_retained=contains(raw,'support_margin_point')&&contains(raw,'support_margin_segment');
C.tail_prime=contains(raw,'r12_tail_prime_hot_path');
C.takeoff_only_call=contains(raw,'isequal(cha,''t'')')&&contains(raw,'r12_tail_prime_done');
C.r10_retained=contains(raw,'AF7R27-R10');
C.r11_retained=contains(raw,'AF7R29-R11');
C.sampled_acceptance_true=contains(compact,'''sampled_acceptance'',true');
C.current_case_constants_hardcoded=any(cellfun(@(s)contains(raw,s), ...
    {'9.850','10.400','33.884','5.5114','6.5339','2.25885'}));
closed=C.r12_marker&&C.native_point&&C.native_segment&&C.native_hull&& ...
    C.matlab_fallback_retained&&C.tail_prime&&C.takeoff_only_call&& ...
    C.r10_retained&&C.r11_retained&&~C.sampled_acceptance_true&& ...
    ~C.current_case_constants_hardcoded;
F=fieldnames(C);
for k=1:numel(F)
    fprintf(' %-38s = %d\n',strrep(F{k},'_',' '),logical(C.(F{k})));
end
fprintf(' R12 source closed                       = %d\n',closed);
fprintf('====================================================================================================\n');
OUT=struct('schema','AF7R30B_R12_SOURCE_AUDIT_V1','checks',C,'closed',closed);
assignin('base','AF7R30B_R12_SOURCE_AUDIT_OUT',OUT);
end
