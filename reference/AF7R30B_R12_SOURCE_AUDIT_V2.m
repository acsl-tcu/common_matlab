function OUT = AF7R30B_R12_SOURCE_AUDIT_V2()
% Strict source audit for AF7R30/R12.
% Unlike V1, this verifies the MEX call exists inside each patched method
% block rather than merely checking that the legacy method name exists.

clc;
fprintf('\n');
fprintf('====================================================================================================\n');
fprintf(' AF7R30B V2 | R12 STRICT SOURCE AUDIT\n');
fprintf('====================================================================================================\n');
fprintf('[AUTHORITY] readOnly=1 | strictMethodBinding=1 | safetyLogicUnchanged=1\n');

src=which('REPLANNING_SYSTEM');
assert(~isempty(src)&&isfile(src),'[AF7R30B-V2] source unresolved.');
raw=fileread(src);
compact=regexprep(raw,'\s+','');

pointBlock = block_between(raw, ...
    '        function [margin,ok] = best_point_support_margin', ...
    '        function [margin,ok] = best_segment_support_margin');

segmentBlock = block_between(raw, ...
    '        function [margin,ok] = best_segment_support_margin', ...
    '        function [margin_m,ok] = support_margin_point');

hullBlock = block_between(raw, ...
    '        function keep = r10_upper_envelope_row_indices', ...
    '        function CANDS = enumerate_piecewise_sign_qp_recursive');

C=struct();
C.r12_marker=contains(raw,'AF7R30-R12');
C.native_point_bound=contains(pointBlock,'af7r30_support_hull_mex') && ...
    contains(pointBlock,'persistent nativeReady');
C.native_segment_bound=contains(segmentBlock,'af7r30_support_hull_mex') && ...
    contains(segmentBlock,'persistent nativeReady');
C.native_hull_bound=contains(hullBlock,'af7r30_support_hull_mex') && ...
    contains(hullBlock,'persistent nativeReady');
C.point_matlab_fallback_retained=contains(pointBlock,'support_margin_point');
C.segment_matlab_fallback_retained=contains(segmentBlock,'support_margin_segment');
C.hull_matlab_fallback_retained=contains(hullBlock,'guard=128*eps(scale)');
C.tail_prime_method=contains(raw,'function r12_tail_prime_hot_path');
C.tail_prime_state=contains(raw,'r12_tail_prime_done = false;') && ...
    contains(raw,'r12_tail_prime_ms = NaN;');
C.takeoff_only_call=contains(raw,'isequal(cha,''t'')') && ...
    contains(raw,'r12_tail_prime_hot_path(time,varargin,xd_nom,tf12)');
C.r10_retained=contains(raw,'AF7R27-R10');
C.r11_retained=contains(raw,'AF7R29-R11');
C.sampled_acceptance_true=contains(compact,'''sampled_acceptance'',true');
C.current_case_constants_hardcoded=any(cellfun(@(s)contains(raw,s), ...
    {'9.850','10.400','33.884','5.5114','6.5339','2.25885'}));

closed=C.r12_marker && ...
    C.native_point_bound && C.native_segment_bound && C.native_hull_bound && ...
    C.point_matlab_fallback_retained && C.segment_matlab_fallback_retained && ...
    C.hull_matlab_fallback_retained && ...
    C.tail_prime_method && C.tail_prime_state && C.takeoff_only_call && ...
    C.r10_retained && C.r11_retained && ...
    ~C.sampled_acceptance_true && ~C.current_case_constants_hardcoded;

F=fieldnames(C);
for k=1:numel(F)
    fprintf(' %-42s = %d\n',strrep(F{k},'_',' '),logical(C.(F{k})));
end
fprintf(' R12 strict source closed                  = %d\n',closed);
fprintf('====================================================================================================\n');

OUT=struct('schema','AF7R30B_R12_SOURCE_AUDIT_V2', ...
    'source',src,'checks',C,'closed',closed);
assignin('base','AF7R30B_R12_SOURCE_AUDIT_V2_OUT',OUT);
end

function b=block_between(raw,a,bpat)
s=strfind(raw,a);
e=strfind(raw,bpat);
assert(numel(s)==1,'[AF7R30B-V2] start anchor count=%d for %s',numel(s),a);
e=e(e>s);
assert(numel(e)==1,'[AF7R30B-V2] end anchor count=%d for %s',numel(e),bpat);
b=raw(s:e-1);
end
