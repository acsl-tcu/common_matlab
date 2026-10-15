function OUT = AF7R34_SOURCE_CONTRACT_AUDIT()
clc;
fprintf('\n====================================================================================================\n');
fprintf(' AF7R34 | R14 SINGLE-CORE SOURCE CONTRACT AUDIT\n');
fprintf('====================================================================================================\n');

src=which('REPLANNING_SYSTEM');
assert(~isempty(src)&&isfile(src),'[AF7R34] REPLANNING_SYSTEM unresolved.');
raw=fileread(src);
compact=regexprep(raw,'\s+','');
needle=[char(39) 'sampled_acceptance' char(39) ',true'];

C=struct();
C.r10=contains(raw,'AF7R27-R10');
C.r11=contains(raw,'AF7R29-R11');
C.r12=contains(raw,'AF7R30-R12');
C.r13c=contains(raw,'AF7R33-R13C');
C.hard4ms=contains(raw,'hard_modify_deadline_s<=4.0e-3');
C.piecewise=contains(raw,'PIECEWISE_DEGREE15_C7_WITH_AFFINE_UNSAFE_COMPONENTS');
C.r9=contains(raw,'R9_LOCAL_VECTOR_COUPLED_ADAPTIVE_BERNSTEIN_FLATNESS_SPEED');
C.wholebody=contains(raw,'MIDPOINT_STATIC_SUPPORT_MINUS_SPEED_TIMES_HALFWIDTH');
C.repeat_model=contains(raw,'batch_flat_model');
C.sampled_acceptance_true=contains(compact,needle);

closed=C.r10&&C.r11&&C.r12&&C.r13c&&C.hard4ms&&C.piecewise&& ...
    C.r9&&C.wholebody&&C.repeat_model&&~C.sampled_acceptance_true;

F=fieldnames(C);
for k=1:numel(F)
    fprintf(' %-42s = %d\n',strrep(F{k},'_',' '),logical(C.(F{k})));
end
fprintf(' R14 source contract closed               = %d\n',closed);
fprintf(' production source modified               = 0\n');
fprintf('====================================================================================================\n');

OUT=struct('schema','AF7R34_SOURCE_CONTRACT_AUDIT_V1', ...
    'closed',closed,'checks',C);
assignin('base','AF7R34_SOURCE_CONTRACT_AUDIT_OUT',OUT);
end
