function OUT = AF7R32_SOURCE_AUDIT()
clc;
fprintf('\n====================================================================================================\n');
fprintf(' AF7R32 | R13B SOURCE / SHADOW CONTRACT AUDIT\n');
fprintf('====================================================================================================\n');
src=which('REPLANNING_SYSTEM'); assert(~isempty(src)&&isfile(src));
raw=fileread(src); compact=regexprep(raw,'\s+','');
needle=[char(39) 'sampled_acceptance' char(39) ',true'];
C=struct();
C.r10=contains(raw,'AF7R27-R10');
C.r11=contains(raw,'AF7R29-R11');
C.r12=contains(raw,'AF7R30-R12');
C.qp_legacy=contains(raw,'solve_piecewise_fixed_sign_qp');
C.r9_local=contains(raw,'build_piecewise_local_vector_speed_certificate');
C.rejoin=contains(raw,'reference_rejoin_cell_continuously_safe');
C.wholebody=contains(raw,'build_static_continuous_wholebody_shadow_certificate');
C.hard_4ms=contains(raw,'hard_modify_deadline_s<=4.0e-3');
C.sampled_acceptance_true=contains(compact,needle);
closed=C.r10&&C.r11&&C.r12&&C.qp_legacy&&C.r9_local&&C.rejoin&&C.wholebody&&C.hard_4ms&&~C.sampled_acceptance_true;
F=fieldnames(C);
for k=1:numel(F), fprintf(' %-42s = %d\n',strrep(F{k},'_',' '),logical(C.(F{k}))); end
fprintf(' R13B source contract closed              = %d\n',closed);
fprintf(' production source modified               = 0\n');
fprintf('====================================================================================================\n');
OUT=struct('schema','AF7R32_SOURCE_AUDIT_V1','checks',C,'closed',closed);
assignin('base','AF7R32_SOURCE_AUDIT_OUT',OUT);
end
