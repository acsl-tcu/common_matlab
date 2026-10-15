function OUT = AF7R33D_R13C_SOURCE_AUDIT()
clc;
fprintf('\n====================================================================================================\n');
fprintf(' AF7R33D | R13C PRODUCTION SOURCE AUDIT\n');
fprintf('====================================================================================================\n');
src=which('REPLANNING_SYSTEM'); assert(~isempty(src)&&isfile(src));
raw=fileread(src); compact=regexprep(raw,'\s+','');
needle=[char(39) 'sampled_acceptance' char(39) ',true'];
C=struct();
C.marker=contains(raw,'AF7R33-R13C');
C.r10=contains(raw,'AF7R27-R10'); C.r11=contains(raw,'AF7R29-R11'); C.r12=contains(raw,'AF7R30-R12');
C.repeat_model_commit=contains(raw,'R.batch_flat_model=af7r33_make_batch_model_from_proposal');
C.episode=contains(raw,'af7r33_episode_batch_allobs');
C.corridor=contains(raw,'af7r33_corridor_batch');
C.qp_native=contains(raw,'af7r32_exact_qp_mex');
C.qp_fallback=contains(raw,'quadprog(');
C.wholebody_batch=contains(raw,'af7r33_wholebody_batch_certified');
C.wholebody_legacy=contains(raw,'MIDPOINT_STATIC_SUPPORT_MINUS_SPEED_TIMES_HALFWIDTH');
C.tail_warm=contains(raw,'af7r33_active_flat_jet_batch(obj,[double(tf)');
C.hard_4ms=contains(raw,'hard_modify_deadline_s<=4.0e-3');
C.sampled_acceptance_true=contains(compact,needle);
closed=C.marker&&C.r10&&C.r11&&C.r12&&C.repeat_model_commit&&C.episode&&C.corridor&& ...
    C.qp_native&&C.qp_fallback&&C.wholebody_batch&&C.wholebody_legacy&&C.tail_warm&& ...
    C.hard_4ms&&~C.sampled_acceptance_true;
F=fieldnames(C);
for k=1:numel(F), fprintf(' %-42s = %d\n',strrep(F{k},'_',' '),logical(C.(F{k}))); end
fprintf(' R13C source closed                       = %d\n',closed);
fprintf('====================================================================================================\n');
OUT=struct('schema','AF7R33D_R13C_SOURCE_AUDIT_V1','closed',closed,'checks',C);
assignin('base','AF7R33D_R13C_SOURCE_AUDIT_OUT',OUT);
end
