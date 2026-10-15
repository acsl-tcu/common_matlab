function OUT = AF7R31_SOURCE_FORENSIC_AUDIT()
clc;
fprintf('\n====================================================================================================\n');
fprintf(' AF7R31 | LATEST-SOURCE R13 FORENSIC AUDIT\n');
fprintf('====================================================================================================\n');
src=which('REPLANNING_SYSTEM'); assert(~isempty(src)&&isfile(src));
raw=fileread(src); compact=regexprep(raw,'\s+','');
needle=[char(39) 'sampled_acceptance' char(39) ',true'];
C=struct();
C.r10=contains(raw,'AF7R27-R10');
C.r11=contains(raw,'AF7R29-R11');
C.r12=contains(raw,'AF7R30-R12');
C.hard_4ms=contains(raw,'hard_modify_deadline_s<=4.0e-3');
C.r9_stageB=contains(raw,'build_piecewise_local_vector_speed_certificate');
C.episode_adaptive=contains(raw,'AF7R27-R10A ADAPTIVE CONTINUOUS SUPPORT COVER');
C.corridor_hull=contains(raw,'AF7R27-R10B CERTIFIED PROJECTED CORRIDOR COMPRESSION');
C.qp_active_set=contains(raw,'''Algorithm'',''active-set''');
C.wholebody_continuous=contains(raw,'MIDPOINT_STATIC_SUPPORT_MINUS_SPEED_TIMES_HALFWIDTH');
C.flat_scalar_hotpath=contains(raw,'z=double(flatfh{rr+1}(tau));');
C.sampled_acceptance_true=contains(compact,needle);
closed=C.r10&&C.r11&&C.r12&&C.hard_4ms&&C.r9_stageB&&C.episode_adaptive&& ...
    C.corridor_hull&&C.qp_active_set&&C.wholebody_continuous&&C.flat_scalar_hotpath&& ...
    ~C.sampled_acceptance_true;
F=fieldnames(C);
for k=1:numel(F), fprintf(' %-42s = %d\n',strrep(F{k},'_',' '),logical(C.(F{k}))); end
fprintf(' forensic source closed                   = %d\n',closed);
fprintf(' production source modified               = 0\n');
fprintf('====================================================================================================\n');
OUT=struct('schema','AF7R31_SOURCE_FORENSIC_AUDIT_V1','checks',C,'closed',closed);
assignin('base','AF7R31_SOURCE_FORENSIC_AUDIT_OUT',OUT);
end
