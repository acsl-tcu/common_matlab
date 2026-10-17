function OUT = AF7R50C_PRE_RUN_AUDIT()
clc;
fprintf('\n====================================================================================================\n');
fprintf(' AF7R50C-A | CLEAN TIMING PRE-RUN AUDIT\n');
fprintf('====================================================================================================\n');
src=which('REPLANNING_SYSTEM'); assert(~isempty(src)&&isfile(src));
expected='223d34c49d7148b8ea14ff4ab9c420d409560c0650ac19e9b0f99318887090c9'; h=sha256_file(src); assert(strcmp(h,expected));
raw=fileread(src); compact=regexprep(raw,'\s+','');
C=struct();
C.r46=contains(raw,'AF7R46-COMPACT-COMMIT');
C.r50c=contains(raw,'AF7R50C-CLEAN-TIMING');
C.light_off=contains(raw,'ENABLE_LIGHT_RUNTIME_AUDIT = 0;');
C.capture_off=contains(raw,'CAPTURE_REAL_TRIGGER_EVENT   = false;');
C.mirror_off=contains(raw,'ENABLE_PROPOSAL_DIAGNOSTIC_MIRROR = false;');
C.mirror_guard=contains(raw,'if obj.ENABLE_PROPOSAL_DIAGNOSTIC_MIRROR');
C.noncommit_diag_off=contains(raw,'ENABLE_NONCOMMIT_DIAGNOSTIC_EVAL = 0;');
C.observer_present=contains(raw,'observer_update_same_event');
C.same_mex=contains(raw,'RAW=af7r36_single_core_mex(1,pkt);');
C.hard4ms=contains(raw,'hard_modify_deadline_s<=4.0e-3');
C.r9=contains(raw,'R9_LOCAL_VECTOR_COUPLED_ADAPTIVE_BERNSTEIN_FLATNESS_SPEED');
C.wholebody=contains(raw,'MIDPOINT_STATIC_SUPPORT_MINUS_SPEED_TIMES_HALFWIDTH');
C.sensor_memory=contains(raw,'apply_simulated_sensor_memory');
C.sampled_true=contains(compact,[char(39) 'sampled_acceptance' char(39) ',true']);
mc=meta.class.fromName('REPLANNING_SYSTEM'); assert(~isempty(mc));
closed=C.r46&&C.r50c&&C.light_off&&C.capture_off&&C.mirror_off&& ...
    C.mirror_guard&&C.noncommit_diag_off&&C.observer_present&&C.same_mex&& ...
    C.hard4ms&&C.r9&&C.wholebody&&C.sensor_memory&&~C.sampled_true;
F=fieldnames(C);
for k=1:numel(F)
    fprintf(' %-42s = %d\n',strrep(F{k},'_',' '),logical(C.(F{k})));
end
fprintf(' source SHA-256                         = %s\n',h);
fprintf(' AF7R50C pre-run closed                 = %d\n',closed);
fprintf('====================================================================================================\n');
OUT=struct('schema','AF7R50C_PRE_RUN_AUDIT_V1','closed',closed,'checks',C,'sha256',h);
assignin('base','AF7R50C_PRE_RUN_AUDIT_OUT',OUT);
end
function h=sha256_file(fp)
fid=fopen(fp,'rb'); assert(fid>=0); c=onCleanup(@()fclose(fid));
b=fread(fid,Inf,'*uint8'); clear c;
md=java.security.MessageDigest.getInstance('SHA-256'); md.update(b);
d=typecast(md.digest(),'uint8'); h=lower(reshape(dec2hex(d,2).',1,[]));
end
