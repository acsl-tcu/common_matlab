function OUT = AF7R49_PRE_RUN_AUDIT()
clc;
fprintf('\n====================================================================================================\n');
fprintf(' AF7R49B | PRODUCTION PRE-RUN AUDIT\n');
fprintf('====================================================================================================\n');
src=which('REPLANNING_SYSTEM'); expected='a18b8cd4953f5e67cc5ddcf9efe1b11c284cce674a941f2dbbba0f68c40a04be'; h=sha256_file(src); assert(strcmp(h,expected));
raw=fileread(src); compact=regexprep(raw,'\s+','');
C=struct();
C.marker=contains(raw,'AF7R49-PRODUCTION-CLOSEOUT');
C.new_callback=contains(raw,'af7r49_single_core_proposal_compact');
C.new_mex=contains(raw,'RAW=af7r49_single_core_mex(1,pkt);');
C.ast_preflight=contains(raw,'af7r49_prime_core_reference_runtime(obj);');
C.light_audit_off=contains(raw,'ENABLE_LIGHT_RUNTIME_AUDIT = 0;');
C.trigger_capture_off=contains(raw,'CAPTURE_REAL_TRIGGER_EVENT   = false;');
C.proposal_mirror_off=contains(raw,'ENABLE_PROPOSAL_DIAGNOSTIC_MIRROR = false;');
C.observer_stats_off=contains(raw,'ENABLE_OBSERVER_RUNTIME_STATS = false;');
C.observer_update_present=contains(raw,'obj.observer_update_same_event(time,cha);');
C.hard4ms=contains(raw,'hard_modify_deadline_s<=4.0e-3');
C.r9=contains(raw,'R9_LOCAL_VECTOR_COUPLED_ADAPTIVE_BERNSTEIN_FLATNESS_SPEED');
C.wholebody=contains(raw,'MIDPOINT_STATIC_SUPPORT_MINUS_SPEED_TIMES_HALFWIDTH');
C.sensor_memory=contains(raw,'apply_simulated_sensor_memory');
C.sampled_true=contains(compact,[char(39) 'sampled_acceptance' char(39) ',true']);
C.mex_exists=exist('af7r49_single_core_mex','file')==3;
mc=meta.class.fromName('REPLANNING_SYSTEM'); C.class_parse=~isempty(mc);
closed=C.marker&&C.new_callback&&C.new_mex&&C.ast_preflight&&C.light_audit_off&& ...
    C.trigger_capture_off&&C.proposal_mirror_off&&C.observer_stats_off&& ...
    C.observer_update_present&&C.hard4ms&&C.r9&&C.wholebody&&C.sensor_memory&& ...
    ~C.sampled_true&&C.mex_exists&&C.class_parse;
F=fieldnames(C);for k=1:numel(F),fprintf(' %-42s = %d\n',strrep(F{k},'_',' '),logical(C.(F{k})));end
fprintf(' source SHA-256                         = %s\n',h);
fprintf(' AF7R49 pre-run closed                  = %d\n',closed);
fprintf('====================================================================================================\n');
OUT=struct('schema','AF7R49B_PRE_RUN_AUDIT_V1','closed',closed,'checks',C,'sha256',h);
assignin('base','AF7R49_PRE_RUN_AUDIT_OUT',OUT);
end
function h=sha256_file(fp)
fid=fopen(fp,'rb');assert(fid>=0);c=onCleanup(@()fclose(fid));b=fread(fid,Inf,'*uint8');clear c;
md=java.security.MessageDigest.getInstance('SHA-256');md.update(b);d=typecast(md.digest(),'uint8');h=lower(reshape(dec2hex(d,2).',1,[]));
end
