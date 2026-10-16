function OUT = AF7R41_PRE_RUN_AUDIT()
clc;
fprintf('\n====================================================================================================\n');
fprintf(' AF7R41B | REPAIRED ROLLING-CACHE PRE-RUN AUDIT\n');
fprintf('====================================================================================================\n');
src=which('REPLANNING_SYSTEM'); assert(~isempty(src)&&isfile(src));
expected='ffce61f125c0d2a8f1f216396344817e32ddfd0d24794842adeee47742d7f762'; h=sha256_file(src); assert(strcmp(h,expected),'[AF7R41B] source hash mismatch.');
raw=fileread(src); compact=regexprep(raw,'\s+','');
C=struct();
C.marker=contains(raw,'AF7R41-REPAIRED-ROLLING-CACHE');
C.cache_props=contains(raw,'repaired_reference_cache_valid = false;');
C.shift1=contains(raw,'REPAIRED_ROLLING_SHIFT1');
C.one_tail=contains(raw,'xf=double(R.flat_eval_fn(t0+(N-1)*dt));');
C.full_batch=contains(raw,'REPAIRED_FULL_BATCH');
C.preflight_full=count(raw,'obj.repaired_reference_packet(double(t_now));')>=2;
C.preflight_shift=contains(raw,'obj.repaired_reference_packet(double(t_now)+double(obj.prediction_dt));');
C.restore=count(raw,'obj.repaired_reference_cache=sp.repairedCache;')>=2;
C.hard4ms=contains(raw,'hard_modify_deadline_s<=4.0e-3');
C.r9=contains(raw,'R9_LOCAL_VECTOR_COUPLED_ADAPTIVE_BERNSTEIN_FLATNESS_SPEED');
C.wholebody=contains(raw,'MIDPOINT_STATIC_SUPPORT_MINUS_SPEED_TIMES_HALFWIDTH');
C.sampled_true=contains(compact,[char(39) 'sampled_acceptance' char(39) ',true']);
mc=meta.class.fromName('REPLANNING_SYSTEM'); assert(~isempty(mc),'[AF7R41B] class parse failed.');
closed=C.marker&&C.cache_props&&C.shift1&&C.one_tail&&C.full_batch&&C.preflight_full&&C.preflight_shift&&C.restore&&C.hard4ms&&C.r9&&C.wholebody&&~C.sampled_true;
F=fieldnames(C); for k=1:numel(F), fprintf(' %-43s = %d\n',strrep(F{k},'_',' '),logical(C.(F{k}))); end
fprintf(' source SHA-256                             = %s\n',h);
fprintf(' AF7R41 pre-run closed                      = %d\n',closed);
fprintf('====================================================================================================\n');
OUT=struct('schema','AF7R41B_PRE_RUN_AUDIT_V1','closed',closed,'checks',C,'sha256',h,'source',src); assignin('base','AF7R41_PRE_RUN_AUDIT_OUT',OUT);
end
function h=sha256_file(fp)
fid=fopen(fp,'rb'); assert(fid>=0); c=onCleanup(@()fclose(fid)); b=fread(fid,Inf,'*uint8'); clear c;
md=java.security.MessageDigest.getInstance('SHA-256'); md.update(b); d=typecast(md.digest(),'uint8'); h=lower(reshape(dec2hex(d,2).',1,[]));
end
