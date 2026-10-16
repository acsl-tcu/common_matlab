function OUT = AF7R43_PRE_RUN_AUDIT()
clc;
fprintf('\n====================================================================================================\n');
fprintf(' AF7R43A | HARD-PATH CLEANUP PRE-RUN AUDIT\n');
fprintf('====================================================================================================\n');

src=which('REPLANNING_SYSTEM'); assert(~isempty(src)&&isfile(src));
expected='52f2e46a73c5e35aec5e436a4484dd7663564e867374a017869e604ae04842c1';
h=sha256_file(src); assert(strcmp(h,expected),'[AF7R43A] source hash mismatch.');
raw=fileread(src); compact=regexprep(raw,'\s+','');

C=struct();
C.marker=contains(raw,'AF7R43-HARDPATH-CLEANUP');
C.r41=contains(raw,'AF7R41-REPAIRED-ROLLING-CACHE');
C.no_r42=~contains(raw,'AF7R42-REQUEST-HARDPATH-ATTRIBUTION');
C.defer_trigger=contains(raw,'deferTriggerCapture = obj.CAPTURE_REAL_TRIGGER_EVENT');
C.trigger_snapshot=contains(raw,'triggerAuthoritySnapshot=struct(');
C.defer_record=contains(raw,'record_guard=onCleanup');
C.nominal_cache=contains(raw,'nominal_batch_model_cache_valid');
C.cached_commit_model=contains(raw,'make_batch_model_from_proposal_cached');
C.hard4ms=contains(raw,'hard_modify_deadline_s<=4.0e-3');
C.r9=contains(raw,'R9_LOCAL_VECTOR_COUPLED_ADAPTIVE_BERNSTEIN_FLATNESS_SPEED');
C.wholebody=contains(raw,'MIDPOINT_STATIC_SUPPORT_MINUS_SPEED_TIMES_HALFWIDTH');
C.sampled_true=contains(compact,[char(39) 'sampled_acceptance' char(39) ',true']);

mc=meta.class.fromName('REPLANNING_SYSTEM');
assert(~isempty(mc),'[AF7R43A] MATLAB class parse failed.');

closed=C.marker&&C.r41&&C.no_r42&&C.defer_trigger&&C.trigger_snapshot&& ...
    C.defer_record&&C.nominal_cache&&C.cached_commit_model&& ...
    C.hard4ms&&C.r9&&C.wholebody&&~C.sampled_true;

F=fieldnames(C);
for k=1:numel(F)
    fprintf(' %-43s = %d\n',strrep(F{k},'_',' '),logical(C.(F{k})));
end
fprintf(' source SHA-256                           = %s\n',h);
fprintf(' AF7R43 pre-run closed                    = %d\n',closed);
fprintf('====================================================================================================\n');

OUT=struct('schema','AF7R43A_PRE_RUN_AUDIT_V1','closed',closed,'checks',C,'sha256',h);
assignin('base','AF7R43_PRE_RUN_AUDIT_OUT',OUT);
end

function h=sha256_file(fp)
fid=fopen(fp,'rb'); assert(fid>=0);
c=onCleanup(@()fclose(fid));
b=fread(fid,Inf,'*uint8'); clear c;
md=java.security.MessageDigest.getInstance('SHA-256');
md.update(b);
d=typecast(md.digest(),'uint8');
h=lower(reshape(dec2hex(d,2).',1,[]));
end
