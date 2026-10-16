function OUT = AF7R42_PRE_RUN_AUDIT()
clc;
fprintf('\n====================================================================================================\n');
fprintf(' AF7R42A | REQUEST-HARDPATH ATTRIBUTION PRE-RUN AUDIT\n');
fprintf('====================================================================================================\n');
fprintf('[AUTHORITY] sourceBound=1 | diagnosticOnly=1 | classParse=1 | safetyLogicUnchanged=1\n');

src=which('REPLANNING_SYSTEM');
assert(~isempty(src)&&isfile(src));
expected='794bd628763d620a0d0cb4be0fb497aed9c0b4d9e1fb05e629c7e83a7e456aef';
h=sha256_file(src);
assert(strcmp(h,expected),'[AF7R42A] source hash mismatch.');
raw=fileread(src);
compact=regexprep(raw,'\s+','');

C=struct();
C.marker=contains(raw,'AF7R42-REQUEST-HARDPATH-ATTRIBUTION');
C.trace_property=contains(raw,'af7r42_request_trace');
C.trace_append=contains(raw,'obj.af7r42_append_request_trace(');
C.pre_checkpoint=contains(raw,'obj.af7r42_hard_pre_ms=1e3*elapsed_before;');
C.context_checkpoint=contains(raw,'obj.af7r42_hard_ctx_end_ms=1e3*toc(t_hard_cycle);');
C.callback_checkpoint=contains(raw,'obj.af7r42_hard_cb_end_ms=1e3*toc(t_hard_cycle);');
C.record_checkpoint=contains(raw,'obj.af7r42_hard_record_end_ms=1e3*toc(t_hard_cycle);');
C.refcert_checkpoint=contains(raw,'obj.af7r42_hard_refcert_end_ms=1e3*toc(t_hard_cycle);');
C.commit_eval=contains(raw,'obj.af7r42_commit_eval_end_ms=1e3*toc(t_hard_cycle);');
C.commit_model=contains(raw,'obj.af7r42_commit_model_end_ms=1e3*toc(t_hard_cycle);');
C.commit_rebuild=contains(raw,'obj.af7r42_commit_rebuild_end_ms=1e3*toc(t_hard_cycle);');
C.commit_assert=contains(raw,'obj.af7r42_commit_assert_end_ms=1e3*toc(t_hard_cycle);');
C.r41=contains(raw,'AF7R41-REPAIRED-ROLLING-CACHE');
C.native=contains(raw,'af7r36_single_core_proposal(obj,CTX)');
C.hard4ms=contains(raw,'hard_modify_deadline_s<=4.0e-3');
C.r9=contains(raw,'R9_LOCAL_VECTOR_COUPLED_ADAPTIVE_BERNSTEIN_FLATNESS_SPEED');
C.wholebody=contains(raw,'MIDPOINT_STATIC_SUPPORT_MINUS_SPEED_TIMES_HALFWIDTH');
C.sampled_true=contains(compact,[char(39) 'sampled_acceptance' char(39) ',true']);

mc=meta.class.fromName('REPLANNING_SYSTEM');
assert(~isempty(mc),'[AF7R42A] MATLAB class parse failed.');

closed=C.marker&&C.trace_property&&C.trace_append&&C.pre_checkpoint&& ...
    C.context_checkpoint&&C.callback_checkpoint&&C.record_checkpoint&& ...
    C.refcert_checkpoint&&C.commit_eval&&C.commit_model&&C.commit_rebuild&& ...
    C.commit_assert&&C.r41&&C.native&&C.hard4ms&&C.r9&&C.wholebody&&~C.sampled_true;

F=fieldnames(C);
for k=1:numel(F)
    fprintf(' %-43s = %d\n',strrep(F{k},'_',' '),logical(C.(F{k})));
end
fprintf(' source SHA-256                            = %s\n',h);
fprintf(' AF7R42 pre-run closed                     = %d\n',closed);
fprintf(' runtime/4-ms performance claimed          = 0\n');
fprintf('====================================================================================================\n');

OUT=struct('schema','AF7R42A_PRE_RUN_AUDIT_V1','closed',closed,'checks',C,'sha256',h);
assignin('base','AF7R42_PRE_RUN_AUDIT_OUT',OUT);
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
