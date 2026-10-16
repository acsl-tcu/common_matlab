function OUT = AF7R40_PRE_RUN_AUDIT()
clc;
fprintf('\n====================================================================================================\n');
fprintf(' AF7R40B | REPAIRED PACKET BATCH PRE-RUN AUDIT\n');
fprintf('====================================================================================================\n');
fprintf('[AUTHORITY] sourceBound=1 | classParse=1 | safetyLogicUnchanged=1 | runtimeClaim=0\n');

src=which('REPLANNING_SYSTEM');
assert(~isempty(src)&&isfile(src));
expected='721fdc7cb5b771299215ebb69ec76fdbe6c1ec65554eaf1576c788e303491c71';
h=sha256_file(src);
assert(strcmp(h,expected),'[AF7R40B] source hash mismatch.');
raw=fileread(src);

C=struct();
C.marker=contains(raw,'AF7R40-REPAIRED-PACKET-BATCH');
C.batch_call=contains(raw,'[Y,valid]=af7r33_flat_jet_batch_model(');
C.batch_model_guard=contains(raw,'repaired batch flat model unavailable');
C.batch_accel_rows=contains(raw,'aL=Y(7:9,:)');
C.scalar_loop_removed=~contains(extract_method(raw,'repaired_reference_packet'), ...
    'for k=1:N');
C.nominal_tail_prime=contains(raw, ...
    'af7r33_active_flat_jet_batch(obj,[double(tf) double(tf)+double(obj.prediction_dt)]);');
C.hard4ms=contains(raw,'hard_modify_deadline_s<=4.0e-3');
compact=regexprep(raw,'\s+','');
C.sampled_true=contains(compact,[char(39) 'sampled_acceptance' char(39) ',true']);

assert(exist('af7r33_flat_jet_batch_model','file')==2, ...
    '[AF7R40B] af7r33_flat_jet_batch_model missing.');
mc=meta.class.fromName('REPLANNING_SYSTEM');
assert(~isempty(mc),'[AF7R40B] MATLAB class parse failed.');

closed=C.marker&&C.batch_call&&C.batch_model_guard&&C.batch_accel_rows&& ...
    C.scalar_loop_removed&&C.nominal_tail_prime&&C.hard4ms&&~C.sampled_true;

fprintf(' source SHA-256                          = %s\n',h);
fprintf(' exact AF7R33 batch call                 = %d\n',C.batch_call);
fprintf(' old 2*N scalar horizon loop removed     = %d\n',C.scalar_loop_removed);
fprintf(' batch evaluator preflight prime present = %d\n',C.nominal_tail_prime);
fprintf(' hard 4 ms unchanged                     = %d\n',C.hard4ms);
fprintf(' sampled acceptance true                 = %d\n',C.sampled_true);
fprintf(' AF7R40 pre-run closed                   = %d\n',closed);
fprintf('====================================================================================================\n');

OUT=struct('schema','AF7R40B_PRE_RUN_AUDIT_V1','closed',closed,'checks',C, ...
    'sha256',h,'source',src);
assignin('base','AF7R40_PRE_RUN_AUDIT_OUT',OUT);
end

function s=extract_method(raw,name)
pat=['(?ms)^\s*function[^\n]*' regexptranslate('escape',name) '\([^\n]*\).*?(?=^\s*function|^\s*end\s*$)'];
m=regexp(raw,pat,'match','once');
if isempty(m)
    % Exact fallback around this known method.
    a=strfind(raw,'function [pL_ref,q_ref,source] = repaired_reference_packet');
    b=strfind(raw,'function assert_repaired_reference_current');
    assert(numel(a)==1&&numel(b)==1&&b>a);
    s=raw(a:b-1);
else
    s=m;
end
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
