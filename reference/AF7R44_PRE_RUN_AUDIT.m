function OUT = AF7R44_PRE_RUN_AUDIT()
clc;
fprintf('\n====================================================================================================\n');
fprintf(' AF7R44A | SENSOR-MEMORY + R43 PRESERVATION PRE-RUN AUDIT\n');
fprintf('====================================================================================================\n');

src=which('REPLANNING_SYSTEM');
assert(~isempty(src)&&isfile(src),'[AF7R44A] REPLANNING_SYSTEM unresolved.');
expected='67c026dd021e26f7fcd7eefb9c1ffa9aa9023b77dd64525ae674fdaae83011d9';
h=sha256_file(src);
assert(strcmp(h,expected),'[AF7R44A] source hash mismatch.');

raw=fileread(src);
compact=regexprep(raw,'\s+','');
C=struct();
C.r43=contains(raw,'AF7R43-HARDPATH-CLEANUP');
C.r44=contains(raw,'AF7R44-SENSOR-MEMORY');
C.memory_method=contains(raw,'apply_simulated_sensor_memory');
C.initial_empty=contains(raw,'simulated_sensor_memory_mask=false(1,Mobs)');
C.synthetic_prime_excluded=contains(raw,'~obj.setup_exact_do_prime_active') && ...
    contains(raw,'~obj.generator_integration_prime_active');
C.flight_origin_gate=contains(raw,'double(time.t)>=tf_mem-1e-10*max(1,abs(tf_mem))');
C.rejoin_release=contains(raw,'t_now>=rel') && contains(raw,'replan_request_active');
C.current_vs_known=contains(raw,'sensor_current_active_mask') && ...
    contains(raw,'sensor_known_active_mask');
C.detected_context_only=contains(raw,'detected_only_obstacle_contract');
C.retained_context=contains(raw,'retained_sensor_memory_through_rejoin');
C.hard4ms=contains(raw,'hard_modify_deadline_s<=4.0e-3');
C.r9=contains(raw,'R9_LOCAL_VECTOR_COUPLED_ADAPTIVE_BERNSTEIN_FLATNESS_SPEED');
C.wholebody=contains(raw,'MIDPOINT_STATIC_SUPPORT_MINUS_SPEED_TIMES_HALFWIDTH');
C.sampled_true=contains(compact,[char(39) 'sampled_acceptance' char(39) ',true']);

mc=meta.class.fromName('REPLANNING_SYSTEM');
assert(~isempty(mc),'[AF7R44A] MATLAB class parse failed.');

closed=C.r43&&C.r44&&C.memory_method&&C.initial_empty&& ...
    C.synthetic_prime_excluded&&C.flight_origin_gate&&C.rejoin_release&& ...
    C.current_vs_known&&C.detected_context_only&&C.retained_context&& ...
    C.hard4ms&&C.r9&&C.wholebody&&~C.sampled_true;

F=fieldnames(C);
for k=1:numel(F)
    fprintf(' %-44s = %d\n',strrep(F{k},'_',' '),logical(C.(F{k})));
end
fprintf(' source SHA-256                           = %s\n',h);
fprintf(' AF7R44 pre-run closed                    = %d\n',closed);
fprintf('====================================================================================================\n');

OUT=struct('schema','AF7R44A_PRE_RUN_AUDIT_V1','closed',closed,'checks',C,'sha256',h);
assignin('base','AF7R44_PRE_RUN_AUDIT_OUT',OUT);
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
