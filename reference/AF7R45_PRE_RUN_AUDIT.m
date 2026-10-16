function OUT = AF7R45_PRE_RUN_AUDIT()
clc;
fprintf('\n====================================================================================================\n');
fprintf(' AF7R45A | TWO-TRACK REALTIME PRE-RUN AUDIT\n');
fprintf('====================================================================================================\n');

src=which('REPLANNING_SYSTEM');
assert(~isempty(src)&&isfile(src),'[AF7R45A] REPLANNING_SYSTEM unresolved.');
expected='ded3e68fb1b7da5187985e1f171dc970afeb82d9ba09fecacbc4f07f553936d9';
h=sha256_file(src);
assert(strcmp(h,expected),'[AF7R45A] source hash mismatch.');

raw=fileread(src);
compact=regexprep(raw,'\s+','');
C=struct();
C.r44=contains(raw,'AF7R44-SENSOR-MEMORY');
C.r45=contains(raw,'AF7R45-TWO-TRACK-RT');
C.exec_only_fast=contains(raw,'execOnlyChange =');
C.exec_stale_failclosed=contains(raw,'obj.certificate_stale.ACTUATOR_DYNAMICS_CERT=true;') && ...
    contains(raw,'obj.certificate_stale.HLC_ROBUST_CERT=true;');
C.packet_reuse_check=contains(raw,'can_reuse_reference_only_runtime_packet_exact');
C.packet_reuse_apply=contains(raw,'reuse_reference_only_runtime_packet_exact');
C.exact_speed_equality=contains(raw,'isequal(double(oldSC.payload_speed_UB_m_s),VL)') && ...
    contains(raw,'isequal(double(oldSC.uav_speed_UB_m_s),VQ)');
C.static_map_exact_check=contains(raw,'oldPacket.Araw(:,ii)');
C.safe_fallback=contains(raw,'obj.prepare_obstacle_packet(obj.static_obstacle_map)');
C.premotion_prime=contains(raw,'AF7R45: pre-motion prime of the exact packet-reuse branch');
C.hard4ms=contains(raw,'hard_modify_deadline_s<=4.0e-3');
C.r9=contains(raw,'R9_LOCAL_VECTOR_COUPLED_ADAPTIVE_BERNSTEIN_FLATNESS_SPEED');
C.wholebody=contains(raw,'MIDPOINT_STATIC_SUPPORT_MINUS_SPEED_TIMES_HALFWIDTH');
C.sensor_memory=contains(raw,'apply_simulated_sensor_memory');
C.sampled_true=contains(compact,[char(39) 'sampled_acceptance' char(39) ',true']);

mc=meta.class.fromName('REPLANNING_SYSTEM');
assert(~isempty(mc),'[AF7R45A] MATLAB class parse failed.');

closed=C.r44&&C.r45&&C.exec_only_fast&&C.exec_stale_failclosed&& ...
    C.packet_reuse_check&&C.packet_reuse_apply&&C.exact_speed_equality&& ...
    C.static_map_exact_check&&C.safe_fallback&&C.premotion_prime&& ...
    C.hard4ms&&C.r9&&C.wholebody&&C.sensor_memory&&~C.sampled_true;

F=fieldnames(C);
for k=1:numel(F)
    fprintf(' %-45s = %d\n',strrep(F{k},'_',' '),logical(C.(F{k})));
end
fprintf(' source SHA-256                            = %s\n',h);
fprintf(' AF7R45 pre-run closed                     = %d\n',closed);
fprintf('====================================================================================================\n');

OUT=struct('schema','AF7R45A_PRE_RUN_AUDIT_V1','closed',closed,'checks',C,'sha256',h);
assignin('base','AF7R45_PRE_RUN_AUDIT_OUT',OUT);
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
