function OUT = AF7R58ZZY_R26_SELFTEST()
% Static/structural selftest for R26. No Flight, no production modification.

root=fileparts(mfilename('fullpath'));
p=fullfile(root,'REPLANNING_SYSTEM_R26_BALANCED.m');
assert(isfile(p),'[R26 SELFTEST] class source missing.');
S=fileread(p);

OUT=struct();
OUT.schema='AF7R58ZZY_R26_SELFTEST_V1';
OUT.closed=false;
OUT.class_name=contains(S,'classdef REPLANNING_SYSTEM_R26_BALANCED < REPLANNING_SYSTEM_R21_BALANCED');
OUT.three_interval_activation=contains(S,'ta=t_now+3*dt;');
OUT.stage2_refcert=contains(S,'R26_REFERENCE_CERTIFIED');
OUT.stage3_core=contains(S,'R26_CORE_PREPARED');
OUT.r21_core_prepare=contains(S, ...
    'prepare_staged_reference_only_bundle@REPLANNING_SYSTEM_R21_BALANCED');
OUT.r23_compile_restored=contains(S,'af7r76_compile_correction_packet_R23');
OUT.r25_compile_not_used=~contains(extractBetweenLocal(S, ...
    'if strcmp(state,''CORE_PREPARED'')', ...
    'obj.r23_stage3_token_valid=true;'), ...
    'af7r77_compile_correction_packet_R25');
OUT.final_token_before_activation=contains(S,'obj.r23_stage3_token_valid=true;');
OUT.pipeline_intervals_3=contains(S,'S.pipeline_intervals=3;');
OUT.remaining_ttc_lb=contains(S, ...
    'double(obj.horizon_s)-4*double(obj.prediction_dt)');
OUT.fast_packet_preserved=contains(S,'obj.r25_stage2_fresh_token_valid=true;');
OUT.fail_closed_deadlines=all(contains(S,{ ...
    'R26_STAGE1_CONTROL_PERIOD_EXCEEDED', ...
    'R26_REFERENCE_CERT_CONTROL_PERIOD_EXCEEDED', ...
    'R26_CORE_PREP_CONTROL_PERIOD_EXCEEDED'}));

vals=struct2cell(OUT);
tf=true;
for k=1:numel(vals)
    if islogical(vals{k})&&isscalar(vals{k}), tf=tf&&vals{k}; end
end
OUT.closed=tf;

fprintf('\n====================================================================================================\n');
fprintf(' AF7R58ZZY-R26 | STRUCTURAL SELFTEST\n');
fprintf('====================================================================================================\n');
fn=fieldnames(OUT);
for k=1:numel(fn)
    v=OUT.(fn{k});
    if islogical(v)&&isscalar(v)
        fprintf(' %-40s = %d\n',fn{k},v);
    end
end
fprintf(' SELFTEST CLOSED                         = %d\n',OUT.closed);
fprintf('====================================================================================================\n');
assert(OUT.closed,'[R26 SELFTEST] structural contract did not close.');
end

function z=extractBetweenLocal(s,a,b)
ia=strfind(s,a); ib=strfind(s,b);
if isempty(ia)||isempty(ib), z=''; return; end
i=ia(1); j=ib(find(ib>i,1,'first'));
if isempty(j), z=''; else, z=s(i:j-1); end
end
