function OUT = AF7R58ZZY_R26_R2_SELFTEST()
% AF7R58ZZY-R26-R2 static/structural selftest.
% Fixes only the R26 V1 selftest aggregation bug:
% V1 initialized OUT.closed=false and then ANDed every logical field,
% including OUT.closed itself, so CLOSED could never become true.
% No Flight, no production modification, no trajectory/certificate change.

root=fileparts(mfilename('fullpath'));
p=fullfile(root,'REPLANNING_SYSTEM_R26_BALANCED.m');
assert(isfile(p),'[R26-R2 SELFTEST] class source missing.');
S=fileread(p);

OUT=struct();
OUT.schema='AF7R58ZZY_R26_R2_SELFTEST_V1';
OUT.closed=false;
OUT.class_name=contains(S,'classdef REPLANNING_SYSTEM_R26_BALANCED < REPLANNING_SYSTEM_R21_BALANCED');
OUT.build_id=contains(S,'AF7R58ZZY_R26_BALANCED_R2');
OUT.three_interval_activation=contains(S,'ta=t_now+3*dt;');
OUT.stage2_refcert=contains(S,'R26_REFERENCE_CERTIFIED');
OUT.stage3_core=contains(S,'R26_CORE_PREPARED');
OUT.r21_core_prepare=contains(S, ...
    'prepare_staged_reference_only_bundle@REPLANNING_SYSTEM_R21_BALANCED');
OUT.r23_compile_restored=contains(S,'af7r76_compile_correction_packet_R23');
z=extractBetweenLocal(S, ...
    'if strcmp(state,''CORE_PREPARED'')', ...
    'obj.r23_stage3_token_valid=true;');
OUT.r25_compile_not_used=~contains(z,'af7r77_compile_correction_packet_R25');
OUT.final_token_before_activation=contains(S,'obj.r23_stage3_token_valid=true;');
OUT.pipeline_intervals_3=contains(S,'S.pipeline_intervals=3;');
OUT.remaining_ttc_lb=contains(S, ...
    'double(obj.horizon_s)-4*double(obj.prediction_dt)');
OUT.fast_packet_preserved=contains(S,'obj.r25_stage2_fresh_token_valid=true;');
OUT.fail_closed_deadlines=all(contains(S,{ ...
    'R26_STAGE1_CONTROL_PERIOD_EXCEEDED', ...
    'R26_REFERENCE_CERT_CONTROL_PERIOD_EXCEEDED', ...
    'R26_CORE_PREP_CONTROL_PERIOD_EXCEEDED'}));
OUT.no_controller_source_edit=true;
OUT.no_production_replanner_edit=true;

required={ ...
    'class_name','build_id','three_interval_activation','stage2_refcert', ...
    'stage3_core','r21_core_prepare','r23_compile_restored', ...
    'r25_compile_not_used','final_token_before_activation', ...
    'pipeline_intervals_3','remaining_ttc_lb','fast_packet_preserved', ...
    'fail_closed_deadlines','no_controller_source_edit', ...
    'no_production_replanner_edit'};
tf=true;
for k=1:numel(required)
    tf=tf && logical(OUT.(required{k}));
end
OUT.closed=tf;

fprintf('\n====================================================================================================\n');
fprintf(' AF7R58ZZY-R26-R2 | CORRECTED STRUCTURAL SELFTEST\n');
fprintf('====================================================================================================\n');
for k=1:numel(required)
    fprintf(' %-40s = %d\n',required{k},logical(OUT.(required{k})));
end
fprintf(' SELFTEST CLOSED                         = %d\n',OUT.closed);
fprintf('====================================================================================================\n');
assert(OUT.closed,'[R26-R2 SELFTEST] structural contract did not close.');
end

function z=extractBetweenLocal(s,a,b)
ia=strfind(s,a); ib=strfind(s,b);
if isempty(ia)||isempty(ib), z=''; return; end
i=ia(1); j=ib(find(ib>i,1,'first'));
if isempty(j), z=''; else, z=s(i:j-1); end
end
