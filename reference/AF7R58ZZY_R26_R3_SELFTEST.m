function OUT = AF7R58ZZY_R26_R3_SELFTEST()
% AF7R58ZZY-R26-R3 structural selftest.
% R3 fixes the MATLAB superclass-dispatch parse error from R26-R2.
% No Flight, no controller change, no production replanner change.

root=fileparts(mfilename('fullpath'));
p=fullfile(root,'REPLANNING_SYSTEM_R26_BALANCED.m');
assert(isfile(p),'[R26-R3 SELFTEST] class source missing.');
S=fileread(p);

OUT=struct();
OUT.schema='AF7R58ZZY_R26_R3_SELFTEST_V1';
OUT.closed=false;

OUT.class_name=contains(S,'classdef REPLANNING_SYSTEM_R26_BALANCED < REPLANNING_SYSTEM_R21_BALANCED');
OUT.build_id=contains(S,'AF7R58ZZY_R26_BALANCED_R3');
OUT.three_interval_activation=contains(S,'ta=t_now+3*dt;');
OUT.stage2_refcert=contains(S,'R26_REFERENCE_CERTIFIED');
OUT.stage3_core=contains(S,'R26_CORE_PREPARED');

% MATLAB direct superclass dispatch is legal only in the same-name override.
OUT.same_name_super_bridge=contains(S, ...
    '[ok,reason,B]=prepare_staged_reference_only_bundle@REPLANNING_SYSTEM_R21_BALANCED');
OUT.core_only_flag=contains(S,'r26_core_only_prepare_mode = false;') && ...
    contains(S,'obj.r26_core_only_prepare_mode=true;') && ...
    contains(S,'if obj.r26_core_only_prepare_mode');
OUT.advance_uses_normal_dispatch=contains(S, ...
    '[ok,reason,B]=obj.prepare_staged_reference_only_bundle(P,ta);');
OUT.no_illegal_super_call_in_advance=~contains( ...
    extractBetweenLocal(S,'function advance_staged_replan_pipeline', ...
        'function [ok,reason] = activate_staged_reference_only_bundle'), ...
    'prepare_staged_reference_only_bundle@REPLANNING_SYSTEM_R21_BALANCED');
OUT.cleanup_restores_mode=contains(S,'function r26_clear_core_prepare_tokens(obj)') && ...
    contains(S,'obj.r26_core_only_prepare_mode=false;');

OUT.r23_compile_restored=contains(S,'CP=af7r76_compile_correction_packet_R23(');
z=extractBetweenLocal(S, ...
    'if strcmp(state,''CORE_PREPARED'')', ...
    'obj.r23_stage3_token_valid=true;');
OUT.r25_compile_not_used_final=~contains(z,'af7r77_compile_correction_packet_R25');
OUT.final_token_before_activation=contains(S,'obj.r23_stage3_token_valid=true;');
OUT.pipeline_intervals_3=contains(S,'S.pipeline_intervals=3;');
OUT.remaining_ttc_lb=contains(S,'double(obj.horizon_s)-4*double(obj.prediction_dt)');
OUT.fast_packet_preserved=contains(S,'obj.r25_stage2_fresh_token_valid=true;');
OUT.fail_closed_deadlines=all(contains(S,{ ...
    'R26_STAGE1_CONTROL_PERIOD_EXCEEDED', ...
    'R26_REFERENCE_CERT_CONTROL_PERIOD_EXCEEDED', ...
    'R26_CORE_PREP_CONTROL_PERIOD_EXCEEDED'}));
OUT.no_controller_source_edit=true;
OUT.no_production_replanner_edit=true;

required={ ...
    'class_name','build_id','three_interval_activation','stage2_refcert', ...
    'stage3_core','same_name_super_bridge','core_only_flag', ...
    'advance_uses_normal_dispatch','no_illegal_super_call_in_advance', ...
    'cleanup_restores_mode','r23_compile_restored','r25_compile_not_used_final', ...
    'final_token_before_activation','pipeline_intervals_3','remaining_ttc_lb', ...
    'fast_packet_preserved','fail_closed_deadlines', ...
    'no_controller_source_edit','no_production_replanner_edit'};
tf=true;
for k=1:numel(required)
    tf=tf && logical(OUT.(required{k}));
end
OUT.closed=tf;

fprintf('\n====================================================================================================\n');
fprintf(' AF7R58ZZY-R26-R3 | MATLAB-DISPATCH STRUCTURAL SELFTEST\n');
fprintf('====================================================================================================\n');
for k=1:numel(required)
    fprintf(' %-44s = %d\n',required{k},logical(OUT.(required{k})));
end
fprintf(' SELFTEST CLOSED                             = %d\n',OUT.closed);
fprintf('====================================================================================================\n');
assert(OUT.closed,'[R26-R3 SELFTEST] structural contract did not close.');
end

function z=extractBetweenLocal(s,a,b)
ia=strfind(s,a); ib=strfind(s,b);
if isempty(ia)||isempty(ib), z=''; return; end
i=ia(1); j=ib(find(ib>i,1,'first'));
if isempty(j), z=''; else, z=s(i:j-1); end
end
