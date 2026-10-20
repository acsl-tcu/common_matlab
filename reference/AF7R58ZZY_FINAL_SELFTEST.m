function OUT=AF7R58ZZY_FINAL_SELFTEST(sourcePath,runnerPath)
% Final integrated source/runtime audit. No Flight and no source writes.
if nargin<1||isempty(sourcePath)
    sourcePath=which('REPLANNING_SYSTEM');
end
if nargin<2||isempty(runnerPath)
    root=fileparts(fileparts(sourcePath));
    runnerPath=fullfile(root,'run_flight_headless.m');
end

clc;
fprintf('\n====================================================================================================\n');
fprintf(' AF7R58ZZY FINAL | INTEGRATED MAIN-CLASS SELFTEST\n');
fprintf('====================================================================================================\n');

OUT=struct(); OUT.closed=false;
EXP_FINAL='643d35abe47f7c79f5ff796ea476988469a02909ebd053bb71c668527790ea9e';

assert(~isempty(sourcePath)&&isfile(sourcePath),'[FINAL SELFTEST] REPLANNING_SYSTEM source missing.');
K=fileread(sourcePath);
OUT.source_sha=sha_file(sourcePath);
OUT.source_sha_exact=strcmp(OUT.source_sha,EXP_FINAL);

OUT.main_class=contains(K,'classdef REPLANNING_SYSTEM < handle');
OUT.no_historical_superclass=~contains(K,'@REPLANNING_SYSTEM') && ...
    ~contains(K,'< REPLANNING_SYSTEM_R');
OUT.no_native_raw=~contains(K,'P.native_raw=RAW');
OUT.r31_direct_stage_store=contains(K,'r31_prepare_core_bundle_direct(P,ta)') && ...
    contains(K,'''r31_direct_store_exact'',true');
OUT.r31_stage1_semantics=contains(K,'candidate=obj.af7r46_single_core_proposal_compact(CTX);') && ...
    contains(K,'obj.staged_replan_candidate=candidate;') && ...
    ~contains(K,'r32_single_core_proposal_direct');
OUT.exact_batch=contains(K,'af7r58i_prepare_batch_model(') && ...
    ~contains(K,'af7r80_fast_batch_append_R29(');
OUT.r19_base_fallbacks=contains(K,'base_build_nominal_reference_table_integrated') && ...
    contains(K,'base_observer_one_step_support_integrated');
OUT.r21_preflight_fallback=contains(K,'r21_prepare_staged_reference_only_bundle_integrated');
OUT.r30_latent_core_available=contains(K,'function [ok,reason,B] = r30_prepare_core_bundle_exact');
OUT.final_status=contains(K,'get_final_integrated_performance_status');
OUT.schedule_unchanged=contains(K,'pipeline_intervals=3') && ...
    contains(K,'pipeline_delay_s=3*double(obj.prediction_dt)');

% Duplicate top-level method names: fail closed.
names=regexp(K,'(?m)^        function\s+(?:\[[^\]]+\]\s*=\s*|[^=\n]+?=\s*)?([A-Za-z]\w*)\s*\(','tokens');
names=cellfun(@(x)x{1},names,'UniformOutput',false);
[un,~,ic]=unique(names);
cnt=accumarray(ic(:),1);
OUT.method_count=numel(names);
OUT.duplicate_method_names=un(cnt>1);
OUT.no_duplicate_methods=isempty(OUT.duplicate_method_names);

assert(isfile(runnerPath),'[FINAL SELFTEST] run_flight_headless.m missing.');
Rtxt=fileread(runnerPath);
OUT.runner_main_binding=count_occ(Rtxt,'REPLANNING_SYSTEM(agent,nominal_ref,replan_opts)')==1;
OUT.runner_no_historical_binding=isempty(regexp(Rtxt, ...
    'REPLANNING_SYSTEM_R\d+_[A-Za-z0-9_]*\(agent,nominal_ref,replan_opts\)','once'));

% Runtime dependencies: exact revisions where a revision API exists.
reqMex={'af7r70_observer_mex','af7r71_decasteljau_batch_mex', ...
    'af7r75_correction_posaccel_mex','af7r76_compiled_eval_mex', ...
    'af7r77_compile_dcp_mex','af7r36_single_core_mex','af7r79_packet_retarget_mex'};
OUT.required_mex_present=true;
for i=1:numel(reqMex)
    OUT.required_mex_present=OUT.required_mex_present && exist(reqMex{i},'file')==3;
end
reqM={'af7r75_posaccel_batch_R19','af7r76_compile_correction_packet_R23', ...
    'af7r77_compile_correction_packet_R25','af7r77_merge_result_reference_fast_R25'};
OUT.required_helpers_present=true;
for i=1:numel(reqM)
    OUT.required_helpers_present=OUT.required_helpers_present && exist(reqM{i},'file')==2;
end

OUT.r28_mex_revision=false;
if exist('af7r36_single_core_mex','file')==3
    q=af7r36_single_core_mex(5);
    OUT.r28_mex_revision=ischar(q)&&strcmp(q,'AF7R58ZZY_R28_NATIVE_ONCE_V1');
end
OUT.r29_packet_mex_revision=false;
if exist('af7r79_packet_retarget_mex','file')==3
    q=af7r79_packet_retarget_mex(2);
    OUT.r29_packet_mex_revision=ischar(q)&&strcmp(q,'AF7R58ZZY_R29_PACKET_RETARGET_V1');
end

OUT.closed=OUT.source_sha_exact&&OUT.main_class&&OUT.no_historical_superclass&& ...
    OUT.no_native_raw&&OUT.r31_direct_stage_store&&OUT.r31_stage1_semantics&& ...
    OUT.exact_batch&&OUT.r19_base_fallbacks&&OUT.r21_preflight_fallback&& ...
    OUT.r30_latent_core_available&&OUT.final_status&&OUT.schedule_unchanged&& ...
    OUT.no_duplicate_methods&&OUT.runner_main_binding&&OUT.runner_no_historical_binding&& ...
    OUT.required_mex_present&&OUT.required_helpers_present&& ...
    OUT.r28_mex_revision&&OUT.r29_packet_mex_revision;

fprintf(' integrated source SHA exact              = %d\n',OUT.source_sha_exact);
fprintf(' main class is REPLANNING_SYSTEM          = %d\n',OUT.main_class);
fprintf(' historical superclass dependency absent  = %d\n',OUT.no_historical_superclass);
fprintf(' unused native_raw absent                 = %d\n',OUT.no_native_raw);
fprintf(' R31 direct Stage3 store retained         = %d\n',OUT.r31_direct_stage_store);
fprintf(' R31 Stage1 semantics retained            = %d\n',OUT.r31_stage1_semantics);
fprintf(' exact AF7R58I batch retained             = %d\n',OUT.exact_batch);
fprintf(' R19 exact fallback bodies retained       = %d\n',OUT.r19_base_fallbacks);
fprintf(' R21 preflight fallback retained          = %d\n',OUT.r21_preflight_fallback);
fprintf(' latent exact R30 core available          = %d\n',OUT.r30_latent_core_available);
fprintf(' four-cycle measurement schedule retained = %d\n',OUT.schedule_unchanged);
fprintf(' duplicate method names                   = %d\n',~OUT.no_duplicate_methods);
fprintf(' runner uses main class directly          = %d\n',OUT.runner_main_binding);
fprintf(' required MEX/helpers present             = %d / %d\n', ...
    OUT.required_mex_present,OUT.required_helpers_present);
fprintf(' AF7R36 / packet MEX revisions exact      = %d / %d\n', ...
    OUT.r28_mex_revision,OUT.r29_packet_mex_revision);
fprintf(' SELFTEST CLOSED                          = %d\n',OUT.closed);
fprintf('====================================================================================================\n');
assert(OUT.closed,'[FINAL SELFTEST] integrated closure failed.');
end

function n=count_occ(s,p), n=numel(strfind(s,p)); end
function h=sha_file(p)
f=fopen(p,'rb'); assert(f>=0); c=onCleanup(@()fclose(f));
b=fread(f,Inf,'*uint8');
md=java.security.MessageDigest.getInstance('SHA-256'); md.update(typecast(b,'int8'));
d=typecast(md.digest(),'uint8'); h=lower(reshape(dec2hex(d,2).',1,[]));
end
