function OUT = AF7R58ZZY_R27_SELFTEST()
% AF7R58ZZY-R27 source/representation equivalence selftest.
% No Flight, no controller construction, no production writes.

clc;
fprintf('\n====================================================================================================\n');
fprintf(' AF7R58ZZY-R27 | EXACT WORK-ELIMINATION SELFTEST\n');
fprintf('====================================================================================================\n');
fprintf('[AUTHORITY] noFlight=1 | noProductionWrite=1 | continuousSafetyMathChanged=0\n');

OUT=struct(); OUT.schema='AF7R58ZZY_R27_SELFTEST_V1'; OUT.closed=false;
root=fileparts(mfilename('fullpath'));
pC=fullfile(root,'REPLANNING_SYSTEM_R27_LEAN_EXACT.m');
pH=fullfile(root,'af7r78_incremental_compile_R27.m');
assert(isfile(pC)&&isfile(pH),'[R27 SELFTEST] package source missing.');
S=fileread(pC); H=fileread(pH);

OUT.class_name=contains(S,'classdef REPLANNING_SYSTEM_R27_LEAN_EXACT < REPLANNING_SYSTEM_R21_BALANCED');
OUT.build_id=contains(S,'AF7R58ZZY_R27_LEAN_EXACT_V1');
OUT.schedule_unchanged=contains(S,'ta=t_now+3*dt;')&&contains(S,'R.pipeline_intervals=3;');
OUT.no_final_full_compile=~contains(extract_between(S, ...
    '% CORE_PREPARED at exact scheduled activation:', ...
    'function [ok,reason] = activate_staged_reference_only_bundle'), ...
    'af7r76_compile_correction_packet_R23(');
OUT.incremental_helper_bound=contains(S,'af7r78_incremental_compile_R27(');
OUT.core_rebind=contains(S,'obj.r27_validate_incremental_binding(')&& ...
    contains(S,'B.r27_incremental_prevalidated=true;');
OUT.lazy_postcache=contains(S,'obj.repaired_reference_cache_valid=false;')&& ...
    contains(S,'obj.r27_lazy_postcache_pending=true;');
OUT.target_guard_preserved=contains(S,'[targetOK,targetReason]=obj.r23_validate_target_bundle(B);');
OUT.deep_freshness_preserved=contains(S,'isequaln(obj.static_obstacle_map,obj.staged_replan_static_map)')&& ...
    contains(S,'isequaln(obj.staged_reference_only_dependency_key(),obj.staged_replan_dependency_key)')&& ...
    contains(S,'isequaln(obj.active_speed_certificate,obj.staged_replan_speed_source)');
OUT.continuous_certificate_preserved=contains(S,'Rref=obj.evaluate_reference_only_proposal_certificate(P);');
OUT.r21_core_preserved=contains(S, ...
    'prepare_staged_reference_only_bundle@REPLANNING_SYSTEM_R21_BALANCED');
OUT.helper_prune_exact=contains(H,'double(base.group_t1(g))>double(t_commit)');
OUT.helper_dcp_mex=contains(H,'af7r77_compile_dcp_mex(cp0,dur)');
OUT.helper_schema=contains(H,'''AF7R58ZZY_R23_COMPILED_CORRECTION_V1''');

% Numeric equivalence of incremental packet against exact full R23 compiler.
assert(exist('af7r77_compile_dcp_mex','file')==3, ...
    '[R27 SELFTEST] af7r77_compile_dcp_mex missing.');
assert(exist('af7r76_compile_correction_packet_R23','file')==2, ...
    '[R27 SELFTEST] R23 exact full compiler missing.');

% Deterministic canonical base with one expired and one surviving group.
C(1)=mkcorr(1.00,2.00,[1.00 1.50 2.00],1.0);
C(2)=mkcorr(2.20,5.00,[2.20 3.00 5.00],2.0);
baseModel=struct('schema','AF7R33_BATCH_FLAT_MODEL_V1','certified',true, ...
    'corrections',C);
baseCP=af7r76_compile_correction_packet_R23(baseModel,7);

tc=2.10; % group 1 expires; group 2 survives
P=struct();
P.t0=tc; P.t1=6.0; P.representation_code=2;
P.piecewise_segment_t0=[tc 4.0 5.0];
P.piecewise_segment_t1=[4.0 5.0 6.0];
P.piecewise_control_points={mkcp(3.0),mkcp(4.0),mkcp(5.0)};

INC=af7r78_incremental_compile_R27(baseCP,P,8,tc);
fullModel=baseModel;
keep=[fullModel.corrections.t1]>tc;
fullModel.corrections=fullModel.corrections(keep);
Cnew=struct('t0',P.t0,'t1',P.t1, ...
    'segment_t0',P.piecewise_segment_t0, ...
    'segment_t1',P.piecewise_segment_t1, ...
    'control_points',{P.piecewise_control_points});
fullModel.corrections(end+1)=Cnew;
FULL=af7r76_compile_correction_packet_R23(fullModel,8);
OUT.incremental_vs_full_bitwise=packet_equal_bitwise(INC,FULL);

req={'class_name','build_id','schedule_unchanged','no_final_full_compile', ...
    'incremental_helper_bound','core_rebind','lazy_postcache','target_guard_preserved', ...
    'deep_freshness_preserved','continuous_certificate_preserved','r21_core_preserved', ...
    'helper_prune_exact','helper_dcp_mex','helper_schema','incremental_vs_full_bitwise'};
for k=1:numel(req)
    fprintf(' %-44s = %d\n',req{k},logical(OUT.(req{k})));
end
OUT.closed=all(cellfun(@(n)logical(OUT.(n)),req));
fprintf(' SELFTEST CLOSED                             = %d\n',OUT.closed);
fprintf('====================================================================================================\n');
assert(OUT.closed,'[R27 SELFTEST] contract did not close.');
end

function C=mkcorr(t0,t1,edges,seed)
C=struct(); C.t0=t0; C.t1=t1;
C.segment_t0=edges(1:end-1); C.segment_t1=edges(2:end);
C.control_points=cell(1,numel(C.segment_t0));
for k=1:numel(C.control_points), C.control_points{k}=mkcp(seed+0.1*k); end
end

function Q=mkcp(seed)
% exactly representable/simple deterministic values are sufficient; the test
% is implementation equivalence, not a trajectory-acceptance test.
[a,b]=ndgrid(1:3,0:15);
Q=seed + double(a)/8 + double(b)/32;
end

function tf=packet_equal_bitwise(A,B)
fields={'reference_version','group_count','segment_count','group_t0','group_t1', ...
    'group_first','group_segments','segment_t0','segment_t1','derivative_control_points', ...
    'max_derivative_order','degree'};
tf=strcmp(char(A.schema),char(B.schema))&& ...
   logical(A.strict_support)==logical(B.strict_support)&& ...
   strcmp(char(A.group_addition_order),char(B.group_addition_order))&& ...
   logical(A.certified_numeric_representation)==logical(B.certified_numeric_representation);
for k=1:numel(fields)
    if ~tf, return; end
    x=double(A.(fields{k})); y=double(B.(fields{k}));
    tf=isequal(size(x),size(y))&& ...
        isequal(typecast(x(:),'uint64'),typecast(y(:),'uint64'));
end
end

function s=extract_between(txt,a,b)
ia=strfind(txt,a); ib=strfind(txt,b);
if isempty(ia)||isempty(ib), s=''; return; end
i=ia(1); j=ib(find(ib>i,1,'first'));
if isempty(j), s=''; else, s=txt(i:j-1); end
end
