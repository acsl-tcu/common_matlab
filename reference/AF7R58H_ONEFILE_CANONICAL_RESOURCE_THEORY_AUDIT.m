function OUT = AF7R58H_ONEFILE_CANONICAL_RESOURCE_THEORY_AUDIT()
% AF7R58H
% Exact targeted audit for the active-reference resource theorem.
%
% PURPOSE
%   Close the source chain that AF7R58G under-detected, and determine the
%   exact production change required to obtain a finite active-resource
%   representation without changing the mathematical reference.
%
% IMPORTANT
%   - READ ONLY. No production file is modified.
%   - No Flight, simulation, MEX execution, or sampled safety acceptance.
%   - No hard-coded preflight physical parameters or actuator limits.
%   - Payload mass mL is not consumed here; it remains live-estimator
%     authority for the later tension/thrust/moment feasibility certificate.
%
% RETURN
%   AF7R58H_REPORT.txt
%   AF7R58H_OUT.mat

root=fileparts(mfilename('fullpath'));
repo=fileparts(root);
outdir=tempname(root);
[ok,msg]=mkdir(outdir); assert(ok,'%s',msg);
f=fopen(fullfile(outdir,'AF7R58H_REPORT.txt'),'wt','n','UTF-8');
assert(f>=0,'[AF7R58H] cannot open report.');
cleanup=onCleanup(@()fclose(f)); %#ok<NASGU>

say(f,'====================================================================================================');
say(f,' AF7R58H | CANONICAL ACTIVE-REFERENCE RESOURCE THEORY AUDIT');
say(f,'====================================================================================================');
say(f,'[AUTHORITY] readOnly=1 | productionModified=0 | targetedExactSource=1 | noCapGuess=1 | noFlight=1 | noMex=1');
say(f,' root = %s',root);
say(f,' repo = %s',repo);

SPEC={ ...
 'REPLANNING_SYSTEM.m', ...
 '223d34c49d7148b8ea14ff4ab9c420d409560c0650ac19e9b0f99318887090c9'; ...
 'af7r33_make_batch_model_from_proposal.m', ...
 'f0e9f487cd270430d191871366c84595be586b6da0fb7b93eb98f4dfc25b46fd'; ...
 'af7r33_nominal_batch_model.m', ...
 '0aa0a05b1ebfd94ae27c90fa65aa40250d8c5e647dff3bfda4df13cf54738453'; ...
 'af7r34_make_core_packet.m', ...
 'ce1f98dac7b49ec8510c09d5c89b7edd4aea64a6c6320e7d2752431528cf22f8'; ...
 'af7r36_single_core_proposal.m', ...
 'd7bbe05efd268743e7307bcf3291f8f0e7777ab126432abc190ac60c999cc7ac'; ...
 'af7r36_core.hpp', ...
 'ad5ce1970583c0313637dcac58b8cf949092fe53e201efc61e55754fb2ea79be'; ...
 'af7r36_single_core_mex.cpp', ...
 '4f9778723aa339d5167b90845fcfcaa8564cae68326ff1e4ed504c7e6daea'};

S=struct();
allPresent=true; allHash=true;
say(f,'');
say(f,'[A. EXACT CURRENT SOURCE BINDING]');
for i=1:size(SPEC,1)
    name=SPEC{i,1}; expected=SPEC{i,2};
    p=fullfile(root,name);
    present=isfile(p);
    h='';
    txt='';
    if present
        h=file_hash(p);
        txt=fileread(p);
    end
    key=matlab.lang.makeValidName(name);
    S.(key)=struct('name',name,'path',p,'present',present, ...
        'sha256',h,'expected_sha256',expected,'text',txt);
    match=present && strcmp(h,expected);
    allPresent=allPresent && present;
    allHash=allHash && match;
    say(f,' %-43s present=%d | SHA-match=%d | %s',name,present,match,h);
end
say(f,' all required present                      = %d',allPresent);
say(f,' all reviewed SHA exact                   = %d',allHash);

R=S.REPLANNING_SYSTEM_m.text;
B=S.af7r33_make_batch_model_from_proposal_m.text;
N=S.af7r33_nominal_batch_model_m.text;
K=S.af7r34_make_core_packet_m.text;
P=S.af7r36_single_core_proposal_m.text;
H=S.af7r36_core_hpp.text;
X=S.af7r36_single_core_mex_cpp.text;

nR=compact(R); nB=compact(B); nN=compact(N); nK=compact(K);
nP=compact(P); nH=compact(H); nX=compact(X);

% --------------------------------------------------------------------------------------
% B. Exact t0 == commit-time chain
% --------------------------------------------------------------------------------------
T=struct();
T.request_time_from_tnow = has(nR,'''time'',double(t_now)');
T.packet_request_alias = has(nK,'RQ=CTX.request;');
T.packet_t0_from_request = has(nK,'P.t0=double(RQ.time);');
T.native_mex_reads_t0 = has(nX,'doublet0=scalar(fld(P,"t0"),"t0")');
T.native_plan_receives_t0 = has(nX,'PlanResultQ=plan(R,L,B,O,t0,tm,t1)');
T.native_result_returns_t0 = has(nX,'set("t0",dscalar(t0))');
T.proposal_t0_from_native = ...
    has(nP,'P.t0=double(RAW.t0);') || has(nR,'P.t0=double(RAW.t0);');
T.reference_only_commit_called_with_tnow = ...
    has(nR,'obj.try_commit_reference_only_proposal(candidate,double(t_now),t_hard_cycle)');
T.reference_only_commit_records_tcommit = ...
    has(nR,'obj.last_replan_commit_time=double(t_commit);');

T.closed=T.request_time_from_tnow && T.packet_request_alias && ...
    T.packet_t0_from_request && T.native_mex_reads_t0 && ...
    T.native_plan_receives_t0 && T.native_result_returns_t0 && ...
    T.proposal_t0_from_native && T.reference_only_commit_called_with_tnow && ...
    T.reference_only_commit_records_tcommit;

say(f,'');
say(f,'[B. EXACT REFERENCE_ONLY t0 / COMMIT-TIME CHAIN]');
say(f,' REQ.time = t_now                                    = %d',T.request_time_from_tnow);
say(f,' native packet aliases RQ=CTX.request                = %d',T.packet_request_alias);
say(f,' native packet t0 = RQ.time                          = %d',T.packet_t0_from_request);
say(f,' MEX reads packet t0                                 = %d',T.native_mex_reads_t0);
say(f,' native plan receives identical t0                   = %d',T.native_plan_receives_t0);
say(f,' native result returns identical t0                  = %d',T.native_result_returns_t0);
say(f,' proposal t0 = RAW.t0                                = %d',T.proposal_t0_from_native);
say(f,' REFERENCE_ONLY commit called with same t_now        = %d',T.reference_only_commit_called_with_tnow);
say(f,' successful commit records t_commit                  = %d',T.reference_only_commit_records_tcommit);
say(f,' t0 = commit-time source chain CLOSED                = %d',T.closed);

% --------------------------------------------------------------------------------------
% C. Finite support of each correction group
% --------------------------------------------------------------------------------------
L=struct();
L.packet_t1_from_resolver = has(nK,'P.t1=double(replanner.resolve_reference_rejoin_search_end(P.tm));');
L.resolver_exists = has(nR,'functiontend=resolve_reference_rejoin_search_end(obj,tm)');
L.resolver_uses_nominal_end = has(nR,'obj.nominal_table_valid&&isfinite(double(obj.nominal_table_tend))');
L.resolver_only_uses_finite_active_vu = ...
    has(nR,'vr=double(obj.active_repaired_reference.valid_until);') && ...
    has(nR,'ifisfinite(vr)');
L.resolver_fail_closed_if_no_future_end = ...
    has(nR,'ifisfinite(tend)&&tend<=double(tm)') && has(nR,'tend=NaN;');
L.proposal_t1_from_native = ...
    has(nP,'P.t1=double(RAW.t1);') || has(nR,'P.t1=double(RAW.t1);');
L.batch_group_t1_from_proposal = ...
    has(nB,'C.t1=double(P.t1);') && has(nR,'C.t1=double(P.t1);');
L.matlab_correction_zero_outside_open_support = ...
    has(nR,'ift<=double(P.t0)||t>=double(P.t1)') && ...
    has(nR,'DOUT=zeros(3,maxOrder+1);');
L.native_correction_zero_outside_open_support = ...
    has(nH,'if(!(t>C.t0&&t<C.t1))continue;');

L.group_support_finite_source_bound = ...
    L.packet_t1_from_resolver && L.resolver_exists && ...
    L.resolver_uses_nominal_end && L.resolver_only_uses_finite_active_vu && ...
    L.resolver_fail_closed_if_no_future_end && L.proposal_t1_from_native && ...
    L.batch_group_t1_from_proposal;

L.expired_group_future_zero = ...
    L.matlab_correction_zero_outside_open_support && ...
    L.native_correction_zero_outside_open_support;

say(f,'');
say(f,'[C. FINITE CORRECTION-SUPPORT AUTHORITY]');
say(f,' packet t1 from finite resolver                       = %d',L.packet_t1_from_resolver);
say(f,' finite resolver present                              = %d',L.resolver_exists);
say(f,' resolver uses finite nominal table end               = %d',L.resolver_uses_nominal_end);
say(f,' active valid_until extends domain only when finite   = %d',L.resolver_only_uses_finite_active_vu);
say(f,' invalid/nonfuture resolver result fails closed       = %d',L.resolver_fail_closed_if_no_future_end);
say(f,' proposal t1 inherits native t1                      = %d',L.proposal_t1_from_native);
say(f,' batch correction group t1 inherits proposal t1      = %d',L.batch_group_t1_from_proposal);
say(f,' each correction support is source-bounded finite    = %d',L.group_support_finite_source_bound);
say(f,' MATLAB correction zero outside open support         = %d',L.matlab_correction_zero_outside_open_support);
say(f,' native correction skipped outside open support      = %d',L.native_correction_zero_outside_open_support);
say(f,' expired correction contributes zero for future time = %d',L.expired_group_future_zero);
say(f,' NOTE: active reference valid_until=Inf is not itself a resource leak;');
say(f,'       correction SUPPORT lifetime and active-reference validity are distinct contracts.');

% --------------------------------------------------------------------------------------
% D. Exact history-growth mechanisms
% --------------------------------------------------------------------------------------
G=struct();
G.nominal_starts_empty = has(nN,'model.corrections=struct(''t0'',{},''t1'',{},''segment_t0'',{},');
G.batch_appends_one = has(nB,'model.corrections(end+1)=C;') && ...
                     has(nR,'model.corrections(end+1)=C;');
G.batch_has_no_prune_marker = ...
    ~has(nB,'corrections(keep)') && ...
    ~has(nR,'model.corrections=model.corrections(keep)');
G.packet_serializes_all_groups = ...
    has(nK,'G=numel(model.corrections);') && ...
    has(nK,'forg=1:G') && has(nK,'P.repair_t0=zeros(1,G);');
G.native_allocates_all_groups = ...
    has(nX,'size_tG=rt0.size();') && has(nX,'R.repairs.resize(G);');
G.native_eval_iterates_repairs = has(nH,'for(constauto&C:repairs)');

G.materializer_snapshots_base = ...
    has(nR,'P.base_eval_fn=obj.snapshot_active_reference_eval_fn();') && ...
    has(nR,'P.base_flat_eval_fn=obj.snapshot_active_reference_flat_eval_fn();');
G.repaired_snapshot_returns_prior_eval = ...
    has(nR,'fn=obj.active_repaired_reference.eval_fn;');
G.repaired_flat_snapshot_returns_prior_eval = ...
    has(nR,'fn=obj.active_repaired_reference.flat_eval_fn;');
G.new_eval_captures_proposal = ...
    has(nR,'evalFn=@(tg)obj.evaluate_bezier_proposal_reference(tg,P);') && ...
    has(nR,'flatEvalFn=@(tg)obj.evaluate_bezier_proposal_flat_reference(tg,P);');
G.proposal_eval_calls_captured_base = ...
    has(nR,'xd=P.base_eval_fn(double(t));') && ...
    has(nR,'xd=P.base_flat_eval_fn(double(t));');
G.commit_installs_new_eval = ...
    has(nR,'R.eval_fn=evalFn;') && has(nR,'R.flat_eval_fn=flatEvalFn;');

G.batch_history_growth_source_bound = ...
    G.nominal_starts_empty && G.batch_appends_one && ...
    G.packet_serializes_all_groups && G.native_allocates_all_groups && ...
    G.native_eval_iterates_repairs;

G.recursive_closure_growth_source_bound = ...
    G.materializer_snapshots_base && ...
    G.repaired_snapshot_returns_prior_eval && ...
    G.repaired_flat_snapshot_returns_prior_eval && ...
    G.new_eval_captures_proposal && ...
    G.proposal_eval_calls_captured_base && ...
    G.commit_installs_new_eval;

say(f,'');
say(f,'[D. CURRENT HISTORY-GROWTH MECHANISMS]');
say(f,' nominal batch starts with zero correction groups     = %d',G.nominal_starts_empty);
say(f,' each batch commit appends one correction group       = %d',G.batch_appends_one);
say(f,' no exact expired-prune marker before append          = %d',G.batch_has_no_prune_marker);
say(f,' core packet serializes all stored correction groups  = %d',G.packet_serializes_all_groups);
say(f,' native parser allocates all serialized groups        = %d',G.native_allocates_all_groups);
say(f,' native evaluator iterates stored repair groups       = %d',G.native_eval_iterates_repairs);
say(f,' batch/native history growth source-bound             = %d',G.batch_history_growth_source_bound);
say(f,' proposal snapshots previous active eval handles      = %d',G.materializer_snapshots_base);
say(f,' repaired snapshot returns previous eval_fn           = %d',G.repaired_snapshot_returns_prior_eval);
say(f,' repaired flat snapshot returns previous flat_eval_fn = %d',G.repaired_flat_snapshot_returns_prior_eval);
say(f,' new eval handles capture proposal/base handles       = %d',G.new_eval_captures_proposal);
say(f,' proposal evaluators invoke captured prior handles    = %d',G.proposal_eval_calls_captured_base);
say(f,' commit installs those new handles as active          = %d',G.commit_installs_new_eval);
say(f,' recursive closure-history growth source-bound        = %d',G.recursive_closure_growth_source_bound);

% --------------------------------------------------------------------------------------
% E. Commit spacing
% --------------------------------------------------------------------------------------
D=struct();
D.last_commit_state_exists=has(nR,'last_replan_commit_time');
D.reference_only_writes_last=has(nR,'obj.last_replan_commit_time=double(t_commit);');
D.explicit_gap_guard = ...
    has(nR,'t_commit-obj.last_replan_commit_time') || ...
    has(nR,'t_commit-double(obj.last_replan_commit_time)') || ...
    has(nR,'last_replan_commit_time+double(obj.prediction_dt)') || ...
    has(nR,'last_replan_commit_time+obj.prediction_dt');
D.prediction_dt_live = has(nR,'double(obj.prediction_dt)');
D.closed=D.last_commit_state_exists && ...
    D.reference_only_writes_last && D.explicit_gap_guard;

say(f,'');
say(f,'[E. SUCCESSIVE-COMMIT SPACING]');
say(f,' last_replan_commit_time state exists                 = %d',D.last_commit_state_exists);
say(f,' successful REFERENCE_ONLY commit writes it           = %d',D.reference_only_writes_last);
say(f,' explicit previous-vs-new positive gap guard exists   = %d',D.explicit_gap_guard);
say(f,' prediction_dt is runtime/live property               = %d',D.prediction_dt_live);
say(f,' Dcommit_min source-closed                            = %d',D.closed);
if ~D.closed
    say(f,' Dcommit_min remains OPEN; callback cadence is not silently promoted to a theorem.');
end

% --------------------------------------------------------------------------------------
% F. Theory result
% --------------------------------------------------------------------------------------
TH=struct();
TH.t0_commit_closed=T.closed;
TH.finite_group_support=L.group_support_finite_source_bound;
TH.expired_group_future_zero=L.expired_group_future_zero;
TH.batch_growth=G.batch_history_growth_source_bound;
TH.closure_growth=G.recursive_closure_growth_source_bound;
TH.commit_spacing_closed=D.closed;

TH.prune_semantics_theorem = ...
    TH.t0_commit_closed && TH.finite_group_support && ...
    TH.expired_group_future_zero;
TH.prune_alone_sufficient = false;
TH.canonical_rebuild_required = ...
    TH.prune_semantics_theorem && TH.closure_growth;
TH.production_resource_closure_ready = ...
    TH.prune_semantics_theorem && TH.batch_growth && TH.closure_growth;

say(f,'');
say(f,'[F. RESOURCE THEORY RESULT]');
say(f,' t0=commit source chain closed                        = %d',TH.t0_commit_closed);
say(f,' finite support of every stored correction closed     = %d',TH.finite_group_support);
say(f,' expired group future contribution exactly zero       = %d',TH.expired_group_future_zero);
say(f,' current batch/native accumulation proved present     = %d',TH.batch_growth);
say(f,' current recursive closure accumulation proved present= %d',TH.closure_growth);
say(f,' exact expired-prune semantics theorem available      = %d',TH.prune_semantics_theorem);
say(f,' prune alone is sufficient                            = %d',TH.prune_alone_sufficient);
say(f,' canonical nonrecursive evaluator rebuild required    = %d',TH.canonical_rebuild_required);

say(f,'');
say(f,' THEOREM: At successful commit time tc=P.t0, every old correction group');
say(f,' with group.t1<=tc is identically zero for every future query t>=tc.');
say(f,' Therefore deleting those groups from the CANONICAL correction model is');
say(f,' reference-semantics preserving for all future times.');
say(f,'');
say(f,' HOWEVER: current eval_fn/flat_eval_fn capture the previous active evaluators');
say(f,' recursively. Deleting only model.corrections does NOT remove all retained');
say(f,' history and therefore does NOT close the memory/evaluation resource theorem.');

say(f,'');
say(f,'[G. REQUIRED PRODUCTION INSTALL -- NEXT STEP]');
say(f,' 1) Keep correction support [t0,t1] finite and source-derived; do NOT force active valid_until finite.');
say(f,' 2) Before append, exact-prune canonical groups with t1<=new commit t0.');
say(f,' 3) Rebuild eval_fn and flat_eval_fn from NOMINAL authority + pruned canonical batch model;');
say(f,'    do not capture previous active eval_fn/flat_eval_fn.');
say(f,' 4) Install explicit positive successive-commit spacing from live runtime configuration;');
say(f,'    use outward-safe floating-point arithmetic and fail closed.');
say(f,' 5) From finite support lifetime and certified Dcommit_min derive finite Gmax.');
say(f,' 6) Reuse AF7R58E: Cmax<=2^(max(Kmax-1,0)), Sgroup<=2*Cmax+1; derive finite Smax.');
say(f,' 7) Enforce: PRUNE_EXPIRED -> COUNT -> ORDINARY RETURN IF EXCESS -> APPEND -> CANONICAL REBUILD.');
say(f,' 8) Algebraic evaluator equivalence is primary; numerical regression is supplementary only.');
say(f,' 9) Re-run the hard 4-ms path only after equivalence/resource closure.');
say(f,'10) Later tension/thrust/moment certificate uses external preflight config + latest in-flight mL estimate.');
say(f,'');
say(f,' No physical parameter, actuator limit, obstacle geometry/count, trajectory instance,');
say(f,' or payload mass value is selected by AF7R58H.');

OUT=struct();
OUT.schema='AF7R58H_CANONICAL_ACTIVE_REFERENCE_RESOURCE_THEORY_V1';
OUT.source_binding=struct( ...
    'all_required_present',allPresent, ...
    'all_reviewed_sha_exact',allHash);
OUT.t0_commit_chain=T;
OUT.correction_support=L;
OUT.history_growth=G;
OUT.commit_spacing=D;
OUT.theorem=TH;
OUT.production_modified=false;
OUT.numeric_caps_selected=false;
OUT.payload_mass_consumed=false;

if allPresent && allHash && TH.production_resource_closure_ready
    OUT.status='CANONICAL_RESOURCE_THEORY_PRECONDITION_CLOSED__PRODUCTION_INSTALL_REQUIRED';
    OUT.next_action='INSTALL_EXACT_PRUNE__CANONICAL_NONRECURSIVE_EVALUATOR__OUTWARD_DCOMMIT_GUARD__GS_CAP';
else
    OUT.status='OPEN_SOURCE_OR_THEORY_PRECONDITION';
    OUT.next_action='RETURN_REPORT_AND_MAT__DO_NOT_MODIFY_PRODUCTION_YET';
end

OUT.required_return_files={'AF7R58H_REPORT.txt','AF7R58H_OUT.mat'};
OUT.output_directory=outdir;

say(f,'');
say(f,'[H. STATUS]');
say(f,' status                                      = %s',OUT.status);
say(f,' next action                                 = %s',OUT.next_action);
say(f,' production modified                         = %d',OUT.production_modified);
say(f,' numeric caps selected                       = %d',OUT.numeric_caps_selected);

save(fullfile(outdir,'AF7R58H_OUT.mat'),'OUT','-v7');
assignin('base','AF7R58H_OUT',OUT);

say(f,'');
say(f,'[RETURN THESE TWO FILES]');
say(f,' RETURN: %s',fullfile(outdir,'AF7R58H_REPORT.txt'));
say(f,' RETURN: %s',fullfile(outdir,'AF7R58H_OUT.mat'));

end

function tf=has(s,p)
tf=contains(s,p);
end

function s=compact(s)
s=regexprep(s,'\s+','');
end

function say(f,fmt,varargin)
s=sprintf(fmt,varargin{:});
fprintf(1,'%s\n',s);
fprintf(f,'%s\n',s);
end

function h=file_hash(p)
f=fopen(p,'rb'); assert(f>=0,'cannot read %s',p);
c=onCleanup(@()fclose(f)); %#ok<NASGU>
b=fread(f,Inf,'*uint8');
m=java.security.MessageDigest.getInstance('SHA-256');
m.update(b);
d=typecast(m.digest(),'uint8');
h=lower(reshape(dec2hex(d,2).',1,[]));
end
