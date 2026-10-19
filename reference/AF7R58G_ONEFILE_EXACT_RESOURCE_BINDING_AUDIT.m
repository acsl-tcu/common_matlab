function OUT = AF7R58G_ONEFILE_EXACT_RESOURCE_BINDING_AUDIT()
% AF7R58G: exact current-source resource-contract binding audit.
%
% Put this file in common_matlab/reference and run:
%     AF7R58G_ONEFILE_EXACT_RESOURCE_BINDING_AUDIT
%
% Return the two files printed at the end:
%     AF7R58G_REPORT.txt
%     AF7R58G_OUT.mat
%
% PURPOSE
%   Narrow, read-only follow-up to AF7R58F.  This file intentionally does NOT
%   rescan the repository.  It source-binds only the current production chain
%   needed for the finite correction-resource theorem:
%
%       request time -> native t0/t1 -> proposal -> immutable batch model
%       -> active reference commit -> correction lifetime
%       -> AF7R58E dyadic component bound -> S_group <= 2*C+1
%
%   It then asks whether current production already proves a strictly positive
%   lower bound Dcommit_min on successive successful commits.  dt by itself is
%   NOT accepted as proof unless an explicit commit-spacing guard is present.
%
% DESIGN CONTRACT PRESERVED
%   * Preflight tunables and actuator/input limits remain externally supplied.
%   * No numeric mQ/J/L/g/thrust/moment limit is selected here.
%   * Payload mass mL is NOT frozen here; later tension/thrust/moment feasibility
%     must use the latest in-flight estimator authority.
%   * No tracking-error margin is invented.
%   * No Flight, simulation, constructor, MEX execution, or sampled safety
%     acceptance is performed.
%   * This audit does not modify production.
%
% IMPORTANT
%   AF7R58G is allowed to conclude OPEN.  In particular, a missing explicit
%   commit-spacing guard or an unbounded generic valid_until path is reported as
%   OPEN rather than replaced by a guessed cap.

root = fileparts(mfilename('fullpath'));
outdir = tempname(root);
[ok,msg] = mkdir(outdir);
assert(ok,'%s',msg);

reportPath = fullfile(outdir,'AF7R58G_REPORT.txt');
f = fopen(reportPath,'wt','n','UTF-8');
assert(f>=0,'[AF7R58G] cannot open report.');
cleanup = onCleanup(@()fclose(f)); %#ok<NASGU>

say(f,'====================================================================================================');
say(f,' AF7R58G | EXACT CURRENT RESOURCE-BINDING AUDIT');
say(f,'====================================================================================================');
say(f,'[AUTHORITY] readOnly=1 | productionModified=0 | targetedSourcesOnly=1 | noCapGuess=1 | noFlight=1 | noMex=1');

%% A. Exact current sources only.
NAMES = { ...
    'REPLANNING_SYSTEM.m', ...
    'af7r33_make_batch_model_from_proposal.m', ...
    'af7r33_nominal_batch_model.m', ...
    'af7r34_make_core_packet.m', ...
    'af7r36_single_core_proposal.m', ...
    'af7r36_core.hpp', ...
    'af7r36_single_core_mex.cpp'};

SRC = struct('name',{},'path',{},'present',{},'sha256',{},'text',{});
for k=1:numel(NAMES)
    p=fullfile(root,NAMES{k});
    s=struct('name',NAMES{k},'path',p,'present',isfile(p),'sha256','','text','');
    if s.present
        s.sha256=file_hash(p);
        s.text=fileread(p);
    end
    SRC(end+1)=s; %#ok<AGROW>
end

expectedReplan='223d34c49d7148b8ea14ff4ab9c420d409560c0650ac19e9b0f99318887090c9';
expectedCore='ad5ce1970583c0313637dcac58b8cf949092fe53e201efc61e55754fb2ea79be';
expectedMex='4f9778723aa3aa339d5167b90845fcfcaa8564cae68326ff1e4ed504c7e6daea';

R=getsrc(SRC,'REPLANNING_SYSTEM.m');
M=getsrc(SRC,'af7r33_make_batch_model_from_proposal.m');
N=getsrc(SRC,'af7r33_nominal_batch_model.m');
P=getsrc(SRC,'af7r34_make_core_packet.m');
Q=getsrc(SRC,'af7r36_single_core_proposal.m');
H=getsrc(SRC,'af7r36_core.hpp');
X=getsrc(SRC,'af7r36_single_core_mex.cpp');

source=struct();
source.replan_current=R.present && strcmp(R.sha256,expectedReplan);
source.core_current=H.present && strcmp(H.sha256,expectedCore);
source.mex_current=X.present && strcmp(X.sha256,expectedMex);
source.all_required_present=all([SRC.present]);

say(f,'');
say(f,'[A. EXACT SOURCE BINDING]');
for k=1:numel(SRC)
    say(f,' %-42s present=%d | SHA-256=%s',SRC(k).name,SRC(k).present,SRC(k).sha256);
end
say(f,' REPLANNING_SYSTEM reviewed SHA match         = %d',source.replan_current);
say(f,' af7r36_core reviewed SHA match               = %d',source.core_current);
say(f,' af7r36_single_core_mex reviewed SHA match    = %d',source.mex_current);

%% B. Request/commit time identity chain.
nR=compact(R.text); nM=compact(M.text); nN=compact(N.text); nP=compact(P.text); nQ=compact(Q.text); nH=compact(H.text);

B=struct();
B.request_time_from_tnow = has(nR,"'time',double(t_now)");
B.native_packet_t0_from_request = has(nP,'P.t0=double(RQ.time);');
B.native_packet_uses_same_CTX = has(nQ,'pkt=af7r34_make_core_packet(replanner,CTX,[]);');
B.proposal_t0_from_native = has(nQ,'P.t0=double(RAW.t0);');
B.reference_only_commit_uses_tnow = has(nR,'obj.try_commit_reference_only_proposal(candidate,double(t_now),t_hard_cycle)');
B.batch_group_t0_from_proposal = has(nM,'C.t0=double(P.t0);');
B.batch_group_t1_from_proposal = has(nM,'C.t1=double(P.t1);');
B.batch_append_exact = has(nM,'model.corrections(end+1)=C;');
B.nominal_model_empty_corrections = has(nN,"model.corrections=struct('t0',{},'t1',{},'segment_t0',{},");
B.current_reference_only_t0_equals_commit = all([ ...
    B.request_time_from_tnow, B.native_packet_t0_from_request, ...
    B.native_packet_uses_same_CTX, B.proposal_t0_from_native, ...
    B.reference_only_commit_uses_tnow, B.batch_group_t0_from_proposal]);

say(f,'');
say(f,'[B. t0 / COMMIT-TIME SOURCE CHAIN]');
print_bool(f,'REQ.time = t_now',B.request_time_from_tnow);
print_bool(f,'native packet t0 = REQ.time',B.native_packet_t0_from_request);
print_bool(f,'AF7R36 proposal uses same CTX packet',B.native_packet_uses_same_CTX);
print_bool(f,'proposal t0 = native RAW.t0',B.proposal_t0_from_native);
print_bool(f,'REFERENCE_ONLY commit called with t_now',B.reference_only_commit_uses_tnow);
print_bool(f,'batch correction t0 = proposal t0',B.batch_group_t0_from_proposal);
print_bool(f,'batch correction t1 = proposal t1',B.batch_group_t1_from_proposal);
print_bool(f,'batch model appends exactly one group',B.batch_append_exact);
print_bool(f,'nominal batch model starts with zero groups',B.nominal_model_empty_corrections);
print_bool(f,'current REFERENCE_ONLY path proves group t0 = commit time',B.current_reference_only_t0_equals_commit);

%% C. t1 / finite lifetime authority.
C=struct();
C.packet_t1_from_finite_resolver = has(nP,'P.t1=double(replanner.resolve_reference_rejoin_search_end(P.tm));');
C.nominal_table_end_from_preflight_te = has(nR,'tend=te+obj.horizon_s;') && ...
    has(nR,'obj.nominal_table_tend=t0+(Ntab-1)*obj.prediction_dt;');
C.resolver_uses_nominal_table_end = has(nR,'tend=double(obj.nominal_table_tend);');
C.resolver_sees_active_valid_until = has(nR,"isfield(obj.active_repaired_reference,'valid_until')") && ...
    has(nR,'vr=double(obj.active_repaired_reference.valid_until);') && ...
    has(nR,'tend=max(tend,vr);');
C.resolver_fail_closed_if_no_future_end = has(nR,'ifisfinite(tend)&&tend<=double(tm)') && has(nR,'tend=NaN;');
C.reference_only_sets_valid_until_inf = has(nR,'R.valid_until=Inf;');
C.generic_commit_copies_candidate_valid_until = has(nR,'R.valid_until=double(candidate.valid_until);');
C.generic_validator_requires_finite_ordered = has(nR,'vf=double(candidate.valid_from);') && ...
    has(nR,'vu=double(candidate.valid_until);') && ...
    has(nR,'isfinite(vf)&&isfinite(vu)&&vu>vf');

% Search specifically for an upper-bound relation tying generic candidate vu to
% a finite reference-validity authority.  A mere finiteness check is not enough.
genericBlock=function_block(R.text,'function validate_repaired_candidate');
C.generic_vu_upper_bounded_to_nominal = regex_present(compact(genericBlock), ...
    {'vu<=double(obj.nominal_table_tend)','candidate.valid_until<=double(obj.nominal_table_tend)', ...
     'vu<=obj.nominal_table_tend','candidate.valid_until<=obj.nominal_table_tend'});
C.generic_vu_external_finite_authority = regex_present(lower(genericBlock), ...
    {'reference_valid_until','reference_validity','validity_upper_bound','valid_until_max'});
C.generic_lifetime_upper_bound_closed = C.generic_vu_upper_bounded_to_nominal || C.generic_vu_external_finite_authority;

% Current REFERENCE_ONLY path ignores its Inf valid_until in the resolver and is
% therefore source-bounded by nominal_table_tend, provided that table authority
% is valid.  This is not promoted to the generic future EXECUTION_AWARE path.
C.current_reference_only_lifetime_source_bound = B.current_reference_only_t0_equals_commit && ...
    C.packet_t1_from_finite_resolver && C.nominal_table_end_from_preflight_te && ...
    C.resolver_uses_nominal_table_end && C.reference_only_sets_valid_until_inf;

say(f,'');
say(f,'[C. t1 / CORRECTION-LIFETIME AUTHORITY]');
print_bool(f,'packet t1 uses resolve_reference_rejoin_search_end',C.packet_t1_from_finite_resolver);
print_bool(f,'nominal table end derives from preflight te + horizon',C.nominal_table_end_from_preflight_te);
print_bool(f,'resolver uses nominal_table_tend',C.resolver_uses_nominal_table_end);
print_bool(f,'resolver may extend to finite active valid_until',C.resolver_sees_active_valid_until);
print_bool(f,'REFERENCE_ONLY active valid_until is Inf (not finite extension)',C.reference_only_sets_valid_until_inf);
print_bool(f,'generic commit copies candidate.valid_until',C.generic_commit_copies_candidate_valid_until);
print_bool(f,'generic validator requires finite ordered valid interval',C.generic_validator_requires_finite_ordered);
print_bool(f,'generic valid_until upper-bounded by finite reference authority',C.generic_lifetime_upper_bound_closed);
print_bool(f,'current REFERENCE_ONLY correction lifetime source-bounded',C.current_reference_only_lifetime_source_bound);
say(f,' symbolic current-path bound: Tcorr <= nominal_table_tend - commit_time');
say(f,' conservative preflight form: Tcorr_max <= nominal_table_tend - flight_reference_time_origin');
say(f,' NOTE: no numeric duration is selected by this audit.');

%% D. Successive-commit lower bound Dcommit_min.
% There are two commit implementations in the current source.  We require an
% explicit source guard comparing the new commit timestamp with the previous
% successful commit timestamp.  Comments about "next cycle" and dt are not proof.
refCommit=function_block(R.text,'function [ok,reason] = try_commit_reference_only_proposal');
genericCommit=function_block(R.text,'function commit_repaired_reference');

D=struct();
D.last_commit_state_exists = has(nR,'last_replan_commit_time=NaN;') || ...
    contains(R.text,'last_replan_commit_time');
D.reference_only_writes_last_commit = contains(refCommit,'obj.last_replan_commit_time=double(t_commit);');
D.generic_writes_last_commit = contains(genericCommit,'obj.last_replan_commit_time=double(t_commit);');
D.reference_only_spacing_guard = explicit_spacing_guard(refCommit);
D.generic_spacing_guard = explicit_spacing_guard(genericCommit);
D.prediction_dt_live_bound = has(nR,'obj.prediction_dt=time.dt;') || ...
    has(nR,'obj.prediction_dt=dt_val;');
D.commit_spacing_closed = D.reference_only_spacing_guard && D.generic_spacing_guard;
D.Dcommit_min = NaN; % deliberately not guessed

say(f,'');
say(f,'[D. SUCCESSIVE COMMIT SPACING]');
print_bool(f,'last_replan_commit_time state exists',D.last_commit_state_exists);
print_bool(f,'REFERENCE_ONLY writes successful commit time',D.reference_only_writes_last_commit);
print_bool(f,'generic commit writes successful commit time',D.generic_writes_last_commit);
print_bool(f,'REFERENCE_ONLY has explicit previous-vs-new commit spacing guard',D.reference_only_spacing_guard);
print_bool(f,'generic commit has explicit previous-vs-new commit spacing guard',D.generic_spacing_guard);
print_bool(f,'prediction_dt is live/runtime-bound',D.prediction_dt_live_bound);
print_bool(f,'Dcommit_min > 0 source-closed',D.commit_spacing_closed);
if ~D.commit_spacing_closed
    say(f,' Dcommit_min = OPEN (dt alone is intentionally NOT promoted to a proof).');
end

%% E. AF7R58E structural resource theorem re-binding.
E=struct();
E.core_reviewed = source.core_current;
E.dyadic_leaf_termination = has(nH,'if((tb-ta)<=L.dt+tolMerge||!splittable)');
E.component_vector = has(nH,'std::vector<Component>comp;');
E.component_merge = has(nH,'E.comp.back().end=std::max(E.comp.back().end,raw[i].end);');
E.actual_candidate_builder = has(nH,'staticinlineCandidatebuild_candidate');
E.first_transition = has(nH,'C.seg0.push_back(t0);C.seg1.push_back(comp[0].start);');
E.last_transition = has(nH,'C.seg0.push_back(comp[j].end);C.seg1.push_back(t1);');
E.segment_relation_source_bound = E.actual_candidate_builder && E.first_transition && E.last_transition;
E.tree_component_formula = 'Cmax <= 2^(max(Kmax-1,0)), with Kmax from AF7R58E outward width recurrence';
E.segment_formula = 'S_group_max <= 2*Cmax + 1';
E.Cmax_numeric = NaN;
E.Sgroup_numeric = NaN;

say(f,'');
say(f,'[E. AF7R58E TREE / COMPONENT / SEGMENT RE-BINDING]');
print_bool(f,'reviewed af7r36_core SHA still exact',E.core_reviewed);
print_bool(f,'production dyadic terminal predicate present',E.dyadic_leaf_termination);
print_bool(f,'production connected-component storage present',E.component_vector);
print_bool(f,'production adjacent episode merge present',E.component_merge);
print_bool(f,'actual production build_candidate present',E.actual_candidate_builder);
print_bool(f,'production segment relation source-bound',E.segment_relation_source_bound);
say(f,' component theorem: %s',E.tree_component_formula);
say(f,' segment theorem:   %s',E.segment_formula);
say(f,' numeric Cmax/Sgroup are NOT selected without a closed generic lifetime + live preflight packet.');

%% F. Gmax/Smax derivability ledger.
F=struct();
F.t0_commit_current_reference_only_closed=B.current_reference_only_t0_equals_commit;
F.Tcorr_current_reference_only_closed=C.current_reference_only_lifetime_source_bound;
F.Tcorr_generic_closed=C.generic_lifetime_upper_bound_closed;
F.Dcommit_closed=D.commit_spacing_closed;
F.C_formula_closed=E.core_reviewed && E.dyadic_leaf_termination && E.segment_relation_source_bound;
F.prune_before_count_append_closed=false; % AF7R58B shadow theorem is not production install.
F.Gmax_closed=F.Tcorr_generic_closed && F.Dcommit_closed && F.prune_before_count_append_closed;
F.Smax_closed=F.Gmax_closed && F.C_formula_closed;
F.production_caps_selected=false;

say(f,'');
say(f,'[F. RESOURCE-CAP CLOSURE LEDGER]');
print_bool(f,'current REFERENCE_ONLY t0=commit chain closed',F.t0_commit_current_reference_only_closed);
print_bool(f,'current REFERENCE_ONLY finite Tcorr source bound closed',F.Tcorr_current_reference_only_closed);
print_bool(f,'generic future EXECUTION_AWARE Tcorr bound closed',F.Tcorr_generic_closed);
print_bool(f,'Dcommit_min source closed',F.Dcommit_closed);
print_bool(f,'Cmax/S_group formula source closed',F.C_formula_closed);
print_bool(f,'production prune-before-count-before-append installed/closed',F.prune_before_count_append_closed);
print_bool(f,'production Gmax closed',F.Gmax_closed);
print_bool(f,'production Smax closed',F.Smax_closed);
print_bool(f,'numeric production caps selected',F.production_caps_selected);

%% G. Exact next production contract (declaration only; no patch in this file).
NEXT=struct();
NEXT.no_hardcoded_preflight_parameters=true;
NEXT.no_hardcoded_actuator_limits=true;
NEXT.payload_mass_remains_live_estimator=true;
NEXT.require_generic_finite_validity_upper_bound=~F.Tcorr_generic_closed;
NEXT.require_explicit_commit_spacing_guard=~F.Dcommit_closed;
NEXT.require_exact_expired_prune_install=~F.prune_before_count_append_closed;
NEXT.commit_spacing_source='LIVE_PREDICTION_DT_OR_STRONGER_EXTERNAL_PREFLIGHT_CONTRACT';
NEXT.resource_formula='Gmax=ceil(Tcorr_max/Dcommit_min)+1; Smax=Gmax*(2*Cmax+1)';
NEXT.guard_order='EXACT_PRUNE_EXPIRED__COUNT_GS__ORDINARY_RETURN_IF_EXCESS__APPEND';
NEXT.ordinary_return='NO_EXCEPTION_RESOURCE_EXCESS; reason codes remain source-bound before production install';

say(f,'');
say(f,'[G. NEXT PRODUCTION CONTRACT -- NOT INSTALLED BY AF7R58G]');
say(f,' no hardcoded preflight parameters            = %d',NEXT.no_hardcoded_preflight_parameters);
say(f,' no hardcoded actuator/input limits          = %d',NEXT.no_hardcoded_actuator_limits);
say(f,' payload mL remains live-estimator authority = %d',NEXT.payload_mass_remains_live_estimator);
say(f,' require generic finite valid-until bound    = %d',NEXT.require_generic_finite_validity_upper_bound);
say(f,' require explicit commit-spacing guard       = %d',NEXT.require_explicit_commit_spacing_guard);
say(f,' require exact expired-prune production install = %d',NEXT.require_exact_expired_prune_install);
say(f,' cap theorem                                 = %s',NEXT.resource_formula);
say(f,' required order                              = %s',NEXT.guard_order);
say(f,' NOTE: later actuator/tension certificate must consume external preflight config + latest in-flight mL estimate.');

%% H. Status / output.
OUT=struct();
OUT.schema='AF7R58G_EXACT_RESOURCE_BINDING_V1';
OUT.contract=struct( ...
    'preflight_tunables_external',true, ...
    'actuator_limits_external_preflight',true, ...
    'payload_mass_live_estimator_authority',true, ...
    'production_modified',false, ...
    'sampled_safety_acceptance',false);
OUT.sources=SRC;
OUT.source_binding=source;
OUT.t0_commit_chain=B;
OUT.lifetime=C;
OUT.commit_spacing=D;
OUT.tree_segment=E;
OUT.ledger=F;
OUT.next_contract=NEXT;
OUT.production_caps_selected=false;
OUT.production_modified=false;
OUT.status='OPEN';
if source.replan_current && source.core_current && source.mex_current && ...
        F.t0_commit_current_reference_only_closed && F.Tcorr_current_reference_only_closed && ...
        F.C_formula_closed && ~F.Dcommit_closed
    OUT.status='CURRENT_PATH_BINDING_CLOSED__GENERIC_LIFETIME_AND_DCOMMIT_AND_PRODUCTION_PRUNE_OPEN';
end
if F.Gmax_closed && F.Smax_closed
    OUT.status='RESOURCE_CAP_THEOREM_CLOSED__READY_FOR_NUMERIC_PREFLIGHT_BINDING';
end
OUT.next_action='INSTALL_GENERIC_VALIDITY_BOUND__EXPLICIT_DCOMMIT_GUARD__EXACT_PRUNE_AND_ORDINARY_GS_RETURN';
OUT.required_return_files={'AF7R58G_REPORT.txt','AF7R58G_OUT.mat'};
OUT.output_directory=outdir;

say(f,'');
say(f,'[H. STATUS]');
say(f,' status                                      = %s',OUT.status);
say(f,' next action                                 = %s',OUT.next_action);
say(f,' production modified                         = %d',OUT.production_modified);
say(f,'');
say(f,'[RETURN THESE TWO FILES]');
say(f,' RETURN: %s',reportPath);
say(f,' RETURN: %s',fullfile(outdir,'AF7R58G_OUT.mat'));

save(fullfile(outdir,'AF7R58G_OUT.mat'),'OUT','-v7');
assignin('base','AF7R58G_OUT',OUT);
end

%% ------------------------------- helpers ---------------------------------
function s=getsrc(SRC,name)
idx=find(strcmp({SRC.name},name),1);
if isempty(idx)
    s=struct('name',name,'path','','present',false,'sha256','','text','');
else
    s=SRC(idx);
end
end

function y=compact(x)
if isempty(x), y=''; return; end
y=regexprep(char(x),'\s+','');
end

function tf=has(compactText,token)
tf=contains(compactText,compact(char(token)));
end

function tf=regex_present(txt,tokens)
tf=false;
for k=1:numel(tokens)
    if contains(txt,tokens{k})
        tf=true;
        return;
    end
end
end

function B=function_block(txt,signature)
% Capture a class-method function body up to the next same-indent function line.
B='';
if isempty(txt), return; end
L=regexp(txt,'\r\n|\n|\r','split');
start=[];
for i=1:numel(L)
    if contains(strtrim(L{i}),signature)
        start=i; break;
    end
end
if isempty(start), return; end
stop=numel(L);
for i=start+1:numel(L)
    t=strtrim(L{i});
    if startsWith(t,'function ')
        stop=i-1; break;
    end
end
B=strjoin(L(start:stop),sprintf('\n'));
end

function tf=explicit_spacing_guard(block)
% Require a real comparison between t_commit and last_replan_commit_time.
% Assignments / save-restore alone do not qualify.
tf=false;
if isempty(block), return; end
b=compact(block);
if ~contains(b,'last_replan_commit_time') || ~contains(b,'t_commit')
    return;
end
patterns={ ...
    't_commit-obj.last_replan_commit_time>=', ...
    't_commit-double(obj.last_replan_commit_time)>=', ...
    'double(t_commit)-obj.last_replan_commit_time>=', ...
    'double(t_commit)-double(obj.last_replan_commit_time)>=', ...
    't_commit>=obj.last_replan_commit_time+', ...
    'double(t_commit)>=double(obj.last_replan_commit_time)+', ...
    'obj.last_replan_commit_time+obj.prediction_dt<=t_commit', ...
    'double(obj.last_replan_commit_time)+double(obj.prediction_dt)<=double(t_commit)'};
for k=1:numel(patterns)
    if contains(b,patterns{k})
        tf=true; return;
    end
end
end

function print_bool(f,label,v)
say(f,' %-64s = %d',label,logical(v));
end

function say(f,fmt,varargin)
s=sprintf(fmt,varargin{:});
fprintf(1,'%s\n',s);
fprintf(f,'%s\n',s);
end

function h=file_hash(p)
h='';
if ~isfile(p), return; end
f=fopen(p,'rb');
assert(f>=0,'cannot read %s',p);
c=onCleanup(@()fclose(f)); %#ok<NASGU>
b=fread(f,Inf,'*uint8');
m=java.security.MessageDigest.getInstance('SHA-256');
m.update(b);
d=typecast(m.digest(),'uint8');
h=lower(reshape(dec2hex(d,2).',1,[]));
end
