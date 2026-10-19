function OUT = AF7R58J_ONEFILE_LIVE_RESOURCE_EQUIV_4MS_AUDIT(replanner)
% AF7R58J
% Read-only post-install audit for AF7R58I-R3.
%
% Usage:
%   AF7R58J_ONEFILE_LIVE_RESOURCE_EQUIV_4MS_AUDIT
%     -> auto-detect exactly one REPLANNING_SYSTEM object in base workspace.
%
%   AF7R58J_ONEFILE_LIVE_RESOURCE_EQUIV_4MS_AUDIT(replanner)
%     -> use the explicitly supplied clean post-install object.
%
% IMPORTANT:
%   Recreate REPLANNING_SYSTEM from a clean setup/run AFTER AF7R58I-R3 before
%   using this audit.  This file NEVER modifies production or commits a plan.
%
% RETURN:
%   AF7R58J_REPORT.txt
%   AF7R58J_OUT.mat

root=fileparts(mfilename('fullpath'));
outdir=tempname(root);
[ok,msg]=mkdir(outdir); assert(ok,'%s',msg);

f=fopen(fullfile(outdir,'AF7R58J_REPORT.txt'),'wt','n','UTF-8');
assert(f>=0,'[AF7R58J] cannot open report.');
cleanup=onCleanup(@()fclose(f)); %#ok<NASGU>

say(f,'====================================================================================================');
say(f,' AF7R58J | LIVE RESOURCE + CANONICAL EQUIVALENCE + OBSERVED 4MS AUDIT');
say(f,'====================================================================================================');
say(f,'[AUTHORITY] readOnly=1 | productionModified=0 | noCommit=1 | noFlightInjection=1 | sampledSafetyAcceptance=0');
say(f,' root = %s',root);

OUT=struct();
OUT.schema='AF7R58J_LIVE_RESOURCE_EQUIV_4MS_AUDIT_V1';
OUT.production_modified=false;
OUT.status='OPEN';
OUT.output_directory=outdir;
OUT.required_return_files={'AF7R58J_REPORT.txt','AF7R58J_OUT.mat'};
OUT.source_binding=struct();
OUT.live_object=struct();
OUT.resource_contract=struct();
OUT.canonical=struct();
OUT.runtime_4ms=struct();
OUT.error=struct();

try
    % ------------------------------------------------------------------
    % A. Exact post-install source binding from AF7R58I-R3.
    % ------------------------------------------------------------------
    expected=struct();
    expected.REPLANNING_SYSTEM='503c4146f10892fdb45004c986bd6359f63e6ce5c59bf3f2ef61de971585badc';
    expected.af7r33_make_batch_model_from_proposal='0e293b36a77ccc4d426eb0e1e9460a949eb4acb4be81dc8dcab539828c8847ed';
    expected.af7r58i_nextup_nonnegative='c2f6a28d1e05301bdafe6eddd200c70366875b4d0b4d7cadf1d8d37677c04116';
    expected.af7r58i_nextdown_positive='4624d94cd8f13f35706f9a97dc8c5f45fd7e2fa3ff114ac35b2cc9f88f78c444';
    expected.af7r58i_resource_contract='772e49a8007bd955cb262b1db513f65c1270f87384b5f12694ee8774ae458392';
    expected.af7r58i_commit_spacing_guard='ba92a9bb05e5cea6916f904646ff416a8379ff5ed9ed4424baf362eacb43054e';
    expected.af7r58i_prune_batch_model_exact='5309875bdfbb7e228921cf1182afb759037a6a3bcd79a0b88d6e76466e4c6db4';
    expected.af7r58i_prepare_batch_model='92f093168f1df95661c5e7f2e75005fe439406eb3ec22e53fd1f25d6bdfb8a15';
    expected.af7r58i_make_canonical_eval_handles='1c9bf7407b9c2482c5f6d2676479966b808995c4d10ee2871a620faba55a82f8';
    expected.af7r58i_decasteljau='aab46ffbeb52e74e40d0298ba35d87bc7b13902f21e541fce24b99aa85b01a04';
    expected.af7r58i_correction_derivatives='472c61619d64401264479a429f63b177f76bcb7e14b0cd5f39b6fe5f5f95ec35';
    expected.af7r58i_eval_canonical_reference='4f8ecd8576983d638b5fbbc71384d333fc90d4d9e4787c6ea694e94dc2d0ca39';

    names=fieldnames(expected);
    bind=struct();
    allBind=true;

    say(f,'');
    say(f,'[A. EXACT AF7R58I-R3 POST-INSTALL SOURCE BINDING]');
    for k=1:numel(names)
        nm=names{k};
        if strcmp(nm,'REPLANNING_SYSTEM')
            file=[nm '.m'];
        else
            file=[nm '.m'];
        end
        p=fullfile(root,file);
        present=isfile(p);
        h='';
        if present, h=file_hash(p); end
        match=present && strcmp(h,expected.(nm));
        bind.(nm)=struct('present',present,'sha256',h,'expected',expected.(nm),'match',match);
        allBind=allBind && match;
        say(f,' %-42s present=%d | SHA-match=%d | %s',file,present,match,h);
    end
    OUT.source_binding=bind;
    OUT.source_binding_all_exact=allBind;

    % Source-level invariants of the installed resource layer.
    Rtxt=fileread(fullfile(root,'REPLANNING_SYSTEM.m'));
    cR=compact_ws(Rtxt);
    src=struct();
    src.spacing_guard_reference_only=contains(cR, ...
        'af7r58i_commit_spacing_guard(obj,double(t_commit))');
    src.canonical_prepare=contains(cR, ...
        'af7r58i_prepare_batch_model(obj,P,targetVersion,double(t_commit))');
    src.canonical_handles=contains(cR, ...
        'af7r58i_make_canonical_eval_handles(obj,batchModel)');
    src.canonical_model_stored=contains(cR,'R.batch_flat_model=batchModel;');
    src.no_recursive_commit_materializer=~contains(extract_between(Rtxt, ...
        'function [ok,reason] = try_commit_reference_only_proposal', ...
        'function R = evaluate_reference_only_proposal_certificate'), ...
        'af7r46_materialize_candidate_evaluators(P)');
    src.mL_not_consumed_by_resource_helper= ...
        ~contains(fileread(fullfile(root,'af7r58i_resource_contract.m')),'get_load_mass') && ...
        ~contains(fileread(fullfile(root,'af7r58i_resource_contract.m')),'mL');
    src.all=all(structfun(@logical,src));

    say(f,'');
    say(f,'[B. INSTALLED RESOURCE-LAYER SOURCE INVARIANTS]');
    say(f,' explicit spacing guard in commit path              = %d',src.spacing_guard_reference_only);
    say(f,' canonical prune/count/append preparation installed = %d',src.canonical_prepare);
    say(f,' canonical nonrecursive handles installed           = %d',src.canonical_handles);
    say(f,' canonical bounded model stored as active history   = %d',src.canonical_model_stored);
    say(f,' old recursive commit materializer absent           = %d',src.no_recursive_commit_materializer);
    say(f,' resource helper does not consume/freeze mL         = %d',src.mL_not_consumed_by_resource_helper);
    say(f,' source invariant conjunction                        = %d',src.all);
    OUT.installed_source_invariants=src;

    % ------------------------------------------------------------------
    % C. Resolve live clean object.
    % ------------------------------------------------------------------
    autoNames={};
    if nargin<1 || isempty(replanner)
        [replanner,autoNames]=find_live_replanner();
    end

    if isempty(replanner)
        say(f,'');
        say(f,'[C. LIVE OBJECT]');
        say(f,' no unique REPLANNING_SYSTEM object found in base workspace.');
        if ~isempty(autoNames)
            say(f,' candidates = %s',strjoin(autoNames,', '));
        end
        OUT.live_object=struct('found',false,'candidate_names',{autoNames});
        OUT.status='SOURCE_INSTALL_CLOSED__CLEAN_LIVE_REPLANNER_OBJECT_REQUIRED';
        save_and_return();
        return;
    end

    assert(isa(replanner,'REPLANNING_SYSTEM'), ...
        '[AF7R58J] supplied object is not REPLANNING_SYSTEM.');

    objInfo=struct();
    objInfo.found=true;
    objInfo.class=class(replanner);
    objInfo.active_reference_mode=char(replanner.active_reference_mode);
    objInfo.active_reference_version=double(replanner.active_reference_version);
    objInfo.prediction_dt=double(replanner.prediction_dt);
    objInfo.flight_reference_time_origin=double(replanner.flight_reference_time_origin);
    objInfo.nominal_table_tend=double(replanner.nominal_table_tend);
    objInfo.last_replan_commit_time=double(replanner.last_replan_commit_time);
    objInfo.generator_request_count=double(replanner.generator_request_count);
    objInfo.generator_deadline_fail_count=double(replanner.generator_deadline_fail_count);
    objInfo.generator_last_cycle_critical_ms=double(replanner.generator_last_cycle_critical_ms);
    objInfo.hard_modify_deadline_s=double(replanner.hard_modify_deadline_s);
    OUT.live_object=objInfo;

    say(f,'');
    say(f,'[C. CLEAN LIVE OBJECT]');
    say(f,' active mode/version                   = %s / %.0f', ...
        objInfo.active_reference_mode,objInfo.active_reference_version);
    say(f,' prediction dt [s]                    = %.17g',objInfo.prediction_dt);
    say(f,' flight reference origin [s]          = %.17g',objInfo.flight_reference_time_origin);
    say(f,' nominal table tend [s]               = %.17g',objInfo.nominal_table_tend);
    say(f,' last commit time [s]                 = %.17g',objInfo.last_replan_commit_time);
    say(f,' generator request count              = %.0f',objInfo.generator_request_count);
    say(f,' recorded deadline fail count         = %.0f',objInfo.generator_deadline_fail_count);
    say(f,' last hard-cycle critical [ms]        = %.17g',objInfo.generator_last_cycle_critical_ms);
    say(f,' configured hard deadline [ms]        = %.17g',1e3*objInfo.hard_modify_deadline_s);

    % ------------------------------------------------------------------
    % D. Direct live resource theorem packet.
    % ------------------------------------------------------------------
    RC=af7r58i_resource_contract(replanner);
    OUT.resource_contract=RC;

    say(f,'');
    say(f,'[D. DIRECT LIVE RESOURCE CONTRACT]');
    say(f,' closed                               = %d',logical(RC.closed));
    say(f,' reason                               = %s',char(RC.reason));
    if RC.closed
        say(f,' flight domain                         = [%.17g, %.17g] s',RC.flight_t0,RC.reference_tend);
        say(f,' Tcorr_max                             = %.17g s',RC.Tcorr_max);
        say(f,' Dcommit_min                           = %.17g s',RC.Dcommit_min);
        say(f,' Kmax                                  = %.0f',RC.Kmax);
        say(f,' Cmax                                  = %.0f',RC.Cmax);
        say(f,' Sgroup_max                            = %.0f',RC.Sgroup_max);
        say(f,' Gmax                                  = %.0f',RC.Gmax);
        say(f,' Smax                                  = %.0f',RC.Smax);
    end

    % Guard consistency: read-only hypothetical calls only.
    guard=struct('tested',false,'first_commit_ok',false, ...
        'boundary_ok',false,'below_boundary_rejected',false);
    if RC.closed
        guard.tested=true;
        last=double(replanner.last_replan_commit_time);
        if isnan(last)
            [gok,gd,gn,gr]=af7r58i_commit_spacing_guard( ...
                replanner,max(RC.flight_t0,0));
            guard.first_commit_ok=logical(gok);
            guard.Dmin=gd; guard.nextAllowed=gn; guard.reason=gr;
        else
            [gok,gd,gn,gr]=af7r58i_commit_spacing_guard( ...
                replanner,last+double(replanner.prediction_dt));
            guard.boundary_ok=logical(gok);
            guard.Dmin=gd; guard.nextAllowed=gn; guard.reason=gr;
            if isfinite(gn)
                bad=nextdown_any(gn);
                [gBad,~,~,rBad]=af7r58i_commit_spacing_guard(replanner,bad);
                guard.below_boundary_rejected=~logical(gBad);
                guard.below_boundary_reason=rBad;
                guard.bad_test_time=bad;
            end
        end
    end
    OUT.commit_spacing_readonly_test=guard;

    say(f,'');
    say(f,'[E. READ-ONLY COMMIT-SPACING BOUNDARY TEST]');
    say(f,' tested                               = %d',guard.tested);
    if isnan(double(replanner.last_replan_commit_time))
        say(f,' first commit accepted                 = %d',guard.first_commit_ok);
    else
        say(f,' represented dt-boundary accepted      = %d',guard.boundary_ok);
        say(f,' next representable below rejected     = %d',guard.below_boundary_rejected);
    end

    % ------------------------------------------------------------------
    % F. Canonical active-resource state + equivalence regression.
    % ------------------------------------------------------------------
    CAN=struct();
    CAN.applicable=false;
    CAN.structural_closed=false;
    CAN.bitwise_eval=false;
    CAN.bitwise_flat=false;
    CAN.max_abs_eval=NaN;
    CAN.max_abs_flat=NaN;
    CAN.group_count=0;
    CAN.segment_count=0;
    CAN.within_Gmax=false;
    CAN.within_Smax=false;
    CAN.active_handle_is_canonical=false;
    CAN.active_flat_handle_is_canonical=false;

    mode=upper(char(replanner.active_reference_mode));
    if strcmp(mode,'NOMINAL')
        CAN.applicable=false;
        CAN.structural_closed=true;
        CAN.reason='NOMINAL_ACTIVE_REFERENCE__NO_REPAIRED_HISTORY';
    elseif strcmp(mode,'REPAIRED')
        CAN.applicable=true;
        AR=replanner.active_repaired_reference;
        assert(isstruct(AR)&&isfield(AR,'batch_flat_model')&& ...
            isstruct(AR.batch_flat_model), ...
            '[AF7R58J] REPAIRED active reference lacks canonical batch model.');
        model=AR.batch_flat_model;
        G=numel(model.corrections);
        S=0;
        for g=1:G
            S=S+numel(model.corrections(g).control_points);
        end
        CAN.group_count=G;
        CAN.segment_count=S;
        if RC.closed
            CAN.within_Gmax=G<=RC.Gmax;
            CAN.within_Smax=S<=RC.Smax;
        end

        fi=functions(AR.eval_fn);
        ff=functions(AR.flat_eval_fn);
        CAN.active_handle_is_canonical=contains(char(fi.function),'af7r58i_eval_canonical_reference');
        CAN.active_flat_handle_is_canonical=contains(char(ff.function),'af7r58i_eval_canonical_reference');

        [freshEval,freshFlat]=af7r58i_make_canonical_eval_handles(replanner,model);

        T=critical_times(model,RC);
        maxE=0; maxF=0; bitE=true; bitF=true;
        for k=1:numel(T)
            t=T(k);
            a=AR.eval_fn(t); b=freshEval(t);
            c=AR.flat_eval_fn(t); d=freshFlat(t);
            a=double(a(:)); b=double(b(:)); c=double(c(:)); d=double(d(:));
            maxE=max(maxE,max(abs(a-b)));
            maxF=max(maxF,max(abs(c-d)));
            bitE=bitE && bitwise_equal_double(a,b);
            bitF=bitF && bitwise_equal_double(c,d);
        end
        CAN.test_times=T;
        CAN.bitwise_eval=bitE;
        CAN.bitwise_flat=bitF;
        CAN.max_abs_eval=maxE;
        CAN.max_abs_flat=maxF;
        CAN.structural_closed=CAN.within_Gmax&&CAN.within_Smax&& ...
            CAN.active_handle_is_canonical&&CAN.active_flat_handle_is_canonical;
        CAN.reason='REPAIRED_CANONICAL_STATE_AUDITED';
    else
        CAN.reason='UNKNOWN_ACTIVE_REFERENCE_MODE';
    end
    OUT.canonical=CAN;

    say(f,'');
    say(f,'[F. ACTIVE CANONICAL RESOURCE STATE / EQUIVALENCE]');
    say(f,' applicable repaired state                 = %d',CAN.applicable);
    say(f,' structural closed                         = %d',CAN.structural_closed);
    say(f,' active group count                        = %.0f',CAN.group_count);
    say(f,' active segment count                      = %.0f',CAN.segment_count);
    if CAN.applicable
        say(f,' group count <= live Gmax                  = %d',CAN.within_Gmax);
        say(f,' segment count <= live Smax                = %d',CAN.within_Smax);
        say(f,' active eval handle canonical              = %d',CAN.active_handle_is_canonical);
        say(f,' active flat handle canonical              = %d',CAN.active_flat_handle_is_canonical);
        say(f,' fresh-canonical eval bitwise equal        = %d',CAN.bitwise_eval);
        say(f,' fresh-canonical flat bitwise equal        = %d',CAN.bitwise_flat);
        say(f,' max eval abs difference                   = %.17g',CAN.max_abs_eval);
        say(f,' max flat abs difference                   = %.17g',CAN.max_abs_flat);
    else
        say(f,' reason                                    = %s',CAN.reason);
    end
    say(f,' NOTE: numerical equality is a regression check; safety acceptance remains theorem/certificate based.');

    % ------------------------------------------------------------------
    % G. Observed real-request 4ms ledger. No synthetic timing is promoted.
    % ------------------------------------------------------------------
    RT=struct();
    RT.request_count=double(replanner.generator_request_count);
    RT.deadline_fail_count=double(replanner.generator_deadline_fail_count);
    RT.last_cycle_ms=double(replanner.generator_last_cycle_critical_ms);
    RT.deadline_ms=1e3*double(replanner.hard_modify_deadline_s);
    RT.has_real_request=RT.request_count>0;
    RT.no_recorded_deadline_miss=RT.has_real_request && RT.deadline_fail_count==0;
    RT.last_request_within_deadline=RT.has_real_request && ...
        isfinite(RT.last_cycle_ms) && RT.last_cycle_ms<=RT.deadline_ms;
    RT.observed_run_pass=RT.no_recorded_deadline_miss && RT.last_request_within_deadline;
    RT.formal_future_wcet_closed=false;

    LS=replanner.get_last_replan_status();
    RT.last_replan_status=LS;
    if isstruct(LS)&&isfield(LS,'cycle_ms')
        RT.last_status_cycle_ms=double(LS.cycle_ms);
    else
        RT.last_status_cycle_ms=NaN;
    end
    if isstruct(LS)&&isfield(LS,'reason')
        RT.last_reason=char(LS.reason);
    else
        RT.last_reason='';
    end
    OUT.runtime_4ms=RT;

    say(f,'');
    say(f,'[G. OBSERVED REAL-REQUEST 4MS LEDGER]');
    say(f,' real request observed                    = %d',RT.has_real_request);
    say(f,' request count                            = %.0f',RT.request_count);
    say(f,' deadline fail count                      = %.0f',RT.deadline_fail_count);
    say(f,' no recorded deadline miss                = %d',RT.no_recorded_deadline_miss);
    say(f,' last critical cycle [ms]                 = %.17g',RT.last_cycle_ms);
    say(f,' hard deadline [ms]                       = %.17g',RT.deadline_ms);
    say(f,' last request within deadline             = %d',RT.last_request_within_deadline);
    say(f,' observed-run 4ms pass                    = %d',RT.observed_run_pass);
    say(f,' formal future WCET theorem closed        = %d',RT.formal_future_wcet_closed);
    if ~isempty(RT.last_reason)
        say(f,' last replan reason                        = %s',RT.last_reason);
    end
    say(f,' NOTE: absence of observed misses does not by itself prove future WCET.');

    % ------------------------------------------------------------------
    % H. Closure ledger.
    % ------------------------------------------------------------------
    liveCore=allBind && src.all && RC.closed && CAN.structural_closed;
    observed4=RT.observed_run_pass;

    OUT.ledger=struct( ...
        'post_install_source_exact',allBind, ...
        'installed_source_invariants',src.all, ...
        'live_resource_contract_closed',logical(RC.closed), ...
        'canonical_active_state_closed',CAN.structural_closed, ...
        'observed_run_4ms_pass',observed4, ...
        'formal_future_wcet_closed',false, ...
        'tension_actuator_live_mL_closed',false, ...
        'failure_backup_closed',false);

    if liveCore && observed4
        OUT.status='LIVE_RESOURCE_AND_CANONICAL_CLOSED__OBSERVED_4MS_PASS__WCET_AND_TOWING_FEASIBILITY_NEXT';
    elseif liveCore
        OUT.status='LIVE_RESOURCE_AND_CANONICAL_CLOSED__REAL_REQUEST_4MS_EVIDENCE_OPEN';
    else
        OUT.status='OPEN_LIVE_RESOURCE_OR_CANONICAL_BINDING';
    end

    say(f,'');
    say(f,'[H. CLOSURE LEDGER]');
    say(f,' post-install source exact                 = %d',allBind);
    say(f,' installed source invariants               = %d',src.all);
    say(f,' live resource contract closed             = %d',logical(RC.closed));
    say(f,' canonical active-state resource closed    = %d',CAN.structural_closed);
    say(f,' observed-run 4ms pass                     = %d',observed4);
    say(f,' formal future WCET closed                 = 0');
    say(f,' tension/thrust/moment + latest live mL    = 0');
    say(f,' final failure backup                      = 0');
    say(f,' status                                    = %s',OUT.status);

catch ME
    OUT.status='AUDIT_ERROR__NO_PRODUCTION_CHANGE';
    OUT.error=struct('identifier',ME.identifier,'message',ME.message,'stack',{ME.stack});
    say(f,'');
    say(f,'[AUDIT ERROR]');
    say(f,' identifier = %s',ME.identifier);
    say(f,' message    = %s',ME.message);
    say(f,' production modified = 0');
end

save_and_return();

    function save_and_return()
        OUT.production_modified=false;
        save(fullfile(outdir,'AF7R58J_OUT.mat'),'OUT','-v7');
        assignin('base','AF7R58J_OUT',OUT);
        say(f,'');
        say(f,'[RETURN THESE TWO FILES]');
        say(f,' RETURN: %s',fullfile(outdir,'AF7R58J_REPORT.txt'));
        say(f,' RETURN: %s',fullfile(outdir,'AF7R58J_OUT.mat'));
        say(f,' status = %s',OUT.status);
    end
end


function [obj,names]=find_live_replanner()
obj=[];
names={};
W=evalin('base','whos');
for k=1:numel(W)
    try
        v=evalin('base',W(k).name);
    catch
        continue;
    end
    if isa(v,'REPLANNING_SYSTEM')
        names{end+1}=W(k).name; %#ok<AGROW>
    end
end
if numel(names)==1
    obj=evalin('base',names{1});
end
end


function T=critical_times(model,RC)
T=[];
if isstruct(RC)&&isfield(RC,'closed')&&logical(RC.closed)
    T=[T double(RC.flight_t0) double(RC.reference_tend)];
end
if isfield(model,'corrections')
    for g=1:numel(model.corrections)
        C=model.corrections(g);
        T=[T double(C.t0) double(C.t1)]; %#ok<AGROW>
        s0=double(C.segment_t0(:).');
        s1=double(C.segment_t1(:).');
        T=[T s0 s1 0.5*(s0+s1)]; %#ok<AGROW>
    end
end
T=T(isfinite(T));
T=unique(T);
end


function tf=bitwise_equal_double(a,b)
if ~isequal(size(a),size(b)), tf=false; return; end
tf=isequal(typecast(a(:),'uint64'),typecast(b(:),'uint64'));
end


function y=nextdown_any(x)
x=double(x);
if isnan(x)||isinf(x)
    y=x;
    return;
end
if x==0
    y=-realmin('double');
    return;
end
u=typecast(x,'uint64');
if x>0
    y=typecast(u-uint64(1),'double');
else
    y=typecast(u+uint64(1),'double');
end
end


function t=extract_between(s,startMarker,endMarker)
a=strfind(s,startMarker);
if isempty(a), t=''; return; end
a=a(1);
b=strfind(s,endMarker);
b=b(b>a);
if isempty(b), t=s(a:end); return; end
t=s(a:b(1)-1);
end


function c=compact_ws(s)
c=regexprep(s,'\s+','');
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


function say(f,fmt,varargin)
s=sprintf(fmt,varargin{:});
fprintf(1,'%s\n',s);
fprintf(f,'%s\n',s);
end
