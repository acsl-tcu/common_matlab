function OUT = AF7R58K_ONEFILE_POSTINSTALL_LIVE_AUDIT(replanner)
% AF7R58K
% Robust read-only post-AF7R58I-R3 live audit.
%
% Run AFTER the normal setup has built the nominal table.
% For the project's normal headless path this means run_flight_headless has
% reached the "nominal table ready" initialization stage.  A real replan
% request is required only for the observed 4-ms ledger.
%
% Usage:
%   AF7R58K_ONEFILE_POSTINSTALL_LIVE_AUDIT
%   AF7R58K_ONEFILE_POSTINSTALL_LIVE_AUDIT(replanner_instance)
%
% No production source is modified. No candidate is committed.
%
% RETURN:
%   AF7R58K_REPORT.txt
%   AF7R58K_OUT.mat

root=fileparts(mfilename('fullpath'));
outdir=tempname(root);
[ok,msg]=mkdir(outdir); assert(ok,'%s',msg);

f=fopen(fullfile(outdir,'AF7R58K_REPORT.txt'),'wt','n','UTF-8');
assert(f>=0,'[AF7R58K] cannot open report.');
cleanup=onCleanup(@()fclose(f)); %#ok<NASGU>

say(f,'====================================================================================================');
say(f,' AF7R58K | ROBUST POST-INSTALL LIVE RESOURCE + 4MS AUDIT');
say(f,'====================================================================================================');
say(f,'[AUTHORITY] readOnly=1 | productionModified=0 | noCommit=1 | noSyntheticSafetyAcceptance=1');
say(f,' root = %s',root);

OUT=struct();
OUT.schema='AF7R58K_POSTINSTALL_LIVE_AUDIT_V1';
OUT.production_modified=false;
OUT.status='OPEN';
OUT.output_directory=outdir;
OUT.required_return_files={'AF7R58K_REPORT.txt','AF7R58K_OUT.mat'};
OUT.error=struct();

try
    % ------------------------------------------------------------------
    % A. Exact installed source binding.
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
    BIND=struct();
    allBind=true;
    say(f,'');
    say(f,'[A. EXACT AF7R58I-R3 SOURCE BINDING]');
    for k=1:numel(names)
        nm=names{k};
        p=fullfile(root,[nm '.m']);
        present=isfile(p);
        h='';
        if present, h=file_hash(p); end
        match=present&&strcmp(h,expected.(nm));
        BIND.(nm)=struct('present',present,'sha256',h,'expected',expected.(nm),'match',match);
        allBind=allBind&&match;
        say(f,' %-42s present=%d | SHA-match=%d | %s',[nm '.m'],present,match,h);
    end
    OUT.source_binding=BIND;
    OUT.source_binding_all_exact=allBind;

    % ------------------------------------------------------------------
    % B. Robust source invariant audit.
    % Strip comments and continuations before semantic token checks.
    % ------------------------------------------------------------------
    Rraw=fileread(fullfile(root,'REPLANNING_SYSTEM.m'));
    Rcode=compact_code(Rraw);
    Cprep=compact_code(fileread(fullfile(root,'af7r58i_prepare_batch_model.m')));
    Cres=compact_code(fileread(fullfile(root,'af7r58i_resource_contract.m')));

    S=struct();
    S.spacing_guard=contains(Rcode,'af7r58i_commit_spacing_guard(obj,double(t_commit))');
    S.prepare_call=contains(Rcode,'af7r58i_prepare_batch_model(obj,P,targetVersion,double(t_commit))');
    S.canonical_handles=contains(Rcode,'af7r58i_make_canonical_eval_handles(obj,batchModel)');
    S.canonical_store=contains(Rcode,'R.batch_flat_model=batchModel;');
    S.prepare_has_prune=contains(Cprep,'af7r58i_prune_batch_model_exact(model,pt0)');
    S.prepare_counts_G=contains(Cprep,'G1=G0+1;');
    S.prepare_counts_S=contains(Cprep,'S1=S0+Snew;');
    S.prepare_ordinary_G_return=contains(Cprep,'reason=''AF7R58I_MAX_REPAIR_GROUPS'';return;');
    S.prepare_ordinary_S_return=contains(Cprep,'reason=''AF7R58I_MAX_REPAIR_SEGMENTS'';return;');
    S.resource_no_mL_code=isempty(regexp(Cres,'(^|[^A-Za-z0-9_])mL([^A-Za-z0-9_]|$)','once'));
    S.resource_no_thrust_code=isempty(regexp(lower(Cres),'(^|[^a-z0-9_])thrust([^a-z0-9_]|$)','once'));
    S.resource_no_moment_code=isempty(regexp(lower(Cres),'(^|[^a-z0-9_])moment([^a-z0-9_]|$)','once'));
    S.all=all(structfun(@logical,S));

    say(f,'');
    say(f,'[B. ROBUST INSTALLED SOURCE INVARIANTS]');
    say(f,' explicit commit-spacing guard                   = %d',S.spacing_guard);
    say(f,' canonical prepare call                          = %d',S.prepare_call);
    say(f,' canonical nonrecursive handles                  = %d',S.canonical_handles);
    say(f,' bounded canonical model stored                  = %d',S.canonical_store);
    say(f,' prepare performs exact expired prune            = %d',S.prepare_has_prune);
    say(f,' prepare counts G/S before append                = %d / %d',S.prepare_counts_G,S.prepare_counts_S);
    say(f,' G/S excess use ordinary return                  = %d / %d',S.prepare_ordinary_G_return,S.prepare_ordinary_S_return);
    say(f,' resource code does not consume mL              = %d',S.resource_no_mL_code);
    say(f,' resource code does not consume thrust/moment   = %d / %d',S.resource_no_thrust_code,S.resource_no_moment_code);
    say(f,' installed invariant conjunction                 = %d',S.all);
    OUT.source_invariants=S;

    % ------------------------------------------------------------------
    % C. Resolve live object.
    % ------------------------------------------------------------------
    baseNames={};
    if nargin<1||isempty(replanner)
        [replanner,baseNames]=find_live_replanner();
    end

    if isempty(replanner)
        OUT.live=struct('found',false,'candidate_names',{baseNames});
        OUT.status='SOURCE_LAYER_CLOSED__LIVE_REPLANNER_REQUIRED';
        say(f,'');
        say(f,'[C. LIVE OBJECT]');
        say(f,' unique REPLANNING_SYSTEM object not found.');
        if ~isempty(baseNames), say(f,' candidates = %s',strjoin(baseNames,', ')); end
        finish();
        return;
    end
    assert(isa(replanner,'REPLANNING_SYSTEM'));

    L=struct();
    L.found=true;
    L.mode=char(replanner.active_reference_mode);
    L.version=double(replanner.active_reference_version);
    L.dt=double(replanner.prediction_dt);
    L.flight_t0=double(replanner.flight_reference_time_origin);
    L.nominal_tend=double(replanner.nominal_table_tend);
    L.last_commit=double(replanner.last_replan_commit_time);
    L.request_count=double(replanner.generator_request_count);
    L.deadline_fail_count=double(replanner.generator_deadline_fail_count);
    L.last_cycle_ms=double(replanner.generator_last_cycle_critical_ms);
    L.deadline_ms=1e3*double(replanner.hard_modify_deadline_s);
    L.nominal_ready=isfinite(L.nominal_tend)&&isfinite(L.flight_t0)&&L.nominal_tend>L.flight_t0;
    OUT.live=L;

    say(f,'');
    say(f,'[C. LIVE OBJECT READINESS]');
    say(f,' active mode/version                  = %s / %.0f',L.mode,L.version);
    say(f,' prediction dt [s]                   = %.17g',L.dt);
    say(f,' flight origin [s]                   = %.17g',L.flight_t0);
    say(f,' nominal table tend [s]              = %.17g',L.nominal_tend);
    say(f,' nominal table ready                 = %d',L.nominal_ready);
    say(f,' real replan request count           = %.0f',L.request_count);

    if ~L.nominal_ready
        OUT.resource=struct('closed',false,'reason','NORMAL_PREFLIGHT_NOMINAL_TABLE_NOT_READY');
        OUT.status='SOURCE_LAYER_CLOSED__NORMAL_PREFLIGHT_NOMINAL_TABLE_NOT_READY';
        say(f,'');
        say(f,'[D. LIVE RESOURCE CONTRACT]');
        say(f,' NOT EVALUATED AS FAILURE: object is before nominal-table-ready stage.');
        say(f,' Run the normal setup/headless path until nominal table ready, then rerun AF7R58K.');
        finish();
        return;
    end

    % ------------------------------------------------------------------
    % D. Live resource theorem.
    % ------------------------------------------------------------------
    RC=af7r58i_resource_contract(replanner);
    OUT.resource=RC;

    say(f,'');
    say(f,'[D. LIVE RESOURCE CONTRACT]');
    say(f,' closed                              = %d',logical(RC.closed));
    say(f,' reason                              = %s',char(RC.reason));
    if RC.closed
        say(f,' finite flight domain                = [%.17g, %.17g] s',RC.flight_t0,RC.reference_tend);
        say(f,' Tcorr_max                           = %.17g s',RC.Tcorr_max);
        say(f,' Dcommit_min                         = %.17g s',RC.Dcommit_min);
        say(f,' Kmax / Cmax                         = %.0f / %.0f',RC.Kmax,RC.Cmax);
        say(f,' Sgroup_max                          = %.0f',RC.Sgroup_max);
        say(f,' Gmax / Smax                         = %.0f / %.0f',RC.Gmax,RC.Smax);
    end

    % Commit-spacing read-only test.
    G=struct();
    G.tested=false;
    if RC.closed
        G.tested=true;
        last=double(replanner.last_replan_commit_time);
        if isnan(last)
            [ok1,Dmin,nxt,why]=af7r58i_commit_spacing_guard(replanner,RC.flight_t0);
            G.first_commit_ok=logical(ok1);
            G.Dmin=Dmin; G.nextAllowed=nxt; G.reason=why;
        else
            boundary=last+double(replanner.prediction_dt);
            [ok1,Dmin,nxt,why]=af7r58i_commit_spacing_guard(replanner,boundary);
            bad=nextdown_any((last+double(replanner.prediction_dt))-64*eps(max([1 abs(RC.flight_t0) abs(RC.reference_tend) abs(RC.dt)])));
            [ok0,~,~,why0]=af7r58i_commit_spacing_guard(replanner,bad);
            G.boundary_ok=logical(ok1);
            G.below_boundary_rejected=~logical(ok0);
            G.Dmin=Dmin; G.nextAllowed=nxt; G.reason=why; G.below_reason=why0;
        end
    end
    OUT.spacing_test=G;

    say(f,'');
    say(f,'[E. COMMIT-SPACING READ-ONLY TEST]');
    say(f,' tested                              = %d',G.tested);
    if G.tested
        if isfield(G,'first_commit_ok')
            say(f,' first commit accepted                = %d',G.first_commit_ok);
        else
            say(f,' represented boundary accepted        = %d',G.boundary_ok);
            say(f,' just-below boundary rejected         = %d',G.below_boundary_rejected);
        end
    end

    % ------------------------------------------------------------------
    % F. Active canonical state.
    % ------------------------------------------------------------------
    C=struct('mode',upper(L.mode),'closed',false,'groups',0,'segments',0, ...
        'within_Gmax',false,'within_Smax',false,'eval_bitwise',false, ...
        'flat_bitwise',false,'max_eval_abs',NaN,'max_flat_abs',NaN,'reason','');

    if strcmp(C.mode,'NOMINAL')
        C.closed=true;
        C.within_Gmax=true;
        C.within_Smax=true;
        C.reason='NOMINAL_ZERO_REPAIR_HISTORY';
    elseif strcmp(C.mode,'REPAIRED')
        AR=replanner.active_repaired_reference;
        assert(isstruct(AR)&&isfield(AR,'batch_flat_model')&&isstruct(AR.batch_flat_model), ...
            '[AF7R58K] repaired state missing canonical batch model.');
        M=AR.batch_flat_model;
        C.groups=numel(M.corrections);
        for g=1:C.groups
            C.segments=C.segments+numel(M.corrections(g).control_points);
        end
        C.within_Gmax=RC.closed&&C.groups<=RC.Gmax;
        C.within_Smax=RC.closed&&C.segments<=RC.Smax;

        [e2,f2]=af7r58i_make_canonical_eval_handles(replanner,M);
        T=critical_times(M,RC);
        bitE=true; bitF=true; me=0; mf=0;
        for k=1:numel(T)
            a=double(AR.eval_fn(T(k))); b=double(e2(T(k)));
            c=double(AR.flat_eval_fn(T(k))); d=double(f2(T(k)));
            a=a(:);b=b(:);c=c(:);d=d(:);
            bitE=bitE&&bitwise_equal_double(a,b);
            bitF=bitF&&bitwise_equal_double(c,d);
            me=max(me,max(abs(a-b)));
            mf=max(mf,max(abs(c-d)));
        end
        C.eval_bitwise=bitE;
        C.flat_bitwise=bitF;
        C.max_eval_abs=me;
        C.max_flat_abs=mf;
        C.closed=C.within_Gmax&&C.within_Smax&&bitE&&bitF;
        C.reason='REPAIRED_CANONICAL_DIRECT_REBUILD_CHECK';
        C.test_times=T;
    else
        C.reason='UNKNOWN_ACTIVE_REFERENCE_MODE';
    end
    OUT.canonical=C;

    say(f,'');
    say(f,'[F. ACTIVE CANONICAL STATE]');
    say(f,' mode                                = %s',C.mode);
    say(f,' resource/equivalence closed         = %d',C.closed);
    say(f,' groups / segments                   = %.0f / %.0f',C.groups,C.segments);
    say(f,' groups<=Gmax / segments<=Smax       = %d / %d',C.within_Gmax,C.within_Smax);
    if strcmp(C.mode,'REPAIRED')
        say(f,' eval / flat bitwise equality         = %d / %d',C.eval_bitwise,C.flat_bitwise);
        say(f,' max eval / flat abs diff             = %.17g / %.17g',C.max_eval_abs,C.max_flat_abs);
    end

    % ------------------------------------------------------------------
    % G. Real-request 4-ms evidence.
    % ------------------------------------------------------------------
    RT=struct();
    RT.request_count=L.request_count;
    RT.deadline_fail_count=L.deadline_fail_count;
    RT.last_cycle_ms=L.last_cycle_ms;
    RT.deadline_ms=L.deadline_ms;
    RT.real_request_observed=L.request_count>0;
    RT.no_recorded_miss=RT.real_request_observed&&L.deadline_fail_count==0;
    RT.last_cycle_pass=RT.real_request_observed&&isfinite(L.last_cycle_ms)&&L.last_cycle_ms<=L.deadline_ms;
    RT.observed_pass=RT.no_recorded_miss&&RT.last_cycle_pass;
    RT.formal_future_wcet_closed=false;
    OUT.runtime_4ms=RT;

    say(f,'');
    say(f,'[G. REAL-REQUEST 4MS EVIDENCE]');
    say(f,' request observed                     = %d',RT.real_request_observed);
    say(f,' request count                        = %.0f',RT.request_count);
    say(f,' deadline fail count                  = %.0f',RT.deadline_fail_count);
    say(f,' last cycle [ms]                      = %.17g',RT.last_cycle_ms);
    say(f,' hard deadline [ms]                   = %.17g',RT.deadline_ms);
    say(f,' observed-run pass                    = %d',RT.observed_pass);
    say(f,' formal future WCET theorem closed    = 0');
    say(f,' NOTE: observed timing is evidence, not a mathematical future WCET theorem.');

    % ------------------------------------------------------------------
    % H. Failure-backup contract reservation.
    % No backup is installed here. This records the required later interface.
    % ------------------------------------------------------------------
    FB=struct();
    FB.required=true;
    FB.default_enabled=true;
    FB.explicit_flag_required=true;
    FB.flag_scope='FAILURE_OR_NO_SOLUTION_FALLBACK_ONLY';
    FB.off_means='DIAGNOSTIC_PURE_REPLANNER_NO_FALLBACK';
    FB.must_not_disable='NORMAL_CERTIFICATES_RESOURCE_GATES_SUCCESSFUL_REPLAN_SAFETY';
    FB.installed_now=false;
    OUT.failure_backup_contract=FB;

    say(f,'');
    say(f,'[H. RESERVED FAILURE-BACKUP SWITCH CONTRACT]');
    say(f,' later backup required                         = 1');
    say(f,' default enabled                               = 1');
    say(f,' explicit ON/OFF flag required                 = 1');
    say(f,' OFF scope = failure/no-solution fallback only');
    say(f,' OFF must NOT bypass normal safety/resource certificates');
    say(f,' backup installed by AF7R58K                   = 0');

    coreClosed=allBind&&S.all&&logical(RC.closed)&&C.closed;
    if coreClosed&&RT.observed_pass
        OUT.status='LIVE_RESOURCE_CANONICAL_CLOSED__OBSERVED_4MS_PASS__TOWING_FEASIBILITY_NEXT';
    elseif coreClosed
        OUT.status='LIVE_RESOURCE_CANONICAL_CLOSED__REAL_REPLAN_4MS_EVIDENCE_OPEN';
    else
        OUT.status='OPEN_POSTINSTALL_LIVE_BINDING';
    end

    say(f,'');
    say(f,'[I. CLOSURE LEDGER]');
    say(f,' exact installed source binding       = %d',allBind);
    say(f,' robust source invariants              = %d',S.all);
    say(f,' live resource theorem                = %d',logical(RC.closed));
    say(f,' active canonical state               = %d',C.closed);
    say(f,' observed real-request 4ms            = %d',RT.observed_pass);
    say(f,' future formal WCET                   = 0');
    say(f,' tension/thrust/moment + live mL      = 0');
    say(f,' failure backup implementation        = 0');
    say(f,' status                               = %s',OUT.status);

catch ME
    OUT.status='AUDIT_ERROR__NO_PRODUCTION_CHANGE';
    OUT.error=struct('identifier',ME.identifier,'message',ME.message,'stack',{ME.stack});
    say(f,'');
    say(f,'[AUDIT ERROR]');
    say(f,' identifier = %s',ME.identifier);
    say(f,' message    = %s',ME.message);
    say(f,' production modified = 0');
end

finish();

    function finish()
        OUT.production_modified=false;
        save(fullfile(outdir,'AF7R58K_OUT.mat'),'OUT','-v7');
        assignin('base','AF7R58K_OUT',OUT);
        say(f,'');
        say(f,'[RETURN THESE TWO FILES]');
        say(f,' RETURN: %s',fullfile(outdir,'AF7R58K_REPORT.txt'));
        say(f,' RETURN: %s',fullfile(outdir,'AF7R58K_OUT.mat'));
        say(f,' status = %s',OUT.status);
    end
end


function [obj,names]=find_live_replanner()
obj=[]; names={};
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


function s=compact_code(s)
% Remove MATLAB comments and continuation tokens before whitespace compaction.
lines=regexp(s,'\r\n|\n|\r','split');
out=cell(size(lines));
for k=1:numel(lines)
    q=lines{k};
    % Conservative comment removal: percent inside a quoted literal is rare
    % in the resource-layer source; preserve quoted strings by scanning.
    inString=false; cut=numel(q)+1; i=1;
    while i<=numel(q)
        if q(i)==''''
            if inString && i<numel(q) && q(i+1)==''''
                i=i+2; continue;
            end
            inString=~inString;
        elseif q(i)=='%' && ~inString
            cut=i; break;
        end
        i=i+1;
    end
    q=q(1:cut-1);
    q=strrep(q,'...','');
    out{k}=q;
end
s=strjoin(out,newline);
s=regexprep(s,'\s+','');
end


function T=critical_times(model,RC)
T=[];
if isstruct(RC)&&isfield(RC,'closed')&&logical(RC.closed)
    T=[double(RC.flight_t0) double(RC.reference_tend)];
end
for g=1:numel(model.corrections)
    C=model.corrections(g);
    s0=double(C.segment_t0(:).');
    s1=double(C.segment_t1(:).');
    T=[T double(C.t0) double(C.t1) s0 s1 0.5*(s0+s1)]; %#ok<AGROW>
end
T=unique(T(isfinite(T)));
end


function tf=bitwise_equal_double(a,b)
tf=isequal(size(a),size(b))&& ...
    isequal(typecast(double(a(:)),'uint64'),typecast(double(b(:)),'uint64'));
end


function y=nextdown_any(x)
x=double(x);
if isnan(x)||isinf(x), y=x; return; end
if x==0, y=-realmin('double'); return; end
u=typecast(x,'uint64');
if x>0
    y=typecast(u-uint64(1),'double');
else
    y=typecast(u+uint64(1),'double');
end
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
