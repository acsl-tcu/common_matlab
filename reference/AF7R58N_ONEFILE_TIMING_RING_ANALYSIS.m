function OUT = AF7R58N_ONEFILE_TIMING_RING_ANALYSIS(replanner)
% AF7R58N
% Read-only analysis of AF7R58M bounded request timing ring.
%
% Run AFTER:
%   1) AF7R58M install succeeded;
%   2) REPLANNING_SYSTEM was recreated from a clean run;
%   3) the same real scenario generated several real replan requests.
%
% No production source or object state is modified.
%
% RETURN:
%   AF7R58N_REPORT.txt
%   AF7R58N_OUT.mat

root=fileparts(mfilename('fullpath'));
outdir=tempname(root);
[ok,msg]=mkdir(outdir); assert(ok,'%s',msg);
f=fopen(fullfile(outdir,'AF7R58N_REPORT.txt'),'wt','n','UTF-8');
assert(f>=0,'[AF7R58N] cannot open report.');
cleanup=onCleanup(@()fclose(f)); %#ok<NASGU>

say(f,'====================================================================================================');
say(f,' AF7R58N | REQUEST-BY-REQUEST 4-MS TIMING-RING ANALYSIS');
say(f,'====================================================================================================');
say(f,'[AUTHORITY] readOnly=1 | productionModified=0 | noCommit=1 | noSyntheticReplan=1');
say(f,' root = %s',root);

OUT=struct();
OUT.schema='AF7R58N_TIMING_RING_ANALYSIS_V1';
OUT.production_modified=false;
OUT.status='OPEN';
OUT.output_directory=outdir;
OUT.required_return_files={'AF7R58N_REPORT.txt','AF7R58N_OUT.mat'};
OUT.error=struct();

try
    % Exact post-AF7R58M source binding.
    pR=fullfile(root,'REPLANNING_SYSTEM.m');
    pC=fullfile(root,'af7r58m_capture_timing_ring.m');
    pL=fullfile(root,'af7r58m_timing_ring_labels.m');

    assert(isfile(pR)&&isfile(pC)&&isfile(pL),'[AF7R58N] AF7R58M production install not found.');
    hR=file_hash(pR); hC=file_hash(pC); hL=file_hash(pL);

    say(f,'');
    say(f,'[A. POST-AF7R58M SOURCE BINDING]');
    say(f,' REPLANNING_SYSTEM.m               = %s',hR);
    say(f,' af7r58m_capture_timing_ring.m      = %s',hC);
    say(f,' af7r58m_timing_ring_labels.m       = %s',hL);

    OUT.source_binding=struct('REPLANNING_SYSTEM',hR,'capture_helper',hC,'label_helper',hL);

    % Live object.
    names={};
    if nargin<1||isempty(replanner)
        [replanner,names]=find_live_replanner();
    end
    if isempty(replanner)
        OUT.status='LIVE_REPLANNER_OBJECT_REQUIRED';
        OUT.live_candidates=names;
        say(f,'');
        say(f,'[B. LIVE OBJECT]');
        say(f,' unique REPLANNING_SYSTEM object not found.');
        if ~isempty(names), say(f,' candidates = %s',strjoin(names,', ')); end
        finish();
        return;
    end
    assert(isa(replanner,'REPLANNING_SYSTEM'));

    req=double(replanner.generator_request_count);
    fail=double(replanner.generator_deadline_fail_count);
    deadline=1e3*double(replanner.hard_modify_deadline_s);
    count=double(replanner.af7r58m_timing_ring_count);
    head=double(replanner.af7r58m_timing_ring_head);
    cap=double(replanner.af7r58m_timing_ring_capacity);
    wid=double(replanner.af7r58m_timing_ring_width);
    ring=double(replanner.af7r58m_timing_ring);
    labels=af7r58m_timing_ring_labels();

    say(f,'');
    say(f,'[B. LIVE RING READINESS]');
    say(f,' generator request count                 = %.0f',req);
    say(f,' deadline fail count                     = %.0f',fail);
    say(f,' hard deadline [ms]                      = %.17g',deadline);
    say(f,' ring capacity / width                   = %.0f / %.0f',cap,wid);
    say(f,' ring count / head                       = %.0f / %.0f',count,head);
    say(f,' max capture overhead [us]               = %.17g',double(replanner.af7r58m_max_capture_overhead_us));
    say(f,' last capture overhead [us]              = %.17g',double(replanner.af7r58m_last_capture_overhead_us));

    assert(isequal(size(ring),[64 32])&&cap==64&&wid==32&&numel(labels)==32, ...
        '[AF7R58N] timing ring schema mismatch.');

    if count<1
        OUT.status='TIMING_RING_EMPTY__RERUN_REAL_REQUEST_SCENARIO';
        finish();
        return;
    end

    R=chronological_rows(ring,head,count,cap);
    n=size(R,1);

    % Column aliases.
    seq=R(:,1);
    cycle_cleanup=R(:,2);
    cycle_recorded=R(:,3);
    elapsed_before=R(:,4);
    budget_remaining=R(:,5);
    context=R(:,6);
    callback=R(:,7);
    validation=R(:,8);
    critical=R(:,9);
    runtime=R(:,10);
    status_code=R(:,11);
    fail_cum=R(:,12);
    candidate_cum=R(:,13);
    valpass_cum=R(:,14);
    valfail_cum=R(:,15);
    refver=R(:,16);
    commit_cum=R(:,17);
    eval_cum=R(:,18);
    rocommit_cum=R(:,19);
    reuse_cum=R(:,20);
    rebuild_cum=R(:,21);
    fast_rebuild_cum=R(:,22);
    compact_cum=R(:,23);
    stage_episode=R(:,24);
    stage_hessian=R(:,25);
    stage_corridor=R(:,26);
    stage_qp=R(:,27);
    stage_candidate_build=R(:,28);
    stage_synthesis=R(:,29);
    stage_candidate_loop=R(:,30);
    stage_total=R(:,31);
    callback_qpc=R(:,32);

    fail_event=counter_events(fail_cum);
    candidate_event=counter_events(candidate_cum);
    valpass_event=counter_events(valpass_cum);
    valfail_event=counter_events(valfail_cum);
    commit_event=counter_events(commit_cum);
    eval_event=counter_events(eval_cum);
    reuse_event=counter_events(reuse_cum);
    rebuild_event=counter_events(rebuild_cum);
    fast_rebuild_event=counter_events(fast_rebuild_cum);
    compact_event=counter_events(compact_cum);

    deadline_by_cycle=isfinite(cycle_cleanup)&cycle_cleanup>deadline;
    deadline_by_recorded=isfinite(cycle_recorded)&cycle_recorded>deadline;

    % Derived accounting. "post" is intentionally residual, not guessed cause.
    known_pre_callback=elapsed_before+context+callback+validation;
    post_residual=cycle_cleanup-known_pre_callback;
    pre_generator=max(0,elapsed_before);
    post_after_callback=cycle_cleanup-(elapsed_before+context+callback);

    % Source timing fields may be absent on some return branches.
    native_present=isfinite(stage_total)|isfinite(stage_episode)|isfinite(stage_synthesis);

    T=struct();
    T.labels=labels;
    T.rows=R;
    T.fail_event=fail_event;
    T.deadline_by_cleanup_cycle=deadline_by_cycle;
    T.deadline_by_recorded_cycle=deadline_by_recorded;
    T.candidate_event=candidate_event;
    T.validation_pass_event=valpass_event;
    T.validation_fail_event=valfail_event;
    T.commit_event=commit_event;
    T.reference_eval_event=eval_event;
    T.packet_reuse_event=reuse_event;
    T.packet_rebuild_event=rebuild_event;
    T.fast_packet_rebuild_event=fast_rebuild_event;
    T.compact_proposal_event=compact_event;
    T.post_residual_ms=post_residual;
    T.post_after_callback_ms=post_after_callback;
    T.native_stage_present=native_present;
    OUT.ring=T;

    say(f,'');
    say(f,'[C. REQUEST-BY-REQUEST TIMING]');
    say(f,' rows retained = %d',n);
    say(f,' Columns: seq | fail | cleanup-cycle | elapsed-before | context | callback | validation | residual-after-known | status');
    for i=1:n
        say(f,' %4.0f | %d | %8.4f | %8.4f | %7.4f | %8.4f | %8.4f | %10.4f | %g', ...
            seq(i),fail_event(i)>0,cycle_cleanup(i),elapsed_before(i),context(i),callback(i), ...
            validation(i),post_residual(i),status_code(i));
    end

    % Aggregate misses vs passes.
    miss=fail_event>0 | deadline_by_cycle;
    pass=~miss & isfinite(cycle_cleanup);

    A=struct();
    A.rows=n;
    A.miss_rows=sum(miss);
    A.pass_rows=sum(pass);
    A.max_cycle_ms=safe_max(cycle_cleanup);
    A.min_cycle_ms=safe_min(cycle_cleanup);
    A.max_callback_ms=safe_max(callback);
    A.max_context_ms=safe_max(context);
    A.max_elapsed_before_ms=safe_max(elapsed_before);
    A.max_validation_ms=safe_max(validation);
    A.max_post_residual_ms=safe_max(post_residual);
    A.capture_overhead_us_max=double(replanner.af7r58m_max_capture_overhead_us);
    A.native_stage_rows=sum(native_present);

    A.miss_cycle_mean=masked_mean(cycle_cleanup,miss);
    A.pass_cycle_mean=masked_mean(cycle_cleanup,pass);
    A.miss_elapsed_before_mean=masked_mean(elapsed_before,miss);
    A.pass_elapsed_before_mean=masked_mean(elapsed_before,pass);
    A.miss_context_mean=masked_mean(context,miss);
    A.pass_context_mean=masked_mean(context,pass);
    A.miss_callback_mean=masked_mean(callback,miss);
    A.pass_callback_mean=masked_mean(callback,pass);
    A.miss_validation_mean=masked_mean(validation,miss);
    A.pass_validation_mean=masked_mean(validation,pass);
    A.miss_post_residual_mean=masked_mean(post_residual,miss);
    A.pass_post_residual_mean=masked_mean(post_residual,pass);

    % Per-stage source-native stats where actually present.
    A.stage_episode_max=safe_max(stage_episode);
    A.stage_hessian_max=safe_max(stage_hessian);
    A.stage_corridor_max=safe_max(stage_corridor);
    A.stage_qp_max=safe_max(stage_qp);
    A.stage_candidate_build_max=safe_max(stage_candidate_build);
    A.stage_synthesis_max=safe_max(stage_synthesis);
    A.stage_candidate_loop_max=safe_max(stage_candidate_loop);
    A.stage_total_max=safe_max(stage_total);
    A.callback_qpc_max=safe_max(callback_qpc);

    OUT.aggregate=A;

    say(f,'');
    say(f,'[D. MISS/PASS AGGREGATES]');
    say(f,' miss / pass rows                       = %d / %d',A.miss_rows,A.pass_rows);
    say(f,' cycle max / min [ms]                   = %.17g / %.17g',A.max_cycle_ms,A.min_cycle_ms);
    say(f,' miss/pass elapsed-before mean [ms]     = %.17g / %.17g',A.miss_elapsed_before_mean,A.pass_elapsed_before_mean);
    say(f,' miss/pass context mean [ms]            = %.17g / %.17g',A.miss_context_mean,A.pass_context_mean);
    say(f,' miss/pass callback mean [ms]           = %.17g / %.17g',A.miss_callback_mean,A.pass_callback_mean);
    say(f,' miss/pass validation mean [ms]         = %.17g / %.17g',A.miss_validation_mean,A.pass_validation_mean);
    say(f,' miss/pass post residual mean [ms]      = %.17g / %.17g',A.miss_post_residual_mean,A.pass_post_residual_mean);
    say(f,' max capture overhead [us]              = %.17g',A.capture_overhead_us_max);
    say(f,' rows with native stage timing          = %d',A.native_stage_rows);

    say(f,'');
    say(f,'[E. NATIVE STAGE MAXIMA WHERE AVAILABLE]');
    say(f,' episode / hessian / corridor [ms]      = %.17g / %.17g / %.17g', ...
        A.stage_episode_max,A.stage_hessian_max,A.stage_corridor_max);
    say(f,' qp / candidate-build [ms]              = %.17g / %.17g', ...
        A.stage_qp_max,A.stage_candidate_build_max);
    say(f,' synthesis / candidate-loop / total [ms]= %.17g / %.17g / %.17g', ...
        A.stage_synthesis_max,A.stage_candidate_loop_max,A.stage_total_max);
    say(f,' callback QPC max [ms]                  = %.17g',A.callback_qpc_max);

    % Conservative attribution.
    ATTR=struct();
    ATTR.cause_closed=false;
    ATTR.dominant_bucket='OPEN';
    ATTR.reason='';

    if A.miss_rows==0
        ATTR.reason='NO_MISS_RECORDED_IN_RING';
    else
        dE=A.miss_elapsed_before_mean-A.pass_elapsed_before_mean;
        dC=A.miss_context_mean-A.pass_context_mean;
        dB=A.miss_callback_mean-A.pass_callback_mean;
        dV=A.miss_validation_mean-A.pass_validation_mean;
        dP=A.miss_post_residual_mean-A.pass_post_residual_mean;
        delta=[dE dC dB dV dP];
        names={'ELAPSED_BEFORE','CONTEXT','CALLBACK','VALIDATION','POST_CALLBACK_OR_COMMIT_RESIDUAL'};
        finiteMask=isfinite(delta);
        if any(finiteMask)
            dd=delta;
            dd(~finiteMask)=-Inf;
            [mx,ix]=max(dd);
            ATTR.delta_miss_minus_pass_ms=delta;
            ATTR.bucket_names=names;
            ATTR.largest_delta_ms=mx;
            ATTR.dominant_bucket=names{ix};

            % Close attribution only if the largest bucket separates by >=0.25 ms
            % and there is at least one observed pass for comparison.
            if A.pass_rows>=1 && isfinite(mx) && mx>=0.25
                ATTR.cause_closed=true;
                ATTR.reason='REQUEST_LEVEL_BUCKET_SEPARATION';
            else
                ATTR.reason='INSUFFICIENT_SEPARATION_OR_NO_PASS_COMPARATOR';
            end
        else
            ATTR.reason='TIMING_BUCKETS_NONFINITE';
        end
    end

    % Strong special cases from counter events.
    ATTR.rebuild_on_miss=sum(rebuild_event(miss));
    ATTR.rebuild_on_pass=sum(rebuild_event(pass));
    ATTR.fast_rebuild_on_miss=sum(fast_rebuild_event(miss));
    ATTR.fast_rebuild_on_pass=sum(fast_rebuild_event(pass));
    ATTR.compact_native_on_miss=sum(compact_event(miss));
    ATTR.compact_native_on_pass=sum(compact_event(pass));
    ATTR.commit_on_miss=sum(commit_event(miss));
    ATTR.commit_on_pass=sum(commit_event(pass));
    ATTR.validation_fail_on_miss=sum(valfail_event(miss));
    ATTR.validation_fail_on_pass=sum(valfail_event(pass));

    OUT.attribution=ATTR;

    say(f,'');
    say(f,'[F. CONSERVATIVE ATTRIBUTION]');
    say(f,' cause closed                           = %d',ATTR.cause_closed);
    say(f,' dominant timing bucket                 = %s',ATTR.dominant_bucket);
    say(f,' reason                                 = %s',ATTR.reason);
    if isfield(ATTR,'largest_delta_ms')
        say(f,' largest miss-pass mean delta [ms]      = %.17g',ATTR.largest_delta_ms);
    end
    say(f,' packet rebuild events miss/pass        = %.0f / %.0f',ATTR.rebuild_on_miss,ATTR.rebuild_on_pass);
    say(f,' fast rebuild events miss/pass          = %.0f / %.0f',ATTR.fast_rebuild_on_miss,ATTR.fast_rebuild_on_pass);
    say(f,' compact native proposal events miss/pass= %.0f / %.0f',ATTR.compact_native_on_miss,ATTR.compact_native_on_pass);
    say(f,' commit events miss/pass                = %.0f / %.0f',ATTR.commit_on_miss,ATTR.commit_on_pass);
    say(f,' validation-fail events miss/pass       = %.0f / %.0f',ATTR.validation_fail_on_miss,ATTR.validation_fail_on_pass);
    say(f,' NOTE: no optimization is authorized solely from correlation; source path must be proven next.');

    % Closure and next action.
    if A.miss_rows==0 && n>=10
        OUT.status='CURRENT_RUN_10PLUS_REQUESTS_NO_4MS_MISS__REMOVE_DIAGNOSTIC_RING_AND_REVALIDATE';
    elseif ATTR.cause_closed
        OUT.status=['4MS_DOMINANT_BUCKET_IDENTIFIED__' ATTR.dominant_bucket '__SOURCE_PATH_OPTIMIZATION_NEXT'];
    else
        OUT.status='4MS_REQUEST_HISTORY_CAPTURED__SOURCE_PATH_CAUSE_STILL_OPEN';
    end

    say(f,'');
    say(f,'[G. STATUS]');
    say(f,' status = %s',OUT.status);
    say(f,' production modified = 0');

catch ME
    OUT.status='AUDIT_ERROR__NO_PRODUCTION_CHANGE';
    OUT.error=struct('identifier',ME.identifier,'message',ME.message,'stack',{ME.stack});
    say(f,'');
    say(f,'[AUDIT ERROR]');
    say(f,' identifier = %s',ME.identifier);
    say(f,' message    = %s',ME.message);
end

finish();

    function finish()
        save(fullfile(outdir,'AF7R58N_OUT.mat'),'OUT','-v7');
        assignin('base','AF7R58N_OUT',OUT);
        say(f,'');
        say(f,'[RETURN THESE TWO FILES]');
        say(f,' RETURN: %s',fullfile(outdir,'AF7R58N_REPORT.txt'));
        say(f,' RETURN: %s',fullfile(outdir,'AF7R58N_OUT.mat'));
        say(f,' status = %s',OUT.status);
    end
end


function R=chronological_rows(ring,head,count,cap)
count=min(max(floor(count),0),cap);
head=floor(head);
if count<cap
    idx=1:count;
else
    idx=[mod(head,cap)+1:cap 1:mod(head,cap)];
end
R=ring(idx,:);
end


function e=counter_events(c)
c=double(c(:));
e=zeros(size(c));
if isempty(c), return; end
if isfinite(c(1)), e(1)=max(0,c(1)); end
for i=2:numel(c)
    if isfinite(c(i))&&isfinite(c(i-1))
        e(i)=max(0,c(i)-c(i-1));
    end
end
end


function m=masked_mean(x,mask)
x=double(x(:)); mask=logical(mask(:));
v=x(mask & isfinite(x));
if isempty(v), m=NaN; else, m=mean(v); end
end


function x=safe_max(v)
v=double(v(:)); v=v(isfinite(v));
if isempty(v), x=NaN; else, x=max(v); end
end


function x=safe_min(v)
v=double(v(:)); v=v(isfinite(v));
if isempty(v), x=NaN; else, x=min(v); end
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
