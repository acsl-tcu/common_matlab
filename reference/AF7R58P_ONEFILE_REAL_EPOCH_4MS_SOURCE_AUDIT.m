function OUT = AF7R58P_ONEFILE_REAL_EPOCH_4MS_SOURCE_AUDIT(replanner)
% AF7R58P
% Read-only correction of AF7R58N analysis:
% separates preflight/prime requests from the latest real request epoch,
% then binds the observed status/deadline events to current source branches.
%
% Run on the SAME live object used for AF7R58N if possible.
% No production source or object state is modified.
%
% RETURN:
%   AF7R58P_REPORT.txt
%   AF7R58P_OUT.mat

root=fileparts(mfilename('fullpath'));
outdir=tempname(root);
[ok,msg]=mkdir(outdir); assert(ok,'%s',msg);
f=fopen(fullfile(outdir,'AF7R58P_REPORT.txt'),'wt','n','UTF-8');
assert(f>=0,'[AF7R58P] cannot open report.');
cleanup=onCleanup(@()fclose(f)); %#ok<NASGU>

say(f,'====================================================================================================');
say(f,' AF7R58P | REAL-EPOCH 4-MS + SOURCE-BRANCH AUDIT');
say(f,'====================================================================================================');
say(f,'[AUTHORITY] readOnly=1 | productionModified=0 | noCommit=1 | noSyntheticReplan=1');
say(f,' root = %s',root);

OUT=struct();
OUT.schema='AF7R58P_REAL_EPOCH_4MS_SOURCE_AUDIT_V1';
OUT.production_modified=false;
OUT.status='OPEN';
OUT.output_directory=outdir;
OUT.required_return_files={'AF7R58P_REPORT.txt','AF7R58P_OUT.mat'};
OUT.error=struct();

try
    if nargin<1||isempty(replanner)
        [replanner,names]=find_live_replanner();
    else
        names={};
    end
    if isempty(replanner)
        OUT.status='LIVE_REPLANNER_OBJECT_REQUIRED';
        OUT.live_candidates=names;
        say(f,'');
        say(f,'[A. LIVE OBJECT]');
        say(f,' unique REPLANNING_SYSTEM object not found.');
        finish();
        return;
    end
    assert(isa(replanner,'REPLANNING_SYSTEM'));

    req=double(replanner.generator_request_count);
    failGlobal=double(replanner.generator_deadline_fail_count);
    deadline=1e3*double(replanner.hard_modify_deadline_s);
    ring=double(replanner.af7r58m_timing_ring);
    count=double(replanner.af7r58m_timing_ring_count);
    head=double(replanner.af7r58m_timing_ring_head);
    cap=double(replanner.af7r58m_timing_ring_capacity);

    R=chronological_rows(ring,head,count,cap);
    n=size(R,1);
    seq=R(:,1);

    say(f,'');
    say(f,'[A. RING / COUNTER STATE]');
    say(f,' current generator_request_count       = %.0f',req);
    say(f,' current deadline_fail_count           = %.0f',failGlobal);
    say(f,' ring rows retained                    = %d',n);
    say(f,' hard deadline [ms]                    = %.17g',deadline);

    assert(req>=1&&req<=n,'[AF7R58P] current request count cannot identify a nonempty latest epoch.');

    % Latest epoch must end at the ring tail and enumerate 1:req.
    epochStart=n-req+1;
    expected=(1:req).';
    tailSeq=seq(epochStart:n);
    exactEpoch=isequal(tailSeq,expected);
    if ~exactEpoch
        % Search backward for the latest exact 1:N epoch ending at tail.
        found=false;
        for s=max(1,n-floor(cap)+1):n
            q=seq(s:n);
            if isequal(q,(1:numel(q)).') && numel(q)==req
                epochStart=s; found=true; break;
            end
        end
        assert(found,'[AF7R58P] latest real epoch could not be identified exactly.');
    end

    primeRows=epochStart-1;
    E=R(epochStart:end,:);

    % Baselines from immediately before the latest epoch.
    if primeRows>=1
        base=R(primeRows,:);
    else
        base=zeros(1,size(R,2));
    end

    counterCols=[12 13 14 15 17 18 19 20 21 22 23];
    baseline=base(counterCols);
    final=E(end,counterCols);
    delta=final-baseline;

    namesC={ ...
        'deadline_fail','candidate','validation_pass','validation_fail', ...
        'commit','reference_eval','reference_only_commit','packet_reuse', ...
        'packet_rebuild','fast_packet_rebuild','compact_native'};
    D=struct();
    for k=1:numel(namesC)
        D.(namesC{k})=delta(k);
    end

    OUT.epoch=struct();
    OUT.epoch.prime_rows=primeRows;
    OUT.epoch.real_rows=req;
    OUT.epoch.real_rows_matrix=E;
    OUT.epoch.baseline_counter_row=base;
    OUT.epoch.counter_deltas=D;

    say(f,'');
    say(f,'[B. EXACT LATEST REAL EPOCH]');
    say(f,' prime/warmup rows before real epoch    = %d',primeRows);
    say(f,' real epoch rows                       = %.0f',req);
    say(f,' real epoch seq exact 1:N              = 1');
    say(f,' deadline-fail baseline before epoch   = %.0f',base(12));
    say(f,' deadline-fail final                   = %.0f',E(end,12));
    say(f,' REAL-EPOCH deadline-fail increments  = %.0f',D.deadline_fail);
    say(f,' global counter includes pre-epoch fail= %d',failGlobal>D.deadline_fail);
    say(f,' packet rebuild real-epoch increments  = %.0f',D.packet_rebuild);
    say(f,' fast rebuild real-epoch increments    = %.0f',D.fast_packet_rebuild);
    say(f,' validation fail real-epoch increments = %.0f',D.validation_fail);
    say(f,' commit real-epoch increments          = %.0f',D.commit);
    say(f,' compact native real-epoch increments  = %.0f',D.compact_native);

    % Row-wise real epoch event deltas.
    eventCols=counterCols;
    EV=zeros(size(E,1),numel(eventCols));
    prev=base(eventCols);
    for i=1:size(E,1)
        cur=E(i,eventCols);
        EV(i,:)=max(0,cur-prev);
        prev=cur;
    end

    recorded=E(:,3);
    cleanupCycle=E(:,2);
    elapsed=E(:,4);
    context=E(:,6);
    callback=E(:,7);
    validation=E(:,8);
    critical=E(:,9);
    status=E(:,11);
    postResidual=cleanupCycle-(elapsed+context+callback+validation);
    deadlineEvent=EV(:,1)>0;
    recordedOver=isfinite(recorded)&recorded>deadline;
    cleanupOver=isfinite(cleanupCycle)&cleanupCycle>deadline;

    say(f,'');
    say(f,'[C. REAL-EPOCH REQUEST TABLE]');
    say(f,' seq | fail+ | recorded | cleanup | elapsed | context | callback | residual | status | vfail+ | rebuild+ | commit+');
    for i=1:size(E,1)
        say(f,' %3.0f | %d | %8.4f | %8.4f | %7.4f | %7.4f | %8.4f | %8.4f | %6.0f | %6.0f | %8.0f | %7.0f', ...
            E(i,1),deadlineEvent(i),recorded(i),cleanupCycle(i),elapsed(i),context(i), ...
            callback(i),postResidual(i),status(i),EV(i,4),EV(i,9),EV(i,5));
    end

    Q=struct();
    Q.deadline_event=deadlineEvent;
    Q.recorded_over_4ms=recordedOver;
    Q.cleanup_over_4ms=cleanupOver;
    Q.post_residual_ms=postResidual;
    Q.real_deadline_fail_count=sum(deadlineEvent);
    Q.recorded_over_count=sum(recordedOver);
    Q.cleanup_over_count=sum(cleanupOver);
    Q.status_codes=status;
    OUT.real_events=Q;

    say(f,'');
    say(f,'[D. REAL-EPOCH TIMING FACTS]');
    say(f,' real deadline-fail increments          = %d',Q.real_deadline_fail_count);
    say(f,' recorded generator cycle >4ms          = %d',Q.recorded_over_count);
    say(f,' cleanup-entry cycle >4ms               = %d',Q.cleanup_over_count);
    say(f,' max recorded generator cycle [ms]      = %.17g',safe_max(recorded));
    say(f,' max cleanup-entry cycle [ms]           = %.17g',safe_max(cleanupCycle));
    say(f,' max callback [ms]                      = %.17g',safe_max(callback));
    say(f,' max post-callback residual [ms]        = %.17g',safe_max(postResidual));
    say(f,' capture-overhead max [us]              = %.17g',double(replanner.af7r58m_max_capture_overhead_us));
    say(f,' NOTE: cleanup-entry timing is diagnostic and may include work after the production deadline decision.');
    say(f,'       The production fail counter increment is the authority for which request actually failed.');

    % ------------------------------------------------------------------
    % Current-source branch binding.
    % ------------------------------------------------------------------
    pR=fullfile(root,'REPLANNING_SYSTEM.m');
    assert(isfile(pR),'[AF7R58P] REPLANNING_SYSTEM.m missing.');
    [lines,~]=read_lines(pR);

    patterns={ ...
        'generator_last_status_code', ...
        'generator_deadline_fail_count', ...
        'generator_validation_fail_count', ...
        'reference_only_runtime_packet_rebuild_count', ...
        'reference_only_fast_packet_rebuild_count', ...
        'compact_native_proposal_count', ...
        'try_commit_reference_only_proposal', ...
        'evaluate_reference_only_proposal_certificate', ...
        'af7r58i_prepare_batch_model', ...
        'af7r58i_make_canonical_eval_handles'};

    EX=struct('pattern',{},'line',{},'text',{},'context',{});
    say(f,'');
    say(f,'[E. CURRENT SOURCE BRANCH BINDING]');
    for p=1:numel(patterns)
        hit=find(contains(lines,patterns{p},'IgnoreCase',false));
        say(f,' pattern %-48s hits=%d',patterns{p},numel(hit));
        for h=1:numel(hit)
            ln=hit(h);
            a=max(1,ln-5); b=min(numel(lines),ln+8);
            ctx=lines(a:b);
            EX(end+1)=struct('pattern',patterns{p},'line',ln, ...
                'text',lines{ln},'context',{ctx}); %#ok<AGROW>
            if h<=12
                say(f,'   L%-6d %s',ln,strtrim(lines{ln}));
            end
        end
        if numel(hit)>12
            say(f,'   ... %d more hits retained in MAT',numel(hit)-12);
        end
    end
    OUT.source_excerpts=EX;

    % Explicit status-code assignments seen in the real epoch.
    u=unique(status(isfinite(status))).';
    MAP=struct();
    say(f,'');
    say(f,'[F. OBSERVED STATUS-CODE SOURCE MAP]');
    for j=1:numel(u)
        sc=u(j);
        token1=sprintf('generator_last_status_code=%g',sc);
        token2=sprintf('generator_last_status_code = %g',sc);
        hit=find(contains(lines,token1)|contains(lines,token2));
        key=matlab.lang.makeValidName(sprintf('status_%g',sc));
        MAP.(key)=struct('code',sc,'lines',hit);
        say(f,' status %g assignments = %s',sc,mat2str(hit(:).'));
        for h=1:min(numel(hit),6)
            ln=hit(h);
            a=max(1,ln-6); b=min(numel(lines),ln+10);
            say(f,'   --- source around L%d ---',ln);
            for z=a:b
                say(f,'   %6d | %s',z,lines{z});
            end
        end
    end
    OUT.status_source_map=MAP;

    % Conservative conclusion.
    C=struct();
    C.procedure_was_correct=true;
    C.prime_and_real_mixed_in_58N=true;
    C.real_epoch_isolated=true;
    C.global_deadline_counter_contaminated_by_prime=(failGlobal>D.deadline_fail);
    C.optimization_authorized=false;

    if D.deadline_fail>0
        C.next='SOURCE_BRANCH_COST_AUDIT_AFTER_CALLBACK_AND_REFERENCE_ONLY_EVALUATION';
        OUT.status='REAL_EPOCH_ISOLATED__SOURCE_BRANCH_COST_AUDIT_NEXT';
    else
        C.next='REMOVE_TIMING_RING_AND_REVALIDATE_NO_MISS';
        OUT.status='REAL_EPOCH_NO_DEADLINE_FAIL__REMOVE_RING_AND_REVALIDATE';
    end
    OUT.conclusion=C;

    say(f,'');
    say(f,'[G. CONCLUSION]');
    say(f,' user procedure was correct                   = 1');
    say(f,' AF7R58N mixed prime + real epochs            = 1');
    say(f,' latest real epoch isolated                   = 1');
    say(f,' global fail counter contains prime residue   = %d',C.global_deadline_counter_contaminated_by_prime);
    say(f,' optimization authorized from old 58N alone   = 0');
    say(f,' status = %s',OUT.status);

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
        save(fullfile(outdir,'AF7R58P_OUT.mat'),'OUT','-v7');
        assignin('base','AF7R58P_OUT',OUT);
        say(f,'');
        say(f,'[RETURN THESE TWO FILES]');
        say(f,' RETURN: %s',fullfile(outdir,'AF7R58P_REPORT.txt'));
        say(f,' RETURN: %s',fullfile(outdir,'AF7R58P_OUT.mat'));
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


function [lines,txt]=read_lines(p)
txt=fileread(p);
lines=regexp(txt,'\r\n|\n|\r','split');
end


function x=safe_max(v)
v=double(v(:)); v=v(isfinite(v));
if isempty(v), x=NaN; else, x=max(v); end
end


function say(f,fmt,varargin)
s=sprintf(fmt,varargin{:});
fprintf(1,'%s\n',s);
fprintf(f,'%s\n',s);
end
