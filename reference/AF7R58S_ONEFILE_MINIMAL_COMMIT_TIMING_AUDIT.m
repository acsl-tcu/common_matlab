function OUT = AF7R58S_ONEFILE_MINIMAL_COMMIT_TIMING_AUDIT(replanner)
% AF7R58S
% Read-only analysis of AF7R58Q-R5 minimal commit-timing ring.
%
% Run AFTER one clean run_flight_headless using the AF7R58Q-R5 production
% source. Keep the live REPLANNING_SYSTEM object in the base workspace.
%
% No production source or object state is modified.
%
% RETURN:
%   AF7R58S_REPORT.txt
%   AF7R58S_OUT.mat

root=fileparts(mfilename('fullpath'));
outdir=tempname(root);
[ok,msg]=mkdir(outdir); assert(ok,'%s',msg);
f=fopen(fullfile(outdir,'AF7R58S_REPORT.txt'),'wt','n','UTF-8');
assert(f>=0,'[AF7R58S] cannot open report.');
cleanup=onCleanup(@()fclose(f)); %#ok<NASGU>

say(f,'====================================================================================================');
say(f,' AF7R58S | MINIMAL REFERENCE-ONLY COMMIT TIMING AUDIT');
say(f,'====================================================================================================');
say(f,'[AUTHORITY] readOnly=1 | productionModified=0 | noCommit=1 | noSyntheticReplan=1');
say(f,' root = %s',root);

OUT=struct();
OUT.schema='AF7R58S_MINIMAL_COMMIT_TIMING_AUDIT_V1';
OUT.production_modified=false;
OUT.status='OPEN';
OUT.output_directory=outdir;
OUT.error=struct();

try
    pR=fullfile(root,'REPLANNING_SYSTEM.m');
    assert(isfile(pR),'[AF7R58S] REPLANNING_SYSTEM.m missing.');
    hR=file_hash(pR);
    expected='1474c591fe2227f461eb43c2853d61909e63e9bef9bf6a40a851956c1c02e440';

    say(f,'');
    say(f,'[A. EXACT AF7R58Q-R5 SOURCE BINDING]');
    say(f,' REPLANNING_SYSTEM SHA exact = %d | %s',strcmp(hR,expected),hR);
    assert(strcmp(hR,expected), ...
        '[AF7R58S] production source is not AF7R58Q-R5; no timing conclusion.');

    if nargin<1||isempty(replanner)
        [replanner,names]=find_live_replanner();
    else
        names={};
    end
    if isempty(replanner)
        OUT.status='LIVE_REPLANNER_OBJECT_REQUIRED';
        OUT.live_candidates=names;
        say(f,'');
        say(f,'[B. LIVE OBJECT]');
        say(f,' unique REPLANNING_SYSTEM object not found.');
        finish();
        return;
    end
    assert(isa(replanner,'REPLANNING_SYSTEM'));

    req=double(replanner.generator_request_count);
    fail=double(replanner.generator_deadline_fail_count);
    deadline=1e3*double(replanner.hard_modify_deadline_s);

    ring=double(replanner.af7r58q_min_ring);
    count=double(replanner.af7r58q_min_ring_count);
    head=double(replanner.af7r58q_min_ring_head);
    cap=double(replanner.af7r58q_min_ring_capacity);
    wid=double(replanner.af7r58q_min_ring_width);

    say(f,'');
    say(f,'[B. LIVE R5 TIMING RING]');
    say(f,' generator request count               = %.0f',req);
    say(f,' global deadline fail count            = %.0f',fail);
    say(f,' hard deadline [ms]                    = %.17g',deadline);
    say(f,' ring capacity / width                 = %.0f / %.0f',cap,wid);
    say(f,' ring count / head                     = %.0f / %.0f',count,head);
    say(f,' last capture overhead [us]            = %.17g',double(replanner.af7r58q_min_last_capture_overhead_us));
    say(f,' max capture overhead [us]             = %.17g',double(replanner.af7r58q_min_max_capture_overhead_us));

    assert(cap==64&&wid==9&&isequal(size(ring),[64 9]), ...
        '[AF7R58S] AF7R58Q-R5 ring schema mismatch.');

    if count<1
        OUT.status='R5_RING_EMPTY__REAL_COMMIT_PATH_NOT_CAPTURED';
        finish();
        return;
    end

    R=chronological_rows(ring,head,count,cap);
    n=size(R,1);
    seq=R(:,1);

    % Identify latest request epoch ending at tail using current request count.
    epochStart=NaN;
    if req>=1 && req<=n
        s=n-req+1;
        if isequal(seq(s:n),(1:req).')
            epochStart=s;
        end
    end
    if ~isfinite(epochStart)
        % Fallback: latest monotone 1..N suffix.
        for s=n:-1:1
            q=seq(s:n);
            if isequal(q,(1:numel(q)).')
                epochStart=s;
            else
                break;
            end
        end
    end
    assert(isfinite(epochStart),'[AF7R58S] cannot isolate latest request epoch.');

    preRows=epochStart-1;
    E=R(epochStart:end,:);

    % R5 ring columns:
    % 1 seq, 2 total, 3 spacing, 4 resource, 5 handles, 6 other,
    % 7 active version, 8 commit count snapshot, 9 deadline fail snapshot.
    total=E(:,2);
    spacing=E(:,3);
    resource=E(:,4);
    handles=E(:,5);
    other=E(:,6);
    version=E(:,7);
    commitSnap=E(:,8);
    failSnap=E(:,9);

    if preRows>=1
        baseCommit=R(preRows,8);
        baseFail=R(preRows,9);
    else
        baseCommit=0;
        baseFail=0;
    end
    commitEvent=counter_events_with_baseline(commitSnap,baseCommit);
    failEvent=counter_events_with_baseline(failSnap,baseFail);

    OUT.latest_epoch=struct();
    OUT.latest_epoch.pre_rows=preRows;
    OUT.latest_epoch.rows=E;
    OUT.latest_epoch.commit_event=commitEvent;
    OUT.latest_epoch.fail_event=failEvent;

    say(f,'');
    say(f,'[C. LATEST REAL REQUEST EPOCH]');
    say(f,' rows before latest epoch (prime/warmup) = %d',preRows);
    say(f,' latest epoch rows                       = %d',size(E,1));
    say(f,' request sequence exact 1:N              = %d',isequal(E(:,1),(1:size(E,1)).'));
    say(f,' commit increments in latest epoch       = %.0f',sum(commitEvent));
    say(f,' deadline-fail increments latest epoch   = %.0f',sum(failEvent));

    say(f,'');
    say(f,'[D. REQUEST-BY-REQUEST COMMIT TIMING]');
    say(f,' seq | total | spacing | resource | handles | OTHER | fail+ | commit+ | version');
    for i=1:size(E,1)
        say(f,' %3.0f | %8.4f | %7.4f | %8.4f | %7.4f | %8.4f | %d | %d | %.0f', ...
            E(i,1),total(i),spacing(i),resource(i),handles(i),other(i), ...
            failEvent(i)>0,commitEvent(i)>0,version(i));
    end

    A=struct();
    A.rows=size(E,1);
    A.total_max=safe_max(total);
    A.spacing_max=safe_max(spacing);
    A.resource_max=safe_max(resource);
    A.handles_max=safe_max(handles);
    A.other_max=safe_max(other);
    A.total_mean=safe_mean(total);
    A.spacing_mean=safe_mean(spacing);
    A.resource_mean=safe_mean(resource);
    A.handles_mean=safe_mean(handles);
    A.other_mean=safe_mean(other);
    A.fail_count=sum(failEvent);
    A.commit_count=sum(commitEvent);

    % Dominance ratios only where total is finite/positive.
    valid=isfinite(total)&total>0;
    ratios=nan(size(E,1),4);
    ratios(valid,1)=spacing(valid)./total(valid);
    ratios(valid,2)=resource(valid)./total(valid);
    ratios(valid,3)=handles(valid)./total(valid);
    ratios(valid,4)=other(valid)./total(valid);
    A.max_fraction_spacing=safe_max(ratios(:,1));
    A.max_fraction_resource=safe_max(ratios(:,2));
    A.max_fraction_handles=safe_max(ratios(:,3));
    A.max_fraction_other=safe_max(ratios(:,4));

    OUT.aggregate=A;

    say(f,'');
    say(f,'[E. AGGREGATE TIMING]');
    say(f,' total max / mean [ms]                 = %.17g / %.17g',A.total_max,A.total_mean);
    say(f,' spacing max / mean [ms]               = %.17g / %.17g',A.spacing_max,A.spacing_mean);
    say(f,' resource max / mean [ms]              = %.17g / %.17g',A.resource_max,A.resource_mean);
    say(f,' handles max / mean [ms]               = %.17g / %.17g',A.handles_max,A.handles_mean);
    say(f,' OTHER max / mean [ms]                 = %.17g / %.17g',A.other_max,A.other_mean);
    say(f,' max stage fraction: spacing/resource/handles/OTHER = %.4f / %.4f / %.4f / %.4f', ...
        A.max_fraction_spacing,A.max_fraction_resource,A.max_fraction_handles,A.max_fraction_other);

    % Conservative conclusion. No optimization is authorized merely from a
    % single timing sample; classify the dominant bucket over the epoch.
    bucketMeans=[A.spacing_mean,A.resource_mean,A.handles_mean,A.other_mean];
    bucketNames={'SPACING','RESOURCE_PREPARE','CANONICAL_HANDLES','OTHER_COMMIT'};
    finiteMask=isfinite(bucketMeans);
    C=struct();
    C.dominant_bucket='OPEN';
    C.dominant_mean_ms=NaN;
    C.source_split_closed=false;
    if any(finiteMask)
        z=bucketMeans; z(~finiteMask)=-Inf;
        [mx,ix]=max(z);
        C.dominant_bucket=bucketNames{ix};
        C.dominant_mean_ms=mx;
        C.source_split_closed=true;
    end

    if strcmp(C.dominant_bucket,'OTHER_COMMIT')
        OUT.status='COMMIT_COST_DOMINATED_BY_OTHER__SOURCE_SPLIT_OTHER_NEXT';
    elseif strcmp(C.dominant_bucket,'RESOURCE_PREPARE')
        OUT.status='COMMIT_COST_DOMINATED_BY_RESOURCE_PREPARE__OPTIMIZATION_AUDIT_NEXT';
    elseif strcmp(C.dominant_bucket,'CANONICAL_HANDLES')
        OUT.status='COMMIT_COST_DOMINATED_BY_CANONICAL_HANDLES__OPTIMIZATION_AUDIT_NEXT';
    elseif strcmp(C.dominant_bucket,'SPACING')
        OUT.status='COMMIT_COST_DOMINATED_BY_SPACING__OPTIMIZATION_AUDIT_NEXT';
    else
        OUT.status='COMMIT_TIMING_CAPTURED__DOMINANT_BUCKET_OPEN';
    end
    OUT.conclusion=C;

    say(f,'');
    say(f,'[F. CONSERVATIVE CONCLUSION]');
    say(f,' dominant bucket                       = %s',C.dominant_bucket);
    say(f,' dominant bucket mean [ms]             = %.17g',C.dominant_mean_ms);
    say(f,' source split closed                   = %d',C.source_split_closed);
    say(f,' NOTE: instrumented timing is diagnostic only.');
    say(f,'       Final 4-ms validation requires removal of diagnostic instrumentation.');
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
        save(fullfile(outdir,'AF7R58S_OUT.mat'),'OUT','-v7');
        assignin('base','AF7R58S_OUT',OUT);
        say(f,'');
        say(f,'[RETURN THESE TWO FILES]');
        say(f,' RETURN: %s',fullfile(outdir,'AF7R58S_REPORT.txt'));
        say(f,' RETURN: %s',fullfile(outdir,'AF7R58S_OUT.mat'));
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


function e=counter_events_with_baseline(c,b)
c=double(c(:));
e=zeros(size(c));
prev=double(b);
for i=1:numel(c)
    if isfinite(c(i))&&isfinite(prev)
        e(i)=max(0,c(i)-prev);
    end
    prev=c(i);
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


function x=safe_max(v)
v=double(v(:)); v=v(isfinite(v));
if isempty(v), x=NaN; else, x=max(v); end
end


function x=safe_mean(v)
v=double(v(:)); v=v(isfinite(v));
if isempty(v), x=NaN; else, x=mean(v); end
end


function h=file_hash(p)
f=fopen(p,'rb'); assert(f>=0,'cannot read %s',p);
cl=onCleanup(@()fclose(f)); %#ok<NASGU>
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
