function OUT = AF7R58S_R2_ONEFILE_JOINED_COMMIT_TIMING_AUDIT(replanner)
% AF7R58S-R2
% Read-only joined analysis of:
%   (1) AF7R58M full request timing ring, and
%   (2) AF7R58Q-R5 sparse commit-function timing ring.
%
% Why R2 exists:
%   The R5 commit ring records only requests that actually enter
%   try_commit_reference_only_proposal(). Therefore commit-ring seq values
%   need not be a dense 1:N series. AF7R58S incorrectly assumed density.
%
% R2 uses the AF7R58M request ring as epoch authority, isolates the latest
% real request epoch there, then joins R5 commit timings by request seq.
%
% No production source or live state is modified.
%
% RETURN:
%   AF7R58S_R2_REPORT.txt
%   AF7R58S_R2_OUT.mat

root=fileparts(mfilename('fullpath'));
outdir=tempname(root);
[ok,msg]=mkdir(outdir); assert(ok,'%s',msg);
f=fopen(fullfile(outdir,'AF7R58S_R2_REPORT.txt'),'wt','n','UTF-8');
assert(f>=0,'[AF7R58S-R2] cannot open report.');
cleanup=onCleanup(@()fclose(f)); %#ok<NASGU>

say(f,'====================================================================================================');
say(f,' AF7R58S-R2 | JOINED REQUEST + SPARSE COMMIT TIMING AUDIT');
say(f,'====================================================================================================');
say(f,'[AUTHORITY] readOnly=1 | productionModified=0 | noCommit=1 | noSyntheticReplan=1');
say(f,' root = %s',root);

OUT=struct();
OUT.schema='AF7R58S_R2_JOINED_COMMIT_TIMING_AUDIT_V1';
OUT.production_modified=false;
OUT.status='OPEN';
OUT.output_directory=outdir;
OUT.error=struct();

try
    pR=fullfile(root,'REPLANNING_SYSTEM.m');
    assert(isfile(pR),'[AF7R58S-R2] REPLANNING_SYSTEM.m missing.');
    hR=file_hash(pR);
    expected='1474c591fe2227f461eb43c2853d61909e63e9bef9bf6a40a851956c1c02e440';

    say(f,'');
    say(f,'[A. EXACT AF7R58Q-R5 SOURCE BINDING]');
    say(f,' REPLANNING_SYSTEM SHA exact = %d | %s',strcmp(hR,expected),hR);
    assert(strcmp(hR,expected), ...
        '[AF7R58S-R2] production source is not AF7R58Q-R5.');

    if nargin<1 || isempty(replanner)
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
    failGlobal=double(replanner.generator_deadline_fail_count);
    deadline=1e3*double(replanner.hard_modify_deadline_s);

    % ==============================================================
    % AF7R58M full request ring = epoch and outer-status authority.
    % ==============================================================
    RM=double(replanner.af7r58m_timing_ring);
    cm=double(replanner.af7r58m_timing_ring_count);
    hm=double(replanner.af7r58m_timing_ring_head);
    capm=double(replanner.af7r58m_timing_ring_capacity);
    wm=double(replanner.af7r58m_timing_ring_width);
    assert(capm==64 && wm==32 && isequal(size(RM),[64 32]), ...
        '[AF7R58S-R2] AF7R58M ring schema mismatch.');
    M=chronological_rows(RM,hm,cm,capm);

    assert(req>=1 && req<=size(M,1), ...
        '[AF7R58S-R2] request ring too short for current epoch.');

    emStart=size(M,1)-req+1;
    dense=(1:req).';
    assert(isequal(M(emStart:end,1),dense), ...
        '[AF7R58S-R2] AF7R58M latest suffix is not exact 1:N.');
    MR=M(emStart:end,:);

    if emStart>1
        baseFail=M(emStart-1,12);
        baseCommit=M(emStart-1,17);
    else
        baseFail=0;
        baseCommit=0;
    end
    failEvent=counter_events_with_baseline(MR(:,12),baseFail);
    commitEvent=counter_events_with_baseline(MR(:,17),baseCommit);

    say(f,'');
    say(f,'[B. FULL REQUEST EPOCH AUTHORITY -- AF7R58M]');
    say(f,' current generator_request_count        = %.0f',req);
    say(f,' global deadline_fail_count             = %.0f',failGlobal);
    say(f,' hard deadline [ms]                     = %.17g',deadline);
    say(f,' full request ring rows                 = %d',size(M,1));
    say(f,' prime/warmup rows before latest epoch = %d',emStart-1);
    say(f,' latest epoch exact seq 1:N             = 1');
    say(f,' latest epoch deadline-fail increments  = %.0f',sum(failEvent));
    say(f,' latest epoch commit increments         = %.0f',sum(commitEvent));

    % ==============================================================
    % AF7R58Q-R5 sparse commit ring.
    % ==============================================================
    RQ=double(replanner.af7r58q_min_ring);
    cq=double(replanner.af7r58q_min_ring_count);
    hq=double(replanner.af7r58q_min_ring_head);
    capq=double(replanner.af7r58q_min_ring_capacity);
    wq=double(replanner.af7r58q_min_ring_width);
    assert(capq==64 && wq==9 && isequal(size(RQ),[64 9]), ...
        '[AF7R58S-R2] R5 commit ring schema mismatch.');
    Q=chronological_rows(RQ,hq,cq,capq);

    say(f,'');
    say(f,'[C. SPARSE COMMIT RING -- AF7R58Q-R5]');
    say(f,' commit ring rows total                = %d',size(Q,1));
    say(f,' commit ring seq raw                    = %s',mat2str(Q(:,1).'));
    say(f,' last capture overhead [us]             = %.17g', ...
        double(replanner.af7r58q_min_last_capture_overhead_us));
    say(f,' max capture overhead [us]              = %.17g', ...
        double(replanner.af7r58q_min_max_capture_overhead_us));

    if isempty(Q)
        OUT.status='R5_COMMIT_RING_EMPTY__NO_COMMIT_PATH_CAPTURE';
        finish();
        return;
    end

    % Find latest commit epoch by request-counter reset/nonincrease.
    qseq=Q(:,1);
    reset=find(qseq(2:end)<=qseq(1:end-1),1,'last');
    if isempty(reset)
        qStart=1;
    else
        qStart=reset+1;
    end
    QR=Q(qStart:end,:);

    % Keep only valid current-epoch request numbers. Sparse is allowed.
    valid=isfinite(QR(:,1)) & QR(:,1)>=1 & QR(:,1)<=req & ...
        abs(QR(:,1)-round(QR(:,1)))==0;
    QR=QR(valid,:);
    assert(~isempty(QR),'[AF7R58S-R2] no R5 commit rows map to current request epoch.');

    % If duplicated seq remains, keep latest row for each seq.
    [useq,iaLast]=unique(QR(:,1),'last');
    [useq,ord]=sort(useq);
    QR=QR(iaLast(ord),:);

    OUT.request_epoch=MR;
    OUT.commit_epoch=QR;

    % ==============================================================
    % Join by request sequence.
    % ==============================================================
    J=nan(req,18);
    % columns:
    % 1 seq
    % 2 outer fail event
    % 3 outer status
    % 4 outer recorded cycle
    % 5 outer cleanup cycle
    % 6 outer callback
    % 7 commit-called
    % 8 commit-total
    % 9 spacing
    % 10 resource
    % 11 handles
    % 12 other
    % 13 active version at commit cleanup
    % 14 commit-count snapshot (inside commit cleanup)
    % 15 outer commit event
    % 16 outer residual-after-known
    % 17 packet rebuild event
    % 18 validation fail event
    J(:,1)=MR(:,1);
    J(:,2)=failEvent;
    J(:,3)=MR(:,11);
    J(:,4)=MR(:,3);
    J(:,5)=MR(:,2);
    J(:,6)=MR(:,7);
    J(:,7)=0;
    J(:,15)=commitEvent;
    J(:,16)=MR(:,2)-(MR(:,4)+MR(:,6)+MR(:,7)+MR(:,8));

    rebuildEvent=counter_events_with_baseline(MR(:,21), ...
        conditional_baseline(M,emStart,21));
    vfailEvent=counter_events_with_baseline(MR(:,15), ...
        conditional_baseline(M,emStart,15));
    J(:,17)=rebuildEvent;
    J(:,18)=vfailEvent;

    for k=1:size(QR,1)
        s=round(QR(k,1));
        J(s,7)=1;
        J(s,8)=QR(k,2);
        J(s,9)=QR(k,3);
        J(s,10)=QR(k,4);
        J(s,11)=QR(k,5);
        J(s,12)=QR(k,6);
        J(s,13)=QR(k,7);
        J(s,14)=QR(k,8);
    end
    OUT.joined=J;

    say(f,'');
    say(f,'[D. JOINED REAL-REQUEST TABLE]');
    say(f,' seq | fail+ | status | cycle | callback | commit? | commitTot | spacing | resource | handles | OTHER | commit+ | rebuild+ | vfail+');
    for i=1:req
        say(f,' %3.0f | %d | %6.0f | %7.3f | %8.3f | %d | %9.3f | %7.3f | %8.3f | %7.3f | %8.3f | %d | %d | %d', ...
            J(i,1),J(i,2)>0,J(i,3),J(i,4),J(i,6),J(i,7)>0, ...
            J(i,8),J(i,9),J(i,10),J(i,11),J(i,12), ...
            J(i,15)>0,J(i,17)>0,J(i,18)>0);
    end

    % ==============================================================
    % Aggregate only requests that actually entered commit function.
    % ==============================================================
    C=J(:,7)>0;
    total=J(C,8);
    spacing=J(C,9);
    resource=J(C,10);
    handles=J(C,11);
    other=J(C,12);
    cfail=J(C,2)>0;
    cstatus=J(C,3);

    A=struct();
    A.commit_called_count=sum(C);
    A.commit_called_fail_count=sum(cfail);
    A.total_max=safe_max(total);
    A.total_mean=safe_mean(total);
    A.spacing_max=safe_max(spacing);
    A.spacing_mean=safe_mean(spacing);
    A.resource_max=safe_max(resource);
    A.resource_mean=safe_mean(resource);
    A.handles_max=safe_max(handles);
    A.handles_mean=safe_mean(handles);
    A.other_max=safe_max(other);
    A.other_mean=safe_mean(other);
    A.status_codes=cstatus(:).';

    means=[A.spacing_mean,A.resource_mean,A.handles_mean,A.other_mean];
    names={'SPACING','RESOURCE_PREPARE','CANONICAL_HANDLES','OTHER_COMMIT'};
    z=means; z(~isfinite(z))=-Inf;
    [mx,ix]=max(z);
    if isfinite(mx)
        A.dominant_bucket=names{ix};
        A.dominant_mean_ms=mx;
    else
        A.dominant_bucket='OPEN';
        A.dominant_mean_ms=NaN;
    end
    OUT.aggregate=A;

    say(f,'');
    say(f,'[E. COMMIT-CALLED AGGREGATE]');
    say(f,' commit-called requests                 = %d',A.commit_called_count);
    say(f,' commit-called deadline fails           = %d',A.commit_called_fail_count);
    say(f,' commit total max / mean [ms]           = %.17g / %.17g',A.total_max,A.total_mean);
    say(f,' spacing max / mean [ms]                = %.17g / %.17g',A.spacing_max,A.spacing_mean);
    say(f,' resource max / mean [ms]               = %.17g / %.17g',A.resource_max,A.resource_mean);
    say(f,' handles max / mean [ms]                = %.17g / %.17g',A.handles_max,A.handles_mean);
    say(f,' OTHER max / mean [ms]                  = %.17g / %.17g',A.other_max,A.other_mean);
    say(f,' dominant bucket                        = %s',A.dominant_bucket);
    say(f,' dominant mean [ms]                     = %.17g',A.dominant_mean_ms);

    % Source attribution is a split, not optimization authorization.
    if strcmp(A.dominant_bucket,'OTHER_COMMIT')
        OUT.status='JOINED_COMMIT_TIMING_CLOSED__OTHER_COMMIT_SPLIT_NEXT';
    elseif strcmp(A.dominant_bucket,'RESOURCE_PREPARE')
        OUT.status='JOINED_COMMIT_TIMING_CLOSED__RESOURCE_PREPARE_AUDIT_NEXT';
    elseif strcmp(A.dominant_bucket,'CANONICAL_HANDLES')
        OUT.status='JOINED_COMMIT_TIMING_CLOSED__CANONICAL_HANDLES_AUDIT_NEXT';
    elseif strcmp(A.dominant_bucket,'SPACING')
        OUT.status='JOINED_COMMIT_TIMING_CLOSED__SPACING_AUDIT_NEXT';
    else
        OUT.status='JOINED_COMMIT_TIMING_CAPTURED__ATTRIBUTION_OPEN';
    end

    say(f,'');
    say(f,'[F. STATUS]');
    say(f,' sparse commit seq accepted             = 1');
    say(f,' full-request epoch authority            = AF7R58M');
    say(f,' deadline-fail authority                 = outer AF7R58M cumulative counter');
    say(f,' instrumented timing final 4-ms proof    = 0');
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
        save(fullfile(outdir,'AF7R58S_R2_OUT.mat'),'OUT','-v7');
        assignin('base','AF7R58S_R2_OUT',OUT);
        say(f,'');
        say(f,'[RETURN THESE TWO FILES]');
        say(f,' RETURN: %s',fullfile(outdir,'AF7R58S_R2_REPORT.txt'));
        say(f,' RETURN: %s',fullfile(outdir,'AF7R58S_R2_OUT.mat'));
        say(f,' status = %s',OUT.status);
    end
end


function b=conditional_baseline(M,startRow,col)
if startRow>1
    b=M(startRow-1,col);
else
    b=0;
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
    if isfinite(c(i)) && isfinite(prev)
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
