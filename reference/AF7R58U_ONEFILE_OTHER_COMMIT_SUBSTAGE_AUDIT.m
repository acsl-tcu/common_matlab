function OUT = AF7R58U_ONEFILE_OTHER_COMMIT_SUBSTAGE_AUDIT(replanner)
% AF7R58U
% Read-only audit of AF7R58T-R2 OTHER_COMMIT substage timing.
%
% Run AFTER one clean run_flight_headless using the AF7R58T-R2 source.
% Keep the live REPLANNING_SYSTEM object in the base workspace.
%
% Epoch authority: AF7R58M full request ring.
% Sparse commit timing: AF7R58T-R2 ring, joined by request seq.
%
% No production source or live state is modified.
%
% RETURN:
%   AF7R58U_REPORT.txt
%   AF7R58U_OUT.mat

root=fileparts(mfilename('fullpath'));
outdir=tempname(root);
[ok,msg]=mkdir(outdir); assert(ok,'%s',msg);

f=fopen(fullfile(outdir,'AF7R58U_REPORT.txt'),'wt','n','UTF-8');
assert(f>=0,'[AF7R58U] cannot open report.');
cleanup=onCleanup(@()fclose(f)); %#ok<NASGU>

say(f,'====================================================================================================');
say(f,' AF7R58U | OTHER_COMMIT SUBSTAGE TIMING AUDIT');
say(f,'====================================================================================================');
say(f,'[AUTHORITY] readOnly=1 | productionModified=0 | noCommit=1 | noSyntheticReplan=1');
say(f,' root = %s',root);

OUT=struct();
OUT.schema='AF7R58U_OTHER_COMMIT_SUBSTAGE_AUDIT_V1';
OUT.production_modified=false;
OUT.status='OPEN';
OUT.output_directory=outdir;
OUT.error=struct();

try
    pR=fullfile(root,'REPLANNING_SYSTEM.m');
    assert(isfile(pR),'[AF7R58U] REPLANNING_SYSTEM.m missing.');
    hR=file_hash(pR);
    expected='72b3b05880362d2bfa4bb81eb61eb04bf69bd38eccf796d860a43b156c4d32ea';

    say(f,'');
    say(f,'[A. EXACT AF7R58T-R2 SOURCE BINDING]');
    say(f,' REPLANNING_SYSTEM SHA exact = %d | %s',strcmp(hR,expected),hR);
    assert(strcmp(hR,expected), ...
        '[AF7R58U] source is not AF7R58T-R2 production.');

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
    % AF7R58M request-ring authority.
    % ==============================================================
    RM=double(replanner.af7r58m_timing_ring);
    cm=double(replanner.af7r58m_timing_ring_count);
    hm=double(replanner.af7r58m_timing_ring_head);
    capm=double(replanner.af7r58m_timing_ring_capacity);
    wm=double(replanner.af7r58m_timing_ring_width);
    assert(capm==64 && wm==32 && isequal(size(RM),[64 32]), ...
        '[AF7R58U] AF7R58M ring schema mismatch.');
    M=chronological_rows(RM,hm,cm,capm);

    assert(req>=1 && req<=size(M,1), ...
        '[AF7R58U] request ring too short for current epoch.');
    emStart=size(M,1)-req+1;
    assert(isequal(M(emStart:end,1),(1:req).'), ...
        '[AF7R58U] AF7R58M latest suffix is not exact 1:N.');
    MR=M(emStart:end,:);

    baseFail=conditional_baseline(M,emStart,12);
    baseCommit=conditional_baseline(M,emStart,17);
    baseRebuild=conditional_baseline(M,emStart,21);
    baseVfail=conditional_baseline(M,emStart,15);

    failEvent=counter_events_with_baseline(MR(:,12),baseFail);
    commitEvent=counter_events_with_baseline(MR(:,17),baseCommit);
    rebuildEvent=counter_events_with_baseline(MR(:,21),baseRebuild);
    vfailEvent=counter_events_with_baseline(MR(:,15),baseVfail);

    say(f,'');
    say(f,'[B. REAL REQUEST EPOCH -- AF7R58M AUTHORITY]');
    say(f,' generator request count               = %.0f',req);
    say(f,' global deadline fail count            = %.0f',failGlobal);
    say(f,' hard deadline [ms]                    = %.17g',deadline);
    say(f,' prime/warmup rows before real epoch   = %d',emStart-1);
    say(f,' latest epoch exact seq 1:N            = 1');
    say(f,' real-epoch deadline-fail increments   = %.0f',sum(failEvent));
    say(f,' real-epoch commit increments          = %.0f',sum(commitEvent));

    % ==============================================================
    % AF7R58T-R2 sparse commit ring.
    % ==============================================================
    RT=double(replanner.af7r58t_ring);
    ct=double(replanner.af7r58t_ring_count);
    ht=double(replanner.af7r58t_ring_head);
    capt=double(replanner.af7r58t_ring_capacity);
    wt=double(replanner.af7r58t_ring_width);
    assert(capt==64 && wt==15 && isequal(size(RT),[64 15]), ...
        '[AF7R58U] AF7R58T-R2 ring schema mismatch.');
    T=chronological_rows(RT,ht,ct,capt);

    say(f,'');
    say(f,'[C. AF7R58T-R2 SPARSE COMMIT RING]');
    say(f,' ring rows total                       = %d',size(T,1));
    if isempty(T)
        say(f,' ring seq raw                          = []');
    else
        say(f,' ring seq raw                          = %s',mat2str(T(:,1).'));
    end
    say(f,' last capture overhead [us]            = %.17g', ...
        double(replanner.af7r58t_last_capture_overhead_us));
    say(f,' max capture overhead [us]             = %.17g', ...
        double(replanner.af7r58t_max_capture_overhead_us));

    if isempty(T)
        OUT.status='T_RING_EMPTY__NO_COMMIT_PATH_CAPTURE';
        finish();
        return;
    end

    % Latest sparse epoch: after last non-increasing seq.
    seq=T(:,1);
    reset=find(seq(2:end)<=seq(1:end-1),1,'last');
    if isempty(reset)
        tStart=1;
    else
        tStart=reset+1;
    end
    TR=T(tStart:end,:);

    valid=isfinite(TR(:,1)) & TR(:,1)>=1 & TR(:,1)<=req & ...
        abs(TR(:,1)-round(TR(:,1)))==0;
    TR=TR(valid,:);
    assert(~isempty(TR),'[AF7R58U] no T rows map to current request epoch.');

    % Keep latest duplicate for each request seq.
    [useq,iaLast]=unique(TR(:,1),'last');
    [useq,ord]=sort(useq);
    TR=TR(iaLast(ord),:);

    OUT.commit_epoch=TR;

    % ==============================================================
    % Join by request seq.
    % ==============================================================
    % J columns:
    % 1 seq
    % 2 outer fail+
    % 3 outer status
    % 4 recorded generator cycle
    % 5 callback
    % 6 commit-called
    % 7 T commit total
    % 8..18 T stages 1..11
    % 19 outer commit+
    % 20 rebuild+
    % 21 validation-fail+
    % 22 active version snapshot
    % 23 T replan commit count snapshot
    J=nan(req,23);
    J(:,1)=MR(:,1);
    J(:,2)=failEvent;
    J(:,3)=MR(:,11);
    J(:,4)=MR(:,3);
    J(:,5)=MR(:,7);
    J(:,6)=0;
    J(:,19)=commitEvent;
    J(:,20)=rebuildEvent;
    J(:,21)=vfailEvent;

    for k=1:size(TR,1)
        s=round(TR(k,1));
        J(s,6)=1;
        J(s,7)=TR(k,2);
        J(s,8:18)=TR(k,3:13);
        J(s,22)=TR(k,14);
        J(s,23)=TR(k,15);
    end
    OUT.joined=J;

    labels={ ...
        'entry_to_spacing', ...
        'spacing_to_resource', ...
        'resource_to_handles', ...
        'post_handles_to_try', ...
        'try_to_canonical_store', ...
        'canonical_store_to_active_assign', ...
        'active_assign_to_commit_state', ...
        'commit_state', ...
        'post_commit_assertions', ...
        'final_deadline_success_path', ...
        'rollback_path'};
    OUT.stage_labels=labels;

    say(f,'');
    say(f,'[D. JOINED REQUEST TABLE]');
    say(f,' seq | fail+ | status | cycle | callback | commit? | Ttotal | s1 | s2 | s3 | s4 | s5 | s6 | s7 | s8 | s9 | s10 | s11 | commit+ | rebuild+ | vfail+');
    for i=1:req
        say(f,[' %3.0f | %d | %6.0f | %7.3f | %8.3f | %d | %7.3f |' ...
            ' %6.3f | %6.3f | %6.3f | %6.3f | %6.3f | %6.3f | %6.3f |' ...
            ' %6.3f | %6.3f | %6.3f | %6.3f | %d | %d | %d'], ...
            J(i,1),J(i,2)>0,J(i,3),J(i,4),J(i,5),J(i,6)>0,J(i,7), ...
            J(i,8),J(i,9),J(i,10),J(i,11),J(i,12),J(i,13),J(i,14), ...
            J(i,15),J(i,16),J(i,17),J(i,18), ...
            J(i,19)>0,J(i,20)>0,J(i,21)>0);
    end

    % ==============================================================
    % Aggregate commit-called requests.
    % ==============================================================
    C=J(:,6)>0;
    S=J(C,8:18);
    total=J(C,7);
    outerFail=J(C,2)>0;

    A=struct();
    A.commit_called_count=sum(C);
    A.commit_called_deadline_fail_count=sum(outerFail);
    A.total_max=safe_max(total);
    A.total_mean=safe_mean(total);
    A.stage_max=nan(1,11);
    A.stage_mean=nan(1,11);
    for k=1:11
        A.stage_max(k)=safe_max(S(:,k));
        A.stage_mean(k)=safe_mean(S(:,k));
    end

    z=A.stage_mean;
    z(~isfinite(z))=-Inf;
    [mx,ix]=max(z);
    if isfinite(mx)
        A.dominant_stage_index=ix;
        A.dominant_stage=labels{ix};
        A.dominant_stage_mean_ms=mx;
    else
        A.dominant_stage_index=NaN;
        A.dominant_stage='OPEN';
        A.dominant_stage_mean_ms=NaN;
    end
    OUT.aggregate=A;

    say(f,'');
    say(f,'[E. COMMIT-CALLED AGGREGATE]');
    say(f,' commit-called requests                = %d',A.commit_called_count);
    say(f,' commit-called deadline fails          = %d',A.commit_called_deadline_fail_count);
    say(f,' T commit total max / mean [ms]        = %.17g / %.17g',A.total_max,A.total_mean);
    for k=1:11
        say(f,' s%02d %-34s max/mean = %.17g / %.17g ms', ...
            k,labels{k},A.stage_max(k),A.stage_mean(k));
    end
    say(f,' dominant T stage                      = s%02d %s', ...
        A.dominant_stage_index,A.dominant_stage);
    say(f,' dominant T stage mean [ms]            = %.17g',A.dominant_stage_mean_ms);

    % ==============================================================
    % Failed vs successful commit-called means, to identify spike-only path.
    % ==============================================================
    F=S(outerFail,:);
    P=S(~outerFail,:);
    CMP=struct();
    CMP.failed_mean=nan(1,11);
    CMP.pass_mean=nan(1,11);
    CMP.delta_fail_minus_pass=nan(1,11);
    for k=1:11
        CMP.failed_mean(k)=safe_mean(F(:,k));
        CMP.pass_mean(k)=safe_mean(P(:,k));
        CMP.delta_fail_minus_pass(k)=CMP.failed_mean(k)-CMP.pass_mean(k);
    end
    OUT.fail_pass_comparison=CMP;

    d=CMP.delta_fail_minus_pass;
    zz=d;
    zz(~isfinite(zz))=-Inf;
    [dmx,dix]=max(zz);

    say(f,'');
    say(f,'[F. DEADLINE-FAIL VS NONFAIL COMMIT PATH]');
    for k=1:11
        say(f,' s%02d %-34s failMean/passMean/delta = %.17g / %.17g / %.17g ms', ...
            k,labels{k},CMP.failed_mean(k),CMP.pass_mean(k),CMP.delta_fail_minus_pass(k));
    end
    if isfinite(dmx)
        say(f,' largest fail-pass stage delta          = s%02d %s | %.17g ms', ...
            dix,labels{dix},dmx);
    end

    if strcmp(A.dominant_stage,'OPEN')
        OUT.status='OTHER_COMMIT_SUBSTAGE_CAPTURED__ATTRIBUTION_OPEN';
    else
        OUT.status=['OTHER_COMMIT_SUBSTAGE_CLOSED__' upper(labels{A.dominant_stage_index}) '__AUDIT_NEXT'];
    end

    say(f,'');
    say(f,'[G. STATUS]');
    say(f,' full request epoch authority           = AF7R58M');
    say(f,' sparse commit join                     = CLOSED');
    say(f,' production deadline-fail authority     = outer AF7R58M counter');
    say(f,' instrumentation timing final 4-ms proof= 0');
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
        save(fullfile(outdir,'AF7R58U_OUT.mat'),'OUT','-v7');
        assignin('base','AF7R58U_OUT',OUT);
        say(f,'');
        say(f,'[RETURN THESE TWO FILES]');
        say(f,' RETURN: %s',fullfile(outdir,'AF7R58U_REPORT.txt'));
        say(f,' RETURN: %s',fullfile(outdir,'AF7R58U_OUT.mat'));
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
