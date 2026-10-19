function OUT = AF7R58W_ONEFILE_PRETRY_TIMING_AUDIT(replanner)
% AF7R58W
% Read-only audit for AF7R58V-R4 pre-transaction timing diagnostics.
%
% Run AFTER one clean run_flight_headless with the AF7R58V-R4 production
% source installed. Keep the live REPLANNING_SYSTEM object in workspace.
%
% Epoch authority:
%   AF7R58M full request ring.
% Sparse commit/pretry timing:
%   AF7R58V-R4 ring, joined by generator request sequence.
%
% No source or live state is modified.
%
% RETURN:
%   AF7R58W_REPORT.txt
%   AF7R58W_OUT.mat

root=fileparts(mfilename('fullpath'));
outdir=tempname(root);
[ok,msg]=mkdir(outdir); assert(ok,'%s',msg);

f=fopen(fullfile(outdir,'AF7R58W_REPORT.txt'),'wt','n','UTF-8');
assert(f>=0,'[AF7R58W] cannot open report.');
cleanup=onCleanup(@()fclose(f)); %#ok<NASGU>

say(f,'====================================================================================================');
say(f,' AF7R58W | PRE-TRANSACTION TIMING AUDIT');
say(f,'====================================================================================================');
say(f,'[AUTHORITY] readOnly=1 | productionModified=0 | noCommit=1 | noSyntheticReplan=1');

OUT=struct();
OUT.schema='AF7R58W_PRETRY_TIMING_AUDIT_V1';
OUT.production_modified=false;
OUT.status='OPEN';
OUT.output_directory=outdir;
OUT.error=struct();

try
    pR=fullfile(root,'REPLANNING_SYSTEM.m');
    assert(isfile(pR),'[AF7R58W] REPLANNING_SYSTEM.m missing.');

    hR=file_hash(pR);
    expected='cfa7e053e86aec9879eef0fb3c6d28e55fd33c8b1e76c09c77bdd83e1f170f79';

    say(f,'');
    say(f,'[A. EXACT AF7R58V-R4 SOURCE BINDING]');
    say(f,' REPLANNING_SYSTEM SHA exact = %d | %s',strcmp(hR,expected),hR);
    assert(strcmp(hR,expected), ...
        '[AF7R58W] source is not AF7R58V-R4 production.');

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

    % --------------------------------------------------------------
    % AF7R58M request ring = real-epoch authority.
    % --------------------------------------------------------------
    RM=double(replanner.af7r58m_timing_ring);
    cm=double(replanner.af7r58m_timing_ring_count);
    hm=double(replanner.af7r58m_timing_ring_head);
    capm=double(replanner.af7r58m_timing_ring_capacity);
    wm=double(replanner.af7r58m_timing_ring_width);

    assert(capm==64 && wm==32 && isequal(size(RM),[64 32]), ...
        '[AF7R58W] AF7R58M ring schema mismatch.');

    M=chronological_rows(RM,hm,cm,capm);
    assert(req>=1 && req<=size(M,1), ...
        '[AF7R58W] AF7R58M ring too short for current request epoch.');

    emStart=size(M,1)-req+1;
    assert(isequal(M(emStart:end,1),(1:req).'), ...
        '[AF7R58W] latest AF7R58M suffix is not exact 1:N.');

    MR=M(emStart:end,:);
    failEvent=counter_events_with_baseline(MR(:,12),conditional_baseline(M,emStart,12));
    commitEvent=counter_events_with_baseline(MR(:,17),conditional_baseline(M,emStart,17));
    rebuildEvent=counter_events_with_baseline(MR(:,21),conditional_baseline(M,emStart,21));
    vfailEvent=counter_events_with_baseline(MR(:,15),conditional_baseline(M,emStart,15));

    say(f,'');
    say(f,'[B. REAL REQUEST EPOCH -- AF7R58M]');
    say(f,' generator request count               = %.0f',req);
    say(f,' global deadline fail count            = %.0f',failGlobal);
    say(f,' hard deadline [ms]                    = %.17g',deadline);
    say(f,' prime/warmup rows before real epoch   = %d',emStart-1);
    say(f,' latest epoch exact seq 1:N            = 1');
    say(f,' real-epoch deadline-fail increments   = %.0f',sum(failEvent));
    say(f,' real-epoch commit increments          = %.0f',sum(commitEvent));

    % --------------------------------------------------------------
    % AF7R58V-R4 sparse pretry ring.
    % --------------------------------------------------------------
    RV=double(replanner.af7r58v_r4_ring);
    cv=double(replanner.af7r58v_r4_ring_count);
    hv=double(replanner.af7r58v_r4_ring_head);
    capv=double(replanner.af7r58v_r4_ring_capacity);
    wv=double(replanner.af7r58v_r4_ring_width);

    assert(capv==64 && wv==15 && isequal(size(RV),[64 15]), ...
        '[AF7R58W] AF7R58V-R4 ring schema mismatch.');

    V=chronological_rows(RV,hv,cv,capv);

    say(f,'');
    say(f,'[C. AF7R58V-R4 SPARSE PRETRY RING]');
    say(f,' ring rows total                       = %d',size(V,1));
    if isempty(V)
        say(f,' ring seq raw                          = []');
    else
        say(f,' ring seq raw                          = %s',mat2str(V(:,1).'));
    end
    say(f,' last capture overhead [us]            = %.17g', ...
        double(replanner.af7r58v_r4_last_capture_overhead_us));
    say(f,' max capture overhead [us]             = %.17g', ...
        double(replanner.af7r58v_r4_max_capture_overhead_us));

    if isempty(V)
        OUT.status='V_R4_RING_EMPTY__NO_COMMIT_PATH_CAPTURE';
        finish();
        return;
    end

    % Latest sparse epoch from last seq reset/nonincrease.
    seq=V(:,1);
    reset=find(seq(2:end)<=seq(1:end-1),1,'last');
    if isempty(reset), vStart=1; else, vStart=reset+1; end

    VR=V(vStart:end,:);
    valid=isfinite(VR(:,1)) & VR(:,1)>=1 & VR(:,1)<=req & ...
        abs(VR(:,1)-round(VR(:,1)))==0;
    VR=VR(valid,:);
    assert(~isempty(VR),'[AF7R58W] no V-R4 rows map to current epoch.');

    [useq,iaLast]=unique(VR(:,1),'last');
    [useq,ord]=sort(useq);
    VR=VR(iaLast(ord),:);
    OUT.pretry_epoch=VR;

    % --------------------------------------------------------------
    % Join by generator request sequence.
    % --------------------------------------------------------------
    % J:
    % 1 seq
    % 2 outer fail+
    % 3 outer status
    % 4 outer recorded cycle
    % 5 outer callback
    % 6 commit/pretry called?
    % 7 V total
    % 8 progress
    % 9 branch
    % 10..19 stages s1..s10
    % 20 outer commit+
    % 21 rebuild+
    % 22 vfail+
    J=nan(req,22);
    J(:,1)=MR(:,1);
    J(:,2)=failEvent;
    J(:,3)=MR(:,11);
    J(:,4)=MR(:,3);
    J(:,5)=MR(:,7);
    J(:,6)=0;
    J(:,20)=commitEvent;
    J(:,21)=rebuildEvent;
    J(:,22)=vfailEvent;

    for k=1:size(VR,1)
        s=round(VR(k,1));
        J(s,6)=1;
        J(s,7)=VR(k,2);
        J(s,8)=VR(k,3);
        J(s,9)=VR(k,4);
        J(s,10:19)=VR(k,5:14);
    end
    OUT.joined=J;

    labels={ ...
        'handles', ...
        'smooth_geom', ...
        'boundary_preeval', ...
        'old_snapshot', ...
        'reuse_test', ...
        'reuse_operation', ...
        'rebuild_sweep', ...
        'rebuild_packet', ...
        'rebuild_sensor_dep', ...
        'predeadline_gate'};
    OUT.stage_labels=labels;

    say(f,'');
    say(f,'[D. JOINED REAL-REQUEST TABLE]');
    say(f,' seq | fail+ | status | cycle | callback | called | total | progress | branch | s1 | s2 | s3 | s4 | s5 | s6 | s7 | s8 | s9 | s10 | commit+ | rebuild+ | vfail+');
    for i=1:req
        say(f,[' %3.0f | %d | %6.0f | %7.3f | %8.3f | %d | %7.3f | %8.0f | %6.0f |' ...
            ' %6.3f | %6.3f | %6.3f | %6.3f | %6.3f | %6.3f | %6.3f | %6.3f | %6.3f | %6.3f | %d | %d | %d'], ...
            J(i,1),J(i,2)>0,J(i,3),J(i,4),J(i,5),J(i,6)>0,J(i,7),J(i,8),J(i,9), ...
            J(i,10),J(i,11),J(i,12),J(i,13),J(i,14),J(i,15),J(i,16),J(i,17),J(i,18),J(i,19), ...
            J(i,20)>0,J(i,21)>0,J(i,22)>0);
    end

    % --------------------------------------------------------------
    % Progress classification.
    % --------------------------------------------------------------
    C=J(:,6)>0;
    progress=J(C,8);
    branch=J(C,9);
    outerFail=J(C,2)>0;
    S=J(C,10:19);
    total=J(C,7);

    A=struct();
    A.called_count=sum(C);
    A.called_deadline_fail_count=sum(outerFail);
    A.progress_codes=progress(:).';
    A.branch_codes=branch(:).';
    A.predeadline_reject_count=sum(progress==10);
    A.try_reached_count=sum(progress==11);
    A.reuse_count=sum(branch==1);
    A.rebuild_count=sum(branch==2);
    A.total_max=safe_max(total);
    A.total_mean=safe_mean(total);
    A.stage_max=nan(1,10);
    A.stage_mean=nan(1,10);

    for k=1:10
        A.stage_max(k)=safe_max(S(:,k));
        A.stage_mean(k)=safe_mean(S(:,k));
    end

    z=A.stage_mean; z(~isfinite(z))=-Inf;
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
    say(f,'[E. PRETRY CLASSIFICATION]');
    say(f,' called requests                       = %d',A.called_count);
    say(f,' called deadline fails                 = %d',A.called_deadline_fail_count);
    say(f,' progress codes                        = %s',mat2str(A.progress_codes));
    say(f,' branch codes                          = %s',mat2str(A.branch_codes));
    say(f,' predeadline rejects (progress=10)     = %d',A.predeadline_reject_count);
    say(f,' transactional try reached (=11)       = %d',A.try_reached_count);
    say(f,' exact reuse branch count (=1)         = %d',A.reuse_count);
    say(f,' rebuild branch count (=2)             = %d',A.rebuild_count);
    say(f,' total max / mean [ms]                 = %.17g / %.17g',A.total_max,A.total_mean);

    for k=1:10
        say(f,' s%02d %-28s max/mean = %.17g / %.17g ms', ...
            k,labels{k},A.stage_max(k),A.stage_mean(k));
    end

    say(f,' dominant measured stage               = s%02d %s', ...
        A.dominant_stage_index,A.dominant_stage);
    say(f,' dominant stage mean [ms]              = %.17g',A.dominant_stage_mean_ms);

    % Failed-vs-pass comparison among called requests.
    F=S(outerFail,:);
    P=S(~outerFail,:);
    CMP=struct();
    CMP.failed_mean=nan(1,10);
    CMP.pass_mean=nan(1,10);
    CMP.delta=nan(1,10);
    for k=1:10
        CMP.failed_mean(k)=safe_mean(F(:,k));
        CMP.pass_mean(k)=safe_mean(P(:,k));
        CMP.delta(k)=CMP.failed_mean(k)-CMP.pass_mean(k);
    end
    OUT.fail_pass=CMP;

    d=CMP.delta; q=d; q(~isfinite(q))=-Inf;
    [dmx,dix]=max(q);

    say(f,'');
    say(f,'[F. DEADLINE-FAIL VS NONFAIL]');
    for k=1:10
        say(f,' s%02d %-28s fail/pass/delta = %.17g / %.17g / %.17g ms', ...
            k,labels{k},CMP.failed_mean(k),CMP.pass_mean(k),CMP.delta(k));
    end
    if isfinite(dmx)
        say(f,' largest fail-pass delta               = s%02d %s | %.17g ms', ...
            dix,labels{dix},dmx);
    end

    % Conservative status based on progress authority first.
    if A.predeadline_reject_count>0
        if A.rebuild_count>0
            OUT.status='PRETRY_CAUSE_CLOSED__PREDEADLINE_REJECT_WITH_REBUILD_PATH__OPTIMIZATION_AUDIT_NEXT';
        else
            OUT.status='PRETRY_CAUSE_CLOSED__PREDEADLINE_REJECT_WITH_REUSE_PATH__OPTIMIZATION_AUDIT_NEXT';
        end
    elseif A.try_reached_count==A.called_count
        OUT.status='PRETRY_ALL_CALLS_REACHED_TRANSACTION__POSTTRY_OPTIMIZATION_NEXT';
    else
        OUT.status='PRETRY_CAPTURED__MIXED_PROGRESS__SOURCE_AUDIT_NEXT';
    end

    say(f,'');
    say(f,'[G. STATUS]');
    say(f,' full request epoch authority          = AF7R58M');
    say(f,' sparse V-R4 join                      = CLOSED');
    say(f,' production deadline-fail authority    = outer AF7R58M counter');
    say(f,' instrumented timing final 4-ms proof  = 0');
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
        save(fullfile(outdir,'AF7R58W_OUT.mat'),'OUT','-v7');
        assignin('base','AF7R58W_OUT',OUT);

        say(f,'');
        say(f,'[RETURN THESE TWO FILES]');
        say(f,' RETURN: %s',fullfile(outdir,'AF7R58W_REPORT.txt'));
        say(f,' RETURN: %s',fullfile(outdir,'AF7R58W_OUT.mat'));
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


function x=safe_max(v)
v=double(v(:));
v=v(isfinite(v));
if isempty(v), x=NaN; else, x=max(v); end
end


function x=safe_mean(v)
v=double(v(:));
v=v(isfinite(v));
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
