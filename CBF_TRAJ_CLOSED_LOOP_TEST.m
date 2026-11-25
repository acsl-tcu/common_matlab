function zipPath=CBF_TRAJ_CLOSED_LOOP_TEST(logger,hlc)
% REV33 diagnostic only. Run after stopped REV30 GUI flight, no Sim edits.
% Five model-in-loop branches; original governor, sensor and mass history
% are recorded exogenous inputs, not recalculated. No live object mutation.
% Outputs one ZIP beside this file; no directories or standalone CSVs.
outDir=fileparts(mfilename('fullpath'));
zipPath=fullfile(outDir,'CBF_TRAJ_CLOSED_LOOP_RESULTS.zip');
summary="REV33 CLOSED LOOP CONDITIONAL REPLAY"+newline;
replay={}; gates={}; snaps={};
try
    if nargin==0
        if ~evalin('base','exist(''app'',''var'')'),error('CBFTest:AppMissing','Run REV30 simulation, keep app.');end
        app=evalin('base','app');
        if ~isa(app,'SimExp')||~isvalid(app)||~strcmp(string(app.cha),"s"),error('CBFTest:NotStopped','Use stopped SimExp app.');end
        logger=app.logger;hlc=app.agent(1).controller.hlc_suspended;
    elseif nargin~=2,error('CBFTest:Arguments','Use no arguments or logger,hlc.');end
    [summary,replay,gates,snaps]=audit(logger,hlc,summary);
catch ME
    summary=summary+"STATUS=FAILED"+newline+string(getReport(ME,'extended','hyperlinks','off'))+newline;
end
% Stream all entries to temporary ZIP, replace old output on completion.
tmp=[tempname(outDir),'.zip'];clean=onCleanup(@()delete_if_exists(tmp));
fos=java.io.FileOutputStream(tmp);zos=java.util.zip.ZipOutputStream(fos);
zc=onCleanup(@()close_zip(zos));
put_entry(zos,'SUMMARY.txt',char(summary));
if ~isempty(replay),put_entry(zos,'BRANCHES.csv',join_lines(replay));end
if ~isempty(gates),put_entry(zos,'RECORDED_REPLAY.csv',join_lines(gates));end
if ~isempty(snaps),put_entry(zos,'SNAPSHOTS.csv',join_lines(snaps));end
zos.close();clear zc
[ok,msg]=movefile(tmp,zipPath,'f');if ~ok,error('CBFTest:ZIPMove','%s',msg);end
clear clean
fprintf('[CBF CLOSED LOOP TEST] %s\n%s\n',zipPath,char(summary));
end

function [summary,rows,gates,snaps]=audit(logger,hlc,summary)
if ~isa(hlc,'HLC_SUSPENDED_LOAD_CBF_REAL')||hlc.cbfConfig.cbfId~=3,error('CBFTest:ID3','ID3 required');end
if nargin('CBF_TRAJ_PHYSICAL_HOLD_DIFF')~=9,error('CBFTest:REV30','Install/use saved REV30 for this test, not REV23.');end
D=logger.Data;L=D.agent(1);dt=hlc.param.dt;
n=min([logger.k,numel(D.t),numel(D.phase),numel(L.controller.result),numel(L.reference.result),numel(L.estimator.result),numel(L.sensor.result)]);
t=double(D.t(1:n));phase=char(D.phase(1:n));
Pfixed=ascol(hlc.self.parameter.get(["mass","jx","jy","jz","gravity","loadmass","cableL"]),7);
first=0;last=0;
for k=2:n
    if phase(k)~='f'||phase(k-1)~='f',continue;end
    r=L.reference.result{k};
    if ~isstruct(r)||~isfield(r,'cbfTraj'),continue;end
    r=r.cbfTraj;
    if r.correctionRevision~=30,error('CBFTest:WrongRevision','Expected recorded REV30.');end
    if first==0 && r.physicalPrevious.comparisonStarted,first=k-1;end
    if first>0,last=k;end
end
if first==0||last<=first,error('CBFTest:NoEpisode','No recorded intervention episode.');end
summary=summary+sprintf('dt=%.17g\nstart=%.17g\nlast_recorded=%.17g\nintervals=%d\n',dt,t(first),t(last),last-first);
summary=summary+"A=REV30 PV + outward gate; B=position only + outward gate; C=PV + basic quiet gate; D=position only + basic quiet gate; E=position only + basic quiet gate + hold last position during feasible quiet window. All use original-reference comparator and existing 7th order return. Same 5m/5m/s comparison guard. XYZ identical. No online mode added."+newline;
summary=summary+"Scope: nonlinear model-in-loop HLC + actual ID3 QP with independent branch T,q and x19; no measured-state reset after initialization. Recorded original reference jets, mass estimates, sensor snapshots fixed across branches; no governor/estimator/sensor re-simulation. Observations may be incomplete for counterfactual positions. This is not a full GUI flight or safety certificate. Stop branch on first QP failure, invalid physical state or missing data, never silently drop an interval. No claim of final recovery beyond recorded horizon."+newline;
summary=summary+"A uses a local copy of REV30 mode update; same actual helper for comparator. Independent RK4 propagation (2 substeps), endpoint tension checks; live plant integrator differs. P reconstructed from live fixed constants and recorded estimator mL, gated by raw replay. Yaw uses equivalent atan2 wrap. Diagnostic durations do not include GUI."+newline;
E=return_transition(dt);
unit_check(E);summary=summary+"MODE_UPDATE_UNIT_CHECK=PASS"+newline;
labels={'A_REV30','B_POS_OUTWARD','C_PV_QUIET','D_POS_QUIET','E_POS_HOLD_QUIET'};
head={'branch','source_t','end_t','mode','update_status','qp_status','compare_status','active','gate','quiet','comparison_valid','success','ms'};
head=[head,vecnames('original28',28),vecnames('command28',28),vecnames('correction',21),vecnames('state',19),vecnames('end_state',19),vecnames('nominal_offset',6),vecnames('raw',4),vecnames('applied',4),vecnames('TqNuTendQend',5),vecnames('position_error',3),vecnames('velocity_error',3),{'h_min','stage_min','raw_outside_box','deltaTend','deltaTauNorm','obs_count','next_active','next_gate','next_comparison_valid','next_quiet'},vecnames('next_offsetPV',6),{'next_mode','next_update_status','next_compare_status'}];
rows={strjoin(head,',')};
gh=[{'source_t','status','logged_qp_success','replayed_qp_success','raw_max_error','hold_max_error','applied_max_error','state_endpoint_max_error','h_max_error','recorded_obs_count'}];gates={strjoin(gh,',')};
sh=[{'source_t','end_t'},vecnames('P',7),vecnames('x',19),vecnames('next_x',19),vecnames('before28',28),vecnames('actual28',28),vecnames('raw',4),vecnames('applied',4),vecnames('hold',5)];snaps={strjoin(sh,',')};
% Copy numeric records, including the sensor struct used by ID3. No calling
% agent.sensor.do, reference.do, controller.do, or modifying logger.Data.
S=cell(last-first,1);rawErr=[];holdErr=[];inputErr=[];endErr=[];hErr=[];obsCounts=[];bad=0;
for k=first:last-1
    try
        if phase(k)~='f'||phase(k+1)~='f'||abs(t(k+1)-t(k)-dt)>1e-8,error('CBFTest:Time','Nonregular interval');end
        c=L.controller.result{k};r=L.reference.result{k}.cbfTraj;rn=L.reference.result{k+1}.cbfTraj;
        if abs(rn.sourceIntervalStart-t(k))>1e-8||abs(rn.sourceIntervalEnd-t(k+1))>1e-8,error('CBFTest:Alignment','Source alignment failed');end
        s=struct();s.t=t(k);s.endt=t(k+1);s.x=ascol(c.x,19);s.xnext=ascol(L.controller.result{k+1}.x,19);
        s.P=Pfixed;s.P(6)=double(L.estimator.result{k}.state.mL);
        s.base=ascol(r.before28,28);s.sent=ascol(c.xd,28);s.raw=ascol(c.tmp,4);s.u=ascol(c.input,4);
        mask=[1:3,5:28];ra=ascol(r.after28,28);if any(abs(ra(mask)-s.sent(mask))>1e-9*(1+abs(s.sent(mask)))),error('CBFTest:RefAlign','Reference differs');end
        h=c.cbf.paperHold;s.hold=[h.T0;h.q0;h.nu;h.Tend;h.qEnd];
        if ~h.active||abs(h.dt-dt)>1e-10||any(~isfinite([s.P;s.x;s.xnext;s.base;s.sent;s.raw;s.u;s.hold]))||any(s.P<=0),error('CBFTest:Snapshot','Invalid snapshot');end
        sr=L.sensor.result{k};if ~isstruct(sr)||~isfield(sr,'cbfObservedWorld')||~isstruct(sr.cbfObservedWorld)||~isfield(sr.cbfObservedWorld,'parts'),error('CBFTest:Sensor','Missing sensor snapshot');end
        if ~isfield(sr,'cbfSensorTime')||~isscalar(sr.cbfSensorTime)||~isfinite(sr.cbfSensorTime)||abs(double(sr.cbfSensorTime)-s.t)>1.5*dt,error('CBFTest:SensorTime','Sensor timestamp inconsistent');end
        s.obs=sr.cbfObservedWorld;
        if isfield(s.obs,'dynamicIndices')&&~isempty(s.obs.dynamicIndices),error('CBFTest:MovingObstacle','Moving obstacles unsupported');end
        s.count=numel(s.obs.parts);s.recordedSuccess=c.cbf.success;s.loggedH=c.cbf.paperCurrentH;
        s.onlineMode=char(r.mode);s.onlineOffset=ascol(r.signedCarryPosition,3);
        s.onlineCompare=char(r.physicalPrevious.comparisonStatus);
        raw=nominal_hlc(hlc,s.x,s.sent,s.P,false);er=max(abs(raw-s.raw));
        if any(abs(raw-s.raw)>1e-7*(1+abs(s.raw))),error('CBFTest:HLCReplay','Raw HLC mismatch');end
        % Replay actual QP with exact recorded raw and independent copied cfg.
        [u,info]=solve(s.x,s.hold(1:2),s.raw,s,hlc,dt);
        he=NaN;ue=NaN;ee=NaN;hh=NaN;
        if info.success
            a=info.paperHold;he=max(abs([a.T0;a.q0;a.nu;a.Tend;a.qEnd]-s.hold));ue=max(abs(u-s.u));
        end
        if numel(info.paperCurrentH)==numel(s.loggedH)&&~isempty(s.loggedH),hh=max(abs(info.paperCurrentH(:)-s.loggedH(:)));end
        % Actual applied one-step reconstruction, including logged fallback.
        try,y=checked_propagate(s.x,s.u,s.hold(1:3).',s.P,dt);ee=max(abs(y-s.xnext));catch,end
        rawErr(end+1)=er;holdErr(end+1)=he;inputErr(end+1)=ue;endErr(end+1)=ee;hErr(end+1)=hh;obsCounts(end+1)=s.count; %#ok<AGROW>
        good=(logical(info.success)==logical(s.recordedSuccess));
        if info.success&&s.recordedSuccess,good=good&&he<=1e-6&&ue<=1e-7;end
        if ~good,bad=bad+1;end
        gtag='OK';if ~good,gtag='QP_REPLAY_MISMATCH';end
        gates{end+1}=[numline(s.t),',',csvtext(gtag),',',numline([s.recordedSuccess;info.success;er;he;ue;ee;hh;s.count])]; %#ok<AGROW>
        ss=[s.t;s.endt;s.P;s.x;s.xnext;s.base;s.sent;s.raw;s.u;s.hold];assert(numel(ss)==numel(sh));snaps{end+1}=numline(ss); %#ok<AGROW>
        if ~good,summary=summary+sprintf('FIRST_QP_REPLAY_MISMATCH t=%.17g\n',s.t);break;end
        S{k-first+1}=s;
    catch ME
        summary=summary+sprintf('SNAPSHOT_FAILURE t=%.17g %s\n',t(k),status_text(ME));break;
    end
end
N=find(cellfun(@isempty,S),1)-1;if isempty(N),N=numel(S);end
summary=summary+sprintf('valid_snapshot_prefix=%d/%d\nrecorded_QP_mismatch=%d\n',N,numel(S),bad);
summary=summary+metric_line('recorded_raw_max',rawErr)+metric_line('recorded_hold_max',holdErr)+metric_line('recorded_applied_max',inputErr)+metric_line('recorded_endpoint_state_max',endErr)+metric_line('recorded_h_max',hErr);
if N==0,summary=summary+"STATUS=REPLAY_GATE_FAILED_NO_BRANCHES"+newline;return;end
if N~=numel(S)||bad>0
    summary=summary+"REPLAY_GATE=PREFIX_ONLY; compare only the validated contiguous prefix, do not extrapolate."+newline;
else
    summary=summary+"REPLAY_GATE=FULL_RECORDED_HORIZON"+newline;
end
fprintf('[CBF CLOSED LOOP TEST] recorded HLC/QP replay complete: %d intervals\n',N);
for v=1:numel(labels)
    fprintf('[CBF CLOSED LOOP TEST] %s\n',labels{v});
    x=S{1}.x;tq=S{1}.hold(1:2);co=[];
    st=struct('mode','NOMINAL','correction',zeros(3,7),'center',zeros(3,1),'anchor',zeros(3,1),'quietCount',0);
    status='FIRST';prev=CBF_TRAJ_PHYSICAL_DIAG('FIRST');
    valid=0;lastt=S{1}.t;finish='RECORDED_HORIZON';maxP=zeros(3,1);maxV=zeros(3,1);maxRaw=zeros(4,1);minH=Inf;neg=0;interventions=0;firstReturn=NaN;firstDegraded=NaN;firstQuiet=NaN;quietV=[];quietD=[];posErr=[];velErr=[];mismatchA=[];quietRun=0;maxQuietRun=0;returnEntries=0;reentries=0;rawOutside=0;
    for j=1:N
        s=S{j};timer=tic;raw=nan(4,1);u=raw;y=nan(19,1);hold=nan(5,1);qpstatus='NOT_RUN';ok=false;hm=NaN;sm=NaN;dT=NaN;dTau=NaN;outside=NaN;next=CBF_TRAJ_PHYSICAL_DIAG('NOT_RUN');new=st;update='NOT_RUN';
        target=s.base+reshape([st.correction;zeros(1,7)],28,1);
        ep=x(8:10)-target(1:3);ev=x(11:13)-target(5:7);mode=st.mode;
        try
            if strcmp(mode,'RETURN')&&isnan(firstReturn),firstReturn=s.t;end
            if strcmp(mode,'DEGRADED')&&isnan(firstDegraded),firstDegraded=s.t;end
            if v==1,mismatchA(end+1,:)=(st.correction(:,1)-s.onlineOffset).';end %#ok<AGROW>
            raw=nominal_hlc(hlc,x,target,s.P,true);
            outside=any(raw<[0;-1;-1;-1]-1e-8|raw>[20;1;1;1]+1e-8);rawOutside=rawOutside+outside;
            maxRaw=max(maxRaw,abs(raw));
            [u,info]=solve(x,tq,raw,s,hlc,dt);qpstatus=char(info.status);
            hm=minimum(info.paperCurrentH);sm=minimum(info.stageValues);
            if isfinite(hm),minH=min(minH,hm);neg=neg+(hm<0);end
            if ~info.success,error('CBFTest:QPFailed','%s',qpstatus);end
            h=info.paperHold;hold=[h.T0;h.q0;h.nu;h.Tend;h.qEnd];
            y=checked_propagate(x,u,hold(1:3).',s.P,dt);
            % Same causal actual interval and original-reference comparison as REV30.
            sent=target;q=x(1:4);yaw=atan2(2*(q(1)*q(4)+q(2)*q(3)),1-2*(q(3)^2+q(4)^2));sent(4)=yaw-atan2(sin(yaw-target(4)),cos(yaw-target(4)));
            c=struct('x',x,'tmp',raw,'input',u,'xd',sent,'cbf',info);
            track=strcmp(mode,'CARRY')||(strcmp(mode,'RETURN')&&~isempty(co));allow=strcmp(mode,'NOMINAL')||track;
            ref=struct('before28',s.base,'after28',target);
            [~,next,coNext]=CBF_TRAJ_PHYSICAL_HOLD_DIFF(c,s.P,dt,3,hlc,co,track,allow,ref);
            if ~next.valid,error('CBFTest:PhysicalInvalid','%s',next.status);end
            dT=next.deltaControl(1);dTau=norm(next.deltaControl(2:4));
            interventions=interventions+next.active;
            [new,update,keep]=test_update(st,next,E,v);
            if strcmp(new.mode,'RETURN')&&~strcmp(mode,'RETURN'),returnEntries=returnEntries+1;end
            if strcmp(mode,'RETURN')&&strcmp(new.mode,'CARRY'),reentries=reentries+1;end
            if ~keep,coNext=[];end
            if ~next.active&&next.returnGate
                if isnan(firstQuiet),firstQuiet=s.endt;end
                quietRun=quietRun+1;maxQuietRun=max(maxQuietRun,quietRun);
                if next.comparisonValid,quietV(end+1,:)=[s.endt,next.modelOffsetPV(4:6).'];quietD(end+1,:)=[s.endt,next.modelOffsetPV(1:3).'];end %#ok<AGROW>
            else,quietRun=0;end
            maxP=max(maxP,abs(st.correction(:,1)));maxV=max(maxV,abs(st.correction(:,2)));
            posErr(end+1,:)=ep.';velErr(end+1,:)=ev.'; %#ok<AGROW>
            ok=true;valid=valid+1;lastt=s.endt;
        catch ME
            finish=status_text(ME);
        end
        tags={labels{v},mode,status,qpstatus,char(prev.comparisonStatus)};
        vals=[prev.active;prev.returnGate;st.quietCount;prev.comparisonValid;ok;1000*toc(timer);s.base;target;st.correction(:);x;y;prev.modelOffsetPV(:);raw;u;hold;ep;ev;hm;sm;outside;dT;dTau;s.count;next.active;next.returnGate;next.comparisonValid;new.quietCount;next.modelOffsetPV(:)];
        row=[csvtext(tags{1}),',',numline([s.t;s.endt]),',',strjoin(cellfun(@csvtext,tags(2:end),'UniformOutput',false),','),',',numline(vals),',',csvtext(new.mode),',',csvtext(update),',',csvtext(next.comparisonStatus)];
        assert(numel(vals)+10==numel(head),'CBFTest:CSVSchema');rows{end+1}=row; %#ok<AGROW>
        if ~ok,break;end
        x=y;tq=hold(4:5);co=coNext;st=new;status=update;prev=next;
    end
    summary=summary+sprintf('\n[%s]\ncompleted=%d/%d\nlast_endpoint=%.17g\nstop=%s\nfirst_quiet=%.17g\nfirst_return=%.17g\nfirst_degraded=%.17g\nreturn_entries=%d\nreentries=%d\ninterventions=%d\nmax_quiet_run=%d\nraw_outside_box=%d\nmin_h=%.17g\nnegative_h_samples=%d\n',labels{v},valid,N,lastt,finish,firstQuiet,firstReturn,firstDegraded,returnEntries,reentries,interventions,maxQuietRun,rawOutside,minH,neg);
    summary=summary+sprintf('max_position_offset_xyz=%s\nmax_velocity_offset_xyz=%s\nmax_raw_abs=%s\nfinal_state=%s\nfinal_mode=%s\n',mat2str(maxP.',9),mat2str(maxV.',9),mat2str(maxRaw.',9),mat2str(x.',9),st.mode);
    if ~isempty(posErr),summary=summary+sprintf('position_RMS_xyz=%s\nvelocity_RMS_xyz=%s\n',mat2str(sqrt(mean(posErr.^2,1)),9),mat2str(sqrt(mean(velErr.^2,1)),9));end
    if ~isempty(quietV),summary=summary+sprintf('first_quiet_comparison_velocity_t_xyz=%s\nlast_quiet_comparison_velocity_t_xyz=%s\nfirst_quiet_comparison_position_t_xyz=%s\nlast_quiet_comparison_position_t_xyz=%s\n',mat2str(quietV(1,:),9),mat2str(quietV(end,:),9),mat2str(quietD(1,:),9),mat2str(quietD(end,:),9));end
    if ~isempty(mismatchA),summary=summary+sprintf('A_vs_recorded_offset_max_xyz=%s (independent integration; not required exact)\n',mat2str(max(abs(mismatchA),[],1),9));end
end
summary=summary+"STATUS=COMPLETE_CONDITIONAL_MODEL_TEST_NOT_FLIGHT_CERTIFICATION"+newline;
end

function [u,info]=solve(x,tq,raw,s,hlc,dt)
% Scheme accesses only agent.sensor.result; use local value struct. No live calls.
cfg=hlc.cbfConfig;cfg.paperThrustState=struct('initialized',true,'T',tq(1),'q',tq(2));
a=struct('sensor',struct('result',struct('cbfObservedWorld',s.obs)));
[u,info]=CBF_REAL_SCHEME_03_PAPER_EXTENSION(a,cfg,x,raw,[s.P(:);0;0],dt);
u=u(:);
end
function unit_check(E)
% Synthetic mode-state checks: A agrees with installed REV30, B/D/E
% preserve signed XYZ position while removing extra velocity, and quiet
% gating variants cannot wait forever on positive d''w.
st=struct('mode','NOMINAL','correction',zeros(3,7),'center',zeros(3,1),'anchor',zeros(3,1),'quietCount',0);
p=CBF_TRAJ_PHYSICAL_DIAG('OK');p.valid=true;p.active=true;p.comparisonValid=true;p.comparisonStarted=true;p.comparisonStatus='OK';p.modelOffsetPV=[1;-2;0.1;0.3;-0.4;0.01];
for v=1:5
    [s,tag,keep]=test_update(st,p,E,v);
    assert(strcmp(s.mode,'CARRY')&&keep&&isequal(s.correction(:,1),p.modelOffsetPV(1:3)));
    if any(v==[2,4,5]),assert(all(s.correction(:,2:7)==0,'all'));else,assert(isequal(s.correction(:,2),p.modelOffsetPV(4:6)));end
    if v==1,[ref,refTag,refKeep]=CBF_TRAJ_CORRECTION_STEP(st,p,E);assert(isequaln(s,ref)&&strcmp(tag,refTag)&&keep==refKeep);end
    q=p;q.active=false;q.returnGate=true;q.comparisonStarted=false;q.modelOffsetPV(1)=1.1;
    for k=1:8
        prior=s;[s,tag,keep]=test_update(s,q,E,v);
        if v==1,[ref,refTag,refKeep]=CBF_TRAJ_CORRECTION_STEP(prior,q,E);assert(isequaln(s,ref)&&strcmp(tag,refTag)&&keep==refKeep);end
    end
    if any(v==[1,2]),assert(strcmp(s.mode,'CARRY'));else,assert(strcmp(s.mode,'RETURN'));end
    if v==5,assert(s.correction(1,1)==1);end
end
end
function E=return_transition(dt)
A=zeros(7);A(1:6,2:7)=eye(6);for j=0:6,A(7,j+1)=-nchoosek(7,j)*1.5^(7-j);end
E=expm(A*dt);
end
function x=minimum(a)
if isempty(a),x=NaN;else,x=min(a(:));end
end
function s=metric_line(name,x)
x=x(isfinite(x));if isempty(x),s=string(name)+"=NA"+newline;else,s=string(name)+sprintf('=%.17g\n',max(abs(x)));end
end

function [st,status,keepComparison]=test_update(st,prev,E,variant)
% REV30 pure causal mode/jet update. E is the exact 7th-order homogeneous
% return transition expm(A*dt). CARRY overwrites the TOTAL original-reference offset;
% it does not integrate it, filter it or add it twice.
prior=st.mode;status=prev.status;keepComparison=false;
% Only a previously selected RETURN law is advanced. A valid new CARRY
% snapshot already describes the end of the completed interval.
if strcmp(prior,'RETURN')
    st.correction=st.correction*E.';
end
if any(~isfinite(st.correction(:)))
    error('CBFTraj:CorrectionNonfinite','Internal return jet failed');
end
if ~prev.valid
    st=hold_invalid(st);return
end
if strcmp(prior,'DEGRADED')
    % Do not re-seed/accumulate an invalid comparison in this episode.
    status='COMPARISON_LATCHED_OFF';
elseif prev.active || strcmp(prior,'CARRY')
    if ~strcmp(prior,'CARRY')
        % An original-reference comparison is a TOTAL offset, not an
        % increment relative to the remaining RETURN offset.
        % Never reseed from the actual state during RETURN.
        if strcmp(prior,'RETURN') && prev.comparisonStarted
            st=hold_invalid(st);status='RETURN_BASELINE_RESTART_REJECTED';return
        end
        st.anchor=zeros(3,1);st.mode='CARRY';
    end
    if ~prev.comparisonValid
        st=hold_invalid(st);status=['COMPARISON_INVALID:' prev.comparisonStatus];return
    end
    pv=prev.modelOffsetPV(:);
    if numel(pv)~=6 || ~isreal(pv) || any(~isfinite(pv))
        st=hold_invalid(st);status='COMPARISON_TARGET_INVALID';return
    end
    candidate=pv(1:3);
    st.anchor=zeros(3,1);
    % Retain REV20's diagnostic domain (not a physical safety constraint).
    if norm(candidate,Inf)>5 || norm(pv(4:6),Inf)>5
        st=hold_invalid(st);status='COMPARISON_TARGET_DOMAIN';return
    end
    % B/E/D keep the original velocity jet; only positions are added.
    if any(variant==[2,4,5]),pv(4:6)=0;end
    % E: when the previous physical intervention has stopped and the gate
    % is feasible, keep the last applied offset during the existing quiet window.
    if variant==5 && strcmp(prior,'CARRY') && ~prev.active && prev.returnGate
        candidate=st.correction(:,1);
    end
    st.correction=[candidate,pv(4:6),zeros(3,5)];
    st.center=candidate;keepComparison=true;
elseif strcmp(prior,'RETURN')
    % Maintain the same comparison baseline without using its offset for
    % the analytic return. If invalid, discard it and prohibit reseeding
    % until NOMINAL. A later intervention then enters DEGRADED.
    keepComparison=prev.comparisonValid && ~prev.comparisonStarted;
    if ~keepComparison && ~strcmp(prev.comparisonStatus,'NOT_USED')
        status=['RETURN_BASELINE_LOST:' prev.comparisonStatus];
    end
end
if prev.active || ~prev.returnGate
    st.quietCount=0;
    if strcmp(st.mode,'RETURN'),st=hold_invalid(st);keepComparison=false;end
    return
end
if strcmp(st.mode,'NOMINAL'),st.quietCount=0;return;end
st.quietCount=st.quietCount+1;
if st.quietCount>=8 && any(strcmp(st.mode,{'CARRY','DEGRADED'}))
    % Same gate as REV20: do not discard a still-growing episode effect.
    if any(variant==[1,2]) && strcmp(st.mode,'CARRY') && prev.comparisonValid && ...
            dot(prev.modelOffsetPV(1:3),prev.modelOffsetPV(4:6))>0
        status='QUIET_WAIT_OUTWARD_MODEL_MOTION';return
    end
    st.mode='RETURN';st.center=zeros(3,1);
    % CARRY retains the valid baseline for a possible re-intervention.
    % DEGRADED has no valid baseline to retain.
    % No return propagation at this switch: current jet is its initial jet.
end
if strcmp(st.mode,'RETURN') && max(abs(st.correction),[],'all')<=1e-5
    st.mode='NOMINAL';st.correction=zeros(3,7);st.center=zeros(3,1);
    st.anchor=zeros(3,1);st.quietCount=0;keepComparison=false;
end
end

function st=hold_invalid(st)
st.quietCount=0;
if strcmp(st.mode,'NOMINAL'),return;end
% Position remains at the last accepted value (or current analytic RETURN
% endpoint). No stale velocity extrapolation and no invalid displacement.
% Derivative reset is deliberate; inter-sample C6 is not required here.
st.correction(:,2:7)=0;st.center=st.correction(:,1);st.mode='DEGRADED';
end

function y=checked_propagate(x,u,p,P,dt)
project_state(x);[~,r0]=rhs(x,at(u,p,0),P);
y=project_state(propagate(x,u,p,P,dt,2));[~,r1]=rhs(y,at(u,p,dt),P);
if any(~isfinite([r0,r1]))||max([r0,r1])>=0,error('CBFBasis:CableSlack','Nonpositive cable tension or nonfinite radial acceleration');end
end
function x=ascol(x,n)
if ~isnumeric(x)||~isreal(x)||numel(x)~=n,error('CBFBasis:SnapshotShape','Unexpected numeric vector shape');end
x=double(x(:));
end
function n=vecnames(s,k)
n=arrayfun(@(j)sprintf('%s_%d',s,j),1:k,'UniformOutput',false);
end
function s=numline(v)
s=sprintf('%.17g,',v(:));s(end)=[];
end
function s=csvtext(s)
s=['"',strrep(char(s),'"','""'),'"'];
end
function s=status_text(ME)
s=[ME.identifier,': ',strrep(strrep(ME.message,newline,' '),char(13),' ')];
end
function s=join_lines(c)
s=[strjoin(c,newline),newline];
end
function put_entry(z,name,s)
e=java.util.zip.ZipEntry(name);z.putNextEntry(e);
b=unicode2native(s,'UTF-8');b=typecast(uint8(b(:)),'int8');
z.write(b,0,numel(b));z.closeEntry();
end
function close_zip(z)
try,z.close();catch,end
end
function delete_if_exists(p)
if exist(p,'file'),delete(p);end
end

function raw=nominal_hlc(hlc,x,xd,P,adjustYaw)
% Verbatim generated pure-function sequence of HLC_SUSPENDED_LOAD_CBF_REAL.do.
% The methods take all numerical inputs explicitly; no live controller do().
if adjustYaw
    q=x(1:4);yaw=atan2(2*(q(1)*q(4)+q(2)*q(3)),1-2*(q(3)^2+q(4)^2));
    xd(4)=yaw-atan2(sin(yaw-xd(4)),cos(yaw-xd(4)));
end
P=[P(:).',0,0];a=hlc.param;
vf=hlc.Vfd_SuspendedLoadxyDst(a.dt,x,xd.',a.F1);
vs=hlc.Vs_SuspendedLoadxyDst(x,xd.',vf,P,a.F2,a.F3,a.F4);
uf=hlc.Uf_SuspendedLoadxyDst(x,xd.',vf,P);
beta=hlc.Beta2_SuspendedLoadxyDst(x,xd.',vf,P);
b=hlc.V2_alpha2_SuspendedLoadxyDst(x,xd.',vf,vs.',P);
if any(~isfinite(beta(:))) || rcond(beta)<1e-12,error('CBFTraj:ComparisonHLCIllConditioned','Ill-conditioned HLC decoupling matrix');end
raw=[uf(1);beta\b];
if ~isreal(raw) || any(~isfinite(raw)),error('CBFTraj:ComparisonHLCNonfinite','Nonfinite or nonreal HLC result');end
end

function y=propagate(x,u,thrust,P,dt,nSub)
h=dt/nSub;y=x;
for k=1:nSub
    t=(k-1)*h;
    k1=rhs(y,at(u,thrust,t),P);
    k2=rhs(y+h*k1/2,at(u,thrust,t+h/2),P);
    k3=rhs(y+h*k2/2,at(u,thrust,t+h/2),P);
    k4=rhs(y+h*k3,at(u,thrust,t+h),P);
    y=y+h*(k1+2*k2+2*k3+k4)/6;
end
end

function uk=at(u,profile,s)
uk=u;
uk(1)=profile(1)+profile(2)*s+0.5*profile(3)*s^2;
end

function [out,radial]=rhs(x,u,p)
% Same 19-state suspended-load ideal dynamics used in V81 research.
% x=[quaternion(4); bodyOmega(3); loadP(3); loadV(3);
%    cableDirection(3); cableOmega(3)].
m=p(1);J=p(2:4);g=p(5);ml=p(6);L=p(7);
q=x(1:4);omega=x(5:7);dir=x(14:16);omegaL=x(17:19);
b3=[2*(q(2)*q(4)+q(1)*q(3)); ...
    2*(q(3)*q(4)-q(1)*q(2)); ...
    q(1)^2-q(2)^2-q(3)^2+q(4)^2];
qd=0.5*[-q(2:4)'*omega; q(1)*omega+cross(q(2:4),omega)];
wd=(u(2:4)-cross(omega,J.*omega))./J;
crossLD=cross(omegaL,dir);
radial=(u(1)*dot(dir,b3)-m*L*dot(crossLD,crossLD))/(m+ml);
out=[qd;wd;x(11:13);-g*[0;0;1]+radial*dir; ...
    crossLD;-(u(1)/(m*L))*cross(dir,b3)];
end


function y=project_state(y)
if any(~isfinite(y)) || abs(norm(y(1:4))-1)>0.01 || ...
        abs(norm(y(14:16))-1)>0.01
    error('CBFTraj:ComparisonManifold','Invalid integrated manifold state');
end
y(1:4)=y(1:4)/norm(y(1:4));y(14:16)=y(14:16)/norm(y(14:16));
end
