function zipPath=CBF_TRAJ_RETURN_JET_TEST()
% REV37 diagnostic only. Run after stopped REV30 GUI flight, no Sim edits.
% Independent governor/sensor/plant branches, fixed mass; no live mutation.
% Outputs one ZIP beside this file; no directories or standalone CSVs.
outDir=fileparts(mfilename('fullpath'));
zipPath=fullfile(outDir,'CBF_TRAJ_RETURN_JET_RESULTS.zip');
summary="REV37 ADAPTIVE MODEL-IN-LOOP TEST"+newline;
replay={}; gates={}; snaps={}; jets={};
try
    if ~evalin('base','exist(''app'',''var'')'),error('CBFTest:AppMissing','Run REV30 simulation, keep app.');end
    app=evalin('base','app');
    if ~isa(app,'SimExp')||~isvalid(app)||~strcmp(string(app.cha),"s"),error('CBFTest:NotStopped','Use stopped SimExp app.');end
    logger=app.logger;hlc=app.agent(1).controller.hlc_suspended;
    wp=app.agent(1).reference.waypoint;sen=app.agent(1).sensor.cbf_spherical;
    if ~isa(wp,'WAYPOINT_REFERENCE')||~isa(sen,'CBF_SIM_SPHERICAL_SENSOR'),error('CBFTest:Config','Expected waypoint and spherical sensor objects');end
    cfg=struct('generator',wp.generator,'adaptive',wp.adaptiveProgress,'start',wp.slowdownStart, ...
        'full',wp.slowdownFull,'minScale',wp.minScale,'filterTime',wp.filterTime, ...
        'world',sen.world0,'sensor',sen.config,'te',double(app.time.te));
    if any([cfg.world.baseObjects.isDynamic]),error('CBFTest:World','Only static obstacle worlds supported');end
    [summary,replay,gates,snaps,jets]=audit(logger,hlc,summary,cfg);
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
if ~isempty(jets),put_entry(zos,'JET_AUDIT.csv',join_lines(jets));end
zos.close();clear zc
[ok,msg]=movefile(tmp,zipPath,'f');if ~ok,error('CBFTest:ZIPMove','%s',msg);end
clear clean
fprintf('[CBF CLOSED LOOP TEST] %s\n%s\n',zipPath,char(summary));
end

function [summary,rows,gates,snaps,jets]=audit(logger,hlc,summary,cfg)
jets={};
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
summary=summary+"Branches R23=archived baseline; D_BASIC=REV35 D_PROGRESS control; D_OUTWARD=same with old dw entry gate; D_CLEAR=same with eight quiet samples while observed parts empty. All D branches use shifted governor and position-only correction; same return dynamics. No online mode added."+newline;
summary=summary+"Independent branch states; original generator and exact wp_governor_step, branch-specific sensor from model drone position. Fixed P at episode start (no EKF or truth sensor simulation); original HLC and ID3 solver. Model-in-loop, not GUI reproduction or safety certificate. Horizon configured app.time.te plus 20 s diagnostic extension, not route completion requirement."+newline;
summary=summary+"Jet audit: methods 4/5 use new residual Bernstein evaluation; no tolerance relaxation; endpoint check added. All 5 methods every 20 ticks and mode changes; 0..6 values at s=0, Gauss integral identities 0..5 with next derivative, one-sided limits through 6, RETURN expm endpoint, per-tick direct-law cross-cycle jumps. Numerical tolerances only. A jet failure is logged and does not change the branch control. Cross-cycle continuity is reported but not required."+newline;
summary=summary+"Replay gate: raw, successful applied/hold, QP success, h, finite endpoint error <= 1e-3 in mixed state coordinates (diagnostic tolerance, not safety). Recorded-prefix gate validates source; future branches use autonomous model, not invalid records. Original governor reconstruction must match logged before28 at episode start and all usable prefix times."+newline;
E=return_transition(dt);unit_check(E);stable_unit_check(dt);summary=summary+"MODE_UPDATE_UNIT_CHECK=PASS"+newline+"RESIDUAL_JET_UNIT_CHECK=PASS"+newline;
labels={'R23_BASELINE','D_BASIC','D_OUTWARD','D_CLEAR'};
variants=[23,4,2,6];
head={'branch','source_t','end_t','mode','update_status','qp_status','compare_status','active','gate','quiet','comparison_valid','success','ms'};
head=[head,vecnames('original28',28),vecnames('command28',28),vecnames('correction',21),vecnames('state',19),vecnames('end_state',19),vecnames('nominal_offset',6),vecnames('raw',4),vecnames('applied',4),vecnames('TqNuTendQend',5),vecnames('position_error',3),vecnames('velocity_error',3),{'h_min','stage_min','raw_outside_box','deltaTend','deltaTauNorm','obs_count','next_active','next_gate','next_comparison_valid','next_quiet'},vecnames('next_offsetPV',6),{'next_mode','next_update_status','next_compare_status','virtual_time','governor_rate','governor_command','governor_error','jet_checked','jet_pass'},vecnames('cycle_jump',7)];
rows={strjoin(head,',')};
jets={strjoin([{'branch','source_t','method','mode','pass','zero_error'},vecnames('integral_error',6),vecnames('integral_ratio',6),vecnames('max_abs_jet',7),{'return_endpoint_error'},vecnames('limit_error',7),vecnames('limit_ratio',7),{'base_endpoint_scaled_error','audit_status'}],',')};
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
        gtag='OK';
        if ~good,gtag='QP_REPLAY_MISMATCH';
        elseif ~isfinite(ee)||ee>1e-3,good=false;gtag='ENDPOINT_REPLAY_MISMATCH';
        elseif ~isempty(s.loggedH)&&(~isfinite(hh)||hh>1e-8*(1+max(abs(s.loggedH(:))))),good=false;gtag='H_REPLAY_MISMATCH';end
        if ~good,bad=bad+1;end
        gates{end+1}=[numline(s.t),',',csvtext(gtag),',',numline([s.recordedSuccess;info.success;er;he;ue;ee;hh;s.count])]; %#ok<AGROW>
        ss=[s.t;s.endt;s.P;s.x;s.xnext;s.base;s.sent;s.raw;s.u;s.hold];assert(numel(ss)==numel(sh));snaps{end+1}=numline(ss); %#ok<AGROW>
        if ~good,summary=summary+sprintf('FIRST_REPLAY_MISMATCH t=%.17g %s\n',s.t,gtag);break;end
        S{k-first+1}=s;
    catch ME
        summary=summary+sprintf('SNAPSHOT_FAILURE t=%.17g %s\n',t(k),status_text(ME));break;
    end
end
N=find(cellfun(@isempty,S),1)-1;if isempty(N),N=numel(S);end
summary=summary+sprintf('valid_snapshot_prefix=%d/%d\nrecorded_replay_mismatch=%d\n',N,numel(S),bad);
summary=summary+metric_line('recorded_raw_max',rawErr)+metric_line('recorded_hold_max',holdErr)+metric_line('recorded_applied_max',inputErr)+metric_line('recorded_endpoint_state_max',endErr)+metric_line('recorded_h_max',hErr);
if N==0,summary=summary+"STATUS=REPLAY_GATE_FAILED_NO_BRANCHES"+newline;return;end
if N~=numel(S)||bad>0
    summary=summary+"REPLAY_GATE=PREFIX_ONLY; independent branches use no subsequent recorded states/inputs."+newline;
else
    summary=summary+"REPLAY_GATE=FULL_RECORDED_HORIZON"+newline;
end
% Reconstruct held progress command and six filter states from beginning of flight.
% Read only the historic controller states, never mutate the waypoint handle.
try
    [g0,gerr]=reconstruct_governor(L,t,phase,first,cfg,S{1}.P(7));
catch ME
    summary=summary+"STATUS=GOVERNOR_INIT_FAILED "+string(status_text(ME))+newline;return
end
summary=summary+sprintf('governor_initial_replay_max=%.17g\n',gerr);
g=g0;gerr=0;
for j=1:N
    sj=S{j};[base,g]=governor_sample(g,sj.x,sj.P(7),cfg,zeros(3,1),false);
    gerr=max(gerr,max(abs(base-sj.base)./(1+abs(sj.base))));
    g=governor_advance(g,dt,cfg);
end
summary=summary+sprintf('governor_prefix_replay_scaled_max=%.17g\n',gerr);
if gerr>1e-7,summary=summary+"STATUS=GOVERNOR_PREFIX_REPLAY_FAILED"+newline;return;end
initial=S{1};P=initial.P;Nbranch=round((cfg.te+20-initial.t)/dt);
if Nbranch<1||Nbranch>20000,error('CBFTest:Horizon','Invalid branch horizon');end
summary=summary+sprintf('branch_P=%s\nconfigured_te=%.17g\nbranch_end=%.17g\nbranch_intervals=%d\n',mat2str(P.',17),cfg.te,initial.t+Nbranch*dt,Nbranch);
fprintf('[CBF ADAPTIVE TEST] replay validated: %d intervals, independent horizon %d\n',N,Nbranch);
for v=1:numel(labels)
    fprintf('[CBF CLOSED LOOP TEST] %s\n',labels{v});
    x=initial.x;tq=initial.hold(1:2);co=[];g=g0;vtype=variants(v);priorJet=[];prevMode='';jetFails=0;jetSamples=0;maxJump=zeros(1,7);maxJet=zeros(1,7);atTe='NOT_REACHED';recovered=NaN;minZ=Inf;
    st=struct('mode','NOMINAL','correction',zeros(3,7),'center',zeros(3,1),'anchor',zeros(3,1),'quietCount',0);
    status='FIRST';prev=CBF_TRAJ_PHYSICAL_DIAG('FIRST');
    valid=0;lastt=S{1}.t;finish='INDEPENDENT_HORIZON';maxP=zeros(3,1);maxV=zeros(3,1);maxRaw=zeros(4,1);minH=Inf;neg=0;interventions=0;firstReturn=NaN;firstDegraded=NaN;firstQuiet=NaN;quietV=[];quietD=[];posErr=[];velErr=[];mismatchA=[];quietRun=0;maxQuietRun=0;returnEntries=0;reentries=0;rawOutside=0;
    for j=1:Nbranch
        s=struct('t',initial.t+(j-1)*dt,'endt',initial.t+j*dt,'P',P,'base',nan(28,1),'count',NaN);
        timer=tic;raw=nan(4,1);u=raw;y=nan(19,1);hold=nan(5,1);qpstatus='NOT_RUN';ok=false;hm=NaN;sm=NaN;dT=NaN;dTau=NaN;outside=NaN;next=CBF_TRAJ_PHYSICAL_DIAG('NOT_RUN');new=st;update='NOT_RUN';
        target=nan(28,1);ep=nan(3,1);ev=nan(3,1);mode=st.mode;
        checked=false;jetPass=NaN;jump=nan(1,7);
        try
            [s.base,g]=governor_sample(g,x,P(7),cfg,st.correction(:,1),v~=1);
            [s.obs,~]=ra_sim_sensor_detect(obs_world_at_time(cfg.world,s.t),x(8:10)-P(7)*x(14:16),cfg.sensor);
            s.count=numel(s.obs.parts);
            target=s.base+reshape([st.correction;zeros(1,7)],28,1);
            ep=x(8:10)-target(1:3);ev=x(11:13)-target(5:7);
            local=@(u) local_jet(1,u,dt,s.base,g,cfg,P(7),mode,st.correction);
            if ~isempty(priorJet),jump=max(abs(reshape(target-priorJet,4,7)),[],1);maxJump=max(maxJump,jump);end
            maxJet=max(maxJet,max(abs(reshape(target,4,7)),[],1));
            if mod(j-1,20)==0||~strcmp(mode,prevMode)
                [jr,jetPass]=jet_audit(labels{v},s.t,dt,s.base,target,g,cfg,P(7),mode,st.correction,E);
                jets=[jets,jr];checked=true;jetSamples=jetSamples+1;jetFails=jetFails+~jetPass; %#ok<AGROW>
            end
            priorJet=local(dt);prevMode=mode;
            if any(~isfinite(target))||any(~isfinite(priorJet)),error('CBFTest:JetNonfinite','Nonfinite local jet');end
            minZ=min(minZ,x(10));
            if isnan(recovered)&&strcmp(mode,'NOMINAL')&&interventions>0,recovered=s.t;end
            if abs(s.t-cfg.te)<dt/2,atTe=sprintf('mode=%s pL=%s offset=%s tau=%.9g',mode,mat2str(x(8:10).',9),mat2str(st.correction(:,1).',9),g.tau);end
            if strcmp(mode,'RETURN')&&isnan(firstReturn),firstReturn=s.t;end
            if strcmp(mode,'DEGRADED')&&isnan(firstDegraded),firstDegraded=s.t;end
            % No reference to recorded state after initialization.
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
            if vtype==23
                track=strcmp(mode,'CARRY');allow=~strcmp(mode,'DEGRADED');
                [~,next,coNext]=rev23_physical(c,s.P,dt,3,hlc,co,track,allow);
            else
                track=strcmp(mode,'CARRY')||(strcmp(mode,'RETURN')&&~isempty(co));allow=strcmp(mode,'NOMINAL')||track;
                ref=struct('before28',s.base,'after28',target);
                [~,next,coNext]=CBF_TRAJ_PHYSICAL_HOLD_DIFF(c,s.P,dt,3,hlc,co,track,allow,ref);
            end
            if ~next.valid,error('CBFTest:PhysicalInvalid','%s',next.status);end
            dT=next.deltaControl(1);dTau=norm(next.deltaControl(2:4));
            interventions=interventions+next.active;
            if vtype==23,[new,update,keep]=rev23_update(st,next,E);else,[new,update,keep]=test_update(st,next,E,vtype,s.count);end
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
        row=[csvtext(tags{1}),',',numline([s.t;s.endt]),',',strjoin(cellfun(@csvtext,tags(2:end),'UniformOutput',false),','),',',numline(vals),',',csvtext(new.mode),',',csvtext(update),',',csvtext(next.comparisonStatus),',',numline([g.tau;g.rate(6);g.cmd;g.err;checked;jetPass;jump(:)])];
        assert(numel(vals)+10+13==numel(head),'CBFTest:CSVSchema');rows{end+1}=row; %#ok<AGROW>
        if ~ok,break;end
        x=y;tq=hold(4:5);co=coNext;st=new;status=update;prev=next;g=governor_advance(g,dt,cfg);
    end
    summary=summary+sprintf('\n[%s]\ncompleted=%d/%d\nlast_endpoint=%.17g\nstop=%s\nfirst_quiet=%.17g\nfirst_return=%.17g\nfirst_degraded=%.17g\nreturn_entries=%d\nreentries=%d\ninterventions=%d\nmax_quiet_run=%d\nraw_outside_box=%d\nmin_h=%.17g\nnegative_h_samples=%d\n',labels{v},valid,Nbranch,lastt,finish,firstQuiet,firstReturn,firstDegraded,returnEntries,reentries,interventions,maxQuietRun,rawOutside,minH,neg);
    summary=summary+sprintf('max_position_offset_xyz=%s\nmax_velocity_offset_xyz=%s\nmax_raw_abs=%s\nfinal_state=%s\nfinal_mode=%s\n',mat2str(maxP.',9),mat2str(maxV.',9),mat2str(maxRaw.',9),mat2str(x.',9),st.mode);
    if ~isempty(posErr),summary=summary+sprintf('position_RMS_xyz=%s\nvelocity_RMS_xyz=%s\n',mat2str(sqrt(mean(posErr.^2,1)),9),mat2str(sqrt(mean(velErr.^2,1)),9));end
    if ~isempty(quietV),summary=summary+sprintf('first_quiet_comparison_velocity_t_xyz=%s\nlast_quiet_comparison_velocity_t_xyz=%s\nfirst_quiet_comparison_position_t_xyz=%s\nlast_quiet_comparison_position_t_xyz=%s\n',mat2str(quietV(1,:),9),mat2str(quietV(end,:),9),mat2str(quietD(1,:),9),mat2str(quietD(end,:),9));end
    summary=summary+sprintf('first_nominal_after_intervention=%.17g\nmin_load_z=%.17g\njet_samples=%d\njet_failed_samples=%d\nmax_abs_jet_by_order=%s\nmax_cross_cycle_jump_by_order=%s\nconfigured_end_snapshot=%s\nfinal_virtual_time=%.17g\n',recovered,minZ,jetSamples,jetFails,mat2str(maxJet,9),mat2str(maxJump,9),atTe,g.tau);
end
summary=summary+"STATUS=COMPLETE_ADAPTIVE_MODEL_TEST_NOT_FLIGHT_CERTIFICATION"+newline;
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
[s,~,keep]=test_update(st,p,E,6,1);assert(keep&&strcmp(s.mode,'CARRY'));
for k=1:16,[s,tag,~]=test_update(s,q,E,6,1);assert(strcmp(s.mode,'CARRY')&&s.quietCount==0&&strcmp(tag,'WAIT_OBSERVED_WORLD_EMPTY'));end
for k=1:7,[s,~,~]=test_update(s,q,E,6,0);assert(strcmp(s.mode,'CARRY'));end
[s,~,~]=test_update(s,q,E,6,0);assert(strcmp(s.mode,'RETURN'));
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

function [st,status,keepComparison]=test_update(st,prev,E,variant,observedCount)
if nargin<5,observedCount=NaN;end
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
    if any(variant==[2,4,5,6]),pv(4:6)=0;end
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
% Diagnostic conservative entry policy, not a clearance certificate.
% Count eight successive quiet samples with no observed obstacle parts.
% Do not alter the actual physical diagnostic or the RETURN law.
if variant==6 && any(strcmp(st.mode,{'CARRY','DEGRADED'})) && ~(isfinite(observedCount)&&observedCount==0)
    st.quietCount=0;status='WAIT_OBSERVED_WORLD_EMPTY';return
end
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

function [deltaPV,diag,nextComparison]=rev23_physical(controllerResult,P,dt,cbfId,hlc,comparison,trackComparison,allowComparison)
% Single method: matched-reference closed-loop causal comparison.
% Actual and comparison use the recorded same reference; comparison HLC
% is evaluated purely on its own state (no do/CBF/live state mutation).
% Only the previous completed interval is evaluated. No future plant input.
nSub=2;
deltaPV=zeros(6,1);nextComparison=[];
diag=CBF_TRAJ_PHYSICAL_DIAG('NOT_EVALUATED');
start=tic;
if ~isnumeric(P) || ~isreal(P) || ~isvector(P) || numel(P)~=7 || any(~isfinite(P(:)))
    diag.status='INVALID_PREVIOUS_PARAMETERS';return
end
validateattributes(dt,{'numeric'},{'scalar','finite','positive'});
validateattributes(nSub,{'numeric'},{'scalar','integer','>=',1,'<=',16});
if ~isstruct(controllerResult) || ~isfield(controllerResult,'x') || ...
        ~isfield(controllerResult,'tmp') || ~isfield(controllerResult,'input')
    diag.status='CONTROLLER_SNAPSHOT_MISSING';return
end
x=double(controllerResult.x(:));raw=double(controllerResult.tmp(:));
u=double(controllerResult.input(:));P=double(P(:));
if numel(x)~=19 || numel(raw)~=4 || numel(u)~=4 || ...
        ~isreal([x;raw;u]) || any(~isfinite([x;raw;u])) || any(P<=0)
    diag.status='INVALID_PHYSICAL_SNAPSHOT';return
end
if abs(norm(x(1:4))-1)>0.01 || abs(norm(x(14:16))-1)>0.01
    diag.status='STATE_OUTSIDE_UNIT_MANIFOLD';return
end
if any(u<[0;-1;-1;-1]-1e-8) || any(u>[20;1;1;1]+1e-8)
    diag.status='APPLIED_INPUT_OUTSIDE_BOX';return
end
uBase=min(max(raw,[0;-1;-1;-1]),[20;1;1;1]);
if cbfId==3
    if ~isfield(controllerResult,'cbf') || ~isstruct(controllerResult.cbf)
        diag.status='ID3_INFO_MISSING';return
    end
    c=controllerResult.cbf;
    if isfield(c,'paperActuatorFallbackRateReset') && ...
            ~isempty(c.paperActuatorFallbackRateReset) && ...
            logical(c.paperActuatorFallbackRateReset)
        diag.status='ID3_FALLBACK_RATE_RESET_NOT_COMPARABLE';return
    end
    if ~isfield(c,'paperHold') || ~isstruct(c.paperHold)
        diag.status='ID3_HOLD_MISSING';return
    end
    h=c.paperHold;
    req={'active','T0','q0','nu','Tend','qEnd','dt'};
    if ~all(isfield(h,req)) || ~isscalar(h.active) || ~logical(h.active)
        diag.status='ID3_HOLD_INVALID';return
    end
    if ~isfield(c,'paperNominalNuRaw') || ...
            ~isfield(c,'paperAdmissibleNuAfterViability') || ...
            numel(c.paperAdmissibleNuAfterViability)~=2
        diag.status='ID3_BASELINE_ENVELOPE_UNKNOWN';return
    end
    vals=[h.T0;h.q0;h.nu;h.Tend;h.qEnd;h.dt; ...
          c.paperNominalNuRaw;c.paperAdmissibleNuAfterViability(:)];
    if any(~isfinite(vals)) || abs(h.dt-dt)>1e-10 || ...
            abs(u(1)-h.T0)>1e-8 || ...
            abs(h.Tend-(h.T0+dt*h.q0+0.5*dt^2*h.nu))>1e-8 || ...
            abs(h.qEnd-(h.q0+dt*h.nu))>1e-8
        diag.status='ID3_HOLD_INCONSISTENT';return
    end
    bounds=double(c.paperAdmissibleNuAfterViability(:));
    if bounds(1)>bounds(2)
        diag.status='ID3_EMPTY_BASELINE_ENVELOPE';return
    end
    nuBase=min(max(double(c.paperNominalNuRaw),bounds(1)),bounds(2));
    applied=[h.T0;u(2:4)];
    uBase=[h.T0;uBase(2:4)];
    diag.T0=h.T0;diag.Tdot0=h.q0;diag.TddotApplied=h.nu;
    diag.TddotBaseline=nuBase;
    diag.TendApplied=h.Tend;
    diag.TendBaseline=h.T0+dt*h.q0+0.5*dt^2*nuBase;
    diag.TdotEndApplied=h.qEnd;
    diag.TdotEndBaseline=h.q0+dt*nuBase;
    diffPhysical=[diag.TendApplied-diag.TendBaseline;u(2:4)-uBase(2:4)];
    policyA=[h.T0,h.q0,h.nu];policyB=[h.T0,h.q0,nuBase];
else
    applied=u;
    diffPhysical=applied-uBase;
    policyA=[u(1),0,0];policyB=[uBase(1),0,0];
end
diag.baseline=uBase;diag.applied=applied;diag.deltaControl=diffPhysical;
diag.active=any(abs(diffPhysical)>1e-8);
[diag.returnGate,diag.returnGateReason]=CBF_TRAJ_RETURN_GATE(controllerResult,diag,cbfId);
if ~diag.active && ~trackComparison
    diag.valid=true;diag.status='EQUIVALENT_BOUNDED_ACTUATION';
    diag.elapsedMs=1000*toc(start);return
end
try
    xA=r23_project_state(r23_propagate(x,applied,policyA,P,dt,nSub));
    if all(diffPhysical==0),xB=xA;else,xB=r23_project_state(r23_propagate(x,uBase,policyB,P,dt,nSub));end
    deltaPV=xA(8:13)-xB(8:13);
    if any(~isfinite(deltaPV)),error('CBFTraj:NonfiniteLocal','CBFTraj:NonfiniteLocal');end
catch ME
    diag.status=['LOCAL_INVALID:' ME.identifier];diag.returnGate=false;
    diag.elapsedMs=1000*toc(start);return
end
diag.valid=true;diag.status='OK';
if ~diag.active,diag.status='EQUIVALENT_BOUNDED_ACTUATION';end
if ~allowComparison
    diag.comparisonStatus='LATCHED_OFF';diag.elapsedMs=1000*toc(start);return
end
try
    if ~isfield(controllerResult,'xd') || numel(controllerResult.xd)~=28
        error('CBFTraj:ComparisonReferenceMissing','CBFTraj:ComparisonReferenceMissing');
    end
    xd=double(controllerResult.xd(:));
    if any(~isfinite(xd)),error('CBFTraj:ComparisonReferenceInvalid','CBFTraj:ComparisonReferenceInvalid');end
    if isempty(comparison)
        % Verify the generated pure HLC matches the recorded live request.
        check=r23_nominal_hlc(hlc,x,xd,P,false);
        diag.replayRawError=max(abs(check-raw));
        if any(abs(check-raw)>1e-7*(1+abs(raw)))
            error('CBFTraj:HLCReplayMismatch','Recorded raw input does not match pure HLC');
        end
        xb=x;tb=policyA(1:2).';rawB=raw;
        % At the common starting state use the exact recorded envelope.
        yb=xB;tbEnd=[diag.TendBaseline;diag.TdotEndBaseline];
        if cbfId~=3,tbEnd=[uBase(1);0];end
        diag.comparisonStarted=true;
    else
        xb=r23_project_state(r23_rebase_state(x,comparison.actual,comparison.nominal));
        tb=comparison.thrust;
        rawB=r23_nominal_hlc(hlc,xb,xd,P,true);
        diag.comparisonRaw=rawB;
        if any(rawB<[0;-1;-1;-1]-1e-8 | rawB>[20;1;1;1]+1e-8)
            error('CBFTraj:ComparisonRawOutsideBox','CBFTraj:ComparisonRawOutsideBox');
        end
        [yb,tbEnd]=r23_baseline_step(xb,tb,rawB,P,dt,nSub,cbfId,hlc.cbfConfig);
    end
    % Reject a comparison outside its configured diagnostic domain;
    % never clip its output and never retain a divergent velocity.
    diag.comparisonRaw=rawB;
    if any(~isfinite(rawB)) || any(rawB<[0;-1;-1;-1]-1e-8 | rawB>[20;1;1;1]+1e-8)
        error('CBFTraj:ComparisonRawOutsideBox','CBFTraj:ComparisonRawOutsideBox');
    end
    [~,ra0]=r23_rhs(x,r23_at(applied,policyA,0),P);
    [~,ra1]=r23_rhs(xA,r23_at(applied,policyA,dt),P);
    [~,rb0]=r23_rhs(x,r23_at(uBase,policyB,0),P);
    [~,rb1]=r23_rhs(xB,r23_at(uBase,policyB,dt),P);
    if any(~isfinite([ra0,ra1,rb0,rb1])) || max([ra0,ra1,rb0,rb1])>=0
        error('CBFTraj:ComparisonCableSlack','CBFTraj:ComparisonCableSlack');
    end
    pv=xA(8:13)-yb(8:13);
    diag.modelActual=xA;diag.modelNominal=yb;diag.modelOffsetPV=pv;
    diag.comparisonThrustEnd=tbEnd;
    if any(~isfinite([pv;tbEnd])) || norm(pv(1:3),Inf)>5 || norm(pv(4:6),Inf)>5
        error('CBFTraj:ComparisonDomain','CBFTraj:ComparisonDomain');
    end
    nextComparison=struct('actual',xA,'nominal',yb,'thrust',tbEnd);
    diag.comparisonValid=true;diag.comparisonStatus='OK';
catch ME
    diag.comparisonStatus=ME.identifier;nextComparison=[];
end
diag.elapsedMs=1000*toc(start);
end

function raw=r23_nominal_hlc(hlc,x,xd,P,adjustYaw)
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
if any(~isfinite(beta(:))) || rcond(beta)<1e-12,error('CBFTraj:ComparisonHLCIllConditioned','CBFTraj:ComparisonHLCIllConditioned');end
raw=[uf(1);beta\b];
if ~isreal(raw) || any(~isfinite(raw)),error('CBFTraj:ComparisonHLCNonfinite','CBFTraj:ComparisonHLCNonfinite');end
end

function y=r23_propagate(x,u,thrust,P,dt,nSub)
h=dt/nSub;y=x;
for k=1:nSub
    t=(k-1)*h;
    k1=r23_rhs(y,r23_at(u,thrust,t),P);
    k2=r23_rhs(y+h*k1/2,r23_at(u,thrust,t+h/2),P);
    k3=r23_rhs(y+h*k2/2,r23_at(u,thrust,t+h/2),P);
    k4=r23_rhs(y+h*k3,r23_at(u,thrust,t+h),P);
    y=y+h*(k1+2*k2+2*k3+k4)/6;
end
end

function uk=r23_at(u,profile,s)
uk=u;
uk(1)=profile(1)+profile(2)*s+0.5*profile(3)*s^2;
end

function [out,radial]=r23_rhs(x,u,p)
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


function [y,tEnd]=r23_baseline_step(x,t,raw,P,dt,nSub,cbfId,ac)
u=min(max(raw,[0;-1;-1;-1]),[20;1;1;1]);
if cbfId==3
    % Exact same actuator-only feasibility routine as ID3 itself, but on
    % the independent comparison actuator state. No live config mutation.
    ac.paperThrustState=struct('initialized',true,'T',t(1),'q',t(2));
    [u,fb]=CBF_REAL_SCHEME_03_PAPER_EXTENSION('fallback',ac,raw,dt);
    if fb.rateReset || abs(u(1)-t(1))>1e-8
        error('CBFTraj:ComparisonRateReset','Comparison actuator reset rejected');
    end
    h=fb.paperHold;policy=[h.T0,h.q0,h.nu];tEnd=[h.Tend;h.qEnd];
else
    policy=[u(1),0,0];tEnd=[u(1);0];
end
% Taut-cable model validity check r23_at integration endpoints (not a proof
% over the whole interval). pT points from the drone towards the load.
[~,r0]=r23_rhs(x,r23_at(u,policy,0),P);
y=r23_project_state(r23_propagate(x,u,policy,P,dt,nSub));
[~,r1]=r23_rhs(y,r23_at(u,policy,dt),P);
if ~all(isfinite([r0,r1])) || max([r0,r1])>=0
    error('CBFTraj:ComparisonCableSlack','Nonpositive model cable tension');
end
end

function y=r23_project_state(y)
if any(~isfinite(y)) || abs(norm(y(1:4))-1)>0.01 || ...
        abs(norm(y(14:16))-1)>0.01
    error('CBFTraj:ComparisonManifold','Invalid integrated manifold state');
end
y(1:4)=y(1:4)/norm(y(1:4));y(14:16)=y(14:16)/norm(y(14:16));
end
function b=r23_align_quat(a,b)
if dot(a(1:4),b(1:4))<0,b(1:4)=-b(1:4);end
end
function b=r23_rebase_state(x,a,bOld)
a=r23_project_state(a);bOld=r23_project_state(r23_align_quat(a,bOld));
b=x+(bOld-a);
% Preserve relative rotation in SO(3) and shortest rotation on S^2.
qRel=r23_qmul(bOld(1:4),[a(1);-a(2:4)]);
b(1:4)=r23_qmul(qRel,x(1:4)/norm(x(1:4)));
v=a(14:16);w=bOld(14:16);c=dot(v,w);
if c<=-1+1e-8,error('CBFTraj:ComparisonAntipodal','Cable rotation ambiguous');end
k=cross(v,w);z=x(14:16)/norm(x(14:16));
b(14:16)=z+cross(k,z)+cross(k,cross(k,z))/(1+c);
end
function q=r23_qmul(a,b)
q=[a(1)*b(1)-dot(a(2:4),b(2:4)); ...
    a(1)*b(2:4)+b(1)*a(2:4)+cross(a(2:4),b(2:4))];
end

function [st,status,keepComparison]=rev23_update(st,prev,E)
% REV23 pure causal mode/jet update. E is the exact 7th-order homogeneous
% return transition expm(A*dt). CARRY overwrites the TOTAL episode offset;
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
    st=r23_hold_invalid(st);return
end
if strcmp(prior,'DEGRADED')
    % Do not re-seed/accumulate an invalid comparison in this episode.
    status='COMPARISON_LATCHED_OFF';
elseif prev.active || strcmp(prior,'CARRY')
    if ~strcmp(prior,'CARRY')
        % At t_k, preserve the position of the old local return law;
        % anchor is then constant throughout the new comparison episode.
        st.anchor=st.correction(:,1);st.mode='CARRY';
    end
    if ~prev.comparisonValid
        st=r23_hold_invalid(st);status=['COMPARISON_INVALID:' prev.comparisonStatus];return
    end
    pv=prev.modelOffsetPV(:);
    if numel(pv)~=6 || ~isreal(pv) || any(~isfinite(pv))
        st=r23_hold_invalid(st);status='COMPARISON_TARGET_INVALID';return
    end
    candidate=st.anchor+pv(1:3);
    % Retain REV20's diagnostic domain (not a physical safety constraint).
    if norm(candidate,Inf)>5 || norm(pv(4:6),Inf)>5
        st=r23_hold_invalid(st);status='COMPARISON_TARGET_DOMAIN';return
    end
    st.correction=[candidate,pv(4:6),zeros(3,5)];
    st.center=candidate;keepComparison=true;
end
if prev.active || ~prev.returnGate
    st.quietCount=0;
    if strcmp(st.mode,'RETURN'),st=r23_hold_invalid(st);end
    return
end
if strcmp(st.mode,'NOMINAL'),st.quietCount=0;return;end
st.quietCount=st.quietCount+1;
if st.quietCount>=8 && any(strcmp(st.mode,{'CARRY','DEGRADED'}))
    % Same gate as REV20: do not discard a still-growing episode effect.
    if strcmp(st.mode,'CARRY') && prev.comparisonValid && ...
            dot(prev.modelOffsetPV(1:3),prev.modelOffsetPV(4:6))>0
        status='QUIET_WAIT_OUTWARD_MODEL_MOTION';return
    end
    st.mode='RETURN';st.center=zeros(3,1);keepComparison=false;
    % No return propagation at this switch: current jet is its initial jet.
end
if strcmp(st.mode,'RETURN') && max(abs(st.correction),[],'all')<=1e-5
    st.mode='NOMINAL';st.correction=zeros(3,7);st.center=zeros(3,1);
    st.anchor=zeros(3,1);st.quietCount=0;keepComparison=false;
end
end

function st=r23_hold_invalid(st)
st.quietCount=0;
if strcmp(st.mode,'NOMINAL'),return;end
% Position remains at the last accepted value (or current analytic RETURN
% endpoint). No stale velocity extrapolation and no invalid displacement.
% Derivative reset is deliberate; inter-sample C6 is not required here.
st.correction(:,2:7)=0;st.center=st.correction(:,1);st.mode='DEGRADED';
end

function [g,err]=reconstruct_governor(L,t,phase,first,cfg,cable)
k0=find(phase(1:first)=='f',1,'last');
if isempty(k0),error('CBFTest:FlightStart','Missing flight start');end
while k0>1 && phase(k0-1)=='f',k0=k0-1;end
% First flight sample has tau=0 and all rate states 1, as WAYPOINT_REFERENCE.do.
g=struct('tau',0,'rate',ones(6,1),'cmd',1,'err',NaN);err=0;
for k=k0:first-1
    c=L.controller.result{k};r=L.reference.result{k};
    x=ascol(c.x,19);[b,g]=governor_sample(g,x,cable,cfg,zeros(3,1),false);
    if isstruct(r)&&isfield(r,'cbfTraj')
        q=ascol(r.cbfTraj.before28,28);err=max(err,max(abs(b-q)./(1+abs(q))));
    else,error('CBFTest:GovernorHistory','Missing original jet in flight history');end
    dt=t(k+1)-t(k);if dt<=0,error('CBFTest:GovernorTime','Nonincreasing flight time');end
    g=governor_advance(g,dt,cfg);
end
if err>1e-7,error('CBFTest:GovernorInitialReplay','Initial governor replay mismatch %.9g',err);end
end
function [b,g]=governor_sample(g,x,cable,cfg,offset,shifted)
b=base_at(g,cfg,cable,0);
if shifted,p=b(1:3)+offset;else,p=b(1:3);end
g.err=norm(p-x(8:10));
if cfg.adaptive
    u=max(0,min(1,(g.err-cfg.start)/(cfg.full-cfg.start)));u=u*u*(3-2*u);
    g.cmd=1-(1-cfg.minScale)*u;
else,g.cmd=1;end
end
function g=governor_advance(g,dt,cfg)
if cfg.adaptive,[g.rate,g.tau]=wp_governor_step(g.rate,g.tau,g.cmd,dt,cfg.filterTime);
else,g.tau=g.tau+dt;end
end
function b=base_at(g,cfg,cable,s)
if cfg.adaptive
    [~,tau,td]=wp_governor_step(g.rate,g.tau,g.cmd,s,cfg.filterTime);
    b=cfg.generator.evaluateWarped(tau,td);
else,b=cfg.generator.evaluate(g.tau+s);end
b=ascol(b,28);b(3)=b(3)-cable;
end
function j=local_jet(m,s,H,b,g,cfg,cable,mode,c,prepared)
f=@(u)base_at(g,cfg,cable,u);
if m<=3,a=CBF_TRAJ_REFERENCE_EVAL(m,s,H,b,f);
else
    if nargin<10,prepared=stable_prepare(b,f,H);end
    a=stable_eval(m,s,H,prepared);
end
d=CBF_TRAJ_LOCAL_CORRECTION_JET(mode,c,s,1.5);
j=a(:)+reshape([d;zeros(1,7)],28,1);
end
function [rows,allPass]=jet_audit(label,t,H,b,target,g,cfg,cable,mode,c,E)
% Fundamental theorem of calculus for derivative pairs j=0..5 on two
% half-intervals, independent 16-point Gauss quadrature. A polynomial is
% analytically C-infinity; DIRECT generator is piecewise C6 by construction.
% Numerical residual is mixed absolute/relative, not a proof of smoothness.
% All five methods are evaluated; selected HLC jet at s=0 is the same.
persistent nodes weights
if isempty(nodes)
    n=16;k=(1:n-1)';q=k./sqrt(4*k.^2-1);[V,D]=eig(diag(q,1)+diag(q,-1));[nodes,idx]=sort(diag(D));weights=2*V(1,idx).^2;
end
rows=cell(1,5);allPass=true;
prep=stable_prepare(b,@(s)base_at(g,cfg,cable,s),H);
for m=1:5
    tag='OK';endpoint=NaN;zero=NaN;ab=nan(1,6);ratio=ab;peak=nan(1,7);re=NaN;le=nan(1,7);lr=le;
    try
    f=@(s)reshape(local_jet(m,s,H,b,g,cfg,cable,mode,c,prep),4,7);
    z=f(0);zero=max(abs(z(:)-target));ab=zeros(1,6);ratio=zeros(1,6);peak=max(abs(z),[],1);
    for h=1:2
        l=(h-1)*H/2;r=h*H/2;zl=f(l);zr=f(r);integ=zeros(4,6);scale=zeros(4,6);
        for i=1:numel(nodes)
            q=f((l+r)/2+(r-l)/2*nodes(i));w=weights(i)*(r-l)/2;
            integ=integ+w*q(:,2:7);scale=scale+w*abs(q(:,2:7));peak=max(peak,max(abs(q),[],1));
        end
        residual=abs(zr(:,1:6)-zl(:,1:6)-integ);
        tol=1e-7+1e-6*max(abs(zr(:,1:6))+abs(zl(:,1:6)),scale);
        ab=max(ab,max(residual,[],1));ratio=max(ratio,max(residual./tol,[],1));peak=max(peak,max(abs(zr),[],1));
    end
    re=0;
    if strcmp(mode,'RETURN'),d=CBF_TRAJ_LOCAL_CORRECTION_JET(mode,c,H,1.5);re=max(abs(d-c*E.'),[],'all');end
    % One-sided limits at both endpoints and at the midpoint. Richardson
    % extrapolation cancels the leading slope; checks all orders, including 6.
    % This detects numerical endpoint discontinuity/cancellation; not proof
    % at unsampled internal knots. eps is a diagnostic probe, not a gain.
    le=zeros(1,7);lr=zeros(1,7);epsT=H*1e-5;
    for bnd=[0,H/2,H]
        c0=f(bnd);
        for dir=[-1,1]
            if bnd+dir*epsT<0||bnd+dir*epsT>H,continue;end
            lim=2*f(bnd+dir*epsT/2)-f(bnd+dir*epsT);
            ee=abs(lim-c0);tt=1e-6+1e-5*max(abs(lim),abs(c0));
            le=max(le,max(ee,[],1));lr=max(lr,max(ee./tt,[],1));
        end
    end
    % Only endpoint-matching methods 4/5 are required to match all seven
    % original endpoint derivatives, to numerical tolerance; not snapped.
    endpoint=0;
    if m>=4
        actual=f(H);expected=reshape(base_at(g,cfg,cable,H),4,7)+[CBF_TRAJ_LOCAL_CORRECTION_JET(mode,c,H,1.5);zeros(1,7)];
        endpoint=max(abs(actual-expected)./(1+abs(expected)),[],'all');
    end
    pass=all(isfinite([zero,ab,ratio,peak,re,le,lr,endpoint]))&&endpoint<=1e-8&&zero<=1e-10&&max(ratio)<=1&&max(lr)<=1&&re<=1e-8*(1+max(abs(c*E.'),[],'all'));
    if ~pass,tag='NUMERIC_JET_CHECK_FAILED';end
    catch ME
        pass=false;tag=status_text(ME);
    end
    allPass=allPass&&pass;
    rows{m}=[csvtext(label),',',numline([t;m]),',',csvtext(mode),',',numline([pass;zero;ab(:);ratio(:);peak(:);re;le(:);lr(:);endpoint]),',',csvtext(tag)];
end
end


function prep=stable_prepare(b,f,H)
% Stable residual formulation. Q(u)=(f(Hu)-Taylor6(Hu))/H^6.
% R(:,k+1)=Q^(k)(1), computed from sixth derivative differences,
% NOT by subtracting nearly equal large position endpoint values.
% Exact integral identity, evaluated by Gauss quadrature; endpoints checked.
persistent nodes weights
if isempty(nodes)
    n=16;k=(1:n-1)';q=k./sqrt(4*k.^2-1);[V,D]=eig(diag(q,1)+diag(q,-1));[nodes,idx]=sort(diag(D));weights=2*V(1,idx).^2;
end
j0=reshape(b,4,7);j1=reshape(f(H),4,7);R=zeros(4,7);
R(:,7)=j1(:,7)-j0(:,7);
for half=1:2
    for i=1:numel(nodes)
        u=(half-1)/2+(1+nodes(i))/4;w=weights(i)/4;
        z=reshape(f(H*u),4,7);delta=z(:,7)-j0(:,7);
        for k=0:5,R(:,k+1)=R(:,k+1)+w*(1-u)^(5-k)/factorial(5-k)*delta;end
    end
end
% Hermite degree 13: first 7 Bernstein coefficients zero; last 7
% satisfy right endpoint derivatives. Flatblend: right residual Taylor
% polynomial of degree 6 multiplied by flatstep of degree 13.
B=zeros(4,14);B(:,8:14)=right_coeff(R,13);
T=right_coeff(R,6);F=[zeros(1,7),ones(1,7)];C=zeros(4,20);
for k=0:19
    for i=max(0,k-6):min(13,k)
        j=k-i;C(:,k+1)=C(:,k+1)+F(i+1)*T(:,j+1)*nchoosek(13,i)*nchoosek(6,j)/nchoosek(19,k);
    end
end
% Precompute derivative coefficients in normalized u, never in seconds.
prep=struct('j0',j0,'hermite',{bern_derivatives(B)},'blend',{bern_derivatives(C)});
end
function B=right_coeff(R,n)
B=zeros(size(R,1),7);
for k=0:6
    for j=0:k,B(:,7-k)=B(:,7-k)+(-1)^j*nchoosek(k,j)*R(:,j+1)/prod(n-j+1:n);end
end
end
function D=bern_derivatives(B)
D=cell(1,7);D{1}=B;
for j=1:6,D{j+1}=(size(D{j},2)-1)*diff(D{j},1,2);end
end
function z=stable_eval(m,s,H,prep)
% No endpoint snapping. Evaluate the same polynomial at s=0, H and inside.
u=s/H;if m==4,D=prep.hermite;else,D=prep.blend;end
z=zeros(4,7);
for j=0:6
    v=zeros(4,1);
    for k=6:-1:j,v=v*s+prep.j0(:,k+1)/factorial(k-j);end
    B=D{j+1};n=size(B,2)-1;w=zeros(n+1,1);
    for k=0:n,w(k+1)=nchoosek(n,k)*u^k*(1-u)^(n-k);end
    z(:,j+1)=v+H^(6-j)*(B*w);
end
z=z(:);
end

function stable_unit_check(H)
% Analytic degree-8 synthetic reference: nonzero derivatives at both ends,
% large position constants, small residual. No live model mutation.
a=[10;20;1;0];b=[0.3;-0.4;0.1;0.02];
f=@(s) synthetic_jet(s,H,a,b);j0=f(0);j1=f(H);
p=stable_prepare(j0,f,H);
for m=4:5
    z0=stable_eval(m,0,H,p);z1=stable_eval(m,H,H,p);
    assert(max(abs(z0-j0))<=1e-12,'CBFTest:StableJetStart');
    assert(max(abs(z1-j1)./(1+abs(j1)))<=1e-9,'CBFTest:StableJetEnd');
    % Constant reference is an exact degree-zero special case.
    const=reshape([a,zeros(4,6)],28,1);pc=stable_prepare(const,@(~)const,H);
    assert(isequal(stable_eval(m,H/3,H,pc),const),'CBFTest:StableConstant');
end
end
function j=synthetic_jet(s,H,a,b)
% a+b*s + b*H^6*(u^7+u^8)/1000, u=s/H.
u=s/H;j=zeros(4,7);j(:,1)=a+b*s;j(:,2)=b;
for k=0:6
    j(:,k+1)=j(:,k+1)+b*H^(6-k)*(prod(8-k:7)*u^(7-k)+prod(9-k:8)*u^(8-k))/1000;
end
j=j(:);
end
