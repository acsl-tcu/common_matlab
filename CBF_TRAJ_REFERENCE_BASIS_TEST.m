function zipPath=CBF_TRAJ_REFERENCE_BASIS_TEST(logger,hlc)
% REV28B offline recorded-reference comparison. No live controller or CBF do().
% After normal GUI flight ends, run in MATLAB Command Window:
% CBF_TRAJ_REFERENCE_BASIS_TEST
% Reads the existing base-workspace app (mainGUI creates it). No Sim edits.
% Explicit alternative (also after flight):
% CBF_TRAJ_REFERENCE_BASIS_TEST(app.logger,app.agent(1).controller.hlc_suspended);
% Do not reload, restart, clear app, or close GUI before this audit.
% The only persistent output is a ZIP beside this m-file, replaced on successful export.
% All comparison branches are diagnostics, not online choices.
outDir=fileparts(mfilename('fullpath'));
zipPath=fullfile(outDir,'CBF_TRAJ_REFERENCE_BASIS_RESULTS.zip');
summary="REV28B OFFLINE REFERENCE BASIS AUDIT"+newline;
rows={}; snap={};
try
    if nargin==0
        if ~evalin('base','exist(''app'',''var'')')
            error('CBFBasis:AppMissing','Run normal mainGUI flight, then call this test before Reload/clear/close.');
        end
        liveApp=evalin('base','app');
        if ~isa(liveApp,'SimExp') || ~isvalid(liveApp)
            error('CBFBasis:AppInvalid','The base-workspace app must be a valid SimExp instance.');
        end
        if ~strcmp(string(liveApp.cha),"s")
            error('CBFBasis:FlightNotStopped','Run the test only after the simulation has stopped.');
        end
        logger=liveApp.logger;
        hlc=liveApp.agent(1).controller.hlc_suspended;
    elseif nargin~=2
        error('CBFBasis:Arguments','Use no arguments, or pass the recorded LOGGER and HLC objects.');
    end
    [rows,snap,summary]=audit(logger,hlc,summary);
catch ME
    summary=summary+"STATUS=FAILED"+newline+string(getReport(ME,'extended','hyperlinks','off'))+newline;
end
% Java ZIP stream avoids output directories and intermediate CSV/MAT files.
% Temporary ZIP is deleted on failure, and completed ZIP replaces old output.
tmp=[tempname(outDir),'.zip'];
clean=onCleanup(@() delete_if_exists(tmp));
fos=java.io.FileOutputStream(tmp); zos=java.util.zip.ZipOutputStream(fos);
zc=onCleanup(@() close_zip(zos));
put_entry(zos,'SUMMARY.txt',char(summary));
if ~isempty(rows),put_entry(zos,'REPLAY.csv',join_lines(rows));end
if ~isempty(snap),put_entry(zos,'SNAPSHOTS.csv',join_lines(snap));end
zos.close();clear zc
[ok,msg]=movefile(tmp,zipPath,'f');if ~ok,error('CBFBasis:ZIPMove','%s',msg);end
clear clean
fprintf('[CBF BASIS AUDIT] %s\n%s\n',zipPath,char(summary));
end

function [rows,snap,summary]=audit(logger,hlc,summary)
if ~isa(hlc,'HLC_SUSPENDED_LOAD_CBF_REAL') || hlc.cbfConfig.cbfId~=3
    error('CBFBasis:ID3Required','Use the same live ID3 HLC object as recorded flight.');
end
D=logger.Data;A=D.agent(1);dt=hlc.param.dt;
validateattributes(dt,{'numeric'},{'scalar','finite','positive'});
n=min([logger.k,numel(D.t),numel(D.phase),numel(A.controller.result),numel(A.reference.result),numel(A.estimator.result)]);
t=double(D.t(1:n));phase=char(D.phase(1:n));
% Physical parameter reconstruction is valid for this Sim: only mL is
% updated during flight by SUSPENDED_LOAD_STATE_MANAGER. Never use final mL.
Pfixed=double(hlc.self.parameter.get(["mass","jx","jy","jz","gravity","loadmass","cableL"]));
Pfixed=Pfixed(:);assert(numel(Pfixed)==7 && all(isfinite(Pfixed)));
summary=summary+sprintf('dt=%.17g\nlogger_samples=%d\nP_fixed_except_mL=%s\n',dt,n,mat2str(Pfixed.',17));
summary=summary+"P_SOURCE=live fixed mass/J/gravity/L; previous estimator.state.mL per sample; raw-HLC replay gate every interval. Not an independently recorded full P history."+newline;
summary=summary+"REFERENCES=recorded post-SLOAD before28 and controllerResult.xd (yaw already adjusted); recorded governor schedule held fixed."+newline;
summary=summary+"A=REV23 matched reference re-anchored; B=original reference re-anchored; C=matched independent; D=original independent; F=applied-input independent. A/B use exact same local applied endpoint. All start at recorded episode start. No anchor addition to B; later episodes are local differences, not full-flight displacement."+newline;
summary=summary+"C/D are independent comparison trajectories, F is open-loop replay of actual applied input (not a closed-loop flight). No future-input assumption. ID3 actuator-only continuation is used on each comparison's own T,q. Raw box rejection retained from REV23. No forced XYZ direction."+newline;
summary=summary+"Independent C/D keep the same REV23 5m/5m/s guard against the re-anchored applied endpoint; B/D are not clipped to fit the measured position."+newline;
summary=summary+"Fixed-history counterfactual only; changing online reference changes CBF/HLC and governor. No closed-loop stability or safety certification."+newline;
% CSV schemas: all vectors use x,y,z or numbered component suffixes.
head={'source_t','end_t','episode','mode','online_status','input_status','A_status','B_status','C_status','D_status','F_status'};
head=[head,vecnames('pre',3),vecnames('post',3),vecnames('est',3),vecnames('online_offset',6),vecnames('online_applied',3), ...
    vecnames('applied_endpoint',6),vecnames('A_endpoint',6),vecnames('B_endpoint',6),vecnames('C_endpoint',6),vecnames('D_endpoint',6),vecnames('F_endpoint',6), ...
    vecnames('A_offset',6),vecnames('B_offset',6),vecnames('C_offset',6),vecnames('D_offset',6),vecnames('ref_effect_B_minus_A',6), ...
    vecnames('A_minus_online',6),vecnames('A_nominal_minus_online',19),vecnames('actual_minus_online',19),vecnames('F_minus_est',3), ...
    vecnames('residual_pre',3),vecnames('residual_online',3),vecnames('residual_B',3),vecnames('residual_D',3), ...
    vecnames('proposed_target_B',3),vecnames('B_nominal_tracking_error',3),vecnames('applied_endpoint_minus_est',3),vecnames('decomposition_error',6), ...
    vecnames('A_raw',4),vecnames('B_raw',4),vecnames('C_raw',4),vecnames('D_raw',4), ...
    vecnames('A_Tq',2),vecnames('B_Tq',2),vecnames('C_Tq',2),vecnames('D_Tq',2),{'HLC_replay_max_abs','online_anchor_norm','A_reference_ms','B_reference_ms'}];
rows={strjoin(head,',')};
sh={'source_t','end_t','episode'};sh=[sh,vecnames('P',7),vecnames('x',19),vecnames('raw',4),vecnames('applied',4),vecnames('hold_T_q_nu_Tend_qend_dt',6),vecnames('admissible_nu',2),vecnames('pre28',28),vecnames('post28',28),vecnames('next_x',19)];
snap={strjoin(sh,',')};
comp=cell(1,4);alive=false(1,5);free=[];episode=0;lastK=0;
records=0;matched=0;validCounts=zeros(1,5);inputReject=0;decompError=[];matchedErr=[];originalErr=[];indErr=[];origResid=[];oldResid=[];preResid=[];dAall=[];dBall=[];Ferror=[];modelError=[];basisEffect=[];replayError=[];rawMax=nan(4,4);failCounts=zeros(1,5);unmatched=0;skip=0;timeA=[];timeB=[];
for k=2:n
    if phase(k)~='f',continue;end
    rr=A.reference.result{k};
    if ~isstruct(rr)||~isfield(rr,'cbfTraj'),continue;end
    r=rr.cbfTraj;
    if ~isfield(r,'correctionRevision')||r.correctionRevision~=23,error('CBFBasis:WrongRevision','Expected REV23 recorded reference.');end
    h=r.physicalPrevious;
    if ~strcmp(r.mode,'CARRY') && ~h.comparisonValid,continue;end
    % r(k).physicalPrevious refers to controller k-1, never controller k.
    if phase(k-1)~='f'||any(~isfinite([t(k-1),t(k),r.sourceIntervalStart,r.sourceIntervalEnd]))||abs(t(k)-t(k-1)-dt)>1e-8||abs(r.sourceIntervalStart-t(k-1))>1e-8||abs(r.sourceIntervalEnd-t(k))>1e-8
        skip=skip+1;alive(:)=false;continue;
    end
    if h.comparisonStarted
        episode=episode+1;comp=cell(1,4);alive(:)=true;free=[];lastK=k-1;
    end
    if episode==0,unmatched=unmatched+1;continue;end
    if lastK~=k-1,alive(:)=false;end
    lastK=k;records=records+1;
    st=repmat({'LATCHED_INVALID'},1,5);
    ds=cell(1,4);for j=1:4,ds{j}=empty_diag();end
    xa=nan(19,1);xf=xa;online=nan(6,1);onlineA=xa;onlineB=xa;
    if h.comparisonValid,online=h.modelOffsetPV(:);onlineA=h.modelActual(:);onlineB=h.modelNominal(:);end
    pre=ascol(r.before28,28);post=ascol(r.after28,28);act=ascol(r.signedCarryPosition,3);
    % Next controller state is the same-time HLC estimator state; no plant
    % endpoint from the next interval is used for this comparison.
    cur=A.controller.result{k};xn=ascol(cur.x,19);est=xn(8:10);
    errRaw=NaN;P=nan(7,1);x=nan(19,1);raw=nan(4,1);u=raw;hold=nan(6,1);bounds=nan(2,1);before=nan(28,1);after=before;
    inputStatus='OK';
    try
        prev=A.reference.result{k-1}.cbfTraj;c=A.controller.result{k-1};
        before=ascol(prev.before28,28);after=ascol(c.xd,28);
        x=ascol(c.x,19);raw=ascol(c.tmp,4);u=ascol(c.input,4);
        mask=[1:3,5:28];a=ascol(prev.after28,28);
        if any(abs(a(mask)-after(mask))>1e-9*(1+abs(after(mask)))),error('CBFBasis:ReferenceAlignment');end
        e=A.estimator.result{k-1}.state;
        if ~(isobject(e)&&isprop(e,'mL')) && ~(isstruct(e)&&isfield(e,'mL')),error('CBFBasis:MassHistoryMissing');end
        P=Pfixed;P(6)=double(e.mL);if any(~isfinite(P))||any(P<=0),error('CBFBasis:ParameterInvalid');end
        if any(~isfinite([before;after;x;raw;u])),error('CBFBasis:SnapshotNonfinite');end
        check=nominal_hlc(hlc,x,after,P,false);errRaw=max(abs(check-raw));
        if any(abs(check-raw)>1e-7*(1+abs(raw))),error('CBFBasis:HLCReplayMismatch');end
        [xa,policy,bounds,hold]=actual_step(c,x,u,P,dt);
        for j=1:4
            if ~alive(j),continue;end
            try
                timer=tic;
                if j==1
                    [~,v,co]=CBF_TRAJ_PHYSICAL_HOLD_DIFF(c,P,dt,3,hlc,comp{j},true,true);
                    if ~v.valid || ~v.comparisonValid,error('CBFBasis:MatchedReplayInvalid','%s / %s',v.status,v.comparisonStatus);end
                    dd=struct('nominal',v.modelNominal(:),'raw',v.comparisonRaw(:),'thrust',v.comparisonThrustEnd(:),'offset',v.modelOffsetPV(:),'ms',1000*toc(timer));
                else
                    target=after;if j==2||j==4,target=before;end
                    [dd,co]=comparison_step(comp{j},x,xa,policy,c,P,dt,hlc,target,j==2);
                    dd.ms=1000*toc(timer);
                end
                ds{j}=dd;comp{j}=co;st{j}='OK';
                aRaw=abs(dd.raw(:)).';if any(isnan(rawMax(j,:))),rawMax(j,:)=aRaw;else,rawMax(j,:)=max(rawMax(j,:),aRaw);end
            catch ME
                alive(j)=false;st{j}=status_text(ME);failCounts(j)=failCounts(j)+1;
            end
        end
        if alive(5)
            try
                if isempty(free),free=x;end
                free=checked_propagate(free,u,policy,P,dt);
                xf=free;st{5}='OK';
            catch ME
                alive(5)=false;st{5}=status_text(ME);failCounts(5)=failCounts(5)+1;
            end
        end
    catch ME
        inputStatus=status_text(ME);alive(:)=false;inputReject=inputReject+1;
    end
    dA=ds{1}.offset;dB=ds{2}.offset;
    dC=xf(8:13)-ds{3}.nominal(8:13);dD=xf(8:13)-ds{4}.nominal(8:13);
    eff=dB-dA;decomp=eff-(ds{1}.nominal(8:13)-ds{2}.nominal(8:13));
    validCounts=validCounts+strcmp(st,'OK');
    if all(isfinite(decomp)),decompError(end+1,:)=decomp.';end %#ok<AGROW>
    resPre=est-pre(1:3);resOld=est-post(1:3);
    resB=est-pre(1:3)-dB(1:3);resD=est-pre(1:3)-dD(1:3);
    ma=dA-online;mn=ds{1}.nominal-onlineB;aa=xa-onlineA;
    % Reproduction gate: compare full numeric snapshots with original online
    % result. No PASS based only on offset self-consistency.
    if h.comparisonValid && strcmp(st{1},'OK')
        matched=matched+1;matchedErr(end+1,:)=ma.'; %#ok<AGROW>
        originalErr(end+1,:)=mn.';modelError(end+1,:)=aa.'; %#ok<AGROW>
    end
    if strcmp(r.mode,'CARRY')&&strcmp(st{1},'OK')&&strcmp(st{2},'OK')&&h.comparisonValid
        oldResid(end+1,:)=resOld.';origResid(end+1,:)=resB.';preResid(end+1,:)=resPre.'; %#ok<AGROW>
        dAall(end+1,:)=dA(1:3).';dBall(end+1,:)=dB(1:3).';basisEffect(end+1,:)=eff(1:3).'; %#ok<AGROW>
    end
    if strcmp(st{5},'OK'),Ferror(end+1,:)=(xf(8:10)-est).';end %#ok<AGROW>
    if all(isfinite(dD)),indErr(end+1,:)=resD.';end %#ok<AGROW>
    if isfinite(errRaw),replayError(end+1)=errRaw;end %#ok<AGROW>
    timeA(end+1)=ds{1}.ms;timeB(end+1)=ds{2}.ms; %#ok<AGROW>
    vals=[pre(1:3);post(1:3);est;online;act;xa(8:13);ds{1}.nominal(8:13);ds{2}.nominal(8:13);ds{3}.nominal(8:13);ds{4}.nominal(8:13);xf(8:13);dA;dB;dC;dD;eff;ma;mn;aa;xf(8:10)-est;resPre;resOld;resB;resD;pre(1:3)+dB(1:3);ds{2}.nominal(8:10)-pre(1:3);xa(8:10)-est;decomp;ds{1}.raw;ds{2}.raw;ds{3}.raw;ds{4}.raw;ds{1}.thrust;ds{2}.thrust;ds{3}.thrust;ds{4}.thrust;errRaw;norm(r.episodeAnchor);ds{1}.ms;ds{2}.ms];
    tags=[{char(r.mode),char(h.comparisonStatus),inputStatus},st];
    row=[numline([t(k-1),t(k),episode]),',',strjoin(cellfun(@csvtext,tags,'UniformOutput',false),','),',',numline(vals)];
    assert(numel(vals)+11==numel(head),'CBFBasis:ReplaySchema');rows{end+1}=row; %#ok<AGROW>
    ss=[t(k-1);t(k);episode;P;x;raw;u;hold;bounds;before;after;xn];
    assert(numel(ss)==numel(sh),'CBFBasis:SnapshotSchema');snap{end+1}=numline(ss); %#ok<AGROW>
end
summary=summary+sprintf('episodes=%d\nintervals=%d\nmatched_valid=%d\nmissing_start=%d\ntime_alignment_reject=%d\nbranch_first_failure_counts_A_B_C_D_F=%s\n',episode,records,matched,unmatched,skip,mat2str(failCounts));
summary=summary+sprintf('valid_counts_A_B_C_D_F=%s\ninput_rejected_count=%d\n',mat2str(validCounts),inputReject);
if records==0,summary=summary+"STATUS=NO_EPISODES_CAPTURED"+newline;return;end
summary=summary+metric('A_minus_online_max_abs',matchedErr,'max')+metric('A_nominal_minus_online_max_abs',originalErr,'max')+metric('actual_minus_online_max_abs',modelError,'max');
summary=summary+metric('decomposition_error_max_abs',decompError,'max');
summary=summary+metric('raw_HLC_replay_max_abs',replayError(:),'max');
if matched==records && skip==0 && unmatched==0 && max(abs(matchedErr),[],'all')<=1e-7 && max(abs(originalErr),[],'all')<=1e-7 && max(abs(modelError),[],'all')<=1e-7
    summary=summary+"MATCHED_REPLAY=PASS_1E-7_ABS"+newline;
else,summary=summary+"MATCHED_REPLAY=INCOMPLETE_OR_MISMATCH"+newline;end
summary=summary+sprintf('paired_CARRY_count=%d\n',size(origResid,1))+metric('paired_pre_RMS_xyz',preResid,'rms')+metric('paired_online_RMS_xyz',oldResid,'rms')+metric('paired_original_ref_RMS_xyz',origResid,'rms')+metric('paired_original_ref_max_residual_xyz',origResid,'max')+metric('paired_A_max_offset_xyz',dAall,'max')+metric('paired_B_max_offset_xyz',dBall,'max')+metric('paired_reference_effect_max_xyz',basisEffect,'max');
summary=summary+metric('independent_F_minus_est_RMS_xyz',Ferror,'rms')+metric('independent_D_residual_RMS_xyz',indErr,'rms');
summary=summary+sprintf('raw_input_max_abs_rows_A_B_C_D=%s\n',mat2str(rawMax,9));
summary=summary+metric('A_offline_time_max_ms',timeA(isfinite(timeA)).','max')+metric('B_offline_time_max_ms',timeB(isfinite(timeB)).','max');
if validCounts(2)~=records||inputReject>0||skip>0||unmatched>0
    summary=summary+"B_COVERAGE=INCOMPLETE_DO_NOT_GENERALIZE_PAIRED_METRICS"+newline;
else,summary=summary+"B_COVERAGE=COMPLETE_RECORDED_EPISODES"+newline;end
summary=summary+"STATUS=COMPLETE_AUDIT_NOT_FLIGHT_CERTIFICATION"+newline;
end

function [xa,policy,bounds,hold]=actual_step(c,x,u,P,dt)
h=c.cbf.paperHold;bounds=ascol(c.cbf.paperAdmissibleNuAfterViability,2);
hold=[h.T0;h.q0;h.nu;h.Tend;h.qEnd;h.dt];
if ~h.active||any(~isfinite([hold;bounds]))||bounds(1)>bounds(2)||abs(h.dt-dt)>1e-10||abs(u(1)-h.T0)>1e-8||abs(h.Tend-h.T0-dt*h.q0-.5*dt^2*h.nu)>1e-8||abs(h.qEnd-h.q0-dt*h.nu)>1e-8
    error('CBFBasis:HoldInvalid');end
if isfield(c.cbf,'paperActuatorFallbackRateReset')&&~isempty(c.cbf.paperActuatorFallbackRateReset)&&c.cbf.paperActuatorFallbackRateReset,error('CBFBasis:ActuatorReset');end
if any(u<[0;-1;-1;-1]-1e-8|u>[20;1;1;1]+1e-8),error('CBFBasis:AppliedBox');end
policy=[h.T0,h.q0,h.nu];xa=checked_propagate(x,u,policy,P,dt);
end
function [d,co]=comparison_step(old,x,xa,policy,c,P,dt,hlc,target,rebase)
if isempty(old)
    xb=x;tb=policy(1:2).';
else
    if rebase,xb=project_state(rebase_state(x,old.actual,old.nominal));else,xb=old.nominal;end
    tb=old.thrust;
end
rawB=nominal_hlc(hlc,xb,target,P,true);
if any(rawB<[0;-1;-1;-1]-1e-8|rawB>[20;1;1;1]+1e-8),error('CBFBasis:ComparisonRawOutsideBox','rawB=%s',mat2str(rawB.',17));end
% At a common start, matched-reference raw is normally exactly identical.
% Use the recorded envelope if raw is equal; otherwise recompute bounds
% using the actual ID3 actuator-only fallback and branch's own T,q.
if isempty(old)&&all(abs(rawB-c.tmp(:))<=1e-12*(1+abs(c.tmp(:))))
    b=c.cbf.paperAdmissibleNuAfterViability(:);nu=min(max(c.cbf.paperNominalNuRaw,b(1)),b(2));
    u=[tb(1);min(max(rawB(2:4),-1),1)];pb=[tb(1),tb(2),nu];
    yb=checked_propagate(xb,u,pb,P,dt);te=[tb(1)+dt*tb(2)+.5*dt^2*nu;tb(2)+dt*nu];
else
    [yb,te]=baseline_step(xb,tb,rawB,P,dt,2,3,hlc.cbfConfig);
end
pv=xa(8:13)-yb(8:13);
if any(~isfinite([pv;te]))||norm(pv(1:3),Inf)>5||norm(pv(4:6),Inf)>5,error('CBFBasis:ComparisonDomain');end
co=struct('actual',xa,'nominal',yb,'thrust',te);
d=struct('nominal',yb,'raw',rawB,'thrust',te,'offset',pv,'ms',NaN);
end
function y=checked_propagate(x,u,p,P,dt)
project_state(x);[~,r0]=rhs(x,at(u,p,0),P);
y=project_state(propagate(x,u,p,P,dt,2));[~,r1]=rhs(y,at(u,p,dt),P);
if any(~isfinite([r0,r1]))||max([r0,r1])>=0,error('CBFBasis:CableSlack');end
end
function d=empty_diag()
d=struct('nominal',nan(19,1),'raw',nan(4,1),'thrust',nan(2,1),'offset',nan(6,1),'ms',NaN);
end
function x=ascol(x,n)
if ~isnumeric(x)||~isreal(x)||numel(x)~=n,error('CBFBasis:SnapshotShape');end
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
function s=metric(name,a,kind)
if isempty(a),s=string(name)+"=NA"+newline;return;end
if strcmp(kind,'rms'),v=sqrt(mean(a.^2,1));else,v=max(abs(a),[],1);end
s=string(name)+"="+string(mat2str(v,10))+newline;
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
if any(~isfinite(beta(:))) || rcond(beta)<1e-12,error('CBFTraj:ComparisonHLCIllConditioned');end
raw=[uf(1);beta\b];
if ~isreal(raw) || any(~isfinite(raw)),error('CBFTraj:ComparisonHLCNonfinite');end
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


function [y,tEnd]=baseline_step(x,t,raw,P,dt,nSub,cbfId,ac)
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
% Taut-cable model validity check at integration endpoints (not a proof
% over the whole interval). pT points from the drone towards the load.
[~,r0]=rhs(x,at(u,policy,0),P);
y=project_state(propagate(x,u,policy,P,dt,nSub));
[~,r1]=rhs(y,at(u,policy,dt),P);
if ~all(isfinite([r0,r1])) || max([r0,r1])>=0
    error('CBFTraj:ComparisonCableSlack','Nonpositive model cable tension');
end
end

function y=project_state(y)
if any(~isfinite(y)) || abs(norm(y(1:4))-1)>0.01 || ...
        abs(norm(y(14:16))-1)>0.01
    error('CBFTraj:ComparisonManifold','Invalid integrated manifold state');
end
y(1:4)=y(1:4)/norm(y(1:4));y(14:16)=y(14:16)/norm(y(14:16));
end
function b=align_quat(a,b)
if dot(a(1:4),b(1:4))<0,b(1:4)=-b(1:4);end
end
function b=rebase_state(x,a,bOld)
a=project_state(a);bOld=project_state(align_quat(a,bOld));
b=x+(bOld-a);
% Preserve relative rotation in SO(3) and shortest rotation on S^2.
qRel=qmul(bOld(1:4),[a(1);-a(2:4)]);
b(1:4)=qmul(qRel,x(1:4)/norm(x(1:4)));
v=a(14:16);w=bOld(14:16);c=dot(v,w);
if c<=-1+1e-8,error('CBFTraj:ComparisonAntipodal','Cable rotation ambiguous');end
k=cross(v,w);z=x(14:16)/norm(x(14:16));
b(14:16)=z+cross(k,z)+cross(k,cross(k,z))/(1+c);
end
function q=qmul(a,b)
q=[a(1)*b(1)-dot(a(2:4),b(2:4)); ...
    a(1)*b(2:4)+b(1)*a(2:4)+cross(a(2:4),b(2:4))];
end
