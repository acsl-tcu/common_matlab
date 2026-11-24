function out = CBF_REAL_DIAG_VIEW(mode, varargin)
% CBF_REAL_DIAG_VIEW  REAL CBF diagnostics only (no controller changes).
%   CBF_REAL_DIAG_VIEW('reset')
%   line = CBF_REAL_DIAG_VIEW('display',agent,time,originalLine,enabled,hlc)
%   CBF_REAL_DIAG_VIEW('plot',logger,20)  % also generates figures 21/22/23
%   CBF_REAL_DIAG_VIEW('failure',logger,t,info)  % failed cycle and plots
%
% Place this ordinary function in a directory on the MATLAB path (e.g. plot).
% In some FUNCTION_CLASS_PROPERTY configurations, controller.result does not
% expose all the fields produced by an HLC. Therefore the optional `hlc`
% handle is used as a read-only fallback for the *actual* HLC output.
% A small in-memory trace is taken at LOGGER display time so that plotting
% still works when LOGGER itself stores an abbreviated controller result.
% Never turn missing CBF data into a fabricated zero/intervention flag.
%
persistent traceTime traceValues
if isempty(traceTime)
    traceTime = zeros(0,1);
    traceValues = cell(0,1);
end
out = [];
switch lower(char(mode))
    case 'reset'
        traceTime = zeros(0,1);
        traceValues = cell(0,1);
    case 'display'
        [out, a, ts] = diagnostic_line(varargin{:});
        if isfinite(ts)
            if ~isempty(traceTime) && abs(traceTime(end)-ts)<1e-9
                traceValues{end}=a;
            else
                traceTime(end+1,1)=ts;
                traceValues{end+1,1}=a;
            end
        end
    case 'failure'
        if numel(varargin)<3
            error('CBFDiag:Usage','failure requires LOGGER, time, and CBF info.');
        end
        logger=varargin{1};
        ts=double(varargin{2});
        info=varargin{3};
        a=extract_values(struct('cbf',info),struct());
        a.failed=true;
        if ~isscalar(ts) || ~isfinite(ts) || ~a.record
            error('CBFDiag:FailureRecord','Invalid failed CBF record.');
        end
        traceTime(end+1,1)=ts;
        traceValues{end+1,1}=a;
        % Sim stops before post(), so draw history at the failure itself.
        plot_history(logger,20,traceTime,traceValues);
    case 'plot'
        if numel(varargin)<1
            error('CBFDiag:Usage','plot requires the LOGGER argument.');
        end
        fig=20;
        if numel(varargin)>=2 && ~isempty(varargin{2}), fig=varargin{2};end
        plot_history(varargin{1},fig,traceTime,traceValues);
    otherwise
        error('CBFDiag:UnknownMode','Unknown mode: %s',char(mode));
end
end

function [line,a,ts] = diagnostic_line(agent,time,originalLine,enabled,varargin)
line=originalLine;
a=empty_values();
ts=scalar_num(member(time,'t'));
if isempty(agent),return;end
controllerResult = member(member(agent(1),'controller'),'result');
sensorResult = member(member(agent(1),'sensor'),'result');
a = extract_values(controllerResult,sensorResult);

% Read *the same HLC object used to register the controller* if necessary.
if ~a.record && ~isempty(varargin)
    hlcResult=member(varargin{1},'result');
    ah=extract_values(hlcResult,sensorResult);
    if ah.record,a=ah;end
end

if ~enabled || isempty(originalLine),return;end
if ~(ischar(line) || (isstring(line)&&isscalar(line))),return;end
line=char(line);
if ~a.record
    line=[line,sprintf('\n  CBF status=NO_RESULT')];
    return;
end
% General presentation contract.  Never inspect CBF scheme number to
% select a formatter; scheme-specific diagnostics belong in info fields.
if isfinite(a.id) && a.id==0,return;end
info=a.cbfInfo;
success=member(info,'success');
if (islogical(success)||isnumeric(success)) && isscalar(success)
    if logical(success),verdict='OK';else,verdict='UNCERTIFIED';end
else
    verdict='UNKNOWN';
end
partsPresent=(isfinite(a.parts) && a.parts>0) || ...
    (isfinite(a.objects) && a.objects>0);
needsLine=partsPresent || (isfinite(a.modified) && a.modified~=0) || ...
    strcmp(verdict,'UNCERTIFIED') || generic_active_tilt_limits(info);
if ~needsLine,return;end
stage=member(info,'stageValues');
discCur=member(info,'discreteCurrentStages');
discNext=member(info,'discreteNextStages');
if isnumeric(stage) && size(stage,1)>=5 && ~isempty(stage)
    m=min(double(stage(1:5,:)),[],2);
    stageText=sprintf('h/psi1..4=[%.3g %.3g %.3g %.3g %.3g]',m);
elseif isnumeric(discCur) && isnumeric(discNext) && ...
        size(discCur,1)==4 && size(discNext,1)==5 && ...
        ~isempty(discCur) && size(discCur,2)==size(discNext,2)
    m=min([double(discCur);double(discNext(5,:))],[],2);
    stageText=sprintf('sampled phi0..3(k),phi4(k+1)=[%.3g %.3g %.3g %.3g %.3g]',m);
else
    stageText='h/psi1..4=NOT_EVALUATED';
end
[badHard,badStages]=generic_violation_text(info);
inputText=generic_input_text(info);
stateText=generic_state_integrity_text(info);
tiltText=generic_tilt_limits_text(info);
previewText=xy_preview_diag_text(info);
massText=xy_cbf_mass_diag_text(info);
zohText=xy_zoh_guard_diag_text(info);
line=[line,sprintf(['\n  CBF id=%s %s status=%s obs=%s parts=%s' ...
    ' nomOK=%s dT=%s dTau=%s viol=%s ms=%s' ...
    ' | %s | hard=[%s] | stages=[%s]%s%s'], ...
    fmt_int(a.id),verdict,a.status,fmt_int(a.objects),fmt_int(a.parts), ...
    fmt_int(a.nominalFeasible),fmt_num(a.dT),fmt_num(a.dTau), ...
    fmt_num(a.violation),fmt_num(a.ms),stageText,badHard,badStages, ...
    inputText,[stateText tiltText previewText zohText massText])];
end


function txt=xy_cbf_mass_diag_text(info)
used=scalar_num(member(info,'massUsedKg'));
if ~isfinite(used),txt='';return;end
est=scalar_num(member(info,'massEstimatorKg'));
txt=sprintf(' | mCBF=%s mEst=%s',fmt_num(used),fmt_num(est));
end

function txt=xy_zoh_guard_diag_text(info)
mode=member(info,'zohGuardStatus');
if isempty(mode),txt='';return;end
if isstring(mode),mode=char(mode);end
if ~ischar(mode),txt='';return;end
rows=scalar_num(member(info,'zohGuardRows'));
margin=scalar_num(member(info,'zohGuardMinNormalized'));
txt=sprintf(' | zohModel=%s rows=%s minApprox=%s', ...
    mode,fmt_int(rows),fmt_num(margin));
end

function txt=xy_preview_diag_text(info)
% All timings here are internal to Scheme 02, NOT a full Sim step.
st=member(info,'previewStatus');
if isempty(st),txt='';return;end
if isstring(st),st=char(st);end
if ~ischar(st),txt='';return;end
N=scalar_num(member(info,'previewCandidates'));
m0=scalar_num(member(info,'previewCurrentReserve'));
m1=scalar_num(member(info,'previewNextReserve'));
b1=scalar_num(member(info,'previewBaseNextReserve'));
a=scalar_num(member(info,'previewGuideActivation'));
gain=scalar_num(member(info,'previewGuideShiftNm'));
pc=scalar_num(member(info,'previewPredictedParts'));
pre=scalar_num(member(info,'previewElapsedMs'));
hard=scalar_num(member(info,'hardQpElapsedMs'));
extra=member(info,'previewExtraTorque');
if isnumeric(extra)&&numel(extra)==2&&all(isfinite(extra(:)))
    extra=double(extra(:));
else
    extra=[NaN;NaN];
end
txt=sprintf([' | guide=%s cand=%s parts=%s' ...
    ' act=%s shiftNm=%s R0=%s R1=%s Rbase=%s' ...
    ' dXYguide=[%+.3g,%+.3g] msHard=%s msGuide=%s'], ...
    st,fmt_int(N),fmt_int(pc),fmt_num(a),fmt_num(gain), ...
    fmt_num(m0),fmt_num(m1),fmt_num(b1), ...
    extra(1),extra(2),fmt_num(hard),fmt_num(pre));
end

function [hard,stages] = generic_violation_text(info)
% 'hard' comes only from explicit scheme-provided violations or hard
% barrier residuals.  Never confuse merely diagnostic HOCBF stages with
% binding safety inequalities from a different scheme.
hard='none';stages='none';
Scheck=member(info,'stageValues');
if isempty(Scheck)
    cert=member(info,'success');
    if (islogical(cert)||isnumeric(cert)) && isscalar(cert) && ~logical(cert)
        hard='NOT_EVALUATED';stages='NOT_EVALUATED';
    end
end
labels=member(info,'constraintNames');
if ~iscell(labels),labels={};end
v=member(info,'violations');
if isstruct(v) && ~isempty(v)
    items=cell(numel(v),1);
    for i=1:numel(v)
        name=member(v(i),'obstacle');
        if isempty(name),name='unknown';end
        tag=member(v(i),'stageLabel');
        if isempty(tag)
            idx=scalar_num(member(v(i),'stage'));
            if isfinite(idx),tag=sprintf('order%d',idx);else,tag='stage';end
        end
        items{i}=sprintf('%s/%s=%s',char(name),char(tag), ...
            fmt_num(scalar_num(member(v(i),'value'))));
    end
    hard=strjoin(items,';');
else
    values=member(info,'barrierResiduals');
    if isnumeric(values) && ~isempty(values)
        items={};
        for i=1:numel(values)
            if ~isfinite(values(i)) || values(i)>=-1e-8,continue;end
            name=sprintf('constraint%d',i);
            if numel(labels)>=i,name=char(labels{i});end
            items{end+1}=sprintf('%s=%+.3g',name,values(i)); %#ok<AGROW>
        end
        if ~isempty(items),hard=strjoin(items,';');end
    end
end
S=member(info,'stageValues');
if isnumeric(S) && size(S,1)>=5 && ~isempty(S)
    items={}; names={'h','psi1','psi2','psi3','psi4'};
    for j=1:size(S,2)
        label=sprintf('constraint%d',j);
        if numel(labels)>=j,label=char(labels{j});end
        for k=1:5
            if isfinite(S(k,j)) && S(k,j)<-1e-8
                items{end+1}=sprintf('%s/%s=%+.3g', ...
                    label,names{k},S(k,j)); %#ok<AGROW>
            end
        end
    end
    if ~isempty(items),stages=strjoin(items,';');end
end
end

function result=generic_input_text(info)
% UNIVERSAL physical-input diagnostic: all four HLC raw inputs, the four
% applied inputs, signed changes, and saturation/clipping. No scheme ID.
% The CBF correction must be separated from the mandatory physical clip.
raw=member(info,'nominal');
actual=member(info,'input');
if ~(isnumeric(raw) && isnumeric(actual) && numel(raw)==4 && ...
        numel(actual)==4 && all(isfinite(raw(:))) && all(isfinite(actual(:))))
    result=' | Uraw/Uout/dU=NA';return
end
raw=double(raw(:));actual=double(actual(:));
change=actual-raw;
result=sprintf([' | Uhlc=[%+.3g,%+.3g,%+.3g,%+.3g]' ...
    ' dU=[%+.3g,%+.3g,%+.3g,%+.3g]' ...
    ' |dU|max=%.3g'],raw(1),raw(2),raw(3),raw(4), ...
    change(1),change(2),change(3),change(4),max(abs(change)));
physical=member(info,'physicalNominal');
if isnumeric(physical) && numel(physical)==4 && all(isfinite(physical(:)))
    physical=double(physical(:));
    deltaCBF=actual-physical;
    clip=abs(physical-raw)>1e-10;
    result=[result,sprintf( ...
        ' dXYcbf=[%+.3g,%+.3g] clipHLC=[%d%d%d%d]', ...
        deltaCBF(2),deltaCBF(3),clip(1),clip(2),clip(3),clip(4))];
end
lo=[0;-1;-1;-1];hi=[20;1;1;1];
sat=(abs(actual-lo)<=1e-8 | abs(actual-hi)<=1e-8);
result=[result,sprintf(' satOut=[%d%d%d%d]',sat(1),sat(2),sat(3),sat(4))];
end

function tf=generic_active_tilt_limits(info)
body=member(info,'tiltBodyEnabled');cable=member(info,'tiltCableEnabled');
tf=(isscalar(body)&&(isnumeric(body)||islogical(body))&&logical(body)) || ...
    (isscalar(cable)&&(isnumeric(cable)||islogical(cable))&&logical(cable));
end

function result=generic_tilt_limits_text(info)
% Display angle hard constraints generically if the scheme reports them.
body=member(info,'tiltBodyDegrees');
cable=member(info,'tiltCableDegrees');
if ~(isnumeric(body)&&isscalar(body)&&isfinite(body) && ...
        isnumeric(cable)&&isscalar(cable)&&isfinite(cable))
    result='';return
end
bmax=member(info,'tiltBodyMaxDeg');cmax=member(info,'tiltCableMaxDeg');
bstage=member(info,'tiltBodyStages');cstage=member(info,'tiltCableStages');
if ~(isnumeric(bmax)&&isscalar(bmax)&&isnumeric(cmax)&&isscalar(cmax)),result='';return;end
result=sprintf(' | tiltBody=%.2f/%.2fdeg tiltCable=%.2f/%.2fdeg', ...
    body,bmax,cable,cmax);
if isnumeric(bstage) && ~isempty(bstage)
    result=[result,sprintf(' bodyPsi2=%+.3g',bstage(end))];
end
if isnumeric(cstage) && ~isempty(cstage)
    result=[result,sprintf(' cablePsi4=%+.3g',cstage(end))];
end
end

function result=generic_state_integrity_text(info)
% State invariants are recorded by schemes that use them. If absent,
% do not invent or infer a valid state.
norms=member(info,'stateNorms');
if ~(isnumeric(norms) && numel(norms)==2 && ...
        all(isfinite(norms(:))))
    result='';return
end
norms=double(norms(:));
result='';
unitOK=member(info,'stateUnitOK');
% Display significant drift and any rejected state, even when the normal
% CBF stage values could not be computed for that sample.
if max(abs(norms-1))>=1e-7 || ...
        (isscalar(unitOK) && ~logical(unitOK))
    result=sprintf(' | norm(q)=%.10g norm(pT)=%.10g', ...
        norms(1),norms(2));
end
end


function plot_history(logger,figureNumber,traceTime,traceValues)
D=member(logger,'Data');
A=member(D,'agent');
t=member(D,'t');
if isempty(A) || ~isnumeric(t)
    warning('CBFDiag:MissingLogger','LOGGER.Data.agent or .t is missing.');
    return;
end
A=A(1);
C=member(member(A,'controller'),'result');
S=member(member(A,'sensor'),'result');
n=numel(t);
loggedK=scalar_num(member(logger,'k'));
if isfinite(loggedK),n=min(n,max(0,floor(loggedK)));end
if n==0,warning('CBFDiag:NoSamples','Empty logger.');return;end
t=double(t(1:n));t=t(:);
obs=nan(n,1);parts=nan(n,1);
logA=cell(n,1);
for k=1:n
    cs=history_element(C,k);
    ss=history_element(S,k);
    a=extract_values(cs,ss);
    obs(k)=a.objects;
    parts(k)=a.parts;
    logA{k}=a;
end

% Prefer the live HLC observation when the dispatcher / LOGGER does not
% retain .cbf in controller.result. Match by actual timestamps, not index.
dt=diff(t);dt=dt(isfinite(dt)&dt>0);
if isempty(dt), tolerance=1e-6;else,tolerance=min(dt)*0.3;end
for k=1:n
    if logA{k}.record,continue;end
    if isempty(traceTime),continue;end
    [err,j]=min(abs(traceTime-t(k)));
    if err<=tolerance && traceValues{j}.record
        logA{k}=traceValues{j};
    end
end
% Exception occurs before LOGGER.logging(). Append precisely the failed
% CBF cycle from the read-only live trace, never fabricate a plant state.
for j=1:numel(traceTime)
    a=traceValues{j};
    if ~a.failed || ~a.record,continue;end
    ts=traceTime(j);
    if any(abs(t-ts)<=tolerance),continue;end
    t(end+1,1)=ts;
    logA{end+1,1}=a;
    obs(end+1,1)=a.objects;
    parts(end+1,1)=a.parts;
end
[t,order]=sort(t);
logA=logA(order);
obs=obs(order);
parts=parts(order);
n=numel(t);

id=nan(n,1);modified=nan(n,1);nomOK=nan(n,1);
dT=nan(n,1);dTau=nan(n,1);viol=nan(n,1);ms=nan(n,1);
for k=1:n
    a=logA{k};
    if a.record,id(k)=a.id;end
    modified(k)=a.modified;
    nomOK(k)=a.nominalFeasible;
    dT(k)=a.dT;
    dTau(k)=a.dTau;
    viol(k)=a.violation;
    ms(k)=a.ms;
    if isnan(obs(k)),obs(k)=a.objects;end
    if isnan(parts(k)),parts(k)=a.parts;end
end

fig=figure(figureNumber);clf(fig);
set(fig,'Name','CBF REAL diagnostics','NumberTitle','off');
ax=gobjects(6,1);
for j=1:6
    ax(j)=subplot(3,2,j,'Parent',fig);
    grid(ax(j),'on');box(ax(j),'on');
    xlabel(ax(j),'Time [s]');
end
stairs(ax(1),t,obs,'LineWidth',1.2,'DisplayName','Detected obstacles');
hold(ax(1),'on');
stairs(ax(1),t,parts,':','LineWidth',1,'DisplayName','Parts');
hold(ax(1),'off');
ylabel(ax(1),'Count');title(ax(1),'Detected obstacles');
if any(isfinite(obs)) || any(isfinite(parts)),legend(ax(1),'Location','best');end

stairs(ax(2),t,modified,'LineWidth',1.2,'DisplayName','CBF modified');
hold(ax(2),'on');
stairs(ax(2),t,nomOK,':','LineWidth',1,'DisplayName','Nominal feasible');
hold(ax(2),'off');
ylabel(ax(2),'0 / 1');title(ax(2),'CBF intervention');
set(ax(2),'YTick',[0 1]);ylim(ax(2),[-0.1 1.1]);
if any(isfinite(modified)),legend(ax(2),'Location','best');end

plot(ax(3),t,dT,'LineWidth',1.1);ylabel(ax(3),'Delta T [N]');title(ax(3),'Thrust correction');
plot(ax(4),t,dTau,'LineWidth',1.1);ylabel(ax(4),'Max |Delta tau| [N m]');title(ax(4),'Torque correction');
plot(ax(5),t,viol,'LineWidth',1.1);hold(ax(5),'on');
yline(ax(5),0,':');hold(ax(5),'off');
ylabel(ax(5),'Violation');title(ax(5),'Reported constraint violation');
plot(ax(6),t,ms,'LineWidth',1.1);hold(ax(6),'on');
if ~isempty(dt),yline(ax(6),1000*median(dt),'--','Sample period');end
hold(ax(6),'off');ylabel(ax(6),'ms');title(ax(6),'CBF calculation time');

validId=nnz(isfinite(id));
validTrace=0;
for j=1:numel(traceValues)
    validTrace=validTrace+double(traceValues{j}.record);
end
fprintf('[CBF DIAG] logger=%d CBF records=%d live HLC trace=%d detected max=%s modified=%d\n', ...
    n,validId,validTrace,fmt_int(max_finite(obs)),nnz(modified==1));
% Mark an exceptional terminal cycle on the ordinary diagnostic plots.
failedTimes=t(cellfun(@(a)a.failed,logA));
for j=1:6
    for k=1:numel(failedTimes)
        xline(ax(j),failedTimes(k),'r--','CBF fail', ...
            'HandleVisibility','off','LabelOrientation','horizontal');
    end
end
% Existing call to plot() now also produces complete per-constraint,
% per-stage and per-step ZOH-barrier histories.
plot_stage_history(t,logA,figureNumber+1,failedTimes);
plot_barrier_history(t,logA,figureNumber+2,failedTimes);
plot_four_input_history(t,logA,figureNumber+3,failedTimes);
if validId==0
    for j=2:6
        text(ax(j),.5,.5,'No CBF result recorded','Units','normalized', ...
            'HorizontalAlignment','center','Color',[.72 .18 .18]);
    end
    fprintf(2,['[CBF DIAG] No CBF data in controller.result OR provided HLC handle.\n' ...
        'Check which HLC class is active and whether do() sets obj.result.cbf.\n']);
end
end

function v=history_element(history,k)
v=[];
if iscell(history) && k<=numel(history),v=history{k};
elseif isstruct(history) && k<=numel(history),v=history(k);
end
end

function a=empty_values()
a=struct('id',NaN,'record',false,'objects',NaN,'parts',NaN,'modified',NaN, ...
    'solverUsed',NaN,'nominalFeasible',NaN,'dT',NaN,'dTau',NaN, ...
    'violation',NaN,'ms',NaN,'status','N/A', ...
    'cbfInfo',[],'failed',false);
end

function a=extract_values(c,s)
a=empty_values();
report=member(s,'cbfSensorReport');
a.objects=scalar_num(member(report,'detectedObjectCount'));
a.parts=scalar_num(member(report,'returnedPartCount'));
info=member(c,'cbf');
a.cbfInfo=info;
a.id=scalar_num(member(info,'cbfId'));
if isnan(a.id),a.id=scalar_num(member(c,'cbfId'));end
a.record=~isempty(info) && isfinite(scalar_num(member(info,'cbfId'))) && ...
    ~isempty(member(info,'status'));
if isnan(a.objects),a.objects=scalar_num(member(info,'observedObjects'));end
if isnan(a.parts),a.parts=scalar_num(member(info,'observedParts'));end
if isnan(a.objects)
    observed=member(s,'cbfObservedWorld');
    objects=member(observed,'objects');
    if isstruct(objects),a.objects=numel(objects);end
end
if ~a.record,return;end
status=member(info,'status');
if ischar(status) || (isstring(status)&&isscalar(status)),a.status=char(status);end
a.nominalFeasible=scalar_num(member(info,'nominalFeasible'));
a.solverUsed=scalar_num(member(info,'solverUsed'));
a.violation=scalar_num(member(info,'maxViolation'));
a.ms=scalar_num(member(info,'elapsedMs'));
if isequal(a.id,0)
    a.modified=0;a.dT=0;a.dTau=0;
    return;
end
a.modified=scalar_num(member(info,'modified'));
a.dT=scalar_num(member(info,'deltaThrust'));
a.dTau=scalar_num(member(info,'deltaTorqueInf'));
if isnan(a.dT) || isnan(a.dTau) || isnan(a.modified)
    uNom=member(c,'tmp');
    if isempty(uNom),uNom=member(info,'nominal');end
    uFinal=member(c,'input');
    if isempty(uFinal),uFinal=member(info,'input');end
    if isnumeric(uNom)&&isnumeric(uFinal)&&numel(uNom)==4&&numel(uFinal)==4 ...
            &&all(isfinite(uNom(:)))&&all(isfinite(uFinal(:)))
        d=double(uFinal(:))-double(uNom(:));
        if isnan(a.dT),a.dT=d(1);end
        if isnan(a.dTau),a.dTau=max(abs(d(2:4)));end
        if isnan(a.modified),a.modified=double(any(abs(d)>1e-10));end
    end
end
end

function x=member(obj,key)
x=[];
if isempty(obj),return;end
% Direct indexing is important for dynamic MATLAB proxy objects where
% isprop can return false even though obj.result / obj.controller works.
try
    x=obj.(key);
catch
end
end

function x=scalar_num(v)
x=NaN;
if (isnumeric(v)||islogical(v)) && isscalar(v) && isreal(v) && isfinite(double(v))
    x=double(v);
end
end
function s=fmt_num(v)
if isfinite(v),s=sprintf('%.4g',v);else,s='NA';end
end
function s=fmt_int(v)
if isfinite(v),s=sprintf('%.0f',v);else,s='NA';end
end
function v=max_finite(x)
x=x(isfinite(x));
if isempty(x),v=NaN;else,v=max(x);end
end


function plot_stage_history(t,records,figNum,failedTimes)
% Five separate orders, with a FIXED identity for every named constraint.
% Do not plot only the final critical constraint: its identity may switch.
[names,stages,~,~,~,~,~]=collect_named_histories(t,records);
if isempty(names),return;end
fig=figure(figNum);clf(fig);
set(fig,'Name','CBF REAL: every HOCBF stage vs time (diagnostics)', ...
    'NumberTitle','off');
labels={'0: h [m]','1: psi_1','2: psi_2','3: psi_3','4: psi_4'};
for order=1:5
    ax=subplot(3,2,order,'Parent',fig);
    hold(ax,'on');grid(ax,'on');box(ax,'on');
    for c=1:numel(names)
        plot(ax,t,stages(:,order,c),'LineWidth',1.1, ...
            'DisplayName',names{c});
    end
    yline(ax,0,'k:','HandleVisibility','off');
    mark_failures(ax,failedTimes);
    xlabel(ax,'Time [s]');ylabel(ax,labels{order});
    title(ax,labels{order});
    if order==1,legend(ax,'Location','best','Interpreter','none');end
end
ax=subplot(3,2,6,'Parent',fig);
hold(ax,'on');grid(ax,'on');box(ax,'on');
[minimumNominal,minimumApplied]=collect_minimum_residuals(records);
plot(ax,t,minimumNominal,'--','LineWidth',1.1,'DisplayName','Nominal min residual');
plot(ax,t,minimumApplied,'-','LineWidth',1.1,'DisplayName','Applied min residual');
yline(ax,0,'k:','HandleVisibility','off');
mark_failures(ax,failedTimes);
xlabel(ax,'Time [s]');ylabel(ax,'Reported residual');
title(ax,'Reported residual (scheme-specific)');legend(ax,'Location','best');
end

function plot_barrier_history(t,records,figNum,failedTimes)
% Compare each named constraint's current clearance, interval lower bound,
% required bound and residual. This identifies the first causal violation.
[names,~,h,lower,required,residual,nominal]=collect_named_histories(t,records);
if isempty(names) || (~any(isfinite(lower(:))) && ...
        ~any(isfinite(required(:)))),return;end
fig=figure(figNum);clf(fig);
set(fig,'Name','CBF REAL: all ZOH constraints vs time','NumberTitle','off');
data={h,lower,required,residual,nominal};
titles={'Current separation h','ZOH interval lower bound', ...
    'Required rho * h','Applied: lower - required', ...
    'Nominal: lower - required'};
for k=1:5
    ax=subplot(3,2,k,'Parent',fig);
    hold(ax,'on');grid(ax,'on');box(ax,'on');
    A=data{k};
    for c=1:numel(names)
        plot(ax,t,A(:,c),'LineWidth',1.0,'DisplayName',names{c});
    end
    if k>=4,yline(ax,0,'k:','HandleVisibility','off');end
    mark_failures(ax,failedTimes);
    xlabel(ax,'Time [s]');ylabel(ax,'m');title(ax,titles{k});
    if k==1,legend(ax,'Location','best','Interpreter','none');end
end
ax=subplot(3,2,6,'Parent',fig);
[minNom,minApplied]=collect_minimum_residuals(records);
plot(ax,t,minNom,'--',t,minApplied,'-','LineWidth',1.1);
hold(ax,'on');grid(ax,'on');box(ax,'on');
yline(ax,0,'k:');mark_failures(ax,failedTimes);
legend(ax,{'Nominal minimum','Applied minimum'},'Location','best');
title(ax,'Minimum over ALL constraints');xlabel(ax,'Time [s]');ylabel(ax,'m');
end

function [names,stage,h,lower,required,residual,nominal]=collect_named_histories(t,records)
names=cell(0,1);
for k=1:numel(records)
    info=records{k}.cbfInfo;
    if ~isstruct(info),continue;end
    labels=member(info,'constraintNames');
    if ~iscell(labels),continue;end
    for j=1:numel(labels)
        name=char(labels{j});
        if ~any(strcmp(names,name)),names{end+1,1}=name;end %#ok<AGROW>
    end
end
n=numel(t);m=numel(names);
stage=nan(n,5,m);h=nan(n,m);lower=nan(n,m);
required=nan(n,m);residual=nan(n,m);nominal=nan(n,m);
for k=1:n
    info=records{k}.cbfInfo;
    if ~isstruct(info),continue;end
    labels=member(info,'constraintNames');
    if ~iscell(labels),continue;end
    for j=1:numel(labels)
        c=find(strcmp(names,char(labels{j})),1);
        if isempty(c),continue;end
        A=member(info,'stageValues');
        if isnumeric(A) && size(A,1)>=5 && size(A,2)>=j
            stage(k,:,c)=double(A(1:5,j)).';
        end
        h(k,c)=indexed_numeric(member(info,'currentH'),j);
        lower(k,c)=indexed_numeric(member(info,'intervalLowerBounds'),j);
        rho=member(info,'zohBarrierFactor');
        if isscalar(rho)&&isnumeric(rho)&&isfinite(rho) ...
                && isfinite(h(k,c))
            required(k,c)=double(rho)*h(k,c);
        end
        residual(k,c)=indexed_numeric(member(info,'barrierResiduals'),j);
        nominal(k,c)=indexed_numeric(member(info,'nominalBarrierResiduals'),j);
    end
end
end

function x=indexed_numeric(a,j)
x=NaN;
if isnumeric(a) && numel(a)>=j && isreal(a(j)) && isfinite(a(j))
    x=double(a(j));
end
end

function [nom,applied]=collect_minimum_residuals(records)
n=numel(records);nom=nan(n,1);applied=nan(n,1);
for k=1:n
    info=records{k}.cbfInfo;
    if ~isstruct(info),continue;end
    a=member(info,'nominalBarrierResiduals');
    if isnumeric(a) && ~isempty(a) && all(isfinite(a(:)))
        nom(k)=min(a(:));
    end
    a=member(info,'barrierResiduals');
    if isnumeric(a) && ~isempty(a) && all(isfinite(a(:)))
        applied(k)=min(a(:));
    end
end
end

function mark_failures(ax,times)
for j=1:numel(times)
    xline(ax,times(j),'r--','Failed', ...
        'HandleVisibility','off','LabelOrientation','horizontal');
end
end


function plot_four_input_history(t,records,figNum,failedTimes)
% Show four physical inputs separately: HLC nominal versus CBF APPLIED.
% A failed candidate is NEVER labelled as applied.
n=numel(t);nom=nan(n,4);applied=nan(n,4);
for k=1:n
    a=records{k};info=a.cbfInfo;
    if ~isstruct(info),continue;end
    uNom=member(info,'nominal');u=member(info,'input');
    if isnumeric(uNom)&&numel(uNom)==4&&all(isfinite(uNom(:)))
        nom(k,:)=double(uNom(:)).';
    end
    if ~a.failed && isnumeric(u)&&numel(u)==4&&all(isfinite(u(:)))
        applied(k,:)=double(u(:)).';
    end
end
labels={'Thrust [N]','Torque x [N m]','Torque y [N m]','Torque z [N m]'};
fig=figure(figNum);clf(fig);
set(fig,'Name','CBF REAL: four input histories','NumberTitle','off');
for j=1:4
    ax=subplot(4,2,2*j-1,'Parent',fig);
    plot(ax,t,nom(:,j),'--','LineWidth',1.0,'DisplayName','HLC nominal');
    hold(ax,'on');
    plot(ax,t,applied(:,j),'-','LineWidth',1.0,'DisplayName','CBF applied');
    grid(ax,'on');box(ax,'on');mark_failures(ax,failedTimes);
    ylabel(ax,labels{j});xlabel(ax,'Time [s]');
    if j==1,legend(ax,'Location','best');end
    ax=subplot(4,2,2*j,'Parent',fig);
    plot(ax,t,applied(:,j)-nom(:,j),'LineWidth',1.1);
    hold(ax,'on');yline(ax,0,':');
    grid(ax,'on');box(ax,'on');mark_failures(ax,failedTimes);
    title(ax,['CBF correction input ' num2str(j)]);
    xlabel(ax,'Time [s]');ylabel(ax,labels{j});
end
end
