function report = CBF_REAL_XY_DERIVATIVE_TEST(action,varargin)
% CBF_REAL_XY_DERIVATIVE_TEST  Standalone, READ-ONLY diagnostic for CBF id=2.
%
% INSTALL: place this ONE .m file directly in common_matlab (beside mainGUI).
% No modifications to HLC, CBF, generator, simulation, or logger are needed.
%
%  CBF_REAL_XY_DERIVATIVE_TEST
%     FIRST use saved app.logger from base workspace (no text export needed).
%     If LOGGER has no CBF stages, recover them from the persistent CBF
%     diagnostic stage plot without modifying any controller/diagnostic files.
%     If no live history is available, optionally detect a console .txt log.
%     Write ONE ZIP in common_matlab.
%
%  CBF_REAL_XY_DERIVATIVE_TEST('run',[],[1;2;8;16])
%     Auto-use the saved app.logger of the LAST run, with its ACTUAL gains.
%  CBF_REAL_XY_DERIVATIVE_TEST('run',app.logger,[1;2;8;16])
%     NEW: also audit x(19), observed obstacle geometry, input and P.
%     If they are retained, decompose measured psi1 derivative discrepancy.
%     For uncertain parameter discovery, pass Praw as FOURTH argument:
%  CBF_REAL_XY_DERIVATIVE_TEST('run',app.logger,[1;2;8;16],Praw)
%     Praw=[mass;jx;jy;jz;gravity;loadmass;cableL;dstx;dsty].
%     Explicit in-memory LOGGER object, WITHOUT saving a console text file.
%  CBF_REAL_XY_DERIVATIVE_TEST('run','cbf_console.txt',[1;2;8;16])
%     Optional text-log fallback, if it already exists.
%     For the lower-gain experiment: [1;1.2;1.4;1.6].
%
% OPTIONAL: collect real full-state snapshots WITHOUT editing other files.
% In MATLAB debugger, stopped INSIDE HLC_SUSPENDED_LOAD_CBF_REAL/do AFTER
% the CBF_REAL_DISPATCH call (with obj,x,tmp,P variables in scope), call:
%  CBF_REAL_XY_DERIVATIVE_TEST('capture',obj,x,tmp,P)
% Repeat on adjacent time steps (e.g. 25.050, 25.075 s), then:
%  CBF_REAL_XY_DERIVATIVE_TEST('run',[],[1;2;8;16])
% Captures remain IN MEMORY only until 'run' or MATLAB clears functions.
% Use CBF_REAL_XY_DERIVATIVE_TEST('clear') to discard captures.
%
% FULL AUDIT additionally covers same-time stage/jet identities, physical
% state/obstacle/actuator history, controller-QP affine matrix and feasible
% polygon, microstep finite differences, full-step RK4 and state components.
% RESULTS: common_matlab/CBF_XY_DERIVATIVE_DIAGNOSTIC_RESULTS.zip
%           (contains SUMMARY.txt, REPORT.mat, and CSV tables).
% Do NOT run mainGUI again before extracting its saved app.logger.
% All diagnostics are non-actuating. In particular this does NOT certify
% sampled-data or intersample safety. A console log alone does not contain
% the full 19-state vector, so cannot prove a generator/plant mismatch.
%
% Important: HOCBF psi_i and their derivatives must be compared for the
% SAME obstacle. Live LOGGER/CBF stage plots preserve more precision than
% console text; no retained history is sufficient to prove plant correctness.
persistent snapshots
if isempty(snapshots)
    snapshots=struct('time',{},'x',{},'u',{},'raw',{},'Praw',{}, ...
        'parts',{},'cfg',{},'info',{});
end
if nargin<1 || isempty(action),action='run';end
action=lower(char(action));
root=fileparts(mfilename('fullpath'));
report=[];
switch action
    case 'help'
        help(mfilename);
    case 'clear'
        snapshots=snapshots([]);
        fprintf('[CBF XY TEST] In-memory captures cleared.\n');
    case 'capture'
        s=xytest_capture(varargin{:});
        snapshots(end+1)=s; %#ok<AGROW>
        fprintf('[CBF XY TEST] Captured t=%.6f, status=%s, count=%d (MEMORY ONLY)\n', ...
            s.time,xytest_fieldstr(s.info,'status'),numel(snapshots));
        report=s;
    case {'run','analyze'}
        requestedSource=[];lambda=[1;2;8;16];
        if ~isempty(varargin) && ~isempty(varargin{1})
            requestedSource=varargin{1};
        end
        if numel(varargin)>=2 && ~isempty(varargin{2})
            lambda=double(varargin{2}(:));
        elseif ~isempty(snapshots) && isfield(snapshots(1).cfg,'xyLambda')
            lambda=double(snapshots(1).cfg.xyLambda(:));
        end
        validateattributes(lambda,{'numeric'}, ...
            {'vector','numel',4,'finite','real','positive'});
        oldPath=path; pathRestore=onCleanup(@()path(oldPath)); %#ok<NASGU>
        logFull='';sourceDescription='none';
        if isempty(requestedSource)
            [logResult,sourceDescription]=xytest_saved_logger(lambda);
            if ~logResult.available
                logFull=xytest_find_log(root,'');
                if ~isempty(logFull)
                    logResult=xytest_log_analysis(logFull,lambda);
                    sourceDescription=logFull;
                end
            end
        elseif ischar(requestedSource) || (isstring(requestedSource) && isscalar(requestedSource))
            logFull=xytest_find_log(root,char(requestedSource));
            if isempty(logFull)
                logResult=xytest_no_log('Requested text log not found.');
            else
                logResult=xytest_log_analysis(logFull,lambda);
                sourceDescription=logFull;
            end
        else
            [logResult,sourceDescription]=xytest_saved_logger(lambda,requestedSource);
        end
        if ~logResult.available
            fprintf(2,['[CBF XY TEST] Saved LOGGER stages not accessible. ', ...
                'The generator/model test will still run.\n']);
        end
        [ready,setupMessage]=xytest_prepare(root);
        if ready
            try
                modelResult=xytest_model_cases(lambda);
            catch ME
                modelResult=struct('available',false, ...
                    'message',sprintf('Model test FAILED: %s: %s',ME.identifier,ME.message), ...
                    'cases',table());
            end
            try
                captureResult=xytest_capture_analysis(snapshots);
            catch ME
                captureResult=struct('available',false, ...
                    'message',sprintf('Capture analysis FAILED: %s: %s', ...
                        ME.identifier,ME.message),'pairs',table(),'samples',table());
            end
        else
            modelResult=struct('available',false,'message',setupMessage,'cases',table());
            captureResult=struct('available',false,'message', ...
                'Generated functions unavailable. Capture checks skipped.', ...
                'pairs',table(),'samples',table());
        end
        % Audit exact historical input/state/obstacle retention.  Do not
        % substitute the CURRENT state for a past one.
        suppliedP=[];
        if numel(varargin)>=3 && ~isempty(varargin{3})
            suppliedP=double(varargin{3}(:));
            validateattributes(suppliedP,{'numeric'}, ...
                {'vector','numel',9,'finite','real'});
        end
        try
            liveResult=xytest_live_history(requestedSource,lambda,ready,suppliedP);
        catch ME
            liveResult=xytest_empty_live(['Live logger audit failed: ' ...
                ME.identifier ' ' ME.message]);
        end
        try
            fullAudit=xyfull_audit(requestedSource,lambda,ready,suppliedP);
        catch ME
            fullAudit=xyfull_empty(['Comprehensive audit failed: ' ...
                ME.identifier ' : ' ME.message]);
        end
        report=struct('createdAt',datestr(now,31),'sourceRoot',root, ...
            'lambda',lambda,'logSource',logFull, ...
            'logSourceDescription',sourceDescription, ...
            'rawCaptures',snapshots, ...
            'log',logResult, ...
            'generatorModel',modelResult,'captures',captureResult, ...
            'live',liveResult,'fullAudit',fullAudit, ...
            'setupMessage',setupMessage, ...
            'interpretation',['Algebraic sampled derivative checks are necessary ', ...
                'diagnostics, not an intersample certificate. Large residuals ', ...
                'do not alone identify their physical cause.']);
        xytest_write_zip(root,report);
        if fullAudit.available
            fprintf('[CBF XY TEST] FULL AUDIT: samples=%d, detailed pairs=%d, stageError=%.3g, algebraError=%.3g, microDerivativeError=%.3g\n', ...
                fullAudit.statistics.recomputed,fullAudit.statistics.validPairs, ...
                fullAudit.statistics.maxSameStateStageError, ...
                fullAudit.statistics.maxSameStateAlgebraError, ...
                fullAudit.statistics.maxMicroDerivativeError);
        else
            fprintf(2,'[CBF XY TEST] FULL AUDIT INCOMPLETE: %s\n',fullAudit.message);
        end
        fprintf('[CBF XY TEST] FINISHED: %s\n', ...
            fullfile(root,'CBF_XY_DERIVATIVE_DIAGNOSTIC_RESULTS.zip'));
        if logResult.available
            fprintf('[CBF XY TEST] Log records=%d, adjacent comparable pairs=%d, suspicious psi1 pairs=%d\n', ...
                height(logResult.rows),height(logResult.pairs), ...
                logResult.counts.suspiciousPsi1);
        end
        if modelResult.available
            fprintf('[CBF XY TEST] Generator/model worst h Taylor prediction error=%.4g m\n', ...
                max(modelResult.cases.hTaylorErrorM));
        end
        fprintf('[CBF XY TEST] Live logger: x19=%d, obstacle=%d, input=%d, full=%d, detailed pairs=%d\n', ...
            liveResult.coverage.countX,liveResult.coverage.countObstacle, ...
            liveResult.coverage.countInput,liveResult.coverage.countComplete, ...
            height(liveResult.pairs));
        if ~isempty(snapshots)
            fprintf('[CBF XY TEST] Snapshots=%d, adjacent snapshot pairs=%d\n', ...
                numel(snapshots),height(captureResult.pairs));
        end
    otherwise
        error('CBFXYTest:Action','Unknown action: %s. Use help.',action);
end
end

function s=xytest_capture(ctrl,x,uNom,Praw)
if ~isobject(ctrl) || ~isprop(ctrl,'self') || ~isprop(ctrl,'result')
    error('CBFXYTest:Capture','Pass HLC object obj from HLC.do debugger.');
end
if ~isstruct(ctrl.result) || ~isfield(ctrl.result,'cbf')
    error('CBFXYTest:Capture','Capture AFTER CBF_REAL_DISPATCH (obj.result.cbf must exist).');
end
if ~isprop(ctrl.self,'sensor') || ~isfield(ctrl.self.sensor.result,'cbfObservedWorld')
    error('CBFXYTest:Capture','Live sensor CBF observation is unavailable.');
end
validateattributes(x,{'numeric'},{'vector','numel',19,'real','finite'});
validateattributes(uNom,{'numeric'},{'vector','numel',4,'real','finite'});
validateattributes(Praw,{'numeric'},{'vector','numel',9,'real','finite'});
info=ctrl.result.cbf;
if ~isfield(info,'cbfId') || info.cbfId~=2
    error('CBFXYTest:Capture','Expected CBF id=2.');
end
cfg=ctrl.cbfConfig;
res=ctrl.self.sensor.result;
if isfield(res,'cbfSensorTime'),t=double(res.cbfSensorTime);else,t=NaN;end
obs=res.cbfObservedWorld;
if ~isfield(obs,'parts'),error('CBFXYTest:Capture','No obstacle parts.');end
low=[0;-1;-1;-1];high=[20;1;1;1];
if isfield(info,'input') && isnumeric(info.input) && ...
        numel(info.input)==4 && all(isfinite(info.input(:)))
    applied=double(info.input(:));
else
    applied=double(uNom(:));
end
applied=min(max(applied,low),high);
s=struct('time',t,'x',double(x(:)),'u',applied, ...
    'raw',double(uNom(:)),'Praw',double(Praw(:)), ...
    'parts',obs.parts,'cfg',cfg,'info',info);
end

function out=xytest_no_log(reason)
out=struct('available',false,'warning',reason,'rows',table(), ...
    'pairs',table(),'counts',struct('suspiciousPsi1',0));
end

function [out,description]=xytest_saved_logger(lambda,varargin)
% No writes and no simulation rerun. Read existing app.logger / LOGGER.Data.
out=xytest_no_log('Neither LOGGER.Data.controller.result.cbf nor a CBF stage plot was available.');
description='none';logger=[];
if ~isempty(varargin)
    item=varargin{1};
    try
        if isobject(item) && ~isempty(xytest_member(item,'logger'))
            logger=item.logger;
        else
            logger=item;
        end
    catch
        logger=item;
    end
else
    try
        if evalin('base','exist(''app'',''var'')')
            app=evalin('base','app');
            logger=xytest_member(app,'logger');
        end
    catch
    end
    if isempty(logger)
        try
            if evalin('base','exist(''logger'',''var'')')
                logger=evalin('base','logger');
            end
        catch
        end
    end
end
if ~isempty(logger)
    try
        out=xytest_logger_records(logger,lambda);
        if out.available
            description='app.logger.Data (named HOCBF stages)';
            return
        end
    catch ME
        fprintf(2,'[CBF XY TEST] Direct logger read skipped: %s\n',ME.message);
    end
end
% The original unmodified CBF_REAL_DIAG_VIEW retains complete per-cycle
% diagnostic stages in a persistent trace, and plots them on request.
% If plots already exist, read them first without repainting anything.
try
    [out,whichFig]=xytest_stage_figure(lambda);
    if out.available
        description=sprintf('existing CBF stage figure %d',whichFig);
        return
    end
catch
end
if ~isempty(logger) && exist('CBF_REAL_DIAG_VIEW','file')==2
    baseFig=180;
    existing=get(groot,'Children');existingNumbers=[];
    for j=1:numel(existing)
        if strcmp(get(existing(j),'Type'),'figure')
            existingNumbers(end+1)=get(existing(j),'Number'); %#ok<AGROW>
        end
    end
    while any(ismember(baseFig+(0:3),existingNumbers)),baseFig=baseFig+4;end
    try
        CBF_REAL_DIAG_VIEW('plot',logger,baseFig);
        [out,~]=xytest_stage_figure(lambda,baseFig+1);
        if out.available
            description='CBF_REAL_DIAG_VIEW persistent live trace + saved LOGGER';
        end
    catch ME
        fprintf(2,'[CBF XY TEST] In-memory stage-plot read skipped: %s\n',ME.message);
    end
    for j=0:3
        if ishghandle(baseFig+j),close(baseFig+j);end
    end
end
end

function out=xytest_logger_records(logger,lambda)
out=xytest_no_log('LOGGER.Data.agent(1).controller.result has no saved CBF stages.');
D=xytest_member(logger,'Data'); agents=xytest_member(D,'agent');
time=xytest_member(D,'t');
if isempty(agents) || ~isnumeric(time) || isempty(time),return;end
agent=agents(1);
history=xytest_member(xytest_member(agent,'controller'),'result');
if isempty(history),return;end
n=numel(time);kLive=xytest_member(logger,'k');
if isnumeric(kLive)&&isscalar(kLive)&&isfinite(kLive)
    n=min(n,max(0,floor(double(kLive))));
end
records=struct('t',{},'h',{},'psi1',{},'psi2',{},'psi3',{}, ...
    'psi4',{},'obs',{},'parts',{},'status',{}, ...
    'bodyDeg',{},'cableDeg',{},'bodyPsi2',{},'cablePsi4',{});
for k=1:n
    ts=double(time(k));if ~isfinite(ts),continue;end
    c=xytest_history_element(history,k);
    info=xytest_member(c,'cbf');
    if ~isstruct(info),continue;end
    stage=xytest_member(info,'stageValues');
    if ~isnumeric(stage) || size(stage,1)<5 || isempty(stage),continue;end
    partCount=xytest_scalar(xytest_member(info,'observedParts'));
    if ~isfinite(partCount),partCount=size(stage,2);end
    if partCount~=1 || size(stage,2)~=1
        % Multiple obstacles: never mix unrelated h/psi minima.
        continue
    end
    A=double(stage(1:5,1));
    if any(~isfinite(A)),continue;end
    r=struct('t',ts,'h',A(1),'psi1',A(2),'psi2',A(3), ...
        'psi3',A(4),'psi4',A(5),'obs',1,'parts',1, ...
        'status',xytest_string(xytest_member(info,'status')), ...
        'bodyDeg',xytest_scalar(xytest_member(info,'tiltBodyDegrees')), ...
        'cableDeg',xytest_scalar(xytest_member(info,'tiltCableDegrees')), ...
        'bodyPsi2',NaN,'cablePsi4',NaN);
    tilt=xytest_member(info,'tiltBodyStages');
    if isnumeric(tilt)&&numel(tilt)>=3,r.bodyPsi2=double(tilt(end));end
    tilt=xytest_member(info,'tiltCableStages');
    if isnumeric(tilt)&&numel(tilt)>=5,r.cablePsi4=double(tilt(end));end
    records(end+1)=r; %#ok<AGROW>
end
if numel(records)<2,return;end
rows=struct2table(records);
[~,ord]=sort(rows.t);rows=rows(ord,:);
out=xytest_analyze_rows(rows,lambda,'LOGGER.Data native double precision');
out.warning=['LOGGER retains full precision, but only cycles with exactly ', ...
    'one active obstacle can be compared; all others are skipped. ', ...
    'Estimator/plant mismatches still require paired 19-state snapshots.'];
end

function [out,figNumber]=xytest_stage_figure(lambda,requestedFigure)
out=xytest_no_log('No complete CBF stage curves found.');figNumber=NaN;
figs=findall(groot,'Type','figure');
if nargin>1
    figs=figs(arrayfun(@(f)round(get(f,'Number'))==requestedFigure,figs));
end
for ii=1:numel(figs)
    fig=figs(ii);
    if ~contains(char(get(fig,'Name')),'every HOCBF stage'),continue;end
    ax=findall(fig,'Type','axes');
    stages=cell(5,1);
    for k=1:numel(ax)
        ttl=get(get(ax(k),'Title'),'String');
        if iscell(ttl),ttl=strjoin(ttl,' ');end
        tag=regexp(char(ttl),'^([0-4]):','tokens','once');
        if isempty(tag),continue;end
        jj=str2double(tag{1})+1;
        curves=findall(ax(k),'Type','line');
        entries=struct('name',{},'time',{},'values',{});
        for z=1:numel(curves)
            name=get(curves(z),'DisplayName');
            xx=get(curves(z),'XData');yy=get(curves(z),'YData');
            if isempty(name)||~isnumeric(xx)||~isnumeric(yy) || ...
                    numel(xx)~=numel(yy)||numel(xx)<2
                continue
            end
            entries(end+1)=struct('name',char(name), ...
                'time',double(xx(:)),'values',double(yy(:))); %#ok<AGROW>
        end
        stages{jj}=entries;
    end
    if any(cellfun(@isempty,stages)),continue;end
    % Only one obstacle; never compare a mixed minimum from different parts.
    common=stages{1};
    if numel(common)~=1,continue;end
    name=common(1).name;
    v=nan(numel(common(1).time),5);t=common(1).time;
    for k=1:5
        candidates=stages{k};
        hit=find(strcmp({candidates.name},name),1);
        if isempty(hit),break;end
        e=candidates(hit);
        if numel(e.time)~=numel(t) || any(abs(e.time-t)>1e-8),break;end
        v(:,k)=e.values;
    end
    if any(~isfinite(v(:)))
        valid=all(isfinite(v),2);
    else
        valid=true(numel(t),1);
    end
    if sum(valid)<2,continue;end
    t=t(valid);v=v(valid,:);
    nn=numel(t);
    rows=table(t,v(:,1),v(:,2),v(:,3),v(:,4),v(:,5), ...
        ones(nn,1),ones(nn,1),repmat({'FROM_STAGE_PLOT'},nn,1), ...
        nan(nn,1),nan(nn,1),nan(nn,1),nan(nn,1), ...
        'VariableNames',{'t','h','psi1','psi2','psi3','psi4', ...
          'obs','parts','status','bodyDeg','cableDeg','bodyPsi2','cablePsi4'});
    out=xytest_analyze_rows(rows,lambda,'CBF stage plot (in-memory)');
    out.counts.uncertified=NaN; % stage figures do not contain success/status
    out.warning=['Stage plot source contains precise stage history, but ', ...
        'no input or certified status in this extraction. Only one plotted ', ...
        'obstacle is supported.'];
    figNumber=get(fig,'Number');return
end
end

function x=xytest_member(v,key)
x=[];
try
    if ~isempty(v),x=v.(key);end
catch
end
end

function value=xytest_history_element(hist,k)
value=[];
if iscell(hist)&&k<=numel(hist),value=hist{k};
elseif isstruct(hist)&&k<=numel(hist),value=hist(k);
end
end

function x=xytest_scalar(v)
x=NaN;
if isnumeric(v)&&isscalar(v)&&isreal(v)&&isfinite(v),x=double(v);end
end

function s=xytest_string(v)
s='UNKNOWN';
if ischar(v)|| (isstring(v)&&isscalar(v)),s=char(v);end
end

function filename=xytest_find_log(root,requested)
filename='';
if ~isempty(requested)
    if isfile(requested),filename=requested;return;end
    trial=fullfile(root,requested);
    if isfile(trial),filename=trial;return;end
    warning('CBFXYTest:LogMissing','Specified log not found: %s',requested);
    return
end
candidates=dir(fullfile(root,'*.txt'));
if isempty(candidates),return;end
[~,ord]=sort([candidates.datenum],'descend');
for k=ord
    trial=fullfile(candidates(k).folder,candidates(k).name);
    try
        fid=fopen(trial,'r');
        if fid<0,continue;end
        head=fread(fid,32768,'*char')';fclose(fid);
        if contains(head,'CBF id=2') && contains(head,'h/psi1..4')
            filename=trial;return
        end
    catch
        if exist('fid','var') && fid>=0,fclose(fid);end
    end
end
end

function out=xytest_log_analysis(filename,lambda)
src=fileread(filename); lines=regexp(src,'\r\n|\n|\r','split');
% Store only fully recognized console records; avoid mixing nonadjacent rows.
t=[];h=[];p1=[];p2=[];p3=[];p4=[];obs=[];parts=[];
status={}; bodyAngle=[];cableAngle=[];bodyFinal=[];cableFinal=[];
for k=1:numel(lines)-1
    tok=regexp(lines{k},'^\s*[tf]\s+([0-9]+(?:\.[0-9]+)?)\s*:', ...
        'tokens','once');
    if isempty(tok),continue;end
    ln=lines{k+1};
    if isempty(regexp(ln,'CBF id=2\s','once')),continue;end
    htok=regexp(ln,'h/psi1\.\.4=\[([^\]]+)\]','tokens','once');
    if isempty(htok),continue;end
    v=sscanf(htok{1},'%f');
    if numel(v)~=5,continue;end
    t(end+1,1)=str2double(tok{1}); %#ok<AGROW>
    h(end+1,1)=v(1);p1(end+1,1)=v(2); %#ok<AGROW>
    p2(end+1,1)=v(3);p3(end+1,1)=v(4);p4(end+1,1)=v(5); %#ok<AGROW>
    obs(end+1,1)=xytest_number(ln,'\bobs=([0-9]+)'); %#ok<AGROW>
    parts(end+1,1)=xytest_number(ln,'\bparts=([0-9]+)'); %#ok<AGROW>
    st=regexp(ln,'\bstatus=([^\s]+)','tokens','once');
    if isempty(st),status{end+1,1}='UNKNOWN';else,status{end+1,1}=st{1};end %#ok<AGROW>
    bodyAngle(end+1,1)=xytest_number(ln,'tiltBody=([-+0-9.eE]+)'); %#ok<AGROW>
    cableAngle(end+1,1)=xytest_number(ln,'tiltCable=([-+0-9.eE]+)'); %#ok<AGROW>
    bodyFinal(end+1,1)=xytest_number(ln,'bodyPsi2=([-+0-9.eE]+)'); %#ok<AGROW>
    cableFinal(end+1,1)=xytest_number(ln,'cablePsi4=([-+0-9.eE]+)'); %#ok<AGROW>
end
rows=table(t,h,p1,p2,p3,p4,obs,parts,status,bodyAngle,cableAngle, ...
    bodyFinal,cableFinal,'VariableNames', ...
    {'t','h','psi1','psi2','psi3','psi4','obs','parts', ...
    'status','bodyDeg','cableDeg','bodyPsi2','cablePsi4'});
out=xytest_analyze_rows(rows,lambda,filename);
end

function out=xytest_analyze_rows(rows,lambda,source)
t=rows.t;h=rows.h;p1=rows.psi1;p2=rows.psi2; ...
    p3=rows.psi3;p4=rows.psi4;obs=rows.obs;parts=rows.parts;
status=rows.status;bodyAngle=rows.bodyDeg;cableAngle=rows.cableDeg;
if height(rows)<2
    out=struct('available',false,'source',source,'rows',rows, ...
        'pairs',table(),'counts',struct('suspiciousPsi1',0), ...
        'warning','No two usable h/psi records found.');return
end
% Detection-only; never assume the five displayed extrema belong to the
% same obstacle when parts>1. Use one obstacle and uninterrupted 25ms steps.
dtSteps=diff(t);dtSteps=dtSteps(isfinite(dtSteps) & dtSteps>0);
if isempty(dtSteps),dtGuess=0.025;else,dtGuess=median(dtSteps);end
if ~isfinite(dtGuess) || dtGuess<=0,dtGuess=0.025;end
n=height(rows)-1;
startTime=zeros(n,1);deltaT=zeros(n,1);rH=nan(n,1);rPsi1=nan(n,1);
rPsi2=nan(n,1);rPsi3=nan(n,1);errPsi1=nan(n,1);
suspect=false(n,1);sameObstacle=false(n,1);statusStart=cell(n,1);
for j=1:n
    startTime(j)=rows.t(j);deltaT(j)=rows.t(j+1)-rows.t(j);
    statusStart{j}=rows.status{j};
    sameObstacle(j)=rows.parts(j)==1 && rows.parts(j+1)==1 && ...
        rows.obs(j)==1 && rows.obs(j+1)==1 && ...
        abs(deltaT(j)-dtGuess)<max(1e-6,dtGuess*.025);
    if ~sameObstacle(j),continue;end
    dt=deltaT(j);
    % Trapezoid relation: D psi_(i-1) = psi_i - lambda_i psi_(i-1).
    % Works for smooth obstacle h in continuous time. Finite differences
    % can deviate because of sample-and-hold jumps and 3-signif-digit logs.
    rH(j)=(h(j+1)-h(j))/dt - ...
        .5*((p1(j)-lambda(1)*h(j))+(p1(j+1)-lambda(1)*h(j+1)));
    rPsi1(j)=(p1(j+1)-p1(j))/dt - ...
        .5*((p2(j)-lambda(2)*p1(j))+(p2(j+1)-lambda(2)*p1(j+1)));
    rPsi2(j)=(p2(j+1)-p2(j))/dt - ...
        .5*((p3(j)-lambda(3)*p2(j))+(p3(j+1)-lambda(3)*p2(j+1)));
    rPsi3(j)=(p3(j+1)-p3(j))/dt - ...
        .5*((p4(j)-lambda(4)*p3(j))+(p4(j+1)-lambda(4)*p3(j+1)));
    errPsi1(j)=(p1(j+1)-p1(j)) - ...
        .5*dt*((p2(j)-lambda(2)*p1(j))+(p2(j+1)-lambda(2)*p1(j+1)));
    % 0.01 is an explicitly chosen diagnostic significance threshold
    % (NOT a proof or numeric tolerance on the physical dynamics).
    suspect(j)=abs(errPsi1(j))>0.01;
end
pairs=table(startTime,deltaT,sameObstacle,rH,rPsi1,rPsi2,rPsi3, ...
    errPsi1,suspect,statusStart,'VariableNames', ...
    {'time','dt','comparable','hDerivativeResidual', ...
    'psi1DerivativeResidual','psi2DerivativeResidual', ...
    'psi3DerivativeResidual','psi1StepResidual', ...
    'suspiciousPsi1','status'});
counts=struct('totalRecords',height(rows),'usablePairs',sum(sameObstacle), ...
    'suspiciousPsi1',sum(suspect), ...
    'negativeH',sum(h<0),'negativePsi1',sum(p1<0), ...
    'negativePsi2',sum(p2<0),'negativePsi3',sum(p3<0), ...
    'negativePsi4',sum(p4<0),'bodyAngleAbove45',sum(bodyAngle>45), ...
    'cableAngleAbove60',sum(cableAngle>60), ...
    'uncertified',sum(~ismember(status, ...
        {'NOMINAL_XY_PASSED','XY_TORQUE_CORRECTED', ...
         'XY_PHYSICAL_INPUT_CLIPPED','NO_ACTIVE_CONSTRAINTS'})));
out=struct('available',true,'source',source,'lambda',lambda, ...
    'rows',rows,'pairs',pairs,'counts',counts, ...
    'warning',['Stepwise derivative residuals require a stable obstacle ', ...
    'identity, a continuous obstacle model, and consistent state sampling. ', ...
    'They do not alone prove a generator or plant error.']);
end

function v=xytest_number(s,pattern)
tok=regexp(s,pattern,'tokens','once');
if isempty(tok),v=NaN;else,v=str2double(tok{1});end
end

function [ok,msg]=xytest_prepare(root)
ok=false;msg='';
opts={fullfile(root,'CBF_GENERATOR'), ...
    fullfile(root,'model','substance','CBF_GENERATOR'), ...
    fullfile(root,'substance','CBF_GENERATOR')};
for j=1:numel(opts)
    gen=opts{j};
    common=fullfile(gen,'COMMON');generated=fullfile(gen,'REAL','XY','GENERATED');
    if isfile(fullfile(generated,'CBF_REAL_XY_EVAL.m')) && ...
            isfile(fullfile(common,'CBF_COMMON_MODEL.m')) && ...
            isfile(fullfile(common,'CBF_COMMON_GEOMETRY.m'))
        addpath(common,generated,'-begin');
        if ~xytest_samepath(which('CBF_REAL_XY_EVAL'), ...
                fullfile(generated,'CBF_REAL_XY_EVAL.m'))
            msg='Generated evaluator is shadowed by another file on MATLAB path.';
            return
        end
        ok=true;msg=sprintf('Using ORIGINAL generated functions: %s',generated);
        return
    end
end
msg='Original CBF_GENERATOR/COMMON and REAL/XY/GENERATED not found under common_matlab.';
end

function tf=xytest_samepath(a,b)
a=strrep(char(a),'/',filesep);b=strrep(char(b),'/',filesep);
if ispc,tf=strcmpi(a,b);else,tf=strcmp(a,b);end
end

function P=xytest_parameters(p)
p=p(:);
P=struct('m',p(1),'J',p(2:4),'gravity',p(5),'mL',p(6), ...
    'cableL',p(7),'dstx',p(8),'dsty',p(9));
end

function G=xytest_geometry(P,cfg)
G=CBF_COMMON_GEOMETRY(P,struct('includeGround',false, ...
    'trimCableWithBodySpheres',false));
G.cableCenterFromLoad=0.5;
if isfield(cfg,'xyCableRadiusM'),b=cfg.xyCableRadiusM;else,b=.2;end
if isfield(cfg,'xyCableAxialMarginM'),margin=cfg.xyCableAxialMarginM;else,margin=.02;end
G.cableAxisShort=b;
G.cableAxisLong=max(P.cableL/2+margin,b);
end

function c=xytest_tiltcfg(cfg)
c=struct('bodyEnabled',true,'cableEnabled',true, ...
    'bodyMaxDeg',45,'cableMaxDeg',60, ...
    'bodyLambda',[2;2],'cableLambda',[1;2;8;16]);
if isfield(cfg,'xyBodyTiltEnabled'),c.bodyEnabled=logical(cfg.xyBodyTiltEnabled);end
if isfield(cfg,'xyCableTiltEnabled'),c.cableEnabled=logical(cfg.xyCableTiltEnabled);end
if isfield(cfg,'xyBodyTiltMaxDeg'),c.bodyMaxDeg=cfg.xyBodyTiltMaxDeg;end
if isfield(cfg,'xyCableTiltMaxDeg'),c.cableMaxDeg=cfg.xyCableTiltMaxDeg;end
if isfield(cfg,'xyBodyTiltLambda'),c.bodyLambda=cfg.xyBodyTiltLambda;end
if isfield(cfg,'xyCableTiltLambda'),c.cableLambda=cfg.xyCableTiltLambda;end
end

function out=xytest_model_cases(lambda)
q=[.98;.08;-.12;.10];q=q/norm(q);
pT=[.08;-.11;-.99];pT=pT/norm(pT);
x=[q;.1;-.12;.08;0;0;3;.4;-.2;.1;pT;.02;.03;-.04];
P=struct('m',.85,'J',[.012;.013;.022], ...
    'gravity',9.81,'mL',.14,'cableL',2,'dstx',0,'dsty',0);
G=xytest_geometry(P,struct());
ca=cos(.24);sa=sin(.24);cb=cos(-.31);sb=sin(-.31);
Rz=[ca -sa 0;sa ca 0;0 0 1];Ry=[cb 0 sb;0 1 0;-sb 0 cb];
part=struct('kind','ellipsoid','center',[1.4;-.9;2.6], ...
    'R',Rz*Ry,'params',struct('axes',[.5;.8;1.2]), ...
    'obstacleName','synthetic3d');
tilt=xytest_tiltcfg(struct());tilt.cableLambda=lambda;
steps=[.001;.005;.025];n=numel(steps);
stepS=steps;hError=zeros(n,1);psi1Error=zeros(n,1);
stateError=zeros(n,1);rhsError=zeros(n,1);
for j=1:n
    t=steps(j);u=[9.5;.25;-.2;.04];
    if j==2,u(2:3)=[-.4;.3];end
    if j==3,u(2:3)=[.6;-.45];end
    E=CBF_REAL_XY_EVAL(x,u,P,G,part,lambda,tilt);
    J=CBF_REAL_XY_JET(x,u,P);
    rhsError(j)=max(abs(J.stateTaylor(:,2)-CBF_COMMON_MODEL(x,u,P)));
    z=xytest_rk4(x,u,P,t,max(20,ceil(t/.0002)));
    zz=z;zz(1:4)=z(1:4)/norm(z(1:4));
    zz(14:16)=z(14:16)/norm(z(14:16));
    Eend=CBF_REAL_XY_EVAL(zz,u,P,G,part,lambda,tilt);
    tt=t.^(0:4).';
    hpredict=E.hTaylor(:,1).'*tt;
    hError(j)=abs(hpredict-Eend.stages(1));
    psi1coefs=(1:4)'.*E.hTaylor(2:5,1) + lambda(1)*E.hTaylor(1:4,1);
    psi1predict=psi1coefs.'*(t.^(0:3)).';
    psi1Error(j)=abs(psi1predict-Eend.stages(2));
    xpred=J.stateTaylor*(t.^(0:4)).';
    stateError(j)=max(abs(xpred-z));
end
out=struct('available',true,'message', ...
    'Synthetic 3D model: generator jets vs original CBF_COMMON_MODEL RK4, fixed ZOH input.', ...
    'cases',table(stepS,hError,psi1Error,stateError,rhsError, ...
    'VariableNames',{'stepS','hTaylorErrorM','psi1TaylorError', ...
    'stateTaylorMaxError','initialRhsError'}), ...
    'warning',['These tests only verify agreement of the generator ', ...
      'with CBF_COMMON_MODEL, NOT with the real Sim/EKF dynamics.']);
end

function z=xytest_rk4(x,u,P,dt,nSteps)
z=double(x(:));h=dt/nSteps;
for k=1:nSteps
    a=CBF_COMMON_MODEL(z,u,P);
    b=CBF_COMMON_MODEL(z+.5*h*a,u,P);
    c=CBF_COMMON_MODEL(z+.5*h*b,u,P);
    d=CBF_COMMON_MODEL(z+h*c,u,P);
    z=z+h*(a+2*b+2*c+d)/6;
end
end

function out=xytest_capture_analysis(S)
if isempty(S)
    out=struct('available',false,'message', ...
        'No real 19-state snapshots supplied. Use optional debugger capture.', ...
        'pairs',table(),'samples',table());return
end
n=numel(S);
time=nan(n,1);evalH=nan(n,1);loggedH=nan(n,1);
sourceStatus=cell(n,1);modelCheckError=nan(n,1);
for i=1:n
    time(i)=S(i).time;sourceStatus{i}=xytest_fieldstr(S(i).info,'status');
    try
        ev=xytest_eval_snapshot(S(i),S(i).x,S(i).parts,S(i).u);
        if ~isempty(ev.stages)
            evalH(i)=ev.stages(1,1);
            if isfield(S(i).info,'stageValues') && ~isempty(S(i).info.stageValues)
                loggedH(i)=S(i).info.stageValues(1,1);
                modelCheckError(i)=abs(evalH(i)-loggedH(i));
            end
        end
    catch
        % Keep failed sample visible rather than silently removing it.
    end
end
samples=table(time,evalH,loggedH,modelCheckError,sourceStatus, ...
    'VariableNames',{'t','recomputedH','controllerH','hMismatch', 'status'});
if n<2
    out=struct('available',true,'message','Only one snapshot; no time-step comparison.', ...
        'pairs',table(),'samples',samples);return
end
[~,ind]=sort(time);S=S(ind);
startT=[];step=[];stateError=[];hPredError=[];psi1PredError=[];
geometryChanged=[];paramChanged=[];
for i=1:numel(S)-1
    a=S(i);b=S(i+1);dt=b.time-a.time;
    if ~isfinite(dt) || dt<=0 || dt>0.1,continue;end
    if numel(a.parts)~=1 || numel(b.parts)~=1,continue;end
    startT(end+1,1)=a.time;step(end+1,1)=dt; %#ok<AGROW>
    geometryChanged(end+1,1)=~isequaln(a.parts,b.parts); %#ok<AGROW>
    paramChanged(end+1,1)=max(abs(a.Praw-b.Praw))>1e-9; %#ok<AGROW>
    try
        P=xytest_parameters(a.Praw);
        xModel=xytest_rk4(a.x,a.u,P,dt,max(25,ceil(dt/.0005)));
        stateError(end+1,1)=max(abs(xModel-b.x)); %#ok<AGROW>
        ev0=xytest_eval_snapshot(a,a.x,a.parts,a.u);
        afterX=b.x;
        afterX(1:4)=afterX(1:4)/norm(afterX(1:4));
        afterX(14:16)=afterX(14:16)/norm(afterX(14:16));
        % SAME old obstacle geometry to isolate plant-vs-jet effect.
        ev1=xytest_eval_snapshot(a,afterX,a.parts,a.u);
        predH=ev0.hTaylor(:,1).'*(dt.^(0:4)).';
        psiCoef=(1:4)'.*ev0.hTaylor(2:5,1)+ ...
            a.cfg.xyLambda(1)*ev0.hTaylor(1:4,1);
        predPsi1=psiCoef.'*(dt.^(0:3)).';
        hPredError(end+1,1)=ev1.stages(1,1)-predH; %#ok<AGROW>
        psi1PredError(end+1,1)=ev1.stages(2,1)-predPsi1; %#ok<AGROW>
    catch
        stateError(end+1,1)=NaN;hPredError(end+1,1)=NaN; %#ok<AGROW>
        psi1PredError(end+1,1)=NaN; %#ok<AGROW>
    end
end
pairs=table(startT,step,geometryChanged,paramChanged,stateError, ...
    hPredError,psi1PredError,'VariableNames', ...
    {'time','dt','obstacleChanged','parametersChanged', ...
    'stateInfError','hPredictionResidual','psi1PredictionResidual'});
out=struct('available',true,'message', ...
    'Real debugger captures: fixed-ZOH model propagated against next estimated state.', ...
    'samples',samples,'pairs',pairs, ...
    'warning',['EKF corrections, changing mass estimates, actual vs ', ...
    'applied input timing, and obstacle changes must be ruled out before ', ...
    'attributing model-vs-state residuals to the CBF generator.']);
end

function E=xytest_eval_snapshot(s,x,parts,u)
P=xytest_parameters(s.Praw);G=xytest_geometry(P,s.cfg);
x=x(:);
if abs(norm(x(1:4))-1)<=.005 && abs(norm(x(14:16))-1)<=.005
    x(1:4)=x(1:4)/norm(x(1:4));
    x(14:16)=x(14:16)/norm(x(14:16));
end
E=CBF_REAL_XY_EVAL(x(:),u(:),P,G,parts(:), ...
    double(s.cfg.xyLambda(:)),xytest_tiltcfg(s.cfg));
end

function s=xytest_fieldstr(o,key)
s='';if isstruct(o) && isfield(o,key),s=char(o.(key));end
end

% =============================================================
% LIVE LOGGER RETENTION AUDIT + PER-STEP CAUSAL DECOMPOSITION
% =============================================================
function out=xytest_empty_live(msg)
emptyAudit=table(zeros(0,1),false(0,1),false(0,1),false(0,1), ...
    false(0,1),false(0,1),nan(0,1),nan(0,1),cell(0,1), ...
    'VariableNames',{'t','hasState19','hasObstacleGeometry','hasAppliedInput', ...
    'hasPhysicalParameters','complete','observedParts','sensorTimestamp','status'});
out=struct('available',false,'message',msg,'audit',emptyAudit, ...
    'pairs',table(),'coverage',struct('total',0,'countX',0, ...
    'countObstacle',0,'countInput',0,'countParameter',0,'countComplete',0));
end

function v=xytest_finite_max(x)
x=x(isfinite(x));if isempty(x),v=NaN;else,v=max(x);end
end

function [logger,app]=xytest_discover_live(source)
logger=[];app=[];
if ~isempty(source) && ~ischar(source) && ~(isstring(source)&&isscalar(source))
    if isobject(source) && ~isempty(xytest_member(source,'logger'))
        app=source;logger=xytest_member(source,'logger');
    else
        logger=source;
    end
end
if isempty(app)
    try
        if evalin('base','exist(''app'',''var'')')
            app=evalin('base','app');
        end
    catch
    end
end
if isempty(logger) && ~isempty(app),logger=xytest_member(app,'logger');end
if isempty(logger)
    try
        if evalin('base','exist(''logger'',''var'')')
            logger=evalin('base','logger');
        end
    catch
    end
end
end

function [Praw,reason]=xytest_getP(app,supplied)
Praw=[];reason='not recorded and no current app.agent.parameter';
if ~isempty(supplied),Praw=double(supplied(:));reason='explicit parameter argument';return;end
if isempty(app),return;end
try
    agents=xytest_member(app,'agent');
    if ~isempty(agents)
        par=xytest_member(agents(1),'parameter');
        if ~isempty(par)
            vals=par.get(["mass","jx","jy","jz","gravity","loadmass","cableL"]);
            vals=double(vals(:));
            if numel(vals)==7 && all(isfinite(vals))
                Praw=[vals;0;0];reason='current agent.parameter (assumed unchanged through history)';
                return
            end
        end
    end
catch
end
end

function x=xytest_extract_x(c,est)
x=xytest_member(c,'x');
if isnumeric(x) && isreal(x) && numel(x)==19 && all(isfinite(x(:)))
    x=double(x(:));return
end
% In case LOGGER retains estimator.state but truncates controller.result.x,
% reconstruct the same HLC x=[q;w;pL;vL;pT;wL].
x=[];
try
    state=xytest_member(est,'state');
    if isempty(state),return;end
    q=state.getq('compact');
    w=xytest_member(state,'w');pL=xytest_member(state,'pL');
    vL=xytest_member(state,'vL');pT=xytest_member(state,'pT');
    wL=xytest_member(state,'wL');
    v=[q(:);w(:);pL(:);vL(:);pT(:);wL(:)];
    if isnumeric(v) && numel(v)==19 && all(isfinite(v(:)))
        x=double(v(:));
    end
catch
    x=[];
end
end

function [parts,tag]=xytest_parts(s)
parts=[];tag='';
obs=xytest_member(s,'cbfObservedWorld');
maybe=xytest_member(obs,'parts');
if ~isstruct(maybe) || isempty(maybe) || numel(maybe)~=1,return;end
try
    part=maybe(1);
    center=xytest_member(part,'center');
    if ~isnumeric(center) || numel(center)~=3 || any(~isfinite(center(:))),return;end
    kind=xytest_member(part,'kind');if isempty(kind),kind='ellipsoid';end
    if strcmp(char(kind),'ellipsoid')
        params=xytest_member(part,'params');axes=xytest_member(params,'axes');
        if isempty(axes),axes=xytest_member(part,'axes');end
        rot=xytest_member(part,'R');
        if ~isnumeric(axes)||numel(axes)~=3 || ~isequal(size(rot),[3 3]),return;end
        if any(~isfinite(axes(:))) || any(axes(:)<=0)||any(~isfinite(rot(:))),return;end
    elseif strcmp(char(kind),'sphere')
        params=xytest_member(part,'params');rad=xytest_member(params,'radius');
        if isempty(rad),rad=xytest_member(part,'radius');end
        if ~isnumeric(rad)||numel(rad)~=1||~isfinite(rad)||rad<=0,return;end
    else
        return
    end
    tag=xytest_member(part,'obstacleName');
    if isempty(tag),tag=xytest_member(part,'name');end
    if isempty(tag),tag='UNNAMED';end
    tag=char(tag);
    parts=maybe;
catch
    parts=[];
end
end

function u=xytest_applied_u(c)
u=xytest_member(c,'input');
if ~isnumeric(u)||~isreal(u)||numel(u)~=4||any(~isfinite(u(:)))
    info=xytest_member(c,'cbf');u=xytest_member(info,'input');
end
if isnumeric(u)&&isreal(u)&&numel(u)==4&&all(isfinite(u(:)))
    u=double(u(:));
else
    u=[];
end
end

function Praw=xytest_loggedP(c,backup)
Praw=xytest_member(c,'Praw');
if isempty(Praw),Praw=xytest_member(c,'CBF_Praw');end
if isempty(Praw),Praw=backup;end
if isnumeric(Praw) && isreal(Praw)&&numel(Praw)==9&&all(isfinite(Praw(:)))
    Praw=double(Praw(:));
else
    Praw=[];
end
end

function out=xytest_live_history(source,lambda,ready,suppliedP)
out=xytest_empty_live('LOGGER could not be inspected');
[logger,app]=xytest_discover_live(source);
if isempty(logger),out.message='app.logger is not accessible; pass app.logger explicitly';return;end
D=xytest_member(logger,'Data');As=xytest_member(D,'agent');time=xytest_member(D,'t');
if isempty(As)||~isnumeric(time)||isempty(time)
    out.message='LOGGER.Data.agent or LOGGER.Data.t missing';return
end
agent=As(1);
ctrl=xytest_member(xytest_member(agent,'controller'),'result');
sensor=xytest_member(xytest_member(agent,'sensor'),'result');
est=xytest_member(xytest_member(agent,'estimator'),'result');
if isempty(ctrl),out.message='LOGGER Data.agent(1).controller.result missing';return;end
n=numel(time);liveK=xytest_member(logger,'k');
if isnumeric(liveK)&&isscalar(liveK)&&isfinite(liveK)
    n=min(n,max(0,floor(double(liveK))));
end
[baseP,pReason]=xytest_getP(app,suppliedP);
t=double(time(1:n));t=t(:);
hasX=false(n,1);hasObs=false(n,1);hasU=false(n,1);hasP=false(n,1);
observedParts=nan(n,1);sensorTs=nan(n,1);cbfStatus=cell(n,1);
samples=struct('x',{},'parts',{},'tag',{},'u',{},'P',{}, ...
    'time',{},'info',{});
samples(1:n)=struct('x',[],'parts',[],'tag','','u',[],'P',[], ...
    'time',NaN,'info',[]);
for k=1:n
    c=xytest_history_element(ctrl,k);
    s=xytest_history_element(sensor,k);
    e=xytest_history_element(est,k);
    x=xytest_extract_x(c,e);[parts,tag]=xytest_parts(s);
    u=xytest_applied_u(c);Praw=xytest_loggedP(c,baseP);
    hasX(k)=~isempty(x);hasObs(k)=~isempty(parts);
    hasU(k)=~isempty(u);hasP(k)=~isempty(Praw);
    info=xytest_member(c,'cbf');
    cbfStatus{k}=xytest_string(xytest_member(info,'status'));
    observedParts(k)=xytest_scalar(xytest_member(info,'observedParts'));
    sensorTs(k)=xytest_scalar(xytest_member(s,'cbfSensorTime'));
    samples(k)=struct('x',x,'parts',parts,'tag',tag, ...
        'u',u,'P',Praw,'time',t(k),'info',info);
end
complete=hasX&hasObs&hasU&hasP&isfinite(t);
out.audit=table(t,hasX,hasObs,hasU,hasP,complete,observedParts,sensorTs,cbfStatus, ...
    'VariableNames',{'t','hasState19','hasObstacleGeometry','hasAppliedInput', ...
    'hasPhysicalParameters','complete','observedParts','sensorTimestamp','status'});
out.coverage=struct('total',n,'countX',sum(hasX),'countObstacle',sum(hasObs), ...
    'countInput',sum(hasU),'countParameter',sum(hasP),'countComplete',sum(complete));
out.available=any(complete);
out.message=sprintf('P source: %s. Exact historical P changes cannot be proven without saved P.',pReason);
if ~ready || sum(complete)<2
    if ~ready,out.message=[out.message ' Generated evaluator not available.'];end
    if sum(complete)<2,out.message=[out.message ' Insufficient complete historical samples.'];end
    return
end
% Prioritize around measured failure and representative stages, bound offline
% evaluation cost; audit always spans the ENTIRE saved logger.
dSteps=diff(t); positiveSteps=dSteps(isfinite(dSteps)&dSteps>0);
if isempty(positiveSteps),out.message=[out.message ' No positive step durations.'];return;end
stepMedian=median(positiveSteps);
pairs=find(complete(1:end-1)&complete(2:end) & ...
    dSteps>0 & abs(dSteps-stepMedian)<max(1e-5,stepMedian*.02));
if isempty(pairs),out.message=[out.message ' No adjacent complete time pairs.'];return;end
priority=(t(pairs)>=24.5 & t(pairs)<=27.2) | ...
    (t(pairs)>=18.0 & t(pairs)<=19.0) | ...
    (t(pairs)>=33.5 & t(pairs)<=38.2);
selected=pairs(priority);
remaining=pairs(~priority);
maxPairs=260;
if numel(selected)>maxPairs
    selected=selected(unique(round(linspace(1,numel(selected),maxPairs))));
elseif numel(selected)<maxPairs && ~isempty(remaining)
    budget=maxPairs-numel(selected);
    if numel(remaining)>budget
        remaining=remaining(unique(round(linspace(1,numel(remaining),budget))));
    end
    selected=[selected;remaining];
end
selected=sort(selected(:));
N=numel(selected);
tt=nan(N,1);dtv=nan(N,1);geomChanged=false(N,1);partChanged=false(N,1);
modelStateErr=nan(N,1);geoPsi1=nan(N,1);statePsi1=nan(N,1);
paramPsi1=nan(N,1);taylorPsi1=nan(N,1);logPsi1=nan(N,1);
recompPsi1=nan(N,1);logPsi1Minus=nan(N,1);
loggedStart=nan(N,1);loggedEnd=nan(N,1);
errText=cell(N,1);
for j=1:N
    k=selected(j);a=samples(k);b=samples(k+1);
    tt(j)=a.time;dtv(j)=b.time-a.time;
    partChanged(j)=~strcmp(a.tag,b.tag);
    geomChanged(j)=~isequaln(a.parts,b.parts);
    try
        % q and pT projection reproduces the Scheme02 evaluation convention.
        xa=xytest_project_state(a.x);xb=xytest_project_state(b.x);
        if partChanged(j),errText{j}='obstacle ID changed; skip decomposition';continue;end
        Pa=xytest_parameters(a.P);Pb=xytest_parameters(b.P);
        % Get exact geometric CBF settings when app.controller is accessible.
        cfg=xytest_actual_cfg(app,lambda);
        Ga=xytest_geometry(Pa,cfg);Gb=xytest_geometry(Pb,cfg);
        tilts=xytest_tiltcfg(cfg);
        E0=CBF_REAL_XY_EVAL(xa,a.u,Pa,Ga,a.parts,lambda,tilts);
        E1state=CBF_REAL_XY_EVAL(xb,a.u,Pa,Ga,a.parts,lambda,tilts);
        E1geom=CBF_REAL_XY_EVAL(xb,a.u,Pa,Ga,b.parts,lambda,tilts);
        E1params=CBF_REAL_XY_EVAL(xb,a.u,Pb,Gb,b.parts,lambda,tilts);
        xm=xytest_rk4(xa,a.u,Pa,dtv(j),max(20,ceil(dtv(j)/.001)));
        xm=xytest_project_state(xm);
        Em=CBF_REAL_XY_EVAL(xm,a.u,Pa,Ga,a.parts,lambda,tilts);
        modelStateErr(j)=max(abs(xm-xb));
        geoPsi1(j)=E1geom.stages(2,1)-E1state.stages(2,1);
        statePsi1(j)=E1state.stages(2,1)-Em.stages(2,1);
        paramPsi1(j)=E1params.stages(2,1)-E1geom.stages(2,1);
        taylorCoef=(1:4)'.*E0.hTaylor(2:5,1)+ ...
             lambda(1)*E0.hTaylor(1:4,1);
        psi1Pred=taylorCoef.'*(dtv(j).^(0:3)).';
        taylorPsi1(j)=Em.stages(2,1)-psi1Pred;
        recompPsi1(j)=E1params.stages(2,1);
        v0=xytest_member(a.info,'stageValues');
        v1=xytest_member(b.info,'stageValues');
        if isnumeric(v0)&&numel(v0)>=5
            loggedStart(j)=double(v0(2,1));
        end
        if isnumeric(v1)&&numel(v1)>=5
            loggedEnd(j)=double(v1(2,1));
        end
        if isfinite(loggedStart(j))&&isfinite(loggedEnd(j))
            logPsi1(j)=loggedEnd(j)-loggedStart(j);
            logPsi1Minus(j)=(loggedEnd(j)-E1params.stages(2,1)) ...
                -(loggedStart(j)-E0.stages(2,1));
        end
        errText{j}='OK';
    catch ME
        errText{j}=[ME.identifier ':' ME.message];
    end
end
out.pairs=table(tt,dtv,partChanged,geomChanged,geoPsi1, ...
    statePsi1,paramPsi1,taylorPsi1,logPsi1,recompPsi1, ...
    logPsi1Minus,modelStateErr,loggedStart,loggedEnd,errText, ...
    'VariableNames',{'t','dt','obstacleIdentityChanged','obstacleGeometryChanged', ...
    'geometryPsi1Step','stateVsModelPsi1Step','parameterPsi1Step', ...
    'modelTaylorPsi1Step','loggedPsi1Step','recomputedEndPsi1', ...
    'loggedPsi1MinusRecomputed','stateInfError', ...
    'loggedStartPsi1','loggedEndPsi1','evaluationStatus'});
out.message=[out.message ' Breakdown uses ONE ZOH held u per interval as model hypothesis, ', ...
    'not a claim the actual plant input was constant. State differences may contain EKF corrections.'];
end

function cfg=xytest_actual_cfg(app,lambda)
cfg=struct('xyLambda',lambda,'xyBodyTiltEnabled',true, ...
    'xyBodyTiltMaxDeg',45,'xyBodyTiltLambda',[2;2], ...
    'xyCableTiltEnabled',true,'xyCableTiltMaxDeg',60, ...
    'xyCableTiltLambda',lambda, 'xyCableRadiusM',.2,'xyCableAxialMarginM',.02);
try
    if ~isempty(app)
        agents=xytest_member(app,'agent');
        if ~isempty(agents)
            obj=xytest_member(agents(1),'controller');
            conf=xytest_member(obj,'cbfConfig');
            if isstruct(conf)
                keys={'xyBodyTiltEnabled','xyBodyTiltMaxDeg','xyBodyTiltLambda', ...
                    'xyCableTiltEnabled','xyCableTiltMaxDeg','xyCableTiltLambda', ...
                    'xyCableRadiusM','xyCableAxialMarginM'};
                for i=1:numel(keys)
                    if isfield(conf,keys{i}),cfg.(keys{i})=conf.(keys{i});end
                end
            end
        end
    end
catch
end
end

function x=xytest_project_state(x)
x=double(x(:));
q=norm(x(1:4));p=norm(x(14:16));
if q<=0 || p<=0 || abs(q-1)>.005 || abs(p-1)>.005
    error('CBFXYTest:StateNorm','Cannot evaluate state outside small projection band.');
end
x(1:4)=x(1:4)/q;x(14:16)=x(14:16)/p;
end

function xytest_write_zip(root,report)
name='CBF_XY_DERIVATIVE_DIAGNOSTIC_RESULTS.zip';
zfile=fullfile(root,name);
work=tempname;mkdir(work);
cleanup=onCleanup(@()xytest_cleanup(work)); %#ok<NASGU>
save(fullfile(work,'REPORT.mat'),'report','-v7.3');
files={'REPORT.mat','SUMMARY.txt'};
if ~isempty(report.logSource) && isfile(report.logSource)
    copyfile(report.logSource,fullfile(work,'CONSOLE_SOURCE.txt'));
    files{end+1}='CONSOLE_SOURCE.txt';
end
if report.log.available
    writetable(report.log.rows,fullfile(work,'LOG_RECORDS.csv'));
    writetable(report.log.pairs,fullfile(work,'LOG_DERIVATIVE_PAIRS.csv'));
    files=[files,{'LOG_RECORDS.csv','LOG_DERIVATIVE_PAIRS.csv'}]; %#ok<AGROW>
end
if report.generatorModel.available
    writetable(report.generatorModel.cases,fullfile(work,'MODEL_JET_VS_RK4.csv'));
    files{end+1}='MODEL_JET_VS_RK4.csv'; %#ok<AGROW>
end
if report.captures.available
    writetable(report.captures.samples,fullfile(work,'CAPTURE_SAMPLES.csv'));
    files{end+1}='CAPTURE_SAMPLES.csv'; %#ok<AGROW>
    if ~isempty(report.captures.pairs)
        writetable(report.captures.pairs,fullfile(work,'CAPTURE_PAIRS.csv'));
        files{end+1}='CAPTURE_PAIRS.csv'; %#ok<AGROW>
    end
end
if isfield(report,'live')
    writetable(report.live.audit,fullfile(work,'LIVE_LOGGER_AVAILABILITY.csv'));
    files{end+1}='LIVE_LOGGER_AVAILABILITY.csv'; %#ok<AGROW>
    if ~isempty(report.live.pairs)
        writetable(report.live.pairs,fullfile(work,'LIVE_STATE_OBSTACLE_DECOMPOSITION.csv'));
        files{end+1}='LIVE_STATE_OBSTACLE_DECOMPOSITION.csv'; %#ok<AGROW>
    end
end
if isfield(report,'fullAudit')
    xyfull_summary(work,report);
    files{end+1}='FULL_AUDIT_SUMMARY.txt'; %#ok<AGROW>
    if report.fullAudit.available
        writetable(report.fullAudit.samples,fullfile(work,'FULL_AUDIT_SAMPLES.csv'));
        writetable(report.fullAudit.pairs,fullfile(work,'FULL_AUDIT_PAIRS.csv'));
        writetable(report.fullAudit.failures,fullfile(work,'FULL_AUDIT_FAILURES.csv'));
        files=[files,{'FULL_AUDIT_SAMPLES.csv','FULL_AUDIT_PAIRS.csv','FULL_AUDIT_FAILURES.csv'}]; %#ok<AGROW>
    end
end
fid=fopen(fullfile(work,'SUMMARY.txt'),'w');
if fid<0,error('CBFXYTest:Files','Cannot create report summary.');end
fidCleanup=onCleanup(@()fclose(fid)); %#ok<NASGU>
fprintf(fid,'CBF id=2 standalone derivative-consistency diagnostic\n');
fprintf(fid,'Date: %s\nRoot: %s\n',report.createdAt,root);
fprintf(fid,'xyLambda = [%.12g %.12g %.12g %.12g]\n',report.lambda);
fprintf(fid,'Console log: %s\n',report.logSource);
fprintf(fid,'Generator setup: %s\n\n',report.setupMessage);
if report.log.available
    c=report.log.counts;
    fprintf(fid,'OBSERVED HISTORY: %d records, %d comparable 25ms pairs\n', ...
        c.totalRecords,c.usablePairs);
    fprintf(fid,'psi1 step-discrepancy >0.01: %d pairs (diagnostic, NOT a proof)\n', ...
        c.suspiciousPsi1);
    fprintf(fid,'Negative h/psi1/psi2/psi3/psi4 counts: %d/%d/%d/%d/%d\n', ...
        c.negativeH,c.negativePsi1,c.negativePsi2,c.negativePsi3,c.negativePsi4);
    fprintf(fid,'Angle limit exceedances (body/cable): %d/%d\n', ...
        c.bodyAngleAbove45,c.cableAngleAbove60);
    if isfinite(c.uncertified)
        fprintf(fid,'Reported uncertified statuses: %d\n',c.uncertified);
    else
        fprintf(fid,'Reported uncertified statuses: UNKNOWN (stage plot only)\n');
    end
    if ~isempty(report.log.pairs)
        p=report.log.pairs;
        vabs=abs(p.psi1StepResidual);vabs(~isfinite(vabs))=-Inf;
        [~,ids]=sort(vabs,'descend');
        ids=ids(isfinite(p.psi1StepResidual(ids)));
        fprintf(fid,'\nTOP psi1 step discrepancies (time, step residual, rate residual):\n');
        for i=1:min(20,numel(ids))
            j=ids(i);
            fprintf(fid,'t=%.3f errStep=%+.6g errRate=%+.6g status=%s\n', ...
                p.time(j),p.psi1StepResidual(j),p.psi1DerivativeResidual(j),p.status{j});
        end
    end
else
    fprintf(fid,'NO OBSERVED HISTORY: %s\n',report.log.warning);
end
if report.generatorModel.available
    fprintf(fid,'\nSYNTHETIC generator-vs-original-model (fixed physical ZOH input):\n');
    v=report.generatorModel.cases;
    for j=1:height(v)
        fprintf(fid,'dt=%.4f hError=%.6g psi1Error=%.6g stateError=%.6g rhsError=%.6g\n', ...
            v.stepS(j),v.hTaylorErrorM(j),v.psi1TaylorError(j), ...
            v.stateTaylorMaxError(j),v.initialRhsError(j));
    end
else
    fprintf(fid,'\nMODEL TEST SKIPPED: %s\n',report.generatorModel.message);
end
if isfield(report,'live')
    l=report.live;c=l.coverage;
    fprintf(fid,'\nSAVED LOGGER STATE / OBSTACLE RETENTION:\n');
    fprintf(fid,'scanned=%d, x19=%d, obstacle geometry=%d, physical input=%d, Praw=%d, all available=%d\n', ...
        c.total,c.countX,c.countObstacle,c.countInput,c.countParameter,c.countComplete);
    fprintf(fid,'detailed consecutive pairs computed=%d, reason=%s\n',height(l.pairs),l.message);
    if ~isempty(l.pairs)
        pp=l.pairs;
        fprintf(fid,'MAX step effect magnitudes: observed geometry %.6g, model/state %.6g, parameter %.6g\n', ...
            xytest_finite_max(abs(pp.geometryPsi1Step)), ...
            xytest_finite_max(abs(pp.stateVsModelPsi1Step)), ...
            xytest_finite_max(abs(pp.parameterPsi1Step)));
        fprintf(fid,'25.0 s nearby pairs (up to 16):\n');
        ks=find(pp.t>=24.85 & pp.t<=25.4);
        for jj=ks(1:min(16,numel(ks)))'
            fprintf(fid,'t=%.3f geo=%+.5g state=%+.5g param=%+.5g jet=%+.5g logMinusEval=%+.5g maxStateError=%.5g\n', ...
                pp.t(jj),pp.geometryPsi1Step(jj),pp.stateVsModelPsi1Step(jj), ...
                pp.parameterPsi1Step(jj),pp.modelTaylorPsi1Step(jj), ...
                pp.loggedPsi1MinusRecomputed(jj),pp.stateInfError(jj));
        end
    else
        fprintf(fid,'No full-state/obstacle decomposition is possible from absent historical data; do not infer a cause.\n');
    end
end
fprintf(fid,'\nReal full-state captured samples: %d\n',height(report.captures.samples));
if report.captures.available
    fprintf(fid,'Adjacent captured pairs: %d\n',height(report.captures.pairs));
end
fprintf(fid,['\nCAVEATS:\n' ...
  '1) Console values may be rounded; plot/LOGGER values may be higher precision.\n' ...
  '2) A residual between sample times is NOT algebraic contradiction by itself.\n' ...
  '3) The generated jet test compares CBF_REAL_XY_EVAL against CBF_COMMON_MODEL, NOT the Sim plant.\n' ...
  '4) Real EKF state differences can include estimator corrections or model parameter updates.\n' ...
  '5) continueBounded can keep flight alive without proving hard CBF constraints.\n' ...
  '6) Missing historical x, obstacle geometry, or inputs cannot be recreated from psi stages.\n' ...
  '7) This tool does not change any controller, generator, or Sim source.\n']);
clear fidCleanup % close SUMMARY.txt before zip
if isfile(zfile),delete(zfile);end
zip(zfile,files,work);
end

function xytest_cleanup(folder)
if isfolder(folder),rmdir(folder,'s');end
end

% =============================================================
% COMPREHENSIVE INDEPENDENT AUDIT, READ-ONLY
% =============================================================
function o=xyfull_empty(msg)
o=struct('available',false,'message',msg,'samples',table(), ...
    'pairs',table(),'failures',table(),'statistics',struct());
end

function o=xyfull_audit(source,lambda,ready,suppliedP)
% This deliberately DOES NOT use/rewrite the CBF solver. It independently
% reconstructs the stored affine inequalities and audits them. The actual
% LOGGER is never modified. A historical parameter substituted by the
% current parameter is clearly labelled and never treated as proven.
o=xyfull_empty('Unstarted');
if ~ready,o.message='Generated CBF evaluator or physical model not found';return;end
[logger,app]=xytest_discover_live(source);
if isempty(logger),o.message='LOGGER unavailable';return;end
D=xytest_member(logger,'Data');agents=xytest_member(D,'agent');
times=xytest_member(D,'t');
if isempty(agents)||isempty(times)||~isnumeric(times)
    o.message='LOGGER.Data.agent or LOGGER.Data.t is unavailable';return;
end
A=agents(1);hc=xytest_member(xytest_member(A,'controller'),'result');
hs=xytest_member(xytest_member(A,'sensor'),'result');
he=xytest_member(xytest_member(A,'estimator'),'result');
if isempty(hc),o.message='Controller result history unavailable';return;end
n=numel(times);kLive=xytest_member(logger,'k');
if isnumeric(kLive)&&isscalar(kLive)&&isfinite(kLive)
    n=min(n,max(0,floor(double(kLive))));
end
cfg=xytest_actual_cfg(app,lambda);
[defaultP,psource]=xytest_getP(app,suppliedP);
if isempty(defaultP)
    psource='unavailable: no historical or present P';
end
% Check the actually instantiated controller configuration separately:
% xytest_actual_cfg uses supplied lambda as a fallback, not evidence.
confLambda=[];
try
    if ~isempty(app)
        agentLive=xytest_member(app,'agent');
        if ~isempty(agentLive)
            liveConf=xytest_member(agentLive(1).controller,'cbfConfig');
            confLambda=xytest_member(liveConf,'xyLambda');
        end
    end
catch
end
if isnumeric(confLambda)&&numel(confLambda)==4
    lambdaMismatch=max(abs(double(confLambda(:))-lambda(:)));
else
    lambdaMismatch=NaN; % live historical gain unknown
end
Bcfg=xytest_tiltcfg(cfg);
if isfield(cfg,'xyTorqueWeights') && numel(cfg.xyTorqueWeights)==2
    wt=double(cfg.xyTorqueWeights(:));
else
    wt=[1;1];
end
% Track every sample, including unavailable records and failures. We do
% not silently replace missing logged state/obstacles with current values.
s=struct('t',NaN,'x',[],'parts',[],'tag','','u',[],'p',[], ...
    'info',[],'c',[],'sensorT',NaN,'Psource','','origin',NaN);
samples=repmat(s,n,1);
cols={'t','hasX19','hasObstacle','hasInput','hasP9','unitQError', ...
    'unitCableError','sensorLagS','reportedSuccess','uncertified', ...
    'hLogged','hRecomputed','psi1Logged','psi1Recomputed', ...
    'psi2Logged','psi2Recomputed','psi3Logged','psi3Recomputed', ...
    'psi4Logged','psi4Recomputed','stageMaxAbsError', ...
    'stageScaledError','algebra0','algebra1','algebra2','algebra3', ...
    'maxLowerViolation','maxFinalViolation','qpFeasibleIndependent', ...
    'controllerQpConsistent','inputWithinBounds','bodyAngleDeg', ...
    'cableAngleDeg','bodyFinal','cableFinal','localEvalError'};
V=nan(n,numel(cols));V(:,1)=double(times(1:n));
status=cell(n,1);err=cell(n,1);obsTag=cell(n,1);
for k=1:n
    c=xytest_history_element(hc,k);
    sensor=xytest_history_element(hs,k);
    est=xytest_history_element(he,k);
    info=xytest_member(c,'cbf');
    x=xytest_extract_x(c,est);
    [parts,tag]=xytest_parts(sensor);
    u=xytest_applied_u(c);
    rawP=xytest_loggedP(c,defaultP);
    lag=xytest_scalar(xytest_member(sensor,'cbfSensorTime'));
    samples(k)=struct('t',V(k,1),'x',x,'parts',parts,'tag',tag, ...
        'u',u,'p',rawP,'info',info,'c',c,'sensorT',lag, ...
        'Psource',psource,'origin',k);
    V(k,2:5)=[~isempty(x),~isempty(parts),~isempty(u),~isempty(rawP)];
    status{k}=xytest_string(xytest_member(info,'status'));
    if isempty(status{k}),status{k}='NO_STATUS';end
    obsTag{k}=tag;
    V(k,8)=lag-V(k,1);
    suc=xytest_member(info,'success');
    if isscalar(suc)&&(isnumeric(suc)||islogical(suc)),V(k,9)=double(suc);end
    if isfinite(V(k,9)),V(k,10)=1-V(k,9);end
    if ~isempty(x)
        V(k,6)=abs(norm(x(1:4))-1);
        V(k,7)=abs(norm(x(14:16))-1);
    end
    V(k,32)=xytest_scalar(xytest_member(info,'tiltBodyDegrees'));
    V(k,33)=xytest_scalar(xytest_member(info,'tiltCableDegrees'));
    bt=xytest_member(info,'tiltBodyStages');
    ct=xytest_member(info,'tiltCableStages');
    if isnumeric(bt)&&~isempty(bt),V(k,34)=double(bt(end));end
    if isnumeric(ct)&&~isempty(ct),V(k,35)=double(ct(end));end
    if ~isempty(u)
        V(k,31)=all(u(:)>=[0;-1;-1;-1]-1e-10 & ...
            u(:)<=[20;1;1;1]+1e-10);
    end
    if isempty(x)||isempty(u)||isempty(rawP)||isempty(parts),continue;end
    try
        xx=xytest_project_state(x);P=xytest_parameters(rawP);
        geom=xytest_geometry(P,cfg);
        % Use the exact applied 4-input and compare with the full vector
        % of logged final HOCBF stages. The source assumption is recorded.
        ev=CBF_REAL_XY_EVAL(xx,u,P,geom,parts,lambda,Bcfg);
        st=double(ev.stages(:,1));
        V(k,[12 14 16 18 20])=st(:)';
        V(k,32)=ev.tiltBodyDegrees;
        V(k,33)=ev.tiltCableDegrees;
        if ~isempty(ev.tiltBodyStages),V(k,34)=ev.tiltBodyStages(end);end
        if ~isempty(ev.tiltCableStages),V(k,35)=ev.tiltCableStages(end);end
        logStages=xytest_member(info,'stageValues');
        if isnumeric(logStages)&&isequal(size(logStages),[5 1])
            V(k,[11 13 15 17 19])=double(logStages(:))';
            delta=st-logStages(:);
            V(k,21)=max(abs(delta));
            V(k,22)=max(abs(delta)./max(1,abs(logStages(:))));
        end
        % Independent derivative identity at the SAME state:
        % d psi_(i-1) = psi_i - lambda_i*psi_(i-1).
        H=ev.hDerivatives(:,1);
        for j=0:3
            pol=1;
            for l=1:j,pol=conv(pol,[lambda(l) 1]);end
            derivPsi=pol*H(2:j+2);
            V(k,23+j)=derivPsi-(st(j+2)-lambda(j+1)*st(j+1));
        end
        lower=[ev.stages(1:4,1); ev.tiltBodyStages(1:end-1); ...
            ev.tiltCableStages(1:end-1)];
        V(k,27)=max([0;-lower]);
        endVals=[ev.stages(5,1);ev.tiltBodyStages(end); ...
            ev.tiltCableStages(end)];
        V(k,28)=max([0;-endVals]);
        V(k,31)=all(u(:)>=[0;-1;-1;-1]-1e-10 & ...
            u(:)<=[20;1;1;1]+1e-10);
        % Reconstruct the strict 2D QP WITHOUT using scheme solver.
        baseU=[u(1);0;0;u(4)];
        bx=CBF_REAL_XY_EVAL(xx,baseU+[0;1;0;0],P,geom,parts,lambda,Bcfg);
        by=CBF_REAL_XY_EVAL(xx,baseU+[0;0;1;0],P,geom,parts,lambda,Bcfg);
        b0=CBF_REAL_XY_EVAL(xx,baseU,P,geom,parts,lambda,Bcfg);
        B=[bx.stages(5,1)-b0.stages(5,1),by.stages(5,1)-b0.stages(5,1)];
        bias=b0.stages(5,1);
        if Bcfg.bodyEnabled
            B=[B;bx.tiltBodyStages(end)-b0.tiltBodyStages(end), ...
                by.tiltBodyStages(end)-b0.tiltBodyStages(end)];
            bias=[bias;b0.tiltBodyStages(end)];
        end
        if Bcfg.cableEnabled
            B=[B;bx.tiltCableStages(end)-b0.tiltCableStages(end), ...
                by.tiltCableStages(end)-b0.tiltCableStages(end)];
            bias=[bias;b0.tiltCableStages(end)];
        end
        V(k,29)=xyfull_box_feasible(B,bias,1e-8);
        loggedB=xytest_member(info,'torqueInfluenceMatrix');
        loggedBias=xytest_member(info,'psi4Bias');
        if isnumeric(loggedB)&&isequal(size(loggedB),size(B))&& ...
                isnumeric(loggedBias)&&numel(loggedBias)==numel(bias)
            scale=max([1;abs(B(:));abs(bias(:))]);
            V(k,30)=max([abs(loggedB(:)-B(:)); ...
                abs(loggedBias(:)-bias(:))])/scale;
        end
    catch ME
        err{k}=[ME.identifier ' : ' ME.message];
        V(k,36)=1;
    end
end
T=array2table(V,'VariableNames',cols);
T.status=status;T.obstacleId=obsTag;T.evalError=err;
% Finite differences at microsteps, direct derivatives, and held-input
% plant-model transition. Measure separately so a bad algebraic relation
% cannot be misdiagnosed as a ZOH/estimator problem.
pp={'t','dt','pairComplete','obstacleChanged','parameterChanged', ...
    'inputChanged','hStepMeasured','hStepPredictedFromJet', ...
    'psi1StepMeasured','psi1StepPredictedFromJet', ...
    'psi1StepPredictedFromRK4','psi1ModelVsActualStep', ...
    'psi1JetVsRK4Step','psi1DerivativeFromStages', ...
    'psi1DerivativeFromTaylor','psi1DerivativeMicroRK4', ...
    'psi1DerivativeMicroError','xStateInfResidual', ...
    'qStateInfResidual','omegaBodyStateInfResidual', ...
    'pLoadStateInfResidual','vLoadStateInfResidual', ...
    'pCableStateInfResidual','omegaCableStateInfResidual', ...
    'observedHstepWithChangedObstacle','observedPsi1StepWithChangedObstacle', ...
    'loggedPsi1Step','loggedVsRecomputedStart','loggedVsRecomputedEnd', ...
    'sensorLagStartS','reportedUncertifiedStart','stateDeltaTimingError'};
% Sampling priority is deterministic, includes all failure-neighbourhoods.
valid=find(V(:,2)&V(:,3)&V(:,4)&V(:,5)&isfinite(V(:,1)));
valid=valid(valid<n);
valid=valid(V(valid+1,2)&V(valid+1,3)&V(valid+1,4)&V(valid+1,5));
allDt=diff(V(:,1));typ=median(allDt(isfinite(allDt)&allDt>0));
valid=valid(allDt(valid)>0 & abs(allDt(valid)-typ)<max(1e-5,.02*typ));
priority=false(numel(valid),1);
for i=1:numel(valid)
    kk=valid(i);
    priority(i)=((V(kk,1)>=24.4&&V(kk,1)<=27.6)|| ...
        (V(kk,1)>=18&&V(kk,1)<=20)|| ...
        (V(kk,1)>=33.4&&V(kk,1)<=38.4)|| ...
        (isfinite(V(kk,10))&&V(kk,10)>0)|| ...
        (isfinite(V(kk+1,10))&&V(kk+1,10)>0)|| ...
        (isfinite(V(kk,27))&&V(kk,27)>1e-8)|| ...
        (isfinite(V(kk,29))&&V(kk,29)==0));
end
strong=valid(priority);weak=valid(~priority);
maxPairs=700;
if numel(strong)>maxPairs
    chosen=strong(unique(round(linspace(1,numel(strong),maxPairs))));
else
    budget=maxPairs-numel(strong);
    if numel(weak)>budget && budget>0
        weak=weak(unique(round(linspace(1,numel(weak),budget))));
    elseif budget==0
        weak=[];
    end
    chosen=[strong;weak];
end
chosen=unique(chosen);
PV=nan(numel(chosen),numel(pp));pe=cell(numel(chosen),1);
for j=1:numel(chosen)
    k=chosen(j);a=samples(k);b=samples(k+1);
    dt=b.t-a.t;PV(j,1:3)=[a.t,dt,1];
    PV(j,4)=~isequaln(a.parts,b.parts);
    PV(j,5)=max(abs(a.p-b.p))>1e-10;
    PV(j,6)=max(abs(a.u-b.u))>1e-10;
    PV(j,30)=a.sensorT-a.t;
    PV(j,31)=V(k,10);
    try
        P=xytest_parameters(a.p);G=xytest_geometry(P,cfg);
        xa=xytest_project_state(a.x);
        xb=xytest_project_state(b.x);
        E=CBF_REAL_XY_EVAL(xa,a.u,P,G,a.parts,lambda,Bcfg);
        H=E.hTaylor(:,1);
        psi1coef=(1:4)'.*H(2:5)+lambda(1)*H(1:4);
        tpol=dt.^(0:4).';
        p0=H'*tpol;
        p1=psi1coef.'*(dt.^(0:3)).';
        Eactual=CBF_REAL_XY_EVAL(xb,a.u,P,G,a.parts,lambda,Bcfg);
        PV(j,7)=Eactual.stages(1,1)-E.stages(1,1);
        PV(j,8)=p0-E.stages(1,1);
        PV(j,9)=Eactual.stages(2,1)-E.stages(2,1);
        PV(j,10)=p1-E.stages(2,1);
        xm=xytest_rk4(xa,a.u,P,dt,max(24,ceil(dt/0.001)));
        xm=xytest_project_state(xm);
        EM=CBF_REAL_XY_EVAL(xm,a.u,P,G,a.parts,lambda,Bcfg);
        PV(j,11)=EM.stages(2,1)-E.stages(2,1);
        PV(j,12)=Eactual.stages(2,1)-EM.stages(2,1);
        PV(j,13)=EM.stages(2,1)-p1;
        PV(j,14)=E.stages(3,1)-lambda(2)*E.stages(2,1);
        PV(j,15)=2*H(3)+lambda(1)*H(2);
        % Tiny positive time displacement: same x, obstacle and u.
        micro=1e-5;
        xx=xytest_rk4(xa,a.u,P,micro,4);
        xx=xytest_project_state(xx);
        EU=CBF_REAL_XY_EVAL(xx,a.u,P,G,a.parts,lambda,Bcfg);
        PV(j,16)=(EU.stages(2,1)-E.stages(2,1))/micro;
        PV(j,17)=PV(j,16)-PV(j,15);
        r=xm-xb;
        PV(j,18)=max(abs(r));
        PV(j,19:24)=[max(abs(r(1:4))),max(abs(r(5:7))), ...
            max(abs(r(8:10))),max(abs(r(11:13))), ...
            max(abs(r(14:16))),max(abs(r(17:19)))];
        if ~isequaln(a.parts,b.parts)
            Eobs=CBF_REAL_XY_EVAL(xb,a.u,P,G,b.parts,lambda,Bcfg);
            PV(j,25)=Eobs.stages(1,1)-Eactual.stages(1,1);
            PV(j,26)=Eobs.stages(2,1)-Eactual.stages(2,1);
        else
            PV(j,25:26)=0;
        end
        aLogged=xytest_member(a.info,'stageValues');
        bLogged=xytest_member(b.info,'stageValues');
        if isnumeric(aLogged)&&numel(aLogged)>=5 && ...
                isnumeric(bLogged)&&numel(bLogged)>=5
            PV(j,27)=double(bLogged(2,1)-aLogged(2,1));
            PV(j,28)=double(aLogged(2,1))-E.stages(2,1);
            % For changed obstacles, only logging comparison is meaningful.
            Eb=CBF_REAL_XY_EVAL(xb,b.u,xytest_parameters(b.p), ...
                xytest_geometry(xytest_parameters(b.p),cfg),b.parts,lambda,Bcfg);
            PV(j,29)=double(bLogged(2,1))-Eb.stages(2,1);
        end
        PV(j,32)=PV(j,12)-PV(j,13);
    catch ME
        pe{j}=[ME.identifier ' : ' ME.message];
        PV(j,3)=0;
    end
end
pairs=array2table(PV,'VariableNames',pp);
pairs.evalError=pe;
% All failures recorded, not merely most severe ones. Numeric thresholds
% are diagnostic heuristic flags, not formal safety certificates.
flag=(isfinite(V(:,21)) & V(:,21)>1e-7) | ...
    (isfinite(V(:,23)) & abs(V(:,23))>1e-6) | ...
    (isfinite(V(:,24)) & abs(V(:,24))>1e-6) | ...
    (isfinite(V(:,25)) & abs(V(:,25))>1e-6) | ...
    (isfinite(V(:,26)) & abs(V(:,26))>1e-6) | ...
    (isfinite(V(:,27)) & V(:,27)>1e-8) | ...
    (isfinite(V(:,28)) & V(:,28)>1e-8) | ...
    (isfinite(V(:,29)) & V(:,29)==0) | ...
    (isfinite(V(:,31)) & V(:,31)==0) | ...
    (isfinite(V(:,10)) & V(:,10)>0) | ...
    (isfinite(V(:,36)) & V(:,36)>0);
failures=T(flag,:);
stat=struct('records',n,'recomputed',sum(isfinite(V(:,12))), ...
    'validPairs',sum(PV(:,3)>0), ...
    'maxSameStateStageError',xytest_finite_max(V(:,21)), ...
    'maxSameStateAlgebraError',xytest_finite_max(max(abs(V(:,23:26)),[],2)), ...
    'maxMicroDerivativeError',xytest_finite_max(abs(PV(:,17))), ...
    'maxJetVsRK4StepError',xytest_finite_max(abs(PV(:,13))), ...
    'maxModelVsActualStepError',xytest_finite_max(abs(PV(:,12))), ...
    'maxStateStepError',xytest_finite_max(PV(:,18)), ...
    'infeasibleIndependent',sum(V(:,29)==0), ...
    'lowerStageViolations',sum(V(:,27)>1e-8), ...
    'finalStageViolations',sum(V(:,28)>1e-8), ...
    'loggedUncertified',sum(V(:,10)==1), ...
    'inputOutOfBounds',sum(V(:,31)==0), ...
    'geometryChangesInPairs',sum(PV(:,4)==1), ...
    'Psource',psource,'lambdaMismatch',lambdaMismatch, ...
    'samplingMedianS',typ,'maxSensorLagS',xytest_finite_max(abs(V(:,8))), ...
    'maxControllerQPDiscrepancy',xytest_finite_max(V(:,30)), ...
    'maxBodyAngleDeg',xytest_finite_max(V(:,32)), ...
    'maxCableAngleDeg',xytest_finite_max(V(:,33)), ...
    'bodyLimitCrossings',sum(V(:,32)>Bcfg.bodyMaxDeg+1e-9), ...
    'cableLimitCrossings',sum(V(:,33)>Bcfg.cableMaxDeg+1e-9), ...
    'minObstacleSeparationH',xyfull_finite_min(V(:,12)));
o=struct('available',true,'message', ...
    'All retained records audited at one instant, representative and failure-focused adjacent states audited.', ...
    'samples',T,'pairs',pairs,'failures',failures,'statistics',stat, ...
    'effectiveConfig',cfg,'lambda',lambda(:),'inputWeights',wt);
end

function yes=xyfull_box_feasible(B,bias,tol)
% Exact 2D polygon intersection using all box vertices and intersections.
% CBF inequalities are bias + B*z >= 0, z in [-1,1]^2.
B=double(B);bias=double(bias(:));
if size(B,2)~=2 || size(B,1)~=numel(bias)
    error('CBFXYTest:QPGeometry','Unexpected inequality dimensions');
end
C=[-B; eye(2);-eye(2)];d=[bias;1;1;1;1];
vertices=[-1,-1; -1,1;1,-1;1,1];
for i=1:size(C,1)
    for j=i+1:size(C,1)
        M=[C(i,:);C(j,:)];
        if abs(det(M))<=1e-12*max(1,norm(M,'fro')^2),continue;end
        z=M\[d(i);d(j)];
        vertices(end+1,:)=z(:).'; %#ok<AGROW>
    end
end
yes=any(all(bsxfun(@le,C*vertices.',d+tol),1));
end

function xyfull_summary(folder,report)
if ~isfield(report,'fullAudit'),return;end
f=fullfile(folder,'FULL_AUDIT_SUMMARY.txt');fid=fopen(f,'w');
if fid<0,return;end
cl=onCleanup(@()fclose(fid)); %#ok<NASGU>
a=report.fullAudit;
fprintf(fid,'CBF id=2 COMPREHENSIVE READ-ONLY AUDIT\nDate: %s\n',report.createdAt);
fprintf(fid,'Status: %s\n',a.message);
if ~a.available
    fprintf(fid,'INCOMPLETE: Required data unavailable; cannot claim correctness.\n');return;
end
st=a.statistics;
fprintf(fid,'\nRecords=%d, evaluations=%d, detailed pairs=%d\n', ...
    st.records,st.recomputed,st.validPairs);
fprintf(fid,'Exact historical P source: %s\n',st.Psource);
fprintf(fid,'Configured xyLambda vs supplied max difference=%.9g\n',st.lambdaMismatch);
fprintf(fid,'Typical control dt=%.9g s, max absolute sensor timestamp lag=%.9g s\n', ...
    st.samplingMedianS,st.maxSensorLagS);
fprintf(fid,'\nSAME-STATE CHECKS (no integration or state transition):\n');
fprintf(fid,'  max stage vs LOGGER absolute residual: %.12g\n',st.maxSameStateStageError);
fprintf(fid,'  max HOCBF algebraic identity error: %.12g\n',st.maxSameStateAlgebraError);
fprintf(fid,'  max controller vs independently reconstructed QP B,bias scaled error: %.12g\n',st.maxControllerQPDiscrepancy);
fprintf(fid,'\nSMALL-STEP / ZOH / REAL STATE CHECKS:\n');
fprintf(fid,'  max microstep RK4 derivative minus Taylor: %.12g\n',st.maxMicroDerivativeError);
fprintf(fid,'  max model RK4 vs fourth order Taylor psi1 step: %.12g\n',st.maxJetVsRK4StepError);
fprintf(fid,'  max estimated-state vs model RK4 psi1 step: %.12g\n',st.maxModelVsActualStepError);
fprintf(fid,'  max estimated-state vs model RK4 state difference: %.12g\n',st.maxStateStepError);
fprintf(fid,'\nQP / SAFETY INDICATORS (counts, not certificates):\n');
fprintf(fid,'  independently infeasible hard-QP samples: %d\n',st.infeasibleIndependent);
fprintf(fid,'  lower-stage violation samples: %d\n',st.lowerStageViolations);
fprintf(fid,'  final-stage violation samples: %d\n',st.finalStageViolations);
fprintf(fid,'  failed/uncertified samples: %d\n',st.loggedUncertified);
fprintf(fid,'  applied physical input out of bounds samples: %d\n',st.inputOutOfBounds);
fprintf(fid,'  geometry changes in sampled pairs: %d\n',st.geometryChangesInPairs);
fprintf(fid,'  failures marked in FULL_AUDIT_FAILURES.csv: %d\n',height(a.failures));
fprintf(fid,'  peak body tilt %.6g deg / peak cable tilt %.6g deg\n', ...
    st.maxBodyAngleDeg,st.maxCableAngleDeg);
fprintf(fid,'  angle-limit exceedance samples body=%d cable=%d\n', ...
    st.bodyLimitCrossings,st.cableLimitCrossings);
fprintf(fid,'  min observed obstacle h=%.9g m\n',st.minObstacleSeparationH);
fprintf(fid,'\nCRITICAL INTERPRETATION:\n');
fprintf(fid,['1. Same-state algebra contradictions suggest logging, gain mismatch, ', ...
    'evaluator, or data alignment faults before blaming physical propagation.\n']);
fprintf(fid,['2. Microstep discrepancy with valid same-state algebra isolates ', ...
    'differentiation of the original model.\n']);
fprintf(fid,['3. Full-step state discrepancy with accurate microstep may ', ...
    'reflect estimator corrections, plant-model mismatch, input timing, or parameters.\n']);
fprintf(fid,['4. Current agent.parameter substitution is NOT proof of historical ', ...
    'P; output identifies this provenance.\n']);
fprintf(fid,['5. Only observed obstacle cable ellipsoid + body/cable tilt ', ...
    'are audited: this does NOT verify body/payload/ground collision.\n']);
fprintf(fid,['6. Continuous-time HOCBF QP feasibility does NOT certify sampled ', ...
    'ZOH or between-sample safety.\n']);
fprintf(fid,['7. continueBounded is UNCERTIFIED, not hard safety even if angles ', ...
    'and separation happened to remain within limits.\n']);
if ~isempty(a.pairs)
    [~,ix]=min(abs(a.pairs.t-25.05));
    r=a.pairs(ix,:);
    fprintf(fid,'\nNEAR 25.050 s (t=%.6f):\n',r.t);
    fprintf(fid,'psi1 derivative: HOCBF stages=%.10g, Taylor=%.10g, 10us microRK4=%.10g\n', ...
        r.psi1DerivativeFromStages,r.psi1DerivativeFromTaylor,r.psi1DerivativeMicroRK4);
    fprintf(fid,'psi1 change: observed=%.10g, jet=%.10g, RK4=%.10g\n', ...
        r.psi1StepMeasured,r.psi1StepPredictedFromJet,r.psi1StepPredictedFromRK4);
    fprintf(fid,'state groups: q=%.5g, w=%.5g, pL=%.5g, vL=%.5g, pT=%.5g, wL=%.5g\n', ...
        r.qStateInfResidual,r.omegaBodyStateInfResidual,r.pLoadStateInfResidual, ...
        r.vLoadStateInfResidual,r.pCableStateInfResidual,r.omegaCableStateInfResidual);
end
end

function v=xyfull_finite_min(x)
x=x(isfinite(x));
if isempty(x),v=NaN;else,v=min(x);end
end
