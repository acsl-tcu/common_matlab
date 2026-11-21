function resultZip = CBF_FORWARD_INVARIANCE_TEST(varargin)
% CBF_FORWARD_INVARIANCE_TEST  Recover CBF history without relying on app.
% Run AFTER simulation and BEFORE clearing functions/figures:
%     CBF_FORWARD_INVARIANCE_TEST
%
% One MATLAB entrypoint. The ONLY persistent output is
% CBF_FORWARD_INVARIANCE_RESULTS.zip, in this file's folder.
%
% Sources, in priority order:
%  (1) Existing diagnostic Figure 21 (DO NOT overwrite before inspection).
%  (2) Existing CBF_REAL_DIAG_VIEW persistent trace, replayed through its
%      existing 'plot' interface with a synthetic LOGGER-shaped struct.
%  (3) An accessible app.logger (only when isvalid(app) is true).
%
% This is a diagnostic audit, NOT a rigorous invariance certificate.
% No CBF controller/diagnostic function is modified or cleared.

% Two uses of this SAME test file:
%   CBF_FORWARD_INVARIANCE_TEST                  % existing workspace/plots
%   CBF_FORWARD_INVARIANCE_TEST('capture',logger) % at simulation post()
% The second form snapshots LOGGER.Data while the GUI handle is still valid.
liveLogger=[];
if nargin>0
    if nargin~=2 || ~strcmpi(char(varargin{1}),'capture')
        error('CBF:Usage','Use TEST() or TEST(''capture'',app.logger) inside post().');
    end
    liveLogger=varargin{2};
end

root=fileparts(mfilename('fullpath'));
resultZip=fullfile(root,'CBF_FORWARD_INVARIANCE_RESULTS.zip');
tmpDir=tempname(tempdir); mkdir(tmpDir);
cleanupDir=onCleanup(@() cleanLocal(tmpDir)); %#ok<NASGU>
notes={};
rawD=[];
notes{end+1,1}=['Generated ', datestr(now,31)];

% 1) Inspect actual base workspace first, never assume valid app handle.
[baseInfo, appLogger, appNotes] = inspectBaseWorkspace(tmpDir);
if ~isempty(liveLogger)
    appLogger=liveLogger;
    notes{end+1,1}='LIVE LOGGER supplied from simulation post() before GUI teardown.';
end
if ~isempty(appLogger)
    try
        D = getDataFromLogger(appLogger);
        rawD=D;
        if ~isempty(D)
            save(fullfile(tmpDir,'LOGGER_DATA_RAW.mat'),'D','-v7.3');
            notes{end+1,1}='LOGGER_DATA_RAW.mat saved as a raw snapshot.';
        end
    catch ME
        notes{end+1,1}=['Could not save LOGGER.Data: ',errtext(ME)];
    end
end
notes=[notes;appNotes(:)];

% In capture mode ensure Figure21 belongs to THIS flight, not a previous
% flight's still-open graph. This does not alter the controller.
if ~isempty(liveLogger)
    try
        CBF_REAL_DIAG_VIEW('plot',liveLogger,20);
        notes{end+1,1}='Refreshed CBF diagnosis figures from LIVE logger before reading Figure21.';
    catch ME
        notes{end+1,1}=['Live CBF diagnostic plot failed: ',errtext(ME)];
    end
end

% 2) FIRST recover existing Figure 21, avoiding destructive re-plotting.
[stages,figureNotes]=getStageFigure();
notes=[notes;figureNotes(:)];
source='EXISTING_FIGURE_21';
if ~isempty(liveLogger),source='CURRENT_FLIGHT_FIGURE_21';end

% Also inspect raw LOGGER.Data independently of figure/persistent state.
% This is a fallback for cases where diagnostics were never drawn.
if isempty(stages) && ~isempty(rawD)
    try
        stages=rawLoggerStages(rawD);
        if ~isempty(stages)
            source='RAW_LOGGER_DATA';
            notes{end+1,1}=sprintf('Recovered %d stage values directly from LOGGER.Data.',height(stages));
        end
    catch ME
        notes{end+1,1}=['Direct LOGGER.Data extraction failed: ',errtext(ME)];
    end
end

% 3) If Figure 21 not present, try available app logger (if still valid).
if isempty(stages) && ~isempty(appLogger)
    try
        CBF_REAL_DIAG_VIEW('plot',appLogger,20);
        [stages,figureNotes]=getStageFigure();
        notes=[notes;{'Existing valid app.logger passed to CBF_REAL_DIAG_VIEW.'};figureNotes(:)];
        source='VALID_APP_LOGGER_PLOT';
    catch ME
        notes{end+1,1}=['Valid app.logger plot failed: ',errtext(ME)];
    end
end

% 4) Last resort: live trace is held in CBF_REAL_DIAG_VIEW persistent vars.
%    DO NOT edit/clear/reload this function before this recovery attempt.
%    Supply the LOGGER-shaped data required by its normal plot interface.
%    No historical app object is needed for this display-only replay.
if isempty(stages)
    try
        fn=which('CBF_REAL_DIAG_VIEW');
        if isempty(fn)
            notes{end+1,1}='CBF_REAL_DIAG_VIEW not found on MATLAB path.';
        else
            notes{end+1,1}=['CBF_REAL_DIAG_VIEW: ',fn];
            syntheticLogger=makeSyntheticLogger(0,0.025,50);
            CBF_REAL_DIAG_VIEW('plot',syntheticLogger,20);
            [stages,figureNotes]=getStageFigure();
            notes=[notes;{'Synthetic LOGGER-shaped replay attempted (t=0:0.025:50).'};figureNotes(:)];
            source='PERSISTENT_TRACE_REPLAY';
        end
    catch ME
        notes{end+1,1}=['Persistent trace replay failed: ',errtext(ME)];
    end
end

if isempty(stages)
    source='NOT_AVAILABLE';
end

% 5) Extract all remaining visible time-series from Figures 20,22,23.
%    This is diagnostic plot data, not an assumed 19-state trajectory.
[figRows,figNotes] = getFigureSeries([20,22,23]);
notes=[notes;figNotes(:)];

% 6) Report evidence and limits explicitly.
if ~isempty(stages)
    writetable(stages,fullfile(tmpDir,'HOCBF_STAGE_HISTORY.csv'));
    [violations,stats]=evaluateStages(stages);
    writetable(violations,fullfile(tmpDir,'HOCBF_VIOLATIONS.csv'));
    writetable(stats,fullfile(tmpDir,'HOCBF_STAGE_SUMMARY.csv'));
else
    notes{end+1,1}='NO HOCBF stage samples could be recovered.';
    notes{end+1,1}=['A deleted SimExp app cannot expose logger/agent even if its ', ...
        'base-workspace variable is still named app.'];
    notes{end+1,1}=['If diagnostic figures and the diagnostic function persistent ', ...
        'trace were cleared, complete past states cannot be recovered from app.'];
end
if ~isempty(figRows)
    writetable(figRows,fullfile(tmpDir,'OTHER_DIAGNOSTIC_FIGURES.csv'));
end
if ~isempty(baseInfo)
    writetable(baseInfo,fullfile(tmpDir,'BASE_WORKSPACE_STATUS.csv'));
end

summary=sprintf(['CBF FORWARD INVARIANCE / DATA RECOVERY REPORT\n', ...
    'History source: %s\n', ...
    'HOCBF stage rows: %d\n', ...
    'Other figure-series rows: %d\n', ...
    'Base variables inventoried: %d\n', ...
    '\nLIMITATIONS:\n', ...
    '- Graph traces are sampled diagnostic results, not a certified ZOH interval bound.\n', ...
    '- A deleted MATLAB GUI app handle does not give access to its former logger.\n', ...
    '- These data alone cannot prove forward invariance for all t.\n', ...
    '- Thrust and all torques are constant *within* each ZOH step and change\n', ...
    '  normally at its 25 ms boundary.\n', ...
    '- Without saved full state/input, strict model-vs-plant replay is unavailable.\n', ...
    '- Deleted MATLAB handle internals cannot be recovered by variable-name searches.\n'],...
    source,height(stages),height(figRows),height(baseInfo));
writeUtf8(fullfile(tmpDir,'SUMMARY.txt'),summary);
writeUtf8(fullfile(tmpDir,'RECOVERY_DIAGNOSTICS.txt'),strjoin(notes,newline));
writeUtf8(fullfile(tmpDir,'HANDOFF_JA.txt'),handoffText(source,height(stages),height(figRows)));

files=dir(tmpDir); names={};
for i=1:numel(files)
    if ~files(i).isdir || strcmp(files(i).name,'BASE_RAW')
        names{end+1}=files(i).name;
    end %#ok<AGROW>
end
zipTemp=[tempname(tempdir),'.zip'];
cleanupZip=onCleanup(@() cleanLocal(zipTemp)); %#ok<NASGU>
zip(zipTemp,names,tmpDir);
if exist(resultZip,'file')==2,delete(resultZip);end
[ok,msg]=movefile(zipTemp,resultZip,'f');
if ~ok,error('CBF:ZipWrite','ZIP move failed: %s',msg);end
fprintf('[CBF INVARIANCE] Source=%s, HOCBF stage rows=%d, extra graph rows=%d\n',...
    source,height(stages),height(figRows));
if isempty(stages)
    fprintf(2,['[CBF INVARIANCE] No stage traces remain in accessible workspace/figures/persistent diagnostics.\n', ...
        '[CBF INVARIANCE] A deleted handle is not recoverable by changing variable names.\n']);
else
    fprintf('[CBF INVARIANCE] Stage data recovered; NOT a proof of continuous-time safety.\n');
end
fprintf('[CBF INVARIANCE] Result ZIP: %s\n',resultZip);
end

function [T,logger,notes]=inspectBaseWorkspace(folder)
% Enumerate ALL base variables. No assumed variable names or object class.
T=table('Size',[0,5],...
    'VariableTypes',{'string','string','string','logical','string'},...
    'VariableNames',{'Name','Class','SizeText','IsValidHandle','Note'});
logger=[]; notes={};
try
    W=evalin('base','whos');
catch ME
    notes{end+1,1}=['Cannot enumerate base workspace: ',errtext(ME)];return;
end
root=fullfile(folder,'BASE_RAW'); mkdir(root);
for i=1:numel(W)
    nm=W(i).name; valid=true; reason=''; x=[];
    try
        x=evalin('base',nm);
        if isa(x,'handle')
            valid=all(isvalid(x(:)));
            if ~valid,reason='DELETED_OR_INVALID_HANDLE';end
        end
        % Save every existing base variable independently. A successful
        % save of a deleted handle is NOT its data being restored.
        try
            value=x; %#ok<NASGU>
            save(fullfile(root,sprintf('BASE_%04d.mat',i)),'value','-v7.3');
        catch ME
            reason=[reason,' RAW_SAVE_FAILED ',errtext(ME)];
        end
        if valid && isempty(logger)
            [found,foundPath]=searchForLogger(x,nm,0,300);
            if ~isempty(found)
                logger=found;
                notes{end+1,1}=['Logger discovered without name assumption: ',foundPath];
            end
        end
    catch ME
        valid=false; reason=errtext(ME);
    end
    T(end+1,:)={string(nm),string(W(i).class),string(mat2str(W(i).size)),valid,string(reason)}; %#ok<AGROW>
    clear x value
end
notes{end+1,1}=sprintf('Inspected ALL %d base variables independently.',numel(W));
if isempty(logger)
    notes{end+1,1}='No readable LOGGER discovered in base workspace (all names/classes inspected).';
end
end

function [L,pathOut]=searchForLogger(x,pathIn,depth,budget)
L=[];pathOut='';
if depth>3 || budget<=0 || isempty(x),return;end
try
    if isa(x,'handle') && ~all(isvalid(x(:))),return;end
catch
    return;
end
try
    D = getDataFromLogger(x);
    if ~isempty(D)
        L=x;pathOut=pathIn;return;
    end
catch
end
% A bare LOGGER.Data struct counts too (t and agent arrays).
try
    if isstruct(x) && isscalar(x) && isfield(x,'t') && isfield(x,'agent')
        L=struct('Data',x,'k',numel(x.t));pathOut=pathIn;return;
    end
catch
end
if numel(x)>1,return;end
try
    if isstruct(x),keys=fieldnames(x);
    elseif isobject(x),keys=properties(x);
    else,keys={};end
catch
    keys={};
end
for i=1:min(numel(keys),budget)
    key=keys{i};
    try
        child=x.(key);
        [L,pathOut]=searchForLogger(child,[pathIn,'.',key],depth+1,budget-i);
        if ~isempty(L),return;end
    catch
        % Do not invent deleted object properties; carry on scanning.
    end
end
end

function D=getDataFromLogger(logger)
D=[];
if isempty(logger),return;end
if isa(logger,'handle') && ~all(isvalid(logger(:))),return;end
if isobject(logger)
    if ~ismember('Data',properties(logger)),return;end
    D=logger.Data;
elseif isstruct(logger) && isscalar(logger) && isfield(logger,'Data')
    D=logger.Data;
else
    return;
end
if ~isstruct(D) || ~isscalar(D) || ~isfield(D,'t') || ~isfield(D,'agent')
    D=[];
end
end

function T=rawLoggerStages(D)
% Raw LOGGER.Data history: D.agent(1).controller.result(k).cbf
% This logic is based on the existing CBF_REAL_DIAG_VIEW extraction path.
T=stageEmpty();
try
    ag=D.agent;
    if iscell(ag),ag=ag{1};else,ag=ag(1);end
    C=ag.controller.result;
    tt=double(D.t(:));
catch
    return;
end
n=min(numel(tt),numel(C));
for k=1:n
    try
        if iscell(C),rec=C{k};else,rec=C(k);end
        if ~isstruct(rec)||~isfield(rec,'cbf'),continue;end
        info=rec.cbf;
        if ~isstruct(info)||~isfield(info,'stageValues'),continue;end
        A=info.stageValues;
        if ~isnumeric(A)||size(A,1)<5,continue;end
        if isfield(info,'constraintNames') && iscell(info.constraintNames)
            labels=info.constraintNames;
        else
            labels=arrayfun(@(j)sprintf('constraint_%d',j),1:size(A,2),'UniformOutput',false);
        end
        for j=1:min(numel(labels),size(A,2))
            for order=0:4
                x=double(A(order+1,j));
                if ~isfinite(x),continue;end
                T(end+1,:)={string(labels{j}),double(order),tt(k),x}; %#ok<AGROW>
            end
        end
    catch
        % No conjectured values for incomplete cycles.
    end
end
end

function L=makeSyntheticLogger(t0,dt,te)
% Match the *existing* plot_history(logger, ...) interface of current
% CBF_REAL_DIAG_VIEW. It reads Data.t, Data.agent.controller.result,
% Data.agent.sensor.result and k, then uses its own persistent trace.
t=(t0:dt:te).'; n=numel(t);
C=repmat(struct('cbf',[]),n,1);
S=repmat(struct('cbfSensorReport',[]),n,1);
A=struct('controller',struct('result',C),...
         'sensor',struct('result',S));
L=struct('Data',struct('t',t,'agent',A),'k',n);
end

function [T,notes]=getStageFigure()
T=stageEmpty();notes={};
try
    figs=findall(groot,'Type','figure');
catch ME
    notes{end+1,1}=['Cannot enumerate open figures: ',errtext(ME)];return;
end
for i=1:numel(figs)
    fig=figs(i);
    try
        nm=char(get(fig,'Name'));number=get(fig,'Number');
        if ~isequal(number,21) && ~contains(nm,'every HOCBF stage','IgnoreCase',true)
            continue;
        end
        axes_=findall(fig,'Type','axes');
        nAdded=0;
        for a=1:numel(axes_)
            ax=axes_(a);
            try
                titleText=get(get(ax,'Title'),'String');
                if iscell(titleText),titleText=strjoin(titleText,' ');end
                titleText=char(string(titleText));
                token=regexp(strtrim(titleText),'^([0-4])\s*:', 'tokens','once');
                if isempty(token),continue;end
                level=str2double(token{1});
                lines=findall(ax,'Type','line');
                for k=1:numel(lines)
                    ln=lines(k);
                    label=char(string(get(ln,'DisplayName')));
                    if isempty(strtrim(label)) || strcmp(label,'data1')
                        continue;
                    end
                    x=get(ln,'XData');y=get(ln,'YData');
                    if ~isnumeric(x)||~isnumeric(y)||numel(x)~=numel(y),continue;end
                    x=x(:);y=y(:);
                    good=isfinite(x)&isfinite(y);
                    x=x(good);y=y(good);
                    if isempty(x),continue;end
                    block=table(repmat(string(label),numel(x),1),...
                        repmat(level,numel(x),1),double(x),double(y),...
                        'VariableNames',{'Constraint','StageOrder','Time_s','Value'});
                    T=[T;block]; %#ok<AGROW>
                    nAdded=nAdded+height(block);
                end
            catch ME
                notes{end+1,1}=['Axis parsing error: ',errtext(ME)];
            end
        end
        notes{end+1,1}=sprintf('Figure %g "%s": %d stage samples',number,nm,nAdded);
    catch ME
        notes{end+1,1}=['Figure parsing error: ',errtext(ME)];
    end
end
if ~isempty(T)
    T=sortrows(T,{'Constraint','StageOrder','Time_s'});
end
end

function T=stageEmpty()
T=table('Size',[0,4],...
 'VariableTypes',{'string','double','double','double'},...
 'VariableNames',{'Constraint','StageOrder','Time_s','Value'});
end

function [T,notes]=getFigureSeries(numbers)
T=table('Size',[0,5],...
 'VariableTypes',{'double','string','string','double','double'},...
 'VariableNames',{'Figure','AxesTitle','Series','Time_s','Value'});
notes={};
try, figs=findall(groot,'Type','figure');
catch ME,notes{end+1,1}=errtext(ME);return;end
for i=1:numel(figs)
    try
        f=figs(i);n=get(f,'Number');
        if ~ismember(n,numbers),continue;end
        aa=findall(f,'Type','axes');
        count=0;
        for j=1:numel(aa)
            ax=aa(j);
            tt=get(get(ax,'Title'),'String');
            if iscell(tt),tt=strjoin(tt,' ');end
            tit=string(tt);
            ll=findall(ax,'Type','line');
            for k=1:numel(ll)
                try
                    x=get(ll(k),'XData');y=get(ll(k),'YData');
                    if ~isnumeric(x)||~isnumeric(y)||numel(x)~=numel(y),continue;end
                    x=x(:);y=y(:);g=isfinite(x)&isfinite(y);
                    x=x(g);y=y(g);if isempty(x),continue;end
                    name=string(get(ll(k),'DisplayName'));
                    if strlength(name)==0,name="unnamed";end
                    b=table(repmat(double(n),numel(x),1),...
                        repmat(tit,numel(x),1),repmat(name,numel(x),1),...
                        double(x),double(y),'VariableNames',T.Properties.VariableNames);
                    T=[T;b]; %#ok<AGROW>
                    count=count+height(b);
                catch
                end
            end
        end
        notes{end+1,1}=sprintf('Figure %g captured %d series samples',n,count);
    catch ME
        notes{end+1,1}=errtext(ME);
    end
end
end

function [V,S]=evaluateStages(T)
V=table('Size',[0,4],...
 'VariableTypes',{'string','double','double','double'},...
 'VariableNames',{'Constraint','StageOrder','Time_s','Value'});
S=table('Size',[0,6],...
 'VariableTypes',{'string','double','double','double','double','double'},...
 'VariableNames',{'Constraint','StageOrder','Count','NegativeCount','Minimum','FirstNegativeTime_s'});
[C,~,g]=unique(strcat(T.Constraint,"__",string(T.StageOrder)));
for i=1:numel(C)
    seg=T(g==i,:);
    neg=seg.Value < -1e-8;
    if any(neg)
        V=[V;seg(neg,:)]; %#ok<AGROW>
        firstNeg=min(seg.Time_s(neg));
    else
        firstNeg=NaN;
    end
    S(end+1,:)={seg.Constraint(1),seg.StageOrder(1),...
        height(seg),sum(neg),min(seg.Value),firstNeg}; %#ok<AGROW>
end
end

function writeUtf8(filename,s)
f=fopen(filename,'w','n','UTF-8');
if f<0,error('CBF:Write','Cannot write file: %s',filename);end
cc=onCleanup(@() fclose(f)); %#ok<NASGU>
fprintf(f,'%s\n',char(s));
end

function s=handoffText(source,stageCount,figCount)
s=sprintf(['CBF前方不変性 調査・引継ぎ 2026-11-21\n',...
    '今回のZIP manifestから確定した原因：全41件のアクセス失敗は、\n',...
    'app (SimExp) が削除済み MATLAB handle であるため。\n',...
    'app の存在は app.logger の有効性を意味しない。保存したapp MATから\n',...
    '完整な LOGGER や 19 状態を回収できると断定しない。\n',...
    '今回はまず既存 Figure21、次に CBF_REAL_DIAG_VIEW の persistent trace を\n',...
    '元の plot インターフェースと synthetic LOGGER で回収する。\n',...
    '本テストは CBF_REAL_DIAG_VIEW を変更/clear しない。\n',...
    '取得元=%s、ステージサンプル=%d、他グラフサンプル=%d。\n',...
    'CBF: 質量初期値0.01/0.14kgの2条件。良好な0.14kgでも下位 HOCBF 負値あり。\n',...
    'Tおよび全トルクは25ms ZOH区間内一定。次周期に制御則が普通に更新する。\n',...
    '現在のZOH Taylor近似ガードは余項誤差上界を持たず厳密保証なし。\n',...
    'サンプル間の安全性、将来の制約両立/再帰可行性を別途証明する。\n',...
    '全障害物/機体/紐、将来は地面/吊り荷/ソフト制約、入力増加を考慮。\n',...
    '既存の分離マージン増加と推定軌道再設計は現在の優先度ではない。\n',...
    '一つのテスト実行入口、結果ZIP一個。\n'],source,stageCount,figCount);
end

function s=errtext(ME)
s=[char(ME.identifier),' ',char(ME.message)];
end
function cleanLocal(p)
if exist(p,'dir')==7
    try,rmdir(p,'s');catch,end
elseif exist(p,'file')==2
    try,delete(p);catch,end
end
end
