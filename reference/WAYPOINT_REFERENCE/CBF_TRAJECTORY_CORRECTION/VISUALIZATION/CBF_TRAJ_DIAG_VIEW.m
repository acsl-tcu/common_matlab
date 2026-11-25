function out=CBF_TRAJ_DIAG_VIEW(action,varargin)
% CBF_TRAJ_DIAG_VIEW  Optional, Sim-only trajectory-correction diagnostics.
%   line=CBF_TRAJ_DIAG_VIEW('display',agent,time,baseLine,enabled)
%   CBF_TRAJ_DIAG_VIEW('plot',logger,figureNo)
%
% This is a READ-ONLY view of reference.result.cbfTraj. It does not call
% HLC, CBF, the plant, or the waypoint generator. No persistent history:
% figures use the already-saved LOGGER reference/estimator snapshots.
% The baseLine may already contain CBF_REAL_DIAG_VIEW output.
% Figures are created ONLY if a non-NOMINAL trajectory-correction mode
% has occurred during the logged flight.

out=[];
switch upper(char(action))
    case 'DISPLAY'
        if numel(varargin)~=4
            error('CBFTrajView:DisplayArgs', ...
                'Expected (agent,time,baseLine,enabled).');
        end
        out=display_line(varargin{:});
    case 'PLOT'
        if isempty(varargin)
            error('CBFTrajView:PlotArgs','LOGGER is required.');
        end
        fig=30;
        if numel(varargin)>=2 && ~isempty(varargin{2}),fig=varargin{2};end
        plot_history(varargin{1},fig);
    otherwise
        error('CBFTrajView:Action','Unsupported action: %s',char(action));
end
end

function line=display_line(agent,~,baseLine,enabled)
line=baseLine;
if ~enabled || isempty(baseLine) || isempty(agent)
    return
end
if ~strcmp(char(agent(1).cha),'f')
    return
end
rr=agent(1).reference.result;
if ~isstruct(rr) || ~isfield(rr,'cbfTraj') || ...
        ~isstruct(rr.cbfTraj)
    return
end
r=rr.cbfTraj;
required={'mode','before28','after28','signedCarryPosition', ...
    'signedCarryVelocity','lastIntervalDeltaPV','sourceStatus'};
if ~all(isfield(r,required)) || strcmpi(char(r.mode),'NOMINAL')
    % First CBF correction is applied to the target from the NEXT cycle.
    % Before then no extra console output is generated.
    return
end
if numel(r.before28)~=28 || numel(r.after28)~=28 || ...
        numel(r.signedCarryPosition)~=3 || ...
        numel(r.signedCarryVelocity)~=3 || ...
        numel(r.lastIntervalDeltaPV)~=6
    return
end
raw=r.before28(:);new=r.after28(:);
d=r.signedCarryPosition(:);w=r.signedCarryVelocity(:);
delta=r.lastIntervalDeltaPV(:);
line=[char(line),sprintf([ ...
    '\n  TRAJ %s src=%s | SLOADpre=[%+.3f,%+.3f,%+.3f]' ...
    ' SLOADpost=[%+.3f,%+.3f,%+.3f]' ...
    ' | carryP=[%+.3g,%+.3g,%+.3g]' ...
    ' carryV=[%+.3g,%+.3g,%+.3g]' ...
    ' | prevDeltaP=[%+.3g,%+.3g,%+.3g]' ...
    ' prevDeltaV=[%+.3g,%+.3g,%+.3g]'], ...
    char(r.mode),char(r.sourceStatus),raw(1:3),new(1:3),d,w, ...
    delta(1:3),delta(4:6))];
if isfield(r,'cbfId') && isnumeric(r.cbfId) && r.cbfId==3 && ...
        isfield(r,'physicalPrevious') && isstruct(r.physicalPrevious)
    h=r.physicalPrevious;
    required={'valid','T0','Tdot0','TddotApplied', ...
        'TddotBaseline','TendApplied','TendBaseline'};
    if all(isfield(h,required)) && isscalar(h.valid) && logical(h.valid)
        vals=[h.T0,h.Tdot0,h.TddotApplied,h.TddotBaseline, ...
              h.TendApplied,h.TendBaseline];
        if isnumeric(vals) && numel(vals)==6 && all(isfinite(vals))
            line=[line,sprintf(['\n  TRAJ prev ID3: T0=%+.4f N' ...
                ' Tdot0=%+.4f N/s Tddot(CBF/base)=[%+.4f,%+.4f] N/s^2' ...
                ' Tend(CBF/base)=[%+.4f,%+.4f] N'],vals)];
        end
    end
end
end

function plot_history(logger,figNum)
if ~isprop(logger,'Data') || ~isfield(logger.Data,'agent') || ...
        isempty(logger.Data.agent) || ...
        ~isfield(logger.Data.agent(1),'reference') || ...
        ~isfield(logger.Data.agent(1).reference,'result')
    return
end
ref=logger.Data.agent(1).reference.result;
tAll=double(logger.Data.t(:));
n=min([double(logger.k),numel(ref),numel(tAll)]);
if n<1,return;end
pre=nan(n,3);post=nan(n,3);loadPos=nan(n,3);
carryP=nan(n,3);carryV=nan(n,3);appliedNu=nan(n,1);
baseNu=nan(n,1);T0=nan(n,1);q0=nan(n,1);
TaEnd=nan(n,1);TbEnd=nan(n,1);qaEnd=nan(n,1);qbEnd=nan(n,1);
modeId=nan(n,1);previousActive=nan(n,1);valid=false(n,1);
if isfield(logger.Data.agent(1),'estimator') && ...
        isfield(logger.Data.agent(1).estimator,'result')
    est=logger.Data.agent(1).estimator.result;
else
    est={};
end
for k=1:n
    e=ref{k};
    if ~isstruct(e) || ~isfield(e,'cbfTraj') || ~isstruct(e.cbfTraj)
        continue
    end
    r=e.cbfTraj;
    req={'before28','after28','signedCarryPosition', ...
        'signedCarryVelocity','mode'};
    if ~all(isfield(r,req)) || numel(r.before28)~=28 || ...
            numel(r.after28)~=28 || ...
            numel(r.signedCarryPosition)~=3 || ...
            numel(r.signedCarryVelocity)~=3
        continue
    end
    valid(k)=true;
    pre(k,:)=double(r.before28(1:3)).';
    post(k,:)=double(r.after28(1:3)).';
    carryP(k,:)=double(r.signedCarryPosition(:)).';
    carryV(k,:)=double(r.signedCarryVelocity(:)).';
    switch upper(char(r.mode))
        case 'NOMINAL',modeId(k)=0;
        case 'CARRY',modeId(k)=1;
        case 'RETURN',modeId(k)=2;
    end
    if k<=numel(est) && isstruct(est{k}) && isfield(est{k},'state')
        st=est{k}.state;
        if isobject(st) && isprop(st,'pL')
            p=st.pL;
            if isnumeric(p) && numel(p)==3 && all(isfinite(p(:)))
                loadPos(k,:)=double(p(:)).';
            end
        end
    end
    if isfield(r,'physicalPrevious') && isstruct(r.physicalPrevious)
        h=r.physicalPrevious;
        if isfield(h,'active') && isscalar(h.active)
            previousActive(k)=double(logical(h.active));
        end
        if isfield(r,'cbfId') && isnumeric(r.cbfId) && r.cbfId==3 && ...
                isfield(h,'valid') && logical(h.valid)
            fields={'T0','Tdot0','TddotApplied','TddotBaseline', ...
                'TendApplied','TendBaseline','TdotEndApplied','TdotEndBaseline'};
            if all(isfield(h,fields))
                v=[h.T0,h.Tdot0,h.TddotApplied,h.TddotBaseline, ...
                    h.TendApplied,h.TendBaseline,h.TdotEndApplied,h.TdotEndBaseline];
                if isnumeric(v) && numel(v)==8 && all(isfinite(v))
                    T0(k)=v(1);q0(k)=v(2);appliedNu(k)=v(3);
                    baseNu(k)=v(4);TaEnd(k)=v(5);TbEnd(k)=v(6);
                    qaEnd(k)=v(7);qbEnd(k)=v(8);
                end
            end
        end
    end
end
% Even if the feature was enabled, no trajectory figures unless the CBF
% has *actually triggered* a CARRY or RETURN mode.
if ~any(valid & (modeId==1 | modeId==2))
    return
end
t=tAll(1:n);
fig=figure(figNum);clf(fig);
tiledlayout(fig,3,1,'TileSpacing','compact');
axisNames={'X','Y','Z'};
for j=1:3
    nexttile;
    plot(t,pre(:,j),'--','LineWidth',1);hold on;
    plot(t,post(:,j),'-','LineWidth',1.1);
    if any(isfinite(loadPos(:,j)))
        plot(t,loadPos(:,j),':','LineWidth',1);
        legend('SLOAD before','CBF trajectory after','estimated load','Location','best');
    else
        legend('SLOAD before','CBF trajectory after','Location','best');
    end
    ylabel([axisNames{j},' [m]']);grid on;hold off;
end
xlabel('time [s]');sgtitle('CBF trajectory correction: target before / after');

fig=figure(figNum+1);clf(fig);
tiledlayout(fig,3,1,'TileSpacing','compact');
nexttile;
plot(t,carryP,'LineWidth',1.05);grid on;
ylabel('carry position [m]');legend('X','Y','Z','Location','best');
nexttile;
plot(t,carryV,'LineWidth',1.05);grid on;
ylabel('carry velocity [m/s]');legend('X','Y','Z','Location','best');
nexttile;
stairs(t,modeId,'LineWidth',1.1);hold on;
plot(t,2.2*previousActive,':');hold off;grid on;
ylim([-0.2,2.5]);yticks([0 1 2]);yticklabels({'NOMINAL','CARRY','RETURN'});
ylabel('mode');xlabel('time [s]');
legend('target mode','previous CBF physical correction','Location','best');
sgtitle('CBF trajectory correction: accumulated state and mode');

% ID3-only diagnostic plot. Values at t(k) describe the PREVIOUS 25ms
% actuation interval, not the current instant's commanded actuation.
if any(isfinite(appliedNu))
    fig=figure(figNum+2);clf(fig);
    tiledlayout(fig,3,1,'TileSpacing','compact');
    nexttile;
    plot(t,appliedNu,'-');hold on;plot(t,baseNu,'--');hold off;
    ylabel('Tddot [N/s^2]');grid on;
    legend('CBF previous','bounded nominal previous','Location','best');
    nexttile;
    plot(t,TaEnd,'-');hold on;plot(t,TbEnd,'--');plot(t,T0,':');hold off;
    ylabel('thrust [N]');grid on;
    legend('Tend CBF','Tend bounded nominal','T0','Location','best');
    nexttile;
    plot(t,qaEnd,'-');hold on;plot(t,qbEnd,'--');plot(t,q0,':');hold off;
    ylabel('Tdot [N/s]');xlabel('time [s]');grid on;
    legend('TdotEnd CBF','TdotEnd bounded nominal','Tdot0','Location','best');
    sgtitle('ID3 thrust extension: previous actuation interval');
end
end
