function [R,S]=wp_six_trigger_tick_v8(agent,S,K,routeCache,known,cfg,flightTime)
%WP_SIX_TRIGGER_TICK_V8 Current EKF -> rolling PV history -> 4s -> 1 mode.
% Call AFTER estimator update and AFTER original reference update, exactly
% once per 25ms flight tick. S is returned to caller; no hidden persistent
% state, no modification of agent, HLC, EKF, parameter or reference.
%
% Exact source interfaces expected:
%  agent.estimator.result.state.pL / vL / mL (CURRENT EKF)
%  agent.reference.result.state.xd (CURRENT original 28 jet)
%  agent.reference.waypoint.virtualTime/rateState/heldRateCommand
%  agent.reference.waypoint.slowdownStart/slowdownFull/minScale/filterTime
%  S starts from empty: S=struct(); requires 41 consecutive real samples.
% Missing/repeated times reset the history; NO fake history or velocity.
% No-available-obstacle still updates the historical buffer for later use.
% Returns a NOMINAL/noncertified status; NEVER a physical-safe verdict.
start=tic;
R=struct('status','UNKNOWN','reason','', 'safeToCommit',false, ...
    'physicalClearCertified',false,'historySamples',0, ...
    'estimatedMassUsed',NaN,'totalMs',NaN, ...
    'selectedOnly',true,'forecastAndGeometryRan',false);
if nargin~=7 || ~isscalar(flightTime) || ~isfinite(flightTime)
    R.reason='invalid_flight_timestamp';R.totalMs=1000*toc(start);return
end
try
    model=wp_six_trigger_model_snapshot(agent);
    R.estimatedMassUsed=model.loadMass;
    st=agent.estimator.result.state;
    g=agent.reference.waypoint;
    xd=agent.reference.result.state.xd;
    if numel(xd)~=28 || any(~isfinite(xd(:))) || ...
            numel(st.pL)~=3 || numel(st.vL)~=3 || ...
            any(~isfinite([st.pL(:);st.vL(:)]))
        error('LightTrigger:CurrentState','Invalid current EKF/reference jet');
    end
    if ~isfield(K,'pastRelativeTimes') || numel(K.pastRelativeTimes)~=41 || ...
            abs(K.pastRelativeTimes(1)+1)>1e-8 || ...
            abs(K.pastRelativeTimes(end))>1e-9
        error('LightTrigger:HistoryGrid','Expected 41 consecutive 25ms samples');
    end
    if ~isstruct(S) || ~isfield(S,'filled') || ...
            ~isfield(S,'lastTime') || ~isfield(S,'pastLoad') || ...
            ~isequal(size(S.pastLoad),[3,41])
        S=initHistory();
    end
    % A missed cycle makes a different history-to-mode map; invalidate it.
    if S.filled>0 && (abs(flightTime-S.lastTime-K.samplePeriod_s)>1e-7 || ...
            flightTime<=S.lastTime)
        S=initHistory();
    end
    refLoad=xd(1:3);refVel=xd(5:7); 
    % Source xd has already undergone SUSPENDED_LOAD_REF_ADJUST.
    if ~isfinite(g.virtualTime) || ~isequal(size(g.rateState),[6,1]) || ...
            ~isfinite(g.heldRateCommand)
        error('LightTrigger:Governor','Current original governor state missing');
    end
    % Before 41 values, keep the real data (no duplicated/faked samples).
    if S.filled<41
        S.filled=S.filled+1;
        ii=S.filled;
        S.pastLoad(:,ii)=st.pL(:);
        S.pastVelocity(:,ii)=st.vL(:);
        S.pastRefLoad(:,ii)=refLoad(:);
        S.pastRefVelocity(:,ii)=refVel(:);
    else
        S.pastLoad(:,1:40)=S.pastLoad(:,2:41);
        S.pastVelocity(:,1:40)=S.pastVelocity(:,2:41);
        S.pastRefLoad(:,1:40)=S.pastRefLoad(:,2:41);
        S.pastRefVelocity(:,1:40)=S.pastRefVelocity(:,2:41);
        S.pastLoad(:,41)=st.pL(:);
        S.pastVelocity(:,41)=st.vL(:);
        S.pastRefLoad(:,41)=refLoad(:);
        S.pastRefVelocity(:,41)=refVel(:);
    end
    S.lastTime=flightTime;
    R.historySamples=S.filled;
    if S.filled<41
        R.status='UNKNOWN_HISTORY_WARMUP';
        R.reason='needs_41_REAL_consecutive_EKF_and_reference_samples';
        R.totalMs=1000*toc(start);return
    end
    H=struct('pastTimes',K.pastRelativeTimes, ...
        'pastLoad',S.pastLoad,'pastVelocity',S.pastVelocity, ...
        'pastRefLoad',S.pastRefLoad, ...
        'pastRefVelocity',S.pastRefVelocity, ...
        'tauNow',g.virtualTime,'rateNow',g.rateState(:), ...
        'heldCommandNow',g.heldRateCommand, ...
        'estimatedML',model.loadMass,'cableLength',model.cableLength, ...
        'gravity',model.gravity, ...
        'slowdownStart',g.slowdownStart, ...
        'slowdownFull',g.slowdownFull, ...
        'minScale',g.minScale,'filterTime',g.filterTime);
    T=wp_six_trigger_selected_nominal_v8(agent,H,K,routeCache,known,cfg,model);
    R=T;
    R.historySamples=S.filled;
    R.estimatedMassUsed=model.loadMass;
    R.forecastAndGeometryRan=~isnan(T.forecastMs);
catch ME
    R.status='UNKNOWN';
    R.reason=[ME.identifier ': ' ME.message];
end
R.totalMs=1000*toc(start);
end

function S=initHistory()
S=struct('filled',0,'lastTime',-inf, ...
    'pastLoad',zeros(3,41),'pastVelocity',zeros(3,41), ...
    'pastRefLoad',zeros(3,41),'pastRefVelocity',zeros(3,41));
end
