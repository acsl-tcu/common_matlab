function [result,cache] = wp_six_trigger_run_selected_v4( ...
    agent,observedWorld,externalGeometry,mode,cover,forecastProvider,observationVersion,cache,measureBreakdown)
%WP_SIX_TRIGGER_RUN_SELECTED_V4 ONE selected geometry and its observed cover.
%
% This is the intended STAGING integration entry point (NOT installed in Flight).
%       ONE controller cycle = ONE call = ONE C/B/D x sphere/ellipsoid mode.
%
% agent               : actual flight agent, with current EKF state.mL
% observedWorld       : sensor OBSERVED/MEMORIZED world for ONE cover ONLY
% externalGeometry    : wp_system_geometry_config(mode), or later CBF source
% mode                : 'C', 'B', or 'D'
% cover               : 'multiple_spheres' or 'multiple_ellipsoids'
% forecastProvider    : function handle @(model) -> 4s forecast from actual
%                       current state, OR an already prepared forecast struct
% observationVersion  : stable external memory revision; MUST change when
%                       observed geometry/IDs/ground change
% cache               : saved value returned by previous invocation (or [])
%
% measureBreakdown    : optional logical, adds separate tic/toc stage timings.
% FORECAST SAFETY: the provider must supply independently justified TRUE
% acceleration and state-error bounds. This function does NOT invent them.
% No agent.parameter.set call is made.
if nargin<8,cache=[];end
if nargin<9,measureBreakdown=false;end
if nargin<7 || isempty(observationVersion)
    error('LightTrigger:ObservationVersion','Caller must supply real observation revision');
end
totalWatch=tic;

% Snapshot the actual current mass and all model parameters without editing.
if measureBreakdown,watch=tic;end
model=wp_six_trigger_model_snapshot(agent);
if measureBreakdown,snapshotMs=1000*toc(watch);watch=tic;end
cfg=wp_six_trigger_config(mode,cover,externalGeometry,model);
if measureBreakdown,configMs=1000*toc(watch);watch=tic;end

% Caching is legal only when both the observed world and chosen mode match.
rebuild=isempty(cache) || ~isstruct(cache) || ...
    ~isfield(cache,'version') || ~isequal(cache.version,observationVersion) || ...
    ~isfield(cache,'cover') || ~strcmp(cache.cover,cfg.obstacleApproximation) || ...
    ~isfield(cache,'known') || ~isfield(cache,'formatVersion') || ...
    cache.formatVersion~=4;
if rebuild
    known=wp_six_trigger_prepare(observedWorld,cfg.obstacleApproximation);
    cache=struct('version',observationVersion,'cover',cfg.obstacleApproximation, ...
                 'known',known,'formatVersion',4);
else
    known=cache.known;
end
if measureBreakdown,cacheMs=1000*toc(watch);end

% For an in-Flight provider, future-state generation time is included.
if isa(forecastProvider,'function_handle')
    forecastWatch=tic;
    forecast=forecastProvider(model);
    forecastMs=1000*toc(forecastWatch);
else
    forecast=forecastProvider;
    forecastMs=NaN; % externally computed forecast; NOT full trigger timing
end
if measureBreakdown,watch=tic;end
result=wp_six_trigger_evaluate_fast_v4(known,cfg,forecast);
if measureBreakdown,geometryMs=1000*toc(watch);end
result.modelLoadMassUsed=model.loadMass;
result.modelCableLengthUsed=model.cableLength;
result.geometrySelectedOnly=true;
result.obstacleCacheRebuilt=rebuild;
result.forecastGenerationMs=forecastMs;
result.tickElapsedExcludingExternalForecastMs=1000*toc(totalWatch);
if isfinite(forecastMs)
    result.selectedTriggerElapsedMs=result.tickElapsedExcludingExternalForecastMs;
else
    result.selectedTriggerElapsedMs=NaN;
end
result.includesEKFAndHLCExecution=false;
result.includesCBFAndTrajectoryGeneration=false;
if measureBreakdown
    result.tickStageMs=struct('modelSnapshot',snapshotMs, ...
        'configuration',configMs,'obstacleCache',cacheMs, ...
        'forecastProvider',forecastMs,'geometryEvaluation',geometryMs, ...
        'instrumentationAndOther',result.tickElapsedExcludingExternalForecastMs- ...
          (snapshotMs+configMs+cacheMs+geometryMs+max(0,forecastMs)));
end
end
