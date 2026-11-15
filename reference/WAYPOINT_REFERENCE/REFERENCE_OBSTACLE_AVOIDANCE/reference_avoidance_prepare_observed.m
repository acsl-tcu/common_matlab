function P=reference_avoidance_prepare_observed(agent,mode,arg3,arg4,arg5)
%REFERENCE_AVOIDANCE_PREPARE_OBSERVED Setup one C/B/D and observed cover.
% Call only when selected mode, physical geometry, or OBSERVED memory
% changes. Do not pass the complete simulator world to this function.
% geometryDefinition may later be replaced by matching CBF geometry.
% Recommended: (agent,mode,observedWorld[,externalGeometry]).
% Compatibility: (agent,mode,expectedMethod,observedWorld[,externalGeometry]).
options=struct('allowMexBuild',true);
if isstruct(arg3)
    observedWorld=arg3;
    if nargin>=4, geometryDefinition=arg4;else,geometryDefinition=[];end
    if nargin>=5 && isstruct(arg5), options=arg5;end
    expected='';
else
    expected=char(arg3);
    if nargin<4,error('RA:Observation','Missing observedWorld');end
    observedWorld=arg4;
    if nargin>=5, geometryDefinition=arg5;else,geometryDefinition=[];end
end
info=ra_obstacle_method_info(observedWorld);
if ~isempty(expected) && ~strcmp(expected,info.method)
    error('RA:Observation','Selected method differs from observedWorld.approximation');
end
cover=info.method;
if isempty(geometryDefinition)
    geometryDefinition=wp_system_geometry_config(mode); % PROVISIONAL
end
model=wp_six_trigger_model_snapshot(agent); % CURRENT EKF mL
cfg=wp_six_trigger_config(mode,cover,geometryDefinition,model);
known=wp_six_trigger_prepare(observedWorld);
% C-only native acceleration is prepared OFF the 25ms loop. The same
% C geometry remains available as pure MATLAB if mex/compilers are absent.
if strcmpi(char(mode),'C') && known.numParts>=3 && known.numParts<=128 && ...
        options.allowMexBuild
    native=reference_avoidance_mex_ensure('prepare');
    cfg.useMexCChainPreclear=native.available;
else
    native=struct('status','NOT_NEEDED_FOR_SELECTED_MODE_OR_PART_COUNT','available',false, ...
        'rebuilt',false,'buildMs',0,'errorText','');
    cfg.useMexCChainPreclear=false;
end
% If someone removes the MEX binary while Flight runs, the official tick
% can try rebuilding. See README: that exceptional cycle is NOT realtime.
cfg.autoRebuildMexDuringTick=false; % never compile in 25ms tick
P=struct('cfg',cfg,'known',known, ...
    'geometryProvisional',cfg.isProvisional,'mex',native, ...
    'obstacleMethodFromWorld',info.method,'obstacleKind',info.kind, ...
    'singlePartPerObject',info.singlePerObject, ...
    'observationSourceVerified',isfield(observedWorld,'sensorSource') && ...
      strcmp(observedWorld.sensorSource,'SIMULATED_SPHERICAL_TRUE_DRONE_CENTER'), ...
    'fullWorldWasSuppliedUnknown',~isfield(observedWorld,'sensorSource'), ...
    'sensorSelectionNotImplemented',false);
end
