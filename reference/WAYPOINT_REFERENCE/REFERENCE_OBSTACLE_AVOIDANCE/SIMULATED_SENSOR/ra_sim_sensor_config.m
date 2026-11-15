function cfg=ra_sim_sensor_config(radiusOrLegacyMethod, legacyRadius)
%RA_SIM_SENSOR_CONFIG Default 20m sensor; obstacle method comes from WORLD.
% NEW: cfg=ra_sim_sensor_config() or cfg=ra_sim_sensor_config(20)
% Legacy: cfg=ra_sim_sensor_config('multiple_spheres',20)
% Legacy method is ONLY an assertion, never a second shape definition.
expected='';
if nargin==0 || isempty(radiusOrLegacyMethod)
    radiusM=20;
elseif isnumeric(radiusOrLegacyMethod)
    if nargin>=2 && ~isempty(legacyRadius)
        error('RaSensor:Config','Radius-only call accepts one argument.');
    end
    radiusM=radiusOrLegacyMethod;
else
    expected=char(radiusOrLegacyMethod);
    if nargin<2 || isempty(legacyRadius),radiusM=20;
    else,radiusM=legacyRadius;end
    if ~ismember(expected, ...
            {'single_sphere','single_ellipsoid','multiple_spheres','multiple_ellipsoids'})
        error('RaSensor:Config','Unsupported expected obstacle method');
    end
end
validateattributes(radiusM,{'numeric'}, ...
    {'scalar','real','finite','positive'},mfilename,'radiusM');
cfg=struct('radiusM',double(radiusM), ...
    'expectedMethod',expected,'approximation','FROM_WORLD', ...
    'sensorFrame','WORLD','centerSource','SIMULATOR_TRUE_DRONE_POSITION', ...
    'observationPolicy','CURRENT_SNAPSHOT_ONLY', ...
    'returnAllPartsOfDetectedObject',true);
% Preserve old reporting interfaces, NOT an independent override.
if ~isempty(expected),cfg.approximation=expected;end
end
